/*
 * This file is part of the libsigrok project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <config.h>
#include <math.h>
#include <string.h>
#include "protocol.h"

static const uint32_t generator_opts[] = {
	SR_CONF_ENABLED | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_OUTPUT_FREQUENCY | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_AMPLITUDE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DUTY_CYCLE | SR_CONF_GET | SR_CONF_SET,
};

static const char *wave_names[] = {
	"Sine", "Square", "Ramp", "Trapezoid", "DC", "Exponential", "Gaussian",
};
static const char *wave_commands[] = {
	"SINusoid", "SQUare", "RAMP", "TRApe", "DC", "EXP", "GAUSs",
};

/* Commands use integer Hz, mV and percent, rather than SI voltage units. */
static const struct {
	uint32_t key;
	const char *command;
	double units, min, max;
} parameters[] = {
	{ SR_CONF_OUTPUT_FREQUENCY, "SOURce:OUTPut:FREQuency", 1, 1, 25000000 },
	{ SR_CONF_AMPLITUDE, "SOURce:OUTPut:AMP", 1000, 0, 3.5 },
	{ SR_CONF_OFFSET, "SOURce:OUTPut:OFFSet", 1000, -3.5, 3.5 },
	{ SR_CONF_DUTY_CYCLE, "SOURce:OUTPut:DUTY", 1, 0, 100 },
};

SR_PRIV int hantek_dso3000a_generator_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi)
{
	char *response, command[64];
	gboolean enabled;
	double value;
	unsigned int i;
	int ret;

	if (key == SR_CONF_ENABLED) {
		ret = sr_scpi_get_bool(sdi->conn, "SOURce:OUTPut?", &enabled);
		if (ret == SR_OK)
			*data = g_variant_new_boolean(enabled);
		return ret;
	}
	if (key == SR_CONF_PATTERN_MODE) {
		ret = sr_scpi_get_string(sdi->conn, "SOURce:FUNCtion:TYPE?", &response);
		if (ret != SR_OK)
			return ret;
		for (i = 0; i < ARRAY_SIZE(wave_commands); i++)
			if (!g_ascii_strcasecmp(response, wave_commands[i]))
				break;
		g_free(response);
		/* Modulation and uploaded arbitrary waves are not implemented. */
		if (i == ARRAY_SIZE(wave_commands))
			return SR_ERR_NA;
		*data = g_variant_new_string(wave_names[i]);
		return SR_OK;
	}
	for (i = 0; i < ARRAY_SIZE(parameters); i++) {
		if (key != parameters[i].key)
			continue;
		g_snprintf(command, sizeof(command), "%s?", parameters[i].command);
		ret = sr_scpi_get_double(sdi->conn, command, &value);
		if (ret != SR_OK)
			return ret;
		if (!isfinite(value))
			return SR_ERR_DATA;
		*data = g_variant_new_double(value / parameters[i].units);
		return SR_OK;
	}
	return SR_ERR_NA;
}

SR_PRIV int hantek_dso3000a_generator_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi)
{
	const char *name;
	char *wave, query[64];
	double value, scaled, actual;
	unsigned int i;
	int ret;

	if (key == SR_CONF_ENABLED)
		return sr_scpi_send(sdi->conn, "SOURce:OUTPut %s",
				g_variant_get_boolean(data) ? "ON" : "OFF");
	if (key == SR_CONF_PATTERN_MODE) {
		name = g_variant_get_string(data, NULL);
		for (i = 0; i < ARRAY_SIZE(wave_names); i++)
			if (!g_ascii_strcasecmp(name, wave_names[i]))
				return sr_scpi_send(sdi->conn, "SOURce:FUNCtion:TYPE %s",
						wave_commands[i]);
		return SR_ERR_ARG;
	}
	for (i = 0; i < ARRAY_SIZE(parameters); i++) {
		if (key != parameters[i].key)
			continue;
		value = g_variant_get_double(data);
		if (!isfinite(value) || value < parameters[i].min || value > parameters[i].max)
			return SR_ERR_ARG;
		scaled = value * parameters[i].units;
		/* Reject values that cannot be represented by the wire protocol. */
		if (fabs(scaled - round(scaled)) > 1e-6)
			return SR_ERR_ARG;
		if (key == SR_CONF_DUTY_CYCLE) {
			ret = sr_scpi_get_string(sdi->conn, "SOURce:FUNCtion:TYPE?", &wave);
			if (ret != SR_OK)
				return ret;
			ret = !g_ascii_strcasecmp(wave, "SQUare") ||
				!g_ascii_strcasecmp(wave, "RAMP") ||
				!g_ascii_strcasecmp(wave, "TRApe");
			g_free(wave);
			if (!ret)
				return SR_ERR_NA;
		}
		ret = sr_scpi_send(sdi->conn, "%s %d", parameters[i].command, (int)round(scaled));
		if (ret != SR_OK)
			return ret;
		g_snprintf(query, sizeof(query), "%s?", parameters[i].command);
		ret = sr_scpi_get_double(sdi->conn, query, &actual);
		if (ret != SR_OK)
			return ret;
		if (!isfinite(actual) || fabs(actual - scaled) > 1e-6) {
			sr_err("Generator rejected %s=%g (read back %g).",
				parameters[i].command, scaled, actual);
			return SR_ERR_DATA;
		}
		return SR_OK;
	}
	return SR_ERR_NA;
}

SR_PRIV int hantek_dso3000a_generator_list(uint32_t key, GVariant **data)
{
	static const double frequency[] = { 1, 25000000, 1 };

	switch (key) {
	case SR_CONF_DEVICE_OPTIONS:
		*data = std_gvar_array_u32(ARRAY_AND_SIZE(generator_opts));
		break;
	case SR_CONF_PATTERN_MODE:
		*data = g_variant_new_strv(wave_names, ARRAY_SIZE(wave_names));
		break;
	case SR_CONF_OUTPUT_FREQUENCY:
		*data = std_gvar_min_max_step_array(frequency);
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}
