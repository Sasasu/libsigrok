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

/* Firmware V2.5.1 differs from the published SCPI tree. */
static const char *sources[] = { "CH1", "CH2" };
static const char *slopes[] = { "r", "f" };
static const char *slope_cmd[] = { "RISe", "FALL" };
static const char *modes[] = { "Auto", "Normal", "Single" };
static const char *mode_cmd[] = { "AUTo", "NORMal", "SINGle" };
static const char *waves[] = { "Sine", "Square", "Ramp", "Pulse" };
static const char *wave_cmd[] = { "SINe", "SQUare", "RAMP", "PULSe" };
static const char *couplings[] = { "DC", "AC" };
static const uint64_t depths[] = { 4000, 8000 };
static const uint32_t genopts[] = {
	SR_CONF_ENABLED | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_PATTERN_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_OUTPUT_FREQUENCY | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_AMPLITUDE | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DUTY_CYCLE | SR_CONF_GET | SR_CONF_SET,
};
static const uint32_t trigopts[] = {
	SR_CONF_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};

static const char *command(uint32_t key, const struct sr_channel_group *cg,
	const char ***names, const char ***commands, size_t *count)
{
	gboolean gen = cg && !strcmp(cg->name, "WG");
	*count = 0;
	if (gen) {
		switch (key) {
		case SR_CONF_ENABLED:
			return ":CHAN";
		case SR_CONF_PATTERN_MODE:
			*names = waves;
			*commands = wave_cmd;
			*count = ARRAY_SIZE(waves);
			return ":FUNC";
		case SR_CONF_OUTPUT_FREQUENCY:
			return ":FUNC:FREQ";
		case SR_CONF_AMPLITUDE:
			return ":FUNC:AMPL";
		case SR_CONF_OFFSET:
			return ":FUNC:OFFSET";
		case SR_CONF_DUTY_CYCLE:
			return ":FUNC:DTYCYCLE";
		default:
			return NULL;
		}
	}
	if (cg) {
		if (!strcmp(cg->name, "Trigger") && key == SR_CONF_COUPLING) {
			*names = *commands = couplings;
			*count = ARRAY_SIZE(couplings);
			return ":TRIG:COUP";
		}
		return NULL;
	}
	switch (key) {
	case SR_CONF_TRIGGER_SOURCE:
		*names = *commands = sources;
		*count = ARRAY_SIZE(sources);
		return ":TRIG:SING:EDGE:SOUR";
	case SR_CONF_TRIGGER_SLOPE:
		*names = slopes;
		*commands = slope_cmd;
		*count = ARRAY_SIZE(slopes);
		return ":TRIG:SING:EDGE";
	case SR_CONF_DEVICE_MODE:
		*names = modes;
		*commands = mode_cmd;
		*count = ARRAY_SIZE(modes);
		return ":TRIG:SWE";
	case SR_CONF_TRIGGER_LEVEL:
		return ":TRIG:SING:EDGE:LEV";
	case SR_CONF_BUFFERSIZE:
		return ":ACQuire:DEPMem";
	default:
		return NULL;
	}
}

SR_PRIV int hds_text(
	const struct sr_dev_inst *sdi, const char *cmd, char *reply)
{
	size_t len;
	int ret = hds_query(sdi, cmd, (uint8_t *)reply, 127, FALSE, &len);
	if (ret != SR_OK)
		return ret;
	reply[len] = 0;
	g_strstrip(reply);
	return SR_OK;
}

SR_PRIV int hds_ext_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	const char **names, **commands, *cmd;
	size_t count, i;
	char query[80], reply[128], *end;
	double v;
	int ret;

	cmd = command(key, cg, &names, &commands, &count);
	if (!cmd)
		return SR_ERR_NA;
	if (cg && !strcmp(cg->name, "WG") && key == SR_CONF_DUTY_CYCLE) {
		if ((ret = hds_text(sdi, ":FUNC?", reply)) != SR_OK)
			return ret;
		if (g_ascii_strcasecmp(reply, "PULSe"))
			return SR_ERR_NA;
	}
	g_snprintf(query, sizeof(query), "%s?", cmd);
	if ((ret = hds_text(sdi, query, reply)) != SR_OK)
		return ret;
	if (count) {
		for (i = 0; i < count; i++)
			if (!g_ascii_strcasecmp(reply, commands[i]))
				break;
		if (i == count)
			return SR_ERR_DATA;
		*data = g_variant_new_string(names[i]);
		return SR_OK;
	}
	if (key == SR_CONF_ENABLED) {
		if (g_ascii_strcasecmp(reply, "ON") &&
			g_ascii_strcasecmp(reply, "OFF"))
			return SR_ERR_DATA;
		*data = g_variant_new_boolean(!g_ascii_strcasecmp(reply, "ON"));
		return SR_OK;
	}
	v = g_ascii_strtod(reply, &end);
	if (!isfinite(v) || end == reply)
		return SR_ERR_DATA;
	if (key == SR_CONF_BUFFERSIZE) {
		if (!g_ascii_strcasecmp(end, "K"))
			v *= 1000;
		if (v != 4000 && v != 8000)
			return SR_ERR_DATA;
		*data = g_variant_new_uint64(v);
		return SR_OK;
	}
	if (key == SR_CONF_TRIGGER_LEVEL) {
		while (g_ascii_isspace(*end))
			end++;
		if (*end == 'm')
			v *= .001;
		else if (*end == 'u')
			v *= .000001;
	}
	*data = g_variant_new_double(v);
	return SR_OK;
}

SR_PRIV int hds_ext_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	const char **names, **commands;
	size_t count;
	double frequency[] = { .1, 25000000, .1 };
	char shape_name[128];
	GVariant *shape;
	int ret;

	if (key == SR_CONF_DEVICE_OPTIONS && cg) {
		if (!strcmp(cg->name, "WG")) {
			if ((ret = hds_text(sdi, ":FUNC?", shape_name)) !=
				SR_OK)
				return ret;
			count = ARRAY_SIZE(genopts);
			if (g_ascii_strcasecmp(shape_name, "PULSe"))
				count--;
			*data = std_gvar_array_u32(genopts, count);
		} else if (!strcmp(cg->name, "Trigger"))
			*data = std_gvar_array_u32(ARRAY_AND_SIZE(trigopts));
		else
			return SR_ERR_NA;
		return SR_OK;
	}
	if (!command(key, cg, &names, &commands, &count))
		return SR_ERR_NA;
	if (count)
		*data = g_variant_new_strv(names, count);
	else if (key == SR_CONF_BUFFERSIZE)
		*data = std_gvar_array_u64(ARRAY_AND_SIZE(depths));
	else if (key == SR_CONF_OUTPUT_FREQUENCY) {
		if ((ret = hds_ext_get(
			     SR_CONF_PATTERN_MODE, &shape, sdi, cg)) != SR_OK)
			return ret;
		g_variant_ref_sink(shape);
		if (!strcmp(g_variant_get_string(shape, NULL), "Ramp"))
			frequency[1] = 1000000;
		else if (strcmp(g_variant_get_string(shape, NULL), "Sine"))
			frequency[1] = 5000000;
		g_variant_unref(shape);
		*data = std_gvar_min_max_step_array(frequency);
	} else
		return SR_ERR_NA;
	return SR_OK;
}

SR_PRIV int hds_ext_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	const char **names, **commands, *cmd, *arg;
	size_t count, i;
	char line[100], number[G_ASCII_DTOSTR_BUF_SIZE], load[128];
	double v = 0, actual, maximum, amp, offset;
	struct dev_context *devc = sdi->priv;
	uint32_t other_key;
	unsigned int trigger_channel = 0;
	GVariant *readback, *other;
	gboolean valid;
	int ret;
	unsigned int attempt;
	cmd = command(key, cg, &names, &commands, &count);
	if (!cmd)
		return SR_ERR_NA;
	if (cg && !strcmp(cg->name, "WG") && key == SR_CONF_DUTY_CYCLE) {
		if ((ret = hds_text(sdi, ":FUNC?", load)) != SR_OK)
			return ret;
		if (g_ascii_strcasecmp(load, "PULSe"))
			return SR_ERR_NA;
	}
	if (count) {
		arg = g_variant_get_string(data, NULL);
		for (i = 0; i < count; i++)
			if (!strcmp(arg, names[i]))
				break;
		if (i == count)
			return SR_ERR_ARG;
		arg = commands[i];
	} else if (key == SR_CONF_ENABLED)
		arg = g_variant_get_boolean(data) ? "ON" : "OFF";
	else if (key == SR_CONF_BUFFERSIZE) {
		if (g_variant_get_uint64(data) == 4000)
			arg = "4k";
		else if (g_variant_get_uint64(data) == 8000)
			arg = "8k";
		else
			return SR_ERR_ARG;
	} else {
		v = g_variant_get_double(data);
		if (!isfinite(v))
			return SR_ERR_ARG;
		if (key == SR_CONF_OUTPUT_FREQUENCY) {
			if ((ret = hds_ext_get(SR_CONF_PATTERN_MODE, &other,
				     sdi, cg)) != SR_OK)
				return ret;
			g_variant_ref_sink(other);
			arg = g_variant_get_string(other, NULL);
			if (!strcmp(arg, "Sine"))
				maximum = 25000000;
			else if (!strcmp(arg, "Ramp"))
				maximum = 1000000;
			else
				maximum = 5000000;
			g_variant_unref(other);
			if (v < .1 || v > maximum)
				return SR_ERR_ARG;
		}
		if (key == SR_CONF_AMPLITUDE || key == SR_CONF_OFFSET) {
			if ((ret = hds_text(sdi, ":FUNC:LOAD?", load)) != SR_OK)
				return ret;
			maximum = !g_ascii_strcasecmp(load, "INF") ? 5.0 : 2.5;
			other_key = key == SR_CONF_AMPLITUDE
				? SR_CONF_OFFSET
				: SR_CONF_AMPLITUDE;
			if ((ret = hds_ext_get(other_key, &other, sdi, cg)) !=
				SR_OK)
				return ret;
			g_variant_ref_sink(other);
			actual = g_variant_get_double(other);
			g_variant_unref(other);
			amp = key == SR_CONF_AMPLITUDE ? v : actual;
			offset = key == SR_CONF_OFFSET ? v : actual;
			if (amp < maximum / 250 || amp > maximum ||
				fabs(offset) + amp / 2 > maximum / 2 + 1e-6)
				return SR_ERR_ARG;
		}
		if (key == SR_CONF_DUTY_CYCLE && (v < .1 || v > 99.9))
			return SR_ERR_ARG;
		if (key == SR_CONF_TRIGGER_LEVEL && fabs(v) > 100000)
			return SR_ERR_ARG;
		/* Scientific notation avoids firmware truncating -0.25 to
		 * -0.24. */
		g_ascii_formatd(number, sizeof(number),
			key == SR_CONF_OFFSET ? "%.6e" : "%.6g", v);
		arg = number;
	}
	if (key == SR_CONF_TRIGGER_LEVEL) {
		if ((ret = hds_read_header(sdi)) != SR_OK ||
			(ret = hds_text(sdi, ":TRIG:SING:EDGE:SOUR?", load)) !=
				SR_OK)
			return ret;
		trigger_channel = !strcmp(load, "CH2");
	}
	g_snprintf(line, sizeof(line), "%s %s%s", cmd, arg,
		key == SR_CONF_TRIGGER_LEVEL ? "V" : "");
	if ((ret = hds_command(sdi, line)) != SR_OK)
		return ret;
	for (attempt = 0; attempt < 20; attempt++) {
		g_usleep(20000);
		if ((ret = hds_ext_get(key, &readback, sdi, cg)) != SR_OK)
			return ret;
		g_variant_ref_sink(readback);
		if (g_variant_is_of_type(data, G_VARIANT_TYPE_DOUBLE)) {
			actual = g_variant_get_double(readback);
			/* Trigger levels are quantized to screen codes; allow
			 * one code. */
			if (key == SR_CONF_TRIGGER_LEVEL)
				maximum =
					devc->wave.scale[trigger_channel] / 25 +
					1e-6;
			else
				maximum = MAX(1e-6, fabs(v) * 1e-5);
			valid = fabs(actual - v) <= maximum;
		} else
			valid = g_variant_equal(data, readback);
		g_variant_unref(readback);
		if (valid)
			return SR_OK;
	}
	sr_err("Instrument rejected %s.", line);
	return SR_ERR_DATA;
}
