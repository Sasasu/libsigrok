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

static const char *sources[] = { "CH1", "CH2", "CH3", "CH4", "EXT/10" };
static const char *source_commands[] = {
	"CHANnel1", "CHANnel2", "CHANnel3", "CHANnel4", "EXT/10",
};
static const char *slopes[] = { "r", "f", "e" };
static const char *slope_commands[] = { "POSItive", "NEGAtive", "RFALl" };

static int command_for(uint32_t key, const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg, char *command, size_t size)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	const char *suffix;
	int pod;

	if (!cg) {
		switch (key) {
		case SR_CONF_TRIGGER_SOURCE: suffix = "SOURce"; break;
		case SR_CONF_TRIGGER_SLOPE: suffix = "POLarity"; break;
		case SR_CONF_TRIGGER_LEVEL: suffix = "LEVel"; break;
		default: return SR_ERR_CHANNEL_GROUP;
		}
		g_snprintf(command, size, "TRIGger:EDGe:%s", suffix);
		return SR_OK;
	}
	if (!g_slist_find(sdi->channel_groups, cg))
		return SR_ERR_ARG;
	if (cg == devc->pods[0] || cg == devc->pods[1]) {
		if (key != SR_CONF_LOGIC_THRESHOLD_CUSTOM)
			return SR_ERR_NA;
		pod = cg == devc->pods[0] ? 1 : 2;
		g_snprintf(command, size, "LA:POD%d:THReshold:USERVolt", pod);
		return SR_OK;
	}
	if (!cg->channels || ((struct sr_channel *)cg->channels->data)->type != SR_CHANNEL_ANALOG)
		return SR_ERR_NA;
	ch = cg->channels->data;
	switch (key) {
	case SR_CONF_OFFSET: suffix = "OFFSet"; break;
	case SR_CONF_INVERTED: suffix = "INVert"; break;
	case SR_CONF_FILTER: suffix = "BWLimit"; break;
	default: return SR_ERR_NA;
	}
	g_snprintf(command, size, "CHANnel%d:%s", ch->index + 1, suffix);
	return SR_OK;
}

SR_PRIV int hantek_dso3000a_control_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	char command[64], query[68], *response;
	const char *const *names, *const *commands;
	unsigned int i, count;
	gboolean value;
	double number;
	int ret;

	if ((ret = command_for(key, sdi, cg, command, sizeof(command))) != SR_OK)
		return ret;
	g_snprintf(query, sizeof(query), "%s?", command);
	if (key == SR_CONF_INVERTED) {
		ret = sr_scpi_get_bool(sdi->conn, query, &value);
		if (ret == SR_OK)
			*data = g_variant_new_boolean(value);
		return ret;
	}
	if (key == SR_CONF_OFFSET || key == SR_CONF_TRIGGER_LEVEL ||
			key == SR_CONF_LOGIC_THRESHOLD_CUSTOM) {
		ret = sr_scpi_get_double(sdi->conn, query, &number);
		if (ret != SR_OK)
			return ret;
		if (!isfinite(number))
			return SR_ERR_DATA;
		*data = g_variant_new_double(number);
		return SR_OK;
	}
	ret = sr_scpi_get_string(sdi->conn, query, &response);
	if (ret != SR_OK)
		return ret;
	if (key == SR_CONF_FILTER) {
		if (g_ascii_strcasecmp(response, "20M") && g_ascii_strcasecmp(response, "OFF")) {
			g_free(response);
			return SR_ERR_DATA;
		}
		*data = g_variant_new_boolean(!g_ascii_strcasecmp(response, "20M"));
		g_free(response);
		return SR_OK;
	}
	names = key == SR_CONF_TRIGGER_SOURCE ? sources : slopes;
	commands = key == SR_CONF_TRIGGER_SOURCE ? source_commands : slope_commands;
	count = key == SR_CONF_TRIGGER_SOURCE ? ARRAY_SIZE(sources) : ARRAY_SIZE(slopes);
	for (i = 0; i < count; i++)
		if (!g_ascii_strcasecmp(response, commands[i]))
			break;
	g_free(response);
	if (i == count)
		return SR_ERR_DATA;
	*data = g_variant_new_string(names[i]);
	return SR_OK;
}

SR_PRIV int hantek_dso3000a_control_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	const char *arg = NULL, *name;
	const char *const *names, *const *commands;
	char command[64], number[G_ASCII_DTOSTR_BUF_SIZE], query[64];
	unsigned int i, count;
	double value, scale, tolerance = 1e-6;
	GVariant *actual;
	int ret;

	if (devc->running)
		return SR_ERR;
	if ((ret = command_for(key, sdi, cg, command, sizeof(command))) != SR_OK)
		return ret;
	if (key == SR_CONF_TRIGGER_SOURCE || key == SR_CONF_TRIGGER_SLOPE) {
		names = key == SR_CONF_TRIGGER_SOURCE ? sources : slopes;
		commands = key == SR_CONF_TRIGGER_SOURCE ? source_commands : slope_commands;
		count = key == SR_CONF_TRIGGER_SOURCE ? ARRAY_SIZE(sources) : ARRAY_SIZE(slopes);
		name = g_variant_get_string(data, NULL);
		for (i = 0; i < count; i++)
			if (!strcmp(name, names[i])) {
				arg = commands[i];
				break;
			}
		if (!arg)
			return SR_ERR_ARG;
	} else if (key == SR_CONF_INVERTED || key == SR_CONF_FILTER) {
		arg = g_variant_get_boolean(data) ?
			(key == SR_CONF_FILTER ? "20M" : "ON") : "OFF";
	} else {
		value = g_variant_get_double(data);
		if (!isfinite(value))
			return SR_ERR_ARG;
		if (key == SR_CONF_OFFSET) {
			ch = cg->channels->data;
			g_snprintf(query, sizeof(query), "CHANnel%d:SCALe?", ch->index + 1);
			if ((ret = sr_scpi_get_double(sdi->conn, query, &scale)) != SR_OK)
				return ret;
			if (!isfinite(scale) || scale <= 0 || fabs(value) > 5 * scale)
				return SR_ERR_ARG;
			tolerance += scale / 50;
		}
		g_ascii_dtostr(number, sizeof(number), value);
		arg = number;
	}
	if (!cg && sr_scpi_send(sdi->conn, "TRIGger:MODE EDGE") != SR_OK)
		return SR_ERR;
	if ((ret = sr_scpi_send(sdi->conn, "%s %s", command, arg)) != SR_OK)
		return ret;
	ret = hantek_dso3000a_control_get(key, &actual, sdi, cg);
	if (ret != SR_OK)
		return ret;
	g_variant_ref_sink(actual);
	if (g_variant_is_of_type(data, G_VARIANT_TYPE_DOUBLE))
		ret = fabs(g_variant_get_double(actual) - g_variant_get_double(data)) <= tolerance;
	else
		ret = g_variant_equal(actual, data);
	g_variant_unref(actual);
	if (!ret) {
		sr_err("Device did not accept %s %s.", command, arg);
		return SR_ERR_DATA;
	}
	return SR_OK;
}

SR_PRIV int hantek_dso3000a_control_list(uint32_t key, GVariant **data)
{
	if (key == SR_CONF_TRIGGER_SOURCE)
		*data = g_variant_new_strv(sources, ARRAY_SIZE(sources));
	else if (key == SR_CONF_TRIGGER_SLOPE)
		*data = g_variant_new_strv(slopes, ARRAY_SIZE(slopes));
	else
		return SR_ERR_NA;
	return SR_OK;
}

SR_PRIV int hantek_dso3000a_arm(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	const char *sweep;

	sweep = devc->single ? "SINGle" :
		!g_ascii_strcasecmp(devc->saved_sweep, "SINGle") ? "AUTO" : devc->saved_sweep;
	if (sr_scpi_send(sdi->conn, "RUNning OFF") != SR_OK ||
			sr_scpi_send(sdi->conn, "TRIGger:MODE EDGE") != SR_OK ||
			sr_scpi_send(sdi->conn, "TRIGger:SWEep %s", sweep) != SR_OK ||
			sr_scpi_send(sdi->conn, "RUNning ON") != SR_OK)
		return SR_ERR;
	devc->waiting_trigger = devc->single;
	return SR_OK;
}
