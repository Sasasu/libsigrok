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
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "protocol.h"

static const uint32_t scanopts[] = { SR_CONF_CONN };
static const uint32_t drvopts[] = { SR_CONF_MULTIMETER };
static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_CONN | SR_CONF_GET,
	SR_CONF_MEASURED_QUANTITY | SR_CONF_GET | SR_CONF_LIST,
	SR_CONF_RANGE | SR_CONF_GET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
};

static const struct {
	const char *name, *short_name;
	enum sr_mq mq;
	enum sr_unit unit;
	uint64_t flags;
} functions[] = {
	{ "VOLTage", "VOLT", SR_MQ_VOLTAGE, SR_UNIT_VOLT, SR_MQFLAG_DC },
	{ "VOLTage", "VOLT", SR_MQ_VOLTAGE, SR_UNIT_VOLT, SR_MQFLAG_AC },
	{ "CURRent", "CURR", SR_MQ_CURRENT, SR_UNIT_AMPERE, SR_MQFLAG_DC },
	{ "CURRent", "CURR", SR_MQ_CURRENT, SR_UNIT_AMPERE, SR_MQFLAG_AC },
	{ "RESistance", "RES", SR_MQ_RESISTANCE, SR_UNIT_OHM, 0 },
	{ "CAPacitance", "CAP", SR_MQ_CAPACITANCE, SR_UNIT_FARAD, 0 },
	{ "DIODe", "DIOD", SR_MQ_VOLTAGE, SR_UNIT_VOLT,
		SR_MQFLAG_DC | SR_MQFLAG_DIODE },
	{ "CONTinuity", "CONT", SR_MQ_CONTINUITY, SR_UNIT_OHM, 0 },
};

struct meter_state {
	size_t function;
	uint64_t flags;
	char range[128];
};

/* V2.5.1 returns "error" for function queries outside the selected family.
 * The generic query describes resistance/capacitance/diode/continuity;
 * voltage and current each have their own AC/DC query.
 */
static int read_function(const struct sr_dev_inst *sdi, size_t *function)
{
	char reply[128], command[64];
	size_t i;
	int ret;

	ret = hds_text(sdi, ":DMM:CONFigure?", reply);
	if (ret != SR_OK)
		return ret;
	for (i = 4; i < ARRAY_SIZE(functions); i++) {
		if (!g_ascii_strcasecmp(reply, functions[i].name) ||
			!g_ascii_strcasecmp(reply, functions[i].short_name)) {
			*function = i;
			return SR_OK;
		}
	}
	for (i = 0; i < 4; i += 2) {
		g_snprintf(command, sizeof(command), ":DMM:CONFigure:%s?",
			functions[i].name);
		if ((ret = hds_text(sdi, command, reply)) != SR_OK)
			return ret;
		if (!g_ascii_strcasecmp(reply, "DC")) {
			*function = i;
			return SR_OK;
		}
		if (!g_ascii_strcasecmp(reply, "AC")) {
			*function = i + 1;
			return SR_OK;
		}
	}
	sr_err("Cannot identify the multimeter function.");
	return SR_ERR_DATA;
}

static int read_state(const struct sr_dev_inst *sdi, struct meter_state *state)
{
	char reply[128];
	int ret;

	if ((ret = read_function(sdi, &state->function)) != SR_OK)
		return ret;
	state->flags = functions[state->function].flags;
	if ((ret = hds_text(sdi, ":DMM:RANGE?", state->range)) != SR_OK)
		return ret;
	/* Relative measurements exist only for DC voltage/current and
	 * resistance. Other functions may reject this query.
	 */
	if (state->function != 0 && state->function != 2 &&
		functions[state->function].mq != SR_MQ_RESISTANCE)
		return SR_OK;
	if ((ret = hds_text(sdi, ":DMM:REL?", reply)) != SR_OK)
		return ret;
	if (!g_ascii_strcasecmp(reply, "ON"))
		state->flags |= SR_MQFLAG_RELATIVE;
	else if (g_ascii_strcasecmp(reply, "OFF"))
		return SR_ERR_DATA;
	return SR_OK;
}

SR_PRIV int hds_dmm_check(const struct sr_dev_inst *sdi)
{
	size_t function;

	return read_function(sdi, &function);
}

/* Range is used only to report display resolution, never to scale a
 * measurement. The 20,000-count display has a few reduced-count ranges.
 */
static int range_digits(const struct meter_state *state, int fallback)
{
	const char *unit;
	char *end;
	double range;
	enum sr_mq mq = functions[state->function].mq;

	if (state->flags & SR_MQFLAG_DIODE)
		return fallback;
	range = g_ascii_strtod(state->range, &end);
	if (end == state->range || !isfinite(range) || range <= 0)
		return fallback;
	while (g_ascii_isspace(*end))
		end++;
	switch (*end) {
	case 'n':
		range *= 1e-9;
		end++;
		break;
	case 'u':
		range *= 1e-6;
		end++;
		break;
	case 'm':
		range *= 1e-3;
		end++;
		break;
	case 'k':
	case 'K':
		range *= 1e3;
		end++;
		break;
	case 'M':
		range *= 1e6;
		end++;
		break;
	}
	switch (functions[state->function].unit) {
	case SR_UNIT_VOLT:
		unit = "V";
		break;
	case SR_UNIT_AMPERE:
		unit = "A";
		break;
	case SR_UNIT_FARAD:
		unit = "F";
		break;
	default:
		unit = "Ohm";
		break;
	}
	if (range < 1e-12 || range > 1e12)
		return fallback;
	if (g_ascii_strcasecmp(end, unit) &&
		!(functions[state->function].unit == SR_UNIT_OHM &&
			!strcmp(end, "Ω")))
		return fallback;
	if (mq == SR_MQ_VOLTAGE && (range == 1000 || range == 750))
		return 1;
	if (mq == SR_MQ_CURRENT && range == 10)
		return 3;
	if (mq == SR_MQ_RESISTANCE && range == 100e6)
		return -4;
	return (int)llround(log10(20000 / range));
}

static int parse_value(const char *reply, double *value, int *digits)
{
	const char *p, *decimal, *exponent;
	char *end;
	long exp = 0;

	*digits = 0;
	if (!g_ascii_strcasecmp(reply, "OL") ||
		!g_ascii_strcasecmp(reply, "+OL") ||
		!g_ascii_strcasecmp(reply, "OVERLOAD")) {
		*value = INFINITY;
		return SR_OK;
	}
	if (!g_ascii_strcasecmp(reply, "-OL")) {
		*value = -INFINITY;
		return SR_OK;
	}
	errno = 0;
	*value = g_ascii_strtod(reply, &end);
	if (end == reply || *end || errno == ERANGE || isnan(*value))
		return SR_ERR_DATA;
	/* Conventional SCPI overload sentinel, if used by the firmware. */
	if (fabs(*value) >= 9e37)
		*value = copysign(INFINITY, *value);
	if (!isfinite(*value))
		return SR_OK;
	decimal = strchr(reply, '.');
	exponent = strpbrk(reply, "eE");
	if (decimal) {
		for (p = decimal + 1; g_ascii_isdigit(*p); p++)
			(*digits)++;
	}
	if (exponent) {
		errno = 0;
		exp = strtol(exponent + 1, NULL, 10);
		if (errno == ERANGE || exp < -308 || exp > 308)
			return SR_ERR_DATA;
	}
	*digits -= exp;
	return SR_OK;
}

SR_PRIV int hds_dmm_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	char reply[128];
	size_t function;
	int ret;

	if (cg)
		return SR_ERR_CHANNEL_GROUP;
	switch (key) {
	case SR_CONF_CONN:
		*data = g_variant_new_string(sdi->connection_id);
		return SR_OK;
	case SR_CONF_LIMIT_SAMPLES:
	case SR_CONF_LIMIT_MSEC:
		return sr_sw_limits_config_get(&devc->limits, key, data);
	case SR_CONF_MEASURED_QUANTITY:
		if ((ret = read_function(sdi, &function)) != SR_OK)
			return ret;
		*data = g_variant_new("(ut)", functions[function].mq,
			functions[function].flags);
		return SR_OK;
	case SR_CONF_RANGE:
		if ((ret = hds_text(sdi, ":DMM:RANGE?", reply)) != SR_OK)
			return ret;
		if (!*reply || !g_ascii_strcasecmp(reply, "error"))
			return SR_ERR_DATA;
		*data = g_variant_new_string(reply);
		return SR_OK;
	default:
		return SR_ERR_NA;
	}
}

SR_PRIV int hds_dmm_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;

	if (cg)
		return SR_ERR_CHANNEL_GROUP;
	if (key != SR_CONF_LIMIT_SAMPLES && key != SR_CONF_LIMIT_MSEC)
		return SR_ERR_NA;
	return sr_sw_limits_config_set(&devc->limits, key, data);
}

SR_PRIV int hds_dmm_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	GVariantBuilder builder;
	size_t i;

	if (cg)
		return SR_ERR_CHANNEL_GROUP;
	if (key == SR_CONF_MEASURED_QUANTITY) {
		g_variant_builder_init(&builder, G_VARIANT_TYPE("a(ut)"));
		for (i = 0; i < ARRAY_SIZE(functions); i++)
			g_variant_builder_add(&builder, "(ut)", functions[i].mq,
				functions[i].flags);
		*data = g_variant_builder_end(&builder);
		return SR_OK;
	}
	return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
}

static int receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct dev_context *devc = sdi->priv;
	struct meter_state before, after;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_datafeed_packet packet;
	char reply[128];
	double value;
	int ret, digits;

	(void)fd;
	(void)revents;
	if (sr_sw_limits_check(&devc->limits))
		goto stop;
	if ((ret = read_state(sdi, &before)) != SR_OK ||
		(ret = hds_text(sdi, ":DMM:MEAS?", reply)) != SR_OK ||
		(ret = parse_value(reply, &value, &digits)) != SR_OK ||
		(ret = read_state(sdi, &after)) != SR_OK)
		goto error;
	/* Drop a reading spanning a function/range/relative-mode change. */
	if (before.function != after.function || before.flags != after.flags ||
		strcmp(before.range, after.range))
		return TRUE;
	if (sr_sw_limits_check(&devc->limits))
		goto stop;
	digits = range_digits(&after, digits);
	sr_analog_init(&analog, &encoding, &meaning, &spec, digits);
	encoding.unitsize = sizeof(value);
	meaning.channels = sdi->channels;
	meaning.mq = functions[after.function].mq;
	meaning.unit = functions[after.function].unit;
	meaning.mqflags = after.flags;
	analog.num_samples = 1;
	analog.data = &value;
	packet.type = SR_DF_ANALOG;
	packet.payload = &analog;
	sr_session_send(sdi, &packet);
	sr_sw_limits_update_samples_read(&devc->limits, 1);
	if (!sr_sw_limits_check(&devc->limits))
		return TRUE;
	goto stop;
error:
	sr_err("Cannot read multimeter measurement (%d).", ret);
stop:
	sr_dev_acquisition_stop(sdi);
	return TRUE;
}

SR_PRIV int hds_dmm_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch = sdi->channels->data;
	int ret;

	if (devc->running || !ch->enabled)
		return SR_ERR_ARG;
	if ((ret = hds_dmm_check(sdi)) != SR_OK)
		return ret;
	/* Poll displayed readings, without changing mode, range or run/hold.
	 * This interval is not the instrument's internal conversion rate.
	 */
	ret = sr_session_source_add(
		sdi->session, -1, 0, 500, receive_data, (void *)sdi);
	if (ret != SR_OK)
		return ret;
	sr_sw_limits_acquisition_start(&devc->limits);
	devc->running = TRUE;
	std_session_send_df_header(sdi);
	return SR_OK;
}

SR_PRIV int hds_dmm_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (!devc->running)
		return SR_OK;
	devc->running = FALSE;
	sr_session_source_remove(sdi->session, -1);
	std_session_send_df_end(sdi);
	return SR_OK;
}
