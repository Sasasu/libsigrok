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

static const uint32_t scanopts[] = { SR_CONF_CONN };
static const uint32_t drvopts[] = { SR_CONF_OSCILLOSCOPE, SR_CONF_LOGIC_ANALYZER,
	SR_CONF_SIGNAL_GENERATOR };
static const uint32_t devopts[] = {
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DATA_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFERSIZE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
};
static const uint32_t cgopts[] = {
	SR_CONF_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_VDIV | SR_CONF_GET,
	SR_CONF_OFFSET | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_INVERTED | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_FILTER | SR_CONF_GET | SR_CONF_SET,
};
static const uint64_t depths[] = {
	1600, 16000, 160000, 1600000, 16000000, 32000000, 64000000, 128000000,
};
static const char *data_sources[] = { "Screen", "Memory" };
static const uint64_t timebases[][2] = {
	{2, 1000000000}, {5, 1000000000}, {10, 1000000000},
	{20, 1000000000}, {50, 1000000000}, {100, 1000000000},
	{200, 1000000000}, {500, 1000000000},
	{1, 1000000}, {2, 1000000}, {5, 1000000}, {10, 1000000},
	{20, 1000000}, {50, 1000000}, {100, 1000000},
	{200, 1000000}, {500, 1000000},
	{1, 1000}, {2, 1000}, {5, 1000}, {10, 1000}, {20, 1000},
	{50, 1000}, {100, 1000}, {200, 1000}, {500, 1000},
	{1, 1}, {2, 1}, {5, 1}, {10, 1}, {20, 1}, {50, 1},
};
static const uint64_t vdivs[][2] = {
	{1, 1000}, {2, 1000}, {5, 1000}, {10, 1000}, {20, 1000},
	{50, 1000}, {100, 1000}, {200, 1000}, {500, 1000},
	{1, 1}, {2, 1}, {5, 1}, {10, 1},
};
static const char *couplings[] = { "AC", "DC", "GND" };
static const uint64_t probes[] = { 1, 10, 100, 1000 };
static struct sr_dev_driver hantek_dso3000a_driver_info;

static struct sr_dev_inst *probe_device(struct sr_scpi_dev_inst *scpi)
{
	struct sr_scpi_hw_info *hw;
	struct sr_dev_inst *sdi;
	struct dev_context *devc;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	char *setup, **fields, name[8];
	unsigned int i, mask[2] = {0, 0};

	if (sr_scpi_get_hw_id(scpi, &hw) != SR_OK)
		return NULL;
	if (g_ascii_strcasecmp(hw->manufacturer, "hantek") ||
			strcmp(hw->model, "DSO3254A")) {
		sr_scpi_hw_info_free(hw);
		return NULL;
	}
	if (sr_scpi_get_string(scpi, "SETUp:ALL?", &setup) != SR_OK) {
		sr_scpi_hw_info_free(hw);
		return NULL;
	}
	fields = g_strsplit(setup, ";", -1);
	g_free(setup);
	if (g_strv_length(fields) < 28) {
		g_strfreev(fields);
		sr_scpi_hw_info_free(hw);
		return NULL;
	}
	sdi = g_malloc0(sizeof(*sdi));
	sdi->vendor = g_strdup("Hantek");
	sdi->model = g_strdup(hw->model);
	sdi->version = g_strdup(hw->firmware_version);
	sdi->serial_num = g_strdup(hw->serial_number);
	sdi->driver = &hantek_dso3000a_driver_info;
	sdi->inst_type = SR_INST_SCPI;
	sdi->conn = scpi;
	sr_scpi_hw_info_free(hw);
	devc = g_malloc0(sizeof(*devc));
	sdi->priv = devc;
	sr_sw_limits_init(&devc->limits);
	for (i = 0; i < 4; i++) {
		g_snprintf(name, sizeof(name), "CH%u", i + 1);
		ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG,
				!g_ascii_strcasecmp(fields[i * 7], "ON"), name);
		cg = sr_channel_group_new(sdi, name, NULL);
		cg->channels = g_slist_append(NULL, ch);
		devc->probe[i] = g_ascii_strtod(fields[i * 7 + 3], NULL);
	}
	/* SETUp:ALL ends with the two POD masks and their thresholds. */
	if (g_strv_length(fields) >= 66) {
		mask[0] = g_ascii_strtoull(fields[62], NULL, 10);
		mask[1] = g_ascii_strtoull(fields[63], NULL, 10);
	}
	devc->pods[0] = sr_channel_group_new(sdi, "POD1", NULL);
	devc->pods[1] = sr_channel_group_new(sdi, "POD2", NULL);
	for (i = 0; i < 16; i++) {
		g_snprintf(name, sizeof(name), "D%u", i);
		ch = sr_channel_new(sdi, i, SR_CHANNEL_LOGIC,
				(mask[i / 8] >> (i % 8)) & 1, name);
		devc->pods[i / 8]->channels = g_slist_append(devc->pods[i / 8]->channels, ch);
	}
	devc->generator = sr_channel_group_new(sdi, "WG", NULL);
	g_strfreev(fields);
	return sdi;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	return sr_scpi_scan(di->context, options, probe_device);
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct waveform wave;
	uint8_t data[WAVE_MAX_SIZE + 1];
	int ret;

	if ((ret = sr_scpi_open(sdi->conn)) != SR_OK)
		return ret;
	ret = hantek_dso3000a_read_waveform(sdi, data, &wave);
	if (ret == SR_OK)
		devc->samplerate = wave.samplerate;
	else if (ret != SR_ERR_NA) {
		sr_scpi_close(sdi->conn);
		return ret;
	}
	return SR_OK;
}

static int dev_close(struct sr_dev_inst *sdi)
{
	return sr_scpi_close(sdi->conn);
}

static int channel_index(const struct sr_dev_inst *sdi,
		const struct sr_channel_group *cg)
{
	struct sr_channel *ch;

	if (!cg || !g_slist_find(sdi->channel_groups, cg) || !cg->channels)
		return -1;
	ch = cg->channels->data;
	return ch->type == SR_CHANNEL_ANALOG ? ch->index : -1;
}

static int config_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	char command[64], *text;
	double value, listed;
	const uint64_t (*ratios)[2];
	unsigned int i, count;
	uint64_t factor;
	int idx = channel_index(sdi, cg), ret;

	if (cg && cg == devc->generator)
		return hantek_dso3000a_generator_get(key, data, sdi);
	if ((cg && (cg == devc->pods[0] || cg == devc->pods[1])) ||
			key == SR_CONF_TRIGGER_SOURCE || key == SR_CONF_TRIGGER_SLOPE ||
			key == SR_CONF_TRIGGER_LEVEL || key == SR_CONF_OFFSET ||
			key == SR_CONF_INVERTED || key == SR_CONF_FILTER)
		return hantek_dso3000a_control_get(key, data, sdi, cg);
	if (cg && idx < 0)
		return SR_ERR_ARG;
	if (!cg) {
		ret = sr_sw_limits_config_get(&devc->limits, key, data);
		if (ret != SR_ERR_NA)
			return ret;
	}
	switch (key) {
	case SR_CONF_DATA_SOURCE:
		*data = g_variant_new_string(data_sources[devc->memory]);
		return SR_OK;
	case SR_CONF_BUFFERSIZE:
		ret = sr_scpi_get_int(sdi->conn, "ACQuire:MDEPth?", &idx);
		if (ret != SR_OK)
			return ret;
		if (idx < 0 || (unsigned int)idx >= ARRAY_SIZE(depths))
			return SR_ERR_DATA;
		*data = g_variant_new_uint64(depths[idx]);
		return SR_OK;
	case SR_CONF_NUM_HDIV:
		*data = g_variant_new_int32(16);
		return SR_OK;
	case SR_CONF_NUM_VDIV:
		if (idx < 0)
			return SR_ERR_CHANNEL_GROUP;
		*data = g_variant_new_int32(8);
		return SR_OK;
	case SR_CONF_SAMPLERATE:
		*data = g_variant_new_uint64(devc->samplerate);
		return SR_OK;
	case SR_CONF_TIMEBASE:
		g_strlcpy(command, "TIMebase:MAIN:SCALe?", sizeof(command));
		break;
	case SR_CONF_VDIV:
	case SR_CONF_PROBE_FACTOR:
	case SR_CONF_COUPLING:
		if (idx < 0)
			return SR_ERR_CHANNEL_GROUP;
		g_snprintf(command, sizeof(command), "CHANnel%d:%s?", idx + 1,
			key == SR_CONF_VDIV ? "SCALe" :
			key == SR_CONF_PROBE_FACTOR ? "PROBe" : "COUPling");
		break;
	default:
		return SR_ERR_NA;
	}
	if ((ret = sr_scpi_get_string(sdi->conn, command, &text)) != SR_OK)
		return ret;
	value = g_ascii_strtod(text, NULL);
	if (key == SR_CONF_COUPLING)
		*data = g_variant_new_string(text);
	else if (key == SR_CONF_PROBE_FACTOR)
		*data = g_variant_new_uint64(value);
	else {
		ratios = key == SR_CONF_TIMEBASE ? timebases : vdivs;
		count = key == SR_CONF_TIMEBASE ? ARRAY_SIZE(timebases) : ARRAY_SIZE(vdivs);
		factor = key == SR_CONF_TIMEBASE ? 1 : devc->probe[idx];
		for (i = 0; i < count; i++) {
			listed = (double)ratios[i][0] * factor / ratios[i][1];
			if (fabs(value - listed) <= listed * 1e-6)
				break;
		}
		if (i < count)
			*data = g_variant_new("(tt)", ratios[i][0] * factor, ratios[i][1]);
		else if (isfinite(value) && value > 0 && value <= 10000)
			*data = g_variant_new("(tt)", (uint64_t)llround(value * 1e12),
					(uint64_t)1000000000000ULL);
		else {
			g_free(text);
			return SR_ERR_DATA;
		}
	}
	g_free(text);
	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	int idx = channel_index(sdi, cg), ret;
	uint64_t p, q, factor;
	char number[G_ASCII_DTOSTR_BUF_SIZE];
	const char *coupling;
	unsigned int i;

	if (cg && cg == devc->generator)
		return hantek_dso3000a_generator_set(key, data, sdi);
	if ((cg && (cg == devc->pods[0] || cg == devc->pods[1])) ||
			key == SR_CONF_TRIGGER_SOURCE || key == SR_CONF_TRIGGER_SLOPE ||
			key == SR_CONF_TRIGGER_LEVEL || key == SR_CONF_OFFSET ||
			key == SR_CONF_INVERTED || key == SR_CONF_FILTER)
		return hantek_dso3000a_control_set(key, data, sdi, cg);
	if (cg && idx < 0)
		return SR_ERR_ARG;
	if (!cg) {
		ret = sr_sw_limits_config_set(&devc->limits, key, data);
		if (ret != SR_ERR_NA)
			return ret;
	}
	if (devc->running)
		return SR_ERR;
	if (!cg && key == SR_CONF_DATA_SOURCE) {
		for (i = 0; i < ARRAY_SIZE(data_sources); i++)
			if (!strcmp(g_variant_get_string(data, NULL), data_sources[i])) {
				devc->memory = i;
				return SR_OK;
			}
		return SR_ERR_ARG;
	}
	if (!cg && key == SR_CONF_BUFFERSIZE) {
		for (i = 0; i < ARRAY_SIZE(depths); i++)
			if (g_variant_get_uint64(data) == depths[i])
				return sr_scpi_send(sdi->conn, "ACQuire:MDEPth %u", i);
		return SR_ERR_ARG;
	}
	if (key == SR_CONF_TIMEBASE || key == SR_CONF_VDIV) {
		if (key == SR_CONF_VDIV && idx < 0)
			return SR_ERR_CHANNEL_GROUP;
		g_variant_get(data, "(tt)", &p, &q);
		if (!p || !q)
			return SR_ERR_ARG;
		g_ascii_dtostr(number, sizeof(number), (double)p / q);
		if (key == SR_CONF_TIMEBASE) {
			if ((double)p / q < 2e-9 || (double)p / q > 50)
				return SR_ERR_ARG;
			return sr_scpi_send(sdi->conn, "TIMebase:MAIN:SCALe %s", number);
		}
		if ((double)p / q < .001 * devc->probe[idx] ||
				(double)p / q > 10 * devc->probe[idx])
			return SR_ERR_ARG;
		return sr_scpi_send(sdi->conn, "CHANnel%d:SCALe %s", idx + 1, number);
	}
	if (idx < 0)
		return SR_ERR_CHANNEL_GROUP;
	switch (key) {
	case SR_CONF_PROBE_FACTOR:
		factor = g_variant_get_uint64(data);
		for (i = 0; i < ARRAY_SIZE(probes); i++)
			if (factor == probes[i])
				break;
		if (i == ARRAY_SIZE(probes))
			return SR_ERR_ARG;
		ret = sr_scpi_send(sdi->conn, "CHANnel%d:PROBe %" PRIu64, idx + 1, factor);
		if (ret == SR_OK)
			devc->probe[idx] = factor;
		return ret;
	case SR_CONF_COUPLING:
		coupling = g_variant_get_string(data, NULL);
		for (i = 0; i < ARRAY_SIZE(couplings); i++)
			if (!strcmp(coupling, couplings[i]))
				return sr_scpi_send(sdi->conn, "CHANnel%d:COUPling %s", idx + 1, coupling);
		return SR_ERR_ARG;
	default:
		return SR_ERR_NA;
	}
}

static int config_list(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc;
	uint64_t scaled[ARRAY_SIZE(vdivs)][2];
	unsigned int i;
	int idx;

	if (sdi && cg && cg == ((struct dev_context *)sdi->priv)->generator)
		return hantek_dso3000a_generator_list(key, data);
	if (sdi && cg && (cg == ((struct dev_context *)sdi->priv)->pods[0] ||
			cg == ((struct dev_context *)sdi->priv)->pods[1])) {
		static const uint32_t options[] = {
			SR_CONF_LOGIC_THRESHOLD_CUSTOM | SR_CONF_GET | SR_CONF_SET,
		};
		if (key != SR_CONF_DEVICE_OPTIONS)
			return SR_ERR_NA;
		*data = std_gvar_array_u32(ARRAY_AND_SIZE(options));
		return SR_OK;
	}
	if (!cg && (key == SR_CONF_TRIGGER_SOURCE || key == SR_CONF_TRIGGER_SLOPE))
		return hantek_dso3000a_control_list(key, data);
	if (key == SR_CONF_SCAN_OPTIONS || key == SR_CONF_DEVICE_OPTIONS) {
		if (cg) {
			if (channel_index(sdi, cg) < 0)
				return SR_ERR_ARG;
			*data = std_gvar_array_u32(ARRAY_AND_SIZE(cgopts));
			return SR_OK;
		}
		return STD_CONFIG_LIST(key, data, sdi, cg, scanopts, drvopts, devopts);
	}
	switch (key) {
	case SR_CONF_DATA_SOURCE:
		*data = g_variant_new_strv(data_sources, ARRAY_SIZE(data_sources));
		break;
	case SR_CONF_BUFFERSIZE:
		*data = std_gvar_array_u64(ARRAY_AND_SIZE(depths));
		break;
	case SR_CONF_TIMEBASE:
		*data = std_gvar_tuple_array(ARRAY_AND_SIZE(timebases));
		break;
	case SR_CONF_VDIV:
		if (!sdi || (idx = channel_index(sdi, cg)) < 0)
			return SR_ERR_CHANNEL_GROUP;
		devc = sdi->priv;
		for (i = 0; i < ARRAY_SIZE(vdivs); i++) {
			scaled[i][0] = vdivs[i][0] * devc->probe[idx];
			scaled[i][1] = vdivs[i][1];
		}
		*data = std_gvar_tuple_array(ARRAY_AND_SIZE(scaled));
		break;
	case SR_CONF_COUPLING:
		*data = g_variant_new_strv(couplings, ARRAY_SIZE(couplings));
		break;
	case SR_CONF_PROBE_FACTOR:
		*data = std_gvar_array_u64(ARRAY_AND_SIZE(probes));
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	GSList *l;
	unsigned int masks[2] = {0, 0}, enabled = 0;
	int ret;
	char command[64], *sweep;

	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		enabled += ch->enabled;
		if (ch->type == SR_CHANNEL_LOGIC && ch->enabled)
			masks[ch->index / 8] |= 1 << (ch->index % 8);
	}
	if (!enabled)
		return SR_ERR_ARG;
	if (sr_scpi_send(sdi->conn, "LA:POD1:STATe %s", masks[0] ? "ON" : "OFF") != SR_OK ||
			sr_scpi_send(sdi->conn, "LA:POD2:STATe %s", masks[1] ? "ON" : "OFF") != SR_OK)
		return SR_ERR;
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		if (ch->type == SR_CHANNEL_ANALOG) {
			ret = sr_scpi_send(sdi->conn, "CHANnel%d:DISPlay %s",
				ch->index + 1, ch->enabled ? "ON" : "OFF");
			if (ret != SR_OK)
				return ret;
			if (!ch->enabled)
				continue;
			g_snprintf(command, sizeof(command), "CHANnel%d:PROBe?", ch->index + 1);
			ret = sr_scpi_get_double(sdi->conn, command, &devc->probe[ch->index]);
		} else {
			ret = sr_scpi_send(sdi->conn, "LA:SOURce D%d,%d", ch->index, ch->enabled);
		}
		if (ret != SR_OK)
			return ret;
	}
	devc->single = devc->memory || devc->limits.limit_frames == 1;
	ret = sr_scpi_get_string(sdi->conn, "TRIGger:SWEep?", &sweep);
	if (ret != SR_OK)
		return ret;
	if (g_ascii_strcasecmp(sweep, "AUTO") &&
			g_ascii_strcasecmp(sweep, "NORMal") &&
			g_ascii_strcasecmp(sweep, "SINGle")) {
		g_free(sweep);
		return SR_ERR_DATA;
	}
	g_strlcpy(devc->saved_sweep, sweep, sizeof(devc->saved_sweep));
	g_free(sweep);
	ret = hantek_dso3000a_arm(sdi);
	if (ret == SR_OK)
		ret = sr_scpi_source_add(sdi->session, sdi->conn, G_IO_IN, 50,
				hantek_dso3000a_receive_data, (void *)sdi);
	if (ret != SR_OK) {
		sr_scpi_send(sdi->conn, "RUNning OFF");
		sr_scpi_send(sdi->conn, "TRIGger:SWEep %s", devc->saved_sweep);
		devc->waiting_trigger = FALSE;
		return ret;
	}
	sr_sw_limits_acquisition_start(&devc->limits);
	devc->samplerate = 0; /* Send fresh metadata with the first frame. */
	devc->downloading = FALSE;
	devc->downloaded = 0;
	devc->frame_open = FALSE;
	devc->running = TRUE;
	std_session_send_df_header(sdi);
	return SR_OK;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;

	if (!devc->running)
		return SR_OK;
	devc->running = FALSE;
	if (devc->memory)
		sr_scpi_send(sdi->conn, "ACQuire:STOP");
	if (devc->frame_open) {
		std_session_send_df_frame_end(sdi);
		devc->frame_open = FALSE;
	}
	sr_scpi_source_remove(sdi->session, sdi->conn);
	sr_scpi_send(sdi->conn, "RUNning OFF");
	sr_scpi_send(sdi->conn, "TRIGger:SWEep %s", devc->saved_sweep);
	devc->waiting_trigger = FALSE;
	std_session_send_df_end(sdi);
	return SR_OK;
}

static struct sr_dev_driver hantek_dso3000a_driver_info = {
	.name = "hantek-dso3000a",
	.longname = "Hantek DSO3000A series",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = config_get,
	.config_set = config_set,
	.config_list = config_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = dev_acquisition_start,
	.dev_acquisition_stop = dev_acquisition_stop,
};
SR_REGISTER_DEV_DRIVER(hantek_dso3000a_driver_info);
