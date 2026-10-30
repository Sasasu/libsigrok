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

static int read_packet_unlocked(const struct sr_dev_inst *sdi, const char *command,
		uint8_t *data, struct waveform *wave)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	gint64 deadline;
	char line[64];
	size_t used = 0, expected = 0, i;
	int ret;

	g_snprintf(line, sizeof(line), "%s\n", command);
	if (scpi->send(scpi->priv, line) != SR_OK ||
			sr_scpi_read_begin(scpi) != SR_OK)
		return SR_ERR;
	deadline = g_get_monotonic_time() + scpi->read_timeout_us;
	do {
		if (used == WAVE_MAX_SIZE + 1)
			return SR_ERR_DATA;
		ret = scpi->read_data(scpi->priv, (char *)data + used, WAVE_MAX_SIZE + 1 - used);
		if (ret < 0)
			return ret;
		used += ret;
		if (g_get_monotonic_time() > deadline)
			return SR_ERR_TIMEOUT;
		if (!expected && used >= 11 && data[2]) {
			if (memcmp(data, "#9", 2))
				return SR_ERR_DATA;
			for (i = 2; i < 11; i++) {
				if (data[i] < '0' || data[i] > '9')
					return SR_ERR_DATA;
				expected = expected * 10 + data[i] - '0';
			}
			expected += 11;
			if (expected < WAVE_HEADER_SIZE || expected > WAVE_MAX_SIZE)
				return SR_ERR_DATA;
		}
		if (sr_scpi_read_complete(scpi)) {
			if (!expected || used >= expected)
				break;
			/* Firmware sets EOM after 10240 bytes, even inside a block. */
			if (sr_scpi_read_begin(scpi) != SR_OK)
				return SR_ERR;
		}
	} while (TRUE);
	ret = hantek_dso3000a_parse_waveform(data, used, wave);
	if (ret == SR_ERR_DATA && used >= WAVE_HEADER_SIZE)
		sr_dbg("Rejected waveform: %zu bytes, length %.9s, total %.9s, analog %.4s, digital %.6s.",
			used, data + 2, data + 13, data + 75, data + 112);
	return ret;
}

/* Keep generator queries from interleaving with waveform continuations. */
static int read_packet(const struct sr_dev_inst *sdi, const char *command,
		uint8_t *data, struct waveform *wave)
{
	struct sr_scpi_dev_inst *scpi = sdi->conn;
	int ret;

	g_mutex_lock(&scpi->scpi_mutex);
	ret = read_packet_unlocked(sdi, command, data, wave);
	g_mutex_unlock(&scpi->scpi_mutex);
	return ret;
}

SR_PRIV int hantek_dso3000a_read_waveform(const struct sr_dev_inst *sdi,
		uint8_t *data, struct waveform *wave)
{
	int ret;

	ret = read_packet(sdi, "WAVeform:DATA:DISP", data, wave);
	if (ret == SR_OK && (wave->uploaded || wave->total != wave->payload ||
			wave->samples > DISPLAY_SAMPLES))
		return SR_ERR_DATA;
	return ret;
}

SR_PRIV int hantek_dso3000a_receive_data(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct dev_context *devc = sdi->priv;
	struct waveform wave;
	uint8_t data[WAVE_MAX_SIZE + 1], logic[12000 * 2];
	float values[12000];
	struct sr_datafeed_packet packet;
	struct sr_datafeed_logic logic_packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	struct sr_datafeed_meta meta;
	struct sr_config config;
	struct sr_channel *ch;
	const uint8_t *raw;
	unsigned int i, j, samples;
	gboolean acquiring;
	char *status;
	int ret;

	(void)fd;
	(void)revents;
	if (sr_sw_limits_check(&devc->limits))
		goto stop;
	if (devc->waiting_trigger) {
		/* Firmware can report the previous TRIGed status while waiting
		 * for a new edge. Single sweep stops running only on completion. */
		ret = sr_scpi_get_bool(sdi->conn, "RUNning?", &acquiring);
		if (ret != SR_OK)
			goto stop;
		if (acquiring)
			return TRUE;
		ret = sr_scpi_get_string(sdi->conn, "TRIGger:STATus?", &status);
		if (ret != SR_OK)
			goto stop;
		ret = !g_ascii_strcasecmp(status, "TRIGed");
		g_free(status);
		if (!ret)
			return TRUE;
		devc->waiting_trigger = FALSE;
		if (devc->memory) {
			if (sr_scpi_send(sdi->conn, "ACQuire:STOP") != SR_OK)
				goto stop;
			devc->downloading = TRUE;
			devc->downloaded = 0;
		}
	}
	ret = devc->downloading ? read_packet(sdi, "WAVeform:DATA:ALL", data, &wave) :
		hantek_dso3000a_read_waveform(sdi, data, &wave);
	if (ret == SR_ERR_NA)
		return TRUE;
	if (ret != SR_OK) {
		sr_err("Invalid or incomplete waveform (%d).", ret);
		goto stop;
	}
	if (devc->memory) {
		if (wave.uploaded != devc->downloaded) {
			sr_err("Out-of-sequence memory packet.");
			goto stop;
		}
		if (!devc->downloaded)
			devc->first_packet = wave;
		else if (wave.total != devc->first_packet.total ||
				wave.samplerate != devc->first_packet.samplerate ||
				memcmp(wave.enabled, devc->first_packet.enabled, sizeof(wave.enabled)) ||
				memcmp(wave.scale, devc->first_packet.scale, sizeof(wave.scale)) ||
				memcmp(wave.offset, devc->first_packet.offset, sizeof(wave.offset)) ||
				memcmp(wave.digital_mask, devc->first_packet.digital_mask, sizeof(wave.digital_mask))) {
			sr_err("Memory acquisition changed during download.");
			goto stop;
		}
		devc->downloaded += wave.payload;
	}
	if (wave.samplerate != devc->samplerate) {
		devc->samplerate = wave.samplerate;
		config.key = SR_CONF_SAMPLERATE;
		config.data = g_variant_ref_sink(g_variant_new_uint64(wave.samplerate));
		meta.config = g_slist_append(NULL, &config);
		packet.type = SR_DF_META;
		packet.payload = &meta;
		sr_session_send(sdi, &packet);
		g_slist_free(meta.config);
		g_variant_unref(config.data);
	}
	samples = wave.samples;
	if (devc->limits.limit_samples)
		samples = MIN(samples, devc->limits.limit_samples - devc->limits.samples_read);
	if (!devc->frame_open) {
		std_session_send_df_frame_begin(sdi);
		devc->frame_open = TRUE;
	}
	raw = data + WAVE_HEADER_SIZE;
	for (i = 0; i < 4; i++) {
		if (!wave.enabled[i])
			continue;
		ch = g_slist_nth_data(sdi->channels, i);
		if (ch->enabled) {
			for (j = 0; j < samples; j++)
				/* Header scale already includes the probe factor. */
				values[j] = ((int8_t)raw[j] - wave.offset[i]) *
					wave.scale[i] / 25.0;
			sr_analog_init(&analog, &encoding, &meaning, &spec, 6);
			meaning.channels = g_slist_append(NULL, ch);
			meaning.mq = SR_MQ_VOLTAGE;
			meaning.unit = SR_UNIT_VOLT;
			analog.num_samples = samples;
			analog.data = values;
			packet.type = SR_DF_ANALOG;
			packet.payload = &analog;
			sr_session_send(sdi, &packet);
			g_slist_free(meaning.channels);
		}
		raw += wave.samples;
	}
	memset(logic, 0, sizeof(logic));
	for (i = 0; i < 2; i++) {
		if (!wave.enabled[4 + i])
			continue;
		for (j = 0; j < samples; j++)
			logic[j * 2 + i] = raw[j] & wave.digital_mask[i];
		raw += wave.samples;
	}
	if (wave.enabled[4] || wave.enabled[5]) {
		logic_packet.length = samples * 2;
		logic_packet.unitsize = 2;
		logic_packet.data = logic;
		packet.type = SR_DF_LOGIC;
		packet.payload = &logic_packet;
		sr_session_send(sdi, &packet);
	}
	sr_sw_limits_update_samples_read(&devc->limits, samples);
	if (!devc->memory || devc->downloaded == wave.total) {
		std_session_send_df_frame_end(sdi);
		devc->frame_open = FALSE;
		sr_sw_limits_update_frames_read(&devc->limits, 1);
		if (devc->single && !sr_sw_limits_check(&devc->limits)) {
			devc->downloading = FALSE;
			if (hantek_dso3000a_arm(sdi) != SR_OK)
				goto stop;
		}
	}
	if (!sr_sw_limits_check(&devc->limits))
		return TRUE;
stop:
	sr_dev_acquisition_stop(sdi);
	return TRUE;
}
