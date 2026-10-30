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

SR_PRIV int hds_command(const struct sr_dev_inst *sdi, const char *command)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	char *line = g_strconcat(command, "\n", NULL);
	int sent, ret;

	ret = libusb_bulk_transfer(usb->devhdl, 0x01, (unsigned char *)line,
		strlen(line), &sent, 1000);
	if (!ret && sent != (int)strlen(line))
		ret = LIBUSB_ERROR_IO;
	g_free(line);
	/* Firmware has a single command buffer. Back-to-back writes can be
	 * lost. */
	if (!ret)
		g_usleep(strchr(command, '?') ? 20000 : 250000);
	return ret ? SR_ERR_IO : SR_OK;
}

/* Length is a little-endian uint32, excluding the length word itself.
 * Read whole USB packets even when only a short header is expected. */
SR_PRIV int hds_query(const struct sr_dev_inst *sdi, const char *command,
	uint8_t *data, size_t capacity, gboolean block, size_t *length)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	uint8_t packet[512], header[4];
	size_t used = 0, prefix = 0, expected = 0, pos, n;
	gint64 deadline = g_get_monotonic_time() + 2000000;
	int ret, got;

	if ((ret = hds_command(sdi, command)) != SR_OK)
		return ret;
	while (g_get_monotonic_time() < deadline) {
		ret = libusb_bulk_transfer(
			usb->devhdl, 0x81, packet, sizeof(packet), &got, 200);
		if (ret && ret != LIBUSB_ERROR_TIMEOUT)
			return SR_ERR_IO;
		pos = 0;
		if (block && prefix < 4) {
			while (pos < (size_t)got && prefix < 4)
				header[prefix++] = packet[pos++];
			if (prefix < 4)
				continue;
			expected = RL32(header);
			if (!expected || expected > capacity)
				return SR_ERR_DATA;
		}
		n = got - pos;
		if (used + n > capacity || (block && used + n > expected))
			return SR_ERR_DATA;
		memcpy(data + used, packet + pos, n);
		used += n;
		if ((block && used == expected) ||
			(!block && used && data[used - 1] == '\n')) {
			*length = used;
			return SR_OK;
		}
	}
	return SR_ERR_TIMEOUT;
}

SR_PRIV int hds_read_header(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	uint8_t data[HDS_MAX_DATA];
	size_t len;
	unsigned int attempt;
	int ret;

	/* A setting change can temporarily invalidate the screen header. */
	for (attempt = 0; attempt < 3; attempt++) {
		ret = hds_query(sdi, ":DATA:WAVE:SCREEN:HEAD?", data,
			sizeof(data), TRUE, &len);
		if (ret != SR_OK)
			return ret;
		ret = hds_parse_header(data, len, &devc->wave);
		if (ret == SR_OK)
			return ret;
		sr_dbg("Screen header not ready (attempt %u).", attempt + 1);
		g_usleep(100000);
	}
	return ret;
}

SR_PRIV int hds_receive(int fd, int revents, void *cb_data)
{
	struct sr_dev_inst *sdi = cb_data;
	struct dev_context *devc = sdi->priv;
	struct hds_wave *w = &devc->wave;
	uint8_t raw[2][HDS_MAX_DATA];
	size_t lengths[2], samples, i;
	struct sr_channel *ch;
	struct sr_datafeed_packet packet;
	struct sr_datafeed_analog analog;
	struct sr_analog_encoding encoding;
	struct sr_analog_meaning meaning;
	struct sr_analog_spec spec;
	float values[HDS_MAX_DATA / 2];
	char command[40], state[128];
	int ret;

	(void)fd;
	(void)revents;
	if (sr_sw_limits_check(&devc->limits))
		goto stop;
	if (devc->normal && !devc->single) {
		if ((ret = hds_text(sdi, ":TRIG:STAT?", state)) != SR_OK)
			goto error;
		if (g_ascii_strcasecmp(state, "TRIG"))
			return TRUE;
	}
	if (devc->waiting_trigger) {
		if ((ret = hds_text(sdi, ":RUNNING?", state)) != SR_OK)
			goto error;
		if (g_ascii_strcasecmp(state, "STOP"))
			return TRUE;
		/* Acquisition stops before the display buffer finishes
		 * updating. */
		g_usleep(200000);
		devc->waiting_trigger = FALSE;
	}
	if ((ret = hds_read_header(sdi)) != SR_OK)
		goto error;
	/* Fetch and validate both channels before emitting a frame. */
	for (i = 0; i < 2; i++) {
		ch = g_slist_nth_data(sdi->channels, i);
		if (!ch->enabled)
			continue;
		if (!w->enabled[i]) {
			ret = SR_ERR_NA;
			goto error;
		}
		g_snprintf(command, sizeof(command), ":DATA:WAVE:SCREEN:CH%zu?",
			i + 1);
		ret = hds_query(sdi, command, raw[i], sizeof(raw[i]), TRUE,
			&lengths[i]);
		if (ret != SR_OK)
			goto error;
		if (lengths[i] != 2 * w->samples) {
			ret = SR_ERR_DATA;
			goto error;
		}
	}
	sr_session_send_meta(
		sdi, SR_CONF_SAMPLERATE, g_variant_new_uint64(w->samplerate));
	samples = w->samples;
	if (devc->limits.limit_samples)
		samples = MIN(samples,
			devc->limits.limit_samples - devc->limits.samples_read);
	std_session_send_df_frame_begin(sdi);
	for (i = 0; i < 2; i++) {
		ch = g_slist_nth_data(sdi->channels, i);
		if (!ch->enabled)
			continue;
		hds_decode(w, i, raw[i], values);
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
	std_session_send_df_frame_end(sdi);
	sr_sw_limits_update_samples_read(&devc->limits, samples);
	sr_sw_limits_update_frames_read(&devc->limits, 1);
	if (!sr_sw_limits_check(&devc->limits)) {
		if (devc->single) {
			if ((ret = hds_command(sdi, ":RUNNING RUN")) != SR_OK)
				goto error;
			devc->waiting_trigger = TRUE;
		}
		return TRUE;
	}
	goto stop;
error:
	sr_err("Cannot read screen waveform (%d).", ret);
stop:
	sr_dev_acquisition_stop(sdi);
	return TRUE;
}
