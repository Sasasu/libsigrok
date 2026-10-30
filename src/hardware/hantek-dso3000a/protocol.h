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

#ifndef LIBSIGROK_HARDWARE_HANTEK_DSO3000A_PROTOCOL_H
#define LIBSIGROK_HARDWARE_HANTEK_DSO3000A_PROTOCOL_H

#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#include "scpi.h"

#define LOG_PREFIX "hantek-dso3000a"
#define DISPLAY_SAMPLES 1600
#define WAVE_HEADER_SIZE 128
#define WAVE_MAX_SIZE (WAVE_HEADER_SIZE + 12000)

struct waveform {
	unsigned int samples;
	uint32_t total, uploaded, payload;
	gboolean enabled[6];
	int offset[4];
	double scale[4];
	uint8_t digital_mask[2];
	uint64_t samplerate;
};

struct dev_context {
	struct sr_sw_limits limits;
	struct sr_channel_group *generator;
	struct sr_channel_group *pods[2];
	double probe[4];
	uint64_t samplerate;
	gboolean running;
	gboolean single, waiting_trigger;
	char saved_sweep[16];
	gboolean memory, downloading, frame_open;
	uint32_t downloaded;
	struct waveform first_packet;
};

SR_PRIV int hantek_dso3000a_parse_waveform(const uint8_t *data, size_t len,
		struct waveform *wave);
SR_PRIV int hantek_dso3000a_read_waveform(const struct sr_dev_inst *sdi,
		uint8_t *data, struct waveform *wave);
SR_PRIV int hantek_dso3000a_receive_data(int fd, int revents, void *cb_data);

SR_PRIV int hantek_dso3000a_generator_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi);
SR_PRIV int hantek_dso3000a_generator_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi);
SR_PRIV int hantek_dso3000a_generator_list(uint32_t key, GVariant **data);

SR_PRIV int hantek_dso3000a_control_get(uint32_t key, GVariant **data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hantek_dso3000a_control_set(uint32_t key, GVariant *data,
		const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hantek_dso3000a_control_list(uint32_t key, GVariant **data);
SR_PRIV int hantek_dso3000a_arm(const struct sr_dev_inst *sdi);

#endif
