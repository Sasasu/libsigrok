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

#ifndef LIBSIGROK_OWON_HDS200_H
#define LIBSIGROK_OWON_HDS200_H
#include <glib.h>
#include <libusb.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"
#define LOG_PREFIX "owon-hds200"
#define HDS_MAX_DATA 16384
struct hds_wave {
	unsigned int samples, fullscreen;
	uint64_t samplerate;
	double timebase, scale[2];
	int offset[2];
	gboolean enabled[2];
};
struct dev_context {
	struct sr_sw_limits limits;
	struct hds_wave wave;
	gboolean dmm;
	gboolean running, single, normal, waiting_trigger, restore_run;
	char saved_sweep[16];
};
SR_PRIV int hds_command(const struct sr_dev_inst *sdi, const char *command);
SR_PRIV int hds_query(const struct sr_dev_inst *sdi, const char *command,
	uint8_t *data, size_t capacity, gboolean block, size_t *length);
SR_PRIV int hds_parse_header(
	const uint8_t *data, size_t length, struct hds_wave *wave);
SR_PRIV void hds_decode(const struct hds_wave *wave, unsigned int channel,
	const uint8_t *raw, float *values);
SR_PRIV int hds_read_header(const struct sr_dev_inst *sdi);
SR_PRIV int hds_receive(int fd, int revents, void *cb_data);
SR_PRIV int hds_text(
	const struct sr_dev_inst *sdi, const char *cmd, char *reply);
SR_PRIV int hds_ext_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hds_ext_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hds_ext_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hds_dmm_check(const struct sr_dev_inst *sdi);
SR_PRIV int hds_dmm_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hds_dmm_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hds_dmm_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg);
SR_PRIV int hds_dmm_start(const struct sr_dev_inst *sdi);
SR_PRIV int hds_dmm_stop(struct sr_dev_inst *sdi);
#endif
