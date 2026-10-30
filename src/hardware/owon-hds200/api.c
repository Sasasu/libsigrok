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
#include <inttypes.h>
#include <string.h>
#include "protocol.h"

static const uint32_t scanopts[] = { SR_CONF_CONN };
static const uint32_t drvopts[] = { SR_CONF_OSCILLOSCOPE,
	SR_CONF_SIGNAL_GENERATOR };
static const uint32_t devopts[] = {
	SR_CONF_CONTINUOUS,
	SR_CONF_TRIGGER_SOURCE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_SLOPE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_TRIGGER_LEVEL | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_DEVICE_MODE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_BUFFERSIZE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_LIMIT_FRAMES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_SAMPLES | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_LIMIT_MSEC | SR_CONF_GET | SR_CONF_SET,
	SR_CONF_SAMPLERATE | SR_CONF_GET,
	SR_CONF_TIMEBASE | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_HDIV | SR_CONF_GET,
};
static const uint32_t cgopts[] = {
	SR_CONF_VDIV | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_COUPLING | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_PROBE_FACTOR | SR_CONF_GET | SR_CONF_SET | SR_CONF_LIST,
	SR_CONF_NUM_VDIV | SR_CONF_GET,
};
static const uint64_t timebases[][2] = {
	{ 2, 1000000000 },
	{ 5, 1000000000 },
	{ 10, 1000000000 },
	{ 20, 1000000000 },
	{ 50, 1000000000 },
	{ 100, 1000000000 },
	{ 200, 1000000000 },
	{ 500, 1000000000 },
	{ 1, 1000000 },
	{ 2, 1000000 },
	{ 5, 1000000 },
	{ 10, 1000000 },
	{ 20, 1000000 },
	{ 50, 1000000 },
	{ 100, 1000000 },
	{ 200, 1000000 },
	{ 500, 1000000 },
	{ 1, 1000 },
	{ 2, 1000 },
	{ 5, 1000 },
	{ 10, 1000 },
	{ 20, 1000 },
	{ 50, 1000 },
	{ 100, 1000 },
	{ 200, 1000 },
	{ 500, 1000 },
	{ 1, 1 },

};
static const uint64_t vdivs[][2] = {
	{ 10, 1000 },
	{ 20, 1000 },
	{ 50, 1000 },
	{ 100, 1000 },
	{ 200, 1000 },
	{ 500, 1000 },
	{ 1, 1 },
	{ 2, 1 },
	{ 5, 1 },
	{ 10, 1 },
};
static const uint64_t probes[] = { 1, 10, 100, 1000 };
static const char *couplings[] = { "AC", "DC", "GND" };
static struct sr_dev_driver owon_hds200_driver_info;
static struct sr_dev_driver owon_hds200_dmm_driver_info;

static int dev_close(struct sr_dev_inst *sdi)
{
	struct sr_usb_dev_inst *usb = sdi->conn;
	if (usb->devhdl) {
		libusb_release_interface(usb->devhdl, 0);
		libusb_close(usb->devhdl);
		usb->devhdl = NULL;
	}
	return SR_OK;
}

static int open_usb(struct sr_dev_inst *sdi)
{
	struct drv_context *drvc = sdi->driver->context;
	struct sr_usb_dev_inst *usb = sdi->conn;
	struct libusb_device_descriptor desc;
	int ret = sr_usb_open(drvc->sr_ctx->libusb_ctx, usb);
	if (ret != SR_OK)
		return ret;
	if (libusb_get_device_descriptor(
		    libusb_get_device(usb->devhdl), &desc) ||
		desc.idVendor != 0x5345 || desc.idProduct != 0x1234)
		goto fail;
	/* The HDS "HID" mode actually exposes bulk endpoints, class 5.
	 * Linux can bind the generic OWON USB serial driver instead.
	 * Auto-detach restores it.
	 */
	ret = libusb_set_auto_detach_kernel_driver(usb->devhdl, 1);
	if (ret && ret != LIBUSB_ERROR_NOT_SUPPORTED)
		goto fail;
	if (libusb_claim_interface(usb->devhdl, 0))
		goto fail;
	/* Rebinding the kernel serial driver can leave bulk endpoints stalled.
	 */
	if (libusb_clear_halt(usb->devhdl, 0x01) ||
		libusb_clear_halt(usb->devhdl, 0x81)) {
		libusb_release_interface(usb->devhdl, 0);
		goto fail;
	}
	/* Let the device settle after kernel-driver detach/endpoint recovery.
	 */
	g_usleep(50000);
	return SR_OK;
fail:
	libusb_close(usb->devhdl);
	usb->devhdl = NULL;
	return SR_ERR_IO;
}

static int dev_open(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	int ret = open_usb(sdi);
	if (ret != SR_OK)
		return ret;
	ret = devc->dmm ? hds_dmm_check(sdi) : hds_read_header(sdi);
	if (ret != SR_OK)
		dev_close(sdi);
	return ret;
}

static GSList *scan(struct sr_dev_driver *di, GSList *options)
{
	struct drv_context *drvc = di->context;
	GSList *found, *l, *devices = NULL;
	struct sr_dev_inst *sdi;
	struct sr_usb_dev_inst *usb;
	struct dev_context *devc;
	struct sr_channel *ch;
	struct sr_channel_group *cg;
	struct sr_config *opt;
	struct libusb_device_descriptor desc;
	const char *conn = "5345.1234";
	uint8_t reply[256];
	size_t len;
	char **id, name[8];
	unsigned int i;
	for (l = options; l; l = l->next) {
		opt = l->data;
		if (opt->key == SR_CONF_CONN)
			conn = g_variant_get_string(opt->data, NULL);
	}
	found = sr_usb_find(drvc->sr_ctx->libusb_ctx, conn);
	for (l = found; l; l = l->next) {
		usb = l->data;
		sdi = g_malloc0(sizeof(*sdi));
		sdi->driver = di;
		sdi->inst_type = SR_INST_USB;
		sdi->conn = usb;
		sdi->priv = g_malloc0(sizeof(struct dev_context));
		devc = sdi->priv;
		devc->dmm = di == &owon_hds200_dmm_driver_info;
		if (open_usb(sdi) != SR_OK)
			goto reject;
		if (libusb_get_device_descriptor(
			    libusb_get_device(usb->devhdl), &desc) ||
			desc.idVendor != 0x5345 || desc.idProduct != 0x1234)
			goto reject;
		if (hds_query(sdi, "*IDN?", reply, sizeof(reply) - 1, FALSE,
			    &len) != SR_OK)
			goto reject;
		reply[len] = 0;
		id = g_strsplit(g_strstrip((char *)reply), ",", -1);
		/* Other OWON families share this USB ID. Never identify by
		 * VID/PID alone.
		 */
		if (g_strv_length(id) != 4 ||
			g_ascii_strcasecmp(id[0], "OWON") ||
			strcmp(id[1], "HDS2202S")) {
			g_strfreev(id);
			goto reject;
		}
		sdi->vendor = g_strdup("OWON");
		sdi->model = g_strdup(id[1]);
		sdi->serial_num = g_strdup(id[2]);
		sdi->version = g_strdup(id[3]);
		g_strfreev(id);
		if (!devc->dmm && hds_read_header(sdi) != SR_OK)
			goto reject;
		sdi->connection_id =
			g_strdup_printf("%u.%u", usb->bus, usb->address);
		sdi->status = SR_ST_INACTIVE;
		sr_sw_limits_init(&devc->limits);
		if (devc->dmm) {
			sr_channel_new(sdi, 0, SR_CHANNEL_ANALOG, TRUE, "DMM");
			goto identified;
		}
		for (i = 0; i < 2; i++) {
			g_snprintf(name, sizeof(name), "CH%u", i + 1);
			ch = sr_channel_new(sdi, i, SR_CHANNEL_ANALOG,
				devc->wave.enabled[i], name);
			cg = sr_channel_group_new(sdi, name, NULL);
			cg->channels = g_slist_append(NULL, ch);
		}
		sr_channel_group_new(sdi, "WG", NULL);
		sr_channel_group_new(sdi, "Trigger", NULL);
	identified:
		dev_close(sdi);
		devices = g_slist_append(devices, sdi);
		continue;
	reject:
		dev_close(sdi);
		g_free(sdi->priv);
		sdi->priv = NULL;
		sr_usb_dev_inst_free(usb);
		sr_dev_inst_free(sdi);
	}
	g_slist_free(found);
	return std_scan_complete(di, devices);
}

static int channel_index(
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct sr_channel *ch;
	if (!cg || !g_slist_find(sdi->channel_groups, cg) || !cg->channels)
		return -1;
	ch = cg->channels->data;
	return ch->index;
}

static int config_get(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	int idx = channel_index(sdi, cg), ret;
	char command[40], reply[128];
	double value;
	ret = hds_ext_get(key, data, sdi, cg);
	if (ret != SR_ERR_NA)
		return ret;
	if ((cg || key == SR_CONF_VDIV || key == SR_CONF_PROBE_FACTOR ||
		    key == SR_CONF_COUPLING) &&
		idx < 0)
		return SR_ERR_CHANNEL_GROUP;
	switch (key) {
	case SR_CONF_LIMIT_FRAMES:
	case SR_CONF_LIMIT_SAMPLES:
	case SR_CONF_LIMIT_MSEC:
		return sr_sw_limits_config_get(&devc->limits, key, data);
	case SR_CONF_NUM_HDIV:
		*data = g_variant_new_int32(12);
		return SR_OK;
	case SR_CONF_NUM_VDIV:
		*data = g_variant_new_int32(8);
		return SR_OK;
	case SR_CONF_SAMPLERATE:
		if ((ret = hds_read_header(sdi)) != SR_OK)
			return ret;
		*data = g_variant_new_uint64(devc->wave.samplerate);
		return SR_OK;
	case SR_CONF_TIMEBASE:
		strcpy(command, ":HOR:SCAL?");
		break;
	case SR_CONF_VDIV:
	case SR_CONF_PROBE_FACTOR:
	case SR_CONF_COUPLING:
		if (idx < 0)
			return SR_ERR_CHANNEL_GROUP;
		g_snprintf(command, sizeof(command), ":CH%d:%s?", idx + 1,
			key == SR_CONF_VDIV		      ? "SCAL"
				: key == SR_CONF_PROBE_FACTOR ? "PROB"
							      : "COUP");
		break;
	default:
		return SR_ERR_NA;
	}
	if ((ret = hds_text(sdi, command, reply)) != SR_OK)
		return ret;
	if (key == SR_CONF_COUPLING) {
		*data = g_variant_new_string(reply);
		return SR_OK;
	}
	value = g_ascii_strtod(reply, NULL);
	if (!isfinite(value) || value <= 0)
		return SR_ERR_DATA;
	if (key == SR_CONF_PROBE_FACTOR)
		*data = g_variant_new_uint64(value);
	else if (value < 1)
		*data = g_variant_new("(tt)", (uint64_t)llround(value * 1e9),
			(uint64_t)1000000000);
	else
		*data = g_variant_new(
			"(tt)", (uint64_t)llround(value), (uint64_t)1);
	return SR_OK;
}

static int config_list(uint32_t key, GVariant **data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	GVariant *probe;
	uint64_t scaled[ARRAY_SIZE(vdivs)][2], factor;
	size_t i;
	int ret;
	ret = hds_ext_list(key, data, sdi, cg);
	if (ret != SR_ERR_NA)
		return ret;
	if (key == SR_CONF_SCAN_OPTIONS || key == SR_CONF_DEVICE_OPTIONS) {
		if (cg) {
			*data = std_gvar_array_u32(ARRAY_AND_SIZE(cgopts));
			return SR_OK;
		}
		return STD_CONFIG_LIST(
			key, data, sdi, cg, scanopts, drvopts, devopts);
	}
	switch (key) {
	case SR_CONF_TIMEBASE:
		*data = std_gvar_tuple_array(ARRAY_AND_SIZE(timebases));
		break;
	case SR_CONF_COUPLING:
		*data = g_variant_new_strv(couplings, ARRAY_SIZE(couplings));
		break;
	case SR_CONF_PROBE_FACTOR:
		*data = std_gvar_array_u64(ARRAY_AND_SIZE(probes));
		break;
	case SR_CONF_VDIV:
		if ((ret = config_get(SR_CONF_PROBE_FACTOR, &probe, sdi, cg)) !=
			SR_OK)
			return ret;
		g_variant_ref_sink(probe);
		factor = g_variant_get_uint64(probe);
		g_variant_unref(probe);
		for (i = 0; i < ARRAY_SIZE(vdivs); i++) {
			scaled[i][0] = vdivs[i][0] * factor;
			scaled[i][1] = vdivs[i][1];
		}
		*data = std_gvar_tuple_array(ARRAY_AND_SIZE(scaled));
		break;
	default:
		return SR_ERR_NA;
	}
	return SR_OK;
}

static int config_set(uint32_t key, GVariant *data,
	const struct sr_dev_inst *sdi, const struct sr_channel_group *cg)
{
	struct dev_context *devc = sdi->priv;
	GVariant *choices, *child;
	int idx = channel_index(sdi, cg), ret;
	uint64_t p, q;
	size_t i;
	gboolean valid = FALSE;
	double wanted, allowed;
	char command[80], number[G_ASCII_DTOSTR_BUF_SIZE];
	const char *unit;
	GVariant *actual;
	unsigned int attempt;
	if (key == SR_CONF_LIMIT_FRAMES || key == SR_CONF_LIMIT_SAMPLES ||
		key == SR_CONF_LIMIT_MSEC)
		return sr_sw_limits_config_set(&devc->limits, key, data);
	if (devc->running)
		return SR_ERR_NA;
	ret = hds_ext_set(key, data, sdi, cg);
	if (ret != SR_ERR_NA)
		return ret;
	if ((cg || key == SR_CONF_VDIV || key == SR_CONF_PROBE_FACTOR ||
		    key == SR_CONF_COUPLING) &&
		idx < 0)
		return SR_ERR_CHANNEL_GROUP;
	if ((ret = config_list(key, &choices, sdi, cg)) != SR_OK)
		return ret;
	g_variant_ref_sink(choices);
	wanted = 0;
	if (key == SR_CONF_TIMEBASE || key == SR_CONF_VDIV) {
		g_variant_get(data, "(tt)", &p, &q);
		if (q)
			wanted = (double)p / q;
	}
	for (i = 0; i < g_variant_n_children(choices); i++) {
		child = g_variant_get_child_value(choices, i);
		if (key == SR_CONF_TIMEBASE || key == SR_CONF_VDIV) {
			g_variant_get(child, "(tt)", &p, &q);
			allowed = (double)p / q;
			if (fabs(wanted - allowed) < allowed * 1e-9)
				valid = TRUE;
		} else if (g_variant_equal(data, child))
			valid = TRUE;
		g_variant_unref(child);
	}
	g_variant_unref(choices);
	if (!valid)
		return SR_ERR_ARG;
	switch (key) {
	case SR_CONF_TIMEBASE:
		unit = wanted < 1e-6	? "ns"
			: wanted < 1e-3 ? "us"
			: wanted < 1	? "ms"
					: "s";
		g_ascii_formatd(number, sizeof(number), "%.1f",
			wanted *
				(wanted < 1e-6 ? 1e9
						: wanted < 1e-3 ? 1e6
						: wanted < 1	? 1e3
								: 1));
		g_snprintf(command, sizeof(command), ":HOR:SCAL %s%s", number,
			unit);
		break;
	case SR_CONF_VDIV:
		g_ascii_dtostr(number, sizeof(number), wanted);
		g_snprintf(command, sizeof(command), ":CH%d:SCAL %sV", idx + 1,
			number);
		break;
	case SR_CONF_PROBE_FACTOR:
		g_snprintf(command, sizeof(command), ":CH%d:PROB %" PRIu64 "X",
			idx + 1, g_variant_get_uint64(data));
		break;
	case SR_CONF_COUPLING:
		g_snprintf(command, sizeof(command), ":CH%d:COUP %s", idx + 1,
			g_variant_get_string(data, NULL));
		break;
	default:
		return SR_ERR_NA;
	}
	if ((ret = hds_command(sdi, command)) != SR_OK)
		return ret;
	/* Commands have no acknowledgement. Verify firmware applied the
	 * setting. */
	for (attempt = 0; attempt < 10; attempt++) {
		g_usleep(20000);
		if ((ret = config_get(key, &actual, sdi, cg)) != SR_OK)
			return ret;
		g_variant_ref_sink(actual);
		if (key == SR_CONF_TIMEBASE || key == SR_CONF_VDIV) {
			g_variant_get(actual, "(tt)", &p, &q);
			valid = fabs((double)p / q - wanted) < wanted * 1e-6;
		} else {
			valid = g_variant_equal(actual, data);
		}
		g_variant_unref(actual);
		if (valid)
			return SR_OK;
	}
	sr_err("Instrument did not apply %s.", command);
	return SR_ERR;
}

static int dev_acquisition_start(const struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	struct sr_channel *ch;
	GSList *l;
	char text[128], cmd[40];
	int ret, count = 0;

	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		count += !!ch->enabled;
	}
	if (!count)
		return SR_ERR_ARG;
	if ((ret = hds_text(sdi, ":RUNNING?", text)) != SR_OK)
		return ret;
	devc->restore_run = !g_ascii_strcasecmp(text, "RUN");
	if ((ret = hds_text(sdi, ":TRIG:SWE?", text)) != SR_OK)
		return ret;
	g_strlcpy(devc->saved_sweep, text, sizeof(devc->saved_sweep));
	devc->normal = !g_ascii_strcasecmp(text, "NORMal");
	devc->single = devc->limits.limit_frames == 1 ||
		!g_ascii_strcasecmp(text, "SINGle");
	for (l = sdi->channels; l; l = l->next) {
		ch = l->data;
		g_snprintf(cmd, sizeof(cmd), ":CH%d:DISP %s", ch->index + 1,
			ch->enabled ? "ON" : "OFF");
		if ((ret = hds_command(sdi, cmd)) != SR_OK)
			return ret;
	}
	if ((ret = hds_read_header(sdi)) != SR_OK)
		return ret;
	if (devc->single) {
		if ((ret = hds_command(sdi, ":RUNNING STOP")) != SR_OK ||
			(ret = hds_command(sdi, ":TRIG:SWE SINGle")) != SR_OK ||
			(ret = hds_command(sdi, ":RUNNING RUN")) != SR_OK)
			goto restore;
	}
	devc->waiting_trigger = devc->single;
	ret = sr_session_source_add(
		sdi->session, -1, 0, 50, hds_receive, (void *)sdi);
	if (ret != SR_OK)
		goto restore;
	sr_sw_limits_acquisition_start(&devc->limits);
	devc->running = TRUE;
	std_session_send_df_header(sdi);
	return SR_OK;
restore:
	g_snprintf(cmd, sizeof(cmd), ":TRIG:SWE %s", devc->saved_sweep);
	hds_command(sdi, cmd);
	hds_command(sdi, devc->restore_run ? ":RUNNING RUN" : ":RUNNING STOP");
	return ret;
}

static int dev_acquisition_stop(struct sr_dev_inst *sdi)
{
	struct dev_context *devc = sdi->priv;
	char cmd[64];
	if (!devc->running)
		return SR_OK;
	devc->running = FALSE;
	if (devc->single) {
		hds_command(sdi, ":RUNNING STOP");
		g_snprintf(cmd, sizeof(cmd), ":TRIG:SWE %s", devc->saved_sweep);
		hds_command(sdi, cmd);
		hds_command(sdi,
			devc->restore_run ? ":RUNNING RUN" : ":RUNNING STOP");
	}
	devc->waiting_trigger = FALSE;
	sr_session_source_remove(sdi->session, -1);
	std_session_send_df_end(sdi);
	return SR_OK;
}

static struct sr_dev_driver owon_hds200_driver_info = {
	.name = "owon-hds200",
	.longname = "OWON HDS200 series",
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
SR_REGISTER_DEV_DRIVER(owon_hds200_driver_info);

static struct sr_dev_driver owon_hds200_dmm_driver_info = {
	.name = "owon-hds200-dmm",
	.longname = "OWON HDS200 series multimeter",
	.api_version = 1,
	.init = std_init,
	.cleanup = std_cleanup,
	.scan = scan,
	.dev_list = std_dev_list,
	.dev_clear = std_dev_clear,
	.config_get = hds_dmm_get,
	.config_set = hds_dmm_set,
	.config_list = hds_dmm_list,
	.dev_open = dev_open,
	.dev_close = dev_close,
	.dev_acquisition_start = hds_dmm_start,
	.dev_acquisition_stop = hds_dmm_stop,
};
SR_REGISTER_DEV_DRIVER(owon_hds200_dmm_driver_info);
