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
#include <json-c/json.h>
#include "protocol.h"

static struct json_object *member(struct json_object *obj, const char *name)
{
	if (!obj || !json_object_is_type(obj, json_type_object))
		return NULL;
	json_object_object_foreach(obj, key, value)
	{
		if (!g_ascii_strcasecmp(key, name))
			return value;
	}
	return NULL;
}

static double quantity(struct json_object *obj)
{
	const char *s = json_object_get_string(obj);
	char *end;
	double v;
	if (!s)
		return NAN;
	v = g_ascii_strtod(s, &end);
	if (end == s)
		return NAN;
	while (g_ascii_isspace(*end))
		end++;
	switch (*end) {
	case 'n':
		v *= 1e-9;
		break;
	case 'u':
		v *= 1e-6;
		break;
	case 'm':
		v *= 1e-3;
		break;
	case 'k':
	case 'K':
		v *= 1e3;
		break;
	case 'V':
	case 'v':
	case 's':
	case 0:
		break;
	default:
		return NAN;
	}
	return v;
}

SR_PRIV int hds_parse_header(
	const uint8_t *data, size_t length, struct hds_wave *wave)
{
	struct json_tokener *tok;
	struct json_object *root, *sample, *channels, *ch, *v;
	struct hds_wave w = { 0 };
	const char *s;
	double rate;
	int i, ret = SR_ERR_DATA, n;

	tok = json_tokener_new();
	root = json_tokener_parse_ex(tok, (const char *)data, length);
	if (json_tokener_get_error(tok) != json_tokener_success)
		goto out;
	w.timebase = quantity(member(member(root, "timebase"), "scale"));
	sample = member(root, "sample");
	n = json_object_get_int(member(sample, "datalen"));
	if (n < 1 || n > HDS_MAX_DATA / 2)
		goto out;
	if (n % 2)
		goto out;
	w.samples = n / 2;
	n = json_object_get_int(member(sample, "fullscreen"));
	if (n < (int)w.samples * 2 || n > HDS_MAX_DATA / 2)
		goto out;
	w.fullscreen = n;
	s = json_object_get_string(member(sample, "type"));
	if (!s ||
		(g_ascii_strcasecmp(s, "sample") &&
			g_ascii_strcasecmp(s, "samp")))
		goto out;
	/* Screen data is decimated: SAMPLE.SAMPLERATE is the ADC rate,
	 * not the rate of the downloaded samples. There are 12 horizontal divs
	 * and two bytes per displayed point. */
	rate = w.fullscreen / (24.0 * w.timebase);
	if (!isfinite(rate) || rate < 1 || rate > 1e12 ||
		fabs(rate - round(rate)) > rate * 1e-9)
		goto out;
	w.samplerate = llround(rate);
	channels = member(root, "channel");
	if (!json_object_is_type(channels, json_type_array) ||
		json_object_array_length(channels) != 2)
		goto out;
	for (i = 0; i < 2; i++) {
		ch = json_object_array_get_idx(channels, i);
		s = json_object_get_string(member(ch, "name"));
		if (!s || g_ascii_strcasecmp(s, i ? "CH2" : "CH1"))
			goto out;
		s = json_object_get_string(member(ch, "display"));
		if (!s ||
			(g_ascii_strcasecmp(s, "on") &&
				g_ascii_strcasecmp(s, "off")))
			goto out;
		w.enabled[i] = !g_ascii_strcasecmp(s, "on");
		w.scale[i] = quantity(member(ch, "scale"));
		v = member(ch, "offset");
		if (!json_object_is_type(v, json_type_int))
			goto out;
		w.offset[i] = json_object_get_int(v);
		if (!isfinite(w.scale[i]) || w.scale[i] <= 0 ||
			(w.offset[i] < -2000 || w.offset[i] > 2000))
			goto out;
	}
	*wave = w;
	ret = SR_OK;
out:
	if (root)
		json_object_put(root);
	json_tokener_free(tok);
	return ret;
}

/* The vendor PC software uses the signed first byte of each pair and
 * duplicates it horizontally. The second byte is NOT an int16 high byte
 * (on V2.5.1 it can differ even in sign near an edge). Emit each actual
 * display point once, at half the header's horizontal byte-coordinate rate.
 * SCALE already includes probe attenuation; OFFSET is in 25 codes/div. */
SR_PRIV void hds_decode(const struct hds_wave *wave, unsigned int channel,
	const uint8_t *raw, float *values)
{
	unsigned int i;
	for (i = 0; i < wave->samples; i++)
		values[i] = ((int8_t)raw[2 * i] - wave->offset[channel]) *
			wave->scale[channel] / 25.0;
}
