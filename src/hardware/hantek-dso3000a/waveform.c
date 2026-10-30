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

/* Fixed-width fields are not NUL terminated. Some firmware writes 0-50. */
static int field_number(const uint8_t *data, size_t len, double *value)
{
	char buf[16], *end;
	size_t i, pos = 0;
	gboolean negative = FALSE;

	if (len >= sizeof(buf))
		return SR_ERR_DATA;
	for (i = 0; i < len; i++) {
		if (data[i] == '-' && i && data[i - 1] == '0' &&
				!memchr(data, 'e', len) && !memchr(data, 'E', len))
			negative = TRUE;
		else
			buf[pos++] = data[i];
	}
	buf[pos] = 0;
	*value = g_ascii_strtod(buf, &end);
	if (end == buf || end != buf + pos || !isfinite(*value))
		return SR_ERR_DATA;
	if (negative)
		*value = -*value;
	return SR_OK;
}

SR_PRIV int hantek_dso3000a_parse_waveform(const uint8_t *data, size_t len,
		struct waveform *wave)
{
	double length, total, uploaded, value;
	unsigned int i, count = 0;

	memset(wave, 0, sizeof(*wave));
	if (len < WAVE_HEADER_SIZE || memcmp(data, "#9", 2))
		return SR_ERR_DATA;
	/* Idle responses contain zeroes; 256-byte USB replies end in 0xff. */
	if (!data[2]) {
		if (len != WAVE_HEADER_SIZE && len != 256)
			return SR_ERR_DATA;
		for (i = 2; i < len; i++)
			if (data[i] && !(len == 256 && i == 255 && data[i] == 0xff))
				return SR_ERR_DATA;
		return SR_ERR_NA;
	}
	if (field_number(data + 2, 9, &length) != SR_OK ||
			length < 117 || length > WAVE_MAX_SIZE - 11 ||
			length != floor(length) || len < length + 11)
		return SR_ERR_DATA;
	if (len != length + 11 &&
			!(len == length + 12 && data[len - 1] == '\n'))
		return SR_ERR_DATA;
	if (field_number(data + 13, 9, &total) != SR_OK ||
			field_number(data + 22, 9, &uploaded) != SR_OK ||
			total < 1 || total > 768000000 || total != floor(total) ||
			uploaded < 0 || uploaded != floor(uploaded) ||
			uploaded + length - 117 > total)
		return SR_ERR_DATA;
	for (i = 0; i < 4; i++) {
		if (data[75 + i] != '0' && data[75 + i] != '1')
			return SR_ERR_DATA;
		wave->enabled[i] = data[75 + i] == '1';
		if (!wave->enabled[i])
			continue;
		count++;
		if (field_number(data + 31 + i * 4, 4, &value) != SR_OK ||
				value < -255 || value > 255 || value != floor(value))
			return SR_ERR_DATA;
		wave->offset[i] = value;
		if (field_number(data + 47 + i * 7, 7, &wave->scale[i]) != SR_OK ||
				wave->scale[i] <= 0)
			return SR_ERR_DATA;
	}
	for (i = 0; i < 2; i++) {
		if (field_number(data + 112 + i * 3, 3, &value) != SR_OK ||
				value < 0 || value > 255 || value != floor(value))
			return SR_ERR_DATA;
		wave->digital_mask[i] = value;
		wave->enabled[4 + i] = value != 0;
		count += wave->enabled[4 + i];
	}
	if (!count || (unsigned int)total % count ||
			(unsigned int)uploaded % count ||
			(unsigned int)(length - 117) % count || length <= 117 ||
			total / count > 128000000)
		return SR_ERR_DATA;
	wave->total = total;
	wave->uploaded = uploaded;
	wave->payload = length - 117;
	wave->samples = wave->payload / count;
	if (field_number(data + 79, 9, &value) != SR_OK || value < 1 || value > 1e9)
		return SR_ERR_DATA;
	wave->samplerate = value;
	return SR_OK;
}
