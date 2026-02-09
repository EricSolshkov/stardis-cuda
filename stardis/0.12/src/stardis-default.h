/* Copyright (C) 2018-2022 |Meso|Star>
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
 * along with this program. If not, see <http://www.gnu.org/licenses/>. */

#ifndef STARDIS_DEFAULT_H
#define STARDIS_DEFAULT_H
#define STARDIS_RENDERING_FMT(Fmt) STARDIS_RENDERING_OUTPUT_FILE_FMT_ ## Fmt

#define STARDIS_DEFAULT_TRAD 300
#define STARDIS_DEFAULT_TRAD_REFERENCE 300
#define STARDIS_DEFAULT_COMPUTE_TIME INF
#define STARDIS_DEFAULT_PICARD_ORDER 1
#define STARDIS_DEFAULT_RENDERING_FOV 70
#define STARDIS_DEFAULT_RENDERING_IMG_HEIGHT 480
#define STARDIS_DEFAULT_RENDERING_IMG_WIDTH 640
#define STARDIS_DEFAULT_RENDERING_OUTPUT_FILE_FMT STARDIS_RENDERING_FMT( HT )
#define STARDIS_DEFAULT_RENDERING_POS 1,1,1
#define STARDIS_DEFAULT_RENDERING_SPP 4
#define STARDIS_DEFAULT_RENDERING_TGT 0,0,0
#define STARDIS_DEFAULT_RENDERING_TIME INF,INF
#define STARDIS_DEFAULT_RENDERING_UP 0,0,1
#define STARDIS_DEFAULT_SAMPLES_COUNT 10000
#define STARDIS_DEFAULT_SCALE_FACTOR 1
#define STARDIS_DEFAULT_VERBOSE_LEVEL 1

#endif /* STARDIS_DEFAULT_H */

