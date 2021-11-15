/* 	Copyright (c) [2020]-[2021] Ittiam Systems Pvt. Ltd.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted (subject to the limitations in the
   disclaimer below) provided that the following conditions are met:
   •	Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   •	Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   •	Neither the names of Dolby Laboratories, Inc. (or its affiliates),
   Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
   to endorse or promote products derived from this software without
   specific prior written permission.

   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
   BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
   LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
   LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
   BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
   IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
---------------------------------------------------------------
*/

#ifndef IMPEGHD_OAM_DEC_DEFINES_H
#define IMPEGHD_OAM_DEC_DEFINES_H

// error_codes
#define INVALID_OAM_FRAMELENGTH (0xFFFF8801)

/* Intra Coded */
#define MAX_OAM_FRAMES (4)
#define MAX_NUM_OAM_OBJS (24)
#define MAX_NUM_OAM_OBJ_ENTITIES_PER_FRAME (MAX_OAM_FRAMES * MAX_NUM_OAM_OBJS)
#define MAX_NUM_EXCLUDED_SECTORS (15)
#define OAM_FLAG_BITS (1)
#define OAM_FRAME_LEN_BITS (6)
#define OAM_I_F_PERIOD_BITS (6)
#define OAM_AZIMUTH_BITS (8)
#define OAM_ELEVATION_BITS (6)
#define OAM_RADIUS_BITS (4)
#define OAM_GAIN_BITS (7)
#define OAM_UNIF_SPREAD_BITS (7)
#define OAM_SPREAD_WIDTH_BITS (7)
#define OAM_SPREAD_HEIGHT_BITS (5)
#define OAM_SPREAD_DEPTH_BITS (4)
#define OAM_DYN_OBJ_PRI_BITS (3)

#define OAM_SINGLE_DYN_OBJ_NBITS (3)

/* Differential Coding */
#define OAM_BITS_PER_POINT_BITS (6)
#define OAM_NBITS_AZIMUTH_BITS (3)
#define OAM_NBITS_ELEVATION_BITS (3)
#define OAM_NBITS_RADIUS_BITS (3)
#define OAM_NBITS_GAIN_BITS (3)
#define OAM_NBITS_SPREAD_BITS (3)
#define OAM_NBITS_DYN_OBJ_PRI_BITS (2)

#define OAM_MAX_POLY_POINTS (64)
#define OAM_MAX_IFRAME_PERIOD (64)

#define OAM_MAX_SPREAD_PARAMS (3)

#endif /* IMPEGHD_OAM_DEFINES_H */
