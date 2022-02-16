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

#ifndef IMPEGHD_OBJ_REN_DEC_DEFINES_H
#define IMPEGHD_OBJ_REN_DEC_DEFINES_H

#define NUM_MDAP_VEC (18 + 1)
#define DEG_2_RAD ((FLOAT32)(M_PI / 180.0f))
#define RAD_2_DEG (180.0f / M_PI)
#define MDAP_ALPHA_MIN_VAL (0.001f * DEG_2_RAD)
#define MDAP_ALPHA_MAX_VAL (90.0f * DEG_2_RAD)
#define MDAP_ALPHA_MIN_VAL_DEGREE (0.001f)
#define MDAP_ALPHA_MAX_VAL_DEGREE (90.0f)
#define CICP_MAX_NUM_LS (44)
#define ROW_SIZE (9)
#define ANGLE_ZERO_TO_TWO_PI (361)
#define FOUR_BIT (16)
#define SEVEN_BIT (128)
#define ANGLE_ZERO_TO_NINETY (181)
#define VECTOR_COPY_CART(a, b)                                                                   \
                                                                                                 \
  {                                                                                              \
    a.x = b.x;                                                                                   \
    a.y = b.y;                                                                                   \
    a.z = b.z;                                                                                   \
  }

#define NEGATE_COPY_CART(a, b)                                                                   \
                                                                                                 \
  {                                                                                              \
    a.x = -b.x;                                                                                  \
    a.y = -b.y;                                                                                  \
    a.z = -b.z;                                                                                  \
  }

#define IMAG_LS_ELEVATION_HEIGHT_THRESH (21.0f)
#define IMAG_LS_ELEVATION_THRESH (45.0f)
#define IMAG_LS_MAX_AZIMUTH_DIFF (160.0f)

#define OBJ_REN_MIN_PROJ_LEN_VAL (-0.00001f)

#define MIN_VEC_CTR_DIST (0.01f)

#define OBJ_REN_NUM_MDAP_VEC (18)

#define OBJ_REN_MAX_VERTICES (24)

#define UNITGAIN_RANGE 25
#define DOWNMIX_MTX1_ROW 1
#define DOWNMIX_MTX2_ROW 5
#define DOWNMIX_MTX3_ROW 6
#define DOWNMIX_MTX4_10_ROW 4
#define DOWNMIX_MTX5_6_ROW 5
#define DOWNMIX_MTX7_12ROW 7
#define DOWNMIX_MTX9_ROW 3
#define DOWNMIX_MTX11_ROW 6
#define DOWNMIX_MTX14_ROW 7
#define DOWNMIX_MTX15_ROW 10
#define DOWNMIX_MTX16_ROW 9
#define DOWNMIX_MTX17_ROW 11
#define DOWNMIX_MTX19_ROW 14
#define DOWNMIX_RANGE_SET 2
#define UNIT_GAIN_RANGE 25

#endif /* IMPEGHD_OBJ_REN_DEC_STRUCT_DEF_H */