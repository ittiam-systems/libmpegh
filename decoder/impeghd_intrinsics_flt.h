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

#ifndef IMPEGHD_INTRINSICS_FLT_H
#define IMPEGHD_INTRINSICS_FLT_H
#define ITT_INTRINSICS
#include <math.h>
#include <stdlib.h>
#define ia_sqrt_flt sqrt
#define ia_floor_flt floor
#define ia_fabs_flt(a) ((FLOAT32)fabs(a))
#ifdef WIN32
#define ia_max_flt(a, b) (max(a, b))
#define ia_min_flt(a, b) (min(a, b))
#define ia_max_int(a, b) (max(a, b))
#define ia_min_int(a, b) (min(a, b))
#else
#define ia_max_flt(a, b) (fmax(a, b))
#define ia_min_flt(a, b) (fmin(a, b))
#define ia_max_int(a, b) (((a) > (b)) ? (a) : (b))
#define ia_min_int(a, b) (((a) < (b)) ? (a) : (b))
#define ia_eq_int(a, b) ((a) == (b) ? (1) : (0))
#endif
#define ia_abs_int abs
#define ia_negate_flt(a) (-a)
#define ia_sub_flt(a, b) ((a) - (b))
#define ia_ceil_flt(a) ceil(a)
#define ia_div_q15_flt(a) (a) / 32768
#define ia_add_flt(a, b) ((a) + (b))
#define ia_add_double(a, b) ((a) + (b))
#define ia_mul_flt(a, b) ((a) * (b))
#define ia_mul_double_flt(a, b) ((a) * (b))
#define ia_lt_flt(a, b) ((a) < (b) ? (1) : (0))
#define ia_lteq_flt(a, b) ((a) <= (b) ? (1) : (0))
#define ia_eq_flt(a, b) ((a) == (b) ? (1) : (0))
#define ia_mac_flt(x, a, b) ((x) + (a) * (b))
#define ia_msu_flt(x, a, b) ((x) - (a) * (b))
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif
#endif /*IMPEGHD_INTRINSICS_FLT_H*/