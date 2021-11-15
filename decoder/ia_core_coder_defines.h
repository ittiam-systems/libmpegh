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

#ifndef IA_CORE_CODER_DEFINES_H
#define IA_CORE_CODER_DEFINES_H

#define MAX_WINDOWS 8
#define MAX_ORDER 31
#define MAX_ORDER_LONG 12
#define MAX_FILTERS 3

#define MAX_BINS_LONG 1024
#define MAX_BINS_SHORT 128
#define MAX_SCALE_FACTOR_BANDS_SHORT 16
#define MAX_SCALE_FACTOR_BANDS_LONG (52)

#define ZERO_HCB 0

#define NOISE_OFFSET 90

#define ESC_HCB 11
#define NOISE_HCB 13
#define INTENSITY_HCB2 14
#define INTENSITY_HCB 15

#define CHANNELS 2

#define SIZE01 (MAX_BINS_LONG / 16)
#define SIZE02 2 * SIZE01
#define SIZE03 3 * SIZE01
#define SIZE04 4 * SIZE01
#define SIZE05 5 * SIZE01
#define SIZE06 6 * SIZE01
#define SIZE07 7 * SIZE01
#define SIZE08 8 * SIZE01
#define SIZE09 9 * SIZE01
#define SIZE10 10 * SIZE01
#define SIZE11 11 * SIZE01
#define SIZE12 12 * SIZE01
#define SIZE13 13 * SIZE01
#define SIZE14 14 * SIZE01
#define SIZE15 15 * SIZE01
#define SIZE16 16 * SIZE01

typedef struct
{
  WORD32 sampling_frequency;
} ia_sampling_rate_info_struct;

#define MAX_SAMPLE_RATE (48000)

#endif
