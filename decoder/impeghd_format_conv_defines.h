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

#ifndef IMPEGHD_FORMAT_CONV_DEFINES_H
#define IMPEGHD_FORMAT_CONV_DEFINES_H

#define FC_EQ_MAX 2.51188636f  // pow(10,8f/20)
#define FC_EQ_MIN 0.316227764f // pow(10,-10.f/20)
#define FC_ERB_BANDS 58
#define FC_ERB_BANDS_MINUS1 57

#define FC_AES 7 // ADAPTIVE_EQ_STRENGTH

#define FC_STFT_SLOTS (4)
#define FC_DELAY (256)
#define FC_OUT_MAX_CH (24)
#define FC_MAX_CHANNELS (24)
#define FC_MAX_IN_OUT (60)
#define FC_MAXBANDS (100)
#define FC_STFT_FRAME (256)
#define FC_STFT_FRAME_1 (255)
#define FC_BANDS_1 (256)
#define FC_BANDS (257)
#define FC_STFT_FRAMEx2 (512)
#define FC_FRAME_SIZE (1024)
#define FC_ALPHA (0.0435f)
#define FC_EPSILON 1e-35f
#define FC_BETA (1 - FC_ALPHA)

#define RULE_NOPROC 0     /* no processing channel copy with gain */
#define RULE_EQ1 1        /* eq for front up-median downmix */
#define RULE_EQ2 2        /* eq for surround up-median downmix */
#define RULE_EQ3 3        /* eq for top-up downmix */
#define RULE_EQ4 4        /* eq for top-median downmix */
#define RULE_EQ5 5        /* eq for horizontal channel that is displaced to height */
#define RULE_REQ 6        /* first eq for random setups */
#define N_EQ 12           /* number of eqs */
#define RULE_PANNING 100  /* manual amplitude panning (specifying alpha0  alpha) */
#define RULE_TOP2ALLU 101 /* top channel to all upper channels */
#define RULE_TOP2ALLM 102 /* top channel to all horizontal channels */
#define RULE_AUTOPAN 103  /* automatic amplitude panning (alpha0 defined by destimation */

/* downmix rules */

#define GAIN1 80
#define GAIN2 60
#define GAIN3 85
#define GAIN4 100

#endif
