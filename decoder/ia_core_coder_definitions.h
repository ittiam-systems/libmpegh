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

#ifndef IA_CORE_CODER_DEFINITIONS_H
#define IA_CORE_CODER_DEFINITIONS_H

#define LIBNAME "IA_MPEG_H_3D_AUD_DEC"

#define LIB_APIVERSION_MAJOR 1
#define LIB_APIVERSION_MINOR 10

#define LIB_APIVERSION IA_MAKE_VERSION_STR(LIB_APIVERSION_MAJOR, LIB_APIVERSION_MINOR)

#define PERSIST_IDX (0)
#define SCRATCH_IDX (1)
#define INPUT_IDX (2)
#define OUTPUT_IDX (3)

#define MAX_PREROLL_FRAMES (1)
#define MAX_OUTPUT_PCM_SIZE (4)
#ifdef LC_LEVEL_4
#define MAX_USAC_CH (56)
#else
#define MAX_USAC_CH (32)
#endif
#define MAX_LOUD_SPEAKERS (24)
#define MAX_OUT_SAMPLES_PER_FRAME (1024)
#define MAX_USAC_ELEMENTS ((MAX_USAC_CH << 1) + (MAX_USAC_CH >> 1))

#define SAMPLES_PER_FRAME (1024)

#define IN_BUF_SIZE (768 * MAX_USAC_ELEMENTS + 128)
#define MAX_RESAMPLER_RATIO (2)
#define OUT_BUF_SIZE                                                                             \
  (MAX_LOUD_SPEAKERS * MAX_PREROLL_FRAMES * MAX_OUT_SAMPLES_PER_FRAME * MAX_OUTPUT_PCM_SIZE *          \
   MAX_RESAMPLER_RATIO)
#endif /* IA_CORE_CODER_DEFINITIONS_H */
