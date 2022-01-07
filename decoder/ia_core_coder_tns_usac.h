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

#ifndef IA_CORE_CODER_TNS_USAC_H
#define IA_CORE_CODER_TNS_USAC_H

#define TNS_MAX_BANDS 49
#define TNS_MAX_ORDER 15
#define TNS_MAX_WIN 8
#define TNS_MAX_FILT 3
#define TNS_SHORT_NUM_FILT_BITS 1
#define TNS_SHORT_START_BAND_BITS 4
#define TNS_SHORT_ORDER_BITS 3
#define TNS_LONG_NUM_FILT_BITS 2
#define TNS_LONG_START_BAND_BITS 6
#define TNS_LONG_ORDER_BITS 4

typedef struct
{
  WORD32 start_band;
  WORD32 stop_band;
  WORD32 order;
  WORD32 direction;
  WORD32 coef_compress;
  WORD16 coef[TNS_MAX_ORDER];

} ia_tns_filter_struct;

typedef struct
{
  WORD32 n_filt;
  WORD32 coef_res;
  ia_tns_filter_struct str_filter[TNS_MAX_FILT];
} ia_tns_info_struct;

typedef struct
{
  WORD32 n_subblocks;
  ia_tns_info_struct str_tns_info[TNS_MAX_WIN];
} ia_tns_frame_info_struct;

#endif /* IA_CORE_CODER_TNS_USAC_H */
