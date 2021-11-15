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

#ifndef __IA_MPEGH_HOA_NFC_FILTERING_H__
#define __IA_MPEGH_HOA_NFC_FILTERING_H__

#ifndef FLOAT64
#define FLOAT64 double
#endif

typedef struct
{
  FLOAT32 a0;
  FLOAT32 a1;
  FLOAT32 a2;
  FLOAT32 b0;
  FLOAT32 b1;
  FLOAT32 b2;
  FLOAT32 x1;
  FLOAT32 x2;
  FLOAT32 y1;
  FLOAT32 y2;
} ia_render_hoa_iir_2_str;

typedef struct
{
  FLOAT32 a0;
  FLOAT32 a1;
  FLOAT32 b0;
  FLOAT32 b1;
  FLOAT32 x1;
  FLOAT32 y1;
} ia_render_hoa_iir_1_str;

typedef struct
{
  WORD32 order;
  WORD32 num_iir_2_filts;
  WORD32 num_iir_1_filts;
  FLOAT32 global_gain;
  FLOAT32 nfc_time;
  FLOAT32 nfs_time;
  ia_render_hoa_iir_2_str str_iir_2[HOA_MAXIMUM_IIR_FILTER];
  ia_render_hoa_iir_1_str str_iir_1;
} ia_render_hoa_nfc_filtering_str;

VOID impeghd_hoa_ren_nfc_set_filter_params(pVOID handle, FLOAT32 tnfc, FLOAT32 tnfs);
VOID impeghd_hoa_ren_nfc_filter_apply(pVOID handle, pFLOAT32 x, pFLOAT32 y, UWORD32 l);
#endif //  __IA_MPEGH_HOA_NFC_FILTERING_H__
