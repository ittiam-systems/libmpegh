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

#ifndef __IA_MPEGH_HOA_RENDER_MTRX_H__
#define __IA_MPEGH_HOA_RENDER_MTRX_H__

typedef struct
{
  FLOAT32 signaled_speaker_pos_rad[HOA_MAX_VECTOR_SIZE];
  UWORD32 spk_idx[HOA_MAX_VECTOR_SIZE];
  WORD32 num_subwoofer;
  WORD32 num_satellites;
  WORD32 order;
  WORD32 l_speakers;
  WORD32 cols;
  ia_render_hoa_simple_mtrx_str sm_mtrx;
} ia_render_hoa_render_mtrx_str;

VOID impeghd_hoa_ren_mtrx_resort_spk_pos(pVOID handle, const pUWORD32 order, pVOID scratch);

IA_ERRORCODE impeghd_hoa_ren_mtrx_init_with_mtrx_param(ia_render_hoa_render_mtrx_str *ren_handle,
                                                       pVOID config_handle);

IA_ERRORCODE
impeghd_hoa_ren_mtrx_init_with_spc_pos_param(ia_render_hoa_render_mtrx_str *ren_handle,
                                             WORD32 spk_id, ia_speaker_config_3d *ref_spk_layout,
                                             WORD32 order, WORD32 add_lfe_ch, pVOID scratch);

#endif // __IA_MPEGH_HOA_RENDER_MTRX_H__
