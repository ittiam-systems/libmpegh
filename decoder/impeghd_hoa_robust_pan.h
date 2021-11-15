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

#ifndef __IA_MPEGH_HOA_ROBUST_PAN_H__
#define __IA_MPEGH_HOA_ROBUST_PAN_H__

#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_space_positions.h"

typedef struct
{
  WORD32 clip_src_height;
  WORD32 is_2d;
  ia_render_hoa_space_positions_str sp_speaker_pos_cart;
  ia_render_hoa_space_positions_str sp_speaker_pos_Sp;
  ia_render_hoa_space_positions_str sp_virt_mic_xyz;
  FLOAT32 max_inc;
  FLOAT32 min_inc;
  FLOAT32 radius;
  FLOAT32 k;
  ia_render_hoa_simple_mtrx_str sm_gain_mtrx;
  ia_render_hoa_simple_mtrx_str sm_hml_r;
  ia_render_hoa_simple_mtrx_str sm_hml_i;
} ia_render_hoa_robust_pan_str;

IA_ERRORCODE ia_render_hoa_robust_pan_calculate_gains_pre(pVOID handle, pVOID spc_pos_handle,
                                                          FLOAT32 beta, WORD32 is_norm,
                                                          pVOID scratch);

IA_ERRORCODE ia_render_hoa_robust_pan_init_with_spc_positions(
    ia_render_hoa_robust_pan_str *rbs_handle, pVOID spc_pos_handle, FLOAT32 freq,
    FLOAT32 mic_radius, WORD32 clip_src_height, pVOID scratch);
#endif // __IA_MPEGH_HOA_ROBUST_PAN_H__
