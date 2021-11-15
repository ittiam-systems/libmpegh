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

#ifndef __IA_MPEGH_HOA_SPACE_POSITIONS_H__
#define __IA_MPEGH_HOA_SPACE_POSITIONS_H__

typedef struct
{
  WORD32 type;
  WORD32 cols;
  WORD32 num_pos;
  FLOAT32 arr[HOA_MAX_ARR_SIZE];
} ia_render_hoa_space_positions_str;

IA_ERRORCODE
impeghd_hoa_ren_space_positions_convert_copy_to_spherical(pVOID handle,
                                                          ia_render_hoa_space_positions_str *tmp);

VOID impeghd_hoa_ren_space_positions_get_max_min_inclination(pVOID handle, pFLOAT32 max_inc,
                                                             pFLOAT32 min_inc, pVOID scratch);

FLOAT32 impeghd_hoa_ren_space_positions_get_max_distance(pVOID handle);

VOID impeghd_hoa_ren_space_positions_add_poles(pVOID handle, FLOAT32 r, pVOID scratch);

WORD32 impeghd_hoa_ren_space_positions_is_2d(pVOID handle, pVOID scratch);

IA_ERRORCODE impeghd_hoa_ren_space_positions_convert_copy_to_cartesian(
    pVOID handle, ia_render_hoa_space_positions_str *tmp, WORD32 force_r1);

IA_ERRORCODE
impeghd_hoa_ren_space_positions_init_with_param(ia_render_hoa_space_positions_str *spc_handle,
                                                const WORD32 spk_idx,
                                                ia_speaker_config_3d *ref_spk_layout,
                                                WORD32 force_lfe);
VOID impeghd_hoa_ren_space_positions_init_pos(ia_render_hoa_space_positions_str *spc_handle,
                                              ia_render_hoa_space_positions_str *handle_original);

IA_ERRORCODE
impeghd_hoa_ren_space_positions_init_with_surround(ia_render_hoa_space_positions_str *spc_handle,
                                                   WORD32 num_pos, pFLOAT32 radius,
                                                   pFLOAT32 spk_inc, pFLOAT32 spk_azimuth);

#endif // __IA_MPEGH_HOA_SPACE_POSITIONS_H__
