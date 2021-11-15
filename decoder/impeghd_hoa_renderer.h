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

#ifndef __IA_MPEGH_HOA_RENDERER_H__
#define __IA_MPEGH_HOA_RENDERER_H__

#ifndef FLOAT64
#define FLOAT64 double
#endif

typedef struct
{
  FLOAT32 ldspk_dist;
  WORD32 mtrx_selected;
  WORD32 preliminary_mtrx_selected;
  WORD32 num_out_channels;
  WORD32 render_mtrx;
  WORD32 use_nfc;
  WORD32 is_scrn_rel;
  WORD32 nfc_filter_sz;
  WORD32 spk_idx;

  // WORD32 mtrx_sz_offset;
  pWORD32 scratch_idx;
  pVOID scratch;

  ia_render_hoa_render_mtrx_str v_render_mtrx[MAX_NUM_HOA_ORDERS + 1];
  ia_render_hoa_nfc_filtering_str nfc_filter[MAXIMUM_NUM_HOA_COEFF];
} ia_render_hoa_str;

WORD32 impeghd_hoa_ren_validate_signaled_rendering_matrix(pVOID handle, pVOID spk_pos,
                                                          WORD32 *ls_types, WORD32 ls_types_sz,
                                                          const WORD32 n);

WORD32 impeghd_hoa_ren_permute_signaled_rendering_matrix(pVOID handle, pVOID spk_pos,
                                                         pVOID lfe_pos, WORD32 *ls_types,
                                                         WORD32 ls_types_sz);

IA_ERRORCODE impeghd_hoa_ren_permute_lfe_channel(pVOID handle, pVOID spk_pos, WORD32 *ls_types,
                                                 WORD32 ls_types_size);

WORD32 impeghd_hoa_ren_get_loudspeaker_type(pVOID handle, WORD32 *ls_types);

IA_ERRORCODE impeghd_hoa_ren_block_process(pVOID handle, pFLOAT32 in_buf, WORD32 rows,
                                           UWORD32 cols, WORD32 is_clipping_protect_enabled,
                                           pVOID scratch, pFLOAT32 out);
IA_ERRORCODE impeghd_hoa_ren_nfc_init(pVOID handle, UWORD32 un_hoa_order, FLOAT32 f_nfc_radius,
                                      UWORD32 un_sample_rate);

IA_ERRORCODE impeghd_hoa_ren_renderer_init(ia_render_hoa_str *rh_handle, WORD32 spk_idx,
                                           ia_speaker_config_3d *ref_spk_layout,
                                           pWORD32 hoa_orders, WORD32 num_hoa_orders,
                                           WORD32 is_lfe_enabled, pVOID scratch);
IA_ERRORCODE impeghd_hoa_ren_input_init(pVOID handle, ia_speaker_config_3d *ref_spk_layout,
                                        WORD32 order, pVOID config_handle,
                                        WORD32 hoa_mtx_file_cnt, FLOAT32 nfc_radius,
                                        WORD32 sampling_rate, UWORD32 mpegh_profile_lvl);

#endif // __IA_MPEGH_HOA_RENDERER_H__
