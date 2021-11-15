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

#ifndef IMPD_DRC_SEL_PROC_DRC_SET_SEL_H
#define IMPD_DRC_SEL_PROC_DRC_SET_SEL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH (1 << 0)
#define IA_SELECTION_FLAG_EXPLICIT_PEAK_INFO_PRESENT (1 << 1)

typedef struct
{
  WORD32 drc_instrns_idx;
  WORD32 downmix_id_request_index;
  WORD32 eq_set_id;

  FLOAT32 output_peak_level;
  FLOAT32 loud_norm_db_gain_adjust;
  FLOAT32 output_loudness;
  FLOAT32 mixing_level;

  WORD32 selection_flags;
} ia_drc_selection_candidate_info_struct;

IA_ERRORCODE
impd_drc_validate_requested_drc_feature(
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct);

IA_ERRORCODE
impd_drc_select_drcs_without_compr_effects(
    ia_drc_config *pstr_drc_config, WORD32 *match_found_flag, WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE
impd_drc_match_effect_types(ia_drc_config *pstr_drc_config,
                            WORD32 effect_type_requested_total_count,
                            WORD32 effect_type_requested_desired_count,
                            WORD8 *requested_effect_type, WORD32 *selection_candidate_count,
                            ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE
impd_drc_match_dynamic_range(ia_drc_config *pstr_drc_config,
                             ia_drc_loudness_info_set_struct *pstr_loudness_info,
                             ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
                             WORD32 num_drc_requests, WORD32 *selection_candidate_count,
                             ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE
impd_match_drc_characteristic_attempt(
    ia_drc_config *pstr_drc_config, WORD32 requested_drc_characteristic, WORD32 *match_found_flag,
    WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE
impd_drc_match_drc_characteristic(
    ia_drc_config *pstr_drc_config, WORD32 requested_drc_characteristic,
    WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE
impd_drc_set_final_selection(ia_drc_config *pstr_drc_config,
                             ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
                             WORD32 *selection_candidate_count,
                             ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE impd_drc_set_pre_selection(
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
    ia_drc_config *pstr_drc_config, ia_drc_loudness_info_set_struct *pstr_loudness_info,
    WORD32 restrict_to_drc_with_album_loudness, WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info);

IA_ERRORCODE
impd_drc_sel_proc_init_dflt(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc);

IA_ERRORCODE
impd_drc_sel_proc_init_sel_proc_params(
    ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct);

VOID impd_drc_sel_proc_init_interface_params(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                             ia_drc_interface_struct *pstr_drc_interface);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
