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

#ifndef IMPD_DRC_SECLECTION_PROCESS_H
#define IMPD_DRC_SECLECTION_PROCESS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
#include "impeghd_error_codes.h"
#define EFFECT_TYPE_REQUESTED_NONE 0
#define EFFECT_TYPE_REQUESTED_NIGHT 1
#define EFFECT_TYPE_REQUESTED_NOISY 2
#define EFFECT_TYPE_REQUESTED_LIMITED 3
#define EFFECT_TYPE_REQUESTED_LOWLEVEL 4
#define EFFECT_TYPE_REQUESTED_DIALOG 5
#define EFFECT_TYPE_REQUESTED_GENERAL_COMPR 6
#define EFFECT_TYPE_REQUESTED_EXPAND 7
#define EFFECT_TYPE_REQUESTED_ARTISTIC 8
#define EFFECT_TYPE_REQUESTED_COUNT 9

#define MATCH_EFFECT_TYPE 0
#define MATCH_DYNAMIC_RANGE 1
#define MATCH_DRC_CHARACTERISTIC 2

typedef struct ia_drc_sel_proc_params_struct
{
  WORD32 base_channel_count;
  WORD32 base_layout;
  WORD32 target_config_request_type;
  WORD32 num_downmix_id_requests;
  WORD8 requested_dwnmix_id[MAX_NUM_DOWNMIX_ID_REQUESTS];
  WORD32 requested_target_layout;
  WORD32 requested_target_ch_count;
  WORD32 target_ch_count_prelim;

  WORD32 loudness_normalization_on;
  FLOAT32 target_loudness;

  WORD32 album_mode;
  WORD32 peak_limiter;
  WORD32 loudness_deviation_max;
  WORD32 loudness_measurement_method;
  WORD32 loudness_measurement_system;
  WORD32 loudness_measurement_pre_proc;
  WORD32 device_cut_off_frequency;

  WORD32 num_bands_supported;
  WORD32 dynamic_range_control_on;
  WORD32 num_drc_feature_requests;
  WORD8 drc_feature_req_type[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_num_drc_effects[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 desired_num_drc_effects_of_requested[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_drc_effect_type[MAX_NUM_DRC_FEATURE_REQUESTS][MAX_NUM_DRC_EFFECT_TYPE_REQUESTS];
  WORD8 requested_dyn_range_measur_type[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_dyn_range_range_flag[MAX_NUM_DRC_FEATURE_REQUESTS];
  FLOAT32 requested_dyn_range_value[MAX_NUM_DRC_FEATURE_REQUESTS];
  FLOAT32 requested_dyn_range_min_val[MAX_NUM_DRC_FEATURE_REQUESTS];
  FLOAT32 requested_dyn_range_max_val[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_drc_characteristic[MAX_NUM_DRC_FEATURE_REQUESTS];

  WORD8 num_group_ids_requested;
  WORD8 group_id_requested[MAX_NUM_GROUP_ID_REQUESTS];
  WORD8 num_group_preset_ids_requested;
  WORD8 group_preset_id_requested[MAX_NUM_GROUP_PRESET_ID_REQUESTS];
  WORD8 num_members_group_preset_ids_requested[MAX_NUM_MEMBERS_GROUP_PRESET];
  WORD8 group_preset_id_requested_preference;

  FLOAT32 boost;
  FLOAT32 compress;
  FLOAT32 loudness_norm_gain_db_max;
  FLOAT32 loudness_norm_gain_modification_db;
  FLOAT32 output_peak_level_max;

  WORD32 drc_characteristic_target;
} ia_drc_sel_proc_params_struct;

typedef struct ia_drc_sel_pro_struct
{
  WORD32 first_frame;
  WORD32 drc_config_flag;
  WORD32 loudness_info_set_flag;
  WORD32 sel_proc_request_flag;
  WORD32 subband_domain_mode;

  WORD8 drc_instrns_idx[SUB_DRC_COUNT];

  ia_drc_sel_proc_params_struct str_uni_drc_sel_proc_params;

  ia_drc_config *drc_config;

  ia_drc_loudness_info_set_struct loudness_info_set;

  WORD8 drc_inst_index_sel;
  WORD8 drc_coef_index_sel;
  WORD8 downmix_inst_index_sel;

  ia_drc_sel_proc_output_struct uni_drc_sel_proc_output;

} ia_drc_sel_pro_struct;

IA_ERRORCODE
impd_drc_uni_selction_proc_init(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
                                ia_drc_interface_struct *pstr_drc_interface,
                                WORD32 sub_band_domain_mode);

IA_ERRORCODE
impd_drc_mpegh_params_set_sel_process(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                      WORD8 num_group_ids_requested, WORD8 *group_id_requested,
                                      WORD8 num_group_preset_ids_requested,
                                      WORD8 *group_preset_id_requested,
                                      WORD8 *num_members_group_preset_ids_requested,
                                      WORD8 group_preset_id_requested_preference);

IA_ERRORCODE
impd_drc_uni_sel_proc_process(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                              ia_drc_config *pstr_drc_config,
                              ia_drc_loudness_info_set_struct *pstr_loudness_info,
                              ia_drc_sel_proc_output_struct *hia_drc_sel_proc_output_struct);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
