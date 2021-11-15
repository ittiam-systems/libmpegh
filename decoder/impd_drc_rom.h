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

#ifndef IMPD_DRC_ROM_H
#define IMPD_DRC_ROM_H
#define MAX_NUM_QMF_BANDS 128

#define NUM_GAIN_TBL_PROF_0_1_ENTRIES 25
#define NUM_GAIN_TBL_PROF_2_ENTRIES 49
#define NUM_SLOPE_TBL_ENTRIES 15

extern const ia_drc_delta_param_table_entry_struct
    ia_drc_gain_tbls_prof_0_1[NUM_GAIN_TBL_PROF_0_1_ENTRIES];

extern const ia_drc_delta_param_table_entry_struct
    ia_drc_gain_tbls_prof_2[NUM_GAIN_TBL_PROF_2_ENTRIES];

extern const ia_drc_delta_param_table_entry_struct
    ia_drc_slope_code_tbl_entries_by_size[NUM_SLOPE_TBL_ENTRIES];

extern const WORD32 ia_drc_characteristic_order_default[][3];

extern const WORD32 ia_drc_measurement_system_def_tbl[];

extern const WORD32 ia_drc_ameasurement_system_bs1770_3_tbl[];
extern const WORD32 ia_drc_measurement_system_user_tbl[];
extern const WORD32 ia_drc_measurement_system_expert_tbl[];
extern const WORD32 ia_drc_measurement_system_rms_a_tbl[];
extern const WORD32 ia_drc_measurement_system_rms_b_tbl[];
extern const WORD32 ia_drc_measurement_system_rms_c_tbl[];
extern const WORD32 ia_drc_measurement_system_rms_d_tbl[];
extern const WORD32 ia_drc_measurement_system_rms_e_tbl[];
extern const WORD32 ia_drc_measurement_method_prog_loudness_tbl[];
extern const WORD32 ia_drc_measurement_method_peak_loudness_tbl[];

#define MAX_NUM_DOWNMIX_ID_REQUESTS_LOCAL 3

typedef struct
{
  WORD32 target_config_request_type;
  WORD32 num_downmix_id_requests;
  WORD8 requested_dwnmix_id[MAX_NUM_DOWNMIX_ID_REQUESTS_LOCAL];
  WORD32 requested_target_layout;
  WORD32 requested_target_ch_count;
} ia_drc_loc_sys_interface_struct;

extern const ia_drc_loc_sys_interface_struct ia_drc_loc_sys_interface[];

typedef struct
{
  WORD32 loudness_normalization_on;
  FLOAT32 target_loudness;
} ia_drc_loc_loudness_norm_ctrl_interface_struct;

extern const ia_drc_loc_loudness_norm_ctrl_interface_struct
    ia_drc_loc_loudness_norm_ctrl_interface[];

typedef struct
{
  WORD32 album_mode;
  WORD32 peak_limiter;
  WORD32 loudness_deviation_max;
  WORD32 loudness_measurement_method;
  WORD32 loudness_measurement_system;
  WORD32 loudness_measurement_pre_proc;
  WORD32 device_cut_off_frequency;
  FLOAT32 loudness_norm_gain_modification_db;

  FLOAT32 loudness_norm_gain_db_max;

  FLOAT32 output_peak_level_max;

} ia_drc_loc_loudness_norm_param_interface_struct;

extern const ia_drc_loc_loudness_norm_param_interface_struct
    ia_drc_loc_loudness_norm_param_interface[];

#define MAX_NUM_DRC_FEATURE_REQUESTS_LOCAL 3
typedef struct
{
  WORD8 dynamic_range_control_on;
  WORD8 num_drc_feature_requests;
  WORD8 drc_feature_req_type[MAX_NUM_DRC_FEATURE_REQUESTS_LOCAL];
  WORD8 requested_dyn_rng_measurement_type;
  WORD8 requested_dyn_range_is_single_val_flag;
  FLOAT32 requested_dyn_range_value;
  FLOAT32 requested_dyn_range_min_val;
  FLOAT32 requested_dyn_range_max_val;
  WORD8 requested_drc_characteristic;
} ia_drc_loc_drc_interface_struct;

extern const ia_drc_loc_drc_interface_struct ia_drc_loc_dyn_range_ctrl_interface[];

#define MAX_NUM_DRC_EFFECT_TYPE_REQUESTS_LOCAL 5
typedef struct
{
  WORD8 requested_num_drc_effects;
  WORD8 desired_num_drc_effects_of_requested;
  WORD8 requested_drc_effect_type[MAX_NUM_DRC_EFFECT_TYPE_REQUESTS_LOCAL];
} ia_drc_loc_requested_drc_effect_struct;

extern const ia_drc_loc_requested_drc_effect_struct ia_drc_loc_requested_drc_effect_type_str[];

typedef struct
{
  FLOAT32 compress;
  FLOAT32 boost;

  WORD8 drc_characteristic_target;
} ia_drc_loc_drc_parameter_interface_struct;

extern const ia_drc_loc_drc_parameter_interface_struct ia_drc_loc_drc_parameter_interface[];

typedef struct
{
  WORD8 num_group_ids_requested;
  WORD8 group_id_requested[3];
  WORD8 num_group_preset_ids_requested;
  WORD8 group_preset_id_requested[3];
  WORD8 num_members_group_preset_ids_requested[3];
  WORD8 group_preset_id_requested_preference;
} ia_drc_loc_mpegh_parameter_interface_struct;

extern const ia_drc_loc_mpegh_parameter_interface_struct ia_drc_loc_mpegh_parameter_interface[];

extern const ia_filter_bank_params_struct ia_drc_normal_cross_freq[FILTER_BANK_PARAMETER_COUNT];
extern const FLOAT32 ia_drc_impd_drc_table[AUDIO_CODEC_SUBBAND_COUNT_STFT256];

#endif
