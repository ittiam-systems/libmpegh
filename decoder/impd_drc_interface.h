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

#ifndef IMPD_DRC_INTERFACE_H
#define IMPD_DRC_INTERFACE_H

typedef struct
{
  WORD32 ext_size_bits;
  WORD32 ext_bit_size;
  WORD32 uni_drc_interface_ext_type;
} ia_drc_specific_interface_extension_struct;

typedef struct
{
  WORD32 interface_ext_count;
  ia_drc_specific_interface_extension_struct specific_interface_ext[EXT_COUNT_MAX];
} ia_drc_uni_interface_ext_struct;

typedef struct
{
  WORD32 change_compress;
  WORD32 change_boost;
  FLOAT32 compress;
  FLOAT32 boost;

  WORD32 change_drc_characteristic_target;
  WORD32 drc_characteristic_target;
} ia_drc_parameter_interface_struct;

typedef struct
{
  WORD32 dynamic_range_control_on;
  WORD32 num_drc_feature_requests;
  WORD8 drc_feature_req_type[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_num_drc_effects[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 desired_num_drc_effects_of_requested[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_drc_effect_type[MAX_NUM_DRC_FEATURE_REQUESTS][MAX_NUM_DRC_EFFECT_TYPE_REQUESTS];
  WORD8 requested_dyn_rng_measurement_type[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_dyn_range_is_single_val_flag[MAX_NUM_DRC_FEATURE_REQUESTS];
  FLOAT32 requested_dyn_range_value[MAX_NUM_DRC_FEATURE_REQUESTS];
  FLOAT32 requested_dyn_range_min_val[MAX_NUM_DRC_FEATURE_REQUESTS];
  FLOAT32 requested_dyn_range_max_val[MAX_NUM_DRC_FEATURE_REQUESTS];
  WORD8 requested_drc_characteristic[MAX_NUM_DRC_FEATURE_REQUESTS];
} ia_drc_dyn_rng_ctrl_interface_struct;

typedef struct
{
  WORD32 album_mode;
  WORD32 peak_limiter;
  WORD32 change_loudness_deviation_max;
  WORD32 loudness_deviation_max;
  WORD32 change_loudness_measur_method;
  WORD32 loudness_measurement_method;
  WORD32 change_loudness_measur_system;
  WORD32 loudness_measurement_system;
  WORD32 change_loudness_measur_pre_proc;
  WORD32 loudness_measurement_pre_proc;
  WORD32 change_device_cut_off_freq;
  WORD32 device_cut_off_frequency;
  WORD32 change_loudness_norm_gain_db_max;
  FLOAT32 loudness_norm_gain_modification_db;
  FLOAT32 loudness_norm_gain_db_max;
  FLOAT32 output_peak_level_max;

  WORD32 change_loudness_norm_gain_modification_db;

  WORD32 change_output_peak_level_max;

} ia_drc_loudness_norm_parameter_interface_struct;

typedef struct
{
  WORD32 loudness_normalization_on;
  FLOAT32 target_loudness;

} ia_drc_loudness_norm_ctrl_interface_struct;

typedef struct
{
  WORD32 target_config_request_type;
  WORD32 num_downmix_id_requests;
  WORD8 requested_dwnmix_id[MAX_NUM_DOWNMIX_ID_REQUESTS];
  WORD32 requested_target_layout;
  WORD32 requested_target_ch_count;
} ia_drc_system_interface_struct;

typedef struct
{
  WORD32 interface_signat_type;
  WORD32 interface_signat_data_len;
  UWORD32 interface_signat_data[MAX_SIGNATURE_DATA_LENGTH_PLUS_ONE * 8];
} ia_drc_drc_uni_interface_signat_struct;

typedef struct ia_drc_interface_struct
{
  WORD32 interface_signat_flag;
  WORD32 system_interface_flag;
  WORD32 loudness_norm_ctrl_interface_flag;
  WORD32 loudness_norm_parameter_interface_flag;
  WORD32 drc_interface_flag;
  WORD32 drc_parameter_interface_flag;
  WORD32 drc_uni_interface_ext_flag;
  ia_drc_drc_uni_interface_signat_struct drc_uni_interface_signature;
  ia_drc_system_interface_struct system_interface;
  ia_drc_loudness_norm_ctrl_interface_struct loudness_norm_ctrl_interface;
  ia_drc_loudness_norm_parameter_interface_struct loudness_norm_param_interface;
  ia_drc_dyn_rng_ctrl_interface_struct drc_ctrl_interface;
  ia_drc_parameter_interface_struct drc_parameter_interface;
  ia_drc_uni_interface_ext_struct drc_uni_interface_ext;
} ia_drc_interface_struct;

#endif
