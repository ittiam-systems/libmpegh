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

#ifndef IMPD_DRC_STURCT_H
#define IMPD_DRC_STURCT_H

#ifdef __cplusplus
extern "C" {
#endif

#define IA_DRC_METHOD_DEFINITION_UNKNOWN_OTHER 0
#define IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS 1
#define IA_DRC_METHOD_DEFINITION_ANCHOR_LOUDNESS 2
#define IA_DRC_METHOD_DEFINITION_MAX_OF_LOUDNESS_RANGE 3
#define IA_DRC_METHOD_DEFINITION_MOMENTARY_LOUDNESS_MAX 4
#define IA_DRC_METHOD_DEFINITION_SHORT_TERM_LOUDNESS_MAX 5
#define IA_DRC_METHOD_DEFINITION_LOUDNESS_RANGE 6
#define IA_DRC_METHOD_DEFINITION_MIXING_LEVEL 7
#define IA_DRC_METHOD_DEFINITION_ROOM_TYPE 8
#define IA_DRC_METHOD_DEFINITION_SHORT_TERM_LOUDNESS 9

#define IA_DRC_MEASUREMENT_SYSTEM_UNKNOWN_OTHER 0
#define IA_DRC_MEASUREMENT_SYSTEM_EBU_R_128 1
#define IA_DRC_MEASUREMENT_SYSTEM_BS_1770_4 2
#define IA_DRC_MEASUREMENT_SYSTEM_BS_1770_3 IA_DRC_MEASUREMENT_SYSTEM_BS_1770_4
#define IA_DRC_MEASUREMENT_SYSTEM_BS_1770_4_PRE_PROCESSING 3
#define IA_DRC_MEASUREMENT_SYSTEM_BS_1770_3_PRE_PROCESSING                                       \
  IA_DRC_MEASUREMENT_SYSTEM_BS_1770_4_PRE_PROCESSING
#define IA_DRC_MEASUREMENT_SYSTEM_USER 4
#define IA_DRC_MEASUREMENT_SYSTEM_EXPERT_PANEL 5
#define IA_DRC_MEASUREMENT_SYSTEM_BS_1771_1 6
#define IA_DRC_MEASUREMENT_SYSTEM_RESERVED_A 7
#define IA_DRC_MEASUREMENT_SYSTEM_RESERVED_B 8
#define IA_DRC_MEASUREMENT_SYSTEM_RESERVED_C 9
#define IA_DRC_MEASUREMENT_SYSTEM_RESERVED_D 10
#define IA_DRC_MEASUREMENT_SYSTEM_RESERVED_E 11

#define IA_DRC_RELIABILITY_UKNOWN 0
#define IA_DRC_RELIABILITY_UNVERIFIED 1
#define IA_DRC_RELIABILITY_CEILING 2
#define IA_DRC_RELIABILITY_ACCURATE 3

#define IA_DRC_EFFECT_BIT_COUNT 12

#define IA_DRC_EFFECT_BIT_NONE (-1)
#define IA_DRC_EFFECT_BIT_NIGHT 0x0001
#define IA_DRC_EFFECT_BIT_NOISY 0x0002
#define IA_DRC_EFFECT_BIT_LIMITED 0x0004
#define IA_DRC_EFFECT_BIT_LOWLEVEL 0x0008
#define IA_DRC_EFFECT_BIT_DIALOG 0x0010
#define IA_DRC_EFFECT_BIT_GENERAL_COMPR 0x0020
#define IA_DRC_EFFECT_BIT_EXPAND 0x0040
#define IA_DRC_EFFECT_BIT_ARTISTIC 0x0080
#define IA_DRC_EFFECT_BIT_CLIPPING 0x0100
#define IA_DRC_EFFECT_BIT_FADE 0x0200
#define IA_DRC_EFFECT_BIT_DUCK_OTHER 0x0400
#define IA_DRC_EFFECT_BIT_DUCK_SELF 0x0800

#define IA_DRC_GAIN_CODING_PROFILE_REGULAR 0
#define IA_DRC_GAIN_CODING_PROFILE_FADING 1
#define IA_DRC_GAIN_CODING_PROFILE_CLIPPING 2
#define IA_DRC_GAIN_CODING_PROFILE_CONSTANT 3
#define IA_DRC_GAIN_CODING_PROFILE_DUCKING IA_DRC_GAIN_CODING_PROFILE_CLIPPING

#define IA_DRC_GAIN_INTERPOLATION_TYPE_SPLINE 0
#define IA_DRC_GAIN_INTERPOLATION_TYPE_LINEAR 1

#define IA_DRC_USER_METHOD_DEFINITION_DEFAULT 0
#define IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS 1
#define IA_DRC_USER_METHOD_DEFINITION_ANCHOR_LOUDNESS 2

#define IA_DRC_USER_MEASUREMENT_SYSTEM_DEFAULT 0
#define IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_4 1
#define IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_3 IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_4
#define IA_DRC_USER_MEASUREMENT_SYSTEM_USER 2
#define IA_DRC_USER_MEASUREMENT_SYSTEM_EXPERT_PANEL 3
#define IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_A 4
#define IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_B 5
#define IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_C 6
#define IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_D 7
#define IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_E 8

#define IA_DRC_USER_LOUDNESS_PREPROCESSING_DEFAULT 0
#define IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF 1
#define IA_DRC_USER_LOUDNESS_PREPROCESSING_HIGHPASS 2

#define IA_DRC_LOUDNESS_DEVIATION_MAX_DEFAULT 63
#define IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT 1000

#define IA_DRC_SHORT_TERM_LOUDNESS_TO_AVG 0
#define IA_DRC_MOMENTARY_LOUDNESS_TO_AVG 1
#define IA_DRC_TOP_OF_LOUDNESS_RANGE_TO_AVG 2

typedef struct
{
  WORD32 base_channel_count;
  WORD32 layout_signaling_present;
  WORD32 defined_layout;
  WORD32 speaker_position[SPEAKER_POS_COUNT_MAX];
} ia_drc_channel_layout_struct;

typedef struct
{
  WORD8 downmix_id;
  WORD32 target_channel_count;
  WORD32 target_layout;
  WORD32 downmix_coefficients_present;
  FLOAT32 downmix_coefficient[DOWNMIX_COEFF_COUNT_MAX];

} ia_drc_downmix_instructions_struct;

typedef struct
{
  WORD8 gain_seq_idx;
  WORD8 drc_characteristic;
  WORD8 crossover_freq_idx;
  WORD8 start_subband_index;
} ia_drc_gain_params_struct;

typedef struct
{
  WORD32 ducking_scaling_flag;

  FLOAT32 ducking_scaling;
  FLOAT32 ducking_scaling_quantized;

} ia_drc_ducking_modifiers_struct;

typedef struct
{
  WORD32 gain_scaling_flag[BAND_COUNT_MAX];
  FLOAT32 attn_scale_factor[BAND_COUNT_MAX];
  FLOAT32 amplfn_scale_factor[BAND_COUNT_MAX];
  FLOAT32 gain_offset[BAND_COUNT_MAX];
  WORD32 gain_offset_flag[BAND_COUNT_MAX];

} ia_drc_gain_modifiers_struct;

typedef struct
{
  WORD8 gain_coding_profile;
  WORD8 gain_interp_type;
  WORD8 full_frame;
  WORD8 time_alignment;
  WORD8 time_delt_min_flag;
  WORD16 time_delt_min_val;
  WORD8 band_count;
  WORD8 drc_band_type;
  ia_drc_gain_params_struct gain_params[BAND_COUNT_MAX];

  WORD8 num_gain_max_values;
  ia_drc_tables_struct str_tables;
} ia_drc_gain_set_params_struct;

typedef struct
{
  WORD8 drc_location;
  WORD8 drc_characteristic;
} ia_drc_coefficients_basic_struct;

typedef struct
{
  WORD8 version;
  WORD8 drc_location;
  WORD8 drc_frame_size_present;
  WORD8 drc_frame_size;
  WORD8 gain_set_count;
  ia_drc_gain_set_params_struct gain_set_params[GAIN_SET_COUNT_MAX];
} ia_drc_uni_drc_coeffs_struct;

typedef struct
{
  WORD8 drc_set_id;
  WORD8 drc_location;
  WORD8 dwnmix_id_count;
  WORD8 downmix_id[DOWNMIX_ID_COUNT_MAX];
  WORD8 drc_set_effect;
  WORD8 limiter_peak_target_present;
  WORD8 limiter_peak_target;
  WORD8 drc_set_target_loudness_present;
  WORD8 drc_set_target_loudness_value_upper;
  WORD8 drc_set_target_loudness_value_lower_present;
  WORD8 drc_set_target_loudness_value_lower;
} ia_drc_instructions_basic_struct;

typedef struct
{
  WORD32 drc_set_id;
  WORD32 drc_apply_to_dwnmix;
  WORD32 drc_location;
  WORD32 dwnmix_id_count;
  WORD32 downmix_id[DOWNMIX_ID_COUNT_MAX];
  WORD32 depends_on_drc_set_present;
  WORD32 depends_on_drc_set;
  WORD32 no_independent_use;
  WORD32 drc_set_effect;
  WORD32 gain_set_index[MAX_CHANNEL_COUNT];
  ia_drc_gain_modifiers_struct str_gain_modifiers_of_ch_group[CHANNEL_GROUP_COUNT_MAX];
  ia_drc_ducking_modifiers_struct str_ducking_modifiers_for_channel[MAX_CHANNEL_COUNT];
  WORD32 limiter_peak_target_present;

  FLOAT32 limiter_peak_target;

  WORD32 drc_set_target_loudness_present;
  WORD32 drc_set_target_loudness_value_upper;
  WORD32 drc_set_target_loudness_value_lower_present;
  WORD32 drc_set_target_loudness_value_lower;

  WORD32 drc_instructions_type;
  WORD32 mae_group_id;
  WORD32 mae_group_preset_id;

  WORD32 audio_num_chan;
  WORD32 num_drc_ch_groups;
  WORD32 gain_set_index_for_channel_group[CHANNEL_GROUP_COUNT_MAX];
  WORD32 band_count_of_ch_group[CHANNEL_GROUP_COUNT_MAX];
  WORD32 gain_interpolation_type_for_channel_group[CHANNEL_GROUP_COUNT_MAX];
  WORD32 time_delta_min_for_channel_group[CHANNEL_GROUP_COUNT_MAX];
  WORD32 time_alignment_for_channel_group[CHANNEL_GROUP_COUNT_MAX];
  ia_drc_ducking_modifiers_struct
      str_ducking_modifiers_for_channel_group[CHANNEL_GROUP_COUNT_MAX];
  WORD32 channel_group_of_ch[MAX_CHANNEL_COUNT];
  WORD32 num_chan_per_ch_group[CHANNEL_GROUP_COUNT_MAX];
  WORD32 gain_element_count;
  WORD32 multiband_audio_sig_count;
} ia_drc_instructions_struct;

typedef struct
{
  WORD32 method_def;
  FLOAT32 method_value;
  WORD32 measurement_system;
  WORD32 reliability;
} ia_drc_loudness_measure_struct;

typedef struct
{
  WORD32 drc_set_id;
  WORD32 downmix_id;
  WORD32 sample_peak_level_present;
  FLOAT32 sample_peak_level;
  FLOAT32 true_peak_level;

  WORD32 true_peak_level_present;

  WORD32 true_peak_level_measurement_system; /* Parsed but unused */
  WORD32 true_peak_level_reliability;        /* Parsed but unused */
  WORD32 measurement_count;
  ia_drc_loudness_measure_struct loudness_measure[MEASUREMENT_COUNT_MAX];
  int loudness_info_type;
  int mae_group_id;
  int mae_group_preset_id;
} ia_drc_loudness_info_struct;

typedef struct
{
  WORD32 drc_config_ext_type[EXT_COUNT_MAX];
  WORD32 ext_bit_size[EXT_COUNT_MAX - 1];

} ia_drc_config_ext;

typedef struct ia_drc_config
{
  FLOAT32 loudness_gain_db;
  WORD32 multi_band_present;
  WORD32 sampling_rate;
  WORD32 drc_dwnmix_instructions_count;
  WORD32 drc_coefficients_drc_count;
  WORD32 drc_instructions_uni_drc_count;
  WORD32 drc_instructions_count_plus;
  WORD32 drc_description_basic_present;
  WORD32 drc_coefficients_basic_count;
  WORD32 drc_instructions_basic_count;
  WORD32 drc_config_ext_present;
  WORD32 apply_drc;
  ia_drc_config_ext str_drc_config_ext;
  ia_drc_coefficients_basic_struct str_drc_coefficients_basic[DRC_COEFF_COUNT_MAX];
  ia_drc_instructions_basic_struct str_drc_instructions_basic[DRC_INSTRUCTIONS_COUNT_MAX];
  ia_drc_uni_drc_coeffs_struct str_p_loc_drc_coefficients_uni_drc[DRC_COEFF_COUNT_MAX];
  ia_drc_instructions_struct str_drc_instruction_str[DRC_INSTRUCTIONS_COUNT_MAX];
  ia_drc_downmix_instructions_struct dwnmix_instructions[DOWNMIX_INSTRUCTION_COUNT_MAX];
  ia_drc_channel_layout_struct channel_layout;

  WORD32 ia_mpegh3da_dwnmix_instructions_count;
  ia_drc_downmix_instructions_struct mpegh3da_dwnmix_instructions[DOWNMIX_INSTRUCTION_COUNT_MAX];
  WORD32 loudness_infoset_present;

} ia_drc_config;

typedef struct
{
  WORD32 loudness_info_set_ext_type[EXT_COUNT_MAX];
  WORD32 ext_bit_size[EXT_COUNT_MAX - 1];
} ia_loudness_info_set_ext_struct;

typedef struct ia_drc_loudness_info_set_struct
{
  WORD32 loudness_info_album_count;
  WORD32 loudness_info_count;
  WORD32 loudness_info_set_ext_present;
  ia_drc_loudness_info_struct str_loudness_info_album[LOUDNESS_INFO_COUNT_MAX];
  ia_drc_loudness_info_struct loudness_info[LOUDNESS_INFO_COUNT_MAX];
  ia_loudness_info_set_ext_struct str_loudness_info_set_ext;
  int loudness_info_album_present;
} ia_drc_loudness_info_set_struct;

typedef struct
{
  FLOAT32 loc_gain_db;
  FLOAT32 slope;
  WORD32 time;
} ia_drc_node_struct;

typedef struct
{
  WORD32 drc_gain_coding_mode;
  WORD32 no_of_nodes;
  ia_drc_node_struct str_node[NODE_COUNT_MAX];
} ia_drc_spline_nodes_struct;

typedef struct
{
  WORD32 band_count;
  ia_drc_spline_nodes_struct str_spline_nodes[BAND_COUNT_MAX];
} ia_drc_gain_sequence_struct;

typedef struct
{
  WORD32 uni_drc_gain_ext_type[EXT_COUNT_MAX];
  WORD32 ext_bit_size[EXT_COUNT_MAX - 1];
} ia_drc_uni_drc_gain_ext_struct;

typedef struct ia_drc_gain_struct
{
  WORD32 num_drc_gain_sequences;
  ia_drc_gain_sequence_struct drc_gain_sequence[SEQUENCE_COUNT_MAX];
  WORD32 uni_drc_gain_ext_flag;
  ia_drc_uni_drc_gain_ext_struct uni_drc_gain_ext;
} ia_drc_gain_struct;

typedef struct
{
  WORD32 delta_tmin_def;
  WORD32 drc_frame_size;
  WORD32 num_gain_values_max_default;
  WORD32 delay_mode;
  WORD32 lfe_channel_map_count;
  WORD32 lfe_channel_map[MAX_CHANNEL_COUNT];
} ia_drc_params_bs_dec_struct;

typedef struct ia_drc_bits_dec_struct
{
  ia_drc_tables_struct tables_default;
  ia_drc_params_bs_dec_struct ia_drc_params_struct;
} ia_drc_bits_dec_struct;

#ifdef __cplusplus
}
#endif
#endif
