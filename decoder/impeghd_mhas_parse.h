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

#ifndef IMPEGHD_MHAS_PARSE
#define IMPEGHD_MHAS_PARSE

#define MHAS_PAC_TYP_FILLDATA (0)
#define MHAS_PAC_TYP_MPEGH3DACFG (1)
#define MHAS_PAC_TYP_MPEGH3DAFRAME (2)
#define MHAS_PAC_TYP_AUDIOSCENEINFO (3)
#define MHAS_PAC_TYP_SYNC (6)
#define MHAS_PAC_TYP_SYNCGAP (7)
#define MHAS_PAC_TYP_MARKER (8)
#define MHAS_PAC_TYP_CRC16 (9)
#define MHAS_PAC_TYP_CRC32 (10)
#define MHAS_PAC_TYP_DESCRIPTOR (11)
#define MHAS_PAC_TYP_USERINTERACTION (12)
#define MHAS_PAC_TYP_LOUDNESS_DRC (13)
#define MHAS_PAC_TYP_BUFFERINFO (14)
#define MHAS_PAC_TYP_GLOBAL_CRC16 (15)
#define MHAS_PAC_TYP_GLOBAL_CRC32 (16)
#define MHAS_PAC_TYP_AUDIOTRUNCATION (17)
#define MHAS_PAC_TYP_GENDATA (18)
#define MHAS_PAC_TYP_EARCON (19)
#define MHAS_PAC_TYP_PCMCONFIG (20)
#define MHAS_PAC_TYP_PCMDATA (21)
#define MHAS_PAC_TYP_LOUDNESS (22)

#define DEFAULT_REF_DIST (177)

#define ID_MAE_GROUP_DESCRIPTION (0)
#define ID_MAE_SWITCHGROUP_DESCRIPTION (1)
#define ID_MAE_GROUP_CONTENT (2)
#define ID_MAE_GROUP_COMPOSITE (3)
#define ID_MAE_SCREEN_SIZE (4)
#define ID_MAE_GROUP_PRESET_DESCRIPTION (5)
#define ID_MAE_DRC_UI_INFO (6)
#define ID_MAE_SCREEN_SIZE_EXTENSION (7)
#define ID_MAE_GROUP_PRESET_EXTENSION (8)
#define ID_MAE_LOUDNESS_COMPENSATION (9)

#define MAX_MAE_NUM_DATASETS (15)
#define MAX_NUM_EARCONS (16)
#define MAX_NUM_EARCON_LANGUAGES (16)
#define MAX_NUM_EARCON_TXT_DATA_LENTGH (256)
#define MAX_NUM_PCM_SAMPLES (1024)
#define MHAS_PAC_TYP_UNDEF (518)
#define MHAS_SYNC_BYTE (0xA5)

// Values obtained from Table 11 of section 4.8.2.2 of specification - 23008-3
// Restrictions applicable for LC profile
#define MAX_GROUP_PRESET_NUM_CONDITIONS (16)
#define MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT (32)
#define MAX_NUM_GROUP_CONDITIONS (16)
#define MAX_NUM_GROUPS (16)
#define MAX_NUM_GROUPS_PRESETS (8)
#define MAX_NUM_PRESET_GROUP_EXTENSIONS (8)
#define MAX_NUM_SWITCH_GROUPS (8)

// Group Definition
#define MAX_GROUP_NUM_MEMBERS (128)
#define MAX_DESCR_LANGUAGE_DATA_LEN (16)
#define MAX_DESCRIPTON_DATA_LEN (256)
#define MAX_NUM_COMPOSITE_PAIRS (128)
#define MAX_NUM_CONTENT_DATA_BLOCKS (128)
#define MAX_NUM_DESCRIPTOIN_BLOCKS (128)
#define MAX_NUM_DESCR_LANGUAGES (4)
#define MAX_NUM_PRESET_PROD_SCREENS (31)
#define MAX_NUM_TGT_LOUDNESS_CONDITIONS (7)
#define MAX_SWITCH_GROUP_NUM_MEMBERS (32)

// structures
typedef struct
{
  WORD32 packet_lbl;
  WORD32 packet_length;
  WORD32 packet_type;
} ia_mhas_pac_info;

typedef struct
{
  WORD8 allow_gain_interact;
  WORD8 allow_pos_interact;
  WORD8 default_on_off;
  WORD8 group_id;
  WORD8 group_num_members;
  WORD8 has_conjunct_members;
  WORD8 has_content_language;
  WORD8 max_az_offset;
  WORD8 max_dist_factor;
  WORD8 max_el_offset;
  WORD8 max_gain;
  WORD8 min_az_offset;
  WORD8 min_dist_factor;
  WORD8 min_el_offset;
  WORD8 min_gain;
  WORD8 start_id;
  WORD8 allow_on_off;
  WORD8 metadata_ele_id[MAX_GROUP_NUM_MEMBERS];
} ia_mae_group_def;

typedef struct
{
  WORD8 allow_on_off;
  WORD8 default_on_off;
  WORD8 group_id;
  WORD8 group_num_members;
  WORD8 member_id[MAX_SWITCH_GROUP_NUM_MEMBERS];
  WORD32 default_group_id;
} ia_mae_switch_group_def;

typedef struct
{
  WORD8 group_id;
  WORD8 num_conditions;
  WORD8 preset_kind;
  WORD8 azimuth_offset[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 cond_on_off[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 disable_gain_interact[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 disable_pos_interact[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 dist_factor[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 elevation_offset[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 gain_flag[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 position_interact[MAX_GROUP_PRESET_NUM_CONDITIONS];
  WORD8 reference_id[MAX_GROUP_PRESET_NUM_CONDITIONS];
  UWORD8 gain[MAX_GROUP_PRESET_NUM_CONDITIONS];
} ia_mae_group_presets_def;

typedef struct
{
  WORD8 group_id;
  WORD8 preset_kind;
  WORD8 downmix_id_is_switch_group_condition[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                            [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                            [MAX_NUM_GROUP_CONDITIONS];
  WORD8 has_downmix_id_group_preset_extensions[MAX_NUM_PRESET_GROUP_EXTENSIONS];
  WORD8 has_switch_group_conditions[MAX_NUM_PRESET_GROUP_EXTENSIONS];
  WORD8 is_switch_group_condition[MAX_NUM_PRESET_GROUP_EXTENSIONS][MAX_NUM_GROUP_CONDITIONS];
  WORD8 num_downmix_id_group_preset_extensions[MAX_NUM_PRESET_GROUP_EXTENSIONS];
  WORD8 group_preset_condition_on_off[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                     [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                     [MAX_NUM_GROUP_CONDITIONS];
  WORD8 group_preset_disable_gain_interactivity[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                               [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                               [MAX_NUM_GROUP_CONDITIONS];
  WORD8 group_preset_disable_pos_interactiviy[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                             [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                             [MAX_NUM_GROUP_CONDITIONS];
  WORD8 group_preset_downmix_id[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                               [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT];
  WORD8 group_preset_gain_flag[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                              [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT][MAX_NUM_GROUP_CONDITIONS];
  WORD8 group_preset_group_id[MAX_NUM_PRESET_GROUP_EXTENSIONS][MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                             [MAX_NUM_GROUP_CONDITIONS];
  WORD8 group_preset_num_conditions[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                   [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT];
  WORD8 group_preset_position_flag[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                  [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT][MAX_NUM_GROUP_CONDITIONS];
  WORD8 group_preset_switch_group_id[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                                    [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                                    [MAX_NUM_GROUP_CONDITIONS];

  UWORD8 group_preset_az_offset[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                               [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT][MAX_NUM_GROUP_CONDITIONS];
  UWORD8 group_preset_dist_fac[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                              [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT][MAX_NUM_GROUP_CONDITIONS];
  UWORD8 group_preset_el_offset[MAX_NUM_PRESET_GROUP_EXTENSIONS]
                               [MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT][MAX_NUM_GROUP_CONDITIONS];
  UWORD8 group_preset_gain[MAX_NUM_PRESET_GROUP_EXTENSIONS][MAX_NUM_DOWNMIXID_GROUP_PRESET_EXT]
                          [MAX_NUM_GROUP_CONDITIONS];
} ia_mae_group_presets_ext_def;

typedef struct
{
  WORD8 num_data_sets;
  WORD8 data_type[MAX_MAE_NUM_DATASETS];
  WORD16 data_length[MAX_MAE_NUM_DATASETS];
} ia_mae_data;

typedef struct
{
  WORD32 num_data_blks;
  WORD32 content_kind[MAX_NUM_CONTENT_DATA_BLOCKS];
  WORD32 content_language[MAX_NUM_CONTENT_DATA_BLOCKS];
  WORD32 data_grp_id[MAX_NUM_CONTENT_DATA_BLOCKS];
  WORD32 has_content_language[MAX_NUM_CONTENT_DATA_BLOCKS];

} ia_content_data;

typedef struct
{
  UWORD8 num_descr_blks;
  WORD8 descr_data_length[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD8 descr_data[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES][MAX_DESCRIPTON_DATA_LEN];
  WORD32 descr_language[MAX_NUM_DESCRIPTOIN_BLOCKS][MAX_NUM_DESCR_LANGUAGES];
  WORD8 grp_id[MAX_NUM_DESCRIPTOIN_BLOCKS];
  WORD8 num_descr_languages[MAX_NUM_DESCRIPTOIN_BLOCKS];
} ia_description_data;

typedef struct
{
  WORD32 num_pairs;
  WORD32 ele_id[MAX_NUM_COMPOSITE_PAIRS][2];
} ia_composite_pair_data;

typedef struct
{
  WORD32 has_non_std_screen_size;
  WORD32 screen_size_az;
  WORD32 screen_size_bot_el;
  WORD32 screen_size_el;

  FLOAT32 az_left;
  FLOAT32 az_right;
  FLOAT32 el_bottom;
  FLOAT32 el_top;
  FLOAT32 offset;
} ia_production_screen_size_data;

typedef struct
{
  WORD32 default_screen_sz_left_az;
  WORD32 default_screen_sz_right_az;
  WORD32 num_preset_prod_screens;
  WORD32 overwrite_prod_screen_size_data;

  WORD32 centered_in_az[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 has_non_std_screen_sz[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_grp_preset_id[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_left_az[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_right_az[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_bot_el[MAX_NUM_PRESET_PROD_SCREENS];
  WORD32 screen_sz_top_el[MAX_NUM_PRESET_PROD_SCREENS];
} ia_production_screen_size_ext_data;

typedef struct
{
  WORD8 default_include_group[MAX_NUM_GROUPS];
  WORD8 group_loudness_present;
  WORD8 group_loudness[MAX_NUM_GROUPS];
  WORD8 preset_include_group[MAX_NUM_GROUPS_PRESETS][MAX_NUM_GROUPS];
  WORD8 preset_max_gain[MAX_NUM_GROUPS_PRESETS];
  WORD8 preset_min_gain[MAX_NUM_GROUPS_PRESETS];
  WORD8 preset_min_max_gain_present[MAX_NUM_GROUPS_PRESETS];
  WORD8 preset_params_present[MAX_NUM_GROUPS_PRESETS];
  WORD32 default_max_gain;
  WORD32 default_min_max_gain_present;
  WORD32 default_min_gain;
  WORD32 default_params_present;
} ia_loudness_compensation_data;

typedef struct
{
  WORD32 num_tgt_loudness_cnd;
  WORD32 version;
  WORD8 set_effect_available[MAX_NUM_TGT_LOUDNESS_CONDITIONS];
  WORD8 tgt_loudness_value[MAX_NUM_TGT_LOUDNESS_CONDITIONS + 1];
} ia_drc_user_interface_info;
typedef struct
{
  // PCM DATACONFIG elements
  WORD32 bs_num_pcm_signals;
  WORD32 bs_pcm_attenuation_gain;
  WORD32 bs_pcm_loudness_value;
  WORD32 pcm_align_audio_flag;
  WORD32 pcm_bits_per_sample_idx;
  WORD32 pcm_bits_per_sample;
  WORD32 pcm_config_present;
  WORD32 pcm_fix_frame_size;
  WORD32 pcm_frame_size_idx;
  WORD32 pcm_has_attenuation_gain;
  WORD32 pcm_packet_type_present;
  WORD32 pcm_sampling_rate_idx;
  WORD32 pcm_sampling_rate;
  WORD32 pcm_signal_id[128];
  // PCM DATA PAYLOAD elements
  WORD32 bsnum_pcm_signals_in_frame;
  WORD32 num_bytes_written;
  WORD32 pcm_packet_data_present;
  WORD32 pcm_var_frame_size;
  FLOAT32 pcm_sample[MAX_NUM_CHANNELS][MAX_NUM_PCM_SAMPLES];
} ia_pcm_data_config;

typedef struct
{ /***Amendment elements***/
  // EARCON elements
  WORD8 earcon_fc_init_done;
  WORD32 earcon_has_gain;
  WORD32 earcon_has_text_label;
  WORD32 earcon_present;
  WORD32 pcm_earcon_idx;
  WORD32 earcon_active[MAX_NUM_EARCONS];
  WORD32 earcon_azimuth[MAX_NUM_EARCONS];
  WORD32 earcon_cicp_spk_idx[MAX_NUM_EARCONS];
  WORD32 earcon_distance[MAX_NUM_EARCONS];
  WORD32 earcon_elevation[MAX_NUM_EARCONS];
  WORD32 earcon_gain[MAX_NUM_EARCONS];
  WORD32 earcon_id[MAX_NUM_EARCONS];
  WORD32 earcon_is_independent[MAX_NUM_EARCONS];
  WORD32 earcon_languages[MAX_NUM_EARCONS][MAX_NUM_EARCON_LANGUAGES];
  WORD32 earcon_num_languages[MAX_NUM_EARCONS];
  WORD32 earcon_pos_type[MAX_NUM_EARCONS];
  WORD32 earcon_text_data_length[MAX_NUM_EARCONS][MAX_NUM_EARCON_LANGUAGES];
  WORD32 earcon_text_data[MAX_NUM_EARCONS][MAX_NUM_EARCON_LANGUAGES]
                         [MAX_NUM_EARCON_TXT_DATA_LENTGH];
  WORD32 earcon_type[MAX_NUM_EARCONS];
} ia_earcon_info;
typedef struct
{
  // asi - audio scene info
  // mae - metadata audio element
  WORD8 num_groups;
  WORD8 num_group_presets;
  WORD32 asi_config_set;
  WORD32 asi_id_present;
  WORD32 asi_id;
  WORD32 asi_present;
  WORD32 ei_present; // element interaction data
  WORD32 mae_id_max_avail;
  WORD32 mae_id_offset;
  WORD32 main_stream_flag;
  WORD32 num_switch_groups;
  ia_composite_pair_data composite_pair_data;
  ia_content_data content_data;
  ia_description_data group_desc_data;
  ia_description_data preset_desc_data;
  ia_description_data switch_group_desc_data;
  ia_drc_user_interface_info drc_interface_info;
  ia_earcon_info earcon_info;
  ia_loudness_compensation_data loud_comp_data;
  ia_mae_data mae_data;
  ia_pcm_data_config pcm_data_config;
  ia_production_screen_size_data screen_size_data;
  ia_production_screen_size_ext_data screen_size_ext_data;
  ia_mae_group_def group_definition[MAX_NUM_GROUPS];
  ia_mae_group_presets_def group_presets_definition[MAX_NUM_GROUPS_PRESETS];
  ia_mae_group_presets_ext_def group_presets_ext_definition;
  ia_mae_switch_group_def switch_group_definition[MAX_NUM_SWITCH_GROUPS];
} ia_mae_audio_scene_info;

IA_ERRORCODE impeghd_mae_asi_parse(ia_mae_audio_scene_info *ptr_mae_asi,
                                   ia_bit_buf_struct *ptr_bit_buf);

IA_ERRORCODE impeghd_mhas_parse(ia_mhas_pac_info *ptr_pac_info,
                                ia_mae_audio_scene_info *ptr_mae_asi,
                                ia_bit_buf_struct *ptr_bit_buf);
#endif
