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

#ifndef IA_CORE_CODER_CONFIG_H
#define IA_CORE_CODER_CONFIG_H

#include <impeghd_type_def.h>

 // MAX_NUM_CHANNELS * 1.5
#define MAX_EXT_ELEMENTS (84)
#ifdef LC_LEVEL_4
// The bit stream extracted value can be 32.
// Currently this is limited to 3 considering:
// Audio Channels, Objects and HOA Transport channels.
#define MAX_NUM_SIGNALGROUPS (32)
/* The multi‐channel coding tool (MCT) shall not employ
 * more stereo boxes than 28 for LC profile lvl 4*/
#define MAX_NUM_MC_BOXES (28)
#else
// The bit stream extracted value can be 32.
// Currently this is limited to 3 considering:
// Audio Channels, Objects and HOA Transport channels.
#define MAX_NUM_SIGNALGROUPS (16)
/*The	multi‐channel	coding	tool	(MCT)	shall	not	employ
 * more	stereo	boxes	than 16 for LC profile lvl 3*/
#define MAX_NUM_MC_BOXES (16)
#endif
#define MAX_NUM_MC_BANDS (64)
#define MAX_TIME_CHANNELS (MAX_NUM_CHANNELS)
#define DEFAULT_BETA (48) /*equals 45 degrees */
#define DEFAULT_ALPHA (0)
#define MAX_NUM_OUT_CHANNELS (MAX_NUM_CHANNELS)
#define MAX_AUDIO_GROUPS (32)
#define MAX_AUDIO_CHANNEL_LAYOUTS (32)
#define MAX_NUM_SPEAKERS (24)
#define MAX_CICP_INDEX (20)
#define MAX_CICP_SPK_INDEX (42)
#define MAX_CONFIG_EXTENSIONS (5)
#define SIG_GROUP_TYPE_CHN (0)
#define SIG_GROUP_TYPE_OBJ (1)
#define SIG_GROUP_TYPE_HOA (3)

#define ID_USAC_SCE 0
#define ID_USAC_CPE 1
#define ID_USAC_LFE 2
#define ID_USAC_EXT 3
#define ID_USAC_INVALID 0xFF

#define SBR_RATIO_NO_SBR 0
#define SBR_RATIO_INDEX_2_1 1
#define SBR_RATIO_INDEX_8_3 2
#define SBR_RATIO_INDEX_4_1 3

#define OUT_FRAMELENGTH_768 768
#define OUT_FRAMELENGTH_1024 1024
#define OUT_FRAMELENGTH_2048 2048
#define OUT_FRAMELENGTH_4096 4096

#define MAX_CORE_SBR_FRAME_LEN_IDX (4)

#define ID_EXT_ELE_FILL 0
#define ID_EXT_ELE_MPEGS 1
#define ID_EXT_ELE_SAOC 2
#define ID_MPEGH_EXT_ELE_OAM 5
#define ID_MPEGH_EXT_ELE_MCT 9
#define ID_MPEGH_EXT_ELE_FMT_CONV 8
#define ID_MPEGH_EXT_ELE_HOA 7
#define ID_MPEGH_EXT_ELE_ENHANCED_OBJ_METADATA 13
#define ID_EXT_ELE_PROD_METADATA 14
#define DIFFAULT_PROD_METADATA_DISTANCE (57)
#define ID_EXT_GOA_PROD_METADATA (0)

#define ID_EXT_ELE_AUDIOPREROLL 3
#define ID_EXT_ELE_UNI_DRC 4

#define ID_CONFIG_EXT_FILL 0
#define ID_CONFIG_EXT_LOUDNESS_INFO (2)
#define ID_CONFIG_EXT_DOWNMIX (1)
#define ID_CONFIG_EXT_AUDIOSCENE_INFO (3)
#define ID_CONFIG_EXT_HOA_MATRIX (4)
#define ID_CONFIG_EXT_SIG_GROUP_INFO (6)
#define ID_CONFIG_EXT_COMPATIBLE_PROFILELVL_SET (7)
#define CONFIG_EXT_FILL_BYTE (0xa5)

#define MPEGH_PROFILE_LC_LVL_1 (0x0B)
#define MPEGH_PROFILE_LC_LVL_2 (0x0C)
#define MPEGH_PROFILE_LC_LVL_3 (0x0D)
#define MPEGH_PROFILE_LC_LVL_4 (0x0E)
#define MPEGH_PROFILE_LC_LVL_5 (0x0F)
#define MPEGH_PROFILE_BP_LVL_1 (0x10)
#define MPEGH_PROFILE_BP_LVL_2 (0x11)
#define MPEGH_PROFILE_BP_LVL_3 (0x12)
#define MPEGH_PROFILE_BP_LVL_4 (0x13)
#define MPEGH_PROFILE_BP_LVL_5 (0x14)

#define MINIMUM_SUPPORTED_LC_PROFILE (MPEGH_PROFILE_LC_LVL_1)
#ifdef LC_LEVEL_4
#define MAXIMUM_SUPPORTED_LC_PROFILE (MPEGH_PROFILE_LC_LVL_4)
#else
#define MAXIMUM_SUPPORTED_LC_PROFILE (MPEGH_PROFILE_LC_LVL_3)
#endif
#define MINIMUM_SUPPORTED_BP_PROFILE (MPEGH_PROFILE_BP_LVL_1)
#ifdef LC_LEVLE_4
#define MAXIMUM_SUPPORTED_BP_PROFILE (MPEGH_PROFILE_BP_LVL_4)
#else
#define MAXIMUM_SUPPORTED_BP_PROFILE (MPEGH_PROFILE_BP_LVL_3)
#endif

typedef UWORD8 UINT8;
typedef UWORD32 UINT32;

typedef struct
{
  WORD8 igf_use_enf;
  WORD8 igf_use_whitening;
  WORD8 igf_after_tns_synth;
  WORD8 igf_independent_tilling;
  WORD8 igf_start_index;
  WORD8 igf_stop_index;
  WORD8 igf_use_high_resolution;
} ia_usac_igf_init_config_struct;

typedef struct
{
  UWORD32 num_transport_ch;
  WORD32 core_coder_frame_length;
  WORD32 order;
  UWORD32 num_coeffs;
  WORD32 is_screen_relative;
  WORD32 uses_nfc;
  FLOAT32 nfc_ref_distance;
  WORD32 min_amb_order;
  WORD32 min_coeffs_for_amb;
  UWORD32 num_addnl_coders;
  WORD32 num_layers;
  WORD32 is_single_layer;
  WORD32 coded_spat_interpolation_time;
  WORD32 spat_interpolation_method;
  WORD32 coded_v_vec_length;
  WORD32 max_gain_corr_amp_exp;
  WORD32 frame_length_indicator;
  WORD32 max_order_to_be_transmitted;
  WORD32 diff_order_bits;
  WORD32 diff_order;
  WORD32 max_num_of_coeffs_to_be_transmitted;
  WORD32 max_num_add_active_amb_coeffs;
  WORD32 vq_conf_bits;
  WORD32 num_v_vec_vq_elements_bits;
  WORD32 use_phase_shift_decorr;
  UWORD32 pred_dir_sigs;
  WORD32 num_bits_per_sf;
  WORD32 pred_subbands_idx;
  WORD32 coded_bw_first_band;
  WORD32 par_subband_table_idx;
  UWORD32 frame_length;
  WORD32 spat_interpolation_time;
  WORD32 amb_asign_m_bits;
  WORD32 active_pred_ids_bits;
  WORD32 num_active_pred_ids_bits;
  WORD32 gain_corr_prev_amp_exp_bits;
  UWORD32 idx_offset;
  WORD32 v_vec_length_used;
  WORD32 num_of_dir_length;
  WORD32 index_length;
  FLOAT32 matrix[64 * 24];
  WORD32 matrix_spk_id;
  WORD32 matrix_present;
  WORD32 matrix_index;
  ia_hoa_matrix_struct str_hoa_matrix;
} ia_hoa_config_struct;

#define BS_OUTPUT_CHANNEL_POS_NA -1   /* n/a                                 */
#define BS_OUTPUT_CHANNEL_POS_L 0     /* Left Front                          */
#define BS_OUTPUT_CHANNEL_POS_R 1     /* Right Front                         */
#define BS_OUTPUT_CHANNEL_POS_C 2     /* Center Front                        */
#define BS_OUTPUT_CHANNEL_POS_LFE 3   /* Low Frequency Enhancement           */
#define BS_OUTPUT_CHANNEL_POS_LS 4    /* Left Surround                       */
#define BS_OUTPUT_CHANNEL_POS_RS 5    /* Right Surround                      */
#define BS_OUTPUT_CHANNEL_POS_LC 6    /* Left Front Center                   */
#define BS_OUTPUT_CHANNEL_POS_RC 7    /* Right Front Center                  */
#define BS_OUTPUT_CHANNEL_POS_LSR 8   /* Rear Surround Left                  */
#define BS_OUTPUT_CHANNEL_POS_RSR 9   /* Rear Surround Right                 */
#define BS_OUTPUT_CHANNEL_POS_CS 10   /* Rear Center                         */
#define BS_OUTPUT_CHANNEL_POS_LSD 11  /* Left Surround Direct                */
#define BS_OUTPUT_CHANNEL_POS_RSD 12  /* Right Surround Direct               */
#define BS_OUTPUT_CHANNEL_POS_LSS 13  /* Left Side Surround                  */
#define BS_OUTPUT_CHANNEL_POS_RSS 14  /* Right Side Surround                 */
#define BS_OUTPUT_CHANNEL_POS_LW 15   /* Left Wide Front                     */
#define BS_OUTPUT_CHANNEL_POS_RW 16   /* Right Wide Front                    */
#define BS_OUTPUT_CHANNEL_POS_LV 17   /* Left Front Vertical Height          */
#define BS_OUTPUT_CHANNEL_POS_RV 18   /* Right Front Vertical Height         */
#define BS_OUTPUT_CHANNEL_POS_CV 19   /* Center Front Vertical Height        */
#define BS_OUTPUT_CHANNEL_POS_LVR 20  /* Left Surround Vertical Height Rear  */
#define BS_OUTPUT_CHANNEL_POS_RVR 21  /* Right Surround Vertical Height Rear */
#define BS_OUTPUT_CHANNEL_POS_CVR 22  /* Center Vertical Height Rear         */
#define BS_OUTPUT_CHANNEL_POS_LVSS 23 /* Left Vertical Height Side Surround  */
#define BS_OUTPUT_CHANNEL_POS_RVSS 24 /* Right Vertical Height Side Surround */
#define BS_OUTPUT_CHANNEL_POS_TS 25   /* Top Center Surround                 */
#define BS_OUTPUT_CHANNEL_POS_LFE2 26 /* Low Frequency Enhancement 2         */
#define BS_OUTPUT_CHANNEL_POS_LB 27   /* Left Front Vertical Bottom          */
#define BS_OUTPUT_CHANNEL_POS_RB 28   /* Right Front Vertical Bottom         */
#define BS_OUTPUT_CHANNEL_POS_CB 29   /* Center Front Vertical Bottom        */
#define BS_OUTPUT_CHANNEL_POS_LVS 30  /* Left Vertical Height Surround       */
#define BS_OUTPUT_CHANNEL_POS_RVS 31  /* Right Vertical Height Surround      */

#define BS_MAX_NUM_OUT_CHANNELS (255)

#ifdef LC_LEVEL_4
#define MAX_CHANNEL_COUNT (28)

#define SEQUENCE_COUNT_MAX (48)
#else
#define MAX_CHANNEL_COUNT (16)

#define SEQUENCE_COUNT_MAX (24)
#endif
#define PARAM_DRC_TYPE_FF_NODE_COUNT_MAX (9)
#define PARAM_DRC_INSTRUCTIONS_COUNT_MAX (8)
#define DOWNMIX_ID_COUNT_MAX (8)
#define DRC_SET_ID_COUNT_MAX (16)
#define EQ_SET_ID_COUNT_MAX (8)
#define LOUD_EQ_GAIN_SEQUENCE_COUNT_MAX (4)
#define LOUD_EQ_INSTRUCTIONS_COUNT_MAX (8)
#define FILTER_ELEMENT_COUNT_MAX (16)
#define FILTER_BLOCK_COUNT_MAX (16)
#define REAL_ZERO_RADIUS_ONE_COUNT_MAX (14)
#define REAL_ZERO_COUNT_MAX (64)
#define COMPLEX_ZERO_COUNT_MAX (64)
#define REAL_POLE_COUNT_MAX (16)
#define COMPLEX_POLE_COUNT_MAX (16)
#define FIR_ORDER_MAX (128)
#define EQ_NODE_COUNT_MAX (33)
#define UNIQUE_SUBBAND_GAIN_COUNT_MAX (16)
#define EQ_SUBBAND_GAIN_COUNT_MAX (135)
#define EQ_CHANNEL_GROUP_COUNT_MAX (4)
#define EQ_FILTER_BLOCK_COUNT_MAX (4)
#define EQ_INSTRUCTIONS_COUNT_MAX (8)

#define N_DELTA_TIME_CODE_TABLE_ENTRIES_MAX (512 + 14)
#define GAIN_SET_COUNT_MAX SEQUENCE_COUNT_MAX
#define SPLIT_CHARACTERISTIC_NODE_COUNT_MAX (4)
#define SPLIT_CHARACTERISTIC_COUNT_MAX (8)
#define SHAPE_FILTER_COUNT_MAX (8)
#define CHANNEL_GROUP_COUNT_MAX SEQUENCE_COUNT_MAX
#define DRC_BAND_COUNT_MAX BAND_COUNT_MAX
#define DOWNMIX_COEFF_COUNT_MAX (32 * 32)
#define MAX_AUDIO_PREROLLS 1
#define MAX_EXT_ELE_PAYLOAD (1536)
#define MAX_CFG_DATA_LENGTH (768)
#define MEASUREMENT_COUNT_MAX 16
#define MAX_INTERP_BUF_STRUCT_SIZE                                                               \
  (2 * sizeof(ia_drc_node_struct) +                                                              \
   (2 * AUDIO_CODEC_FRAME_SIZE_MAX + MAX_SIGNAL_DELAY) * sizeof(WORD32))
#define MAX_UNIDRC_PERSISTENT_SIZE                                                               \
                                                                                                 \
  ((MAX_GAIN_ELE_COUNT * SEL_DRC_COUNT * (MAX_INTERP_BUF_STRUCT_SIZE + 32)) +                    \
   (BAND_COUNT_MAX * MAX_NUM_CHANNELS * (OUT_FRAMELENGTH_1024 + sizeof(void *))))

typedef struct multichannel_data_tag
{
  WORD32 stereo_filling;
  WORD32 signal_type;
  WORD32 prev_signal_type;
  WORD32 use_tool;
  WORD32 keep_tree;
  WORD32 num_pairs;
  WORD32 start_element;
  WORD32 start_channel;
  WORD32 win_seq_is_long_prev[MAX_NUM_MC_BOXES];
  WORD32 channel_mask[MAX_NUM_CHANNELS];
  WORD32 channel_map[MAX_NUM_CHANNELS];
  WORD32 num_ch_to_apply;
  WORD32 code_pairs[MAX_NUM_MC_BOXES][2];
  WORD32 pair_coeffq_sfb_prev[MAX_NUM_MC_BOXES][MAX_NUM_MC_BANDS]; // pairCoeffQSfbPrev
  WORD32 pair_coeffq_fb_prev[MAX_NUM_MC_BOXES];
  /* fullband angle */ // pairCoeffQFbPrev
  WORD32 b_delta_time[MAX_NUM_MC_BOXES];
  WORD32 pred_dir[MAX_NUM_MC_BOXES];
  WORD32 pair_coeffq_sfb[MAX_NUM_MC_BOXES][MAX_NUM_MC_BANDS];
  WORD32 num_mask_band[MAX_NUM_MC_BOXES];
  WORD32 mask[MAX_NUM_MC_BOXES][MAX_NUM_MC_BANDS];
  WORD32 mask_flag[MAX_NUM_MC_BOXES];
  WORD32 bandwise_coeff_flag[MAX_NUM_MC_BOXES];
  WORD32 stereo_filling_flag[MAX_NUM_MC_BOXES];
  FLOAT32 *prev_out_spec[MAX_NUM_CHANNELS];
} ia_multichannel_data;

typedef struct
{
  WORD32 islong;
  WORD32 max_win_len;
  WORD32 samp_per_bk;
  WORD32 sfb_per_bk;
  WORD32 bins_per_sbk;
  WORD32 sfb_per_sbk;

  const WORD16 *ptr_sfb_tbl;
  pWORD16 sfb_width;
  WORD16 sfb_idx_tbl[125];
  WORD32 num_groups;
  WORD16 group_len[8];
} ia_sfb_info;

typedef struct ia_speaker_info_str_t
{
  WORD16 elevation;
  WORD16 azimuth;
  WORD16 is_lfe;
  WORD16 original_position;
  WORD16 is_already_used;
  WORD32 pair_type;
  struct ia_speaker_info_str_t *symmetric_pair;
} ia_speaker_info_str;

typedef struct
{
  UWORD16 is_cicp_spk_idx;
  UWORD16 cicp_spk_idx;
  UWORD16 el_class;
  WORD16 el_angle_idx;
  UWORD16 el_direction;
  WORD16 az_angle_idx;
  UWORD16 az_direction;
  UWORD16 is_lfe;
} ia_flex_spk_cfg_str;

#define CICP2GEOMETRY_MAX_LOUDSPEAKERS (32)

typedef struct
{
  UWORD16 also_add_symetric_pair;
  UWORD16 angular_precision;
  ia_flex_spk_cfg_str str_flex_spk_descr[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  FLOAT32 azimuth[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  FLOAT32 elevation[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
} ia_flex_spk_data_str;

typedef struct
{
  WORD32 spk_layout_type;
  WORD32 cicp_spk_layout_idx;
  WORD32 num_speakers;
  WORD32 cicp_spk_idx[MAX_NUM_OUT_CHANNELS];
  ia_flex_spk_data_str str_flex_spk;
} ia_speaker_config_3d;

typedef struct
{
  WORD32 cicp_loudspeaker_idx;
  WORD32 az;
  WORD32 el;
  WORD32 lfe;
  WORD32 screen_relative;
  WORD32 loudspeaker_type;
} ia_ch_geometry;

typedef struct
{
  WORD32 spk_layout_type;
  WORD32 cicp_spk_layout_idx;
  WORD32 cicp_spk_idx[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  ia_flex_spk_data_str str_flex_spk_data;

  UWORD32 num_speakers;
  WORD32 num_ch;
  WORD32 num_lfes;
  ia_ch_geometry geometry[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
} ia_interface_speaker_config_3d;

/*Object Metadta Decoding config structure*/
#define MAX_NUM_OAM_OBJS (24)

typedef struct
{
  WORD32 frame_length;
  WORD32 cc_frame_length;
  WORD32 num_objects;
  WORD32 sig_start_id;
  FLAG ld_md_coding_present;
  FLAG dyn_obj_priority_present;
  FLAG uniform_spread_present;
  FLAG is_screen_rel_obj[MAX_NUM_OAM_OBJS];
  FLAG screen_rel_objs_present;
} ia_oam_dec_config_struct;

typedef struct
{
  WORD8 has_diffuseness;
  WORD8 has_common_group_diffuseness;
  WORD8 has_excluded_sectors;
  WORD8 has_common_group_excluded_sectors;
  WORD8 has_closest_speaker_condition;
  WORD8 closest_spk_thr_angle;
  WORD8 has_divergence[MAX_NUM_OAM_OBJS];
  WORD8 divergence_az_range[MAX_NUM_OAM_OBJS];
  WORD8 use_only_predefined_sectors[MAX_NUM_OAM_OBJS];
  WORD32 num_obj_with_divergence;
} ia_enh_oam_config_struct;

typedef struct
{
  UWORD32 num_sig_group;
  UWORD32 group_type[MAX_AUDIO_GROUPS];
  UWORD32 num_sig[MAX_AUDIO_GROUPS];
  UWORD32 differs_from_ref_layout[MAX_AUDIO_GROUPS];
  UWORD32 num_grp_objs[MAX_AUDIO_GROUPS];
  UWORD32 num_ch;
  UWORD32 num_audio_obj;
  UWORD32 num_saoc_transport_ch;
  UWORD32 num_hoa_transport_ch;
  UWORD32 fixed_position[MAX_AUDIO_GROUPS];
  UWORD32 group_priority[MAX_AUDIO_GROUPS];
  UWORD32 signal_grp_info_present;
  UWORD32 format_converter_enable;
  UWORD32 domain_switcher_enable;
  WORD32 num_ch_based_groups;
  WORD32 num_obj_based_groups;
  WORD32 num_hoa_based_groups;
  ia_speaker_config_3d audio_ch_layout[MAX_AUDIO_CHANNEL_LAYOUTS];
} ia_signals_3d; // SIGNALS_3D;

typedef struct
{
  UINT32 tw_mdct;
  UINT32 noise_filling;
  UINT32 enhanced_noise_filling;
  UINT32 full_band_lpd;
  UINT32 lpd_stereo_index;
  UINT32 stereo_config_index;

  UINT32 usac_ext_eleme_def_len;
  UINT32 usac_ext_elem_pld_frag;

  ia_usac_igf_init_config_struct str_usac_ele_igf_init_config;
} ia_usac_dec_element_config_struct;

/* MPEGD DRC related structures */
#define MAX_SIGNAL_DELAY (1024 + 512)
#define SEL_DRC_COUNT (3)

#define ASCPARSER_MAX_DMX_MATRICES_PER_ID (8)
#define ASCPARSER_MAX_DMX_MAX_GROUPS_ASSIGNED (16)
#define ASCPARSER_MAX_DMX_MATRIX_SIZE (256)   /* bytes */
#define ASCPARSER_MAX_DMX_MATRIX_ELEMENTS (6) /* Max number of downmix matrices one can embed */

typedef struct
{
  WORD8 dmx_id;
  WORD32 dmx_type;
  WORD32 cicp_spk_layout_idx;
  WORD32 downmix_mtx_count;
  WORD32 num_assigned_group_ids[ASCPARSER_MAX_DMX_MATRICES_PER_ID];
  WORD32 signal_group_id[ASCPARSER_MAX_DMX_MATRICES_PER_ID]
                        [ASCPARSER_MAX_DMX_MAX_GROUPS_ASSIGNED];
  WORD32 dmx_matrix_len_bits[ASCPARSER_MAX_DMX_MATRICES_PER_ID];
  WORD8 downmix_matrix[ASCPARSER_MAX_DMX_MATRICES_PER_ID][ASCPARSER_MAX_DMX_MATRIX_SIZE];
} ia_usac_ext_cfg_dmx_mtx;

typedef struct
{
  UWORD32 dmx_id;
  UWORD32 dmx_config_type;
  UWORD32 passive_dmx_flag;
  UWORD32 phase_align_strength;
  UWORD32 immersive_downmix_flag;
  UWORD32 downmix_id_count;
  ia_usac_ext_cfg_dmx_mtx dmx_matrix[ASCPARSER_MAX_DMX_MATRIX_ELEMENTS];
} ia_usac_ext_cfg_dmx_cfg;

typedef struct
{
  UWORD32 bsnum_compatible_sets;
  UWORD32 reserved;
  UWORD32 compatible_set_indication[16];
  UWORD32 compat_lc_lvl;
  UWORD32 compat_bp_lvl;

} ia_struct_compatible_profile_config;
typedef struct
{
  UWORD32 num_elements;
  UWORD32 num_config_extensions;
  UWORD32 usac_element_type[MAX_EXT_ELEMENTS];
  WORD32 ia_ext_ele_payload_type[MAX_EXT_ELEMENTS];
  ia_multichannel_data ia_mcdata[MAX_NUM_SIGNALGROUPS];
  UWORD32 ia_ext_element_start;
  UWORD32 ia_ext_element_stop;
  ia_sfb_info *ia_sfb_info[MAX_TIME_CHANNELS];
  WORD32 num_output_chns[MAX_EXT_ELEMENTS];
  ia_hoa_config_struct str_hoa_config;
  WORD32 ele_length_present;
  ia_usac_ext_cfg_dmx_cfg dmx_cfg;
  WORD32 cicp_idx;
  ia_usac_dec_element_config_struct str_usac_element_config[MAX_EXT_ELEMENTS];

  WORD32 usac_cfg_ext_info_present[MAX_CONFIG_EXTENSIONS];
  WORD32 usac_ext_ele_payload_present[MAX_EXT_ELEMENTS];
  WORD32 usac_cfg_ext_info_len[MAX_CONFIG_EXTENSIONS];
  WORD32 usac_cfg_ext_info_type[MAX_CONFIG_EXTENSIONS];
  WORD32 usac_ext_ele_payload_len[MAX_EXT_ELEMENTS];
  WORD32 usac_ext_gain_payload_len[MAX_EXT_ELEMENTS];
  WORD32 usac_ext_gain_payload_prev_len;
  UWORD8 usac_cfg_ext_info_buf[MAX_CONFIG_EXTENSIONS][MAX_EXT_ELE_PAYLOAD];
  UWORD8 usac_ext_ele_payload_buf[MAX_EXT_ELEMENTS][MAX_EXT_ELE_PAYLOAD];
  UWORD8 usac_ext_gain_payload_buf[MAX_EXT_ELEMENTS][MAX_AUDIO_PREROLLS * MAX_EXT_ELE_PAYLOAD];
  UWORD8 usac_ext_gain_payload_prev_buf[MAX_EXT_ELE_PAYLOAD];
  WORD32 loudness_ext_config_present;
  WORD32 loudness_ext_config_idx;
  WORD32 downmix_ext_config_idx;
  WORD32 downmix_ext_config_present;
  UWORD32 preroll_bytes[MAX_AUDIO_PREROLLS + 2]; // Contain the number of bytes in each preroll
  WORD32 preroll_counter;                        // count the number o prerolls in a frame.
  ia_struct_compatible_profile_config str_compatible_profile_config;
  WORD32 preroll_flag;
  WORD32 hoa_matrix_ext_config_present;
  UWORD32 mpegh_profile_lvl;
  WORD32 compat_profile_cfg_present;
} ia_usac_decoder_config_struct;
typedef struct
{
  WORD32 has_reference_distance;
  WORD32 bs_reference_distance;
  WORD32 has_object_distance[MAX_AUDIO_GROUPS];
  WORD32 direct_head_phone[MAX_AUDIO_GROUPS];
  WORD32 has_intracoded_data;
  WORD32 fixed_distance;
  WORD32 default_distance;
  WORD32 common_distance;
  WORD32 position_distance[MAX_AUDIO_GROUPS];
  WORD32 obj_meta_data_present;
  WORD32 flag_dist_absolute;
  WORD32 position_dist;
  WORD32 flag_dist;
  WORD32 nbit_dist;
  WORD32 position_bits_dist_diff;
  WORD32 prod_metadata_present;
  WORD32 payload_length;
} ia_prod_meta_data_struct;
typedef struct
{
  UINT32 usac_sampling_frequency_index;
  UINT32 usac_sampling_frequency;
  UINT32 core_sbr_framelength_index;
  UINT32 channel_configuration_index;

  UINT32 num_out_channels;
  ia_prod_meta_data_struct str_prod_metat_data;
  UINT32 enh_obj_md_present;
  UINT32 receiver_delay_compensation;
  ia_usac_decoder_config_struct str_usac_dec_config;
  ia_signals_3d signals_3d;
  ia_oam_dec_config_struct obj_md_cfg[MAX_AUDIO_GROUPS];
  ia_enh_oam_config_struct enh_obj_md_cfg;
  ia_drc_config uni_drc_cfg;
  ia_drc_params_bs_dec_struct uni_drc_bs_params;
  ia_drc_loudness_info_set_struct str_loudness_info;
} ia_usac_config_struct;

VOID ia_core_coder_conf_default(ia_usac_config_struct *pstr_usac_conf);

VOID ia_core_coder_read_escape_value(ia_bit_buf_struct *it_bit_buff, UWORD32 *ext_ele_value,
                                     UWORD32 no_bits1, UWORD32 no_bits2, UWORD32 no_bits3);

IA_ERRORCODE impeghd_speaker_config_3d(VOID *buf_handle,
                                       ia_interface_speaker_config_3d *spk_config_3d);

WORD32 impeghd_elev_idx_degree(WORD32 idx, WORD32 direction, WORD32 precision);
WORD32 impeghd_azi_idx_degree(WORD32 idx, WORD32 direction, WORD32 precision);
IA_ERRORCODE impeghd_get_geom_frm_spk_cfg(ia_interface_speaker_config_3d *speaker_config_3d,
                                          ia_ch_geometry *ptr_geometry, WORD32 *num_channels,
                                          WORD32 *num_lfes);

#endif /* IA_CORE_CODER_CONFIG_H */
