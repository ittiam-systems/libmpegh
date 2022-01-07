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

#ifndef __IMPEGHD_ELE_INTRFC_INTERACTION__
#define __IMPEGHD_ELE_INTRFC_INTERACTION__

#define MAX_NUM_EI_BRIR_PAIRS 10
#define MAX_EI_BRIR_SIZE 128
#define MAX_EI_LENGTH_DIRECT_FILTER 1024
#define MAX_NUM_DIFFUSE_BLOCKS_EI 10

#define MAE_MAX_NUM_OBJECTS (32)
#define MAE_MAX_NUM_EXCLUDED_SECTORS (15)
#define MAE_MAX_NUM_SPEAKERS (32)
#define MAE_DIFFUSE_DECORR_LENGTH (256)
#define MAE_MAX_NUM_GROUPS (32)

#define MAE_MAX_NUM_GROUP_PRESETS (32)

#define GCA_DEFAULT_REF_DIST 57
#define ID_EXT_GCA_PROD_METADATA 0
#define GCA_CHANNEL_GAIN 96
#define ID_EXT_GHA_PROD_METADATA 0

typedef struct
{
  FLOAT32 ls_azimuth;
  FLOAT32 ls_elevation;
  FLOAT32 ls_azimuth_start;
  FLOAT32 ls_azimuth_end;
  FLOAT32 ls_elevation_start;
  FLOAT32 ls_elevation_end;
  WORD32 lfe_flag;
  WORD32 screen_rel_flag;
} ia_cicp_ls_geo_str_1;
typedef struct
{
  UWORD32 wire_id[128];
  WORD32 enable_ele_int;
  UWORD32 num_spk;
  WORD32 *ptr_spk_dist;
  WORD32 ext_dist_compensation;
  WORD32 has_knwn_pos[MAX_NUM_CHANNELS];
  WORD32 loud_spk_elevation[MAX_NUM_CHANNELS];
  WORD32 loud_spk_dist[MAX_NUM_CHANNELS];
  FLOAT32 loud_spk_azimuth[MAX_NUM_CHANNELS];
  FLOAT32 loud_spk_calibration_gain[MAX_NUM_CHANNELS];
  FLOAT32 *ptr_spk_gain;

  UWORD8 binaural_name[256];
  UWORD8 use_tracking_mode;
  UWORD32 brir_sampling[MAX_NUM_CHANNELS];
  UWORD32 brir_pairs;
  WORD32 measurement_setup;

  WORD32 flag_hrir;
  WORD32 d_init;
  WORD32 k_max;
  WORD32 k_conv;
  WORD32 k_ana;

  WORD32 lcl_scrn_sz_az;
  WORD32 lcl_scrn_sz_left_az;
  WORD32 lcl_scrn_sz_right_az;
  WORD32 lcl_scrn_sz_top_el;
  WORD32 lcl_scrn_sz_bottom_el;
  FLOAT32 lcl_scrn_offset;

  ia_interface_speaker_config_3d spk_config;
  ia_cicp_ls_geo_str_1 ptr_cicp_ls_geo[MAX_NUM_CHANNELS];
  ia_binaural_renderer *pstr_binaural_renderer;

  WORD32 dmx_id;
  WORD32 has_local_screen_size_info;
  WORD32 ren_type;
  WORD32 has_lcl_scrn_elevation_info;

  WORD32 num_individ_setups;
  WORD32 use_individ_grp_setups;
  WORD32 is_brir_rendering;
} ia_local_setup_struct;

typedef struct
{
  UWORD8 gca_direct_headphone[8][32];
  UWORD8 gca_has_ref_dist[8];
  UWORD8 gca_bs_ref_dis[8];
} ia_prod_meta_data;

typedef struct
{
  UWORD8 gca_frame_length;
  UWORD8 gca_audio_truncation;
  UWORD32 gca_num_samples;

  UWORD32 gca_num_out_ch_grp;
  UWORD32 gca_num_ch[512];
  UWORD32 gca_element_id[512][65536];

  UWORD8 gca_fixed_pos[512];

  UWORD8 gca_grp_priority[512];
  UWORD8 gca_ch_gain[512];

  UWORD8 gca_downmix_avail[512];
  ia_interface_speaker_config_3d spk_config;
} ia_ch_metadata;

typedef struct
{
  UWORD8 ei_intrctn_sign_data_type;
  UWORD8 ei_intrctn_sign_data_len;
  UWORD8 ei_intrctn_sign_data[256];
  UWORD8 has_zoom_area_sz;

  UWORD8 ei_intrctn_mode;
  UWORD8 ei_num_groups;
  UWORD8 ei_group_preset_id;

  WORD32 zoom_az_center;
  WORD32 zoom_az;
  WORD32 zoom_el_center;
  WORD32 zoom_el;

  UWORD8 ei_grp_id[128];
  UWORD8 ei_on_off[128];
  UWORD8 ei_route_to_wire[128];
  WORD32 route_to_wire_id[128];
  UWORD8 ei_change_pos[128];
  UWORD8 ei_az_offset[128];
  UWORD8 ei_el_offset[128];
  UWORD8 ei_dist_fact[128];
  UWORD8 ei_change_gain[128];
  UWORD8 ei_gain[128];
  UWORD8 ei_data_present;
} ia_ele_intrctn;

typedef struct
{
  UWORD32 sd_yaw;
  UWORD32 sd_pitch;
  UWORD32 sd_roll;
  UWORD32 sd_azimuth;
  UWORD32 sd_elevation;
  UWORD32 sd_radius;
  UWORD32 sd_buf_init_done;
  WORD32 min_bits_needed;
  ia_bit_buf_struct sd_buf;
  WORD32 scene_dspl_data_present;
} ia_scene_disp_data;

typedef struct
{
  WORD32 spk_layout_type;
  WORD32 cicp_spk_layout_idx;
  WORD32 num_speakers;
  WORD32 num_lfes;
  WORD32 cicp_spk_idx[MAX_NUM_OUT_CHANNELS];
  WORD32 has_knwn_pos[MAX_NUM_OUT_CHANNELS];
  WORD32 group_setup_id[MAE_MAX_NUM_GROUPS];
  FLOAT32 loud_spk_azimuth[MAX_NUM_OUT_CHANNELS];
  FLOAT32 loud_spk_elevation[MAX_NUM_OUT_CHANNELS];
  ia_flex_spk_data_str str_flex_spk;
} ia_sig_group_ls_setup_data;

typedef struct
{
  FLOAT32 gain_modified_prev_frm[MAE_MAX_NUM_GROUPS];
  FLOAT32 az_modified[MAE_MAX_NUM_GROUPS];
  FLOAT32 ele_modified[MAE_MAX_NUM_GROUPS];
  WORD32 on_off_status_modified[MAE_MAX_NUM_GROUPS];
  WORD32 process[MAE_MAX_NUM_GROUPS];
  WORD32 scrn_rel_objs[MAE_MAX_NUM_GROUPS];
  WORD32 orig_num_grp_members[MAE_MAX_NUM_GROUPS];
  WORD32 divergence_asi_modified;

  FLOAT32 gain_modified_grp[MAE_MAX_NUM_GROUPS];
  FLOAT32 gain_modified_single_object[MAE_MAX_NUM_GROUPS];
  FLOAT32 dist_modified[MAE_MAX_NUM_GROUPS];
  FLOAT32 diffuse_decorr_filt_states[MAE_MAX_NUM_SPEAKERS][MAE_DIFFUSE_DECORR_LENGTH];
  WORD32 diffuse_counter;
  WORD32 diffuse_compensateDelay;

  WORD32 list_oam[MAE_MAX_NUM_OBJECTS];
  WORD32 oam_count;
  WORD32 num_objs_in;
  WORD32 num_objs_out;
  WORD32 num_spks;

  WORD32 apply_intrct_data;

  WORD32 interaction_type;
  WORD32 dflt_preset_mode;
  WORD32 num_decoded_groups;
  WORD32 num_switch_groups;
  WORD32 en_scrn_rel_processing;
  WORD32 has_scrn_sz;
  WORD32 num_ele_in;
  WORD32 num_ele_out;
  FLOAT32 loudness_comp_gain;
  FLOAT32 **buffer_temp;
  ia_sig_group_ls_setup_data str_ls_setup_intrct[MAE_MAX_NUM_GROUPS];
  ia_ele_intrctn *ptr_ele_interaction_data;
  ia_local_setup_struct *ptr_local_setup_info;
} ia_interaction_data_struct;

IA_ERRORCODE impeghd_3da_local_setup_information(ia_bit_buf_struct *ptr_bit_buf,
                                                 ia_local_setup_struct *ptr_ele_intrface_data,
                                                 ia_signals_3d pstr_signals_3d);

/* mpegh3daElementInteraction */
IA_ERRORCODE impeghd_ele_interaction(ia_bit_buf_struct *ptr_bit_buf,
                                     ia_ele_intrctn *ptr_ele_intrctn,
                                     ia_mae_audio_scene_info *str_mae_asi);

/* mpegh3daSceneDisplacementData */
IA_ERRORCODE impeghd_scene_displacement_data(ia_scene_disp_data *ptr_scene_disp_data);

#endif /* __IMPEGHD_ELE_INTRFC_INTERACTION__ */