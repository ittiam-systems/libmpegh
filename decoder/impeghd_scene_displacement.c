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

#include <stdio.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_defines.h"

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"
#include "ia_core_coder_acelp_info.h"

#include "ia_core_coder_channel.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_definitions.h"
#include "impeghd_memory_standards.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct_def.h"
#include "impeghd_binaural.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_config_params.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_format_conv_rom.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_nfc_filtering.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_renderer.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_spatial_decoder.h"
#include "impeghd_hoa_decoder.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_multichannel.h"
#include "ia_core_coder_headerdecode.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_oam_dec.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"
#include "impeghd_tbe_dec.h"
#include "impeghd_uni_drc_struct.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_struct.h"

#include "impeghd_peak_limiter_struct_def.h"
#include "impeghd_binaural.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_resampler.h"
#include "ia_core_coder_create.h"
#include "ia_core_coder_dec_main.h"
#include "impd_drc_host_params.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_metadata_preprocessor.h"
#include "impeghd_3d_vec_basic_ops.h"
#include "impeghd_scene_displacement.h"

/**
 * @defgroup SceDisp Scene Displacement Processing
 * @ingroup  SceDisp
 * @brief Scene Displacement Processing
 *
 * @{
 */

/**
 *  impeghd_get_sig_group_idx
 *
 *  \brief Calculate signal group index value.
 *
 *  \param [in]  pstr_asc            Pointer to audio scene config structure.
 *  \param [in]  pstr_signals_3d     Pointer to 3d signals depiction structure.
 *  \param [in]  group_idx           Group index value.
 *  \param [in]  member_idx          Member index value.
 *
 *  \return WORD32 Signal group index.
 *
 */
static WORD32 impeghd_get_sig_group_idx(ia_audio_specific_config_struct *pstr_asc,
                                        ia_signals_3d *pstr_signals_3d, WORD32 group_idx,
                                        WORD32 member_idx)
{
  WORD32 i, j;
  WORD32 sig_group_idx = -1;
  WORD32 mae_id = -1;
  WORD32 mae_id_in = -1;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;

  for (i = 0; i < member_idx; i++)
  {
    if (pstr_mae_asi->group_definition[group_idx].has_conjunct_members != 1)
    {
      mae_id_in = pstr_mae_asi->group_definition[group_idx].metadata_ele_id[member_idx];
    }
    else
    {
      mae_id_in = pstr_mae_asi->group_definition[group_idx].start_id + member_idx;
    }
  }

  for (i = 0; i < (WORD32)pstr_signals_3d->num_sig_group; i++)
  {
    for (j = 0; j < (WORD32)pstr_signals_3d->num_sig[i]; j++)
    {
      mae_id++;
      if (mae_id == mae_id_in)
      {
        sig_group_idx = i;
        break;
      }
    }
  }

  if (sig_group_idx == -1)
  {
    /* old signaling as fallback*/
    sig_group_idx = group_idx;
  }

  return sig_group_idx;
}

/**
 *  impeghd_rotate_cart_coord
 *
 *  \brief Brief description    Rotate the cartesian coordinates
 *
 *  \param pstr_cart_coord      Pointer to the cartesian coordinate structure
 *  \param pstr_scene_dspl_data Pointer to the scene displacement data structure
 *
 */
static VOID impeghd_rotate_cart_coord(ia_cart_coord_str *pstr_cart_coord,
                                      ia_scene_disp_data *pstr_scene_dspl_data)
{
  FLOAT32 rot1[3][3];
  FLOAT32 rot2[3][3];
  FLOAT32 rot3[3][3];
  FLOAT32 matrix[3][3] = {{0}};
  FLOAT32 temp[3][3] = {{0}};
  FLOAT32 posOut[3] = {0};
  FLOAT32 posIn[3] = {0};

  /* Rotational scene displacement parameters */
  FLOAT32 yaw = (FLOAT32)pstr_scene_dspl_data->sd_yaw;
  FLOAT32 pitch = (FLOAT32)pstr_scene_dspl_data->sd_pitch;
  FLOAT32 roll = (FLOAT32)pstr_scene_dspl_data->sd_roll;

  /* Positional scene displacement parameters */
  ia_cart_coord_str str_cart_coord;
  ia_polar_coord_str str_polar_coord;
  str_polar_coord.phi = (FLOAT32)pstr_scene_dspl_data->sd_elevation;
  str_polar_coord.theta = (FLOAT32)pstr_scene_dspl_data->sd_azimuth;

  str_polar_coord.phi = DEG_2_RAD * (90.0f - str_polar_coord.phi);
  str_polar_coord.theta = DEG_2_RAD * (str_polar_coord.theta + 90.0f);
  str_polar_coord.radius = (FLOAT32)pstr_scene_dspl_data->sd_radius;
  impeghd_pol_2_cart(&str_polar_coord, &str_cart_coord);

  posIn[2] = str_cart_coord.z;
  posIn[1] = str_cart_coord.y;
  posIn[0] = str_cart_coord.x;

  rot3[0][0] = (FLOAT32)cos(DEG_2_RAD * (roll));
  rot3[0][1] = 0;
  rot3[0][2] = (FLOAT32)sin(DEG_2_RAD * (roll));
  rot3[1][0] = 0;
  rot3[1][1] = 1;
  rot3[1][2] = 0;
  rot3[2][0] = (FLOAT32)-sin(DEG_2_RAD * (roll));
  rot3[2][1] = 0;
  rot3[2][2] = (FLOAT32)cos(DEG_2_RAD * (roll));

  rot2[0][0] = 1;
  rot2[0][1] = 0;
  rot2[0][2] = 0;
  rot2[1][0] = 0;
  rot2[1][1] = (FLOAT32)cos(DEG_2_RAD * (pitch));
  rot2[1][2] = (FLOAT32)-sin(DEG_2_RAD * (pitch));
  rot2[2][0] = 0;
  rot2[2][1] = (FLOAT32)sin(DEG_2_RAD * (pitch));
  rot2[2][2] = (FLOAT32)cos(DEG_2_RAD * (pitch));

  rot1[0][0] = (FLOAT32)cos(DEG_2_RAD * (yaw));
  rot1[0][1] = (FLOAT32)sin(DEG_2_RAD * (yaw));
  rot1[0][2] = 0;
  rot1[1][0] = (FLOAT32)-sin(DEG_2_RAD * (yaw));
  rot1[1][1] = (FLOAT32)cos(DEG_2_RAD * (yaw));
  rot1[1][2] = 0;
  rot1[2][0] = 0;
  rot1[2][1] = 0;
  rot1[2][2] = 1;

  { /* multiply rot1, rot2 */
    WORD32 l, c, i;
    for (l = 0; l < 3; l++)
    {
      for (c = 0; c < 3; c++)
      {
        (temp[l][c]) = 0.0f;
        for (i = 0; i < 3; i++)
        {
          (temp[l][c]) += (rot2[i][c] * rot1[l][i]);
        }
      }
    }
  }
  { /* multiply temp, rot3 */
    WORD32 l, c, i;
    for (l = 0; l < 3; l++)
    {
      for (c = 0; c < 3; c++)
      {
        (matrix[l][c]) = 0.0f;
        for (i = 0; i < 3; i++)
        {
          (matrix[l][c]) += (rot3[i][c] * temp[l][i]);
        }
      }
    }
  }

  { /* post-multiply with position */
    WORD32 i;
    for (i = 0; i < 3; i++)
    {
      posOut[i] = matrix[i][0] * (posIn[0] + str_cart_coord.x);
      posOut[i] += matrix[i][1] * (posIn[1] + str_cart_coord.y);
      posOut[i] += matrix[i][2] * (posIn[2] + str_cart_coord.z);
    }
  }
  pstr_cart_coord->z = posOut[2];
  pstr_cart_coord->y = posOut[1];
  pstr_cart_coord->x = posOut[0];
}

/**
 *  impeghd_apply_scene_displacement
 *
 *  \brief Processing of scene displacement data
 *
 *	\param [i/o] pstr_dec_data Pointer
 *to
 *decoder data structure
 *  \param [in]  pstr_asc                     Pointer to audio specific config structure
 *  \param [i/o] pstr_interact_cfg            Pointer to interaction data structure
 *  \param [in]  ptr_scratch_mem              Pointer to scratch memory
 *  \param [in]  oamDataAvailable             Indicates if oam data is available
 *  \param [in]  pstr_scene_displacement_data Pointer to scene displacement data structure
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE impeghd_apply_scene_displacement(ia_dec_data_struct *pstr_dec_data,
                                              ia_audio_specific_config_struct *pstr_asc,
                                              ia_interaction_data_struct *pstr_interact_cfg,
                                              pWORD8 ptr_scratch_mem, WORD32 oamDataAvailable,
                                              ia_scene_disp_data *pstr_scene_displacement_data)
{
  IA_ERRORCODE error = IA_MPEGH_DEC_NO_ERROR;
  WORD32 use_tracking_mode = 0;
  WORD32 i, j, k, s;
  WORD32 num_repro_spks = 0;
  WORD32 object_count = 0;
  WORD32 channel_count = 0;
  WORD32 num_groups;
  WORD8 *ptr_scratch = ptr_scratch_mem;
  (void)ptr_scratch;
  WORD32 *channel_names;
  FLOAT32 max_ls_dist = 0.0f;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  ia_signals_3d *pstr_signals_3d = &pstr_asc->str_usac_config.signals_3d;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_local_setup_struct *pstr_local_setup = &pstr_dec_data->str_local_setup_interaction;
  ia_cicp_ls_geo_str str_ls_geo[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
  ia_polar_coord_str str_polar_coord;
  ia_cart_coord_str str_cart_coord;
  num_groups = pstr_mae_asi->num_groups;
  use_tracking_mode = pstr_local_setup->use_tracking_mode;

  if (use_tracking_mode == 1)
  {
    /* get reproduction setup */
    if (pstr_local_setup->ren_type != 0)
    {
      /* binaural reproduction */
      WORD32 layout_type = pstr_local_setup->spk_config.spk_layout_type;
      WORD32 num_channels = 0;
      WORD32 num_lfes = 0;
      num_repro_spks = pstr_local_setup->spk_config.spk_layout_type;

      for (i = 0; i < num_repro_spks; i++)
      {
        if (layout_type == 2)
        {
          if (pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .is_cicp_spk_idx == 1)
          {
            memcpy(&str_ls_geo[i],
                   &ia_cicp_ls_geo_tbls[pstr_local_setup->spk_config.str_flex_spk_data
                                            .str_flex_spk_descr[i]
                                            .cicp_spk_idx],
                   sizeof(str_ls_geo[i]));
          }
          else
          {
            WORD32 az_angle_idx =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].az_angle_idx;
            WORD32 az_direction =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].az_direction;
            WORD32 el_angle_idx =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].el_angle_idx;
            WORD32 el_direction =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].el_direction;
            WORD32 prec = pstr_local_setup->spk_config.str_flex_spk_data.angular_precision;

            str_ls_geo[i].ls_elevation =
                impeghd_md_proc_asc_az_ele_idx_to_deg(el_angle_idx, el_direction, prec);
            str_ls_geo[i].ls_azimuth =
                impeghd_md_proc_asc_az_ele_idx_to_deg(az_angle_idx, az_direction, prec);
            str_ls_geo[i].lfe_flag =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].is_lfe;
            str_ls_geo[i].screen_rel_flag = 0;
          }
        }
        else if (layout_type == 1)
        {
          memcpy(&str_ls_geo[i],
                 &ia_cicp_ls_geo_tbls[pstr_local_setup->spk_config.str_flex_spk_data
                                          .str_flex_spk_descr[i]
                                          .cicp_spk_idx],
                 sizeof(str_ls_geo[i]));
        }
        else if ((layout_type == 0) && (i == 0))
        {
          impeghd_cicpidx_2_ls_geometry(pstr_local_setup->spk_config.cicp_spk_layout_idx,
                                        (const ia_cicp_ls_geo_str **)&str_ls_geo[0],
                                        &num_channels, &num_lfes,
                                        (const WORD32 **)&channel_names);
        }
      }
    }
    else
    {
      WORD32 layout_type = pstr_local_setup->spk_config.spk_layout_type;
      WORD32 num_channels = 0;
      WORD32 num_lfes = 0;
      num_repro_spks = pstr_local_setup->spk_config.num_speakers;

      for (i = 0; i < num_repro_spks; i++)
      {

        if (layout_type == 2)
        {
          if (pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .is_cicp_spk_idx == 1)
          {
            memcpy(&str_ls_geo[i],
                   &ia_cicp_ls_geo_tbls[pstr_local_setup->spk_config.str_flex_spk_data
                                            .str_flex_spk_descr[i]
                                            .cicp_spk_idx],
                   sizeof(str_ls_geo[i]));
          }
          else
          {
            str_ls_geo[i].ls_azimuth = pstr_local_setup->spk_config.str_flex_spk_data.azimuth[i];
            str_ls_geo[i].ls_elevation =
                pstr_local_setup->spk_config.str_flex_spk_data.elevation[i];
            str_ls_geo[i].lfe_flag =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].is_lfe;
            str_ls_geo[i].screen_rel_flag = 0;
          }
        }
        else if (layout_type == 1)
        {
          memcpy(&str_ls_geo[i],
                 &ia_cicp_ls_geo_tbls[pstr_local_setup->spk_config.cicp_spk_idx[i]],
                 sizeof(str_ls_geo[i]));
        }
        else if ((layout_type == 0) && (i == 0))
        {
          impeghd_cicpidx_2_ls_geometry(pstr_local_setup->spk_config.cicp_spk_layout_idx,
                                        (const ia_cicp_ls_geo_str **)&str_ls_geo[0],
                                        &num_channels, &num_lfes,
                                        (const WORD32 **)&channel_names);
        }
      }
    }

    channel_count = 0;
    object_count = 0;

    for (i = 0; i < num_groups; i++)
    {
      WORD32 update_positions = 0;
      WORD32 num_members = pstr_mae_asi->group_definition[i].group_num_members;
      WORD32 type = impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i);

      if ((type == 1) && (!oamDataAvailable))
      {
        break;
      }

      /* check if scene displacement processing is enabled for the current group */
      if (pstr_signals_3d
              ->fixed_position[impeghd_get_sig_group_idx(pstr_asc, pstr_signals_3d, i, 0)] ==
          0) /* assume mae_group as subset of signal group */
      {
        update_positions = 1;
      }

      if (update_positions == 1)
      {
        WORD32 mem_id = -1;
        WORD32 oam_idx = -1;

        max_ls_dist = 0.0f;
        for (s = 0; s < num_repro_spks; s++)
        {
          if (pstr_local_setup->ren_type == 0)
          {
            WORD32 has_knwn_pos = (WORD32)pstr_local_setup->has_knwn_pos[s];
            if (has_knwn_pos == 1)
            {
              if ((pstr_local_setup->ptr_spk_dist != NULL) &&
                  (pstr_local_setup->ptr_spk_dist[s] > 0.0f))
              {
                max_ls_dist = ia_max_flt(max_ls_dist, (FLOAT32)pstr_local_setup->ptr_spk_dist[s]);
              }
            }
          }
        }

        for (k = 0; k < num_members; k++)
        {
          WORD32 sig_grp_idx = impeghd_get_sig_group_idx(pstr_asc, pstr_signals_3d, i, k);
          if (type == 1) /* OBJECTS */
          {
            if (pstr_mae_asi->group_definition[i].has_conjunct_members)
            {
              mem_id = pstr_mae_asi->group_definition[i].start_id + k;
            }
            else
            {
              mem_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
            }
            /* get OAM data for the current group member */
            for (j = 0; j < pstr_interact_cfg->oam_count; j++)
            {
              if (pstr_interact_cfg->list_oam[j] == mem_id)
              {
                oam_idx = j;
                break;
              }
            }
            if (oam_idx == -1)
            {
              return -1;
            }
            pstr_interact_cfg->az_modified[k] = pstr_oam_dec_state->azimuth_descaled[oam_idx];
            pstr_interact_cfg->ele_modified[k] = pstr_oam_dec_state->elevation_descaled[oam_idx];
            pstr_interact_cfg->dist_modified[k] = pstr_oam_dec_state->radius_descaled[oam_idx];
            object_count++;
          }
          else if (type == 0) /* CHANNELS */
          {
            ia_cicp_ls_geo_str *ptr_str_geo[CICP2GEOMETRY_MAX_LOUDSPEAKERS];
            ia_cicp_ls_geo_str geo[CICP2GEOMETRY_MAX_LOUDSPEAKERS] = {{0}};
            /* get intended speaker position*/
            if (pstr_signals_3d->audio_ch_layout[sig_grp_idx].cicp_spk_layout_idx > 0)
            { /* CICP speaker layout idx */
              WORD32 num_channels = -1;
              WORD32 num_lfes = -1;
              impeghd_cicpidx_2_ls_geometry(
                  pstr_signals_3d->audio_ch_layout[sig_grp_idx].cicp_spk_layout_idx,
                  (const ia_cicp_ls_geo_str **)&ptr_str_geo[0], &num_channels, &num_lfes,
                  (const WORD32 **)&channel_names);
              pstr_interact_cfg->az_modified[k] = geo[k].ls_azimuth;
              pstr_interact_cfg->ele_modified[k] = geo[k].ls_elevation;
              if (max_ls_dist > 0)
              {
                pstr_interact_cfg->dist_modified[k] = max_ls_dist;
              }
              else
              {
                pstr_interact_cfg->dist_modified[k] = (1023.0f);
              }
              if ((num_channels + num_lfes) != num_members)
              {
                error = -1;
                break;
              }
            }
            else if (pstr_signals_3d->audio_ch_layout[sig_grp_idx].cicp_spk_idx[k] >= 0)
            { /* list of CICP speaker idx */
              if (pstr_signals_3d
                      ->audio_ch_layout[impeghd_get_sig_group_idx(pstr_asc, pstr_signals_3d, i,
                                                                  k)]
                      .num_speakers != num_members)
              {
                error = -1;
                break;
              }
              memcpy(&geo[k],
                     &ia_cicp_ls_geo_tbls[pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                              .str_flex_spk.str_flex_spk_descr[k]
                                              .cicp_spk_idx],
                     sizeof(geo[k]));
              pstr_interact_cfg->az_modified[k] = geo[k].ls_azimuth;
              pstr_interact_cfg->ele_modified[k] = geo[k].ls_elevation;
              if (max_ls_dist > 0.0f)
              {
                pstr_interact_cfg->dist_modified[k] = max_ls_dist;
              }
              else
              {
                pstr_interact_cfg->dist_modified[k] = 1023.0f;
              }
            }
            else /* flexible signaling */
            {
              if (pstr_signals_3d->audio_ch_layout[sig_grp_idx].num_speakers != num_members)
              {
                error = -1;
                break;
              }
              if (pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                      .str_flex_spk.str_flex_spk_descr[k]
                      .is_cicp_spk_idx == 1)
              {
                memcpy(&geo[i],
                       &ia_cicp_ls_geo_tbls[pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                                .str_flex_spk.str_flex_spk_descr[k]
                                                .cicp_spk_idx],
                       sizeof(geo[i]));
                pstr_interact_cfg->az_modified[k] = geo[k].ls_azimuth;
                pstr_interact_cfg->ele_modified[k] = geo[k].ls_elevation;
                if (max_ls_dist > 0.0f)
                {
                  pstr_interact_cfg->dist_modified[k] = max_ls_dist;
                }
                else
                {
                  pstr_interact_cfg->dist_modified[k] = 1023.0f;
                }
              }
              else
              {
                WORD32 az_angle_idx = pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                          .str_flex_spk.str_flex_spk_descr[k]
                                          .az_angle_idx;
                WORD32 az_direction = pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                          .str_flex_spk.str_flex_spk_descr[k]
                                          .az_direction;
                WORD32 el_angle_idx = pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                          .str_flex_spk.str_flex_spk_descr[k]
                                          .el_angle_idx;
                WORD32 el_direction = pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                          .str_flex_spk.str_flex_spk_descr[k]
                                          .el_direction;
                WORD32 prec =
                    pstr_signals_3d->audio_ch_layout[sig_grp_idx].str_flex_spk.angular_precision;

                geo[k].ls_elevation =
                    impeghd_md_proc_asc_az_ele_idx_to_deg(el_angle_idx, el_direction, prec);
                geo[k].ls_azimuth =
                    impeghd_md_proc_asc_az_ele_idx_to_deg(az_angle_idx, az_direction, prec);
                geo[k].lfe_flag = pstr_signals_3d->audio_ch_layout[sig_grp_idx]
                                      .str_flex_spk.str_flex_spk_descr[k]
                                      .is_lfe;
                geo[k].screen_rel_flag = 0;

                pstr_interact_cfg->az_modified[k] = geo[k].ls_azimuth;
                pstr_interact_cfg->ele_modified[k] = geo[k].ls_elevation;
                if (max_ls_dist > 0.0f)
                {
                  pstr_interact_cfg->dist_modified[k] = max_ls_dist;
                }
                else
                {
                  pstr_interact_cfg->dist_modified[k] = 1023.0f;
                }
              }
            }

            /* check if the speaker exists in reproduction setup and include known position if so
             */
            for (s = 0; s < num_repro_spks; s++)
            {
              WORD32 exists = !(memcmp(&geo[k], &str_ls_geo[s], sizeof(str_ls_geo[s])));
              if (exists == 1)
              {
                if (pstr_local_setup->ren_type == 0)
                {
                  WORD32 has_knwn_pos = (WORD32)pstr_local_setup->has_knwn_pos[s];
                  if (has_knwn_pos == 1)
                  {
                    /* overwrite geometry with known reproduction speaker position */
                    pstr_interact_cfg->az_modified[k] =
                        (FLOAT32)pstr_local_setup->loud_spk_azimuth[s];
                    pstr_interact_cfg->ele_modified[k] =
                        (FLOAT32)pstr_local_setup->loud_spk_elevation[s];
                    /* overwrite distance with speaker distance */
                    if ((pstr_local_setup->ptr_spk_dist != NULL) &&
                        (pstr_local_setup->ptr_spk_dist[s] > 0.0f))
                    {
                      pstr_interact_cfg->dist_modified[k] =
                          (FLOAT32)pstr_local_setup->ptr_spk_dist[s];
                    }
                  }
                }
                else /* binaural rendering */
                {
                  /* set to BRIR measurement position */
                  pstr_interact_cfg->az_modified[k] = str_ls_geo[s].ls_azimuth;
                  pstr_interact_cfg->ele_modified[k] = str_ls_geo[s].ls_elevation;
                }
                break;
              }
            }
            channel_count++;
          }
          str_polar_coord.phi = pstr_interact_cfg->ele_modified[k];
          str_polar_coord.theta = pstr_interact_cfg->az_modified[k];
          str_polar_coord.radius = pstr_interact_cfg->dist_modified[k];
          str_polar_coord.theta = DEG_2_RAD * (90.0f + str_polar_coord.theta);
          str_polar_coord.phi = DEG_2_RAD * (90.0f - str_polar_coord.phi);
          impeghd_pol_2_cart(&str_polar_coord, &str_cart_coord);
          impeghd_rotate_cart_coord(&str_cart_coord, pstr_scene_displacement_data);
          impeghd_cart_2_pol(&str_cart_coord, &str_polar_coord);
          str_polar_coord.theta = (FLOAT32)(90.0f + RAD_2_DEG * str_polar_coord.theta);
          str_polar_coord.phi = (FLOAT32)(90.0f - RAD_2_DEG * str_polar_coord.phi);
          if (str_polar_coord.theta > 180)
          {
            str_polar_coord.theta -= 360;
          }
          if (str_polar_coord.theta < -180)
          {
            str_polar_coord.theta += 360;
          }

          if (str_polar_coord.phi > 90)
          {
            str_polar_coord.phi = 90 - (str_polar_coord.phi - 90);
          }
          if (str_polar_coord.phi < -90)
          {
            str_polar_coord.phi = -90 + (str_polar_coord.phi + 90);
          }

          if (type == 1) /* OBJECTS */
          {
            pstr_oam_dec_state->azimuth_descaled[oam_idx] = str_polar_coord.theta;
            pstr_oam_dec_state->elevation_descaled[oam_idx] = str_polar_coord.phi;
            pstr_oam_dec_state->radius_descaled[oam_idx] = str_polar_coord.radius;
          }
          else if (type == 0 && channel_count > 0) /* CHANNELS */
          {
            pstr_oam_dec_state->azimuth_descaled[channel_count - 1] =
                pstr_interact_cfg->az_modified[k];
            pstr_oam_dec_state->elevation_descaled[channel_count - 1] =
                pstr_interact_cfg->ele_modified[k];
            pstr_oam_dec_state->radius_descaled[channel_count - 1] =
                pstr_interact_cfg->dist_modified[k];
            pstr_oam_dec_state->gain_descaled[channel_count - 1] = 1.0f;
            pstr_oam_dec_state->spread_width_descaled[channel_count - 1] = 0.0f;
            pstr_oam_dec_state->spread_depth_descaled[channel_count - 1] = 0.0f;
            pstr_oam_dec_state->spread_height_descaled[channel_count - 1] = 0.0f;
            pstr_oam_dec_state->dyn_obj_priority_descaled[channel_count - 1] = 7;
          }
        }
      }
    }
  }
  return error;
}
/** @} */ /* End of SceDisp */
