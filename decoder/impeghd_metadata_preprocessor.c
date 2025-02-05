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
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"
#include "ia_core_coder_acelp_info.h"

#include "ia_core_coder_channel.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_igf_data_struct.h"
#include "impeghd_memory_standards.h"
#include "ia_core_coder_struct_def.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_config_params.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_multichannel.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_error_codes.h"
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_format_conv_rom.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_data_types.h"
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
#include "ia_core_coder_definitions.h"
#include "ia_core_coder_headerdecode.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_binaural_renderer.h"
#include "impd_drc_host_params.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_peak_limiter_struct_def.h"
#include "impeghd_resampler.h"
#include "ia_core_coder_struct.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_create.h"
#include "ia_core_coder_dec_main.h"
#include "ia_core_coder_arith_dec.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_metadata_preprocessor_defines.h"
#include "impeghd_metadata_preprocessor.h"
#include "impeghd_metadata_preprocessor_rom.h"
#include "impeghd_ren_interface_utils.h"
#include "impeghd_scene_displacement.h"

/**
 * @defgroup MDPreProc MetaData pre-processing
 * @ingroup  MDPreProc
 * @brief MetaData pre-processing
 *
 * @{
 */

/**
 *  impeghd_mp_ls_indivi_setups
 *
 *  \brief Metadata preprocessing loud speaker individual Setups.
 *
 *  \param [in] pstr_ls_setup_data Pointer to loudspeaker setup data.
 *  \param [in] num_groups         Number of groupds.
 *  \param [in] pstr_enh_oam_cfg   Pointer to enhanced OAM configuration structure.
 *
 *  \return num_individ_setups
 *
 */
static WORD32 impeghd_mp_ls_indivi_setups(ia_sig_group_ls_setup_data *pstr_ls_setup_data,
                                          WORD32 num_groups,
                                          ia_enh_oam_config_struct *pstr_enh_oam_cfg)
{
  /* standard setup */
  WORD32 num_individ_setups = 1;
  WORD32 i, j;
  WORD32 num_excluded_groups = 0;
  WORD32 isequal;
  WORD32 group_setup_id[MAE_MAX_NUM_GROUPS];
  WORD32 compare_flag[MAE_MAX_NUM_GROUPS][MAE_MAX_NUM_GROUPS];
  WORD32 list_spk_lfe[MAE_MAX_NUM_GROUPS][MAE_MAX_NUM_SPEAKERS];
  FLOAT32 list_spk_az[MAE_MAX_NUM_GROUPS][MAE_MAX_NUM_SPEAKERS];
  FLOAT32 list_spk_el[MAE_MAX_NUM_GROUPS][MAE_MAX_NUM_SPEAKERS];

  memset(list_spk_az, 0, sizeof(list_spk_az));
  memset(list_spk_el, 0, sizeof(list_spk_el));
  memset(list_spk_lfe, 0, sizeof(list_spk_lfe));

  /* assign standard setup to all groups */
  for (i = 0; i < MAE_MAX_NUM_GROUPS; i++)
  {
    if (i < num_groups)
    {
      group_setup_id[i] = 0; /* group has standard layout */
    }
    else
    {
      group_setup_id[i] = -1;
    }
    for (j = 0; j < MAE_MAX_NUM_GROUPS; j++)
    {
      compare_flag[i][j] = 1;
    }
  }

  /* mark groups with excluded sectors, copy speakers to lists for comparison */
  for (i = 0; i < num_groups; i++)
  {
    if (pstr_enh_oam_cfg->has_excluded_sectors > 0)
    {
      group_setup_id[i] = -2; /* group completely disabled, no speakers left */
      if (pstr_ls_setup_data[i].num_speakers > 0)
      {
        group_setup_id[i] = -3; /* group has a layout different then standard layout */
        num_excluded_groups++;
        for (j = 0; j < (WORD32)pstr_ls_setup_data[i].num_speakers; j++)
        {
          list_spk_az[i][j] = (FLOAT32)pstr_ls_setup_data[i].str_flex_spk.azimuth[j];
          list_spk_el[i][j] = (FLOAT32)pstr_ls_setup_data[i].str_flex_spk.elevation[j];
          list_spk_lfe[i][j] = pstr_ls_setup_data[i].str_flex_spk.str_flex_spk_descr[j].is_lfe;
        }
      }
    }
  }
  /* compare the setups for all groups with sectors */
  for (i = 0; i < num_groups; i++)
  {
    for (j = 0; j < num_groups; j++)
    {
      isequal = -1;
      if ((group_setup_id[i] < -2) && (group_setup_id[j] < -2) && (compare_flag[i][j] == 1))
      {
        WORD32 l, m;
        isequal = 1;
        for (l = 0; l < pstr_ls_setup_data[i].num_speakers; l++)
        {
          for (m = 0; m < pstr_ls_setup_data[i].num_speakers; m++)
          {
            if ((list_spk_az[i][l] != list_spk_az[j][m]) ||
                (list_spk_el[i][l] != list_spk_el[j][m]) ||
                (list_spk_lfe[i][l] != list_spk_lfe[j][m]))
            {
              isequal = 0;
            }
          }
        }

        compare_flag[i][j] = 0;
        compare_flag[j][i] = 0;

        /* for groups with the same setup, a unique setup number is copied to the setup list for
         * the corresponding groups */
        if (isequal == 1)
        {
          group_setup_id[i] = num_individ_setups;
          group_setup_id[j] = num_individ_setups;
        }
      }
    }
    /* if there is just one group with a specified setup, the number is copied to the setup list
     * for the corresponding group */
    if (isequal == 0)
    {
      group_setup_id[i] = num_individ_setups;
    }
    if (isequal > -1)
    {
      num_individ_setups++;
    }
  }

  memcpy(pstr_ls_setup_data->group_setup_id, group_setup_id,
         MAE_MAX_NUM_GROUPS * sizeof(group_setup_id[0]));
  return num_individ_setups;
}

/**
 *  impeghd_check_spk_exclusion_defined_sect
 *
 *  \brief Flags presence of a loudspeaker in a defined excluded sector.
 *
 *  \param [in] pstr_ls_geo      Pointer to loud speaker geometry structure.
 *  \param [in] pstr_local_setup Pointer to local setup information structure.
 *  \param [in] exc_sect_idx     Excluded sector index.
 *
 *  \return WORD32 Flag indicating loudspeaker's presence in excluded sector.
 *
 */
static WORD32 impeghd_check_spk_exclusion_defined_sect(ia_cicp_ls_geo_str *pstr_ls_geo,
                                                       ia_local_setup_struct *pstr_local_setup,
                                                       WORD32 exc_sect_idx)
{
  WORD32 exclude = 0;
  /*
   * Based on Table 271 of Specification ISO-IEC-23008-3
   */
  if (pstr_ls_geo->lfe_flag == 0)
  {
    switch (exc_sect_idx)
    {
    case 6: /* screen only */
    {
      if (((pstr_ls_geo->ls_azimuth > pstr_local_setup->lcl_scrn_sz_left_az) ||
           (pstr_ls_geo->ls_azimuth < pstr_local_setup->lcl_scrn_sz_right_az)) &&
          ((pstr_ls_geo->ls_elevation > pstr_local_setup->lcl_scrn_sz_top_el) ||
           (pstr_ls_geo->ls_elevation < pstr_local_setup->lcl_scrn_sz_bottom_el)))
      {
        exclude = 1;
      }
      break;
    }
    case 5: /* no surround */
    {
      if ((fabs(pstr_ls_geo->ls_azimuth) >= NINETY_DEGREE) ||
          ((pstr_ls_geo->ls_azimuth == ZERO_DEGREE) &&
           (pstr_ls_geo->ls_elevation == NINETY_DEGREE)))
      {
        exclude = 1;
      }
      break;
    }
    case 4: /* no left side */
    {
      if ((pstr_ls_geo->ls_azimuth > ZERO_DEGREE) &&
          (pstr_ls_geo->ls_azimuth < ONE_EIGHTY_DEGREE))
      {
        exclude = 1;
      }
      break;
    }
    case 3: /* no rigth side */
    {
      if ((pstr_ls_geo->ls_azimuth > -ONE_EIGHTY_DEGREE) &&
          (pstr_ls_geo->ls_azimuth < ZERO_DEGREE))
      {
        exclude = 1;
      }
      break;
    }
    case 2: /* no front */
    {
      if ((fabs(pstr_ls_geo->ls_azimuth) < NINETY_DEGREE) &&
          (fabs(pstr_ls_geo->ls_elevation) < NINETY_DEGREE))
      {
        exclude = 1;
      }
      break;
    }
    case 1: /* no negative elevation */
    {
      if (pstr_ls_geo->ls_elevation < ZERO_DEGREE)
      {
        exclude = 1;
      }
      break;
    }
    case 0: /* no positive elevation */
    {
      if (pstr_ls_geo->ls_elevation > ZERO_DEGREE)
      {
        exclude = 1;
      }
      break;
    }
    }
  }
  return exclude;
}

/**
 *  impeghd_mdp_dec_ei_init
 *
 *  \brief Metadata PreProcessing decoder init function
 *
 *  \param [in]  pstr_asc         Pointer to audio scene config structure
 *  \param [in,out] pstr_dec_data    Pointer to decoder data structure
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impeghd_mdp_dec_ei_init(ia_audio_specific_config_struct *pstr_asc,
                                     ia_dec_data_struct *pstr_dec_data)
{
  IA_ERRORCODE error = 0;
  WORD32 num_groups_with_exclusion = 0;
  UWORD32 i = 0;
  WORD32 j = 0, k = 0;
  WORD32 num_channels, num_lfes;
  ia_interaction_data_struct *pstr_intrct_data = &pstr_dec_data->str_interaction_config;
  ia_local_setup_struct *pstr_local_setup = &pstr_dec_data->str_local_setup_interaction;
  ia_enh_oam_config_struct *pstr_enh_oam_cfg = &pstr_asc->str_usac_config.enh_obj_md_cfg;
  ia_enh_obj_md_frame_str *pstr_enh_oam_frm = &pstr_dec_data->str_enh_obj_md_frame;
  ia_signals_3d *pstr_signals_3d = &pstr_asc->str_usac_config.signals_3d;
  ia_interface_speaker_config_3d *pstr_spk_config = &pstr_local_setup->spk_config;
  ia_cicp_ls_geo_str *pstr_ls_geo[CICP_MAX_NUM_LS];

  if (pstr_local_setup->ren_type == 0)
  {
    /* restriction of excluded sectors to speakerLayoutSignaling Type 0 and 1 */
    if (pstr_spk_config->spk_layout_type == 2)
    {
      pstr_local_setup->num_individ_setups = 1;
      pstr_local_setup->use_individ_grp_setups = 0;
      return 0;
    }
    else if (pstr_spk_config->spk_layout_type == 1)
    {
      for (j = 0; j < (WORD32)pstr_spk_config->num_speakers; j++)
      {
        WORD32 spk_cicp_idx = pstr_spk_config->cicp_spk_idx[j];
        pstr_ls_geo[j] = (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[spk_cicp_idx];
      }
    }
    else if (pstr_spk_config->cicp_spk_layout_idx == 0)
    {
      const WORD32 *channel_names = NULL;
      if (impeghd_cicpidx_2_ls_geometry(pstr_spk_config->cicp_spk_layout_idx,
                                        (const ia_cicp_ls_geo_str **)&pstr_ls_geo[0],
                                        &num_channels, &num_lfes, &channel_names))
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
      }
    }
  }
  if (pstr_enh_oam_cfg->has_common_group_excluded_sectors == 0 &&
      pstr_enh_oam_cfg->has_common_group_excluded_sectors == 1)
  {
    pstr_enh_oam_cfg->has_common_group_excluded_sectors = 1;
    for (i = 0; i < pstr_signals_3d->num_sig_group; i++)
    {
      if (pstr_signals_3d->group_type[i] > 0)
      {
        if (pstr_asc->str_usac_config.enh_obj_md_present)
        {
          if (pstr_enh_oam_frm->num_exclusion_sectors[i] > 0)
          {
            num_groups_with_exclusion++;
          }
        }
      }
    }
    /* define group-wise setups */
    if (num_groups_with_exclusion <= 0)
    {
      pstr_local_setup->num_individ_setups = 1;
      pstr_local_setup->use_individ_grp_setups = 0;
    }
    else
    {
      for (i = 0; i < pstr_signals_3d->num_sig_group; i++)
      {
        WORD32 num_excluded = 0;
        if ((pstr_signals_3d->group_type[i] == 1) &&
            (pstr_enh_oam_frm->num_exclusion_sectors[i] > 0))
        {
          WORD32 num_sectors = pstr_enh_oam_frm->num_exclusion_sectors[i];
          WORD32 to_be_excluded[MAE_MAX_NUM_SPEAKERS] = {0};
          pstr_intrct_data->str_ls_setup_intrct[i].spk_layout_type = 2;
          pstr_intrct_data->str_ls_setup_intrct[i].num_lfes = 0;
          pstr_intrct_data->str_ls_setup_intrct[i].num_speakers = 0;

          /* loop over all speakers */
          for (j = 0; j < (WORD32)pstr_spk_config->num_speakers; j++)
          {
            to_be_excluded[j] = 0;
            for (k = 0; k < num_sectors; k++)
            {
              if (to_be_excluded[j] == 0)
              {
                if (pstr_enh_oam_cfg->use_only_predefined_sectors[0] ||
                    pstr_enh_oam_frm->use_predefined_sector[0])
                {
                  WORD32 exc_sect_idx = pstr_enh_oam_frm->exclude_sector_index[0][k];
                  to_be_excluded[j] = impeghd_check_spk_exclusion_defined_sect(
                      pstr_ls_geo[j], pstr_local_setup, exc_sect_idx);
                }
                else
                {
                  if ((pstr_ls_geo[j]->ls_azimuth <=
                       pstr_enh_oam_frm->exclude_sector_max_az[0][k]) &&
                      (pstr_ls_geo[j]->ls_azimuth <=
                       pstr_enh_oam_frm->exclude_sector_max_az[0][k]) &&
                      (pstr_ls_geo[j]->ls_azimuth <=
                       pstr_enh_oam_frm->exclude_sector_max_az[0][k]) &&
                      (pstr_ls_geo[j]->ls_azimuth <=
                       pstr_enh_oam_frm->exclude_sector_max_az[0][k]))
                  {
                    to_be_excluded[j] = 1;
                  }
                  else
                  {
                    to_be_excluded[j] = 0;
                  }
                }
              }
            }
            if (to_be_excluded[j] == 1)
            {
              num_excluded++;
            }
            else
            {
              WORD32 spk_idx = pstr_intrct_data->str_ls_setup_intrct[i].num_speakers;
              pstr_intrct_data->str_ls_setup_intrct[i].num_speakers += 1;
              if (pstr_ls_geo[j]->lfe_flag)
              {
                pstr_intrct_data->str_ls_setup_intrct[i].num_lfes += 1;
              }
              pstr_intrct_data->str_ls_setup_intrct[i].str_flex_spk.azimuth[spk_idx] =
                  pstr_ls_geo[j]->ls_azimuth;
              pstr_intrct_data->str_ls_setup_intrct[i].str_flex_spk.elevation[spk_idx] =
                  pstr_ls_geo[j]->ls_elevation;
              pstr_intrct_data->str_ls_setup_intrct[i].has_knwn_pos[spk_idx] =
                  pstr_local_setup->has_knwn_pos[j];
              if (pstr_local_setup->has_knwn_pos[j])
              {
                pstr_intrct_data->str_ls_setup_intrct[i].loud_spk_azimuth[spk_idx] =
                    (FLOAT32)pstr_local_setup->loud_spk_azimuth[j];
                pstr_intrct_data->str_ls_setup_intrct[i].loud_spk_elevation[spk_idx] =
                    (FLOAT32)pstr_local_setup->loud_spk_elevation[j];
              }
            }
          }
        }
        else
        {
          /* standard setup */
          pstr_intrct_data->str_ls_setup_intrct[i].spk_layout_type =
              pstr_spk_config->spk_layout_type;
          pstr_intrct_data->str_ls_setup_intrct[i].num_speakers = pstr_spk_config->num_speakers;
          pstr_intrct_data->str_ls_setup_intrct[i].num_lfes = num_lfes;
          {
            /* standard setup */

            /* copy known positions for loudspeaker rendering (to be used for closest speaker
             * processing later on) */
            if (pstr_spk_config->spk_layout_type < 2)
            {
              for (j = 0; j < (WORD32)pstr_spk_config->num_speakers; j++)
              {
                pstr_intrct_data->str_ls_setup_intrct[i].has_knwn_pos[j] =
                    pstr_local_setup->has_knwn_pos[j];
                if (pstr_local_setup->has_knwn_pos[j] == 1)
                {
                  pstr_intrct_data->str_ls_setup_intrct[i].loud_spk_azimuth[j] =
                      (FLOAT32)pstr_local_setup->loud_spk_azimuth[j];
                  pstr_intrct_data->str_ls_setup_intrct[i].loud_spk_elevation[j] =
                      (FLOAT32)pstr_local_setup->loud_spk_elevation[j];
                }
              }
            }
          }
        }
      }
      pstr_local_setup->num_individ_setups =
          impeghd_mp_ls_indivi_setups(&pstr_intrct_data->str_ls_setup_intrct[0],
                                      pstr_asc->str_mae_asi.num_groups, pstr_enh_oam_cfg);
      if (pstr_local_setup->num_individ_setups > 1)
      {
        pstr_local_setup->use_individ_grp_setups = 1;
      }
      else
      {
        pstr_local_setup->use_individ_grp_setups = 0;
      }
    }
  }
  return error;
}

/**
 *  impeghd_md_proc_signal_group_type
 *
 *  \brief Finds signal group type.
 *
 *  \param [in]  pstr_asc            Pointer to audio scene config structure.
 *  \param [in]  pstr_signals_3d     Pointer to 3d signals depiction structure.
 *  \param [in]  grp_index           Group index value.
 *
 *  \return WORD32 Signal group type value.
 *
 */
WORD32 impeghd_md_proc_signal_group_type(ia_audio_specific_config_struct *pstr_asc,
                                         ia_signals_3d *pstr_signals_3d, WORD32 grp_index)
{
  WORD32 i, j;
  WORD32 sel_grp_idx = -1;
  WORD32 group_type = -1;
  WORD32 mae_id = -1;
  WORD32 start_id;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  start_id = pstr_mae_asi->group_definition[grp_index].start_id;
  /* new signaling, assuming all members from one signal group (and of equal type) */

  for (i = 0; i < (WORD32)pstr_signals_3d->num_sig_group; i++)
  {
    for (j = 0; j < (WORD32)pstr_signals_3d->num_sig[i]; j++)
    {
      mae_id++;
      if (mae_id == start_id)
      {
        sel_grp_idx = i;
        break;
      }
    }
  }

  if (sel_grp_idx != -1)
  {
    group_type = pstr_signals_3d->group_type[sel_grp_idx];
  }
  else
  {
    /* old signaling as fallback*/
    group_type = pstr_signals_3d->group_type[grp_index];
  }

  return group_type;
}

/**
 *  impeghd_md_proc_grp_idx
 *
 *  \brief Returns the group index whose group Id matches with input argument.
 *         Returns -1 if there is no match.
 *
 *  \param [in]  pstr_asc Pointer to audio scene config structure.
 *  \param [in]  group_id Group Id chosen.
 *
 *  \return WORD32 matching group index value.
 *
 */
static WORD32 impeghd_md_proc_grp_idx(ia_audio_specific_config_struct *pstr_asc, WORD32 group_id)
{
  WORD32 grp_idx = -1;
  WORD32 k;
  WORD32 group_count;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  group_count = pstr_mae_asi->num_groups;

  for (k = 0; k < group_count; k++)
  {
    if (pstr_mae_asi->group_definition[k].group_id == group_id)
    {
      grp_idx = k;
      break;
    }
  }
  return grp_idx;
}

/**
 *  impeghd_md_proc_is_switch_group_mem
 *
 *  \brief Identifies if switch group member.
 *
 *  \param [in] num_switch_groups    Number of switch roups.
 *  \param [out] pstr_switch_group_def Pointer to switch group definition structure
 *  \param [in] group_id                    Group id
 *  \param [in] switch_group_id             Pointer to switch group id
 *  \return WORD32
 *
 */
static WORD32 impeghd_md_proc_is_switch_group_mem(WORD32 num_switch_groups,
                                                  ia_mae_switch_group_def *pstr_switch_group_def,
                                                  WORD32 group_id, WORD32 *switch_group_id)
{
  WORD32 i, k;
  WORD32 is_member = 0;
  WORD32 sg_idx = 0;

  for (k = 0; k < num_switch_groups; k++)
  {
    WORD32 num_members = pstr_switch_group_def[k].group_num_members;
    for (i = 0; i < num_members; i++)
    {
      if (pstr_switch_group_def[k].member_id[i] == group_id)
      {
        sg_idx = k;
        is_member = 1;
      }
    }
  }
  if (is_member && (switch_group_id != NULL))
  {
    *switch_group_id = pstr_switch_group_def[sg_idx].group_id;
  }
  return is_member;
}

/**
 *  impeghd_md_proc_obj_idx
 *
 *  \brief Helper function that finds object index value.
 *
 *  \param [in]  pstr_signals_3d Pointer to 3d signals depiction structure.
 *  \param [in]  mae_id          Metadata audio element id.
 *
 *  \return WORD32 Object index value.
 *
 */
static WORD32 impeghd_md_proc_obj_idx(ia_signals_3d *pstr_signals_3d, WORD32 mae_id)
{
  WORD32 i, j;
  WORD32 obj_idx = -1;
  WORD32 ct = -1;
  for (i = 0; i < (WORD32)pstr_signals_3d->num_sig_group; i++)
  {
    for (j = 0; j < (WORD32)pstr_signals_3d->num_sig[i]; j++)
    {
      ct++;
      if (pstr_signals_3d->group_type[i] == 1)
      {
        obj_idx++;
      }
      if (ct == mae_id)
      {
        return obj_idx;
      }
    }
  }
  return obj_idx;
}

/**
 *  impeghd_get_oam_list
 *
 *  \brief Create OAM objects list based on metadata.
 *
 *  \param [in] pstr_interact_cfg Pointer to interaction data structure.
 *  \param [in] pstr_signals_3d   Pointer to 3d audio signals structure.
 *  \param [in] pstr_asc          Pointer to Audio specific configuration structure.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_get_oam_list(ia_interaction_data_struct *pstr_interact_cfg,
                                         ia_signals_3d *pstr_signals_3d,
                                         ia_audio_specific_config_struct *pstr_asc)
{
  WORD32 i, j, temp;
  WORD32 oam_count = 0;
  WORD32 *list_oam;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  list_oam = (WORD32 *)pstr_interact_cfg->list_oam;
  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
    {
      for (j = 0; j < pstr_mae_asi->group_definition[i].group_num_members; j++)
      {
        if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
        {
          list_oam[oam_count] = pstr_mae_asi->group_definition[i].metadata_ele_id[j];
        }
        else
        {
          list_oam[oam_count] = pstr_mae_asi->group_definition[i].start_id + j;
        }
        oam_count++;
      }
    }
  }
  // Sorting based on metadata ID values
  for (i = 0; i < oam_count; i++)
  {
    for (j = i + 1; j < oam_count; j++)
    {
      if (list_oam[i] > list_oam[j])
      {
        temp = list_oam[i];
        list_oam[i] = list_oam[j];
        list_oam[j] = temp;
      }
    }
  }
  pstr_interact_cfg->oam_count = oam_count;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_md_proc_switch_group_allow_on_off
 *
 *  \brief Returns the on off status of the group whose group Id matches with input argument.
 *         Returns -1 if there is no match.
 *
 *  \param [in]  pstr_asc        Pointer to audio scene config structure.
 *  \param [in]  switch_group_id Group Id chosen.
 *
 *  \return WORD32 On off condition of the group with the given switch_group_id.
 *
 */
static WORD32 impeghd_md_proc_switch_group_allow_on_off(ia_audio_specific_config_struct *pstr_asc,
                                                        WORD32 switch_group_id)
{
  WORD32 l;
  WORD32 cond = -1;
  WORD32 switch_group_index = -1;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  for (l = 0; l < pstr_mae_asi->num_switch_groups; l++)
  {
    if (pstr_mae_asi->switch_group_definition[l].group_id == switch_group_id)
    {
      switch_group_index = l;
      cond = pstr_mae_asi->switch_group_definition[switch_group_index].allow_on_off;
      break;
    }
  }
  return cond;
}

/**
 *  impeghd_md_proc_closest_spk_proc
 *
 *  \brief Apply closest speaker processing
 *
 *  \param [in]  pstr_interact_cfg   Pointer to interaction config data structure.
 *  \param [in]  pstr_asc            Pointer to audio scene config structure.
 *  \param [in]  pstr_signals_3d     Pointer to 3d signals depiction structure.
 *  \param [in]  pstr_local_setup    Pointer to local setup information structure.
 *  \param [in]  pstr_enh_oam_config Pointer to enhanced OAM config structure.
 *  \param [in]  pstr_oam_dec_state  Pointer to OAM state structure.
 *  \param [in]  ptr_scratch_mem     Pointer to scratch memory.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_proc_closest_spk_proc(
    ia_interaction_data_struct *pstr_interact_cfg, ia_audio_specific_config_struct *pstr_asc,
    ia_signals_3d *pstr_signals_3d, ia_local_setup_struct *pstr_local_setup,
    ia_enh_oam_config_struct *pstr_enh_oam_config, ia_oam_dec_state_struct *pstr_oam_dec_state,
    pVOID ptr_scratch_mem)
{
  WORD32 i, j = 0, k;
  WORD32 num_channels = 0, num_lfe = 0;
  WORD32 ct = 0;
  WORD8 *ptr_scratch = ptr_scratch_mem;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_interface_speaker_config_3d *pstr_el_spk_cfg =
      &pstr_interact_cfg->ptr_local_setup_info->spk_config;
  ia_sig_group_ls_setup_data *pstr_ls_setup_data = (ia_sig_group_ls_setup_data *)ptr_scratch;
  ptr_scratch += sizeof(*pstr_ls_setup_data);
  ia_interface_speaker_config_3d *pstr_el_spk_cfg_local =
      (ia_interface_speaker_config_3d *)ptr_scratch;
  ptr_scratch += sizeof(*pstr_el_spk_cfg_local);

  if (pstr_local_setup->use_individ_grp_setups == 0)
  {
    if (pstr_local_setup->ren_type != 0) /* Get virtual speaker setup for binaural rendering */
    {
      impeghd_get_geom_frm_spk_cfg(&pstr_local_setup->spk_config,
                                   &pstr_local_setup->spk_config.geometry[0],
                                   (WORD32 *)(&pstr_local_setup->spk_config.num_speakers),
                                   &pstr_local_setup->spk_config.num_lfes);
      num_lfe = pstr_local_setup->spk_config.num_lfes;
      num_channels = pstr_local_setup->spk_config.num_speakers - num_lfe;
      pstr_ls_setup_data->num_speakers = num_channels + num_lfe;
    }
    else
    {
      pstr_ls_setup_data->num_speakers = pstr_local_setup->spk_config.num_speakers;
      pstr_ls_setup_data->spk_layout_type = pstr_local_setup->spk_config.spk_layout_type;

      if (pstr_ls_setup_data->spk_layout_type != 0)
      {
        if (pstr_ls_setup_data->spk_layout_type == 2)
        {
          i = 0;
          pstr_ls_setup_data->str_flex_spk.angular_precision =
              pstr_local_setup->spk_config.str_flex_spk_data.angular_precision;
          while (i < (WORD32)pstr_ls_setup_data->num_speakers)
          {
            pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].is_cicp_spk_idx =
                pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                    .is_cicp_spk_idx;
            if (pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].is_cicp_spk_idx)
            {
              pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].cicp_spk_idx =
                  pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                      .cicp_spk_idx;
            }
            else
            {
              pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].el_class =
                  pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].el_class;
              if (pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].el_class == 3)
              {
                pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].el_angle_idx =
                    pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                        .el_angle_idx;
                pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].el_direction =
                    pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                        .el_direction;
              }

              pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].az_angle_idx =
                  pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                      .az_angle_idx;
              pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].az_direction =
                  pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                      .az_direction;
              pstr_ls_setup_data->str_flex_spk.str_flex_spk_descr[i].is_lfe =
                  pstr_local_setup->spk_config.str_flex_spk_data.str_flex_spk_descr[i].is_lfe;
            }
            i++;
          }
        }
        else if (pstr_ls_setup_data->spk_layout_type == 1)
        {
          for (i = 0; i < (WORD32)pstr_ls_setup_data->num_speakers; i++)
          {
            pstr_ls_setup_data->cicp_spk_idx[i] = pstr_local_setup->spk_config.cicp_spk_idx[i];
          }
        }
      }
      else if (pstr_ls_setup_data->spk_layout_type < 3)
      {
        pstr_ls_setup_data->cicp_spk_layout_idx =
            pstr_local_setup->spk_config.cicp_spk_layout_idx;
      }
      impeghd_get_geom_frm_spk_cfg(&pstr_local_setup->spk_config,
                                   &pstr_local_setup->spk_config.geometry[0],
                                   (WORD32 *)(&pstr_local_setup->spk_config.num_speakers),
                                   &pstr_local_setup->spk_config.num_lfes);
      num_lfe = pstr_local_setup->spk_config.num_lfes;
      num_channels = pstr_local_setup->spk_config.num_speakers - num_lfe;
      pstr_ls_setup_data->num_speakers = num_channels + num_lfe;

      /* include known positions for speaker layout type 0 and 1 */
      if (pstr_ls_setup_data->spk_layout_type < 2)
      {
        for (i = 0; i < pstr_ls_setup_data->num_speakers; i++)
        {
          if (pstr_local_setup->has_knwn_pos[i] == 1)
          {
            pstr_ls_setup_data->str_flex_spk.azimuth[i] =
                (FLOAT32)pstr_local_setup->loud_spk_azimuth[i];
            pstr_ls_setup_data->str_flex_spk.elevation[i] =
                (FLOAT32)pstr_local_setup->loud_spk_elevation[i];
          }
        }
      }
    }
  }
  /********************* PROCESSING ************************/
  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    WORD32 has_divergence = 0;

    if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
    {
      WORD32 *idx_min = NULL;
      WORD32 num_members = 0;
      (void)idx_min;
      if (pstr_interact_cfg->divergence_asi_modified == 1)
      {
        num_members = pstr_mae_asi->group_definition[i].group_num_members;
      }
      else
      {
        num_members = pstr_interact_cfg->orig_num_grp_members[i];
      }

      idx_min = (WORD32 *)(ptr_scratch);
      ptr_scratch += (num_members * sizeof(*idx_min));

      for (k = 0; k < num_members; k++)
      {
        has_divergence = pstr_enh_oam_config->has_divergence[0];

        if (has_divergence == 0) /* only process if object has no divergence */
        {
          WORD32 m;
          WORD32 first = 0;
          WORD32 member_id = -1;
          WORD32 oam_index = -1;
          FLOAT32 loud_spk_diff = 0.0f;
          if (pstr_local_setup->use_individ_grp_setups == 1) /* copy individual setup */
          {
            memset(pstr_el_spk_cfg_local, 0, sizeof(*pstr_el_spk_cfg));
            memcpy(pstr_el_spk_cfg_local, pstr_el_spk_cfg, sizeof(*pstr_el_spk_cfg_local));

            /* include known positions */
            if (pstr_local_setup->ren_type == 0)
            {
              for (m = 0; m < (WORD32)pstr_el_spk_cfg_local->num_speakers; m++)
              {
                if (pstr_local_setup->has_knwn_pos[m] == 1)
                {
                  pstr_el_spk_cfg_local->str_flex_spk_data.elevation[j] =
                      pstr_local_setup->loud_spk_azimuth[m];
                  pstr_el_spk_cfg_local->str_flex_spk_data.azimuth[j] =
                      pstr_local_setup->loud_spk_azimuth[m];
                }
              }
            }
          }
          if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
          {
            member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
          }
          else
          {
            member_id = pstr_mae_asi->group_definition[i].start_id + k;
          }

          /* get OAM data for the current group member */
          for (j = 0; j < pstr_interact_cfg->oam_count; j++)
          {
            if (pstr_interact_cfg->list_oam[j] == member_id)
            {
              oam_index = j;
              break;
            }
          }
          if (oam_index == -1)
          {
            return IA_MPEGH_MAE_EXE_FATAL_INVALID_OAM_INDEX;
          }
          pstr_interact_cfg->az_modified[k] = pstr_oam_dec_state->azimuth_descaled[oam_index];
          pstr_interact_cfg->ele_modified[k] = pstr_oam_dec_state->elevation_descaled[oam_index];

          for (j = 0; j < (WORD32)pstr_el_spk_cfg_local->num_speakers; j++)
          {
            if (pstr_el_spk_cfg_local->str_flex_spk_data.str_flex_spk_descr[j].is_lfe == 0)
            {
              WORD32 condition = 0;
              FLOAT32 temp_diff = ZERO_DEGREE;
              FLOAT32 az_this = pstr_el_spk_cfg_local->str_flex_spk_data.azimuth[j];
              FLOAT32 el_this = pstr_el_spk_cfg_local->str_flex_spk_data.elevation[j];
              FLOAT32 min_el1 = ZERO_DEGREE, max_el1 = ZERO_DEGREE, min_el2 = ZERO_DEGREE,
                      max_el2 = ZERO_DEGREE, min_az1 = ZERO_DEGREE, max_az1 = ZERO_DEGREE,
                      min_az2 = ZERO_DEGREE, max_az2 = ZERO_DEGREE, min_az3 = ZERO_DEGREE,
                      max_az3 = ZERO_DEGREE;
              FLOAT32 min_az = ZERO_DEGREE, min_el = ZERO_DEGREE, max_az = ZERO_DEGREE,
                      max_el = ZERO_DEGREE;

              if (pstr_enh_oam_config->has_closest_speaker_condition != 1)
              {
                min_az = -ONE_EIGHTY_DEGREE;
                max_az = ONE_EIGHTY_DEGREE;
                min_el = -NINETY_DEGREE;
                max_el = NINETY_DEGREE;
              }
              else
              {
                min_az = pstr_interact_cfg->az_modified[k] -
                         pstr_enh_oam_config->closest_spk_thr_angle;
                max_az = pstr_interact_cfg->az_modified[k] +
                         pstr_enh_oam_config->closest_spk_thr_angle;
                min_el = pstr_interact_cfg->ele_modified[k] -
                         pstr_enh_oam_config->closest_spk_thr_angle;
                max_el = pstr_interact_cfg->ele_modified[k] +
                         pstr_enh_oam_config->closest_spk_thr_angle;
              }

              if (((max_az > ONE_EIGHTY_DEGREE) || (min_az < -ONE_EIGHTY_DEGREE)) &&
                  ((min_el < -NINETY_DEGREE) && (max_el > NINETY_DEGREE)))
              {
                if (min_az > ONE_EIGHTY_DEGREE)
                {
                  min_az1 = min_az;
                  max_az1 = ONE_EIGHTY_DEGREE;
                  min_az2 = -ONE_EIGHTY_DEGREE;
                  max_az2 = -ONE_EIGHTY_DEGREE + (max_az - ONE_EIGHTY_DEGREE);
                }
                else
                {
                  min_az1 = -ONE_EIGHTY_DEGREE;
                  max_az1 = max_az;
                  min_az2 = ONE_EIGHTY_DEGREE - (min_az + ONE_EIGHTY_DEGREE);
                  max_az2 = ONE_EIGHTY_DEGREE;
                }
                if (max_el <= NINETY_DEGREE)
                {
                  min_el1 = -NINETY_DEGREE;
                  max_el1 = max_el;
                  min_el2 = -NINETY_DEGREE;
                  max_el2 = NINETY_DEGREE + ia_fabs_flt(NINETY_DEGREE + min_el);
                  min_az3 = min_az + ONE_EIGHTY_DEGREE;
                  max_az3 = max_az + ONE_EIGHTY_DEGREE;
                }
                else
                {
                  min_el1 = min_el;
                  max_el1 = NINETY_DEGREE;
                  min_el2 = NINETY_DEGREE - ia_fabs_flt(NINETY_DEGREE - max_el);
                  max_el2 = NINETY_DEGREE;
                  min_az3 = min_az + ONE_EIGHTY_DEGREE;
                  max_az3 = max_az + ONE_EIGHTY_DEGREE;
                }
                condition = ((((az_this >= min_az1) && (az_this <= max_az1)) &&
                              ((el_this >= min_el1) && (el_this <= max_el1))) ||
                             (((az_this >= min_az2) && (az_this <= max_az2)) &&
                              ((el_this >= min_el1) && (el_this <= max_el1))) ||
                             (((az_this >= min_az3) && (az_this <= max_az3)) &&
                              ((el_this >= min_el2) && (el_this <= max_el2))));
              }
              else if (((max_el > NINETY_DEGREE) || (min_el < -NINETY_DEGREE)) &&
                       ((min_az >= -ONE_EIGHTY_DEGREE) && (max_az <= ONE_EIGHTY_DEGREE)))
              {
                if (max_el <= NINETY_DEGREE)
                {
                  min_el1 = -NINETY_DEGREE;
                  max_el1 = max_el;
                  min_el2 = -NINETY_DEGREE;
                  max_el2 = NINETY_DEGREE + ia_fabs_flt(NINETY_DEGREE + min_el);
                  min_az2 = min_az + ONE_EIGHTY_DEGREE;
                  max_az2 = max_az + ONE_EIGHTY_DEGREE;
                }
                else
                {

                  min_el1 = min_el;
                  max_el1 = NINETY_DEGREE;
                  min_el2 = NINETY_DEGREE - ia_fabs_flt(NINETY_DEGREE - max_el);
                  max_el2 = NINETY_DEGREE;
                  min_az2 = min_az + ONE_EIGHTY_DEGREE;
                  max_az2 = max_az + ONE_EIGHTY_DEGREE;
                }
                condition = ((((az_this >= min_az) && (az_this <= max_az)) &&
                              ((el_this >= min_el1) && (el_this <= max_el1))) ||
                             (((az_this >= min_az2) && (az_this <= max_az2)) &&
                              ((el_this >= min_el2) && (el_this <= max_el2))));
              }
              else if (((max_az > ONE_EIGHTY_DEGREE) || (min_az < -ONE_EIGHTY_DEGREE)) &&
                       ((min_el >= -NINETY_DEGREE) && (max_el <= NINETY_DEGREE)))
              {
                if (min_az >= ONE_EIGHTY_DEGREE)
                {
                  min_az1 = min_az;
                  max_az1 = ONE_EIGHTY_DEGREE;
                  min_az2 = -ONE_EIGHTY_DEGREE;
                  max_az2 = -ONE_EIGHTY_DEGREE + (max_az - ONE_EIGHTY_DEGREE);
                }
                else
                {
                  min_az1 = -ONE_EIGHTY_DEGREE;
                  max_az1 = max_az;
                  min_az2 = ONE_EIGHTY_DEGREE - (min_az + ONE_EIGHTY_DEGREE);
                  max_az2 = ONE_EIGHTY_DEGREE;
                }
                condition = ((((az_this >= min_az1) && (az_this <= max_az1)) &&
                              ((el_this >= min_el) && (el_this <= max_el))) ||
                             (((az_this >= min_az2) && (az_this <= max_az2)) &&
                              ((el_this >= min_el) && (el_this <= max_el))));
              }

              else
              {
                condition = (((az_this >= min_az) && (az_this <= max_az)) &&
                             ((el_this >= min_el) && (el_this <= max_el)));
              }

              if (condition == 1)
              {

                temp_diff = (ia_fabs_flt(el_this - pstr_interact_cfg->ele_modified[k]) +
                             ia_fabs_flt(az_this - pstr_interact_cfg->az_modified[k]));

                if (first == 1 || first == 0)
                {
                  loud_spk_diff = temp_diff;
                  idx_min[k] = j;
                  first = -1;
                }
                else
                {
                  /* Test for "smaller", not "smaller or equal" --> if there are several speakers
                   * with the same minimum distance, */
                  /* the first in the SpeakerConfig3D structure shall be taken as the closest
                   * speaker */
                  if (temp_diff < loud_spk_diff)
                  {
                    loud_spk_diff = temp_diff;
                    idx_min[k] = j;
                  }
                }
              }
            }
          }
          if (first != 0)
          {
            pstr_interact_cfg->ele_modified[k] =
                pstr_el_spk_cfg_local->str_flex_spk_data.elevation[idx_min[k]];
            pstr_interact_cfg->az_modified[k] =
                pstr_el_spk_cfg_local->str_flex_spk_data.azimuth[idx_min[k]];
            pstr_oam_dec_state->azimuth_descaled[oam_index] = pstr_interact_cfg->az_modified[k];
            pstr_oam_dec_state->elevation_descaled[oam_index] =
                pstr_interact_cfg->ele_modified[k];
          }
        }

        ct++;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_md_proc_apply_pos_interactivity
 *
 *  \brief Apply prosition interactivity.
 *
 *  \param [in]  pstr_interact_cfg   Pointer to interaction config data structure.
 *  \param [in]  pstr_asc            Pointer to audio scene config structure.
 *  \param [in,out] pstr_dec_data       Pointer to decoder data structure.
 *  \param [in]  pstr_signals_3d     Pointer to 3d signals depiction structure.
 *  \param [in]  ptr_scratch_mem     Pointer to scratch memory.
 *  \param [in]  downmix_id          Downmix id.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_proc_apply_pos_interactivity(
    ia_interaction_data_struct *pstr_interact_cfg, ia_audio_specific_config_struct *pstr_asc,
    ia_dec_data_struct *pstr_dec_data, ia_signals_3d *pstr_signals_3d, pWORD8 ptr_scratch_mem,
    WORD32 downmix_id)
{
  IA_ERRORCODE error = 0;
  WORD32 i, k, j;
  WORD32 e;
  WORD32 list_switch_grp_cond[MAE_MAX_NUM_GROUPS][5];
  WORD32 chosen_preset = -1;
  WORD32 ext_index = -1;
  WORD32 group_has_cond = -1;
  WORD32 preset_index = -1;
  WORD8 *ptr_scratch = ptr_scratch_mem;
  (void)ptr_scratch;
  ia_ele_intrctn *pstr_ele_intrctn = pstr_interact_cfg->ptr_ele_interaction_data;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    group_has_cond = -1;
    list_switch_grp_cond[i][0] = 0;
    list_switch_grp_cond[i][1] = -1;
    list_switch_grp_cond[i][2] = -1;
    list_switch_grp_cond[i][3] = -1;

    if (pstr_interact_cfg->process[i])
    {
      if (pstr_ele_intrctn->route_to_wire_id[i] == -1)
      {
        if (pstr_mae_asi->group_definition[i].allow_pos_interact)
        {
          WORD32 signal_group_type =
              impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i);
          if (signal_group_type == 1) /* at the moment not possible for SAOC */
          {
            WORD32 apply_preset_dependent_vals = 0;
            WORD32 apply = 0;
            if (pstr_interact_cfg->interaction_type != 0)
            {
              chosen_preset = pstr_interact_cfg->ptr_ele_interaction_data->ei_group_preset_id;
              for (k = 0; k < pstr_mae_asi->num_group_presets; k++)
              {
                if (pstr_mae_asi->group_presets_definition[k].group_id == chosen_preset)
                {
                  preset_index = k;
                  if (pstr_mae_asi->group_presets_ext_definition
                          .num_downmix_id_group_preset_extensions[k] > 0)
                  {
                    for (e = 0; e < pstr_mae_asi->group_presets_ext_definition
                                        .num_downmix_id_group_preset_extensions[k];
                         e++)
                    {
                      if (pstr_mae_asi->group_presets_ext_definition
                              .group_preset_downmix_id[k][e] == downmix_id)
                      {
                        ext_index = e;
                        break;
                      }
                    }
                  }
                  group_has_cond = -1;

                  if (ext_index <= -1)
                  {
                    for (j = 0; j < pstr_mae_asi->group_presets_definition[k].num_conditions; j++)
                    {
                      if (pstr_mae_asi->group_definition[i].group_id ==
                          pstr_mae_asi->group_presets_definition[k].reference_id[j])
                      {
                        group_has_cond = j;
                        if (pstr_mae_asi->group_presets_definition[k]
                                .disable_pos_interact[group_has_cond] == 0)
                        {
                          apply = 1;
                        }
                        else
                        {
                          /* do not apply user position interaction, but "offset" values defined
                           * by preset */
                          apply = 1;
                          apply_preset_dependent_vals = 1;
                        }
                      }
                    }

                    /* if group has no condition, but is part of a SG with SGC, then the group has
                     * to be processed nevertheless with the values from the preset */
                    if (group_has_cond == -1)
                    {
                      WORD32 is_sg_member_tmp = -1;
                      WORD32 sg_id_tmp = -1;
                      WORD32 run_to = 0;
                      WORD32 r = 0;
                      WORD32 has_sg_cond = 0;

                      /* check if group is member of a switchGroup */
                      is_sg_member_tmp = impeghd_md_proc_is_switch_group_mem(
                          pstr_mae_asi->num_switch_groups, pstr_mae_asi->switch_group_definition,
                          pstr_mae_asi->group_definition[i].group_id, &sg_id_tmp);
                      if (is_sg_member_tmp)
                      {
                        run_to =
                            pstr_mae_asi->group_presets_definition[preset_index].num_conditions;
                        for (
                            r = 0; r < run_to;
                            r++) /* run over all conditions of chosen preset
                                                                                                                                    (preset extension) */
                        {
                          WORD32 sg_id = -1;

                          WORD32 is_sg_cond = pstr_mae_asi->group_presets_ext_definition
                                                  .is_switch_group_condition[preset_index][r];
                          if (is_sg_cond == 1)
                          {
                            sg_id = pstr_mae_asi->group_presets_definition[preset_index]
                                        .reference_id[r];
                          }

                          if (sg_id_tmp == sg_id)
                          {
                            has_sg_cond = 1;
                            list_switch_grp_cond[i][0] = 1;     /* has switch group condition */
                            list_switch_grp_cond[i][2] = sg_id; /* switch group id */
                            list_switch_grp_cond[i][3] = r;     /* condition index */
                            break;
                          }
                        }
                        if (has_sg_cond)
                        {
                          /* do apply "offset" values defined by preset */
                          apply = 1;
                          if (pstr_mae_asi->group_presets_definition[preset_index]
                                  .disable_pos_interact[r] == 1)
                          {
                            apply_preset_dependent_vals = 1;
                          }
                        }
                      }
                    }
                  }
                  else
                  {
                    for (j = 0; j < pstr_mae_asi->group_presets_ext_definition
                                        .group_preset_num_conditions[k][ext_index];
                         j++)
                    {
                      if ((pstr_mae_asi->group_definition[i].group_id ==
                           pstr_mae_asi->group_presets_ext_definition
                               .group_preset_group_id[k][ext_index][j]) &&
                          (pstr_mae_asi->group_presets_ext_definition
                               .downmix_id_is_switch_group_condition[k][ext_index][j] == 0))
                      {
                        group_has_cond = j;
                        if (pstr_mae_asi->group_presets_ext_definition
                                .group_preset_disable_pos_interactiviy[k][group_has_cond][j] == 0)
                        {
                          apply = 1;
                        }
                        else
                        {
                          /* do not apply user position interaction, but "offset" values defined
                           * by preset */
                          apply = 1;
                          apply_preset_dependent_vals = 1;
                        }
                      }
                    }
                    /* if group has no condition, but is part of a SG with SGC, then the group has
                     * to be processed nevertheless with the values from the preset */
                    if (group_has_cond == -1)
                    {
                      WORD32 is_sg_member_tmp = -1;
                      WORD32 sg_id_tmp = -1;
                      WORD32 run_to = 0;
                      WORD32 r = 0;
                      WORD32 has_sg_cond = 0;

                      /* check if group is member of a switchGroup */
                      is_sg_member_tmp = impeghd_md_proc_is_switch_group_mem(
                          pstr_mae_asi->num_switch_groups, pstr_mae_asi->switch_group_definition,
                          pstr_mae_asi->group_definition[i].group_id, &sg_id_tmp);
                      if (is_sg_member_tmp)
                      {
                        /* if yes -> check if a condition exists for this switchGroup */
                        if (ext_index > -1)
                        {
                          run_to = pstr_mae_asi->group_presets_ext_definition
                                       .group_preset_num_conditions[preset_index][ext_index];
                        }
                        else
                        {
                          run_to =
                              pstr_mae_asi->group_presets_definition[preset_index].num_conditions;
                        }
                        for (
                            r = 0; r < run_to;
                            r++) /* run over all conditions of chosen preset
                                                                                                                                    (preset extension) */
                        {
                          WORD32 sg_id = -1;
                          if (ext_index > -1)
                          {
                            WORD32 is_sg_cond =
                                pstr_mae_asi->group_presets_ext_definition
                                    .downmix_id_is_switch_group_condition[preset_index][ext_index]
                                                                         [r];
                            if (is_sg_cond == 1)
                            {
                              sg_id = pstr_mae_asi->group_presets_ext_definition
                                          .group_preset_group_id[preset_index][ext_index][r];
                            }
                          }
                          else
                          {
                            WORD32 is_sg_cond = pstr_mae_asi->group_presets_ext_definition
                                                    .is_switch_group_condition[preset_index][r];
                            if (is_sg_cond == 1)
                            {
                              sg_id = pstr_mae_asi->group_presets_definition[preset_index]
                                          .reference_id[r];
                            }
                          }
                          if (sg_id_tmp == sg_id)
                          {
                            has_sg_cond = 1;
                            list_switch_grp_cond[i][0] = 1;     /* has switch group condition */
                            list_switch_grp_cond[i][2] = sg_id; /* switch group id */
                            list_switch_grp_cond[i][3] = r;     /* condition index */
                            list_switch_grp_cond[i][4] = ext_index; /* extension index */
                            break;
                          }
                        }
                        if (has_sg_cond)
                        {
                          /* do apply "offset" values defined by preset */
                          apply = 1;
                          if (pstr_mae_asi->group_presets_ext_definition
                                  .group_preset_disable_pos_interactiviy[preset_index][ext_index]
                                                                        [r] == 1)
                          {
                            apply_preset_dependent_vals = 1;
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
            else
            {
              apply = 1;
            }
            if (apply == 1)
            {
              WORD32 num_members = pstr_mae_asi->group_definition[i].group_num_members;
              WORD32 member_id = -1;
              WORD32 oam_index = -1;
              FLOAT32 el_mod;
              FLOAT32 az_mod;
              FLOAT32 dist_mod;

              for (k = 0; k < num_members; k++)
              {
                if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
                {
                  member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
                }
                else
                {
                  member_id = pstr_mae_asi->group_definition[i].start_id + k;
                }

                /* get OAM data for the current group member */
                for (j = 0; j < pstr_interact_cfg->oam_count; j++)
                {
                  if (pstr_interact_cfg->list_oam[j] == member_id)
                  {
                    oam_index = j;
                  }
                }
                if (oam_index == -1)
                {
                  return IA_MPEGH_MAE_EXE_FATAL_INVALID_OAM_INDEX;
                }
                /* copy unmodified values to interactConfig struct */
                pstr_interact_cfg->dist_modified[k] =
                    pstr_oam_dec_state->radius_descaled[oam_index];
                pstr_interact_cfg->ele_modified[k] =
                    pstr_oam_dec_state->elevation_descaled[oam_index];
                pstr_interact_cfg->az_modified[k] =
                    pstr_oam_dec_state->azimuth_descaled[oam_index];

                /* copy modification values to variables */
                if (apply_preset_dependent_vals != 0)
                {
                  /* do not copy user interaction */
                  el_mod = ZERO_DEGREE;
                  az_mod = ZERO_DEGREE;
                  dist_mod = 1.0f;
                }
                else
                {
                  /* copy user interaction */
                  el_mod = pstr_interact_cfg->ptr_ele_interaction_data->ei_el_offset[i];
                  az_mod = pstr_interact_cfg->ptr_ele_interaction_data->ei_az_offset[i];
                  dist_mod = pstr_interact_cfg->ptr_ele_interaction_data->ei_dist_fact[i];
                }

                if (pstr_interact_cfg->interaction_type == 1)
                {
                  if (group_has_cond <=
                      -1) /* group has no condition, but is part of a switch group with a switch
         group condition */
                  {
                    if (list_switch_grp_cond[i][0] == 1)
                    {
                      if (list_switch_grp_cond[i][4] <= -1)
                      {
                        el_mod = ia_add_flt(el_mod,
                                            pstr_mae_asi->group_presets_definition[preset_index]
                                                .elevation_offset[list_switch_grp_cond[i][3]]);
                        az_mod = ia_add_flt(az_mod,
                                            pstr_mae_asi->group_presets_definition[preset_index]
                                                .azimuth_offset[list_switch_grp_cond[i][3]]);
                        dist_mod = ia_mul_flt(dist_mod,
                                              pstr_mae_asi->group_presets_definition[preset_index]
                                                  .dist_factor[list_switch_grp_cond[i][3]]);
                      }
                      else
                      {
                        el_mod = ia_add_flt(
                            el_mod,
                            (FLOAT32)pstr_mae_asi->group_presets_ext_definition
                                .group_preset_el_offset[preset_index][list_switch_grp_cond[i][4]]
                                                       [list_switch_grp_cond[i][3]]);
                        az_mod = ia_add_flt(
                            az_mod,
                            (FLOAT32)pstr_mae_asi->group_presets_ext_definition
                                .group_preset_az_offset[preset_index][list_switch_grp_cond[i][4]]
                                                       [list_switch_grp_cond[i][3]]);
                        dist_mod = ia_mul_flt(
                            dist_mod,
                            (FLOAT32)pstr_mae_asi->group_presets_ext_definition
                                .group_preset_dist_fac[preset_index][list_switch_grp_cond[i][4]]
                                                      [list_switch_grp_cond[i][3]]);
                      }
                    }
                  }
                  else
                  {
                    if (pstr_mae_asi->group_presets_ext_definition
                            .group_preset_group_id[preset_index][ext_index][group_has_cond] == 1)
                    {
                      /* add preset extension modification values */
                      el_mod = ia_add_flt(
                          el_mod,
                          (FLOAT32)pstr_mae_asi->group_presets_ext_definition
                              .group_preset_el_offset[preset_index][ext_index][group_has_cond]);
                      az_mod = ia_add_flt(
                          az_mod,
                          (FLOAT32)pstr_mae_asi->group_presets_ext_definition
                              .group_preset_az_offset[preset_index][ext_index][group_has_cond]);
                      dist_mod = ia_mul_flt(
                          dist_mod,
                          (FLOAT32)pstr_mae_asi->group_presets_ext_definition
                              .group_preset_dist_fac[preset_index][ext_index][group_has_cond]);
                    }
                  }
                }

                /* check for modification ranges */
                if (ia_lt_flt(dist_mod, pstr_mae_asi->group_definition[i].min_dist_factor))
                {
                  error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
                  /* Warning: Violation of interactivity ranges. */
                  dist_mod = (FLOAT32)pstr_mae_asi->group_definition[i].min_dist_factor;
                }
                if (ia_lt_flt(pstr_mae_asi->group_definition[i].max_dist_factor, dist_mod))
                {
                  error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
                  /* Warning: Violation of interactivity ranges. */
                  dist_mod = (FLOAT32)pstr_mae_asi->group_definition[i].max_dist_factor;
                }
                if (ia_lt_flt(el_mod, pstr_mae_asi->group_definition[i].min_el_offset))
                {
                  error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
                  /* Warning: Violation of interactivity ranges. */
                  el_mod = (FLOAT32)pstr_mae_asi->group_definition[i].min_el_offset;
                }
                if (ia_lt_flt(pstr_mae_asi->group_definition[i].max_el_offset, el_mod))
                {
                  error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
                  /* Warning: Violation of interactivity ranges. */
                  el_mod = (FLOAT32)pstr_mae_asi->group_definition[i].max_el_offset;
                }
                if (ia_lt_flt(az_mod, (FLOAT32)pstr_mae_asi->group_definition[i].min_az_offset))
                {
                  error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
                  /* Warning: Violation of interactivity ranges. */
                  az_mod = (FLOAT32)pstr_mae_asi->group_definition[i].min_az_offset;
                }
                if (ia_lt_flt(pstr_mae_asi->group_definition[i].max_az_offset, az_mod))
                {
                  error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
                  /* Warning: Violation of interactivity ranges. */
                  az_mod = (FLOAT32)pstr_mae_asi->group_definition[i].min_az_offset;
                }

                /* apply modification */
                if (!ia_eq_flt(dist_mod, 1.0f))
                {
                  pstr_interact_cfg->dist_modified[k] =
                      ia_mul_flt(pstr_interact_cfg->dist_modified[k], dist_mod);
                }
                if (!ia_eq_flt(el_mod, ZERO_DEGREE))
                {
                  pstr_interact_cfg->ele_modified[k] =
                      ia_add_flt(pstr_interact_cfg->ele_modified[k], el_mod);
                }
                if (!ia_eq_flt(az_mod, ZERO_DEGREE))
                {
                  pstr_interact_cfg->az_modified[k] =
                      ia_add_flt(pstr_interact_cfg->az_modified[k], az_mod);
                }

                /* copy modified values to modifies OAM sample */
                pstr_oam_dec_state->radius_descaled[oam_index] =
                    pstr_interact_cfg->dist_modified[k];
                pstr_oam_dec_state->elevation_descaled[oam_index] =
                    pstr_interact_cfg->ele_modified[k];
                pstr_oam_dec_state->azimuth_descaled[oam_index] =
                    pstr_interact_cfg->az_modified[k];
              }
            }
          }
        }
      }
    }
  }
  return error;
}

/**
 *  impeghd_md_proc_modified_gains
 *
 *  \brief Apply modified gains.
 *
 *  \param [in]  pstr_interact_cfg   Pointer to interaction config data structure.
 *  \param [in]  pstr_asc            Pointer to audio scene config structure.
 *  \param [in]  pstr_signals_3d     Pointer to 3d signals depiction structure.
 *  \param [in]  ptr_scratch_mem     Pointer to scratch memory.
 *  \param [in]  is_oam_decoded      Flag that indicates if oam data id decoded.
 *  \param [in]  downmix_id          Downmix id.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_proc_modified_gains(ia_interaction_data_struct *pstr_interact_cfg,
                                                   ia_audio_specific_config_struct *pstr_asc,
                                                   ia_signals_3d *pstr_signals_3d,
                                                   ia_dec_data_struct *pstr_dec_data,
                                                   pWORD8 ptr_scratch_mem, WORD32 is_oam_decoded,
                                                   WORD32 downmix_id)
{
  IA_ERRORCODE error = 0;
  WORD32 i, k, j;
  WORD32 e;
  WORD32 ext_index = -1;
  WORD32 chosen_preset = -1;
  WORD32 group_has_cond = -1;
  WORD32 preset_index = -1;
  pWORD8 ptr_scratch = ptr_scratch_mem;
  (void)ptr_scratch;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    if (!(pstr_interact_cfg->process[i]))
    {
      WORD32 member_id = -1;
      WORD32 oam_index = -1;
      WORD32 num_members = pstr_mae_asi->group_definition[i].group_num_members;

      if (!((impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1) &&
            (is_oam_decoded == 1)))
      {
        pstr_interact_cfg->gain_modified_grp[i] = 0;
      }
      else
      {
        pstr_interact_cfg->gain_modified_grp[i] = -1.0f; /* object-based group */
        for (k = 0; k < num_members; k++)
        {
          if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
          {
            member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
          }
          else
          {
            member_id = pstr_mae_asi->group_definition[i].start_id + k;
          }

          /* get OAM data for the current group member */
          j = 0;
          {
            if (pstr_interact_cfg->list_oam[j] == member_id)
            {
              oam_index = j;
            }
            if (oam_index == -1)
            {
              return IA_MPEGH_MAE_EXE_FATAL_INVALID_OAM_INDEX;
            }
            pstr_oam_dec_state->gain_descaled[oam_index] = 0;
          }
        }
      }
    }
    else
    {
      if (pstr_mae_asi->group_definition[i].allow_gain_interact)
      {
        FLOAT32 intrct_gain_db = 0.0f;
        WORD32 apply_preset_dependent_vals = 0;

        if (pstr_interact_cfg->interaction_type == 1)
        {
          chosen_preset = pstr_interact_cfg->ptr_ele_interaction_data->ei_group_preset_id;

          for (k = 0; k < pstr_mae_asi->num_group_presets; k++)
          {
            if (pstr_mae_asi->group_presets_definition[k].group_id == chosen_preset)
            {
              preset_index = k;
              group_has_cond = -1;

              if (pstr_mae_asi->group_presets_ext_definition
                      .num_downmix_id_group_preset_extensions[k] > 0)
              {
                for (e = 0; e < pstr_mae_asi->group_presets_ext_definition
                                    .num_downmix_id_group_preset_extensions[k];
                     e++)
                {
                  if (pstr_mae_asi->group_presets_ext_definition.group_preset_downmix_id[k][e] ==
                      downmix_id)
                  {
                    ext_index = e;
                    break;
                  }
                }
              }

              if (ext_index > -1)
              {
                for (j = 0; j < pstr_mae_asi->group_presets_ext_definition
                                    .group_preset_num_conditions[k][ext_index];
                     j++)
                {
                  if ((pstr_mae_asi->group_definition[i].group_id ==
                       pstr_mae_asi->group_presets_ext_definition
                           .group_preset_downmix_id[ext_index][j]) &&
                      (pstr_mae_asi->group_presets_ext_definition
                           .downmix_id_is_switch_group_condition[k][ext_index][j] == 0))
                  {
                    group_has_cond = j;
                    if (pstr_mae_asi->group_presets_ext_definition
                            .group_preset_disable_gain_interactivity[k][ext_index][j] == 1)
                    {
                      apply_preset_dependent_vals = 1;
                    }
                    if (pstr_mae_asi->group_presets_ext_definition
                            .group_preset_gain_flag[k][ext_index][j] == 1)
                    {
                      WORD32 gain = (WORD32)pstr_mae_asi->group_presets_ext_definition
                                        .group_preset_gain[k][ext_index][group_has_cond];
                      FLOAT32 preset_gain = (gain - 255) * 0.5f + 32.0f;
                      intrct_gain_db = ia_add_flt(intrct_gain_db, preset_gain);
                    }
                    break;
                  }
                }
                if (group_has_cond == -1)
                {
                  WORD32 is_switch_grp_mem = -1;
                  WORD32 sg_id_tmp = -1;
                  WORD32 run_to = 0;
                  WORD32 r = 0;
                  WORD32 has_sg_cond = 0;

                  /* check if group is member of a switchGroup */
                  is_switch_grp_mem = impeghd_md_proc_is_switch_group_mem(
                      pstr_mae_asi->num_switch_groups, pstr_mae_asi->switch_group_definition,
                      pstr_mae_asi->group_definition[i].group_id, &sg_id_tmp);
                  if (is_switch_grp_mem)
                  {
                    /* if yes -> check if a condition exists for this switchGroup */
                    if (ext_index > -1)
                    {
                      run_to = pstr_mae_asi->group_presets_ext_definition
                                   .group_preset_num_conditions[preset_index][ext_index];
                    }
                    else
                    {
                      run_to =
                          pstr_mae_asi->group_presets_definition[preset_index].num_conditions;
                    }
                    for (r = 0; r < run_to;
                         r++) /* run over all conditions of chosen preset (preset extension) */
                    {
                      WORD32 sg_id = -1;
                      if (ext_index > -1)
                      {
                        WORD32 is_sg_cond =
                            pstr_mae_asi->group_presets_ext_definition
                                .downmix_id_is_switch_group_condition[preset_index][ext_index][r];
                        if (is_sg_cond == 1)
                        {
                          sg_id = pstr_mae_asi->group_presets_ext_definition
                                      .group_preset_group_id[preset_index][ext_index][r];
                        }
                      }
                      else
                      {
                        WORD32 is_sg_cond = pstr_mae_asi->group_presets_ext_definition
                                                .is_switch_group_condition[preset_index][r];
                        if (is_sg_cond == 1)
                        {
                          sg_id = pstr_mae_asi->group_presets_definition[preset_index]
                                      .reference_id[r];
                        }
                      }
                      if (sg_id_tmp == sg_id)
                      {
                        has_sg_cond = 1;
                        break;
                      }
                    }
                    if (has_sg_cond)
                    {
                      /* do apply "offset" values defined by preset */
                      if (pstr_mae_asi->group_presets_ext_definition
                              .group_preset_disable_gain_interactivity[preset_index][ext_index]
                                                                      [r] == 1)
                      {
                        apply_preset_dependent_vals = 1;
                      }
                      if (pstr_mae_asi->group_presets_ext_definition
                              .group_preset_gain_flag[preset_index][ext_index][r] == 1)
                      {

                        WORD32 gain = (WORD32)pstr_mae_asi->group_presets_ext_definition
                                          .group_preset_gain[preset_index][ext_index][r];
                        FLOAT32 preset_gain = (gain - 255) * 0.5f + 32.0f;
                        intrct_gain_db = ia_add_flt(intrct_gain_db, preset_gain);
                      }
                    }
                  }
                }
              }
              else
              {
                for (j = 0; j < pstr_mae_asi->group_presets_definition[k].num_conditions; j++)
                {
                  if (pstr_mae_asi->group_definition[i].group_id ==
                      pstr_mae_asi->group_presets_definition[k].reference_id[j])
                  {
                    group_has_cond = j;
                    if (pstr_mae_asi->group_presets_definition[k]
                            .disable_gain_interact[group_has_cond] == 1)
                    {
                      apply_preset_dependent_vals = 1;
                    }
                    if (pstr_mae_asi->group_presets_definition[k].gain_flag[group_has_cond] == 1)
                    {
                      WORD32 gain =
                          (WORD32)pstr_mae_asi->group_presets_definition[k].gain[group_has_cond];
                      FLOAT32 preset_gain = (gain - 255) * 0.5f + 32.0f;
                      intrct_gain_db = ia_add_flt(intrct_gain_db, preset_gain);
                    }
                    break;
                  }
                }
                if (group_has_cond == -1)
                {
                  WORD32 is_switch_grp_mem = -1;
                  WORD32 sg_id_tmp = -1;
                  WORD32 run_to = 0;
                  WORD32 r = 0;
                  WORD32 has_sg_cond = 0;

                  /* check if group is member of a switchGroup */
                  is_switch_grp_mem = impeghd_md_proc_is_switch_group_mem(
                      pstr_mae_asi->num_switch_groups, pstr_mae_asi->switch_group_definition,
                      pstr_mae_asi->group_definition[i].group_id, &sg_id_tmp);
                  if (is_switch_grp_mem)
                  {
                    run_to = pstr_mae_asi->group_presets_definition[preset_index].num_conditions;
                    for (r = 0; r < run_to; r++)
                    {
                      /* run over all conditions of chosen preset (preset extension) */
                      WORD32 sg_id = -1;

                      WORD32 is_sg_cond = pstr_mae_asi->group_presets_ext_definition
                                              .is_switch_group_condition[preset_index][r];
                      if (is_sg_cond == 1)
                      {
                        sg_id =
                            pstr_mae_asi->group_presets_definition[preset_index].reference_id[r];
                      }

                      if (sg_id_tmp == sg_id)
                      {
                        has_sg_cond = 1;
                        break;
                      }
                    }
                    if (has_sg_cond)
                    {
                      /* do apply "offset" values defined by preset */
                      if (pstr_mae_asi->group_presets_definition[preset_index]
                              .disable_pos_interact[r] == 1)
                      {
                        apply_preset_dependent_vals = 1;
                      }
                      if (pstr_mae_asi->group_presets_definition[preset_index].gain_flag[r] == 1)
                      {
                        WORD32 gain =
                            (WORD32)pstr_mae_asi->group_presets_definition[preset_index].gain[r];
                        FLOAT32 preset_gain = (gain - 255) * 0.5f + 32.0f;
                        intrct_gain_db = ia_add_flt(intrct_gain_db, preset_gain);
                      }
                    }
                  }
                }
              }
            }
          }
        }
        if (apply_preset_dependent_vals == 0)
        {
          WORD32 gain = (WORD32)pstr_interact_cfg->ptr_ele_interaction_data->ei_gain[i];
          FLOAT32 ei_gain = (FLOAT32)ia_min_flt(ia_max_flt((gain - 64), -63), 31);
          intrct_gain_db = ia_add_flt(intrct_gain_db, ei_gain);
        }

        FLOAT32 min_gain = (FLOAT32)ia_min_flt(
            ia_max_flt(((WORD32)pstr_mae_asi->group_definition[i].min_gain - 64), -63), 31);
        FLOAT32 max_gain = (FLOAT32)pstr_mae_asi->group_definition[i].max_gain;
        if (ia_lt_flt(intrct_gain_db, min_gain))
        {
          error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
          /* Warning: Violation of minimum interactivity gain border. */
          intrct_gain_db = min_gain;
        }

        if (ia_lt_flt(max_gain, intrct_gain_db))
        {
          error = IA_MPEGH_MAE_EXE_FATAL_INTERACTIVITY_VIOLATION;
          /* Warning: Violation of maximum interactivity gain border. */
          intrct_gain_db = max_gain;
        }

        if ((impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1) &&
            (is_oam_decoded == 1)) /* at the moment not possible for SAOC */
        {
          WORD32 num_members = pstr_mae_asi->group_definition[i].group_num_members;
          WORD32 member_id = -1;
          WORD32 oam_index = -1;

          for (k = 0; k < num_members; k++)
          {
            if (pstr_mae_asi->group_definition[i].has_conjunct_members)
            {
              member_id = pstr_mae_asi->group_definition[i].start_id + k;
            }
            else
            {
              member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
            }

            /* get OAM data for the current group member */
            for (j = 0; j < pstr_interact_cfg->oam_count; j++)
            {
              if (pstr_interact_cfg->list_oam[j] == member_id)
              {
                oam_index = j;
                pstr_interact_cfg->gain_modified_single_object[k] =
                    pstr_oam_dec_state->gain_descaled[oam_index];
                break;
              }
            }

            if (oam_index == -1)
            {
              return IA_MPEGH_MAE_EXE_FATAL_INVALID_OAM_INDEX;
            }

            if (!ia_eq_flt(pstr_interact_cfg->gain_modified_single_object[k], 0.0f))
            {
              FLOAT32 lin_intrct_gain = 0.0f;
              /* Interaction gain to linear scale */
              lin_intrct_gain = (FLOAT32)pow(10.0f, intrct_gain_db / 20.0f);

              /* mul interaction gain */
              pstr_interact_cfg->gain_modified_single_object[k] =
                  ia_mul_flt(pstr_interact_cfg->gain_modified_single_object[k], lin_intrct_gain);
            }
            /* copy to OAM structure */
            pstr_oam_dec_state->gain_descaled[oam_index] =
                pstr_interact_cfg->gain_modified_single_object[k];
          }
          pstr_interact_cfg->gain_modified_grp[i] = -1.0f; /* object-based group */
        }
        else
        {
          pstr_interact_cfg->gain_modified_grp[i] =
              ia_add_flt(pstr_interact_cfg->gain_modified_grp[i], intrct_gain_db);
        }
      }
      else
      {
        if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
        {
          pstr_interact_cfg->gain_modified_grp[i] = -1.0f; /* object-based group */
        }
        else
        {
          pstr_interact_cfg->gain_modified_grp[i] = 1.0f;
        }
      }
    }

    /* add loudness compensation gain */
    if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) !=
        1) /* object-based group */
    {
      pstr_interact_cfg->gain_modified_grp[i] =
          ia_mul_flt(pstr_interact_cfg->gain_modified_grp[i],
                     (FLOAT32)pow(10.0f, pstr_interact_cfg->loudness_comp_gain / 20.0f));
    }
    else
    {
      WORD32 num_members = pstr_mae_asi->group_definition[i].group_num_members;
      WORD32 member_id = -1;
      WORD32 oam_index = -1;
      for (k = 0; k < num_members; k++)
      {
        if (pstr_mae_asi->group_definition[i].has_conjunct_members)
        {
          member_id = pstr_mae_asi->group_definition[i].start_id + k;
        }
        else
        {
          member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
        }
      }
      j = 0;
      {
        if (pstr_interact_cfg->list_oam[j] == member_id)
        {
          oam_index = j;
        }
        if (oam_index == -1)
        {
          return IA_MPEGH_MAE_EXE_FATAL_INVALID_OAM_INDEX;
        }
        pstr_oam_dec_state->gain_descaled[oam_index] =
            ia_mul_flt(pstr_oam_dec_state->gain_descaled[oam_index],
                       (FLOAT32)pow(10.0f, pstr_interact_cfg->loudness_comp_gain / 20.0f));
      }
    }
  }
  return error;
}

/**
 *  impeghd_md_proc_asc_az_ele_idx_to_deg
 *
 *  \brief Helper function to compute azimuth or elevation angle.
 *
 *  \param [in] idx       Azimuth or elevation index value.
 *  \param [in] direction Direction flag.
 *  \param [in] precision Angular precision.
 *
 *  \return Azimuth or elevation angle value computed from the input data
 *
 */
FLOAT32 impeghd_md_proc_asc_az_ele_idx_to_deg(WORD32 idx, WORD32 direction, WORD32 precision)
{
  FLOAT32 result = (FLOAT32)idx;
  if (direction == 1)
  {
    result = result * -1;
  }

  if (precision == 0)
  {
    result = result * 5;
  }
  return result;
}

/**
 *  impeghd_md_proc_diffuse_decorr_output
 *
 *  \brief Diffuseness rendering filter processing function
 *
 *  \param [out] ptr_output        Pointer to output buffer.
 *  \param [in]  ptr_input         Pointer to input buffer.
 *  \param [in,out] pstr_interact_cfg Pointer to interaction data config structure.
 *  \param [in]  spk_idx           Index of the speaker feed being processed.
 *  \param [in]  frame_length      Framelength value.
 *
 *
 *
 */
static VOID impeghd_md_proc_diffuse_decorr_output(FLOAT32 *ptr_output, FLOAT32 *ptr_input,
                                                  ia_interaction_data_struct *pstr_interact_cfg,
                                                  WORD32 spk_idx, WORD32 frame_length)
{
  WORD32 i, k;
  WORD32 index = 0;
  WORD32 length = MAE_DIFFUSE_DECORR_LENGTH;
  FLOAT32 result = 0.0f;
  for (k = 0; k < frame_length; k++)
  {
    index = pstr_interact_cfg->diffuse_counter;
    pstr_interact_cfg->diffuse_decorr_filt_states[spk_idx][index] = ptr_input[k];
    for (i = 0; i < length; i++)
    {
      result = ia_mac_flt(result, impeghd_metadata_diffuseness_decorr_filter_tbl[spk_idx][i],
                          pstr_interact_cfg->diffuse_decorr_filt_states[spk_idx][index--]);
      if (index < 0)
      {
        index = length - 1;
      }
    }
    if (++pstr_interact_cfg->diffuse_counter >= length)
    {
      pstr_interact_cfg->diffuse_counter = 0;
    }
    ptr_output[k] = result;
  }
}
/**
 *  impeghd_md_create_diffuse_part
 *
 *  \brief Create diffuse part
 *
 *  \param [in,out]  pstr_interact_cfg             Pointer to interaction data config structure
 *  \param [in]  pstr_asc                      Pointer to audio scene config structure
 *  \param [in] pstr_signals_3d                Pointer to signals 3d config structure
 *  \param [in] pstr_local_setup_config        Pointer to local set up config structure
 *  \param [in] pstr_enh_oam_cfg               Pointer to enhanced oam config structure
 *  \param [in] pstr_enh_oam_frame             Pointer to enhanced oam frame structure
 *  \param [in] pstr_dec_data                  Pointer to decoder data structure
 *  \param [in] ptr_scratch_mem                Pointer to scratch memory
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_create_diffuse_part(
    ia_interaction_data_struct *pstr_interact_cfg, ia_audio_specific_config_struct *pstr_asc,
    ia_signals_3d *pstr_signals_3d, ia_local_setup_struct *pstr_local_setup_config,
    ia_enh_oam_config_struct *pstr_enh_oam_cfg, ia_enh_obj_md_frame_str *pstr_enh_oam_frame,
    ia_dec_data_struct *pstr_dec_data, pWORD8 ptr_scratch_mem)
{
  IA_ERRORCODE error = 0;
  WORD32 i, j, k;
  WORD32 ren_type = pstr_local_setup_config->ren_type;
  WORD32 num_speakers = 0;
  WORD32 num_non_lfe_spks = 0;
  WORD32 ct = 0;
  WORD32 framesize = pstr_dec_data->str_usac_data.ccfl;
  WORD32 num_objects = 0;
  pWORD8 ptr_scratch = ptr_scratch_mem;
  FLOAT32 spk_reciproc_fac = 1.0f;
  FLOAT32 **ptr_diffuse_in_buf = NULL;
  FLOAT32 **ptr_diffuse_out_buf = NULL;
  FLOAT32 **ptr_diffuse_out_buf_decorr = NULL;
  FLOAT32 oam_and_diffuse_gain[MAX_NUM_ELEMENTS];
  ia_cicp_ls_geo_str str_ls_geo[CICP2GEOMETRY_MAX_LOUDSPEAKERS] = {0};
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  ia_core_coder_memset(&oam_and_diffuse_gain[0], MAX_NUM_ELEMENTS);
  if (ren_type != 0)
  {
    WORD32 num_ch;
    WORD32 num_lfes;
    WORD32 *channel_names;
    num_speakers = pstr_local_setup_config->brir_pairs;
    for (i = 0; i < num_speakers; i++)
    {
      if (pstr_local_setup_config->spk_config.spk_layout_type == 2)
      {
        if (pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                .is_cicp_spk_idx == 1)
        {
          memcpy(&str_ls_geo[i],
                 &ia_cicp_ls_geo_tbls[pstr_local_setup_config->spk_config.str_flex_spk_data
                                          .str_flex_spk_descr[i]
                                          .cicp_spk_idx],
                 sizeof(str_ls_geo[i]));
        }
        else
        {
          WORD32 az_angle_idx =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .az_angle_idx;
          WORD32 az_direction =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .az_direction;
          WORD32 el_angle_idx =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .el_angle_idx;
          WORD32 el_direction =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .el_direction;
          WORD32 precision =
              pstr_local_setup_config->spk_config.str_flex_spk_data.angular_precision;

          str_ls_geo[i].ls_elevation =
              impeghd_md_proc_asc_az_ele_idx_to_deg(el_angle_idx, el_direction, precision);
          str_ls_geo[i].ls_azimuth =
              impeghd_md_proc_asc_az_ele_idx_to_deg(az_angle_idx, az_direction, precision);
          str_ls_geo[i].lfe_flag =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i].is_lfe;
          str_ls_geo[i].screen_rel_flag = 0;
        }
      }
      else if (pstr_local_setup_config->spk_config.spk_layout_type == 1)
      {
        memcpy(&str_ls_geo[i],
               &ia_cicp_ls_geo_tbls[pstr_local_setup_config->spk_config.cicp_spk_idx[i]],
               sizeof(str_ls_geo[i]));
      }
      else if ((pstr_local_setup_config->spk_config.spk_layout_type == 0) && (i == 0))
      {
        impeghd_cicpidx_2_ls_geometry(pstr_local_setup_config->spk_config.cicp_spk_layout_idx,
                                      (const ia_cicp_ls_geo_str **)&str_ls_geo[0], &num_ch,
                                      &num_lfes, (const WORD32 **)&channel_names);
      }

      if (str_ls_geo[i].lfe_flag == 0)
      {
        num_non_lfe_spks++;
      }
    }
  }
  else
  {
    WORD32 num_ch;
    WORD32 num_lfes;
    const WORD32 *channel_names;
    num_speakers = pstr_local_setup_config->num_spk;

    for (i = 0; i < num_speakers; i++)
    {
      if (pstr_local_setup_config->spk_config.spk_layout_type == 2)
      {
        if (pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                .is_cicp_spk_idx == 1)
        {
          memcpy(&str_ls_geo[i],
                 &ia_cicp_ls_geo_tbls[pstr_local_setup_config->spk_config.str_flex_spk_data
                                          .str_flex_spk_descr[i]
                                          .cicp_spk_idx],
                 sizeof(str_ls_geo[i]));
        }
        else
        {
          str_ls_geo[i].ls_azimuth =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .az_angle_idx;
          str_ls_geo[i].ls_elevation =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i]
                  .el_angle_idx;
          str_ls_geo[i].lfe_flag =
              pstr_local_setup_config->spk_config.str_flex_spk_data.str_flex_spk_descr[i].is_lfe;
          str_ls_geo[i].screen_rel_flag = 0;
        }
      }
      else if (pstr_local_setup_config->spk_config.spk_layout_type == 1)
      {
        memcpy(&str_ls_geo[i],
               &ia_cicp_ls_geo_tbls[pstr_local_setup_config->spk_config.cicp_spk_idx[i]],
               sizeof(str_ls_geo[i]));
      }
      else if ((pstr_local_setup_config->spk_config.spk_layout_type == 0) && (i == 0))
      {
        impeghd_cicpidx_2_ls_geometry(pstr_local_setup_config->spk_config.cicp_spk_layout_idx,
                                      (const ia_cicp_ls_geo_str **)&str_ls_geo[0], &num_ch,
                                      &num_lfes, (const WORD32 **)&channel_names);
      }

      if (str_ls_geo[i].lfe_flag == 0)
      {
        num_non_lfe_spks++;
      }
    }
  }

  if (num_non_lfe_spks > 0)
  {
    spk_reciproc_fac = (FLOAT32)(1.0f / ia_sqrt_flt((FLOAT32)num_non_lfe_spks));
  }

  if (!(pstr_enh_oam_cfg->has_diffuseness))
  {
    error = IA_MPEGH_MAE_EXE_FATAL_NO_DIFFUSION;
  }
  else
  {
    /* get num_objects */
    for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
    {
      if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
      {
        if (pstr_interact_cfg->divergence_asi_modified != 1)
        {
          ct += pstr_mae_asi->group_definition[i].group_num_members;
        }
        else
        {
          ct += pstr_interact_cfg->orig_num_grp_members[i];
        }
      }
    }
    num_objects = ct;

    /* init buffers */

    ptr_diffuse_in_buf = (FLOAT32 **)ptr_scratch;
    ptr_scratch += (num_objects * sizeof(ptr_diffuse_in_buf[0]));
    for (i = 0; i < num_objects; i++)
    {
      ptr_diffuse_in_buf[i] = (FLOAT32 *)ptr_scratch;
      ptr_scratch += (framesize * sizeof(ptr_diffuse_in_buf[i][0]));
    }

    ptr_diffuse_out_buf = (FLOAT32 **)ptr_scratch;
    ptr_scratch += (num_speakers * sizeof(ptr_diffuse_out_buf[0]));
    ptr_diffuse_out_buf_decorr = (FLOAT32 **)ptr_scratch;
    ptr_scratch += (num_speakers * sizeof(ptr_diffuse_out_buf_decorr[0]));
    for (i = 0; i < num_speakers; i++)
    {
      ptr_diffuse_out_buf[i] = (FLOAT32 *)ptr_scratch;
      ptr_scratch += (framesize * sizeof(ptr_diffuse_out_buf[i][0]));
      ptr_diffuse_out_buf_decorr[i] = (FLOAT32 *)ptr_scratch;
      ptr_scratch += (framesize * sizeof(ptr_diffuse_out_buf_decorr[i][0]));
    }

    /* copy input signals, get oam gain and multiply with diffuse gain */
    ct = 0;
    for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
    {
      WORD32 num_members = 0;
      if (pstr_interact_cfg->divergence_asi_modified != 1)
      {
        num_members = pstr_mae_asi->group_definition[i].group_num_members;
      }
      else
      {
        num_members = pstr_interact_cfg->orig_num_grp_members[i];
      }
      if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
      {
        for (j = 0; j < num_members; j++)
        {
          WORD32 mae_id = -1;
          FLOAT32 gain = 0.0f;
          if (pstr_mae_asi->group_definition[i].has_conjunct_members != 1)
          {
            mae_id = pstr_mae_asi->group_definition[i].metadata_ele_id[j];
          }
          else
          {
            mae_id = pstr_mae_asi->group_definition[i].start_id + j;
          }
          for (k = 0; k < num_objects; k++)
          {
            if (mae_id == pstr_interact_cfg->list_oam[k])
            {
              break;
            }
          }

          if (!(pstr_enh_oam_cfg->has_common_group_diffuseness))
          {
            gain = (FLOAT32)ia_sqrt_flt(
                pstr_enh_oam_frame
                    ->diffuseness[impeghd_md_proc_obj_idx(pstr_signals_3d, mae_id)]);
          }
          else
          {
            gain = (FLOAT32)ia_sqrt_flt(pstr_enh_oam_frame->diffuseness[0]);
          }

          /* overall modified gain = oam modified gain * diffuse gain */
          oam_and_diffuse_gain[ct] = ia_mul_flt(pstr_oam_dec_state->gain_descaled[k], gain);
          for (k = 0; k < framesize; k++)
          {
            ptr_diffuse_in_buf[ct][k] =
                (FLOAT32)pstr_dec_data->str_usac_data.time_sample_vector[mae_id][k];
          }
          ct++;
        }
      }
    }

    /* apply mofified OAM gain and diffuse gain */
    for (i = 0; i < num_objects; i++)
    {
      for (k = 0; k < framesize; k++)
      {
        ptr_diffuse_in_buf[i][k] = ia_mul_flt(ptr_diffuse_in_buf[i][k], oam_and_diffuse_gain[i]);
      }
    }

    /* send to speakers and decorrelate speaker signals */
    for (i = 0; i < num_objects; i++)
    {
      for (j = 0; j < num_speakers; j++)
      {
        if (str_ls_geo[j].lfe_flag != 0)
        {
          memset(&ptr_diffuse_out_buf[j][0], 0, framesize * sizeof(ptr_diffuse_out_buf[j][0]));
        }
        else
        {
          for (k = 0; k < framesize; k++)
          {
            ptr_diffuse_out_buf[j][k] =
                ia_mac_flt(ptr_diffuse_out_buf[j][k], ptr_diffuse_in_buf[i][k], spk_reciproc_fac);
          }
        }
      }
    }
    /* decorrelate */
    for (j = 0; j < num_speakers; j++)
    {
      if (str_ls_geo[j].lfe_flag != 0)
      {
        memset(&ptr_diffuse_out_buf_decorr[j][0], 0,
               sizeof(ptr_diffuse_out_buf_decorr[j][0]) * framesize);
      }
      else
      {
        impeghd_md_proc_diffuse_decorr_output(&ptr_diffuse_out_buf_decorr[j][0],
                                              &ptr_diffuse_out_buf[j][0], pstr_interact_cfg, j,
                                              framesize);
      }
    }
  }

  return error;
}
/**
 *  impeghd_md_proc_apply_dir_gain
 *
 *  \brief Applies dir gain.
 *
 *  \param [in] pstr_dec_data       Pointer to decoder data structure.
 *  \param [in] pstr_interact_cfg   Pointer to interaction data config structure.
 *  \param [in] pstr_asc              Pointer to audio scene config structure.
 *  \param [in] pstr_signals_3d       Pointer to signals 3d config structure.
 *  \param [in] pstr_enh_oam_cfg      Pointer to enhanced oam config structure.
 *  \param [in] pstr_enh_obj_md_frame Pointer to enhanced object metadata frame structure.
 *  \param [in] div_objs_added        Divergence objects added value.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_proc_apply_dir_gain(ia_dec_data_struct *pstr_dec_data,
                                                   ia_interaction_data_struct *pstr_interact_cfg,
                                                   ia_audio_specific_config_struct *pstr_asc,
                                                   ia_signals_3d *pstr_signals_3d,
                                                   ia_enh_oam_config_struct *pstr_enh_oam_cfg,
                                                   ia_enh_obj_md_frame_str *pstr_enh_obj_md_frame,
                                                   WORD32 div_objs_added)
{
  WORD32 i, j, k;
  WORD32 ct = 0, num_objects;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  /* get num_objects */
  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
    {
      if (pstr_interact_cfg->divergence_asi_modified != 1)
      {
        ct += pstr_asc->str_mae_asi.group_definition[i].group_num_members;
      }
      else
      {
        ct += pstr_interact_cfg->orig_num_grp_members[i];
      }
    }
  }
  num_objects = ct;

  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    WORD32 num_members = 0;
    if (div_objs_added <= 0)
    {
      num_members = pstr_asc->str_mae_asi.group_definition[i].group_num_members;
    }
    else
    {
      num_members = pstr_interact_cfg->orig_num_grp_members[i];
    }

    if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
    {
      for (j = 0; j < num_members; j++)
      {
        WORD32 gain_idx = 0;
        WORD32 mae_id = -1;
        FLOAT32 gain = 0.0f;
        FLOAT32 gain_temp = 0.0f;
        if (pstr_mae_asi->group_definition[i].has_conjunct_members != 1)
        {
          mae_id = pstr_mae_asi->group_definition[i].metadata_ele_id[j];
        }
        else
        {
          mae_id = pstr_mae_asi->group_definition[i].start_id + j;
        }
        for (k = 0; k < num_objects; k++)
        {
          if (pstr_interact_cfg->list_oam[k] == mae_id)
          {
            break;
          }
        }
        /* new modified gain = oam modified gain * direct gain */
        if (!pstr_enh_oam_cfg->has_common_group_diffuseness)
        {
          gain_idx = impeghd_md_proc_obj_idx(pstr_signals_3d, mae_id);
        }

        gain_temp = (FLOAT32)ia_sqrt_flt(pstr_enh_obj_md_frame->diffuseness[gain_idx]);
        gain = (FLOAT32)ia_sqrt_flt(ia_sub_flt(1.0f, gain_temp));
        pstr_oam_dec_state->gain_descaled[k] *= gain;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_md_proc_preset_cnd
 *
 *  \brief Applies preset condition.
 *
 *  \param [in]  pstr_asc    Pointer toaudio scene config structure
 *  \param [in]  presetID    Preset id
 *  \param [in] group_id     Group id
 *  \param [in] downmix_id   Down mix id
 *  \param [out] ext_index    Extension index
 *  \return cond
 *
 */
static WORD32 impeghd_md_proc_preset_cnd(ia_audio_specific_config_struct *pstr_asc,
                                         WORD32 presetID, WORD32 group_id, WORD32 downmix_id,
                                         WORD32 *ext_index)
{
  WORD32 i, k;
  WORD32 preset_idx = -1;
  WORD32 grp_index = -1;
  WORD32 cond = -1;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  *ext_index = -1;

  for (k = 0; k < pstr_mae_asi->num_group_presets; k++)
  {
    if (pstr_mae_asi->group_presets_definition[k].group_id == presetID)
    {
      preset_idx = k;
      if (pstr_mae_asi->group_presets_ext_definition.num_downmix_id_group_preset_extensions[k] >
          0)
      {
        for (i = 0;
             i <
             pstr_mae_asi->group_presets_ext_definition.num_downmix_id_group_preset_extensions[k];
             i++)
        {
          if (pstr_mae_asi->group_presets_ext_definition.group_preset_downmix_id[k][i] ==
              downmix_id)
          {
            *ext_index = i;
            break;
          }
        }
      }
      break;
    }
  }

  if (preset_idx > -1)
  {
    if (*ext_index <= -1)
    {
      for (k = 0; k < pstr_mae_asi->group_presets_definition[preset_idx].num_conditions; k++)
      {
        if (pstr_mae_asi->group_presets_definition[preset_idx].reference_id[k] == group_id)
        {
          grp_index = k;
          break;
        }
      }
    }
    else
    {
      for (k = 0; k < pstr_mae_asi->group_presets_ext_definition
                          .group_preset_num_conditions[preset_idx][*ext_index];
           k++)
      {
        if ((pstr_mae_asi->group_presets_ext_definition.group_preset_downmix_id[*ext_index][k] ==
             group_id) &&
            (pstr_mae_asi->group_presets_ext_definition
                 .downmix_id_is_switch_group_condition[preset_idx][*ext_index][k] == 0))
        {
          grp_index = k;
          break;
        }
      }
    }
    if (grp_index > -1)
    {
      if (*ext_index <= -1)
      {
        cond = pstr_mae_asi->group_presets_definition[preset_idx].cond_on_off[grp_index];
      }
      else
      {

        cond = pstr_mae_asi->group_presets_ext_definition
                   .group_preset_condition_on_off[preset_idx][*ext_index][grp_index];
      }
    }
  }
  return cond;
}

/**
 *  impehgd_apply_switch_group_off
 *
 *  \brief Apply switch group off.
 *
 *  \param [in]  pstr_asc               Pointer to audio scene configuration structure.
 *  \param [out] group_id               group id
 *  \param [out] on_off_status_modified Pointer to on or off status
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impehgd_apply_switch_group_off(ia_audio_specific_config_struct *pstr_asc,
                                                   WORD32 group_id,
                                                   WORD32 *on_off_status_modified)
{

  WORD32 i, k;
  WORD32 switch_group_idx = 0;
  WORD32 num_switch_groups;
  WORD32 group_id2;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  num_switch_groups = pstr_mae_asi->num_switch_groups;

  if (num_switch_groups > 1)
  {
    for (k = 0; k < num_switch_groups; k++)
    {
      for (i = 0; i < pstr_mae_asi->switch_group_definition[k].group_num_members; i++)
      {
        if (pstr_mae_asi->switch_group_definition[k].member_id[i] == group_id)
        {
          switch_group_idx = k;
          break;
        }
      }
      if (switch_group_idx > 0)
      {
        break;
      }
    }
  }

  for (k = 0; k < pstr_mae_asi->switch_group_definition[switch_group_idx].group_num_members; k++)
  {
    WORD32 grp_index = -1;
    group_id2 = pstr_mae_asi->switch_group_definition[switch_group_idx].member_id[k];

    for (i = 0; i < pstr_mae_asi->num_groups; i++)
    {
      if (group_id2 == pstr_mae_asi->group_definition[i].group_id)
      {
        grp_index = i;
        break;
      }
    }

    if (group_id2 != group_id)
    {
      if (on_off_status_modified[grp_index] == 1)
      {
        return IA_MPEGH_MAE_EXE_FATAL_SWITCH_GROUP_OFF;
      }
      else
      {
        on_off_status_modified[grp_index] = 0;
      }
    }
    else
    {

      on_off_status_modified[grp_index] = 1;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_md_proc_apply_switch_grp_logic
 *
 *  \brief Appy switch group logic.
 *
 *  \param [in]     pstr_interact_cfg  Pointer to interaction config structure
 *  \param [in,out] pstr_asc           Pointer to audio specific config structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE
impeghd_md_proc_apply_switch_grp_logic(ia_interaction_data_struct *pstr_interact_cfg,
                                       ia_audio_specific_config_struct *pstr_asc)
{
  WORD32 i, k;
  WORD32 num_switch_groups;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  num_switch_groups = pstr_mae_asi->num_switch_groups;

  /* loop over all switch groups */
  for (i = 0; i < num_switch_groups; i++)
  {
    WORD32 valid = 1;
    WORD32 num_non_zero = 0;
    WORD32 idx = -1;
    WORD32 member_id = -1;
    WORD32 sw_grp_allow_on_off = -1;
    WORD32 num_members = pstr_mae_asi->switch_group_definition[i].group_num_members;
    WORD32 default_id = pstr_mae_asi->switch_group_definition[i].default_group_id;

    for (k = 0; k < num_members; k++)
    {
      idx = impeghd_md_proc_grp_idx(pstr_asc,
                                    pstr_mae_asi->switch_group_definition[i].member_id[k]);
      if (pstr_interact_cfg->on_off_status_modified[idx] == 1)
      {
        num_non_zero++;
      }
    }
    if (num_non_zero > 1)
    {
      /* if more than one member is activated -> error */
      valid = 0;
    }
    if (num_non_zero == 0)
    {
      sw_grp_allow_on_off = impeghd_md_proc_switch_group_allow_on_off(
          pstr_asc, pstr_mae_asi->switch_group_definition[i].group_id);
      if (sw_grp_allow_on_off != 1)
      {
        idx = impeghd_md_proc_grp_idx(pstr_asc, default_id);
        pstr_interact_cfg->on_off_status_modified[idx] = 1;
      }
      else
      {
        valid = 1;
      }
    }
    if (valid == 0)
    {
      for (k = 0; k < num_members; k++)
      {
        member_id = pstr_mae_asi->switch_group_definition[i].member_id[k];
        if (default_id != member_id)
        {
          idx = impeghd_md_proc_grp_idx(pstr_asc, member_id);
          pstr_interact_cfg->on_off_status_modified[idx] = 0;
        }
        else
        {
          idx = impeghd_md_proc_grp_idx(pstr_asc, default_id);
          pstr_interact_cfg->on_off_status_modified[idx] = 1;
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_apply_on_off_intrct
 *
 *  \brief Apply on off interaction.
 *
 *  \param [in]  pstr_interact_cfg Pointer to interaction configuration structure.
 *  \param [in]  pstr_asc          Pointer to audio specific configuration structure.
 *  \param [in]  dmx_id            Downmix id.
 *  \param [out] sel_preset        Selected preset.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_apply_on_off_intrct(ia_interaction_data_struct *pstr_interact_cfg,
                                                ia_audio_specific_config_struct *pstr_asc,
                                                WORD32 dmx_id, WORD32 *sel_preset)
{
  IA_ERRORCODE error = 0;
  WORD32 i, k, e, j;
  WORD32 num_switch_groups;
  WORD32 preset_index = -1;
  WORD32 chosen_preset = -1;
  WORD32 valid_preset_id = -1;
  WORD32 run_to = 0;
  WORD32 ext_index = -1;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  num_switch_groups = pstr_mae_asi->num_switch_groups;
  valid_preset_id = 0;

  if (pstr_interact_cfg->dflt_preset_mode != 1)
  {
    if (pstr_interact_cfg->apply_intrct_data == 1)
    {
      chosen_preset = pstr_interact_cfg->ptr_ele_interaction_data->ei_group_preset_id;
    }
    else
    {
      chosen_preset = *sel_preset;
    }
  }
  else
  {
    WORD32 min_preset_id = MAE_MAX_NUM_GROUP_PRESETS;

    for (i = 0; i < pstr_mae_asi->num_group_presets; i++)
    {
      if (pstr_mae_asi->group_presets_definition[i].group_id < min_preset_id)
      {
        min_preset_id = pstr_mae_asi->group_presets_definition[i].group_id;
        if (min_preset_id == 0)
        {
          break;
        }
      }
    }
    chosen_preset = min_preset_id;
  }

  for (i = 0; i < pstr_mae_asi->num_group_presets; i++)
  {
    if (pstr_mae_asi->group_presets_definition[i].group_id == chosen_preset)
    {
      valid_preset_id = 1;
      preset_index = i;
      break;
    }
  }

  if (valid_preset_id == 1)
  {
    *sel_preset = chosen_preset;
  }
  else
  {
    if (*sel_preset > -1)
    {
      chosen_preset = *sel_preset;
    }
    else
    {
      return IA_MPEGH_MAE_EXE_FATAL_ON_OFF_INTERACT_FAILED;
    }
  }

  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    pstr_interact_cfg->on_off_status_modified[i] = -1;
  }

  if (chosen_preset <= -1)
  {
    /* no preset chosen and advanced interaction mode is active */
    if (pstr_interact_cfg->interaction_type == 0)
    {
      for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
      {
        pstr_interact_cfg->on_off_status_modified[i] = -1;
      }

      /* loop over all groups */
      for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
      {
        WORD32 allow_on_off = pstr_mae_asi->group_definition[i].allow_on_off;
        WORD32 grp_index = -1;
        WORD32 group_id = pstr_mae_asi->group_definition[i].group_id;
        WORD32 temp = 0;

        if (!(allow_on_off))
        {
          pstr_interact_cfg->on_off_status_modified[i] =
              pstr_mae_asi->group_definition[i].default_on_off;
        }
        else
        {
          WORD32 idx;
          grp_index = -1;
          for (idx = 0; idx < pstr_interact_cfg->ptr_ele_interaction_data->ei_num_groups; idx++)
          {
            if (group_id == pstr_interact_cfg->ptr_ele_interaction_data->ei_grp_id[idx])
            {
              grp_index = idx;
              break;
            }
          }
          if (grp_index > -1)
          {
            pstr_interact_cfg->on_off_status_modified[i] =
                pstr_interact_cfg->ptr_ele_interaction_data->ei_on_off[grp_index];
            temp = impeghd_md_proc_is_switch_group_mem(pstr_mae_asi->num_switch_groups,
                                                       pstr_mae_asi->switch_group_definition,
                                                       group_id, NULL);
            if ((pstr_interact_cfg->on_off_status_modified[i] == 1) && (temp == 1))
            {
              error = impehgd_apply_switch_group_off(pstr_asc, group_id,
                                                     pstr_interact_cfg->on_off_status_modified);
              if (error != 0)
              {
                /*
                 * Warning: Violation of SwitchGroup Logic, two SwitchGroup
                 * members were defined to be active simultaneously.
                 */
                return error;
              }
            }
          }
          else
          {
            pstr_interact_cfg->on_off_status_modified[i] =
                pstr_mae_asi->group_definition[i].default_on_off;
          }
        }
      }
      error = impeghd_md_proc_apply_switch_grp_logic(pstr_interact_cfg, pstr_asc);
      if (error != 0)
      {
        return error;
      }
    }
  }
  else
  {
    for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
    {
      WORD32 grp_index = -1;
      WORD32 preset_cond = -1;
      WORD32 group_id = pstr_mae_asi->group_definition[i].group_id;

      ext_index = -1;
      preset_cond =
          impeghd_md_proc_preset_cnd(pstr_asc, chosen_preset, group_id, dmx_id, &ext_index);
      if (preset_cond == -1)
      {
        /* if current group onoff status is not defined in the preset conditions... */
        grp_index = i;
        if (pstr_mae_asi->group_definition[i].allow_on_off == 1)
        {
          if (pstr_interact_cfg->apply_intrct_data == 1)
          {
            /* ... the onoff status of the interaction data should be used, if allow_on_off == 1
             * and  interaction data is available */
            pstr_interact_cfg->on_off_status_modified[i] =
                pstr_interact_cfg->ptr_ele_interaction_data->ei_on_off[i];
          }
          else
          {
            /* ... the default onoff status should be used, if allow_on_off == 1 and no
             * interaction data is available */
            pstr_interact_cfg->on_off_status_modified[i] =
                pstr_mae_asi->group_definition[grp_index].default_on_off;
          }
        }
        else
        {
          /* ... the default onoff status of the interaction data should be used, if allow_on_off
           * == 0 */
          pstr_interact_cfg->on_off_status_modified[i] =
              pstr_mae_asi->group_definition[grp_index].default_on_off;
        }
      }
      else if ((preset_cond == 1) && (pstr_interact_cfg->on_off_status_modified[i] == -1))
      {
        pstr_interact_cfg->on_off_status_modified[i] = 1;
        if (impeghd_md_proc_is_switch_group_mem(pstr_mae_asi->num_switch_groups,
                                                pstr_mae_asi->switch_group_definition, group_id,
                                                NULL))
        {
          error = impehgd_apply_switch_group_off(pstr_asc, group_id,
                                                 pstr_interact_cfg->on_off_status_modified);
          if (error != 0)
          {
            /*
             * Error: Violation of SwitchGroup Logic in the chosen preset, two Switch group
             *        members were defined to be active simultaneously.
             */
            return error;
          }
        }
      }
      else if ((preset_cond == 0) && (pstr_interact_cfg->on_off_status_modified[i] == -1))
      {
        pstr_interact_cfg->on_off_status_modified[i] = 0;
      }
    }

    /* Check for switch group conditions */
    if (preset_index != -1)
    {
      if (dmx_id > -1)
      {
        for (e = 0; e < pstr_mae_asi->group_presets_ext_definition
                            .num_downmix_id_group_preset_extensions[preset_index];
             e++)
        {
          if (pstr_mae_asi->group_presets_ext_definition
                  .group_preset_downmix_id[preset_index][e] == dmx_id)
          {
            ext_index = e;
            break;
          }
        }
      }
      if (ext_index <= -1)
      {
        run_to = pstr_mae_asi->group_presets_definition[preset_index].num_conditions;
      }
      else
      {
        run_to = pstr_mae_asi->group_presets_ext_definition
                     .group_preset_num_conditions[preset_index][ext_index];
      }
    }

    /* Run over all conditions of chosen preset (preset extension) */
    for (i = 0; i < run_to; i++)
    {
      WORD32 cond = -1;
      WORD32 sg_id = -1;
      if (preset_index != -1)
      {
        if (ext_index <= -1)
        {
          WORD32 is_sg_cond = pstr_mae_asi->group_presets_ext_definition
                                  .is_switch_group_condition[preset_index][i];
          if (is_sg_cond == 1)
          {
            cond = pstr_mae_asi->group_presets_definition[preset_index].cond_on_off[i];
            sg_id = pstr_mae_asi->group_presets_definition[preset_index].reference_id[i];
          }
        }
        else
        {
          WORD32 is_sg_cond =
              pstr_mae_asi->group_presets_ext_definition
                  .downmix_id_is_switch_group_condition[preset_index][ext_index][i];
          if (is_sg_cond == 1)
          {
            cond = pstr_mae_asi->group_presets_ext_definition
                       .group_preset_condition_on_off[preset_index][ext_index][i];
            sg_id = pstr_mae_asi->group_presets_ext_definition
                        .group_preset_group_id[preset_index][ext_index][i];
          }
        }
      }

      /* All members have to be switched off, if SG is allowed to be switched off. If not, the
       * preset definition is not correct */
      if (cond == 0)
      {
        WORD32 switch_grp_found = 0;
        for (k = 0; k < num_switch_groups; k++)
        {
          if (pstr_mae_asi->switch_group_definition[k].group_id == sg_id)
          {
            WORD32 idx = -1;
            WORD32 allow_on_off = pstr_mae_asi->switch_group_definition[k].allow_on_off;

            switch_grp_found = 1;

            if (allow_on_off == 0)
            {
              /*
               * Error: Violation of SwitchGroup Logic, is is not allowed to
               * switch it off completely. Preset definition is invalid
               */
              break;
            }
            else
            {
              for (j = 0; j < pstr_mae_asi->switch_group_definition[k].group_num_members; j++)
              {
                idx = impeghd_md_proc_grp_idx(
                    pstr_asc, pstr_mae_asi->switch_group_definition[i].member_id[k]);
                if (pstr_interact_cfg->on_off_status_modified[idx] == 1)
                {
                  pstr_interact_cfg->on_off_status_modified[idx] = 0;
                }
              }
            }
          }
        }
        if (switch_grp_found == 0)
        {
          /*
           * Error: There is no switch group defined with the ID referenced in the
           * switch group condition of the chosen preset.
           */
        }
      }
      else if (cond == 1) /* one member has to be activated, if not: default is activated */
      {
        WORD32 switch_grp_found = 0;
        for (k = 0; k < num_switch_groups; k++)
        {
          if (pstr_mae_asi->switch_group_definition[k].group_id == sg_id)
          {
            WORD32 num_non_zero = 0;
            WORD32 idx = -1;
            WORD32 default_id = pstr_mae_asi->switch_group_definition[k].default_group_id;
            switch_grp_found = 1;

            for (j = 0; j < pstr_mae_asi->switch_group_definition[k].group_num_members; j++)
            {
              idx = impeghd_md_proc_grp_idx(
                  pstr_asc, pstr_mae_asi->switch_group_definition[i].member_id[k]);
              if (pstr_interact_cfg->on_off_status_modified[idx] == 1)
              {
                num_non_zero++;
              }
            }
            if (num_non_zero == 0)
            {
              idx = impeghd_md_proc_grp_idx(pstr_asc, default_id);
              pstr_interact_cfg->on_off_status_modified[idx] = 1;
            }
            else if (num_non_zero > 1)
            {
              /*
               * Error: Violation of SwitchGroup Logic, two SwitchGroup members
               * were defined to be active simultaneously
              */
              break;
            }
          }
        }
        if (switch_grp_found == 0)
        {
          /*
           * Error: There is no switch group defined with the ID referenced in the
           * switch group condition of the chosen preset.
           */
        }
      }
    }
    error = impeghd_md_proc_apply_switch_grp_logic(pstr_interact_cfg, pstr_asc);
    if (error != 0)
    {
      return error;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_screen_rel_remap_ele
 *
 *  \brief Remapping of sreen related elevation values
 *  \param [in,out] elevation                Pointer to elevation value
 *  \param [in]     pstr_local_setup         Pointer to local setup structure
 *  \param [in,out] pstr_prod_screen_data    Pointer to production screen size data structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE
impeghd_screen_rel_remap_ele(FLOAT32 *elevation, ia_local_setup_struct *pstr_local_setup,
                             ia_production_screen_size_data *pstr_prod_screen_data)
{
  FLOAT32 el_temp = *elevation;
  FLOAT32 height_rep = 0.0f, height_ref = 0.0f;
  FLOAT32 temp = 0;
  if (ia_lt_flt(*elevation, ia_negate_flt(NINETY_DEGREE)) || ia_lt_flt(NINETY_DEGREE, *elevation))
  {
    return IA_MPEGH_MAE_EXE_FATAL_UNSUPPORTED_ELEVATION_RANGE;
  }

  if (ia_lt_flt(NINETY_DEGREE, pstr_prod_screen_data->el_top))
  {
    pstr_prod_screen_data->el_top =
        ia_sub_flt(NINETY_DEGREE, ia_sub_flt(pstr_prod_screen_data->el_top, NINETY_DEGREE));
  }
  if (ia_lt_flt(pstr_prod_screen_data->el_top, ia_negate_flt(NINETY_DEGREE)))
  {
    pstr_prod_screen_data->el_top = ia_add_flt(
        ia_negate_flt(NINETY_DEGREE), ia_add_flt(pstr_prod_screen_data->el_top, NINETY_DEGREE));
  }
  if (ia_lt_flt(NINETY_DEGREE, pstr_prod_screen_data->el_bottom))
  {
    pstr_prod_screen_data->el_bottom =
        ia_sub_flt(NINETY_DEGREE, ia_sub_flt(pstr_prod_screen_data->el_bottom, NINETY_DEGREE));
  }
  if (ia_lt_flt(pstr_prod_screen_data->el_bottom, ia_negate_flt(NINETY_DEGREE)))
  {
    pstr_prod_screen_data->el_bottom =
        ia_sub_flt(ia_negate_flt(NINETY_DEGREE),
                   ia_add_flt(pstr_prod_screen_data->el_bottom, NINETY_DEGREE));
  }
  if (pstr_local_setup->lcl_scrn_sz_top_el > (WORD32)NINETY_DEGREE)
  {
    pstr_local_setup->lcl_scrn_sz_top_el = (WORD32)ia_sub_flt(
        NINETY_DEGREE, ia_sub_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el, NINETY_DEGREE));
  }
  if (pstr_local_setup->lcl_scrn_sz_top_el < (WORD32)(-NINETY_DEGREE))
  {
    pstr_local_setup->lcl_scrn_sz_top_el = (WORD32)ia_add_flt(
        -NINETY_DEGREE, ia_add_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el, NINETY_DEGREE));
  }
  if (pstr_local_setup->lcl_scrn_sz_bottom_el > (WORD32)(NINETY_DEGREE))
  {
    pstr_local_setup->lcl_scrn_sz_bottom_el = (WORD32)ia_sub_flt(
        NINETY_DEGREE,
        ia_sub_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el, NINETY_DEGREE));
  }
  if (pstr_local_setup->lcl_scrn_sz_bottom_el < (WORD32)(-NINETY_DEGREE))
  {
    pstr_local_setup->lcl_scrn_sz_bottom_el = (WORD32)ia_add_flt(
        -NINETY_DEGREE,
        ia_add_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el, NINETY_DEGREE));
  }
  height_rep = (FLOAT32)ia_fabs_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el -
                                    (FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el);
  height_ref = (FLOAT32)ia_fabs_flt(
      ia_sub_flt(pstr_prod_screen_data->el_top, pstr_prod_screen_data->el_bottom));
  if (ia_lteq_flt(ia_negate_flt(NINETY_DEGREE), *elevation) &&
      ia_lt_flt(*elevation, pstr_prod_screen_data->el_bottom))
  {
    temp = ia_mul_flt(
        (ia_sub_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el,
                    ia_negate_flt(NINETY_DEGREE)) /
         ia_sub_flt((FLOAT32)pstr_prod_screen_data->el_bottom, ia_negate_flt(NINETY_DEGREE))),
        ia_sub_flt(*elevation, ia_negate_flt(NINETY_DEGREE)));
    el_temp = ia_add_flt(temp, ia_negate_flt(NINETY_DEGREE));
  }

  if (ia_lteq_flt((FLOAT32)pstr_prod_screen_data->el_bottom, *elevation) &&
      ia_lt_flt(*elevation, (FLOAT32)pstr_prod_screen_data->el_top))
  {
    temp = ia_mul_flt((height_rep / height_ref),
                      ia_sub_flt(*elevation, pstr_prod_screen_data->el_bottom));
    el_temp = ia_add_flt(temp, (FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el);
  }
  if (ia_lteq_flt(pstr_prod_screen_data->el_top, *elevation) &&
      ia_lt_flt(*elevation, NINETY_DEGREE))
  {
    temp = ia_mul_flt((ia_sub_flt(NINETY_DEGREE, (FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el) /
                       ia_sub_flt(NINETY_DEGREE, (FLOAT32)pstr_prod_screen_data->el_top)),
                      ia_sub_flt(*elevation, (FLOAT32)pstr_prod_screen_data->el_top));
    el_temp = ia_add_flt(temp, (FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el);
  }

  *elevation = el_temp;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_screen_rel_remap_az
 *
 *  \brief Remapping of sreen related azimuth values
 *  \param [in,out] azimuth                        Pointer to azimuth value
 *  \param [in]  pstr_local_setup         Pointer to local setup structure
 *  \param [in,out] pstr_screen_size_data    Pointer to production screen size data structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE
impeghd_screen_rel_remap_az(FLOAT32 *azimuth, ia_local_setup_struct *pstr_local_setup,
                            ia_production_screen_size_data *pstr_screen_size_data)
{
  FLOAT32 az_temp_out = *azimuth;
  FLOAT32 width_repro = 0.0f, width_refer = 0.0f;
  FLOAT32 lcl_scrn_sz_right_az_temp;
  FLOAT32 lcl_scrn_sz_left_az_temp;
  FLOAT32 az_right_temp;
  FLOAT32 az_left_temp;
  FLOAT32 az_temp = 0.0f;
  FLOAT32 temp = 0.0f;

  if (ia_lt_flt((*azimuth), -ONE_EIGHTY_DEGREE) || ia_lt_flt(ONE_EIGHTY_DEGREE, *azimuth))
  {
    return IA_MPEGH_MAE_EXE_FATAL_UNSUPPORTED_AZIMUTH;
  }

  if (ia_lt_flt(ONE_EIGHTY_DEGREE, pstr_screen_size_data->az_left))
  {
    pstr_screen_size_data->az_left =
        ia_sub_flt(pstr_screen_size_data->az_left, THREE_SIXTY_DEGREE);
  }
  if (ia_lt_flt(pstr_screen_size_data->az_left, -ONE_EIGHTY_DEGREE))
  {
    pstr_screen_size_data->az_left =
        ia_add_flt(pstr_screen_size_data->az_left, THREE_SIXTY_DEGREE);
  }
  if (ia_lt_flt(ONE_EIGHTY_DEGREE, pstr_screen_size_data->az_right))
  {
    pstr_screen_size_data->az_right =
        ia_sub_flt(pstr_screen_size_data->az_right, THREE_SIXTY_DEGREE);
  }
  if (ia_lt_flt(pstr_screen_size_data->az_right, -ONE_EIGHTY_DEGREE))
  {
    pstr_screen_size_data->az_right =
        ia_add_flt(pstr_screen_size_data->az_right, THREE_SIXTY_DEGREE);
  }

  if (pstr_local_setup->lcl_scrn_sz_left_az > 180)
  {
    pstr_local_setup->lcl_scrn_sz_left_az = pstr_local_setup->lcl_scrn_sz_left_az - 360;
  }
  if (pstr_local_setup->lcl_scrn_sz_left_az < -180)
  {
    pstr_local_setup->lcl_scrn_sz_left_az = pstr_local_setup->lcl_scrn_sz_left_az + 360;
  }
  if (pstr_local_setup->lcl_scrn_sz_right_az > 180)
  {
    pstr_local_setup->lcl_scrn_sz_right_az = pstr_local_setup->lcl_scrn_sz_right_az - 360;
  }
  if (pstr_local_setup->lcl_scrn_sz_right_az < -180)
  {
    pstr_local_setup->lcl_scrn_sz_right_az = pstr_local_setup->lcl_scrn_sz_right_az + 360;
  }

  if (pstr_local_setup->lcl_scrn_sz_right_az > pstr_local_setup->lcl_scrn_sz_left_az)
  {
    pstr_local_setup->lcl_scrn_offset =
        (THREE_SIXTY_DEGREE + pstr_local_setup->lcl_scrn_sz_left_az +
         pstr_local_setup->lcl_scrn_sz_right_az);
  }
  else
  {
    pstr_local_setup->lcl_scrn_offset =
        (FLOAT32)(pstr_local_setup->lcl_scrn_sz_left_az + pstr_local_setup->lcl_scrn_sz_right_az);
  }
  pstr_local_setup->lcl_scrn_offset = (pstr_local_setup->lcl_scrn_offset) / 2.0f;

  lcl_scrn_sz_right_az_temp =
      pstr_local_setup->lcl_scrn_sz_right_az - pstr_local_setup->lcl_scrn_offset;
  lcl_scrn_sz_left_az_temp =
      pstr_local_setup->lcl_scrn_sz_left_az - pstr_local_setup->lcl_scrn_offset;

  if (lcl_scrn_sz_left_az_temp > ONE_EIGHTY_DEGREE)
  {
    lcl_scrn_sz_left_az_temp = lcl_scrn_sz_left_az_temp - THREE_SIXTY_DEGREE;
  }
  if (lcl_scrn_sz_left_az_temp < -ONE_EIGHTY_DEGREE)
  {
    lcl_scrn_sz_left_az_temp = lcl_scrn_sz_left_az_temp + THREE_SIXTY_DEGREE;
  }
  if (lcl_scrn_sz_right_az_temp > ONE_EIGHTY_DEGREE)
  {
    lcl_scrn_sz_right_az_temp = lcl_scrn_sz_right_az_temp - THREE_SIXTY_DEGREE;
  }
  if (lcl_scrn_sz_right_az_temp < -ONE_EIGHTY_DEGREE)
  {
    lcl_scrn_sz_right_az_temp = lcl_scrn_sz_right_az_temp + THREE_SIXTY_DEGREE;
  }

  if (pstr_screen_size_data->az_right > pstr_screen_size_data->az_left)
  {
    pstr_screen_size_data->offset =
        (THREE_SIXTY_DEGREE + pstr_screen_size_data->az_left + pstr_screen_size_data->az_right);
  }
  else
  {
    pstr_screen_size_data->offset =
        (pstr_screen_size_data->az_left + pstr_screen_size_data->az_right);
  }
  pstr_screen_size_data->offset = pstr_screen_size_data->offset / 2.0f;
  az_right_temp = pstr_screen_size_data->az_right - pstr_screen_size_data->offset;
  az_left_temp = pstr_screen_size_data->az_left - pstr_screen_size_data->offset;

  if (az_left_temp > ONE_EIGHTY_DEGREE)
  {
    az_left_temp = az_left_temp - THREE_SIXTY_DEGREE;
  }
  if (az_left_temp < -ONE_EIGHTY_DEGREE)
  {
    az_left_temp = az_left_temp + THREE_SIXTY_DEGREE;
  }
  if (az_right_temp > ONE_EIGHTY_DEGREE)
  {
    az_right_temp = az_right_temp - THREE_SIXTY_DEGREE;
  }
  if (az_right_temp < -ONE_EIGHTY_DEGREE)
  {
    az_right_temp = az_right_temp + THREE_SIXTY_DEGREE;
  }

  width_repro = ia_fabs_flt(lcl_scrn_sz_left_az_temp - lcl_scrn_sz_right_az_temp);
  width_refer = ia_fabs_flt(az_left_temp - az_right_temp);

  az_temp = *azimuth - pstr_screen_size_data->offset;
  if (az_temp > ONE_EIGHTY_DEGREE)
  {
    az_temp -= THREE_SIXTY_DEGREE;
  }
  if (az_temp < -ONE_EIGHTY_DEGREE)
  {
    az_temp += THREE_SIXTY_DEGREE;
  }

  if ((az_temp >= -ONE_EIGHTY_DEGREE) && (az_temp < (pstr_screen_size_data->az_right)))
  {
    temp = ((lcl_scrn_sz_right_az_temp - (-ONE_EIGHTY_DEGREE)) /
            ((az_right_temp) - (-ONE_EIGHTY_DEGREE))) *
           (az_temp - (-ONE_EIGHTY_DEGREE));
    az_temp_out = temp - ONE_EIGHTY_DEGREE;
  }
  if ((az_temp >= (az_right_temp)) && (az_temp < az_left_temp))
  {
    temp = (width_repro / width_refer) * (az_temp - (az_right_temp));
    az_temp_out = temp + lcl_scrn_sz_right_az_temp;
  }
  if ((az_temp >= az_left_temp) && (az_temp < ONE_EIGHTY_DEGREE))
  {
    temp = ((ONE_EIGHTY_DEGREE - lcl_scrn_sz_left_az_temp) / (ONE_EIGHTY_DEGREE - az_left_temp)) *
           (az_temp - az_left_temp);
    az_temp_out = temp + lcl_scrn_sz_left_az_temp;
  }

  az_temp_out = az_temp_out + pstr_local_setup->lcl_scrn_offset;
  if (az_temp_out > ONE_EIGHTY_DEGREE)
  {
    az_temp_out -= THREE_SIXTY_DEGREE;
  }
  if (az_temp_out < -ONE_EIGHTY_DEGREE)
  {
    az_temp_out += THREE_SIXTY_DEGREE;
  }

  *azimuth = az_temp_out;

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_screen_rel_zooming
 *
 *  \brief Calculate sreen related zooming values
 *
 *  \param [in,out] azimuth                   Pointer to azimuth value
 *  \param [in,out] elevation                 Pointer to elevation value
 *  \param [in]  pstr_ele_interaction_data Pointer to element interaction data structure
 *  \param [in,out] pstr_local_setup         Pointer to localsetup  structure
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_screen_rel_zooming(FLOAT32 *azimuth, FLOAT32 *elevation,
                                               ia_ele_intrctn *pstr_ele_interaction_data,
                                               ia_local_setup_struct *pstr_local_setup)
{
  IA_ERRORCODE error = 0;
  WORD32 za_el_bottom;
  WORD32 za_el_top;
  WORD32 za_az_left;
  WORD32 za_az_right;
  FLOAT32 el_temp = *elevation;
  FLOAT32 az_temp = *azimuth;

  za_az_left = pstr_ele_interaction_data->zoom_az_center + pstr_ele_interaction_data->zoom_az;
  za_az_right = pstr_ele_interaction_data->zoom_az_center - pstr_ele_interaction_data->zoom_az;
  za_el_top = pstr_ele_interaction_data->zoom_el_center + pstr_ele_interaction_data->zoom_el;
  za_el_bottom = pstr_ele_interaction_data->zoom_el_center - pstr_ele_interaction_data->zoom_el;

  if (ia_lt_flt(*elevation, ia_negate_flt(NINETY_DEGREE)) ||
      ia_lt_flt(NINETY_DEGREE, *elevation) || ia_lt_flt(*azimuth, -ONE_EIGHTY_DEGREE) ||
      ia_lt_flt(ONE_EIGHTY_DEGREE, *azimuth))
  {
    error = -1;
  }
  if (error == 0)
  {
    if (ia_lteq_flt(ia_negate_flt(NINETY_DEGREE), *elevation) &&
        ia_lt_flt(*elevation, (FLOAT32)za_el_bottom))
    {
      el_temp =
          ia_sub_flt(ia_mul_flt((ia_add_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el,
                                            NINETY_DEGREE) /
                                 ia_add_flt((FLOAT32)za_el_bottom, NINETY_DEGREE)),
                                ia_add_flt(*elevation, NINETY_DEGREE)),
                     NINETY_DEGREE);
    }
    if (ia_lteq_flt((FLOAT32)za_el_bottom, *elevation) &&
        ia_lt_flt(*elevation, (FLOAT32)za_el_top))
    {
      el_temp =
          ia_add_flt(ia_mul_flt((ia_sub_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el,
                                            (FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el) /
                                 ia_sub_flt((FLOAT32)za_el_top, (FLOAT32)za_el_bottom)),
                                ia_sub_flt(*elevation, (FLOAT32)za_el_bottom)),
                     (FLOAT32)pstr_local_setup->lcl_scrn_sz_bottom_el);
    }
    if (ia_lteq_flt((FLOAT32)za_el_top, *elevation) && ia_lt_flt(*elevation, NINETY_DEGREE))
    {
      el_temp = ia_add_flt(
          ia_mul_flt((ia_sub_flt(NINETY_DEGREE, (FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el) /
                      ia_sub_flt(NINETY_DEGREE, (FLOAT32)za_el_top)),
                     ia_sub_flt(*elevation, (FLOAT32)za_el_top)),
          (FLOAT32)pstr_local_setup->lcl_scrn_sz_top_el);
    }

    if (ia_lteq_flt(-ONE_EIGHTY_DEGREE, *azimuth) && ia_lt_flt(*azimuth, (FLOAT32)za_az_right))
    {
      az_temp = ia_sub_flt(ia_mul_flt((ia_add_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_right_az,
                                                  ONE_EIGHTY_DEGREE) /
                                       ia_add_flt((FLOAT32)za_az_right, ONE_EIGHTY_DEGREE)),
                                      ia_add_flt(*azimuth, ONE_EIGHTY_DEGREE)),
                           ONE_EIGHTY_DEGREE);
    }
    if (ia_lteq_flt((FLOAT32)za_az_right, *azimuth) && ia_lt_flt(*azimuth, (FLOAT32)za_az_left))
    {
      az_temp =
          ia_add_flt(ia_mul_flt((ia_sub_flt((FLOAT32)pstr_local_setup->lcl_scrn_sz_left_az,
                                            (FLOAT32)pstr_local_setup->lcl_scrn_sz_right_az) /
                                 ia_sub_flt((FLOAT32)za_az_left, (FLOAT32)za_az_right)),
                                ia_sub_flt(*azimuth, (FLOAT32)za_az_right)),
                     (FLOAT32)pstr_local_setup->lcl_scrn_sz_right_az);
    }
    if (ia_lteq_flt((FLOAT32)za_az_left, *azimuth) && ia_lt_flt(*azimuth, ONE_EIGHTY_DEGREE))
    {
      az_temp =
          ia_add_flt(ia_mul_flt((ia_sub_flt(ONE_EIGHTY_DEGREE,
                                            (FLOAT32)pstr_local_setup->lcl_scrn_sz_left_az) /
                                 ia_sub_flt(ONE_EIGHTY_DEGREE, (FLOAT32)za_az_left)),
                                ia_sub_flt(*azimuth, (FLOAT32)za_az_left)),
                     (FLOAT32)pstr_local_setup->lcl_scrn_sz_left_az);
    }
  }
  *elevation = el_temp;
  *azimuth = az_temp;
  return error;
}
/**
 *  impeghd_md_proc_apply_scrn_rel_remapping_zooming
 *
 *  \brief Remapping of screen related zooming
 *
 *  \param [in]  pstr_interaction_config Pointer to interaction configuration structure
 *  \param [in]  pstr_asc                Pointer to audio scene config structure
 *  \param [in,out] pstr_dec_data           Pointer to decoder data structure.
 *  \param [in]  pstr_signals_3d          Pointer to sd signal configuration structure
 *  \param [in,out] ptr_scratch_mem         Pointer to scratch memory
 *  \param [in]  sel_preset              Selection preset
 *  \param [in]  oam_grp_idx             Index of OAM group that is to be processed.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_proc_apply_scrn_rel_remapping_zooming(
    ia_interaction_data_struct *pstr_interaction_config,
    ia_audio_specific_config_struct *pstr_asc, ia_dec_data_struct *pstr_dec_data,
    ia_signals_3d *pstr_signals_3d, pWORD8 ptr_scratch_mem, WORD32 sel_preset, WORD32 oam_grp_idx)
{
  IA_ERRORCODE error = 0;
  WORD32 i, k, j;
  pWORD8 ptr_scratch = ptr_scratch_mem;
  (void)ptr_scratch;
  ia_production_screen_size_data str_prod_sceen;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_ele_intrctn *pstr_ei_data = pstr_interaction_config->ptr_ele_interaction_data;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;

  if (pstr_interaction_config->en_scrn_rel_processing) /* local screen info is available +
                                                          screen-relative objects are present */
  {
    /* use the standard production screen in case no preset is chosen or it has no production
     * screen */
    str_prod_sceen.el_top = pstr_mae_asi->screen_size_data.el_top;
    str_prod_sceen.el_bottom = pstr_mae_asi->screen_size_data.el_bottom;
    str_prod_sceen.az_left = pstr_mae_asi->screen_size_data.az_left;
    str_prod_sceen.az_right = pstr_mae_asi->screen_size_data.az_right;
    str_prod_sceen.has_non_std_screen_size =
        pstr_mae_asi->screen_size_data.has_non_std_screen_size;
    if (str_prod_sceen.has_non_std_screen_size == 0)
    {
      str_prod_sceen.el_top = 17.5f;
      str_prod_sceen.el_bottom = -17.5f;
      str_prod_sceen.az_left = 29.0f;
      str_prod_sceen.az_right = -29.0f;
    }

    /* check for off-centered screen extension of standard screen */
    if (pstr_mae_asi->screen_size_ext_data.overwrite_prod_screen_size_data)
    {
      str_prod_sceen.az_right =
          (FLOAT32)pstr_mae_asi->screen_size_ext_data.default_screen_sz_right_az;
      str_prod_sceen.az_left =
          (FLOAT32)pstr_mae_asi->screen_size_ext_data.default_screen_sz_left_az;
    }

    /* preset is chosen -> use preset-dependent screen */
    if (sel_preset > -1)
    {
      WORD32 scrn_idx = 0;
      WORD32 p = 0;
      WORD32 has_preset_depend_scrn = 0;

      for (p = 0; p < pstr_mae_asi->screen_size_ext_data.num_preset_prod_screens; p++)
      {
        if (pstr_mae_asi->screen_size_ext_data.screen_grp_preset_id[p] == sel_preset)
        {
          has_preset_depend_scrn = 1;
          scrn_idx = p;
          break;
        }
      }
      if (has_preset_depend_scrn == 1)
      {
        str_prod_sceen.el_top =
            (FLOAT32)pstr_mae_asi->screen_size_ext_data.screen_sz_top_el[scrn_idx];
        str_prod_sceen.el_bottom =
            (FLOAT32)pstr_mae_asi->screen_size_ext_data.screen_sz_bot_el[scrn_idx];
        str_prod_sceen.az_left =
            (FLOAT32)pstr_mae_asi->screen_size_ext_data.screen_sz_left_az[scrn_idx];
        str_prod_sceen.az_right =
            (FLOAT32)pstr_mae_asi->screen_size_ext_data.screen_sz_right_az[scrn_idx];
        str_prod_sceen.has_non_std_screen_size =
            pstr_mae_asi->screen_size_ext_data.has_non_std_screen_sz[scrn_idx];
      }
    }

    for (i = 0; i < pstr_interaction_config->oam_count; i++)
    {
      pstr_interaction_config->scrn_rel_objs[i] = -1;
      if (pstr_asc->str_usac_config.obj_md_cfg[oam_grp_idx].is_screen_rel_obj[i] == 1)
      {
        pstr_interaction_config->scrn_rel_objs[pstr_interaction_config->list_oam[i]] = 1;
      }
    }

    /* check and correct value ranges */
    str_prod_sceen.el_top =
        ia_min_flt(ia_max_flt(str_prod_sceen.el_top, -NINETY_DEGREE), NINETY_DEGREE);
    str_prod_sceen.el_bottom =
        ia_min_flt(ia_max_flt(str_prod_sceen.el_bottom, -NINETY_DEGREE), NINETY_DEGREE);
    str_prod_sceen.az_left =
        ia_min_flt(ia_max_flt(str_prod_sceen.az_left, -ONE_EIGHTY_DEGREE), ONE_EIGHTY_DEGREE);
    str_prod_sceen.az_right =
        ia_min_flt(ia_max_flt(str_prod_sceen.az_right, -ONE_EIGHTY_DEGREE), ONE_EIGHTY_DEGREE);

    if (pstr_interaction_config->ptr_local_setup_info->has_local_screen_size_info == 1)
    {
      for (i = 0; i < pstr_interaction_config->num_decoded_groups; i++)
      {
        /* check if group is active and not routed to a WIRE output */
        if ((pstr_interaction_config->on_off_status_modified[i] == 1) &&
            (pstr_ei_data->route_to_wire_id[i] == -1))
        {
          /* check if group is applicable for screen-related processing (object-based with OAM
           * data) */
          if (pstr_signals_3d->group_type[i] == 1) /* not possible for SAOC at the moment */
          {
            WORD32 member_id = -1;
            WORD32 oam_index = -1;
            WORD32 num_members = pstr_asc->str_mae_asi.group_definition[i].group_num_members;

            for (k = 0; k < num_members; k++)
            {
              if (!(pstr_asc->str_mae_asi.group_definition[i].has_conjunct_members))
              {
                member_id = pstr_asc->str_mae_asi.group_definition[i].metadata_ele_id[k];
              }
              else
              {
                member_id = pstr_asc->str_mae_asi.group_definition[i].start_id + k;
              }

              /* get OAM data for the current group member */
              for (j = 0; j < pstr_interaction_config->oam_count; j++)
              {
                if (pstr_interaction_config->list_oam[j] == member_id)
                {
                  oam_index = j;
                  break;
                }
              }
              if (oam_index == -1)
              {
                return IA_MPEGH_MAE_EXE_FATAL_INVALID_OAM_INDEX;
              }
              pstr_interaction_config->az_modified[k] =
                  pstr_oam_dec_state->azimuth_descaled[oam_index];
              pstr_interaction_config->ele_modified[k] =
                  pstr_oam_dec_state->elevation_descaled[oam_index];
              if (pstr_interaction_config->scrn_rel_objs[member_id] == 1)
              {
                /* screen related remapping */
                error = impeghd_screen_rel_remap_az(&pstr_interaction_config->az_modified[k],
                                                    pstr_interaction_config->ptr_local_setup_info,
                                                    &pstr_mae_asi->screen_size_data);
                if (error)
                {
                  return error;
                }
                if (pstr_interaction_config->ptr_local_setup_info->has_lcl_scrn_elevation_info ==
                    1)
                {
                  error =
                      impeghd_screen_rel_remap_ele(&pstr_interaction_config->ele_modified[k],
                                                   pstr_interaction_config->ptr_local_setup_info,
                                                   &pstr_mae_asi->screen_size_data);
                  if (error)
                  {
                    return error;
                  }
                }

                /* screen related zooming */
                if (pstr_interaction_config->ptr_ele_interaction_data != NULL)
                {
                  if (pstr_interaction_config->ptr_ele_interaction_data->has_zoom_area_sz == 1)
                  {
                    if (pstr_interaction_config->ptr_local_setup_info
                            ->has_lcl_scrn_elevation_info ==
                        1) /* apply zooming only if local screen info is known */
                    {
                      error = impeghd_screen_rel_zooming(
                          &pstr_interaction_config->az_modified[k],
                          &pstr_interaction_config->ele_modified[k],
                          pstr_interaction_config->ptr_ele_interaction_data,
                          pstr_interaction_config->ptr_local_setup_info);
                      if (error)
                      {
                        return error;
                      }
                    }
                  }
                }
              }
              pstr_oam_dec_state->elevation_descaled[oam_index] =
                  pstr_interaction_config->ele_modified[k];
              pstr_oam_dec_state->azimuth_descaled[oam_index] =
                  pstr_interaction_config->az_modified[k];
            }
          }
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_md_apply_gain_interactivity
 *
 *  \brief Gain interactivity applying is carried out.
 *
 *  \param [in]  pstr_intrct_data Pointer to interaction configuration structure.
 *  \param [in]  pstr_asc         Pointer to audio scene config structure.
 *  \param [in,out] pstr_dec_data    Pointer to decoder data structure.
 *  \param [in]  pstr_signals_3d  Pointer to scratch memory
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_apply_gain_interactivity(
    ia_interaction_data_struct *pstr_intrct_data, ia_audio_specific_config_struct *pstr_asc,
    ia_dec_data_struct *pstr_dec_data, ia_signals_3d *pstr_signals_3d)
{
  WORD32 i, j, k;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  for (i = 0; i < pstr_intrct_data->num_decoded_groups; i++)
  {
    /* set everything to zero if group is switched off */
    WORD32 num_members = pstr_mae_asi->group_definition[i].group_num_members;
    WORD32 member_id = -1;
    if (pstr_intrct_data->on_off_status_modified[i] == 0)
    {
      for (k = 0; k < num_members; k++)
      {
        if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
        {
          member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
        }
        else
        {

          member_id = pstr_mae_asi->group_definition[i].start_id + k;
        }

        memset(&pstr_dec_data->str_usac_data.time_sample_vector[member_id][0], 0,
               sizeof(pstr_dec_data->str_usac_data.time_sample_vector[member_id][j]) *
                   pstr_dec_data->str_usac_data.ccfl);
      }
    }
    if (pstr_intrct_data->process[i] == 1)
    {
      if (pstr_intrct_data->apply_intrct_data == 1)
      {
        /* only apply gain for channel-based elements */
        if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 0)
        {
          FLOAT32 current_gain_db = pstr_intrct_data->gain_modified_grp[i];

          if (current_gain_db != -1.0f)
          {
            FLOAT32 prev_gain_db = pstr_intrct_data->gain_modified_prev_frm[i];
            for (k = 0; k < num_members; k++)
            {
              FLOAT32 initial_gain = (FLOAT32)pow(10.0f, prev_gain_db / 20.0f);
              FLOAT32 gain_diff = (FLOAT32)pow(10.0f, current_gain_db / 20.0f) - initial_gain;
              FLOAT32 gain_step = gain_diff / ((pstr_dec_data->str_usac_data.ccfl - 1));
              FLOAT32 gain_factor_applied = initial_gain;
              if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
              {
                member_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
              }
              else
              {
                member_id = pstr_mae_asi->group_definition[i].start_id + k;
              }

              /* interpolate between lastGain and currentGain over the interval of 1 frame */

              for (j = 0; j < pstr_dec_data->str_usac_data.ccfl; j++)
              {
                pstr_dec_data->str_usac_data.time_sample_vector[member_id][j] =
                    ia_mul_flt(pstr_dec_data->str_usac_data.time_sample_vector[member_id][j],
                               gain_factor_applied);
                gain_factor_applied += gain_step;
              }
            }
          }
        }
      }
    }
    pstr_intrct_data->gain_modified_prev_frm[i] = pstr_intrct_data->gain_modified_grp[i];
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_md_proc_max_mae_id
 *
 *  \brief Finds maximum MAE ID value.
 *
 *  \param [in]  pstr_asc Pointer to audio scene configuration structure.
 *
 *  \return WORD32        Maximum MAE ID value.
 *
 */
static WORD32 impeghd_md_proc_max_mae_id(ia_audio_specific_config_struct *pstr_asc)
{
  WORD32 i, j;
  WORD32 mae_id;
  WORD32 mae_max_id = -1;
  WORD32 num_groups = pstr_asc->str_mae_asi.num_groups;
  for (i = 0; i < num_groups; i++)
  {
    for (j = 0; j < pstr_asc->str_mae_asi.group_definition[i].group_num_members; j++)
    {
      if (pstr_asc->str_mae_asi.group_definition[i].has_conjunct_members != 1)
      {
        mae_id = pstr_asc->str_mae_asi.group_definition[i].metadata_ele_id[j];
      }
      else
      {
        mae_id = pstr_asc->str_mae_asi.group_definition[i].start_id + j;
      }

      if (mae_id > mae_max_id)
      {
        mae_max_id = mae_id;
      }
    }
  }
  return mae_max_id;
}

/**
 *  impeghd_md_proc_apply_div_proc
 *
 *  \brief Applies divergence processing.
 *
 *  \param [in,out] pstr_interact_cfg          Pointer to interaction configuration structure.
 *  \param [in]  pstr_asc                   Pointer to audio scene configuration structure.
 *  \param [in,out] overall_num_div_objs_added Pointer to overall number of divergence objects
 * added.
 *  \param [in]  pstr_enh_obj_md_cfg        Pointer to enhanced object metadata config structure.
 *  \param [in]  pstr_enh_obj_md_frame      Pointer to enhanced object metadata frame structure.
 *  \param [in]  pstr_signals_3d Pointer to 3d signals depiction structure.
 *  \param [in]  pstr_dec_data              Pointer to decoder data structure.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_md_proc_apply_div_proc(ia_interaction_data_struct *pstr_interact_cfg,
                                                   ia_audio_specific_config_struct *pstr_asc,
                                                   WORD32 *overall_num_div_objs_added,
                                                   ia_enh_oam_config_struct *pstr_enh_obj_md_cfg,
                                                   ia_enh_obj_md_frame_str *pstr_enh_obj_md_frame,
                                                   ia_signals_3d *pstr_signals_3d,
                                                   ia_dec_data_struct *pstr_dec_data,
                                                   WORD32 obj_grp_idx)
{
  WORD32 i, k, m;
  WORD32 n;
  WORD32 ct = 0;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  ia_oam_dec_config_struct *pstr_oam_dec_cfg = &pstr_asc->str_usac_config.obj_md_cfg[obj_grp_idx];

  if (pstr_interact_cfg->num_objs_out > pstr_interact_cfg->num_objs_in)
  {
    if (pstr_interact_cfg->divergence_asi_modified == 0)
    {
      for (i = 0; i < pstr_mae_asi->num_groups; i++)
      {
        pstr_interact_cfg->orig_num_grp_members[i] =
            pstr_mae_asi->group_definition[i].group_num_members;
      }
    }

    *overall_num_div_objs_added = 0;
    for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
    {
      WORD32 new_oam_size = pstr_interact_cfg->oam_count;
      WORD32 screen_rel_status_orig = 0;
      WORD32 count = 0;
      WORD32 max_mae_id = 0;
      WORD32 source_el, mae_id = 0;
      WORD32 has_divergence = 0;
      FLOAT32 divergence_az = ZERO_DEGREE;
      FLOAT32 divergence_gain_orig = 0.0f;
      FLOAT32 temp_az = ZERO_DEGREE;
      FLOAT32 temp_gain_orig = 1.0f, temp_gain_virt = 1.0f, mod_gain_orig = 1.0f,
              mod_gain_virt = 1.0f;
      FLOAT32 divergence = ZERO_DEGREE;
      if (impeghd_md_proc_signal_group_type(pstr_asc, pstr_signals_3d, i) == 1)
      {
        if (pstr_interact_cfg->divergence_asi_modified != 1)
        {
          n = pstr_mae_asi->group_definition[i].group_num_members;
        }
        else
        {
          n = pstr_interact_cfg->orig_num_grp_members[i];
        }

        for (k = 0; k < n; k++)
        {
          has_divergence = pstr_enh_obj_md_cfg->has_divergence[ct];
          if (!(pstr_mae_asi->group_definition[i].has_conjunct_members))
          {
            mae_id = pstr_mae_asi->group_definition[i].metadata_ele_id[k];
          }
          else
          {
            mae_id = pstr_mae_asi->group_definition[i].start_id + k;
          }

          if (has_divergence)
          {
            for (m = 0; m < pstr_interact_cfg->oam_count; m++)
            {
              if (pstr_interact_cfg->list_oam[m] == mae_id)
              {
                screen_rel_status_orig = pstr_oam_dec_cfg->is_screen_rel_obj[m];
                divergence_gain_orig = pstr_oam_dec_state->gain_descaled[m];
                divergence_az = pstr_oam_dec_state->azimuth_descaled[m];
                break;
              }
            }

            temp_az =
                ia_min_flt(3 * pstr_enh_obj_md_cfg->divergence_az_range[ct], ONE_EIGHTY_DEGREE);
            temp_az = ia_sub_flt(divergence_az, temp_az);
            if (ia_lt_flt(ONE_EIGHTY_DEGREE, temp_az))
            {
              temp_az = ia_sub_flt(temp_az, THREE_SIXTY_DEGREE);
            }
            if (ia_lt_flt(temp_az, -ONE_EIGHTY_DEGREE))
            {
              temp_az = ia_add_flt(temp_az, THREE_SIXTY_DEGREE);
            }
            divergence = pstr_enh_obj_md_frame->divergence[ct];

            temp_gain_orig =
                (FLOAT32)ia_sqrt_flt(ia_sub_flt(1.0f, (FLOAT32)pow(divergence, 0.58497f)));
            mod_gain_orig = ia_mul_flt(divergence_gain_orig, temp_gain_orig);
            temp_gain_virt =
                (FLOAT32)ia_sqrt_flt(ia_mul_flt(0.5f, ((FLOAT32)pow(divergence, 0.58497f))));
            mod_gain_virt = ia_mul_flt(divergence_gain_orig, temp_gain_virt);

            pstr_oam_dec_state->gain_descaled[m] = mod_gain_orig;

            /* add first virtual object */
            pstr_oam_dec_state->spread_width[pstr_interact_cfg->num_objs_in + count] =
                pstr_oam_dec_state->spread_width[m];
            pstr_oam_dec_state->spread_depth[pstr_interact_cfg->num_objs_in + count] =
                pstr_oam_dec_state->spread_depth[m];
            pstr_oam_dec_state->spread_height[pstr_interact_cfg->num_objs_in + count] =
                pstr_oam_dec_state->spread_height[m];
            pstr_oam_dec_state->elevation_descaled[pstr_interact_cfg->num_objs_in + count] =
                pstr_oam_dec_state->elevation_descaled[m];
            pstr_oam_dec_state->azimuth_descaled[pstr_interact_cfg->num_objs_in + count] =
                temp_az;
            pstr_oam_dec_state->gain_descaled[pstr_interact_cfg->num_objs_in + count] =
                mod_gain_virt;
            pstr_oam_dec_state->sample[pstr_interact_cfg->num_objs_in + count] =
                pstr_oam_dec_state->sample[m];
            pstr_oam_dec_state
                ->dyn_obj_priority_descaled[pstr_interact_cfg->num_objs_in + count] =
                pstr_oam_dec_state->dyn_obj_priority_descaled[m];

            (*overall_num_div_objs_added)++;
            max_mae_id = impeghd_md_proc_max_mae_id(pstr_asc);
            pstr_interact_cfg
                ->list_oam[pstr_interact_cfg->oam_count - 1 + *overall_num_div_objs_added] =
                max_mae_id + *overall_num_div_objs_added;

            /* update group definition, part 1*/
            if (pstr_interact_cfg->divergence_asi_modified == 0)
            {
              if (pstr_mae_asi->group_definition[i].has_conjunct_members)
              {
                pstr_mae_asi->group_definition[i].metadata_ele_id[k] =
                    pstr_mae_asi->group_definition[i].start_id + k;
              }
              pstr_mae_asi->group_definition[i]
                  .metadata_ele_id[pstr_mae_asi->group_definition[i].group_num_members] =
                  max_mae_id + *overall_num_div_objs_added;

              /* update screen-relative object list */
              pstr_oam_dec_cfg->is_screen_rel_obj[pstr_interact_cfg->oam_count - 1 +
                                                  *overall_num_div_objs_added] =
                  screen_rel_status_orig;
            }

            /* duplicate audio data */
            source_el = mae_id;
            memcpy(&pstr_dec_data->str_usac_data
                        .time_sample_vector[pstr_interact_cfg->num_ele_in - 1 +
                                            *overall_num_div_objs_added][0],
                   &pstr_dec_data->str_usac_data.time_sample_vector[source_el][0],
                   sizeof(pstr_dec_data->str_usac_data.time_sample_vector[source_el][0]) *
                       pstr_dec_data->str_usac_data.ccfl);

            /* add second virtual object */
            temp_az =
                ia_min_flt(3 * pstr_enh_obj_md_cfg->divergence_az_range[ct], ONE_EIGHTY_DEGREE);
            temp_az = ia_add_flt(divergence_az, temp_az);
            if (ia_lt_flt(ONE_EIGHTY_DEGREE, temp_az))
            {
              temp_az = ia_sub_flt(temp_az, THREE_SIXTY_DEGREE);
            }
            if (ia_lt_flt(temp_az, -ONE_EIGHTY_DEGREE))
            {
              temp_az = ia_add_flt(temp_az, THREE_SIXTY_DEGREE);
            }
            pstr_oam_dec_state
                ->spread_width_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                pstr_oam_dec_state->spread_width_descaled[m];
            pstr_oam_dec_state
                ->spread_depth_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                pstr_oam_dec_state->spread_depth_descaled[m];
            pstr_oam_dec_state
                ->spread_height_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                pstr_oam_dec_state->spread_height_descaled[m];
            pstr_oam_dec_state->elevation_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                pstr_oam_dec_state->elevation_descaled[m];
            pstr_oam_dec_state->azimuth_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                temp_az;
            pstr_oam_dec_state->gain_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                mod_gain_virt;
            pstr_oam_dec_state->sample[pstr_interact_cfg->num_objs_in + count + 1] =
                pstr_oam_dec_state->sample[m];
            pstr_oam_dec_state
                ->dyn_obj_priority_descaled[pstr_interact_cfg->num_objs_in + count + 1] =
                pstr_oam_dec_state->dyn_obj_priority_descaled[m];

            (*overall_num_div_objs_added)++;
            pstr_interact_cfg
                ->list_oam[pstr_interact_cfg->oam_count - 1 + *overall_num_div_objs_added] =
                max_mae_id + *overall_num_div_objs_added;

            if (pstr_interact_cfg->divergence_asi_modified == 0)
            {
              /* update group definition, part 2*/
              pstr_mae_asi->group_definition[i]
                  .metadata_ele_id[pstr_mae_asi->group_definition[i].group_num_members + 1] =
                  max_mae_id + *overall_num_div_objs_added;

              /* update screen-relative object list */
              pstr_oam_dec_cfg->is_screen_rel_obj[pstr_interact_cfg->oam_count - 1 +
                                                  *overall_num_div_objs_added] =
                  screen_rel_status_orig;
            }

            /* duplicate audio data */
            source_el = mae_id;
            memcpy(&pstr_dec_data->str_usac_data
                        .time_sample_vector[pstr_interact_cfg->num_ele_in - 1 +
                                            *overall_num_div_objs_added][0],
                   &pstr_dec_data->str_usac_data.time_sample_vector[source_el][0],
                   sizeof(pstr_dec_data->str_usac_data.time_sample_vector[source_el][0]) *
                       pstr_dec_data->str_usac_data.ccfl);
            count += 2;
            ct++;
          }
        }
        /* update size information of OAM multidata */
        new_oam_size += count;
        pstr_interact_cfg->oam_count = new_oam_size;

        if (pstr_interact_cfg->divergence_asi_modified == 0)
        {
          /* final update group definition */
          pstr_mae_asi->group_definition[i].has_conjunct_members = 0;
          pstr_mae_asi->group_definition[i].group_num_members += count;
        }
      }
    }
    if ((pstr_interact_cfg->num_objs_in + *overall_num_div_objs_added) !=
        pstr_interact_cfg->num_objs_out)
    {
      return IA_MPEGH_MAE_EXE_FATAL_DIVERGENCE_ERROR;
    }
    if (pstr_interact_cfg->divergence_asi_modified == 0)
    {
      pstr_interact_cfg->divergence_asi_modified = 1;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_md_proc_local_scrn_cfg
 *
 *  \brief Fetches the local screen configuration information.
 *
 *  \param [in] pstr_asc               Pointer to audio specific configuration structure.
 *  \param [in] pstr_local_interaction Pointer to local setup structure.
 *
 *  \return Flag indicating enabling or disabling of screen related processing.
 *
 */
static WORD32 impeghd_md_proc_local_scrn_cfg(ia_audio_specific_config_struct *pstr_asc,
                                             ia_local_setup_struct *pstr_local_interaction,
                                             WORD32 obj_grp_idx)
{
  WORD32 i;
  WORD32 has_screen_relative_objects = 0;
  ia_oam_dec_config_struct *pstr_oam_dec_config = &pstr_asc->str_usac_config.obj_md_cfg[obj_grp_idx];
  for (i = 0; i < MAX_NUM_OAM_OBJS; i++)
  {
    if (pstr_oam_dec_config->is_screen_rel_obj[i] == 1)
    {
      has_screen_relative_objects = 1;
      break;
    }
  }

  return (has_screen_relative_objects && pstr_local_interaction->has_local_screen_size_info);
}

/**
 *  impeghd_md_proc_loudness_comp_gain
 *
 *  \brief Calculate loudness compensation gain
 *
 *  \param [in]  pstr_asc                 Pointer to audio scene config structure
 *  \param [in,out] pstr_interact_cfg        Pointer to interaction configuration structure
 *
 *  \return FLOAT32 loudness compesation gain in db.
 *
 */
static FLOAT32 impeghd_md_proc_loudness_comp_gain(ia_audio_specific_config_struct *pstr_asc,
                                                  ia_interaction_data_struct *pstr_interact_cfg)
{

  WORD32 n = 0, c = 0, gp = 0;
  WORD32 grp_loudness_value_missing_flag = 0;
  WORD32 num_groups;
  WORD32 num_group_presets;
  WORD32 sel_preset = -1;
  WORD32 group_state_default[MAE_MAX_NUM_GROUPS] = {1};
  WORD32 group_state_interactivity[MAE_MAX_NUM_GROUPS] = {1};
  WORD32 include_group[MAE_MAX_NUM_GROUPS] = {1};
  WORD32 group_gain_default_db[MAE_MAX_NUM_GROUPS] = {0};
  WORD32 group_gain_interactivity_db[MAE_MAX_NUM_GROUPS] = {0};
  WORD8 *groupLoudness;
  FLOAT32 min_gain_db = -1000.f;
  FLOAT32 max_gain_db = 21.f;
  FLOAT32 tmp1 = 0.f, tmp2 = 0.f;
  FLOAT32 loudness_ref = 0.f;
  FLOAT32 loudness_post_interactivity = 0.f;
  FLOAT32 loudness_comp_gain_db = 0.f;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_loudness_compensation_data *pstr_loudness_comp = &pstr_mae_asi->loud_comp_data;
  groupLoudness = &pstr_loudness_comp->group_loudness[0];
  num_groups = pstr_mae_asi->num_groups;
  num_group_presets = pstr_mae_asi->num_group_presets;
  if (pstr_loudness_comp->group_loudness_present == 1)
  {
    for (n = 0; n < num_groups; n++)
    {
      group_state_default[n] = pstr_mae_asi->group_definition[n].default_on_off;
      group_state_interactivity[n] = pstr_interact_cfg->ptr_ele_interaction_data->ei_on_off[n];
      group_gain_interactivity_db[n] = pstr_interact_cfg->ptr_ele_interaction_data->ei_gain[n];
    }

    if (pstr_interact_cfg->ptr_ele_interaction_data->ei_intrctn_mode != 1)
    {
      if (pstr_loudness_comp->default_params_present == 1)
      {
        for (n = 0; n < num_groups; n++)
        {
          include_group[n] = pstr_loudness_comp->default_include_group[n];
        }
      }
      if (pstr_loudness_comp->default_min_max_gain_present == 1)
      {
        min_gain_db = (FLOAT32)pstr_loudness_comp->default_min_gain;
        max_gain_db = (FLOAT32)pstr_loudness_comp->default_max_gain;
      }
    }
    else
    {
      sel_preset = pstr_interact_cfg->ptr_ele_interaction_data->ei_group_preset_id;

      for (gp = 0; gp < num_group_presets; gp++)
      {
        if (pstr_mae_asi->group_presets_definition[gp].group_id == sel_preset)
        {
          break;
        }
      }
      if (gp < num_group_presets)
      {
        for (c = 0; c < pstr_mae_asi->group_presets_definition[gp].num_conditions; c++)
        {
          for (n = 0; n < num_groups; n++)
          {
            if (pstr_mae_asi->group_presets_definition[gp].reference_id[c] ==
                pstr_mae_asi->group_definition[n].group_id)
            {
              break;
            }
          }
          if (n < num_groups)
          {
            include_group[n] = pstr_mae_asi->group_presets_definition[gp].cond_on_off[c];
            if (pstr_mae_asi->group_presets_definition[gp].gain_flag[c])
            {
              group_gain_default_db[n] = pstr_mae_asi->group_presets_definition[gp].gain[c];
            }
          }
        }
        if (pstr_loudness_comp->preset_params_present[gp] == 1)
        {
          for (n = 0; n < num_groups; n++)
          {
            include_group[n] = pstr_loudness_comp->preset_include_group[gp][n];
          }
        }
        else
        {
          if (pstr_loudness_comp->default_params_present == 1)
          {
            for (n = 0; n < num_groups; n++)
            {
              include_group[n] = pstr_loudness_comp->default_include_group[n];
            }
          }
        }
        if (pstr_loudness_comp->preset_min_max_gain_present[gp] == 1)
        {
          min_gain_db = pstr_loudness_comp->preset_min_gain[gp];
          max_gain_db = pstr_loudness_comp->preset_max_gain[gp];
        }
        else
        {
          if (pstr_loudness_comp->default_min_max_gain_present == 1)
          {
            min_gain_db = (FLOAT32)pstr_loudness_comp->default_min_gain;
            max_gain_db = (FLOAT32)pstr_loudness_comp->default_max_gain;
          }
        }
      }
      else
      {
        if (pstr_loudness_comp->default_params_present == 1)
        {
          for (n = 0; n < num_groups; n++)
          {
            include_group[n] = pstr_loudness_comp->default_include_group[n];
          }
        }
        if (pstr_loudness_comp->default_min_max_gain_present == 1)
        {
          min_gain_db = (FLOAT32)pstr_loudness_comp->default_min_gain;
          max_gain_db = (FLOAT32)pstr_loudness_comp->default_max_gain;
        }
      }
    }

    if (!(pstr_loudness_comp->group_loudness_present))
    {
      grp_loudness_value_missing_flag = 1;
    }
    else
    {
      groupLoudness = pstr_loudness_comp->group_loudness;
    }

    /* compute components of loudness compensation gain */
    for (n = 0; n < num_groups; n++)
    {
      if (grp_loudness_value_missing_flag != 0)
      {
        tmp2 = powf(10.0f, (FLOAT32)group_gain_interactivity_db[n] / 10.0f);
        tmp1 = powf(10.0f, (FLOAT32)group_gain_default_db[n] / 10.0f);
      }
      else
      {
        tmp2 =
            powf(10.0f,
                 ia_add_flt((FLOAT32)group_gain_interactivity_db[n], (FLOAT32)groupLoudness[n]) /
                     10.0f);
        tmp1 = powf(10.0f,
                    ia_add_flt((FLOAT32)group_gain_default_db[n], (FLOAT32)groupLoudness[n]) /
                        10.0f);
      }
      loudness_ref = ia_mac_flt(
          loudness_ref, ia_mul_flt((FLOAT32)include_group[n], (FLOAT32)group_state_default[n]),
          tmp1);
      loudness_post_interactivity = ia_mac_flt(
          loudness_post_interactivity,
          ia_mul_flt((FLOAT32)include_group[n], (FLOAT32)group_state_interactivity[n]), tmp2);
    }

    /* loudness compensation gain in dB */
    loudness_comp_gain_db =
        ia_mul_flt(10.0f, (FLOAT32)log10(loudness_ref / loudness_post_interactivity));

    /* clip loudness compensation gain to min/max gain */
    if (ia_lt_flt(loudness_comp_gain_db, min_gain_db))
    {
      loudness_comp_gain_db = min_gain_db;
    }
    if (ia_lt_flt(max_gain_db, loudness_comp_gain_db))
    {
      loudness_comp_gain_db = max_gain_db;
    }
  }
  return loudness_comp_gain_db;
}

/**
 *  impeghd_mp_process_frame
 *
 *  \brief Carries out processing on audio data based on metadata elements.
 *
 *  \param [in]  pstr_asc                 Pointer to audio scene config structure
 *  \param [in,out] pstr_dec_data            Pointer to decoder data structure
 *  \param [in]  ptr_scratch_mem          Pointer to scratch memory
 *  \param [in]  num_out_channels         Number of output channels
 *  \param [in,out] selected_preset_id       Pointer to selected preset id
 *  \param [in]  choose_switch_grp_member SG member
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
static IA_ERRORCODE impeghd_mp_process_frame(ia_audio_specific_config_struct *pstr_asc,
                                             ia_dec_data_struct *pstr_dec_data,
                                             WORD8 *ptr_scratch_mem, WORD32 num_out_channels,
                                             WORD32 *selected_preset_id,
                                             WORD32 choose_switch_grp_member)
{
  /* apply interaction data */
  /* apply scene displacement data */
  WORD32 enable_obj_op_interface = 0;
  IA_ERRORCODE error = 0;
  WORD32 i;
  WORD32 sel_preset = -1;
  WORD32 overall_num_div_objs_added = 0;
  pWORD8 ptr_scratch = ptr_scratch_mem;
  WORD32 add_objects;
  WORD32 obj_grp_idx = 0;
  ia_oam_dec_state_struct *pstr_oam_dec_state =
      &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state;
  ia_enh_oam_config_struct *pstr_enh_oam_cfg = &pstr_asc->str_usac_config.enh_obj_md_cfg;
  ia_enh_obj_md_frame_str *pstr_enh_oam_frame = &pstr_dec_data->str_enh_obj_md_frame;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  ia_ele_intrctn *pstr_ei_data = &pstr_dec_data->str_element_interaction;
  ia_interaction_data_struct *pstr_interact_cfg = &pstr_dec_data->str_interaction_config;
  pstr_interact_cfg->ptr_ele_interaction_data = pstr_ei_data;
  ia_signals_3d *pstr_signals_3d = &pstr_asc->str_usac_config.signals_3d;
  ia_scene_disp_data *pstr_scene_displacement = &pstr_dec_data->str_scene_displacement;
  ia_oam_dec_config_struct *pstr_oam_cfg = 
      &pstr_asc->str_usac_config.obj_md_cfg[obj_grp_idx];
  add_objects = (pstr_enh_oam_cfg->num_obj_with_divergence << 1);
  (void)num_out_channels;

  pstr_interact_cfg->apply_intrct_data = 0;
  if (!pstr_ei_data->ei_data_present)
  {
    /* Element interaction data is not available */
    pstr_interact_cfg->apply_intrct_data = 0;

    if (*selected_preset_id < 0) /* preset selection via commandline */
    {
      if (pstr_mae_asi->num_group_presets <= 0)
      {
        pstr_interact_cfg->dflt_preset_mode = 0;
      }
      else
      {
        pstr_interact_cfg->dflt_preset_mode = 1;
      }
    }
    else
    {
      pstr_interact_cfg->dflt_preset_mode = 0;
    }
  }
  else
  {
    /* pstr_ei_data - structure containing element interaction data*/
    pstr_interact_cfg->apply_intrct_data = 1;
    pstr_interact_cfg->interaction_type = pstr_ei_data->ei_intrctn_mode;
    if (pstr_interact_cfg->interaction_type != 0)
    {
      if (pstr_mae_asi->num_group_presets > 0)
      {
        pstr_interact_cfg->dflt_preset_mode = 0;
      }
      else
      {
        pstr_interact_cfg->dflt_preset_mode = 0;
        /*
         * Basic interaction mode is chosen,
         * however the number of presets is not bigger than 0.
         */
        return IA_MPEGH_MAE_EXE_FATAL_UNSUPPORTED_GROUP_PRESET;
      }
    }
    else
    {
      if (pstr_mae_asi->num_group_presets > 0)
      {
        /*
         * Advanced interaction mode is chosen,
         * however the number of presets is bigger than 0.
         * This is not allowed. Default preset is selected.
        */
        pstr_interact_cfg->interaction_type = 1;

        /* Do not apply interaction data */
        pstr_interact_cfg->apply_intrct_data = 0;

        pstr_interact_cfg->dflt_preset_mode = 1;
      }
      else
      {
        pstr_interact_cfg->dflt_preset_mode = 0;
      }
    }
  }

  if ((pstr_interact_cfg->apply_intrct_data == 1) &&
      (pstr_ei_data->ei_num_groups != pstr_mae_asi->num_groups))
  {
    /*
     * Number of groups in ASI and interaction data is not the same.
    */
    return IA_MPEGH_MAE_EXE_FATAL_ASI_NUM_GROUP_DIFFER;
  }

  /*************** init of processing ********************/
  pstr_interact_cfg->num_decoded_groups = pstr_mae_asi->num_groups;
  pstr_interact_cfg->num_switch_groups = pstr_mae_asi->num_switch_groups;
  pstr_interact_cfg->num_ele_in = 0;
  if (pstr_oam_cfg->num_objects > 0)
  {
    pstr_interact_cfg->num_objs_out = pstr_oam_cfg->num_objects + add_objects;
  }

  for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
  {
    if (pstr_interact_cfg->divergence_asi_modified == 1)
    {
      pstr_interact_cfg->num_ele_in += pstr_interact_cfg->orig_num_grp_members[i];
    }
    else
    {
      pstr_interact_cfg->num_ele_in += pstr_mae_asi->group_definition[i].group_num_members;
    }
  }

  for (i = 0; i < pstr_mae_asi->num_groups; i++)
  {
    pstr_interact_cfg->on_off_status_modified[i] = 0;
    pstr_interact_cfg->gain_modified_grp[i] = 0.0f;
  }

  pstr_interact_cfg->en_scrn_rel_processing =
      impeghd_md_proc_local_scrn_cfg(pstr_asc, &pstr_dec_data->str_local_setup_interaction, obj_grp_idx);

  if (pstr_interact_cfg->apply_intrct_data != 0)
  {
    if (pstr_interact_cfg->dflt_preset_mode == 0)
    {
      error = impeghd_apply_on_off_intrct(pstr_interact_cfg, pstr_asc,
                                          pstr_dec_data->str_local_setup_interaction.dmx_id,
                                          &sel_preset);
      if (error)
      {
        return error;
      }
      for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
      {
        if ((pstr_ei_data->route_to_wire_id[i] > -1) ||
            (pstr_interact_cfg->on_off_status_modified[i] == 1))
        {
          pstr_interact_cfg->process[i] = 1;
        }
      }

      pstr_interact_cfg->loudness_comp_gain =
          impeghd_md_proc_loudness_comp_gain(pstr_asc, pstr_interact_cfg);
    }
  }
  else
  {
    if (pstr_interact_cfg->dflt_preset_mode != 1)
    {
      if (*selected_preset_id >= 0)
      {
        sel_preset = *selected_preset_id;
        error = impeghd_apply_on_off_intrct(pstr_interact_cfg, pstr_asc,
                                            pstr_dec_data->str_local_setup_interaction.dmx_id,
                                            &sel_preset);
        if (error)
        {
          return error;
        }
        for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
        {
          if (pstr_interact_cfg->on_off_status_modified[i] == 1)
          {
            pstr_interact_cfg->process[i] = 1;
          }
        }
      }
      else
      {
        for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
        {
          pstr_interact_cfg->process[i] = pstr_mae_asi->group_definition[i].default_on_off;
          pstr_interact_cfg->on_off_status_modified[i] =
              pstr_mae_asi->group_definition[i].default_on_off;
        }
        error = impeghd_md_proc_apply_switch_grp_logic(pstr_interact_cfg, pstr_asc);
        if (error)
        {
          return error;
        }
      }
    }
    else
    {
      error = impeghd_apply_on_off_intrct(pstr_interact_cfg, pstr_asc,
                                          pstr_dec_data->str_local_setup_interaction.dmx_id,
                                          &sel_preset);
      if (error)
      {
        return error;
      }
      for (i = 0; i < pstr_interact_cfg->num_decoded_groups; i++)
      {
        if (pstr_interact_cfg->on_off_status_modified[i] == 1)
        {
          pstr_interact_cfg->process[i] = 1;
        }
      }
    }
  }

  *selected_preset_id = sel_preset;

  if (choose_switch_grp_member > 0)
  {
    WORD32 m;
    WORD32 group_id;
    WORD32 grp_index;

    /* assumtion: Only 1 Switch group in bitstream (interaction with 1st switch group) */
    if (pstr_mae_asi->num_switch_groups == 1)
    {
      WORD32 sg_num_members = pstr_mae_asi->switch_group_definition[0].group_num_members;

      if ((choose_switch_grp_member > sg_num_members - 1) || (choose_switch_grp_member < 0))
      {
        /* error */
      }
      else
      {
        for (m = 0; m < sg_num_members; m++)
        {
          group_id = pstr_mae_asi->switch_group_definition[0].member_id[m];
          grp_index = impeghd_md_proc_grp_idx(pstr_asc, group_id);
          if (m == choose_switch_grp_member)
          {
            pstr_mae_asi->group_definition[grp_index].default_on_off = 1;
          }
          else
          {
            pstr_mae_asi->group_definition[grp_index].default_on_off = 0;
          }
        }
      }
    }
  }

  if (pstr_enh_oam_frame->oam_has_been_decoded == 1)
  {
    error = impeghd_get_oam_list(pstr_interact_cfg, pstr_signals_3d, pstr_asc);
    if (error)
    {
      return error;
    }
  }

  if (pstr_interact_cfg->apply_intrct_data)
  {
    error = impeghd_md_proc_modified_gains(pstr_interact_cfg, pstr_asc, pstr_signals_3d,
                                           pstr_dec_data, ptr_scratch,
                                           pstr_enh_oam_frame->oam_has_been_decoded,
                                           pstr_dec_data->str_local_setup_interaction.dmx_id);
    if (error)
    {
      return error;
    }
  }

  if (pstr_enh_oam_cfg->has_diffuseness)
  {
    /*
     * take objects + modified oam gains + diffuse gain and create diffuse output
     * directly apply modified gains + diffuse gains to object signals and create speaker signal
     */
    error = impeghd_md_create_diffuse_part(
        pstr_interact_cfg, pstr_asc, pstr_signals_3d, &pstr_dec_data->str_local_setup_interaction,
        pstr_enh_oam_cfg, pstr_enh_oam_frame, pstr_dec_data, ptr_scratch);
    if (error)
    {
      return error;
    }

    /*
     * take objects + modified oam gains + direct gain and process further
     * create new modified oam gains by multiplying with direct gain
     */
    error = impeghd_md_proc_apply_dir_gain(pstr_dec_data, pstr_interact_cfg, pstr_asc,
                                           pstr_signals_3d, pstr_enh_oam_cfg, pstr_enh_oam_frame,
                                           overall_num_div_objs_added);
    if (error)
    {
      return error;
    }
  }

  if (enable_obj_op_interface == 0)
  {
    error = impeghd_md_proc_apply_div_proc(pstr_interact_cfg, pstr_asc,
                                           &overall_num_div_objs_added, pstr_enh_oam_cfg,
                                           pstr_enh_oam_frame, pstr_signals_3d, pstr_dec_data,
                                           obj_grp_idx);
    if (error)
    {
      return error;
    }
  }

  if (pstr_scene_displacement->scene_dspl_data_present)
  {
    error = impeghd_apply_scene_displacement(
        pstr_dec_data, pstr_asc, pstr_interact_cfg, ptr_scratch,
        (pstr_oam_cfg->num_objects > 0 ? 1 : 0), pstr_scene_displacement);
    if (error)
    {
      return error;
    }
  }

  if (pstr_enh_oam_frame->oam_has_been_decoded == 1)
  {
    // TODO: Support to be extended for all active OAM groups
    WORD32 oam_grp_idx = 0;
    error = impeghd_md_proc_apply_scrn_rel_remapping_zooming(
        pstr_interact_cfg, pstr_asc, pstr_dec_data, pstr_signals_3d, ptr_scratch, sel_preset,
        oam_grp_idx);
    if (error)
    {
      return error;
    }
    if (pstr_interact_cfg->apply_intrct_data)
    {
      error = impeghd_md_proc_apply_pos_interactivity(
          pstr_interact_cfg, pstr_asc, pstr_dec_data, pstr_signals_3d, ptr_scratch,
          pstr_dec_data->str_local_setup_interaction.dmx_id);
      if (error)
      {
        return error;
      }
    }

    error = impeghd_md_proc_closest_spk_proc(
        pstr_interact_cfg, pstr_asc, pstr_signals_3d, &pstr_dec_data->str_local_setup_interaction,
        pstr_enh_oam_cfg, &pstr_dec_data->str_obj_ren_dec_state.str_obj_md_dec_state,
        (pVOID)ptr_scratch_mem);
    if (error)
    {
      return error;
    }
  }

  error = impeghd_md_apply_gain_interactivity(pstr_interact_cfg, pstr_asc, pstr_dec_data,
                                              pstr_signals_3d);
  if (error)
  {
    return error;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mdp_dec_ei_process
 *
 *  \brief Metadata PreProcessing decoder main function
 *
 *  \param [in]  pstr_asc         Pointer to audio scene config structure
 *  \param [in,out] pstr_dec_data    Pointer to decoder data structure
 *  \param [in]  num_out_channels Number of output channels info
 *  \param [in]  ptr_scratch_mem  Pointer to scratch memory
 *  \param [in]  preset_id        Pointer to Preset ID value
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE impeghd_mdp_dec_ei_process(ia_audio_specific_config_struct *pstr_asc,
                                        ia_dec_data_struct *pstr_dec_data,
                                        WORD32 num_out_channels, WORD8 *ptr_scratch_mem,
                                        WORD32 *preset_id)
{
  IA_ERRORCODE error = 0;
  WORD32 switch_group_member = 0;
  ia_mae_audio_scene_info *pstr_mae_asi = &pstr_asc->str_mae_asi;
  if (pstr_mae_asi->num_group_presets > 0)
  {
    error = impeghd_mp_process_frame(pstr_asc, pstr_dec_data, ptr_scratch_mem, num_out_channels,
                                     preset_id, switch_group_member);
  }
  return error;
}
/** @} */ /* End of MDPreProc */
