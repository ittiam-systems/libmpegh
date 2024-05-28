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

#include "math.h"
#include <stdio.h>
#include <stdlib.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_binaural.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_oam_dec.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"

/**
 * @defgroup ExtRenIfc External Renderer Interaces
 * @ingroup  ExtRenIfc
 * @brief External Renderer Interaces
 *
 * @{
 */

/**
*  impeghd_write_spk_description
*
*  \brief Write flexible speaker config data to bitstream
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_flex_spk_desc          Pointer to BRIR speaker
* description
*  \param [in]  angular_precision      Angular precision
*
*  \return IA_ERRORCODE              Error code
*
*/
static IA_ERRORCODE impeghd_write_spk_description(ia_write_bit_buf_struct *pstr_bit_buf,
                                                  ia_flex_spk_cfg_str *pstr_flex_spk_desc,
                                                  WORD32 angular_precision)
{
  impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->is_cicp_spk_idx, 1);

  if (!pstr_flex_spk_desc->is_cicp_spk_idx)
  {
    impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->el_class, 2);
    if (pstr_flex_spk_desc->el_class == 3)
    {

      if (angular_precision != 0)
      {
        impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->el_angle_idx, 7);
      }
      else
      {
        impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->el_angle_idx, 5);
      }
      if (impeghd_elev_idx_degree(pstr_flex_spk_desc->el_angle_idx,
                                  pstr_flex_spk_desc->el_direction, angular_precision) != 0)
      {
        impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->el_direction, 1);
      }
    }
    if (angular_precision != 0)
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->az_angle_idx, 8);
    }
    else
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->az_angle_idx, 6);
    }
    WORD32 az_ang_idx_to_deg = 0;
    az_ang_idx_to_deg = impeghd_azi_idx_degree(
        pstr_flex_spk_desc->az_angle_idx, pstr_flex_spk_desc->az_direction, angular_precision);
    if ((az_ang_idx_to_deg != 0) && (az_ang_idx_to_deg != 180))
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->az_direction, 1);
    }
    impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->is_lfe, 1);
  }
  else
  {
    impeghd_write_bits_buf(pstr_bit_buf, pstr_flex_spk_desc->cicp_spk_idx, 7);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_write_flexispk_config
*
*  \brief Write BRIR flexible spk config data to bitstream
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_flexi_spk_data        Pointer to BRIR flexible spk config
*  \param [in]  num_spk            Number of speakers
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_write_flexispk_config(ia_write_bit_buf_struct *pstr_bit_buf,
                                           ia_flex_spk_data_str *pstr_flexi_spk_data,
                                           WORD32 num_spk)
{
  WORD32 spk = 0;
  WORD32 add_symetric_pair_also = 0;
  impeghd_write_bits_buf(pstr_bit_buf, pstr_flexi_spk_data->angular_precision, 1);
  for (spk = 0; spk < num_spk; spk++)
  {
    WORD32 azimuth = 0;
    impeghd_write_spk_description(pstr_bit_buf, &(pstr_flexi_spk_data->str_flex_spk_descr[spk]),
                                  pstr_flexi_spk_data->angular_precision);

    if (!pstr_flexi_spk_data->str_flex_spk_descr[spk].is_cicp_spk_idx)
    {
      azimuth = impeghd_azi_idx_degree(pstr_flexi_spk_data->str_flex_spk_descr[spk].az_angle_idx,
                                       pstr_flexi_spk_data->str_flex_spk_descr[spk].az_direction,
                                       pstr_flexi_spk_data->angular_precision);
    }
    if ((azimuth != 0) && (azimuth != 180))
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_flexi_spk_data->also_add_symetric_pair, 1);
      if (add_symetric_pair_also)
      {
        spk++;
        pstr_flexi_spk_data->str_flex_spk_descr[spk] =
            pstr_flexi_spk_data->str_flex_spk_descr[spk - 1];
        pstr_flexi_spk_data->str_flex_spk_descr[spk].az_direction =
            1 - pstr_flexi_spk_data->str_flex_spk_descr[spk].az_direction;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_write_speaker_config_3d
*
*  \brief Write 3d speaker config data to bitstream
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_spk_config_3d        Pointer to BRIR 3d speaker configuretion
*
*  \return IA_ERRORCODE              Error code
*
*/
static IA_ERRORCODE impeghd_write_speaker_config_3d(ia_write_bit_buf_struct *pstr_bit_buf,
                                                    ia_speaker_config_3d *pstr_spk_config_3d)
{
  WORD32 spk;
  impeghd_write_bits_buf(pstr_bit_buf, pstr_spk_config_3d->spk_layout_type, 2);
  switch (pstr_spk_config_3d->spk_layout_type)
  {
  case 0:
    impeghd_write_bits_buf(pstr_bit_buf, pstr_spk_config_3d->cicp_spk_layout_idx, 6);
    break;
  case 1:
    pstr_spk_config_3d->cicp_spk_layout_idx = (WORD16)-1;
    impeghd_write_escape_value(pstr_bit_buf, pstr_spk_config_3d->num_speakers - 1, 5, 8, 16);
    for (spk = 0; spk < pstr_spk_config_3d->num_speakers; spk++)
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_spk_config_3d->cicp_spk_idx[spk], 7);
    }
    break;
  case 2:
    pstr_spk_config_3d->cicp_spk_layout_idx = (WORD16)-1;
    impeghd_write_escape_value(pstr_bit_buf, pstr_spk_config_3d->num_speakers - 1, 5, 8, 16);
    impeghd_write_flexispk_config(pstr_bit_buf, &(pstr_spk_config_3d->str_flex_spk),
                                  pstr_spk_config_3d->num_speakers);
    break;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_write_gca_prod_metadata_frm_ext_ren
*
*  \brief Write gca prod metadata
*
*  \param [in]  pstr_prod_meta_data   Pointer to prod metadata struct
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_3d_signals      Pointer to 3d signals
*
*
*
*/
static VOID
impeghd_write_gca_prod_metadata_frm_ext_ren(ia_prod_meta_data_struct *pstr_prod_meta_data,
                                            ia_write_bit_buf_struct *pstr_bit_buf,
                                            ia_signals_3d *pstr_3d_signals)
{
  for (WORD32 grp = 0; grp < pstr_3d_signals->num_ch_based_groups; grp++)
  {
    impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->direct_head_phone[grp], 1);
  }
  impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->has_reference_distance, 1);
  if (pstr_prod_meta_data->has_reference_distance)
  {
    impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->bs_reference_distance, 7);
  }
}

/**
*  impeghd_write_goa_prod_meta_data_for_ext_ren
*
*  \brief Write goa prod metadata to bitbuffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_prod_meta_data   Pointer to prod metadata struct
*  \param [in]  num_out_objs         No of out objects
*
*  \return IA_ERRORCODE              Error code
*
*/
static IA_ERRORCODE
impeghd_write_goa_prod_meta_data_for_ext_ren(ia_write_bit_buf_struct *pstr_bit_buf,
                                             ia_prod_meta_data_struct *pstr_prod_meta_data,
                                             WORD32 num_out_objs)
{
  impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->has_object_distance[0], 1);
  if (pstr_prod_meta_data->has_object_distance[0])
  {
    for (WORD32 obj = 0; obj < num_out_objs; obj++)
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->position_distance[obj], 9);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_write_gha_prod_meta_data_for_ext_ren
*
*  \brief Write gha prod metadata to bitbuffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_prod_meta_data   Pointer to prod metadata struct
*
*
*
*/
static VOID
impeghd_write_gha_prod_meta_data_for_ext_ren(ia_write_bit_buf_struct *pstr_bit_buf,
                                             ia_prod_meta_data_struct *pstr_prod_meta_data)
{
  impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->has_reference_distance, 1);
  if (pstr_prod_meta_data->has_reference_distance)
  {
    impeghd_write_bits_buf(pstr_bit_buf, pstr_prod_meta_data->bs_reference_distance, 7);
  }
}

/**
*  impeghd_get_goa_ele_id
*
*  \brief Returns the appropriate element id for a particular object.
*
*  \param [in]  pstr_mae_asi   pointer to ASI structure
*  \param [in]  grp            group number
*  \param [in]  obj            object number
*  \param [in]  ele_id_offset  element id offset
*
*
*
*/
static WORD16
impeghd_get_goa_ele_id(ia_mae_audio_scene_info *pstr_mae_asi, WORD32 grp, WORD32 obj, WORD32 ele_id_offset)
{
  if (pstr_mae_asi->asi_present)
  {
    pstr_mae_asi->group_definition[grp].metadata_ele_id[obj];
  }
  else
  {
    return (ele_id_offset + obj);
  }
}

/**
*  impeghd_write_downmix_cfg
*
*  \brief Write downmix config data to bitbuffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  ptr_ext_dmx_cfg			 Pointer to downmix config struct
*  \param [in]  ui_cicp_layout_idx   cicp layout index
*
*
*
*/
VOID impeghd_write_downmix_cfg(ia_write_bit_buf_struct *pstr_bit_buf,
                               ia_usac_ext_cfg_dmx_cfg *ptr_ext_dmx_cfg,
                               WORD32 ui_cicp_layout_idx)
{
  WORD32 dmx_id = 0;
  impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->dmx_config_type, 2);
  if (ptr_ext_dmx_cfg->dmx_config_type == 0 || ptr_ext_dmx_cfg->dmx_config_type == 2)
  {
    impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->passive_dmx_flag, 1);
    if (ptr_ext_dmx_cfg->passive_dmx_flag == 0)
    {
      impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->phase_align_strength, 3);
    }
    impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->immersive_downmix_flag, 1);
  }

  if (ptr_ext_dmx_cfg->dmx_config_type == 1 || ptr_ext_dmx_cfg->dmx_config_type == 2)
  {
    /* DownmixMatrixConfig() */
    impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->downmix_id_count, 5);

    for (dmx_id = 0; dmx_id < (WORD32)ptr_ext_dmx_cfg->downmix_id_count; ++dmx_id)
    {
      if (ptr_ext_dmx_cfg->dmx_matrix[dmx_id].cicp_spk_layout_idx == ui_cicp_layout_idx)
      {
        WORD32 byte_count = 0;
        WORD32 dmx_matrix_len_bytes;
        WORD32 dmx_matrix_bits_left;
        impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_id, 7);
        impeghd_write_bits_buf(pstr_bit_buf, ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_type, 2);
        if (ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_type == 1)
        {
          WORD32 dmx_mtx, asgn_id;
          UWORD32 temp = ptr_ext_dmx_cfg->dmx_matrix[dmx_id].downmix_mtx_count - 1;
          impeghd_write_bits_buf(pstr_bit_buf,
                                 ptr_ext_dmx_cfg->dmx_matrix[dmx_id].cicp_spk_layout_idx, 6);
          impeghd_write_escape_value(pstr_bit_buf, temp, 1, 3, 0);
          for (dmx_mtx = 0; dmx_mtx < ptr_ext_dmx_cfg->dmx_matrix[dmx_id].downmix_mtx_count;
               dmx_mtx++)
          {
            temp = ptr_ext_dmx_cfg->dmx_matrix[dmx_id].num_assigned_group_ids[dmx_mtx] - 1;
            impeghd_write_escape_value(pstr_bit_buf, temp, 1, 4, 4);
            for (asgn_id = 0;
                 asgn_id < ptr_ext_dmx_cfg->dmx_matrix[dmx_id].num_assigned_group_ids[dmx_mtx];
                 asgn_id++)
            {
              impeghd_write_bits_buf(
                  pstr_bit_buf,
                  ptr_ext_dmx_cfg->dmx_matrix[dmx_id].signal_group_id[dmx_mtx][asgn_id], 5);
            }
            temp = ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_matrix_len_bits[dmx_mtx];
            impeghd_write_escape_value(pstr_bit_buf, temp, 8, 8, 12);
            dmx_matrix_len_bytes =
                ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_matrix_len_bits[dmx_mtx] / 8;
            dmx_matrix_bits_left =
                ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_matrix_len_bits[dmx_mtx] % 8;
            byte_count = 0;
            while (dmx_matrix_len_bytes--)
            {
              impeghd_write_bits_buf(
                  pstr_bit_buf,
                  ptr_ext_dmx_cfg->dmx_matrix[dmx_id].downmix_matrix[dmx_mtx][byte_count++], 8);
            }
            if (dmx_matrix_bits_left)
            {
              impeghd_write_bits_buf(
                  pstr_bit_buf,
                  ptr_ext_dmx_cfg->dmx_matrix[dmx_id].downmix_matrix[dmx_mtx][byte_count] >>
                      (8 - dmx_matrix_bits_left),
                  dmx_matrix_bits_left);
            }
          }
        }
        else if (ptr_ext_dmx_cfg->dmx_matrix[dmx_id].dmx_type == 0)
        {
          impeghd_write_bits_buf(pstr_bit_buf,
                                 ptr_ext_dmx_cfg->dmx_matrix[dmx_id].cicp_spk_layout_idx, 6);
        }
      }
    }
  }
  return;
}

/**
*  impeghd_write_ch_meta_data_for_ext_ren
*
*  \brief Write ch metadata to bitbuffer
*
*  \param [out]  pstr_bit_buf				Pointer to input configuration
* structure
*  \param [in]  pstr_usac_config			Pointer to usac config struct
*  \param [in]  pstr_3d_signals			Pointer to 3d signals struct
*  \param [in]  ui_cicp_idx				cicp index
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_write_ch_meta_data_for_ext_ren(ia_write_bit_buf_struct *pstr_bit_buf,
                                                    ia_usac_config_struct *pstr_usac_config,
                                                    ia_signals_3d *pstr_3d_signals,
                                                    WORD32 ui_cicp_idx)
{
  WORD32 audio_truncation_gca = 0;
  WORD32 num_samples_gca = AUDIO_CODEC_FRAME_SIZE_MAX >> 6;
  WORD32 idx_ch_grp = 0;
  WORD32 num_ext_elements;
  WORD32 id_element[32][32] = {{0}};
  WORD32 *ptr_id_element[32] = {0};
  for (WORD32 id = 0; id < 32; id++)
  {
    ptr_id_element[id] = &id_element[id][0];
  }
  /* FRAME CONFIGURATION */
  impeghd_write_bits_buf(pstr_bit_buf, (AUDIO_CODEC_FRAME_SIZE_MAX >> 6), 6);
  impeghd_write_bits_buf(pstr_bit_buf, audio_truncation_gca, 2);
  if (audio_truncation_gca > 0)
  {
    impeghd_write_bits_buf(pstr_bit_buf, num_samples_gca, 13);
  }

  /* CHANNEL METADATA*/
  impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->num_ch_based_groups, 9);
  for (WORD32 grp = 0; grp < pstr_3d_signals->num_ch_based_groups; grp++)
  {
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->num_sig[idx_ch_grp], 16);
    impeghd_write_speaker_config_3d(pstr_bit_buf, &pstr_3d_signals->audio_ch_layout[idx_ch_grp]);
    while (pstr_3d_signals->group_type[idx_ch_grp] != 0)
    {
      idx_ch_grp++;
    }
    for (UWORD32 ch = 0; ch < pstr_3d_signals->num_sig[idx_ch_grp]; ch++)
    {
      impeghd_write_bits_buf(pstr_bit_buf, ptr_id_element[grp][ch], 9);
    }

    /* TRACKING-RELATED METADATA */
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->fixed_position[idx_ch_grp], 1);

    /* GROUP-RELATED METADATA */
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->group_priority[idx_ch_grp], 3);
    impeghd_write_bits_buf(pstr_bit_buf, 96, 8); // currently this parameter is hardcoded

    /* DOWNMIX MATRIX ELEMENT*/
    impeghd_write_bits_buf(pstr_bit_buf,
                           pstr_usac_config->str_usac_dec_config.downmix_ext_config_present, 1);

    if (pstr_usac_config->str_usac_dec_config.downmix_ext_config_present)
    {
      impeghd_write_downmix_cfg(pstr_bit_buf, &pstr_usac_config->str_usac_dec_config.dmx_cfg,
                                ui_cicp_idx);
    }

    if (pstr_usac_config->str_prod_metat_data.prod_metadata_present != 1)
    {
      num_ext_elements = 0;
      impeghd_write_bits_buf(pstr_bit_buf, num_ext_elements, 3);
    }
    else
    {
      WORD32 ext_element_type = ID_EXT_GCA_PROD_METADATA;
      num_ext_elements = 1;
      impeghd_write_bits_buf(pstr_bit_buf, num_ext_elements, 3);
      for (UWORD8 ext = 0; ext < num_ext_elements; ext++)
      {
        impeghd_write_bits_buf(pstr_bit_buf, ID_EXT_GCA_PROD_METADATA, 3);
        impeghd_write_bits_buf(pstr_bit_buf, pstr_usac_config->str_prod_metat_data.payload_length,
                               10);
        switch (ext_element_type)
        {
        case ID_EXT_GCA_PROD_METADATA:
          impeghd_write_gca_prod_metadata_frm_ext_ren(&pstr_usac_config->str_prod_metat_data,
                                                      pstr_bit_buf, pstr_3d_signals);
          break;
        default:
          break;
        }
      }
    }
    idx_ch_grp++;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_write_oam_meta_data_for_ext_ren
*
*  \brief Write oam metadata to bitbuffer
*
*  \param [out]  pstr_bit_buf				Pointer to input configuration
* structure
*  \param [in]  pstr_usac_config			Pointer to usac config struct
*  \param [in]  pstr_obj_ren_dec_state  Pointer to object renderer state struct
*  \param [in]  pstr_3d_signals			Pointer to 3d signals struct
*  \param [in]  pstr_enh_oam_frm        Pointer to oam frame struct
*  \param [in]  pstr_mae_asi            Pointer to ASI structure
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_write_oam_meta_data_for_ext_ren(
    ia_write_bit_buf_struct *pstr_bit_buf, ia_usac_config_struct *pstr_usac_config,
    ia_obj_ren_dec_state_struct *pstr_obj_ren_dec_state, ia_signals_3d *pstr_3d_signals,
    ia_enh_obj_md_frame_str *pstr_enh_oam_frm, ia_mae_audio_scene_info *pstr_mae_asi)
{
  ia_oam_dec_config_struct *pstr_oam_cfg = &pstr_usac_config->obj_md_cfg;
  ia_oam_dec_state_struct *pstr_oam_dec_state = &pstr_obj_ren_dec_state->str_obj_md_dec_state;
  ia_enh_oam_config_struct *pstr_enh_oam_cfg = &pstr_usac_config->enh_obj_md_cfg;
  WORD32 audio_truncation_goa = 0, goa_num_samples = 0;
  WORD32 num_frames_goa = (pstr_oam_cfg->cc_frame_length / pstr_oam_cfg->frame_length);
  WORD32 num_objects = pstr_oam_dec_state->num_objects;
  WORD32 num_ext_elements, grp, mae_ele_id_offset = 0;
  /* FRAME CONFIGURATION */
  impeghd_write_bits_buf(pstr_bit_buf, (pstr_oam_cfg->frame_length >> 6), 6);
  impeghd_write_bits_buf(pstr_bit_buf, audio_truncation_goa, 2);
  if (audio_truncation_goa > 0)
  {
    impeghd_write_bits_buf(pstr_bit_buf, goa_num_samples, 13);
  }

  for (grp = 0; grp < pstr_3d_signals->num_sig_group; grp++)
  {
    if (SIG_GROUP_TYPE_OBJ == pstr_3d_signals->group_type[grp])
    {
      break;
    }
    else
    {
      mae_ele_id_offset += pstr_3d_signals->num_sig[grp];
    }
  }

  /* OBJECT METADATA */
  impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->num_objects, 9);
  for (WORD32 obj = 0; obj < pstr_oam_dec_state->num_objects; obj++)
  {
    impeghd_write_bits_buf(pstr_bit_buf, impeghd_get_goa_ele_id(pstr_mae_asi, grp, obj, mae_ele_id_offset), 9); /*goa_element_id[obj]*/
    impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_cfg->dyn_obj_priority_present, 1);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_cfg->uniform_spread_present, 1);

    /* OAM Data */
    impeghd_write_bits_buf(pstr_bit_buf, num_frames_goa, 6);
    for (WORD32 frame = 0; frame < num_frames_goa; frame++)
    {
      WORD32 obj_idx = frame * num_objects + obj;
      impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->has_obj_md[obj_idx], 1);
      if (pstr_oam_dec_state->has_obj_md[obj_idx] == 1)
      {
        impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->azimuth[obj_idx], 8);
        impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->elevation[obj_idx], 6);
        impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->radius[obj_idx], 4);
        impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->gain[obj_idx], 7);
        if (pstr_oam_cfg->dyn_obj_priority_present)
        {
          impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->dyn_obj_priority[obj_idx], 3);
        }
        if (!pstr_oam_cfg->uniform_spread_present)
        {
          impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->spread_width[obj_idx], 7);
          impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->spread_height[obj_idx], 5);
          impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->spread_depth[obj_idx], 4);
        }
        else
        {
          impeghd_write_bits_buf(pstr_bit_buf, pstr_oam_dec_state->spread_width[obj_idx], 7);
        }
      }
    }

    /* Signal group related data */
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->fixed_position[obj], 1);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->group_priority[obj], 3);

    /* Enhanced Object Metadata */
    impeghd_write_bits_buf(pstr_bit_buf, (WORD32)pstr_enh_oam_frm->diffuseness[obj], 7);
    impeghd_write_bits_buf(pstr_bit_buf, (WORD32)pstr_enh_oam_frm->divergence[obj], 7);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_enh_oam_cfg->divergence_az_range[obj], 6);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_enh_oam_frm->num_exclusion_sectors[obj], 4);

    for (WORD32 sect = 0; sect < pstr_enh_oam_frm->num_exclusion_sectors[obj]; sect++)
    {
      impeghd_write_bits_buf(pstr_bit_buf, pstr_enh_oam_frm->use_predefined_sector[obj][sect], 1);
      if (!pstr_enh_oam_frm->use_predefined_sector[obj][sect])
      {
        impeghd_write_bits_buf(pstr_bit_buf,
                               (WORD32)pstr_enh_oam_frm->exclude_sector_min_az[obj][sect], 7);
        impeghd_write_bits_buf(pstr_bit_buf,
                               (WORD32)pstr_enh_oam_frm->exclude_sector_max_az[obj][sect], 7);
        impeghd_write_bits_buf(pstr_bit_buf,
                               (WORD32)pstr_enh_oam_frm->exclude_sector_min_ele[obj][sect], 5);
        impeghd_write_bits_buf(pstr_bit_buf,
                               (WORD32)pstr_enh_oam_frm->exclude_sector_max_ele[obj][sect], 5);
      }
      else
      {
        impeghd_write_bits_buf(pstr_bit_buf, pstr_enh_oam_frm->exclude_sector_index[obj][sect],
                               4);
      }
    }
  }
  if (pstr_usac_config->str_prod_metat_data.prod_metadata_present != 1)
  {
    num_ext_elements = 0;
    impeghd_write_bits_buf(pstr_bit_buf, num_ext_elements, 3);
  }
  else
  {
    WORD32 ext_ele_type = ID_EXT_GOA_PROD_METADATA;
    num_ext_elements = 1;
    impeghd_write_bits_buf(pstr_bit_buf, num_ext_elements, 3);
    for (UWORD8 ext = 0; ext < num_ext_elements; ext++)
    {
      impeghd_write_bits_buf(pstr_bit_buf, ID_EXT_GOA_PROD_METADATA, 3);
      impeghd_write_bits_buf(pstr_bit_buf, pstr_usac_config->str_prod_metat_data.payload_length,
                             10);
      switch (ext_ele_type)
      {
      case ID_EXT_ELE_PROD_METADATA:
        impeghd_write_goa_prod_meta_data_for_ext_ren(pstr_bit_buf,
                                                     &pstr_usac_config->str_prod_metat_data,
                                                     pstr_oam_dec_state->num_objects);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_write_mae_prod_screen_size
*
*  \brief Write screen size data to bitbuffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  ptr_mae_prod_scrnsize_data       Pointer to screen size data struct
*
*
*
*/
static VOID
impeghd_write_mae_prod_screen_size(ia_write_bit_buf_struct *pstr_bit_buf,
                                   ia_production_screen_size_data *ptr_mae_prod_scrnsize_data)
{
  impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_scrnsize_data->has_non_std_screen_size, 1);
  if (ptr_mae_prod_scrnsize_data->has_non_std_screen_size)
  {
    impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_scrnsize_data->screen_size_az, 9);
    impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_scrnsize_data->screen_size_el, 9);
    impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_scrnsize_data->screen_size_bot_el, 9);
  }
}

/**
*  impeghd_write_mae_prod_screen_size_dataext
*
*  \brief Write screen size data ext to bitbuffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  ptr_mae_prod_screen_szdata_ext       Pointer to screen size data ext struct
*
*
*
*/
VOID impeghd_write_mae_prod_screen_size_dataext(
    ia_write_bit_buf_struct *pstr_bit_buf,
    ia_production_screen_size_ext_data *ptr_mae_prod_screen_szdata_ext)
{
  impeghd_write_bits_buf(pstr_bit_buf,
                         ptr_mae_prod_screen_szdata_ext->overwrite_prod_screen_size_data, 1);
  if (ptr_mae_prod_screen_szdata_ext->overwrite_prod_screen_size_data)
  {
    impeghd_write_bits_buf(pstr_bit_buf,
                           ptr_mae_prod_screen_szdata_ext->default_screen_sz_left_az, 10);
    impeghd_write_bits_buf(pstr_bit_buf,
                           ptr_mae_prod_screen_szdata_ext->default_screen_sz_right_az, 10);
  }
  impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_screen_szdata_ext->num_preset_prod_screens,
                         5);
  for (WORD32 scrn = 0; scrn < ptr_mae_prod_screen_szdata_ext->num_preset_prod_screens; scrn++)
  {
    impeghd_write_bits_buf(pstr_bit_buf,
                           ptr_mae_prod_screen_szdata_ext->screen_grp_preset_id[scrn], 5);
    impeghd_write_bits_buf(pstr_bit_buf,
                           ptr_mae_prod_screen_szdata_ext->has_non_std_screen_sz[scrn], 1);
    if (ptr_mae_prod_screen_szdata_ext->has_non_std_screen_sz[scrn])
    {
      impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_screen_szdata_ext->centered_in_az[scrn],
                             1);
      if (!ptr_mae_prod_screen_szdata_ext->centered_in_az[scrn])
      {
        impeghd_write_bits_buf(pstr_bit_buf,
                               ptr_mae_prod_screen_szdata_ext->screen_sz_left_az[scrn], 10);
        impeghd_write_bits_buf(pstr_bit_buf,
                               ptr_mae_prod_screen_szdata_ext->screen_sz_right_az[scrn], 10);
      }
      else
      {
        impeghd_write_bits_buf(pstr_bit_buf,
                               ptr_mae_prod_screen_szdata_ext->screen_sz_left_az[scrn], 9);
      }
      impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_screen_szdata_ext->screen_sz_top_el[scrn],
                             9);
      impeghd_write_bits_buf(pstr_bit_buf, ptr_mae_prod_screen_szdata_ext->screen_sz_bot_el[scrn],
                             9);
    }
  }
}

/**
*  impeghd_write_hoa_matrix_wrapper
*
*  \brief Write hoa matrix to bit buffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  ptr_matrix_data      Pointer to HOA matrix struct
*  \param [in]  num_hoa_coeff        No of HOA coefficients
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_write_hoa_matrix_wrapper(ia_write_bit_buf_struct *pstr_bit_buf,
                                              ia_hoa_matrix_struct *ptr_matrix_data,
                                              WORD32 num_hoa_coeff)
{
  WORD32 pair;
  WORD32 num_pairs = 0;
  WORD32 sqr = 7;
  while (((sqr * sqr) > num_hoa_coeff))
  {
    sqr--;
  }
  WORD32 max_hoa_order = sqr - 1;
  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->precision_level, 2);
  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_normalized, 1);
  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->gain_limit_per_hoa_order, 1);

  if (1 != ptr_matrix_data->gain_limit_per_hoa_order)
  {
    WORD32 val = -ptr_matrix_data->max_gain[0];
    impeghd_write_escape_value(pstr_bit_buf, val, 3, 5, 6);

    val = ptr_matrix_data->max_gain[0] - ptr_matrix_data->min_gain[0] - 1;
    impeghd_write_escape_value(pstr_bit_buf, val, 4, 5, 6);
  }
  else
  {
    for (WORD32 order = 0; order < (max_hoa_order + 1); order++)
    {
      WORD32 val = -ptr_matrix_data->max_gain[order];
      impeghd_write_escape_value(pstr_bit_buf, val, 3, 5, 6);

      val = ptr_matrix_data->max_gain[order] - ptr_matrix_data->min_gain[order] - 1;
      impeghd_write_escape_value(pstr_bit_buf, val, 4, 5, 6);
    }
  }
  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_full_matrix, 1);
  if (0 == ptr_matrix_data->is_full_matrix)
  {
    WORD32 n_bits_hoa_order = impeghd_hoa_get_ceil_log2(max_hoa_order + 1);
    impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->first_sparse_order, n_bits_hoa_order);
  }

  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->has_lfe_rendering, 1);

  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->zeroth_order_always_positive, 1);
  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_all_value_symmetric, 1);
  if (!ptr_matrix_data->is_all_value_symmetric)
  {
    impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_any_value_symmetric, 1);
    if (!ptr_matrix_data->is_any_value_symmetric)
    {
      impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_all_sign_symmetric, 1);
      if (!ptr_matrix_data->is_all_sign_symmetric)
      {
        impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_any_sign_symmetric, 1);
        if (ptr_matrix_data->is_any_sign_symmetric)
        {
          for (pair = 0; pair < num_pairs; pair++)
          {
            impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->sign_symmetric_pairs[pair], 1);
          }
        }
      }
    }
    else
    {
      for (pair = 0; pair < num_pairs; pair++)
      {
        impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->value_symmetric_pairs[pair], 1);
      }
      impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->is_any_sign_symmetric, 1);
      if (ptr_matrix_data->is_any_sign_symmetric)
      {
        for (pair = 0; pair < num_pairs; pair++)
        {
          if (0 == ptr_matrix_data->value_symmetric_pairs[pair])
          {
            impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->sign_symmetric_pairs[pair], 1);
          }
        }
      }
    }
  }
  impeghd_write_bits_buf(pstr_bit_buf, ptr_matrix_data->has_vertical_coef, 1);
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
*  impeghd_write_hoa_meta_data_for_ext_ren
*
*  \brief Write hoa metadata to bitbuffer
*
*  \param [out]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in,out] pstr_usac_config        Pointer to usac config struct
*  \param [in]  pstr_3d_signals      Pointer to 3d signals struct
*  \param [in,out] pstr_mae_asi         Pointer to audio scene info struct
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_write_hoa_meta_data_for_ext_ren(ia_write_bit_buf_struct *pstr_bit_buf,
                                                     ia_usac_config_struct *pstr_usac_config,
                                                     ia_signals_3d *pstr_3d_signals,
                                                     ia_mae_audio_scene_info *pstr_mae_asi)
{
  ia_hoa_config_struct *pstr_hoa_config = &pstr_usac_config->str_usac_dec_config.str_hoa_config;
  ia_hoa_matrix_struct *ptr_matrix_data = &pstr_hoa_config->str_hoa_matrix;
  WORD32 gha_audio_trunc = 0, gha_num_samples = 0;
  WORD32 hoa_group_idx = 0;
  impeghd_write_bits_buf(pstr_bit_buf, (pstr_hoa_config->frame_length >> 6), 6);
  impeghd_write_bits_buf(pstr_bit_buf, gha_audio_trunc, 2);
  if (gha_audio_trunc > 0)
  {
    impeghd_write_bits_buf(pstr_bit_buf, gha_num_samples, 13);
  }
  impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->num_hoa_based_groups, 9);
  for (WORD32 hgrp = 0; hgrp < pstr_3d_signals->num_hoa_based_groups; hgrp++)
  {
    while (pstr_3d_signals->group_type[hoa_group_idx] != 3)
    {
      hoa_group_idx++;
    }
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->fixed_position[hoa_group_idx], 1);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_3d_signals->group_priority[hoa_group_idx], 3);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_hoa_config->order, 9);
    impeghd_write_bits_buf(pstr_bit_buf, pstr_hoa_config->uses_nfc, 1);
    if (pstr_hoa_config->uses_nfc)
    {
      impeghd_write_bits_buf(pstr_bit_buf, (UWORD32)pstr_hoa_config->nfc_ref_distance, 32);
    }
    impeghd_write_bits_buf(pstr_bit_buf, pstr_hoa_config->matrix_present, 1);
    if (pstr_hoa_config->matrix_present)
    {
      impeghd_write_escape_value(pstr_bit_buf, ptr_matrix_data->hoa_matrix_len_bits, 8, 8, 12);
      impeghd_write_hoa_matrix_wrapper(pstr_bit_buf, ptr_matrix_data,
                                       pstr_hoa_config->num_coeffs);
    }
    impeghd_write_bits_buf(pstr_bit_buf, pstr_hoa_config->is_screen_relative, 1);
    if (pstr_hoa_config->is_screen_relative)
    {
      impeghd_write_mae_prod_screen_size(pstr_bit_buf, &pstr_mae_asi->screen_size_data);
      impeghd_write_mae_prod_screen_size_dataext(pstr_bit_buf,
                                                 &pstr_mae_asi->screen_size_ext_data);
    }
    hoa_group_idx++;
  }
  if (pstr_usac_config->str_prod_metat_data.prod_metadata_present == 1)
  {
    WORD32 ext_ele_type = ID_EXT_GOA_PROD_METADATA;
    WORD32 num_ext_elements = 1;
    impeghd_write_bits_buf(pstr_bit_buf, num_ext_elements, 3);
    for (UWORD8 ext = 0; ext < num_ext_elements; ext++)
    {
      impeghd_write_bits_buf(pstr_bit_buf, ID_EXT_GOA_PROD_METADATA, 3);
      impeghd_write_bits_buf(pstr_bit_buf, pstr_usac_config->str_prod_metat_data.payload_length,
                             10);
      switch (ext_ele_type)
      {
      case ID_EXT_ELE_PROD_METADATA:
        impeghd_write_gha_prod_meta_data_for_ext_ren(pstr_bit_buf,
                                                     &pstr_usac_config->str_prod_metat_data);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of ExtRenIfc */