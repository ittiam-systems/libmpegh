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
#include "impeghd_error_codes.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
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

/**
 * @defgroup ExtRenIfc External Renderer Interaces
 * @ingroup  ExtRenIfc
 * @brief External Renderer Interaces
 *
 * @{
 */

/**
 *  impeghd_read_bits_buf_util
 *
 *  \brief Function to read bits from bit buffer and compare
 *  with reference value.
 *
 *  \param [in]  pstr_bit_buf	Pointer to bit buffer.
 *  \param [in]  ref_value	reference value.
 *  \param [in]  num_bits	no of bits to be read.
 *
 *  \return VOID.
 *
 */

VOID impeghd_read_bits_buf_util(ia_bit_buf_struct *pstr_bit_buf, WORD32 ref_value,
                                WORD32 num_bits)
{
  (void)ref_value;
  ia_core_coder_read_bits_buf(pstr_bit_buf, num_bits);
}

/**
 *  impeghd_read_escape_value_util
 *
 *  \brief Read an integer value using varying number of bits and compare
 *  with reference value.
 *
 *  \param [in]    pstr_bit_buf		Pointer to bit buffer.
 *  \param [in]    ref_value	reference value.
 *  \param [in]    num_bits1			First set of Bits
 *  \param [in]    num_bits2			Second set of Bits
 *  \param [in]    num_bits3			Third set of Bits
 *
 *  \return VOID
 *
 */
VOID impeghd_read_escape_value_util(ia_bit_buf_struct *pstr_bit_buf, WORD32 ref_value,
                                    WORD32 num_bits1, WORD32 num_bits2, WORD32 num_bits3)
{
  WORD32 bs_val = 0;
  (void)ref_value;
  ia_core_coder_read_escape_value(pstr_bit_buf, (UWORD32 *)(&bs_val), num_bits1, num_bits2,
                                  num_bits3);
}

/**
*  impeghd_read_spk_description
*
*  \brief Update BRIR speaker config data from bitstream
*
*  \param [in]  pstr_bit_buf			 Pointer to input configuration
* structure
*  \param [in]  pstr_spk_desc		 Pointer to BRIR speaker
* description
*  \param [in]  angular_precision Angular precision
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_spk_description(ia_bit_buf_struct *pstr_bit_buf,
                                          ia_flex_spk_cfg_str *pstr_spk_desc,
                                          WORD32 angular_precision)
{
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->is_cicp_spk_idx, 1);

  if (pstr_spk_desc->is_cicp_spk_idx == 0)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->el_class, 2);
    if (pstr_spk_desc->el_class == 3)
    {

      if (angular_precision != 0)
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->el_angle_idx, 7);
      }
      else
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->el_angle_idx, 5);
      }
      if (impeghd_elev_idx_degree(pstr_spk_desc->el_angle_idx, pstr_spk_desc->el_direction,
                                  angular_precision) != 0)
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->el_direction, 1);
      }
    }
    if (angular_precision != 0)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->az_angle_idx, 8);
    }
    else
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->az_angle_idx, 6);
    }
    WORD32 az_ang_idx_to_deg = 0;
    az_ang_idx_to_deg = impeghd_azi_idx_degree(pstr_spk_desc->az_angle_idx,
                                               pstr_spk_desc->az_direction, angular_precision);
    if ((az_ang_idx_to_deg != 0) && (az_ang_idx_to_deg != 180))
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->az_direction, 1);
    }
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->is_lfe, 1);
  }
  else
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_spk_desc->cicp_spk_idx, 7);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_read_flexispk_config
*
*  \brief Update BRIR flexible speaker config data from bitstream
*
*  \param [in]  pstr_bit_buf			  Pointer to input configuration
* structure
*  \param [in]  pstr_flex_spk_data    Pointer to BRIR flexible spk config
*  \param [in]  num_spk            Number of speakers
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_flexispk_config(ia_bit_buf_struct *pstr_bit_buf,
                                          ia_flex_spk_data_str *pstr_flex_spk_data,
                                          WORD32 num_spk)
{
  WORD32 spk;
  WORD32 also_add_symetric_pair = 0;
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_flex_spk_data->angular_precision, 1);
  for (spk = 0; spk < num_spk; spk++)
  {
    WORD32 azimuth = 0;
    impeghd_read_spk_description(pstr_bit_buf, &(pstr_flex_spk_data->str_flex_spk_descr[spk]),
                                 pstr_flex_spk_data->angular_precision);

    if (pstr_flex_spk_data->str_flex_spk_descr[spk].is_cicp_spk_idx == 0)
    {
      azimuth = impeghd_azi_idx_degree(pstr_flex_spk_data->str_flex_spk_descr[spk].az_angle_idx,
                                       pstr_flex_spk_data->str_flex_spk_descr[spk].az_direction,
                                       pstr_flex_spk_data->angular_precision);
    }
    if ((azimuth != 0) && (azimuth != 180))
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_flex_spk_data->also_add_symetric_pair, 1);
      if (also_add_symetric_pair)
      {
        spk++;
        pstr_flex_spk_data->str_flex_spk_descr[spk] =
            pstr_flex_spk_data->str_flex_spk_descr[spk - 1];
        pstr_flex_spk_data->str_flex_spk_descr[spk].az_direction =
            1 - pstr_flex_spk_data->str_flex_spk_descr[spk].az_direction;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_read_speaker_config_3d
*
*  \brief Update 3d speaker config data from bitstream
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  spk_config_3d        Pointer to BRIR 3d speaker configuration
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_speaker_config_3d(ia_bit_buf_struct *pstr_bit_buf,
                                            ia_speaker_config_3d *spk_config_3d)
{
  WORD32 spk;
  impeghd_read_bits_buf_util(pstr_bit_buf, spk_config_3d->spk_layout_type, 2);
  switch (spk_config_3d->spk_layout_type)
  {
  case 0:
    impeghd_read_bits_buf_util(pstr_bit_buf, spk_config_3d->cicp_spk_layout_idx, 6);
    break;
  case 1:
    spk_config_3d->cicp_spk_layout_idx = (WORD16)-1;
    impeghd_read_escape_value_util(pstr_bit_buf, spk_config_3d->num_speakers - 1, 5, 8, 16);
    for (spk = 0; spk < spk_config_3d->num_speakers; spk++)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, spk_config_3d->cicp_spk_idx[spk], 7);
    }
    break;
  case 2:
    spk_config_3d->cicp_spk_layout_idx = (WORD16)-1;
    impeghd_read_escape_value_util(pstr_bit_buf, spk_config_3d->num_speakers - 1, 5, 8, 16);
    impeghd_read_flexispk_config(pstr_bit_buf, &(spk_config_3d->str_flex_spk),
                                 spk_config_3d->num_speakers);
    break;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_read_gca_prod_metadata_frm_ext_ren
*
*  \brief Read gca prod metadata
*
*  \param [in]  pstr_prod_meta_data   Pointer to prod metadata struct
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_3d_signals      Pointer to 3d signals
*
*  \return VOID
*
*/
VOID impeghd_read_gca_prod_metadata_frm_ext_ren(ia_prod_meta_data_struct *pstr_prod_meta_data,
                                                ia_bit_buf_struct *pstr_bit_buf,
                                                ia_signals_3d *pstr_3d_signals)
{
  for (WORD32 grp = 0; grp < pstr_3d_signals->num_ch_based_groups; grp++)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->direct_head_phone[grp], 1);
  }
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->has_reference_distance, 1);
  if (pstr_prod_meta_data->has_reference_distance)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->bs_reference_distance, 7);
  }
}
/**
*  impeghd_read_goa_prod_meta_data_for_ext_ren
*
*  \brief Read goa prod metadata
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_prod_meta_data   Pointer to prod metadata struct
*  \param [in]  num_out_objs         No of out objects
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE
impeghd_read_goa_prod_meta_data_for_ext_ren(ia_bit_buf_struct *pstr_bit_buf,
                                            ia_prod_meta_data_struct *pstr_prod_meta_data,
                                            WORD32 num_out_objs)
{
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->has_object_distance[0], 1);
  if (pstr_prod_meta_data->has_object_distance[0])
  {
    for (WORD32 obj = 0; obj < num_out_objs; obj++)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->position_distance[obj], 9);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
*  impeghd_read_gha_prod_meta_data_for_ext_ren
*
*  \brief Read gha prod metadata
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_prod_meta_data   Pointer to prod metadata struct
*
*  \return VOID
*
*/
VOID impeghd_read_gha_prod_meta_data_for_ext_ren(ia_bit_buf_struct *pstr_bit_buf,
                                                 ia_prod_meta_data_struct *pstr_prod_meta_data)
{
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->has_reference_distance, 1);
  if (pstr_prod_meta_data->has_reference_distance)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_prod_meta_data->bs_reference_distance, 7);
  }
}
/**
*  impeghd_read_downmix_cfg
*
*  \brief Read downmix config data
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_dmx_cfg			 Pointer to downmix config struct
*  \param [in]  ui_cicp_layout_idx   cicp layout index
*
*  \return VOID
*
*/
VOID impeghd_read_downmix_cfg(ia_bit_buf_struct *pstr_bit_buf,
                              ia_usac_ext_cfg_dmx_cfg *pstr_dmx_cfg, WORD32 ui_cicp_layout_idx)
{
  WORD32 dmx_id = 0;
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->dmx_config_type, 2);
  if (pstr_dmx_cfg->dmx_config_type == 0 || pstr_dmx_cfg->dmx_config_type == 2)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->passive_dmx_flag, 1);
    if (pstr_dmx_cfg->passive_dmx_flag == 0)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->phase_align_strength, 3);
    }
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->immersive_downmix_flag, 1);
  }
  if (pstr_dmx_cfg->dmx_config_type == 1 || pstr_dmx_cfg->dmx_config_type == 2)
  {
    /* DownmixMatrixConfig() */
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->downmix_id_count, 5);

    for (dmx_id = 0; dmx_id < (WORD32)pstr_dmx_cfg->downmix_id_count; ++dmx_id)
    {
      if (pstr_dmx_cfg->dmx_matrix[dmx_id].cicp_spk_layout_idx == ui_cicp_layout_idx)
      {
        WORD32 byte_count = 0;
        WORD32 dmx_matrix_len_bytes;
        WORD32 dmx_matrix_bits_left;
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_id, 7);
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_type, 2);
        if (pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_type == 0)
        {
          impeghd_read_bits_buf_util(pstr_bit_buf,
                                     pstr_dmx_cfg->dmx_matrix[dmx_id].cicp_spk_layout_idx, 6);
        }
        else if (pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_type == 1)
        {
          WORD32 dmx_cnt, grp;
          UWORD32 temp = pstr_dmx_cfg->dmx_matrix[dmx_id].downmix_mtx_count - 1;
          impeghd_read_bits_buf_util(pstr_bit_buf,
                                     pstr_dmx_cfg->dmx_matrix[dmx_id].cicp_spk_layout_idx, 6);
          impeghd_read_escape_value_util(pstr_bit_buf, temp, 1, 3, 0);
          for (dmx_cnt = 0; dmx_cnt < pstr_dmx_cfg->dmx_matrix[dmx_id].downmix_mtx_count;
               dmx_cnt++)
          {
            temp = pstr_dmx_cfg->dmx_matrix[dmx_id].num_assigned_group_ids[dmx_cnt] - 1;
            impeghd_read_escape_value_util(pstr_bit_buf, temp, 1, 4, 4);
            for (grp = 0; grp < pstr_dmx_cfg->dmx_matrix[dmx_id].num_assigned_group_ids[dmx_cnt];
                 grp++)
            {
              impeghd_read_bits_buf_util(
                  pstr_bit_buf, pstr_dmx_cfg->dmx_matrix[dmx_id].signal_group_id[dmx_cnt][grp],
                  5);
            }
            temp = pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_matrix_len_bits[dmx_cnt];
            impeghd_read_escape_value_util(pstr_bit_buf, temp, 8, 8, 12);
            dmx_matrix_len_bytes =
                pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_matrix_len_bits[dmx_cnt] / 8;
            dmx_matrix_bits_left =
                pstr_dmx_cfg->dmx_matrix[dmx_id].dmx_matrix_len_bits[dmx_cnt] % 8;
            byte_count = 0;
            while (dmx_matrix_len_bytes--)
            {
              impeghd_read_bits_buf_util(
                  pstr_bit_buf,
                  pstr_dmx_cfg->dmx_matrix[dmx_id].downmix_matrix[dmx_cnt][byte_count++], 8);
            }
            if (dmx_matrix_bits_left)
            {
              impeghd_read_bits_buf_util(
                  pstr_bit_buf,
                  pstr_dmx_cfg->dmx_matrix[dmx_id].downmix_matrix[dmx_cnt][byte_count] >>
                      (8 - dmx_matrix_bits_left),
                  dmx_matrix_bits_left);
            }
          }
        }
      }
    }
  }
  return;
}
/**
*  impeghd_read_ch_meta_data_for_ext_ren
*
*  \brief Read channel metadata
*
*  \param [in]  pstr_bit_buf				Pointer to input configuration
* structure
*  \param [in]  pstr_usac_cfg			Pointer to usac config struct
*  \param [in]  pstr_3d_signals			Pointer to 3d signals struct
*  \param [in]  ui_cicp_idx				cicp index
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_ch_meta_data_for_ext_ren(ia_bit_buf_struct *pstr_bit_buf,
                                                   ia_usac_config_struct *pstr_usac_cfg,
                                                   ia_signals_3d *pstr_3d_signals,
                                                   WORD32 ui_cicp_idx)
{
  WORD32 audio_truncation_gca = 0;
  WORD32 num_samples_gca = AUDIO_CODEC_FRAME_SIZE_MAX >> 6;
  WORD32 idx_ch_group = 0;
  WORD32 id_element[32][32] = {{0}};
  WORD32 *ptr_id_element[32] = {0};
  for (WORD32 i = 0; i < 32; i++)
  {
    ptr_id_element[i] = &id_element[i][0];
  }
  /* FRAME CONFIGURATION */
  impeghd_read_bits_buf_util(pstr_bit_buf, (AUDIO_CODEC_FRAME_SIZE_MAX >> 6), 6);
  impeghd_read_bits_buf_util(pstr_bit_buf, audio_truncation_gca, 2);
  if (audio_truncation_gca > 0)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, num_samples_gca, 13);
  }

  /* CHANNEL METADATA*/
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->num_ch_based_groups, 9);
  for (WORD32 grp = 0; grp < pstr_3d_signals->num_ch_based_groups; grp++)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->num_sig[idx_ch_group], 16);
    impeghd_read_speaker_config_3d(pstr_bit_buf, &pstr_3d_signals->audio_ch_layout[idx_ch_group]);
    while (pstr_3d_signals->group_type[idx_ch_group] != 0)
    {
      idx_ch_group++;
    }
    for (UWORD32 ch = 0; ch < pstr_3d_signals->num_sig[idx_ch_group]; ch++)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, ptr_id_element[grp][ch], 9);
    }

    /* TRACKING-RELATED METADATA */
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->fixed_position[idx_ch_group], 1);

    /* GROUP-RELATED METADATA */
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->group_priority[idx_ch_group], 3);
    impeghd_read_bits_buf_util(pstr_bit_buf, GCA_CHANNEL_GAIN,
                               8); // currently this parameter is hardcoded

    /* DOWNMIX MATRIX ELEMENT*/
    impeghd_read_bits_buf_util(pstr_bit_buf,
                               pstr_usac_cfg->str_usac_dec_config.downmix_ext_config_present, 1);

    if (pstr_usac_cfg->str_usac_dec_config.downmix_ext_config_present)
    {
      impeghd_read_downmix_cfg(pstr_bit_buf, &pstr_usac_cfg->str_usac_dec_config.dmx_cfg,
                               ui_cicp_idx);
    }
    if (pstr_usac_cfg->str_prod_metat_data.prod_metadata_present == 1)
    {
      WORD32 ext_element_type = ID_EXT_GCA_PROD_METADATA;
      WORD32 num_ext_elements = 1;
      impeghd_read_bits_buf_util(pstr_bit_buf, num_ext_elements, 3);
      for (UWORD8 ext = 0; ext < num_ext_elements; ext++)
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, ext_element_type, 3);
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   pstr_usac_cfg->str_prod_metat_data.payload_length, 10);
        switch (ext_element_type)
        {
        case ID_EXT_GCA_PROD_METADATA:
          impeghd_read_gca_prod_metadata_frm_ext_ren(&pstr_usac_cfg->str_prod_metat_data,
                                                     pstr_bit_buf, pstr_3d_signals);
          break;
        default:
          break;
        }
      }
    }
    else
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, 0, 3);
    }
    idx_ch_group++;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

#define CONVERT_TO_SIGNED_PARAM(a, bits)                                                         \
                                                                                                 \
  {                                                                                              \
    if (a < 0)                                                                                   \
    {                                                                                            \
      a = a + (1 << bits);                                                                       \
    }                                                                                            \
  }
/**
*  impeghd_read_oam_meta_data_for_ext_ren
*
*  \brief Read OAM metadata
*
*  \param [in]  pstr_bit_buf			Pointer to input configuration
* structure
*  \param [in]  pstr_usac_cfg			Pointer to usac config struct
*  \param [in]  pstr_obj_ren_dec_state  Pointer to object renderer state struct
*  \param [in]  pstr_3d_signals			Pointer to 3d signals struct
*  \param [in]  pstr_enh_oam_frm        Pointer to OAM frame struct
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_oam_meta_data_for_ext_ren(
    ia_bit_buf_struct *pstr_bit_buf, ia_usac_config_struct *pstr_usac_cfg,
    ia_obj_ren_dec_state_struct *pstr_obj_ren_dec_state, ia_signals_3d *pstr_3d_signals,
    ia_enh_obj_md_frame_str *pstr_enh_oam_frm)
{
  WORD32 goa_audio_truncation = 0, goa_num_samples = 0, goa_num_frames, num_objects;
  WORD32 num_ext_elements;
  ia_oam_dec_config_struct *pstr_oam_cfg = &pstr_usac_cfg->obj_md_cfg;
  ia_oam_dec_state_struct *pstr_oam_dec_state = &pstr_obj_ren_dec_state->str_obj_md_dec_state;
  ia_enh_oam_config_struct *pstr_enh_oam_cfg = &pstr_usac_cfg->enh_obj_md_cfg;
  goa_num_frames = (pstr_oam_cfg->cc_frame_length / pstr_oam_cfg->frame_length);
  num_objects = pstr_oam_dec_state->num_objects;

  /* FRAME CONFIGURATION */
  impeghd_read_bits_buf_util(pstr_bit_buf, (pstr_oam_cfg->frame_length >> 6), 6);
  impeghd_read_bits_buf_util(pstr_bit_buf, goa_audio_truncation, 2);
  if (goa_audio_truncation > 0)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, goa_num_samples, 13);
  }

  /* OBJECT METADATA */
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_oam_dec_state->num_objects, 9);
  for (WORD32 obj = 0; obj < pstr_oam_dec_state->num_objects; obj++)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, 0, 9); /*goa_element_id[obj]*/
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_oam_cfg->dyn_obj_priority_present, 1);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_oam_cfg->uniform_spread_present, 1);

    /* OAM Data */
    impeghd_read_bits_buf_util(pstr_bit_buf, goa_num_frames, 6);
    for (WORD32 frame = 0; frame < goa_num_frames; frame++)
    {
      WORD32 obj_idx = frame * num_objects + obj;
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_oam_dec_state->has_obj_md[obj_idx], 1);
      if (pstr_oam_dec_state->has_obj_md[obj_idx] == 1)
      {
        WORD32 val = pstr_oam_dec_state->azimuth[obj_idx];
        CONVERT_TO_SIGNED_PARAM(val, 8);
        impeghd_read_bits_buf_util(pstr_bit_buf, val, 8);
        val = pstr_oam_dec_state->elevation[obj_idx];
        CONVERT_TO_SIGNED_PARAM(val, 6);
        impeghd_read_bits_buf_util(pstr_bit_buf, val, 6);
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_oam_dec_state->radius[obj_idx], 4);
        val = pstr_oam_dec_state->gain[obj_idx];
        CONVERT_TO_SIGNED_PARAM(val, 7);
        impeghd_read_bits_buf_util(pstr_bit_buf, val, 7);
        if (pstr_oam_cfg->dyn_obj_priority_present)
        {
          impeghd_read_bits_buf_util(pstr_bit_buf, pstr_oam_dec_state->dyn_obj_priority[obj_idx],
                                     3);
        }
        if (!pstr_oam_cfg->uniform_spread_present)
        {
          val = pstr_oam_dec_state->spread_width[obj_idx];
          CONVERT_TO_SIGNED_PARAM(val, 7);
          impeghd_read_bits_buf_util(pstr_bit_buf, val, 7);
          val = pstr_oam_dec_state->spread_height[obj_idx];
          CONVERT_TO_SIGNED_PARAM(val, 5);
          impeghd_read_bits_buf_util(pstr_bit_buf, val, 5);
          val = pstr_oam_dec_state->spread_depth[obj_idx];
          CONVERT_TO_SIGNED_PARAM(val, 4);
          impeghd_read_bits_buf_util(pstr_bit_buf, val, 4);
        }
        else
        {
          val = pstr_oam_dec_state->spread_width[obj_idx];
          CONVERT_TO_SIGNED_PARAM(val, 7);
          impeghd_read_bits_buf_util(pstr_bit_buf, val, 7);
        }
      }
    }

    /* Signal group related data */
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->fixed_position[obj], 1);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->group_priority[obj], 3);

    /* Enhanced Object Metadata */
    impeghd_read_bits_buf_util(pstr_bit_buf, (WORD32)pstr_enh_oam_frm->diffuseness[obj], 7);
    impeghd_read_bits_buf_util(pstr_bit_buf, (WORD32)pstr_enh_oam_frm->divergence[obj], 7);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_enh_oam_cfg->divergence_az_range[obj], 6);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_enh_oam_frm->num_exclusion_sectors[obj], 4);

    for (WORD32 s = 0; s < pstr_enh_oam_frm->num_exclusion_sectors[obj]; s++)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_enh_oam_frm->use_predefined_sector[obj][s],
                                 1);
      if (!pstr_enh_oam_frm->use_predefined_sector[obj][s])
      {
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   (WORD32)pstr_enh_oam_frm->exclude_sector_min_az[obj][s], 7);
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   (WORD32)pstr_enh_oam_frm->exclude_sector_max_az[obj][s], 7);
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   (WORD32)pstr_enh_oam_frm->exclude_sector_min_ele[obj][s], 5);
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   (WORD32)pstr_enh_oam_frm->exclude_sector_max_ele[obj][s], 5);
      }
      else
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_enh_oam_frm->exclude_sector_index[obj][s],
                                   4);
      }
    }
  }

  if (pstr_usac_cfg->str_prod_metat_data.prod_metadata_present != 1)
  {
    num_ext_elements = 0;
    impeghd_read_bits_buf_util(pstr_bit_buf, num_ext_elements, 3);
  }
  else
  {
    num_ext_elements = 1;
    impeghd_read_bits_buf_util(pstr_bit_buf, num_ext_elements, 3);
    for (UWORD8 ext = 0; ext < num_ext_elements; ext++)
    {
      WORD32 ext_ele_type = ID_EXT_GOA_PROD_METADATA;
      impeghd_read_bits_buf_util(pstr_bit_buf, ext_ele_type, 3);
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_usac_cfg->str_prod_metat_data.payload_length,
                                 10);
      switch (ext_ele_type)
      {
      case ID_EXT_ELE_PROD_METADATA:
        impeghd_read_goa_prod_meta_data_for_ext_ren(
            pstr_bit_buf, &pstr_usac_cfg->str_prod_metat_data, pstr_oam_dec_state->num_objects);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
*  impeghd_read_mae_prod_screen_size
*
*  \brief Read screen size data
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  ptr_mae_prod_scrnsize_data       Pointer to screen size data struct
*
*  \return VOID
*
*/
VOID impeghd_read_mae_prod_screen_size(ia_bit_buf_struct *pstr_bit_buf,
                                       ia_production_screen_size_data *ptr_mae_prod_scrnsize_data)
{
  impeghd_read_bits_buf_util(pstr_bit_buf, ptr_mae_prod_scrnsize_data->has_non_std_screen_size,
                             1);
  if (ptr_mae_prod_scrnsize_data->has_non_std_screen_size)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, ptr_mae_prod_scrnsize_data->screen_size_az, 9);
    impeghd_read_bits_buf_util(pstr_bit_buf, ptr_mae_prod_scrnsize_data->screen_size_el, 9);
    impeghd_read_bits_buf_util(pstr_bit_buf, ptr_mae_prod_scrnsize_data->screen_size_bot_el, 9);
  }
}
/**
*  impeghd_read_mae_prod_screen_size_dataext
*
*  \brief Read screen size data ext
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_mae_prod_screen_szdata_ext       Pointer to screen size data ext struct
*
*  \return VOID
*
*/
VOID impeghd_read_mae_prod_screen_size_dataext(
    ia_bit_buf_struct *pstr_bit_buf,
    ia_production_screen_size_ext_data *pstr_mae_prod_screen_szdata_ext)
{
  impeghd_read_bits_buf_util(pstr_bit_buf,
                             pstr_mae_prod_screen_szdata_ext->overwrite_prod_screen_size_data, 1);
  if (pstr_mae_prod_screen_szdata_ext->overwrite_prod_screen_size_data)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf,
                               pstr_mae_prod_screen_szdata_ext->default_screen_sz_left_az, 10);
    impeghd_read_bits_buf_util(pstr_bit_buf,
                               pstr_mae_prod_screen_szdata_ext->default_screen_sz_right_az, 10);
  }
  impeghd_read_bits_buf_util(pstr_bit_buf,
                             pstr_mae_prod_screen_szdata_ext->num_preset_prod_screens, 5);
  for (WORD32 count = 0; count < pstr_mae_prod_screen_szdata_ext->num_preset_prod_screens;
       count++)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf,
                               pstr_mae_prod_screen_szdata_ext->screen_grp_preset_id[count], 5);
    impeghd_read_bits_buf_util(pstr_bit_buf,
                               pstr_mae_prod_screen_szdata_ext->has_non_std_screen_sz[count], 1);
    if (pstr_mae_prod_screen_szdata_ext->has_non_std_screen_sz[count])
    {
      impeghd_read_bits_buf_util(pstr_bit_buf,
                                 pstr_mae_prod_screen_szdata_ext->centered_in_az[count], 1);
      if (pstr_mae_prod_screen_szdata_ext->centered_in_az[count])
      {
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   pstr_mae_prod_screen_szdata_ext->screen_sz_left_az[count], 9);
      }
      else
      {
        impeghd_read_bits_buf_util(pstr_bit_buf,
                                   pstr_mae_prod_screen_szdata_ext->screen_sz_left_az[count], 10);
        impeghd_read_bits_buf_util(
            pstr_bit_buf, pstr_mae_prod_screen_szdata_ext->screen_sz_right_az[count], 10);
      }
      impeghd_read_bits_buf_util(pstr_bit_buf,
                                 pstr_mae_prod_screen_szdata_ext->screen_sz_top_el[count], 9);
      impeghd_read_bits_buf_util(pstr_bit_buf,
                                 pstr_mae_prod_screen_szdata_ext->screen_sz_bot_el[count], 9);
    }
  }
}
/**
*  impeghd_read_hoa_matrix_wrapper
*
*  \brief Read hoa matrix
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [in]  pstr_matrix_data      Pointer to HOA matrix struct
*  \param [in]  num_hoa_coeff        No of HOA coefficients
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_hoa_matrix_wrapper(ia_bit_buf_struct *pstr_bit_buf,
                                             ia_hoa_matrix_struct *pstr_matrix_data,
                                             WORD32 num_hoa_coeff)
{
  WORD32 i;
  WORD32 sqr = 7;
  WORD32 max_hoa_order, num_pairs = 0;
  while (((sqr * sqr) > num_hoa_coeff))
  {
    sqr--;
  }
  max_hoa_order = sqr - 1;
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->precision_level, 2);
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_normalized, 1);
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->gain_limit_per_hoa_order, 1);
  if (1 != pstr_matrix_data->gain_limit_per_hoa_order)
  {
    WORD32 val = -pstr_matrix_data->max_gain[0];
    impeghd_read_escape_value_util(pstr_bit_buf, val, 3, 5, 6);

    val = pstr_matrix_data->max_gain[0] - pstr_matrix_data->min_gain[0] - 1;
    impeghd_read_escape_value_util(pstr_bit_buf, val, 4, 5, 6);
  }
  else
  {
    for (i = 0; i < (max_hoa_order + 1); i++)
    {
      WORD32 val = -pstr_matrix_data->max_gain[i];
      impeghd_read_escape_value_util(pstr_bit_buf, val, 3, 5, 6);

      val = pstr_matrix_data->max_gain[i] - pstr_matrix_data->min_gain[i] - 1;
      impeghd_read_escape_value_util(pstr_bit_buf, val, 4, 5, 6);
    }
  }
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_full_matrix, 1);
  if (0 == pstr_matrix_data->is_full_matrix)
  {
    WORD32 n_bits_hoa_order = impeghd_hoa_get_ceil_log2(max_hoa_order + 1);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->first_sparse_order,
                               n_bits_hoa_order);
  }

  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->has_lfe_rendering, 1);

  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->zeroth_order_always_positive, 1);
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_all_value_symmetric, 1);
  if (!pstr_matrix_data->is_all_value_symmetric)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_any_value_symmetric, 1);

    if (!pstr_matrix_data->is_any_value_symmetric)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_all_sign_symmetric, 1);
      if (!pstr_matrix_data->is_all_sign_symmetric)
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_any_sign_symmetric, 1);
        if (pstr_matrix_data->is_any_sign_symmetric)
        {
          for (i = 0; i < num_pairs; i++)
          {
            impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->sign_symmetric_pairs[i],
                                       1);
          }
        }
      }
    }
    else
    {
      for (i = 0; i < num_pairs; i++)
      {
        impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->value_symmetric_pairs[i], 1);
      }
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->is_any_sign_symmetric, 1);
      if (pstr_matrix_data->is_any_sign_symmetric)
      {
        for (i = 0; i < num_pairs; i++)
        {
          if (0 == pstr_matrix_data->value_symmetric_pairs[i])
          {
            impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->sign_symmetric_pairs[i],
                                       1);
          }
        }
      }
    }
  }
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_matrix_data->has_vertical_coef, 1);
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
*  impeghd_read_hoa_meta_data_for_ext_ren
*
*  \brief Read hoa metadata
*
*  \param [in]  pstr_bit_buf          Pointer to input configuration
* structure
*  \param [i/o] pstr_usac_cfg        Pointer to usac config struct
*  \param [in]  pstr_3d_signals      Pointer to 3d signals struct
*  \param [i/o] pstr_mae_asi         Pointer to audio scene info struct
*
*  \return IA_ERRORCODE              Error code
*
*/
IA_ERRORCODE impeghd_read_hoa_meta_data_for_ext_ren(ia_bit_buf_struct *pstr_bit_buf,
                                                    ia_usac_config_struct *pstr_usac_cfg,
                                                    ia_signals_3d *pstr_3d_signals,
                                                    ia_mae_audio_scene_info *pstr_mae_asi)
{
  WORD32 gha_audio_trunc = 0, gha_num_samples = 0;
  WORD32 hoa_group_idx = 0;
  ia_hoa_config_struct *pstr_hoa_config = &pstr_usac_cfg->str_usac_dec_config.str_hoa_config;
  ia_hoa_matrix_struct *pstr_matrix_data = &pstr_hoa_config->str_hoa_matrix;
  impeghd_read_bits_buf_util(pstr_bit_buf, (pstr_hoa_config->frame_length >> 6), 6);
  impeghd_read_bits_buf_util(pstr_bit_buf, gha_audio_trunc, 2);
  if (gha_audio_trunc > 0)
  {
    impeghd_read_bits_buf_util(pstr_bit_buf, gha_num_samples, 13);
  }
  impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->num_hoa_based_groups, 9);
  for (WORD32 hgrp = 0; hgrp < pstr_3d_signals->num_hoa_based_groups; hgrp++)
  {
    while (pstr_3d_signals->group_type[hoa_group_idx] != 3)
    {
      hoa_group_idx++;
    }
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->fixed_position[hoa_group_idx], 1);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_3d_signals->group_priority[hoa_group_idx], 3);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_hoa_config->order, 9);
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_hoa_config->uses_nfc, 1);
    if (pstr_hoa_config->uses_nfc)
    {
      impeghd_read_bits_buf_util(pstr_bit_buf, (UWORD32)pstr_hoa_config->nfc_ref_distance, 32);
    }
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_hoa_config->matrix_present, 1);
    if (pstr_hoa_config->matrix_present)
    {
      impeghd_read_escape_value_util(pstr_bit_buf, pstr_matrix_data->hoa_matrix_len_bits, 8, 8,
                                     12);
      impeghd_read_hoa_matrix_wrapper(pstr_bit_buf, pstr_matrix_data,
                                      pstr_hoa_config->num_coeffs);
    }
    impeghd_read_bits_buf_util(pstr_bit_buf, pstr_hoa_config->is_screen_relative, 1);
    if (pstr_hoa_config->is_screen_relative)
    {
      impeghd_read_mae_prod_screen_size(pstr_bit_buf, &pstr_mae_asi->screen_size_data);
      impeghd_read_mae_prod_screen_size_dataext(pstr_bit_buf,
                                                &pstr_mae_asi->screen_size_ext_data);
    }
    hoa_group_idx++;
  }
  if (pstr_usac_cfg->str_prod_metat_data.prod_metadata_present == 1)
  {
    WORD32 num_ext_elements = 1;
    impeghd_read_bits_buf_util(pstr_bit_buf, num_ext_elements, 3);
    for (UWORD8 ext = 0; ext < num_ext_elements; ext++)
    {
      WORD32 ext_ele_type = ID_EXT_GOA_PROD_METADATA;
      impeghd_read_bits_buf_util(pstr_bit_buf, ext_ele_type, 3);
      impeghd_read_bits_buf_util(pstr_bit_buf, pstr_usac_cfg->str_prod_metat_data.payload_length,
                                 10);
      switch (ext_ele_type)
      {
      case ID_EXT_ELE_PROD_METADATA:
        impeghd_read_gha_prod_meta_data_for_ext_ren(pstr_bit_buf,
                                                    &pstr_usac_cfg->str_prod_metat_data);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of ExtRenIfc */