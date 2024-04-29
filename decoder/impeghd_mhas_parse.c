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

#include <string.h>
#include <impeghd_type_def.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "impeghd_error_codes.h"
#include "impeghd_mhas_parse.h"
#include "impeghd_uni_drc_struct.h"

/**
 * @defgroup MHASParse MHAS Parsing
 * @ingroup  MHASParse
 * @brief MHAS Header parsing utilities
 *
 * @{
 */

/**
 *  impeghd_mae_parse_description_data
 *
 *  \brief Parse description of metadata audio element
 *
 *  \param [out] pstr_description_data Description data handle
 *  \param [in]  pstr_bit_buf          Bit buffer handle
 *  \param [in]  grp_id_bits           Bitstream parameter group id bits
 *
 *  \return IA_ERRORCODE               Processing error if any
 *
 */
static IA_ERRORCODE impeghd_mae_parse_description_data(ia_description_data *pstr_description_data,
                                                       ia_bit_buf_struct *pstr_bit_buf,
                                                       WORD32 grp_id_bits)
{
  pstr_description_data->num_descr_blks = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7) + 1;
  if (pstr_description_data->num_descr_blks > MAX_NUM_DESCRIPTOIN_BLOCKS)
  {
    pstr_description_data->num_descr_blks = 0;
    return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_DESCRIPTION_BLOCKS;
  }

  for (UWORD32 blk = 0; blk < pstr_description_data->num_descr_blks; blk++)
  {
    pstr_description_data->grp_id[blk] =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, grp_id_bits);
    pstr_description_data->num_descr_languages[blk] =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4) + 1;

    if (pstr_description_data->num_descr_languages[blk] > MAX_NUM_DESCR_LANGUAGES)
    {
      pstr_description_data->num_descr_languages[blk] = 0;
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_DESCRIPTION_LANGUAGES;
    }

    for (WORD32 cnt = 0; cnt < pstr_description_data->num_descr_languages[blk]; cnt++)
    {
      pstr_description_data->descr_language[blk][cnt] =
          ia_core_coder_read_bits_buf(pstr_bit_buf, 24);
      pstr_description_data->descr_data_length[blk][cnt] =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 8) + 1;

      for (WORD32 idx = 0; idx < pstr_description_data->descr_data_length[blk][cnt]; idx++)
      {
        pstr_description_data->descr_data[blk][cnt][idx] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mae_parse_content_data
 *
 *  \brief Parse content data of metadata audio element
 *
 *  \param [out] pstr_content_data   Pointer to content data structure
 *  \param [in]  pstr_bit_buf       Bit buffer handle
 *
 *
 *
 */
static VOID impeghd_mae_parse_content_data(ia_content_data *pstr_content_data,
                                           ia_bit_buf_struct *pstr_bit_buf)
{
  pstr_content_data->num_data_blks = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  for (WORD32 blk = 0; blk < pstr_content_data->num_data_blks + 1; blk++)
  {
    pstr_content_data->data_grp_id[blk] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    pstr_content_data->content_kind[blk] = ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
    if ((pstr_content_data->has_content_language[blk] =
             ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_content_data->content_language[blk] = ia_core_coder_read_bits_buf(pstr_bit_buf, 24);
    }
  }
}

/**
 *  impeghd_mae_parse_composite_pair
 *
 *  \brief Parse composite pair data of metadata audio element
 *
 *  \param [out] pstr_composite_pair   Pointer to composite pair data structure
 *  \param [in]  pstr_bit_buf       Bit buffer handle
 *
 *
 *
 */
static VOID impeghd_mae_parse_composite_pair(ia_composite_pair_data *pstr_composite_pair,
                                             ia_bit_buf_struct *pstr_bit_buf)
{
  pstr_composite_pair->num_pairs = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  for (WORD32 cnt = 0; cnt < pstr_composite_pair->num_pairs; cnt++)
  {
    pstr_composite_pair->ele_id[cnt][0] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    pstr_composite_pair->ele_id[cnt][1] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  }
}

/**
 *  impeghd_mae_parse_prod_screen_sz_data
 *
 *  \brief Parse production screen size data of metadata audio element
 *
 *  \param [out] pstr_screen_sz_data   Pointer to production screen size data structure
 *  \param [in]  pstr_bit_buf       Bit buffer handle
 *
 *
 *
 */
static VOID
impeghd_mae_parse_prod_screen_sz_data(ia_production_screen_size_data *pstr_screen_sz_data,
                                      ia_bit_buf_struct *pstr_bit_buf)
{
  if ((pstr_screen_sz_data->has_non_std_screen_size =
           ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
  {
    pstr_screen_sz_data->screen_size_az = ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
    pstr_screen_sz_data->screen_size_el = ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
    pstr_screen_sz_data->screen_size_bot_el = ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
  }
}

/**
 *  impeghd_mae_parse_prod_screen_sz_data_ext
 *
 *  \brief Parse mae production screen size data extension
 *
 *  \param [out] pstr_screen_sz_ext_data   Pointer to screen size extension data structure
 *  \param [in]  pstr_bit_buf              Bit buffer handle
 *
 *
 *
 */
static VOID impeghd_mae_parse_prod_screen_sz_data_ext(
    ia_production_screen_size_ext_data *pstr_screen_sz_ext_data, ia_bit_buf_struct *pstr_bit_buf)
{
  if ((pstr_screen_sz_ext_data->overwrite_prod_screen_size_data =
           ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
  {
    pstr_screen_sz_ext_data->default_screen_sz_left_az =
        ia_core_coder_read_bits_buf(pstr_bit_buf, 10);
    pstr_screen_sz_ext_data->default_screen_sz_right_az =
        ia_core_coder_read_bits_buf(pstr_bit_buf, 10);
  }
  pstr_screen_sz_ext_data->num_preset_prod_screens = ia_core_coder_read_bits_buf(pstr_bit_buf, 5);

  for (WORD32 cnt = 0; cnt < pstr_screen_sz_ext_data->num_preset_prod_screens; cnt++)
  {
    pstr_screen_sz_ext_data->screen_grp_preset_id[cnt] =
        ia_core_coder_read_bits_buf(pstr_bit_buf, 5);

    if ((pstr_screen_sz_ext_data->has_non_std_screen_sz[cnt] =
             ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      if ((pstr_screen_sz_ext_data->centered_in_az[cnt] =
               ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
      {
        pstr_screen_sz_ext_data->screen_sz_left_az[cnt] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
      }
      else
      {
        pstr_screen_sz_ext_data->screen_sz_left_az[cnt] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 10);
        pstr_screen_sz_ext_data->screen_sz_right_az[cnt] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 10);
      }
      pstr_screen_sz_ext_data->screen_sz_top_el[cnt] =
          ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
      pstr_screen_sz_ext_data->screen_sz_bot_el[cnt] =
          ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
    }
  }
}

/**
 *  impeghd_mae_parse_loudness_comp_data
 *
 *  \brief Parse loudess compensation of metadata audio element
 *
 *  \param [out]  pstr_mae_asi    Metadata audio element audio scene information
 *  \param [in]   pstr_bit_buf    Bit buffer handle
 * handle
 *
 *
 *
 */
static VOID impeghd_mae_parse_loudness_comp_data(ia_mae_audio_scene_info *pstr_mae_asi,
                                                 ia_bit_buf_struct *pstr_bit_buf)
{
  ia_loudness_compensation_data *pstr_loud_comp = &pstr_mae_asi->loud_comp_data;
  WORD32 grp, cnt;
  WORD32 num_grps = pstr_mae_asi->num_groups;
  WORD32 num_grp_presets = pstr_mae_asi->num_group_presets;

  if ((pstr_loud_comp->group_loudness_present =
           (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
  {
    for (grp = 0; grp < num_grps; grp++)
    {
      pstr_loud_comp->group_loudness[grp] = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
    }
  }
  if ((pstr_loud_comp->default_params_present = ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
  {
    for (grp = 0; grp < num_grps; grp++)
    {
      pstr_loud_comp->default_include_group[grp] =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
    }
    if ((pstr_loud_comp->default_min_max_gain_present =
             ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_loud_comp->default_min_gain = ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
      pstr_loud_comp->default_max_gain = ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
    }
  }
  for (cnt = 0; cnt < num_grp_presets; cnt++)
  {
    if ((pstr_loud_comp->preset_params_present[cnt] =
             (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      for (grp = 0; grp < num_grps; grp++)
      {
        pstr_loud_comp->preset_include_group[cnt][grp] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
      }
      if ((pstr_loud_comp->preset_min_max_gain_present[cnt] =
               (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
      {
        pstr_loud_comp->preset_min_gain[cnt] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
        pstr_loud_comp->preset_max_gain[cnt] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
      }
    }
  }
}

/**
 *  impeghd_mae_parse_drc_user_interface_info
 *
 *  \brief Parse drc user interface of metadata audio element
 *
 *  \param [out]  pstr_drc_user_ix_info  DRC user interface handle
 *  \param [in]   pstr_bit_buf        Bit buffer handle
 *
 *
 *
 */
static VOID
impeghd_mae_parse_drc_user_interface_info(ia_drc_user_interface_info *pstr_drc_user_ix_info,
                                          ia_bit_buf_struct *pstr_bit_buf)
{
  if (0 == (pstr_drc_user_ix_info->version = ia_core_coder_read_bits_buf(pstr_bit_buf, 2)))
  {
    pstr_drc_user_ix_info->num_tgt_loudness_cnd = ia_core_coder_read_bits_buf(pstr_bit_buf, 3);
    pstr_drc_user_ix_info->tgt_loudness_value[0] = -63;
    for (WORD32 cnt = 0; cnt < pstr_drc_user_ix_info->num_tgt_loudness_cnd; cnt++)
    {
      pstr_drc_user_ix_info->tgt_loudness_value[cnt + 1] =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 6);
      pstr_drc_user_ix_info->set_effect_available[cnt] =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 16);
    }
  }
}

/**
 *  impeghd_mae_asi_group_def
 *
 *  \brief Audio scene info group definition of metadata audio element
 *
 *  \param [out]  pstr_group_definition  Group definition array
 *  \param [in]   pstr_bit_buf        Bit buffer handle
 *  \param [in]   num_grps        Number of group definition
 *
 *
 *
 */
static IA_ERRORCODE impeghd_mae_asi_group_def(ia_mae_group_def *pstr_group_definition,
                                              ia_bit_buf_struct *pstr_bit_buf, WORD32 num_grps)
{
  for (WORD32 grp = 0; grp < num_grps; grp++)
  {
    pstr_group_definition[grp].group_id = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);

    /* mae_groupID shall not be the same value for different groups */
    for (WORD32 grp1 = grp - 1; grp1 >= 0; grp1--)
    {
      if (pstr_group_definition[grp].group_id == pstr_group_definition[grp1].group_id)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_GRP_ID;
      }
    }

    pstr_group_definition[grp].allow_on_off = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);

    pstr_group_definition[grp].default_on_off =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);

    if ((pstr_group_definition[grp].allow_pos_interact =
             (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_group_definition[grp].min_az_offset =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);

      pstr_group_definition[grp].max_az_offset =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);

      pstr_group_definition[grp].min_el_offset =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);

      pstr_group_definition[grp].max_el_offset =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);

      pstr_group_definition[grp].min_dist_factor =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);

      pstr_group_definition[grp].max_dist_factor =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
    }

    if ((pstr_group_definition[grp].allow_gain_interact =
             (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_group_definition[grp].min_gain = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 6);
      pstr_group_definition[grp].max_gain = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);
    }

    pstr_group_definition[grp].group_num_members =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7) + 1;

    if ((pstr_group_definition[grp].has_conjunct_members =
             (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_group_definition[grp].start_id = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    }
    else
    {
      for (WORD32 i = 0; i < pstr_group_definition[grp].group_num_members; i++)
      {
        pstr_group_definition[grp].metadata_ele_id[i] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mae_asi_switch_group_def
 *
 *  \brief Audio scene info switch group definition of metadata audio element
 *
 *  \param [out]   pstr_switch_group_definition   Switch group definition array
 *  \param [in]    pstr_bit_buf            Bit buffer handle
 *  \param [in]    num_switch_grps        Number of switch group
 * definition
 *
 *
 *
 */
static IA_ERRORCODE
impeghd_mae_asi_switch_group_def(ia_mae_switch_group_def *pstr_switch_group_definition,
                                 ia_bit_buf_struct *pstr_bit_buf, WORD32 num_switch_grps)
{
  for (WORD32 grp = 0; grp < num_switch_grps; grp++)
  {
    pstr_switch_group_definition[grp].group_id =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);

    /*mae_switchGroupID shall not be the same value for different switch groups*/
    for (WORD32 grp1 = grp - 1; grp1 >= 0; grp1--)
    {
      if (pstr_switch_group_definition[grp].group_id ==
          pstr_switch_group_definition[grp1].group_id)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_GRP_ID;
      }
    }

    if ((pstr_switch_group_definition[grp].allow_on_off =
             (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_switch_group_definition[grp].default_on_off =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
    }

    pstr_switch_group_definition[grp].group_num_members =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5) + 1;

    for (WORD32 cnt = 0; cnt < pstr_switch_group_definition[grp].group_num_members; cnt++)
    {
      pstr_switch_group_definition[grp].member_id[cnt] =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    }
    pstr_switch_group_definition[grp].default_group_id =
        ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mae_asi_group_presets_def
 *
 *  \brief Audio scene info group preset definition of
 *      metadata audio element
 *
 *  \param [out]   pstr_group_presets_definition  Group preset array
 *  \param [in]    pstr_bit_buf            Bit buffer handle
 *  \param [in]    num_grp_presets        Number of group preset
 *
 *
 *
 */
static IA_ERRORCODE
impeghd_mae_asi_group_presets_def(ia_mae_group_presets_def *pstr_group_presets_definition,
                                  ia_mae_group_def *pstr_group_definition,
                                  ia_bit_buf_struct *pstr_bit_buf, WORD32 num_grp_presets)
{
  for (WORD32 grp = 0; grp < num_grp_presets; grp++)
  {
    pstr_group_presets_definition[grp].group_id =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);

    /*mae_groupPresetID shall not be the same value for different presets*/
    for (WORD32 grp1 = grp - 1; grp1 >= 0; grp1--)
    {
      if (pstr_group_presets_definition[grp].group_id ==
          pstr_group_presets_definition[grp1].group_id)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_GRP_ID;
      }
    }
    pstr_group_presets_definition[grp].preset_kind =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);
    pstr_group_presets_definition[grp].num_conditions =
        (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4) + 1;

    if (pstr_group_presets_definition[grp].num_conditions > MAX_GROUP_PRESET_NUM_CONDITIONS)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_PR_GRP_NUM_COND;
    }

    for (WORD32 cnt = 0; cnt < pstr_group_presets_definition[grp].num_conditions; cnt++)
    {
      pstr_group_presets_definition[grp].reference_id[cnt] =
          (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);

      if ((pstr_group_presets_definition[grp].cond_on_off[cnt] =
               (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
      {
        pstr_group_presets_definition[grp].disable_gain_interact[cnt] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);

        if ((pstr_group_presets_definition[grp].gain_flag[cnt] =
                 (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
        {
          pstr_group_presets_definition[grp].gain[cnt] =
              (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
        }

        /*mae_groupPresetGainFlag shall be 0 if mae_allowGainInteractivity of the referenced group
         * is 0*/
        if ((pstr_group_definition[grp].allow_gain_interact == 0) &&
            (pstr_group_presets_definition[grp].gain_flag[cnt] != 0))
        {
          return IA_MPEGH_DEC_INIT_FATAL_ASI_PRESET_GRP_DEF_CONFIG_FAIL;
        }

        pstr_group_presets_definition[grp].disable_pos_interact[cnt] =
            (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1);

        if ((pstr_group_presets_definition[grp].position_interact[cnt] =
                 (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
        {
          pstr_group_presets_definition[grp].azimuth_offset[cnt] =
              (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 8);

          pstr_group_presets_definition[grp].elevation_offset[cnt] =
              (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 6);

          pstr_group_presets_definition[grp].dist_factor[cnt] =
              (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
        }

        /*mae_groupPresetPositionFlag shall be 0 if mae_allowPositionInteractivity of the
        referenced group is 0*/
        if ((pstr_group_definition[grp].allow_pos_interact == 0) &&
            (pstr_group_presets_definition[grp].position_interact[cnt] != 0))
        {
          return IA_MPEGH_DEC_INIT_FATAL_ASI_PRESET_GRP_DEF_CONFIG_FAIL;
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mae_asi_group_presets_def_ext
 *
 *  \brief Parse mae asi group presets definition extension
 *
 *  \param [out] pstr_grp_presets_ext_definition Pointer to group presets definition structure
 *  \param [in]  pstr_bit_buf                  Bit buffer handle
 *  \param [in]  num_grp_presets               Number of group presets
 *  \param [in]  num_conditions                 Number of conditions
 *
 *
 */
static IA_ERRORCODE impeghd_mae_asi_group_presets_def_ext(
    ia_mae_group_presets_ext_def *pstr_grp_presets_ext_definition,
    ia_mae_group_def *pstr_group_definition, ia_bit_buf_struct *pstr_bit_buf,
    WORD32 num_grp_presets, WORD32 num_conditions)
{
  WORD32 grp, cnt, idx;
  for (grp = 0; grp < num_grp_presets; grp++)
  {
    if ((pstr_grp_presets_ext_definition->has_switch_group_conditions[grp] =
             ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      for (cnt = 0; cnt < num_conditions; cnt++)
      {
        pstr_grp_presets_ext_definition->is_switch_group_condition[grp][cnt] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
      }
    }
    if ((pstr_grp_presets_ext_definition->has_downmix_id_group_preset_extensions[grp] =
             ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_grp_presets_ext_definition->num_downmix_id_group_preset_extensions[grp] =
          ia_core_coder_read_bits_buf(pstr_bit_buf, 5) + 1;

      if (pstr_grp_presets_ext_definition->num_downmix_id_group_preset_extensions[grp] >
          MAX_NUM_DM_ID)
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_NUM_DM_ID_GRP_PR_EXT;
      }
      for (cnt = 0;
           cnt < pstr_grp_presets_ext_definition->num_downmix_id_group_preset_extensions[grp];
           cnt++)
      {
        pstr_grp_presets_ext_definition->group_preset_downmix_id[grp][cnt] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
        pstr_grp_presets_ext_definition->group_preset_num_conditions[grp][cnt] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 4);

        for (idx = 0;
             idx < pstr_grp_presets_ext_definition->group_preset_num_conditions[grp][cnt]; idx++)
        {
          if ((pstr_grp_presets_ext_definition
                   ->downmix_id_is_switch_group_condition[grp][cnt][idx] =
                   ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
          {
            pstr_grp_presets_ext_definition->group_preset_switch_group_id[grp][cnt][idx] =
                ia_core_coder_read_bits_buf(pstr_bit_buf, 5);
          }
          else
          {
            pstr_grp_presets_ext_definition->group_preset_group_id[grp][cnt][idx] =
                ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
          }
          if ((pstr_grp_presets_ext_definition->group_preset_condition_on_off[grp][cnt][idx] =
                   ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
          {
            pstr_grp_presets_ext_definition
                ->group_preset_disable_gain_interactivity[grp][cnt][idx] =
                ia_core_coder_read_bits_buf(pstr_bit_buf, 1);

            /*mae_groupPresetDisableGainInteractivity shall be 1 if mae_allowGainInteractivity of
            the
            referenced group is 0*/
            if ((pstr_group_definition[grp].allow_gain_interact == 0) &&
                (pstr_grp_presets_ext_definition
                     ->group_preset_disable_gain_interactivity[grp][cnt][idx] != 1))
            {
              return IA_MPEGH_DEC_INIT_FATAL_ASI_PRESET_GRP_DEF_EXT_CONFIG_FAIL;
            }

            if ((pstr_grp_presets_ext_definition->group_preset_gain_flag[grp][cnt][idx] =
                     ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
            {
              pstr_grp_presets_ext_definition->group_preset_gain[grp][cnt][idx] =
                  ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
            }

            /*mae_groupPresetGainFlag shall be 0 if mae_allowGainInteractivity of the
            referenced group is 0*/
            if ((pstr_group_definition[grp].allow_gain_interact == 0) &&
                (pstr_grp_presets_ext_definition->group_preset_gain_flag[grp][cnt][idx] != 0))
            {
              return IA_MPEGH_DEC_INIT_FATAL_ASI_PRESET_GRP_DEF_EXT_CONFIG_FAIL;
            }
            pstr_grp_presets_ext_definition
                ->group_preset_disable_pos_interactiviy[grp][cnt][idx] =
                ia_core_coder_read_bits_buf(pstr_bit_buf, 1);

            /*mae_groupPresetDisablePositionInteractivity shall be 1 if
            mae_allowPositionInteractivity of the
            referenced group is 0*/
            if ((pstr_group_definition[grp].allow_pos_interact == 0) &&
                (pstr_grp_presets_ext_definition
                     ->group_preset_disable_pos_interactiviy[grp][cnt][idx] != 1))
            {
              return IA_MPEGH_DEC_INIT_FATAL_ASI_PRESET_GRP_DEF_EXT_CONFIG_FAIL;
            }

            if ((pstr_grp_presets_ext_definition->group_preset_position_flag[grp][cnt][idx] =
                     ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
            {
              pstr_grp_presets_ext_definition->group_preset_az_offset[grp][cnt][idx] =
                  ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
              pstr_grp_presets_ext_definition->group_preset_el_offset[grp][cnt][idx] =
                  ia_core_coder_read_bits_buf(pstr_bit_buf, 6);
              pstr_grp_presets_ext_definition->group_preset_dist_fac[grp][cnt][idx] =
                  ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
            }

            /*mae_groupPresetPositionFlag shall be 0 if mae_allowPositionInteractivity of the
            referenced group is 0*/
            if ((pstr_group_definition[grp].allow_pos_interact == 0) &&
                (pstr_grp_presets_ext_definition->group_preset_position_flag[grp][cnt][idx] != 0))
            {
              return IA_MPEGH_DEC_INIT_FATAL_ASI_PRESET_GRP_DEF_EXT_CONFIG_FAIL;
            }
          }
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_sign_extend_16
 *
 *  \brief Helper function to extend the sign bit to MSB
 *
 *  \param [in] pcm_val 16-bit PCM value
 *
 *  \return WORD32      32-bit signed PCM value
 *
 */
static WORD32 ia_sign_extend_16(WORD32 pcm_val)
{
  WORD32 val = (0x0000FFFF & pcm_val);
  WORD32 mask = 0x00008000;
  if (mask & pcm_val)
  {
    val += 0xFFFF0000;
  }
  return val;
}

/**
 *  ia_sign_extend_24
 *
 *  \brief Helper function to extend the sign bit to MSB
 *
 *  \param [in] pcm_val 24-bit PCM value
 *
 *  \return WORD32      32-bit signed PCM value
 *
 */
static WORD32 ia_sign_extend_24(WORD32 pcm_val)
{
  WORD32 val = (0x00FFFFFF & pcm_val);
  WORD32 mask = 0x00800000;
  if (mask & pcm_val)
  {
    val += 0xFF000000;
  }
  return val;
}
/**
 *  impeghd_pcm_data_payload
 *
 *  \brief  description               Parsing of pcm data payload packet
 *
 *  \param [out]  pstr_pcm_data_config Pointer to pcm gata configuration structure
 *  \param [in]   pstr_bit_buf        Bit buffer handle
 *
 *  \return IA_ERRORCODE error if any.
 *
 */
IA_ERRORCODE impeghd_pcm_data_payload(ia_pcm_data_config *pstr_pcm_data_config,
                                      ia_bit_buf_struct *pstr_bit_buf)
{

  WORD32 cnt, idx, pcm_fix_frame_size;
  pstr_pcm_data_config->bsnum_pcm_signals_in_frame = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);

  if (pstr_pcm_data_config->bsnum_pcm_signals_in_frame > (MAX_NUM_CHANNELS - 1))
  {
    pstr_pcm_data_config->bsnum_pcm_signals_in_frame = 0;
    return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_NUM_PCM_SIG;
  }

  for (cnt = 0; cnt < pstr_pcm_data_config->bsnum_pcm_signals_in_frame + 1; cnt++)
  {
    pstr_pcm_data_config->pcm_signal_id[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  }

  if (2 == pstr_pcm_data_config->pcm_has_attenuation_gain)
  {
    pstr_pcm_data_config->bs_pcm_attenuation_gain = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
  }

  if (6 == pstr_pcm_data_config->pcm_frame_size_idx)
  {
    pstr_pcm_data_config->pcm_var_frame_size = ia_core_coder_read_bits_buf(pstr_bit_buf, 16);
  }

  switch (pstr_pcm_data_config->pcm_frame_size_idx)
  {
  case 0:
    return IA_MPEGH_DEC_NO_ERROR;
    break;
  case 5:
    pstr_pcm_data_config->pcm_frame_size_idx = pstr_pcm_data_config->pcm_fix_frame_size;
    pcm_fix_frame_size = ia_pcm_frame_size_tbl[pstr_pcm_data_config->pcm_frame_size_idx];
    break;
  case 6:
    pstr_pcm_data_config->pcm_frame_size_idx = pstr_pcm_data_config->pcm_var_frame_size;
    pcm_fix_frame_size = ia_pcm_frame_size_tbl[pstr_pcm_data_config->pcm_frame_size_idx];
    break;
  default:
    pcm_fix_frame_size = ia_pcm_frame_size_tbl[pstr_pcm_data_config->pcm_frame_size_idx];
    break;
  }

  WORD32 pcm_sample, temp, temp_num;
  for (cnt = 0; cnt < pcm_fix_frame_size; cnt++)
  {
    for (idx = 0; idx < pstr_pcm_data_config->bsnum_pcm_signals_in_frame + 1; idx++)
    {
      switch (pstr_pcm_data_config->pcm_bits_per_sample)
      {
      case 16:
        pcm_sample =
            ia_core_coder_read_bits_buf(pstr_bit_buf, pstr_pcm_data_config->pcm_bits_per_sample);
        pcm_sample = ia_sign_extend_16(pcm_sample);
        pstr_pcm_data_config->pcm_sample[idx][cnt] = (FLOAT32)pcm_sample;
        break;
      case 24:
        pcm_sample =
            ia_core_coder_read_bits_buf(pstr_bit_buf, pstr_pcm_data_config->pcm_bits_per_sample);
        pcm_sample = ia_sign_extend_24(pcm_sample);
        pstr_pcm_data_config->pcm_sample[idx][cnt] = (FLOAT32)pcm_sample / 256.0f;
        break;
      case 32:
        pcm_sample = 0;
        temp_num = 32;
        while (temp_num)
        {
          temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
          pcm_sample <<= 8;
          pcm_sample |= temp;
          temp_num -= 8;
        }
        pstr_pcm_data_config->pcm_sample[idx][cnt] = (FLOAT32)pcm_sample / 65536.0f;
        break;
      default:
        pcm_sample =
            ia_core_coder_read_bits_buf(pstr_bit_buf, pstr_pcm_data_config->pcm_bits_per_sample);
        pstr_pcm_data_config->pcm_sample[idx][cnt] = (FLOAT32)pcm_sample;
        break;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_pcm_data_config
 *
 *  \brief  description               Parsing of pcm data config packet
 *
 *  \param [out]  pstr_pcm_data_config Pointer to pcm gata configuration structure
 *  \param [in]   pstr_bit_buf        Bit buffer handle
 *
 *  \return IA_ERRORCODE error if any.
 *
 */
IA_ERRORCODE impeghd_pcm_data_config(ia_pcm_data_config *pstr_pcm_data_config,
                                     ia_bit_buf_struct *pstr_bit_buf)
{
  pstr_pcm_data_config->bs_num_pcm_signals = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  pstr_pcm_data_config->pcm_align_audio_flag = ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
  pstr_pcm_data_config->pcm_sampling_rate_idx = ia_core_coder_read_bits_buf(pstr_bit_buf, 5);
  pstr_pcm_data_config->pcm_sampling_rate =
      ia_sampling_rate_tbl[pstr_pcm_data_config->pcm_sampling_rate_idx];
  if (0x1f == pstr_pcm_data_config->pcm_sampling_rate_idx)
  {
    pstr_pcm_data_config->pcm_sampling_rate = ia_core_coder_read_bits_buf(pstr_bit_buf, 24);
  }
  pstr_pcm_data_config->pcm_bits_per_sample_idx = ia_core_coder_read_bits_buf(pstr_bit_buf, 3);
  if (2 < pstr_pcm_data_config->pcm_bits_per_sample_idx)
  {
    return IA_MPEGH_DEC_NO_ERROR;
  }
  pstr_pcm_data_config->pcm_bits_per_sample =
      ia_pcm_bits_per_sample_table[pstr_pcm_data_config->pcm_bits_per_sample_idx];
  pstr_pcm_data_config->pcm_frame_size_idx = ia_core_coder_read_bits_buf(pstr_bit_buf, 3);

  if (5 == pstr_pcm_data_config->pcm_frame_size_idx)
  {
    pstr_pcm_data_config->pcm_fix_frame_size = ia_core_coder_read_bits_buf(pstr_bit_buf, 16);
  }

  for (WORD32 cnt = 0; cnt < pstr_pcm_data_config->bs_num_pcm_signals + 1; cnt++)
  {
    pstr_pcm_data_config->pcm_signal_id[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  }

  pstr_pcm_data_config->bs_pcm_loudness_value = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
  pstr_pcm_data_config->pcm_has_attenuation_gain = ia_core_coder_read_bits_buf(pstr_bit_buf, 2);

  if (1 == pstr_pcm_data_config->pcm_has_attenuation_gain)
  {
    pstr_pcm_data_config->bs_pcm_attenuation_gain = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_econ_config_info
 *
 *  \brief Parsing of earcon configuration information
 *
 *  \param [out]   pstr_earcon_info Pointer to earcon information structure
 *  \param [in]    pstr_bit_buf     Bit buffer handle
 *
 *  \return IA_ERRORCODE            Processing error if any
 *
 */
IA_ERRORCODE impeghd_econ_config_info(ia_earcon_info *pstr_earcon_info,
                                      ia_bit_buf_struct *pstr_bit_buf)
{
  WORD32 num_earcons = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  if (num_earcons > MAX_NUM_EARCONS)
  {
    return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_NUM_EARCONS;
  }

  for (WORD32 cnt = 0; cnt < num_earcons + 1; cnt++)
  {
    pstr_earcon_info->earcon_is_independent[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
    pstr_earcon_info->earcon_id[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    pstr_earcon_info->earcon_type[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
    pstr_earcon_info->earcon_active[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 1);
    pstr_earcon_info->earcon_pos_type[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 2);
    switch (pstr_earcon_info->earcon_pos_type[cnt])
    {
    case 0:
      pstr_earcon_info->earcon_cicp_spk_idx[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
      break;
    case 1:
    {
      pstr_earcon_info->earcon_azimuth[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      pstr_earcon_info->earcon_elevation[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 6);
      pstr_earcon_info->earcon_distance[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 9);
    }
    break;
    default:
    {
      pstr_earcon_info->earcon_azimuth[cnt] = 0;
      pstr_earcon_info->earcon_elevation[cnt] = 0;
      pstr_earcon_info->earcon_distance[cnt] = DEFAULT_REF_DIST;
    }
    break;
    }

    if ((pstr_earcon_info->earcon_has_gain = ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_earcon_info->earcon_gain[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    }

    if ((pstr_earcon_info->earcon_has_text_label = ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_earcon_info->earcon_num_languages[cnt] = ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
    }

    for (WORD32 idx = 0; idx < pstr_earcon_info->earcon_num_languages[cnt]; idx++)
    {
      pstr_earcon_info->earcon_languages[cnt][idx] =
          ia_core_coder_read_bits_buf(pstr_bit_buf, 24);
      pstr_earcon_info->earcon_text_data_length[cnt][idx] =
          ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      for (WORD32 len = 0; len < pstr_earcon_info->earcon_text_data_length[cnt][idx]; len++)
      {
        pstr_earcon_info->earcon_text_data[cnt][idx][len] =
            ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_mae_asi_data_parse
 *
 *  \brief Parse audio screen info data of metadata audio element
 *
 *  \param [out] pstr_mae_asi Metadata audio element audio scene information handle
 *  \param [in]  pstr_bit_buf Bit buffer handle
 *
 *  \return IA_ERRORCODE      Processing error if any
 *
 */
static IA_ERRORCODE impeghd_mae_asi_data_parse(ia_mae_audio_scene_info *pstr_mae_asi,
                                               ia_bit_buf_struct *pstr_bit_buf)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_mae_data *pstr_mae_data = &pstr_mae_asi->mae_data;
  WORD32 data_len;
  WORD32 num_loud_com = 0, num_grp_pr_ext = 0, num_scrn_sz_ext = 0;
  WORD32 num_drc_ui_info = 0, num_grp_pr_des = 0, num_scrn_sz = 0;
  WORD32 num_grp_comp = 0, num_grp_cont = 0, num_grp_des = 0, num_sw_grp_des = 0;

  pstr_mae_data->num_data_sets = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);

  for (WORD32 cnt = 0; cnt < pstr_mae_data->num_data_sets; cnt++)
  {
    WORD32 cnt_bits = 0;
    pstr_mae_data->data_type[cnt] = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 4);
    data_len = ia_core_coder_read_bits_buf(pstr_bit_buf, 16);
    pstr_mae_data->data_length[cnt] = (WORD16)data_len;
    cnt_bits = pstr_bit_buf->cnt_bits;
    switch (pstr_mae_data->data_type[cnt])
    {
    case ID_MAE_LOUDNESS_COMPENSATION:
      /*mae_dataType each data type shall only occur once at most*/
      num_loud_com++;
      if (num_loud_com > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      impeghd_mae_parse_loudness_comp_data(pstr_mae_asi, pstr_bit_buf);
      break;
    case ID_MAE_GROUP_PRESET_EXTENSION:
      num_grp_pr_ext++;
      if (num_grp_pr_ext > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      err_code = impeghd_mae_asi_group_presets_def_ext(
          &pstr_mae_asi->group_presets_ext_definition, &pstr_mae_asi->group_definition[0],
          pstr_bit_buf, pstr_mae_asi->num_group_presets,
          pstr_mae_asi->group_presets_definition[0].num_conditions);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }
      break;
    case ID_MAE_SCREEN_SIZE_EXTENSION:
      num_scrn_sz_ext++;
      if (num_scrn_sz_ext > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      impeghd_mae_parse_prod_screen_sz_data_ext(&pstr_mae_asi->screen_size_ext_data,
                                                pstr_bit_buf);
      break;
    case ID_MAE_DRC_UI_INFO:
      num_drc_ui_info++;
      if (num_drc_ui_info > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      impeghd_mae_parse_drc_user_interface_info(&pstr_mae_asi->drc_interface_info, pstr_bit_buf);
      break;
    case ID_MAE_GROUP_PRESET_DESCRIPTION:
      num_grp_pr_des++;
      if (num_grp_pr_des > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      err_code =
          impeghd_mae_parse_description_data(&pstr_mae_asi->preset_desc_data, pstr_bit_buf, 5);
      if (err_code)
      {
        return err_code;
      }
      break;
    case ID_MAE_SCREEN_SIZE:
      num_scrn_sz++;
      if (num_scrn_sz > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      impeghd_mae_parse_prod_screen_sz_data(&pstr_mae_asi->screen_size_data, pstr_bit_buf);
      break;
    case ID_MAE_GROUP_COMPOSITE:
      num_grp_comp++;
      if (num_grp_comp > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      impeghd_mae_parse_composite_pair(&pstr_mae_asi->composite_pair_data, pstr_bit_buf);
      break;
    case ID_MAE_GROUP_CONTENT:
      num_grp_cont++;
      if (num_grp_cont > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      impeghd_mae_parse_content_data(&pstr_mae_asi->content_data, pstr_bit_buf);
      break;
    case ID_MAE_SWITCHGROUP_DESCRIPTION:
      num_sw_grp_des++;
      if (num_sw_grp_des > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      err_code = impeghd_mae_parse_description_data(&pstr_mae_asi->switch_group_desc_data,
                                                    pstr_bit_buf, 5);
      if (err_code)
      {
        return err_code;
      }
      break;
    case ID_MAE_GROUP_DESCRIPTION:
      num_grp_des++;
      if (num_grp_des > 1)
      {
        return IA_MPEGH_DEC_INIT_FATAL_ASI_PARSE_FAIL;
      }
      err_code =
          impeghd_mae_parse_description_data(&pstr_mae_asi->group_desc_data, pstr_bit_buf, 7);
      if (err_code)
      {
        return err_code;
      }
      break;
    default:
      while (data_len > 0)
      {
        ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
        data_len--;
      }
      break;
    }
    cnt_bits = cnt_bits - pstr_bit_buf->cnt_bits;
    data_len = pstr_mae_data->data_length[cnt];
    if (cnt_bits < (data_len << 3))
    {
      WORD32 skip_bits = (data_len << 3) - cnt_bits;
      ia_core_coder_skip_bits_buf(pstr_bit_buf, skip_bits);
    }
  }
  return err_code;
}

/**
 *  impeghd_mae_asi_parse
 *
 *  \brief Parse audio scene info of metadata audio element
 *
 *  \param [out]  pstr_mae_asi    Metadata audio element audio scene information
 * handle
 *  \param [in]    pstr_bit_buf    Bit buffer handle
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE impeghd_mae_asi_parse(ia_mae_audio_scene_info *pstr_mae_asi,
                                   ia_bit_buf_struct *pstr_bit_buf)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  if ((pstr_mae_asi->main_stream_flag = ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
  {
    if ((pstr_mae_asi->asi_id_present = ia_core_coder_read_bits_buf(pstr_bit_buf, 1)))
    {
      pstr_mae_asi->asi_id = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
    }
    /* ASI - Group Definition*/
    pstr_mae_asi->num_groups = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    if (pstr_mae_asi->num_groups > MAX_NUM_GROUPS)
    {
      pstr_mae_asi->num_groups = 0;
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_GROUP;
    }
    err_code = impeghd_mae_asi_group_def(&pstr_mae_asi->group_definition[0], pstr_bit_buf,
                                         pstr_mae_asi->num_groups);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }

    /* ASI - Switch Group Definition*/
    pstr_mae_asi->num_switch_groups = ia_core_coder_read_bits_buf(pstr_bit_buf, 5);
    if (pstr_mae_asi->num_switch_groups > MAX_NUM_SWITCH_GROUPS)
    {
      pstr_mae_asi->num_switch_groups = 0;
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_SWITCH_GROUPS;
    }
    err_code = impeghd_mae_asi_switch_group_def(&pstr_mae_asi->switch_group_definition[0],
                                                pstr_bit_buf, pstr_mae_asi->num_switch_groups);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }

    /* ASI - Group presets Definition*/
    pstr_mae_asi->num_group_presets = (WORD8)ia_core_coder_read_bits_buf(pstr_bit_buf, 5);
    if (pstr_mae_asi->num_group_presets > MAX_NUM_GROUPS_PRESETS)
    {
      pstr_mae_asi->num_group_presets = 0;
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ASI_NUM_GROUPS_PRESETS;
    }
    err_code = impeghd_mae_asi_group_presets_def(&pstr_mae_asi->group_presets_definition[0],
                                                 &pstr_mae_asi->group_definition[0], pstr_bit_buf,
                                                 pstr_mae_asi->num_group_presets);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }

    /* ASI - MAE Data*/
    err_code = impeghd_mae_asi_data_parse(pstr_mae_asi, pstr_bit_buf);
    if (err_code)
    {
      return err_code;
    }

    pstr_mae_asi->mae_id_offset = 0;
    pstr_mae_asi->mae_id_max_avail = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
    pstr_mae_asi->mae_id_offset = 0;
  }
  else
  {
    pstr_mae_asi->mae_id_offset = ia_core_coder_read_bits_buf(pstr_bit_buf, 7) + 1;
    pstr_mae_asi->mae_id_max_avail = ia_core_coder_read_bits_buf(pstr_bit_buf, 7);
  }
  return err_code;
}

/**
 *  impeghd_mhas_parse
 *
 *  \brief Parse MPEG-H 3D audio stream
 *
 *  \param [out]  pstr_pac_info  MHAS packet info handle
 *  \param [out]  pstr_mae_asi    Metadata audio element audio scene information
 * handle
 *  \param [in]    pstr_bit_buf    Bit buffer handle
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_mhas_parse(ia_mhas_pac_info *pstr_pac_info,
                                ia_mae_audio_scene_info *pstr_mae_asi,
                                ia_bit_buf_struct *pstr_bit_buf)
{
  WORD32 pkt_type, pkt_lbl, pkt_len, temp;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  pstr_mae_asi->pcm_data_config.pcm_packet_type_present = 0;
  pstr_mae_asi->pcm_data_config.pcm_packet_data_present = 0;
  do
  {
    temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 3);
    if (temp == 7)
    {
      pkt_type = temp;
      temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      if (temp == 255)
      {
        pkt_type = pkt_type + temp;
        temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
        pkt_type = pkt_type + temp;
      }
      else
      {
        pkt_type += temp;
      }
    }
    else
    {
      pkt_type = temp;
    }
    temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 2);
    if (temp == 3)
    {
      pkt_lbl = temp;
      temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      if (temp == 255)
      {
        pkt_lbl = pkt_lbl + temp;
        temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 32);
        pkt_lbl = pkt_lbl + temp;
      }
      else
      {
        pkt_lbl += temp;
      }
    }
    else
    {
      pkt_lbl = temp;
    }
    temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 11);
    if (temp == 2047)
    {
      pkt_len = temp;
      temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 24);
      if (temp == (WORD32)((1 << 24) - 1))
      {
        pkt_len = pkt_len + temp;
        temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 24);
        pkt_len = pkt_len + temp;
      }
      else
      {
        pkt_len += temp;
      }
    }
    else
    {
      pkt_len = temp;
    }

    switch (pkt_type)
    {
    case MHAS_PAC_TYP_AUDIOSCENEINFO:
    {
      WORD32 cnt_bits = pstr_bit_buf->cnt_bits;
      err_code = impeghd_mae_asi_parse(pstr_mae_asi, pstr_bit_buf);
      if (err_code)
      {
        return err_code;
      }
      pstr_mae_asi->asi_present = 1;
      cnt_bits = cnt_bits - pstr_bit_buf->cnt_bits;
      if (cnt_bits < (pkt_len << 3))
      {
        WORD32 skip_bits = (pkt_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(pstr_bit_buf, skip_bits);
      }
    }
    break;
    case MHAS_PAC_TYP_SYNC:
    {
      temp = ia_core_coder_read_bits_buf(pstr_bit_buf, 8);
      if (MHAS_SYNC_BYTE != temp || 1 != pkt_len)
      {
        return IA_MPEGH_DEC_INIT_FATAL_MHAS_SYNCWORD_MISMATCH;
      }
    }
    break;
    case MHAS_PAC_TYP_USERINTERACTION:
    {
      pstr_mae_asi->ei_present = 1;
      /* Currently these bytes are skipped.*/
      temp = pkt_len << 3;
      ia_core_coder_skip_bits_buf(pstr_bit_buf, temp);
    }
    break;
    case MHAS_PAC_TYP_EARCON:
    {
      WORD32 cnt_bits = pstr_bit_buf->cnt_bits;
      pstr_mae_asi->earcon_info.earcon_present = 1;
      err_code = impeghd_econ_config_info(&pstr_mae_asi->earcon_info, pstr_bit_buf);
      if (err_code)
      {
        return err_code;
      }
      cnt_bits = cnt_bits - pstr_bit_buf->cnt_bits;
      if (cnt_bits < (pkt_len << 3))
      {
        WORD32 skip_bits = (pkt_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(pstr_bit_buf, skip_bits);
      }
    }
    break;
    case MHAS_PAC_TYP_PCMCONFIG:
    {
      WORD32 cnt_bits = pstr_bit_buf->cnt_bits;

      pstr_mae_asi->pcm_data_config.pcm_packet_type_present = 1;
      impeghd_pcm_data_config(&pstr_mae_asi->pcm_data_config, pstr_bit_buf);
      cnt_bits = cnt_bits - pstr_bit_buf->cnt_bits;
      if (cnt_bits < (pkt_len << 3))
      {
        WORD32 skip_bits = (pkt_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(pstr_bit_buf, skip_bits);
      }
    }
    break;
    case MHAS_PAC_TYP_PCMDATA:
    {
      WORD32 cnt_bits = pstr_bit_buf->cnt_bits;
      pstr_mae_asi->pcm_data_config.pcm_packet_type_present = 1;
      pstr_mae_asi->pcm_data_config.pcm_packet_data_present = 1;
      pstr_mae_asi->pcm_data_config.pcm_config_present = 1;
      impeghd_pcm_data_payload(&pstr_mae_asi->pcm_data_config, pstr_bit_buf);
      cnt_bits = cnt_bits - pstr_bit_buf->cnt_bits;
      if (cnt_bits < (pkt_len << 3))
      {
        WORD32 skip_bits = (pkt_len << 3) - cnt_bits;
        ia_core_coder_skip_bits_buf(pstr_bit_buf, skip_bits);
      }
    }
    break;
    case MHAS_PAC_TYP_LOUDNESS:
    {
      ia_drc_payload_struct str_drc_payload;
      err_code = impd_drc_mpegh3da_parse_loudness_info_set(&str_drc_payload.str_loud_info,
                                                           pstr_bit_buf);
    }
    break;
    default:
      if ((MHAS_PAC_TYP_MPEGH3DACFG != pkt_type) && (MHAS_PAC_TYP_MPEGH3DAFRAME != pkt_type))
      {
        temp = pkt_len << 3;
        ia_core_coder_skip_bits_buf(pstr_bit_buf, temp);
      }
      break;
    }
  } while ((MHAS_PAC_TYP_MPEGH3DACFG != pkt_type) && (MHAS_PAC_TYP_MPEGH3DAFRAME != pkt_type) &&
           (1 != pstr_mae_asi->pcm_data_config.pcm_packet_type_present));

  pstr_pac_info->packet_type = pkt_type;
  pstr_pac_info->packet_lbl = pkt_lbl;
  pstr_pac_info->packet_length = pkt_len;

  return err_code;
} /** @} */ /* End of MHASParse */