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

#include <impeghd_type_def.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_memory_standards.h"
#include "impeghd_mhas_parse.h"

/**
 *  impeghd_mp4_get_mael
 *
 *  \brief Brief description   Function to parse mael box
 *
 *  \param ptr_read_buff              pointer to bit buffer
 *  \param ptr_mae_audio_scene_info   pointer to asi struct
 *
 *  \return
 *
 */
VOID impeghd_mp4_get_mael(ia_bit_buf_struct *ptr_read_buff,
                          ia_mae_audio_scene_info *ptr_mae_audio_scene_info)
{
  WORD32 k, j;

  ptr_mae_audio_scene_info->mae_data.num_data_sets = 3;
  ptr_mae_audio_scene_info->mae_data.data_type[0] = 0;
  ptr_mae_audio_scene_info->mae_data.data_type[1] = 1;
  ptr_mae_audio_scene_info->mae_data.data_type[2] = 5;
  ia_core_coder_read_bits_buf(ptr_read_buff, 4); // reserved bits
  ptr_mae_audio_scene_info->group_desc_data.num_descr_languages[0] =
      ia_core_coder_read_bits_buf(ptr_read_buff, 4) + 1; // description languages
  ptr_mae_audio_scene_info->group_desc_data.descr_language[0][0] = 0;
  ia_core_coder_read_bits_buf(ptr_read_buff, 24);
  ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved bits
  ptr_mae_audio_scene_info->group_desc_data.num_descr_blks =
      ia_core_coder_read_bits_buf(ptr_read_buff, 7) + 1; // reserved bits
  for (j = 0; j < ptr_mae_audio_scene_info->group_desc_data.num_descr_blks; j++)
  {

    ptr_mae_audio_scene_info->group_desc_data.num_descr_languages[j] = 1;
    ptr_mae_audio_scene_info->group_desc_data.descr_language[j][0] = 0;
    ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved bits
    ptr_mae_audio_scene_info->group_desc_data.grp_id[j] =
        ia_core_coder_read_bits_buf(ptr_read_buff, 7);
    ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0] = 1;
    ia_core_coder_read_bits_buf(ptr_read_buff, 8);
    for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0]; k++)
    {
      ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;
      ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    }
    ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;
  }
  ptr_mae_audio_scene_info->group_desc_data.num_descr_languages[j] = 1;
  ptr_mae_audio_scene_info->group_desc_data.descr_language[j][0] = 0;

  ptr_mae_audio_scene_info->group_desc_data.grp_id[j] = 0;
  ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0] = 1;

  for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0] + 1; k++)
  {
    ptr_mae_audio_scene_info->group_desc_data.descr_data[j][0][k] = 0;
  }

  ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
  ptr_mae_audio_scene_info->switch_group_desc_data.num_descr_blks =
      ia_core_coder_read_bits_buf(ptr_read_buff, 5) + 1;
  for (j = 0; j < ptr_mae_audio_scene_info->switch_group_desc_data.num_descr_blks; j++)
  {
    ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->switch_group_desc_data.num_descr_languages[j] = 1;
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_language[j][0] = 0;
    ptr_mae_audio_scene_info->switch_group_desc_data.grp_id[j] =
        ia_core_coder_read_bits_buf(ptr_read_buff, 5);
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_data_length[j][0] = 1;
    ia_core_coder_read_bits_buf(ptr_read_buff, 8);
    for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0]; k++)
    {
      ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] =
          ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    }
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] = 0;
  }
  ptr_mae_audio_scene_info->switch_group_desc_data.num_descr_languages[j] = 1;
  ptr_mae_audio_scene_info->switch_group_desc_data.descr_language[j][0] = 0;
  ptr_mae_audio_scene_info->switch_group_desc_data.grp_id[j] = 0;

  ptr_mae_audio_scene_info->switch_group_desc_data.descr_data_length[j][0] = 1;

  for (k = 0; k < ptr_mae_audio_scene_info->group_desc_data.descr_data_length[j][0] + 1; k++)
  {
    ptr_mae_audio_scene_info->switch_group_desc_data.descr_data[j][0][k] = 0;
  }

  ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
  ptr_mae_audio_scene_info->preset_desc_data.num_descr_blks =
      ia_core_coder_read_bits_buf(ptr_read_buff, 5) + 1;
  for (j = 0; j < ptr_mae_audio_scene_info->preset_desc_data.num_descr_blks; j++)
  {
    ptr_mae_audio_scene_info->preset_desc_data.num_descr_languages[j] = 1;
    ptr_mae_audio_scene_info->preset_desc_data.descr_language[j][0] = 0;
    ptr_mae_audio_scene_info->preset_desc_data.grp_id[j] =
        ia_core_coder_read_bits_buf(ptr_read_buff, 5);
    ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0] = 1;
    ia_core_coder_read_bits_buf(ptr_read_buff, 8);
    for (k = 0; k < ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0]; k++)
    {
      ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;
      ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    }
    ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;
  }

  ptr_mae_audio_scene_info->preset_desc_data.num_descr_languages[j] = 1;
  ptr_mae_audio_scene_info->preset_desc_data.descr_language[j][0] = 0;
  ptr_mae_audio_scene_info->preset_desc_data.grp_id[j] = 0;

  ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0] = 1;

  for (k = 0; k < ptr_mae_audio_scene_info->preset_desc_data.descr_data_length[j][0] + 1; k++)
  {
    ptr_mae_audio_scene_info->preset_desc_data.descr_data[j][0][k] = 0;
  }
}
/**
 *  impeghd_mp4_get_maep
 *
 *  \brief Brief description   Function to parse maep box
 *
 *  \param ptr_read_buff              pointer to bit buffer
 *  \param ptr_mae_audio_scene_info   pointer to asi struct
 *
 *  \return
 *
 */
VOID impeghd_mp4_get_maep(ia_bit_buf_struct *ptr_read_buff,
                          ia_mae_audio_scene_info *ptr_mae_audio_scene_info)
{
  WORD32 i, j, num_conditions, num_group_presets;
  ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved bits
  ptr_mae_audio_scene_info->num_group_presets =
      ia_core_coder_read_bits_buf(ptr_read_buff, 7); // no:of groups
  num_group_presets = ptr_mae_audio_scene_info->num_group_presets;
  for (i = 0; i < num_group_presets; i++)
  {
    ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->group_presets_definition[i].group_id =
        ia_core_coder_read_bits_buf(ptr_read_buff, 5); // preset id

    ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->group_presets_definition[i].preset_kind =
        ia_core_coder_read_bits_buf(ptr_read_buff, 5);
    ptr_mae_audio_scene_info->group_presets_definition[i].num_conditions =
        ia_core_coder_read_bits_buf(ptr_read_buff, 8);
    num_conditions = ptr_mae_audio_scene_info->group_presets_definition[i].num_conditions;
    for (j = 0; j < num_conditions; j++)
    {
      ptr_mae_audio_scene_info->group_presets_definition[i].reference_id[j] =
          ia_core_coder_read_bits_buf(ptr_read_buff, 7); // preset group id

      ptr_mae_audio_scene_info->group_presets_definition[i].cond_on_off[j] =
          ia_core_coder_read_bits_buf(ptr_read_buff, 1);
      if (ptr_mae_audio_scene_info->group_presets_definition[i].cond_on_off[j])
      {
        ia_core_coder_read_bits_buf(ptr_read_buff, 4); // reserved bits
        ptr_mae_audio_scene_info->group_presets_definition[i].disable_gain_interact[j] =
            ia_core_coder_read_bits_buf(ptr_read_buff, 1); // group disable gain Interactvity

        ptr_mae_audio_scene_info->group_presets_definition[i].gain_flag[j] =
            ia_core_coder_read_bits_buf(ptr_read_buff, 1);

        ptr_mae_audio_scene_info->group_presets_definition[i].disable_pos_interact[j] =
            ia_core_coder_read_bits_buf(ptr_read_buff, 1); // disable position interactivity
        ptr_mae_audio_scene_info->group_presets_definition[i].position_interact[j] =
            ia_core_coder_read_bits_buf(ptr_read_buff, 1); // position flag

        if (ptr_mae_audio_scene_info->group_presets_definition[i].gain_flag[j])
        {
          ptr_mae_audio_scene_info->group_presets_definition[i].gain[j] =
              ia_core_coder_read_bits_buf(ptr_read_buff, 8);
        }
        if (ptr_mae_audio_scene_info->group_presets_definition[i].position_interact[j])
        {
          ptr_mae_audio_scene_info->group_presets_definition[i].azimuth_offset[j] =
              ia_core_coder_read_bits_buf(ptr_read_buff, 8); // azimuth offset

          ptr_mae_audio_scene_info->group_presets_definition[i].elevation_offset[j] =
              ia_core_coder_read_bits_buf(ptr_read_buff, 8); // elevation offset

          ptr_mae_audio_scene_info->group_presets_definition[i].dist_factor[j] =
              ia_core_coder_read_bits_buf(ptr_read_buff, 8); // dist factor
        }
      }
    }
  }
}
/**
 *  impeghd_mp4_get_maes
 *
 *  \brief Brief description  Function to parse maes box
 *
 *  \param ptr_read_buff              pointer to bit buffer
 *  \param ptr_mae_audio_scene_info   pointer to asi struct
 *
 *  \return
 *
 */
VOID impeghd_mp4_get_maes(ia_bit_buf_struct *ptr_read_buff,
                          ia_mae_audio_scene_info *ptr_mae_audio_scene_info)
{
  WORD32 num_switch_groups, i, j;

  ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved bits
  ptr_mae_audio_scene_info->num_switch_groups =
      ia_core_coder_read_bits_buf(ptr_read_buff, 7); // no:of groups
  num_switch_groups = ptr_mae_audio_scene_info->num_switch_groups;
  for (i = 0; i < num_switch_groups; i++)
  {
    ptr_mae_audio_scene_info->switch_group_definition[i].allow_on_off = 0;
    ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->switch_group_definition[i].group_id =
        ia_core_coder_read_bits_buf(ptr_read_buff, 5); // reserved bits + switch group id

    ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->switch_group_definition[i].group_num_members =
        ia_core_coder_read_bits_buf(ptr_read_buff, 5);

    for (j = 0; j < ptr_mae_audio_scene_info->switch_group_definition[i].group_num_members; j++)
    {

      ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserve bit
      ptr_mae_audio_scene_info->switch_group_definition[i].member_id[j] =
          ia_core_coder_read_bits_buf(ptr_read_buff, 7); // switch group member id
    }

    ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved
    ptr_mae_audio_scene_info->switch_group_definition[i].default_group_id =
        ia_core_coder_read_bits_buf(ptr_read_buff, 7); // default id
  }
}
/**
 *  impeghd_mp4_get_maeg
 *
 *  \brief Brief description  Function to parse maeg box
 *
 *  \param ptr_read_buff              pointer to bit buffer
 *  \param ptr_mae_audio_scene_info   pointer to asi struct
 *
 *  \return
 *
 */
VOID impeghd_mp4_get_maeg(ia_bit_buf_struct *ptr_read_buff,
                          ia_mae_audio_scene_info *ptr_mae_audio_scene_info)
{
  WORD32 num_grps;
  ptr_mae_audio_scene_info->main_stream_flag = 1;
  ptr_mae_audio_scene_info->asi_id_present = 1;
  ptr_mae_audio_scene_info->asi_id =
      ia_core_coder_read_bits_buf(ptr_read_buff, 8); // writing asi_id
  ia_core_coder_read_bits_buf(ptr_read_buff, 1);     // reserved bits
  ptr_mae_audio_scene_info->num_groups =
      ia_core_coder_read_bits_buf(ptr_read_buff, 7); // no:of groups
  num_grps = ptr_mae_audio_scene_info->num_groups;
  for (WORD32 grp = 0; grp < num_grps; grp++)
  {
    ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved bits
    ptr_mae_audio_scene_info->group_definition[grp].group_id =
        (WORD8)ia_core_coder_read_bits_buf(ptr_read_buff, 7);
    ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved bits
    ptr_mae_audio_scene_info->group_definition[grp].allow_on_off =
        (WORD8)ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info->group_definition[grp].default_on_off =
        (WORD8)ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info->group_definition[grp].allow_pos_interact =
        (WORD8)ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info->group_definition[grp].allow_gain_interact =
        (WORD8)ia_core_coder_read_bits_buf(ptr_read_buff, 1);
    ptr_mae_audio_scene_info->group_definition[grp].has_content_language =
        ia_core_coder_read_bits_buf(ptr_read_buff, 1); // content language
    ia_core_coder_read_bits_buf(ptr_read_buff, 8);     // reserved + content kind
    if (ptr_mae_audio_scene_info->group_definition[grp].allow_pos_interact)
    {
      ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved
      ptr_mae_audio_scene_info->group_definition[grp].min_az_offset =
          ia_core_coder_read_bits_buf(ptr_read_buff, 7);
      ia_core_coder_read_bits_buf(ptr_read_buff, 1); // reserved
      ptr_mae_audio_scene_info->group_definition[grp].max_az_offset =
          ia_core_coder_read_bits_buf(ptr_read_buff, 7);
      ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved
      ptr_mae_audio_scene_info->group_definition[grp].min_el_offset =
          ia_core_coder_read_bits_buf(ptr_read_buff, 5);
      ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved
      ptr_mae_audio_scene_info->group_definition[grp].max_el_offset =
          ia_core_coder_read_bits_buf(ptr_read_buff, 5);
      ptr_mae_audio_scene_info->group_definition[grp].min_dist_factor =
          ia_core_coder_read_bits_buf(ptr_read_buff, 4);
      ptr_mae_audio_scene_info->group_definition[grp].max_dist_factor =
          ia_core_coder_read_bits_buf(ptr_read_buff, 4);
    }
    ptr_mae_audio_scene_info->group_definition[grp].group_num_members = 2;
    ptr_mae_audio_scene_info->group_definition[grp].has_conjunct_members = 1;

    if (ptr_mae_audio_scene_info->group_definition[grp].allow_gain_interact)
    {
      ia_core_coder_read_bits_buf(ptr_read_buff, 2); // reserved
      ptr_mae_audio_scene_info->group_definition[grp].min_gain =
          ia_core_coder_read_bits_buf(ptr_read_buff, 6);
      ia_core_coder_read_bits_buf(ptr_read_buff, 3); // reserved
      ptr_mae_audio_scene_info->group_definition[grp].max_gain =
          ia_core_coder_read_bits_buf(ptr_read_buff, 5);
    }
    if (ptr_mae_audio_scene_info->group_definition[grp].has_content_language)
    {
      ia_core_coder_read_bits_buf(ptr_read_buff, 8); // content language
    }
  }
  ptr_mae_audio_scene_info->group_definition[0].start_id = 0;
  ptr_mae_audio_scene_info->group_definition[1].start_id = 2;
}
