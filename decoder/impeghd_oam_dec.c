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
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_oam_dec.h"

/**
 * @defgroup OAMDec Object Audio Metadata Decoder
 * @ingroup  OAMDec
 * @brief Object Audio Metadata Decoder
 *
 * @{
 */

/**
 *  impeghd_obj_md_dec
 *
 *  \brief oam object decoder
 *
 *  \param [in/out]  ptr_oam_dec_state  Object metadata decoder handle
 *  \param [in]    ptr_bit_buf      Bit buffer handle
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE impeghd_obj_md_dec(ia_oam_dec_state_struct *ptr_oam_dec_state,
                                ia_bit_buf_struct *ptr_bit_buf)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  if (ptr_oam_dec_state->p_obj_md_cfg->ld_md_coding_present != 0)
  {
    err_code = impeghd_obj_md_low_delay_dec(ptr_oam_dec_state, ptr_bit_buf);
  }
  else
  {
    return IA_MPEGH_OAM_EXE_FATAL_UNSUPPORTED_OBJ_MD_DECODING;
  }
  return err_code;
}

/**
 *  impeghd_obj_md_cfg
 *
 *  \brief Object oam configuration
 *
 *  \param [in/out]  p_obj_md_cfg  Object metadata config structure
 *  \param [in]    ptr_bit_buf    Bit buffer handle
 *  \param [in]    cc_frame_len  Frame length
 *  \param [in]    num_objects    Number of audio object
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE impeghd_obj_md_cfg(ia_oam_dec_config_struct *p_obj_md_cfg,
                                ia_bit_buf_struct *ptr_bit_buf, WORD32 cc_frame_len,
                                WORD32 num_objects)
{
  WORD32 i;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 core_len_present = 0;
  WORD32 screen_rel_objs_present = 0;
  p_obj_md_cfg->ld_md_coding_present = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
  core_len_present = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
  p_obj_md_cfg->cc_frame_length = cc_frame_len;
  if (core_len_present)
  {
    p_obj_md_cfg->frame_length = cc_frame_len;
  }
  else
  {
    WORD32 frame_length = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FRAME_LEN_BITS);
    frame_length = (frame_length + 1) << 6;
    if ((frame_length <= cc_frame_len) &&
        ((frame_length == 256) || (frame_length == 512) || (frame_length == 1024)))
    {
      p_obj_md_cfg->frame_length = frame_length;
    }
    else
    {
      return IA_MPEGH_OAM_EXE_FATAL_UNSUPPORTED_FRAMELENGTH;
    }
  }
  screen_rel_objs_present = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
  if (screen_rel_objs_present)
  {
    p_obj_md_cfg->screen_rel_objs_present = 0;
    screen_rel_objs_present = 0;
    FLAG *ptr_screen_rel_obj = &p_obj_md_cfg->is_screen_rel_obj[0];
    for (i = num_objects; i > 0; i--)
    {
      *ptr_screen_rel_obj = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
      screen_rel_objs_present |= *ptr_screen_rel_obj;
      ptr_screen_rel_obj++;
    }
    p_obj_md_cfg->screen_rel_objs_present = screen_rel_objs_present;
  }
  p_obj_md_cfg->dyn_obj_priority_present =
      ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
  p_obj_md_cfg->uniform_spread_present = ia_core_coder_read_bits_buf(ptr_bit_buf, OAM_FLAG_BITS);
  return err_code;
}

/**
 *  impeghd_enh_obj_md_config
 *
 *  \brief Enhanced Object metadata configuration data reading function.
 *
 *  \param [i/o] p_enh_obj_md_cfg Pointer to enhanced object metadata config structure.
 *  \param [in]  ptr_bit_buf      Pointer to bitstream buffer structure.
 *  \param [in]  num_objects      Number of objects.
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE impeghd_enh_obj_md_config(ia_enh_oam_config_struct *p_enh_obj_md_cfg,
                                       ia_bit_buf_struct *ptr_bit_buf, WORD32 num_objects)
{
  WORD32 i;
  WORD32 num_obj_with_divergence = 0;

  p_enh_obj_md_cfg->has_diffuseness = (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (p_enh_obj_md_cfg->has_diffuseness)
  {
    p_enh_obj_md_cfg->has_common_group_diffuseness =
        (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }
  p_enh_obj_md_cfg->has_excluded_sectors = (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (!p_enh_obj_md_cfg->has_excluded_sectors)
  {
    p_enh_obj_md_cfg->has_common_group_excluded_sectors = 0;
  }
  else
  {
    p_enh_obj_md_cfg->has_common_group_excluded_sectors =
        (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (p_enh_obj_md_cfg->has_common_group_excluded_sectors)
    {
      memset(p_enh_obj_md_cfg->use_only_predefined_sectors,
             (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1),
             sizeof(p_enh_obj_md_cfg->use_only_predefined_sectors));
    }
  }
  p_enh_obj_md_cfg->has_closest_speaker_condition =
      (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (p_enh_obj_md_cfg->has_closest_speaker_condition)
  {
    p_enh_obj_md_cfg->closest_spk_thr_angle = (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
  }

  for (i = 0; i < num_objects; i++)
  {
    p_enh_obj_md_cfg->has_divergence[i] = (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (p_enh_obj_md_cfg->has_divergence[i])
    {
      num_obj_with_divergence++;
      p_enh_obj_md_cfg->divergence_az_range[i] =
          (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 6);
    }
    if (p_enh_obj_md_cfg->has_common_group_excluded_sectors == 0)
    {
      p_enh_obj_md_cfg->use_only_predefined_sectors[i] =
          (WORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    }
  }
  p_enh_obj_md_cfg->num_obj_with_divergence = num_obj_with_divergence;
  if (((num_obj_with_divergence << 1) + num_objects) > MAX_NUM_OAM_OBJS)
  {
    return -1;
  }
  return 0;
}

/**
 *  impeghd_enh_obj_md_frame
 *
 *  \brief Enhanced Object metadata frame data reading function.
 *
 *  \param [i/o] p_enh_oj_md_frame Pointer to enhanced object metadata frame structure
 *  \param [in]  ptr_bit_buf       Pointer to bitstream buffer structure
 *  \param [in]  num_objects       Number of objects
 *  \param [in]  independency_flag MPEG-H 3D Audio Low Complexity Profile decoder independency
 * flag
 *
 *  \return IA_ERRORCODE Processing error if any.
 *
 */
IA_ERRORCODE impeghd_enh_obj_md_frame(ia_enh_obj_md_frame_str *p_enh_oj_md_frame,
                                      ia_bit_buf_struct *ptr_bit_buf, WORD32 num_objects,
                                      WORD32 independency_flag)
{
  IA_ERRORCODE error = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, j, obj;
  ia_enh_oam_config_struct *p_enh_obj_md_cfg = p_enh_oj_md_frame->p_enh_obj_md_cfg;

  {
    /* push current frame to the old frame structure */
    for (obj = 0; obj < num_objects; obj++)
    {

      for (j = 0; j < MAX_NUM_EXCLUDED_SECTORS; j++)
      {
        p_enh_oj_md_frame->prev_exclude_sector_max_ele[obj][j] =
            p_enh_oj_md_frame->exclude_sector_max_ele[obj][j];
        p_enh_oj_md_frame->prev_exclude_sector_min_ele[obj][j] =
            p_enh_oj_md_frame->exclude_sector_min_ele[obj][j];
        p_enh_oj_md_frame->prev_exclude_sector_max_az[obj][j] =
            p_enh_oj_md_frame->exclude_sector_max_az[obj][j];
        p_enh_oj_md_frame->prev_exclude_sector_min_az[obj][j] =
            p_enh_oj_md_frame->exclude_sector_min_az[obj][j];
        p_enh_oj_md_frame->prev_exclude_sector_index[obj][j] =
            p_enh_oj_md_frame->exclude_sector_index[obj][j];
      }
      p_enh_oj_md_frame->prev_num_exclusion_sectors[obj] =
          p_enh_oj_md_frame->num_exclusion_sectors[obj];
      p_enh_oj_md_frame->prev_divergence[obj] = p_enh_oj_md_frame->divergence[obj];
      p_enh_oj_md_frame->prev_diffuseness[obj] = p_enh_oj_md_frame->diffuseness[obj];
    }

    /* read new frame */

    if (p_enh_obj_md_cfg->has_common_group_excluded_sectors)
    {
      if (independency_flag != 0)
      {
        p_enh_oj_md_frame->keep_exclusion[0] = 0;
      }
      else
      {
        p_enh_oj_md_frame->keep_exclusion[0] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      }
      if (p_enh_oj_md_frame->keep_exclusion[0] != 0)
      {
        p_enh_oj_md_frame->num_exclusion_sectors[0] =
            p_enh_oj_md_frame->prev_num_exclusion_sectors[0];
        for (i = 0; i < p_enh_oj_md_frame->num_exclusion_sectors[0]; i++)
        {
          p_enh_oj_md_frame->exclude_sector_index[0][i] =
              p_enh_oj_md_frame->prev_exclude_sector_index[0][i];
          p_enh_oj_md_frame->use_predefined_sector[0][i] =
              p_enh_oj_md_frame->prev_use_predefined_sector[0][i];
          p_enh_oj_md_frame->exclude_sector_index[0][i] =
              p_enh_oj_md_frame->prev_exclude_sector_index[0][i];
          p_enh_oj_md_frame->exclude_sector_min_az[0][i] =
              p_enh_oj_md_frame->prev_exclude_sector_min_az[0][i];
          p_enh_oj_md_frame->exclude_sector_max_az[0][i] =
              p_enh_oj_md_frame->prev_exclude_sector_max_az[0][i];
          p_enh_oj_md_frame->exclude_sector_min_ele[0][i] =
              p_enh_oj_md_frame->prev_exclude_sector_min_ele[0][i];
          p_enh_oj_md_frame->exclude_sector_max_ele[0][i] =
              p_enh_oj_md_frame->prev_exclude_sector_max_ele[0][i];
        }
      }
      else
      {
        p_enh_oj_md_frame->num_exclusion_sectors[0] = ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
        if (p_enh_obj_md_cfg->use_only_predefined_sectors[0] == 1)
        {
          for (i = 0; i < p_enh_oj_md_frame->num_exclusion_sectors[0]; i++)
          {
            p_enh_oj_md_frame->exclude_sector_index[0][i] =
                ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
          }
        }
        else
        {
          for (i = 0; i < p_enh_oj_md_frame->num_exclusion_sectors[0]; i++)
          {
            p_enh_oj_md_frame->use_predefined_sector[0][i] =
                ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
            if (p_enh_oj_md_frame->use_predefined_sector[0][i] != 1)
            {
              p_enh_oj_md_frame->exclude_sector_min_az[0][i] = ia_min_flt(
                  ia_max_flt(
                      (FLOAT32)ia_mul_flt(
                          3.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 7),
                                           63.0f)),
                      -180.0f),
                  180.0f);
              p_enh_oj_md_frame->exclude_sector_max_az[0][i] = ia_min_flt(
                  ia_max_flt(
                      (FLOAT32)ia_mul_flt(
                          3.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 7),
                                           63.0f)),
                      -180.0f),
                  180.0f);
              ;
              p_enh_oj_md_frame->exclude_sector_min_ele[0][i] = ia_min_flt(
                  ia_max_flt(
                      (FLOAT32)ia_mul_flt(
                          6.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 5),
                                           15.0f)),
                      -90.0f),
                  90.0f);
              p_enh_oj_md_frame->exclude_sector_max_ele[0][i] = ia_min_flt(
                  ia_max_flt(
                      (FLOAT32)ia_mul_flt(
                          6.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 5),
                                           15.0f)),
                      -90.0f),
                  90.0f);
            }
            else
            {
              p_enh_oj_md_frame->exclude_sector_index[0][i] =
                  ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
            }
          }
        }
      }
    }

    if ((p_enh_obj_md_cfg->has_diffuseness) && (p_enh_obj_md_cfg->has_common_group_diffuseness))
    {
      if (independency_flag != 0)
      {
        p_enh_oj_md_frame->keep_diffuseness[0] = 0;
      }
      else
      {
        p_enh_oj_md_frame->keep_diffuseness[0] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      }
      if (p_enh_oj_md_frame->keep_diffuseness[0] != 0)
      {
        p_enh_oj_md_frame->diffuseness[0] = p_enh_oj_md_frame->prev_diffuseness[0];
      }
      else
      {
        p_enh_oj_md_frame->diffuseness[0] =
            (ia_core_coder_read_bits_buf(ptr_bit_buf, 7)) / 127.0f;
      }
    }
    for (obj = 0; obj < num_objects; obj++)
    {
      p_enh_oj_md_frame->closest_spkr_playout[obj] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (p_enh_obj_md_cfg->has_divergence[obj] == 1)
      {
        if (independency_flag != 0)
        {
          p_enh_oj_md_frame->keep_divergence[obj] = 0;
        }
        else
        {
          p_enh_oj_md_frame->keep_divergence[obj] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }
        if (p_enh_oj_md_frame->keep_divergence[obj] != 0)
        {
          p_enh_oj_md_frame->divergence[obj] = p_enh_oj_md_frame->prev_divergence[obj];
        }
        else
        {
          p_enh_oj_md_frame->divergence[obj] =
              (ia_core_coder_read_bits_buf(ptr_bit_buf, 7)) / 127.0f;
        }
      }
      if ((p_enh_obj_md_cfg->has_diffuseness) &&
          (p_enh_obj_md_cfg->has_common_group_diffuseness == 0))
      {
        if (independency_flag != 0)
        {
          p_enh_oj_md_frame->keep_diffuseness[obj] = 0;
        }
        else
        {
          p_enh_oj_md_frame->keep_diffuseness[obj] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }
        if (p_enh_oj_md_frame->keep_diffuseness[obj] != 0)
        {
          p_enh_oj_md_frame->diffuseness[obj] = p_enh_oj_md_frame->prev_diffuseness[obj];
        }
        else
        {
          p_enh_oj_md_frame->diffuseness[obj] =
              (ia_core_coder_read_bits_buf(ptr_bit_buf, 7)) / 127.0f;
        }
      }

      if (p_enh_obj_md_cfg->has_common_group_excluded_sectors == 0)
      {
        if (independency_flag != 0)
        {
          p_enh_oj_md_frame->keep_exclusion[obj] = 0;
        }
        else
        {
          p_enh_oj_md_frame->keep_exclusion[obj] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }
        if (p_enh_oj_md_frame->keep_exclusion[obj] != 0)
        {
          p_enh_oj_md_frame->num_exclusion_sectors[obj] =
              p_enh_oj_md_frame->prev_num_exclusion_sectors[obj];
          for (j = 0; j < p_enh_oj_md_frame->num_exclusion_sectors[obj]; j++)
          {
            p_enh_oj_md_frame->exclude_sector_index[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_index[obj][j];
            p_enh_oj_md_frame->use_predefined_sector[obj][j] =
                p_enh_oj_md_frame->prev_use_predefined_sector[obj][j];
            p_enh_oj_md_frame->exclude_sector_index[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_index[obj][j];
            p_enh_oj_md_frame->use_predefined_sector[obj][j] =
                p_enh_oj_md_frame->prev_use_predefined_sector[obj][j];
            p_enh_oj_md_frame->exclude_sector_index[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_index[obj][j];
            p_enh_oj_md_frame->exclude_sector_min_az[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_min_az[obj][j];
            p_enh_oj_md_frame->exclude_sector_max_az[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_max_az[obj][j];
            p_enh_oj_md_frame->exclude_sector_min_ele[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_min_ele[obj][j];
            p_enh_oj_md_frame->exclude_sector_max_ele[obj][j] =
                p_enh_oj_md_frame->prev_exclude_sector_max_ele[obj][j];
          }
        }
        else
        {
          p_enh_oj_md_frame->num_exclusion_sectors[obj] =
              ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
          if (p_enh_obj_md_cfg->use_only_predefined_sectors[obj] == 1)
          {
            for (j = 0; j < p_enh_oj_md_frame->num_exclusion_sectors[obj]; j++)
            {
              p_enh_oj_md_frame->exclude_sector_index[obj][j] =
                  ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
            }
          }
          else
          {
            for (j = 0; j < p_enh_oj_md_frame->num_exclusion_sectors[obj]; j++)
            {
              p_enh_oj_md_frame->use_predefined_sector[obj][j] =
                  ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
              if (p_enh_oj_md_frame->use_predefined_sector[obj][j] != 1)
              {
                p_enh_oj_md_frame->exclude_sector_min_az[obj][j] = ia_min_flt(
                    ia_max_flt(
                        (FLOAT32)ia_mul_flt(
                            3.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 7),
                                             63.0f)),
                        -180.0f),
                    180.0f);
                p_enh_oj_md_frame->exclude_sector_max_az[obj][j] = ia_min_flt(
                    ia_max_flt(
                        (FLOAT32)ia_mul_flt(
                            3.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 7),
                                             63.0f)),
                        -180.0f),
                    180.0f);
                p_enh_oj_md_frame->exclude_sector_min_ele[obj][j] = ia_min_flt(
                    ia_max_flt(
                        (FLOAT32)ia_mul_flt(
                            6.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 5),
                                             15.0f)),
                        -90.0f),
                    90.0f);
                p_enh_oj_md_frame->exclude_sector_max_ele[obj][j] = ia_min_flt(
                    ia_max_flt(
                        (FLOAT32)ia_mul_flt(
                            6.0f, ia_sub_flt((FLOAT32)ia_core_coder_read_bits_buf(ptr_bit_buf, 5),
                                             15.0f)),
                        -90.0f),
                    90.0f);
              }
              else
              {

                p_enh_oj_md_frame->exclude_sector_index[obj][j] =
                    ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
              }
            }
          }
        }
      }
    }
  }
  return error;
}
/** @} */ /* End of OAMDec */