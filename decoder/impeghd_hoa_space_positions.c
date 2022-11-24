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

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_hoa_rom.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_amb_syn.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_tbe_dec.h"

#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_ren_space_positions_init_with_surround
 *
 *  \brief Initialize space position with surround parameter
 *
 *  \param [out]  spc_handle  HOA renderer space position handle
 *  \param [in]    num_pos    Number of position
 *  \param [in]    radius    Radius array
 *  \param [in]    spk_incl  Speaker inclination array
 *  \param [in]    spk_azimuth  Speaker azimuth array
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE
impeghd_hoa_ren_space_positions_init_with_surround(ia_render_hoa_space_positions_str *spc_handle,
                                                   WORD32 num_pos, pFLOAT32 radius,
                                                   pFLOAT32 spk_incl, pFLOAT32 spk_azimuth)
{
  WORD32 i;
  spc_handle->type = 1;
  spc_handle->cols = 3;
  spc_handle->num_pos = num_pos;
  ia_core_coder_memset(spc_handle->arr, HOA_MAX_ARR_SIZE);

  if (radius != NULL)
  {
    for (i = 0; i < spc_handle->num_pos; i++)
    {
      spc_handle->arr[i * spc_handle->cols + 2] = spk_azimuth[i];
      spc_handle->arr[i * spc_handle->cols + 1] = spk_incl[i];
      spc_handle->arr[i * spc_handle->cols] = radius[i];
    }
  }
  else
  {
    for (i = 0; i < spc_handle->num_pos; i++)
    {
      spc_handle->arr[i * spc_handle->cols + 2] = spk_azimuth[i];
      spc_handle->arr[i * spc_handle->cols + 1] = spk_incl[i];
      spc_handle->arr[i * spc_handle->cols] = 1;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_ren_space_positions_init_with_param
 *
 *  \brief Initialize space position with parameters
 *
 *  \param [in,out]  spc_handle  HOA renderer space position handle
 *  \param [in]    spk_idx    Speaker index
 *  \param [in]    force_lfe  LFE flag
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE
impeghd_hoa_ren_space_positions_init_with_param(ia_render_hoa_space_positions_str *spc_handle,
                                                const WORD32 spk_idx,
                                                ia_speaker_config_3d *ref_spk_layout,
                                                WORD32 force_lfe)
{
  FLOAT32 s_f = 0, s_t = 0;
  ia_core_coder_memset(spc_handle->arr, HOA_MAX_ARR_SIZE);

  WORD32 loop;
  const WORD32 *speaker_table;
  spc_handle->type = 1;
  spc_handle->num_pos = 0;
  spc_handle->cols = 3;

  if (ref_spk_layout->spk_layout_type == 0)
  {
    WORD32 loop = impgehd_cicp_get_num_ls[spk_idx];
    speaker_table = ia_cicp_idx_ls_set_map_tbl[spk_idx];
    for (WORD32 i = 0; i < loop; i++)
    {
      if (force_lfe && ia_cicp_ls_geo_tbls[speaker_table[i]].lfe_flag == 1)
      {
        s_t = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_elevation;
        s_f = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_azimuth;

        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 2] =
            ia_mul_flt((s_f), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 1] =
            ia_mul_flt(ia_sub_flt(90, (s_t)), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols] = 2.0f;
        spc_handle->num_pos++;
      }
      else if (ia_cicp_ls_geo_tbls[speaker_table[i]].lfe_flag == 0 && !force_lfe)
      {
        s_t = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_elevation;
        s_f = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_azimuth;
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 2] =
            ia_mul_flt((s_f), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 1] =
            ia_mul_flt(ia_sub_flt(90, (s_t)), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols] = 2.0f;
        spc_handle->num_pos++;
      }
    }
  }
  else if (ref_spk_layout->spk_layout_type == 1)
  {
    loop = ref_spk_layout->num_speakers;
    for (WORD32 i = 0; i < loop; i++)
    {
      if (!ref_spk_layout->cicp_spk_idx[i] || ref_spk_layout->cicp_spk_idx[i] > 20)
      {
        continue;
      }
      else if (ref_spk_layout->cicp_spk_idx[i] == 8)
      {
        return IA_MPEGH_DEC_CONFIG_NONFATAL_INVALID_CICP_INDEX;
      }
      speaker_table = ia_cicp_idx_ls_set_map_tbl[ref_spk_layout->cicp_spk_idx[i]];
      if (force_lfe && ia_cicp_ls_geo_tbls[speaker_table[i]].lfe_flag == 1)
      {
        s_t = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_elevation;
        s_f = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_azimuth;
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 2] =
            ia_mul_flt((s_f), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 1] =
            ia_mul_flt(ia_sub_flt(90, (s_t)), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols] = 2.0f;
        spc_handle->num_pos++;
      }
      else if (ia_cicp_ls_geo_tbls[speaker_table[i]].lfe_flag == 0 && !force_lfe)
      {
        s_t = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_elevation;
        s_f = (FLOAT32)ia_cicp_ls_geo_tbls[speaker_table[i]].ls_azimuth;
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 2] =
            ia_mul_flt((s_f), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 1] =
            ia_mul_flt(ia_sub_flt(90, (s_t)), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols] = 2.0f;
        spc_handle->num_pos++;
      }
    }
  }
  else if (ref_spk_layout->spk_layout_type == 2)
  {
    loop = ref_spk_layout->num_speakers;
    for (WORD32 i = 0; i < loop; i++)
    {
      if (force_lfe && ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].is_lfe == 1)
      {
        s_t = (FLOAT32)ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].el_angle_idx;
        s_f = (FLOAT32)ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].az_angle_idx;
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 2] =
            ia_mul_flt((s_f), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 1] =
            ia_mul_flt(ia_sub_flt(90, (s_t)), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols] = 2.0f;
        spc_handle->num_pos++;
      }
      else if (ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].is_lfe == 0 && !force_lfe)
      {
        s_t = (FLOAT32)ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].el_angle_idx;
        s_f = (FLOAT32)ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].az_angle_idx;
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 2] =
            ia_mul_flt((s_f), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols + 1] =
            ia_mul_flt(ia_sub_flt(90, (s_t)), ((FLOAT32)PI) / 180.0f);
        spc_handle->arr[spc_handle->num_pos * spc_handle->cols] = 2.0f;
        spc_handle->num_pos++;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_space_positions_init_pos
 *
 *  \brief Initialize space position
 *
 *  \param [out]  spc_handle    Handle to be initialized
 *  \param [in]    handle_original  Original handle
 *
 *
 *
 */
VOID impeghd_hoa_ren_space_positions_init_pos(ia_render_hoa_space_positions_str *spc_handle,
                                              ia_render_hoa_space_positions_str *handle_original)
{
  spc_handle->type = handle_original->type;
  spc_handle->cols = 3;
  spc_handle->num_pos = handle_original->num_pos;
  ia_core_coder_memset(spc_handle->arr, HOA_MAX_ARR_SIZE);
  ia_core_coder_mem_cpy(handle_original->arr, spc_handle->arr, (3 * spc_handle->num_pos));
}
/**
 *  impeghd_hoa_ren_space_positions_add_poles
 *
 *  \brief Add poles to space position handle
 *
 *  \param [out]  handle  HOA renderer space position handle
 *  \param [in]    r    2D handling flag
 *  \param [in]    scratch  Pointer to scratch buffer for intermediate processing
 *
 *
 *
 */
VOID impeghd_hoa_ren_space_positions_add_poles(pVOID handle, FLOAT32 r, pVOID scratch)
{
  ia_render_hoa_space_positions_str *spc_handle = (ia_render_hoa_space_positions_str *)handle;
  pFLOAT32 pt_orig = (pFLOAT32)scratch;
  pFLOAT32 pt = pt_orig;
  ia_core_coder_mem_cpy(spc_handle->arr, pt, 3 * spc_handle->num_pos);
  ia_core_coder_memset(&pt[3 * spc_handle->num_pos], 6);

  pt += (3 * spc_handle->num_pos);
  if (spc_handle->type != 1)
  {
    *pt++ = 0;
    *pt++ = 0;
    *pt++ = r;
    *pt++ = 0;
    *pt++ = 0;
    *pt = -r;
  }
  else
  {
    *pt++ = r;
    *pt++ = 0;
    *pt++ = 0;
    *pt++ = r;
    *pt++ = (FLOAT32)PI;
    *pt = 0;
  }

  pt -= 3 * (spc_handle->num_pos + 2) - 1;
  spc_handle->num_pos += 2;
  ia_core_coder_mem_cpy(pt, spc_handle->arr, 3 * spc_handle->num_pos);
}
/**
 *  impeghd_hoa_ren_space_positions_is_2d
 *
 *  \brief Checks if matrix is associted with circular(i.e. 2D) HOA or not
 *
 *  \param [in]  handle  HOA Renderer space position handle
 *  \param [in]  scratch  Pointer to scratch buffer for intermediate processing
 *
 *  \return WORD32  Circular(i.e. 2D) Flag
 *
 */
WORD32 impeghd_hoa_ren_space_positions_is_2d(pVOID handle, pVOID scratch)
{
  ia_render_hoa_space_positions_str *spc_handle = (ia_render_hoa_space_positions_str *)handle;
  WORD32 is_2d_flag = 1;
  pWORD8 buf = (pWORD8)scratch;

  FLOAT32 ut = 1.69296944141387939F; // PI / 2 + HOA_SPACE_POSITION_2D_THRESHOLD;
  FLOAT32 lt = 1.44862329959869385F; // PI / 2 - HOA_SPACE_POSITION_2D_THRESHOLD;

  WORD32 i;

  if (spc_handle->type != 1)
  {
    ia_render_hoa_space_positions_str *tmp_handle = (ia_render_hoa_space_positions_str *)(buf);
    impeghd_hoa_ren_space_positions_convert_copy_to_spherical(spc_handle, tmp_handle);
    for (i = 0; i < spc_handle->num_pos; i++)
    {
      if ((tmp_handle->arr[i * tmp_handle->cols + 1] < lt) ||
          (tmp_handle->arr[i * tmp_handle->cols + 1] > ut))
      {
        is_2d_flag = 0;
        break;
      }
    }
  }
  else
  {
    for (i = 0; i < spc_handle->num_pos; i++)
    {
      if ((spc_handle->arr[i * spc_handle->cols + 1] < lt) ||
          (spc_handle->arr[i * spc_handle->cols + 1] > ut))
      {
        is_2d_flag = 0;
        break;
      }
    }
  }
  return is_2d_flag;
}

/**
 *  impeghd_hoa_ren_space_positions_convert_copy_to_cartesian
 *
 *  \brief Convert and copy space positions to cartesian plane
 *
 *  \param [in]    handle    Input handle
 *  \param [out]  tmp_handle  Output handle
 *  \param [in]    force_r1  Plane wave model Flag
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_space_positions_convert_copy_to_cartesian(
    pVOID handle, ia_render_hoa_space_positions_str *tmp_handle, WORD32 force_r1)
{
  ia_render_hoa_space_positions_str *spc_handle = (ia_render_hoa_space_positions_str *)handle;
  switch (spc_handle->type)
  {
  case 0:
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_TYPE;
  }
  break;
  case 1:
  {
    WORD32 i;
    memset(tmp_handle, 0, sizeof(ia_render_hoa_space_positions_str));
    tmp_handle->num_pos = spc_handle->num_pos;
    tmp_handle->cols = spc_handle->cols;
    for (i = 0; i < tmp_handle->num_pos; i++)
    {
      if (!force_r1)
      {
        tmp_handle->arr[(i * spc_handle->cols) + 2] =
            (FLOAT32)(ia_mul_double_flt(cos(spc_handle->arr[i * 3 + 1]), spc_handle->arr[i * 3]));
        tmp_handle->arr[(i * spc_handle->cols) + 1] = (FLOAT32)(
            ia_mul_double_flt(sin(spc_handle->arr[i * 3 + 1]) * sin(spc_handle->arr[i * 3 + 2]),
                              spc_handle->arr[i * 3]));
        tmp_handle->arr[i * spc_handle->cols] = (FLOAT32)(
            ia_mul_double_flt(sin(spc_handle->arr[i * 3 + 1]) * cos(spc_handle->arr[i * 3 + 2]),
                              spc_handle->arr[i * 3]));
      }
      else
      {
        tmp_handle->arr[(i * spc_handle->cols) + 2] = (FLOAT32)cos(spc_handle->arr[i * 3 + 1]);
        tmp_handle->arr[(i * spc_handle->cols) + 1] =
            (FLOAT32)(sin(spc_handle->arr[i * 3 + 1]) * sin(spc_handle->arr[i * 3 + 2]));
        tmp_handle->arr[i * spc_handle->cols] =
            (FLOAT32)(sin(spc_handle->arr[i * 3 + 1]) * cos(spc_handle->arr[i * 3 + 2]));
      }
    }
    tmp_handle->type = 2;
  }
  break;
  case 2:
  {
    impeghd_hoa_ren_space_positions_init_pos(tmp_handle, spc_handle);
  }
  break;
  }
  tmp_handle->num_pos = spc_handle->num_pos;
  tmp_handle->cols = spc_handle->cols;
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_ren_space_positions_convert_copy_to_spherical
 *
 *  \brief Convert and copy space position to spherical
 *
 *  \param [in]    handle    Input handle
 *  \param [out]  tmp_handle  Output handle
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_space_positions_convert_copy_to_spherical(
    pVOID handle, ia_render_hoa_space_positions_str *tmp_handle)
{
  ia_render_hoa_space_positions_str *spc_handle = (ia_render_hoa_space_positions_str *)handle;
  if (spc_handle->type == 0)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_TYPE;
  }
  else if (spc_handle->type == 2)
  {
    WORD32 i;
    memset(tmp_handle, 0, sizeof(ia_render_hoa_space_positions_str));
    tmp_handle->cols = spc_handle->cols;
    tmp_handle->num_pos = spc_handle->num_pos;
    for (i = 0; i < tmp_handle->num_pos; i++)
    {
      tmp_handle->arr[(i * tmp_handle->cols) + 2] =
          (FLOAT32)atan2(spc_handle->arr[i * 3 + 1], spc_handle->arr[i * 3]);
      tmp_handle->arr[(i * tmp_handle->cols) + 1] =
          (FLOAT32)atan2(ia_sqrt_flt(spc_handle->arr[i * 3] * spc_handle->arr[i * 3] +
                                     spc_handle->arr[i * 3 + 1] * spc_handle->arr[i * 3 + 1]),
                         spc_handle->arr[i * 3 + 2]);
      tmp_handle->arr[i * tmp_handle->cols] =
          (FLOAT32)ia_sqrt_flt(spc_handle->arr[i * 3 + 2] * spc_handle->arr[i * 3 + 2] +
                               spc_handle->arr[i * 3 + 1] * spc_handle->arr[i * 3 + 1] +
                               spc_handle->arr[i * 3] * spc_handle->arr[i * 3]);
    }
    tmp_handle->type = 1;
  }
  else if (spc_handle->type == 1)
  {
    impeghd_hoa_ren_space_positions_init_pos(tmp_handle, spc_handle);
  }
  else
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_TYPE;
  tmp_handle->cols = spc_handle->cols;
  tmp_handle->num_pos = spc_handle->num_pos;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_space_positions_get_max_min_inclination
 *
 *  \brief Sets maximum and minimum inclination of space position
 *
 *  \param [in]    handle  HOA Renderer space position handle
 *  \param [out]  max_inc  Maximum inclination
 *  \param [out]  min_inc  Minimum inclination
 *  \param [in]    scratch Pointer to scratch buffer for intermediate processing
 *
 *
 *
 */
VOID impeghd_hoa_ren_space_positions_get_max_min_inclination(pVOID handle, pFLOAT32 max_inc,
                                                             pFLOAT32 min_inc, pVOID scratch)
{
  ia_render_hoa_space_positions_str *spc_handle = (ia_render_hoa_space_positions_str *)handle;
  FLOAT32 min_i = (FLOAT32)PI;
  FLOAT32 max_i = 0;
  pWORD8 buf = (pWORD8)scratch;

  if (spc_handle->type != 1)
  {
    ia_render_hoa_space_positions_str *tmp_handle = (ia_render_hoa_space_positions_str *)(buf);
    impeghd_hoa_ren_space_positions_convert_copy_to_spherical(spc_handle, tmp_handle);
    for (WORD32 i = 0; i < tmp_handle->num_pos; i++)
    {
      min_i = ia_min_flt(tmp_handle->arr[i * tmp_handle->cols + 1], min_i);
      max_i = ia_max_flt(tmp_handle->arr[i * tmp_handle->cols + 1], max_i);
    }
  }
  else
  {
    for (WORD32 i = 0; i < spc_handle->num_pos; i++)
    {
      min_i = ia_min_flt(spc_handle->arr[i * spc_handle->cols + 1], min_i);
      max_i = ia_max_flt(spc_handle->arr[i * spc_handle->cols + 1], max_i);
    }
  }
  *max_inc = max_i;
  *min_inc = min_i;
}
/**
 *  ia_render_hoa_space_positions_get_max_distance
 *
 *  \brief Gives the maximum distance of space position
 *
 *  \param [in]  handle  HOA Space position handle
 *
 *  \return WORD32  Maximum distance of space position
 *
 */
FLOAT32 impeghd_hoa_ren_space_positions_get_max_distance(pVOID handle)
{
  ia_render_hoa_space_positions_str *spc_handle = (ia_render_hoa_space_positions_str *)handle;
  FLOAT32 max_r = 0;

  if (spc_handle->type != 1)
  {
    for (WORD32 i = 0; i < spc_handle->num_pos; i++)
    {
      FLOAT32 r = (FLOAT32)ia_sqrt_flt(
          spc_handle->arr[i * spc_handle->cols + 2] * spc_handle->arr[i * spc_handle->cols + 2] +
          spc_handle->arr[i * spc_handle->cols + 1] * spc_handle->arr[i * spc_handle->cols + 1] +
          spc_handle->arr[i * spc_handle->cols] * spc_handle->arr[i * spc_handle->cols]);
      max_r = ia_max_flt(r, max_r);
    }
  }
  else
  {
    for (WORD32 i = 0; i < spc_handle->num_pos; i++)
    {
      max_r = ia_max_flt(spc_handle->arr[i * spc_handle->cols], max_r);
    }
  }
  return max_r;
}
/** @} */ /* End of HOAProc */