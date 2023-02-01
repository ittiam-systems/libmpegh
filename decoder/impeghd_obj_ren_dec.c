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
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"

/*3D vectors*/
#include "impeghd_3d_vec_struct_def.h"

/* CICP Module */
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_config.h"
#include "impeghd_mhas_parse.h"

/* Decoder interfaces */
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"

/* Object Metadata Module */
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"

/* Object Renderer Module */
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_obj_ren_dec.h"

#include "impeghd_3d_vec_basic_ops.h"

#define LS_SUBSET_A_LEN (4)
#define LS_SUBSET_B_LEN (4)
#define LS_SUBSET_C_LEN (4)
#define LS_SUBSET_D_LEN (4)

const WORD32 ia_loud_speaker_subset_A[LS_SUBSET_A_LEN] = {CH_M_L030, CH_M_R030, CH_U_R030,
                                                          CH_U_L030};
const WORD32 ia_loud_speaker_subset_B[LS_SUBSET_B_LEN] = {CH_M_L045, CH_M_R045, CH_U_R045,
                                                          CH_U_L045};
const WORD32 ia_loud_speaker_subset_C[LS_SUBSET_C_LEN] = {CH_M_L110, CH_U_L110, CH_U_R110,
                                                          CH_M_R110};

const WORD32 ia_loud_speaker_subset_D[LS_SUBSET_D_LEN] = {CH_M_L135, CH_U_R135, CH_U_L135,
                                                          CH_M_R135};

/**
 * @defgroup OAMProc Object Audio Renderer Processing
 * @ingroup  OAMProc
 * @brief Object Audio Renderer Processing
 *
 * @{
 */

/**
 *  impeghd_flex_spk_2_ls_geometry
 *
 *  \brief Copying flexible speaker parameter to corresponding structure
 *
 *  \param [in,out]  array_cicp_ls_geometry Pointer to ia_cicp_ls_geo_str structure
 *  \param [in]      ptr_ref_spk_layout     Pointer to ia_speaker_config_3d structure
 *  \param [out]     pp_cicp_ls_geometry    Double pointer to ia_cicp_ls_geo_str structure
 *  \param [in,out]  num_channels           Number of channels
 *  \param [out]     num_lfe                Number of LFE
 *  \param [out]     channel_names          Channel names
 *
 *  \return IA_ERRORCODE error code if any
 */
static IA_ERRORCODE impeghd_flex_spk_2_ls_geometry(ia_cicp_ls_geo_str *array_cicp_ls_geometry,
                                                   ia_speaker_config_3d *ptr_ref_spk_layout,
                                                   ia_cicp_ls_geo_str **pp_cicp_ls_geometry,
                                                   WORD32 *num_channels, WORD32 *num_lfe,
                                                   const WORD32 **channel_names)
{
  WORD32 i;
  WORD32 num_speakers;
  ia_flex_spk_data_str *ptr_flex_spk = &ptr_ref_spk_layout->str_flex_spk;
  num_speakers = ptr_ref_spk_layout->num_speakers;

  if (ptr_ref_spk_layout->spk_layout_type == 2)
  {
    for (i = 0; i < num_speakers; i++)
    {
      pp_cicp_ls_geometry[i] = &array_cicp_ls_geometry[i];
      pp_cicp_ls_geometry[i]->ls_elevation = ptr_flex_spk->elevation[i];
      pp_cicp_ls_geometry[i]->ls_elevation_start = ptr_flex_spk->elevation[i];
      pp_cicp_ls_geometry[i]->ls_elevation_end = ptr_flex_spk->elevation[i];
      pp_cicp_ls_geometry[i]->ls_azimuth = ptr_flex_spk->azimuth[i];
      pp_cicp_ls_geometry[i]->ls_azimuth_start = ptr_flex_spk->azimuth[i];
      pp_cicp_ls_geometry[i]->ls_azimuth_end = ptr_flex_spk->azimuth[i];
      pp_cicp_ls_geometry[i]->lfe_flag = ptr_flex_spk->str_flex_spk_descr[i].is_lfe;
      *num_lfe += ptr_flex_spk->str_flex_spk_descr[i].is_lfe;
    }
  }

  if (ptr_ref_spk_layout->spk_layout_type == 1)
  {
    for (i = 0; i < num_speakers; i++)
    {
      WORD32 idx = ptr_ref_spk_layout->cicp_spk_idx[i];
      pp_cicp_ls_geometry[i] = &array_cicp_ls_geometry[idx];
      pp_cicp_ls_geometry[i]->ls_elevation = ia_cicp_ls_geo_tbls[idx].ls_elevation;
      pp_cicp_ls_geometry[i]->ls_elevation_start = ia_cicp_ls_geo_tbls[idx].ls_elevation_start;
      pp_cicp_ls_geometry[i]->ls_elevation_end = ia_cicp_ls_geo_tbls[idx].ls_elevation_end;
      pp_cicp_ls_geometry[i]->ls_azimuth = ia_cicp_ls_geo_tbls[idx].ls_azimuth;
      pp_cicp_ls_geometry[i]->ls_azimuth_start = ia_cicp_ls_geo_tbls[idx].ls_azimuth_start;
      pp_cicp_ls_geometry[i]->ls_azimuth_end = ia_cicp_ls_geo_tbls[idx].ls_azimuth_end;
      pp_cicp_ls_geometry[i]->lfe_flag = ia_cicp_ls_geo_tbls[idx].lfe_flag;
      *num_lfe += ptr_flex_spk->str_flex_spk_descr[idx].is_lfe;
    }
  }
  return 0;
}

/**
 *  impegh_quick_sort_idx
 *
 *  \brief Simple quick sort implementation.
 *
 *  \param [in,out] io_buf      Buffer containing elements to be sorted.
 *  \param [in]  scratch_buf Pointer to scratch buffer for processing.
 *  \param [in]  num_ele     Number of elements to be sorted
 *
 *
 *
 */
VOID impegh_quick_sort_idx(ia_ls_ord_idx_params *io_buf, ia_ls_ord_idx_params *scratch_buf,
                           WORD32 num_ele)
{
  WORD32 i, pivot_idx = 0;
  WORD32 spk_order = io_buf[num_ele - 1].ls_order;
  WORD32 pivot_element = io_buf[num_ele - 1].ls_index;
  ia_ls_ord_idx_params *ptr_start = &scratch_buf[0];
  ia_ls_ord_idx_params *ptr_end = &scratch_buf[num_ele - 1];
  for (i = 0; i < (num_ele - 1); i++)
  {
    if (io_buf[i].ls_index < pivot_element)
    {
      memcpy(ptr_start, io_buf + i, sizeof(*ptr_end));
      ptr_start++;
      pivot_idx++;
    }
    else
    {
      memcpy(ptr_end, io_buf + i, sizeof(*ptr_end));
      ptr_end--;
    }
  }
  if (ptr_start == ptr_end)
  {
    ptr_start->ls_order = spk_order;
    ptr_start->ls_index = pivot_element;
  }
  memcpy(io_buf, scratch_buf, num_ele * sizeof(*io_buf));
  if (pivot_idx > 1)
  {
    impegh_quick_sort_idx(&io_buf[0], &scratch_buf[0], pivot_idx);
  }
  if (num_ele - (pivot_idx + 1) > 1)
  {
    impegh_quick_sort_idx(&io_buf[pivot_idx + 1], &scratch_buf[pivot_idx + 1],
                          num_ele - (pivot_idx + 1));
  }
  return;
}

/**
 *  impegh_quick_sort_azi
 *
 *  \brief Simple quick sort implementation for azimuth values.
 *
 *  \param [in,out] io_buf      Buffer containing elements to be sorted.
 *  \param [in]  scratch_buf Pointer to scratch buffer for processing.
 *  \param [in]  num_ele     Number of elements to be sorted
 *
 *
 *
 */
VOID impegh_quick_sort_azi(ia_spk_azi_idx_str *io_buf, ia_spk_azi_idx_str *scratch_buf,
                           WORD32 num_ele)
{
  WORD32 i, pivot_idx = 0;
  WORD32 spk_order = io_buf[num_ele - 1].spk_order;
  FLOAT32 pivot_element = io_buf[num_ele - 1].spk_azi;
  ia_spk_azi_idx_str *ptr_start = &scratch_buf[0];
  ia_spk_azi_idx_str *ptr_end = &scratch_buf[num_ele - 1];
  for (i = 0; i < (num_ele - 1); i++)
  {
    if (io_buf[i].spk_azi < pivot_element)
    {
      memcpy(ptr_start, io_buf + i, sizeof(io_buf[i]));
      ptr_start++;
      pivot_idx++;
    }
    else
    {
      memcpy(ptr_end, io_buf + i, sizeof(io_buf[i]));
      ptr_end--;
    }
  }
  if (ptr_start == ptr_end)
  {
    ptr_start->spk_order = spk_order;
    ptr_start->spk_azi = pivot_element;
  }
  memcpy(io_buf, scratch_buf, num_ele * sizeof(*io_buf));
  if (pivot_idx > 1)
  {
    impegh_quick_sort_azi(&io_buf[0], &scratch_buf[0], pivot_idx);
  }
  if (num_ele - (pivot_idx + 1) > 1)
  {
    impegh_quick_sort_azi(&io_buf[pivot_idx + 1], &scratch_buf[pivot_idx + 1],
                          num_ele - (pivot_idx + 1));
  }
  return;
}

/**
 *  impeghd_ls_get_ele_index
 *
 *  \brief Calculates Elevation index based on elevation value.
 *
 *  \param [in] elevation Elevation value of the speaker coordinates.
 *
 *  \return WORD32   Elevation Index value.
 *
 */
static WORD32 impeghd_ls_get_ele_index(FLOAT32 elevation)
{
  WORD32 idx_elevation;
  FLOAT32 ele;
  ele = (FLOAT32)(ia_max_flt(-90.0, ia_min_flt(90., elevation)));
  idx_elevation = (WORD32)(ia_fabs_flt(ele) + 0.5f);
  return idx_elevation;
}

/**
 *  impeghd_ls_get_azi_index
 *
 *  \brief Calculates Azimuth index based on azimuth value.
 *
 *  \param [in] azimuth   Azimuth value of the speaker coordinates.
 *
 *  \return WORD32   Azimuth Index value.
 *
 */
static WORD32 impeghd_ls_get_azi_index(FLOAT32 azimuth)
{
  WORD32 idx_azimuth;
  FLOAT32 azi;
  azi = (FLOAT32)(180.0f - fmod(180.0f - azimuth, 360.0));
  if (azi >= 180.0)
  {
    azi = ia_sub_flt(azi, 360.0f);
  }
  idx_azimuth = (WORD32)(ia_fabs_flt(ia_sub_flt(90.0f, (FLOAT32)ia_fabs_flt(azi)) + 0.5f));
  return idx_azimuth;
}

/**
 *  impeghd_ls_get_index
 *
 *  \brief Calculates loud speaker index based on speaker spatial position.
 *
 *  \param [in] azimuth   Azimuth value of the speaker coordinates.
 *  \param [in] elevation Elevation value of the speaker coordinates.
 *
 *  \return WORD32   Index value.
 *
 */
static WORD32 impeghd_ls_get_index(FLOAT32 azimuth, FLOAT32 elevation)
{

  WORD32 idx_azi;
  WORD32 idx_ele;
  WORD32 index;
  idx_azi = impeghd_ls_get_azi_index(azimuth);
  idx_ele = impeghd_ls_get_ele_index(elevation);
  index = (WORD32)(idx_azi + 181.0 * idx_ele);
  return index;
}

/**
 *  impeghd_ls_subset_empty
 *
 *  \brief Returns if the loudspeaker vertices are in the polygon.
 *
 *  \param [in] ptr_ls_vertex   Loud speaker vertex structure
 *  \param [in] ptr_ls_idx      Loud speaker index.
 *  \param [in] num_sub_ls      Number of loud speakers subsets.
 *  \param [in] num_ls          Number of loud speakers.
 *  \param [in] ptr_ls_geo_tbls Pointer to loud speaker gemoetry tables.
 *
 *  \return WORD32 Flag that indicates whether ls subset is empty or not.
 *
 */
static WORD32 impeghd_ls_subset_empty(ia_renderer_ls_params *ptr_ls_vertex, WORD32 *ptr_ls_idx,
                                      WORD32 num_sub_ls, WORD32 num_ls,
                                      ia_cicp_ls_geo_str *ptr_ls_geo_tbls)
{
  WORD32 i = 0, j = 0;
  WORD32 ls_idx = 0;
  WORD32 index, k;
  WORD32 flag_empty = 1;
  WORD32 flag_poly;
  FLOAT32 polygon[OBJ_REN_MAX_VERTICES];
  FLOAT32 vertices[OBJ_REN_MAX_VERTICES];

  for (i = 0; i < num_sub_ls; i++)
  {
    ls_idx = ptr_ls_idx[i];
    polygon[2 * i + 1] = ptr_ls_vertex[ls_idx].ls_elevation;
    polygon[2 * i + 0] = ptr_ls_vertex[ls_idx].ls_azimuth;
  }
  for (j = 0; j < num_ls; j++)
  {
    for (i = 0; i < num_sub_ls; i++)
    {
      ls_idx = ptr_ls_idx[i];
      if (ls_idx == j)
        break;
    }
    if (j != ls_idx)
    {
      FLOAT32 point[2] = {0};
      point[1] = ptr_ls_vertex[j].ls_elevation;
      point[0] = ptr_ls_vertex[j].ls_azimuth;

      flag_poly = 1;
      FLOAT32 front = 180 - (FLOAT32)fmod(360 - point[0], 360);
      for (index = 0; index < num_sub_ls; index++)
      {
        vertices[2 * index + 0] =
            ia_sub_flt(180 - (FLOAT32)fmod(360 - polygon[2 * index], 360), front);
        vertices[2 * index + 1] = ia_sub_flt(polygon[2 * index + 1], point[1]);
      }

      for (index = 0; index < num_sub_ls; index++)
      {
        k = (index + 1) % num_sub_ls;
        flag_poly = flag_poly && ((ia_mul_flt(vertices[2 * k + 0], vertices[2 * index + 1]) -
                                   ia_mul_flt(vertices[2 * k + 1], vertices[2 * index])) >= 0);
      }

      flag_empty = flag_empty && !flag_poly;
    }
  }

  return flag_empty;
}
/**
 *  impeghd_ls_subset_exist
 *
 *  \brief Finds loudspeaker subsets.
 *
 *  \param [in] ptr_ls_vertex   Loud speaker vertex structure
 *  \param [in] ptr_ls_idx      Loud speaker index.
 *  \param [in] num_sub_ls      Number of loud speakers subsets.
 *  \param [in] num_ls          Number of loud speakers.
 *  \param [in] ptr_ls_geo_tbls Pointer to loud speaker gemoetry tables.
 *  \param [in] ptr_scratch     Pointer to scratch memory.
 *
 *  \return WORD32 Flag that indicates whether or not ls subset exists.
 *
 */
static WORD32 impeghd_ls_subset_exist(ia_renderer_ls_params *ptr_ls_vertex, WORD32 *ptr_ls_idx,
                                      WORD32 num_sub_ls, WORD32 num_ls,
                                      ia_cicp_ls_geo_str *ptr_ls_geo_tbls, pVOID ptr_scratch)
{
  WORD32 i, j;
  WORD32 num_sub_ls_present = 0;
  WORD32 flag;
  WORD32 *subset = ptr_scratch;

  for (i = 0; i < num_sub_ls; i++)
  {
    WORD32 ls_idx = ptr_ls_idx[i];
    subset[i] = ls_idx;
    flag = 0;
    for (j = 0; j < num_ls; j++)
    {
      FLOAT32 ele = ptr_ls_vertex[j].ls_elevation;
      FLOAT32 azi = ptr_ls_vertex[j].ls_azimuth;
      if (ele >= ptr_ls_geo_tbls[ls_idx].ls_elevation_start &&
          azi >= ptr_ls_geo_tbls[ls_idx].ls_azimuth_start &&
          ele <= ptr_ls_geo_tbls[ls_idx].ls_elevation_end &&
          azi <= ptr_ls_geo_tbls[ls_idx].ls_azimuth_end)
      {
        flag = 1;
        subset[i] = j;
      }
    }
    num_sub_ls_present = num_sub_ls_present + flag;
  }

  if (num_sub_ls == num_sub_ls_present)
  {
    flag = impeghd_ls_subset_empty(ptr_ls_vertex, subset, num_sub_ls, num_ls, ptr_ls_geo_tbls);
    return flag;
  }
  else
  {
    return 0;
  }
}

/**
 *  impeghd_adjust_ele_azi_ind
 *
 *  \brief Addjust ele,azi,idx of speaker.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structurem,number of
 *                  imaginary ls, max_ele_speak_idx min_ele_speak_idx
 *
 *  \return WORD32 index.
 *
 */
static WORD32 impeghd_adjust_ele_azi_ind(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state,
                                         WORD32 *num_imag_ls, WORD32 *max_ele_spk_idx,
                                         WORD32 *min_ele_spk_idx)
{
  WORD32 index = 0;
  FLOAT32 max_elevation = 0.0;
  FLOAT32 min_elevation = 0.0;
  for (index = 0; index < ptr_obj_ren_dec_state->num_non_lfe_ls; index++)
  {
    FLOAT32 elevation = ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_elevation;
    if (elevation > max_elevation)
    {
      *max_ele_spk_idx = index;
      max_elevation = elevation;
    }
    if (ia_lt_flt(elevation, min_elevation))
    {
      *min_ele_spk_idx = index;
      min_elevation = elevation;
    }
  }
  if (max_elevation > IMAG_LS_ELEVATION_HEIGHT_THRESH)
  {
    ptr_obj_ren_dec_state->height_spks_present = 1;
  }
  else
  {
    ptr_obj_ren_dec_state->height_spks_present = 0;
  }
  if (min_elevation > -IMAG_LS_ELEVATION_THRESH)
  {
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_azimuth = 0.0f;
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_elevation = -90.0f;
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_index = impeghd_ls_get_index(0.0f, -90.0f);
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_order = index;
    *min_ele_spk_idx = index;
    index++;
    *num_imag_ls = *num_imag_ls + 1;
  }
  if (max_elevation < IMAG_LS_ELEVATION_THRESH)
  {
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_azimuth = 0.0f;
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_elevation = 90.0f;
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_index = impeghd_ls_get_index(0.0f, 90.0f);
    ptr_obj_ren_dec_state->non_lfe_ls_str[index].ls_order = index;
    *num_imag_ls = *num_imag_ls + 1;
    *max_ele_spk_idx = index;
    index++;
  }
  return index;
}

/**
 *  impeghd_cicpidx_calc_ls_mean_ele
 *
 *  \brief Calculates mean elevation value of a set of loudspeakders.
 *
 *  \param [in] ptr_ls_idx      Pointer to loud speaker index array.
 *  \param [in] num_sub_ls      Number of loud speaker subsets.
 *  \param [in] ptr_ls_geo_tbls Pointer to loud speaker gemoetry tables
 *
 *  \return WORD32 Mean elevation value of the loudspeakers.
 *
 */
static FLOAT32 impeghd_cicpidx_calc_ls_mean_ele(WORD32 *ptr_ls_idx, WORD32 num_sub_ls,
                                                ia_cicp_ls_geo_str *ptr_ls_geo_tbls)
{
  WORD32 i = 0;
  FLOAT32 mean_ele = 0.0f;
  for (i = 0; i < num_sub_ls; i++)
  {
    WORD32 ls_idx = ptr_ls_idx[i];
    mean_ele = ia_add_flt(mean_ele, ptr_ls_geo_tbls[ls_idx].ls_elevation);
  }
  return mean_ele / ((FLOAT32)num_sub_ls);
}

/**
 *  impeghd_cicpidx_calc_ls_mean_azi
 *
 *  \brief Calculates mean azimuth value of a set of loudspeakders.
 *
 *  \param [in] ptr_ls_idx      Pointer to loud speaker index array.
 *  \param [in] num_sub_ls      Number of loud speaker subsets
 *  \param [in] ptr_ls_geo_tbls Pointer to loud speaker gemoetry tables
 *
 *  \return WORD32 Mean azimuth value of the loudspeakers
 *
 */
static FLOAT32 impeghd_cicpidx_calc_ls_mean_azi(WORD32 *ptr_ls_idx, WORD32 num_sub_ls,
                                                ia_cicp_ls_geo_str *ptr_ls_geo_tbls)
{
  WORD32 i = 0;
  FLOAT32 mean_azi = 0.0f;
  for (i = 0; i < num_sub_ls; i++)
  {
    WORD32 ls_idx = ptr_ls_idx[i];
    mean_azi = ia_add_flt(mean_azi, ptr_ls_geo_tbls[ls_idx].ls_azimuth);
  }
  return mean_azi / ((FLOAT32)num_sub_ls);
}

/**
 *  impeghd_add_imaginary_ls
 *
 *  \brief Add imaginary loud speakers.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *  \param [in] ptr_scratch     Pointer to scratch memory.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE
impeghd_add_imaginary_ls(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state, pVOID ptr_scratch)
{
  WORD32 i, j = 0;
  WORD32 ls_order;
  WORD32 ls_azi_order;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_ls = ptr_obj_ren_dec_state->num_non_lfe_ls;
  WORD32 num_imag_ls = 0;
  WORD32 flag = 0;
  WORD32 max_ele_spk_idx = 0;
  WORD32 min_ele_spk_idx = 0;
  ia_spk_azi_idx_str ls_azi[CICP_MAX_NUM_LS] = {{0}};
  ia_spk_azi_idx_str ls_azi_sorted_scratch[CICP_MAX_NUM_LS] = {{0}};

  i = impeghd_adjust_ele_azi_ind(ptr_obj_ren_dec_state, &num_imag_ls, &max_ele_spk_idx,
                                 &min_ele_spk_idx);
  /* Step 3 Check if subset A speaker set is present in the speaker layout */
  flag = impeghd_ls_subset_exist(&ptr_obj_ren_dec_state->non_lfe_ls_str[0],
                                 (WORD32 *)&ia_loud_speaker_subset_A[0], LS_SUBSET_A_LEN,
                                 (num_ls + num_imag_ls),
                                 (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0], ptr_scratch);
  if (flag)
  {
    FLOAT32 mean_azi = 0.0f, mean_ele = 0.0f;
    mean_ele =
        impeghd_cicpidx_calc_ls_mean_ele((WORD32 *)&ia_loud_speaker_subset_A[0], LS_SUBSET_A_LEN,
                                         (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_elevation = mean_ele;
    mean_azi =
        impeghd_cicpidx_calc_ls_mean_azi((WORD32 *)&ia_loud_speaker_subset_A[0], LS_SUBSET_A_LEN,
                                         (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_azimuth = mean_azi;
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_index = impeghd_ls_get_index(mean_azi, mean_ele);
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_order = i;
    i++;
    num_imag_ls = num_imag_ls + 1;
  }
  else
  {
    /* Step 3 Check if subset B speaker set is present in the speaker layout */
    flag = impeghd_ls_subset_exist(&ptr_obj_ren_dec_state->non_lfe_ls_str[0],
                                   (WORD32 *)&ia_loud_speaker_subset_B[0], LS_SUBSET_B_LEN,
                                   (num_ls + num_imag_ls),
                                   (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0], ptr_scratch);
    if (flag)
    {
      FLOAT32 mean_azi = 0.0f, mean_ele = 0.0f;
      mean_ele = impeghd_cicpidx_calc_ls_mean_ele((WORD32 *)&ia_loud_speaker_subset_B[0],
                                                  LS_SUBSET_B_LEN,
                                                  (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_elevation = mean_ele;
      mean_azi = impeghd_cicpidx_calc_ls_mean_azi((WORD32 *)&ia_loud_speaker_subset_B[0],
                                                  LS_SUBSET_B_LEN,
                                                  (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_azimuth = mean_azi;
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_index =
          impeghd_ls_get_index(mean_azi, mean_ele);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_order = num_ls + num_imag_ls;
      i++;
      num_imag_ls = num_imag_ls + 1;
    }
  }

  /* Step 4 Check if subset C speaker set is present in the speaker layout */
  flag = impeghd_ls_subset_exist(&ptr_obj_ren_dec_state->non_lfe_ls_str[0],
                                 (WORD32 *)&ia_loud_speaker_subset_C[0], LS_SUBSET_C_LEN,
                                 (num_ls + num_imag_ls),
                                 (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0], ptr_scratch);
  if (flag)
  {
    FLOAT32 mean_azi = 0.0f, mean_ele = 0.0f;
    mean_ele =
        impeghd_cicpidx_calc_ls_mean_ele((WORD32 *)&ia_loud_speaker_subset_C[0], LS_SUBSET_C_LEN,
                                         (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_elevation = mean_ele;
    mean_azi =
        impeghd_cicpidx_calc_ls_mean_azi((WORD32 *)&ia_loud_speaker_subset_C[0], LS_SUBSET_C_LEN,
                                         (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_azimuth = mean_azi + 180.0f;

    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_index =
        impeghd_ls_get_index(mean_azi + 180.0f, mean_ele);
    ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_order = num_ls + num_imag_ls;
    i++;
    num_imag_ls = num_imag_ls + 1;
  }
  else
  {
    /* Step 4 Check if subset D speaker set is present in the speaker layout */
    flag = impeghd_ls_subset_exist(&ptr_obj_ren_dec_state->non_lfe_ls_str[0],
                                   (WORD32 *)&ia_loud_speaker_subset_D[0], LS_SUBSET_D_LEN,
                                   (num_ls + num_imag_ls),
                                   (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0], ptr_scratch);
    if (flag)
    {
      FLOAT32 mean_azi = 0.0f, mean_ele = 0.0f;
      mean_ele = impeghd_cicpidx_calc_ls_mean_ele((WORD32 *)&ia_loud_speaker_subset_D[0],
                                                  LS_SUBSET_D_LEN,
                                                  (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_elevation = mean_ele;
      mean_azi = impeghd_cicpidx_calc_ls_mean_azi((WORD32 *)&ia_loud_speaker_subset_D[0],
                                                  LS_SUBSET_D_LEN,
                                                  (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[0]);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_azimuth = ia_add_flt(mean_azi, 180.0f);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_index =
          impeghd_ls_get_index(mean_azi + 180.0f, mean_ele);
      ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_order = num_ls + num_imag_ls;
      i++;
      num_imag_ls = num_imag_ls + 1;
    }
  }
  ls_order = i;
  /* Step 5 */
  for (i = 0, j = 0; i < num_ls; i++)
  {
    if (ia_lt_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_elevation,
                  IMAG_LS_ELEVATION_THRESH))
    {
      ls_azi[j].spk_azi = ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_azimuth;
      ls_azi[j].spk_order = i;
      j++;
    }
  }
  ls_azi_order = j;
  if (ls_azi_order > 0)
  {
    impegh_quick_sort_azi(&ls_azi[0], &ls_azi_sorted_scratch[0], ls_azi_order);
  }

  for (i = 0; i < (ls_azi_order - 1); i++)
  {
    FLOAT32 angle_diff = ia_sub_flt(ls_azi[i + 1].spk_azi, ls_azi[i].spk_azi);
    if (angle_diff > IMAG_LS_MAX_AZIMUTH_DIFF)
    {
      WORD32 num_eq_space_speakers = (WORD32)(angle_diff / IMAG_LS_MAX_AZIMUTH_DIFF);
      WORD32 k;
      FLOAT32 step = angle_diff / (num_eq_space_speakers + 1);
      for (k = 1; k <= num_eq_space_speakers; k++)
      {
        FLOAT32 azimuth = ia_add_flt(ls_azi[i].spk_azi, k * step);
        ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_elevation = 0.0f;
        ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_azimuth = azimuth;
        ls_azi[ls_azi_order].spk_azi = ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_azimuth;
        ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_index =
            impeghd_ls_get_index(azimuth, 0.0f);
        ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_order = ls_order;
        ls_azi[ls_azi_order].spk_order = ls_order;
        num_imag_ls++;
        ls_order++;
        ls_azi_order++;
      }
    }
  }
  if ((ls_azi[0].spk_azi + 360.0f - ls_azi[i].spk_azi) > IMAG_LS_MAX_AZIMUTH_DIFF)
  {
    FLOAT32 angle_diff = ia_sub_flt(ls_azi[0].spk_azi, ls_azi[i].spk_azi) + 360;
    WORD32 num_eq_space_speakers = (WORD32)(angle_diff / IMAG_LS_MAX_AZIMUTH_DIFF);
    WORD32 k;
    FLOAT32 step = angle_diff / (num_eq_space_speakers + 1);
    for (k = 1; k <= num_eq_space_speakers; k++)
    {
      FLOAT32 azimuth = ia_add_flt(ls_azi[i].spk_azi, k * step);
      ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_elevation = 0.0f;
      ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_azimuth = azimuth;
      ls_azi[ls_azi_order].spk_azi = ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_azimuth;
      ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_index =
          impeghd_ls_get_index(azimuth, 0.0f);
      ptr_obj_ren_dec_state->non_lfe_ls_str[ls_order].ls_order = ls_order;
      ls_azi[ls_azi_order].spk_order = ls_order;
      num_imag_ls++;
      ls_order++;
      ls_azi_order++;
    }
  }

  j = 0;
  ptr_obj_ren_dec_state->tri_ls_index[j++] = max_ele_spk_idx;
  ptr_obj_ren_dec_state->tri_ls_index[j++] = min_ele_spk_idx;
  if (ls_azi_order > 0)
  {
    impegh_quick_sort_azi(&ls_azi[0], &ls_azi_sorted_scratch[0], ls_azi_order);
  }
  for (i = 1; i < ls_azi_order; i++)
  {
    if (ia_lteq_flt(ls_azi[i].spk_azi, (ls_azi[0].spk_azi + 160.0f)))
    {
      min_ele_spk_idx = i;
    }
    if (ls_azi[i].spk_azi >= (ls_azi[0].spk_azi + 200.0f))
    {
      max_ele_spk_idx = i;
      break;
    }
  }

  ptr_obj_ren_dec_state->tri_ls_index[j++] = ls_azi[0].spk_order;
  ptr_obj_ren_dec_state->tri_ls_index[j++] = ls_azi[min_ele_spk_idx].spk_order;
  ptr_obj_ren_dec_state->tri_ls_index[j++] = ls_azi[max_ele_spk_idx].spk_order;

  ptr_obj_ren_dec_state->num_init_polyhedron_vtx = j;
  ptr_obj_ren_dec_state->num_imag_ls = num_imag_ls;
  ptr_obj_ren_dec_state->num_vertices = num_imag_ls + ptr_obj_ren_dec_state->num_non_lfe_ls;

  return err_code;
}

/**
 *  impeghd_extract_unique_edges
 *
 *  \brief Unique loudspeaker edge extraction.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *  \param [in]  ptr_triangle_order    Order of the loudspeaker triangles.
 *  \param [in]  num_triangles         Number of loudspeaker triangles.
 *
 *
 *
 */
VOID impeghd_extract_unique_edges(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state,
                                  ia_renderer_ls_triangle *ptr_triangle_order,
                                  WORD32 num_triangles)
{
  WORD32 i = 0, j = 0;
  WORD32 num_non_dup_edges = 0;
  ia_ls_edge_params *ptr_edge_params = &ptr_obj_ren_dec_state->edge_params[0];
  ia_ls_edge_params *ptr_non_dup_edge = &ptr_obj_ren_dec_state->non_dup_edges[0];
  for (i = 0; i < num_triangles; i++)
  {
    WORD32 idx_s, idx_l;

    idx_l = ia_max_int(ptr_triangle_order[i].spk_index[1], ptr_triangle_order[i].spk_index[2]);
    idx_s = ia_min_int(ptr_triangle_order[i].spk_index[1], ptr_triangle_order[i].spk_index[2]);
    ptr_edge_params[3 * i + 2].ls_order_vtx_a = idx_s;
    ptr_edge_params[3 * i + 2].ls_order_vtx_b = idx_l;

    idx_l = ia_max_int(ptr_triangle_order[i].spk_index[0], ptr_triangle_order[i].spk_index[2]);
    idx_s = ia_min_int(ptr_triangle_order[i].spk_index[0], ptr_triangle_order[i].spk_index[2]);
    ptr_edge_params[3 * i + 1].ls_order_vtx_a = idx_s;
    ptr_edge_params[3 * i + 1].ls_order_vtx_b = idx_l;

    idx_l = ia_max_int(ptr_triangle_order[i].spk_index[0], ptr_triangle_order[i].spk_index[1]);
    idx_s = ia_min_int(ptr_triangle_order[i].spk_index[0], ptr_triangle_order[i].spk_index[1]);
    ptr_edge_params[3 * i + 0].ls_order_vtx_a = idx_s;
    ptr_edge_params[3 * i + 0].ls_order_vtx_b = idx_l;
  }

  for (i = 0; i < num_triangles * 3; i++)
  {
    WORD32 unique_edge = 1;
    for (j = 0; j < num_triangles * 3; j++)
    {
      if (j != i)
      {
        if (ptr_edge_params[i].ls_order_vtx_a == ptr_edge_params[j].ls_order_vtx_a &&
            ptr_edge_params[i].ls_order_vtx_b == ptr_edge_params[j].ls_order_vtx_b)
        {
          unique_edge = 0;
        }
      }
    }
    if (unique_edge)
    {
      ptr_non_dup_edge[num_non_dup_edges].ls_order_vtx_b = ptr_edge_params[i].ls_order_vtx_b;
      ptr_non_dup_edge[num_non_dup_edges].ls_order_vtx_a = ptr_edge_params[i].ls_order_vtx_a;
      num_non_dup_edges++;
    }
  }
  ptr_obj_ren_dec_state->num_non_dup_edges = num_non_dup_edges;
}

/**
 *  impeghd_remap_imag_spks
 *
 *  \brief Remap the imaginary loud speakers
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *
 *
 *
 */
VOID impeghd_remap_imag_spks(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state)
{
  WORD32 num_imag_spks = ptr_obj_ren_dec_state->num_imag_ls;
  WORD32 num_real_spks = ptr_obj_ren_dec_state->num_non_lfe_ls;
  WORD32 num_triangles = ptr_obj_ren_dec_state->num_triangles;
  WORD32 i, j;
  WORD32 total_spks = num_real_spks + num_imag_spks;
  FLOAT32 scratch_mtx[CICP_MAX_NUM_LS][CICP_MAX_NUM_LS];
  memset(scratch_mtx, 0, sizeof(scratch_mtx));
  for (i = (num_real_spks); i > 0; i--)
  {
    ptr_obj_ren_dec_state->downmix_mtx[i - 1][i - 1] = 1.0f;
  }
  for (i = num_real_spks; i < total_spks; i++)
  {
    FLOAT32 num_neighbours = 0.0f;
    for (j = 0; j < num_triangles; j++)
    {
      WORD32 v3 = ptr_obj_ren_dec_state->ls_triangle[j].spk_index[2];
      WORD32 v2 = ptr_obj_ren_dec_state->ls_triangle[j].spk_index[1];
      WORD32 v1 = ptr_obj_ren_dec_state->ls_triangle[j].spk_index[0];
      if (v3 == i)
      {
        ptr_obj_ren_dec_state->downmix_mtx[v2][i] = 1.0f;
        ptr_obj_ren_dec_state->downmix_mtx[v1][i] = 1.0f;
      }
      else if (v2 == i)
      {
        ptr_obj_ren_dec_state->downmix_mtx[v3][i] = 1.0f;
        ptr_obj_ren_dec_state->downmix_mtx[v1][i] = 1.0f;
      }
      else if (v1 == i)
      {
        ptr_obj_ren_dec_state->downmix_mtx[v3][i] = 1.0f;
        ptr_obj_ren_dec_state->downmix_mtx[v2][i] = 1.0f;
      }
    }
    for (j = total_spks; j > 0; j--)
    {
      num_neighbours += ptr_obj_ren_dec_state->downmix_mtx[j - 1][i];
    }
    if (num_neighbours != 0.0f)
    {
      for (j = total_spks; j > 0; j--)
      {
        ptr_obj_ren_dec_state->downmix_mtx[j - 1][i] =
            ptr_obj_ren_dec_state->downmix_mtx[j - 1][i] / num_neighbours;
      }
    }
  }
  for (i = 0; i < 20; i++)
  {
    WORD32 l, m, n;
    FLOAT32 max_val = 0.0f;
    for (l = total_spks; l > 0; l--)
    {
      for (m = total_spks; m > 0; m--)
      {
        FLOAT32 mult_res = 0.0f;
        for (n = total_spks; n > 0; n--)
        {
          mult_res += ptr_obj_ren_dec_state->downmix_mtx[n - 1][m - 1] *
                      ptr_obj_ren_dec_state->downmix_mtx[l - 1][n - 1];
        }
        scratch_mtx[l - 1][m - 1] = mult_res;
      }
    }
    for (l = total_spks; l > 0; l--)
    {
      for (m = total_spks; m > 0; m--)
      {
        ptr_obj_ren_dec_state->downmix_mtx[l - 1][m - 1] = scratch_mtx[l - 1][m - 1];
      }
    }
    for (l = num_real_spks; l < total_spks; l++)
    {
      for (m = num_real_spks; m < total_spks; m++)
      {
        max_val = ia_max_int(max_val, scratch_mtx[l][m]);
      }
    }
    if (max_val <= 0.0001f)
    {
      break;
    }
  }

  for (i = total_spks; i > 0; i--)
  {
    for (j = total_spks; j > 0; j--)
    {
      ptr_obj_ren_dec_state->downmix_mtx[i - 1][j - 1] =
          (FLOAT32)ia_sqrt_flt(ptr_obj_ren_dec_state->downmix_mtx[i - 1][j - 1]);
    }
  }
}

/**
 *  impeghd_calc_inv_matrix
 *
 *  \brief Calculates inverse of a matrix.
 *
 *  \param [in]  row1  Pointer to elements of first row of matrix.
 *  \param [in]  row2  Pointer to elements of second row of matrix.
 *  \param [in]  row3  Pointer to elements of third row of matrix.
 *  \param [out] inv_mtx_row Pointer to store inverse matrix.
 *
 *
 *
 */

VOID impeghd_calc_inv_matrix(ia_cart_coord_str *row1, ia_cart_coord_str *row2,
                             ia_cart_coord_str *row3, ia_cart_coord_str inv_mtx_row[3])
{
  FLOAT32 det =
      (ia_mul_flt(row1->z,
                  (ia_sub_flt(ia_mul_flt(row2->x, row3->y), ia_mul_flt(row2->y, row3->x)))) -
       ia_mul_flt(row1->y,
                  (ia_sub_flt(ia_mul_flt(row2->x, row3->z), ia_mul_flt(row2->z, row3->x)))) +
       ia_mul_flt(row1->x,
                  (ia_sub_flt(ia_mul_flt(row2->y, row3->z), ia_mul_flt(row2->z, row3->y)))));
  if (det == 0.0f)
  {
    memset(&inv_mtx_row[0].x, 0, sizeof(inv_mtx_row[3]));
  }
  else
  {
    FLOAT32 one_by_det = 1.0f / det;
    inv_mtx_row[0].x =
        ia_sub_flt(ia_mul_flt(row2->y, row3->z), ia_mul_flt(row2->z, row3->y)) * (one_by_det);
    inv_mtx_row[0].y =
        ia_sub_flt(ia_mul_flt(row2->x, row3->z), ia_mul_flt(row2->z, row3->x)) * (-one_by_det);
    inv_mtx_row[0].z =
        ia_sub_flt(ia_mul_flt(row2->x, row3->y), ia_mul_flt(row2->y, row3->x)) * (one_by_det);
    inv_mtx_row[1].x =
        ia_sub_flt(ia_mul_flt(row1->y, row3->z), ia_mul_flt(row1->z, row3->y)) * (-one_by_det);
    inv_mtx_row[1].y =
        ia_sub_flt(ia_mul_flt(row1->x, row3->z), ia_mul_flt(row1->z, row3->x)) * (one_by_det);
    inv_mtx_row[1].z =
        ia_sub_flt(ia_mul_flt(row1->x, row3->y), ia_mul_flt(row1->y, row3->x)) * (-one_by_det);
    inv_mtx_row[2].x =
        ia_sub_flt(ia_mul_flt(row1->y, row2->z), ia_mul_flt(row1->z, row2->y)) * (one_by_det);
    inv_mtx_row[2].y =
        ia_sub_flt(ia_mul_flt(row1->x, row2->z), ia_mul_flt(row1->z, row2->x)) * (-one_by_det);
    inv_mtx_row[2].z =
        ia_sub_flt(ia_mul_flt(row1->x, row2->y), ia_mul_flt(row1->y, row2->x)) * (one_by_det);
  }
  return;
}

/**
 *  impeghd_vbap_core_init
 *
 *  \brief Vector base amplitude panning core init function.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
static IA_ERRORCODE impeghd_vbap_core_init(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_triangles = ptr_obj_ren_dec_state->num_triangles;
  WORD32 i;

  for (i = 0; i < num_triangles; i++)
  {
    WORD32 v3 = ptr_obj_ren_dec_state->ls_triangle[i].spk_index[2];
    WORD32 v2 = ptr_obj_ren_dec_state->ls_triangle[i].spk_index[1];
    WORD32 v1 = ptr_obj_ren_dec_state->ls_triangle[i].spk_index[0];

    if ((v3 >= CICP_MAX_NUM_LS) || (v2 >= CICP_MAX_NUM_LS) || (v1 >= CICP_MAX_NUM_LS))
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;

    impeghd_calc_inv_matrix(&ptr_obj_ren_dec_state->non_lfe_ls_str[v1].ls_cart_coord,
                            &ptr_obj_ren_dec_state->non_lfe_ls_str[v2].ls_cart_coord,
                            &ptr_obj_ren_dec_state->non_lfe_ls_str[v3].ls_cart_coord,
                            ptr_obj_ren_dec_state->ls_inv_mtx[i].row);
  }

  return err_code;
}

/**
 *  impeghd_obj_renderer_dec_init
 *
 *  \brief Object renderer decoder state init function.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *  \param [in] ptr_scratch     Pointer to scratch memory.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE impeghd_obj_renderer_dec_init(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state,
                                           ia_speaker_config_3d *ptr_ref_spk_layout,
                                           pVOID ptr_scratch)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;

  WORD32 i, j;
  WORD32 in_num_channels, in_num_lfes;
  WORD32 cicp_idx;
  const WORD32 *in_channel_names;
  WORD32 num_visible_triangles = 0, num_additional_triangles = 0;
  ia_renderer_ls_triangle visible_triangle[CICP_MAX_NUM_LS] = {{{0}}};
  ia_renderer_ls_triangle additional_triangle[CICP_MAX_NUM_LS] = {{{0}}};
  ia_ls_ord_idx_params ls_ord_idx[CICP_MAX_NUM_LS] = {{0}};
  ia_ls_ord_idx_params ls_ord_idx_scratch[CICP_MAX_NUM_LS] = {{0}};

  ia_local_setup_struct *pstr_local_setup = ptr_obj_ren_dec_state->pstr_local_setup;
  if (ptr_ref_spk_layout->spk_layout_type == 2 || ptr_ref_spk_layout->spk_layout_type == 1)
  {
    ptr_obj_ren_dec_state->num_cicp_speakers = ptr_ref_spk_layout->num_speakers;
    err_code = impeghd_flex_spk_2_ls_geometry(
        &ptr_obj_ren_dec_state->array_cicp_ls_geo[0], ptr_ref_spk_layout,
        (ia_cicp_ls_geo_str **)&ptr_obj_ren_dec_state->ptr_cicp_ls_geo[0], &in_num_channels,
        &in_num_lfes, (const WORD32 **)&in_channel_names);
    if (err_code)
    {
      return err_code;
    }
  }
  else
  {
    if (pstr_local_setup->enable_ele_int == 0)
    {
      cicp_idx = ptr_obj_ren_dec_state->cicp_out_idx;
      ptr_obj_ren_dec_state->num_cicp_speakers = impgehd_cicp_get_num_ls[cicp_idx];
    }
    else
    {
      cicp_idx = pstr_local_setup->spk_config.cicp_spk_layout_idx;
      ptr_obj_ren_dec_state->num_cicp_speakers = impgehd_cicp_get_num_ls[cicp_idx];
    }
    if (cicp_idx <= 0 || cicp_idx == 8 || cicp_idx > NUM_LS_CFGS)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_CICP_SPKR_INDEX;
    }
    else
    {
      in_channel_names = ia_cicp_idx_ls_set_map_tbl[cicp_idx];
      in_num_channels = impgehd_cicp_get_num_ls[cicp_idx];
      in_num_lfes = 0;
      const ia_cicp_ls_geo_str **pp_cicp_ls_geometry =
          (const ia_cicp_ls_geo_str **)&ptr_obj_ren_dec_state->ptr_cicp_ls_geo[0];
      for (i = 0; i < in_num_channels; i++)
      {
        pp_cicp_ls_geometry[i] = &ia_cicp_ls_geo_tbls[in_channel_names[i]];
        in_num_lfes += pp_cicp_ls_geometry[i]->lfe_flag;
      }
    }
  }
  if (pstr_local_setup->enable_ele_int == 1)
  {
    for (UWORD32 k = 0; k < pstr_local_setup->num_spk; k++)
    {
      pstr_local_setup->ptr_cicp_ls_geo[k].ls_elevation =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->ls_elevation;
      pstr_local_setup->ptr_cicp_ls_geo[k].ls_elevation_start =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->ls_elevation_start;
      pstr_local_setup->ptr_cicp_ls_geo[k].ls_elevation_end =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->ls_elevation_end;
      if (pstr_local_setup->has_knwn_pos[k])
        pstr_local_setup->ptr_cicp_ls_geo[k].ls_elevation =
            (FLOAT32)pstr_local_setup->loud_spk_elevation[k];
      pstr_local_setup->ptr_cicp_ls_geo[k].ls_azimuth =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->ls_azimuth;
      pstr_local_setup->ptr_cicp_ls_geo[k].ls_azimuth_start =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->ls_azimuth_start;
      pstr_local_setup->ptr_cicp_ls_geo[k].ls_azimuth_end =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->ls_azimuth_end;
      if (pstr_local_setup->has_knwn_pos[k])
        pstr_local_setup->ptr_cicp_ls_geo[k].ls_azimuth = pstr_local_setup->loud_spk_azimuth[k];
      pstr_local_setup->ptr_cicp_ls_geo[k].lfe_flag =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->lfe_flag;
      pstr_local_setup->ptr_cicp_ls_geo[k].screen_rel_flag =
          ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k]->screen_rel_flag;
      ptr_obj_ren_dec_state->ptr_cicp_ls_geo[k] =
          (ia_cicp_ls_geo_str *)&pstr_local_setup->ptr_cicp_ls_geo[k];
    }
  }
  j = 0;
  for (i = 0; i < ptr_obj_ren_dec_state->num_cicp_speakers; i++)
  {
    if (ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i] &&
        !ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->lfe_flag)
    {
      ptr_obj_ren_dec_state->non_lfe_ls_str[j].ls_elevation = (FLOAT32)ia_max_flt(
          -90.0, ia_min_flt(90., ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->ls_elevation));
      if (ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->ls_azimuth >= 180)
      {
        ptr_obj_ren_dec_state->non_lfe_ls_str[j].ls_azimuth =
            ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->ls_azimuth - 360;
      }
      else
      {
        ptr_obj_ren_dec_state->non_lfe_ls_str[j].ls_azimuth =
            ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->ls_azimuth;
      }
      ptr_obj_ren_dec_state->non_lfe_ls_str[j].ls_index =
          impeghd_ls_get_index(ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->ls_azimuth,
                               ptr_obj_ren_dec_state->ptr_cicp_ls_geo[i]->ls_elevation);
      ptr_obj_ren_dec_state->non_lfe_ls_str[j].ls_order = j;
      j++;
    }
  }
  ptr_obj_ren_dec_state->num_non_lfe_ls = j;

  err_code = impeghd_add_imaginary_ls(ptr_obj_ren_dec_state, ptr_scratch);
  if (err_code)
  {
    return err_code;
  }

  for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls + ptr_obj_ren_dec_state->num_imag_ls; i++)
  {
    ia_polar_coord_str tmp;
    tmp.radius = 1.0f;
    tmp.phi = ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_elevation;
    tmp.theta = ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_azimuth;
    impeghd_pol_2_cart_degree(&tmp, &ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_cart_coord);
  }

  WORD32 num_triangles = ptr_obj_ren_dec_state->num_init_polyhedron_vtx - 3;
  ia_renderer_ls_triangle *ptr_ren_ls_tri = &ptr_obj_ren_dec_state->ls_triangle[0];
  for (i = 0; i < num_triangles; i++)
  {
    ptr_ren_ls_tri->spk_index[2] = ptr_obj_ren_dec_state->tri_ls_index[i + 3];
    ptr_ren_ls_tri->spk_index[1] = ptr_obj_ren_dec_state->tri_ls_index[i + 2];
    ptr_ren_ls_tri->spk_index[0] = ptr_obj_ren_dec_state->tri_ls_index[0];
    ptr_ren_ls_tri++;
  }
  ptr_ren_ls_tri->spk_index[2] = ptr_obj_ren_dec_state->tri_ls_index[2];
  ptr_ren_ls_tri->spk_index[1] = ptr_obj_ren_dec_state->tri_ls_index[i + 2];
  ptr_ren_ls_tri->spk_index[0] = ptr_obj_ren_dec_state->tri_ls_index[0];
  ptr_ren_ls_tri++;

  for (i = 0; i < num_triangles; i++)
  {
    ptr_ren_ls_tri->spk_index[2] = ptr_obj_ren_dec_state->tri_ls_index[i + 2];
    ptr_ren_ls_tri->spk_index[1] = ptr_obj_ren_dec_state->tri_ls_index[i + 3];
    ptr_ren_ls_tri->spk_index[0] = ptr_obj_ren_dec_state->tri_ls_index[1];
    ptr_ren_ls_tri++;
  }
  ptr_ren_ls_tri->spk_index[2] = ptr_obj_ren_dec_state->tri_ls_index[i + 2];
  ptr_ren_ls_tri->spk_index[1] = ptr_obj_ren_dec_state->tri_ls_index[2];
  ptr_ren_ls_tri->spk_index[0] = ptr_obj_ren_dec_state->tri_ls_index[1];
  ptr_obj_ren_dec_state->num_triangles = 2 * num_triangles + 2;

  for (i = 0; i < ptr_obj_ren_dec_state->num_vertices; i++)
  {
    ls_ord_idx[i].ls_order = ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_order;
    ls_ord_idx[i].ls_index = ptr_obj_ren_dec_state->non_lfe_ls_str[i].ls_index;
  }
  if (ptr_obj_ren_dec_state->num_vertices > 1)
  {
    impegh_quick_sort_idx(&ls_ord_idx[0], &ls_ord_idx_scratch[0],
                          ptr_obj_ren_dec_state->num_vertices);
  }

  ptr_obj_ren_dec_state->num_non_dup_edges = 0;
  for (i = 0; i < (ptr_obj_ren_dec_state->num_non_lfe_ls + ptr_obj_ren_dec_state->num_imag_ls);
       i++)
  {
    WORD32 j;
    num_additional_triangles = 0;
    num_visible_triangles = 0;
    for (j = 0; j < ptr_obj_ren_dec_state->num_triangles; j++)
    {
      WORD32 vx = ls_ord_idx[i].ls_order;
      WORD32 v3 = ptr_obj_ren_dec_state->ls_triangle[j].spk_index[2];
      WORD32 v2 = ptr_obj_ren_dec_state->ls_triangle[j].spk_index[1];
      WORD32 v1 = ptr_obj_ren_dec_state->ls_triangle[j].spk_index[0];
      FLOAT32 proj_len;
      proj_len =
          impeghd_get_vx_proj_len(&ptr_obj_ren_dec_state->non_lfe_ls_str[v1].ls_cart_coord,
                                  &ptr_obj_ren_dec_state->non_lfe_ls_str[v2].ls_cart_coord,
                                  &ptr_obj_ren_dec_state->non_lfe_ls_str[v3].ls_cart_coord,
                                  &ptr_obj_ren_dec_state->non_lfe_ls_str[vx].ls_cart_coord);

      if (proj_len <= OBJ_REN_MIN_PROJ_LEN_VAL)
      {
        additional_triangle[num_additional_triangles].spk_index[2] = v3;
        additional_triangle[num_additional_triangles].spk_index[1] = v2;
        additional_triangle[num_additional_triangles].spk_index[0] = v1;
        num_additional_triangles++;
      }
      else
      {
        visible_triangle[num_visible_triangles].spk_index[2] = v3;
        visible_triangle[num_visible_triangles].spk_index[1] = v2;
        visible_triangle[num_visible_triangles].spk_index[0] = v1;
        num_visible_triangles++;
      }
    }
    impeghd_extract_unique_edges(ptr_obj_ren_dec_state, &visible_triangle[0],
                                 num_visible_triangles);
    for (j = 0; j < ptr_obj_ren_dec_state->num_non_dup_edges; j++)
    {
      FLOAT32 proj_len;
      WORD32 v3 = ls_ord_idx[i].ls_order;
      WORD32 v2 = ptr_obj_ren_dec_state->non_dup_edges[j].ls_order_vtx_b;
      WORD32 v1 = ptr_obj_ren_dec_state->non_dup_edges[j].ls_order_vtx_a;
      ia_cart_coord_str cart_orig = {0.0f, 0.0f, 0.0f};
      proj_len = impeghd_get_vx_proj_len(&ptr_obj_ren_dec_state->non_lfe_ls_str[v1].ls_cart_coord,
                                         &ptr_obj_ren_dec_state->non_lfe_ls_str[v2].ls_cart_coord,
                                         &ptr_obj_ren_dec_state->non_lfe_ls_str[v3].ls_cart_coord,
                                         &cart_orig);

      if (proj_len <= 0)
      {
        additional_triangle[num_additional_triangles].spk_index[2] = v3;
        additional_triangle[num_additional_triangles].spk_index[1] = v2;
        additional_triangle[num_additional_triangles].spk_index[0] = v1;
      }
      else
      {
        additional_triangle[num_additional_triangles].spk_index[2] = v1;
        additional_triangle[num_additional_triangles].spk_index[1] = v2;
        additional_triangle[num_additional_triangles].spk_index[0] = v3;
      }
      num_additional_triangles++;
    }
    memcpy(&ptr_obj_ren_dec_state->ls_triangle[0], &additional_triangle[0],
           num_additional_triangles * sizeof(ptr_obj_ren_dec_state->ls_triangle[0]));
    ptr_obj_ren_dec_state->num_triangles = num_additional_triangles;
  }
  if (ptr_obj_ren_dec_state->num_imag_ls > 0)
  {
    impeghd_remap_imag_spks(ptr_obj_ren_dec_state);
  }
  impeghd_vbap_core_init(ptr_obj_ren_dec_state);

  for (i = MAX_NUM_OAM_OBJS; i > 0; i--)
  {
    ptr_obj_ren_dec_state->first_frame_flag[i - 1] = 1;
  }
  return err_code;
}

/**
 *  impeghd_obj_renderer_dec
 *
 *  \brief Object renderer decode function
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *  \param [in]  p_in_buf              Pointer to input buffer - audio object data.
 *  \param [out] p_out_buf             Pointer to rendered output buffer.
 *  \param [in]  cc_frame_len          Core coder frame length.
 *
 *  \return IA_ERRORCODE               Error code if any.
 *
 */

IA_ERRORCODE
impeghd_obj_renderer_dec(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state, FLOAT32 *p_in_buf,
                         FLOAT32 *p_out_buf, WORD32 cc_frame_len)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ch, obj;
  WORD32 frame_length, num_objects, num_channels, sub_frame_offset;
  WORD32 lfe_counter, obj_idx;
  ia_oam_dec_state_struct *ptr_obj_md_dec_state = &ptr_obj_ren_dec_state->str_obj_md_dec_state;
  num_objects = ptr_obj_md_dec_state->num_objects;
  frame_length = ptr_obj_md_dec_state->p_obj_md_cfg->frame_length;
  num_channels = ptr_obj_ren_dec_state->num_cicp_speakers;
  sub_frame_offset = (ptr_obj_md_dec_state->sub_frame_number - 1) * num_objects;

  for (obj = 0; obj < num_objects; obj++)
  {
    lfe_counter = 0;
    obj_idx = sub_frame_offset + obj;
    err_code = impegh_obj_ren_vbap_process(ptr_obj_ren_dec_state, obj_idx);
    if (err_code)
    {
      return err_code;
    }
    for (ch = 0; ch < num_channels; ch++)
    {
      if (ptr_obj_ren_dec_state->ptr_cicp_ls_geo[ch]->lfe_flag)
      {
        lfe_counter++;
      }
      else
      {
        WORD32 j = 0;
        WORD32 ch_idx = ch - lfe_counter;
        FLOAT32 step = ia_sub_flt(ptr_obj_ren_dec_state->final_gains[ch_idx],
                                  ptr_obj_ren_dec_state->initial_gains[obj][ch_idx]) /
                       frame_length;
        FLOAT32 scale = ptr_obj_ren_dec_state->initial_gains[obj][ch_idx] + step;
        for (j = 0; j < frame_length; j++)
        {
          p_out_buf[j] = ia_mac_flt(p_out_buf[j], p_in_buf[j], scale);
          scale = ia_add_flt(scale, step);
        }
      }
      p_out_buf += cc_frame_len;
    }
    memcpy(&ptr_obj_ren_dec_state->initial_gains[obj][0], &ptr_obj_ren_dec_state->final_gains[0],
           sizeof(ptr_obj_ren_dec_state->final_gains[0]) * (num_channels - lfe_counter));
    p_out_buf -= (cc_frame_len * num_channels);
    p_in_buf += cc_frame_len;
    ptr_obj_ren_dec_state->first_frame_flag[obj] = 0;
  }
  return err_code;
}
/** @} */ /* End of OAMProc */