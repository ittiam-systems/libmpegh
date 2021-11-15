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
#include <stdlib.h>
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
#include "impeghd_hoa_nfc_filtering.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_renderer.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_2_geometry_rom.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_ren_nfc_init
 *
 *  \brief Intialize renderer
 *      near field compensation (NFC)
 *
 *  \param [in/out]  handle    HOA Renderer handle
 *  \param [in]    order    HOA order
 *  \param [in]    nfc_radius  Near field compensation radius
 *  \param [in]    sample_rate  Sample rate
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_nfc_init(pVOID handle, UWORD32 order, FLOAT32 nfc_radius,
                                      UWORD32 sample_rate)
{
  UWORD32 order_idx, max_comp, j;
  UWORD32 curr_comp = 0;
  UWORD32 num_coeff = (order + 1) * (order + 1);
  FLOAT32 tnfs;
  FLOAT32 tnfc;
  ia_render_hoa_nfc_filtering_str *nfc_handle;
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;

  rh_handle->nfc_filter_sz = num_coeff;

  tnfs = nfc_radius / 343.f * sample_rate;
  tnfc = rh_handle->ldspk_dist / 343.f * sample_rate;

  for (order_idx = 0; order_idx <= order; order_idx++)
  {
    max_comp = 2 * order_idx + 1;
    for (j = 0; j < max_comp; j++)
    {
      nfc_handle = &(rh_handle->nfc_filter[curr_comp]);
      nfc_handle->order = order;
      nfc_handle->num_iir_1_filts = nfc_handle->order % 2;
      nfc_handle->num_iir_2_filts = (nfc_handle->order - nfc_handle->num_iir_1_filts) / 2;
      impeghd_hoa_ren_nfc_set_filter_params(nfc_handle, tnfc, tnfs);
      curr_comp++;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_preprocess_nfc
 *
 *  \brief Preprocess renderer
 *      near field compensation (NFC)
 *
 *  \param [in/out]  handle  HOA Renderer handle
 *  \param [in]    in_buf  Input buffer
 *  \param [in]    rows  Number of rows
 *  \param [in]    cols  Number of columns
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
static IA_ERRORCODE impeghd_hoa_ren_preprocess_nfc(pVOID handle, pFLOAT32 in_buf, WORD32 rows,
                                                   UWORD32 cols)
{
  pFLOAT32 ptr_in_buf;
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;

  WORD32 hoa_channels = ((rh_handle->v_render_mtrx[rh_handle->mtrx_selected].order) + 1) *
                        ((rh_handle->v_render_mtrx[rh_handle->mtrx_selected].order) + 1);

  if (hoa_channels != rows || hoa_channels < rh_handle->nfc_filter_sz)
  {
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_HOA_CHANNELS;
  }
  else if (rh_handle->use_nfc)
  {
    for (WORD32 mn = 0; mn < hoa_channels; mn++)
    {
      ptr_in_buf = in_buf + mn * cols;
      impeghd_hoa_ren_nfc_filter_apply(&(rh_handle->nfc_filter[mn]), ptr_in_buf, ptr_in_buf,
                                       cols);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_block_process
 *
 *  \brief Process renderer block
 *
 *  \param [in]    handle              HOA Renderer
 * handle
 *  \param [in]    in_buf              Input buffer
 *  \param [in]    rows              Number of rows
 *  \param [in]    cols              Number of columns
 * buffer
 *  \param [in]    is_clipping_protect_enabled    Clipping protect flag
 *  \param [in]    scratch              Pointer to scratch buffer
 * for
 * intermediate processing
 *  \param [out]  out                Output
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_block_process(pVOID handle, pFLOAT32 in_buf, WORD32 rows,
                                           UWORD32 cols, WORD32 is_clipping_protect_enabled,
                                           pVOID scratch, pFLOAT32 out)
{
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;
  FLOAT32 val;
  pFLOAT32 trans_in_buf, trans_head, ptr_in_buf, ptr_out_buf;
  WORD32 ch, och, nt, mn;
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 valid_samples = cols;

  if (rh_handle->mtrx_selected == -1)
  {
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_MATRIX_SELECTED;
  }
  ch = ((rh_handle->v_render_mtrx[rh_handle->mtrx_selected].order) + 1) *
       ((rh_handle->v_render_mtrx[rh_handle->mtrx_selected].order) + 1);
  if (ch != rows)
    return IA_MPEGH_HOA_INIT_FATAL_INVALID_MATRIX_SELECTED;
  if (valid_samples == 0)
    return IA_MPEGH_DEC_NO_ERROR;

  if (rh_handle->use_nfc)
  {
    err = impeghd_hoa_ren_preprocess_nfc(rh_handle, in_buf, rows, cols);
    if (err)
      return err;
  }

  trans_head = (FLOAT32 *)(scratch);
  trans_in_buf = trans_head;
  ptr_out_buf = out;

  for (nt = 0; nt < valid_samples; nt++)
  {
    ptr_in_buf = in_buf + nt;
    for (mn = 0; mn < ch; mn++)
    {
      *trans_in_buf = (*ptr_in_buf);
      ptr_in_buf += cols;
      trans_in_buf++;
    }
  }
  WORD32 cols_a = 0, offset = 0;
  const FLOAT32 *ptr_a;
  const FLOAT32 *mtx_a =
      (const FLOAT32 *)&(rh_handle->v_render_mtrx[rh_handle->mtrx_selected].sm_mtrx.mtrx);

  cols_a = rh_handle->v_render_mtrx[rh_handle->mtrx_selected].sm_mtrx.cols;

  for (och = 0; och < rh_handle->num_out_channels; och++)
  {
    ptr_in_buf = trans_head;
    offset = och * cols_a;
    for (nt = 0; nt < valid_samples; nt++)
    {
      ptr_a = mtx_a + offset;
      val = 0;
      for (mn = 0; mn < ch; mn++)
      {
        val = ia_mac_flt(val, (*ptr_a), (*ptr_in_buf));
        ptr_in_buf++;
        ptr_a++;
      }
      if (is_clipping_protect_enabled)
      {
        val = ia_min_int(val, MAX_HOA_OUT_SAMP);
        val = ia_max_int(val, MIN_HOA_OUT_SAMP);
      }
      *ptr_out_buf = val;
      ptr_out_buf++;
    }
  }
  return err;
}

/**
 *  impeghd_hoa_ren_get_loudspeaker_type
 *
 *  \brief Get loudspeaker type
 *
 *  \param [in]  handle    HOA Renderer handle
 *  \param [in]  ls_types  Ludspeaker types array
 *
 *  \return WORD32  Index of loudspeaker type
 *
 */
WORD32 impeghd_hoa_ren_get_loudspeaker_type(pVOID handle, WORD32 *ls_types)
{
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;
  WORD32 spk_idx = rh_handle->spk_idx;
  WORD32 loop = impgehd_cicp_get_num_ls[spk_idx];
  WORD32 idx;
  const WORD32 *speaker_table = ia_cicp_idx_ls_set_map_tbl[spk_idx];

  for (idx = 0; idx < loop; idx++)
  {
    if (1 != ia_cicp_ls_geo_tbls[speaker_table[idx]].lfe_flag)
    {
      ls_types[idx] = HOA_LS_TYPE_SATELLITE;
    }
    else
    {
      ls_types[idx] = HOA_LS_TYPE_SUBWOOFER;
    }
  }
  for (; idx < HOA_MAX_ARR_SIZE; idx++)
  {
    ls_types[idx] = HOA_LS_TYPE_SATELLITE;
  }
  return loop;
}

/**
 *  impeghd_hoa_ren_permute_lfe_channel
 *
 *  \brief Permute Low-frequency effects channel
 *
 *  \param [in/out]  handle    HOA Renderer handle
 *  \param [in]    spk_pos    HOA renderer space position handle
 *  \param [in]    ls_types    Loudspeaker types array
 *  \param [in]    ls_types_sz  Loudspeaker types array size
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_permute_lfe_channel(pVOID handle, pVOID spk_pos, WORD32 *ls_types,
                                                 WORD32 ls_types_size)
{
  ia_render_hoa_simple_mtrx_str *permute_mtrx;
  ia_render_hoa_space_positions_str *sp_handle = (ia_render_hoa_space_positions_str *)spk_pos;
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;
  const WORD32 num_hoa_orders = rh_handle->render_mtrx;
  UWORD32 num_sub_idx, num_hoa_idx, idx;
  WORD32 scratch_idx = 0, n;
  pWORD8 buf;

  if ((sp_handle->type) != 1)
    return IA_MPEGH_HOA_INIT_NONFATAL_RENDERER_INIT_FAILED;
  buf = (pWORD8)(rh_handle->scratch) + *(rh_handle->scratch_idx);
  permute_mtrx = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  impeghd_hoa_ren_simple_mtrx_init_with_size(permute_mtrx, ls_types_size, ls_types_size);

  num_hoa_idx = 0;
  num_sub_idx = sp_handle->num_pos;

  for (n = 0; n < ls_types_size; ++n)
  {
    if (ls_types[n] != HOA_LS_TYPE_SUBWOOFER)
    {
      idx = num_hoa_idx;
      num_hoa_idx++;
    }
    else
    {
      idx = num_sub_idx;
      num_sub_idx++;
    }
    ia_core_coder_memset(&permute_mtrx->mtrx[n * permute_mtrx->cols], ls_types_size);
    permute_mtrx->mtrx[n * permute_mtrx->cols + idx] = 1.0;
  }
  for (n = 0; n < num_hoa_orders; n++)
  {
    impeghd_hoa_ren_simple_mtrx_mult(permute_mtrx, &(rh_handle->v_render_mtrx[n].sm_mtrx),
                                     &(rh_handle->v_render_mtrx[n].sm_mtrx), buf + scratch_idx);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_ren_angular_distance
 *
 *  \brief Calculate angular distance
 *
 *  \param [in]  incl_a  Inclination A
 *  \param [in]  azi_a  Azimuth A
 *  \param [in]  incl_b  Inclination B
 *  \param [in]  azi_b  Azimuth B
 *
 *  \return FLOAT32  Angular distance
 *
 */
static FLOAT32 impeghd_hoa_ren_angular_distance(FLOAT32 incl_a, FLOAT32 azi_a, FLOAT32 incl_b,
                                                FLOAT32 azi_b)
{
  return (FLOAT32)(
      acos(ia_min_flt(1.0, (ia_max_flt(-1.0,
                                       (cos(azi_a - azi_b) * (sin(incl_a) * sin(incl_b))) +
                                           (cos(incl_b) * cos(incl_a)))))));
}

/**
 *  impeghd_hoa_ren_permute_signaled_rendering_matrix
 *
 *  \brief Permutes signaled rendering matrix
 *
 *  \param [in/out]  handle    HOA Renderer handle
 *  \param [in]    spk_pos    HOA renderer space position handle
 *  \param [in]    lfe_pos    LFE handle
 *  \param [in]    ls_types  Loudspeaker types array
 *  \param [in]    ls_types_sz  Loudspeaker types array size
 *
 *  \return WORD32  Is matching mtx
 *
 */
WORD32 impeghd_hoa_ren_permute_signaled_rendering_matrix(pVOID handle, pVOID spk_pos,
                                                         pVOID lfe_pos, WORD32 *ls_types,
                                                         WORD32 ls_types_sz)
{
  ia_render_hoa_space_positions_str *lfe_handle = (ia_render_hoa_space_positions_str *)lfe_pos;
  ia_render_hoa_space_positions_str *spk_handle = (ia_render_hoa_space_positions_str *)spk_pos;
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;

  const WORD32 n = rh_handle->render_mtrx;
  WORD32 scratch_idx = 0;
  pWORD8 buf = (pWORD8)rh_handle->scratch + *(rh_handle->scratch_idx);
  WORD32 is_matching_mtx = 1;
  if ((rh_handle->num_out_channels != (rh_handle->v_render_mtrx[n].l_speakers)) ||
      ((rh_handle->v_render_mtrx[n].num_satellites) != (spk_handle->num_pos)) ||
      ((rh_handle->v_render_mtrx[n].num_subwoofer) != (lfe_handle->num_pos)))
  {
    is_matching_mtx = 0;
  }
  else if (is_matching_mtx)
  {
    FLOAT32 big_value = 1111111;
    ia_render_hoa_simple_mtrx_str *permute_mtrx;
    FLOAT32 incl_local, azi_local, incl_signaled, azi_signaled;
    UWORD32 index;
    pUWORD32 new_order;
    pFLOAT32 ptr_dist_mtrx;
    WORD32 signaled_ls, local_ls, m;
    WORD32 local_ls_idx = 0;
    WORD32 local_lfe_idx = 0;

    pFLOAT32 dist_mtrx = (pFLOAT32)(buf + scratch_idx);
    scratch_idx +=
        (sizeof(FLOAT32) * (rh_handle->num_out_channels) * (rh_handle->num_out_channels));

    for (local_ls = 0; local_ls < rh_handle->num_out_channels; ++local_ls)
    {
      if (!ls_types[local_ls])
      {
        azi_local = spk_handle->arr[local_ls_idx * 3 + 2];
        incl_local = spk_handle->arr[local_ls_idx * 3 + 1];
        local_ls_idx++;
      }
      else
      {
        azi_local = lfe_handle->arr[local_lfe_idx * 3 + 2];
        incl_local = lfe_handle->arr[local_lfe_idx * 3 + 1];
        local_lfe_idx++;
      }

      for (signaled_ls = 0; signaled_ls < rh_handle->num_out_channels; ++signaled_ls)
      {
        ptr_dist_mtrx = dist_mtrx + signaled_ls * (rh_handle->num_out_channels);

        if ((WORD32)rh_handle->v_render_mtrx[n].spk_idx[signaled_ls] == ls_types[local_ls])
        {
          azi_signaled =
              rh_handle->v_render_mtrx[n].signaled_speaker_pos_rad[signaled_ls * 3 + 2];
          incl_signaled =
              rh_handle->v_render_mtrx[n].signaled_speaker_pos_rad[signaled_ls * 3 + 1];
          ptr_dist_mtrx[local_ls] = impeghd_hoa_ren_angular_distance(incl_local, azi_local,
                                                                     incl_signaled, azi_signaled);
        }
        else
        {
          ptr_dist_mtrx[local_ls] = big_value;
        }
      }
    }
    new_order = (UWORD32 *)(buf + scratch_idx);
    scratch_idx += (sizeof(UWORD32) * rh_handle->num_out_channels);
    for (signaled_ls = 0; signaled_ls < rh_handle->num_out_channels; ++signaled_ls)
    {
      ptr_dist_mtrx = dist_mtrx + signaled_ls * (rh_handle->num_out_channels);

      FLOAT32 min = *ptr_dist_mtrx;
      index = 0;
      for (WORD32 i = 0; i < rh_handle->num_out_channels; i++)
      {
        if (min > ptr_dist_mtrx[i])
        {
          min = ptr_dist_mtrx[i];
          index = i;
        }
      }
      new_order[signaled_ls] = index;
      for (m = 0; m < rh_handle->num_out_channels; ++m)
        dist_mtrx[m * (rh_handle->num_out_channels) + index] = big_value;
    }
    permute_mtrx = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
    scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
    impeghd_hoa_ren_simple_mtrx_init_with_size(permute_mtrx, ls_types_sz, ls_types_sz);
    for (signaled_ls = 0; signaled_ls < rh_handle->num_out_channels; ++signaled_ls)
    {
      for (local_ls = 0; local_ls < rh_handle->num_out_channels; ++local_ls)
      {
        if ((WORD32)new_order[signaled_ls] != local_ls)
        {
          permute_mtrx->mtrx[local_ls * permute_mtrx->cols + signaled_ls] = 0.0;
        }
        else
        {
          permute_mtrx->mtrx[local_ls * permute_mtrx->cols + signaled_ls] = 1.0;
        }
      }
    }
    impeghd_hoa_ren_simple_mtrx_mult(permute_mtrx, &(rh_handle->v_render_mtrx[n].sm_mtrx),
                                     &(rh_handle->v_render_mtrx[n].sm_mtrx), (buf + scratch_idx));
    impeghd_hoa_ren_mtrx_resort_spk_pos(&(rh_handle->v_render_mtrx[n]), new_order,
                                        (buf + scratch_idx));
  }

  return is_matching_mtx;
}

/**
 *  impeghd_hoa_ren_validate_signaled_rendering_matrix
 *
 *  \brief Validate renderer signaled matrix
 *
 *  \param handle    HOA Renderer handle
 *  \param spk_pos    HOA renderer space position handle
 *  \param ls_types    Loudspeaker types array
 *  \param ls_types_sz  Loudspeaker types array size
 *  \param n      Matrix index
 *
 *  \return WORD32  Is matching mtx
 *
 */
WORD32 impeghd_hoa_ren_validate_signaled_rendering_matrix(pVOID handle, pVOID spk_pos,
                                                          WORD32 *ls_types, WORD32 ls_types_sz,
                                                          const WORD32 n)
{
  FLOAT32 incl_local, azi_local, incl_signaled, azi_signaled;
  UWORD32 spk_idx = 0;
  FLOAT32 tolerance = HOA_REN_VALIDATE_TOLERANCE;
  WORD32 is_matching_mtx = 1;
  ia_render_hoa_space_positions_str *spk_handle = (ia_render_hoa_space_positions_str *)spk_pos;
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;

  for (WORD32 i = 0; i < ls_types_sz; ++i)
  {
    if ((UWORD32)ls_types[i] != rh_handle->v_render_mtrx[n].spk_idx[i])
    {
      is_matching_mtx = 0;
    }

    if (!ls_types[i])
    {
      azi_local = spk_handle->arr[spk_idx * 3 + 2];
      incl_local = spk_handle->arr[spk_idx * 3 + 1];
      spk_idx++;

      azi_signaled = rh_handle->v_render_mtrx[n].signaled_speaker_pos_rad[i * 3 + 2];
      incl_signaled = rh_handle->v_render_mtrx[n].signaled_speaker_pos_rad[i * 3 + 1];

      if (impeghd_hoa_ren_angular_distance(incl_local, azi_local, incl_signaled, azi_signaled) >
          tolerance)
      {
        is_matching_mtx = 0;
      }
    }
  }
  return is_matching_mtx;
}
/** @} */ /* End of HOAProc */