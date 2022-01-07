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
#include "impeghd_hoa_space_positions.h"

#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_hoa_simple_mtrx.h"

#include "impeghd_hoa_robust_pan.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_ren_mtrx_init_with_mtrx_param
 *
 *  \brief Initialize matrix with parameter
 *
 *  \param [out]  ren_handle    HOA renderer matrix handle
 *  \param [in]    config_handle  HOA configuration handle
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_mtrx_init_with_mtrx_param(ia_render_hoa_render_mtrx_str *ren_handle,
                                                       pVOID config_handle)
{
  ia_hoa_config_struct *ptr_hoa_config = (ia_hoa_config_struct *)config_handle;
  WORD32 c, r;
  ren_handle->order = ptr_hoa_config->order;
  ren_handle->cols = ptr_hoa_config->num_coeffs;
  WORD32 mtx_spk_id = ptr_hoa_config->matrix_spk_id;
  ren_handle->l_speakers = impgehd_cicp_get_num_ls[mtx_spk_id];
  const WORD32 *spk_table = ia_cicp_idx_ls_set_map_tbl[mtx_spk_id];
  ren_handle->num_satellites = 0;
  ren_handle->num_subwoofer = 0;
  for (c = 0; c < ren_handle->l_speakers; c++)
  {
    FLOAT32 s_f, s_t;

    if (ia_cicp_ls_geo_tbls[spk_table[c]].lfe_flag != 0)
    {
      ren_handle->num_subwoofer++;
      ren_handle->spk_idx[c] = 1;
    }
    else
    {
      ren_handle->num_satellites++;
      ren_handle->spk_idx[c] = 0;
    }
    s_t = (FLOAT32)ia_cicp_ls_geo_tbls[spk_table[c]].ls_elevation;
    s_f = (FLOAT32)ia_cicp_ls_geo_tbls[spk_table[c]].ls_azimuth;

    ren_handle->signaled_speaker_pos_rad[c * 3 + 2] = (FLOAT32)(ia_mul_flt((s_f), PI / 180.0f));
    ren_handle->signaled_speaker_pos_rad[c * 3 + 1] =
        (FLOAT32)(ia_mul_flt(ia_sub_flt(90, (s_t)), PI / 180.0f));
    ren_handle->signaled_speaker_pos_rad[c * 3 + 0] = (FLOAT32)(2.0f);
  }
  ren_handle->sm_mtrx.cols = ren_handle->cols;
  ren_handle->sm_mtrx.rows = ren_handle->l_speakers;
  for (r = 0; r < ren_handle->l_speakers; r++)
  {
    for (c = 0; c < ren_handle->cols; c++)
    {
      ren_handle->sm_mtrx.mtrx[r * ren_handle->sm_mtrx.cols + c] =
          ptr_hoa_config->matrix[c * ren_handle->l_speakers + r];
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_mtrx_init
 *
 *  \brief Initialize renderer matrix from speaker position
 *
 *  \param [in,out]  handle    HOA renderer matrix handle
 *  \param [in]    speaker_pos  HOA Space position handle
 *  \param [in]    add_lfe_ch  LFE flag
 *  \param [in]    scratch    Pointer to scratch buffer for intermediate processing
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
static IA_ERRORCODE impeghd_hoa_ren_mtrx_init(pVOID handle, pVOID speaker_pos, WORD32 add_lfe_ch,
                                              pVOID scratch)
{
  pWORD8 buf = (pWORD8)scratch;
  WORD32 scratch_idx = 0, num_s, cnt, c;
  WORD32 spcl_2d_handling = 0;
  IA_ERRORCODE err_code;
  FLOAT32 beta, thresh, scale;
  pFLOAT32 psi_float;
  pFLOAT32 ew;
  ia_render_hoa_simple_mtrx_svd_str *svd;
  ia_render_hoa_simple_mtrx_str *p_g, *psi, *svd_temp;
  ia_render_hoa_space_positions_str *virt_sorces_rtp;
  ia_render_hoa_robust_pan_str *r_pan;
  ia_render_hoa_space_positions_str *spc_handle =
      (ia_render_hoa_space_positions_str *)speaker_pos;
  ia_render_hoa_render_mtrx_str *ren_handle = (ia_render_hoa_render_mtrx_str *)handle;

  if (impeghd_hoa_ren_space_positions_is_2d(spc_handle, (buf + scratch_idx)))
  {
    impeghd_hoa_ren_space_positions_add_poles(spc_handle, 1.0, (buf + scratch_idx));
    spcl_2d_handling = 1;
  }

  r_pan = (ia_render_hoa_robust_pan_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_robust_pan_str);
  ren_handle->l_speakers = spc_handle->num_pos;

  err_code = ia_render_hoa_robust_pan_init_with_spc_positions(r_pan, speaker_pos, 1000, 0, 0,
                                                              (buf + scratch_idx));
  if (err_code)
  {
    return err_code;
  }
  virt_sorces_rtp = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_space_positions_str);
  if (impeghd_hoa_ren_space_positions_init_with_surround(virt_sorces_rtp, HOA_N_PSI_POINTS, NULL,
                                                         (FLOAT32 *)ia_hoa_spk_incl_s1_theta,
                                                         (FLOAT32 *)ia_spk_azimuth_s1_phi))
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
  }
  c = ia_min_int(ren_handle->order - 1, 4);
  beta = ia_hoa_beta_value_table[c];
  err_code = ia_render_hoa_robust_pan_calculate_gains_pre(r_pan, virt_sorces_rtp, beta, 0,
                                                          (buf + scratch_idx));
  if (err_code)
  {
    return err_code;
  }
  p_g = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  impeghd_hoa_ren_simple_mtrx_init_with_matrix(&(r_pan->sm_gain_mtrx), p_g);
  psi_float = (FLOAT32 *)ia_hoa_psi_n6;
  psi = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  WORD32 data_sz = ren_handle->cols * HOA_N_PSI_POINTS;
  psi->cols = HOA_N_PSI_POINTS;
  psi->rows = ren_handle->cols;
  if (ren_handle->cols > MAXIMUM_NUM_HOA_COEFF)
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_MATRIX;
  ia_core_coder_mem_cpy(psi_float, psi->mtrx, data_sz);
  impeghd_hoa_ren_simple_mtrx_transpose(p_g, (buf + scratch_idx));

  svd_temp = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  impeghd_hoa_ren_simple_mtrx_mult(psi, p_g, svd_temp, NULL);

  svd = (ia_render_hoa_simple_mtrx_svd_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_svd_str);
  err_code = impeghd_hoa_ren_simple_mtrx_svd_init(svd, svd_temp, (buf + scratch_idx));
  if (err_code)
  {
    return err_code;
  }
  num_s = svd->w_s_sz;
  pFLOAT32 is_col = (pFLOAT32)(buf + scratch_idx);
  ia_core_coder_memset(is_col, num_s);
  scratch_idx += sizeof(pFLOAT32) * num_s;
  is_col[0] = 1.0;
  thresh = 1e-3f * (svd->w_s[0]);
  for (cnt = 1; cnt < num_s; cnt++)
  {
    is_col[cnt] = 1.0;
    if (svd->w_s[cnt] < thresh)
      break;
  }
  impeghd_hoa_ren_simple_mtrx_diag_mult(&(svd->v), is_col, num_s);
  impeghd_hoa_ren_simple_mtrx_transpose(&(svd->u), (buf + scratch_idx));
  impeghd_hoa_ren_simple_mtrx_mult(&(svd->v), &(svd->u), &(ren_handle->sm_mtrx), NULL);
  scratch_idx += sizeof(FLOAT32) * (HOA_MAX_ARR_SIZE);

  if (ren_handle->l_speakers >= ((ren_handle->order + 1) * (ren_handle->order + 1)))
  {
    ew = (FLOAT32 *)ia_hoa_3d_mix_ren_win_tbl[ren_handle->order];
  }
  else
  {
    ew = (FLOAT32 *)ia_hoa_kaiser_win_tbl[ren_handle->order];
  }
  impeghd_hoa_ren_simple_mtrx_diag_mult(&(ren_handle->sm_mtrx), ew, ren_handle->cols);

  scale = 1.0f / impeghd_hoa_ren_simple_mtrx_norm_fro(&(ren_handle->sm_mtrx));
  for (WORD32 rc = 0; rc < ren_handle->sm_mtrx.rows * ren_handle->sm_mtrx.cols; rc++)
  {
    ren_handle->sm_mtrx.mtrx[rc] = ia_mul_flt(ren_handle->sm_mtrx.mtrx[rc], scale);
  }
  if (spcl_2d_handling)
  {
    WORD32 r;
    FLOAT32 scale_m, gain;
    ia_render_hoa_simple_mtrx_str *m;

    ren_handle->l_speakers = ren_handle->l_speakers - 2;

    m = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
    scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
    impeghd_hoa_ren_simple_mtrx_init_with_size(m, ren_handle->l_speakers, ren_handle->cols);
    gain = (FLOAT32)(1.0f / ia_sqrt_flt((ren_handle->l_speakers)));
    for (r = 0; r < ren_handle->l_speakers; r++)
      for (c = 0; c < ren_handle->cols; c++)
      {
        m->mtrx[r * m->cols + c] =
            ((ren_handle->sm_mtrx
                  .mtrx[(ren_handle->l_speakers + 1) * ren_handle->sm_mtrx.cols + c] +
              ren_handle->sm_mtrx.mtrx[ren_handle->l_speakers * ren_handle->sm_mtrx.cols + c]) *
             gain) +
            ren_handle->sm_mtrx.mtrx[r * ren_handle->sm_mtrx.cols + c];
      }
    scale_m = 1.0f / impeghd_hoa_ren_simple_mtrx_norm_fro(m);

    for (WORD32 rc = 0; rc < m->rows * m->cols; rc++)
    {
      m->mtrx[rc] = ia_mul_flt(m->mtrx[rc], scale_m);
    }
    impeghd_hoa_ren_simple_mtrx_init_with_matrix(m, &(ren_handle->sm_mtrx));

    scratch_idx -= sizeof(ia_render_hoa_simple_mtrx_str);
  }

  if (add_lfe_ch > 0)
  {
    WORD32 r, l;
    ia_render_hoa_simple_mtrx_str *m_tmp = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
    impeghd_hoa_ren_simple_mtrx_init_with_matrix(&(ren_handle->sm_mtrx), m_tmp);
    ren_handle->sm_mtrx.cols = ren_handle->cols;
    ren_handle->sm_mtrx.rows = ren_handle->l_speakers + add_lfe_ch;

    for (c = 0; c < ren_handle->cols; c++)
    {
      for (r = 0; r < ren_handle->l_speakers; r++)
      {
        ren_handle->sm_mtrx.mtrx[r * ren_handle->sm_mtrx.cols + c] =
            m_tmp->mtrx[r * m_tmp->cols + c];
      }
      for (l = 0; l < add_lfe_ch; l++)
      {
        ren_handle->sm_mtrx.mtrx[(ren_handle->l_speakers + l) * ren_handle->sm_mtrx.cols + c] =
            0.0;
      }
    }
  }
  ren_handle->l_speakers += ren_handle->num_subwoofer;
  ren_handle->num_subwoofer = add_lfe_ch;
  ren_handle->num_satellites = ren_handle->l_speakers;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_mtrx_init_with_spc_pos_param
 *
 *  \brief Initialize renderer matrix with
 *      space position parameters
 *
 *  \param [out]  ren_handle    HOA renderer matrix handle
 *  \param [in]    parsed_spk_id  Speaker Id
 *  \param [in]    order      HOA order
 *  \param [in]    add_lfe_ch    LFE flag
 *  \param [in]    scratch      Pointer to scratch buffer for intermediate
 *processing
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE
impeghd_hoa_ren_mtrx_init_with_spc_pos_param(ia_render_hoa_render_mtrx_str *ren_handle,
                                             WORD32 parsed_spk_id,
                                             ia_speaker_config_3d *ref_spk_layout, WORD32 order,
                                             WORD32 add_lfe_ch, pVOID scratch)
{
  WORD32 scratch_idx = 0, add_n_lfe_ch = 0, i;
  pWORD8 buf = (pWORD8)scratch;
  const WORD32 *speaker_table;
  WORD32 loop;
  ia_render_hoa_space_positions_str *speaker_pos;
  speaker_pos = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_space_positions_str);

  ren_handle->cols = (order + 1) * (order + 1);
  ren_handle->order = order;

  if (add_lfe_ch)
  {
    ia_render_hoa_space_positions_str *lfe_pos =
        (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
    scratch_idx += sizeof(ia_render_hoa_space_positions_str);
    if (impeghd_hoa_ren_space_positions_init_with_param(lfe_pos, parsed_spk_id, ref_spk_layout,
                                                        1))
    {
      return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
    }
    add_n_lfe_ch = lfe_pos->num_pos;
  }
  if (impeghd_hoa_ren_space_positions_init_with_param(speaker_pos, parsed_spk_id, ref_spk_layout,
                                                      0))
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
  }
  if (impeghd_hoa_ren_mtrx_init(ren_handle, speaker_pos, add_n_lfe_ch, (buf + scratch_idx)))
  {
    return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;
  }
  memset(ren_handle->spk_idx, 0, HOA_MAX_VECTOR_SIZE * sizeof(UWORD32));
  if (add_n_lfe_ch)
  {
    if (ref_spk_layout->spk_layout_type == 0)
    {
      WORD32 parsed_spk_id = ref_spk_layout->cicp_spk_layout_idx;
      loop = impgehd_cicp_get_num_ls[parsed_spk_id];
      speaker_table = ia_cicp_idx_ls_set_map_tbl[parsed_spk_id];
      for (i = 0; i < loop; i++)
      {
        if (1 == ia_cicp_ls_geo_tbls[speaker_table[i]].lfe_flag)
          ren_handle->spk_idx[i] = 1;
      }
    }
    else if (ref_spk_layout->spk_layout_type == 1)
    {

      loop = ref_spk_layout->num_speakers;

      for (i = 0; i < loop; i++)
      {
        speaker_table = ia_cicp_idx_ls_set_map_tbl[ref_spk_layout->cicp_spk_idx[i]];
        if (1 == ia_cicp_ls_geo_tbls[speaker_table[i]].lfe_flag)
          ren_handle->spk_idx[i] = 1;
      }
    }
    else if (ref_spk_layout->spk_layout_type == 2)
    {
      loop = ref_spk_layout->num_speakers;

      for (i = 0; i < loop; i++)
      {
        if (1 == ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].is_lfe)
          ren_handle->spk_idx[i] = 1;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_hoa_ren_mtrx_resort_spk_pos
*
*  \brief Resort speaker position for speaker matrix
*
*  \param [in,out]  handle  HOA renderer matrix handle
*  \param [in]    order  Order array
*  \param [in]    scratch  Pointer to scratch buffer for intermediate processing
*
 *
 *
*/
VOID impeghd_hoa_ren_mtrx_resort_spk_pos(pVOID handle, const pUWORD32 order, pVOID scratch)
{
  pUWORD32 temp_ls_type = (UWORD32 *)scratch;
  FLOAT32 temp_signaled_speaker_pos_rad[HOA_MAX_VECTOR_SIZE];
  WORD32 i;
  ia_render_hoa_render_mtrx_str *ren_handle = (ia_render_hoa_render_mtrx_str *)handle;

  for (i = 0; i < HOA_MAX_VECTOR_SIZE; i++)
  {
    temp_signaled_speaker_pos_rad[i] = 1000.0;
    temp_ls_type[i] = 2;
  }

  for (i = 0; i < ren_handle->l_speakers; ++i)
  {
    temp_ls_type[order[i]] = ren_handle->spk_idx[i];
  }

  for (i = 0; i < ren_handle->l_speakers; ++i)
  {
    ia_core_coder_mem_cpy(&ren_handle->signaled_speaker_pos_rad[i * 3],
                          &temp_signaled_speaker_pos_rad[order[i] * 3], 3);
  }

  memcpy(&ren_handle->spk_idx[0], &temp_ls_type[0], sizeof(UWORD32) * HOA_MAX_VECTOR_SIZE);
  ia_core_coder_mem_cpy(&temp_signaled_speaker_pos_rad[0],
                        &ren_handle->signaled_speaker_pos_rad[0], HOA_MAX_VECTOR_SIZE);
}
/** @} */ /* End of HOAProc */