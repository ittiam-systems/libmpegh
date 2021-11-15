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
#include "impeghd_tbe_dec.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_robust_pan.h"

#define IS_PLANE_WAVE_MODEL 1

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  ia_render_hoa_robust_pan_init
 *
 *  \brief Initialze robust panning
 *
 *  \param [in/out]  handle  HOA robust panning handle
 *  \param [in]    scratch Pointer to scratch buffer for intermediate processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE ia_render_hoa_robust_pan_init(pVOID handle, pVOID scratch)
{
  ia_render_hoa_robust_pan_str *rbs_handle = (ia_render_hoa_robust_pan_str *)handle;
  ia_render_hoa_space_positions_str *virt_mic_rtp;
  WORD32 l, s, scratch_idx = 0, i;
  FLOAT32 imag;
  pFLOAT32 r;
  pWORD8 buf = (pWORD8)scratch;

  impeghd_hoa_ren_space_positions_convert_copy_to_spherical(&(rbs_handle->sp_speaker_pos_cart),
                                                            &(rbs_handle->sp_speaker_pos_Sp));

  if (rbs_handle->radius == 0.0)
  {
    FLOAT32 num_pos = (FLOAT32)(rbs_handle->sp_speaker_pos_cart.num_pos);
    rbs_handle->radius =
        (FLOAT32)(ia_mul_double_flt((ia_sqrt_flt(num_pos) - 1) / rbs_handle->k, 0.4667f));
  }
  r = (FLOAT32 *)(buf + scratch_idx);
  scratch_idx += sizeof(FLOAT32) * (HOA_NUM_VIRTMICS);
  for (i = 0; i < HOA_NUM_VIRTMICS; i++)
    r[i] = rbs_handle->radius;
  virt_mic_rtp = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_space_positions_str);
  if (impeghd_hoa_ren_space_positions_init_with_surround(virt_mic_rtp, HOA_NUM_VIRTMICS, r,
                                                         (FLOAT32 *)ia_hoa_spk_incl_m1_theta,
                                                         (FLOAT32 *)ia_hoa_spk_azimuth_m1_phi))
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
  }
  impeghd_hoa_ren_space_positions_convert_copy_to_cartesian(virt_mic_rtp,
                                                            &(rbs_handle->sp_virt_mic_xyz), 0);
  scratch_idx -= (sizeof(ia_render_hoa_space_positions_str));
  rbs_handle->sm_hml_i.cols = rbs_handle->sp_speaker_pos_cart.num_pos;
  rbs_handle->sm_hml_i.rows = HOA_NUM_VIRTMICS;
  rbs_handle->sm_hml_r.cols = rbs_handle->sp_speaker_pos_cart.num_pos;
  rbs_handle->sm_hml_r.rows = HOA_NUM_VIRTMICS;

  if (!IS_PLANE_WAVE_MODEL)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_SPHERICAL_WAVE_MODEL_NOT_IMPLEMENTED;
  }
  else
  {
    for (l = 0; l < (rbs_handle->sp_speaker_pos_cart.num_pos); l++)
    {
      for (s = 0; s < HOA_NUM_VIRTMICS; s++)
      {
        imag = ia_mul_flt(
            (ia_add_flt(
                ia_mul_flt(
                    rbs_handle->sp_virt_mic_xyz.arr[s * rbs_handle->sp_virt_mic_xyz.cols + 2],
                    rbs_handle->sp_speaker_pos_cart
                        .arr[l * rbs_handle->sp_speaker_pos_cart.cols + 2]),
                ia_add_flt(
                    ia_mul_flt(
                        rbs_handle->sp_virt_mic_xyz.arr[s * rbs_handle->sp_virt_mic_xyz.cols + 1],
                        rbs_handle->sp_speaker_pos_cart
                            .arr[l * rbs_handle->sp_speaker_pos_cart.cols + 1]),
                    ia_mul_flt(
                        rbs_handle->sp_virt_mic_xyz.arr[s * rbs_handle->sp_virt_mic_xyz.cols + 0],
                        rbs_handle->sp_speaker_pos_cart
                            .arr[l * rbs_handle->sp_speaker_pos_cart.cols + 0])))),
            rbs_handle->k);

        rbs_handle->sm_hml_i.mtrx[s * rbs_handle->sm_hml_i.cols + l] = (FLOAT32)sin(imag);
        rbs_handle->sm_hml_r.mtrx[s * rbs_handle->sm_hml_r.cols + l] = (FLOAT32)cos(imag);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_render_hoa_robust_pan_init_with_spc_positions
 *
 *  \brief Initialize robust panning with space position
 *
 *  \param [in/out]  rbs_handle    HOA ronust panning handle
 *  \param [in]    spc_pos_handle  HOA Space position handle
 *  \param [in]    freq      Frequency
 *  \param [in]    mic_radius    Mic radius
 *  \param [in]    clip_src_height  Clip source height
 *  \param [in]    scratch      Pointer to scratch buffer for intermediate
 * processing
 *
 *  \return IA_ERRORCODE  Error code
 *
 */
IA_ERRORCODE ia_render_hoa_robust_pan_init_with_spc_positions(
    ia_render_hoa_robust_pan_str *rbs_handle, pVOID spc_pos_handle, FLOAT32 freq,
    FLOAT32 mic_radius, WORD32 clip_src_height, pVOID scratch)
{
  pWORD8 buf = (pWORD8)scratch;
  WORD32 scratch_idx = 0;
  IA_ERRORCODE err_code;
  ia_render_hoa_space_positions_str *spc_handle =
      (ia_render_hoa_space_positions_str *)spc_pos_handle;

  memset(&(rbs_handle->sp_virt_mic_xyz), 0, sizeof(ia_render_hoa_space_positions_str));
  memset(&(rbs_handle->sp_speaker_pos_Sp), 0, sizeof(ia_render_hoa_space_positions_str));

  impeghd_hoa_ren_space_positions_init_pos(&(rbs_handle->sp_speaker_pos_cart), spc_handle);

  rbs_handle->k = 2 * ia_mul_flt(((FLOAT32)PI), freq / 340);
  rbs_handle->radius = mic_radius;
  rbs_handle->clip_src_height = clip_src_height;

  if ((spc_handle->type) != 1)
  {
    ia_render_hoa_space_positions_str *tmpSp =
        (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
    scratch_idx += (sizeof(ia_render_hoa_space_positions_str));
    impeghd_hoa_ren_space_positions_convert_copy_to_spherical(spc_handle, tmpSp);
    rbs_handle->is_2d = impeghd_hoa_ren_space_positions_is_2d(tmpSp, (buf + scratch_idx));
    impeghd_hoa_ren_space_positions_get_max_min_inclination(
        tmpSp, &rbs_handle->max_inc, &rbs_handle->min_inc, (buf + scratch_idx));
    if (!IS_PLANE_WAVE_MODEL)
    {
      impeghd_hoa_ren_space_positions_init_pos(&(rbs_handle->sp_speaker_pos_cart), spc_handle);
    }
    else
    {
      ia_render_hoa_space_positions_str *tmp_spc =
          (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
      scratch_idx += (sizeof(ia_render_hoa_space_positions_str));
      impeghd_hoa_ren_space_positions_convert_copy_to_cartesian(spc_handle, tmp_spc, 1);
      impeghd_hoa_ren_space_positions_init_pos(&(rbs_handle->sp_speaker_pos_cart), tmp_spc);
      scratch_idx -= (sizeof(ia_render_hoa_space_positions_str));
    }
    scratch_idx -= (sizeof(ia_render_hoa_space_positions_str));
  }
  else
  {
    ia_render_hoa_space_positions_str *tmp_spc;
    rbs_handle->is_2d = impeghd_hoa_ren_space_positions_is_2d(spc_handle, (buf + scratch_idx));
    impeghd_hoa_ren_space_positions_get_max_min_inclination(
        spc_handle, &rbs_handle->max_inc, &rbs_handle->min_inc, (buf + scratch_idx));

    tmp_spc = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
    scratch_idx += (sizeof(ia_render_hoa_space_positions_str));
    impeghd_hoa_ren_space_positions_convert_copy_to_cartesian(spc_handle, tmp_spc,
                                                              IS_PLANE_WAVE_MODEL);
    impeghd_hoa_ren_space_positions_init_pos(&(rbs_handle->sp_speaker_pos_cart), tmp_spc);
    scratch_idx -= (sizeof(ia_render_hoa_space_positions_str));
  }
  err_code = ia_render_hoa_robust_pan_init(rbs_handle, (buf + scratch_idx));
  if (err_code)
  {
    return err_code;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_render_hoa_robust_pan_calculate_gains_pre
 *
 *  \brief Calculates robust panning gains
 *
 *  \param [in/out]  handle      HOA Robust panning handle
 *  \param [in]    spc_pos_handle  HOA Space position handle
 *  \param [in]    beta      Beta value
 *  \param [in]    is_norm      Normalize flag
 *  \param [in]    scratch      Pointer to scratch buffer for intermediate
 * processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE ia_render_hoa_robust_pan_calculate_gains_pre(pVOID handle, pVOID spc_pos_handle,
                                                          FLOAT32 beta, WORD32 is_norm,
                                                          pVOID scratch)
{
  pWORD8 buf = (pWORD8)scratch;
  WORD32 ll, nl, i, s, l, r, c;
  ia_render_hoa_pan_calculate_gains_pre_scratch *gains_scratch =
      (ia_render_hoa_pan_calculate_gains_pre_scratch *)(buf);
  buf += sizeof(ia_render_hoa_pan_calculate_gains_pre_scratch);
  memset(gains_scratch, 0, sizeof(ia_render_hoa_pan_calculate_gains_pre_scratch));
  ia_render_hoa_robust_pan_str *rbs_handle = (ia_render_hoa_robust_pan_str *)handle;
  ia_render_hoa_space_positions_str *spc_handle =
      (ia_render_hoa_space_positions_str *)spc_pos_handle;
  ia_render_hoa_space_positions_str *source_pos_sp = &gains_scratch->source_pos_sp;
  ia_render_hoa_space_positions_str *source_pos_ca = &gains_scratch->source_pos_ca;
  ia_render_hoa_pan_mtrx_str *c1, *c2;
  FLOAT32 cos_gamma, tmp, hms_r[HOA_NUM_VIRTMICS], hms_i[HOA_NUM_VIRTMICS];
  ia_core_coder_memset(hms_r, HOA_NUM_VIRTMICS);
  ia_core_coder_memset(hms_i, HOA_NUM_VIRTMICS);

  impeghd_hoa_ren_space_positions_convert_copy_to_spherical(spc_handle, source_pos_sp);
  if (rbs_handle->clip_src_height)
  {
    for (ll = 0; ll < source_pos_sp->num_pos; ll++)
    {
      source_pos_sp->arr[ll * source_pos_sp->cols + 1] =
          ia_max_flt(source_pos_sp->arr[ll * source_pos_sp->cols + 1], rbs_handle->min_inc);
      source_pos_sp->arr[ll * source_pos_sp->cols + 1] =
          ia_min_flt(source_pos_sp->arr[ll * source_pos_sp->cols + 1], rbs_handle->max_inc);
    }
  }
  impeghd_hoa_ren_space_positions_convert_copy_to_cartesian(source_pos_sp, source_pos_ca,
                                                            IS_PLANE_WAVE_MODEL);

  rbs_handle->sm_gain_mtrx.cols = source_pos_ca->num_pos;
  rbs_handle->sm_gain_mtrx.rows = rbs_handle->sp_speaker_pos_cart.num_pos;
  nl = (rbs_handle->sp_speaker_pos_Sp.num_pos);

  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->bb, nl, nl);
  impeghd_hoa_ren_simple_mtrx_transpose_keep(&(rbs_handle->sm_hml_r), &gains_scratch->hml_t_r);
  impeghd_hoa_ren_simple_mtrx_transpose_keep(&(rbs_handle->sm_hml_i), &gains_scratch->hml_t_i);

  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->a, nl, nl);
  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->c_mat, nl, nl);
  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->t, nl, nl);
  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->y00, nl, nl);
  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->y01, nl, nl);

  c1 = &gains_scratch->a1;
  c2 = &gains_scratch->a2;

  impeghd_hoa_ren_simple_mtrx_init_with_size(&gains_scratch->r0, nl, nl);

  for (s = 0; s < source_pos_ca->num_pos; s++)
  {
    if (!IS_PLANE_WAVE_MODEL)
    {
      return IA_MPEGH_HOA_INIT_NONFATAL_SPHERICAL_WAVE_MODEL_NOT_IMPLEMENTED;
    }
    else
    {
      WORD32 m, s_ca_x = 0;
      FLOAT32 imag;
      s_ca_x = s * source_pos_ca->cols;
      for (m = 0; m < HOA_NUM_VIRTMICS; m++)
      {
        imag = ia_mul_flt(
            (ia_add_flt(
                ia_mul_flt(
                    rbs_handle->sp_virt_mic_xyz.arr[m * rbs_handle->sp_virt_mic_xyz.cols + 2],
                    source_pos_ca->arr[s_ca_x + 2]),
                ia_add_flt(
                    ia_mul_flt(
                        rbs_handle->sp_virt_mic_xyz.arr[m * rbs_handle->sp_virt_mic_xyz.cols + 1],
                        source_pos_ca->arr[s_ca_x + 1]),
                    ia_mul_flt(
                        rbs_handle->sp_virt_mic_xyz.arr[m * rbs_handle->sp_virt_mic_xyz.cols + 0],
                        source_pos_ca->arr[s_ca_x + 0])))),
            rbs_handle->k);

        hms_i[m] = (FLOAT32)sin(imag);
        hms_r[m] = (FLOAT32)cos(imag);
      }
    }

    WORD32 s_sp_a = s * source_pos_sp->cols;
    for (l = 0; l < nl; l++)
    {
      cos_gamma = (FLOAT32)(
          (cos(source_pos_sp->arr[s_sp_a + 2] -
               rbs_handle->sp_speaker_pos_Sp.arr[l * rbs_handle->sp_speaker_pos_Sp.cols + 2]) *
           sin(rbs_handle->sp_speaker_pos_Sp.arr[l * rbs_handle->sp_speaker_pos_Sp.cols + 1]) *
           sin(source_pos_sp->arr[s_sp_a + 1])) +
          cos(rbs_handle->sp_speaker_pos_Sp.arr[l * rbs_handle->sp_speaker_pos_Sp.cols + 1]) *
              cos(source_pos_sp->arr[s_sp_a + 1]));
      tmp = ia_add_flt(0.5f, ia_mul_flt(0.5f, cos_gamma));
      tmp = 1 - ia_mul_flt(tmp, tmp);
      tmp = ia_mul_flt(tmp, tmp);
      gains_scratch->bb.mtrx[l * gains_scratch->bb.cols + l] = ia_mul_flt(tmp, beta);
    }

    gains_scratch->a1.cols = gains_scratch->a2.cols = gains_scratch->a1.rows =
        gains_scratch->a2.rows = nl;
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->hml_t_i, &(rbs_handle->sm_hml_i),
                                     &gains_scratch->a2, NULL);
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->hml_t_r, &(rbs_handle->sm_hml_r),
                                     &gains_scratch->a1, NULL);

    for (i = 0; i < (gains_scratch->a1.cols * gains_scratch->a1.rows); i++)
    {
      gains_scratch->a.mtrx[i] =
          ia_add_flt(gains_scratch->bb.mtrx[i],
                     ia_add_flt(gains_scratch->a2.mtrx[i], gains_scratch->a1.mtrx[i]));
    }
    gains_scratch->a.cols = gains_scratch->a1.cols;
    gains_scratch->a.rows = gains_scratch->a1.rows;

    c2->cols = c1->cols = c1->rows = c2->rows = nl;
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->hml_t_i, &(rbs_handle->sm_hml_r), c2, NULL);
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->hml_t_r, &(rbs_handle->sm_hml_i), c1, NULL);

    gains_scratch->c_mat.cols = c1->cols;
    gains_scratch->c_mat.rows = c1->rows;
    for (i = 0; i < nl * nl; i++)
    {
      gains_scratch->c_mat.mtrx[i] = ia_sub_flt(c1->mtrx[i], c2->mtrx[i]);
    }

    FLOAT32 temp_ta;
    WORD32 tmp_a = 0;
    for (r = 0; r < gains_scratch->hml_t_r.rows; r++)
    {
      temp_ta = 0.0;
      tmp_a = r * gains_scratch->hml_t_r.cols;
      for (c = 0; c < gains_scratch->hml_t_r.cols; c++)
      {
        (temp_ta) = ia_mac_flt((temp_ta), hms_r[c], gains_scratch->hml_t_r.mtrx[tmp_a + c]);
      }
      gains_scratch->b1[r] = temp_ta;
    }
    for (r = 0; r < gains_scratch->hml_t_i.rows; r++)
    {
      temp_ta = 0.0;
      tmp_a = r * gains_scratch->hml_t_i.cols;
      for (c = 0; c < gains_scratch->hml_t_i.cols; c++)
      {
        (temp_ta) = ia_mac_flt((temp_ta), hms_i[c], gains_scratch->hml_t_i.mtrx[tmp_a + c]);
      }
      gains_scratch->b2[r] = temp_ta;
    }

    for (i = 0; i < HOA_MAX_VECTOR_SIZE; i++)
    {
      gains_scratch->b[i] = ia_add_flt(gains_scratch->b1[i], gains_scratch->b2[i]);
    }

    for (r = 0; r < gains_scratch->hml_t_r.rows; r++)
    {
      temp_ta = 0.0;
      tmp_a = r * gains_scratch->hml_t_r.cols;
      for (c = 0; c < gains_scratch->hml_t_r.cols; c++)
      {
        (temp_ta) = ia_mac_flt((temp_ta), hms_i[c], gains_scratch->hml_t_r.mtrx[tmp_a + c]);
      }
      gains_scratch->d1[r] = temp_ta;
    }
    for (r = 0; r < gains_scratch->hml_t_i.rows; r++)
    {
      temp_ta = 0.0;
      tmp_a = r * gains_scratch->hml_t_i.cols;
      for (c = 0; c < gains_scratch->hml_t_i.cols; c++)
      {
        (temp_ta) = ia_mac_flt((temp_ta), hms_r[c], gains_scratch->hml_t_i.mtrx[tmp_a + c]);
      }
      gains_scratch->d2[r] = temp_ta;
    }
    for (i = 0; i < HOA_MAX_VECTOR_SIZE; i++)
    {
      gains_scratch->d[i] = ia_sub_flt(gains_scratch->d1[i], gains_scratch->d2[i]);
    }

    impeghd_hoa_ren_simple_mtrx_svd_init(
        &gains_scratch->svd_a, (ia_render_hoa_simple_mtrx_str *)(&gains_scratch->a), (buf));
    gains_scratch->r0temp.rows = gains_scratch->r0temp.cols = nl;

    impeghd_hoa_ren_simple_mtrx_svd_pinv(&gains_scratch->svd_a, &gains_scratch->r0temp, 1e-5f,
                                         (buf));
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->r0temp, &gains_scratch->c_mat,
                                     &gains_scratch->r0, NULL);
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->c_mat, &gains_scratch->r0,
                                     &gains_scratch->t_temp, NULL);

    gains_scratch->t.cols = gains_scratch->t_temp.cols;
    gains_scratch->t.rows = gains_scratch->t_temp.rows;
    for (i = 0; i < (gains_scratch->a.cols * gains_scratch->a.rows); i++)
    {
      gains_scratch->t.mtrx[i] =
          ia_add_flt(gains_scratch->a.mtrx[i], gains_scratch->t_temp.mtrx[i]);
    }

    impeghd_hoa_ren_simple_mtrx_svd_init(
        &gains_scratch->svd_t, (ia_render_hoa_simple_mtrx_str *)(&gains_scratch->t), (buf));
    impeghd_hoa_ren_simple_mtrx_svd_pinv(&gains_scratch->svd_t, &gains_scratch->y00, 1e-5f,
                                         (buf));
    impeghd_hoa_ren_simple_mtrx_mult(&gains_scratch->r0, &gains_scratch->y00, &gains_scratch->y01,
                                     NULL);
    for (r = 0; r < gains_scratch->y00.rows; r++)
    {
      temp_ta = 0.0;
      tmp_a = r * gains_scratch->y00.cols;
      for (c = 0; c < gains_scratch->y00.cols; c++)
      {
        (temp_ta) =
            ia_mac_flt((temp_ta), gains_scratch->b[c], gains_scratch->y00.mtrx[tmp_a + c]);
      }
      gains_scratch->g1[r] = temp_ta;
    }
    for (r = 0; r < gains_scratch->y01.rows; r++)
    {
      temp_ta = 0.0;
      tmp_a = r * gains_scratch->y01.cols;
      for (c = 0; c < gains_scratch->y01.cols; c++)
      {
        (temp_ta) =
            ia_mac_flt((temp_ta), gains_scratch->d[c], gains_scratch->y01.mtrx[tmp_a + c]);
      }
      gains_scratch->g2[r] = temp_ta;
    }

    for (i = 0; i < HOA_MAX_VECTOR_SIZE; i++)
    {
      gains_scratch->g[i] = ia_add_flt(gains_scratch->g1[i], gains_scratch->g2[i]);
    }

    if (is_norm)
    {
      FLOAT32 gn = 0.0;
      for (l = 0; l < nl; l++)
      {
        if (gains_scratch->g[l] < 1e-6)
        {
          gains_scratch->g[l] = 0.0;
        }
        else
        {
          gn = ia_mac_flt(gn, gains_scratch->g[l], gains_scratch->g[l]);
        }
      }
      gn = (FLOAT32)(1.0f / ia_sqrt_flt(gn));
      for (l = 0; l < nl; l++)
        gains_scratch->g[l] = ia_mul_flt(gains_scratch->g[l], gn);
    }

    for (l = 0; l < nl; l++)
      rbs_handle->sm_gain_mtrx.mtrx[l * rbs_handle->sm_gain_mtrx.cols + s] = gains_scratch->g[l];
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/** @} */ /* End of HOAProc */