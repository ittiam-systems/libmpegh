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
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_frame_params.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_init.h"
#include "impeghd_hoa_inverse_dyn_correction.h"
#include "impeghd_hoa_nfc_filtering.h"
#include "impeghd_hoa_pre_dom_sound_syn.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_renderer.h"
#include "impeghd_hoa_rom.h"

#include "impeghd_hoa_vector_based_predom_sound_syn.h"
#include "impeghd_hoa_amb_syn.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOADecInit HOA decoder initialization
 * @ingroup  HOADecInit
 * @brief HOA decoder initialization
 *
 * @{
 */

/**
 *  impeghd_hoa_dec_frame_param_init
 *
 *  \brief HOA decoder frame parameters initialization
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return VOID
 *
 */
VOID impeghd_hoa_dec_frame_param_init(ia_spatial_dec_str *dec_handle)
{
  ia_hoa_dec_frame_param_str *pstr_frame_params = &(dec_handle->frame_params_handle);
  UWORD32 total_perc_coders = dec_handle->ia_hoa_config->num_transport_ch;

  for (UWORD32 ch = 0; ch < MAX_HOA_CHANNELS; ch++)
  {
    ia_core_coder_memset(&pstr_frame_params->vectors[ch].sig_id[0], MAXIMUM_NUM_HOA_COEFF);
  }
  pstr_frame_params->num_enable_coeff = 0;
  pstr_frame_params->num_disable_coeff = 0;
  pstr_frame_params->num_en_dis_able_coeff = 0;

  memset(&pstr_frame_params->prediction_type_vec[0], 0,
         sizeof(pstr_frame_params->prediction_type_vec));

  memset(pstr_frame_params->prediction_indices_mat, 0,
         sizeof(pstr_frame_params->prediction_indices_mat));
  memset(pstr_frame_params->quant_pred_fact_mat, 0,
         sizeof(pstr_frame_params->quant_pred_fact_mat));

  pstr_frame_params->num_dir_predom_sounds = 0;
  pstr_frame_params->num_vec_predom_sounds = 0;

  memset(&pstr_frame_params->exponents[0], 0,
         sizeof(pstr_frame_params->exponents[0]) * total_perc_coders);
  memset(&pstr_frame_params->amb_hoa_assign[0], 0,
         sizeof(pstr_frame_params->amb_hoa_assign[0]) * total_perc_coders);
}

/**
 *  impeghd_hoa_dir_based_pre_dom_sound_syn_init
 *
 *  \brief HOA direction based pre-dominant sound synthesis initialization
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE Error code if any
 *
 */
static IA_ERRORCODE impeghd_hoa_dir_based_pre_dom_sound_syn_init(ia_spatial_dec_str *dec_handle)
{
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle.dir_based_pre_dom_sound_syn_handle);
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  IA_ERRORCODE err_code;

  syn_handle->num_prev_dir_predom_sounds = 0;
  syn_handle->num_prev_ps_indices = 0;
  ia_core_coder_memset(syn_handle->prediction_factors_mat,
                       HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF);
  memset(syn_handle->last_prediction_type, 0, sizeof(syn_handle->last_prediction_type));
  memset(syn_handle->last_prediction_indices_mat, 0,
         sizeof(syn_handle->last_prediction_indices_mat));
  ia_core_coder_memset(syn_handle->last_prediction_factors_mat,
                       HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF);
  ia_core_coder_memset(syn_handle->pred_grid_dir_sigs_smthed,
                       MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE);
  ia_core_coder_memset(syn_handle->pred_grid_dir_sigs_last,
                       MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE);

  err_code = impeghd_hoa_table_get_mode_mat(syn_handle->mode_mt_all_coarse_grid_points,
                                            pstr_hoa_config->num_coeffs, pstr_hoa_config->order);
  if (err_code)
  {
    return err_code;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_vector_based_predom_sound_syn_init
 *
 *  \brief HOA vector based pre-dominant sound synthesis initialization
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE        Error
 *
 */
static IA_ERRORCODE impeghd_hoa_vector_based_predom_sound_syn_init(ia_spatial_dec_str *dec_handle)
{
  ia_spatial_dec_vector_based_predom_sound_syn_str *syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle.vec_based_pre_dom_sound_syn_handle);
  UWORD32 frame, sig_id;

  for (frame = 0; frame < (UWORD32)(2 * dec_handle->ia_hoa_config->frame_length); frame++)
  {
    dec_handle->ones_long_vec[frame] = (FLOAT32)(1.0);
  }

  syn_handle->num_prev_vec = 0;

  for (sig_id = 0; sig_id < HOA_MAXIMUM_VECTOR_SIG_INDICES; sig_id++)
  {
    syn_handle->prev_ps_sig_indices_set[sig_id] = (UWORD32)-1;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_pre_dom_sound_syn_init
 *
 *  \brief HOA pre-dominant sound synthesis initialization
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE        Error code if any
 *
 */
IA_ERRORCODE impeghd_hoa_pre_dom_sound_syn_init(ia_spatial_dec_str *dec_handle)
{
  ia_spatial_dec_pre_dom_sound_syn_str *syn_handle = &(dec_handle->pre_dom_sound_syn_handle);
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  pFLOAT32 ptr_cross_fade_win_for_dir_sigs = NULL, ptr_cross_fade_win_for_vec_sigs;
  UWORD32 frame;
  UWORD32 interp_method = pstr_hoa_config->spat_interpolation_method;
  UWORD32 interp_samples = pstr_hoa_config->spat_interpolation_time;
  UWORD32 frame_sz = pstr_hoa_config->frame_length;
  IA_ERRORCODE err_code;

  ptr_cross_fade_win_for_dir_sigs =
      (FLOAT32 *)&ia_hoa_cos_table_compute_fade_win_for_dir_based_syn[0];
  err_code = impeghd_hoa_compute_fade_win_for_vec_based_syn(&ptr_cross_fade_win_for_vec_sigs,
                                                            interp_samples, interp_method);
  if (err_code)
  {
    return err_code;
  }

  for (frame = 0; frame < frame_sz; frame++)
  {
    syn_handle->fade_out_win_for_vec_sigs[frame] =
        ptr_cross_fade_win_for_vec_sigs[frame + frame_sz];
    syn_handle->fade_in_win_for_vec_sigs[frame] = ptr_cross_fade_win_for_vec_sigs[frame];
    syn_handle->fade_out_win_for_dir_sigs[frame] =
        ptr_cross_fade_win_for_dir_sigs[frame + frame_sz];
    syn_handle->fade_in_win_for_dir_sigs[frame] = ptr_cross_fade_win_for_dir_sigs[frame];
  }
  err_code = impeghd_hoa_dir_based_pre_dom_sound_syn_init(dec_handle);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_vector_based_predom_sound_syn_init(dec_handle);
  if (err_code)
  {
    return err_code;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ambience_synthesis_init
 *
 *  \brief HOA ambience synthesis initialization
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE             Error
 *
 */
IA_ERRORCODE impeghd_hoa_ambience_synthesis_init(ia_spatial_dec_str *dec_handle)
{
  ia_spatial_dec_amb_syn_str *syn_handle = &(dec_handle->amb_syn_handle);
  WORD32 order = dec_handle->ia_hoa_config->min_amb_order;
  UWORD32 mat_dim;
  pFLOAT32 ptr_mode_mat;

  mat_dim = (order + 1) * (order + 1);
  syn_handle->low_order_mat_sz = mat_dim;
  ptr_mode_mat = syn_handle->order_matrix;

  ia_core_coder_memset(ptr_mode_mat, mat_dim * mat_dim);

  if (order == 0)
  {
    *(syn_handle->order_matrix) = (1.0);
  }
  else if (order > 0)
  {
    UWORD32 dir_idx;
    WORD32 order_idx, degree_idx, row_idx;
    FLOAT32 tmp_azim;
    FLOAT32 cos_tmp_inclin, sin_tmp_inclin;
    pFLOAT32 ptr_tmp_azim = (pFLOAT32)(ia_hoa_azimuths[order - 1]);
    pFLOAT32 ptr_cos_tmp_inclin = (pFLOAT32)(ia_hoa_cos_inclinations[order - 1]);
    pFLOAT32 ptr_sin_tmp_inclin = (pFLOAT32)(ia_hoa_sin_inclinations[order - 1]);
    for (dir_idx = 0; dir_idx < mat_dim; dir_idx++)
    {
      tmp_azim = *ptr_tmp_azim;
      cos_tmp_inclin = *ptr_cos_tmp_inclin;
      sin_tmp_inclin = *ptr_sin_tmp_inclin;
      for (order_idx = 0; order_idx <= order; order_idx++)
      {
        for (degree_idx = -order_idx; degree_idx <= order_idx; degree_idx++)
        {
          row_idx = order_idx * (order_idx + 1) + degree_idx;
          ptr_mode_mat = (syn_handle->order_matrix) + mat_dim * row_idx + dir_idx;
          *ptr_mode_mat = (FLOAT32)(impeghd_hoa_spherical_harmonics(
              order_idx, degree_idx, cos_tmp_inclin, sin_tmp_inclin, tmp_azim));
        }
      }
      ptr_tmp_azim++;
      ptr_cos_tmp_inclin++;
      ptr_sin_tmp_inclin++;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_table_get_dict_cicp_speaker_points
 *
 *  \brief Get HOA table CICP speaker points
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impeghd_hoa_table_get_dict_cicp_speaker_points(ia_spatial_dec_str *dec_handle)
{
  pFLOAT32 mode_mat = dec_handle->dict_cicp_speaker_points;
  UWORD32 order = dec_handle->ia_hoa_config->order;
  pFLOAT32 ptr_cos_inclin, ptr_sin_inclin;
  FLOAT32 tmp_azim;
  pUWORD32 ptr_tmp_azim;
  pFLOAT32 ptr_mode_mat;
  WORD32 order_idx, degree_idx;
  UWORD32 num_dirs, dir_idx;

  num_dirs = 34;
  ptr_sin_inclin = (pFLOAT32)(ia_hoa_sin_cicp_speaker_inclinations);
  ptr_cos_inclin = (pFLOAT32)(ia_hoa_cos_cicp_speaker_inclinations);
  ptr_tmp_azim = (pUWORD32)(ia_hoa_cicp_speakerd_azimuths);

  for (dir_idx = 0; dir_idx < num_dirs; dir_idx++)
  {
    ptr_mode_mat = mode_mat + dir_idx * HOA_MAXIMUM_MATRIX_SIZE;
    tmp_azim = *ptr_tmp_azim * (HOA__M_PI__ / 180);
    for (order_idx = 0; order_idx <= (WORD32)(order); order_idx++)
    {
      for (degree_idx = -order_idx; degree_idx <= order_idx; degree_idx++)
      {
        *ptr_mode_mat++ = (FLOAT32)(impeghd_hoa_spherical_harmonics(
            order_idx, degree_idx, *ptr_cos_inclin, *ptr_sin_inclin, tmp_azim));
      }
    }
    ptr_sin_inclin++;
    ptr_cos_inclin++;
    ptr_tmp_azim++;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_table_get_dict_2d_points
 *
 *  \brief Get HOA table 2D points
 *
 *  \param [in/out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE              Error
 *
 */
IA_ERRORCODE impeghd_hoa_table_get_dict_2d_points(ia_spatial_dec_str *dec_handle)
{
  WORD32 order_idx, degree_idx;
  pFLOAT32 ptr_mode_mat;
  pFLOAT32 mode_mat = dec_handle->dict_2d_points;
  UWORD32 order = dec_handle->ia_hoa_config->order;
  UWORD32 num_dirs, dir_idx;
  FLOAT32 step_sz, offset, tmp_azim;

  num_dirs = 64;

  step_sz = 2 * HOA__M_PI__ / num_dirs;
  offset = ia_mul_flt(step_sz, 0.745f);
  for (dir_idx = 0; dir_idx < num_dirs; dir_idx++)
  {
    ptr_mode_mat = mode_mat + dir_idx * HOA_MAXIMUM_MATRIX_SIZE;
    tmp_azim = step_sz * dir_idx + offset;
    for (order_idx = 0; order_idx <= (WORD32)(order); order_idx++)
    {
      for (degree_idx = -order_idx; degree_idx <= order_idx; degree_idx++)
      {
        *ptr_mode_mat++ = (FLOAT32)(impeghd_hoa_spherical_harmonics(
            order_idx, degree_idx, (FLOAT32)(HOA_COSINE_OF_NINETY_DEGREES),
            (FLOAT32)(HOA_SINE_OF_NINETY_DEGREES), tmp_azim));
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_renderer_init
 *
 *  \brief HOA renderer initialization
 *
 *  \param [in/out] rh_handle       Pointer to HOA renderer structure
 *  \param [in]     spk_id          Speaker index
 *  \param [in]     hoa_orders      HOA orders
 *  \param [in]     num_hoa_orders  Number of HOA orders
 *  \param [in]     is_lfe_enabled  low frequency element enable flag
 *  \param [in]     scratch         Pointer to scratch buffer
 *
 *  \return IA_ERRORCODE                  Error
 *
 */
IA_ERRORCODE impeghd_hoa_ren_renderer_init(ia_render_hoa_str *rh_handle, WORD32 spk_id,
                                           ia_speaker_config_3d *ref_spk_layout,
                                           pWORD32 hoa_orders, WORD32 num_hoa_orders,
                                           WORD32 is_lfe_enabled, pVOID scratch)
{

  WORD32 *ls_types;
  ia_render_hoa_space_positions_str *lfe_pos, *speaker_pos;
  WORD32 ls_types_size = 0, n_lfe = 0, scratch_idx = 0, order_idx;
  pWORD8 buf;

  rh_handle->spk_idx = spk_id;
  rh_handle->num_out_channels = 0;
  rh_handle->mtrx_selected = -1;
  rh_handle->scratch_idx = &scratch_idx;
  rh_handle->scratch = scratch;
  buf = (pWORD8)(rh_handle->scratch);
  if (spk_id > CICP2GEOMETRY_MAX_SPKIDX)
    return IA_MPEGH_HOA_INIT_FATAL_RENDER_INIT_FAILED;
  speaker_pos = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_space_positions_str);
  if (impeghd_hoa_ren_space_positions_init_with_param(speaker_pos, rh_handle->spk_idx,
                                                      ref_spk_layout, 0))
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
  }
  lfe_pos = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_space_positions_str);
  if (impeghd_hoa_ren_space_positions_init_with_param(lfe_pos, rh_handle->spk_idx, ref_spk_layout,
                                                      1))
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
  }

  ls_types = (WORD32 *)(buf + scratch_idx);
  scratch_idx += sizeof(WORD32) * (HOA_MAX_ARR_SIZE);
  ls_types_size = impeghd_hoa_ren_get_loudspeaker_type(rh_handle, ls_types);
  if (is_lfe_enabled)
    n_lfe = lfe_pos->num_pos;

  rh_handle->render_mtrx = num_hoa_orders;
  rh_handle->use_nfc = 0;
  rh_handle->ldspk_dist = (FLOAT32)impeghd_hoa_ren_space_positions_get_max_distance(speaker_pos);

  for (order_idx = 0; order_idx < num_hoa_orders; order_idx++)
  {
    if (impeghd_hoa_ren_mtrx_init_with_spc_pos_param(
            &(rh_handle->v_render_mtrx[order_idx]), spk_id, ref_spk_layout, hoa_orders[order_idx],
            is_lfe_enabled, (buf + scratch_idx)))
    {
      return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;
    }
  }
  if (is_lfe_enabled)
  {
    if (impeghd_hoa_ren_permute_lfe_channel(rh_handle, speaker_pos, ls_types, ls_types_size))
    {
      return IA_MPEGH_HOA_INIT_NONFATAL_RENDERER_INIT_FAILED;
    }
  }

  rh_handle->spk_idx = spk_id;
  rh_handle->num_out_channels = n_lfe + speaker_pos->num_pos;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_ren_input_init
 *
 *  \brief HOA renderer input initialization
 *
 *  \param [in/out] handle              Pointer to HOA renderer structure
 *  \param [in]     order               HOA order
 *  \param [in]     config_handle       Pointer to HOA config structure
 *  \param [in]     hoa_mtx_file_cnt    Matrix present flag
 *  \param [in]     f_nfc_radius        Near Field Compensation (NFC) radius
 *  \param [in]     sampling_rate       Sample rate
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghd_hoa_ren_input_init(pVOID handle, ia_speaker_config_3d *ref_spk_layout,
                                        WORD32 order, pVOID config_handle,
                                        WORD32 hoa_mtx_file_cnt, FLOAT32 f_nfc_radius,
                                        WORD32 sampling_rate, UWORD32 mpegh_profile_lvl)
{
  IA_ERRORCODE err_code;
  WORD32 scratch_idx = 0, idx, order_idx;
  ia_render_hoa_str *rh_handle = (ia_render_hoa_str *)handle;
  pWORD8 buf = (pWORD8)(rh_handle->scratch);

  rh_handle->scratch_idx = &scratch_idx;
  rh_handle->preliminary_mtrx_selected = -1;

  if (f_nfc_radius <= rh_handle->ldspk_dist)
  {
    rh_handle->use_nfc = 0;
  }
  else
  {
    /*The	near	field	compensation	(NFC)	processing	may	be	applied	to HOA content of an
    order
    which	is	smaller	or	equal	to 2 for LC lvl 3, 1 for LC lvl 2, not allowed for LC
    lvl 1*/
    if (((mpegh_profile_lvl == MPEGH_PROFILE_LC_LVL_3) && (order <= 2)) ||
        ((mpegh_profile_lvl == MPEGH_PROFILE_LC_LVL_2) && (order <= 1)) ||
        ((mpegh_profile_lvl == MPEGH_PROFILE_BP_LVL_3) ||
         (mpegh_profile_lvl == MPEGH_PROFILE_BP_LVL_2) ||
         (mpegh_profile_lvl == MPEGH_PROFILE_BP_LVL_1)))
    {
      rh_handle->use_nfc = 1;
      err_code = impeghd_hoa_ren_nfc_init(rh_handle, order, f_nfc_radius, sampling_rate);
      if (err_code)
      {
        return err_code;
      }
    }
    else
    {
      return IA_MPEGH_HOA_INIT_FATAL_NFC_NOT_ALLOWED;
    }
  }

  idx = rh_handle->render_mtrx;

  if (hoa_mtx_file_cnt)
  {
    ia_render_hoa_space_positions_str *speaker_pos, *lfe_pos;
    WORD32 *ls_types;
    WORD32 num_signaled_matrices, ls_types_size, is_matching_mtx, n;
    lfe_pos = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
    scratch_idx += sizeof(ia_render_hoa_space_positions_str);
    if (impeghd_hoa_ren_space_positions_init_with_param(lfe_pos, rh_handle->spk_idx,
                                                        ref_spk_layout, 1))
    {
      return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
    }
    speaker_pos = (ia_render_hoa_space_positions_str *)(buf + scratch_idx);
    scratch_idx += sizeof(ia_render_hoa_space_positions_str);
    if (impeghd_hoa_ren_space_positions_init_with_param(speaker_pos, rh_handle->spk_idx,
                                                        ref_spk_layout, 0))
    {
      return IA_MPEGH_HOA_INIT_NONFATAL_SPACE_POSITION_INIT_FAILED;
    }

    ls_types = (WORD32 *)(buf + scratch_idx);
    scratch_idx += (sizeof(WORD32) * HOA_MAX_ARR_SIZE);
    ls_types_size = impeghd_hoa_ren_get_loudspeaker_type(rh_handle, ls_types);
    num_signaled_matrices = hoa_mtx_file_cnt;

    for (n = 0; n < num_signaled_matrices; ++n)
    {
      if (impeghd_hoa_ren_mtrx_init_with_mtrx_param(&(rh_handle->v_render_mtrx[idx]),
                                                    config_handle))
      {
        return IA_MPEGH_HOA_INIT_FATAL_RENDER_MATRIX_INIT_FAILED;
      }
      is_matching_mtx = impeghd_hoa_ren_permute_signaled_rendering_matrix(
          rh_handle, speaker_pos, lfe_pos, ls_types, ls_types_size);
      if (is_matching_mtx)
      {
        is_matching_mtx = impeghd_hoa_ren_validate_signaled_rendering_matrix(
            rh_handle, speaker_pos, ls_types, ls_types_size, idx);
      }
      if (is_matching_mtx)
      {
        rh_handle->preliminary_mtrx_selected = idx;
        break;
      }
    }
  }

  if (-1 == rh_handle->preliminary_mtrx_selected)
  {
    for (order_idx = 0; order_idx < (WORD32)idx; order_idx++)
    {
      if ((rh_handle->v_render_mtrx[order_idx].order) == order)
      {
        rh_handle->preliminary_mtrx_selected = order_idx;
        break;
      }
    }
  }

  if (-1 != rh_handle->preliminary_mtrx_selected)
  {
    rh_handle->mtrx_selected = rh_handle->preliminary_mtrx_selected;
  }
  return (IA_MPEGH_DEC_NO_ERROR);
}
/** @} */ /* End of HOADecInit */