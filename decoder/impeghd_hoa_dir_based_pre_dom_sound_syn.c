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
#include <math.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_rom.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_dir_based_pre_dom_sound_syn.h"
#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_table_get_matrix
 *
 *  \brief Get HOA table mode matrix
 *
 *  \param [out] ptr_mode_mat   Pointer to mode matrix
 *  \param [in]  row_max_sz     Maximum matrix size
 *  \param [in]  order          HOA order
 *	\param [in]  mat_dim        Matrix Dimension
 *
 */
static VOID impeghd_hoa_table_get_matrix(pFLOAT32 ptr_mode_mat, UWORD32 row_max_sz, WORD32 order,
                                         UWORD32 mat_dim)
{
  UWORD32 dir_idx;
  WORD32 order_idx, degree_idx, row_idx;
  pFLOAT32 matrix;
  pFLOAT32 ptr_tmp_azim = (pFLOAT32)(ia_hoa_azimuths[order - 1]);
  pFLOAT32 ptr_cos_tmp_inclin = (pFLOAT32)(ia_hoa_cos_inclinations[order - 1]);
  pFLOAT32 ptr_sin_tmp_inclin = (pFLOAT32)(ia_hoa_sin_inclinations[order - 1]);
  for (dir_idx = 0; dir_idx < mat_dim; dir_idx++)
  {
    matrix = ptr_mode_mat;
    for (order_idx = 0; order_idx <= order; order_idx++)
    {
      for (degree_idx = -order_idx; degree_idx <= order_idx; degree_idx++)
      {
        row_idx = order_idx * (order_idx + 1) + degree_idx;

        *(matrix + (row_idx * row_max_sz + dir_idx) /*sizeof(FLOAT64  )*/) =
            impeghd_hoa_spherical_harmonics(order_idx, degree_idx, *ptr_cos_tmp_inclin,
                                            *ptr_sin_tmp_inclin, *ptr_tmp_azim);
      }
    }
    ptr_sin_tmp_inclin++;
    ptr_cos_tmp_inclin++;
    ptr_tmp_azim++;
  }
}
/**
 *  impeghd_hoa_table_get_mode_mat
 *
 *  \brief Get HOA table mode matrix
 *
 *  \param [out] ptr_mode_mat   Pointer to mode matrix
 *  \param [in]  row_max_sz     Maximum matrix size
 *  \param [in]  order          HOA order
 *
 *  \return IA_ERRORCODE              Error
 *
 */
IA_ERRORCODE impeghd_hoa_table_get_mode_mat(pFLOAT32 ptr_mode_mat, UWORD32 row_max_sz,
                                            WORD32 order)
{
  UWORD32 mat_dim;
  mat_dim = (order + 1) * (order + 1);

  if (order != 0)
  {
    impeghd_hoa_table_get_matrix(ptr_mode_mat, row_max_sz, order, mat_dim);
  }
  else
  {
    ptr_mode_mat[0] = (FLOAT32)(1.0);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_compute_hoa_repr_of_dir_sigs
 *
 *  \brief Computation of HOA representation of direction signals
 *
 *  \param [in,out] dec_handle      Pointer to spatial decoder handle
 *  \param [in]     sig_indices_set Pointer to indices set
 *
 *  \return IA_ERRORCODE                  Error
 *
 */
IA_ERRORCODE impeghd_hoa_compute_hoa_repr_of_dir_sigs(ia_spatial_dec_str *dec_handle,
                                                      pUWORD32 sig_indices_set)
{
  const pFLOAT32 ptr_in_sigs_frame = dec_handle->dyn_corr_sample_buf;
  pFLOAT32 out_smth_dir_sigs = dec_handle->dir_sigs_hoa_frame_buf;
  ia_hoa_dec_frame_param_str *frame_param = &(dec_handle->frame_params_handle);
  const ia_hoa_dir_id_str *ptr_active_dir_and_grid_indices =
      frame_param->active_and_grid_dir_indices;
  const pUWORD32 ptr_set_of_sig_indices = sig_indices_set;
  ia_spatial_dec_pre_dom_sound_syn_str *sound_syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *syn_handle =
      &(sound_syn_handle->dir_based_pre_dom_sound_syn_handle);
  const pFLOAT32 fade_in_win_for_dir_sigs = sound_syn_handle->fade_in_win_for_dir_sigs;
  const pFLOAT32 fade_out_win_for_dir_sigs = sound_syn_handle->fade_out_win_for_dir_sigs;
  const pFLOAT32 fade_out_win_for_vec_sigs = sound_syn_handle->fade_out_win_for_vec_sigs;
  UWORD32 num_dir, curr_dir_idx, curr_grid_dir_idx;
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  ia_core_coder_memset(out_smth_dir_sigs,
                       pstr_hoa_config->frame_length * pstr_hoa_config->num_coeffs);

  for (num_dir = 0; num_dir < syn_handle->num_prev_dir_predom_sounds; num_dir++)
  {
    curr_grid_dir_idx = syn_handle->last_active_dir_and_grid_dir_indices[num_dir].dir_id;
    curr_dir_idx = syn_handle->last_active_dir_and_grid_dir_indices[num_dir].ch_idx;
    if (curr_grid_dir_idx != 0)
    {
      FLOAT32 curr_hoa_contribution;
      pFLOAT32 ptr_in_ps_sigs_frame, ptr_out_smth_dir_sigs;
      pFLOAT32 ptr_transpose_mode_mat_all_fine_grid_points;
      pFLOAT32 ptr_curr_fade_out_win;
      UWORD32 dir_idx, coeff_idx, sample_idx;

      for (dir_idx = 0; dir_idx < frame_param->num_dir_predom_sounds; dir_idx++)
      {
        if (ptr_active_dir_and_grid_indices[dir_idx].ch_idx == (WORD32)curr_dir_idx)
          break;
      }
      ptr_curr_fade_out_win = &(dec_handle->ones_long_vec[0]);

      if (HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED == dir_idx)
      {
        for (dir_idx = 0; dir_idx < HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED; dir_idx++)
        {
          if (ptr_set_of_sig_indices[dir_idx] == curr_dir_idx)
            break;
        }

        if (dir_idx != HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED)
        {
          ptr_curr_fade_out_win = &(fade_out_win_for_vec_sigs[0]);
        }
      }
      else if (ptr_active_dir_and_grid_indices[dir_idx].dir_id != 0)
      {
        ptr_curr_fade_out_win = &(fade_out_win_for_dir_sigs[0]);
      }
      ptr_out_smth_dir_sigs = out_smth_dir_sigs;
      for (coeff_idx = 0; coeff_idx < pstr_hoa_config->num_coeffs; coeff_idx++)
      {
        ptr_transpose_mode_mat_all_fine_grid_points =
            dec_handle->transpose_mod_mtx_grid_pts + coeff_idx +
            (curr_grid_dir_idx - 1) * HOA_MAXIMUM_MATRIX_SIZE; /* pstr_hoa_config->num_coeffs*/

        ptr_in_ps_sigs_frame =
            (pFLOAT32)ptr_in_sigs_frame + (curr_dir_idx - 1) * pstr_hoa_config->frame_length;
        for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
        {
          curr_hoa_contribution =
              (*ptr_in_ps_sigs_frame++) * (*ptr_transpose_mode_mat_all_fine_grid_points);

          curr_hoa_contribution *= ptr_curr_fade_out_win[sample_idx];

          *ptr_out_smth_dir_sigs += curr_hoa_contribution;
          ptr_out_smth_dir_sigs++;
        }
      }
    }
  }
  for (num_dir = 0; num_dir < frame_param->num_dir_predom_sounds; num_dir++)
  {
    curr_grid_dir_idx = ptr_active_dir_and_grid_indices[num_dir].dir_id;
    curr_dir_idx = ptr_active_dir_and_grid_indices[num_dir].ch_idx;

    if (curr_grid_dir_idx != 0)
    {
      FLOAT32 curr_hoa_contribution;
      pFLOAT32 ptr_input_ps_sigs_frame, ptr_output_smth_dir_sigs;
      pFLOAT32 ptr_transpose_mode_mat_all_fine_grid_points;
      UWORD32 dir_idx, coeff_idx, sample_idx;
      pFLOAT32 curr_fade_in_win = &(dec_handle->ones_long_vec[0]);

      for (dir_idx = 0; dir_idx < syn_handle->num_prev_dir_predom_sounds; dir_idx++)
      {
        if ((WORD32)curr_dir_idx ==
            syn_handle->last_active_dir_and_grid_dir_indices[dir_idx].ch_idx)
          break;
      }

      if (HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED == dir_idx)
      {
        for (dir_idx = 0; dir_idx < syn_handle->num_prev_ps_indices; dir_idx++)
        {
          if (curr_dir_idx == syn_handle->set_of_old_ps_sig_indices[dir_idx])
            break;
        }
        if (dir_idx != HOA_MAXIMUM_NUM_DIR_SIG_FOR_PRED)
        {
          curr_fade_in_win = &(fade_in_win_for_dir_sigs[0]);
        }
      }
      else if (syn_handle->last_active_dir_and_grid_dir_indices[dir_idx].dir_id != 0)
      {
        curr_fade_in_win = &(fade_in_win_for_dir_sigs[0]);
      }

      ptr_output_smth_dir_sigs = out_smth_dir_sigs;
      for (coeff_idx = 0; coeff_idx < pstr_hoa_config->num_coeffs; coeff_idx++)
      {
        ptr_transpose_mode_mat_all_fine_grid_points =
            (curr_grid_dir_idx - 1) * HOA_MAXIMUM_MATRIX_SIZE + /* pstr_hoa_config->num_coeffs */
            dec_handle->transpose_mod_mtx_grid_pts + coeff_idx;
        ptr_input_ps_sigs_frame =
            (pFLOAT32)ptr_in_sigs_frame + (curr_dir_idx - 1) * pstr_hoa_config->frame_length;
        for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
        {
          curr_hoa_contribution =
              (*ptr_input_ps_sigs_frame++) * *(ptr_transpose_mode_mat_all_fine_grid_points);
          curr_hoa_contribution *= curr_fade_in_win[sample_idx];
          *ptr_output_smth_dir_sigs += curr_hoa_contribution;
          ptr_output_smth_dir_sigs++;
        }
      }
    }
  }

  syn_handle->num_prev_ps_indices = frame_param->num_vec_predom_sounds;
  memcpy(&syn_handle->set_of_old_ps_sig_indices[0], ptr_set_of_sig_indices,
         sizeof(*ptr_set_of_sig_indices) * frame_param->num_vec_predom_sounds);

  syn_handle->num_prev_dir_predom_sounds = frame_param->num_dir_predom_sounds;
  memcpy(&syn_handle->last_active_dir_and_grid_dir_indices[0], ptr_active_dir_and_grid_indices,
         sizeof(*ptr_active_dir_and_grid_indices) * frame_param->num_dir_predom_sounds);
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_dir_based_pre_dom_sound_syn_process
 *
 *  \brief HOA direction based pre-dominant sound synthesis processing
 *
 *  \param [in]  dec_handle                             Pointer to spatial decoder handle
 *  \param [in]  sig_indices_set                        Pointer to indices set
 *  \param [out] out_dir_based_pre_dom_sounds           Direction based pre-dom sounds frame ptr
 *
 *  \return IA_ERRORCODE                                      Error
 *
 */
IA_ERRORCODE
impeghd_hoa_dir_based_pre_dom_sound_syn_process(ia_spatial_dec_str *dec_handle,
                                                pUWORD32 sig_indices_set,
                                                pFLOAT32 out_dir_based_pre_dom_sounds)
{
  const pUWORD32 set_of_sig_indices = sig_indices_set;
  pFLOAT32 out_smth_dir_sigs = dec_handle->dir_sigs_hoa_frame_buf;
  ia_spatial_dec_pre_dom_sound_syn_str *sound_syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle);
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *syn_handle =
      &(sound_syn_handle->dir_based_pre_dom_sound_syn_handle);

  UWORD32 frame, row_idx;
  IA_ERRORCODE err_code;
  pFLOAT32 ptr_in_amb_hoa_coeffs, ptr_out_smth_dir_sigs, ptr_out_dir_based_pre_dom_sounds,
      ptr_pred_grid_dir_sigs_smth;

  err_code = impeghd_hoa_compute_hoa_repr_of_dir_sigs(dec_handle, set_of_sig_indices);
  if (err_code)
  {
    return err_code;
  }

  err_code = impeghd_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs(dec_handle);
  if (err_code)
  {
    return err_code;
  }
  ptr_in_amb_hoa_coeffs = dec_handle->amb_hoa_coeffs_frame_buf;
  ptr_pred_grid_dir_sigs_smth = syn_handle->pred_grid_dir_sigs_smthed;
  ptr_out_smth_dir_sigs = out_smth_dir_sigs;
  ptr_out_dir_based_pre_dom_sounds = out_dir_based_pre_dom_sounds;

  for (row_idx = 0; row_idx < pstr_hoa_config->num_coeffs; row_idx++)
  {
    WORD32 k = row_idx * pstr_hoa_config->frame_length;
    for (frame = 0; frame < pstr_hoa_config->frame_length; frame++)
    {
      ptr_out_dir_based_pre_dom_sounds[k + frame] +=
          (ptr_in_amb_hoa_coeffs[k + frame] + ptr_pred_grid_dir_sigs_smth[k + frame] +
           ptr_out_smth_dir_sigs[k + frame]);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs
 *
 *  \brief Computation of smoothed HOA representation of spatial
 *         prediction signals
 *
 *  \param [in,out] dec_handle  Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE              Error
 *
 */
IA_ERRORCODE
impeghd_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs(ia_spatial_dec_str *dec_handle)
{
  ia_hoa_dec_frame_param_str *frame_param = &(dec_handle->frame_params_handle);
  const pUWORD32 ptr_non_en_dis_able_acrt_hoa_coeff_indices =
      frame_param->non_en_dis_able_act_hoa_coeff_indices;
  const pWORD32 quant_pred_fact_mat =
      frame_param->quant_pred_fact_mat; /* HOA_MAX_PRED_DIR_SIGNALS * MAXIMUM_NUM_HOA_COEFF */
  const pUWORD32 prediction_indices_mat = frame_param->prediction_indices_mat;
  const pUWORD32 prediction_type_vec = frame_param->prediction_type_vec;

  ia_spatial_dec_pre_dom_sound_syn_str *sound_syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *syn_handle =
      &(sound_syn_handle->dir_based_pre_dom_sound_syn_handle);
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  const pFLOAT32 fade_in_win_for_dir_sigs = sound_syn_handle->fade_in_win_for_dir_sigs;
  const pFLOAT32 fade_out_win_for_dir_sigs = sound_syn_handle->fade_out_win_for_dir_sigs;
  pFLOAT32 ptr_last_prediction_factors_mat, ptr_out_pred_grid_dir_sigs_smth,
      ptr_pred_grid_dir_sigs_last;

  WORD32 is_spatial_prediction_in_current_frame = 0, is_spatial_prediction_in_last_frame = 0;
  IA_ERRORCODE err_code;
  UWORD32 grid_dir_idx, coeff_idx, coeff, row_idx, sample_idx;
  FLOAT32 dequantized_factor;

  FLOAT32 scale_factor = impeghd_hoa_pow_2(1 - pstr_hoa_config->num_bits_per_sf);
  pFLOAT32 ptr_prediction_factors_mat = syn_handle->prediction_factors_mat;
  pWORD32 ptr_quantized_prediction_factors_mat = quant_pred_fact_mat;
  pUWORD32 ptr_last_prediction_indices_mat, ptr_prediction_indices_mat;

  for (row_idx = 0; row_idx < pstr_hoa_config->pred_dir_sigs; row_idx++)
  {
    for (grid_dir_idx = 0; grid_dir_idx < pstr_hoa_config->num_coeffs; grid_dir_idx++)
    {
      dequantized_factor =
          (FLOAT32)(scale_factor * ((FLOAT32)(*ptr_quantized_prediction_factors_mat++) + 0.5));
      *ptr_prediction_factors_mat++ = dequantized_factor;
    }
  }

  if (pstr_hoa_config->num_addnl_coders != 0)
  {
    err_code = impeghd_hoa_compute_hoa_representation_of_spat_pred(
        dec_handle, syn_handle->last_prediction_type, syn_handle->last_prediction_indices_mat,
        syn_handle->last_prediction_factors_mat, fade_out_win_for_dir_sigs,
        syn_handle->pred_grid_dir_sigs_last, &is_spatial_prediction_in_last_frame);

    if (err_code)
    {
      return err_code;
    }

    err_code = impeghd_hoa_compute_hoa_representation_of_spat_pred(
        dec_handle, prediction_type_vec, prediction_indices_mat,
        syn_handle->prediction_factors_mat, fade_in_win_for_dir_sigs,
        syn_handle->pred_grid_dir_sigs_smthed, &is_spatial_prediction_in_current_frame);

    if (err_code)
    {
      return err_code;
    }
  }
  else
  {
    ia_core_coder_memset(syn_handle->pred_grid_dir_sigs_smthed,
                         MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE);
    ia_core_coder_memset(syn_handle->pred_grid_dir_sigs_last,
                         MAXIMUM_NUM_HOA_COEFF * MAXIMUM_FRAME_SIZE);
  }

  for (coeff_idx = 0; coeff_idx < pstr_hoa_config->num_coeffs; coeff_idx++)
  {
    ptr_out_pred_grid_dir_sigs_smth =
        &syn_handle->pred_grid_dir_sigs_smthed[0] + coeff_idx * pstr_hoa_config->frame_length;
    ptr_pred_grid_dir_sigs_last =
        &syn_handle->pred_grid_dir_sigs_last[0] + coeff_idx * pstr_hoa_config->frame_length;
    for (coeff = 0; coeff < MAXIMUM_NUM_HOA_COEFF; coeff++)
    {
      if (ptr_non_en_dis_able_acrt_hoa_coeff_indices[coeff] == (coeff_idx + 1))
        break;
    }

    if (MAXIMUM_NUM_HOA_COEFF != coeff)
    {
      ia_core_coder_memset(ptr_out_pred_grid_dir_sigs_smth, pstr_hoa_config->frame_length);
    }
    else
    {
      for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
      {
        *ptr_out_pred_grid_dir_sigs_smth++ += (*ptr_pred_grid_dir_sigs_last++);
      }
    }
  }
  if (is_spatial_prediction_in_current_frame)
  {
    const pUWORD32 ptr_amb_coeff_indices_to_be_enabled = frame_param->amb_coeff_indices_to_enable;
    UWORD32 coeff_idx_for_pred_hoa_to_be_faded_out;
    for (coeff = 0; coeff < frame_param->num_enable_coeff; coeff++)
    {
      coeff_idx_for_pred_hoa_to_be_faded_out = ptr_amb_coeff_indices_to_be_enabled[coeff] - 1;
      ptr_out_pred_grid_dir_sigs_smth =
          &syn_handle->pred_grid_dir_sigs_smthed[0] +
          coeff_idx_for_pred_hoa_to_be_faded_out * pstr_hoa_config->frame_length;
      for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
      {
        *ptr_out_pred_grid_dir_sigs_smth++ *= fade_out_win_for_dir_sigs[sample_idx];
      }
    }
  }
  if (is_spatial_prediction_in_last_frame)
  {
    const pUWORD32 ptr_amb_coeff_indices_to_be_disabled =
        frame_param->amb_coeff_indices_to_disable;
    UWORD32 coeff_idx_for_pred_hoa_to_be_faded_in;
    for (coeff = 0; coeff < frame_param->num_disable_coeff; coeff++)
    {
      coeff_idx_for_pred_hoa_to_be_faded_in = ptr_amb_coeff_indices_to_be_disabled[coeff] - 1;
      ptr_out_pred_grid_dir_sigs_smth =
          &syn_handle->pred_grid_dir_sigs_smthed[0] +
          coeff_idx_for_pred_hoa_to_be_faded_in * pstr_hoa_config->frame_length;
      for (sample_idx = 0; sample_idx < pstr_hoa_config->frame_length; sample_idx++)
      {
        *ptr_out_pred_grid_dir_sigs_smth++ *= fade_in_win_for_dir_sigs[sample_idx];
      }
    }
  }
  memcpy(&syn_handle->last_prediction_type[0], &prediction_type_vec[0],
         (pstr_hoa_config->num_coeffs * sizeof(*prediction_type_vec)));

  ptr_prediction_indices_mat = prediction_indices_mat;
  ptr_prediction_factors_mat = syn_handle->prediction_factors_mat;
  ptr_last_prediction_factors_mat = syn_handle->last_prediction_factors_mat;
  ptr_last_prediction_indices_mat = syn_handle->last_prediction_indices_mat;

  ia_core_coder_mem_cpy(ptr_prediction_factors_mat, ptr_last_prediction_factors_mat,
                        pstr_hoa_config->num_coeffs * pstr_hoa_config->pred_dir_sigs);
  memcpy(ptr_last_prediction_indices_mat, ptr_prediction_indices_mat,
         sizeof(*ptr_last_prediction_indices_mat) * pstr_hoa_config->num_coeffs *
             pstr_hoa_config->pred_dir_sigs);

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_compute_hoa_representation_of_spat_pred
 *
 *  \brief Computation of HOA representation of spatial prediction
 *
 *  \param [in]     dec_handle                          Pointer to spatial decoder handle
 *  \param [in]     prediction_type_vec                 Pointer to prediction type vector
 *  \param [in]     prediction_indices_mat              Pointer to prediction indices matrix
 *  \param [in,out] prediction_factors_mat              Pointer to prediction factors matrix
 *  \param [in]     ptr_fade_win                        Pointer to fade window
 *  \param [out]    ptr_pred_grid_dir_sigs              Pointer to prediction grid direction
 * signal
 *  \param [in]     spat_prediction_in_curr_frame       Pointer to spatial pred val in cur frame
 *
 *  \return IA_ERRORCODE                                     Error
 *
 */
IA_ERRORCODE impeghd_hoa_compute_hoa_representation_of_spat_pred(
    ia_spatial_dec_str *dec_handle, const pUWORD32 prediction_type_vec,
    const pUWORD32 prediction_indices_mat, const pFLOAT32 prediction_factors_mat,
    const pFLOAT32 ptr_fade_win, pFLOAT32 ptr_pred_grid_dir_sigs,
    WORD32 *spat_prediction_in_curr_frame)
{
  ia_hoa_config_struct *pstr_hoa_config = dec_handle->ia_hoa_config;
  ia_spatial_dec_pre_dom_sound_syn_str *sound_syn_handle =
      &(dec_handle->pre_dom_sound_syn_handle);
  ia_spatial_dec_dir_based_pre_dom_sound_syn_str *syn_handle =
      &(sound_syn_handle->dir_based_pre_dom_sound_syn_handle);
  UWORD32 prediction_vector_sz = pstr_hoa_config->num_coeffs;
  UWORD32 max_num_dir_sigs_for_pred = pstr_hoa_config->pred_dir_sigs;
  UWORD32 num_coeffs = pstr_hoa_config->num_coeffs;
  UWORD32 frame_sz = pstr_hoa_config->frame_length;

  const pFLOAT32 in_ps_sigs_frame = dec_handle->dyn_corr_sample_buf;
  const pFLOAT32 ptr_mode_mat_all_coarse_grid_points =
      syn_handle
          ->mode_mt_all_coarse_grid_points; /*[MAXIMUM_NUM_HOA_COEFF][HOA_MAXIMUM_PRED_VEC_SIZE]*/
  pVOID scratch = dec_handle->scratch;

  ia_core_coder_memset(ptr_pred_grid_dir_sigs, frame_sz * num_coeffs);
  pFLOAT32 ptr_tmp_faded_pred_grid_dir_sigs_frame = (pFLOAT32)scratch;

  *spat_prediction_in_curr_frame = 0;
  for (UWORD32 grid_idx = 0; grid_idx < prediction_vector_sz; grid_idx++)
  {
    if (prediction_type_vec[grid_idx] != 0)
    {
      *spat_prediction_in_curr_frame = 1;
      pFLOAT32 ptr_mode_mat_all_coarse_grid_points_lcl, ptr_prediction_factors_mat,
          ptr_pred_grid_dir_sigs_lcl;
      pUWORD32 ptr_prediction_indices_mat;
      UWORD32 num_valid_sigs_for_pred = 0, sig_idx, sample_idx, coeff_idx;

      ia_core_coder_memset(ptr_tmp_faded_pred_grid_dir_sigs_frame, MAXIMUM_FRAME_SIZE);
      ptr_prediction_indices_mat = (UWORD32 *)prediction_indices_mat + grid_idx;
      for (sig_idx = 0; sig_idx < max_num_dir_sigs_for_pred; sig_idx++)
      {
        if (0 == *ptr_prediction_indices_mat)
        {
          break;
        }
        else
        {
          num_valid_sigs_for_pred++;
        }
        ptr_prediction_indices_mat += prediction_vector_sz;
      }

      for (sample_idx = 0; sample_idx < frame_sz; sample_idx++)
      {
        ptr_prediction_indices_mat = (UWORD32 *)prediction_indices_mat + grid_idx;
        ptr_prediction_factors_mat = (pFLOAT32)prediction_factors_mat + grid_idx;

        for (sig_idx = 0; sig_idx < num_valid_sigs_for_pred; sig_idx++)
        {
          ptr_tmp_faded_pred_grid_dir_sigs_frame[sample_idx] +=
              *ptr_prediction_factors_mat *
              *(in_ps_sigs_frame + (*ptr_prediction_indices_mat - 1) * frame_sz + sample_idx);

          ptr_prediction_indices_mat += prediction_vector_sz;
          ptr_prediction_factors_mat += prediction_vector_sz;
        }
        ptr_tmp_faded_pred_grid_dir_sigs_frame[sample_idx] *= ptr_fade_win[sample_idx];
      }
      ptr_pred_grid_dir_sigs_lcl = ptr_pred_grid_dir_sigs;
      for (coeff_idx = 0; coeff_idx < num_coeffs; coeff_idx++)
      {
        ptr_mode_mat_all_coarse_grid_points_lcl =
            (pFLOAT32)ptr_mode_mat_all_coarse_grid_points + coeff_idx * num_coeffs + grid_idx;
        for (sample_idx = 0; sample_idx < frame_sz; sample_idx++)
        {
          *ptr_pred_grid_dir_sigs_lcl += ptr_tmp_faded_pred_grid_dir_sigs_frame[sample_idx] *
                                         *ptr_mode_mat_all_coarse_grid_points_lcl;
          ptr_pred_grid_dir_sigs_lcl++;
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */