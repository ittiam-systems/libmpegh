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
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_frame_params.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_amb_syn.h"
#include "impeghd_hoa_ch_reassignment.h"
#include "impeghd_hoa_config.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame.h"
#include "impeghd_hoa_init.h"
#include "impeghd_hoa_inverse_dyn_correction.h"
#include "impeghd_hoa_pre_dom_sound_syn.h"
#include "impeghd_hoa_rom.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_spatial_decoder.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_tbe_dec.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_spatial_sort_array_ascending
 *
 *  \brief Bubble sorting array in asending order to new given array
 *
 *  \param [in]  src Unsorted input array.
 *  \param [out] dst Sorted output array.
 *  \param [in]  sz  Size of the input array.
 *
 *
 *
 */
static VOID impeghd_hoa_spatial_sort_array_ascending(pUWORD32 src, pUWORD32 dst, UWORD32 sz)
{
  UWORD32 i, set_sz;
  for (i = 0; (WORD32)src[i] != -1; i++)
  {
    if (i == sz)
      break;
  }
  set_sz = i;

  if (!set_sz)
  {
    WORD32 j;
    for (j = sz - 1; j >= 0; j--)
    {
      dst[j] = (UWORD32)-1;
    }
  }
  else
  {
    UWORD32 j, next = 0xFFFF, next_id = (UWORD32)-1;
    for (j = 0; j < set_sz; j++)
    {
      next = 0xFFFF;
      for (i = 0; i < set_sz; i++)
      {
        if (((WORD32)src[i] != -1) && (next > src[i]))
        {
          next_id = i;
          next = src[i];
        }
        else if (next == src[i])
        {
          src[i] = (UWORD32)-1;
        }
      }
      if (next == 0xFFFF)
        break;

      src[next_id] = (UWORD32)-1;
      dst[j] = next;
    }
    for (; j < sz; j++)
    {
      dst[j] = (UWORD32)-1;
    }
  }
}

/**
 *  impeghd_hoa_spatial_init
 *
 *  \brief Initializes spatial decoder
 *
 *  \param [out]  handle          Spatial decoder handle
 *  \param [in]    pstr_hoa_config  HOA configuration handle
 *  \param [in]    scratch          Pointer to scratch buffer for
 * intermediate
 * processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_spatial_init(pVOID handle, ia_hoa_config_struct *pstr_hoa_config,
                                      pVOID scratch)
{
  ia_spatial_dec_str *pstr_spatial_dec = (ia_spatial_dec_str *)handle;
  pstr_spatial_dec->scratch = scratch;
  pstr_spatial_dec->ia_hoa_config = pstr_hoa_config;
  UWORD32 order = pstr_hoa_config->order;
  UWORD32 ch;
  IA_ERRORCODE err_code;

  pstr_spatial_dec->vec_start = 0;
  pstr_spatial_dec->num_bits_for_vec_elem_quant = 8;
  pstr_spatial_dec->min_num_coeffs_for_amb_hoa =
      (UWORD32)((pstr_hoa_config->min_amb_order + 1) * (pstr_hoa_config->min_amb_order + 1));

  if (0 < pstr_hoa_config->coded_v_vec_length)
  {
    pstr_spatial_dec->vec_start = pstr_spatial_dec->min_num_coeffs_for_amb_hoa;
  }

  pstr_spatial_dec->num_vec_elems = pstr_hoa_config->num_coeffs - pstr_spatial_dec->vec_start;
  pstr_spatial_dec->max_num_transmitted_hoa_coeffs =
      (pstr_hoa_config->max_order_to_be_transmitted + 1) *
      (pstr_hoa_config->max_order_to_be_transmitted + 1);

  impeghd_hoa_dec_frame_param_init(pstr_spatial_dec);

  if ((pstr_hoa_config->num_transport_ch < 1) ||
      (pstr_hoa_config->num_transport_ch < pstr_hoa_config->num_addnl_coders))
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_TRANSPORT_CHANNEL;
  }
  err_code = impeghd_hoa_ambience_synthesis_init(pstr_spatial_dec);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_pre_dom_sound_syn_init(pstr_spatial_dec);
  if (err_code)
  {
    return err_code;
  }

  pstr_spatial_dec->inv_dyn_corr_win_func =
      (FLOAT32 *)&ia_hoa_cos_table_inv_dyn_correction_compute_win_func[0];

  for (UWORD32 i = 0; i < MAX_HOA_CHANNELS; i++)
  {
    pstr_spatial_dec->old_nominators[i].index = (UWORD32)-1;
  }

  for (ch = 0; ch < pstr_hoa_config->num_transport_ch; ch++)
  {
    pstr_spatial_dec->inv_dyn_corr_prev_gain[ch] = (FLOAT32)(1.0);
  }

  pstr_spatial_dec->ptr_inv_mode_mat = (pFLOAT32)(ia_hoa_coded_vec_q_mat_elems[order]);

  err_code = impeghd_hoa_table_get_dict_2d_points(pstr_spatial_dec);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_table_get_dict_cicp_speaker_points(pstr_spatial_dec);
  if (err_code)
  {
    return err_code;
  }

  err_code = impeghd_hoa_table_get_transpose_mode_mat_for_fine_grid(
      pstr_spatial_dec->transpose_mod_mtx_grid_pts, HOA_MAXIMUM_MATRIX_SIZE,
      pstr_hoa_config->order); // Direction grid table index=2
  if (err_code)
  {
    return err_code;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_get_exponents
 *
 *  \brief Function to get exponents
 *
 *  \param [in]  coded_gain_correction_exp coded gain correction exponent
 *
 *  \return WORD32 exponents
 *
 */
static WORD32 impeghd_hoa_get_exponents(WORD32 coded_gain_correction_exp)
{
  if (1 == coded_gain_correction_exp)
    return 0;
  else if (2 == coded_gain_correction_exp)
    return -1;
  else
    return (coded_gain_correction_exp - 2);
}

/**
 *  impeghd_hoa_spatial_decode_frame_side_info
 *
 *  \brief Function to decode frame side info
 *
 *  \param [out]  handle        Spatial decoder handle
 *  \param [in]    ia_hoa_frame_ptr  HOA frame data
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE
impeghd_hoa_spatial_decode_frame_side_info(pVOID handle, ia_hoa_frame_struct *ia_hoa_frame_ptr)
{
  ia_hoa_config_struct *pstr_hoa_config = ia_hoa_frame_ptr->ptr_config_data;
  ia_hoa_frame_struct *ia_hoa_frame_t = ia_hoa_frame_ptr;

  UWORD32 old_amb_hoa_assignment_vec[MAX_HOA_CHANNELS] = {0}, ch, i, vec_ch, dir_ch, num_coeffs;
  ia_hoa_vec_sig_str *new_nominators_ptr;
  UWORD32 *enable_coeff, *disable_coeff, *en_dis_able_coeff;
  ia_spatial_dec_str *pstr_spatial_dec = (ia_spatial_dec_str *)handle;
  ia_hoa_dec_frame_param_str *frame_param = &(pstr_spatial_dec->frame_params_handle);
  UWORD32 *ptr_coeff = pstr_spatial_dec->scratch;
  UWORD32 scratch_idx = 3 * HOA_MAXIMUM_SET_SIZE * sizeof(*en_dis_able_coeff);
  pstr_spatial_dec->scratch = (WORD8 *)(pstr_spatial_dec->scratch) + scratch_idx;
  memset(ptr_coeff, 0xFF, sizeof(ptr_coeff[0]) * 3 * HOA_MAXIMUM_SET_SIZE);
  enable_coeff = &ptr_coeff[0];
  disable_coeff = &ptr_coeff[HOA_MAXIMUM_SET_SIZE];
  en_dis_able_coeff = &ptr_coeff[2 * HOA_MAXIMUM_SET_SIZE];

  for (ch = 0; ch < pstr_hoa_config->num_transport_ch; ch++)
  {
    frame_param->exponents[ch] =
        impeghd_hoa_get_exponents(ia_hoa_frame_t->coded_gain_correction_exp[ch]);
  }

  frame_param->ptr_is_exception = &ia_hoa_frame_t->gain_correction_exception[0];

  if (ia_hoa_frame_t->ps_prediction_active)
  {
    pUWORD32 ptr_pred_indices_mat;
    pWORD32 ptr_quant_pred_fact_mat;
    UWORD32 pred_idx, total_prediction_factors_mat_idx = 0, num_pred_factors_idx = 0,
                      num_pred_indices_idx = 0;

    for (dir_ch = 0; dir_ch < pstr_hoa_config->num_coeffs; dir_ch++)
    {
      if (!ia_hoa_frame_t->active_pred[dir_ch])
      {
        frame_param->prediction_type_vec[dir_ch] = 0;
      }
      else
      {
        num_pred_indices_idx++;
        ptr_pred_indices_mat = frame_param->prediction_indices_mat + dir_ch;
        ptr_quant_pred_fact_mat = frame_param->quant_pred_fact_mat + dir_ch;
        frame_param->prediction_type_vec[dir_ch] = 1;
        for (pred_idx = 0; pred_idx < pstr_hoa_config->pred_dir_sigs; pred_idx++)
        {
          *ptr_pred_indices_mat = ia_hoa_frame_t->pred_dir_sig_ids[num_pred_factors_idx];
          num_pred_factors_idx++;
          if (*ptr_pred_indices_mat != 0)
          {
            *ptr_quant_pred_fact_mat =
                ia_hoa_frame_t->pred_gains[total_prediction_factors_mat_idx];
            total_prediction_factors_mat_idx++;
          }
          ptr_quant_pred_fact_mat += pstr_hoa_config->num_coeffs;
          ptr_pred_indices_mat += pstr_hoa_config->num_coeffs;
        }
      }
    }
  }
  else
  {
    memset(frame_param->prediction_type_vec, 0,
           sizeof(frame_param->prediction_type_vec[0]) * pstr_hoa_config->num_coeffs);
  }
  new_nominators_ptr = (ia_hoa_vec_sig_str *)pstr_spatial_dec->scratch + scratch_idx;
  memset(new_nominators_ptr, 0, sizeof(*new_nominators_ptr) * MAX_HOA_CHANNELS);
  for (ch = 0; ch < pstr_hoa_config->num_transport_ch; ch++)
  {
    old_amb_hoa_assignment_vec[ch] = frame_param->amb_hoa_assign[ch];
  }
  for (ch = 0; ch < MAX_HOA_CHANNELS; ch++)
  {
    ia_core_coder_memset(frame_param->vectors[ch].sig_id, MAXIMUM_NUM_HOA_COEFF);
    new_nominators_ptr[ch].index = (UWORD32)-1;
  }
  memset(frame_param->amb_hoa_assign, 0,
         sizeof(frame_param->amb_hoa_assign[0]) * pstr_hoa_config->num_transport_ch);
  frame_param->num_dir_predom_sounds = 0;
  frame_param->num_vec_predom_sounds = 0;

  WORD32 offset = ia_max_int(((WORD32)pstr_hoa_config->num_transport_ch -
                              (WORD32)pstr_spatial_dec->min_num_coeffs_for_amb_hoa),
                             0);
  for (i = 0; i < pstr_spatial_dec->min_num_coeffs_for_amb_hoa; i++)
  {
    frame_param->amb_hoa_assign[offset + i] = i + 1;
  }

  dir_ch = 0;
  vec_ch = 0;

  for (ch = 0; ch < pstr_hoa_config->num_addnl_coders; ch++)
  {
    switch (ia_hoa_frame_t->channel_type[ch])
    {
    case HOA_VEC_CHANNEL:
    {
      if (ia_hoa_frame_t->n_bits_q[ch] > 15)
      {
        return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_BIT_SIZE;
      }
      new_nominators_ptr[vec_ch].index = ch + 1;
      FLOAT32 tmp_vec_long[MAXIMUM_NUM_HOA_COEFF];
      ia_core_coder_memset(tmp_vec_long, MAXIMUM_NUM_HOA_COEFF);
      pFLOAT32 tmp_vec = frame_param->vectors[vec_ch].sig_id;

      if (HOA_VVEC_VQ_WORD == ia_hoa_frame_t->n_bits_q[ch])
      {
        UWORD32 mat_offset = HOA_MAXIMUM_MATRIX_SIZE;
        WORD32 normalize = 1;
        pFLOAT32 curr_vq_vec_ptr =
            &(pstr_spatial_dec->transpose_mod_mtx_grid_pts[pstr_spatial_dec->vec_start]);
        FLOAT32 inv_norm_coeff = impeghd_hoa_pow_2(14) / (FLOAT32)(pstr_hoa_config->order + 1);
        FLOAT32 weight_value;
        WORD32 v_idx = ((WORD32)ia_hoa_frame_t->vvec_idx[0] - (WORD32)1);

        switch (ia_hoa_frame_t->codebk_idx[ch])
        {
        case 7:
          mat_offset = pstr_hoa_config->num_coeffs;
          curr_vq_vec_ptr = &(pstr_spatial_dec->ptr_inv_mode_mat[0]);
          break;
        case 3:
          curr_vq_vec_ptr = &(pstr_spatial_dec->dict_2d_points[0]);
          break;
        case 2:
          normalize = 0;
          curr_vq_vec_ptr = &(pstr_spatial_dec->dict_cicp_speaker_points[0]);
          break;
        case 1:
          curr_vq_vec_ptr = &(pstr_spatial_dec->dict_cicp_speaker_points[0]);
          break;
        case 0:
          curr_vq_vec_ptr = &(pstr_spatial_dec->transpose_mod_mtx_grid_pts[0]);
        }
        if (1 != ia_hoa_frame_t->num_vvec_indices[ch])
        {
          FLOAT32 norm_value = 1.0f;
          for (UWORD32 vec_idx = 0; vec_idx < ia_hoa_frame_t->num_vvec_indices[ch]; ++vec_idx)
          {
            v_idx = ((WORD32)ia_hoa_frame_t->vvec_idx[vec_idx] - 1) * (WORD32)mat_offset;

            if (vec_idx >= 8)
            {
              weight_value = ia_hoa_weighting_cdbk_256x8[ia_hoa_frame_t->weight_idx[ch] * 8 +
                                                         (vec_idx % 2 + 6)];
            }
            else
            {
              weight_value =
                  ia_hoa_weighting_cdbk_256x8[ia_hoa_frame_t->weight_idx[ch] * 8 + vec_idx];
            }

            weight_value *= 2 * ia_hoa_frame_t->sgn_val[ch][vec_idx] - 1;
            if (v_idx < 0)
            {
              return IA_MPEGH_HOA_INIT_FATAL_INVALID_VIDX;
            }
            for (UWORD32 elem_idx = 0; elem_idx < pstr_hoa_config->num_coeffs; elem_idx++)
            {
              tmp_vec_long[elem_idx] =
                  ia_mac_flt(tmp_vec_long[elem_idx], curr_vq_vec_ptr[v_idx + (WORD32)elem_idx],
                             weight_value);
            }
          }
          if (1 == normalize)
          {
            norm_value = 0.0;
            for (UWORD32 elem_idx = 0; elem_idx < pstr_hoa_config->num_coeffs; elem_idx++)
            {
              norm_value = ia_mac_flt(norm_value, tmp_vec_long[elem_idx], tmp_vec_long[elem_idx]);
            }
            norm_value = (FLOAT32)((pstr_hoa_config->order + 1) / ia_sqrt_flt(norm_value));
          }
          for (UWORD32 elem_idx = 0; elem_idx < pstr_spatial_dec->num_vec_elems; ++elem_idx)
          {
            tmp_vec[elem_idx] =
                ia_mul_flt(tmp_vec_long[elem_idx + pstr_spatial_dec->vec_start], norm_value);
          }
        }
        else
        {
          curr_vq_vec_ptr += (mat_offset * v_idx);
          weight_value = (FLOAT32)(2 * ia_hoa_frame_t->sgn_val[ch][0] - 1);
          for (UWORD32 ele = 0; ele < pstr_spatial_dec->num_vec_elems; ele++)
          {
            tmp_vec[ele] =
                ia_mul_flt(curr_vq_vec_ptr[ele + pstr_spatial_dec->vec_start], weight_value);
          }
        }
        if (1 == pstr_hoa_config->coded_v_vec_length)
        {
          for (UWORD32 elem_idx = 0; elem_idx < pstr_spatial_dec->num_vec_elems; ++elem_idx)
          {
            if (!ia_hoa_frame_t->additional_info[ch][elem_idx])
            {
              tmp_vec[elem_idx] = 0.0;
            }
          }
        }
        for (UWORD32 elem_idx = 0; elem_idx < pstr_spatial_dec->num_vec_elems; ++elem_idx)
        {
          new_nominators_ptr[ch].sig_id[elem_idx] = (FLOAT32)(
              2.0f *
              ia_floor_flt(ia_add_flt(ia_mul_flt(inv_norm_coeff, tmp_vec[elem_idx]), 0.5f)));
        }
      }
      else if (HOA_BIT_QUANTIZER_WORD == ia_hoa_frame_t->n_bits_q[ch])
      {
        for (UWORD32 elem_idx = 0; elem_idx < pstr_spatial_dec->num_vec_elems; elem_idx++)
        {
          if (pstr_spatial_dec->num_bits_for_vec_elem_quant != 0)
          {
            UWORD32 pow_result = 1;
            for (UWORD32 exp_idx = 0;
                 exp_idx < (pstr_spatial_dec->num_bits_for_vec_elem_quant - 1); exp_idx++)
            {
              pow_result <<= 1;
            }
            tmp_vec[elem_idx] = (FLOAT32)ia_sub_flt(
                ((FLOAT32)(ia_hoa_frame_t->c_8bit_quantizer_word[ch][elem_idx]) /
                 (FLOAT32)(pow_result)),
                1.0);
          }
          else
          {
            return IA_MPEGH_HOA_INIT_NONFATAL_INVALID_QUANT;
          }
          new_nominators_ptr[ch].sig_id[elem_idx] = tmp_vec[elem_idx] * (UWORD32)(HOA_TWO_POW_15);
          tmp_vec[elem_idx] =
              ia_mul_flt((FLOAT32)(pstr_hoa_config->order + 1), tmp_vec[elem_idx]);
        }
      }
      else if (HOA_BIT_QUANTIZER_WORD < ia_hoa_frame_t->n_bits_q[ch])
      {
        FLOAT32 nominator;
        for (UWORD32 elem_idx = 0; elem_idx < pstr_spatial_dec->num_vec_elems; elem_idx++)
        {
          nominator = 0.0;
          if (ia_hoa_frame_t->decoded_huffmann_word[ch][elem_idx])
          {
            WORD32 sign = ia_hoa_frame_t->sgn_val[ch][elem_idx] * 2 - 1;
            WORD32 val =
                sign * ((WORD32)(impeghd_hoa_pow_2(
                                     ia_hoa_frame_t->decoded_huffmann_word[ch][elem_idx] - 1) +
                                 ia_hoa_frame_t->additional_value[ch][elem_idx]));
            nominator =
                (FLOAT32)((WORD32)(impeghd_hoa_pow_2(16 - ia_hoa_frame_t->n_bits_q[ch])) * val);
          }
          if (ia_hoa_frame_t->p_flag[ch] && ia_hoa_frame_t->element_bitmask[ch][elem_idx])
          {
            UWORD32 find_idx = 0;
            for (find_idx = 0; find_idx < MAX_HOA_CHANNELS; find_idx++)
            {
              if ((ch + 1) == pstr_spatial_dec->old_nominators[find_idx].index)
                break;
            }
            if (MAX_HOA_CHANNELS != find_idx)
            {
              nominator = ia_add_flt(nominator,
                                     pstr_spatial_dec->old_nominators[find_idx].sig_id[elem_idx]);
            }
          }
          new_nominators_ptr[ch].sig_id[elem_idx] = nominator;
          tmp_vec[elem_idx] = nominator / HOA_TWO_POW_15;
          tmp_vec[elem_idx] =
              ia_mul_flt((FLOAT32)(pstr_hoa_config->order + 1), tmp_vec[elem_idx]);
        }
      }
      frame_param->vectors[vec_ch].index = (ch + 1);
      vec_ch++;
    }
    break;
    case ADD_HOA_CHANNEL:
    {
      if (1 != ia_hoa_frame_t->amb_coeff_transition_state[ch])
      {

        if (!ia_hoa_frame_t->hoa_independency_flag)
        {
          frame_param->amb_hoa_assign[ch] = old_amb_hoa_assignment_vec[ch];
        }
        else
        {
          frame_param->amb_hoa_assign[ch] = ia_hoa_frame_t->amb_coeff_idx[ch];
        }

        if (2 != ia_hoa_frame_t->amb_coeff_transition_state[ch])
        {
          *en_dis_able_coeff++ = ia_hoa_frame_t->amb_coeff_idx[ch];
        }
        else
        {
          *disable_coeff++ = ia_hoa_frame_t->amb_coeff_idx[ch];
        }
      }
      else
      {
        frame_param->amb_hoa_assign[ch] = ia_hoa_frame_t->amb_coeff_idx[ch];
        *enable_coeff++ = ia_hoa_frame_t->amb_coeff_idx[ch];
      }
    }
    break;
    case HOA_DIR_CHANNEL:
    {
      frame_param->active_and_grid_dir_indices[dir_ch].dir_id =
          ia_hoa_frame_t->active_dirs_ids[ch];
      frame_param->active_and_grid_dir_indices[dir_ch].ch_idx = ch + 1;
      dir_ch++;
    }
    break;
    }
  }
  frame_param->num_vec_predom_sounds = vec_ch;
  frame_param->num_dir_predom_sounds = dir_ch;
  for (ch = 0; ch < MAX_HOA_CHANNELS; ch++)
  {
    if (-1 != (WORD32)new_nominators_ptr[ch].index)
    {
      ia_core_coder_mem_cpy(new_nominators_ptr[ch].sig_id,
                            pstr_spatial_dec->old_nominators[ch].sig_id,
                            pstr_spatial_dec->num_vec_elems);
    }
    pstr_spatial_dec->old_nominators[ch].index = new_nominators_ptr[ch].index;
  }

  pstr_spatial_dec->scratch = ptr_coeff;
  num_coeffs = (UWORD32)((enable_coeff - ptr_coeff) / sizeof(enable_coeff[0]));
  frame_param->num_enable_coeff = num_coeffs;
  impeghd_hoa_spatial_sort_array_ascending(ptr_coeff, frame_param->amb_coeff_indices_to_enable,
                                           HOA_MAXIMUM_SET_SIZE);
  ptr_coeff += HOA_MAXIMUM_SET_SIZE;

  num_coeffs = (UWORD32)((disable_coeff - ptr_coeff) / sizeof(disable_coeff[0]));
  frame_param->num_disable_coeff = num_coeffs;
  impeghd_hoa_spatial_sort_array_ascending(ptr_coeff, frame_param->amb_coeff_indices_to_disable,
                                           HOA_MAXIMUM_SET_SIZE);
  ptr_coeff += HOA_MAXIMUM_SET_SIZE;

  num_coeffs = (UWORD32)((en_dis_able_coeff - ptr_coeff) / sizeof(en_dis_able_coeff[0]));
  frame_param->num_en_dis_able_coeff = num_coeffs;
  impeghd_hoa_spatial_sort_array_ascending(
      ptr_coeff, frame_param->non_en_dis_able_act_hoa_coeff_indices, HOA_MAXIMUM_SET_SIZE);

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_spatial_process
 *
 *  \brief Process the spatial decoder
 *
 *  \param [in,out]  handle            Spatial decoder handle
 *  \param [in]    in_sample_buf        Input sample buffer
 *  \param [out]  out_final_comp_coeffs_buf  Output coefficient buffer
 *  \param [in]    scratch            Pointer to scratch buffer
 * for
 * intermediate processing
 *  \param [in]    ia_hoa_frame_ptr      HOA frame data
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_spatial_process(pVOID handle, const pFLOAT32 in_sample_buf,
                                         pFLOAT32 out_final_comp_coeffs_buf, pVOID scratch,
                                         ia_hoa_frame_struct *ia_hoa_frame_ptr)
{
  WORD32 *ptr_reassign_vec = scratch;
  WORD32 scratch_idx = (HOA_MAXIMUM_NUM_PERC_CODERS * sizeof(*ptr_reassign_vec));
  ia_spatial_dec_str *pstr_spatial_dec = (ia_spatial_dec_str *)handle;
  ia_hoa_dec_frame_param_str *frameParam = &(pstr_spatial_dec->frame_params_handle);
  pstr_spatial_dec->scratch = (WORD8 *)scratch + scratch_idx;
  ia_hoa_frame_struct *ia_hoa_frame_t = ia_hoa_frame_ptr;
  ia_hoa_config_struct *pstr_hoa_config = ia_hoa_frame_ptr->ptr_config_data;

  IA_ERRORCODE err_code =
      impeghd_hoa_spatial_decode_frame_side_info((pVOID)pstr_spatial_dec, ia_hoa_frame_ptr);
  if (err_code)
  {
    return err_code;
  }
  if (ia_hoa_frame_t->hoa_independency_flag)
  {
    for (UWORD32 ch = 0; ch < pstr_hoa_config->num_transport_ch; ch++)
    {
      pstr_spatial_dec->inv_dyn_corr_prev_gain[ch] =
          impeghd_hoa_pow_2(ia_hoa_frame_t->gain_corr_prev_amp_exp[ch]);
    }
  }

  memset(ptr_reassign_vec, 0xFF, sizeof(*ptr_reassign_vec) * HOA_MAXIMUM_NUM_PERC_CODERS);

  // Get channel reassignment vector
  for (UWORD32 chan = 0; chan < pstr_spatial_dec->ia_hoa_config->num_transport_ch; chan++)
  {
    if (frameParam->amb_hoa_assign[chan] != 0)
    {
      ptr_reassign_vec[frameParam->amb_hoa_assign[chan] - 1] = chan;
    }
  }
  err_code = impeghd_hoa_inv_dyn_correction_process((FLOAT32 *)in_sample_buf, pstr_spatial_dec);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_ambience_synthesis_process(pstr_spatial_dec, ptr_reassign_vec);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_ch_reassignment_process(pstr_spatial_dec);
  if (err_code)
  {
    return err_code;
  }
  err_code = impeghd_hoa_pre_dom_sound_syn_process(pstr_spatial_dec, out_final_comp_coeffs_buf);
  if (err_code)
  {
    return err_code;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */