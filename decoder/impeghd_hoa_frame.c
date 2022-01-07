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
#include <stdio.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_rom.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_hoa_frame.h"

/**
 * @defgroup HOAFrmParse HOA Frame data parsing
 * @ingroup  HOAFrmParse
 * @brief HOA Frame data parsing
 *
 * @{
 */

/**
 *  impeghd_hoa_get_sub_idx
 *
 *  \brief Convert HOA coeff index to subtract index
 *
 *  \param [in] hoa_coeff_idx   HOA coefficient index
 *
 *  \return WORD32              HOA sub-index
 *
 */
static WORD32 impeghd_hoa_get_sub_idx(WORD32 hoa_coeff_idx)
{
  if (hoa_coeff_idx >= 50 || hoa_coeff_idx <= 0)
  {
    return (0);
  }
  else if (hoa_coeff_idx >= 10 && hoa_coeff_idx <= 49)
  {
    return (3);
  }
  else if (hoa_coeff_idx >= 5 && hoa_coeff_idx <= 9)
  {
    return (2);
  }
  else
  {
    return (1);
  }
}

/**
 *  impeghd_hoa_huffman_decode
 *
 *  \brief HOA Huffman decoding
 *
 *  \param [in] ptr_bit_buf Pointer to bit buffer structure
 *  \param [in] n_bit_q     Number of bits
 *  \param [in] huff_idx    Huffman index
 *
 *  \return UWORD16         Decoded value
 *
 */
static UWORD16 impeghd_hoa_huffman_decode(ia_bit_buf_struct *ptr_bit_buf, WORD32 n_bit_q,
                                          WORD32 huff_idx)
{
  UWORD16 cid = 0;
  const impeghd_word_struct *table = ia_hoa_frame_huffman_table[n_bit_q - 6][huff_idx - 1];
  WORD32 huff_val = 0, huff_word_found = 0, word_length = 0, table_length = n_bit_q, l, word = 0;

  while (huff_word_found == 0)
  {
    huff_val = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    word <<= 1;
    if (huff_val == 1)
      word |= 1;

    word_length++;
    for (l = 0; l < table_length && (!huff_word_found); l++)
    {
      if ((word == table[l].code_word) && word_length == table[l].len)
      {
        cid = table[l].symbol;
        huff_word_found = 1;
      }
    }
  }
  return cid;
}

/**
 *  impeghd_hoa_frame
 *
 *  \brief Fetch HOA frame data
 *
 *  \param [in]     ptr_bit_buf     Pointer to bit buffer structure
 *  \param [in,out] ptr_frame_data  Pointer to HOA frame structure
 *
 *  \return IA_ERRORCODE  error code
 *
 */
IA_ERRORCODE impeghd_hoa_frame(ia_bit_buf_struct *ptr_bit_buf,
                               ia_hoa_frame_struct *ptr_frame_data)
{
  ia_hoa_config_struct *pstr_hoa_config = ptr_frame_data->ptr_config_data;
  ptr_frame_data->num_of_vec_sigs = 0;
  ptr_frame_data->num_of_dir_sigs = 0;
  UWORD32 i, coded_gain;

  memset(ptr_frame_data->new_vec_channels, 0, sizeof(ptr_frame_data->new_vec_channels));
  memset(ptr_frame_data->fade_in_add_hoa_channels, 0,
         sizeof(ptr_frame_data->fade_in_add_hoa_channels));
  memset(ptr_frame_data->non_transitional_add_hoa_channels, 0,
         sizeof(ptr_frame_data->non_transitional_add_hoa_channels));

  ptr_frame_data->hoa_independency_flag = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  for (i = 0; i < pstr_hoa_config->num_addnl_coders; i++)
  {
    ptr_frame_data->channel_type[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
    if (2 == ptr_frame_data->channel_type[i])
    {
      if (!ptr_frame_data->hoa_independency_flag)
      {
        WORD32 amb_coeff_idx_transition = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        if (amb_coeff_idx_transition != 1)
        {
          ptr_frame_data->amb_coeff_transition_state[i] = 0;
        }
        else if (ptr_frame_data->amb_coeff_transition_state[i] <= 1)
        {
          ptr_frame_data->amb_coeff_transition_state[i] = 2;
        }
        else
        {
          ptr_frame_data->amb_coeff_transition_state[i] = 1;
          WORD32 coded_amb_coeff_idx =
              ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->amb_asign_m_bits);
          ptr_frame_data->amb_coeff_idx[i] =
              coded_amb_coeff_idx + 1 + pstr_hoa_config->min_coeffs_for_amb;
        }
      }
      else
      {
        ptr_frame_data->amb_coeff_transition_state[i] =
            ia_core_coder_read_bits_buf(ptr_bit_buf, 2);

        /*AmbCoeffTransitionState shall NOT be 3*/
        if (ptr_frame_data->amb_coeff_transition_state[i] == 3)
        {
          return IA_MPEGH_HOA_INIT_FATAL_INVALID_AMB_COEF_TRANS_STATE;
        }

        WORD32 coded_amb_coeff_idx =
            ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->amb_asign_m_bits);

        /*CodedAmbCoeffIdx shall be smaller than MaxNumAddActiveAmbCoeffs*/
        if (coded_amb_coeff_idx >= ptr_frame_data->ptr_config_data->max_num_add_active_amb_coeffs)
        {
          return IA_MPEGH_HOA_INIT_FATAL_INVALID_AMB_COEF_IDX;
        }

        ptr_frame_data->amb_coeff_idx[i] =
            coded_amb_coeff_idx + 1 + pstr_hoa_config->min_coeffs_for_amb;
      }
      if (1 == pstr_hoa_config->coded_v_vec_length)
      {
        if (ptr_frame_data->amb_coeff_transition_state[i] == 1)
        {
          ptr_frame_data->fade_in_add_hoa_channels[ptr_frame_data->amb_coeff_idx[i]] = 1;
        }
        else if (ptr_frame_data->amb_coeff_transition_state[i] == 0)
        {
          ptr_frame_data->non_transitional_add_hoa_channels[ptr_frame_data->amb_coeff_idx[i]] = 1;
        }
      }
    }
    else if (1 == ptr_frame_data->channel_type[i])
    {
      WORD32 ba, bb, c;
      if (!ptr_frame_data->hoa_independency_flag)
      {
        if (pstr_hoa_config->coded_v_vec_length == 1 && ptr_frame_data->prev_channel_type[i] != 1)
        {
          ptr_frame_data->new_vec_channels[i] = 1;
        }
        ba = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        bb = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

        if ((ba + bb) != 0)
        {
          c = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
          ptr_frame_data->n_bits_q[i] = (8 * ba) + (4 * bb) + c;
          if (ptr_frame_data->n_bits_q[i] == 4)
          {
            ptr_frame_data->codebk_idx[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
            ptr_frame_data->num_vvec_indices[i] = ia_core_coder_read_bits_buf(
                ptr_bit_buf, pstr_hoa_config->num_v_vec_vq_elements_bits);
            ptr_frame_data->num_vvec_indices[i]++;
          }
          else if (ptr_frame_data->n_bits_q[i] >= 6)
          {
            ptr_frame_data->p_flag[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
            ptr_frame_data->cb_flag[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
          }
        }
        else
        {
          ptr_frame_data->num_vvec_indices[i] = ptr_frame_data->prev_num_vvec_indices[i];
          ptr_frame_data->codebk_idx[i] = ptr_frame_data->prev_codebk_idx[i];
          ptr_frame_data->cb_flag[i] = ptr_frame_data->prev_cb_flag[i];
          ptr_frame_data->p_flag[i] = ptr_frame_data->prev_p_flag[i];
          ptr_frame_data->n_bits_q[i] = ptr_frame_data->prev_n_bits_q[i];
        }
      }
      else
      {
        if (pstr_hoa_config->coded_v_vec_length == 1)
        {
          ptr_frame_data->new_vec_channels[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }

        ptr_frame_data->n_bits_q[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 4);
        /*if hoaIndependencyFlag == 1 then the two MSBs of NbitsQ shall not be 00 binary*/

        if (((ptr_frame_data->n_bits_q[i] | 0x0000) >> 2) == 0)
        {
          return IA_MPEGH_HOA_INIT_FATAL_INVALID_HOA_NBITSQ;
        }

        if (ptr_frame_data->n_bits_q[i] == 4)
        {
          ptr_frame_data->codebk_idx[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
          ptr_frame_data->num_vvec_indices[i] = ia_core_coder_read_bits_buf(
              ptr_bit_buf, pstr_hoa_config->num_v_vec_vq_elements_bits);
          ptr_frame_data->num_vvec_indices[i]++;
        }
        else if (ptr_frame_data->n_bits_q[i] >= 6)
        {
          ptr_frame_data->p_flag[i] = 0;
          ptr_frame_data->cb_flag[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }
      }
    }
    else if (0 == ptr_frame_data->channel_type[i])
    {
      ptr_frame_data->active_dirs_ids[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 10);
      if (ptr_frame_data->active_dirs_ids[i] > HOA_MAX_ACTIVE_DIRS_IDS)
      {
        ptr_frame_data->active_dirs_ids[i] = 0;
        return IA_MPEGH_HOA_INIT_FATAL_INVALID_ACTIVE_DIRS_IDS;
      }
    }

    if (!ptr_frame_data->hoa_independency_flag)
    {
      ptr_frame_data->gain_corr_prev_amp_exp[i] = 0;
    }
    else
    {
      WORD32 bs_gain_corr_prev_amp_exp =
          ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->gain_corr_prev_amp_exp_bits);
      ptr_frame_data->gain_corr_prev_amp_exp[i] =
          bs_gain_corr_prev_amp_exp -
          impeghd_hoa_get_ceil_log2((WORD32)(1.5f * pstr_hoa_config->num_coeffs));
    }
    coded_gain = 0;
    do
    {
      coded_gain++;
    } while (!(ia_core_coder_read_bits_buf(ptr_bit_buf, 1)));
    ptr_frame_data->coded_gain_correction_exp[i] = coded_gain;
    ptr_frame_data->gain_correction_exception[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

    switch (ptr_frame_data->channel_type[i])
    {
    case 1:
      ptr_frame_data->vec_sig_channel_ids[ptr_frame_data->num_of_vec_sigs] = i + 1;
      ptr_frame_data->num_of_vec_sigs++;
      break;
    case 0:
      ptr_frame_data->dir_sig_channel_ids[ptr_frame_data->num_of_dir_sigs] = i + 1;
      ptr_frame_data->num_of_dir_sigs++;
      break;
    }
  }
  for (i = pstr_hoa_config->num_addnl_coders; i < pstr_hoa_config->num_transport_ch; ++i)
  {
    if (ptr_frame_data->hoa_independency_flag)
    {
      WORD32 bs_gain_corr_prev_amp_exp =
          ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->gain_corr_prev_amp_exp_bits);
      ptr_frame_data->gain_corr_prev_amp_exp[i] =
          bs_gain_corr_prev_amp_exp -
          impeghd_hoa_get_ceil_log2((WORD32)(1.5f * pstr_hoa_config->num_coeffs));
    }
    coded_gain = 0;
    do
    {
      coded_gain++;
    } while (!(ia_core_coder_read_bits_buf(ptr_bit_buf, 1)));
    ptr_frame_data->coded_gain_correction_exp[i] = coded_gain;
    ptr_frame_data->gain_correction_exception[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }

  for (i = 0; i < ptr_frame_data->num_of_vec_sigs; i++)
  {
    ptr_frame_data->sgn_val_size[i] = ptr_frame_data->prev_sgn_val_size[i];
  }

  for (i = 0; i < ptr_frame_data->num_of_vec_sigs; i++)
  {
    IA_ERRORCODE err = impeghd_hoa_frame_v_vector_data(
        ptr_bit_buf, ptr_frame_data, ptr_frame_data->vec_sig_channel_ids[i] - 1, i);
    if (err)
    {
      return err;
    }
  }

  for (i = 0; i < ptr_frame_data->num_of_vec_sigs; i++)
  {
    ptr_frame_data->prev_sgn_val_size[i] = ptr_frame_data->sgn_val_size[i];
  }
  if (pstr_hoa_config->is_single_layer == 1)
  {
    if (ptr_frame_data->num_of_dir_sigs > 0)
    {
      WORD32 pred_ids_bits, loop;
      pred_ids_bits = impeghd_hoa_get_ceil_log2(ptr_frame_data->num_of_dir_sigs + 1);
      ptr_frame_data->ps_prediction_active = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (ptr_frame_data->ps_prediction_active)
      {
        ptr_frame_data->num_active_pred = 0;
        ptr_frame_data->kind_of_coded_pred_ids = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        for (i = 0; i < pstr_hoa_config->num_coeffs; i++)
          ptr_frame_data->active_pred[i] = 0;
        if (!ptr_frame_data->kind_of_coded_pred_ids)
        {
          for (i = 0; i < pstr_hoa_config->num_coeffs; i++)
          {
            ptr_frame_data->active_pred[i] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
            if (ptr_frame_data->active_pred[i])
            {
              ptr_frame_data->num_active_pred++;
            }
          }
        }
        else
        {
          ptr_frame_data->num_active_pred =
              ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->num_active_pred_ids_bits);
          ptr_frame_data->num_active_pred = ptr_frame_data->num_active_pred + 1;
          i = 0;
          while (i < (UWORD32)ptr_frame_data->num_active_pred)
          {
            WORD32 pred_id =
                ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->active_pred_ids_bits);
            ptr_frame_data->active_pred[pred_id] = 1;
            i++;
          }
        }

        ptr_frame_data->num_of_gains = 0;
        loop = ptr_frame_data->num_active_pred * pstr_hoa_config->pred_dir_sigs;
        for (i = 0; i < (UWORD32)loop; i++)
        {
          ptr_frame_data->pred_dir_sig_ids[i] =
              ia_core_coder_read_bits_buf(ptr_bit_buf, pred_ids_bits);
          if (ptr_frame_data->pred_dir_sig_ids[i] > 0)
          {
            ptr_frame_data->pred_dir_sig_ids[i] =
                ptr_frame_data->dir_sig_channel_ids[ptr_frame_data->pred_dir_sig_ids[i] - 1];
            ptr_frame_data->num_of_gains++;
          }
        }
        for (i = 0; i < (UWORD32)ptr_frame_data->num_of_gains; i++)
        {
          if (ptr_frame_data->pred_dir_sig_ids[i] > 0)
          {
            WORD32 bs_pred_gains =
                ia_core_coder_read_bits_buf(ptr_bit_buf, pstr_hoa_config->num_bits_per_sf);
            ptr_frame_data->pred_gains[i] = bs_pred_gains;
          }
        }
      }
    }
  }

  // Keep required data in prev variable.
  memcpy(ptr_frame_data->prev_num_vvec_indices, ptr_frame_data->num_vvec_indices,
         sizeof(ptr_frame_data->prev_num_vvec_indices));
  memcpy(ptr_frame_data->prev_codebk_idx, ptr_frame_data->codebk_idx,
         sizeof(ptr_frame_data->prev_codebk_idx));
  memcpy(ptr_frame_data->prev_cb_flag, ptr_frame_data->cb_flag,
         sizeof(ptr_frame_data->prev_cb_flag));
  memcpy(ptr_frame_data->prev_p_flag, ptr_frame_data->p_flag,
         sizeof(ptr_frame_data->prev_p_flag));
  memcpy(ptr_frame_data->prev_n_bits_q, ptr_frame_data->n_bits_q,
         sizeof(ptr_frame_data->prev_n_bits_q));
  memcpy(ptr_frame_data->prev_channel_type, ptr_frame_data->channel_type,
         sizeof(ptr_frame_data->prev_channel_type));
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_frame_v_vector_data
 *
 *  \brief Fetch HOA frame vector data
 *
 *  \param [in]     ptr_bit_buf     Pointer to bit buffer structure
 *  \param [in,out] ptr_frame_data  Pointer to HOA frame structure
 *  \param [in]     sig_ch_idx               Vector signal channel index
 *  \param [in]     idx             Vector signal number
 *
 *  \return IA_ERRORCODE  error code
 *
 */
IA_ERRORCODE impeghd_hoa_frame_v_vector_data(ia_bit_buf_struct *ptr_bit_buf,
                                             ia_hoa_frame_struct *ptr_frame_data,
                                             WORD32 sig_ch_idx, WORD32 idx)
{
  ia_hoa_config_struct *pstr_hoa_config = ptr_frame_data->ptr_config_data;
  UWORD32 n_bit_idx = 0;
  UWORD32 hoa_coeff_idx = pstr_hoa_config->idx_offset;
  WORD32 vec_len;
  ptr_frame_data->v_vec_length_used = pstr_hoa_config->v_vec_length_used;
  if (ptr_frame_data->n_bits_q[sig_ch_idx] == 4)
  {
    WORD32 j, bs_vvec_idx;

    for (j = ptr_frame_data->sgn_val_size[idx];
         j < (WORD32)ptr_frame_data->num_vvec_indices[sig_ch_idx]; j++)
    {
      ptr_frame_data->sgn_val[idx][j] = 0;
    }
    ptr_frame_data->sgn_val_size[idx] = ptr_frame_data->num_vvec_indices[sig_ch_idx];
    switch (ptr_frame_data->codebk_idx[sig_ch_idx])
    {
    case 7: // reserved
    case 6: // reserved
    case 5: // reserved
    case 4:
      n_bit_idx = impeghd_hoa_get_ceil_log2(pstr_hoa_config->num_coeffs);
      break;
    case 3:
      n_bit_idx = (6);
      break;
    case 2:
      n_bit_idx = (6);
      break;
    case 1:
      n_bit_idx = (6);
      break;
    case 0:
      n_bit_idx = (10);
    }
    if (1 != ptr_frame_data->num_vvec_indices[sig_ch_idx])
    {
      ptr_frame_data->weight_idx[sig_ch_idx] = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
      for (j = 0; j < (WORD32)ptr_frame_data->num_vvec_indices[sig_ch_idx]; j++)
      {
        bs_vvec_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, n_bit_idx);
        ptr_frame_data->sgn_val[idx][j] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        ptr_frame_data->vvec_idx[j] = bs_vvec_idx + 1;
      }
    }
    else
    {
      bs_vvec_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, n_bit_idx);
      ptr_frame_data->sgn_val[idx][0] = (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      ptr_frame_data->vvec_idx[0] = (UWORD32)bs_vvec_idx + 1;
    }
    for (j = 0; j < (WORD32)ptr_frame_data->num_vvec_indices[sig_ch_idx]; j++)
    {
      /*ISO/IEC 23008-9 section 5.19.6*/
      switch (ptr_frame_data->codebk_idx[j])
      {
      case 0:
        if (ptr_frame_data->vvec_idx[j] > 900)
        {
          return IA_MPEGH_HOA_INIT_FATAL_INVALID_VVEC_IDX;
        }
        break;
      case 1:
      case 2:
        if (ptr_frame_data->vvec_idx[j] > 34)
        {
          return IA_MPEGH_HOA_INIT_FATAL_INVALID_VVEC_IDX;
        }
        break;
      case 3:
        if (ptr_frame_data->vvec_idx[j] > 64)
        {
          return IA_MPEGH_HOA_INIT_FATAL_INVALID_VVEC_IDX;
        }
        break;
      case 7:
        if (ptr_frame_data->ptr_config_data->order == 4)
        {
          if (ptr_frame_data->vvec_idx[j] > 32)
          {
            return IA_MPEGH_HOA_INIT_FATAL_INVALID_VVEC_IDX;
          }
        }
        else
        {
          UWORD32 max;
          max = ((ptr_frame_data->ptr_config_data->order + 1) *
                 (ptr_frame_data->ptr_config_data->order + 1)) -
                1;
          if (ptr_frame_data->vvec_idx[j] > (max + 1))
          {
            return IA_MPEGH_HOA_INIT_FATAL_INVALID_VVEC_IDX;
          }
        }
        break;
      default:
        break;
      }
    }

    for (vec_len = 0; vec_len < ptr_frame_data->v_vec_length_used; vec_len++, hoa_coeff_idx++)
    {
      if (!(ptr_frame_data->non_transitional_add_hoa_channels[hoa_coeff_idx] == 0) ||
          ((ptr_frame_data->new_vec_channels[idx] == 1) &&
           (ptr_frame_data->fade_in_add_hoa_channels[hoa_coeff_idx] == 1)))
      {
        ptr_frame_data->additional_info[idx][vec_len] = 0;
      }
      else
      {
        ptr_frame_data->additional_info[idx][vec_len] = 1;
      }
    }
  }
  else if (ptr_frame_data->n_bits_q[sig_ch_idx] == 5)
  {
    WORD32 vec_val;
    for (vec_len = 0; vec_len < ptr_frame_data->v_vec_length_used; vec_len++, hoa_coeff_idx++)
    {
      if (!(ptr_frame_data->non_transitional_add_hoa_channels[hoa_coeff_idx] == 0) ||
          ((ptr_frame_data->new_vec_channels[idx] == 1) &&
           (ptr_frame_data->fade_in_add_hoa_channels[hoa_coeff_idx] == 1)))
      {
        vec_val = 128;
      }
      else
      {
        vec_val = ia_core_coder_read_bits_buf(ptr_bit_buf, 8);
      }
      ptr_frame_data->c_8bit_quantizer_word[idx][vec_len] = vec_val;
    }
  }
  else if (ptr_frame_data->n_bits_q[sig_ch_idx] >= 6)
  {
    UWORD16 cid = 0;
    WORD32 huff_idx, int_add_val;

    for (vec_len = ptr_frame_data->sgn_val_size[idx]; vec_len < ptr_frame_data->v_vec_length_used;
         vec_len++)
    {
      ptr_frame_data->sgn_val[idx][vec_len] = 1;
    }

    for (vec_len = 0; vec_len < ptr_frame_data->v_vec_length_used; vec_len++, hoa_coeff_idx++)
    {
      if (!(ptr_frame_data->non_transitional_add_hoa_channels[hoa_coeff_idx] == 0) ||
          ((ptr_frame_data->new_vec_channels[idx] == 1) &&
           (ptr_frame_data->fade_in_add_hoa_channels[hoa_coeff_idx] == 1)))
      {
        ptr_frame_data->element_bitmask[sig_ch_idx][vec_len] = 0;
        ptr_frame_data->decoded_huffmann_word[sig_ch_idx][vec_len] = 0;
        ptr_frame_data->additional_value[sig_ch_idx][vec_len] = 0;
      }
      else
      {
        ptr_frame_data->element_bitmask[sig_ch_idx][vec_len] = 1;
        // huffselect
        {
          huff_idx = 5;
          if (ptr_frame_data->cb_flag[sig_ch_idx] == 1)
            huff_idx = impeghd_hoa_get_sub_idx(hoa_coeff_idx);
          else if (ptr_frame_data->p_flag[sig_ch_idx] == 1)
            huff_idx = 4;
        }
        cid = impeghd_hoa_huffman_decode(ptr_bit_buf, ptr_frame_data->n_bits_q[sig_ch_idx],
                                         huff_idx);
        ptr_frame_data->decoded_huffmann_word[sig_ch_idx][vec_len] = cid;
        if (cid > 0)
        {
          ptr_frame_data->additional_value[sig_ch_idx][vec_len] = 0;
          ptr_frame_data->sgn_val[idx][vec_len] =
              (UWORD8)ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
          if (cid > 1)
          {
            int_add_val = ia_core_coder_read_bits_buf(ptr_bit_buf, cid - 1);
            ptr_frame_data->additional_value[sig_ch_idx][vec_len] = int_add_val;
          }
        }
      }
    }
    ptr_frame_data->sgn_val_size[idx] = ptr_frame_data->v_vec_length_used;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAFrmParse */