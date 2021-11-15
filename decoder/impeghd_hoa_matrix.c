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
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_hoa_rom.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_read_range
 *
 *  \brief Read range value
 *
 *  \param [in/out] ptr_bit_buf Pointer to bit buffer structure
 *  \param [in]     alphabet_sz Size of alphabet
 *
 *  \return UWORD32 Read range value
 *
 */
static UWORD32 impeghd_read_range(ia_bit_buf_struct *ptr_bit_buf, UWORD32 alphabet_sz)
{
  WORD32 n, n_bits, temp;
  n = alphabet_sz;
  WORD32 log_2 = 0;
  while (n > 1) // Calculate log to base 2 value
  {
    n >>= 1;
    ++log_2;
  }
  n_bits = log_2;
  UWORD32 n_unused = (1U << (n_bits + 1)) - alphabet_sz;
  UWORD32 val = ia_core_coder_read_bits_buf(ptr_bit_buf, n_bits);

  if (val >= n_unused)
  {
    temp = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    val = (val << 1) - n_unused + temp;
  }
  return val;
}

/**
 *  impeghd_find_symmetric_speakers
 *
 *  \brief Search for symmetric speakers
 *
 *  \param [in]  out_cnt            Output count
 *  \param [out] out_config         Pointer to speaker information
 *  \param [in]  has_lfe_rendering  flag for low frequency element rendering
 *
 *  \return WORD32                  Number of symmetric speaker pairs
 *
 */
static WORD32 impeghd_find_symmetric_speakers(WORD32 out_cnt, ia_speaker_info_str *out_config,
                                              UWORD32 has_lfe_rendering)
{
  WORD32 i, j, numPairs = 0;
  for (i = 0; i < out_cnt; ++i)
  {
    out_config[i].symmetric_pair = NULL;
    out_config[i].is_already_used = 0;
    out_config[i].original_position = (WORD16)i;
  }

  for (i = 0; i < out_cnt; ++i)
  {
    if (out_config[i].is_already_used)
      continue;

    if (!((out_config[i].azimuth == 0) || (ia_abs_int(out_config[i].azimuth) == 180)))
    {
      for (j = i + 1; j < out_cnt; ++j)
      {
        if (out_config[j].is_already_used)
          continue;

        if ((out_config[i].azimuth == -out_config[j].azimuth) &&
            (out_config[i].elevation == out_config[j].elevation) &&
            (out_config[i].is_lfe == out_config[j].is_lfe))
        {
          /* we found a symmetric L/R pair for speaker on position i */
          out_config[i].pair_type = HOA_SP_PAIR_SYMMETRIC;
          out_config[i].symmetric_pair = &(out_config[j]);
          if (0 == (out_config[i].is_lfe & !has_lfe_rendering))
          {
            numPairs++;
          }
          out_config[j].pair_type = HOA_SP_PAIR_NONE;
          out_config[j].symmetric_pair = NULL;

          out_config[j].is_already_used = 1;
          out_config[i].is_already_used = 1;
          break;
        }
      }

      if (!out_config[i].is_already_used)
      {
        out_config[i].is_already_used = 1;

        /* we did not found a symmetric L/R pair for speaker on position i */
        out_config[i].pair_type = HOA_SP_PAIR_SINGLE;
        out_config[i].symmetric_pair = NULL;
      }
    }
    else
    {
      out_config[i].is_already_used = 1;

      /* the speaker is in the center, it has no pair */
      out_config[i].pair_type = HOA_SP_PAIR_CENTER;
      out_config[i].symmetric_pair = NULL;
    }
  }

  for (i = 0; i < out_cnt; ++i)
  {
    out_config[i].is_already_used = 0;
  }
  return numPairs;
}

/**
 *  impegh_decode_hoa_matrix_data
 *
 *  \brief Decode HOA matrix data
 *
 *  \param [in/out] ptr_bit_buf     Pointer to bit buffer structure
 *  \param [in/out] ptr_matrix_data Pointer to HOA matrix structure
 *  \param [in]     num_hoa_coeff   Number of HOA coefficients
 *  \param [in]     out_cnt         Output count
 *  \param [in]     out_config      Pointer to speaker information structure
 *  \param [out]    hoa_matrix      Pointer to HOA matrix
 *
 *  \return VOID
 *
 */
static VOID impegh_decode_hoa_matrix_data(ia_bit_buf_struct *ptr_bit_buf,
                                          ia_hoa_matrix_struct *ptr_matrix_data,
                                          WORD32 num_hoa_coeff, WORD32 out_cnt,
                                          ia_speaker_info_str *out_config, FLOAT32 *hoa_matrix)
{
  WORD32 pair_idx, in_cnt = num_hoa_coeff, i, j = 0, has_value = 0, curr_hoa_order = 0, curr_idx;
  FLOAT32 curr_scalar;

  memset(ptr_matrix_data->is_value_symmetric, 0, sizeof(UWORD32) * out_cnt);
  memset(ptr_matrix_data->is_sign_symmetric, 0, sizeof(UWORD32) * out_cnt);
  for (i = 0; i < out_cnt; i++)
  {
    if ((out_config[i].symmetric_pair != NULL) &&
        (out_config[i].pair_type == HOA_SP_PAIR_SYMMETRIC))
    {
      if (0 == ((0 == ptr_matrix_data->has_lfe_rendering) && out_config[i].is_lfe))
      {
        ptr_matrix_data->is_value_symmetric[i] = ptr_matrix_data->value_symmetric_pairs[j];
        ptr_matrix_data->is_sign_symmetric[i] = ptr_matrix_data->sign_symmetric_pairs[j++];
      }
    }
  }
  for (i = 0; i < in_cnt; i++)
  {
    curr_hoa_order = ia_hoa_curr_order[i];
    for (j = out_cnt - 1; j >= 0; j--)
    {
      hoa_matrix[i * out_cnt + j] = 0.0;
      ptr_matrix_data->sign_matrix[i * out_cnt + j] = 1;
      if ((ptr_matrix_data->vert_bitmask[i] && ptr_matrix_data->has_vertical_coef) ||
          !(ptr_matrix_data->vert_bitmask[i]))
      {
        has_value = 1;
        if (0 != ptr_matrix_data->is_value_symmetric[j])
        {
          pair_idx = (out_config[j].symmetric_pair)
                         ? out_config[j].symmetric_pair->original_position
                         : 0;
          hoa_matrix[i * out_cnt + j] = hoa_matrix[i * out_cnt + pair_idx];
          ptr_matrix_data->sign_matrix[i * out_cnt + j] =
              ptr_matrix_data->sym_signs[i] *
              ptr_matrix_data->sign_matrix[i * out_cnt + pair_idx];
        }
        else if ((ptr_matrix_data->has_lfe_rendering && out_config[j].is_lfe) ||
                 (!out_config[j].is_lfe))
        {
          if (ptr_matrix_data->is_hoa_coef_sparse[i])
          {
            has_value = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
          }
          if (has_value)
          {
            FLOAT32 decode_hoa_gain_value = HOA_MATRIX_LINEAR_GAIN_ZERO;
            {
              WORD32 n_alphabet = ((ptr_matrix_data->max_gain[curr_hoa_order] -
                                    ptr_matrix_data->min_gain[curr_hoa_order])
                                   << ptr_matrix_data->precision_level) +
                                  2;
              WORD32 gain_value_index = impeghd_read_range(ptr_bit_buf, n_alphabet);

              decode_hoa_gain_value =
                  ptr_matrix_data->max_gain[curr_hoa_order] -
                  gain_value_index / (FLOAT32)(1 << ptr_matrix_data->precision_level);

              if (decode_hoa_gain_value >= ptr_matrix_data->min_gain[curr_hoa_order])
              {
                decode_hoa_gain_value = (FLOAT32)pow(10.0f, decode_hoa_gain_value / 20.0f);
              }
              else
              {
                decode_hoa_gain_value = HOA_MATRIX_LINEAR_GAIN_ZERO;
              }
            }
            hoa_matrix[i * out_cnt + j] = decode_hoa_gain_value;
            if (0 != ptr_matrix_data->is_sign_symmetric[j])
            {
              pair_idx = (out_config[j].symmetric_pair)
                             ? out_config[j].symmetric_pair->original_position
                             : 0;
              ptr_matrix_data->sign_matrix[i * out_cnt + j] =
                  ptr_matrix_data->sym_signs[i] *
                  ptr_matrix_data->sign_matrix[i * out_cnt + pair_idx];
            }
            else if ((hoa_matrix[i * out_cnt + j] != HOA_MATRIX_LINEAR_GAIN_ZERO) &&
                     (curr_hoa_order || !(ptr_matrix_data->zeroth_order_always_positive)))
            {
              WORD32 sign_val = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
              ptr_matrix_data->sign_matrix[i * out_cnt + j] = sign_val * 2 - 1;
            }
          }
        }
      }
    }
  }
  curr_idx = 0;
  for (i = 0; i < in_cnt; i++)
  {
    curr_hoa_order = ia_hoa_curr_order[i];
    curr_scalar = ia_hoa_curr_scalar[curr_hoa_order];
    for (j = 0; j < out_cnt; j++)
    {
      hoa_matrix[curr_idx] *= ptr_matrix_data->sign_matrix[curr_idx];
      hoa_matrix[curr_idx] = ia_mul_flt(hoa_matrix[curr_idx], curr_scalar);
      curr_idx++;
    }
  }

  if (1 == ptr_matrix_data->is_normalized)
  {
    curr_scalar = 0.0;
    curr_idx = 0;
    /*compute Frobenius Norm:*/
    for (i = 0; i < in_cnt; i++)
    {
      for (j = 0; j < out_cnt; j++)
      {
        if (!out_config[j].is_lfe)
        {
          curr_scalar = ia_mac_flt(curr_scalar, hoa_matrix[curr_idx], hoa_matrix[curr_idx]);
        }
        curr_idx++;
      }
    }
    curr_scalar = (FLOAT32)(1.0 / ia_sqrt_flt(curr_scalar));
    curr_idx = 0;
    /* normalize with inverse Frobenius Norm: */
    for (i = 0; i < in_cnt; i++)
    {
      for (j = 0; j < out_cnt; j++)
      {
        if (!out_config[j].is_lfe)
        {
          hoa_matrix[curr_idx] = ia_mul_flt(hoa_matrix[curr_idx], curr_scalar);
        }
        curr_idx++;
      }
    }
  }
}

/**
 *  impeghd_hoa_rendering_matrix
 *
 *  \brief Fetch HOA rendering matrix
 *
 *  \param [in/out] ptr_bit_buf     Pointer to bit buffer structure
 *  \param [out]    ptr_matrix_data Pointer to HOA matrix structure
 *  \param [in]     num_hoa_coeff   Number of HOA coefficients
 *  \param [in]     out_cnt         Output count
 *  \param [out]    out_config      Pointer to speaker information structure
 *  \param [out]    hoa_matrix      Pointer to HOA matrix
 *
 *  \return VOID
 *
 */
static IA_ERRORCODE impeghd_hoa_rendering_matrix(ia_bit_buf_struct *ptr_bit_buf,
                                                 ia_hoa_matrix_struct *ptr_matrix_data,
                                                 WORD32 num_hoa_coeff, WORD32 out_cnt,
                                                 ia_speaker_info_str *out_config,
                                                 FLOAT32 *hoa_matrix)
{
  WORD32 num_pairs = 0, lfe_exist = 0, i;
  WORD32 max_hoa_order = (WORD32)(ia_sqrt_flt(num_hoa_coeff) - 1);
  WORD32 input_count = num_hoa_coeff;
  WORD32 n, m, k = 0;
  ptr_matrix_data->has_lfe_rendering = 0;

  for (n = 0; n <= max_hoa_order; ++n)
  {
    for (m = -n; m <= n; ++m)
    {
      ptr_matrix_data->sym_signs[k] = ((m >= 0) * 2) - 1;    // Create symbol signs
      ptr_matrix_data->vert_bitmask[k] = ia_abs_int(m) != n; // Create 2D bit-mask
      k++;
    }
  }
  ptr_matrix_data->precision_level = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
  ptr_matrix_data->is_normalized = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  ptr_matrix_data->gain_limit_per_hoa_order = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  if (1 != ptr_matrix_data->gain_limit_per_hoa_order)
  {
    UWORD32 val = 0;
    ia_core_coder_read_escape_value(ptr_bit_buf, &val, 3, 5, 6);
    ptr_matrix_data->max_gain[0] = -((WORD32)val);

    ia_core_coder_read_escape_value(ptr_bit_buf, &val, 4, 5, 6);
    ptr_matrix_data->min_gain[0] = -((WORD32)val + 1 - ptr_matrix_data->max_gain[0]);

    for (i = 0; i < (max_hoa_order + 1); i++)
    {
      ptr_matrix_data->max_gain[i] = ptr_matrix_data->max_gain[0];
      ptr_matrix_data->min_gain[i] = ptr_matrix_data->min_gain[0];
    }
  }
  else
  {
    for (i = 0; i < (max_hoa_order + 1); i++)
    {
      UWORD32 val = 0;
      ia_core_coder_read_escape_value(ptr_bit_buf, &val, 3, 5, 6);
      ptr_matrix_data->max_gain[i] = -((WORD32)val);

      ia_core_coder_read_escape_value(ptr_bit_buf, &val, 4, 5, 6);
      ptr_matrix_data->min_gain[i] = -((WORD32)val + 1 - ptr_matrix_data->max_gain[i]);
    }
  }
  ptr_matrix_data->is_full_matrix = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  if (ptr_matrix_data->is_full_matrix)
  {
    memset(ptr_matrix_data->is_hoa_coef_sparse, 0, input_count * sizeof(UWORD32));
  }
  else
  {
    WORD32 n_bits_hoa_order = impeghd_hoa_get_ceil_log2(max_hoa_order + 1);
    ptr_matrix_data->first_sparse_order =
        ia_core_coder_read_bits_buf(ptr_bit_buf, n_bits_hoa_order);

    /*firstSparseOrder shall be smaller than or equal to maxHoaOrder*/
    if (ptr_matrix_data->first_sparse_order > (UWORD32)max_hoa_order)
    {
      return IA_MPEGH_HOA_INIT_FATAL_INVALID_SPARSE_ORD;
    }

    memset(ptr_matrix_data->is_hoa_coef_sparse, 0,
           (ptr_matrix_data->first_sparse_order * ptr_matrix_data->first_sparse_order) *
               sizeof(UWORD32));
    for (i = (ptr_matrix_data->first_sparse_order * ptr_matrix_data->first_sparse_order);
         i < input_count; i++)
    {
      ptr_matrix_data->is_hoa_coef_sparse[i] = 1;
    }
  }

  for (i = 0; i < out_cnt; i++)
  {
    if (out_config[i].is_lfe)
    {
      lfe_exist = 1;
      break;
    }
  }
  if (lfe_exist)
  {
    ptr_matrix_data->has_lfe_rendering = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  }

  num_pairs =
      impeghd_find_symmetric_speakers(out_cnt, out_config, ptr_matrix_data->has_lfe_rendering);

  memset(ptr_matrix_data->sign_symmetric_pairs, 0, num_pairs * sizeof(UWORD32));
  memset(ptr_matrix_data->value_symmetric_pairs, 0, num_pairs * sizeof(UWORD32));

  ptr_matrix_data->zeroth_order_always_positive = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  ptr_matrix_data->is_all_value_symmetric = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (!ptr_matrix_data->is_all_value_symmetric)
  {
    ptr_matrix_data->is_any_value_symmetric = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    if (!ptr_matrix_data->is_any_value_symmetric)
    {
      ptr_matrix_data->is_all_sign_symmetric = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (!ptr_matrix_data->is_all_sign_symmetric)
      {
        ptr_matrix_data->is_any_sign_symmetric = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        if (ptr_matrix_data->is_any_sign_symmetric)
        {
          UWORD32 bool_val = 0;
          for (i = 0; i < num_pairs; i++)
          {
            bool_val = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
            ptr_matrix_data->sign_symmetric_pairs[i] = bool_val;
          }
        }
      }
      if (ptr_matrix_data->is_all_sign_symmetric)
      {
        for (i = 0; i < num_pairs; i++)
        {
          ptr_matrix_data->sign_symmetric_pairs[i] = 1;
        }
      }
    }
    else
    {
      UWORD32 bool_val = 0;
      for (i = 0; i < num_pairs; i++)
      {
        bool_val = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        ptr_matrix_data->value_symmetric_pairs[i] = bool_val;
      }
      ptr_matrix_data->is_any_sign_symmetric = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      if (ptr_matrix_data->is_any_sign_symmetric)
      {
        for (i = 0; i < num_pairs; i++)
        {
          if (0 == ptr_matrix_data->value_symmetric_pairs[i])
          {
            bool_val = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
            ptr_matrix_data->sign_symmetric_pairs[i] = bool_val;
          }
        }
      }
    }
  }
  else
  {
    for (i = 0; i < num_pairs; i++)
    {
      ptr_matrix_data->value_symmetric_pairs[i] = 1;
    }
  }
  ptr_matrix_data->has_vertical_coef = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  impegh_decode_hoa_matrix_data(ptr_bit_buf, ptr_matrix_data, num_hoa_coeff, out_cnt, out_config,
                                hoa_matrix);
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_hoa_matrix_wrapper
 *
 *  \brief Fetch HOA matrix wrapper
 *
 *  \param [in/out] ptr_bit_buf     Pointer to bit buffer structure
 *  \param [in]     num_hoa_coeff   Number of HOA coefficients
 *  \param [out]    hoa_matrix      Pointer to HOA matrix
 *  \param [out]    ptr_matrix_data Pointer to HOA matrix structure
 *
 *  \return IA_ERRORCODE  error code
 *
 */
static IA_ERRORCODE impeghd_hoa_matrix_wrapper(ia_bit_buf_struct *ptr_bit_buf,
                                               WORD32 num_hoa_coeff, FLOAT32 *hoa_matrix,
                                               ia_hoa_matrix_struct *ptr_matrix_data)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  ia_speaker_info_str output_config[HOA_MATRIX_MAX_SPEAKER_COUNT];
  WORD32 spk_idx = ptr_matrix_data->cicp_speaker_layout_idx; // Speaker ID
  WORD32 output_count = impgehd_cicp_get_num_ls[spk_idx];
  const WORD32 *speaker_table = ia_cicp_idx_ls_set_map_tbl[spk_idx];
  WORD32 idx;
  for (WORD32 i = output_count - 1; i >= 0; i--)
  {
    idx = speaker_table[i];
    output_config[i].is_lfe = (WORD16)ia_cicp_ls_geo_tbls[idx].lfe_flag;
    output_config[i].elevation = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_elevation;
    output_config[i].azimuth = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_azimuth;
  }

  err_code =
      impeghd_hoa_rendering_matrix(ptr_bit_buf, ptr_matrix_data, num_hoa_coeff, output_count,
                                   (ia_speaker_info_str *)&output_config, hoa_matrix);
  return err_code;
}

/**
 *  impeghd_hoa_matrix
 *
 *  \brief Fetch HOA matrix
 *
 *  \param [in/out] ptr_bit_buf         Pointer to bit buffer structure
 *  \param [in]     num_hoa_coeff       Number of HOA coefficients
 *  \param [in]     cicp_id             CICP ID
 *  \param [out]    is_matrix_present   Matrix present flag
 *  \param [out]    hoa_matrix          Pointer to HOA matrix
 *  \param [in]     scratch             Pointer to scratch buffer
 *
 *  \return IA_ERRORCODE                Error
 *
 */
IA_ERRORCODE impeghd_hoa_matrix(ia_bit_buf_struct *ptr_bit_buf, WORD32 num_hoa_coeff,
                                WORD32 cicp_id, pWORD32 is_matrix_present, WORD32 *ptr_matrix_idx,
                                FLOAT32 *hoa_matrix, ia_hoa_matrix_struct *ptr_matrix_data)
{
  IA_ERRORCODE err;
  ptr_matrix_data->num_of_hoa_rendering_matrices = ia_core_coder_read_bits_buf(ptr_bit_buf, 5);
  for (UWORD32 k = 0; k < ptr_matrix_data->num_of_hoa_rendering_matrices; k++)
  {
    ptr_matrix_data->hoa_rendering_matrix_id[k] = ia_core_coder_read_bits_buf(ptr_bit_buf, 7);
    ptr_matrix_data->cicp_speaker_layout_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 6);

    /* ISO/IEC 23091-3 section 6.2 (21-63 reserved)*/
    if (ptr_matrix_data->cicp_speaker_layout_idx > MAX_CICP_INDEX)
    {
      return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_CICP_LAYOUT_INDEX;
    }

    ia_core_coder_read_escape_value(ptr_bit_buf, &(ptr_matrix_data->hoa_matrix_len_bits), 8, 8,
                                    12);

    if (cicp_id > CICP2GEOMETRY_MAX_SPKIDX)
      return IA_MPEGH_HOA_INIT_FATAL_RENDER_INIT_FAILED;

    if ((UWORD32)cicp_id != ptr_matrix_data->cicp_speaker_layout_idx)
    {
      ia_core_coder_skip_bits_buf(ptr_bit_buf, ptr_matrix_data->hoa_matrix_len_bits);
    }
    else
    {
      *ptr_matrix_idx = k;
      *is_matrix_present = 1;
      err = impeghd_hoa_matrix_wrapper(ptr_bit_buf, num_hoa_coeff, hoa_matrix, ptr_matrix_data);
      if (err)
        return err;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */