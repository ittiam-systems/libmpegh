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

#define CHSIGN(v1, v2) ((v2) >= 0.0 ? ia_fabs_flt(v1) : -ia_fabs_flt(v1))

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_ren_simple_mtrx_init_with_size
 *
 *  \brief Initialize matrix row and column with given size,
 *      Set default value of matrix to zero
 *
 *  \param [out]  handle  Matrix handle
 *  \param [in]    rows  Number of rows
 *  \param [in]    cols  Number of columns
 *
 *
 *
 */
VOID impeghd_hoa_ren_simple_mtrx_init_with_size(pVOID handle, WORD32 rows, WORD32 cols)
{
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)handle;
  ia_core_coder_memset(sm_handle->mtrx, HOA_MAX_MATRIX_SIZE);
  sm_handle->rows = rows;
  sm_handle->cols = cols;
}

/**
*  impeghd_hoa_ren_simple_mtrx_init_with_matrix
*
*  \brief Initialize matrix with given matrix
*
*  \param [in]  handle_og  Input matrix
*  \param [out]  handle    Output matrix
*
*
*
*/
VOID impeghd_hoa_ren_simple_mtrx_init_with_matrix(pVOID handle_og, pVOID handle)
{
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)handle;
  ia_render_hoa_simple_mtrx_str *sm_handle_og = (ia_render_hoa_simple_mtrx_str *)handle_og;
  sm_handle->rows = sm_handle_og->rows;
  sm_handle->cols = sm_handle_og->cols;

  ia_core_coder_mem_cpy(sm_handle_og->mtrx, sm_handle->mtrx, sm_handle->rows * sm_handle->cols);
}

/**
 *  impeghd_hoa_ren_simple_mtrx_transpose_keep
 *
 *  \brief Store the transpose of matrix, to new matrix.
 *
 *  \param [in]    handle_og  Input matrix
 *  \param [out]  handle    Output matrix
 *
 *
 *
 */
VOID impeghd_hoa_ren_simple_mtrx_transpose_keep(pVOID handle_og, pVOID handle)
{
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)handle;
  ia_render_hoa_simple_mtrx_str *sm_handle_og = (ia_render_hoa_simple_mtrx_str *)handle_og;
  sm_handle->rows = sm_handle_og->cols;
  sm_handle->cols = sm_handle_og->rows;
  for (WORD32 c = 0; c < sm_handle_og->cols; c++)
  {
    for (WORD32 r = 0; r < sm_handle_og->rows; r++)
    {
      sm_handle->mtrx[c * sm_handle->cols + r] = sm_handle_og->mtrx[r * sm_handle_og->cols + c];
    }
  }
}

/**
 *  impeghd_hoa_ren_simple_mtrx_transpose
 *
 *  \brief Transpose matrix and store in same matrix using temproary matrix.
 *
 *  \param [in,out]  handle  Matrix for inplace transpose
 *  \param [out]  scratch  Pointer to scratch buffer for intermediate processing
 *
 *
 *
 */
VOID impeghd_hoa_ren_simple_mtrx_transpose(pVOID handle, pVOID scratch)
{
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)handle;
  pFLOAT32 ptr_tmp = (pFLOAT32)scratch;
  for (WORD32 c = 0; c < sm_handle->cols; c++)
  {
    for (WORD32 r = 0; r < sm_handle->rows; r++)
    {
      ptr_tmp[c * sm_handle->rows + r] = sm_handle->mtrx[r * sm_handle->cols + c];
    }
  }
  WORD32 t = sm_handle->cols;
  sm_handle->cols = sm_handle->rows;
  sm_handle->rows = t;
  ia_core_coder_mem_cpy(ptr_tmp, sm_handle->mtrx, ((sm_handle->rows) * (sm_handle->cols)));
}

/**
*  impeghd_hoa_ren_simple_mtrx_mult
*
*  \brief Multiplication of matrixes and store in new matrix.
*
*  \param [in]  handle_1  Matrix A for multiplication
*  \param [in]  handle_2  Matrix B for multiplication
*  \param [out]  handle_mult  Matrix to store multiplicaiton
*  \param [in]  scratch    Pointer to scratch buffer for intermediate processing
*
*  \return IA_ERRORCODE  Error code
*
*/
IA_ERRORCODE impeghd_hoa_ren_simple_mtrx_mult(pVOID handle_1, pVOID handle_2, pVOID handle_mult,
                                              pVOID scratch)
{
  ia_render_hoa_simple_mtrx_str *sm_handle_mult = (ia_render_hoa_simple_mtrx_str *)scratch;
  ia_render_hoa_simple_mtrx_str *sm_handle_ans = (ia_render_hoa_simple_mtrx_str *)handle_mult;
  ia_render_hoa_simple_mtrx_str *sm_handle_2 = (ia_render_hoa_simple_mtrx_str *)handle_2;
  ia_render_hoa_simple_mtrx_str *sm_handle_1 = (ia_render_hoa_simple_mtrx_str *)handle_1;

  if (sm_handle_1->cols != sm_handle_2->rows)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_MATRIX_MISMATCH;
  }
  else if (NULL == sm_handle_mult)
  {
    sm_handle_mult = sm_handle_ans;
  }

  sm_handle_mult->cols = sm_handle_2->cols;
  sm_handle_mult->rows = sm_handle_1->rows;

  for (WORD32 r = 0; r < sm_handle_mult->rows; r++)
  {
    for (WORD32 c = 0; c < sm_handle_mult->cols; c++)
    {
      FLOAT32 sum = 0;
      for (WORD32 i = 0; i < sm_handle_1->cols; i++)
      {
        sum = ia_mac_flt(sum, sm_handle_2->mtrx[i * sm_handle_2->cols + c],
                         sm_handle_1->mtrx[r * sm_handle_1->cols + i]);
      }
      sm_handle_mult->mtrx[r * sm_handle_mult->cols + c] = sum;
    }
  }
  if (scratch)
    impeghd_hoa_ren_simple_mtrx_init_with_matrix(sm_handle_mult, sm_handle_ans);
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_hoa_ren_simple_mtrx_diag_mult
*
*  \brief Multiplication of matrixes by diagnoal matrix with only diagonal array.
*
*  \param [out]  handle  Matrix handle
*  \param [in]  arr    Diagonal array
*
*
*
*/
VOID impeghd_hoa_ren_simple_mtrx_diag_mult(pVOID handle, pVOID arr, WORD32 diag_len)
{
  WORD32 col, row;
  ia_render_hoa_simple_mtrx_str *data = (ia_render_hoa_simple_mtrx_str *)handle;
  pFLOAT32 diag_arr = (pFLOAT32)arr;

  for (row = 0; row < data->rows; row++)
  {
    ia_core_coder_memset(&data->mtrx[row * data->cols + diag_len], (data->cols - diag_len));
  }
  for (row = 0; row < data->rows; row++)
  {
    for (col = diag_len - 1; col >= 0; col--)
    {
      data->mtrx[row * data->cols + col] =
          ia_mul_flt(data->mtrx[row * data->cols + col], diag_arr[col]);
    }
  }
}

/**
*  impeghd_hoa_ren_simple_mtrx_norm_fro
*
*  \brief Calcluates the normal of matrix
*
*  \param [in]  handle  Matrix handle
*
*  \return FLOAT32  Returns normal of matrix
*
*/
FLOAT32 impeghd_hoa_ren_simple_mtrx_norm_fro(pVOID handle)
{
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)handle;
  FLOAT32 norm = 0.0;
  WORD32 rc = 0;
  while (rc < sm_handle->cols * sm_handle->rows)
  {
    norm = ia_mac_flt(norm, sm_handle->mtrx[rc], sm_handle->mtrx[rc]);
    rc++;
  }
  return (FLOAT32)ia_sqrt_flt(norm);
}

/**
 *  impeghd_hoa_ren_simple_mtrx_swap_cols
 *
 *  \brief Reaggrange matrix columns with given set of columns
 *
 *  \param [in,out]  handle  Matrix handle
 *  \param [in]    idx    Array of indices
 *  \param [in]    scratch  Pointer to scratch buffer for intermediate processing
 *
 *
 *
 */
static VOID impeghd_hoa_ren_simple_mtrx_swap_cols(pVOID handle, pWORD32 idx, pVOID scratch)
{
  pFLOAT32 ptr_m2 = (pFLOAT32)scratch;
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)handle;
  for (WORD32 rc = 0; rc < (sm_handle->rows) * (sm_handle->cols); rc++)
    ptr_m2[rc] = 0.0;
  for (WORD32 c = 0; c < sm_handle->cols; c++)
  {
    for (WORD32 r = 0; r < sm_handle->rows; r++)
    {
      ptr_m2[r * sm_handle->cols + c] = sm_handle->mtrx[r * sm_handle->cols + idx[c]];
    }
  }
  ia_core_coder_mem_cpy(ptr_m2, sm_handle->mtrx, (sm_handle->rows) * (sm_handle->cols));
}

/**
 *  impeghd_hoa_get_sort_idx
 *
 *  \brief Bubble sorting and stores sorted index in given array.
 *
 *  \param [in]    arr  Array to be sorted
 *  \param [in]    n  Size of array
 *  \param [out]  idx  Sorted array index sequence of original array
 *
 *
 *
 */
static VOID impeghd_hoa_get_sort_idx(const pFLOAT32 arr, WORD32 n, pWORD32 idx)
{
  WORD32 i, j, ival;
  for (i = 0; i < n; i++)
  {
    idx[i] = i;
  }

  for (i = 0; i < n; i++)
  {
    for (j = i + 1; j < n; j++)
    {
      if (arr[idx[i]] < arr[idx[j]])
      {
        ival = idx[i];
        idx[i] = idx[j];
        idx[j] = ival;
      }
    }
  }
}

/**
 *  impeghd_hoa_pythagoras
 *
 *  \brief Calculates pythagoras value
 *
 *  \param [in]  v1  Value a
 *  \param [in]  v2  Value b
 *
 *  \return FLOAT32  Pythagoras value
 *
 */
static FLOAT32 impeghd_hoa_pythagoras(FLOAT32 v1, FLOAT32 v2)
{
  FLOAT32 ct, res;
  FLOAT32 bt = (FLOAT32)ia_fabs_flt(v2);
  FLOAT32 at = (FLOAT32)ia_fabs_flt(v1);

  if (at > bt)
  {
    ct = bt / at;
    res = (FLOAT32)(ia_mul_double_flt(ia_sqrt_flt(ia_mul_flt(ct, ct) + 1.0f), at));
  }
  else if (bt > 0.0)
  {
    ct = at / bt;
    res = (FLOAT32)(ia_mul_double_flt(ia_sqrt_flt(ia_mul_flt(ct, ct) + 1.0f), bt));
  }
  else
    res = 0.0;
  return (res);
}

/**
 *  impeghd_hoa_dsvd
 *
 *  \brief Singular value decomposition
 *
 *  \param [out]  handle_a  Matix U
 *  \param [in]    rows    Number of rows
 *  \param [in]    cols    Number of columns
 *  \param [out]  w      Values array of diagonal matrix
 *  \param [out]  handle_v  Matix V
 *  \param [in]    scratch Pointer to scratch buffer for intermediate processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
static IA_ERRORCODE impeghd_hoa_dsvd(pVOID handle_a, WORD32 rows, WORD32 cols, pFLOAT32 w,
                                     pVOID handle_v, pVOID scratch)
{
  pFLOAT32 rv1 = (FLOAT32 *)scratch;
  FLOAT32 anorm = 0.0, g = 0.0, scale = 0.0;
  FLOAT32 c, h, s, x, y, z;
  FLOAT64 f;
  WORD32 flag, i, its, j, jj, k, nm = 0, l = 0;
  ia_render_hoa_simple_mtrx_str *sm_handle_v = (ia_render_hoa_simple_mtrx_str *)handle_v;
  ia_render_hoa_simple_mtrx_str *sm_handle_a = (ia_render_hoa_simple_mtrx_str *)handle_a;
  memset(scratch, 0, sizeof(FLOAT32) * cols);
  for (i = 0; i < cols; i++)
  {
    l = i + 1;
    rv1[i] = scale * g;
    g = s = scale = 0.0;
    if (i < rows)
    {
      for (k = i; k < rows; k++)
        scale += (FLOAT32)ia_fabs_flt(sm_handle_a->mtrx[k * sm_handle_a->cols + i]);
      if (scale)
      {
        for (k = i; k < rows; k++)
        {
          sm_handle_a->mtrx[k * sm_handle_a->cols + i] /= scale;
          s = ia_mac_flt(s, sm_handle_a->mtrx[k * sm_handle_a->cols + i],
                         sm_handle_a->mtrx[k * sm_handle_a->cols + i]);
        }
        f = sm_handle_a->mtrx[i * sm_handle_a->cols + i];
        g = (FLOAT32)(-CHSIGN(ia_sqrt_flt(s), f));
        h = ia_sub_flt(ia_mul_flt(f, g), s);
        sm_handle_a->mtrx[i * sm_handle_a->cols + i] = ia_sub_flt(f, g);
        if (i != cols - 1)
        {
          for (j = l; j < cols; j++)
          {
            for (s = 0.0, k = i; k < rows; k++)
              s = ia_mac_flt(s, sm_handle_a->mtrx[k * sm_handle_a->cols + i],
                             sm_handle_a->mtrx[k * sm_handle_a->cols + j]);
            f = s / h;
            for (k = i; k < rows; k++)
              sm_handle_a->mtrx[k * sm_handle_a->cols + j] =
                  ia_mac_flt(sm_handle_a->mtrx[k * sm_handle_a->cols + j],
                             sm_handle_a->mtrx[k * sm_handle_a->cols + i], f);
          }
        }
        for (k = i; k < rows; k++)
          sm_handle_a->mtrx[k * sm_handle_a->cols + i] =
              ia_mul_flt(scale, sm_handle_a->mtrx[k * sm_handle_a->cols + i]);
      }
    }
    w[i] = (scale * g);
    g = s = scale = 0.0;
    if (i != cols - 1 && i < rows)
    {
      for (k = l; k < cols; k++)
        scale =
            ia_add_flt(scale, (FLOAT32)ia_fabs_flt(sm_handle_a->mtrx[i * sm_handle_a->cols + k]));
      if (scale)
      {
        for (k = l; k < cols; k++)
        {
          sm_handle_a->mtrx[i * sm_handle_a->cols + k] /= scale;
          s = ia_mac_flt(s, sm_handle_a->mtrx[i * sm_handle_a->cols + k],
                         sm_handle_a->mtrx[i * sm_handle_a->cols + k]);
        }
        f = sm_handle_a->mtrx[i * sm_handle_a->cols + l];
        g = (FLOAT32)(-CHSIGN(ia_sqrt_flt(s), f));
        h = ia_sub_flt(ia_mul_flt(f, g), s);
        sm_handle_a->mtrx[i * sm_handle_a->cols + l] = ia_sub_flt(f, g);
        for (k = l; k < cols; k++)
          rv1[k] = sm_handle_a->mtrx[i * sm_handle_a->cols + k] / h;
        if (i != rows - 1)
        {
          for (j = l; j < rows; j++)
          {
            for (s = 0.0, k = l; k < cols; k++)
              s = ia_mac_flt(s, sm_handle_a->mtrx[j * sm_handle_a->cols + k],
                             sm_handle_a->mtrx[i * sm_handle_a->cols + k]);
            for (k = l; k < cols; k++)
              sm_handle_a->mtrx[j * sm_handle_a->cols + k] =
                  ia_mac_flt(sm_handle_a->mtrx[j * sm_handle_a->cols + k], rv1[k], s);
          }
        }
        for (k = l; k < cols; k++)
          sm_handle_a->mtrx[i * sm_handle_a->cols + k] =
              ia_mul_flt(sm_handle_a->mtrx[i * sm_handle_a->cols + k], scale);
      }
    }
    anorm = (FLOAT32)(ia_max_int(anorm, (ia_fabs_flt(w[i]) + ia_fabs_flt(rv1[i]))));
  }
  for (i = cols - 1; i >= 0; i--)
  {
    if (i < cols - 1)
    {
      if (g)
      {
        for (j = l; j < cols; j++)
          sm_handle_v->mtrx[j * sm_handle_v->cols + i] =
              (sm_handle_a->mtrx[i * sm_handle_a->cols + j] /
               sm_handle_a->mtrx[i * sm_handle_a->cols + l]) /
              g;
        for (j = l; j < cols; j++)
        {
          for (s = 0.0, k = l; k < cols; k++)
            s = ia_mac_flt(s, sm_handle_v->mtrx[k * sm_handle_v->cols + j],
                           sm_handle_a->mtrx[i * sm_handle_a->cols + k]);
          for (k = l; k < cols; k++)
            sm_handle_v->mtrx[k * sm_handle_v->cols + j] =
                ia_mac_flt(sm_handle_v->mtrx[k * sm_handle_v->cols + j],
                           sm_handle_v->mtrx[k * sm_handle_v->cols + i], s);
        }
      }
      for (j = l; j < cols; j++)
      {
        sm_handle_v->mtrx[j * sm_handle_v->cols + i] =
            sm_handle_v->mtrx[i * sm_handle_v->cols + j] = 0.0;
      }
    }
    sm_handle_v->mtrx[i * sm_handle_v->cols + i] = 1.0;
    g = rv1[i];
    l = i;
  }
  for (i = cols - 1; i >= 0; i--)
  {
    l = i + 1;
    g = w[i];
    if (i < cols - 1)
      for (j = l; j < cols; j++)
        sm_handle_a->mtrx[i * sm_handle_a->cols + j] = 0.0;

    if (!g)
    {
      for (j = i; j < rows; j++)
        sm_handle_a->mtrx[j * sm_handle_a->cols + i] = 0.0;
    }
    else
    {
      g = 1.0f / g;
      if (i != cols - 1)
      {
        for (j = l; j < cols; j++)
        {
          for (s = 0.0, k = l; k < rows; k++)
            s = ia_mac_flt(s, sm_handle_a->mtrx[k * sm_handle_a->cols + j],
                           sm_handle_a->mtrx[k * sm_handle_a->cols + i]);
          f = (s / sm_handle_a->mtrx[i * sm_handle_a->cols + i]) * g;
          for (k = i; k < rows; k++)
            sm_handle_a->mtrx[k * sm_handle_a->cols + j] =
                ia_mac_flt(sm_handle_a->mtrx[k * sm_handle_a->cols + j],
                           sm_handle_a->mtrx[k * sm_handle_a->cols + i], f);
        }
      }
      for (j = i; j < rows; j++)
        sm_handle_a->mtrx[j * sm_handle_a->cols + i] =
            ia_mul_flt(sm_handle_a->mtrx[j * sm_handle_a->cols + i], g);
    }
    ++sm_handle_a->mtrx[i * sm_handle_a->cols + i];
  }
  for (k = cols - 1; k >= 0; k--)
  {
    for (its = 0; its < 30; its++)
    {
      flag = 1;
      for (l = k; l >= 0; l--)
      {
        nm = l - 1;
        if (ia_fabs_flt(rv1[l]) + anorm == anorm)
        {
          flag = 0;
          break;
        }
        if (ia_fabs_flt(w[nm]) + anorm == anorm)
          break;
      }
      if (flag)
      {
        s = 1.0;
        for (i = l; i <= k; i++)
        {
          f = ia_mul_flt(rv1[i], s);
          if (ia_fabs_flt(f) + anorm != anorm)
          {
            g = w[i];
            h = impeghd_hoa_pythagoras(f, g);
            w[i] = h;
            h = 1.0f / h;
            c = ia_mul_flt(h, g);
            s = ia_mul_flt(h, -f);
            for (j = 0; j < rows; j++)
            {
              z = sm_handle_a->mtrx[j * sm_handle_a->cols + i];
              y = sm_handle_a->mtrx[j * sm_handle_a->cols + nm];
              sm_handle_a->mtrx[j * sm_handle_a->cols + i] =
                  ia_sub_flt(ia_mul_flt(c, z), ia_mul_flt(s, y));
              sm_handle_a->mtrx[j * sm_handle_a->cols + nm] =
                  ia_add_flt(ia_mul_flt(s, z), ia_mul_flt(c, y));
            }
          }
        }
      }
      z = w[k];
      if (l == k)
      {
        if (z < 0.0)
        {
          w[k] = (-z);
          for (j = 0; j < cols; j++)
            sm_handle_v->mtrx[j * sm_handle_v->cols + k] =
                -sm_handle_v->mtrx[j * sm_handle_v->cols + k];
        }
        break;
      }
      x = w[l];
      nm = k - 1;
      y = w[nm];
      g = rv1[nm];
      h = rv1[k];
      f = (FLOAT32)(ia_mul_flt(ia_add_flt(y, z), ia_sub_flt(y, z)) +
                    ia_mul_flt(ia_add_flt(g, h), ia_sub_flt(g, h)) / (2.0f * ia_mul_flt(h, y)));
      g = impeghd_hoa_pythagoras(f, 1.0f);
      f = (FLOAT32)(
          (ia_mul_flt((x - z), (x + z)) + ia_mul_flt(h, ((y / (f + CHSIGN(g, f))) - h))) / x);
      c = s = 1.0;
      for (j = l; j <= nm; j++)
      {
        i = j + 1;
        g = rv1[i];
        y = w[i];
        h = ia_mul_flt(g, s);
        g = ia_mul_flt(g, c);
        z = impeghd_hoa_pythagoras(f, h);
        rv1[j] = z;
        s = h / z;
        c = f / z;
        f = ia_add_flt(ia_mul_flt(s, g), ia_mul_flt(c, x));
        g = ia_sub_flt(ia_mul_flt(c, g), ia_mul_flt(s, x));
        h = ia_mul_flt(s, y);
        y = ia_mul_flt(c, y);
        for (jj = 0; jj < cols; jj++)
        {
          z = sm_handle_v->mtrx[jj * sm_handle_v->cols + i];
          x = sm_handle_v->mtrx[jj * sm_handle_v->cols + j];
          sm_handle_v->mtrx[jj * sm_handle_v->cols + j] =
              ia_add_flt(ia_mul_flt(s, z), ia_mul_flt(c, x));
          sm_handle_v->mtrx[jj * sm_handle_v->cols + i] =
              ia_sub_flt(ia_mul_flt(c, z), ia_mul_flt(s, x));
        }
        z = impeghd_hoa_pythagoras(f, h);
        w[j] = z;
        if (z)
        {
          z = 1.0f / z;
          c = ia_mul_flt(z, f);
          s = ia_mul_flt(z, h);
        }
        f = ia_add_flt(ia_mul_flt(s, y), ia_mul_flt(c, g));
        x = ia_sub_flt(ia_mul_flt(c, y), ia_mul_flt(s, g));
        for (jj = 0; jj < rows; jj++)
        {
          z = sm_handle_a->mtrx[jj * sm_handle_a->cols + i];
          y = sm_handle_a->mtrx[jj * sm_handle_a->cols + j];
          sm_handle_a->mtrx[jj * sm_handle_a->cols + j] =
              ia_add_flt(ia_mul_flt(z, s), ia_mul_flt(y, c));
          sm_handle_a->mtrx[jj * sm_handle_a->cols + i] =
              ia_sub_flt(ia_mul_flt(z, c), ia_mul_flt(y, s));
        }
      }
      rv1[l] = 0.0;
      rv1[k] = f;
      w[k] = x;
    }
  }
  return (IA_MPEGH_DEC_NO_ERROR);
}
/**
 *  impeghd_hoa_ren_simple_mtrx_svd_init
 *
 *  \brief Initialize simple matrix for svd
 *
 *  \param [in,out]  svd_handle    Matrix svd handle to init
 *  \param [out]  sm_handle_og  Matrix handle
 *  \param [in]    scratch      Pointer to scratch buffer for intermediate
 * processing
 *
 *  \return IA_ERRORCODE Error code
 *
 */
IA_ERRORCODE impeghd_hoa_ren_simple_mtrx_svd_init(ia_render_hoa_simple_mtrx_svd_str *svd_handle,
                                                  ia_render_hoa_simple_mtrx_str *sm_handle_og,
                                                  pVOID scratch)
{
  WORD32 rows, cols, transpose_flag = 0, scratch_idx = 0, i;
  pWORD8 buf = (pWORD8)scratch;
  ia_render_hoa_simple_mtrx_str *sm_handle = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  pFLOAT32 wsx;
  pWORD32 idx;

  impeghd_hoa_ren_simple_mtrx_init_with_matrix(sm_handle_og, sm_handle);
  if (sm_handle->rows < sm_handle->cols)
  {
    impeghd_hoa_ren_simple_mtrx_transpose(sm_handle, (buf + scratch_idx));
    transpose_flag = 1;
  }
  cols = sm_handle->cols;
  rows = sm_handle->rows;

  wsx = (FLOAT32 *)(buf + scratch_idx);
  scratch_idx += sizeof(FLOAT32) * (HOA_MAX_MATRIX_SIZE);
  ia_core_coder_memset(wsx, HOA_MAX_MATRIX_SIZE);
  svd_handle->v.cols = svd_handle->v.rows = svd_handle->w_s_sz = cols;

  if (impeghd_hoa_dsvd(sm_handle, rows, cols, wsx, &(svd_handle->v), (buf + scratch_idx)) != 0)
  {
    return IA_MPEGH_HOA_INIT_NONFATAL_MATRIX_MISMATCH;
  }
  idx = (WORD32 *)(buf + scratch_idx);
  scratch_idx += sizeof(WORD32) * (HOA_MAX_MATRIX_SIZE);
  memset(idx, 0, sizeof(WORD32) * HOA_MAX_MATRIX_SIZE);
  impeghd_hoa_get_sort_idx(wsx, cols, idx);
  svd_handle->s.cols = svd_handle->s.rows = cols;
  for (i = 0; i < cols; i++)
  {
    svd_handle->w_s[i] = wsx[idx[i]];
    svd_handle->s.mtrx[i * svd_handle->s.cols + i] = svd_handle->w_s[i];
  }
  impeghd_hoa_ren_simple_mtrx_swap_cols(&(svd_handle->v), idx, (buf + scratch_idx));
  impeghd_hoa_ren_simple_mtrx_swap_cols(sm_handle, idx, (buf + scratch_idx));

  if (!transpose_flag)
  {
    impeghd_hoa_ren_simple_mtrx_init_with_matrix(sm_handle, &(svd_handle->u));
  }
  else
  {
    impeghd_hoa_ren_simple_mtrx_init_with_matrix(&(svd_handle->v), &(svd_handle->u));
    impeghd_hoa_ren_simple_mtrx_init_with_matrix(sm_handle, &(svd_handle->v));
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_ren_simple_mtrx_svd_pinv
 *
 *  \brief svd matrix inverse calculation function
 *
 *  \param [in]    handle_svd  Matrix svd matrix
 *  \param [out]   handle_sm   Output matrix
 *  \param [in]    threshold   Threshold value
 *  \param [in]    scratch     Scratch pointer
 *
 *
 *
 */
VOID impeghd_hoa_ren_simple_mtrx_svd_pinv(pVOID handle_svd, pVOID handle_sm, FLOAT32 threshold,
                                          pVOID scratch)
{
  ia_render_hoa_simple_mtrx_svd_str *svd_handle = (ia_render_hoa_simple_mtrx_svd_str *)handle_svd;
  ia_render_hoa_simple_mtrx_str *ret_m = (ia_render_hoa_simple_mtrx_str *)handle_sm;
  WORD32 scratch_idx = 0;
  pWORD8 buf = (pWORD8)scratch;
  ia_render_hoa_simple_mtrx_str *u_t, *s_i, *interm;
  FLOAT32 cmp;

  u_t = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  impeghd_hoa_ren_simple_mtrx_transpose_keep(&(svd_handle->u), u_t);
  s_i = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  impeghd_hoa_ren_simple_mtrx_init_with_matrix(&(svd_handle->s), s_i);
  cmp = ia_mul_flt(s_i->mtrx[1 * s_i->cols + 1], threshold);
  for (WORD32 i = 0; i < s_i->cols; i++)
  {
    FLOAT32 si = s_i->mtrx[i * s_i->cols + i];
    s_i->mtrx[i * s_i->cols + i] = (FLOAT32)((si > cmp) ? (1.0f / si) : 0);
  }
  interm = (ia_render_hoa_simple_mtrx_str *)(buf + scratch_idx);
  scratch_idx += sizeof(ia_render_hoa_simple_mtrx_str);
  impeghd_hoa_ren_simple_mtrx_mult(&(svd_handle->v), s_i, interm, NULL);
  impeghd_hoa_ren_simple_mtrx_mult(interm, u_t, ret_m, buf + scratch_idx);
}
/** @} */ /* End of HOAProc */