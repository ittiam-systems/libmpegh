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

#include <impeghd_type_def.h>
#include <ia_core_coder_constants.h>
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_avq_rom.h"
#include <ia_core_coder_basic_ops32.h>
#include <ia_core_coder_basic_ops40.h>

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_nearest_2d_neighbor
 *
 *  \brief Function to find nearest neighbour in 2d.
 *
 *  \param [in]  ptr_x     Pointer to input buffer.
 *  \param [out] ptr_y     Pointer to output buffer.
 *  \param [in]  count Data length parameter.
 *  \param [out] ptr_rem   Pointer to data buffer.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_nearest_2d_neighbor(WORD32 *ptr_x, WORD32 *ptr_y, WORD32 count,
                                              WORD32 *ptr_rem)
{
  WORD32 i, j = 0, sum = 0;
  WORD32 s, em = 0, rem_temp[8];
  for (i = 0; i < 8; i++)
  {
    rem_temp[i] = ptr_rem[i];
    if ((ptr_x[i] & 1) != 0)
    {
      rem_temp[i] =
          (ptr_x[i] < 0)
              ? ia_core_coder_negate32_sat(ia_core_coder_sub32_sat(rem_temp[i], (1 << count)))
              : ia_core_coder_sub32_sat(rem_temp[i], (1 << count));
    }
    ptr_y[i] = (ptr_x[i] < 0)
                   ? ia_core_coder_negate32_sat(
                         ia_core_coder_shl32_sat((ia_core_coder_sub32_sat(1, ptr_x[i]) >> 1), 1))
                   : ia_core_coder_shl32_sat((ia_core_coder_add32_sat(1, ptr_x[i]) >> 1), 1);
    sum = ia_core_coder_add32_sat(ptr_y[i], sum);
  }

  if (sum & 3)
  {

    for (i = 0; i < 8; i++)
    {
      s = (rem_temp[i] < 0) ? -rem_temp[i] : rem_temp[i];
      j = (em < s) ? i : j;
      em = (em < s) ? s : em;
    }

    ptr_y[j] = (rem_temp[j] < 0) ? ptr_y[j] - 2 : ia_core_coder_add32_sat(ptr_y[j], 2);
    rem_temp[j] = (rem_temp[j] < 0) ? ia_core_coder_add32_sat(rem_temp[j], (2 << count))
                                    : ia_core_coder_sub32_sat(rem_temp[j], (2 << count));
  }
  for (i = 0; i < 8; i++)
  {
    ptr_rem[i] = rem_temp[i];
  }
  return;
}

/**
 *  ia_core_coder_search_voronoi
 *
 *  \brief Voronoi search method.
 *
 *  \param [in]  ptr_x     Pointer to input buffer.
 *  \param [out] ptr_y     Pointer to output buffer.
 *  \param [in]  count Data length parameter.
 *  \param [in]  ptr_rem1  Pointer to interim data buffer.
 *  \param [in]  ptr_rem2  Pointer to interim data buffer.
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_search_voronoi(WORD32 *ptr_x, WORD32 *ptr_y, WORD32 count, WORD32 *ptr_rem1,
                                  WORD32 *ptr_rem2)
{
  WORD32 x1[8], i, y0[8], y1[8];
  WORD32 e0 = 0, e1 = 0;

  ia_core_coder_nearest_2d_neighbor(ptr_x, y0, count, ptr_rem1);
  for (i = 0; i < 8; i++)
  {
    x1[i] = (ptr_x[i] == 0) ? ((ptr_rem2[i] == 0) ? (ptr_x[i] - 1) : 0)
                            : ia_core_coder_sub32_sat(ptr_x[i], 1);
    ptr_rem2[i] =
        (ptr_x[i] == 0)
            ? ((ptr_rem2[i] == 0) ? 0 : ia_core_coder_sub32_sat(ptr_rem2[i], (1 << count)))
            : ptr_rem2[i];
  }

  ia_core_coder_nearest_2d_neighbor(x1, y1, count, ptr_rem2);
  for (i = 0; i < 8; i++)
  {
    e0 = ia_core_coder_add32_sat(
        ia_core_coder_sat64_32((WORD64)ptr_rem1[i] * (WORD64)ptr_rem1[i]), e0);
    e1 = ia_core_coder_add32_sat(
        ia_core_coder_sat64_32((WORD64)ptr_rem2[i] * (WORD64)ptr_rem2[i]), e1);
  }

  if (e0 >= e1)
  {
    for (i = 0; i < 8; i++)
    {
      ptr_y[i] = ia_core_coder_add32_sat(y1[i], 1);
    }
  }
  else
  {
    for (i = 0; i < 8; i++)
    {
      ptr_y[i] = y0[i];
    }
  }
  return;
}

/**
 *  ia_core_coder_decode_voronoi_idx
 *
 *  \brief Voronoi index decoding.
 *
 *  \param [in]  ptr_kv    Pointer to input buffer.
 *  \param [in]  m     Voronoi extension parameter.
 *  \param [out] ptr_y     Pointer to output buffer.
 *  \param [in]  count Data length parameter.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_decode_voronoi_idx(WORD32 *ptr_kv, WORD32 m, WORD32 *ptr_y,
                                             WORD32 count)
{
  WORD32 i, v[8], temp, sum = 0;
  WORD32 z[8], rem1[8], rem2[8];
  ;
  for (i = 6; i >= 1; i--)
  {
    temp = ia_core_coder_shl32_sat(ptr_kv[i], 1);
    ptr_y[i] = ia_core_coder_add32_sat(ptr_kv[7], temp);
    z[i] = ptr_y[i] >> count;
    rem1[i] = ptr_y[i] & (m - 1);
    sum = ia_core_coder_add32_sat(sum, temp);
  }
  ptr_y[7] = ptr_kv[7];
  z[7] = ptr_kv[7] >> count;
  rem1[7] = ptr_kv[7] & (m - 1);
  ptr_y[0] = ia_core_coder_add32_sat(
      ptr_kv[7],
      ia_core_coder_add32_sat(ia_core_coder_sat64_32((WORD64)4 * (WORD64)ptr_kv[0]), sum));
  z[0] = (ia_core_coder_sub32_sat(ptr_y[0], 2)) >> count;
  temp = ia_core_coder_sub32_sat(ptr_y[0], 2);
  rem1[0] = (m != 0) ? temp % m : temp;
  for (i = 0; i < 8; i++)
  {
    rem2[i] = rem1[i];
  }
  ia_core_coder_search_voronoi(z, v, count, rem1, rem2);
  for (i = 0; i < 8; i++)
  {
    *ptr_y = ia_core_coder_sub32_sat(*ptr_y, ia_core_coder_sat64_32((WORD64)m * (WORD64)v[i]++));
    ptr_y++;
  }
}

/**
 *  ia_core_coder_gosset_rank_of_permutation
 *
 *  \brief AVQ Gosset lattice ranking.
 *
 *  \param [in]  rank Rank value.
 *  \param [i/o] ptr_xs   Pointer to data buffer
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_gosset_rank_of_permutation(WORD32 rank, WORD32 *ptr_xs)
{
  WORD32 i, j, a[8], w_value[8], base_value, factorial, factorial_b, target_value;

  j = 0;
  w_value[j] = 1;
  a[j] = ptr_xs[0];
  base_value = 1;
  for (i = 1; i < 8; i++)
  {

    if (ptr_xs[i] == ptr_xs[i - 1])
    {
      w_value[j]++;
      base_value *= w_value[j];
    }
    else
    {
      j++;
      w_value[j] = 1;
      a[j] = ptr_xs[i];
    }
  }
  if (w_value[0] != 8)
  {
    target_value = rank * base_value;
    factorial_b = 1;

    for (i = 0; i < 8; i++)
    {
      factorial = factorial_b * ia_core_coder_7_factorial[i];
      j = -1;
      do
      {
        target_value -= w_value[++j] * factorial;
      } while (target_value >= 0);
      ptr_xs[i] = a[j];

      target_value += w_value[j] * factorial;
      factorial_b *= w_value[j];
      w_value[j]--;
    }
  }
  else
  {
    for (i = 0; i < 8; i++)
      ptr_xs[i] = a[0];
  }
  return;
}

/**
 *  ia_core_coder_get_abs_leader_tbl
 *
 *  \brief AVQ helper function.
 *
 *  \param [in] table         Pointer to ROM table.
 *  \param [in] code_book_idx Code book index value.
 *  \param [in] size          Data size parameter.
 *
 *  \return WORD32 Index value.
 *
 */
static WORD32 ia_core_coder_get_abs_leader_tbl(const UWORD32 *ptr_table, UWORD32 code_book_idx,
                                               WORD32 data_size)
{
  WORD32 i;

  for (i = 4; i < data_size; i += 4)
  {
    if (code_book_idx < ptr_table[i])
      break;
  }
  i = (i > data_size) ? data_size : i;
  i = (code_book_idx < ptr_table[i - 2]) ? (i - 2) : i;
  i = (code_book_idx < ptr_table[i - 1]) ? (i - 1) : i;
  i--;

  return (i);
}

/**
 *  ia_core_coder_gosset_decode_base_index
 *
 *  \brief AVQ gosset lattice decoding helper function.
 *
 *  \param [in]  n             Index Value.
 *  \param [in]  code_book_ind Code book index value.
 *  \param [out] ptr_ya            Pointer to output buffer.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_gosset_decode_base_index(WORD32 n, UWORD32 code_book_ind,
                                                   WORD32 *ptr_ya)
{
  WORD32 im, t, i, ks;
  WORD32 sign_code, idx = 0, rank;

  if (n >= 2)
  {
    switch (n)
    {
    case 4:
      i = ia_core_coder_get_abs_leader_tbl(ia_core_coder_i4_cardinality_offset_tab, code_book_ind,
                                           LEN_I4);
      idx = ia_core_coder_a4_pos_abs_leaders[i];
      break;
    case 3:
    case 2:
      i = ia_core_coder_get_abs_leader_tbl(ia_core_coder_i3_cardinality_offset_table,
                                           code_book_ind, LEN_I3);
      idx = ia_core_coder_a3_pos_abs_leaders[i];
      break;
    }

    im = ia_core_coder_num_table_iso_code[idx];
    t = ia_core_coder_index_table_iso_code[idx];

    ks = ia_core_coder_get_abs_leader_tbl(ia_core_coder_is_signed_leader + t, code_book_ind, im);

    sign_code = (ia_core_coder_data_table_iso_code[t + ks]) << 1;
    rank = code_book_ind - ia_core_coder_is_signed_leader[t + ks];
    for (i = 7; i >= 0; i--)
    {
      ptr_ya[i] = (ia_core_coder_tab_da_absolute_leader[idx][i]) * (1 - (sign_code & 2));
      sign_code >>= 1;
    }

    ia_core_coder_gosset_rank_of_permutation(rank, ptr_ya);
  }
  else
  {
    for (i = 0; i < 8; i++)
      ptr_ya[i] = 0;
  }
  return;
}

/**
 *  ia_core_coder_rotated_gosset_mtx_dec
 *
 *  \brief AVQ gosset matrix decoding helper function.
 *
 *  \param [in]  qn            Index Parameter.
 *  \param [in]  code_book_idx Code book index value.
 *  \param [out] ptr_kv            Pointer to voronoi data buffer.
 *  \param [out] ptr_b             Pointer to intermediate data buffer.
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_rotated_gosset_mtx_dec(WORD32 qn, WORD32 code_book_idx, WORD32 *ptr_kv,
                                          WORD32 *ptr_b)
{
  WORD32 i, m, c[8];
  WORD32 count = 0;

  if (qn > 4)
  {
    for (; qn > 4; qn -= 2)
      count++;
    m = (count >= 31) ? MAX_32 : (1 << count);
    ia_core_coder_decode_voronoi_idx(ptr_kv, m, c, count);
    ia_core_coder_gosset_decode_base_index(qn, code_book_idx, ptr_b);
    for (i = 0; i < 7; i = i + 2)
    {
      ptr_b[i] =
          ia_core_coder_add32_sat(ia_core_coder_sat64_32((WORD64)m * (WORD64)ptr_b[i]), c[i]);
      ptr_b[i + 1] = ia_core_coder_add32_sat(
          ia_core_coder_sat64_32((WORD64)m * (WORD64)ptr_b[i + 1]), c[i + 1]);
    }
  }
  else
  {
    ia_core_coder_gosset_decode_base_index(qn, code_book_idx, ptr_b);
  }
  return;
}
/** @} */ /* End of CoreDecProc */