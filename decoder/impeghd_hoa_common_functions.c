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
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_rom.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_cal_legendre
 *
 *  \brief HOA legendre calculation
 *
 *  \param [in] n           order index
 *  \param [in] abs_val     absolute value of degree index
 *  \param [in] cos_theta   cos theta value
 *  \param [in] sin_theta   sin theta value
 *
 *  \return FLOAT32 legendre value
 *
 */
static FLOAT32 impeghd_hoa_cal_legendre(WORD32 n, WORD32 abs_val, FLOAT32 cos_theta,
                                        FLOAT32 sin_theta)
{
  FLOAT32 legendre = 1.0;
  FLOAT32 prt2 = 1.0;
  for (WORD32 i = 1; i <= abs_val; i++)
  {
    legendre = ia_mul_flt(legendre, ia_mul_flt(-prt2, sin_theta));
    prt2 += 2.0;
  }

  if (abs_val & 1)
    legendre = -legendre;

  if (n != abs_val)
  {
    FLOAT32 leg_tmp = (FLOAT32)((cos_theta * (2.0 * abs_val + 1) * legendre));
    if (n == (abs_val + 1))
      legendre = leg_tmp;
    else
    {
      FLOAT32 pll = 1.0;
      for (WORD32 j = abs_val + 2; j <= n; j++)
      {
        pll = (FLOAT32)(((cos_theta * (2.0 * j - 1) * leg_tmp) - ((j + abs_val - 1) * legendre)) /
                        (j - abs_val));
        legendre = leg_tmp;
        leg_tmp = pll;
      }
      legendre = pll;
    }
  }
  return legendre;
}
/**
 *  impeghd_hoa_spherical_harmonics
 *
 *  \brief HOA spherical harmonics calculation
 *
 *  \param [in] n           order index
 *  \param [in] m           degree index
 *  \param [in] cos_theta   cos theta value
 *  \param [in] sin_theta   sin theta value
 *  \param [in] phi         azimuth
 *
 *  \return FLOAT32
 *
 */
FLOAT32 impeghd_hoa_spherical_harmonics(WORD32 n, WORD32 m, FLOAT32 cos_theta, FLOAT32 sin_theta,
                                        FLOAT32 phi)
{

  WORD32 abs_val = ia_abs_int(m);
  FLOAT32 sh, dd = 1.0, legendre;

  legendre = impeghd_hoa_cal_legendre(n, abs_val, cos_theta, sin_theta);

  if (n < abs_val)
  {
    dd = 0.0;
  }
  else if ((n + abs_val) > (n - abs_val))
  {
    dd = ia_hoa_product_n_to_m[n + abs_val][n - abs_val + 1];
  }

  if (0 != abs_val)
    sh = (FLOAT32)(ia_sqrt_flt(2.0 * (2.0 * n + 1.0) / (dd)) * legendre);
  else
    sh = (FLOAT32)(ia_sqrt_flt((2.0 * n + 1.0) / (dd)) * legendre);

  if (m < 0)
    return (FLOAT32)(sh * sin(ia_abs_int(m) * phi));
  else
    return (FLOAT32)(sh * cos(m * phi));
}

/**
 *  impeghd_hoa_table_get_transpose_mode_mat_for_fine_grid
 *
 *  \brief Fetch transport mode matrix for fine grid
 *
 *  \param [out] transpose_mode_mat     Transport mode matrix
 *  \param [in]  row_max_sx             Max matrix size
 *  \param [in]  order                  HOA order
 *
 *  \return IA_ERRORCODE                      Error
 *
 */
IA_ERRORCODE impeghd_hoa_table_get_transpose_mode_mat_for_fine_grid(pFLOAT32 transpose_mode_mat,
                                                                    UWORD32 row_max_sx,
                                                                    UWORD32 order)
{
  UWORD32 num_dirs = 900;
  UWORD32 dir_idx;
  WORD32 order_idx, degree_idx, col_idx, dir_grid_tbl = 2;
  pFLOAT32 ptr_tmp_azim, matrix, ptr_cos_tmp_inclin, ptr_sin_tmp_inclin;
  ptr_tmp_azim = (pFLOAT32)(ia_hoa_search_grid_azimuths[dir_grid_tbl]);

  ptr_cos_tmp_inclin = (pFLOAT32)(ia_hoa_cos_search_grid_inclinations[dir_grid_tbl]);
  ptr_sin_tmp_inclin = (pFLOAT32)(ia_hoa_sin_search_grid_inclinations[dir_grid_tbl]);

  for (dir_idx = 0; dir_idx < num_dirs; dir_idx++)
  {
    matrix = transpose_mode_mat;
    for (order_idx = 0; order_idx <= (WORD32)(order); order_idx++)
    {
      for (degree_idx = -order_idx; degree_idx <= order_idx; degree_idx++)
      {
        col_idx = order_idx * (order_idx + 1) + degree_idx;

        *(matrix + ((dir_idx * row_max_sx) + col_idx) /*sizeof(FLOAT64  )*/) =
            impeghd_hoa_spherical_harmonics(order_idx, degree_idx, *ptr_cos_tmp_inclin,
                                            *ptr_sin_tmp_inclin, *ptr_tmp_azim);
      }
    }
    ptr_tmp_azim++;
    ptr_cos_tmp_inclin++;
    ptr_sin_tmp_inclin++;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_get_sample
 *
 *  \brief Computation of sample count using interp_samples for vector based synthesis
 *
 *  \param  [in] interp_samples Interpolation samples
 *  \return UWORD32 sample
 *
 */
static UWORD32 impeghd_hoa_get_sample(UWORD32 interp_samples)
{
  UWORD32 sample = 0;
  switch (interp_samples)
  {
  case 1024:
    sample = 7;
    break;
  case 768:
    sample = 6;
    break;
  case 512:
    sample = 5;
    break;
  case 384:
    sample = 4;
    break;
  case 256:
    sample = 3;
    break;
  case 128:
    sample = 2;
    break;
  case 64:
    sample = 1;
    break;
  case 0:
    sample = 0;
    break;
  }
  return sample;
}
/**
 *  impeghd_hoa_pow_2
 *
 *  \brief Get power of 2 value
 *
 *  \param [in] exp     Exponent
 *
 *  \return FLOAT32     Power of 2 value
 *
 */
FLOAT32 impeghd_hoa_pow_2(WORD32 exp)
{
  if (exp >= 0)
  {
    return ia_hoa_pow_2_table[exp];
  }
  else
  {
    exp *= -1;
    return ia_hoa_pow_2_inv_table[exp];
  }
}
/**
 *  impeghd_hoa_compute_fade_win_for_vec_based_syn
 *
 *  \brief Computation of fade window for vector based synthesis
 *
 *  \param [out] out            Pointer to output fade window for vector based synthesis
 *  \param [in]  interp_samples Interpolation samples
 *  \param [in]  interp_method  Interpolation method
 *
 *  \return IA_ERRORCODE              Error
 *
 */
IA_ERRORCODE impeghd_hoa_compute_fade_win_for_vec_based_syn(pFLOAT32 *out, UWORD32 interp_samples,
                                                            UWORD32 interp_method)
{
  UWORD32 sample = impeghd_hoa_get_sample(interp_samples);
  if (0 != interp_samples)
  {
    if (0 == interp_method || 1 == interp_method)
    {
      *out = (pFLOAT32)&ia_hoa_cos_table_fade_win_for_vec_based_syn[interp_method][sample];
    }
  }
  else
  {
    *out = (pFLOAT32)&ia_hoa_cos_table_fade_win_for_vec_based_syn[0][0];
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impeghd_hoa_get_ceil_log2
 *
 *  \brief Get log2 ceiling value
 *
 *  \param  [in] inp        Input
 *
 *  \return WORD32  Ceiling value of log 2 of given input
 *
 */
WORD32 impeghd_hoa_get_ceil_log2(const WORD32 inp)
{
  WORD32 ceil = 0;
  WORD32 tmp = 1;
  while ((tmp << (ceil)) < inp)
  {
    ceil++;
  }
  return ceil;
}
/** @} */ /* End of HOAProc */