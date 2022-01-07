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
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_config.h"
#include "impeghd_mhas_parse.h"

/* Decoder interfaces */
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"

#include "impeghd_3d_vec_basic_ops.h"

/**
 * @defgroup OAMProc Object Audio Renderer Processing
 * @ingroup  OAMProc
 * @brief Object Audio Renderer Processing
 *
 * @{
 */

/**
 *  impeghd_nrm_values
 *
 *  \brief Normalizes a 1D vector of length num_val.
 *
 *  \param [in,out] ptr_val Pointer to the input vector.
 *  \param [in]  num_val Length of the input vector.
 *
 *
 *
 */
static VOID impeghd_nrm_values(FLOAT32 *ptr_val, WORD32 num_val)
{
  WORD32 idx;
  FLOAT32 len = 0.0f;
  for (idx = 0; idx < num_val; idx++)
  {
    len = ia_mac_flt(len, ptr_val[idx], ptr_val[idx]);
  }
  len = (FLOAT32)ia_sqrt_flt(len);
  len = 1.0f / len;
  if (len != 0.0f)
  {
    for (idx = 0; idx < num_val; idx++)
    {
      ptr_val[idx] = ia_mul_flt(ptr_val[idx], len);
    }
  }
}

/**
 *  impeghd_calc_spread_gains
 *
 *  \brief Calculates spread gains.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object renderer state structure
 *  \param [in]  spread_angle_rad      Spread angle value in radians.
 *
 *
 *
 */

static VOID impeghd_calc_spread_gains(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state,
                                      FLOAT32 spread_angle_rad)
{
  WORD32 i;
  WORD32 tot_num_spks =
      ptr_obj_ren_dec_state->num_non_lfe_ls + ptr_obj_ren_dec_state->num_imag_ls;
  FLOAT32 norm = 0.0f;
  FLOAT32 unitGain = (FLOAT32)(1.f / ia_sqrt_flt((FLOAT32)tot_num_spks));
  FLOAT32 alpha = (FLOAT32)((spread_angle_rad * DEG_2_RAD) / (M_PI / 2.f) - 1.f);
  for (i = 0; i < tot_num_spks; i++)
  {
    norm =
        ia_mac_flt(norm, ptr_obj_ren_dec_state->ls_gains[i], ptr_obj_ren_dec_state->ls_gains[i]);
  }
  norm = (FLOAT32)ia_sqrt_flt(norm);
  if (alpha > 1.0f)
  {
    alpha = 1.0f;
  }
  else if (alpha < 0.0f)
  {
    alpha = 0.0f;
  }
  for (i = 0; i < tot_num_spks; ++i)
  {
    ptr_obj_ren_dec_state->ls_gains[i] =
        ia_sub_flt(1.f, alpha) * ptr_obj_ren_dec_state->ls_gains[i] / norm + alpha * unitGain;
  }
}

/**
 *  impeghd_calc_one_source_point
 *
 *  \brief Calculates one source point
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object renderer state structure.
 *  \param [in,out] ptr_vec Pointer to loudspeaker coordinate vector.
 *
 *
 *
 */

VOID impeghd_calc_one_source_point(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state,
                                   ia_cart_coord_str *ptr_vec)
{

  WORD32 i, j;
  WORD32 winner_set = 0;
  WORD32 hi_lo_gain_idx = 3;
  WORD32 gains_modif_flag = 0;
  WORD32 v1_idx = 0, v2_idx = 0, v3_idx = 0;
  FLOAT32 power;
  FLOAT32 highest_least_gain = -100000.f;
  FLOAT32 g[3] = {0.f, 0.f, 0.f};

  for (i = 0; i < ptr_obj_ren_dec_state->num_triangles; i++)
  {
    FLOAT32 gains[3];
    FLOAT32 min_gain = 10000000.f;
    WORD32 neg_gain_idx = 3;
    for (j = 2; j >= 0; j--)
    {
      gains[j] = ia_mul_flt(ptr_vec->x, ptr_obj_ren_dec_state->ls_inv_mtx[i].row[j].x);
      gains[j] = ia_mac_flt(gains[j], ptr_vec->y, ptr_obj_ren_dec_state->ls_inv_mtx[i].row[j].y);
      gains[j] = ia_mac_flt(gains[j], ptr_vec->z, ptr_obj_ren_dec_state->ls_inv_mtx[i].row[j].z);
      if (gains[j] >= -MIN_VEC_CTR_DIST)
      {
        neg_gain_idx--;
      }
      if (gains[j] < min_gain)
      {
        min_gain = gains[j];
      }
    }

    if ((neg_gain_idx < hi_lo_gain_idx) ||
        (neg_gain_idx <= hi_lo_gain_idx && min_gain > highest_least_gain))
    {
      g[0] = gains[0];
      g[1] = gains[1];
      g[2] = gains[2];
      highest_least_gain = min_gain;
      hi_lo_gain_idx = neg_gain_idx;
      winner_set = i;
    }
  }
  v3_idx = ptr_obj_ren_dec_state->ls_triangle[winner_set].spk_index[2];
  v2_idx = ptr_obj_ren_dec_state->ls_triangle[winner_set].spk_index[1];
  v1_idx = ptr_obj_ren_dec_state->ls_triangle[winner_set].spk_index[0];
  for (i = 3; i > 0; i--)
  {
    if (g[i - 1] < -MIN_VEC_CTR_DIST)
    {
      gains_modif_flag = 1;
      g[i - 1] = 0.f;
    }
  }

  if (gains_modif_flag == 1)
  {
    ptr_vec->z = ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v1_idx].ls_cart_coord.z, g[0]) +
                 ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v2_idx].ls_cart_coord.z, g[1]) +
                 ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v3_idx].ls_cart_coord.z, g[2]);
    ptr_vec->y = ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v1_idx].ls_cart_coord.y, g[0]) +
                 ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v2_idx].ls_cart_coord.y, g[1]) +
                 ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v3_idx].ls_cart_coord.y, g[2]);
    ptr_vec->x = ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v1_idx].ls_cart_coord.x, g[0]) +
                 ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v2_idx].ls_cart_coord.x, g[1]) +
                 ia_mul_flt(ptr_obj_ren_dec_state->non_lfe_ls_str[v3_idx].ls_cart_coord.x, g[2]);
  }
  if (ia_lt_flt(g[2], 0.f))
  {
    g[2] = 0.f;
  }
  if (ia_lt_flt(g[1], 0.f))
  {
    g[1] = 0.f;
  }
  if (ia_lt_flt(g[0], 0.f))
  {
    g[0] = 0.f;
  }

  power = (FLOAT32)ia_sqrt_flt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]);
  if (power != 0.f)
  {
    for (i = 0; i < 3; i++)
    {
      g[i] = g[i] / power;
    }
  }
  ptr_obj_ren_dec_state->ls_gains[v3_idx] =
      ia_add_flt(g[2], ptr_obj_ren_dec_state->ls_gains[v3_idx]);
  ptr_obj_ren_dec_state->ls_gains[v2_idx] =
      ia_add_flt(g[1], ptr_obj_ren_dec_state->ls_gains[v2_idx]);
  ptr_obj_ren_dec_state->ls_gains[v1_idx] =
      ia_add_flt(g[0], ptr_obj_ren_dec_state->ls_gains[v1_idx]);
}

/**
 *  impeghd_vector_process
 *
 *  \brief Calculates Muliple Direction Amplitude Panned Vectors.
 *
 *  \param [out] ptr_mdap_vec_array Pointer to buffer that stores MDAP vectors.
 *  \param [in]  u              Pointer to vector u coordinate's structure.
 *  \param [in]  v			    Pointer to vector v coordinate's structure.
 *  \param [in]  p0				Pointer to vector p0 coordinate's structure.
 *
 *
 *
 */
VOID impeghd_vector_process(ia_cart_coord_str *ptr_mdap_vec_array, ia_cart_coord_str u,
                            ia_cart_coord_str v, ia_cart_coord_str p0)
{
  VECTOR_COPY_CART(ptr_mdap_vec_array[1], u);
  /* p'2 = 0.75 u + 0.25 p0 */
  impeghd_scaled_vec_addition_cart(&u, 0.75f, &p0, 0.25f, &ptr_mdap_vec_array[2]);
  /* p'3 = 0.375 u + 0.625 p0 */
  impeghd_scaled_vec_addition_cart(&u, 0.375f, &p0, 0.625f, &ptr_mdap_vec_array[3]);
  /* p'4 = -u */
  NEGATE_COPY_CART(ptr_mdap_vec_array[4], u);
  /* p'5 = -0.75 u + 0.25 p0 */
  impeghd_scaled_vec_addition_cart(&u, -0.75f, &p0, 0.25f, &ptr_mdap_vec_array[5]);
  /* p'6 = -0.375 u + 0.625 p0 */
  impeghd_scaled_vec_addition_cart(&u, -0.375f, &p0, 0.625f, &ptr_mdap_vec_array[6]);
  /* p'7 =  0.5 u + 0.866v + p0/3 */
  impeghd_scaled_vec_addition_cart(&u, 0.5f, &v, 0.866f, &ptr_mdap_vec_array[7]);
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[7], 1.0f, &p0, (0.333f),
                                   &ptr_mdap_vec_array[7]);
  /* p'8 =  0.5 p'7 + 0.5 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[7], 0.5f, &p0, 0.5f,
                                   &ptr_mdap_vec_array[8]);
  /* p'9 =  0.25 p'7 + 0.75 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[7], 0.25f, &p0, 0.75f,
                                   &ptr_mdap_vec_array[9]);
  /* p'10 =  -0.5 u + 0.866v + p0/3 */
  impeghd_scaled_vec_addition_cart(&u, -0.5f, &v, 0.866f, &ptr_mdap_vec_array[10]);
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[10], 1.0f, &p0, (0.333f),
                                   &ptr_mdap_vec_array[10]);
  /* p'11 =  0.5 p'10 + 0.5 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[10], 0.5f, &p0, 0.5f,
                                   &ptr_mdap_vec_array[11]);
  /* p'12 =  0.25 p'10 + 0.75 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[10], 0.25f, &p0, 0.75f,
                                   &ptr_mdap_vec_array[12]);
  /* p'13 =  -0.5 u - 0.866v + p0/3 */
  impeghd_scaled_vec_addition_cart(&u, -0.5f, &v, -0.866f, &ptr_mdap_vec_array[13]);
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[13], 1.0f, &p0, (0.333f),
                                   &ptr_mdap_vec_array[13]);
  /* p'14 =  0.5 p'13 + 0.5 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[13], 0.5f, &p0, 0.5f,
                                   &ptr_mdap_vec_array[14]);
  /* p'15 =  0.25 p'13 + 0.75 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[13], 0.25f, &p0, 0.75f,
                                   &ptr_mdap_vec_array[15]);
  /* p'16 =  0.5 u - 0.866v + p0/3 */
  impeghd_scaled_vec_addition_cart(&u, 0.5f, &v, -0.866f, &ptr_mdap_vec_array[16]);
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[16], 1.0f, &p0, (0.333f),
                                   &ptr_mdap_vec_array[16]);
  /* p'17 =  0.5 p'16 + p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[16], 0.5f, &p0, 0.5f,
                                   &ptr_mdap_vec_array[17]);
  /* p'18 =  0.25 p'16 + 0.75 p0 */
  impeghd_scaled_vec_addition_cart(&ptr_mdap_vec_array[16], 0.25f, &p0, 0.75f,
                                   &ptr_mdap_vec_array[18]);
}

/**
 *  impeghd_spreading_calc_mdap_vectors
 *
 *  \brief Calculates Muliple Direction Amplitude Panned Vectors.
 *
 *  \param [out] ptr_mdap_vec_array Pointer to buffer that stores MDAP vectors.
 *  \param [in]  vec_p              Pointer to vector P coordinate's structure.
 *  \param [in]  in_spread_params   Pointer to input spread params.
 *  \param [in]  height_spks_flag   Flag that indicates presence of height speakers.
 *
 *
 *
 */

VOID impeghd_spreading_calc_mdap_vectors(ia_cart_coord_str *ptr_mdap_vec_array,
                                         ia_polar_coord_str *vec_p, FLOAT32 *in_spread_params,
                                         WORD32 height_spks_flag)
{
  WORD32 i;
  FLOAT32 alpha;
  FLOAT32 spread_params[3];
  FLOAT32 tan_alpha = 0;
  ia_polar_coord_str p;
  ia_cart_coord_str u, v, p0;
  spread_params[2] = in_spread_params[2];
  spread_params[1] = in_spread_params[1];
  spread_params[0] = in_spread_params[0];
  for (i = 0; i < 2; i++)
  {
    if (spread_params[i] > MDAP_ALPHA_MAX_VAL_DEGREE)
    {
      spread_params[i] = MDAP_ALPHA_MAX_VAL_DEGREE;
    }
    else if (spread_params[i] < MDAP_ALPHA_MIN_VAL_DEGREE)
    {
      spread_params[i] = MDAP_ALPHA_MIN_VAL_DEGREE;
    }
  }
  if (spread_params[2] > 16.0f)
  {
    spread_params[2] = 16.0f;
  }
  else if (spread_params[2] < 0.0f)
  {
    spread_params[2] = 0.0f;
  }

  alpha = spread_params[0];
  tan_alpha = (FLOAT32)tan(alpha * DEG_2_RAD);

  p.radius = 1;
  p.phi = vec_p->phi;
  p.theta = vec_p->theta;

  impeghd_pol_2_cart_degree(&p, &p0);

  if (p.phi < 0)
  {
    p.phi += 90;
  }
  else
  {
    p.phi -= 90;
  }
  impeghd_pol_2_cart_degree(&p, &v);
  u.z = ia_sub_flt(ia_mul_flt(v.x, p0.y), ia_mul_flt(v.y, p0.x));
  u.y = ia_sub_flt(ia_mul_flt(v.z, p0.x), ia_mul_flt(v.x, p0.z));
  u.x = ia_sub_flt(ia_mul_flt(v.y, p0.z), ia_mul_flt(v.z, p0.y));

  if (height_spks_flag == 0)
  {
    v.x = 0.0f;
    v.y = 0.0f;
    v.z = 0.0f;
  }

  impeghd_vector_process(&ptr_mdap_vec_array[0], u, v, p0);

  p0.x = p0.x / tan_alpha;
  p0.y = p0.y / tan_alpha;
  p0.z = p0.z / tan_alpha;
  for (i = 0; i < NUM_MDAP_VEC; i++)
  {
    ptr_mdap_vec_array[i].x = ia_add_flt(ptr_mdap_vec_array[i].x, p0.x);
    ptr_mdap_vec_array[i].y = ia_add_flt(ptr_mdap_vec_array[i].y, p0.y);
    ptr_mdap_vec_array[i].z = ia_add_flt(ptr_mdap_vec_array[i].z, p0.z);
    FLOAT32 vector_length = (FLOAT32)ia_sqrt_flt(
        ia_add_flt(ia_mul_flt((ptr_mdap_vec_array[i].x), (ptr_mdap_vec_array[i].x)),
                   ia_add_flt(ia_mul_flt((ptr_mdap_vec_array[i].y), (ptr_mdap_vec_array[i].y)),
                              ia_mul_flt((ptr_mdap_vec_array[i].z), (ptr_mdap_vec_array[i].z)))));
    if (vector_length != 0.0f)
    {
      FLOAT32 one_by_vlen = 1.0f / vector_length;
      ptr_mdap_vec_array[i].x = ia_mul_flt((ptr_mdap_vec_array[i].x), one_by_vlen);
      ptr_mdap_vec_array[i].y = ia_mul_flt((ptr_mdap_vec_array[i].y), one_by_vlen);
      ptr_mdap_vec_array[i].z = ia_mul_flt((ptr_mdap_vec_array[i].z), one_by_vlen);
    }
  }
}

/**
 *  impeghd_calc_vbap
 *
 *  \brief Calculates gains that are derived from object audio params
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *  \param [in]  gain_fac              Gain value
 *  \param [in]  spread_params_rad     Spread params in radians
 *  \param [in]  vec_p                 Pointer to vector P
 *  \param [in,out] mdap_vec_array        Multi Direction Amplitude Panning Vec array
 *
 *
 *
 */

VOID impeghd_calc_vbap(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state, FLOAT32 gain_fac,
                       FLOAT32 *spread_params_rad, ia_polar_coord_str *vec_p,
                       ia_cart_coord_str *mdap_vec_array)
{
  WORD32 i;
  WORD32 tot_num_spks =
      ptr_obj_ren_dec_state->num_non_lfe_ls + ptr_obj_ren_dec_state->num_imag_ls;
  ia_cart_coord_str vec_c;
  impeghd_pol_2_cart_degree(vec_p, &vec_c);
  memcpy(&mdap_vec_array[0], &vec_c, sizeof(vec_c));
  memset(&ptr_obj_ren_dec_state->ls_gains[0], 0,
         sizeof(ptr_obj_ren_dec_state->ls_gains[0]) * tot_num_spks);
  if ((spread_params_rad[0] <= 0.0f) && (spread_params_rad[1] <= 0.0f))
  {
    impeghd_calc_one_source_point(ptr_obj_ren_dec_state, &vec_c);
  }
  else
  {

    impeghd_spreading_calc_mdap_vectors(&mdap_vec_array[0], vec_p, spread_params_rad,
                                        ptr_obj_ren_dec_state->height_spks_present);
    for (i = 0; i < NUM_MDAP_VEC; i++)
    {
      impeghd_calc_one_source_point(ptr_obj_ren_dec_state, &mdap_vec_array[i]);
    }
    impeghd_calc_spread_gains(ptr_obj_ren_dec_state, spread_params_rad[0]);
  }

  impeghd_nrm_values(&ptr_obj_ren_dec_state->ls_gains[0], tot_num_spks);

  for (i = 0; i < tot_num_spks; i++)
  {
    ptr_obj_ren_dec_state->ls_gains[0] = ptr_obj_ren_dec_state->ls_gains[0] * gain_fac;
  }
}

/**
 *  impegh_obj_ren_vbap_process
 *
 *  \brief Vector base amplitude panning processing function.
 *
 *  \param [in,out] ptr_obj_ren_dec_state Pointer to object rederer state structure.
 *  \param [in]  obj_idx               Index of the object data that is processed.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE
impegh_obj_ren_vbap_process(ia_obj_ren_dec_state_struct *ptr_obj_ren_dec_state, WORD32 obj_idx)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, j = 0;
  FLOAT32 spread_params_rad[3];
  FLOAT32 gain_sum = 0.0f;
  FLOAT32 gain_fac = ptr_obj_ren_dec_state->str_obj_md_dec_state.gain_descaled[obj_idx];
  WORD32 tot_num_spks =
      (ptr_obj_ren_dec_state->num_imag_ls + ptr_obj_ren_dec_state->num_non_lfe_ls);
  ia_cart_coord_str mdap_vec_array[NUM_MDAP_VEC];
  ia_polar_coord_str vec_p;
  ia_oam_dec_state_struct *ptr_obj_md_dec_state = &ptr_obj_ren_dec_state->str_obj_md_dec_state;
  if (ptr_obj_ren_dec_state->first_frame_flag[obj_idx])
  {
    vec_p.phi =
        (FLOAT32)(ptr_obj_ren_dec_state->str_obj_md_dec_state.elevation_descaled[obj_idx]);
    vec_p.theta =
        (FLOAT32)(ptr_obj_ren_dec_state->str_obj_md_dec_state.azimuth_descaled[obj_idx]);
    vec_p.radius =
        (FLOAT32)(ptr_obj_ren_dec_state->str_obj_md_dec_state.radius_descaled[obj_idx]);
    spread_params_rad[2] =
        ia_mul_flt(ptr_obj_ren_dec_state->str_obj_md_dec_state.spread_depth_descaled[obj_idx], 1);
    spread_params_rad[1] = ia_mul_flt(
        ptr_obj_ren_dec_state->str_obj_md_dec_state.spread_height_descaled[obj_idx], 1);
    spread_params_rad[0] =
        ia_mul_flt(ptr_obj_ren_dec_state->str_obj_md_dec_state.spread_width_descaled[obj_idx], 1);
    if (!(ptr_obj_ren_dec_state->num_non_lfe_ls > 0))
    {

      impeghd_calc_vbap(ptr_obj_ren_dec_state, gain_fac, &spread_params_rad[0], &vec_p,
                        &mdap_vec_array[0]);
      for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls; i++)
      {
        ptr_obj_ren_dec_state->initial_gains[obj_idx][i] = (ptr_obj_ren_dec_state->ls_gains[i]);
      }
    }
    else
    {
      impeghd_calc_vbap(ptr_obj_ren_dec_state, 1.0f, &spread_params_rad[0], &vec_p,
                        &mdap_vec_array[0]);
      gain_sum = 0.0f;
      for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls; i++)
      {
        FLOAT32 value = 0;
        for (j = 0; j < tot_num_spks; j++)
        {
          value = ia_mac_flt(value, ptr_obj_ren_dec_state->downmix_mtx[i][j],
                             ptr_obj_ren_dec_state->ls_gains[j]);
        }
        ptr_obj_ren_dec_state->initial_gains[obj_idx][i] = value;
        gain_sum = ia_mac_flt(gain_sum, value, value);
      }
      gain_sum = (FLOAT32)ia_sqrt_flt(gain_sum);

      if (gain_sum != 0.0f)
      {
        for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls; i++)
        {
          ptr_obj_ren_dec_state->initial_gains[obj_idx][i] =
              ia_mul_flt(ptr_obj_ren_dec_state->initial_gains[obj_idx][i], gain_fac) / gain_sum;
        }
      }
    }
  }

  vec_p.phi = (FLOAT32)(ptr_obj_ren_dec_state->str_obj_md_dec_state.elevation_descaled[obj_idx]);
  vec_p.theta = (FLOAT32)(ptr_obj_ren_dec_state->str_obj_md_dec_state.azimuth_descaled[obj_idx]);
  vec_p.radius = (FLOAT32)(ptr_obj_ren_dec_state->str_obj_md_dec_state.radius_descaled[obj_idx]);

  spread_params_rad[2] =
      ia_mul_flt(ptr_obj_ren_dec_state->str_obj_md_dec_state.spread_depth_descaled[obj_idx], 1);
  spread_params_rad[1] =
      ia_mul_flt(ptr_obj_ren_dec_state->str_obj_md_dec_state.spread_height_descaled[obj_idx], 1);
  spread_params_rad[0] =
      ia_mul_flt(ptr_obj_ren_dec_state->str_obj_md_dec_state.spread_width_descaled[obj_idx], 1);

  gain_fac = ptr_obj_md_dec_state->gain_descaled[obj_idx];

  if (!(ptr_obj_ren_dec_state->num_non_lfe_ls > 0))
  {
    impeghd_calc_vbap(ptr_obj_ren_dec_state, gain_fac, &spread_params_rad[0], &vec_p,
                      &mdap_vec_array[0]);
    for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls; i++)
    {
      ptr_obj_ren_dec_state->final_gains[i] = (ptr_obj_ren_dec_state->ls_gains[i]);
    }
  }
  else
  {

    impeghd_calc_vbap(ptr_obj_ren_dec_state, 1.0f, &spread_params_rad[0], &vec_p,
                      &mdap_vec_array[0]);

    gain_sum = 0.0f;
    for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls; i++)
    {
      FLOAT32 value = 0;
      for (j = 0; j < tot_num_spks; j++)
      {
        value = ia_mac_flt(value, ptr_obj_ren_dec_state->downmix_mtx[i][j],
                           ptr_obj_ren_dec_state->ls_gains[j]);
      }
      ptr_obj_ren_dec_state->final_gains[i] = value;
      gain_sum = ia_mac_flt(gain_sum, value, value);
    }
    gain_sum = (FLOAT32)ia_sqrt_flt(gain_sum);

    if (gain_sum != 0.0f)
    {
      for (i = 0; i < ptr_obj_ren_dec_state->num_non_lfe_ls; i++)
      {
        ptr_obj_ren_dec_state->final_gains[i] =
            ia_mul_flt(ptr_obj_ren_dec_state->final_gains[i], gain_fac) / gain_sum;
      }
    }
  }

  return err_code;
}
/** @} */ /* End of OAMProc */