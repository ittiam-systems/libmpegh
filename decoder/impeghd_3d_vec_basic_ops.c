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
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_obj_ren_dec_defines.h"

/**
 * @defgroup 3DVecOps 3D Vector Operations
 * @ingroup  3DVecOps
 * @brief 3D Vector basic operation utils
 *
 * @{
 */

/**
 *  impeghd_pol_2_cart
 *
 *  \brief Convert Polar to Cartesian co-ordinates
 *
 *  \param [in]  ptr_polar_coord    Pointer to Polar co-ordinates structure
 *  \param [out] ptr_cart_coord     Pointer to Cartesian co-ordinates structure
 *
 *  \return VOID
 *
 */
VOID impeghd_pol_2_cart(ia_polar_coord_str *ptr_polar_coord, ia_cart_coord_str *ptr_cart_coord)
{
  FLOAT32 sin_phi = (FLOAT32)(sin(ptr_polar_coord->phi));
  FLOAT32 cos_phi = (FLOAT32)(cos(ptr_polar_coord->phi));
  FLOAT32 sin_theta = (FLOAT32)(sin(ptr_polar_coord->theta));
  FLOAT32 cos_theta = (FLOAT32)(cos(ptr_polar_coord->theta));
  FLOAT32 r = ptr_polar_coord->radius;
  ptr_cart_coord->z = ia_mul_flt(r, sin_phi);
  ptr_cart_coord->y = ia_mul_flt(ia_mul_flt(r, sin_theta), cos_phi);
  ptr_cart_coord->x = ia_mul_flt(ia_mul_flt(r, cos_theta), cos_phi);
}

/**
 *  impeghd_pol_2_cart_degree
 *
 *  \brief Convert Polar to Cartesian co-ordinates
 *
 *  \param [in]  ptr_polar_coord    Pointer to Polar co-ordinates structure
 *  \param [out] ptr_cart_coord     Pointer to Cartesian co-ordinates structure
 *
 *  \return VOID
 *
 */
VOID impeghd_pol_2_cart_degree(ia_polar_coord_str *ptr_polar_coord,
                               ia_cart_coord_str *ptr_cart_coord)
{
  FLOAT32 sin_phi = (FLOAT32)(sin(ptr_polar_coord->phi * DEG_2_RAD));
  FLOAT32 cos_phi = (FLOAT32)(cos(ptr_polar_coord->phi * DEG_2_RAD));
  FLOAT32 sin_theta = (FLOAT32)(sin(ptr_polar_coord->theta * DEG_2_RAD));
  FLOAT32 cos_theta = (FLOAT32)(cos(ptr_polar_coord->theta * DEG_2_RAD));
  FLOAT32 r = ptr_polar_coord->radius;
  ptr_cart_coord->z = ia_mul_flt(r, sin_phi);
  ptr_cart_coord->y = ia_mul_flt(ia_mul_flt(r, sin_theta), cos_phi);
  ptr_cart_coord->x = ia_mul_flt(ia_mul_flt(r, cos_theta), cos_phi);
}

/**
 *  impeghd_cart_2_pol
 *
 *  \brief Convert Cartesian to Polar co-ordinates
 *
 *  \param [in]  ptr_polar_coord    Pointer to Polar co-ordinates structure
 *  \param [out] ptr_cart_coord     Pointer to Cartesian co-ordinates structure
 *
 *  \return VOID
 *
 */
VOID impeghd_cart_2_pol(ia_cart_coord_str *pstr_cart_coord, ia_polar_coord_str *pstr_polar_str)
{
  FLOAT32 vec_len = (FLOAT32)ia_sqrt_flt(
      ia_add_flt(ia_mul_flt(pstr_cart_coord->x, pstr_cart_coord->x),
                 ia_add_flt(ia_mul_flt(pstr_cart_coord->y, pstr_cart_coord->y),
                            ia_mul_flt(pstr_cart_coord->z, pstr_cart_coord->z))));
  FLOAT32 vec_xy_proj = (FLOAT32)ia_sqrt_flt((pstr_cart_coord->x * pstr_cart_coord->x) +
                                             (pstr_cart_coord->y * pstr_cart_coord->y));
  if (vec_len != 0.0f)
  {
    pstr_polar_str->radius = vec_len;
    pstr_polar_str->phi = (FLOAT32)acos(vec_xy_proj / vec_len);
    if (vec_xy_proj != 0)
      pstr_polar_str->phi = (FLOAT32)acos(pstr_cart_coord->x / vec_xy_proj);
  }
  else
  {
    memset(pstr_polar_str, 0, sizeof(*pstr_polar_str));
  }
}

/* g1 * V1 + g2 * V2 */
/**
 *  impeghd_scaled_vec_addition_cart
 *
 *  \brief Scaled vector addition of Cartesian co-ordinates
 *
 *  \param [in]  ptr_vec_a      Pointer to vector a
 *  \param [in]  gain_a         Gain for vector a
 *  \param [in]  ptr_vec_b      Pointer to vector b
 *  \param [in]  gain_b         Gain for vector a
 *  \param [out] ptr_vec_res    Pointer to scaled addition output vector
 *
 *  \return VOID
 *
 */
VOID impeghd_scaled_vec_addition_cart(ia_cart_coord_str *ptr_vec_a, FLOAT32 gain_a,
                                      ia_cart_coord_str *ptr_vec_b, FLOAT32 gain_b,
                                      ia_cart_coord_str *ptr_vec_res)
{
  ptr_vec_res->z = ia_add_flt(ia_mul_flt(gain_a, ptr_vec_a->z), ia_mul_flt(gain_b, ptr_vec_b->z));
  ptr_vec_res->y = ia_add_flt(ia_mul_flt(gain_a, ptr_vec_a->y), ia_mul_flt(gain_b, ptr_vec_b->y));
  ptr_vec_res->x = ia_add_flt(ia_mul_flt(gain_a, ptr_vec_a->x), ia_mul_flt(gain_b, ptr_vec_b->x));
  return;
}

/**
 *  impeghd_get_vx_proj_len
 *
 *  \brief Get vector projection length
 *
 *  \param [in] v1      Pointer to vector 1
 *  \param [in] v2      Pointer to vector 2
 *  \param [in] v3      Pointer to vector 3
 *  \param [in] vx      Pointer to vector x
 *
 *  \return WORD32     Vector projection length
 *
 */

FLOAT32 impeghd_get_vx_proj_len(ia_cart_coord_str *v1, ia_cart_coord_str *v2,
                                ia_cart_coord_str *v3, ia_cart_coord_str *vx)
{
  FLOAT32 vector_length;
  FLOAT32 result;
  ia_cart_coord_str v_orth, v12, v13, vx1;
  vx1.x = ia_sub_flt(vx->x, v1->x);
  vx1.y = ia_sub_flt(vx->y, v1->y);
  vx1.z = ia_sub_flt(vx->z, v1->z);
  v12.x = ia_sub_flt(v1->x, v2->x);
  v12.y = ia_sub_flt(v1->y, v2->y);
  v12.z = ia_sub_flt(v1->z, v2->z);
  v13.x = ia_sub_flt(v1->x, v3->x);
  v13.y = ia_sub_flt(v1->y, v3->y);
  v13.z = ia_sub_flt(v1->z, v3->z);

  v_orth.x = ia_sub_flt(ia_mul_flt(v12.y, v13.z), ia_mul_flt(v12.z, v13.y));
  v_orth.y = ia_sub_flt(ia_mul_flt(v12.z, v13.x), ia_mul_flt(v12.x, v13.z));
  v_orth.z = ia_sub_flt(ia_mul_flt(v12.x, v13.y), ia_mul_flt(v12.y, v13.x));
  vector_length = (FLOAT32)ia_sqrt_flt(ia_add_flt(
      ia_mul_flt((v_orth.x), (v_orth.x)),
      ia_add_flt(ia_mul_flt((v_orth.y), (v_orth.y)), ia_mul_flt((v_orth.z), (v_orth.z)))));
  if (vector_length != 0.0f)
  {
    FLOAT32 one_by_vlen = 1.0f / vector_length;
    v_orth.x = ia_mul_flt((v_orth.x), one_by_vlen);
    v_orth.y = ia_mul_flt((v_orth.y), one_by_vlen);
    v_orth.z = ia_mul_flt((v_orth.z), one_by_vlen);
  }
  result = ia_mul_flt(v_orth.x, vx1.x);
  result = ia_mac_flt(result, v_orth.y, vx1.y);
  result = ia_mac_flt(result, v_orth.z, vx1.z);

  return result;
}
/** @} */ /* End of 3DVecOps */