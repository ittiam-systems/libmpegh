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

#include <impeghd_type_def.h>

#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_nfc_filtering.h"
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
 *  impeghd_hoa_ren_iir_ord_2_filter
 *
 *  \brief Filter second order cell of near field compensation
 *
 *  \param [in/out] handle	Second order cell handle
 *  \param [in]     x		Input array
 *  \param [out]    y		Output array
 *  \param [in]     length		Length of array
 *
 *  \return VOID
 *
 */
static VOID impeghd_hoa_ren_iir_ord_2_filter(pVOID handle, pFLOAT32 x, pFLOAT32 y, WORD32 length)
{
  ia_render_hoa_iir_2_str *iir_handle = (ia_render_hoa_iir_2_str *)handle;
  FLOAT32 x0;
  for (WORD32 bin = 0; bin < length; bin++)
  {
    x0 = x[bin];
    y[bin] = ia_add_flt(ia_sub_flt((ia_sub_flt((ia_mul_flt(iir_handle->x2, iir_handle->b2)),
                                               ia_mul_flt(iir_handle->y1, iir_handle->a1))),
                                   ia_mul_flt(iir_handle->y2, iir_handle->a2)),
                        ia_add_flt(ia_mul_flt(iir_handle->x1, iir_handle->b1),
                                   ia_mul_flt(x[bin], iir_handle->b0)));
    iir_handle->y2 = iir_handle->y1;
    iir_handle->y1 = y[bin];
    iir_handle->x2 = iir_handle->x1;
    iir_handle->x1 = x0;
  }
}

/**
 *  impeghd_hoa_ren_iir_ord_1_filter
 *
 *  \brief Filter first order cell of near field compensation
 *
 *  \param [in/out]	handle	First order cell handle
 *  \param [in]		x		Input array
 *  \param [out]	y		Output array
 *  \param [in]		length		Length of array
 *
 *  \return VOID
 *
 */
static VOID impeghd_hoa_ren_iir_ord_1_filter(pVOID handle, pFLOAT32 x, pFLOAT32 y, WORD32 length)
{
  FLOAT32 x0;
  ia_render_hoa_iir_1_str *iir_handle = (ia_render_hoa_iir_1_str *)handle;
  for (WORD32 bin = 0; bin < length; bin++)
  {
    x0 = x[bin];
    y[bin] = ia_add_flt(ia_sub_flt(ia_mul_flt(iir_handle->x1, iir_handle->b1),
                                   ia_mul_flt(iir_handle->y1, iir_handle->a1)),
                        ia_mul_flt(x[bin], iir_handle->b0));
    iir_handle->y1 = y[bin];
    iir_handle->x1 = x0;
  }
}

/**
 *  impeghd_hoa_ren_nfc_set_filter_params
 *
 *  \brief Set near field compensation parameters
 *
 *  \param [in/out]	handle	Near field compensation handle
 *  \param [in]		tnfc	Near field compensation time array
 *  \param [in]		tnfs	Near field scanner time array
 *
 *  \return VOID
 *
 */
VOID impeghd_hoa_ren_nfc_set_filter_params(pVOID handle, FLOAT32 tnfc, FLOAT32 tnfs)
{
  WORD32 ind1, n;
  FLOAT32 a0, a1, a2, b0, b1, b2, re_x, ab_x, tmp, coeff, alpha_den, alpha_num;
  ia_render_hoa_nfc_filtering_str *nfc_handle = (ia_render_hoa_nfc_filtering_str *)handle;
  nfc_handle->nfs_time = (FLOAT32)tnfs;
  nfc_handle->nfc_time = (FLOAT32)tnfc;

  alpha_num = 4 * nfc_handle->nfs_time;
  alpha_den = 4 * nfc_handle->nfc_time;

  coeff = 1.0f;

  ind1 = (nfc_handle->order * (nfc_handle->order - 1)) / 2;

  for (n = 0; n < nfc_handle->num_iir_2_filts; n++)
  {
    ab_x = (FLOAT32)ia_hoa_nf_roots_re_abs[ind1 + 2 * n + 1];
    re_x = (FLOAT32)ia_hoa_nf_roots_re_abs[ind1 + 2 * n];
    tmp = ab_x / alpha_num;
    tmp = ia_mul_flt(tmp, tmp);
    b1 = ia_sub_flt(tmp, 1) * 2;
    tmp += 1;
    b0 = -2 * re_x / alpha_num;
    b2 = ia_add_flt(tmp, -b0);
    b0 = ia_add_flt(tmp, b0);
    tmp = ab_x / alpha_den;
    tmp = ia_mul_flt(tmp, tmp);
    a1 = ia_sub_flt(tmp, 1) * 2;
    tmp += 1;
    a0 = -2 * re_x / alpha_den;
    a2 = ia_add_flt(tmp, -a0);
    a0 = ia_add_flt(tmp, a0);

    coeff = ia_mul_flt(coeff, b0 / a0);

    ia_render_hoa_iir_2_str *iir_handle = &(nfc_handle->str_iir_2[n]);
    iir_handle->a0 = 1;
    iir_handle->a1 = (FLOAT32)(a1 / a0);
    iir_handle->a2 = (FLOAT32)(a2 / a0);
    iir_handle->b0 = 1;
    iir_handle->b1 = (FLOAT32)(b1 / b0);
    iir_handle->b2 = (FLOAT32)(b2 / b0);
  }
  if (nfc_handle->num_iir_1_filts)
  {
    re_x = (FLOAT32)ia_hoa_nf_roots_re_abs[ind1 + nfc_handle->order - 1];
    tmp = re_x / alpha_num;
    b1 = -(1 + tmp);
    b0 = ia_sub_flt(1, tmp);
    tmp = re_x / alpha_den;
    a1 = -(1 + tmp);
    a0 = ia_sub_flt(1, tmp);

    coeff *= b0 / a0;
    ia_render_hoa_iir_1_str *iir_handle = &(nfc_handle->str_iir_1);
    iir_handle->a0 = 1;
    iir_handle->a1 = (FLOAT32)(a1 / a0);
    iir_handle->b0 = 1;
    iir_handle->b1 = (FLOAT32)(b1 / b0);
  }

  nfc_handle->global_gain = (FLOAT32)coeff;
}

/**
 *  impeghd_hoa_ren_nfc_filter_apply
 *
 *  \brief Filter near field compensation
 *
 *  \param [in/out]	handle	Near field compensation filter handle
 *  \param [in]		x		Input array
 *  \param [in/out]	y		Input and output array, inplace processing
 *  \param [in]		length		Length of array
 *
 *  \return VOID
 *
 */
VOID impeghd_hoa_ren_nfc_filter_apply(pVOID handle, pFLOAT32 x, pFLOAT32 y, UWORD32 length)
{
  ia_render_hoa_nfc_filtering_str *nfc_handle = (ia_render_hoa_nfc_filtering_str *)handle;
  if (nfc_handle->global_gain != 1.0 || y != x)
  {
    for (WORD32 bin = length - 1; bin >= 0; bin--)
    {
      y[bin] = ia_mul_flt(x[bin], nfc_handle->global_gain);
    }
  }
  for (WORD32 n = 0; n < nfc_handle->num_iir_2_filts; n++)
  {
    impeghd_hoa_ren_iir_ord_2_filter(&(nfc_handle->str_iir_2[n]), y, y, length);
  }
  if (nfc_handle->num_iir_1_filts)
  {
    impeghd_hoa_ren_iir_ord_1_filter(&(nfc_handle->str_iir_1), y, y, length);
  }
}
/** @} */ /* End of HOAProc */