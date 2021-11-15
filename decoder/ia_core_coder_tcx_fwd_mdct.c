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
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include <impeghd_type_def.h>

#include "impeghd_error_codes.h"
#include <impeghd_fft_ifft.h>
#include "impeghd_intrinsics_flt.h"

#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_lsp_to_lp_conversion
 *
 *  \brief lsp to lpc coefficient conversion
 *
 *  \param [in]		lsp				LSP coefficients
 *  \param [out]	lp_flt_coff_a	LPC filter coefficient
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_lsp_to_lp_conversion(FLOAT32 *lsp, FLOAT32 *lp_flt_coff_a)
{
  WORD32 i, j;
  FLOAT32 a1, a2;
  FLOAT32 *ppoly_f1, *ppoly_f2, *plsp;
  FLOAT32 *plp_flt_coff_a_bott, *plp_flt_coff_a_top;
  FLOAT32 poly1[ORDER_BY_2 + 2], poly2[ORDER_BY_2 + 2];
  FLOAT32 *ppoly1, *ppoly2;

  poly1[0] = 0.0f;
  poly1[1] = 1.0f;
  poly2[0] = 0.0f;
  poly2[1] = 1.0f;
  plsp = lsp;

  ppoly1 = &poly1[1];
  ppoly2 = &poly2[1];

  for (i = 1; i <= ORDER_BY_2; i++)
  {
    a1 = ia_mul_flt(-2.0f, (*plsp++));
    a2 = ia_mul_flt(-2.0f, (*plsp++));

    ppoly2[i] = ia_mac_flt((ia_mul_flt(a2, ppoly2[i - 1])), 2.0f, ppoly2[i - 2]);
    ppoly1[i] = ia_mac_flt((ia_mul_flt(a1, ppoly1[i - 1])), 2.0f, ppoly1[i - 2]);

    for (j = i - 1; j > 0; j--)
    {
      ppoly2[j] = ia_add_flt(ppoly2[j], (ia_mac_flt(ppoly2[j - 2], a2, ppoly2[j - 1])));
      ppoly1[j] = ia_add_flt(ppoly1[j], (ia_mac_flt(ppoly1[j - 2], a1, ppoly1[j - 1])));
    }
  }

  ppoly_f2 = poly2 + ORDER_BY_2 + 1;
  ppoly_f1 = poly1 + ORDER_BY_2 + 1;

  for (i = 0; i < ORDER_BY_2; i++)
  {
    ppoly_f2[0] = ia_sub_flt(ppoly_f2[0], ppoly_f2[-1]);
    ppoly_f2--;
    ppoly_f1[0] = ia_add_flt(ppoly_f1[0], ppoly_f1[-1]);
    ppoly_f1--;
  }

  plp_flt_coff_a_bott = lp_flt_coff_a;
  *plp_flt_coff_a_bott++ = 1.0f;
  plp_flt_coff_a_top = lp_flt_coff_a + ORDER;
  ppoly_f1 = poly1 + 2;
  ppoly_f2 = poly2 + 2;
  for (i = 0; i < ORDER_BY_2; i++)
  {
    *plp_flt_coff_a_top-- = ia_mul_flt(0.5f, ia_sub_flt(*ppoly_f1, *ppoly_f2));
    *plp_flt_coff_a_bott++ = ia_mul_flt(0.5f, ia_add_flt(*ppoly_f1++, *ppoly_f2++));
  }

  return;
}
/**
 *  ia_core_coder_lpc_to_mdct
 *
 *  \brief LPC coeffs to mdct gain factor transformation for noise shaping calculations
 *
 *  \param [in]		coeff		LPC coeff
 *  \param [in]		order		Order of filter
 *  \param [out]	gains		Gain
 *  \param [in]		lg			Subframe length
 *  \param [in]		ptr_scratch	Scratch buffer
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_lpc_to_mdct(FLOAT32 *coeff, WORD32 order, FLOAT32 *gains, WORD32 lg,
                                       FLOAT32 *ptr_scratch)
{
  FLOAT32 tmp;
  FLOAT64 avg_fac;
  WORD32 i, size_n;
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;

  ia_core_coder_lpc_scratch_t *lpc_scratch = (ia_core_coder_lpc_scratch_t *)ptr_scratch;

  ptr_scratch += LEN_SUPERFRAME * 4;

  size_n = 2 * lg;
  avg_fac = PI / (FLOAT32)(size_n);

  for (i = 0; i < order + 1; i++)
  {
    tmp = (FLOAT32)ia_mul_double_flt(avg_fac, ((FLOAT32)i));
    lpc_scratch->data_i[i] = (FLOAT32)ia_mul_double_flt(sin(tmp), -coeff[i]);
    lpc_scratch->data_r[i] = (FLOAT32)ia_mul_double_flt(cos(tmp), coeff[i]);
  }

  ia_core_coder_memset(&lpc_scratch->data_i[i], size_n - i);
  ia_core_coder_memset(&lpc_scratch->data_r[i], size_n - i);

  impeghd_rad2_cplx_fft(lpc_scratch->data_r, lpc_scratch->data_i, size_n, ptr_scratch);

  for (i = 0; i < lg; i++)
  {
    gains[i] = (FLOAT32)(1.0f / ia_sqrt_flt(ia_mac_flt(
                                    (ia_mul_flt(lpc_scratch->data_r[i], lpc_scratch->data_r[i])),
                                    lpc_scratch->data_i[i], lpc_scratch->data_i[i])));
  }
  return err;
}
/**
 *  ia_core_coder_noise_shaping
 *
 *  \brief Interpolates the noise between 2 spectral represetation of noise
 *
 *  \param [in/out]		r			Input/output
 *  \param [in]			lg			subframe length
 *  \param [in]			fdns_pts	Noise shaping resolution
 *  \param [in]			gain1		Lower gain
 *  \param [in]			gain2		Upper gain
 *  \param [in]			ptr_scratch	Scratch buffer
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_noise_shaping(FLOAT32 *r, WORD32 lg, WORD32 fdns_pts, FLOAT32 *gain1,
                                 FLOAT32 *gain2, FLOAT32 *ptr_scratch)
{
  WORD32 i, j, k;
  FLOAT32 rr_prev, a = 0, b = 0;
  FLOAT32 *rr = ptr_scratch;

  k = lg / fdns_pts;

  rr_prev = 0;

  ia_core_coder_mem_cpy(r, rr, lg);

  for (i = 0; i < lg; i++)
  {
    if ((i % k) == 0)
    {
      j = i / k;
      b = ia_sub_flt(gain2[j], gain1[j]) / ia_add_flt(gain2[j], gain1[j]);
      a = ia_mul_flt(gain2[j], ia_mul_flt(2.0f, gain1[j])) / ia_add_flt(gain2[j], gain1[j]);
    }

    rr[i] = ia_add_flt(ia_mul_flt(b, rr_prev), ia_mul_flt(a, rr[i]));
    rr_prev = rr[i];
  }

  ia_core_coder_mem_cpy(rr, r, lg);

  return;
}
/**
 *  ia_core_coder_interpolate_lpc_coef
 *
 *  \brief Find TCX interpolated LPC coeff in every subframes from the LSF coeff
 *
 *  \param [in]			lsf_old		Previous Line Spectral Frequencies
 *  \param [in]			lsf_new		Present Line Spectral Frequencies
 *  \param [out]		a			interpolated LPC coeff
 *  \param [in]			nb_subfr	Number of subframe
 *  \param [in]			m			ORDER
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_interpolate_lpc_coef(FLOAT32 *lsf_prev, FLOAT32 *lsf_curr, FLOAT32 *lpc_coeff,
                                        WORD32 nb_subfr, WORD32 order)
{

  FLOAT32 lsf[ORDER], *ptr_lpc;
  FLOAT32 inc, fcurr, fprev;
  WORD32 i;

  inc = 1.0f / (FLOAT32)nb_subfr;
  fcurr = ia_sub_flt(0.5f, ia_mul_flt(0.5f, inc));
  fprev = ia_sub_flt(1.0f, fcurr);

  for (i = 0; i < order; i++)
  {
    lsf[i] = ia_add_flt(ia_mul_flt(lsf_curr[i], fcurr), ia_mul_flt(lsf_prev[i], fprev));
  }

  ptr_lpc = lpc_coeff + (order << 1) + 2;
  ia_core_coder_lsp_to_lp_conversion(lsf_curr, ptr_lpc);
  ptr_lpc = lpc_coeff + order + 1;
  ia_core_coder_lsp_to_lp_conversion(lsf_prev, ptr_lpc);
  ia_core_coder_lsp_to_lp_conversion(lsf, lpc_coeff);

  return;
}

/**
 *  ia_core_coder_interpolate_lsp_params
 *
 *  \brief Interpolate LSP coeffs in every subframe
 *
 *  \param [in]			lsp_old			Previous LSP coeff
 *  \param [in]			lsp_new			Present LSP coeff
 *  \param [in]			lp_flt_coff_a	LPC coeff
 *  \param [in]			nb_subfr		Number of subframes
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_interpolate_lsp_params(FLOAT32 *lsp_prev, FLOAT32 *lsp_curr,
                                          FLOAT32 *lp_flt_coff_a, WORD32 nb_subfr)
{
  FLOAT32 lsp[ORDER];
  FLOAT32 factor;
  WORD32 i, k;
  FLOAT32 x_plus_y, x_minus_y;

  factor = 1.0f / (FLOAT32)nb_subfr;

  x_plus_y = ia_mul_flt(0.5f, factor);

  for (k = 0; k < nb_subfr; k++)
  {
    x_minus_y = ia_sub_flt(1.0f, x_plus_y);
    for (i = 0; i < ORDER; i++)
    {
      lsp[i] = ia_add_flt(ia_mul_flt(lsp_curr[i], x_plus_y), ia_mul_flt(lsp_prev[i], x_minus_y));
    }
    x_plus_y = ia_add_flt(x_plus_y, factor);

    ia_core_coder_lsp_to_lp_conversion(lsp, lp_flt_coff_a);

    lp_flt_coff_a += (ORDER + 1);
  }

  ia_core_coder_lsp_to_lp_conversion(lsp_curr, lp_flt_coff_a);

  return;
}
/** @} */ /* End of CoreDecProc */