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
#include <impeghd_fft_ifft.h>
#include "impeghd_intrinsics_flt.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_func_def.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_pre_twid
 *
 *  \brief Carries out pre twiddle processing as part of MDCT realization through FFT.
 *
 *  \param [in]  in              Pointer to input buffer.
 *  \param [out] r_ptr           Pointer to real buffer of output.
 *  \param [out] i_ptr           Pointer to imaginary buffer of output.
 *  \param [in]  nlength         Length of input.
 *  \param [in]  ptr_pre_cos_sin Pointer to cos sin values used by pretwiddle function.
 *
 *
 *
 */
static VOID ia_core_coder_pre_twid(FLOAT32 *in, FLOAT32 *r_ptr, FLOAT32 *i_ptr, WORD32 nlength,
                                   const FLOAT32 *ptr_pre_cos_sin)
{
  WORD32 idx;

  const FLOAT32 *cos_ptr = &ptr_pre_cos_sin[0];
  const FLOAT32 *sin_ptr = &ptr_pre_cos_sin[nlength];

  for (idx = 0; idx < nlength; idx += 4)
  {
    *r_ptr++ = ia_msu_flt(ia_mul_flt(in[idx], cos_ptr[idx]), in[nlength + idx], sin_ptr[idx]);
    *i_ptr++ = ia_mac_flt(ia_mul_flt(in[idx], sin_ptr[idx]), in[nlength + idx], cos_ptr[idx]);

    *r_ptr++ = ia_msu_flt(ia_mul_flt(in[idx + 1], cos_ptr[idx + 1]), in[nlength + idx + 1],
                          sin_ptr[idx + 1]);
    *i_ptr++ = ia_mac_flt(ia_mul_flt(in[idx + 1], sin_ptr[idx + 1]), in[nlength + idx + 1],
                          cos_ptr[idx + 1]);

    *r_ptr++ = ia_msu_flt(ia_mul_flt(in[idx + 2], cos_ptr[idx + 2]), in[nlength + idx + 2],
                          sin_ptr[idx + 2]);
    *i_ptr++ = ia_mac_flt(ia_mul_flt(in[idx + 2], sin_ptr[idx + 2]), in[nlength + idx + 2],
                          cos_ptr[idx + 2]);

    *r_ptr++ = ia_msu_flt(ia_mul_flt(in[idx + 3], cos_ptr[idx + 3]), in[nlength + idx + 3],
                          sin_ptr[idx + 3]);
    *i_ptr++ = ia_mac_flt(ia_mul_flt(in[idx + 3], sin_ptr[idx + 3]), in[nlength + idx + 3],
                          cos_ptr[idx + 3]);
  }
  return;
}

/**
 *  ia_core_coder_post_twid
 *
 *  \brief Twiddle factor multiplicaton done on the output of FFT/IFFT
 *
 *  \param [in]  data_re          Pointer to real part of input data.
 *  \param [in]  data_im          Pointer to imaginary part of input data.
 *  \param [out] out              Pointer to output buffer.
 *  \param [in]  nlength          Length of the FFT
 *  \param [in]  ptr_post_cos_sin Pointer to cosine sine values used by the function.
 *
 *
 *
 */
static VOID ia_core_coder_post_twid(FLOAT32 *data_re, FLOAT32 *data_im, FLOAT32 *out,
                                    WORD32 nlength, const FLOAT32 *ptr_post_cos_sin)
{
  WORD32 idx;

  const FLOAT32 *cos_ptr = &ptr_post_cos_sin[nlength * 2];
  const FLOAT32 *sin_ptr = &ptr_post_cos_sin[nlength * 3];

  FLOAT32 *out_ptr = &out[2 * nlength - 1];
  for (idx = 0; idx < nlength; idx += 4)
  {
    out[0] = ia_msu_flt(ia_mul_flt(data_re[idx], cos_ptr[idx]), data_im[idx], sin_ptr[idx]);
    out_ptr[0] = ia_negate_flt(
        ia_mac_flt(ia_mul_flt(data_re[idx], sin_ptr[idx]), data_im[idx], cos_ptr[idx]));

    out[2] = ia_msu_flt(ia_mul_flt(data_re[idx + 1], cos_ptr[idx + 1]), data_im[idx + 1],
                        sin_ptr[idx + 1]);
    out_ptr[-2] = ia_negate_flt(ia_mac_flt(ia_mul_flt(data_re[idx + 1], sin_ptr[idx + 1]),
                                           data_im[idx + 1], cos_ptr[idx + 1]));

    out[4] = ia_msu_flt(ia_mul_flt(data_re[idx + 2], cos_ptr[idx + 2]), data_im[idx + 2],
                        sin_ptr[idx + 2]);
    out_ptr[-4] = ia_negate_flt(ia_mac_flt(ia_mul_flt(data_re[idx + 2], sin_ptr[idx + 2]),
                                           data_im[idx + 2], cos_ptr[idx + 2]));

    out[6] = ia_msu_flt(ia_mul_flt(data_re[idx + 3], cos_ptr[idx + 3]), data_im[idx + 3],
                        sin_ptr[idx + 3]);
    out_ptr[-6] = ia_negate_flt(ia_mac_flt(ia_mul_flt(data_re[idx + 3], sin_ptr[idx + 3]),
                                           data_im[idx + 3], cos_ptr[idx + 3]));

    out += 8;
    out_ptr -= 8;
  }
  return;
}

/**
 *  ia_core_coder_acelp_mdct
 *
 *  \brief Carries out MDCT in the ACELP portion
 *
 *  \param [in]  ptr_in          Pointer to input buffer.
 *  \param [out] ptr_out         Pointer to ouptut buffer.
 *  \param [in]  length          MDCT length related parameter.
 *  \param [in]  ptr_scratch     Pointer to MDCT scratch buffer.
 *  \param [in]  ptr_fft_scratch Pointer to FFT scratch buffer.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE ia_core_coder_acelp_mdct(FLOAT32 *ptr_in, FLOAT32 *ptr_out, WORD32 length,
                                      FLOAT32 *ptr_scratch, FLOAT32 *ptr_fft_scratch)
{
  FLOAT32 *ptr_data_r = ptr_scratch;
  FLOAT32 *ptr_data_i = ptr_scratch + (length >> 1);
  FLOAT32 *pin_tmp = ptr_fft_scratch + 2 * length;
  FLOAT32 *in_ptr;
  WORD32 idx;
  const FLOAT32 *ptr_pre_post_twid;
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;

  switch (length)
  {
  case 128:
    ptr_pre_post_twid = &ia_core_coder_pre_post_twid_cos_sin_64[0][0];
    break;
  case 256:
    ptr_pre_post_twid = &ia_core_coder_pre_post_twid_cos_sin_128[0][0];
    break;
  case 512:
    ptr_pre_post_twid = &ia_core_coder_pre_post_twid_cos_sin_256[0][0];
    break;
  case 1024:
    ptr_pre_post_twid = &ia_core_coder_pre_post_twid_cos_sin_512[0][0];
    break;
  default:
    ptr_pre_post_twid = &ia_core_coder_pre_post_twid_cos_sin_64[0][0];
    break;
  }
  for (idx = 0; idx < (length >> 1); idx++)
  {
    pin_tmp[idx] = ptr_in[2 * idx];
    pin_tmp[(length >> 1) + idx] = ptr_in[length - 1 - 2 * idx];
  }

  in_ptr = &pin_tmp[0];

  ia_core_coder_pre_twid(in_ptr, ptr_data_r, ptr_data_i, (length >> 1), ptr_pre_post_twid);

  impeghd_rad2_cplx_fft(ptr_data_r, ptr_data_i, (length >> 1), ptr_fft_scratch);

  ia_core_coder_post_twid(ptr_data_r, ptr_data_i, ptr_out, (length >> 1), ptr_pre_post_twid);

  return err;
}

/**
 *  ia_core_coder_acelp_mdct_main
 *
 *  \brief ACELP MDCT main function.
 *
 *  \param [in,out] usac_data     Pointer to USAC data structure.
 *  \param [in]     ptr_in        Pointer to input buffer.
 *  \param [out]    ptr_out       Pointer to output buffer.
 *  \param [in]     l             MDCT length related parameter.
 *  \param [in]     m             MDCT length related parameter.
 *  \param [in]     scratch       Pointer to scratch buffer.
 *
 *  \return IA_ERRORCODE                Processing error if any.
 *
 */
IA_ERRORCODE ia_core_coder_acelp_mdct_main(ia_usac_data_struct *usac_data, FLOAT32 *ptr_in,
                                           FLOAT32 *ptr_out, WORD32 l, WORD32 m, FLOAT32 *scratch)

{
  WORD32 idx;
  FLOAT32 *ptr_scratch = &usac_data->scratch_buffer_float[0];
  FLOAT32 *output_buffer = &usac_data->x_ac_dec_float[0];
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;

  err = ia_core_coder_acelp_mdct(ptr_in, output_buffer, l + m, ptr_scratch, scratch);

  if (err == -1)
  {
    return err;
  }

  for (idx = 0; idx < (m >> 1); idx++)
  {
    ptr_out[l + (m >> 1) - 1 - idx] = ia_negate_flt(output_buffer[(m >> 1) + (l >> 1) + idx]);
    ptr_out[l + (m >> 1) + idx] = ia_negate_flt(output_buffer[(m >> 1) + (l >> 1) - 1 - idx]);
  }
  for (idx = 0; idx < (l >> 1); idx++)
  {
    ptr_out[idx] = output_buffer[m + (l >> 1) + idx];
    ptr_out[l - 1 - idx] = ia_negate_flt(output_buffer[m + (l >> 1) + idx]);
    ptr_out[l + m + idx] = ia_negate_flt(output_buffer[(l >> 1) - 1 - idx]);
    ptr_out[2 * l + m - 1 - idx] = ia_negate_flt(output_buffer[(l >> 1) - 1 - idx]);
  }
  return err;
}
/** @} */ /* End of CoreDecProc */