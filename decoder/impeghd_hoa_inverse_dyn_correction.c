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

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_common_functions.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_frame_params.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "impeghd_hoa_rom.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_inverse_dyn_correction.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup HOAProc HOA Processing
 * @ingroup  HOAProc
 * @brief HOA Processing
 *
 * @{
 */

/**
 *  impeghd_hoa_inv_dyn_correction_process
 *
 *  \brief HOA inverse dynamic correction processing
 *
 *  \param [in]      ptr_inp    Pointer to input buffer
 *  \param [in,out]  pstr_spatial_dec Pointer to spatial decoder handle
 *
 *  \return IA_ERRORCODE   Error
 *
 */
IA_ERRORCODE impeghd_hoa_inv_dyn_correction_process(FLOAT32 *ptr_inp,
                                                    ia_spatial_dec_str *pstr_spatial_dec)
{
  UWORD32 ch, frame_sz = pstr_spatial_dec->ia_hoa_config->frame_length;
  FLOAT32 *ptr_out = &pstr_spatial_dec->dyn_corr_sample_buf[0];
  FLOAT32 *d_win_func = pstr_spatial_dec->inv_dyn_corr_win_func;
  ia_hoa_dec_frame_param_str *ptr_frame_param = &(pstr_spatial_dec->frame_params_handle);
  for (ch = 0; ch < pstr_spatial_dec->ia_hoa_config->num_transport_ch; ch++)
  {
    FLOAT32 tmp;
    UWORD32 i;
    WORD32 is_exception = ptr_frame_param->ptr_is_exception[ch];
    FLOAT32 last_gain = pstr_spatial_dec->inv_dyn_corr_prev_gain[ch];
    WORD32 exp = ptr_frame_param->exponents[ch];
    WORD32 offset = (exp < 0) ? (ia_abs_int(exp) + 5) : (exp - 1);
    FLOAT32 reci_last_gain = (FLOAT32)(1.0 / (last_gain));

    offset = ia_min_int(offset, 6);
    /** Occurs only when hoa frame data is not available.
    * Use the same table as exp = -1 in such case.*/
    if (exp == 0)
    {
      for (i = 0; i < frame_sz; i++)
      {
        ptr_out[i] = ia_mul_flt(reci_last_gain, ptr_inp[i]);
      }
    }
    else
    {
      if (is_exception)
      {
        FLOAT32 mult_fact = ia_mul_flt((impeghd_hoa_pow_2(exp)), reci_last_gain);
        for (i = 0; i < frame_sz; i++)
        {
          ptr_out[i] = ia_mul_flt(mult_fact, ptr_inp[i]);
        }
        last_gain = ia_mul_flt((impeghd_hoa_pow_2(-exp)), last_gain);
      }
      else
      {
        if (ia_abs_int(exp) > 6)
        {
          for (i = 0; i < frame_sz; i++)
          {
            tmp = ia_mul_flt(reci_last_gain, ptr_inp[i]);
            ptr_out[i] = ia_mul_flt((FLOAT32)(pow(d_win_func[i], exp)), tmp);
          }
          last_gain = ia_mul_flt(last_gain, (FLOAT32)(pow(d_win_func[frame_sz - 1], exp)));
        }
        else
        {
          for (i = 0; i < frame_sz; i++)
          {
            ptr_out[i] = ia_mul_flt(ia_mul_flt(ptr_inp[i], reci_last_gain),
                                    ia_hoa_inv_dyn_win_pow_table[offset][i]);
          }
          last_gain = last_gain / ia_hoa_inv_dyn_win_pow_table[offset][frame_sz - 1];
        }
      }
      pstr_spatial_dec->inv_dyn_corr_prev_gain[ch] = last_gain;
    }
    ptr_out += frame_sz;
    ptr_inp += frame_sz;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of HOAProc */