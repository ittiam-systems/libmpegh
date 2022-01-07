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

#include <stdio.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "impeghd_intrinsics_flt.h"
#include "impeghd_resampler.h"
#include "impeghd_resampler_rom.h"
#include "ia_core_coder_cnst.h"

#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_vec_baisc_ops.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_func_def.h"

/**
 * @defgroup ResampProc Resampler Processing
 * @ingroup  ResampProc
 * @brief Resampler Processing
 *
 * @{
 */

/**
 *  impeghd_resample
 *
 *  \brief According to Sec 4.8.2.2 of ISO_IEC_23008-3(2019 version)
 *         The LPD mode shall only be employed at 3D audio core coder
 *         sampling rates ≤ 32000 Hz. If ptr_inp_l signal is > 32000 Hz the
 *         encoder should resample ptr_inp_l signal to ≤ 32000 Hz
 *
 *  \param [in]  pstr_resampler Pointer to structure holding delayed input signal
 *  \param [in]  ptr_input      Pointer to input PCM samples
 *  \param [out] ptr_output     Pointer to output PCM samples
 *  \param [in]  ptr_scratch    Pointer to scratch buffer
 *
 *
 *
 */
VOID impeghd_resample(ia_resampler_struct *pstr_resampler, FLOAT32 *ptr_input,
                      FLOAT32 *ptr_output, FLOAT32 *ptr_scratch)
{
  WORD32 idx1, idx2, idx3, fac_down, fac_up;
  FLOAT32 *ptr_inp_l, *ptr_out_l;
  FLOAT32 *ptr_poly_filt_mem, *ptr_temp, *ptr_delay;
  FLOAT32 poly_filt_outputs[RESAMPLE_MAX_RATIO] = {0};
  const FLOAT32 *ptr_resamp_filt_tbl = &impeghd_resampler_filter[0];

  fac_down = pstr_resampler->fac_down;
  fac_up = pstr_resampler->fac_up;
  ptr_out_l = ptr_output;

  if (fac_up == 3)
  {
    ptr_resamp_filt_tbl = &impeghd_resampler_filter_3[0];
  }
  if (fac_down != 1)
  {
    ptr_out_l = ptr_scratch;
  }

  ptr_poly_filt_mem = &pstr_resampler->poly_filt_mem[0];
  for (idx1 = 0; idx1 < (pstr_resampler->input_length); idx1++)
  {
    const FLOAT32 *ptr_filt_coef = ptr_resamp_filt_tbl;
    ptr_delay = &pstr_resampler->delay_samples_upsamp[idx1];
    ia_core_coder_memset(poly_filt_outputs, RESAMPLE_MAX_RATIO);

    for (idx2 = 0; (idx1 + idx2) < (RESAMPLE_DELAY / fac_up); idx2++)
    {
      for (idx3 = 0; idx3 < fac_up; idx3++)
      {
        poly_filt_outputs[idx3] += ia_mul_flt(*ptr_delay, *ptr_filt_coef);
        ptr_filt_coef++;
      }
      ptr_delay++;
    }

    ptr_temp = &ptr_input[idx1 + idx2 - (RESAMPLE_DELAY / fac_up)];
    for (; idx2 < (RESAMPLE_FILTER_LEN / fac_up); idx2++)
    {
      for (idx3 = 0; idx3 < fac_up; idx3++)
      {
        poly_filt_outputs[idx3] += ia_mul_flt(*ptr_temp, *ptr_filt_coef);
        ptr_filt_coef++;
      }
      ptr_temp++;
    }

    for (idx2 = 0; idx2 < fac_up; idx2++)
    {
      *ptr_out_l = 0;
      for (idx3 = idx2; idx3 < (fac_up - 1); idx3++)
      {
        *ptr_out_l = (FLOAT32)ia_add_flt(*ptr_out_l, ptr_poly_filt_mem[idx3]);
      }
      for (idx3 = 0; idx3 <= idx2; idx3++)
      {
        *ptr_out_l = (FLOAT32)ia_add_flt(*ptr_out_l, poly_filt_outputs[idx3]);
      }
      ptr_out_l++;
    }
    for (idx3 = 1; idx3 < fac_up; idx3++)
    {
      ptr_poly_filt_mem[idx3 - 1] = poly_filt_outputs[idx3];
    }
  }

  /* Store delay samples for next frame */
  ia_core_coder_mem_cpy(&ptr_input[pstr_resampler->input_length - (RESAMPLE_DELAY / fac_up)],
                        &pstr_resampler->delay_samples_upsamp[0], (RESAMPLE_DELAY / fac_up));

  if (fac_down == 2)
  {
    FLOAT32 mac = 0, *ptr_temp, *ptr_delay;
    WORD32 idx2 = 0;
    ptr_inp_l = ptr_scratch;
    ptr_out_l = ptr_output;

    for (idx1 = 0; idx1 < (pstr_resampler->output_length * fac_down); idx1 += fac_down)
    {
      const FLOAT32 *ptr_filt_coef = &impeghd_resampler_filter[0];

      mac = 0;
      ptr_delay = &pstr_resampler->delay_samples[idx1];
      for (idx2 = 0; idx1 + idx2 < RESAMPLE_DELAY; idx2++)
      {
        mac += ia_mul_flt(*ptr_delay, *ptr_filt_coef);
        ptr_delay++;
        ptr_filt_coef++;
      }

      ptr_temp = &ptr_inp_l[idx1 + idx2 - RESAMPLE_DELAY];
      for (; idx2 < RESAMPLE_FILTER_LEN; idx2++)
      {
        mac += ia_mul_flt(*ptr_temp, *ptr_filt_coef);
        ptr_temp++;
        ptr_filt_coef++;
      }
      *ptr_out_l++ = mac;
    }

    /* Store delay samples for next frame */
    ia_core_coder_mem_cpy(&ptr_inp_l[(pstr_resampler->input_length * fac_up) - RESAMPLE_DELAY],
                          &pstr_resampler->delay_samples[0], RESAMPLE_DELAY);
  }

  return;
}
/** @} */ /* End of ResampProc */