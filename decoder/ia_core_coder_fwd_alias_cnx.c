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
#include "impeghd_intrinsics_flt.h"
#include <ia_core_coder_constants.h>
#include <ia_core_coder_basic_ops32.h>
#include <ia_core_coder_basic_ops40.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_acelp_com.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_fwd_alias_cancel_tool
 *
 *  \brief Forward alias cancellation tool
 *
 *  \param [in]        usac_data        USAC datastructure
 *  \param [in]        fac_prm          USAC FAC data pointer
 *  \param [in]        td_config        TD config structure
 *  \param [in]        fac_length       fac length
 *  \param [in]        lp_filt_coeff    LPC flter coeff
 *  \param [in,out]    fac_time_sig     fac time signal
 *  \param [in,out]    fac_time_sig_fb  fac time signal for full band lpd
 *  \param [in]        fscale           fscale value
 *  \param [in]        scratch          Scratch buffer for internal operations
 *  \param [in]        zir              Zero Impulse response
 *  \param [in]        stereo_lpd       Stereo lpd index
 *  \param [in]        len_subfrm       Subframe length
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_fwd_alias_cancel_tool(ia_usac_data_struct *usac_data, WORD32 *fac_prm,
                                                 ia_usac_td_config_handle td_config,
                                                 WORD32 fac_length, FLOAT32 *lp_filt_coeff,
                                                 FLOAT32 *fac_time_sig, FLOAT32 *fac_time_sig_fb,
                                                 WORD32 fscale, FLOAT32 *scratch, FLOAT32 *zir,
                                                 WORD32 stereo_lpd, WORD32 len_subfrm)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 fac_idx;
  WORD32 lfac_fb;
  WORD32 fac_length_2;
  WORD32 full_band_lpd = td_config->full_band_lpd;
  WORD32 fscale_full_band = td_config->fscale_full_band;
  WORD32 rs_delay;

  FLOAT32 gain;

  FLOAT32 lp_coeff[ORDER + 1];
  FLOAT32 fac_win[2 * FAC_LENGTH];
  FLOAT32 fac_gain[128];
  FLOAT32 *ptr_fac_signal_flt = scratch;
  FLOAT32 *ptr_scratch = ptr_fac_signal_flt + ((fac_length << 1) + 16);
  FLOAT32 *tmp_scratch = ptr_scratch + usac_data->ccfl;
  FLOAT32 *tmp_fac;
  const FLOAT32 *sine_window = NULL;

  lfac_fb = fac_length;
  if (full_band_lpd)
  {
    fac_length = fac_length >> 1;
  }
  fac_length_2 = fac_length << 1;
  gain = ia_core_coder_pow_10_i_by_128[fac_prm[0]];
  /* gain multiplication */
  for (fac_idx = 0; fac_idx < fac_length; fac_idx++)
  {
    fac_gain[fac_idx] = ia_mul_flt((FLOAT32)fac_prm[fac_idx + 1], gain);
  }

  err = ia_core_coder_acelp_mdct(fac_gain, ptr_fac_signal_flt, fac_length, ptr_scratch,
                                 tmp_scratch);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }

  ia_core_coder_lpc_coeff_wt_apply(lp_filt_coeff, lp_coeff);

  ia_core_coder_memset(ptr_fac_signal_flt + fac_length, fac_length);

  ia_core_coder_synthesis_tool(lp_coeff, ptr_fac_signal_flt, fac_time_sig, fac_length_2,
                               ptr_fac_signal_flt + fac_length, tmp_scratch);

  /* add ACELP ZIR */
  if (zir != NULL && (stereo_lpd == 0 || full_band_lpd == 0))
  {
    switch (fac_length)
    {
    case 96:
      sine_window = ia_core_coder_sine_window192;
      break;
    case 64:
      sine_window = ia_core_coder_sine_window128;
      break;
    case 48:
      sine_window = ia_core_coder_sine_window96;
      break;
    case 32:
      sine_window = ia_core_coder_sine_window64;
      break;
    default:
      sine_window = ia_core_coder_sine_window256;
      break;
    }

    for (fac_idx = 0; fac_idx < fac_length; fac_idx++)
    {
      fac_win[fac_idx] =
          ia_mul_flt(sine_window[fac_idx], sine_window[fac_length_2 - 1 - fac_idx]);
      fac_win[fac_length + fac_idx] = ia_sub_flt(
          1.0f, ia_mul_flt(sine_window[fac_length + fac_idx], sine_window[fac_length + fac_idx]));
    }
    for (fac_idx = 0; fac_idx < fac_length; fac_idx++)
    {
      fac_time_sig[fac_idx] = ia_add_flt(
          ia_add_flt(fac_time_sig[fac_idx], ia_mul_flt(zir[1 + (len_subfrm / 2) + fac_idx],
                                                       fac_win[fac_length + fac_idx])),
          ia_mul_flt(zir[1 + (len_subfrm / 2) - 1 - fac_idx], fac_win[fac_length - 1 - fac_idx]));
    }
  }
  tmp_fac = tmp_scratch;
  tmp_scratch = tmp_fac + (2 * FAC_LENGTH + L_FILT_MAX);

  if (full_band_lpd)
  {
    rs_delay = L_FILT_MAX;
    ia_core_coder_mem_cpy(fac_time_sig, tmp_fac, fac_length_2);
    ia_core_coder_memset(tmp_fac + (fac_length_2), rs_delay >> 1);
    ia_core_coder_td_resampler(tmp_fac, (WORD16)(2 * fac_length), fscale, fac_time_sig_fb,
                               fscale_full_band, NULL, 0, tmp_scratch);
    if (stereo_lpd == 1 && full_band_lpd == 1 && zir != NULL)
    {
      WORD32 len_subfrm_fb = len_subfrm << 1;

      switch (fac_length)
      {
      case 64:
        sine_window = ia_core_coder_sine_window256;
        break;
      case 32:
        sine_window = ia_core_coder_sine_window128;
        break;
      default:
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_FAC_LEN;
        break;
      }

      for (fac_idx = 0; fac_idx < lfac_fb; fac_idx++)
      {
        fac_win[fac_idx] =
            ia_mul_flt(sine_window[fac_idx], sine_window[(lfac_fb << 1) - 1 - fac_idx]);
        fac_win[lfac_fb + fac_idx] = ia_sub_flt(
            1.0f, ia_mul_flt(sine_window[lfac_fb + fac_idx], sine_window[lfac_fb + fac_idx]));
      }

      for (fac_idx = 0; fac_idx < lfac_fb; fac_idx++)
      {
        fac_time_sig_fb[fac_idx] =
            ia_add_flt(ia_add_flt(fac_time_sig_fb[fac_idx],
                                  ia_mul_flt(zir[1 + (len_subfrm_fb / 2) + fac_idx],
                                             fac_win[lfac_fb + fac_idx])),
                       ia_mul_flt(zir[1 + (len_subfrm_fb / 2) - 1 - fac_idx],
                                  fac_win[lfac_fb - 1 - fac_idx]));
      }
    }
  }
  return err;
}
/** @} */ /* End of CoreDecProc */