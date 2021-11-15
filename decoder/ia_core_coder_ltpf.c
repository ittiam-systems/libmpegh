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
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_vec_baisc_ops.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_main.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_ltpf_dec_params
 *
 *  \brief Decode LTPF params such as gain, and pitch parameters
 *
 *  \param [in/out]    pitch_int    Integer part of pitch
 *  \param [in/out]    pitch_fr    Fractional part of pitch
 *  \param [in/out]    gain      gain
 *  \param [in]      ptr_ltpf_data  ltpf data structure
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_ltpf_dec_params(WORD32 *pitch_int, WORD32 *pitch_fr, FLOAT32 *gain,
                                          ia_ltpf_data_str *ptr_ltpf_data)
{
  WORD32 data_present = ptr_ltpf_data->data_present;

  if (data_present)
  {
    WORD32 pitch_lag_idx = ptr_ltpf_data->pitch_lag_idx;
    WORD32 pitch_min = ptr_ltpf_data->pitch_min;
    WORD32 pitch_fr1 = ptr_ltpf_data->pitch_fr1;
    WORD32 pitch_fr2 = ptr_ltpf_data->pitch_fr2;
    WORD32 pitch_res = ptr_ltpf_data->pitch_res;
    WORD32 pitch_fr2_mi_min_mul_res = (pitch_fr2 - pitch_min) * pitch_res;
    WORD32 pitch_fr1_mi_fr2 = pitch_fr1 - pitch_fr2;
    *pitch_fr = 0;

    if (pitch_lag_idx < pitch_fr2_mi_min_mul_res)
    {
      *pitch_int = pitch_min + (pitch_lag_idx / pitch_res);
      *pitch_fr = pitch_lag_idx - (*pitch_int - pitch_min) * pitch_res;
    }
    else if (pitch_lag_idx < (pitch_fr2_mi_min_mul_res + pitch_fr1_mi_fr2 * (pitch_res >> 1)))
    {
      *pitch_int = pitch_fr2 + pitch_lag_idx - pitch_fr2_mi_min_mul_res;
    }
    else
    {
      *pitch_int =
          (pitch_lag_idx - pitch_fr2_mi_min_mul_res - pitch_fr1_mi_fr2) * pitch_res + pitch_fr1;
    }

    *gain = ia_mul_flt((FLOAT32)(ptr_ltpf_data->gain_idx + 1), 0.0625f);
  }
  else
  {
    *pitch_int = 0;
    *pitch_fr = 0;
    *gain = 0.0f;
  }
  return;
}
/**
 *  ia_core_coder_ltpf_filter
 *
 *  \brief Performs LTPF filtering and generates output
 *
 *  \param [in]    input    Input buffer
 *  \param [out]  output    Output buffer
 *  \param [in]    length    frame length
 *  \param [in]    pitch_int  Integer part of pitch
 *  \param [in]    pitch_fr  Fractional part of pitch
 *  \param [in]    gain    gain
 *  \param [in]    gain_idx  gain index
 *  \param [in]    mode    mode
 *  \param [in]    zir      zero input response buffer
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_ltpf_filter(FLOAT32 *input, FLOAT32 *output, WORD32 length,
                                      WORD32 pitch_int, WORD32 pitch_fr, FLOAT32 gain,
                                      WORD32 gain_idx, WORD32 mode, FLOAT32 *zir)
{
  FLOAT32 s1, s2, alpha = 0.0f, step = 0.0f;
  WORD32 n, k;

  if (ia_eq_flt(gain, 0))
  {
    ia_core_coder_mem_cpy(input, output, length);
  }
  else
  {
    if (mode == 2)
    {
      alpha = 0.f;
      step = 1.f / (FLOAT32)length;
    }
    else if (mode == 3)
    {
      alpha = 1.f;
      step = -1.f / (FLOAT32)length;
    }

    for (n = 0; n < length; n++)
    {
      s1 = 0;
      for (k = 0; k < LTPF_NUM_FILT_COEF1; k++)
      {
        s1 = ia_add_flt(s1, ia_mul_flt(output[n - pitch_int + k - LTPF_NUM_FILT_COEF1 / 2],
                                       ia_ltpf_filter_coef1[pitch_fr][k]));
      }

      s2 = 0;
      for (k = 0; k < LTPF_NUM_FILT_COEF2; k++)
      {
        s2 = ia_add_flt(s2, ia_mul_flt(input[n - k], ia_ltpf_filter_coef2[gain_idx][k]));
      }

      if (mode == 0)
      {
        output[n] = ia_sub_flt(ia_add_flt(input[n], ia_mul_flt(gain, s1)),
                               ia_mul_flt(LTPF_ATT_FAC, ia_mul_flt(gain, s2)));
      }
      else if (mode == 1)
      {
        output[n] = ia_sub_flt(ia_sub_flt(ia_add_flt(input[n], ia_mul_flt(gain, s1)),
                                          ia_mul_flt(LTPF_ATT_FAC, ia_mul_flt(gain, s2))),
                               zir[n]);
      }
      else
      {
        output[n] = ia_add_flt(
            input[n],
            ia_mul_flt(alpha, ia_sub_flt(ia_mul_flt(gain, s1),
                                         ia_mul_flt(LTPF_ATT_FAC, ia_mul_flt(gain, s2)))));
      }

      if (mode > 1)
      {
        alpha = ia_add_flt(alpha, step);
      }
    }
  }

  return;
}
/**
 *  ia_core_coder_ltpf_get_lpc
 *
 *  \brief Calculates LPC coeffs using Levinson Durbin Algorithm
 *
 *  \param [in]    input      input buffer
 *  \param [in]    length      lag length under consideration
 *  \param [in/out]  lpc_coeffs    LPC coefficients
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_ltpf_get_lpc(FLOAT32 *input, WORD32 length, FLOAT32 *lpc_coeffs)
{
  WORD32 i, j;
  FLOAT32 s, r[LTPF_NUM_LPC_COEFFS + 1], rc[LTPF_NUM_LPC_COEFFS];
  FLOAT32 value, sum, sigma2;

  /* Autocorrelation */
  for (i = 0; i <= LTPF_NUM_LPC_COEFFS; i++)
  {
    s = 0.0;

    for (j = 0; j < length - i; j++)
    {
      s = ia_add_flt(s, ia_mul_flt(input[j - length], input[j - length + i]));
    }
    r[i] = s;
  }

  r[0] = ia_max_flt(r[0], 100.0f);
  r[0] = ia_mul_flt(r[0], 1.0001f);

  /* Levinson-Durbin */
  lpc_coeffs[0] = 1.0f;
  rc[0] = -r[1] / r[0];
  lpc_coeffs[1] = rc[0];
  sigma2 = ia_add_flt(r[0], ia_mul_flt(r[1], rc[0]));
  for (i = 2; i <= LTPF_NUM_LPC_COEFFS; i++)
  {
    sum = 0.0f;
    for (j = 0; j < i; j++)
    {
      sum = ia_add_flt(sum, ia_mul_flt(r[i - j], lpc_coeffs[j]));
    }
    rc[i - 1] = -sum / sigma2;
    sigma2 = ia_mul_flt(sigma2, ia_sub_flt(1.0f, ia_mul_flt(rc[i - 1], rc[i - 1])));
    if (ia_lteq_flt(sigma2, 1.0E-09f))
    {
      ia_core_coder_memset(&rc[i - 1], (LTPF_NUM_LPC_COEFFS + 1 - i));
      ia_core_coder_memset(&lpc_coeffs[i], (LTPF_NUM_LPC_COEFFS + 1 - i));
      return;
    }
    for (j = 1; j <= (i / 2); j++)
    {
      value = ia_add_flt(lpc_coeffs[j], ia_mul_flt(rc[i - 1], lpc_coeffs[i - j]));
      lpc_coeffs[i - j] = ia_add_flt(lpc_coeffs[i - j], ia_mul_flt(rc[i - 1], lpc_coeffs[j]));
      lpc_coeffs[j] = value;
    }
    lpc_coeffs[i] = rc[i - 1];
  }

  return;
}
/**
 *  ia_core_coder_ltpf_get_zir
 *
 *  \brief Calculate zir coeff
 *
 *  \param [in]    input      input buffer
 *  \param [in]    output      output buff
 *  \param [in/out]  zir        zir coeff
 *  \param [in]    length      length
 *  \param [in]    lpc_coeffs    LPC coeff
 *  \param [in]    lpc_order    order of LPC coeff
 *  \param [in]    gain      gain
 *  \param [in]    gain_idx    gain index
 *  \param [in]    pitch_int    integer part of pitch
 *  \param [in]    pitch_fr    fractional part of pitch
 *  \param [in]    ptr_scratch_buf  Scratch buffer for internal processing
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_ltpf_get_zir(FLOAT32 *input, FLOAT32 *output, FLOAT32 *zir,
                                       WORD32 length, FLOAT32 *lpc_coeffs, WORD32 lpc_order,
                                       FLOAT32 gain, WORD32 gain_idx, WORD32 pitch_int,
                                       WORD32 pitch_fr, FLOAT32 *ptr_scratch_buf)
{
  FLOAT32 *buf;
  FLOAT32 alpha, step;
  FLOAT32 s1, s2;
  WORD32 n, k;

  buf = ptr_scratch_buf;

  for (n = 0; n < lpc_order; n++)
  {
    s1 = 0;
    for (k = 0; k < LTPF_NUM_FILT_COEF1; k++)
    {
      s1 = ia_add_flt(s1, ia_mul_flt(output[n - lpc_order - pitch_int + k - 4],
                                     ia_ltpf_filter_coef1[pitch_fr][k]));
    }

    s2 = 0;
    for (k = 0; k < LTPF_NUM_FILT_COEF2; k++)
    {
      s2 =
          ia_add_flt(s2, ia_mul_flt(input[n - lpc_order - k], ia_ltpf_filter_coef2[gain_idx][k]));
    }

    buf[n] = ia_sub_flt(
        ia_sub_flt(input[n - lpc_order], ia_mul_flt(LTPF_ATT_FAC, ia_mul_flt(gain, s2))),
        ia_sub_flt(output[n - lpc_order], ia_mul_flt(gain, s1)));
  }

  for (n = 0; n < length; n++)
  {
    for (k = 1; k <= lpc_order; k++)
    {
      buf[lpc_order + n] =
          ia_sub_flt(buf[lpc_order + n], ia_mul_flt(lpc_coeffs[k], buf[lpc_order + n - k]));
    }
  }

  ia_core_coder_mem_cpy(&buf[lpc_order], zir, length / 2);

  alpha = 1.f;
  step = 1.f / (FLOAT32)(length / 2);

  for (n = length / 2; n < length; n++)
  {
    zir[n] = ia_mul_flt(buf[lpc_order + n], alpha);
    alpha = ia_sub_flt(alpha, step);
  }

  return;
}

/**
 *  ia_core_coder_ltpf_init
 *
 *  \brief Initialize LTPF structure
 *
 *  \param [in/out]    ptr_ltpf_data      LTPF data structure
 *  \param [in]      samp_rate        Sample rate of processing
 *  \param [in]      block_size_samples    Samples to process
 *  \param [in]      ptr_scratch        Scratch buffer for
 * intermediate
 * processing
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_ltpf_init(ia_ltpf_data_str *ptr_ltpf_data, WORD32 samp_rate,
                             WORD32 block_size_samples, FLOAT32 *ptr_scratch)
{
  ptr_ltpf_data->frame_len = block_size_samples;
  ptr_ltpf_data->transition_len = block_size_samples >> 3;
  ptr_ltpf_data->codec_delay = block_size_samples >> 1;

  ptr_ltpf_data->pitch_min =
      (WORD32)ia_add_flt(ia_mul_flt(34.f, ((FLOAT32)samp_rate / 2.f) / 12800.f), 0.5f) * 2;
  ptr_ltpf_data->pitch_fr1 = 320;
  ptr_ltpf_data->pitch_fr2 = 324 - ptr_ltpf_data->pitch_min;
  ptr_ltpf_data->pitch_max = 54 + 6 * ptr_ltpf_data->pitch_min;
  ptr_ltpf_data->pitch_res = 2;

  ptr_ltpf_data->data_present = 0;
  ptr_ltpf_data->pitch_lag_idx = 0;
  ptr_ltpf_data->gain_idx = 0;

  ptr_ltpf_data->prev_pitch = 0;
  ptr_ltpf_data->prev_pitch_fr = 0;
  ptr_ltpf_data->prev_gain = 0.f;
  ptr_ltpf_data->prev_gain_idx = 0;

  ptr_ltpf_data->ptr_ltpf_scratch = ptr_scratch;

  return;
}

/**
 *   ia_core_coder_usac_ltpf_process
 *
 *  \brief LTPF processing function
 *
 *  \param [in]    ptr_ltpf_data    LTPF data structure
 *  \param [in]    time_sample_vector  Output data pointer
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_usac_ltpf_process(ia_ltpf_data_str *ptr_ltpf_data, FLOAT32 *time_sample_vector)
{
  WORD32 pitch_int, pitch_fr;
  WORD32 gain_idx, mode, pitch_int_p, pitch_fr_p;

  FLOAT32 gain, gain_p;

  FLOAT32 lpc_coeffs[LTPF_NUM_LPC_COEFFS + 1];
  FLOAT32 *buf_in, *buf_out, *sig_in, *sig_out, *zir;
  FLOAT32 *ptr_scratch_buf = ptr_ltpf_data->ptr_ltpf_scratch;
  FLOAT32 *zir_p;

  /******** Init ********/

  /* Input buffer */
  ia_core_coder_memset(
      ptr_scratch_buf,
      ((LTPF_NUM_FILT_COEF2 - 1 + ptr_ltpf_data->frame_len) +
       (ptr_ltpf_data->pitch_max + LTPF_NUM_FILT_COEF1 / 2 + ptr_ltpf_data->frame_len) +
       (ptr_ltpf_data->transition_len) + (LTPF_NUM_LPC_COEFFS + ptr_ltpf_data->transition_len)));

  buf_in = ptr_scratch_buf;
  sig_in = buf_in + LTPF_NUM_FILT_COEF2 - 1;

  ia_core_coder_mem_cpy(ptr_ltpf_data->prev_in_buf, buf_in, LTPF_NUM_FILT_COEF2 - 1);
  ia_core_coder_mem_cpy(time_sample_vector, sig_in, ptr_ltpf_data->frame_len);
  ia_core_coder_mem_cpy(&buf_in[ptr_ltpf_data->frame_len], ptr_ltpf_data->prev_in_buf,
                        LTPF_NUM_FILT_COEF2 - 1);

  /* Output buffer */

  buf_out = sig_in + ptr_ltpf_data->frame_len;
  sig_out = buf_out + ptr_ltpf_data->pitch_max + LTPF_NUM_FILT_COEF1 / 2;

  ia_core_coder_mem_cpy(ptr_ltpf_data->prev_out_buf, buf_out,
                        ptr_ltpf_data->pitch_max + LTPF_NUM_FILT_COEF1 / 2);

  zir = sig_out + ptr_ltpf_data->frame_len;
  ptr_scratch_buf = zir + ptr_ltpf_data->transition_len;

  ia_core_coder_ltpf_dec_params(&pitch_int, &pitch_fr, &gain, ptr_ltpf_data);

  /******** Previous-frame part ********/
  ia_core_coder_ltpf_filter(sig_in, sig_out, ptr_ltpf_data->codec_delay,
                            ptr_ltpf_data->prev_pitch, ptr_ltpf_data->prev_pitch_fr,
                            ptr_ltpf_data->prev_gain, ptr_ltpf_data->prev_gain_idx, 0, NULL);

  /******** Transition part ********/
  if (gain == ptr_ltpf_data->prev_gain && pitch_int == ptr_ltpf_data->prev_pitch &&
      pitch_fr == ptr_ltpf_data->prev_pitch_fr)
  {
    gain_idx = ptr_ltpf_data->gain_idx;
    pitch_int_p = pitch_int;
    pitch_fr_p = pitch_fr;
    gain_p = gain;
    zir_p = NULL;
    mode = 0;
  }
  else if (ptr_ltpf_data->prev_gain == 0.f)
  {
    gain_idx = ptr_ltpf_data->gain_idx;
    pitch_int_p = pitch_int;
    pitch_fr_p = pitch_fr;
    gain_p = gain;
    zir_p = NULL;
    mode = 2;
  }
  else if (gain == 0.f)
  {
    gain_idx = ptr_ltpf_data->prev_gain_idx;
    pitch_int_p = ptr_ltpf_data->prev_pitch;
    pitch_fr_p = ptr_ltpf_data->prev_pitch_fr;
    gain_p = ptr_ltpf_data->prev_gain;
    zir_p = NULL;
    mode = 3;
  }
  else
  {
    ia_core_coder_ltpf_get_lpc(sig_out + ptr_ltpf_data->codec_delay, ptr_ltpf_data->frame_len / 4,
                               lpc_coeffs);

    ia_core_coder_ltpf_get_zir(
        sig_in + ptr_ltpf_data->codec_delay, sig_out + ptr_ltpf_data->codec_delay, zir,
        ptr_ltpf_data->transition_len, lpc_coeffs, LTPF_NUM_LPC_COEFFS, gain,
        ptr_ltpf_data->gain_idx, pitch_int, pitch_fr, ptr_scratch_buf);

    gain_idx = ptr_ltpf_data->gain_idx;
    pitch_int_p = pitch_int;
    pitch_fr_p = pitch_fr;
    gain_p = gain;
    zir_p = zir;
    mode = 1;
  }

  ia_core_coder_ltpf_filter(sig_in + ptr_ltpf_data->codec_delay,
                            sig_out + ptr_ltpf_data->codec_delay, ptr_ltpf_data->transition_len,
                            pitch_int_p, pitch_fr_p, gain_p, gain_idx, mode, zir_p);

  /******** Current-frame part ********/
  ia_core_coder_ltpf_filter(sig_in + ptr_ltpf_data->codec_delay + ptr_ltpf_data->transition_len,
                            sig_out + ptr_ltpf_data->codec_delay + ptr_ltpf_data->transition_len,
                            ptr_ltpf_data->frame_len - ptr_ltpf_data->codec_delay -
                                ptr_ltpf_data->transition_len,
                            pitch_int, pitch_fr, gain, ptr_ltpf_data->gain_idx, 0, NULL);

  /******** Finalize ********/

  /* copy to output */
  ia_core_coder_mem_cpy(sig_out, time_sample_vector, ptr_ltpf_data->frame_len);

  /* Update */
  ptr_ltpf_data->prev_pitch = pitch_int;
  ptr_ltpf_data->prev_pitch_fr = pitch_fr;
  ptr_ltpf_data->prev_gain = gain;
  ptr_ltpf_data->prev_gain_idx = ptr_ltpf_data->gain_idx;

  ia_core_coder_mem_cpy(&buf_out[ptr_ltpf_data->frame_len], ptr_ltpf_data->prev_out_buf,
                        (ptr_ltpf_data->pitch_max + LTPF_NUM_FILT_COEF1 / 2));
}
/** @} */ /* End of CoreDecProc */