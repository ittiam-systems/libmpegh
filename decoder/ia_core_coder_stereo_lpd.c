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
#include <impeghd_fft_ifft.h>
#include <impeghd_error_codes.h>
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_avq_rom.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_rom.h"
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

#define DELTA ((FLOAT32)PI / 4.0f)

#include <stdio.h>
#include "impeghd_intrinsics_flt.h"

/**
 *  ia_core_coder_update_synth_out
 *
 *  \brief Update synthesis output buffer
 *
 *  \param [out]	  synth_out		Output synthesis buffer
 *  \param [in]			synth		    Input buffer
 *  \param [in]			noise		    Noise buffer
 *  \param [in]			lp_filt		  LP filter coefficients
 *  \param [in]			buf_size		Size of buffer
 *  \param [in]			filt_type   Filter type
 *
 *
 *
 */
static VOID ia_core_coder_update_synth_out(FLOAT32 *synth_out, FLOAT32 *synth, FLOAT32 *noise,
                                           const FLOAT32 *lp_filt, WORD32 buf_size,
                                           WORD32 filt_type)
{
  WORD32 i, j;
  FLOAT32 tmp;
  if (filt_type == 0)
  {
    for (i = 0; i < buf_size; i++)
    {
      tmp = ia_mul_flt(lp_filt[0], noise[i]);
      for (j = 1; j <= FILTER_DELAY; j++)
      {
        tmp = ia_mac_flt(tmp, lp_filt[j], ia_add_flt(noise[i - j], noise[i + j]));
      }
      synth_out[i] = ia_sub_flt(synth[i], tmp);
    }
  }
  else
  {
    for (i = 0; i < buf_size; i++)
    {
      tmp = ia_mul_flt(lp_filt[0], noise[i]);
      for (j = 1; j <= 2 * FILTER_DELAY + 1; j++)
      {
        tmp = ia_mac_flt(tmp, lp_filt[j], ia_add_flt(noise[i - j], noise[i + j]));
      }
      synth_out[i] = ia_sub_flt(synth[i], tmp);
    }
  }
}

/**
 *  ia_core_coder_stereo_lpd_bass_pf
 *
 *  \brief Stereo LPD bass post filter
 *
 *  \param [in]			synth		Input buffer
 *  \param [in]			fac_fb		full band factor
 *  \param [in]			t_sf		bpf pitch
 *  \param [in]			gain_t_sf	bpf gain
 *  \param [out]		synth_out	Output synthesis buffer
 *  \param [in]			len_frame	framelength
 *  \param [in]			len_subfrm	subframe length
 *  \param [in]			l_next		length of next subframe
 *  \param [in]			mem_bpf		prev noise buff
 *
 *
 *
 */

static VOID ia_core_coder_stereo_lpd_bass_pf(FLOAT32 *synth, WORD32 fac_fb, WORD32 *t_sf,
                                             FLOAT32 *gain_t_sf, FLOAT32 *synth_out,
                                             WORD32 len_frame, WORD32 len_subfrm, WORD32 l_next,
                                             FLOAT32 *mem_bpf)
{
  WORD32 delay = fac_fb * FILTER_DELAY + 1 + LTPF_MAX_DELAY;
  WORD32 i, t, t2, lg, l = 0;
  WORD32 sf = 0;
  WORD32 i_sub_frame = delay;
  FLOAT32 tmp, ener = 0.0f, corr = 0.0f, gain;
  FLOAT32 noise_buf[(2 * FILTER_DELAY + 1) * 2 + LTPF_MAX_DELAY + LEN_SUBFR * 2] = {0.0f};
  FLOAT32 *noise = noise_buf + fac_fb * FILTER_DELAY + 1;
  FLOAT32 *noise_in = noise_buf + fac_fb * FILTER_DELAY + 1 + delay;
  FLOAT32 *x, *y;

  while (i_sub_frame < len_frame + delay)
  {
    l = len_subfrm - (i_sub_frame % len_subfrm);

    if (i_sub_frame + l > len_frame + delay)
    {
      l = len_frame + delay - i_sub_frame;
    }

    t = fac_fb * t_sf[sf];
    gain = gain_t_sf[sf];

    if (ia_lt_flt(0, gain))
    {
      for (i = 0; i < l; i++)
      {
        corr = ia_add_flt(corr, ia_mul_flt(synth[i_sub_frame + i], synth[i_sub_frame - t + i]));
        ener =
            ia_add_flt(ener, ia_mul_flt(synth[i_sub_frame - t + i], synth[i_sub_frame - t + i]));
      }
      gain = corr / ener;
    }

    gain = ia_min_flt(gain, 1.0f);
    gain = ia_max_flt(gain, 0.0f);

    t2 = t >> 1;
    x = &synth[i_sub_frame - L_EXTRA];
    y = &synth[i_sub_frame - t2 - L_EXTRA];

    ener = corr = tmp = 0.01f;

    for (i = 0; i < l + L_EXTRA; i++)
    {
      ener = ia_add_flt(ener, ia_mul_flt(x[i], x[i]));
      corr = ia_add_flt(corr, ia_mul_flt(x[i], y[i]));
      tmp = ia_add_flt(tmp, ia_mul_flt(y[i], y[i]));
    }

    tmp = corr / (FLOAT32)ia_sqrt_flt(ia_mul_flt(tmp, ener));

    if (ia_lt_flt(0.95f, tmp))
    {
      t = t2;
    }

    lg = len_frame + l_next - t - i_sub_frame;

    lg = ia_max_int(lg, 0);
    lg = ia_min_int(lg, 1);

    if (ia_lt_flt(0, gain))
    {
      if (lg > 0)
      {
        tmp = ener = 0.01f;
        for (i = 0; i < lg; i++)
        {
          tmp = ia_add_flt(tmp, ia_mul_flt(synth[i + i_sub_frame], synth[i + i_sub_frame]));
          ener = ia_add_flt(ener,
                            ia_mul_flt(synth[i + i_sub_frame + t], synth[i + i_sub_frame + t]));
        }
        tmp = (FLOAT32)ia_sqrt_flt(tmp / ener);
        gain = ia_min_flt(tmp, gain);
      }

      tmp = ia_mul_flt(gain, 0.5f);
      for (i = 0; i < lg; i++)
      {
        noise_in[i] = ia_mul_flt(
            tmp, ia_msu_flt(ia_msu_flt(synth[i + i_sub_frame], 0.5f, synth[i + i_sub_frame - t]),
                            0.5f, synth[i + i_sub_frame + t]));
      }
      for (i = lg; i < l; i++)
      {
        noise_in[i] =
            ia_mul_flt(tmp, ia_sub_flt(synth[i + i_sub_frame], synth[i + i_sub_frame - t]));
      }
    }
    else
    {
      ia_core_coder_memset(noise_in, l);
    }

    ia_core_coder_mem_cpy(mem_bpf, noise_buf, fac_fb * FILTER_DELAY + 1 + delay);

    ia_core_coder_mem_cpy(noise_buf + l, mem_bpf, fac_fb * FILTER_DELAY + 1 + delay);

    if (fac_fb == 2)
    {
      ia_core_coder_update_synth_out(&synth_out[i_sub_frame - delay], &synth[i_sub_frame - delay],
                                     noise, ia_core_coder_fir_lp2_filt, l, 1);
    }
    else
    {
      ia_core_coder_update_synth_out(&synth_out[i_sub_frame - delay], &synth[i_sub_frame - delay],
                                     noise, ia_core_coder_fir_lp_filt, l, 0);
    }
    i_sub_frame += l;
    sf++;
  }

  if (fac_fb == 2)
  {
    ia_core_coder_update_synth_out(&synth_out[i_sub_frame - delay], &synth[i_sub_frame - delay],
                                   &noise[l], ia_core_coder_fir_lp_filt, LTPF_MAX_DELAY, 0);
  }
  else
  {
    ia_core_coder_update_synth_out(&synth_out[i_sub_frame - delay], &synth[i_sub_frame - delay],
                                   &noise[l], ia_core_coder_fir_lp2_filt, LTPF_MAX_DELAY, 1);
  }
}
/**
 *  ia_core_coder_slpd_band_config
 *
 *  \brief Calculate Stereo LPD band limits
 *
 *  \param [in,out]	band_limits	Stereo band info bandlimit
 *  \param [in]			res_mode	  Stereo LPD ERB mode
 *  \param [in]			N			      DFT size
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_slpd_band_config(WORD32 *band_limits, WORD32 res_mode, WORD32 N)
{
  WORD32 num_bands = 0;

  band_limits[0] = 1;
  if (0 == res_mode)
  {
    while (band_limits[num_bands++] < (N / 2))
    {
      band_limits[num_bands] = (WORD32)ia_core_coder_band_lim_erb2[num_bands];
    }
  }
  else
  {
    while (band_limits[num_bands++] < (N / 2))
    {
      band_limits[num_bands] = (WORD32)ia_core_coder_band_lim_erb4[num_bands];
    }
  }

  num_bands--;
  band_limits[num_bands] = N / 2;

  return num_bands;
}

/**
 *  ia_core_coder_slpd_code_book_indices
 *
 *  \brief Calculate Stereo LPD code book indices
 *
 *  \param [in,out]		res_sig		residual signal
 *  \param [in]			it_bit_buf	Bit buffer stream
 *
 *
 *
 */
static VOID ia_core_coder_slpd_code_book_indices(FLOAT32 *res_sig, ia_bit_buf_struct *it_bit_buf)
{
  WORD32 i = 0;
  WORD32 n = 0;
  WORD32 qn = 0;
  WORD32 nk = 0;
  WORD32 kv[8] = {0};
  WORD32 prm[8] = {0};
  long I = 0;

  /* Unary code for codebook numbers */
  while (1 == ia_core_coder_read_bits_buf(it_bit_buf, 1))
  {
    qn += 1;
  }

  if (qn != 0)
  {
    qn += 1;
  }

  n = qn;
  if (qn > 4)
  {
    nk = (qn - 3) >> 1;
    n = qn - nk * 2;
  }

  I = ia_core_coder_read_bits_buf(it_bit_buf, 4 * n);

  for (i = 0; i < 8; i++)
  {
    kv[i] = ia_core_coder_read_bits_buf(it_bit_buf, nk);
  }

  ia_core_coder_rotated_gosset_mtx_dec(qn, I, kv, prm);

  for (i = 0; i < 8; i++)
  {
    res_sig[i] = (FLOAT32)prm[i];
  }
}
/**
 *  ia_core_coder_slpd_init
 *
 *  \brief Initialize Stereo LPD data
 *
 *  \param [in]		slpd_dec_data	Stereo LPD data
 *  \param [in]		fs				Sampling frequency
 *  \param [in]		full_band_lpd	Flag to check full band LPD
 *  \param [in]		ccfl			Frame length
 *
 *
 *
 */
VOID ia_core_coder_slpd_init(ia_usac_slpd_dec_data_handle slpd_dec_data, WORD32 fs,
                             WORD32 full_band_lpd, WORD32 ccfl)
{
  slpd_dec_data->lpd_stereo_config.fs = fs;
  slpd_dec_data->lpd_stereo_config.ccfl = ccfl;
  ia_usac_slpd_bitstream_handle slpd_bs_handle = &slpd_dec_data->lpd_stereo_bitstream;
  ia_usac_slpd_band_info_handle band_info = &slpd_dec_data->lpd_stereo_bandinfo;
  if (1 == full_band_lpd)
  {
    slpd_dec_data->fac_fb = 2;
    slpd_dec_data->win = ia_core_coder_sin_window160;
    slpd_dec_data->lpd_stereo_config.dft_size = 672;
    slpd_dec_data->lpd_stereo_config.frame_size = 512;
    slpd_dec_data->lpd_stereo_config.overlap_size = 160;
  }
  else
  {
    slpd_dec_data->fac_fb = 1;
    slpd_dec_data->win = ia_core_coder_sin_window80;
    slpd_dec_data->lpd_stereo_config.dft_size = 336;
    slpd_dec_data->lpd_stereo_config.frame_size = 256;
    slpd_dec_data->lpd_stereo_config.overlap_size = 80;
  }

  band_info->num_bands =
      ia_core_coder_slpd_band_config(band_info->band_limits, slpd_bs_handle->res_mode,
                                     slpd_dec_data->lpd_stereo_config.dft_size);
  band_info->cod_band_max =
      ia_core_coder_max_band[slpd_bs_handle->res_mode][slpd_bs_handle->cod_mode];
  band_info->ipd_band_max =
      ia_core_coder_max_band[slpd_bs_handle->res_mode][slpd_bs_handle->ipd_mode];
  band_info->num_dft_lines = (band_info->band_limits[band_info->cod_band_max] - 1) << 1;

  switch (slpd_dec_data->lpd_stereo_config.dft_size)
  {
  case 672:
    slpd_dec_data->p_slpd_sin_table = ia_core_coder_slpd_sintable_336;
    break;
  case 336:
    slpd_dec_data->p_slpd_sin_table = ia_core_coder_slpd_sintable_168;
    break;
  }
}

/**
 *  ia_core_coder_slpd_set_past
 *
 *  \brief Update previous left and right stereo LPD data buffers
 *
 *  \param [in,out]		slpd_dec_data	Stereo LPD data
 *  \param [in]			olap_data_l		Left Overlap data buffer
 *  \param [in]			olap_data_r		Right Overlap data buffer
 *  \param [in]			length			length of the buffer to be updated
 *
 *
 *
 */
VOID ia_core_coder_slpd_set_past(ia_usac_slpd_dec_data_handle slpd_dec_data, FLOAT32 *olap_data_l,
                                 FLOAT32 *olap_data_r, WORD32 length)
{
  FLOAT32 *p_prev_left = slpd_dec_data->prev_left + 2 * MAX_PITCH;
  FLOAT32 *p_prev_right = slpd_dec_data->prev_right + 2 * MAX_PITCH;

  ia_core_coder_memset(slpd_dec_data->prev_left, MAX_PITCH << 1);
  ia_core_coder_memset(slpd_dec_data->prev_right, MAX_PITCH << 1);

  ia_core_coder_mem_cpy(olap_data_l + 1, p_prev_left, length);
  ia_core_coder_mem_cpy(olap_data_r, p_prev_right, length);

  ia_core_coder_memset(p_prev_left + length,
                       (slpd_dec_data->lpd_stereo_config.ccfl / 2 + FAC_LENGTH - length));
  ia_core_coder_memset(p_prev_right + length,
                       (slpd_dec_data->lpd_stereo_config.ccfl / 2 + FAC_LENGTH - length));
}

/**
 *  ia_core_coder_slpd_data_dequant
 *
 *  \brief SLPD data de-quantization
 *
 *  \param [in,out]		slpd_dec_data	Stereo LPD structure
 *
 *
 *
 */
static VOID ia_core_coder_slpd_data_dequant(ia_usac_slpd_dec_data_handle slpd_dec_data)
{
  WORD32 i, band;
  ia_usac_slpd_bitstream_handle lpd_stereo_bitstream = &slpd_dec_data->lpd_stereo_bitstream;
  ia_usac_slpd_param_handle lpd_stereo_parameter = &slpd_dec_data->lpd_stereo_parameter;
  ia_usac_slpd_band_info_handle lpd_stereo_bandinfo = &slpd_dec_data->lpd_stereo_bandinfo;
  ia_usac_slpd_config_handle lpd_stereo_config = &slpd_dec_data->lpd_stereo_config;
  WORD32 num_div = lpd_stereo_config->ccfl / lpd_stereo_config->frame_size;

  /* De-quantization */
  memset(lpd_stereo_parameter->ild, 0, sizeof(lpd_stereo_parameter->ild));
  memset(lpd_stereo_parameter->ipd, 0, sizeof(lpd_stereo_parameter->ipd));
  memset(lpd_stereo_parameter->pred_gain, 0, sizeof(lpd_stereo_parameter->pred_gain));
  memset(lpd_stereo_parameter->cod_gain, 0, sizeof(lpd_stereo_parameter->cod_gain));

  for (i = num_div - 1; i >= 0; i--)
  {
    lpd_stereo_parameter->cod_gain[i] = 1.f;

    if ((lpd_stereo_bitstream->q_mode == 0) || (i % 2 == 1))
    {
      for (band = 0; band < lpd_stereo_bandinfo->num_bands; band++)
      {
        lpd_stereo_parameter->ild[i][band] =
            (FLOAT32)ia_core_coder_ild_qtable[lpd_stereo_bitstream->ild_idx[i][band]];
      }

      for (band = 0; band < lpd_stereo_bandinfo->ipd_band_max; band++)
      {
        lpd_stereo_parameter->ipd[i][band] = ia_sub_flt(
            ia_mul_flt((FLOAT32)lpd_stereo_bitstream->ipd_idx[i][band], DELTA), (FLOAT32)PI);
      }

      if (lpd_stereo_bitstream->pred_mode)
      {
        for (band = 0; band < lpd_stereo_bandinfo->num_bands; band++)
        {
          lpd_stereo_parameter->pred_gain[i][band] =
              ia_core_coder_res_pred_gain_qtable[lpd_stereo_bitstream->pred_gain_idx[i][band]];
        }
      }
    }
    else
    {
      ia_core_coder_mem_cpy(&lpd_stereo_parameter->ild[i + 1][0],
                            &lpd_stereo_parameter->ild[i][0], STEREO_LPD_BAND_MAX);

      ia_core_coder_mem_cpy(&lpd_stereo_parameter->ipd[i + 1][0],
                            &lpd_stereo_parameter->ipd[i][0], STEREO_LPD_BAND_MAX);

      ia_core_coder_mem_cpy(&lpd_stereo_parameter->pred_gain[i + 1][0],
                            &lpd_stereo_parameter->pred_gain[i][0], STEREO_LPD_BAND_MAX);
    }

    if (lpd_stereo_bitstream->cod_mode > 0)
    {
      lpd_stereo_parameter->cod_gain[i] =
          ia_mul_flt((FLOAT32)pow(10.0f,
                                  ((FLOAT32)lpd_stereo_bitstream->cod_gain_idx[i]) /
                                      ia_mul_flt(20.0f, (127.0f / 90.0f))),
                     (FLOAT32)lpd_stereo_config->frame_size);
    }
  }
}

/**
 *  ia_core_coder_slpd_data_read
 *
 *  \brief Read SLPD data from bit stream
 *
 *  \param [in,out]		slpd_dec_data	Stereo LPD structure
 *  \param [in]		it_bit_buff		Bit buffer stream
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_slpd_data_read(ia_usac_slpd_dec_data_handle slpd_dec_data,
                                          ia_bit_buf_struct *it_bit_buff)
{
  WORD32 i, j, band;
  ia_usac_slpd_config_handle lpd_stereo_config = &slpd_dec_data->lpd_stereo_config;
  ia_usac_slpd_bitstream_handle lpd_stereo_bitstream = &slpd_dec_data->lpd_stereo_bitstream;
  ia_usac_slpd_param_handle lpd_stereo_parameter = &slpd_dec_data->lpd_stereo_parameter;
  ia_usac_slpd_band_info_handle lpd_stereo_bandinfo = &slpd_dec_data->lpd_stereo_bandinfo;
  WORD32 num_div = lpd_stereo_config->ccfl / lpd_stereo_config->frame_size;

  /* read lpd stream data */
  memset(lpd_stereo_bitstream, 0, sizeof(ia_usac_slpd_bitstream_struct));

  lpd_stereo_bitstream->res_mode = (WORD32)ia_core_coder_read_bits_buf(it_bit_buff, 1);
  lpd_stereo_bitstream->q_mode = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  lpd_stereo_bitstream->ipd_mode = ia_core_coder_read_bits_buf(it_bit_buff, 2);
  lpd_stereo_bitstream->pred_mode = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  lpd_stereo_bitstream->cod_mode = ia_core_coder_read_bits_buf(it_bit_buff, 2);

  lpd_stereo_bandinfo->num_bands =
      ia_core_coder_slpd_band_config(lpd_stereo_bandinfo->band_limits,
                                     lpd_stereo_bitstream->res_mode, lpd_stereo_config->dft_size);
  lpd_stereo_bandinfo->cod_band_max =
      ia_core_coder_max_band[lpd_stereo_bitstream->res_mode][lpd_stereo_bitstream->cod_mode];
  lpd_stereo_bandinfo->ipd_band_max =
      ia_core_coder_max_band[lpd_stereo_bitstream->res_mode][lpd_stereo_bitstream->ipd_mode];
  lpd_stereo_bandinfo->num_dft_lines =
      2 * (lpd_stereo_bandinfo->band_limits[lpd_stereo_bandinfo->cod_band_max] - 1);

  for (i = num_div - 1; i >= 0; i--)
  {
    if ((lpd_stereo_bitstream->q_mode == 0) || (i % 2 == 1))
    {
      for (band = 0; band < lpd_stereo_bandinfo->num_bands; band++)
      {
        lpd_stereo_bitstream->ild_idx[i][band] = ia_core_coder_read_bits_buf(it_bit_buff, 5);
        if (lpd_stereo_bitstream->ild_idx[i][band] > STEREO_LPD_MAX_ILD_IDX)
        {
          lpd_stereo_bitstream->ild_idx[i][band] = 0;
          return IA_MPEGH_DEC_INIT_FATAL_UNSUPPORTED_ILD_IDX;
        }
      }

      for (band = 0; band < lpd_stereo_bandinfo->ipd_band_max; band++)
      {
        lpd_stereo_bitstream->ipd_idx[i][band] = ia_core_coder_read_bits_buf(it_bit_buff, 3);
      }

      if (1 == lpd_stereo_bitstream->pred_mode)
      {
        for (band = lpd_stereo_bandinfo->cod_band_max; band < lpd_stereo_bandinfo->num_bands;
             band++)
        {
          lpd_stereo_bitstream->pred_gain_idx[i][band] =
              ia_core_coder_read_bits_buf(it_bit_buff, 3);
        }
      }
    }

    if (lpd_stereo_bitstream->cod_mode > 0)
    {
      lpd_stereo_bitstream->cod_gain_idx[i] = ia_core_coder_read_bits_buf(it_bit_buff, 7);

      for (j = 0; j < (lpd_stereo_bandinfo->num_dft_lines >> 3); j++)
      {
        ia_core_coder_slpd_code_book_indices(&lpd_stereo_parameter->res_sig[i][2 + 8 * j],
                                             it_bit_buff);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_slpd_data
 *
 *  \brief Decode SLPD data by reading bit stream
 *
 *  \param [in,out]		slpd_dec_data	Stereo LPD structure
 *  \param [in]		it_bit_buff		Bit buffer stream
 *  \param [in]		td_start_flag	Flag to check if prev frame is TD
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_slpd_data(ia_usac_slpd_dec_data_handle slpd_dec_data,
                                     ia_bit_buf_struct *it_bit_buff, WORD32 td_start_flag)
{
  IA_ERRORCODE error_code = IA_MPEGH_DEC_NO_ERROR;
  if (td_start_flag)
  {
    ia_core_coder_memset(slpd_dec_data->dft_prev_dmx, STEREO_LPD_DFT_SIZE);
  }

  /* read lpd stream data */
  error_code = ia_core_coder_slpd_data_read(slpd_dec_data, it_bit_buff);

  /* De-quantization */
  ia_core_coder_slpd_data_dequant(slpd_dec_data);

  return error_code;
}

/**
 *  ia_core_coder_update_dft
 *
 *  \brief Update dft buffers of 'band'
 *
 *  \param [in]		slpd_dec_data	stereo lpd structure
 *  \param [in,out]	pscr			scratch structure containing dft buffers
 *  \param [in]		band			band number
 *  \param [in]		alpha			alpha
 *  \param [in]		beta			beta
 *
 *
 *
 */
static VOID ia_core_coder_update_dft(ia_usac_slpd_dec_data_handle slpd_dec_data,
                                     ia_core_coder_slpd_apply_scr_t *pscr, WORD32 band,
                                     FLOAT32 alpha, FLOAT32 beta)
{
  WORD32 i;
  FLOAT32 energy_l, energy_r, sum_r, sum_i, sum, c;
  FLOAT32 cos_beta, sin_beta, bet_min_alp, cos_bet_min_alp, sin_bet_min_alp;
  for (i = slpd_dec_data->lpd_stereo_bandinfo.band_limits[band];
       i < slpd_dec_data->lpd_stereo_bandinfo.band_limits[band + 1]; i++)
  {
    energy_l = ia_add_flt(ia_mul_flt(pscr->dft_left[2 * i], pscr->dft_left[2 * i]),
                          ia_mul_flt(pscr->dft_left[2 * i + 1], pscr->dft_left[2 * i + 1]));
    energy_r =
        ia_add_flt(ia_add_flt(ia_mul_flt(pscr->dft_right[2 * i], pscr->dft_right[2 * i]),
                              ia_mul_flt(pscr->dft_right[2 * i + 1], pscr->dft_right[2 * i + 1])),
                   STEREO_LPD_FLT_MIN);
    sum_r = pscr->dft_dmx[2 * i];
    sum_i = pscr->dft_dmx[2 * i + 1];
    sum = ia_add_flt(ia_add_flt(ia_mul_flt(sum_r, sum_r), ia_mul_flt(sum_i, sum_i)),
                     STEREO_LPD_FLT_MIN);
    c = (FLOAT32)ia_sqrt_flt(2.0 * sum / ia_add_flt(energy_l, energy_r));
    c = ia_max_int(ia_min_int(c, STERO_LPD_DMX_LIMIT), 1.0f / STERO_LPD_DMX_LIMIT);
    pscr->dft_left[2 * i] = ia_mul_flt(pscr->dft_left[2 * i], c);
    pscr->dft_left[2 * i + 1] = ia_mul_flt(pscr->dft_left[2 * i + 1], c);
    pscr->dft_right[2 * i] = ia_mul_flt(pscr->dft_right[2 * i], c);
    pscr->dft_right[2 * i + 1] = ia_mul_flt(pscr->dft_right[2 * i + 1], c);

    if (band < slpd_dec_data->lpd_stereo_bandinfo.ipd_band_max)
    {
      cos_beta = (FLOAT32)cos(beta);
      sin_beta = (FLOAT32)sin(beta);
      sum_r = ia_msu_flt(ia_mul_flt(pscr->dft_left[2 * i], cos_beta), pscr->dft_left[2 * i + 1],
                         sin_beta);
      pscr->dft_left[2 * i + 1] = ia_mac_flt(ia_mul_flt(pscr->dft_left[2 * i], sin_beta),
                                             pscr->dft_left[2 * i + 1], cos_beta);
      pscr->dft_left[2 * i] = sum_r;
      bet_min_alp = ia_add_flt(-alpha, beta);
      cos_bet_min_alp = (FLOAT32)cos(bet_min_alp);
      sin_bet_min_alp = (FLOAT32)sin(bet_min_alp);
      sum_r = ia_msu_flt(ia_mul_flt(pscr->dft_right[2 * i], cos_bet_min_alp),
                         pscr->dft_right[2 * i + 1], sin_bet_min_alp);
      pscr->dft_right[2 * i + 1] = ia_mac_flt(ia_mul_flt(pscr->dft_right[2 * i], sin_bet_min_alp),
                                              pscr->dft_right[2 * i + 1], cos_bet_min_alp);
      pscr->dft_right[2 * i] = sum_r;
    }
  }
}

/**
 *  ia_core_coder_update_dft_gain
 *
 *  \brief Update dft buffers of 'band' with gain
 *
 *  \param [in]		slpd_dec_data	stereo lpd structure
 *  \param [in,out]	pscr			scratch structure containing dft buffers
 *  \param [in]		sub_frm			Sub-frame index
 *  \param [in]		band			band number
 *  \param [in]		gain_1			Gain 1
 *  \param [in]		gain_2			Gain 2
 *
 *
 *
 */
static VOID ia_core_coder_update_dft_gain(ia_usac_slpd_dec_data_handle slpd_dec_data,
                                          ia_core_coder_slpd_apply_scr_t *pscr, WORD32 sub_frm,
                                          WORD32 band, FLOAT32 gain_1, FLOAT32 gain_2)
{
  WORD32 i;
  FLOAT32 *p_dft_res = &slpd_dec_data->lpd_stereo_parameter.res_sig[sub_frm][0];
  FLOAT32 cod_gain;

  for (i = slpd_dec_data->lpd_stereo_bandinfo.band_limits[band];
       i < slpd_dec_data->lpd_stereo_bandinfo.band_limits[band + 1]; i++)
  {
    pscr->dft_left[2 * i] =
        ia_add_flt(ia_add_flt(pscr->dft_dmx[2 * i], ia_mul_flt(gain_1, pscr->dft_dmx[2 * i])),
                   ia_mul_flt(gain_2, slpd_dec_data->dft_prev_dmx[2 * i]));
    pscr->dft_left[2 * i + 1] = ia_add_flt(
        ia_add_flt(pscr->dft_dmx[2 * i + 1], ia_mul_flt(gain_1, pscr->dft_dmx[2 * i + 1])),
        ia_mul_flt(gain_2, slpd_dec_data->dft_prev_dmx[2 * i + 1]));
    pscr->dft_right[2 * i] =
        ia_msu_flt(ia_msu_flt(pscr->dft_dmx[2 * i], gain_1, pscr->dft_dmx[2 * i]), gain_2,
                   slpd_dec_data->dft_prev_dmx[2 * i]);
    pscr->dft_right[2 * i + 1] = ia_sub_flt(
        ia_sub_flt(pscr->dft_dmx[2 * i + 1], ia_mul_flt(gain_1, pscr->dft_dmx[2 * i + 1])),
        ia_mul_flt(gain_2, slpd_dec_data->dft_prev_dmx[2 * i + 1]));

    if (band < slpd_dec_data->lpd_stereo_bandinfo.cod_band_max)
    {
      cod_gain = slpd_dec_data->lpd_stereo_parameter.cod_gain[sub_frm];
      pscr->dft_left[2 * i] =
          ia_add_flt(pscr->dft_left[2 * i], ia_mul_flt(cod_gain, p_dft_res[2 * i]));
      pscr->dft_left[2 * i + 1] =
          ia_add_flt(pscr->dft_left[2 * i + 1], ia_mul_flt(cod_gain, p_dft_res[2 * i + 1]));
      pscr->dft_right[2 * i] =
          ia_sub_flt(pscr->dft_right[2 * i], ia_mul_flt(cod_gain, p_dft_res[2 * i]));
      pscr->dft_right[2 * i + 1] =
          ia_sub_flt(pscr->dft_right[2 * i + 1], ia_mul_flt(cod_gain, p_dft_res[2 * i + 1]));
    }
  }
}

/**
 *  ia_core_coder_slpd_calc_time_sig
 *
 *  \brief Apply stereo LPD and update stereo buffer
 *
 *  \param [in]		slpd_dec_data	stereo lpd structure
 *  \param [in,out]	pscr			scratch structure containing time buffers
 *  \param [in]		sub_frm			Sub-frame index
 *  \param [in]		td_start_flag	flag to check prev frame is TD
 *  \param [in]		delay			Delay
 *  \param [in]		offset			offset
 *  \param [in]		fft_scratch		Scratch buffer for fft
 *
 *
 *
 */

static VOID ia_core_coder_slpd_calc_time_sig(ia_usac_slpd_dec_data_handle slpd_dec_data,
                                             ia_core_coder_slpd_apply_scr_t *pscr, WORD32 sub_frm,
                                             WORD32 td_start_flag, WORD32 delay, WORD32 offset,
                                             FLOAT32 *fft_scratch)
{
  WORD32 i;
  ia_core_coder_mem_cpy(pscr->dft_left, pscr->time_buff,
                        slpd_dec_data->lpd_stereo_config.dft_size);

  ia_core_coder_real_fft(pscr->time_buff, slpd_dec_data->p_slpd_sin_table,
                         slpd_dec_data->lpd_stereo_config.dft_size, 1, fft_scratch);

  if (sub_frm == 0 && td_start_flag)
  {
    for (i = 0; i < delay; i++)
    {
      pscr->time_buff_left[offset + i] =
          ia_add_flt(ia_mul_flt(ia_mul_flt(pscr->time_buff_left[offset + i],
                                           slpd_dec_data->win[delay - 1 - i]),
                                slpd_dec_data->win[delay - 1 - i]),
                     ia_mul_flt(pscr->time_buff[i], slpd_dec_data->win[i]));
      pscr->time_buff_left[offset + slpd_dec_data->lpd_stereo_config.dft_size - 1 - i] =
          ia_mul_flt(pscr->time_buff[slpd_dec_data->lpd_stereo_config.dft_size - 1 - i],
                     slpd_dec_data->win[i]);
    }

    ia_core_coder_mem_cpy(&pscr->time_buff[delay], &pscr->time_buff_left[offset + delay],
                          slpd_dec_data->lpd_stereo_config.frame_size - delay);
  }
  else
  {
    for (i = 0; i < delay; i++)
    {
      pscr->time_buff_left[offset + i] =
          ia_add_flt(pscr->time_buff_left[offset + i],
                     ia_mul_flt(pscr->time_buff[i], slpd_dec_data->win[i]));
      pscr->time_buff_left[offset + slpd_dec_data->lpd_stereo_config.dft_size - 1 - i] =
          ia_mul_flt(pscr->time_buff[slpd_dec_data->lpd_stereo_config.dft_size - 1 - i],
                     slpd_dec_data->win[i]);
    }

    ia_core_coder_mem_cpy(&pscr->time_buff[delay], &pscr->time_buff_left[offset + delay],
                          slpd_dec_data->lpd_stereo_config.frame_size - delay);
  }

  ia_core_coder_mem_cpy(pscr->dft_right, pscr->time_buff,
                        slpd_dec_data->lpd_stereo_config.dft_size);

  ia_core_coder_real_fft(pscr->time_buff, slpd_dec_data->p_slpd_sin_table,
                         slpd_dec_data->lpd_stereo_config.dft_size, 1, fft_scratch);

  if (sub_frm == 0 && td_start_flag)
  {
    for (i = 0; i < delay; i++)
    {
      pscr->time_buff_right[offset + i] =
          ia_add_flt(ia_mul_flt(ia_mul_flt(pscr->time_buff_right[offset + i],
                                           slpd_dec_data->win[delay - 1 - i]),
                                slpd_dec_data->win[delay - 1 - i]),
                     ia_mul_flt(pscr->time_buff[i], slpd_dec_data->win[i]));
      pscr->time_buff_right[offset + slpd_dec_data->lpd_stereo_config.dft_size - 1 - i] =
          ia_mul_flt(pscr->time_buff[slpd_dec_data->lpd_stereo_config.dft_size - 1 - i],
                     slpd_dec_data->win[i]);
    }

    ia_core_coder_mem_cpy(&pscr->time_buff[delay], &pscr->time_buff_right[offset + delay],
                          slpd_dec_data->lpd_stereo_config.frame_size - delay);
  }
  else
  {
    for (i = 0; i < delay; i++)
    {
      pscr->time_buff_right[offset + i] =
          ia_add_flt(pscr->time_buff_right[offset + i],
                     ia_mul_flt(pscr->time_buff[i], slpd_dec_data->win[i]));
      pscr->time_buff_right[offset + slpd_dec_data->lpd_stereo_config.dft_size - 1 - i] =
          ia_mul_flt(pscr->time_buff[slpd_dec_data->lpd_stereo_config.dft_size - 1 - i],
                     slpd_dec_data->win[i]);
    }

    ia_core_coder_mem_cpy(&pscr->time_buff[delay], &pscr->time_buff_right[offset + delay],
                          slpd_dec_data->lpd_stereo_config.frame_size - delay);
  }
}

/**
 *  ia_core_coder_slpd_apply
 *
 *  \brief Apply stereo LPD and update stereo buffer
 *
 *  \param [in]		slpd_dec_data	stereo lpd structure
 *  \param [in,out]	synth_left		synth_stereo buffer for left
 *  \param [in,out]	synth_right		synth_stereo buffer for right
 *  \param [in]		td_start_flag	flag to check prev frame is TD
 *  \param [in]		ptr_scratch		Scratch buffer for processing
 *
 *
 *
 */

VOID ia_core_coder_slpd_apply(ia_usac_slpd_dec_data_handle slpd_dec_data, FLOAT32 *synth_left,
                              FLOAT32 *synth_right, WORD32 td_start_flag, FLOAT32 *ptr_scratch)
{
  WORD32 k, i;
  WORD32 b = 0;
  WORD32 offset = 0;
  WORD32 offset_dmx = 0;
  WORD32 fac_fb = slpd_dec_data->fac_fb;
  WORD32 past = fac_fb * MAX_PITCH + slpd_dec_data->lpd_stereo_config.ccfl / 2 - 256;
  WORD32 delay = slpd_dec_data->lpd_stereo_config.overlap_size;

  ia_core_coder_slpd_apply_scr_t *pscr = (ia_core_coder_slpd_apply_scr_t *)ptr_scratch;

  FLOAT32 alpha = 0.0f;
  FLOAT32 beta = 0.0f;
  FLOAT32 g = 0.0f;
  FLOAT32 c, nrg_l, nrg_r, sum_real, sum_abs;
  FLOAT32 *p_ild, *p_ipd, *p_pred_gain;
  FLOAT32 *fft_scratch;

  fft_scratch = ptr_scratch +
                (sizeof(ia_core_coder_slpd_apply_scr_t) + (sizeof(*ptr_scratch) - 1)) /
                    sizeof(*ptr_scratch);
  ;

  if (td_start_flag)
  {
    ia_core_coder_mem_cpy(slpd_dec_data->prev_left, pscr->time_buff_left,
                          fac_fb * MAX_PITCH + slpd_dec_data->lpd_stereo_config.ccfl / 2 +
                              FAC_LENGTH);

    ia_core_coder_mem_cpy(slpd_dec_data->prev_right, pscr->time_buff_right,
                          fac_fb * MAX_PITCH + slpd_dec_data->lpd_stereo_config.ccfl / 2 +
                              FAC_LENGTH);

    memset(slpd_dec_data->bpf_pitch, 0, BPF_DELAY * sizeof(WORD32));

    ia_core_coder_memset(slpd_dec_data->bpf_gain, BPF_DELAY);

    ia_core_coder_memset(slpd_dec_data->old_noise_pf_left,
                         ((2 * FILTER_DELAY + 1) * 2 + LTPF_MAX_DELAY));

    ia_core_coder_memset(slpd_dec_data->old_noise_pf_right,
                         ((2 * FILTER_DELAY + 1) * 2 + LTPF_MAX_DELAY));
  }
  else
  {
    ia_core_coder_mem_cpy(slpd_dec_data->prev_left, pscr->time_buff_left, past);

    ia_core_coder_mem_cpy(slpd_dec_data->prev_right, pscr->time_buff_right, past);
  }

  for (k = 0;
       k < slpd_dec_data->lpd_stereo_config.ccfl / slpd_dec_data->lpd_stereo_config.frame_size;
       k++)
  {
    offset = past + k * slpd_dec_data->lpd_stereo_config.frame_size - delay;
    offset_dmx = offset - fac_fb * MAX_PITCH;

    ia_core_coder_mem_cpy(&slpd_dec_data->time_buff_dmx[offset_dmx], pscr->time_buff,
                          slpd_dec_data->lpd_stereo_config.dft_size);

    for (i = 0; i < delay; i++)
    {
      pscr->time_buff[i] = ia_mul_flt(pscr->time_buff[i], slpd_dec_data->win[i]);
      pscr->time_buff[slpd_dec_data->lpd_stereo_config.dft_size - 1 - i] =
          ia_mul_flt(pscr->time_buff[slpd_dec_data->lpd_stereo_config.dft_size - 1 - i],
                     slpd_dec_data->win[i]);
    }

    ia_core_coder_mem_cpy(pscr->time_buff, pscr->dft_dmx,
                          slpd_dec_data->lpd_stereo_config.dft_size);
    ia_core_coder_real_fft(pscr->dft_dmx, slpd_dec_data->p_slpd_sin_table,
                           slpd_dec_data->lpd_stereo_config.dft_size, -1, fft_scratch);

    p_ild = &slpd_dec_data->lpd_stereo_parameter.ild[k][0];
    p_ipd = &slpd_dec_data->lpd_stereo_parameter.ipd[k][0];
    p_pred_gain = &slpd_dec_data->lpd_stereo_parameter.pred_gain[k][0];

    pscr->dft_left[0] = pscr->dft_right[0] = pscr->dft_dmx[0];

    for (b = 0; b < slpd_dec_data->lpd_stereo_bandinfo.num_bands; b++)
    {
      /* c is ((FLOAT32)pow(10.0, p_ild[b] / 20.0)) */
      if (p_ild[b] < 0)
      {
        c = ia_pow_10_20_slpd_apply[0][(WORD32)(-1 * p_ild[b])];
      }
      else
      {
        c = ia_pow_10_20_slpd_apply[1][(WORD32)(p_ild[b])];
      }
      g = (ia_sub_flt(c, 1) / ia_add_flt(c, 1));

      ia_core_coder_update_dft_gain(slpd_dec_data, pscr, k, b, g, p_pred_gain[b]);

      if (b < slpd_dec_data->lpd_stereo_bandinfo.ipd_band_max)
      {
        alpha = p_ipd[b];
        beta = (FLOAT32)(atan2(sin(alpha), ia_add_double(cos(alpha), (FLOAT64)c)));
      }

      ia_core_coder_update_dft(slpd_dec_data, pscr, b, alpha, beta);
    }

    pscr->dft_left[1] = ia_mac_flt(pscr->dft_dmx[1], g, pscr->dft_dmx[1]);
    pscr->dft_right[1] = ia_msu_flt(pscr->dft_dmx[1], g, pscr->dft_dmx[1]);
    nrg_l = ia_mul_flt(pscr->dft_left[1], pscr->dft_left[1]);
    nrg_r = ia_mac_flt(STEREO_LPD_FLT_MIN, pscr->dft_right[1], pscr->dft_right[1]);
    sum_real = pscr->dft_dmx[1];
    sum_abs = ia_mac_flt(STEREO_LPD_FLT_MIN, sum_real, sum_real);
    c = (FLOAT32)ia_sqrt_flt(2.0 * sum_abs / ia_add_flt(nrg_l, nrg_r));
    c = ia_max_int(ia_min_int(c, STERO_LPD_DMX_LIMIT), 1.0f / STERO_LPD_DMX_LIMIT);
    pscr->dft_left[1] = ia_mul_flt(pscr->dft_left[1], c);
    pscr->dft_right[1] = ia_mul_flt(pscr->dft_right[1], c);

    ia_core_coder_mem_cpy(pscr->dft_dmx, slpd_dec_data->dft_prev_dmx,
                          slpd_dec_data->lpd_stereo_config.dft_size);

    ia_core_coder_slpd_calc_time_sig(slpd_dec_data, pscr, k, td_start_flag, delay, offset,
                                     fft_scratch);
  }

  ia_core_coder_mem_cpy(pscr->time_buff_left + slpd_dec_data->lpd_stereo_config.ccfl,
                        slpd_dec_data->prev_left, past);

  ia_core_coder_mem_cpy(pscr->time_buff_right + slpd_dec_data->lpd_stereo_config.ccfl,
                        slpd_dec_data->prev_right, past);
  ia_core_coder_stereo_lpd_bass_pf(
      pscr->time_buff_left + fac_fb * MAX_PITCH, fac_fb, slpd_dec_data->bpf_pitch,
      slpd_dec_data->bpf_gain, pscr->out_buff, slpd_dec_data->lpd_stereo_config.ccfl,
      fac_fb * LEN_SUBFR, past - fac_fb * MAX_PITCH - delay, slpd_dec_data->old_noise_pf_left);

  ia_core_coder_mem_cpy(pscr->out_buff, synth_left, slpd_dec_data->lpd_stereo_config.ccfl);

  ia_core_coder_stereo_lpd_bass_pf(
      pscr->time_buff_right + fac_fb * MAX_PITCH, fac_fb, slpd_dec_data->bpf_pitch,
      slpd_dec_data->bpf_gain, pscr->out_buff, slpd_dec_data->lpd_stereo_config.ccfl,
      fac_fb * LEN_SUBFR, past - fac_fb * MAX_PITCH - delay, slpd_dec_data->old_noise_pf_right);

  ia_core_coder_mem_cpy(pscr->out_buff, synth_right, slpd_dec_data->lpd_stereo_config.ccfl);
}
/** @} */ /* End of CoreDecProc */