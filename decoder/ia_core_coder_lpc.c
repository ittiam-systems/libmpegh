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
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_avq_rom.h"
#include "ia_core_coder_bitbuffer.h"
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
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_windows.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_reset_acelp_data
 *
 *  \brief Reset acelp data
 *
 *  \param [in/out]    usac_data      USAC data structure
 *  \param [in/out]    st          LPD decoder handle
 *  \param [in/out]    td_config      TD configuration handle
 *  \param [in]    ptr_overlap_buf    Overlap buffer
 *  \param [in]    was_last_short    Flag to check if last frame was short
 *  \param [in]    chn          Channel being processed
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_reset_acelp_data(ia_usac_data_struct *usac_data,
                                            ia_usac_lpd_decoder_handle st,
                                            ia_usac_td_config_handle td_config,
                                            FLOAT32 *ptr_overlap_buf, WORD32 was_last_short,
                                            WORD32 chn)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, lfac = 0;
  WORD32 fbd_lpd;

  if (td_config == NULL)
  {
    return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_TD_CFG_HANDLE;
  }
  fbd_lpd = !td_config->full_band_lpd;

  if (st == NULL)
    return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_LPD_DEC_HANDLE;

  if (1 != was_last_short)
  {
    st->mode_prev = -1;
    if (td_config)
      lfac = (td_config->len_frame) >> 3;
  }
  else
  {
    st->mode_prev = -2;
    if (td_config)
      lfac = (td_config->len_frame) >> 4;
  }

  st->gain_threshold = 0.0f;

  ia_core_coder_memset(&st->exc_prev_fb[0], (4 * FAC_LENGTH));
  ia_core_coder_memset(&st->exc_prev[0], (1 + (2 * FAC_LENGTH)));
  ia_core_coder_memset(&st->lp_flt_coeff_a_prev[0], 2 * (ORDER + 1));

  if (td_config->full_band_lpd == 1)
  {
    if (ptr_overlap_buf != NULL)
    {
      ia_core_coder_mem_cpy(&ptr_overlap_buf[2 * td_config->len_subfrm - lfac - 1],
                            &st->exc_prev[(td_config->len_subfrm) / 2 - lfac], (2 * lfac + 1));
    }
    ia_core_coder_mem_cpy(&st->overlap_buf_fb[td_config->len_frame_fb / 2 - 2 * lfac],
                          &st->exc_prev_fb[(2 * td_config->len_subfrm) / 2 - 2 * lfac],
                          2 * 2 * lfac);
  }

  ia_core_coder_memset(&st->bpf_prev[0], (2 * FILTER_DELAY + 1 + 2 * LEN_SUBFR));
  ia_core_coder_memset(&st->synth_prev[0], (MAX_PITCH + SYNTH_DELAY_LMAX));
  ia_core_coder_memset(&st->xcitation_prev[0], (MAX_PITCH + INTER_LP_FIL_ORDER + 1));

  for (i = 0; i < NUM_SUBFR_SUPERFRAME_BY2 - 1; i++)
  {
    st->pitch_prev[i] = 64;
  }
  memset(st->gain_prev, 0, (NUM_SUBFR_SUPERFRAME_BY2 - 1) * sizeof(st->gain_prev[0]));

  /* Initialize the ISFs */
  ia_core_coder_mem_cpy(ia_core_coder_lsf_init, st->lsf_prev, ORDER);

  /* Initialize the ISPs */
  for (i = 0; i < ORDER; i++)
  {
    st->lspold[i] =
        (FLOAT32)cos(ia_mul_double_flt(3.141592654, (FLOAT32)(i + 1)) / (FLOAT32)(ORDER + 1));
  }

  st->bpf_active_prev = 0;

  if (ptr_overlap_buf != NULL && fbd_lpd)
  {
    const FLOAT32 *ptr_window_coeff;
    FLOAT32 *ptr_loc_ovbuf;

    switch (lfac)
    {
    case 96:
      ptr_window_coeff = ia_core_coder_sine_window192;
      break;

    case 64:
      ptr_window_coeff = ia_core_coder_sine_window128;
      break;

    case 48:
      ptr_window_coeff = ia_core_coder_sine_window96;
      break;

    default:
      ptr_window_coeff = ia_core_coder_sine_window256;
      break;
    }

    ptr_loc_ovbuf = &ptr_overlap_buf[(usac_data->ccfl) / 2 - lfac];
    for (i = 0; i < 2 * lfac; i++)
    {
      ptr_loc_ovbuf[i] = ia_mul_flt(ptr_loc_ovbuf[i], ptr_window_coeff[2 * lfac - 1 - i]);
    }

    ia_core_coder_memset(&ptr_overlap_buf[(usac_data->ccfl) / 2 + lfac],
                         (usac_data->ccfl) / 2 - lfac);
  }
  st->overlap_buf = ptr_overlap_buf;

  if (ptr_overlap_buf != NULL && fbd_lpd)
  {
    ia_core_coder_mem_cpy(&ptr_overlap_buf[td_config->len_frame / 2 - lfac - 1],
                          &st->exc_prev[(td_config->len_subfrm) / 2 - lfac], 2 * lfac + 1);
  }

  if (td_config != NULL && td_config->full_band_lpd == 1)
  {
    ia_core_coder_memset(st->td_resamp_mem, 2 * L_FILT_MAX);
    st->td_resamp_delay =
        (td_config->fscale < td_config->fscale_full_band) ? L_FILT_MAX : L_FILT_MAX / 2;
    ia_core_coder_memset(st->synth_prev_fb, 2 * (MAX_PITCH + SYNTH_DELAY_LMAX));
    ia_core_coder_tbe_init(&st->tbe_dec_data);
  }

  st->fdp_data_present = 0;
  st->fdp_spacing_index = 0;
  st->max_sfb = 0;
  st->window_shape = 0;
  st->tns_data_present = 0;

  memset(st->fdp_quant_spec_prev, 0, sizeof(st->fdp_quant_spec_prev));
  memset(st->fdp_int, 0, sizeof(st->fdp_int));

  ia_core_coder_memset(usac_data->igf_dec_data[chn].igf_input_spec,
                       sizeof(usac_data->igf_dec_data[chn].igf_input_spec) / sizeof(FLOAT32));

  return err_code;
}

/**
 *  ia_core_coder_init_acelp_data
 *
 *  \brief Initialize acelp data
 *
 *  \param [in]    usac_data  USAC data structure
 *  \param [in]    st      LPD decoder handle
 *  \param [in]    chn      channel being processed
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_init_acelp_data(ia_usac_data_struct *usac_data,
                                           ia_usac_lpd_decoder_handle st, WORD32 chn)
{
  return ia_core_coder_reset_acelp_data(usac_data, st, usac_data->td_config, NULL, 0, chn);
}

/**
 *  ia_core_coder_lsf_2_lsp_conversion
 *
 *  \brief Convert lsf to lsp coeff
 *
 *  \param [in]    lsf    Input LSF coeff
 *  \param [out]  lsp    Output LSP coeff
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_lsf_2_lsp_conversion(FLOAT32 *lsf, FLOAT32 *lsp) //, WORD32 m)
{
  WORD32 i;
  for (i = ORDER - 1; i >= 0; i--)
  {
    lsp[i] = (FLOAT32)cos(ia_mul_double_flt((FLOAT64)PI_BY_6400, lsf[i]));
  }
  return;
}
/**
 *  ia_core_coder_bass_post_filter
 *
 *  \brief Bass post filter applied on input and synthesis output generated
 *
 *  \param [in]      synth_sig    Input to postfilter
 *  \param [in]      synth_fb    FB Input to postfiler
 *  \param [in]      fac_fb      fblpd factor
 *  \param [in]      pitch      pitch perios ofr akk subframe
 *  \param [in]      pitch_gain    pitch gain for all subframes
 *  \param [in/out]    synth_out    Filtered synthesized output signal
 *  \param [in]      len_fr      frame length
 *  \param [in]      len2      subframe length
 *  \param [in]      bpf_prev    Previous bpf state
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_bass_post_filter(FLOAT32 *synth_sig, FLOAT32 *synth_fb, WORD32 fac_fb,
                                            WORD32 *pitch, FLOAT32 *pitch_gain,
                                            FLOAT32 *synth_out, WORD32 len_fr, WORD32 len2,
                                            FLOAT32 *bpf_prev)
{
  WORD32 i, j, sf, num_subfr, pitch_lag, lg;
  WORD32 len_fr_mod_64, fac;
  FLOAT32 x_energy, xy_corr, y_energy, norm_corr, energy, gain, tmp, alpha;
  FLOAT32 noise_buf[2 * FILTER_DELAY + 1 + (2 * 2 * LEN_SUBFR)] = {0.0f};
  FLOAT32 *noise_loc1, *noise_loc2, *x, *y;
  const FLOAT32 *lp_filt;
  FLOAT32 *synth_loc;

  if (2 == fac_fb)
  {
    fac = 2;
    synth_loc = synth_fb;
    lp_filt = ia_core_coder_fir_lp2_filt;
  }
  else
  {
    fac = 1;
    synth_loc = synth_sig;
    lp_filt = ia_core_coder_fir_lp_filt;
  }

  noise_loc1 = noise_buf + fac * FILTER_DELAY + fac - 1;
  noise_loc2 = noise_buf + fac * FILTER_DELAY + fac - 1 + fac * LEN_SUBFR;

  ia_core_coder_mem_cpy(synth_loc - fac * LEN_SUBFR, synth_out, fac * len_fr);

  len_fr_mod_64 = len_fr % 64;
  if (0 != len_fr_mod_64)
  {
    ia_core_coder_memset(synth_out + fac * len_fr, fac * (LEN_SUBFR - len_fr_mod_64));
  }

  sf = 0;
  for (num_subfr = 0; num_subfr < len_fr; num_subfr += LEN_SUBFR, sf++)
  {
    pitch_lag = pitch[sf];
    gain = pitch_gain[sf];
    if (((pitch_lag >> 1) + 96 - num_subfr) > MAX_PITCH)
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PITCH_LAG;

    gain = ia_min_flt(1.0f, gain);
    gain = ia_max_flt(0.0f, gain);

    x = &synth_sig[num_subfr - 96];
    y = &synth_sig[num_subfr - pitch_lag / 2 - 96];

    x_energy = 0.01f;
    xy_corr = 0.01f;
    y_energy = 0.01f;
    for (i = 0; i < LEN_SUBFR + 96; i++)
    {
      x_energy = ia_add_flt(x_energy, ia_mul_flt(x[i], x[i]));
      y_energy = ia_add_flt(y_energy, ia_mul_flt(y[i], y[i]));
      xy_corr = ia_add_flt(xy_corr, ia_mul_flt(x[i], y[i]));
    }

    norm_corr = xy_corr / (FLOAT32)ia_sqrt_flt(ia_mul_flt(x_energy, y_energy));

    if (ia_lt_flt(0.95f, norm_corr))
      pitch_lag >>= 1;

    lg = len_fr + len2 - pitch_lag - num_subfr;
    lg = ia_max_int(0, lg);
    lg = ia_min_int(LEN_SUBFR, lg);

    if (ia_lt_flt(0, gain))
    {
      if (lg > 0)
      {
        tmp = 0.01f;
        energy = 0.01f;
        for (i = 0; i < lg; i++)
        {
          tmp = ia_add_flt(tmp, ia_mul_flt(synth_sig[i + num_subfr], synth_sig[i + num_subfr]));

          energy = ia_add_flt(energy, ia_mul_flt(synth_sig[i + num_subfr + pitch_lag],
                                                 synth_sig[i + num_subfr + pitch_lag]));
        }
        tmp = (FLOAT32)ia_sqrt_flt(tmp / energy);
        gain = ia_min_flt(tmp, gain);
      }

      alpha = ia_mul_flt(0.5f, gain);

      for (i = 0; i < fac * lg; i++)
      {
        noise_loc2[i] = ia_mul_flt(
            alpha,
            (ia_sub_flt(
                ia_sub_flt(synth_loc[i + fac * num_subfr],
                           ia_mul_flt(0.5f, synth_loc[i + fac * num_subfr - fac * pitch_lag])),
                ia_mul_flt(0.5f, synth_loc[i + fac * num_subfr + fac * pitch_lag]))));
      }
      for (i = fac * lg; i < fac * LEN_SUBFR; i++)
      {
        noise_loc2[i] =
            ia_mul_flt(alpha, ia_sub_flt(synth_loc[i + fac * num_subfr],
                                         synth_loc[i + fac * num_subfr - fac * pitch_lag]));
      }
    }
    else
    {
      ia_core_coder_memset(noise_loc2, fac * LEN_SUBFR);
    }

    ia_core_coder_mem_cpy(bpf_prev, noise_buf, (fac * FILTER_DELAY + fac - 1 + fac * LEN_SUBFR));
    ia_core_coder_mem_cpy(noise_buf + fac * LEN_SUBFR, bpf_prev,
                          (fac * FILTER_DELAY + fac - 1 + fac * LEN_SUBFR));

    for (i = 0; i < fac * LEN_SUBFR; i++)
    {
      tmp = ia_mul_flt(lp_filt[0], noise_loc1[i]);
      for (j = 1; j <= fac * FILTER_DELAY + fac - 1; j++)
      {
        tmp = ia_add_flt(
            tmp, ia_mul_flt(lp_filt[j], ia_add_flt(noise_loc1[i - j], noise_loc1[i + j])));
      }
      synth_out[i + fac * num_subfr] = ia_sub_flt(synth_out[i + fac * num_subfr], tmp);
    }
  }

  /* Filter LTPF_MAX_DELAY samples in the future for LTPF in TCX */
  for (i = fac * len_fr; i < fac * len_fr + LTPF_MAX_DELAY; i++)
  {
    tmp = ia_mul_flt(lp_filt[0], noise_loc1[i + fac * LEN_SUBFR - fac * len_fr]);
    for (j = 1; j <= fac * FILTER_DELAY + fac - 1; j++)
    {
      tmp = ia_add_flt(
          tmp,
          ia_mul_flt(lp_filt[j], ia_add_flt(noise_loc1[i + fac * LEN_SUBFR - fac * len_fr - j],
                                            noise_loc1[i + fac * LEN_SUBFR - fac * len_fr + j])));
      tmp = ia_mul_flt(tmp, (FLOAT32)fac);
    }
    synth_out[i] = ia_sub_flt(synth_loc[i - fac * LEN_SUBFR], tmp);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_lpd_dec_preemph_res
 *
 *  \brief Performs preemphesis and residual tools part of lpd decoder
 *
 *  \param [in/out]    usac_data      USAC data structure
 *  \param [in/out]    st          LPD decoder handle
 *  \param [in/out]    pstr_td_frame_data  TD frame data structure
 *  \param [in]        td_config          TD Config structure handle
 *  \param [in]      short_fac_flag    short fac flag read from bitstream
 *  \param [in]        tmp_scratch        Pointer to temporary scratch
 *  \param [in]        synth_fb           Pointer to synth_fb
 *  \param [in]        synth            Pointer to synth
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_lpd_dec_preemph_res(ia_usac_data_struct *usac_data,
                                                      ia_usac_lpd_decoder_handle st,
                                                      ia_td_frame_data_struct *pstr_td_frame_data,
                                                      ia_usac_td_config_handle td_config,
                                                      WORD32 short_fac_flag, FLOAT32 *tmp_scratch,
                                                      FLOAT32 *synth_fb, FLOAT32 *synth)

{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 fac_length_fb, k, length, len_fr, len_sub_frm;
  WORD32 num_samples, tmp_start;
  FLOAT32 lp_flt_coeff_a[9 * (ORDER + 1)];
  FLOAT32 tmp_buf[3 * LEN_FRAME + ORDER];
  FLOAT32 tmp_res_buf[3 * LEN_FRAME];
  FLOAT32 fac_time_sig[2 * LEN_FRAME];
  FLOAT32 fac_time_sig_fb[2 * LEN_FRAME];
  WORD32 *ptr_scratch = &usac_data->scratch_int_buf[0];
  WORD32 *pmode = pstr_td_frame_data->mod;
  FLOAT32 *tmp = &(tmp_buf[LEN_FRAME]);
  FLOAT32 *ptr_tmp = &(tmp_res_buf[LEN_FRAME]);
  FLOAT32 *xcitation_curr;

  if (st == NULL)
  {
    return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_LPD_DEC_HANDLE;
  }

  ia_core_coder_memset(lp_flt_coeff_a, (9 * (ORDER + 1)));
  ia_core_coder_memset(tmp_buf, 3 * LEN_FRAME + ORDER);
  ia_core_coder_memset(tmp_res_buf, 3 * LEN_FRAME);
  ia_core_coder_memset(fac_time_sig, 2 * LEN_FRAME);
  ia_core_coder_memset(fac_time_sig_fb, 2 * LEN_FRAME);
  len_fr = td_config->len_frame;
  len_sub_frm = td_config->len_subfrm;
  xcitation_curr = usac_data->exc_buf + MAX_PITCH + INTER_LP_FIL_ORDER + 1;

  ia_core_coder_interpolate_lsp_params(st->lspold, st->lspold, lp_flt_coeff_a, 8);
  ia_core_coder_mem_cpy(lp_flt_coeff_a, st->lp_flt_coeff_a_prev, (ORDER + 1));
  ia_core_coder_mem_cpy(lp_flt_coeff_a, st->lp_flt_coeff_a_prev + ORDER + 1, (ORDER + 1));

  if (pmode[0] == 0)
  {
    WORD32 fac_length;
    if (0 == short_fac_flag)
    {
      fac_length = len_fr >> 3;
    }
    else
    {
      fac_length = len_fr >> 4;
    }

    if ((pstr_td_frame_data->fd_fac[0] > 128) || (pstr_td_frame_data->fd_fac[0] < 0))
    {
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_FD_FACTOR;
    }
    fac_length_fb = fac_length;
    if (td_config->full_band_lpd)
    {
      fac_length_fb = fac_length << 1;
    }

    memcpy(ptr_scratch, &pstr_td_frame_data->fd_fac[0], (FAC_LENGTH + 1) * sizeof(WORD32));

    err = ia_core_coder_fwd_alias_cancel_tool(
        usac_data, pstr_td_frame_data->fd_fac, td_config, fac_length_fb, lp_flt_coeff_a,
        fac_time_sig, fac_time_sig_fb, st->fscale, tmp_scratch, NULL, 0, len_sub_frm);
    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }

    if (st->overlap_buf)
    {
      FLOAT32 *ola_buf = &st->overlap_buf[2 * len_sub_frm - fac_length];
      for (k = 0; k < fac_length; k++)
      {
        ola_buf[k] = ia_add_flt(ola_buf[k], fac_time_sig[k]);
      }
    }
    if (td_config->full_band_lpd)
    {
      FLOAT32 *ola_buf_fb = st->overlap_buf_fb + 2 * len_sub_frm - fac_length_fb;
      if (st != NULL)
      {
        for (k = 0; k < fac_length_fb; k++)
        {
          ola_buf_fb[k] = ia_add_flt(ola_buf_fb[k], fac_time_sig_fb[k]);
        }
        ia_core_coder_memset(st->overlap_buf_fb + 2 * len_sub_frm, fac_length_fb);
        ia_core_coder_mem_cpy(st->overlap_buf_fb, synth_fb - 2 * len_sub_frm, 2 * len_sub_frm);
      }
      else
      {
        ia_core_coder_mem_cpy(fac_time_sig_fb, synth_fb - fac_length_fb, fac_length_fb);
      }
    }
    if (st->overlap_buf)
    {
      ia_core_coder_memset(&st->overlap_buf[2 * len_sub_frm], fac_length);
    }
  }

  if (st->overlap_buf)
  {
    ia_core_coder_mem_cpy(st->overlap_buf, &st->fd_synth[ORDER], len_sub_frm << 1);
  }
  ia_core_coder_mem_cpy(st->fd_synth + ORDER, synth - 2 * len_sub_frm, 2 * len_sub_frm);

  ia_core_coder_preemphsis_tool(st->fd_synth + ORDER, PREEMPH_FILT_FAC, 2 * len_sub_frm, 0);

  ia_core_coder_memset(tmp, ORDER);
  ia_core_coder_mem_cpy(st->fd_synth + ORDER, tmp + ORDER, 2 * len_sub_frm);
  tmp_start = 0;

  ia_core_coder_memset(ptr_tmp - len_sub_frm, 3 * len_sub_frm);
  memset(st->fd_synth, 0, ORDER * sizeof(FLOAT32));
  length = (2 * len_sub_frm - tmp_start) / LEN_SUBFR;

  ia_core_coder_residual_tool(lp_flt_coeff_a, &st->fd_synth[ORDER + tmp_start],
                              &ptr_tmp[tmp_start], LEN_SUBFR, length, 0);
  if (pmode[0] != 0 && (len_sub_frm == LEN_FRAME || pmode[1] != 0))
  {
    num_samples = ia_min_int(len_sub_frm, MAX_PITCH + INTER_LP_FIL_ORDER + 1);
  }
  else
  {
    num_samples = ia_min_int(2 * len_sub_frm, MAX_PITCH + INTER_LP_FIL_ORDER + 1);
  }
  ia_core_coder_mem_cpy(ptr_tmp + 2 * len_sub_frm - num_samples, xcitation_curr - num_samples,
                        num_samples);

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_lpd_dec_tbe_resample
 *
 *  \brief Performs TBE and Resampling as part of lpd decoder
 *
 *  \param [in/out]    usac_data          USAC data structure
 *  \param [in/out]    st                 LPD decoder handle
 *  \param [in/out]    pstr_td_frame_data TD frame data structure
 *  \param [in]        td_config          TD Config structure handle
 *  \param [in/out]    pscr               LPD dec scratch structure handle
 *  \param [in]        bpf_control_info   bpf control info flag read from bitstream
 *  \param [in]        lp_filt_coef       Pointer to LP filter coefficients
 *  \param [in]        tmp_scratch        Pointer to temporary scratch
 *  \param [in]        synth_fb           Pointer to synth_fb
 *  \param [in]        synth              Pointer to synth
 *  \param [in]        stability_factor   Stability factor
 *  \param [in]        k                  current frame index
 *  \return IA_ERRORCODE
 *
 */

static IA_ERRORCODE ia_core_coder_lpd_dec_tbe_resample(
    ia_usac_data_struct *usac_data, ia_usac_lpd_decoder_handle st,
    ia_td_frame_data_struct *pstr_td_frame_data, ia_usac_td_config_handle td_config,
    ia_core_coder_lpd_dec_scr_t *pscr, WORD32 bpf_control_info, FLOAT32 *lp_filt_coef,
    FLOAT32 *tmp_scratch, FLOAT32 *synth_fb, FLOAT32 *synth, FLOAT32 stability_factor, WORD32 k)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, len_sub_frm;
  WORD32 num_subfr, num_subfr_by2, first_tbe_frame;
  WORD32 rs_delay;

  WORD32 *pmode = pstr_td_frame_data->mod;
  WORD32 *pitch = usac_data->pitch;
  WORD32 tbe_delay_internal, tbe_delay;

  FLOAT32 *pitch_gain = usac_data->pitch_gain;

  len_sub_frm = td_config->len_subfrm;
  num_subfr = td_config->num_subfrm;
  num_subfr_by2 = ((td_config->num_frame * td_config->num_subfrm) / 2) - 1;

  ia_core_coder_interpolate_lsp_params(st->lspold, pscr->lsp_curr, lp_filt_coef, num_subfr);
  err = ia_core_coder_acelp_alias_cnx(
      usac_data, pstr_td_frame_data, k, lp_filt_coef, stability_factor, st, td_config,
      &synth_fb[k * len_sub_frm * td_config->fscale_full_band / td_config->fscale],
      pscr->pitch_buffer, pscr->voice_factors, tmp_scratch);

  if (td_config->full_band_lpd)
  {
    WORD32 tbe_frm_class;

    if (k == 0)
    {
      tbe_frm_class = 0;
    }
    else
    {
      tbe_frm_class = 1;
    }

    tbe_delay_internal = TBE_DEC_DELAY;
    tbe_delay = 2 * (len_sub_frm / 2 - tbe_delay_internal);

    first_tbe_frame = 0;
    if ((k == 1 && pmode[0] != 0) || (k == 0 && st->mode_prev != 0))
    {
      first_tbe_frame = 1;
    }

    ia_core_coder_tbe_apply(&st->tbe_dec_data, tbe_frm_class, pscr->bwe_exc_extended,
                            pscr->voice_factors, &pscr->tbe_synth[2 * k * len_sub_frm],
                            first_tbe_frame, tmp_scratch);

    /* delay alignment */
    ia_core_coder_mem_cpy(&pscr->tbe_synth[2 * k * len_sub_frm], pscr->tmp_tbe, 2 * len_sub_frm);
    ia_core_coder_mem_cpy(st->tbe_dec_data.synth_curr, pscr->tbe_synth_align, tbe_delay);
    ia_core_coder_mem_cpy(pscr->tmp_tbe, pscr->tbe_synth_align + tbe_delay,
                          2 * len_sub_frm - tbe_delay);
    ia_core_coder_mem_cpy(pscr->tmp_tbe + (2 * len_sub_frm - tbe_delay),
                          st->tbe_dec_data.synth_curr, tbe_delay);
  }
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }

  if (bpf_control_info && (st->mode_prev != 0))
  {
    i = (k * num_subfr) + num_subfr_by2;
    if (st->mode_prev == -2)
    {
      pitch[i - 1] = pitch[i];
      pitch_gain[i - 1] = pitch_gain[i];
    }
    else
    {
      pitch[i - 2] = pitch[i - 1] = pitch[i];
      pitch_gain[i - 2] = pitch_gain[i - 1] = pitch_gain[i];
    }
  }

  if (td_config->full_band_lpd)
  {
    if (st->mode_prev != 0)
    {
      ia_core_coder_mem_cpy(synth + k * len_sub_frm - len_sub_frm / 2 - st->td_resamp_delay / 2,
                            st->td_resamp_mem, st->td_resamp_delay);
    }
    ia_core_coder_td_resampler(synth + k * len_sub_frm - len_sub_frm / 2 +
                                   st->td_resamp_delay / 2,
                               (WORD16)len_sub_frm, td_config->fscale, pscr->synth_rs,
                               td_config->fscale_full_band, st->td_resamp_mem, 0, tmp_scratch);
    /* add TBE signal to to resampled ACELP signal */
    for (i = 0; i < td_config->fac_fb * len_sub_frm; i++)
    {
      pscr->synth_rs[i] = ia_add_flt(pscr->synth_rs[i], pscr->tbe_synth_align[i]);
    }

    /*TCX->ACELP*/
    if (st->mode_prev != 0)
    {
      rs_delay = st->td_resamp_delay;
      /* crossfade from FB-TCX (incl. FAC) to resampled LB synth */
      for (i = 1; i < rs_delay; i++)
      {
        synth_fb[k * len_sub_frm * td_config->fac_fb - rs_delay + i] =
            ia_mul_flt(synth_fb[k * len_sub_frm * td_config->fac_fb - rs_delay + i],
                       ((rs_delay - i) / ((FLOAT32)rs_delay)));
        synth_fb[k * len_sub_frm * td_config->fac_fb - rs_delay + i] =
            ia_add_flt(synth_fb[k * len_sub_frm * td_config->fac_fb - rs_delay + i],
                       (ia_mul_flt(pscr->synth_rs[len_sub_frm + i - rs_delay], (FLOAT32)i) /
                        ((FLOAT32)rs_delay)));
      }

      /* copy the rest of resampled LB synth to FB synth */
      ia_core_coder_mem_cpy(pscr->synth_rs + len_sub_frm,
                            synth_fb + (k * len_sub_frm * td_config->fac_fb), len_sub_frm);
    }
    /*ACELP->ACELP*/
    else
    {
      /* copy from resampled LB synth to FB synth */
      ia_core_coder_mem_cpy(pscr->synth_rs,
                            synth_fb + (k * len_sub_frm * td_config->fac_fb) - len_sub_frm,
                            len_sub_frm * td_config->fac_fb);
    }

    /* reset previous TCX window shape if current frame is ACELP */
    st->window_shape_prev = WIN_SEL_0;
  }

  return err;
}

/**
 *  ia_core_coder_lpd_dec_tcx_resample
 *
 *  \brief Performs TCX MDCT and Resampling as part of lpd decoder
 *
 *  \param [in/out]    usac_data          USAC data structure
 *  \param [in/out]    st                 LPD decoder handle
 *  \param [in/out]    pstr_td_frame_data TD frame data structure
 *  \param [in]        td_config          TD Config structure handle
 *  \param [in/out]    pscr               LPD dec scratch structure handle
 *  \param [in]        lp_filt_coef       Pointer to LP filter coefficients
 *  \param [in]        tmp_scratch        Pointer to temporary scratch
 *  \param [in]        synth_fb           Pointer to synth_fb
 *  \param [in]        synth              Pointer to synth
 *  \param [in]        elem_idx           Element Index
 *  \param [in]        k                  current frame index
 *  \param [in]        mode               Mode
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_lpd_dec_tcx_resample(
    ia_usac_data_struct *usac_data, ia_usac_lpd_decoder_handle st,
    ia_td_frame_data_struct *pstr_td_frame_data, ia_usac_td_config_handle td_config,
    ia_core_coder_lpd_dec_scr_t *pscr, FLOAT32 *lp_filt_coef, FLOAT32 *tmp_scratch,
    FLOAT32 *synth_fb, FLOAT32 *synth, WORD32 elem_idx, WORD32 k, WORD32 mode)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, len_sub_frm;
  WORD32 fac_length_fb;
  WORD32 num_subfr;
  WORD32 rs_delay;
  WORD32 n_subfr = 0, subfr_len = 0, num_subfr_in_superfr;
  WORD32 tbe_delay_internal, tbe_delay;

  len_sub_frm = td_config->len_subfrm;
  num_subfr = td_config->num_subfrm;
  num_subfr_in_superfr = td_config->num_frame * num_subfr;

  switch (mode)
  {
  case 1:
    subfr_len = len_sub_frm;
    n_subfr = num_subfr;
    break;
  case 2:
    subfr_len = len_sub_frm << 1;
    n_subfr = num_subfr_in_superfr >> 1;
    break;
  case 3:
    subfr_len = len_sub_frm << 2;
    n_subfr = num_subfr_in_superfr;
    break;
  }

  if (td_config->full_band_lpd && st->mode_prev == 0)
  {
    tbe_delay_internal = TBE_DEC_DELAY;
    tbe_delay = 2 * (len_sub_frm / 2 - tbe_delay_internal);

    ia_core_coder_tbe_gen_transition(&st->tbe_dec_data,
                                     (len_sub_frm + st->td_resamp_delay) - tbe_delay,
                                     &pscr->tbe_synth[2 * k * len_sub_frm], tmp_scratch);

    /* delay alignment */
    ia_core_coder_mem_cpy(&pscr->tbe_synth[2 * k * len_sub_frm], pscr->tmp_tbe, 2 * len_sub_frm);
    ia_core_coder_mem_cpy(st->tbe_dec_data.synth_curr, pscr->tbe_synth_align, tbe_delay);
    ia_core_coder_mem_cpy(pscr->tmp_tbe, pscr->tbe_synth_align + tbe_delay,
                          2 * len_sub_frm - tbe_delay);
    ia_core_coder_mem_cpy(pscr->tmp_tbe + (2 * len_sub_frm - tbe_delay),
                          st->tbe_dec_data.synth_curr, tbe_delay);

    ia_core_coder_reset_tbe_state(&st->tbe_dec_data);
  }
  ia_core_coder_interpolate_lpc_coef(st->lspold, pscr->lsp_curr, lp_filt_coef, n_subfr, ORDER);

  err = ia_core_coder_tcx_mdct(
      usac_data, pstr_td_frame_data, k, lp_filt_coef,
      &synth_fb[k * len_sub_frm * (td_config->fscale_full_band / td_config->fscale)], subfr_len,
      st, elem_idx, tmp_scratch);
  if (td_config->full_band_lpd && st->mode_prev == 0)
  {
    /*ACELP -> TCX*/
    ia_core_coder_td_resampler(
        synth + k * len_sub_frm - len_sub_frm / 2 + st->td_resamp_delay / 2,
        (WORD16)(len_sub_frm / 2 + st->td_resamp_delay / 2), td_config->fscale, pscr->synth_rs,
        td_config->fscale_full_band, st->td_resamp_mem, 0, tmp_scratch);
    fac_length_fb = (len_sub_frm / 2) * td_config->fac_fb;
    rs_delay = st->td_resamp_delay;

    /* add TBE transition signal */
    for (i = 0; i < (len_sub_frm + st->td_resamp_delay); i++)
    {
      pscr->synth_rs[i] = ia_add_flt(pscr->synth_rs[i], pscr->tbe_synth_align[i]);
    }

    /* copy the beginning of resampled LB synth to FB synth */
    ia_core_coder_mem_cpy(pscr->synth_rs,
                          synth_fb + (k * len_sub_frm * td_config->fac_fb) - len_sub_frm,
                          fac_length_fb);

    /* crossfade from resampled LB synth to FB-TCX (incl. FAC) */
    for (i = 0; i < rs_delay; i++)
    {
      synth_fb[k * len_sub_frm * td_config->fac_fb - len_sub_frm + fac_length_fb + i] =
          ia_mul_flt(
              synth_fb[k * len_sub_frm * td_config->fac_fb - len_sub_frm + fac_length_fb + i],
              ((FLOAT32)(i) / ((FLOAT32)rs_delay)));
      synth_fb[k * len_sub_frm * td_config->fac_fb - len_sub_frm + fac_length_fb + i] =
          ia_add_flt(
              synth_fb[k * len_sub_frm * td_config->fac_fb - len_sub_frm + fac_length_fb + i],
              (ia_mul_flt(pscr->synth_rs[fac_length_fb + i], (FLOAT32)(rs_delay - i)) /
               ((FLOAT32)rs_delay)));
    }
  }

  return err;
}

/**
 *  ia_core_coder_lpd_dec_lpd_stereo_proc
 *
 *  \brief Performs LPD stereo processing as part of lpd decoder
 *
 *  \param [in/out]    usac_data          USAC data structure
 *  \param [in/out]    st                 LPD decoder handle
 *  \param [in]        td_config          TD Config structure handle
 *  \param [in]        fsynth             Pointer to fsynth
 *  \param [in/out]    pscr               LPD dec scratch structure handle
 *  \param [in]        synth_fb           Pointer to synth_fb
 *  \param [in]        chan               Channel Number
 *  \param [in]        mode               Frame mode
 *  \param [in]        synth_delay        Synthesis delay
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_lpd_dec_lpd_stereo_proc(
    ia_usac_data_struct *usac_data, ia_usac_lpd_decoder_handle st,
    ia_usac_td_config_handle td_config, FLOAT32 *fsynth, ia_core_coder_lpd_dec_scr_t *pscr,
    FLOAT32 *synth_fb, WORD32 chan, WORD32 mode, WORD32 synth_delay)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD8 i, k, slpd_index;
  WORD32 bpf_syn_delay, tp;
  WORD32 len_fr, len_sub_frm, num_subfr_in_superfr;

  FLOAT32 gain;
  FLOAT32 synth_corr, synth_energy;

  WORD32 *pitch = usac_data->pitch;

  FLOAT32 *pitch_gain = usac_data->pitch_gain;
  FLOAT32 *synth;
  FLOAT32 *synth_buf = usac_data->synth_buf;

  len_fr = td_config->len_frame;
  len_sub_frm = td_config->len_subfrm;
  num_subfr_in_superfr = td_config->num_frame * td_config->num_subfrm;

  if (td_config->lpd_stereo_idx)
  {
    slpd_index = usac_data->slpd_index[chan / 2];
    ia_usac_slpd_dec_data_struct *slpd_dec_data = &usac_data->slpd_dec_data[slpd_index];
    if (td_config->full_band_lpd == 1)
    {
      ia_core_coder_mem_cpy(
          pscr->synth_buf_fb + (MAX_PITCH - BPF_DELAY * LEN_SUBFR) * td_config->fac_fb,
          slpd_dec_data->time_buff_dmx,
          slpd_dec_data->lpd_stereo_config.ccfl + (slpd_dec_data->lpd_stereo_config.ccfl / 2));
      ia_core_coder_mem_cpy(pscr->synth_buf_fb +
                                (MAX_PITCH - BPF_DELAY * LEN_SUBFR) * td_config->fac_fb,
                            fsynth, len_fr * td_config->fac_fb);
    }
    else
    {
      ia_core_coder_mem_cpy(
          synth_buf + (MAX_PITCH - BPF_DELAY * LEN_SUBFR), slpd_dec_data->time_buff_dmx,
          slpd_dec_data->lpd_stereo_config.ccfl + (slpd_dec_data->lpd_stereo_config.ccfl / 2));

      ia_core_coder_mem_cpy(synth_buf + (MAX_PITCH - BPF_DELAY * LEN_SUBFR), fsynth,
                            td_config->len_frame);
    }

    memcpy(slpd_dec_data->bpf_pitch, &slpd_dec_data->bpf_pitch[num_subfr_in_superfr],
           BPF_DELAY * sizeof(slpd_dec_data->bpf_pitch[0]));
    ia_core_coder_mem_cpy(&slpd_dec_data->bpf_gain[num_subfr_in_superfr], slpd_dec_data->bpf_gain,
                          BPF_DELAY);

    memcpy(&slpd_dec_data->bpf_pitch[BPF_DELAY], pitch,
           num_subfr_in_superfr * sizeof(slpd_dec_data->bpf_pitch[0]));
    ia_core_coder_mem_cpy(pitch_gain, &slpd_dec_data->bpf_gain[BPF_DELAY], num_subfr_in_superfr);
  }
  else
  {
    synth = synth_buf + MAX_PITCH;

    if (td_config->full_band_lpd)
    {
      synth_fb = pscr->synth_buf_fb + MAX_PITCH * td_config->fac_fb;
    }

    for (i = 0; i < num_subfr_in_superfr; i++)
    {
      tp = pitch[i];
      gain = pitch_gain[i];
      if (ia_lt_flt(0, gain))
      {
        synth_corr = 0.0f, synth_energy = 1e-6f;
        for (k = 0; k < LEN_SUBFR; k++)
        {
          synth_corr = ia_add_flt(
              synth_corr, ia_mul_flt(synth[i * LEN_SUBFR + k], synth[(i * LEN_SUBFR) - tp + k]));
          synth_energy = ia_add_flt(synth_energy, ia_mul_flt(synth[(i * LEN_SUBFR) - tp + k],
                                                             synth[(i * LEN_SUBFR) - tp + k]));
        }
        pitch_gain[i] = synth_corr / synth_energy;
      }
    }

    if (mode == 0 && td_config->full_band_lpd == 0)
    {
      bpf_syn_delay = synth_delay;
    }
    else
    {
      bpf_syn_delay = synth_delay - (len_sub_frm / 2);
    }
    err = ia_core_coder_bass_post_filter(synth, synth_fb, td_config->fac_fb, pitch, pitch_gain,
                                         fsynth, len_fr, bpf_syn_delay, st->bpf_prev);
  }

  return err;
}
/**
 *  ia_core_coder_lpd_dec
 *
 *  \brief Performs lpd decoder
 *
 *  \param [in/out]    usac_data      USAC data structure
 *  \param [in/out]    st          LPD decoder handle
 *  \param [in/out]    pstr_td_frame_data  TD frame data structure
 *  \param [in/out]    fsynth        synthesized signal
 *  \param [in]      first_lpd_flag    flag to detect if first LPD frame
 *  \param [in]      short_fac_flag    short fac flag read from bitstream
 *  \param [in]      bpf_control_info  bpf control info flag read from bitstream
 *  \param [in]      elem_idx      element index in num of elements
 *  \param [in]      chan          channel number
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_lpd_dec(ia_usac_data_struct *usac_data, ia_usac_lpd_decoder_handle st,
                                   ia_td_frame_data_struct *pstr_td_frame_data, FLOAT32 *fsynth,
                                   WORD32 first_lpd_flag, WORD32 short_fac_flag,
                                   WORD32 bpf_control_info, WORD32 elem_idx, WORD32 chan)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 len_fr;
  WORD32 len_subfrm;
  WORD32 num_subfr;
  WORD32 num_subfr_in_superfr;
  WORD32 num_subfr_by2;
  WORD32 synth_delay;
  WORD32 i, k, mode = 0;

  FLOAT32 tmp_val = 0.0f, tmp_val2;
  FLOAT32 stability_factor;

  WORD32 *pitch = usac_data->pitch;
  WORD32 *pmode;

  FLOAT32 *synth_buf = usac_data->synth_buf;
  FLOAT32 *xcitation_buff = usac_data->exc_buf;
  FLOAT32 *lp_flt_coff_a = usac_data->lp_flt_coff;
  FLOAT32 *pitch_gain = usac_data->pitch_gain;
  FLOAT32 *synth;
  FLOAT32 *synth_fb = NULL;
  FLOAT32 *tmp_scratch;
  ia_usac_td_config_handle td_config = &usac_data->td_config[elem_idx];
  ia_core_coder_lpd_dec_scr_t *pscr = (ia_core_coder_lpd_dec_scr_t *)usac_data->ptr_fft_scratch;

  len_fr = td_config->len_frame;
  len_subfrm = td_config->len_subfrm;
  num_subfr = td_config->num_subfrm;
  num_subfr_in_superfr = td_config->num_frame * num_subfr;
  num_subfr_by2 = (num_subfr_in_superfr / 2) - 1;
  synth_delay = num_subfr_by2 * LEN_SUBFR;

  tmp_scratch =
      usac_data->ptr_fft_scratch +
      (sizeof(ia_core_coder_lpd_dec_scr_t) + (sizeof(usac_data->ptr_fft_scratch[0]) - 1)) /
          sizeof(usac_data->ptr_fft_scratch[0]);

  synth = synth_buf + MAX_PITCH + synth_delay;
  ia_core_coder_mem_cpy(st->synth_prev, synth_buf, MAX_PITCH + synth_delay);
  ia_core_coder_memset(synth, SYNTH_DELAY_LMAX + LEN_SUPERFRAME - synth_delay);

  ia_core_coder_mem_cpy(st->xcitation_prev, xcitation_buff, MAX_PITCH + INTER_LP_FIL_ORDER + 1);

  if (td_config->full_band_lpd)
  {
    synth_fb = pscr->synth_buf_fb + (MAX_PITCH + synth_delay) * td_config->fac_fb;
    ia_core_coder_mem_cpy(st->synth_prev_fb, pscr->synth_buf_fb,
                          (MAX_PITCH + synth_delay) * td_config->fac_fb);
  }

  pmode = pstr_td_frame_data->mod;
  memcpy(pitch, st->pitch_prev, num_subfr_by2 * sizeof(pitch[0]));
  memcpy(pitch_gain, st->gain_prev, num_subfr_by2 * sizeof(pitch[0]));
  ia_core_coder_memset(&pitch_gain[num_subfr_by2], num_subfr_in_superfr);
  for (i = 0; i < num_subfr_in_superfr; i++)
  {
    pitch[i + num_subfr_by2] = 64;
  }
  if (!first_lpd_flag)
  {
    ia_core_coder_mem_cpy(st->lsf_prev, pscr->lsf, ORDER);
  }
  ia_core_coder_alg_vec_dequant(pstr_td_frame_data, first_lpd_flag, pscr->lsf,
                                pstr_td_frame_data->mod, td_config->num_frame);

  if (first_lpd_flag)
  {
    ia_core_coder_mem_cpy(&pscr->lsf[0], st->lsf_prev, ORDER);
    ia_core_coder_lsf_2_lsp_conversion(st->lsf_prev, st->lspold);
  }

  if (first_lpd_flag && (pmode[0] == 0 || pmode[1] == 0 ||
                         (!td_config->full_band_lpd && pmode[2] == 0 && len_subfrm != LEN_FRAME)))
  {
    ia_core_coder_lpd_dec_preemph_res(usac_data, st, pstr_td_frame_data, td_config,
                                      short_fac_flag, tmp_scratch, synth_fb, synth);
  }

  if (first_lpd_flag && pmode[0] > 0 && td_config->full_band_lpd && st != NULL)
  {
    ia_core_coder_mem_cpy(st->overlap_buf_fb, synth_fb - 2 * len_subfrm, 2 * len_subfrm);
  }

  k = 0;

  while (k < td_config->num_frame)
  {
    mode = pmode[k];
    if ((st->mode_prev == 0) && (mode > 0) && (k != 0 || st->bpf_active_prev == 1))
    {
      i = (k * num_subfr) + num_subfr_by2;
      pitch_gain[i + 1] = pitch_gain[i] = pitch_gain[i - 1];
      pitch[i + 1] = pitch[i] = pitch[i - 1];
    }

    switch (mode)
    {
    case 0:
    case 1:
      ia_core_coder_mem_cpy(&pscr->lsf[(k + 1) * ORDER], pscr->lsf_curr, ORDER);
      break;
    case 2:
      ia_core_coder_mem_cpy(&pscr->lsf[(k + 2) * ORDER], pscr->lsf_curr, ORDER);
      break;
    default:
      ia_core_coder_mem_cpy(&pscr->lsf[(k + 4) * ORDER], pscr->lsf_curr, ORDER);
      break;
    }

    ia_core_coder_lsf_2_lsp_conversion(pscr->lsf_curr, pscr->lsp_curr);
    tmp_val = 0.0f;
    for (i = 0; i < ORDER; i++)
    {
      tmp_val2 = ia_sub_flt(pscr->lsf_curr[i], st->lsf_prev[i]);
      tmp_val = ia_add_flt(tmp_val, ia_mul_flt(tmp_val2, tmp_val2));
    }

    stability_factor = (FLOAT32)ia_sub_flt(1.25f, (tmp_val / 400000.0f));
    stability_factor = ia_min_flt(1.0f, stability_factor);
    stability_factor = ia_max_flt(0.0f, stability_factor);

    if (mode == 0)
    {
      ia_core_coder_lpd_dec_tbe_resample(usac_data, st, pstr_td_frame_data, td_config, pscr,
                                         bpf_control_info, lp_flt_coff_a, tmp_scratch, synth_fb,
                                         synth, stability_factor, k);
      k++;
    }
    else
    {
      err = ia_core_coder_lpd_dec_tcx_resample(usac_data, st, pstr_td_frame_data, td_config, pscr,
                                               lp_flt_coff_a, tmp_scratch, synth_fb, synth,
                                               elem_idx, k, mode);
      if (err != IA_MPEGH_DEC_NO_ERROR)
      {
        return err;
      }
      k += (1 << (mode - 1));
    }

    st->mode_prev = mode;

    ia_core_coder_mem_cpy(pscr->lsp_curr, st->lspold, ORDER);
    ia_core_coder_mem_cpy(pscr->lsf_curr, st->lsf_prev, ORDER);
  }
  ia_core_coder_mem_cpy(xcitation_buff + len_fr, st->xcitation_prev,
                        MAX_PITCH + INTER_LP_FIL_ORDER + 1);
  ia_core_coder_mem_cpy(synth_buf + len_fr, st->synth_prev, MAX_PITCH + synth_delay);

  if (td_config->full_band_lpd)
  {
    ia_core_coder_mem_cpy(pscr->synth_buf_fb + len_fr * td_config->fac_fb, st->synth_prev_fb,
                          (MAX_PITCH + synth_delay) * td_config->fac_fb);
  }

  /* reset FD predictor states */
  if (mode != (td_config->full_band_lpd ? 2 : 3))
  {
    WORD32 lines;
    if (td_config->num_frame == 192)
    {
      lines = 120;
    }
    else
    {
      lines = 160;
    }
    memset(st->fdp_quant_spec_prev[0], 0, lines * sizeof(st->fdp_quant_spec_prev[0][0]));
    memset(st->fdp_quant_spec_prev[1], 0, lines * sizeof(st->fdp_quant_spec_prev[1][0]));
  }

  if (!bpf_control_info)
  {
    if (st->bpf_active_prev && pmode[0] != 0)
    {
      ia_core_coder_memset(&pitch_gain[num_subfr_by2 + 2], num_subfr_in_superfr - 2);
    }
    else
    {
      ia_core_coder_memset(&pitch_gain[num_subfr_by2], num_subfr_in_superfr);
    }
  }
  st->bpf_active_prev = bpf_control_info;

  memcpy(st->pitch_prev, &pitch[num_subfr_in_superfr], num_subfr_by2 * sizeof(st->pitch_prev[0]));
  ia_core_coder_mem_cpy(&pitch_gain[num_subfr_in_superfr], st->gain_prev, num_subfr_by2);
  err = ia_core_coder_lpd_dec_lpd_stereo_proc(usac_data, st, td_config, fsynth, pscr, synth_fb,
                                              chan, mode, synth_delay);
  return err;
}

/**
 *  impeghd_synth_end_fullband
 *
 *  \brief Performs final synthesis part updation of fullband
 *
 *  \param [in/out]  signal_out    synthesized output signal
 *  \param [in]    st        LPD decoder handle
 *  \param [in]    td_config    TD config handle
 *  \param [in]			pscr			Synth end scratch handle
 *  \param [in]			scratch_mem		Scratch buffer used for internal
 * processing
 *
 *  \return VOID
 *
 */
VOID impeghd_synth_end_fullband(FLOAT32 *signal_out, ia_usac_lpd_decoder_handle st,
                                ia_usac_td_config_handle td_config,
                                ia_core_coder_synth_end_scr_t *pscr, FLOAT32 *scratch_mem)
{
  WORD32 fac_length, frame_len, lpd_sfd, lpd_delay, syn_sfd, synth_delay;

  FLOAT32 *synth;

  frame_len = td_config->len_frame_fb;
  lpd_sfd = (td_config->num_frame * td_config->num_subfrm) / 2;
  lpd_delay = lpd_sfd * LEN_SUBFR;
  syn_sfd = lpd_sfd - BPF_SUBFRAME;
  synth_delay = syn_sfd * LEN_SUBFR;
  fac_length = (td_config->len_subfrm) / 2;
  ia_core_coder_memset(pscr->synth_buf,
                       (MAX_PITCH + synth_delay) * td_config->fac_fb + frame_len);

  synth = pscr->synth_buf + (MAX_PITCH + synth_delay) * td_config->fac_fb;
  if (st->mode_prev != 0)
  {
    ia_core_coder_mem_cpy(st->synth_prev_fb, pscr->synth_buf,
                          (MAX_PITCH + synth_delay) * td_config->fac_fb);

    ia_core_coder_mem_cpy(st->exc_prev_fb, synth - 2 * fac_length, 4 * fac_length);
  }
  else
  {
    ia_core_coder_mem_cpy(st->synth_prev_fb, pscr->synth_buf,
                          (MAX_PITCH + synth_delay) * td_config->fac_fb - td_config->len_subfrm -
                              st->td_resamp_delay);

    ia_core_coder_mem_cpy(st->synth_prev + (MAX_PITCH + synth_delay) - td_config->len_subfrm / 2 -
                              st->td_resamp_delay,
                          st->td_resamp_mem, st->td_resamp_delay);

    ia_core_coder_td_resampler(st->exc_prev + 1, (WORD16)(fac_length * 2), st->fscale,
                               pscr->synth_resamp, td_config->fscale_full_band, st->td_resamp_mem,
                               0, scratch_mem);
    ia_core_coder_mem_cpy(pscr->synth_resamp, synth - fac_length * 2 - st->td_resamp_delay,
                          4 * fac_length);

    ia_core_coder_memset(synth + 2 * fac_length - st->td_resamp_delay, st->td_resamp_delay);
  }

  synth = pscr->synth_buf + (MAX_PITCH - (BPF_SUBFRAME * LEN_SUBFR)) * td_config->fac_fb;
  ia_core_coder_mem_cpy(synth, signal_out, frame_len);
  ia_core_coder_memset(signal_out + lpd_delay * td_config->fac_fb + fac_length,
                       frame_len - lpd_delay * td_config->fac_fb - fac_length);
}

/**
 *  impeghd_synth_end
 *
 *  \brief Performs final synthesis part updation
 *
 *  \param [in/out]  signal_out    synthesized output signal
 *  \param [in]    st        LPD decoder handle
 *  \param [in]    td_config    TD config handle
 *  \param [in]    scratch_mem    Scratch buffer used for internal processing
 *
 *  \return VOID
 *
 */
static VOID impeghd_synth_end(FLOAT32 *signal_out, ia_usac_lpd_decoder_handle st,
                              ia_usac_td_config_handle td_config, FLOAT32 *scratch_mem)
{
  WORD32 fac_length, frame_len, lpd_sfd, lpd_delay, syn_sfd, synth_delay;

  FLOAT32 *synth;
  FLOAT32 *resample_scratch;

  ia_core_coder_synth_end_scr_t *pscr = (ia_core_coder_synth_end_scr_t *)scratch_mem;

  resample_scratch =
      scratch_mem +
      (sizeof(ia_core_coder_synth_end_scr_t) + sizeof(scratch_mem) - 1) / sizeof(scratch_mem);

  if (td_config->full_band_lpd)
  {
    impeghd_synth_end_fullband(signal_out, st, td_config, pscr, resample_scratch);
  }
  else
  {
    frame_len = td_config->len_frame;
    lpd_sfd = (td_config->num_frame * td_config->num_subfrm) / 2;
    lpd_delay = lpd_sfd * LEN_SUBFR;
    syn_sfd = lpd_sfd - BPF_SUBFRAME;
    synth_delay = syn_sfd * LEN_SUBFR;
    fac_length = (td_config->len_subfrm) / 2;

    synth = pscr->synth_buf + MAX_PITCH + synth_delay;

    ia_core_coder_mem_cpy(st->synth_prev, pscr->synth_buf, MAX_PITCH + synth_delay);
    ia_core_coder_mem_cpy(&st->exc_prev[1], &synth[-fac_length], fac_length << 1);

    synth = pscr->synth_buf + MAX_PITCH - (BPF_SUBFRAME * LEN_SUBFR);
    ia_core_coder_mem_cpy(synth, signal_out, frame_len);

    ia_core_coder_memset(signal_out + lpd_delay + fac_length, frame_len - lpd_delay - fac_length);
  }

  return;
}

/**
 *  ia_core_coder_lpd_dec_update
 *
 *  \brief Update lpd decoder if previous frame is TD
 *
 *  \param [in]    tddec    LPD decoder handle
 *  \param [in]    usac_data  USAC data structure
 *  \param [in]    i_ch    Channel for processing
 *  \param [in]    elem_idx  element index among number of elements
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_lpd_dec_update(ia_usac_lpd_decoder_handle tddec,
                                  ia_usac_data_struct *usac_data, WORD32 i_ch, WORD32 elem_idx)
{
  ia_core_coder_lpd_dec_upd_scr_t *pscr =
      (ia_core_coder_lpd_dec_upd_scr_t *)usac_data->ptr_fft_scratch;

  FLOAT32 *ptr_fft_scratch =
      usac_data->ptr_fft_scratch +
      (sizeof(ia_core_coder_lpd_dec_upd_scr_t) + sizeof(ptr_fft_scratch[0]) - 1) /
          sizeof(ptr_fft_scratch[0]);

  impeghd_synth_end(pscr->synth, tddec, &usac_data->td_config[elem_idx], ptr_fft_scratch);
  ia_core_coder_mem_cpy(pscr->synth, usac_data->overlap_data_ptr[i_ch], usac_data->ccfl);

  if (tddec->mode_prev != 0)
  {
    usac_data->window_shape_prev[i_ch] = tddec->window_shape_prev;
    usac_data->window_sequence_last[i_ch] = EIGHT_SHORT_SEQUENCE;
    usac_data->td_frame_prev[i_ch] = 1;
  }
  else
  {
    usac_data->window_shape_prev[i_ch] = WIN_SEL_0;
    usac_data->window_sequence_last[i_ch] = EIGHT_SHORT_SEQUENCE;
    usac_data->td_frame_prev[i_ch] = 1;

    memmove(usac_data->lpc_prev[i_ch], &tddec->lp_flt_coeff_a_prev[ORDER + 1],
            (ORDER + 1) * sizeof(FLOAT32));
    memmove(usac_data->acelp_in[i_ch], tddec->exc_prev, (1 + (2 * FAC_LENGTH)) * sizeof(FLOAT32));
  }

  return;
}

/**
 *  ia_core_coder_lpd_bpf_calc_pitch_gain
 *
 *  \brief Performs pitch gain calculation for bass post filtering in LPD
 *
 *  \param [out]	pitch_gain	pointer to pitch gain buffer
 *  \param [in]		pitch		pointer to pitch buffer
 *  \param [in]		synth		pointer to synth buffer
 *  \param [in]		num_sub_frm	Number of subframe / 2
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_lpd_bpf_calc_pitch_gain(FLOAT32 *pitch_gain, WORD32 *pitch,
                                                          FLOAT32 *synth, WORD32 num_sub_frm)
{
  WORD32 i, k, pitch_tmp;
  FLOAT32 synth_corr, synth_energy;
  for (i = 0; i < num_sub_frm + 2; i++)
  {
    pitch_tmp = pitch[i];
    if ((i * LEN_SUBFR + MAX_PITCH) < pitch_tmp)
    {
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PITCH;
    }
    else if (((i * LEN_SUBFR + MAX_PITCH - pitch_tmp) >= 1883) ||
             (((i * LEN_SUBFR) + LEN_SUBFR) > LEN_SUPERFRAME) ||
             ((((i * LEN_SUBFR) + LEN_SUBFR) - pitch_tmp) > LEN_SUPERFRAME))
    {
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PITCH;
    }

    if (ia_lt_flt(0.0f, pitch_gain[i]))
    {
      synth_corr = 0.0f, synth_energy = 1e-6f;
      for (k = 0; k < LEN_SUBFR; k++)
      {
        synth_corr = ia_add_flt(synth_corr, ia_mul_flt(synth[i * LEN_SUBFR + k],
                                                       synth[(i * LEN_SUBFR) - pitch_tmp + k]));
        synth_energy =
            ia_add_flt(synth_energy, ia_mul_flt(synth[(i * LEN_SUBFR) - pitch_tmp + k],
                                                synth[(i * LEN_SUBFR) - pitch_tmp + k]));
      }
      pitch_gain[i] = synth_corr / synth_energy;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_lpd_bpf
 *
 *  \brief Performs bass post filtering in LPD and updates buffer
 *
 *  \param [in]    usac_data    USAC data structure
 *  \param [in]    is_short_flag  Flag for short window or not
 *  \param [inout]  out_buffer    output buffer
 *  \param [in]    st        LPD handle structure
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_lpd_bpf(ia_usac_data_struct *usac_data, WORD32 is_short_flag,
                                   FLOAT32 *out_buffer, ia_usac_lpd_decoder_handle st)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 len_fr, lpd_sbf_len, num_subfr_by2, synth_delay;
  WORD32 pitch[NUM_SUBFR_SUPERFRAME_BY2 + 3];
  FLOAT32 synth_buf[MAX_PITCH + SYNTH_DELAY_LMAX + LEN_SUPERFRAME];
  FLOAT32 pitch_gain[NUM_SUBFR_SUPERFRAME_BY2 + 3];
  FLOAT32 *synth;
  memset(pitch, 0, sizeof(pitch));
  ia_core_coder_memset(synth_buf, (MAX_PITCH + SYNTH_DELAY_LMAX + LEN_SUPERFRAME));
  ia_core_coder_memset(pitch_gain, (NUM_SUBFR_SUPERFRAME_BY2 + 3));
  len_fr = usac_data->ccfl;
  lpd_sbf_len = (NUM_FRAMES * usac_data->num_subfrm) / 2;
  num_subfr_by2 = lpd_sbf_len - 1;
  synth_delay = num_subfr_by2 * LEN_SUBFR;

  ia_core_coder_mem_cpy(st->synth_prev, synth_buf, MAX_PITCH + synth_delay);
  ia_core_coder_mem_cpy(out_buffer, synth_buf + MAX_PITCH - (LEN_SUBFR),
                        synth_delay + len_fr + (LEN_SUBFR));

  memcpy(pitch, st->pitch_prev, num_subfr_by2 * sizeof(pitch[0]));
  ia_core_coder_mem_cpy(st->gain_prev, pitch_gain, num_subfr_by2);

  ia_core_coder_memset(&pitch_gain[num_subfr_by2], (lpd_sbf_len + 3 - num_subfr_by2));
  for (i = num_subfr_by2; i < lpd_sbf_len + 3; i++)
  {
    pitch[i] = 64;
  }
  if (st->mode_prev == 0)
  {
    if (is_short_flag)
    {
      pitch[num_subfr_by2] = pitch[num_subfr_by2 - 1];
      pitch_gain[num_subfr_by2] = pitch_gain[num_subfr_by2 - 1];
    }
    else
    {
      pitch[num_subfr_by2] = pitch[num_subfr_by2 + 1] = pitch[num_subfr_by2 - 1];
      pitch_gain[num_subfr_by2] = pitch_gain[num_subfr_by2 + 1] = pitch_gain[num_subfr_by2 - 1];
    }
  }

  synth = synth_buf + MAX_PITCH;

  err = ia_core_coder_lpd_bpf_calc_pitch_gain(pitch_gain, pitch, synth, num_subfr_by2);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }
  err = ia_core_coder_bass_post_filter(synth, NULL, 0, pitch, pitch_gain, out_buffer,
                                       (lpd_sbf_len + 2) * LEN_SUBFR + LEN_SUBFR,
                                       len_fr - (lpd_sbf_len + 4) * LEN_SUBFR, st->bpf_prev);
  return err;
}
/** @} */ /* End of CoreDecProc */