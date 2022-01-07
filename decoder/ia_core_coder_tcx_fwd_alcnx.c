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

#include <float.h>
#include <math.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_acelp_com.h"
#include <ia_core_coder_basic_ops32.h>
#include "ia_core_coder_basic_ops40.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_igf_dec.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

static FLOAT32 ia_core_coder_randomsign(UWORD32 *seed);

/**
 *  ia_core_coder_lpc_coeff_wt_apply
 *
 *  \brief LPC coeffs are weighted using gamma=0.92
 *
 *  \param [in]			a		LPC coeff
 *  \param [out]		ap		Wtd LPC coeff
 *
 *
 *
 */
VOID ia_core_coder_lpc_coeff_wt_apply(FLOAT32 *a, FLOAT32 *ap)
{
  FLOAT32 fac, mul_fac;
  WORD32 i;
  ap[0] = a[0];
  fac = 1.0f;
  mul_fac = 0.92f;
  for (i = 1; i < 17; i++)
  {
    fac = ia_mul_flt(fac, mul_fac);
    ap[i] = ia_mul_flt(fac, a[i]);
  }
  return;
}

/**
 *  ia_core_coder_low_fq_deemphasis
 *
 *  \brief Low freq deemphasis of spectral coeff(Adaptive)
 *
 *  \param [in,out]		x		Spectral coeff
 *  \param [in]			lg		Subframe length
 *  \param [in]			fb_lpd	FB LPD flag
 *  \param [in]			gains	Output gain of all blocks
 *  \param [in]			gain1	mdct gainL
 *  \param [in]			gain2	mdct gainR
 *
 *
 *
 */
static VOID ia_core_coder_low_fq_deemphasis(FLOAT32 *x, WORD32 lg, WORD32 fb_lpd, FLOAT32 *gains,
                                            FLOAT32 *gain1, FLOAT32 *gain2)
{
  WORD32 i, j;
  FLOAT32 max, factor, rm;

  if (0 == fb_lpd)
  {
    WORD32 lg_by_4 = lg / 4;

    max = 0.01f;
    for (i = 0; i < lg_by_4; i += 8)
    {
      rm = 0.01f;
      for (j = i; j < i + 8; j++)
        rm = ia_add_flt(rm, ia_mul_flt(x[j], x[j]));

      max = ia_max_flt(max, rm);
    }

    factor = 0.1f;
    for (i = 0; i < lg_by_4; i += 8)
    {
      rm = 0.01f;
      for (j = i; j < i + 8; j++)
        rm = ia_add_flt(rm, ia_mul_flt(x[j], x[j]));

      rm = (FLOAT32)ia_sqrt_flt(rm / max);
      factor = ia_max_flt(factor, rm);

      for (j = i; j < i + 8; j++)
        x[j] = ia_mul_flt(x[j], factor);

      gains[i / 8] = factor;
    }
  }
  else
  {
    const WORD32 alfeLength = lg >> 3;
    WORD32 alfeLenBy4 = alfeLength >> 2;
    FLOAT32 f = (FLOAT32)ia_sqrt_flt(gain1[0] * gain2[0]);
    FLOAT32 fMax = f, fMin = f;

    for (i = 0; i < alfeLenBy4; i++)
    {
      gains[i] = 1.0f;
    }

    for (i = 8; i > 0; i--)
    {
      f = (FLOAT32)ia_sqrt_flt(gain1[i] * gain2[i]);
      fMax = ia_max_flt(fMax, f);
      fMin = ia_min_flt(fMin, f);
    }

    f = ia_mul_flt(fMin, 32.0f);

    if (ia_lt_flt(fMax, f) && ia_lt_flt(FLT_MIN, f))
    {
      FLOAT32 fa = 1.0f;
      f = (FLOAT32)pow(fMax / f, 1.0f / ia_mul_flt(4.0f, (FLOAT32)alfeLength));

      for (i = alfeLength - 1; i >= 0; i--)
      {
        fa = ia_mul_flt(fa, f);
        x[i] = ia_mul_flt(x[i], fa);
        if ((i % 8) == 4)
          gains[i >> 3] = fa;
      }
    }
  }

  return;
}
/**
 *  ia_core_coder_tcx_fdp_decode
 *
 *  \brief	Modifies FDP quantization spec
 *
 *  \param [in]			st				lpd_decoder handle
 *  \param [in,out]		out_spec_curr	Spectral coeff
 *  \param [in]			tcx_quant		TCX quantized spectrum
 *  \param [in]			quant_gain_curr	Quantization gain
 *  \param [in]			i_gain			TCX gain
 *  \param [in]			lg				Subframe length
 *  \param [in]			max_lines		max number of spectral lines
 *  \param [in]			pred_bw			Prediction Bandwidth
 *
 *
 *
 */

static VOID ia_core_coder_tcx_fdp_decode(ia_usac_lpd_decoder_handle st, FLOAT32 *out_spec_curr,
                                         WORD32 *tcx_quant, WORD32 quant_gain_curr,
                                         FLOAT32 i_gain, WORD32 lg, WORD32 max_lines,
                                         WORD32 pred_bw)
{
  WORD32 i, s1, s2, x, reg32, fdp_spacing_value, harmonic_spacing;
  WORD32 harmonic_idx = -128;
  WORD32 compare_idx = 256;

  /* step 1 */
  fdp_spacing_value = (894 / 3) - st->fdp_spacing_index;
  harmonic_spacing = (894 * 512 + fdp_spacing_value) / (2 * fdp_spacing_value);

  /* step 2 */
  pred_bw = ia_min_int(lg, pred_bw);

  /* step 3*/
  s1 = 0;
  s2 = 0;
  st->fdp_quant_spec_prev[1][0] = 0;

  if (st->fdp_data_present)
  {
    for (i = 0; i < pred_bw; i++)
    {
      if (ia_abs_int(i * 256 - harmonic_idx) < 384)
      {
        reg32 = s1 * st->fdp_quant_spec_prev[0][i] + s2 * st->fdp_quant_spec_prev[1][i];
        st->fdp_int[i] = (((UWORD32)ia_abs_int(reg32) + 16384) >> 15);
        if (reg32 < 0)
        {
          st->fdp_int[i] *= -1;
        }
        out_spec_curr[i] =
            ia_add_flt(out_spec_curr[i], ia_mul_flt(i_gain, (FLOAT32)st->fdp_int[i]));
      }
      else
      {
        st->fdp_int[i] = 0;
      }
      if (i * 256 == compare_idx)
      {
        harmonic_idx += harmonic_spacing;
        compare_idx = harmonic_idx & 255;
        if (compare_idx > 128)
        {
          compare_idx = 256 - compare_idx;
        }

        s1 = ia_core_coder_fdp_s1[compare_idx];
        s2 = ia_core_coder_fdp_s2[compare_idx];

        compare_idx = harmonic_idx >> 8;
        if ((compare_idx & 1) == 0)
        {
          s1 *= -1;
        }
        compare_idx = 256 + ((harmonic_idx + 128) >> 8) * 256;
      }
    }
  }

  /* step 4 */
  if (st->fdp_data_present)
  {
    for (i = 0; i < pred_bw; i++)
    {
      x = tcx_quant[i] * quant_gain_curr + st->fdp_int[i];
      st->fdp_quant_spec_prev[1][i] = st->fdp_quant_spec_prev[0][i];
      st->fdp_quant_spec_prev[0][i] =
          (WORD16)ia_min_int(ia_max_int(x, QUANT_SPEC_MIN), QUANT_SPEC_MAX);
    }
  }
  else
  {
    for (i = 0; i < pred_bw; i++)
    {
      x = tcx_quant[i] * quant_gain_curr;
      st->fdp_quant_spec_prev[1][i] = st->fdp_quant_spec_prev[0][i];
      st->fdp_quant_spec_prev[0][i] =
          (WORD16)ia_min_int(ia_max_int(x, QUANT_SPEC_MIN), QUANT_SPEC_MAX);
    }
  }

  i = ia_min_int(i, max_lines);
  memset(&st->fdp_quant_spec_prev[0][i], 0, sizeof(WORD32) * (172 - i));
  memset(&st->fdp_quant_spec_prev[1][i], 0, sizeof(WORD32) * (172 - i));
}

/**
 *  ia_core_coder_get_noise_bw
 *
 *  \brief Calculate noise BW and returns the value
 *
 *  \param [in]			sfb_info	sfb info handle
 *  \param [in]			mode		variable dependent on frame length
 *  \param [in]			max_sfb		max sfb value
 *  \param [in]			len			frame length
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_get_noise_bw(ia_sfb_info_struct *sfb_info, WORD32 mode,
                                         WORD32 max_sfb, WORD32 len)
{
  const WORD16 *swb_offset = sfb_info->ptr_sfb_tbl;
  WORD32 swb_shift = mode << 2;
  WORD32 noise_bw;

  if (max_sfb == 0)
  {
    return 0;
  }
  if ((max_sfb < 0) || (max_sfb > sfb_info->sfb_per_sbk))
  {
    return len;
  }

  noise_bw = swb_offset[max_sfb - 1] * swb_shift;

  if (noise_bw > len)
  {
    return len;
  }
  else
  {
    return noise_bw;
  }
}

/**
 *  ia_core_coder_tns_lpd_synth_filter
 *
 *  \brief TNS LPD synthesis filter
 *
 *  \param [in]			spec		TNS spectrum
 *  \param [in]			size		Length, filtering done if length >0
 *  \param [in]			direction	Direction of operation
 *  \param [in]			lpc			LPC filter coeff
 *  \param [in]			order		TNS filter order
 *
 *
 *
 */
static VOID ia_core_coder_tns_lpd_synth_filter(FLOAT32 *spec, WORD32 size, WORD32 direction,
                                               FLOAT32 *lpc, WORD32 order)
{
  WORD32 i, j, inc;
  FLOAT32 y, state[8];

  if (0 == direction)
  {
    inc = 1;
  }
  else
  {
    inc = -1;
  }

  if (size > 0 && order <= 8)
  {
    ia_core_coder_memset(state, order);

    if (inc == -1)
    {
      spec += size - 1;
    }
    for (i = 0; i < size; i++)
    {
      y = *spec;
      for (j = 0; j < order; j++)
      {
        y = ia_sub_flt(y, ia_mul_flt(lpc[j + 1], state[j]));
      }
      for (j = order - 1; j > 0; j--)
      {
        state[j] = state[j - 1];
      }
      state[0] = *spec = y;
      spec += inc;
    }
  }
}

/**
 *  ia_core_coder_tns_lpd_prepare_lp_coeffs
 *
 *  \brief Prepare LP coeff
 *
 *  \param [in]			filter		TNS filter structure
 *  \param [in,out]		lpc			LPC filter coeff
 *  \param [in]			coef_res	Coeff residual
 *
 *
 *
 */
static VOID ia_core_coder_tns_lpd_prepare_lp_coeffs(ia_tns_filter_struct *filter, FLOAT32 *lpc,
                                                    WORD32 coef_res)
{
  FLOAT32 twoDivPi = 0.63661977f; // clean later
  FLOAT32 iqFac_n = 1.0f / ia_mul_flt(ia_add_flt((FLOAT32)(1 << (coef_res - 1)), 0.5f), twoDivPi);
  FLOAT32 iqFac_p = 1.0f / ia_mul_flt(ia_sub_flt((FLOAT32)(1 << (coef_res - 1)), 0.5f), twoDivPi);
  FLOAT32 tmp[8], b[8];
  WORD32 i, m;

  if (filter->order > 0)
  {
    for (i = 0; i < filter->order; i++)
    {
      if (filter->coef[i] < 0)
      {
        tmp[i + 1] = (FLOAT32)sin((double)filter->coef[i] * (double)(iqFac_n));
      }
      else
      {
        tmp[i + 1] = (FLOAT32)sin((double)filter->coef[i] * (double)(iqFac_p));
      }
    }
    lpc[0] = 1.0f;
    for (m = 1; m <= filter->order; m++)
    {
      b[0] = lpc[0];
      for (i = 1; i < m; i++)
      {
        b[i] = ia_add_flt(lpc[i], ia_mul_flt(tmp[m], lpc[m - i]));
      }
      b[m] = tmp[m];

      ia_core_coder_mem_cpy(b, lpc, m + 1);
    }
  }
}

/**
 *  ia_core_coder_tns_dec_lpd
 *
 *  \brief Performs tns lpd decoding involving tns lpd synthesis
 *
 *  \param [in]			sfb_info		sfb info structure
 *  \param [in]			tns_info		tns info structure
 *  \param [in]			sub_frame_idx	frame index in a super frame
 *  \param [in]			sub_frame_cnt	Number of subframes
 *  \param [in]			mode			variable dependent on frame length
 *  \param [in,out]		spec_coef		Spectral coeff vector
 *  \param [in]			len				length
 *
 *
 *
 */
static VOID ia_core_coder_tns_dec_lpd(ia_sfb_info_struct *sfb_info,
                                      ia_tns_frame_info_struct *tns_info, WORD32 sub_frame_idx,
                                      WORD32 sub_frame_cnt, WORD32 mode, FLOAT32 *spec_coef,
                                      WORD32 len)
{

  ia_tns_info_struct *tns = &tns_info->str_tns_info[sub_frame_idx];
  ia_tns_filter_struct *tns_filter = tns_info->str_tns_info[sub_frame_idx].str_filter;
  FLOAT32 filter_coef[8];
  WORD32 filter_start;
  WORD32 filter_stop;
  WORD32 filter_size;

  if (!tns->n_filt || (tns_filter->stop_band > 15) || (tns_filter->order <= 0))
  {
    return;
  }

  ia_core_coder_tns_lpd_prepare_lp_coeffs(tns_filter, filter_coef, tns->coef_res);

  filter_start = sfb_info->sfb_idx_tbl[tns_filter->start_band - 1] * (mode << 2);
  filter_stop = ia_min_int(len, sub_frame_cnt * 256);
  filter_size = filter_stop - filter_start;

  if (filter_start < 0)
  {
    return;
  }

  ia_core_coder_tns_lpd_synth_filter(spec_coef + filter_start, filter_size, tns_filter->direction,
                                     filter_coef, tns_filter->order);
}

/**
 *  ia_core_coder_tcx_igf_inf_spec
 *
 *  \brief Populate IGF spectrucm coefficients
 *
 *  \param [in]			usac_data			USAC data structure
 *  \param [in]			ptr_igf_config		IGF Configuration structure
 *  \param [in]			igf_win_type		IGF Window type
 *  \param [in]			igf_bgn				IGF
 *  \param [in]			noise_level_gain	Noise level gain
 *  \param [in]			igf_inf_mask		IGF Mask array
 *  \param [in]			coef				Coefficients
 *
 *
 *
 */
static VOID ia_core_coder_tcx_igf_inf_spec(ia_usac_data_struct *usac_data,
                                           ia_usac_igf_config_struct *ptr_igf_config,
                                           WORD32 igf_win_type, WORD32 igf_bgn,
                                           FLOAT32 noise_level_gain, WORD32 *igf_inf_mask,
                                           FLOAT32 *coef)
{
  WORD32 left = usac_data->present_chan;
  WORD32 igf_cnt, i;
  FLOAT32 *igf_inf_spec_float = usac_data->igf_dec_data[left].igf_input_spec;

  for (i = 0; i < igf_bgn; i++)
  {
    for (igf_cnt = 0; igf_cnt < ptr_igf_config->igf_grid[igf_win_type].igf_num_tiles; igf_cnt++)
    {
      if (ptr_igf_config->use_inf && igf_inf_mask[i] == 0.0f)
      {
        igf_inf_spec_float[igf_cnt * MAX_IGF_LEN + i] =
            ia_mul_flt(noise_level_gain,
                       ia_core_coder_igf_get_random_sign(
                           &usac_data->seed_value[left])); // (&(st->seed_tcx));
      }
      else
      {
        igf_inf_spec_float[igf_cnt * MAX_IGF_LEN + i] = coef[i];
      }
    }
  }
}

/**
 *  ia_core_coder_tcx_mdct
 *
 *  \brief Modifies synthesis buffer and LPC coeff by tcx decoding
 *
 *  \param [in]			usac_data			USAC data structure
 *  \param [in]			pstr_td_frame_data	USAC td frame data structure
 *  \param [in]			frame_index			Index of subframe
 *  \param [in,out]		lp_flt_coff_a		LPC filter coeff
 *  \param [in,out]		synth_fb			Fullband Synthesis buffer
 *  \param [in]			lg					Subframe length
 *  \param [in]			st					Lpd_decoder structure
 *  \param [in]			elem_idx				Element index
 *  \param [in]			tmp_scratch			Scratch memory
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_tcx_mdct(ia_usac_data_struct *usac_data,
                                    ia_td_frame_data_struct *pstr_td_frame_data,
                                    WORD32 frame_index, FLOAT32 *lp_flt_coff_a, FLOAT32 *synth_fb,
                                    WORD32 lg, ia_usac_lpd_decoder_handle st, WORD32 elem_idx,
                                    FLOAT32 *tmp_scratch)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, mode;
  WORD32 fac_length_prev;
  WORD32 len_res;

  WORD32 loop_count = 0;
  WORD32 fac_fb = 0, fac_length_fb = 0, index, win_index;
  WORD32 len_frame_fb = lg;
  WORD32 noise_bandwidth;
  WORD32 nf_bgn, nf_end, igf_bgn = 0;
  WORD32 igf_win_type = 0;
  WORD32 igf_frm_id = 0;
  WORD32 igf_sfb_start;
  WORD32 left = usac_data->present_chan;
  WORD32 fac_length = (usac_data->len_subfrm) / 2;
  WORD32 pred_bw;

  FLOAT32 tmp, gain_tcx, noise_level, energy, temp, energy_sqrt;

  WORD32 *ptr_tcx_quant;

  FLOAT32 *ptr_a;
  const FLOAT32 *sine_window_prev, *sine_window;
  const FLOAT32 *sine_window_prev_fb = NULL, *sine_window_fb = NULL;
  FLOAT32 *xn;
  FLOAT32 *ptr_scratch = &usac_data->scratch_buffer_float[0];
  FLOAT32 *synth;
  FLOAT32 *exc =
      &usac_data
           ->exc_buf[usac_data->len_subfrm * frame_index + MAX_PITCH + INTER_LP_FIL_ORDER + 1];

  ia_usac_td_config_handle td_config = &usac_data->td_config[elem_idx];
  ia_usac_igf_config_struct *ptr_igf_config = &usac_data->igf_config[elem_idx];
  ia_core_coder_tcx_mdct_scratch_t *pscr = (ia_core_coder_tcx_mdct_scratch_t *)tmp_scratch;

  synth = &usac_data->synth_buf[td_config->len_subfrm * frame_index + MAX_PITCH +
                                (((NUM_FRAMES * td_config->num_frame) / 2) - 1) * LEN_SUBFR];

  FLOAT32 *fft_scratch = tmp_scratch + (sizeof(ia_core_coder_tcx_mdct_scratch_t) / 4);

  mode = ia_min_int(3, lg / (usac_data->len_subfrm));

  if (td_config->full_band_lpd)
  {
    fac_fb = td_config->fscale_full_band / td_config->fscale;
    fac_length_fb = fac_length * fac_fb;
    len_frame_fb = lg * fac_fb;

    switch (st->mode_prev)
    {
    case -1:
      fac_length_prev = fac_length >> 1;
      win_index = 2;
      break;
    case -2:
      fac_length_prev = fac_length >> 2;
      win_index = 0;
      break;
    default:
      fac_length_prev = fac_length;
      win_index = 4;
      break;
    }
  }
  else
  {
    switch (st->mode_prev)
    {
    case -2:
      fac_length_prev = (usac_data->ccfl) >> 4;
      win_index = 2;
      break;
    default:
      fac_length_prev = fac_length;
      win_index = 4;
      break;
    }
  }

  if (td_config->igf_active)
  {
    igf_win_type = (WORD32)(mode + 1);
    igf_sfb_start = ptr_igf_config->igf_grid[igf_win_type].igf_sfb_start;
    if (3 == igf_win_type)
    {
      igf_frm_id = 4;
    }
    else
    {
      igf_frm_id = (WORD32)(mode + 1 + frame_index);
    }

    ia_sfb_info_struct *pstr_usac_winmap = usac_data->pstr_usac_winmap[EIGHT_SHORT_SEQUENCE];
    ia_core_coder_igf_get_swb_offset(igf_win_type, pscr->swb_offset, pstr_usac_winmap);
    igf_bgn = pscr->swb_offset[igf_sfb_start];

    for (i = 0; i < LEN_SUPERFRAME; i++)
    {
      pscr->igf_inf_mask[i] = 1;
    }
  }
  {
    const FLOAT32 *win_total[2][5] = {
        {ia_core_coder_sine_window64, ia_core_coder_sine_window96, ia_core_coder_sine_window128,
         ia_core_coder_sine_window192, ia_core_coder_sine_window256},
        {ia_core_coder_kbd_window64, ia_core_coder_kbd_window96, ia_core_coder_kbd_window128,
         ia_core_coder_kbd_window192, ia_core_coder_kbd_window256}};

    const FLOAT32 **win_used_prev = win_total[st->window_shape_prev & 1];
    const FLOAT32 **win_used_curr = win_total[st->window_shape & 1];
    sine_window_prev = win_used_prev[win_index];
    sine_window = win_used_curr[4];
    if (td_config->full_band_lpd)
    {
      const FLOAT32 *win_total_fb[2][5] = {
          {ia_core_coder_sine_window128, ia_core_coder_sine_window192,
           ia_core_coder_sine_window256, ia_core_coder_sine_window384,
           ia_core_coder_sine_window512},
          {ia_core_coder_kbd_window128, ia_core_coder_kbd_window192, ia_core_coder_kbd_window256,
           ia_core_coder_kbd_window384, ia_core_coder_kbd_window512}};
      const FLOAT32 **win_used_prev_fb = win_total_fb[st->window_shape_prev & 1];
      const FLOAT32 **win_used_curr_fb = win_total_fb[st->window_shape & 1];
      sine_window_prev_fb = win_used_prev_fb[win_index];
      sine_window_fb = win_used_curr_fb[4];
    }

    st->window_shape_prev = st->window_shape;
  }

  xn = pscr->xn_buf + fac_length;

  if (st->mode_prev != 0)
  {
    if (st->mode_prev > 0)
    {
      for (i = 0; i < fac_length_prev; i++)
      {
        st->exc_prev[i + fac_length - fac_length_prev + 1] =
            ia_mul_flt(st->exc_prev[i + fac_length - fac_length_prev + 1],
                       sine_window_prev[(2 * fac_length_prev) - 1 - i]);
        st->exc_prev[i + fac_length + 1] = ia_mul_flt(st->exc_prev[i + fac_length + 1],
                                                      sine_window_prev[fac_length_prev - 1 - i]);
      }
      if (td_config->full_band_lpd)
      {
        for (i = 0; i < (fac_length_prev * fac_fb); i++)
        {
          st->exc_prev_fb[i + fac_length_fb - fac_length_prev * fac_fb] =
              ia_mul_flt(st->exc_prev_fb[i + fac_length_fb - fac_length_prev * fac_fb],
                         sine_window_prev_fb[(2 * fac_length_prev * fac_fb) - 1 - i]);
          st->exc_prev_fb[i + fac_length_fb] =
              ia_mul_flt(st->exc_prev_fb[i + fac_length_fb],
                         sine_window_prev_fb[(fac_length_prev * fac_fb) - 1 - i]);
        }
      }
    }

    ia_core_coder_memset(&st->exc_prev[fac_length + fac_length_prev + 1],
                         fac_length - fac_length_prev);

    if (td_config->full_band_lpd)
    {
      ia_core_coder_memset(&st->exc_prev_fb[fac_length_fb + fac_length_prev * fac_fb],
                           fac_length_fb - fac_length_prev * fac_fb);
    }
  }

  noise_level = ia_mul_flt(
      0.0625f, ia_sub_flt(8.0f, ((FLOAT32)pstr_td_frame_data->noise_factor[frame_index])));

  ptr_tcx_quant = pstr_td_frame_data->x_tcx_invquant;
  for (i = 0; i < frame_index; i++)
    ptr_tcx_quant += pstr_td_frame_data->tcx_lg[i];

  for (i = 0; i < len_frame_fb; i++)
    pscr->x[i] = (FLOAT32)ptr_tcx_quant[i];
  if (td_config->full_band_lpd)
  {
    noise_bandwidth = ia_core_coder_get_noise_bw(
        usac_data->pstr_usac_winmap[EIGHT_SHORT_SEQUENCE], mode, st->max_sfb, len_frame_fb);
    nf_bgn = (len_frame_fb / 6) & 2040;
    if (td_config->igf_active)
    {
      nf_end = ia_min_int(igf_bgn, ia_min_int(len_frame_fb, noise_bandwidth));
    }
    else
    {
      nf_end = ia_min_int(len_frame_fb, noise_bandwidth);
    }
  }
  else
  {
    nf_bgn = (lg / 6);
    nf_end = lg;
  }
  if (td_config->len_subfrm == 192)
  {
    pred_bw = 120;
  }
  else
  {
    pred_bw = 160;
  }
  for (i = nf_bgn; i < nf_end; i += 8)
  {
    WORD32 k, max_k = ia_min_int(nf_end, i + 8);
    tmp = 0.0f;
    for (k = i; k < max_k; k++)
      tmp = ia_add_flt(tmp, ia_mul_flt((FLOAT32)ptr_tcx_quant[k], (FLOAT32)ptr_tcx_quant[k]));

    if (ia_eq_flt(tmp, 0.0f))
    {
      for (k = i; k < max_k; k++)
      {
        pscr->x[k] = ia_mul_flt(
            noise_level,
            ia_core_coder_randomsign(&(usac_data->seed_value[usac_data->present_chan])));
      }
      if (td_config->igf_active)
      {
        memset(&pscr->igf_inf_mask[i], 0, (max_k - i) * sizeof(WORD32));
      }
    }
  }
  index = pstr_td_frame_data->global_gain[frame_index];
  energy = 0.01f;
  for (i = 0; i < lg; i++)
    energy = ia_add_flt(energy, ia_mul_flt(pscr->x[i], pscr->x[i]));

  energy_sqrt = (FLOAT32)ia_sqrt_flt(energy);

  if (mode == (td_config->full_band_lpd ? 2 : 3))
  {
    WORD32 quant_gain_curr = 0;
    FLOAT32 x_float_sq_ac = 0;
    for (i = 0; i < lg; i++)
    {
      x_float_sq_ac = ia_mac_flt(x_float_sq_ac, pscr->x[i], pscr->x[i]);
    }
    if (x_float_sq_ac >= (FLOAT32)Q23)
    {
      quant_gain_curr = MAX_32;
    }
    else
    {
      quant_gain_curr = (WORD32)ia_mul_flt(x_float_sq_ac, Q8);
    }

    tmp = (FLOAT32)ia_mul_flt(energy_sqrt, 64.f);
    gain_tcx = tmp / ia_mul_flt(ia_mul_flt(0.5f, (FLOAT32)pow(10.0f, (FLOAT32)index / 28.0f)),
                                (FLOAT32)lg);
    quant_gain_curr =
        (WORD32)ia_core_coder_rounded_sqrt64((WORD64)41 + 16 * (WORD64)quant_gain_curr);

    if (td_config->full_band_lpd)
    {
      quant_gain_curr =
          (WORD32)(ia_add_flt(0.5f, ia_mul_flt((FLOAT32)quant_gain_curr, 1024.0f)) / (FLOAT32)lg);
    }

    quant_gain_curr =
        (ia_core_coder_pow10_gain_div28[index] + quant_gain_curr) / (2 * quant_gain_curr);

    if (usac_data->usac_independency_flg)
    {
      memset(st->fdp_quant_spec_prev[0], 0, 172 * sizeof(WORD32));
      memset(st->fdp_quant_spec_prev[1], 0, 172 * sizeof(WORD32));
    }

    ia_core_coder_tcx_fdp_decode(st, pscr->x, ptr_tcx_quant, quant_gain_curr, gain_tcx, lg,
                                 pred_bw, pred_bw);
  }

  ia_core_coder_lpc_coeff_wt_apply(lp_flt_coff_a + (ORDER + 1), pscr->i_ap);
  err = ia_core_coder_lpc_to_mdct(pscr->i_ap, ORDER, pscr->gain1, usac_data->len_subfrm / 4,
                                  fft_scratch);
  if (err)
    return err;

  ia_core_coder_lpc_coeff_wt_apply(lp_flt_coff_a + (2 * (ORDER + 1)), pscr->i_ap);

  err = ia_core_coder_lpc_to_mdct(pscr->i_ap, ORDER, pscr->gain2, usac_data->len_subfrm / 4,
                                  fft_scratch);
  if (err)
    return err;

  ia_core_coder_low_fq_deemphasis(pscr->x, lg, td_config->full_band_lpd, pscr->alfd_gains,
                                  pscr->gain1, pscr->gain2);

  if (!td_config->full_band_lpd)
  {
    energy = 0.01f;
    for (i = 0; i < lg; i++)
      energy = ia_add_flt(energy, ia_mul_flt(pscr->x[i], pscr->x[i]));
  }
  energy_sqrt = (FLOAT32)ia_sqrt_flt(energy);
  temp = 2.0f * energy_sqrt / lg;

  gain_tcx =
      (FLOAT32)pow(10.0f, ((FLOAT32)pstr_td_frame_data->global_gain[frame_index]) / 28.0f) /
      (temp);
  if (td_config->igf_active)
  {
    const WORD16 group_len[MAX_SBK] = {1, 0, 0, 0, 0, 0, 0, 0};
    const FLOAT32 noise_level_gain = noise_level * gain_tcx;
    FLOAT32 gain;
    gain = ia_mul_flt(ia_mul_flt(gain_tcx, 0.5f),
                      (FLOAT32)ia_sqrt_flt(((FLOAT32)fac_length) /
                                           ia_mul_flt(2, (FLOAT32)td_config->len_subfrm)));
    st->fac_gain = gain;

    ia_core_coder_vec_cnst_mul(gain_tcx, pscr->x, pscr->x, ia_min_int(len_frame_fb, igf_bgn));

    ia_core_coder_tcx_igf_inf_spec(usac_data, ptr_igf_config, igf_win_type, igf_bgn,
                                   noise_level_gain, pscr->igf_inf_mask, pscr->x);

    ia_core_coder_igf_mono(usac_data, ptr_igf_config, left, igf_win_type, igf_frm_id, 1,
                           (WORD16 *)group_len, lg, EIGHT_SHORT_SEQUENCE, pscr->x, fft_scratch);
    ia_core_coder_igf_tnf(usac_data, ptr_igf_config, left, igf_frm_id, igf_win_type,
                          EIGHT_SHORT_SEQUENCE, pscr->x, fft_scratch);
  }

  ia_core_coder_noise_shaping(pscr->x, lg, (usac_data->len_subfrm) / 4, pscr->gain1, pscr->gain2,
                              fft_scratch);

  ia_core_coder_noise_shaping(
      pscr->x + lg, len_frame_fb - lg, 1,
      &pscr->gain1[FD_NOISE_SHAPING_RES * usac_data->len_subfrm / LEN_FRAME - 1],
      &pscr->gain2[FD_NOISE_SHAPING_RES * usac_data->len_subfrm / LEN_FRAME - 1], fft_scratch);

  noise_bandwidth = ia_core_coder_get_noise_bw(usac_data->pstr_usac_winmap[EIGHT_SHORT_SEQUENCE],
                                               mode, st->max_sfb, len_frame_fb);

  // q factor for x is 13
  ia_core_coder_tns_dec_lpd(usac_data->pstr_usac_winmap[EIGHT_SHORT_SEQUENCE],
                            &st->str_tns_info_td, frame_index,
                            len_frame_fb / td_config->len_subfrm, mode, pscr->x, noise_bandwidth);

  if (td_config->full_band_lpd)
  {
    err = ia_core_coder_acelp_mdct_main(usac_data, pscr->x, pscr->xn_buf_fb, (2 * fac_length_fb),
                                        len_frame_fb - (2 * fac_length_fb), fft_scratch);
    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }

    ia_core_coder_vec_cnst_mul((2.0f / lg), pscr->xn_buf_fb, pscr->xn_buf_fb,
                               len_frame_fb + (2 * fac_length_fb));
  }

  err = ia_core_coder_acelp_mdct_main(usac_data, pscr->x, pscr->xn_buf, (2 * fac_length),
                                      lg - (2 * fac_length), fft_scratch);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }

  ia_core_coder_vec_cnst_mul((2.0f / lg), pscr->xn_buf, pscr->xn_buf, lg + (2 * fac_length));

  if (!td_config->igf_active)
  {
    st->fac_gain = ia_mul_flt(ia_mul_flt(gain_tcx, 0.5f),
                              (FLOAT32)ia_sqrt_flt(((FLOAT32)fac_length) / (FLOAT32)lg));
  }
  for (i = 0; i < fac_length / 4; i++)
    st->fac_fd_data[i] = pscr->alfd_gains[i * lg / (8 * fac_length)];

  if (st->mode_prev == 0)
  {
    for (i = 0; i < fac_length_prev; i++)
    {
      pscr->facwindow[i] =
          ia_mul_flt(sine_window_prev[i], sine_window_prev[(2 * fac_length_prev) - 1 - i]);
      pscr->facwindow[fac_length_prev + i] =
          ia_sub_flt(1.0f, ia_mul_flt(sine_window_prev[fac_length_prev + i],
                                      sine_window_prev[fac_length_prev + i]));
    }

    WORD32 *ptr_fac_fix = &pstr_td_frame_data->fac[frame_index * FAC_LENGTH];

    for (i = 0; i < fac_length / 4; i++)
    {
      pscr->x[i] =
          ia_mul_flt(st->fac_fd_data[i], ia_mul_flt((FLOAT32)ptr_fac_fix[i], st->fac_gain));
    }

    for (; i < fac_length; i++)
    {
      pscr->x[i] = ia_mul_flt((FLOAT32)ptr_fac_fix[i], st->fac_gain);
    }

    err = ia_core_coder_acelp_mdct(pscr->x, pscr->xn1, fac_length, ptr_scratch, fft_scratch);
    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }

    ia_core_coder_vec_cnst_mul((2.0f / (FLOAT32)fac_length), pscr->xn1, pscr->xn1, fac_length);

    ia_core_coder_memset(pscr->xn1 + fac_length, fac_length); // ORDER

    ia_core_coder_lpc_coeff_wt_apply(lp_flt_coff_a + (ORDER + 1), pscr->i_ap);

    ia_core_coder_synthesis_tool(pscr->i_ap, pscr->xn1, pscr->xn1, 2 * fac_length,
                                 pscr->xn1 + fac_length, fft_scratch);

    for (i = 0; i < fac_length; i++)
    {
      pscr->xn1[i] = ia_add_flt(
          pscr->xn1[i],
          ia_add_flt(
              ia_mul_flt(st->exc_prev[1 + fac_length + i], pscr->facwindow[fac_length + i]),
              ia_mul_flt(st->exc_prev[fac_length - i], pscr->facwindow[fac_length - 1 - i])));
    }

    if (td_config->full_band_lpd)
    {
      ia_core_coder_td_resampler(pscr->xn1, (WORD16)(2 * fac_length), td_config->fscale,
                                 pscr->xn1_fb, td_config->fscale_full_band, NULL, 0, fft_scratch);
    }
  }

  if (td_config->full_band_lpd)
  {
    if (!td_config->igf_active)
    {
      for (i = 0; i < len_frame_fb + (2 * fac_length_fb); i++)
      {
        pscr->xn_buf_fb[i] = ia_mul_flt(pscr->xn_buf_fb[i], gain_tcx);
      }
    }
    for (i = 0; i < (2 * fac_length_prev * fac_fb); i++)
    {
      pscr->xn_buf_fb[i + fac_length_fb - fac_length_prev * fac_fb] = ia_mul_flt(
          pscr->xn_buf_fb[i + fac_length_fb - fac_length_prev * fac_fb], sine_window_prev_fb[i]);
    }

    ia_core_coder_memset(pscr->xn_buf_fb, (fac_length_fb - fac_length_prev * fac_fb));

    if (st->mode_prev == 0)
    {
      for (i = fac_length_fb - fac_length_prev * fac_fb;
           i < (fac_length_fb + fac_length_prev * fac_fb); i++)
      {
        pscr->xn_buf_fb[i + fac_length_fb] =
            ia_add_flt(pscr->xn_buf_fb[i + fac_length_fb], pscr->xn1_fb[i]);
      }
    }
    else
    {
      for (i = fac_length_fb - fac_length_prev * fac_fb;
           i < (fac_length_fb + fac_length_prev * fac_fb); i++)
      {
        pscr->xn_buf_fb[i] = ia_add_flt(pscr->xn_buf_fb[i], st->exc_prev_fb[i]);
      }
    }

    ia_core_coder_mem_cpy(pscr->xn_buf_fb + len_frame_fb, st->exc_prev_fb, (2 * fac_length_fb));

    for (i = 0; i < (2 * fac_length_fb); i++)
    {
      pscr->xn_buf_fb[i + len_frame_fb] = ia_mul_flt(pscr->xn_buf_fb[i + len_frame_fb],
                                                     sine_window_fb[(2 * fac_length_fb) - 1 - i]);
    }

    ia_core_coder_mem_cpy(pscr->xn_buf_fb + fac_length_fb - fac_length_prev * fac_fb,
                          synth_fb - fac_length_prev * fac_fb,
                          fac_length_prev * fac_fb + len_frame_fb);
  }

  if (!td_config->igf_active)
  {
    for (i = 0; i < lg + (2 * fac_length); i++)
      pscr->xn_buf[i] = ia_mul_flt(pscr->xn_buf[i], gain_tcx);
  }

  for (i = 0; i < (2 * fac_length_prev); i++)
    pscr->xn_buf[i + fac_length - fac_length_prev] =
        ia_mul_flt(pscr->xn_buf[i + fac_length - fac_length_prev], sine_window_prev[i]);

  ia_core_coder_memset(pscr->xn_buf, (fac_length - fac_length_prev));

  if (st->mode_prev == 0)
  {
    for (i = fac_length - fac_length_prev; i < (fac_length + fac_length_prev); i++)
      pscr->xn_buf[i + fac_length] = ia_add_flt(pscr->xn_buf[i + fac_length], pscr->xn1[i]);
  }
  else
  {
    for (i = fac_length - fac_length_prev; i < (fac_length + fac_length_prev); i++)
      pscr->xn_buf[i] = ia_add_flt(pscr->xn_buf[i], st->exc_prev[1 + i]);
  }

  ia_core_coder_mem_cpy(pscr->xn_buf + lg - 1, st->exc_prev, 1 + (2 * fac_length));

  for (i = 0; i < (2 * fac_length); i++)
  {
    pscr->xn_buf[i + lg] =
        ia_mul_flt(pscr->xn_buf[i + lg], sine_window[(2 * fac_length) - 1 - i]);
  }

  if (st->mode_prev != 0)
  {
    ia_core_coder_mem_cpy(pscr->xn_buf + fac_length - fac_length_prev, synth - fac_length_prev,
                          fac_length_prev);

    for (i = 0; i < ORDER + fac_length; i++)
      pscr->buf[i] = ia_sub_flt(synth[i - ORDER - fac_length],
                                ia_mul_flt(PREEMPH_FILT_FAC, synth[i - ORDER - fac_length - 1]));

    ptr_a = st->lp_flt_coeff_a_prev;
    len_res = fac_length % LEN_SUBFR;
    if (len_res != 0)
    {
      ia_core_coder_residual_tool(ptr_a, &pscr->buf[ORDER], &exc[-fac_length], len_res, 1,
                                  ORDER + 1);
    }
    loop_count = (fac_length - len_res) / LEN_SUBFR;
    ia_core_coder_residual_tool(ptr_a, &pscr->buf[ORDER + len_res], &exc[len_res - fac_length],
                                LEN_SUBFR, loop_count, ORDER + 1);
  }

  ia_core_coder_mem_cpy(xn, synth, lg);

  ia_core_coder_mem_cpy(synth - ORDER - 1, xn - ORDER - 1, ORDER + 1);
  tmp = xn[-ORDER - 1];
  ia_core_coder_preemphsis_tool(xn - ORDER, PREEMPH_FILT_FAC, ORDER + lg, tmp);

  ptr_a = lp_flt_coff_a + (2 * (ORDER + 1));

  ia_core_coder_residual_tool(ptr_a, xn, exc, lg, 1, 17);
  ia_core_coder_mem_cpy(ptr_a, st->lp_flt_coeff_a_prev, ORDER + 1);
  ia_core_coder_mem_cpy(ptr_a, st->lp_flt_coeff_a_prev + ORDER + 1, ORDER + 1);

  return err;
}
/**
 *  ia_core_coder_randomsign
 *
 *  \brief Calculates sign based on seed and returns the sign value
 *
 *  \param [in]	seed
 *
 *  \return FLOAT32
 *
 */
static FLOAT32 ia_core_coder_randomsign(UWORD32 *seed)
{
  FLOAT32 sign = 0.0f;
  *seed = (UWORD32)(((UWORD64)(*seed) * (UWORD64)69069) + 5);

  if (((*seed) & 0x10000) > 0)
    sign = -1.f;
  else
    sign = +1.f;

  return sign;
}
/** @} */ /* End of CoreDecProc */