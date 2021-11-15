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
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_acelp_com.h"
#include <ia_core_coder_basic_ops32.h>
#include <ia_core_coder_basic_ops40.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_function_selector.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_vec_baisc_ops.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_calc_pre_twid_dec
 *
 *  \brief Calculate pre twiddle in FFT processing
 *  \param [in]    ptr_x          data buffer
 *  \param [in]    r_ptr          real data buffer
 *  \param [in]    i_ptr          imag data buffer
 *  \param [in]    nlength          FFT length
 *  \param [in]    transform_kernel_type  kernel type
 *  \param [in]    cos_ptr          pointer to cos table
 *  \param [in]    sin_ptr          pointer to sin table
*
 *  \return VOID
 *
 */
VOID ia_core_coder_calc_pre_twid_dec(FLOAT32 *ptr_x, FLOAT32 *r_ptr, FLOAT32 *i_ptr,
                                     WORD32 nlength, WORD16 transform_kernel_type,
                                     const FLOAT32 *cos_ptr, const FLOAT32 *sin_ptr)
{
  WORD32 i;
  FLOAT32 *ptr_y, tmp = 0.0f;

  WORD32 size = nlength * 2;

  switch (transform_kernel_type)
  {
  case 1:
  case 2:
  { // DCT-II //DST-II
    FLOAT32 re1, re2, im1, im2;

    const FLOAT32 sin_pi_by_4 = (FLOAT32)sin(ia_mul_double_flt(PI, 0.25f));
    const FLOAT32 point5 = 0.5;

    /* reorder input */
    if (transform_kernel_type == 1)
    {
      FLOAT32 norm;
      for (i = 0; i < nlength; i++)
      {
        norm = ptr_x[i]; /* reuse MDCT: spectrally reverse all bins */
        ptr_x[i] = ptr_x[size - 1 - i];
        ptr_x[size - 1 - i] = norm;
      }
    }

    r_ptr[0] = ia_mac_flt(ptr_x[0], ptr_x[nlength], sin_pi_by_4);
    i_ptr[0] = ia_msu_flt(ptr_x[0], ptr_x[nlength], sin_pi_by_4);

    for (i = 1; i <= nlength / 2; i++)
    {
      /* pre-modulation */
      re1 = ia_mul_flt(
          ia_mac_flt(ia_mul_flt(ptr_x[i], cos_ptr[i - 1]), ptr_x[size - i], sin_ptr[i - 1]),
          point5);
      re2 = ia_mul_flt(ia_mac_flt(ia_mul_flt(ptr_x[nlength - i], sin_ptr[nlength + (i - 1)]),
                                  ptr_x[nlength + i], cos_ptr[nlength + (i - 1)]),
                       point5);
      im1 = ia_mul_flt(
          ia_msu_flt(ia_mul_flt(ptr_x[i], sin_ptr[i - 1]), ptr_x[size - i], cos_ptr[i - 1]),
          point5);
      im2 = ia_mul_flt(ia_msu_flt(ia_mul_flt(ptr_x[nlength - i], cos_ptr[nlength + (i - 1)]),
                                  ptr_x[nlength + i], sin_ptr[nlength + (i - 1)]),
                       point5);

      tmp = ia_sub_flt(im1, im2);
      i_ptr[i] = ia_msu_flt(ia_mul_flt(ia_sub_flt(re1, re2), cos_ptr[(4 * i) - 1]),
                            ia_add_flt(im1, im2), sin_ptr[(4 * i) - 1]);
      i_ptr[nlength - i] = ia_sub_flt(i_ptr[i], tmp);
      i_ptr[i] = ia_add_flt(i_ptr[i], tmp);

      tmp = ia_add_flt(re1, re2);
      r_ptr[i] = ia_mac_flt(ia_mul_flt(ia_add_flt(im1, im2), cos_ptr[(4 * i) - 1]),
                            ia_sub_flt(re1, re2), sin_ptr[(4 * i) - 1]);
      r_ptr[nlength - i] = ia_add_flt(tmp, r_ptr[i]);
      r_ptr[i] = ia_sub_flt(tmp, r_ptr[i]);
    }
    r_ptr[nlength / 2] = tmp - r_ptr[nlength / 2]; /* add missing contribution of tmp */
  }
  break;
  case 3:
  { // DST-IV
    /* reorder input */
    for (i = 0; i < size / 4; i++)
    {
      tmp = ptr_x[2 * i + 1];
      ptr_x[2 * i + 1] = ia_negate_flt(ptr_x[size - 1 - 2 * i]);
      ptr_x[size - 1 - 2 * i] = ia_negate_flt(tmp);
    }

    /* pre-modulation */
    for (i = 0; i < size / 2; i++)
    {
      tmp = ia_msu_flt(ia_mul_flt(ptr_x[i * 2 + 1], cos_ptr[i]), ptr_x[i * 2], sin_ptr[i]);
      i_ptr[i] = ia_mac_flt(ia_mul_flt(ptr_x[i * 2], cos_ptr[i]), ptr_x[i * 2 + 1], sin_ptr[i]);
      r_ptr[i] = tmp;
    }
  }
  break;
  default:
  {
    ptr_y = &ptr_x[2 * nlength - 1];
    for (i = 0; i < nlength; i++)
    {
      *r_ptr++ =
          (ia_msu_flt(ia_negate_flt(ia_mul_flt((*ptr_x), (*cos_ptr))), (*ptr_y), (*sin_ptr)));
      *i_ptr++ = (ia_msu_flt(ia_mul_flt((*ptr_y), (*cos_ptr++)), (*ptr_x), (*sin_ptr++)));
      ptr_x += 2;
      ptr_y -= 2;
    }
  }
  break;
  }
}

/**
 *  ia_core_coder_calc_post_twid_dec
 *
 *  \brief Calculate post twiddle in FFT processing
 *
 *  \param [in/out]    xptr          data buffer
 *  \param [in]      r_ptr          real data buffer
 *  \param [in]      i_ptr          imag data buffer
 *  \param [in]      nlength          FFT length
 *  \param [in]      transform_kernel_type  kernel type
 *  \param [in]      cos_ptr          pointer to cos table
 *  \param [in]      sin_ptr          pointer to sin table
 *
 *  \return VOID
 *
 */
void ia_core_coder_calc_post_twid_dec(FLOAT32 *xptr, FLOAT32 *r_ptr, FLOAT32 *i_ptr,
                                      WORD32 nlength, WORD16 transform_kernel_type,
                                      const FLOAT32 *cos_ptr, const FLOAT32 *sin_ptr)
{
  WORD32 i;
  FLOAT32 tmp;

  WORD32 size = nlength * 2;

  switch (transform_kernel_type)
  {
  case 1:
  case 2:
  { // IMDCT-II
    WORD32 k;

    /* reorder output */
    for (i = 0, k = 0; k < nlength / 2; k++)
    {
      xptr[i++] = r_ptr[k];
      xptr[i++] = i_ptr[nlength - 1 - k];
      xptr[i++] = i_ptr[k];
      xptr[i++] = r_ptr[nlength - 1 - k];
    }

    if (transform_kernel_type == 1)
    {
      for (i = 1; i < size; i += 2)
      {
        xptr[i] = ia_negate_flt(xptr[i]); /* reuse MDCT: flip signs at odd indices */
      }
    }
  }
  break;
  case 3:
  { // IMDST-IV

    for (i = 0; i < size / 2; i++)
    {
      tmp = ia_msu_flt(ia_mul_flt(r_ptr[i], cos_ptr[i]), i_ptr[i], sin_ptr[i]);
      xptr[i * 2 + 1] = ia_msu_flt(ia_mul_flt(-i_ptr[i], cos_ptr[i]), r_ptr[i], sin_ptr[i]);
      xptr[i * 2] = tmp;
    }

    /* reorder output */
    for (i = 0; i < size / 4; i++)
    {
      xptr[2 * i] = ia_negate_flt(xptr[2 * i]);
      xptr[2 * i + size / 2] = ia_negate_flt(xptr[2 * i + size / 2]);

      tmp = xptr[2 * i + 1];
      xptr[2 * i + 1] = ia_negate_flt(xptr[size - 1 - 2 * i]);
      xptr[size - 1 - 2 * i] = ia_negate_flt(tmp);
    }
  }
  break;
  default:
  {
    FLOAT32 *yptr = &xptr[2 * nlength - 1];

    for (i = 0; i < nlength; i++)
    {
      *xptr =
          ia_negate_flt(ia_msu_flt(ia_mul_flt((r_ptr[i]), (*cos_ptr)), (i_ptr[i]), (*sin_ptr)));
      *yptr = ia_negate_flt(
          ia_mac_flt(ia_mul_flt((i_ptr[i]), (*cos_ptr++)), (r_ptr[i]), (*sin_ptr++)));

      xptr += 2;
      yptr -= 2;
    }
  }
  break;
  }
}

/**
 *  ia_core_coder_fft_based_imdct
 *
 *  \brief Performs MDCT using fft
 *
 *  \param [in/out]    data          in/out data
 *  \param [in]      transform_kernel_type  Kernel type
 *  \param [in]      npoints          FFT points
 *  \param [in]      tmp_data        temporary buffer for data
 * handling
 *  \param [in]      ptr_fft_scratch      Scratch buffer for internal
 * processing
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_fft_based_imdct(FLOAT32 *data, WORD16 transform_kernel_type,
                                                  WORD32 npoints, FLOAT32 *tmp_data,
                                                  FLOAT32 *ptr_fft_scratch)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  FLOAT32 *data_r;
  FLOAT32 *data_i;
  WORD32 nlength = npoints >> 1;
  const FLOAT32 *cos_ptr;
  const FLOAT32 *sin_ptr;

  data_r = tmp_data;
  data_i = tmp_data + 512;
  if (transform_kernel_type == 2 || transform_kernel_type == 1)
  {
    cos_ptr = ia_core_coder_pre_twid_type2_dct_dst_cos_1024;
    sin_ptr = ia_core_coder_pre_twid_type2_dct_dst_sin_1024;
  }
  else
  {
    switch (nlength)
    {
    case 512:
      cos_ptr = ia_core_coder_pre_post_twid_cos_512;
      sin_ptr = ia_core_coder_pre_post_twid_sin_512;
      break;

    case 384:
      cos_ptr = ia_core_coder_pre_post_twid_cos_384;
      sin_ptr = ia_core_coder_pre_post_twid_sin_384;
      break;

    case 64:
      cos_ptr = ia_core_coder_pre_post_twid_cos_64;
      sin_ptr = ia_core_coder_pre_post_twid_sin_64;
      break;

    default:
      cos_ptr = ia_core_coder_pre_post_twid_cos_48;
      sin_ptr = ia_core_coder_pre_post_twid_sin_48;
      break;
    }
  }

  (*ia_core_coder_calc_pre_twid)(data, data_r, data_i, nlength, transform_kernel_type, cos_ptr,
                                 sin_ptr);
  impeghd_rad2_cplx_ifft(data_r, data_i, nlength, ptr_fft_scratch);

  (*ia_core_coder_calc_post_twid)(data, data_r, data_i, nlength, transform_kernel_type, cos_ptr,
                                  sin_ptr);
  return err;
}

#define N_LONG_LEN_MAX 1024

/**
 *  ia_core_coder_acelp_imdct
 *
 *  \brief Performs IMDCT in FD processing path
 *
 *  \param [in/out]    imdct_in        imdct in/out buffer
 *  \param [in]      npoints          number of FFT points
 *  \param [in]      transform_kernel_type  kernel type
 *  \param [in]      tmp_data        Scratch buffer for
 * internal
 * processing
 *  \param [in]      ptr_fft_scratch      Scratch buffer for internal
 * processing
 *
 *  \return IA_ERRORCODE
 *
 */
WORD32 ia_core_coder_acelp_imdct(FLOAT32 *imdct_in, WORD32 npoints, WORD16 transform_kernel_type,
                                 FLOAT32 *tmp_data, FLOAT32 *ptr_fft_scratch)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  FLOAT32 norm = 2.0f / (FLOAT32)npoints;

  for (i = 0; i < (npoints / 2); i++)
  {
    imdct_in[i] = ia_mul_flt(imdct_in[i], norm);
  }

  err = ia_core_coder_fft_based_imdct(imdct_in, transform_kernel_type, npoints / 2, tmp_data,
                                      ptr_fft_scratch);

  return err;
}

/**
 *  ia_core_coder_fd_imdct_short
 *
 *  \brief Calculates MDCT for short window in FD processing
 *
 *  \param [in/out]    usac_data        USAC data structure
 *  \param [in]      i_ch          Channel under process
 *  \param [in]      fac_data_out      buffer for FAC data out
 *  \param [in]      ia_core_coder_drc_offset    offset length
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_fd_imdct_short(ia_usac_data_struct *usac_data, WORD32 i_ch,
                                                 FLOAT32 *fac_data_out,
                                                 offset_lengths *ia_core_coder_drc_offset)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  FLOAT32 *window_short, *window_short_prev_ptr;
  WORD32 k;
  FLOAT32 *overlap_data, *fp;
  FLOAT32 *overlap_data_buf;
  UWORD32 overlap_data_buf_size;
  FLOAT32 *ptr_fft_scratch;
  FLOAT32 *p_overlap_ibuffer = usac_data->overlap_data_ptr[i_ch];
  FLOAT32 *p_in_ibuffer = usac_data->coef[i_ch];
  FLOAT32 *p_out_buffer = usac_data->time_sample_vector[i_ch];
  FLOAT32 *scratch_mem = usac_data->scratch_buffer_float;

  WORD32 td_frame_prev = usac_data->td_frame_prev[i_ch];
  WORD32 fac_apply = usac_data->fac_data_present[i_ch];

  WORD32 window_select = usac_data->window_shape[i_ch];
  WORD32 window_select_prev = usac_data->window_shape_prev[i_ch];
  WORD16 transform_kernel_type = (usac_data->prev_aliasing_symmetry[i_ch] << 1) +
                                 (usac_data->curr_aliasing_symmetry[i_ch] & 1);
  ia_usac_lpd_decoder_handle st = usac_data->str_tddec[i_ch];

  ptr_fft_scratch = usac_data->ptr_fft_scratch;
  overlap_data_buf_size = 2 * N_LONG_LEN_MAX;
  overlap_data_buf = ptr_fft_scratch;
  memset(overlap_data_buf, 0, overlap_data_buf_size * sizeof(FLOAT32));

  ptr_fft_scratch += overlap_data_buf_size;

  memcpy(overlap_data_buf, p_overlap_ibuffer, sizeof(FLOAT32) * ia_core_coder_drc_offset->n_long);
  overlap_data = overlap_data_buf;

  fp = overlap_data + ia_core_coder_drc_offset->n_flat_ls;

  for (k = 0; k < 8; k++)
  {
    err_code = ia_core_coder_acelp_imdct(p_in_ibuffer + (k * ia_core_coder_drc_offset->n_short),
                                         2 * ia_core_coder_drc_offset->n_short,
                                         transform_kernel_type, scratch_mem, ptr_fft_scratch);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  err_code =
      ia_core_coder_calc_window(&window_short, ia_core_coder_drc_offset->n_short, window_select);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }
  err_code = ia_core_coder_calc_window(&window_short_prev_ptr,
                                       ia_core_coder_drc_offset->n_trans_ls, window_select_prev);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
  {
    return err_code;
  }

  if (fac_apply)
    ia_core_coder_windowing_short1(p_in_ibuffer + ia_core_coder_drc_offset->n_short / 2,
                                   window_short_prev_ptr, fp, ia_core_coder_drc_offset);

  else
    ia_core_coder_windowing_short2(p_in_ibuffer + ia_core_coder_drc_offset->n_short / 2,
                                   window_short_prev_ptr, fp, ia_core_coder_drc_offset,
                                   transform_kernel_type);

  ia_core_coder_windowing_short3(p_in_ibuffer,
                                 window_short + ia_core_coder_drc_offset->n_short - 1,
                                 fp + ia_core_coder_drc_offset->n_short,
                                 ia_core_coder_drc_offset->n_short, transform_kernel_type);
  p_in_ibuffer += ia_core_coder_drc_offset->n_short;
  fp += ia_core_coder_drc_offset->n_short;
  window_short_prev_ptr = window_short;

  for (k = 1; k < 7; k++)
  {
    ia_core_coder_windowing_short4(p_in_ibuffer, window_short_prev_ptr, fp,
                                   window_short_prev_ptr + ia_core_coder_drc_offset->n_short - 1,
                                   ia_core_coder_drc_offset->n_short, 1, transform_kernel_type);
    p_in_ibuffer += ia_core_coder_drc_offset->n_short;
    fp += ia_core_coder_drc_offset->n_short;
    window_short_prev_ptr = window_short;
  }

  ia_core_coder_windowing_short4(p_in_ibuffer, window_short_prev_ptr, fp,
                                 window_short_prev_ptr + ia_core_coder_drc_offset->n_short - 1,
                                 ia_core_coder_drc_offset->n_short, 0, transform_kernel_type);

  if (fac_apply)
  {
    WORD32 i, len;
    FLOAT32 *data1, *data2;
    data1 = overlap_data + ia_core_coder_drc_offset->n_flat_ls + ia_core_coder_drc_offset->lfac;
    data2 = fac_data_out;
    len = ia_core_coder_drc_offset->lfac << 1;
    for (i = 0; i < len; i++)
    {
      data1[i] = ia_add_flt(data1[i], data2[i]);
    }
  }
  memset(overlap_data + 2 * ia_core_coder_drc_offset->n_long -
             ia_core_coder_drc_offset->n_flat_ls,
         0, sizeof(FLOAT32) * ia_core_coder_drc_offset->n_flat_ls);

  ia_core_coder_mem_cpy(overlap_data + ia_core_coder_drc_offset->n_long, p_overlap_ibuffer,
                        ia_core_coder_drc_offset->n_long);
  ia_core_coder_mem_cpy(overlap_data, p_out_buffer, ia_core_coder_drc_offset->n_long);

  if (td_frame_prev)
  {
    err_code = ia_core_coder_lpd_bpf(usac_data, 1, p_out_buffer, st);

    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_get_gain
 *
 *  \brief Calculate gain based on correlation and energy
 *
 *  \param [in]    x    Signal1
 *  \param [in]    y    Signal2
 *  \param [in]    n    length
 *
 *  \return WORD32
 *
 */
static FLOAT32 impeghd_get_gain(FLOAT32 *x, FLOAT32 *y, WORD32 n)
{
  FLOAT32 corr = 0.0f, ener = 1e-6f;
  WORD16 i;
  for (i = 0; i < n; i++)
  {
    corr = ia_add_flt(corr, ia_mul_flt(x[i], y[i]));
    ener = ia_add_flt(ener, ia_mul_flt(y[i], y[i]));
  }
  return (corr / ener);
}
/**
 *  impeghd_lpd_bpf
 *
 *  \brief LPD bass post filtering
 *
 *  \param [in]    isShort    Flag to check whether window sequence is Short
 *  \param [out]  out_buffer  Filtered out buffer
 *  \param [in]    st      LPD decoder handle
 *  \param [in]    td_config  USAC TD configure structure
 *  \param [in]    scratch    Scratch buffer for internal processing
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghd_lpd_bpf(WORD32 isShort, FLOAT32 out_buffer[], ia_usac_lpd_decoder_handle st,
                             ia_usac_td_config_struct *td_config, FLOAT32 *scratch)
{
  WORD32 i, T;
  FLOAT32 *synth_buf;
  FLOAT32 *signal_out;
  FLOAT32 *synth;
  WORD32 *pitch;
  FLOAT32 gain;
  FLOAT32 *pit_gain;
  WORD32 len_frame, lpd_sfd, syn_sfd, syn_delay;
  WORD32 full_band_lpd;
  WORD32 max_pitch = 0;

  synth_buf = (FLOAT32 *)scratch;
  signal_out = (FLOAT32 *)synth_buf + 2 * (MAX_PITCH + SYNTH_DELAY_LMAX) + LEN_SUPERFRAME;
  pitch = (WORD32 *)((FLOAT32 *)signal_out + LEN_SUPERFRAME);
  pit_gain = (FLOAT32 *)((WORD32 *)pitch + NUM_SUBFR_SUPERFRAME_BY2 + 3);

  full_band_lpd = td_config->full_band_lpd;
  if (!full_band_lpd)
  {
    len_frame = td_config->len_frame;
    lpd_sfd = (td_config->num_frame * td_config->num_subfrm) / 2;
    syn_sfd = lpd_sfd - BPF_SUBFRAME;
    syn_delay = syn_sfd * LEN_SUBFR;

    memset(synth_buf, 0, (MAX_PITCH + syn_delay + len_frame) * sizeof(FLOAT32));
    /* Initialize pointers for synthesis */
    ia_core_coder_mem_cpy(st->synth_prev, synth_buf, MAX_PITCH + syn_delay);
    ia_core_coder_mem_cpy(out_buffer, synth_buf + MAX_PITCH - (BPF_SUBFRAME * LEN_SUBFR),
                          syn_delay + len_frame + (BPF_SUBFRAME * LEN_SUBFR));
  }
  else
  {
    len_frame = td_config->len_frame_fb;
    lpd_sfd = (td_config->num_frame * td_config->num_subfrm) / 2;
    syn_sfd = lpd_sfd - BPF_SUBFRAME;
    syn_delay = syn_sfd * LEN_SUBFR;

    memset(synth_buf, 0,
           ((MAX_PITCH + syn_delay) * td_config->fac_fb + len_frame) * sizeof(FLOAT32)); //
    ia_core_coder_mem_cpy(st->synth_prev_fb, synth_buf,
                          (MAX_PITCH + syn_delay) * td_config->fac_fb);
    ia_core_coder_mem_cpy(
        out_buffer, synth_buf + (MAX_PITCH - (BPF_SUBFRAME * LEN_SUBFR)) * td_config->fac_fb,
        len_frame);
  }

  for (i = 0; i < syn_sfd; i++)
  {
    pitch[i] = st->pitch_prev[i];
    pit_gain[i] = st->gain_prev[i];
  }
  for (i = syn_sfd; i < lpd_sfd + 3; i++)
  {
    pitch[i] = LEN_SUBFR;
    pit_gain[i] = 0.0f;
  }
  if (st->mode_prev == 0)
  {
    pitch[syn_sfd] = pitch[syn_sfd - 1];
    pit_gain[syn_sfd] = pit_gain[syn_sfd - 1];
    if (!isShort)
    {
      pitch[syn_sfd + 1] = pitch[syn_sfd];
      pit_gain[syn_sfd + 1] = pit_gain[syn_sfd];
    }
  }

  synth = synth_buf + MAX_PITCH * td_config->fac_fb;
  max_pitch =
      (TMAX + (6 * ((((td_config->fscale * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN)));

  for (i = 0; i < syn_sfd + 2; i++)
  {
    T = pitch[i];
    if ((i * LEN_SUBFR + max_pitch) < T)
    {
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PITCH;
    }
    else if (((i * LEN_SUBFR + max_pitch - T) >= 1883) ||
             (((i * LEN_SUBFR) + LEN_SUBFR) > LEN_SUPERFRAME) ||
             ((((i * LEN_SUBFR) + LEN_SUBFR) - T) > LEN_SUPERFRAME))
    {
      return IA_MPEGH_DEC_EXE_FATAL_INVALID_PITCH;
    }
    gain = pit_gain[i];

    if (ia_lt_flt(0.0f, gain))
    {
      gain = impeghd_get_gain(&synth[i * LEN_SUBFR * td_config->fac_fb],
                              &synth[(i * LEN_SUBFR * td_config->fac_fb) - td_config->fac_fb * T],
                              LEN_SUBFR * td_config->fac_fb);
      pit_gain[i] = gain;
    }
  }

  ia_core_coder_bass_post_filter(synth, synth, td_config->fac_fb, pitch, pit_gain, signal_out,
                                 (lpd_sfd + 2) * LEN_SUBFR + (BPF_SUBFRAME * LEN_SUBFR),
                                 len_frame - (lpd_sfd + 4) * LEN_SUBFR, st->bpf_prev);

  ia_core_coder_mem_cpy(signal_out, out_buffer,
                        td_config->fac_fb *
                            ((lpd_sfd + 2) * LEN_SUBFR + (BPF_SUBFRAME * LEN_SUBFR)));
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  ia_core_coder_fd_imdct_long
 *
 *  \brief IMDCT for long windowin FD processing
 *
 *  \param [in/out]    usac_data        usac data structure
 *  \param [in]      i_ch          Channel under process
 *  \param [in]      fac_idata        pointer to FAC data buffer
 *  \param [in]      ia_core_coder_drc_offset    pointer to offset lengths
 *  \param [in]      ele_id          element id in the total number
 * of
 * elements present
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE ia_core_coder_fd_imdct_long(ia_usac_data_struct *usac_data, WORD32 i_ch,
                                                FLOAT32 *fac_idata,
                                                offset_lengths *ia_core_coder_drc_offset,
                                                WORD32 ele_id)
{
  WORD32 /*k,*/ i;
  FLOAT32 *window_long_prev, *window_short_prev_ptr;
  FLOAT32 *p_in_ibuffer = usac_data->coef[i_ch];
  FLOAT32 *p_overlap_ibuffer = usac_data->overlap_data_ptr[i_ch];
  FLOAT32 *p_out_ibuffer = usac_data->time_sample_vector[i_ch];
  FLOAT32 *scratch_mem = usac_data->scratch_buffer_float;
  WORD32 n_long = usac_data->ccfl;
  WORD32 fac_apply = usac_data->fac_data_present[i_ch];
  // WORD8 shiftp = 0, output_q = 0, shift_olap = 14;

  WORD32 window_sequence = usac_data->window_sequence[i_ch];
  WORD32 window_select_prev = usac_data->window_shape_prev[i_ch];
  WORD16 transform_kernel_type = (usac_data->prev_aliasing_symmetry[i_ch] << 1) +
                                 (usac_data->curr_aliasing_symmetry[i_ch] & 1);

  ia_usac_lpd_decoder_handle st = usac_data->str_tddec[i_ch];
  ia_usac_td_config_struct *td_config = &usac_data->td_config[ele_id];

  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;

  err_code =
      ia_core_coder_acelp_imdct(p_in_ibuffer, 2 * ia_core_coder_drc_offset->n_long,
                                transform_kernel_type, scratch_mem, usac_data->ptr_fft_scratch);

  if (err_code)
    return err_code;

  switch (window_sequence)
  {
  case ONLY_LONG_SEQUENCE:
  case LONG_START_SEQUENCE:
    err_code = ia_core_coder_calc_window(&window_long_prev, ia_core_coder_drc_offset->n_long,
                                         window_select_prev);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
    ia_core_coder_windowing_long1(p_in_ibuffer + n_long / 2, p_overlap_ibuffer, window_long_prev,
                                  window_long_prev + ia_core_coder_drc_offset->n_long - 1,
                                  p_out_ibuffer, transform_kernel_type,
                                  ia_core_coder_drc_offset->n_long);

    break;

  case STOP_START_SEQUENCE:
  case LONG_STOP_SEQUENCE:
    err_code = ia_core_coder_calc_window(
        &window_short_prev_ptr, ia_core_coder_drc_offset->n_trans_ls, window_select_prev);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
    if (fac_apply)
    {
      ia_core_coder_windowing_long2(p_in_ibuffer + n_long / 2, window_short_prev_ptr, fac_idata,
                                    p_overlap_ibuffer, p_out_ibuffer, ia_core_coder_drc_offset);
    }
    else
    {
      if (td_config->full_band_lpd && usac_data->td_frame_prev[i_ch] == 1 && st->mode_prev == 0)
      {
        FLOAT32 *pola = p_overlap_ibuffer + n_long / 4;
        for (i = 0; i < n_long / 4 + L_FILT_MAX; i++)
        {
          pola[i] = ia_add_flt(pola[i], st->tbe_dec_data.synth_curr[i]);
        }
      }
      ia_core_coder_windowing_long3(
          p_in_ibuffer + n_long / 2, window_short_prev_ptr, p_overlap_ibuffer, p_out_ibuffer,
          window_short_prev_ptr + ia_core_coder_drc_offset->n_trans_ls - 1,
          ia_core_coder_drc_offset, transform_kernel_type);
    }
    break;
  }

  switch (transform_kernel_type)
  {
  case 1:
    for (i = 0; i < ia_core_coder_drc_offset->n_long / 2; i++)
    {
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 + i] = p_in_ibuffer[i];
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 - i - 1] =
          ia_negate_flt(p_in_ibuffer[i]);
    }
    break;
  case 2:
    for (i = 0; i < ia_core_coder_drc_offset->n_long / 2; i++)
    {
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 + i] = p_in_ibuffer[i];
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 - i - 1] = p_in_ibuffer[i];
    }
    break;
  case 3:
    for (i = 0; i < ia_core_coder_drc_offset->n_long / 2; i++)
    {
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 + i] =
          ia_negate_flt(p_in_ibuffer[i]);
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 - i - 1] = p_in_ibuffer[i];
    }
    break;
  default:
    if (usac_data->td_frame_prev[i_ch])
    {
      /* base postfilter FAC area */
      if (td_config->lpd_stereo_idx == 1)
      {
        /* Need to add stereo lpd  bpf function */
      }
      else
      {
        err_code = impeghd_lpd_bpf(window_sequence == EIGHT_SHORT_SEQUENCE, p_out_ibuffer, st,
                                   td_config, usac_data->ptr_fft_scratch);

        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }
      }
    }

    for (i = 0; i < ia_core_coder_drc_offset->n_long / 2; i++)
    {
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 + i] =
          ia_negate_flt(p_in_ibuffer[i]);
      p_overlap_ibuffer[ia_core_coder_drc_offset->n_long / 2 - i - 1] =
          ia_negate_flt(p_in_ibuffer[i]);
    }
    break;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_fd_frm_dec
 *
 *  \brief Performs FD frame processing
 *
 *  \param [in/out]  usac_data  USAC data structure
 *  \param [in]    i_ch    Channel for processing
 *  \param [in]    ele_id    element id in the total number of elements present
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_fd_frm_dec(ia_usac_data_struct *usac_data, WORD32 i_ch, WORD32 ele_id)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;

  WORD32 td_frame_prev = usac_data->td_frame_prev[i_ch];
  WORD32 fac_apply = usac_data->fac_data_present[i_ch];
  WORD32 window_sequence = usac_data->window_sequence[i_ch];

  FLOAT32 fac_time_sig[2 * FAC_LENGTH + 16];

  offset_lengths ia_core_coder_drc_offset;

  ia_core_coder_drc_offset.n_long = usac_data->ccfl;
  ia_core_coder_drc_offset.n_short = ia_core_coder_drc_offset.n_long >> 3;

  memset(fac_time_sig, 0, sizeof(fac_time_sig));

  if (!td_frame_prev)
  {
    ia_core_coder_drc_offset.lfac = FAC_LENGTH;
    ia_core_coder_drc_offset.n_flat_ls =
        (ia_core_coder_drc_offset.n_long - ia_core_coder_drc_offset.n_short) >> 1;
    ia_core_coder_drc_offset.n_trans_ls = ia_core_coder_drc_offset.n_short;
  }
  else
  {
    if (window_sequence == EIGHT_SHORT_SEQUENCE)
    {
      ia_core_coder_drc_offset.lfac = ia_core_coder_drc_offset.n_long >> 4;
    }
    else
    {
      ia_core_coder_drc_offset.lfac = ia_core_coder_drc_offset.n_long >> 3;
    }
    ia_core_coder_drc_offset.n_flat_ls =
        (ia_core_coder_drc_offset.n_long - ((ia_core_coder_drc_offset.lfac) << 1)) >> 1;

    ia_core_coder_drc_offset.n_trans_ls = (ia_core_coder_drc_offset.lfac) << 1;
  }

  if (fac_apply)
  {
    err = ia_core_coder_fwd_alias_cancel_tool(
        usac_data, usac_data->fac_data[i_ch], &usac_data->td_config[ele_id],
        ia_core_coder_drc_offset.lfac, usac_data->lpc_prev[i_ch], fac_time_sig, fac_time_sig,
        usac_data->str_tddec[i_ch]->fscale, usac_data->ptr_fft_scratch, usac_data->acelp_in[i_ch],
        usac_data->td_config->lpd_stereo_idx, ia_core_coder_drc_offset.n_long >> 2);

    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }
  }

  if (window_sequence == EIGHT_SHORT_SEQUENCE)
  {
    err = ia_core_coder_fd_imdct_short(usac_data, i_ch, fac_time_sig, &ia_core_coder_drc_offset);
    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }
  }
  else
  {
    err = ia_core_coder_fd_imdct_long(usac_data, i_ch, fac_time_sig, &ia_core_coder_drc_offset,
                                      ele_id);
    if (err != IA_MPEGH_DEC_NO_ERROR)
    {
      return err;
    }
  }

  return err;
}
/** @} */ /* End of CoreDecProc */