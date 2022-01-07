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
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_constants.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_rom.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_func_def.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_vec_baisc_ops.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_igf_dec.h"
#include "ia_core_coder_ltpf.h"
#include "ia_core_coder_rom.h"
#include "impeghd_multichannel.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_usac_cplx_save_prev
 *
 *  \brief Save previous complex prediction data
 *
 *  \param [in,out] info        Pointer to scale factor band info structure.
 *  \param [in]  l_spec      Pointer to left spectrum data.
 *  \param [in]  r_spec      Pointer to right spectrum data.
 *  \param [out] l_spec_prev Pointer to left spectrum previous data.
 *  \param [out] r_spec_prev Pointer to right spectrum previous data.
 *  \param [in]  save_zeros  Flag to set previous data to zeros.
 *
 *
 *
 */
static VOID ia_core_coder_usac_cplx_save_prev(ia_sfb_info_struct *info, FLOAT32 *l_spec,
                                              FLOAT32 *r_spec, FLOAT32 *l_spec_prev,
                                              FLOAT32 *r_spec_prev, WORD32 save_zeros)
{
  WORD32 drc_offset;

  drc_offset = info->samp_per_bk - info->bins_per_sbk;
  if (0 == save_zeros)
  {
    ia_core_coder_mem_cpy(l_spec + drc_offset, l_spec_prev + drc_offset, info->bins_per_sbk);
    ia_core_coder_mem_cpy(r_spec + drc_offset, r_spec_prev + drc_offset, info->bins_per_sbk);
  }
  else
  {
    ia_core_coder_memset(l_spec_prev + drc_offset, info->bins_per_sbk);
    ia_core_coder_memset(r_spec_prev + drc_offset, info->bins_per_sbk);
  }
}

/**
 *  ia_core_coder_cplx_pred_data
 *
 *  \brief Complex predication data
 *
 *  \param [in,out] usac_data         Pointer to USAC data structure.
 *  \param [in,out] pstr_core_coder   Pointer to core coder data.
 *  \param [in]  num_win_groups    Number of window groups.
 *  \param [in]  it_bit_buff       Pointer to bit buffer structure.
 *  \param [in]  chn               Channel value.
 *
 *
 *
 */
static VOID ia_core_coder_cplx_pred_data(ia_usac_data_struct *usac_data,
                                         ia_usac_tmp_core_coder_struct *pstr_core_coder,
                                         WORD32 num_win_groups, ia_bit_buf_struct *it_bit_buff,
                                         WORD32 chn)
{
  ia_huff_code_book_struct *ptr_huff_code_book = &ia_core_coder_book;
  const ia_huff_code_word_struct *ptr_huff_code_word = ptr_huff_code_book->pstr_huff_code_word;
  WORD32 cplx_pred_all;
  WORD32 delta_code_time;
  WORD32 grp, sfb;
  WORD32 dpcm_alpha, last_alpha_q_re, last_alpha_q_im;
  UWORD8 max_sfb_ste = (UWORD8)pstr_core_coder->max_sfb_ste;

  WORD32(*alpha_q_re)[SFB_NUM_MAX] = usac_data->alpha_q_re[chn];
  WORD32(*alpha_q_im)[SFB_NUM_MAX] = usac_data->alpha_q_im[chn];
  WORD32 *alpha_q_re_prev = usac_data->alpha_q_re_prev[chn];
  WORD32 *alpha_q_im_prev = usac_data->alpha_q_im_prev[chn];
  UWORD8(*cplx_pred_used)[SFB_NUM_MAX] = usac_data->cplx_pred_used[chn];

  cplx_pred_all = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (cplx_pred_all == 1)
  {
    for (grp = 0; grp < num_win_groups; grp++)
    {
      memset(cplx_pred_used[grp], 1, max_sfb_ste * sizeof(cplx_pred_used[grp][0]));
      memset(&cplx_pred_used[grp][max_sfb_ste], 0,
             (SFB_NUM_MAX - max_sfb_ste) * sizeof(cplx_pred_used[grp][0]));
    }
  }
  else
  {
    for (grp = 0; grp < num_win_groups; grp++)
    {
      for (sfb = 0; sfb < max_sfb_ste; sfb += SFB_PER_PRED_BAND)
      {
        cplx_pred_used[grp][sfb] = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);

        if (sfb + 1 < max_sfb_ste)
          cplx_pred_used[grp][sfb + 1] = cplx_pred_used[grp][sfb];
      }
      memset(&cplx_pred_used[grp][max_sfb_ste], 0,
             (SFB_NUM_MAX - max_sfb_ste) * sizeof(cplx_pred_used[grp][0]));
    }
  }

  pstr_core_coder->pred_dir = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  pstr_core_coder->complex_coef = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (pstr_core_coder->complex_coef)
  {
    if (0 == usac_data->usac_independency_flg)
      pstr_core_coder->use_prev_frame = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    else
      pstr_core_coder->use_prev_frame = 0;
  }

  if (0 == usac_data->usac_independency_flg)
    delta_code_time = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  else
    delta_code_time = 0;

  for (grp = 0; grp < num_win_groups; grp++)
  {
    for (sfb = 0; sfb < max_sfb_ste; sfb += SFB_PER_PRED_BAND)
    {
      if (0 == delta_code_time)
      {
        if (sfb > 0)
        {
          last_alpha_q_re = alpha_q_re[grp][sfb - 1];
          last_alpha_q_im = alpha_q_im[grp][sfb - 1];
        }
        else
        {
          last_alpha_q_re = last_alpha_q_im = 0;
        }
      }
      else
      {
        last_alpha_q_re = alpha_q_re_prev[sfb];
        last_alpha_q_im = alpha_q_im_prev[sfb];
      }

      if (0 == cplx_pred_used[grp][sfb])
      {
        alpha_q_re[grp][sfb] = 0;
        alpha_q_im[grp][sfb] = 0;
      }
      else
      {
        dpcm_alpha = -ia_core_coder_huff_codeword(ptr_huff_code_word, it_bit_buff) + 60;
        alpha_q_re[grp][sfb] = dpcm_alpha + last_alpha_q_re;

        if (pstr_core_coder->complex_coef)
        {
          dpcm_alpha = -ia_core_coder_huff_codeword(ptr_huff_code_word, it_bit_buff) + 60;
          alpha_q_im[grp][sfb] = dpcm_alpha + last_alpha_q_im;
        }
        else
        {
          alpha_q_im[grp][sfb] = 0;
        }
      }

      if ((sfb + 1) < max_sfb_ste)
      {
        alpha_q_re[grp][sfb + 1] = alpha_q_re[grp][sfb];
        alpha_q_im[grp][sfb + 1] = alpha_q_im[grp][sfb];
      }

      alpha_q_re_prev[sfb] = alpha_q_re[grp][sfb];
      alpha_q_im_prev[sfb] = alpha_q_im[grp][sfb];
    }

    memset(&alpha_q_re[grp][max_sfb_ste], 0,
           sizeof(alpha_q_re[grp][0]) * (pstr_core_coder->max_sfb_ste_clear - max_sfb_ste));
    memset(&alpha_q_im[grp][max_sfb_ste], 0,
           sizeof(alpha_q_im[grp][0]) * (pstr_core_coder->max_sfb_ste_clear - max_sfb_ste));
    memset(&alpha_q_re_prev[max_sfb_ste], 0,
           sizeof(alpha_q_re_prev[0]) * (pstr_core_coder->max_sfb_ste_clear - max_sfb_ste));
    memset(&alpha_q_im_prev[max_sfb_ste], 0,
           sizeof(alpha_q_im_prev[0]) * (pstr_core_coder->max_sfb_ste_clear - max_sfb_ste));
  }

  return;
}

/**
 *  ia_core_coder_read_ms_mask
 *
 *  \brief Reads M/S coding or predication mask
 *
 *  \param [in,out] usac_data       Pointer to USAC data structure.
 *  \param [in,out] pstr_core_coder Pointer to core coder data structure.
 *  \param [in]  it_bit_buff     Pointer to bit buffer structure.
 *  \param [in]  chn             Channel index.
 *
 *  \return WORD32 M/S mask flag.
 *
 */
static WORD32 ia_core_coder_read_ms_mask(ia_usac_data_struct *usac_data,
                                         ia_usac_tmp_core_coder_struct *pstr_core_coder,
                                         ia_bit_buf_struct *it_bit_buff, WORD32 chn)
{
  WORD32 grp, sfb;
  WORD32 ms_mask_present;

  UWORD8 *sfb_group = usac_data->group_dis[chn];
  UWORD8 max_sfb = (UWORD8)pstr_core_coder->max_sfb_ste;
  UWORD8 *ms_used = usac_data->ms_used[chn];
  ia_sfb_info_struct *info = usac_data->pstr_sfb_info[chn];

  ms_mask_present = ia_core_coder_read_bits_buf(it_bit_buff, 2);

  switch (ms_mask_present)
  {
  case 3:
    ia_core_coder_cplx_pred_data(usac_data, pstr_core_coder, info->num_groups, it_bit_buff, chn);
    break;

  case 2:
    for (grp = 0; grp < info->max_win_len; grp = *sfb_group++)
    {
      memset(ms_used, 1, pstr_core_coder->max_sfb_ste);
      ms_used += pstr_core_coder->max_sfb_ste;
      if (info->sfb_per_sbk > pstr_core_coder->max_sfb_ste)
      {
        memset(ms_used, 0, (info->sfb_per_sbk - pstr_core_coder->max_sfb_ste));
      }
      ms_used += (info->sfb_per_sbk - pstr_core_coder->max_sfb_ste);
    }
    break;

  case 1:
    for (grp = 0; grp < info->max_win_len; grp = *sfb_group++)
    {
      for (sfb = 0; sfb < max_sfb; sfb++)
      {
        *ms_used = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
        ms_used++;
      }
      if (info->sfb_per_sbk > max_sfb)
      {
        memset(ms_used, 0, (info->sfb_per_sbk - max_sfb));
      }
      ms_used += (info->sfb_per_sbk - max_sfb);
    }
    break;
  }

  return ms_mask_present;
}

/**
 *  ia_core_coder_read_igf_mask
 *
 *  \brief Reads intelligent gap filling mask
 *
 *  \param [in,out] usac_data       Pointer to USAC data structure.
 *  \param [in]  it_bit_buff     Pointer to bit buffer structure.
 *  \param [in]  chn             Channel index.
 *  \param [in]  elem_idx        Element index.
 *
 *  \return WORD32 IGF mask flag.
 *
 */
WORD32 ia_core_coder_read_igf_mask(ia_usac_data_struct *usac_data, ia_bit_buf_struct *it_bit_buff,
                                   WORD32 chn, WORD32 elem_idx)
{
  WORD32 grp, sfb;
  WORD32 igf_mask_present;
  WORD32 sfb_per_ms_band = 2;
  WORD32 igf_start_sfb;
  WORD32 igf_stop_sfb;

  UWORD8 *sfb_group = usac_data->group_dis[chn];
  UWORD8 *ms_used = usac_data->ms_used[chn];
  ia_sfb_info_struct *info = usac_data->pstr_sfb_info[chn];
  ia_usac_igf_config_struct *ptr_igf_config = &usac_data->igf_config[elem_idx];
  const WORD32 igf_win_type = (!(info->islong)) ? 1 : 0;
  igf_start_sfb = ptr_igf_config->igf_grid[igf_win_type].igf_sfb_start;
  igf_stop_sfb = ptr_igf_config->igf_grid[igf_win_type].igf_sfb_stop;

  if (ptr_igf_config->use_high_res || igf_win_type > 1)
  {
    sfb_per_ms_band = 1;
  }

  igf_mask_present = ia_core_coder_read_bits_buf(it_bit_buff, 2);

  switch (igf_mask_present)
  {
  case 3:
    ia_core_coder_igf_stereo_pred_data(usac_data, info->num_groups, igf_win_type, it_bit_buff,
                                       chn, elem_idx);
    break;

  case 2:
    for (grp = 0; grp < info->max_win_len; grp = *sfb_group++)
    {
      ms_used += igf_start_sfb;
      memset(ms_used, 1, (igf_stop_sfb - igf_start_sfb));
      ms_used += igf_stop_sfb - igf_start_sfb;
      memset(ms_used, 0, (info->sfb_per_sbk - igf_stop_sfb));
      ms_used += (info->sfb_per_sbk - igf_stop_sfb);
    }
    break;

  case 1:
    for (grp = 0; grp < info->max_win_len; grp = *sfb_group++)
    {
      ms_used += igf_start_sfb;
      for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += sfb_per_ms_band)
      {
        *ms_used = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);
        ms_used++;
        if (sfb + 1 < igf_stop_sfb && sfb_per_ms_band == 2)
        {
          *ms_used = *(ms_used - 1);
          ms_used++;
        }
      }
      memset(ms_used, 0, (info->sfb_per_sbk - igf_stop_sfb));
      ms_used += (info->sfb_per_sbk - igf_stop_sfb);
    }
    break;
  }

  return igf_mask_present;
}

/**
 *  ia_core_coder_filter_and_add
 *
 *  \brief Helper function to filter and add
 *
 *  \param [in]  in          Pointer to input buffer.
 *  \param [in]  length      Length of the input data.
 *  \param [in]  filter      Filter coefficients.
 *  \param [out] out         Pointer to output buffer.
 *  \param [in]  factor_even Multiplication factor.
 *
 *
 *
 */
static VOID ia_core_coder_filter_and_add(const FLOAT32 *in, const WORD32 length,
                                         const FLOAT32 *filter, FLOAT32 *out,
                                         const WORD32 factor_even)
{
  WORD32 idx;
  FLOAT32 s;

  idx = 0;
  s = ia_add_flt(ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[2]), filter[5], in[1]),
                            ia_mac_flt(ia_mul_flt(filter[4], in[0]), filter[3], in[0])),
                 ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[1]), filter[1], in[2]),
                            ia_mul_flt(filter[0], in[3])));
  out[idx] = ia_mac_flt(out[idx], s, (FLOAT32)factor_even);
  idx = 1;
  s = ia_add_flt(ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[1]), filter[5], in[0]),
                            ia_mac_flt(ia_mul_flt(filter[4], in[0]), filter[3], in[1])),
                 ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[2]), filter[1], in[3]),
                            ia_mul_flt(filter[0], in[4])));
  out[idx] = ia_add_flt(out[idx], s);
  idx = 2;
  s = ia_add_flt(ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[0]), filter[5], in[0]),
                            ia_mac_flt(ia_mul_flt(filter[4], in[1]), filter[3], in[2])),
                 ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[3]), filter[1], in[4]),
                            ia_mul_flt(filter[0], in[5])));
  out[idx] = ia_mac_flt(out[idx], s, (FLOAT32)factor_even);

  for (idx = 3; idx < length - 4; idx += 2)
  {
    s = ia_add_flt(
        ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[idx - 3]), filter[5], in[idx - 2]),
                   ia_mac_flt(ia_mul_flt(filter[4], in[idx - 1]), filter[3], in[idx])),
        ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[idx + 1]), filter[1], in[idx + 2]),
                   ia_mul_flt(filter[0], in[idx + 3])));
    out[idx] = ia_add_flt(out[idx], s);
    s = ia_add_flt(
        ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[idx - 2]), filter[5], in[idx - 1]),
                   ia_mac_flt(ia_mul_flt(filter[4], in[idx]), filter[3], in[idx + 1])),
        ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[idx + 2]), filter[1], in[idx + 3]),
                   ia_mul_flt(filter[0], in[idx + 4])));
    out[idx + 1] = ia_mac_flt(out[idx + 1], s, (FLOAT32)factor_even);
  }

  idx = length - 3;
  s = ia_add_flt(
      ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[idx - 3]), filter[5], in[idx - 2]),
                 ia_mac_flt(ia_mul_flt(filter[4], in[idx - 1]), filter[3], in[idx])),
      ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[idx + 1]), filter[1], in[idx + 2]),
                 ia_mul_flt(filter[0], in[idx + 2])));
  out[idx] = ia_add_flt(out[idx], s);
  idx = length - 2;
  s = ia_add_flt(
      ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[idx - 3]), filter[5], in[idx - 2]),
                 ia_mac_flt(ia_mul_flt(filter[4], in[idx - 1]), filter[3], in[idx])),
      ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[idx + 1]), filter[1], in[idx + 1]),
                 ia_mul_flt(filter[0], in[idx])));
  out[idx] = ia_mac_flt(out[idx], s, (FLOAT32)factor_even);
  idx = length - 1;
  s = ia_add_flt(
      ia_add_flt(ia_mac_flt(ia_mul_flt(filter[6], in[idx - 3]), filter[5], in[idx - 2]),
                 ia_mac_flt(ia_mul_flt(filter[4], in[idx - 1]), filter[3], in[idx])),
      ia_add_flt(ia_mac_flt(ia_mul_flt(filter[2], in[idx]), filter[1], in[idx - 1]),
                 ia_mul_flt(filter[0], in[idx - 2])));
  out[idx] = ia_add_flt(out[idx], s);
}

/**
 *  ia_core_coder_estimate_dmx_im
 *
 *  \brief Computation of the downmix MDST estimate
 *
 *  \param [in]  dmx_re        Pointer to downmix real data.
 *  \param [in]  dmx_re_prev   Pointer to previous downmix real data.
 *  \param [out] dmx_im        Pointer to downmix imaginary data.
 *  \param [in]  pstr_sfb_info Pointer to scalefactor band information.
 *  \param [in]  window        Window type.
 *  \param [in]  w_shape       Window shape.
 *  \param [in]  prev_w_shape  Previous window shape.
 *
 *
 *
 */
static VOID ia_core_coder_estimate_dmx_im(const FLOAT32 *dmx_re, const FLOAT32 *dmx_re_prev,
                                          FLOAT32 *dmx_im, ia_sfb_info_struct *pstr_sfb_info,
                                          WORD32 window, const WORD32 w_shape,
                                          const WORD32 prev_w_shape)
{
  WORD32 win;
  WORD32 bins_per_sbk = pstr_sfb_info->bins_per_sbk;
  const FLOAT32 *curr_fcoeff, *prev_fcoeff;

  switch (window)
  {
  case LONG_START_SEQUENCE:
    curr_fcoeff = ia_core_coder_mdst_fcoeff_start_curr[prev_w_shape][w_shape];
    prev_fcoeff = ia_core_coder_mdst_fcoeff_l_s_start_left_prev[prev_w_shape];
    break;
  case STOP_START_SEQUENCE:
    curr_fcoeff = ia_core_coder_mdst_fcoeff_stopstart_cur[prev_w_shape][w_shape];
    prev_fcoeff = ia_core_coder_mdst_fcoeff_stop_stopstart_left_prev[prev_w_shape];
    break;
  case LONG_STOP_SEQUENCE:
    curr_fcoeff = ia_core_coder_mdst_fcoeff_stop_cur[prev_w_shape][w_shape];
    prev_fcoeff = ia_core_coder_mdst_fcoeff_stop_stopstart_left_prev[prev_w_shape];
    break;
  case ONLY_LONG_SEQUENCE:
  case EIGHT_SHORT_SEQUENCE:
    curr_fcoeff = ia_core_coder_mdst_fcoeff_longshort_curr[prev_w_shape][w_shape];
    prev_fcoeff = ia_core_coder_mdst_fcoeff_l_s_start_left_prev[prev_w_shape];
    break;
  default:
    curr_fcoeff = ia_core_coder_mdst_fcoeff_stopstart_cur[prev_w_shape][w_shape];
    prev_fcoeff = ia_core_coder_mdst_fcoeff_stop_stopstart_left_prev[prev_w_shape];
    break;
  }

  for (win = 0; win < pstr_sfb_info->max_win_len; win++)
  {
    ia_core_coder_filter_and_add(dmx_re, bins_per_sbk, curr_fcoeff, dmx_im, 1);

    if (dmx_re_prev)
      ia_core_coder_filter_and_add(dmx_re_prev, bins_per_sbk, prev_fcoeff, dmx_im, -1);

    dmx_re_prev = dmx_re;
    dmx_re += bins_per_sbk;
    dmx_im += bins_per_sbk;
  }
  return;
}

/**
 *  ia_core_coder_cplx_pred_upmixing
 *
 *  \brief Computes complex prediction upmixing
 *
 *  \param [in,out] usac_data       Pointer to USAC data structure.
 *  \param [out] left_spec       Pointer to left spectrum data.
 *  \param [out] right_spec      Pointer to right spectrum data.
 *  \param [in,out] pstr_core_coder Pointer to core coder structure.
 *  \param [in]  chn             Channel index.
 *  \param [in]  pred_dir        Prediction direction flag.
 *  \param [in]  complex_coef    Complext coefficient flag.
 *  \param [in]  sfb_start       Scale factor band start position.
 *  \param [in]  sfb_stop        Scale factor band stop position.
 *
 *
 *
 */
static VOID ia_core_coder_cplx_pred_upmixing(ia_usac_data_struct *usac_data, FLOAT32 *left_spec,
                                             FLOAT32 *right_spec,
                                             ia_usac_tmp_core_coder_struct *pstr_core_coder,
                                             WORD32 chn, WORD32 pred_dir, WORD32 complex_coef,
                                             WORD32 sfb_start, WORD32 sfb_stop)
{
  WORD32 factor = -1;
  WORD32 grp, sfb, grp_len, idx, bin;
  WORD32 num_grps, sfb_per_sbk;

  FLOAT32 alpha_q_re_loc;
  FLOAT32 alpha_q_im_loc;
  FLOAT32 mix;

  WORD32(*alpha_q_re)[SFB_NUM_MAX] = usac_data->alpha_q_re[chn];
  WORD32(*alpha_q_im)[SFB_NUM_MAX] = usac_data->alpha_q_im[chn];
  UWORD8(*cplx_pred_used)[SFB_NUM_MAX] = usac_data->cplx_pred_used[chn];

  FLOAT32 *dmx_real = &usac_data->scratch_buffer_float[0];
  FLOAT32 *dmx_imag = &usac_data->x_ac_dec_float[0];
  FLOAT32 *dmx_re_prev = usac_data->dmx_re_prev[chn];

  ia_sfb_info_struct *pstr_sfb_info = usac_data->pstr_sfb_info[chn];

  if (0 == pred_dir)
    factor = 1;
  num_grps = pstr_sfb_info->num_groups;
  sfb_per_sbk = pstr_sfb_info->sfb_per_sbk;
  idx = 0;

  for (grp = 0; grp < num_grps; grp++)
  {
    for (grp_len = 0; grp_len < pstr_sfb_info->group_len[grp]; grp_len++)
    {
      for (sfb = 0; sfb < sfb_per_sbk; sfb++)
      {
        if (1 != cplx_pred_used[grp][sfb])
        {
          for (bin = 0; bin < pstr_sfb_info->sfb_width[sfb]; bin++, idx++)
          {
            dmx_real[idx] = ia_mul_flt(
                0.5f, ia_add_flt(left_spec[idx], ia_mul_flt((FLOAT32)factor, right_spec[idx])));
          }
        }

        else
        {
          ia_core_coder_mem_cpy(&left_spec[idx], &dmx_real[idx], pstr_sfb_info->sfb_width[sfb]);
          idx += pstr_sfb_info->sfb_width[sfb];
        }
      }
    }
  }

  ia_core_coder_memset(dmx_imag, BLOCK_LEN_LONG);
  if (0 == complex_coef)
  {
    for (grp = 0, idx = 0; grp < num_grps; grp++)
    {
      for (grp_len = 0; grp_len < pstr_sfb_info->group_len[grp]; grp_len++)
      {
        for (sfb = 0; sfb < sfb_per_sbk; sfb++)
        {
          alpha_q_re_loc = ia_mul_flt((FLOAT32)alpha_q_re[grp][sfb], 0.1f);
          if (cplx_pred_used[grp][sfb] && sfb >= sfb_start && sfb < sfb_stop)
          {
            for (bin = 0; bin < pstr_sfb_info->sfb_width[sfb]; bin++, idx++)
            {
              mix = ia_sub_flt(right_spec[idx], ia_mul_flt(alpha_q_re_loc, left_spec[idx]));
              right_spec[idx] = ia_mul_flt((FLOAT32)factor, ia_sub_flt(left_spec[idx], mix));
              left_spec[idx] = ia_add_flt(left_spec[idx], mix);
            }
          }
          else
          {
            idx += pstr_sfb_info->sfb_width[sfb];
          }
        }
      }
    }
  }
  else
  {
    if (pstr_core_coder->use_prev_frame)
    {
      ia_core_coder_estimate_dmx_im(dmx_real, dmx_re_prev, dmx_imag, pstr_sfb_info,
                                    usac_data->window_sequence[chn], usac_data->window_shape[chn],
                                    usac_data->window_shape_prev[chn]);
    }
    else
    {
      ia_core_coder_estimate_dmx_im(dmx_real, NULL, dmx_imag, pstr_sfb_info,
                                    usac_data->window_sequence[chn], usac_data->window_shape[chn],
                                    usac_data->window_shape_prev[chn]);
    }

    for (grp = 0, idx = 0; grp < num_grps; grp++)
    {
      for (grp_len = 0; grp_len < pstr_sfb_info->group_len[grp]; grp_len++)
      {
        for (sfb = 0; sfb < sfb_per_sbk; sfb++)
        {
          alpha_q_re_loc = ia_mul_flt((FLOAT32)alpha_q_re[grp][sfb], 0.1f);
          alpha_q_im_loc = ia_mul_flt((FLOAT32)alpha_q_im[grp][sfb], 0.1f);
          if (0 == cplx_pred_used[grp][sfb])
          {
            idx += pstr_sfb_info->sfb_width[sfb];
          }
          else
          {
            for (bin = 0; bin < pstr_sfb_info->sfb_width[sfb]; bin++, idx++)
            {
              mix = ia_sub_flt(
                  ia_sub_flt(right_spec[idx], ia_mul_flt(alpha_q_re_loc, left_spec[idx])),
                  ia_mul_flt(alpha_q_im_loc, dmx_imag[idx]));
              right_spec[idx] = ia_mul_flt((FLOAT32)factor, ia_sub_flt(left_spec[idx], mix));
              left_spec[idx] = ia_add_flt(left_spec[idx], mix);
            }
          }
        }
      }
    }
  }

  return;
}

/**
 *  ia_core_coder_cplx_prev_mdct_dmx
 *
 *  \brief Previous frame's complex downmix
 *
 *  \param [in]  pstr_sfb_info Pointer to scalefactor band information.
 *  \param [in]  left_spec     Pointer to left spectrum data.
 *  \param [in]  right_spec    Pointer to right spectrum data.
 *  \param [out] dmx_real_prev Pointer to previous dowminx real buffer.
 *  \param [in]  pred_dir      Prediction direction flag.
 *  \param [in]  save_zeros    Save zeros flag.
 *
 *
 *
 */
VOID ia_core_coder_cplx_prev_mdct_dmx(ia_sfb_info_struct *pstr_sfb_info, FLOAT32 *left_spec,
                                      FLOAT32 *right_spec, FLOAT32 *dmx_real_prev,
                                      WORD32 pred_dir, WORD32 save_zeros)
{
  WORD32 offset, bin;
  WORD32 bins_per_sbk;

  bins_per_sbk = pstr_sfb_info->bins_per_sbk;
  offset = pstr_sfb_info->samp_per_bk - bins_per_sbk;

  if (0 == save_zeros)
  {
    WORD32 factor = 1;
    if (pred_dir)
    {
      factor = -1;
    }

    for (bin = 0; bin < bins_per_sbk; bin++)
    {
      dmx_real_prev[bin] =
          ia_mul_flt(0.5f, ia_add_flt(left_spec[bin + offset],
                                      ia_mul_flt((FLOAT32)factor, right_spec[bin + offset])));
    }
  }
  else
  {
    ia_core_coder_memset(dmx_real_prev, bins_per_sbk);
  }
}

/**
 *  ia_core_coder_ics_info
 *
 *  \brief Syntax of USAC syntactic element of ics info
 *      Defined in ISO/IEC 23003?3:2012, 5.3, Table 22
 *
 *  \param [in,out] usac_data     Pointer to USAC data structure.
 *  \param [in]  chn           Channel index.
 *  \param [out] max_sfb       Max Scale factor band.
 *  \param [in]  it_bit_buff   Pointer to bit buffer structure.
 *  \param [in]     win_seq_prev  Previous window sequence
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE ia_core_coder_ics_info(ia_usac_data_struct *usac_data, WORD32 chn, UWORD8 *max_sfb,
                                    ia_bit_buf_struct *it_bit_buff, WORD32 win_seq_prev)
{
  WORD32 win_seq, scf_grp, scf_idx;
  WORD32 mask = 0x40;

  UWORD8 *scf_group_ptr = usac_data->group_dis[chn];

  win_seq = ia_core_coder_read_bits_buf(it_bit_buff, 2);

  win_seq = usac_data->window_sequence[chn] = ia_core_coder_win_seq_select(win_seq, win_seq_prev);
  if (win_seq == -1)
  {
    return IA_MPEGH_DEC_INIT_FATAL_WRONG_WIN_SEQ;
  }

  usac_data->window_shape[chn] = (WORD32)ia_core_coder_read_bits_buf(it_bit_buff, 1);
  usac_data->pstr_sfb_info[chn] = usac_data->pstr_usac_winmap[usac_data->window_sequence[chn]];

  if (0 == usac_data->pstr_usac_winmap[win_seq]->islong)
  {

    *max_sfb = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 4);

    scf_grp = ia_core_coder_read_bits_buf(it_bit_buff, 7);
    /*max_sfb shall be <= num_swb_long_window or num_swb_short_window as appropriate for
      window_sequence and sampling frequency and core coder frame length*/
    if (*max_sfb > ia_core_coder_samp_rate_info[usac_data->sampling_rate_idx].num_sfb_128)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_MAX_SFB;
    }

    for (scf_idx = 1; scf_idx < 8; scf_idx++)
    {
      if (!(scf_grp & mask))
        *scf_group_ptr++ = (UWORD8)scf_idx;

      mask = mask >> 1;
    }
    *scf_group_ptr++ = (UWORD8)scf_idx;

    ia_core_coder_calc_grp_offset(usac_data->pstr_usac_winmap[win_seq],
                                  &usac_data->group_dis[chn][0]);
  }
  else
  {
    *max_sfb = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 6);
    *scf_group_ptr = 1;
    /*max_sfb shall be <= num_swb_long_window or num_swb_short_window as appropriate for
     window_sequence and sampling frequency and core coder frame length*/
    if (*max_sfb > ia_core_coder_samp_rate_info[usac_data->sampling_rate_idx].num_sfb_1024)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_MAX_SFB;
    }
  }

  if (*max_sfb > usac_data->pstr_sfb_info[chn]->sfb_per_sbk)
  {
    *max_sfb = (UWORD8)usac_data->pstr_sfb_info[chn]->sfb_per_sbk;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_read_lpd_fd_data
 *
 *  \brief Compute core coder LPD/FD data
 *
 *  \param [in]  id                     USAC syntactic element id
 *  \param [in,out] usac_data              Pointer USAC data structure.
 *  \param [in]  elem_idx               Element index value.
 *  \param [in]  chan_offset            Channel offset.
 *  \param [in]  it_bit_buff            Pointer to bit buffer structure.
 *  \param [in]  nr_core_coder_channels Number of core coder channels.
 *  \param [in]  pstr_usac_dec_config   Pointer to USAC decoder config.
 *  \param [in,out] pstr_core_coder        Pointer to core coder data structure.
 *  \param [in]     pscr                   Pointer to scratch buffers
 *  \param [in]     td_frame_data          Pointer to scratch buffers
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
static IA_ERRORCODE ia_core_coder_read_lpd_fd_data(
    WORD32 id, ia_usac_data_struct *usac_data, WORD32 elem_idx, WORD32 chan_offset,
    ia_bit_buf_struct *it_bit_buff, WORD32 nr_core_coder_channels,
    ia_usac_decoder_config_struct *pstr_usac_dec_config,
    ia_usac_tmp_core_coder_struct *pstr_core_coder, ia_core_coder_data_scr_t *pscr,
    ia_td_frame_data_struct *td_frame_data)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ch, chn, left, right, j;
  WORD32 slpd_index;
  UWORD32 k;
  FLOAT32 *resample_scratch;

  ia_usac_td_config_handle td_config = &usac_data->td_config[elem_idx];
  ia_usac_lpd_decoder_handle *td_decoder = usac_data->str_tddec;

  left = chan_offset;
  right = chan_offset + 1;

  resample_scratch = &usac_data->ptr_fft_scratch[0] +
                     (sizeof(ia_core_coder_data_scr_t) + sizeof(*resample_scratch) - 1) /
                         sizeof(*resample_scratch);

  for (ch = 0, chn = chan_offset; ch < nr_core_coder_channels; ch++, chn++)
  {
    if (pstr_core_coder->core_mode[ch] == 1)
    { // this is TD
      if (0 == usac_data->td_frame_prev[chn])
      {
        if (td_config->fscale > FSCALE_MAX)
        {
          return IA_MPEGH_DEC_EXE_FATAL_INVALID_FSCALE;
        }
        if (td_config->full_band_lpd)
        {
          WORD32 lfac, win_idx;
          if (usac_data->window_sequence_last[chn] == EIGHT_SHORT_SEQUENCE)
          {
            lfac = 64;
            win_idx = 1;
          }
          else
          {
            lfac = 128;
            win_idx = 2;
          }
          const FLOAT32 *ptr_win_coef;
          const FLOAT32 *win_table[2][5] = {
              {ia_core_coder_kbd_window96, ia_core_coder_kbd_window128,
               ia_core_coder_kbd_window256, ia_core_coder_kbd_window192},
              {ia_core_coder_sine_window96, ia_core_coder_sine_window128,
               ia_core_coder_sine_window256, ia_core_coder_sine_window192}

          };
          const FLOAT32 **win_tab_prev = win_table[!usac_data->window_shape_prev[chn]];

          ia_core_coder_memset(pscr->buffer_lb, 1 + 2 * LEN_FRAME + LEN_SUPERFRAME);

          td_decoder[chn]->window_shape_prev = usac_data->window_shape_prev[chn];

          ptr_win_coef = win_tab_prev[win_idx];

          if (td_config->lpd_stereo_idx == 0)
          {
            for (k = 0; k < (UWORD32)2 * lfac; k++)
            {
              usac_data->overlap_data_ptr[chn][(td_config->len_frame) - lfac + k] =
                  ia_mul_flt(usac_data->overlap_data_ptr[chn][(td_config->len_frame) - lfac + k],
                             ptr_win_coef[(lfac << 1) - 1 - k]);
            }

            ia_core_coder_memset(usac_data->overlap_data_ptr[chn] + (td_config->len_frame) + lfac,
                                 td_config->len_frame - lfac);

            ia_core_coder_mem_cpy(usac_data->overlap_data_ptr[chn],
                                  td_decoder[chn]->overlap_buf_fb, usac_data->ccfl);

            ia_core_coder_td_resampler(
                td_decoder[chn]->overlap_buf_fb, (WORD16)(1 + 2 * td_config->len_subfrm + lfac),
                td_config->fscale_full_band, pscr->buffer_lb + td_config->len_subfrm,
                td_config->fscale, NULL, 0, resample_scratch);
            err_code = ia_core_coder_reset_acelp_data(
                usac_data, td_decoder[chn], td_config, pscr->buffer_lb,
                usac_data->window_sequence_last[chn] == EIGHT_SHORT_SEQUENCE, chn);
            if (err_code != IA_MPEGH_DEC_NO_ERROR)
            {
              return err_code;
            }
          }
          else if ((td_config->lpd_stereo_idx == 1) && (chn == left))
          {
            for (k = 0; k < (UWORD32)2 * lfac; k++)
            {
              usac_data->overlap_data_ptr[left][(td_config->len_frame) - lfac + k] =
                  ia_mul_flt(usac_data->overlap_data_ptr[left][(td_config->len_frame) - lfac + k],
                             ptr_win_coef[2 * lfac - 1 - k]);

              usac_data->overlap_data_ptr[right][(td_config->len_frame) - lfac + k] = ia_mul_flt(
                  usac_data->overlap_data_ptr[right][(td_config->len_frame) - lfac + k],
                  ptr_win_coef[2 * lfac - 1 - k]);
            }
            memset(usac_data->overlap_data_ptr[left] + (td_config->len_frame) + lfac, 0,
                   (td_config->len_frame) - lfac);
            memset(usac_data->overlap_data_ptr[right] + (td_config->len_frame) + lfac, 0,
                   (td_config->len_frame) - lfac);
            slpd_index = usac_data->slpd_index[left >> 1];
            ia_core_coder_slpd_set_past(
                &usac_data->slpd_dec_data[slpd_index], usac_data->overlap_data_ptr[left],
                usac_data->overlap_data_ptr[right], (usac_data->ccfl >> 1) + lfac);
            for (k = 0; k < (UWORD32)usac_data->ccfl; k++)
            {
              td_decoder[chn]->overlap_buf_fb[k] =
                  ia_mul_flt(0.5f, ia_add_flt(usac_data->overlap_data_ptr[left][k],
                                              usac_data->overlap_data_ptr[right][k]));
            }

            ia_core_coder_td_resampler(
                td_decoder[chn]->overlap_buf_fb, (WORD16)(1 + 2 * td_config->len_subfrm + lfac),
                td_config->fscale_full_band, pscr->buffer_lb + td_config->len_subfrm,
                td_config->fscale, NULL, 0, resample_scratch);
            err_code = ia_core_coder_reset_acelp_data(
                usac_data, td_decoder[chn], td_config, pscr->buffer_lb,
                usac_data->window_sequence_last[chn] == EIGHT_SHORT_SEQUENCE, chn);
            if (err_code != IA_MPEGH_DEC_NO_ERROR)
            {
              return err_code;
            }
          }
        }
        else
        {
          err_code = ia_core_coder_tw_buff_update(usac_data, chn, usac_data->str_tddec[chn]);
          if (err_code != IA_MPEGH_DEC_NO_ERROR)
          {
            return err_code;
          }
        }
      }

      usac_data->present_chan = chn;
      if (td_config->lpd_stereo_idx == 0)
      {
        err_code = ia_core_coder_lpd_channel_stream(usac_data, td_frame_data, it_bit_buff,
                                                    usac_data->time_sample_vector[chn], elem_idx);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        usac_data->window_shape[chn] = WIN_SEL_0;

        WORD32 lDiv_2 = td_config->len_subfrm << 1;
        WORD32 lfac;
        if (usac_data->window_sequence_last[chn] == EIGHT_SHORT_SEQUENCE)
        {
          lfac = 64;
        }
        else
        {
          lfac = 128;
        }

        if (0 == usac_data->td_frame_prev[chn] && td_config->full_band_lpd)
        {
          ia_core_coder_mem_cpy(td_decoder[chn]->overlap_buf_fb + lDiv_2 - lfac,
                                usac_data->overlap_data_ptr[chn] + lDiv_2 - lfac, lfac);
          ia_core_coder_memset(usac_data->overlap_data_ptr[chn] + lDiv_2 - lfac, lfac << 1);
        }
      }
      else if (td_config->lpd_stereo_idx == 1 && chn == left)
      {
        err_code = ia_core_coder_lpd_channel_stream(usac_data, td_frame_data, it_bit_buff,
                                                    usac_data->time_sample_vector[chn], elem_idx);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        usac_data->window_shape[chn] = WIN_SEL_0;
        slpd_index = usac_data->slpd_index[chn >> 1];
        err_code = ia_core_coder_slpd_data(&usac_data->slpd_dec_data[slpd_index], it_bit_buff,
                                           !usac_data->td_frame_prev[chn]);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        ia_core_coder_slpd_apply(&usac_data->slpd_dec_data[slpd_index], pscr->synth_stereo[left],
                                 pscr->synth_stereo[right], !usac_data->td_frame_prev[chn],
                                 pscr->slpd_scratch);

        usac_data->str_tddec[right]->ltpf_data.data_present =
            usac_data->str_tddec[left]->ltpf_data.data_present;
        usac_data->str_tddec[right]->ltpf_data.pitch_lag_idx =
            usac_data->str_tddec[left]->ltpf_data.pitch_lag_idx;
        usac_data->str_tddec[right]->ltpf_data.gain_idx =
            usac_data->str_tddec[left]->ltpf_data.gain_idx;
        if (0 == usac_data->td_frame_prev[chn])
        {
          WORD32 lDiv_2 = td_config->len_subfrm << 1;
          WORD32 lfac;
          if (usac_data->window_sequence_last[chn] == EIGHT_SHORT_SEQUENCE)
          {
            lfac = 64;
          }
          else
          {
            lfac = 128;
          }

          ia_core_coder_mem_cpy(pscr->synth_stereo[left], usac_data->overlap_data_ptr[left],
                                lDiv_2 - lfac);
          ia_core_coder_mem_cpy(pscr->synth_stereo[right], usac_data->overlap_data_ptr[right],
                                lDiv_2 - lfac);

          ia_core_coder_mem_cpy(td_decoder[chn]->overlap_buf_fb + lDiv_2 - lfac,
                                usac_data->overlap_data_ptr[left] + lDiv_2 - lfac, lfac);
          ia_core_coder_mem_cpy(td_decoder[chn]->overlap_buf_fb + lDiv_2 - lfac,
                                usac_data->overlap_data_ptr[right] + lDiv_2 - lfac, lfac);

          ia_core_coder_memset(usac_data->overlap_data_ptr[left] + lDiv_2 - lfac, lfac << 1);
          ia_core_coder_memset(usac_data->overlap_data_ptr[right] + lDiv_2 - lfac, lfac << 1);
        }
      }

      if (usac_data->td_config[elem_idx].lpd_stereo_idx == 0)
      {
        ia_core_coder_td_frm_dec(usac_data, chn, td_frame_data->mod[0]);
        ia_core_coder_usac_ltpf_process(&usac_data->str_tddec[chn]->ltpf_data,
                                        &usac_data->time_sample_vector[chn][0]);
      }
      else if (chn == left)
      {
        for (j = left; j <= right; j++)
        {
          ia_core_coder_mem_cpy(pscr->synth_stereo[j], usac_data->time_sample_vector[j],
                                usac_data->ccfl);

          ia_core_coder_td_frm_dec(usac_data, j, td_frame_data->mod[0]);

          ia_core_coder_usac_ltpf_process(&usac_data->str_tddec[j]->ltpf_data,
                                          &usac_data->time_sample_vector[j][0]);
        }
      }

      usac_data->window_shape_prev[chn] = usac_data->str_tddec[chn]->window_shape;
      usac_data->window_sequence_last[chn] = EIGHT_SHORT_SEQUENCE;

      memset(usac_data->fdp_quant_spec_prev[chn], 0,
             2 * 172 * sizeof(usac_data->fdp_quant_spec_prev[chn][0]));
      usac_data->prev_aliasing_symmetry[chn] = usac_data->curr_aliasing_symmetry[chn] = 0;
    }
    else
    {
      if (usac_data->coef[chn] == NULL)
      {
        return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_COEF;
      }

      ia_core_coder_memset(usac_data->coef[chn], LN2);

      if (usac_data->td_frame_prev[chn] && usac_data->str_tddec[chn])
      {
        ia_core_coder_lpd_dec_update(usac_data->str_tddec[chn], usac_data, chn, elem_idx);
      }

      if ((id != ID_USAC_LFE) &&
          ((nr_core_coder_channels == 1) ||
           (pstr_core_coder->core_mode[0] != pstr_core_coder->core_mode[1])))
      {
        pstr_core_coder->tns_data_present[ch] = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      }
      err_code = ia_core_coder_fd_channel_stream( // parseit
          usac_data, pstr_core_coder, &pstr_core_coder->max_sfb[ch],
          usac_data->window_sequence_last[chn], chn, usac_data->noise_filling_config[elem_idx],
          &usac_data->igf_config[elem_idx], ch, it_bit_buff, id);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }

      pstr_usac_dec_config->ia_sfb_info[chn] = (ia_sfb_info *)usac_data->pstr_sfb_info[chn];

      if (usac_data->usac_first_frame_flag[chn])
      {
        usac_data->window_shape_prev[chn] = usac_data->window_shape[chn];
        usac_data->usac_first_frame_flag[chn] = 0;
      }
      if (id == ID_USAC_LFE && (usac_data->curr_aliasing_symmetry[chn] != 0 ||
                                usac_data->prev_aliasing_symmetry[chn] != 0))
      {
        return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_data
 *
 *  \brief Compute core coder data
 *
 *  \param [in]  id                     USAC syntactic element id
 *  \param [in,out] usac_data              Pointer USAC data structure.
 *  \param [in]  elem_idx               Element index value.
 *  \param [in]  chan_offset            Channel offset.
 *  \param [in]  it_bit_buff            Pointer to bit buffer structure.
 *  \param [in]  nr_core_coder_channels Number of core coder channels.
 *  \param [in]  pstr_usac_dec_config   Pointer to USAC decoder config.
 *  \param [in,out] pstr_core_coder        Pointer to core coder data structure.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE
ia_core_coder_data(WORD32 id, ia_usac_data_struct *usac_data, WORD32 elem_idx, WORD32 chan_offset,
                   ia_bit_buf_struct *it_bit_buff, WORD32 nr_core_coder_channels,
                   ia_usac_decoder_config_struct *pstr_usac_dec_config,
                   ia_usac_tmp_core_coder_struct *pstr_core_coder)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ch = 0, left = 0, right = 0;

  ia_usac_igf_config_struct *igf_config = &usac_data->igf_config[elem_idx];
  ia_core_coder_data_scr_t *pscr;

  ia_td_frame_data_struct td_frame_data;
  memset(&td_frame_data, 0, sizeof(ia_td_frame_data_struct));

  pscr = (ia_core_coder_data_scr_t *)&usac_data->ptr_fft_scratch[0];

  pstr_core_coder->tns_on_lr = 1;
  pstr_core_coder->pred_dir = 0;

  if (id == ID_USAC_LFE)
  {
    memset(pstr_core_coder->core_mode, 0,
           nr_core_coder_channels * sizeof(pstr_core_coder->core_mode[0]));
  }
  else
  {
    for (ch = 0; ch < nr_core_coder_channels; ch++)
    {
      pstr_core_coder->core_mode[ch] = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    }
  }

  if (nr_core_coder_channels == 2 && pstr_core_coder->core_mode[0] == 0 &&
      pstr_core_coder->core_mode[1] == 0)
  {
    pstr_core_coder->tns_active = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    pstr_core_coder->common_window = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    if ((0 == pstr_core_coder->common_window) || usac_data->usac_independency_flg)
    {
      memset(usac_data->alpha_q_re_prev[chan_offset], 0,
             SFB_NUM_MAX * sizeof(usac_data->alpha_q_re_prev[chan_offset][0]));
      memset(usac_data->alpha_q_im_prev[chan_offset], 0,
             SFB_NUM_MAX * sizeof(usac_data->alpha_q_im_prev[chan_offset][0]));
    }

    if (0 == pstr_core_coder->common_window)
    {
      left = chan_offset;
      right = chan_offset + 1;

      pstr_core_coder->ms_mask_present[0] = 0;
      pstr_core_coder->ms_mask_present[1] = 0;
    }
    else
    {
      left = chan_offset;
      right = chan_offset + 1;

      err = ia_core_coder_ics_info(usac_data, left, &pstr_core_coder->max_sfb[CORE_LEFT],
                                   it_bit_buff, usac_data->window_sequence_last[left]);

      if (err != IA_MPEGH_DEC_NO_ERROR)
      {
        return err;
      }

      pstr_core_coder->common_max_sfb = ia_core_coder_read_bits_buf(it_bit_buff, 1);

      if (1 == pstr_core_coder->common_max_sfb)
      {
        pstr_core_coder->max_sfb[CORE_RIGHT] = pstr_core_coder->max_sfb[CORE_LEFT];
      }
      else
      {
        if (usac_data->window_sequence[left] != EIGHT_SHORT_SEQUENCE)
          pstr_core_coder->max_sfb[CORE_RIGHT] =
              (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 6);
        else
          pstr_core_coder->max_sfb[CORE_RIGHT] =
              (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 4);
      }

      pstr_core_coder->max_sfb_ste =
          ia_max_int(pstr_core_coder->max_sfb[CORE_LEFT], pstr_core_coder->max_sfb[CORE_RIGHT]);

      pstr_core_coder->max_sfb_ste_clear = SFB_NUM_MAX;

      if (usac_data->igf_config[elem_idx].igf_active)
      {
        if (0 == usac_data->pstr_sfb_info[left]->islong)
        {
          usac_data->igf_sfb_stop[elem_idx] =
              usac_data->igf_config[elem_idx].igf_grid[1].igf_sfb_stop;
          usac_data->igf_sfb_start[elem_idx] =
              usac_data->igf_config[elem_idx].igf_grid[1].igf_sfb_start;
        }
        else
        {
          usac_data->igf_sfb_stop[elem_idx] =
              usac_data->igf_config[elem_idx].igf_grid[0].igf_sfb_stop;
          usac_data->igf_sfb_start[elem_idx] =
              usac_data->igf_config[elem_idx].igf_grid[0].igf_sfb_start;
        }
        if (0 == usac_data->igf_config[elem_idx].igf_independent_tiling)
        {
          pstr_core_coder->max_sfb_ste =
              ia_min_int(pstr_core_coder->max_sfb_ste, usac_data->igf_sfb_start[elem_idx]);
          pstr_core_coder->max_sfb_ste_clear =
              ia_min_int(pstr_core_coder->max_sfb_ste_clear, usac_data->igf_sfb_start[elem_idx]);
        }
      }

      usac_data->window_shape[right] = usac_data->window_shape[left];
      usac_data->window_sequence[right] = usac_data->window_sequence[left];
      memcpy(&usac_data->group_dis[right][0], &usac_data->group_dis[left][0], 8);
      usac_data->pstr_sfb_info[right] = usac_data->pstr_sfb_info[left];
      if (pstr_core_coder->max_sfb[CORE_RIGHT] > usac_data->pstr_sfb_info[right]->sfb_per_sbk)
        pstr_core_coder->max_sfb[CORE_RIGHT] =
            (UWORD8)usac_data->pstr_sfb_info[right]->sfb_per_sbk;

      pstr_core_coder->ms_mask_present[0] =
          (UWORD8)ia_core_coder_read_ms_mask(usac_data, pstr_core_coder, it_bit_buff, left);

      /*If	 the	 independent	 noise	 filling	 (INF)	 of
      the	 intelligent	 gap	 filling	 (IGF)	 is	 activated
      (i.e.	 if	igfUseEnf==1),	 then	 the	 complex	 prediction	 tool
      shall be restricted	 to	 real-only	 prediction,
      i.e. complex_coef	shall	be	0*/

      /*If	 stereo	 filling	 is	 activated	 (i.e.	 if
      stereo_filling==1),	 then	 the	 complex	 prediction	 tool	 shall be
      restricted	to	real-only	prediction,	i.e.	complex_coef	shall	be
      0*/

      if (((pstr_usac_dec_config->str_usac_element_config->str_usac_ele_igf_init_config
                .igf_use_enf == 1) ||
           (usac_data->igf_config[elem_idx].igf_stereo_filling == 1)) &&
          (pstr_core_coder->complex_coef != 0))
      {
        return IA_MPEGH_DEC_INIT_FATAL_COMP_PRED_NOT_SUPPORTED;
      }

      if (usac_data->igf_config[elem_idx].igf_independent_tiling == 0 &&
          usac_data->igf_config[elem_idx].igf_active)
      {
        usac_data->igf_hasmask[elem_idx][left] =
            ia_core_coder_read_igf_mask(usac_data, it_bit_buff, left, elem_idx);
      }
      else
      {
        usac_data->igf_hasmask[elem_idx][left] = 0;
      }
      usac_data->igf_hasmask[elem_idx][right] = usac_data->igf_hasmask[elem_idx][left];
      usac_data->ms_used[right] = usac_data->ms_used[left];
    }

    pstr_core_coder->common_ltpf = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    if (pstr_core_coder->common_ltpf)
    {
      if (1 == usac_data->is_base_line_profile_3b)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_PROFILE_CONFIG;
      }
      usac_data->str_tddec[right]->ltpf_data.data_present =
          usac_data->str_tddec[left]->ltpf_data.data_present =
              ia_core_coder_read_bits_buf(it_bit_buff, 1);
      if (usac_data->str_tddec[left]->ltpf_data.data_present)
      {
        usac_data->str_tddec[right]->ltpf_data.pitch_lag_idx =
            usac_data->str_tddec[left]->ltpf_data.pitch_lag_idx =
                ia_core_coder_read_bits_buf(it_bit_buff, 9);
        usac_data->str_tddec[right]->ltpf_data.gain_idx =
            usac_data->str_tddec[left]->ltpf_data.gain_idx =
                ia_core_coder_read_bits_buf(it_bit_buff, 2);
      }
    }
    if (pstr_core_coder->tns_active)
    {
      if (0 == pstr_core_coder->common_window)
      {
        pstr_core_coder->common_tns = 0;
      }
      else
      {
        pstr_core_coder->common_tns = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      }

      if (igf_config->igf_active && !igf_config->igf_after_tns_synth)
      {
        pstr_core_coder->tns_on_lr = 1;
      }
      else
      {
        pstr_core_coder->tns_on_lr = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      }

      if (0 == pstr_core_coder->common_tns)
      {
        pstr_core_coder->tns_present_both = ia_core_coder_read_bits_buf(it_bit_buff, 1);

        if (pstr_core_coder->tns_present_both)
        {
          pstr_core_coder->tns_data_present[0] = 1;
          pstr_core_coder->tns_data_present[1] = 1;
        }
        else
        {
          pstr_core_coder->tns_data_present[1] = ia_core_coder_read_bits_buf(it_bit_buff, 1);
          pstr_core_coder->tns_data_present[0] = 1 - pstr_core_coder->tns_data_present[1];
        }
      }
      else
      {
        err = ia_core_coder_read_tns_u(usac_data->pstr_sfb_info[0], &usac_data->pstr_tns[left][0],
                                       it_bit_buff);
        if (err != IA_MPEGH_DEC_NO_ERROR)
        {
          return err;
        }
        memcpy(&usac_data->pstr_tns[right][0], &usac_data->pstr_tns[left][0],
               sizeof(ia_tns_frame_info_struct));

        pstr_core_coder->tns_data_present[0] = 2;
        pstr_core_coder->tns_data_present[1] = 2;
      }
    }
    else
    {
      pstr_core_coder->common_tns = 0;
      pstr_core_coder->tns_data_present[1] = pstr_core_coder->tns_data_present[0] = 0;
    }
  }
  else
  {
    pstr_core_coder->common_window = 0;
    pstr_core_coder->common_tw = 0;
    right = left = chan_offset;
    if (nr_core_coder_channels == 2)
      right = chan_offset + 1;
  }

  err = ia_core_coder_read_lpd_fd_data(id, usac_data, elem_idx, chan_offset, it_bit_buff,
                                       nr_core_coder_channels, pstr_usac_dec_config,
                                       pstr_core_coder, pscr, &td_frame_data);
  if (err != IA_MPEGH_DEC_NO_ERROR)
  {
    return err;
  }

  if (nr_core_coder_channels == 2 && pstr_core_coder->core_mode[0] == CORE_MODE_FD &&
      pstr_core_coder->core_mode[1] == CORE_MODE_FD)
  {
    ia_core_coder_cplx_prev_mdct_dmx(
        usac_data->pstr_sfb_info[left], usac_data->coef_save_float[left],
        usac_data->coef_save_float[right], usac_data->dmx_re_prev[left],
        pstr_core_coder->pred_dir,
        (usac_data->usac_independency_flg || usac_data->td_frame_prev[left] ||
         usac_data->td_frame_prev[right]));
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_ms_stereo
 *
 *  \brief  Process M/S coding or predication stereo
 *
 *  \param [in]  info       Pointer to scale factor band info.
 *  \param [in]  group      Pointer to scale factor group data.
 *  \param [in]  mask       Pointer to M/S mask data.
 *  \param [in]  first_band First band index.
 *  \param [in]  last_band  Last band index.
 *  \param [out] right_spec Pointer to right channel spectrum data.
 *  \param [out] left_spec  Pointer to left channel spectrum data.
 *
 *
 *
 */
VOID impeghd_ms_stereo(ia_sfb_info_struct *info, WORD8 *group, WORD8 *mask, WORD32 first_band,
                       WORD32 last_band, FLOAT32 *right_spec, FLOAT32 *left_spec)
{
  WORD32 band_idx_1, band_idx_2, band_n, win, k;
  WORD32 bins_per_sbk, sfb_per_sbk;

  FLOAT32 right_spec_loc, left_spec_loc;

  const WORD16 *band;

  FLOAT32 *ptr_lspec, *ptr_rspec;

  /* Grouping masks*/
  k = 0;
  bins_per_sbk = info->bins_per_sbk;
  sfb_per_sbk = info->sfb_per_sbk;
  for (win = 0; win < info->max_win_len;)
  {
    band = info->ptr_sfb_tbl;

    win = *group++; /* index of last sbk */
    for (; k < win; k++)
    {
      if (0 == first_band)
      {
        band_idx_2 = 0;
      }
      else
      {
        band_idx_2 = band[first_band - 1];
      }

      for (band_idx_1 = first_band; band_idx_1 < last_band; band_idx_1++)
      {
        band_n = band[band_idx_1];
        if (mask[band_idx_1])
        {
          ptr_lspec = left_spec + band_idx_2;
          ptr_rspec = right_spec + band_idx_2;
          for (; band_idx_2 < band_n; band_idx_2++)
          {
            left_spec_loc = *ptr_lspec;
            right_spec_loc = *ptr_rspec;
            *ptr_lspec = ia_add_flt(left_spec_loc, right_spec_loc);
            *ptr_rspec = ia_sub_flt(left_spec_loc, right_spec_loc);
            ptr_lspec++;
            ptr_rspec++;
          }
        }
        band_idx_2 = band_n;
      }
      left_spec += bins_per_sbk;
      right_spec += bins_per_sbk;
    }
    mask += sfb_per_sbk;
  }
}

/**
 *  ia_core_coder_data_process_igf_joint_stereo_before_tns
 *
 *  \brief Core coder data processing - IGF Joint stereo before TNS
 *
 *  \param [in,out]   usac_data              Poitner to USAC data structure.
 *  \param [in]       elem_idx               Element index value.
 *  \param [in]       chan_offset            Channel offset value.
 *  \param [in]       nr_core_coder_channels Number of core coder channels.
 *  \param [in,out]   pstr_core_coder        Pointer to core coder structure.
 *  \param [in]       igf_start_sfb          SFB start offset for IGF
 *  \param [in]       igf_stop_sfb           SFB stop offset for IGF
 *
 *
 *
 */
static VOID ia_core_coder_data_process_igf_joint_stereo_before_tns(
    ia_usac_data_struct *usac_data, WORD32 elem_idx, WORD32 chan_offset,
    WORD32 nr_core_coder_channels, ia_usac_tmp_core_coder_struct *pstr_core_coder,
    WORD32 igf_start_sfb, WORD32 igf_stop_sfb)
{
  WORD32 wn, chn, left, right;
  WORD32 igf_pred_dir = usac_data->igf_pred_dir[elem_idx];
  WORD32 left_winseq_short = 0;

  FLOAT32 *scratch = usac_data->ptr_fft_scratch;

  ia_usac_igf_config_struct *igf_config = &usac_data->igf_config[elem_idx];
  ia_sfb_info_struct *info;
  FLAG *igf_hasmask = usac_data->igf_hasmask[elem_idx];

  chn = wn = left = chan_offset;

  if (usac_data->window_sequence[left] == EIGHT_SHORT_SEQUENCE)
  {
    left_winseq_short = 1;
  }

  if (2 == nr_core_coder_channels)
  {
    right = chan_offset + 1;
  }
  else
  {
    right = chan_offset;
  }
  info = usac_data->pstr_usac_winmap[usac_data->window_sequence[wn]];

  ia_core_coder_igf_apply_stereo(
      usac_data, chn, igf_config, left_winseq_short, (const WORD8 *)usac_data->ms_used[chn],
      igf_hasmask[wn], usac_data->pstr_sfb_info[chn]->num_groups,
      usac_data->pstr_sfb_info[chn]->group_len, usac_data->pstr_sfb_info[chn]->bins_per_sbk,
      usac_data->pstr_sfb_info[chn]->sfb_per_sbk);

  if (3 != igf_hasmask[wn])
  {
    memset(&usac_data->alpha_q_re_prev[left][igf_start_sfb], 0,
           (SFB_NUM_MAX - igf_start_sfb) * sizeof(usac_data->alpha_q_re_prev[left][0]));
    memset(&usac_data->alpha_q_im_prev[left][igf_start_sfb], 0,
           (SFB_NUM_MAX - igf_start_sfb) * sizeof(usac_data->alpha_q_im_prev[left][0]));
  }

  if (igf_hasmask[wn] == 3)
  {
    ia_core_coder_cplx_pred_upmixing(usac_data, usac_data->coef[left], usac_data->coef[right],
                                     pstr_core_coder, left, igf_pred_dir, 0, igf_start_sfb,
                                     igf_stop_sfb);

    if (igf_config->use_tnf && EIGHT_SHORT_SEQUENCE != usac_data->window_sequence[chn])
    {
      ia_core_coder_cplx_pred_upmixing(
          usac_data, usac_data->igf_dec_data[left].igf_tnf_spec_float,
          usac_data->igf_dec_data[right].igf_tnf_spec_float, pstr_core_coder, left, igf_pred_dir,
          0, igf_start_sfb, igf_stop_sfb);
    }
  }
  else if (igf_hasmask[wn])
  {
    impeghd_ms_stereo(info, (WORD8 *)usac_data->group_dis[wn], (WORD8 *)usac_data->ms_used[wn],
                      igf_start_sfb, igf_stop_sfb, usac_data->coef[right], usac_data->coef[left]);

    if (igf_config->use_tnf && EIGHT_SHORT_SEQUENCE != usac_data->window_sequence[chn])
    {
      impeghd_ms_stereo(info, (WORD8 *)usac_data->group_dis[wn], (WORD8 *)usac_data->ms_used[wn],
                        igf_start_sfb, igf_stop_sfb,
                        usac_data->igf_dec_data[right].igf_tnf_spec_float,
                        usac_data->igf_dec_data[left].igf_tnf_spec_float);
    }
  }
  for (chn = left; chn <= right; chn++)
  {
    ia_core_coder_igf_tnf(usac_data, igf_config, chn, left_winseq_short, left_winseq_short,
                          usac_data->window_sequence[chn], usac_data->coef[chn], scratch);
  }
}

/**
 *  ia_core_coder_data_process_igf_joint_stereo_after_tns
 *
 *  \brief Core coder data processing - IGF Joint stereo after TNS
 *
 *  \param [in,out]   usac_data              Poitner to USAC data structure.
 *  \param [in]       elem_idx               Element index value.
 *  \param [in]       chan_offset            Channel offset value.
 *  \param [in]       nr_core_coder_channels Number of core coder channels.
 *  \param [in,out]   pstr_core_coder        Pointer to core coder structure.
 *  \param [in]       igf_start_sfb          SFB start offset for IGF
 *  \param [in]       igf_stop_sfb           SFB stop offset for IGF
 *
 *
 *
 */
static VOID ia_core_coder_data_process_igf_joint_stereo_after_tns(
    ia_usac_data_struct *usac_data, WORD32 elem_idx, WORD32 chan_offset,
    WORD32 nr_core_coder_channels, ia_usac_tmp_core_coder_struct *pstr_core_coder,
    WORD32 igf_start_sfb, WORD32 igf_stop_sfb)
{
  WORD32 wn, chn, left, right;
  WORD32 igf_pred_dir = usac_data->igf_pred_dir[elem_idx];

  FLOAT32 *scratch = usac_data->ptr_fft_scratch;

  ia_usac_igf_config_struct *igf_config = &usac_data->igf_config[elem_idx];
  ia_sfb_info_struct *info;
  FLAG *igf_hasmask = usac_data->igf_hasmask[elem_idx];

  chn = wn = left = chan_offset;
  if (2 == nr_core_coder_channels)
  {
    right = chan_offset + 1;
  }
  else
  {
    right = chan_offset;
  }

  info = usac_data->pstr_usac_winmap[usac_data->window_sequence[wn]];
  ia_core_coder_calc_grp_offset(info, &usac_data->group_dis[wn][0]);
  ia_core_coder_igf_apply_stereo(
      usac_data, chn, igf_config,
      (usac_data->window_sequence[chn] == EIGHT_SHORT_SEQUENCE) ? 1 : 0,
      (const WORD8 *)usac_data->ms_used[chn], pstr_core_coder->ms_mask_present[0],
      info->num_groups, info->group_len, info->bins_per_sbk, info->sfb_per_sbk);

  if (3 != igf_hasmask[wn])
  {
    memset(&usac_data->alpha_q_re_prev[left][igf_start_sfb], 0,
           (SFB_NUM_MAX - igf_start_sfb) * sizeof(usac_data->alpha_q_re_prev[left][0]));
    memset(&usac_data->alpha_q_im_prev[left][igf_start_sfb], 0,
           (SFB_NUM_MAX - igf_start_sfb) * sizeof(usac_data->alpha_q_im_prev[left][0]));
  }

  if (igf_hasmask[wn] == 3)
  {
    ia_core_coder_cplx_pred_upmixing(usac_data, usac_data->coef[left], usac_data->coef[right],
                                     pstr_core_coder, left, igf_pred_dir, 0, igf_start_sfb,
                                     igf_stop_sfb);

    if (igf_config->use_tnf && EIGHT_SHORT_SEQUENCE != usac_data->window_sequence[chn])
    {
      ia_core_coder_cplx_pred_upmixing(
          usac_data, usac_data->igf_dec_data[left].igf_tnf_spec_float,
          usac_data->igf_dec_data[right].igf_tnf_spec_float, pstr_core_coder, left, igf_pred_dir,
          0, igf_start_sfb, igf_stop_sfb);
    }
  }
  else if (igf_hasmask[wn])
  {
    impeghd_ms_stereo(info, (WORD8 *)usac_data->group_dis[wn], (WORD8 *)usac_data->ms_used[wn],
                      igf_start_sfb, igf_stop_sfb, usac_data->coef[right], usac_data->coef[left]);

    if (igf_config->use_tnf && EIGHT_SHORT_SEQUENCE != usac_data->window_sequence[chn])
    {
      impeghd_ms_stereo(info, (WORD8 *)usac_data->group_dis[wn], (WORD8 *)usac_data->ms_used[wn],
                        igf_start_sfb, igf_stop_sfb,
                        usac_data->igf_dec_data[right].igf_tnf_spec_float,
                        usac_data->igf_dec_data[left].igf_tnf_spec_float);
    }
  }
  for (chn = left; chn <= right; chn++)
  {
    WORD32 igf_win;
    if (usac_data->window_sequence[chn] == EIGHT_SHORT_SEQUENCE)
    {
      igf_win = 1;
    }
    else
    {
      igf_win = 0;
    }
    ia_core_coder_igf_tnf(usac_data, igf_config, chn, igf_win, igf_win,
                          usac_data->window_sequence[chn], usac_data->coef[chn], scratch);
  }
  return;
}

/**
 *  ia_core_coder_data_process
 *
 *  \brief Process core coder data
 *
 *  \param [in,out] usac_data              Poitner to USAC data structure.
 *  \param [in]  elem_idx               Element index value.
 *  \param [in]  chan_offset            Channel offset value.
 *  \param [in]  nr_core_coder_channels Number of core coder channels.
 *  \param [in,out] pstr_core_coder        Pointer to core coder structure.
 *
 *  \return IA_ERRORCODE Error code if any.
 *
 */
IA_ERRORCODE ia_core_coder_data_process(ia_usac_data_struct *usac_data, WORD32 elem_idx,
                                        WORD32 chan_offset, WORD32 nr_core_coder_channels,
                                        ia_usac_tmp_core_coder_struct *pstr_core_coder)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 k = 0, ch = 0, chn, left = 0, right = 0;
  WORD32 igf_num_tiles = 4, igf_cnt, save_zeros = 0;
  WORD32 igf_active;
  WORD32 igf_after_tns_synth;
  WORD32 igf_independent_tiling;
  WORD32 igf_win_type;
  WORD32 igf_start_sfb = 0;
  WORD32 igf_stop_sfb = 0;
  WORD32 win_type;
  WORD32 tns_on_lr = pstr_core_coder->tns_on_lr;
  WORD32 common_window = pstr_core_coder->common_window;
  WORD32 wn;
  WORD32 left_winseq_short = 0, right_winseq_short = 0;

  UWORD8 *has_ms_mask;
  WORD32 *core_mode = pstr_core_coder->core_mode;

  FLOAT32 *scratch = usac_data->ptr_fft_scratch;

  ia_sfb_info_struct *info;
  ia_usac_igf_config_struct *igf_config = &usac_data->igf_config[elem_idx];

  igf_active = igf_config->igf_active;
  igf_after_tns_synth = igf_config->igf_after_tns_synth;
  igf_independent_tiling = igf_config->igf_independent_tiling;
  left = chan_offset;
  if (2 == nr_core_coder_channels)
  {
    right = chan_offset + 1;
  }
  else
  {
    right = chan_offset;
  }
  if (usac_data->window_sequence[left] == EIGHT_SHORT_SEQUENCE)
  {
    left_winseq_short = 1;
  }
  if (usac_data->window_sequence[right] == EIGHT_SHORT_SEQUENCE)
  {
    right_winseq_short = 1;
  }

  if (!(nr_core_coder_channels == 2 && pstr_core_coder->core_mode[0] == 0 &&
        pstr_core_coder->core_mode[1] == 0))
  {
    pstr_core_coder->common_tw = 0;
    pstr_core_coder->common_window = 0;
  }

  has_ms_mask = pstr_core_coder->ms_mask_present;

  for (k = 0; k < 2; k++)
  {
    if (k != tns_on_lr)
    {
      if (core_mode[0] == CORE_MODE_FD && core_mode[1] == CORE_MODE_FD &&
          nr_core_coder_channels == 2 && common_window == 1 &&
          (((chan_offset + k) == left) || (!tns_on_lr)))
      {
        /* stereo processing */
        wn = left;
        info = usac_data->pstr_usac_winmap[usac_data->window_sequence[wn]];
        if (0 == info->islong)
        {
          igf_win_type = 1;
        }
        else
        {
          igf_win_type = 0;
        }
        igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;
        igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;
        ia_core_coder_calc_grp_offset(info, &usac_data->group_dis[wn][0]);

        if (3 != pstr_core_coder->ms_mask_present[wn - left])
        {
          memset(usac_data->alpha_q_re_prev[left], 0,
                 pstr_core_coder->max_sfb_ste_clear *
                     sizeof(usac_data->alpha_q_re_prev[left][0]));
          memset(usac_data->alpha_q_im_prev[left], 0,
                 pstr_core_coder->max_sfb_ste_clear *
                     sizeof(usac_data->alpha_q_im_prev[left][0]));
        }

        if (igf_active)
        {
          if (0 == usac_data->pstr_sfb_info[wn]->islong)
          {
            igf_num_tiles = igf_config->igf_grid[1].igf_num_tiles;
          }
          else
          {
            igf_num_tiles = igf_config->igf_grid[0].igf_num_tiles;
          }
        }

        if (3 == has_ms_mask[wn - left])
        {
          if (igf_active)
          {
            for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
            {
              ia_core_coder_cplx_pred_upmixing(
                  usac_data, &usac_data->igf_dec_data[left].igf_input_spec[MAX_IGF_LEN * igf_cnt],
                  &usac_data->igf_dec_data[right].igf_input_spec[MAX_IGF_LEN * igf_cnt],
                  pstr_core_coder, left, pstr_core_coder->pred_dir, pstr_core_coder->complex_coef,
                  0, pstr_core_coder->max_sfb_ste);
            }
          }

          ia_core_coder_cplx_pred_upmixing(
              usac_data, usac_data->coef[left], usac_data->coef[right], pstr_core_coder, left,
              pstr_core_coder->pred_dir, pstr_core_coder->complex_coef, 0,
              pstr_core_coder->max_sfb_ste);
        }
        else if (has_ms_mask[wn - left])
        {
          if (igf_active)
          {
            if (igf_start_sfb > info->sfb_per_sbk)
            {
              return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_SFB_INDEX;
            }
            for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
            {
              impeghd_ms_stereo(
                  info, (WORD8 *)usac_data->group_dis[wn], (WORD8 *)usac_data->ms_used[wn], 0,
                  igf_start_sfb,
                  &usac_data->igf_dec_data[right].igf_input_spec[MAX_IGF_LEN * igf_cnt],
                  &usac_data->igf_dec_data[left].igf_input_spec[MAX_IGF_LEN * igf_cnt]);
            }
          }

          if (pstr_core_coder->max_sfb_ste > info->sfb_per_sbk)
          {
            pstr_core_coder->max_sfb_ste = 0;
            return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_SFB_INDEX;
          }
          impeghd_ms_stereo(info, (WORD8 *)usac_data->group_dis[wn],
                            (WORD8 *)usac_data->ms_used[wn], 0, pstr_core_coder->max_sfb_ste,
                            usac_data->coef[right], usac_data->coef[left]);
        }

        /* IGF joint stereo before TNS */
        if (igf_independent_tiling == 0 && igf_active && !igf_after_tns_synth)
        {
          ia_core_coder_data_process_igf_joint_stereo_before_tns(
              usac_data, elem_idx, chan_offset, nr_core_coder_channels, pstr_core_coder,
              igf_start_sfb, igf_stop_sfb);
        }
      }
    }
    else
    { /* TNS processing before stereo */
      for (ch = 0, chn = left; chn <= right; ch++, chn++)
      {
        if (core_mode[ch] == CORE_MODE_FD)
        {

          /* if (!(mip->ch_info[ch].present)) continue; */
          wn = chn;
          info = usac_data->pstr_usac_winmap[usac_data->window_sequence[wn]];

          ia_core_coder_calc_grp_offset(info, &usac_data->group_dis[wn][0]);

          if (igf_active)
          {
            if (0 == info->islong)
            {
              igf_num_tiles = igf_config->igf_grid[1].igf_num_tiles;
            }
            else
            {
              igf_num_tiles = igf_config->igf_grid[0].igf_num_tiles;
            }
          }

          /* IGF mono before TNS */
          if (igf_active && (!igf_after_tns_synth) &&
              (igf_independent_tiling == 1 || common_window == 0))
          {
            if (EIGHT_SHORT_SEQUENCE == usac_data->window_sequence[chn])
            {
              win_type = 1;
            }
            else
            {
              win_type = 0;
            }
            ia_core_coder_igf_mono(usac_data, igf_config, chn, win_type, win_type,
                                   info->num_groups, info->group_len, info->bins_per_sbk,
                                   usac_data->window_sequence[chn], usac_data->coef[chn],
                                   scratch);

            ia_core_coder_igf_tnf(usac_data, igf_config, chn, win_type, win_type,
                                  usac_data->window_sequence[chn], usac_data->coef[chn], scratch);
          }

          if (igf_active && igf_after_tns_synth)
          {
            for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
            {
              err_code = ia_core_coder_tns_apply(
                  usac_data, &usac_data->igf_dec_data[chn].igf_input_spec[MAX_IGF_LEN * igf_cnt],
                  pstr_core_coder->max_sfb[ch], usac_data->pstr_sfb_info[chn],
                  usac_data->pstr_tns[chn], igf_config);

              if (err_code != IA_MPEGH_DEC_NO_ERROR)
              {
                return err_code;
              }
            }
          }

          err_code = ia_core_coder_tns_apply(
              usac_data, usac_data->coef[chn], pstr_core_coder->max_sfb[ch],
              usac_data->pstr_sfb_info[chn], usac_data->pstr_tns[chn], igf_config);
          if (err_code != IA_MPEGH_DEC_NO_ERROR)
          {
            return err_code;
          }

          /* IGF mono after TNS */
          if (igf_active && igf_after_tns_synth &&
              (igf_independent_tiling == 1 || pstr_core_coder->common_window == 0))
          {
            if (usac_data->window_sequence[chn] == EIGHT_SHORT_SEQUENCE)
            {
              win_type = 1;
            }
            else
            {
              win_type = 0;
            }
            ia_core_coder_igf_mono(usac_data, igf_config, chn, win_type, win_type,
                                   info->num_groups, info->group_len, info->bins_per_sbk,
                                   usac_data->window_sequence[chn], usac_data->coef[chn],
                                   scratch);

            ia_core_coder_igf_tnf(usac_data, igf_config, chn, win_type, win_type,
                                  usac_data->window_sequence[chn], usac_data->coef[chn], scratch);
          }
        }
      }
    }
  }

  /* IGF joint stereo after TNS */
  if (igf_independent_tiling == 0 && igf_active && igf_after_tns_synth && common_window == 1)
  {
    ia_core_coder_data_process_igf_joint_stereo_after_tns(usac_data, elem_idx, chan_offset,
                                                          nr_core_coder_channels, pstr_core_coder,
                                                          igf_start_sfb, igf_stop_sfb);
  }

  if (core_mode[0] == CORE_MODE_FD && core_mode[1] == CORE_MODE_FD && nr_core_coder_channels == 2)
  {
    save_zeros = ((left_winseq_short && !right_winseq_short) ||
                  (!left_winseq_short && right_winseq_short));

    ia_core_coder_usac_cplx_save_prev(usac_data->pstr_sfb_info[left], usac_data->coef[left],
                                      usac_data->coef[right], usac_data->coef_save_float[left],
                                      usac_data->coef_save_float[right], save_zeros);
  }

  for (ch = left; ch <= right; ch++)
  {
    if (pstr_core_coder->core_mode[ch - left] == CORE_MODE_FD)
    {
      {
        err_code = ia_core_coder_fd_frm_dec(usac_data, ch, elem_idx);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return err_code;
        }

        ia_core_coder_usac_ltpf_process(&usac_data->str_tddec[ch]->ltpf_data,
                                        &usac_data->time_sample_vector[ch][0]);
      }

      usac_data->window_sequence_last[ch] = usac_data->window_sequence[ch];
      usac_data->window_shape_prev[ch] = usac_data->window_shape[ch];
    }
  }

  for (ch = 0, chn = left; chn <= right; chn++, ch++)
    usac_data->td_frame_prev[chn] = pstr_core_coder->core_mode[ch];

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */
