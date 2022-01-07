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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <impeghd_type_def.h>
#include "impeghd_hoa_common_values.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impd_drc_common.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_rom.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_dmx_mtx_data.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup DmxMtxProc Downmix Matrix processing
 * @ingroup  DmxMtxProc
 * @brief Downmix Matrix processing
 *
 * @{
 */

/**
 *  impeghd_read_range
 *
 *  \brief Read range value
 *
 *  \param [in,out] ptr_bit_buf Pointer to bit buffer structure
 *  \param [in]     alphabet_sz Size of alphabet
 *
 *  \return UWORD32 Read range value
 *
 */
static UWORD32 impeghd_read_range(ia_bit_buf_struct *ptr_bit_buf, UWORD32 alphabet_sz)
{
  WORD32 n, n_bits, temp;
  n = alphabet_sz;
  WORD32 log_2 = 0;
  while (n > 1) // Calculate log to base 2 value
  {
    n >>= 1;
    ++log_2;
  }
  n_bits = log_2;
  UWORD32 n_unused = (1U << (n_bits + 1)) - alphabet_sz;
  UWORD32 val = ia_core_coder_read_bits_buf(ptr_bit_buf, n_bits);

  if (val >= n_unused)
  {
    temp = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    val = (val << 1) - n_unused + temp;
  }
  return val;
}

/**
 *  impeghd_decode_gain
 *
 *  \brief Decode gain value
 *
 *  \param [in,out] ptr_bit_buf Pointer to bit buffer structure
 *  \param [in]     coder       Pointer to gain coder structure
 *
 *  \return FLOAT32 Decoded gain value
 *
 */
static FLOAT32 impeghd_decode_gain(ia_bit_buf_struct *ptr_bit_buf, ia_dmx_mtx_gain_coder_t *coder)
{
  FLOAT32 gain = IA_DMX_MTX_GAIN_ZERO;
  WORD32 gain_idx;

  if (coder->raw_code_non_zeros)
  {
    WORD32 alphabet_sz = ((coder->max_gain - coder->min_gain) << coder->precision_level) + 2;
    gain_idx = impeghd_read_range(ptr_bit_buf, alphabet_sz);
    gain = coder->max_gain - gain_idx / (FLOAT32)(1 << coder->precision_level);
  }
  else
  {
    WORD32 max_head = (coder->gain_tab_size - 1) >> coder->gain_lg_param;
    WORD32 head = 0;
    gain_idx = -1;

    while (head < max_head)
    {
      if (0 != ia_core_coder_read_bits_buf(ptr_bit_buf, 1))
        break;
      head++;
    }

    gain_idx = (head << coder->gain_lg_param) +
               ia_core_coder_read_bits_buf(ptr_bit_buf, coder->gain_lg_param);
    gain = coder->gain_tab[gain_idx];
  }

  if (gain < coder->min_gain)
  {
    gain = IA_DMX_MTX_GAIN_ZERO;
  }
  return gain;
}

/**
 *  impeghd_find_comp_tmplt
 *
 *  \brief Select compact template table
 *
 *  \param [in]   cicp_in   Input CICP index
 *  \param [in]   cicp_out  Outout CICP index
 *  \param [out]  tmplt     Compact template table pointer
 *
 *  \return Error if any
 *
 */
static IA_ERRORCODE impeghd_find_comp_tmplt(WORD32 cicp_in, WORD32 cicp_out, WORD32 *tmplt)
{
  WORD32 idx;
  tmplt = NULL;

  if (cicp_in == -1 || cicp_out == -1)
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_CICP_IDX;
  }

  for (idx = 0; idx < 8; idx++)
  {
    if (ia_comp_tmplt_inp_idx[idx] != cicp_in)
    {
      continue;
    }
    if (ia_comp_tmplt_out_idx[idx] != cicp_out)
    {
      continue;
    }
    tmplt = (WORD32 *)ia_comp_tmplt_data[idx];
    return IA_MPEGH_DEC_NO_ERROR;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_convert_to_compact_config
 *
 *  \brief Conversion to compact configuration
 *
 *  \param [in]   num_conf        Number of spekaer configs
 *  \param [in]   ptr_conf        Pointer to speaker configs
 *  \param [out]  num_comp_conf   Number of compact configs
 *  \param [out]  ptr_conf_comp   Pointer to compact configs
 *
 *  \return Error if any
 *
 */
static IA_ERRORCODE impeghd_convert_to_compact_config(WORD32 num_conf,
                                                      ia_speaker_info_str *ptr_conf,
                                                      WORD32 *num_comp_conf,
                                                      ia_speaker_info_str *ptr_conf_comp[])
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i, j;

  for (i = 0; i < num_conf; i++)
  {
    ptr_conf[i].is_already_used = 0;
    ptr_conf[i].original_position = i;
    ptr_conf[i].symmetric_pair = NULL;
  }

  *num_comp_conf = 0;

  for (i = 0; i < num_conf; i++)
  {
    if (1 == ptr_conf[i].is_already_used)
      continue;

    if (ptr_conf[i].azimuth == 0 || ptr_conf[i].azimuth == 180 || ptr_conf[i].azimuth == -180)
    {
      ptr_conf_comp[*num_comp_conf] = &(ptr_conf[i]);
      (*num_comp_conf)++;
      ptr_conf[i].symmetric_pair = NULL;
      ptr_conf[i].pair_type = DMX_SPK_CENTRE;
      ptr_conf[i].is_already_used = 1;
    }
    else
    {
      for (j = i + 1; j < num_conf; j++)
      {
        if (ptr_conf[j].is_already_used)
          continue;

        if ((ptr_conf[i].is_lfe == ptr_conf[j].is_lfe) &&
            (ptr_conf[i].elevation == ptr_conf[j].elevation) &&
            (ptr_conf[i].azimuth == -ptr_conf[j].azimuth))
        {
          if (ptr_conf[i].azimuth > 0)
          {
            ptr_conf_comp[*num_comp_conf] = &(ptr_conf[i]);
            (*num_comp_conf)++;
            ptr_conf[i].symmetric_pair = &(ptr_conf[j]);
            ptr_conf[i].pair_type = DMX_SPK_SYMMETRIC;
            ptr_conf[j].symmetric_pair = NULL;
            ptr_conf[j].pair_type = DMX_SPK_NONE;
          }
          else
          {
            ptr_conf_comp[*num_comp_conf] = &(ptr_conf[j]);
            (*num_comp_conf)++;
            ptr_conf[j].symmetric_pair = &(ptr_conf[i]);
            ptr_conf[j].pair_type = DMX_SPK_SYMMETRIC;
            ptr_conf[i].symmetric_pair = NULL;
            ptr_conf[i].pair_type = DMX_SPK_NONE;
          }

          ptr_conf[i].is_already_used = 1;
          ptr_conf[j].is_already_used = 1;
          break;
        }
      }

      if (0 == ptr_conf[i].is_already_used)
      {
        ptr_conf_comp[*num_comp_conf] = &(ptr_conf[i]);
        (*num_comp_conf)++;
        ptr_conf[i].symmetric_pair = NULL;
        ptr_conf[i].pair_type = DMX_SPK_SINGLE;
        ptr_conf[i].is_already_used = 1;
      }
    }
  }
  return err_code;
}

/**
 *  impeghd_decode_flat_compact_mtx
 *
 *  \brief Decode flat compact matrix
 *
 *  \param [in,out]   ptr_bit_buf Pointer to bit buffer structure
 *  \param [out]      mtx         Pointer to matrix data
 *  \param [in]       cnt         Total count of configs
 *
 *
 *
 */
static VOID impeghd_decode_flat_compact_mtx(ia_bit_buf_struct *ptr_bit_buf, WORD32 *mtx,
                                            WORD32 cnt)
{
  WORD32 n_bits = (cnt >= 256) ? 4 : 3;
  WORD32 lpg_param, head, max_head, val;
  WORD32 mtx_idx = 0;

  lpg_param = ia_core_coder_read_bits_buf(ptr_bit_buf, n_bits);
  max_head = cnt >> lpg_param;
  do
  {
    head = 0;

    while (head < max_head)
    {
      if (0 == ia_core_coder_read_bits_buf(ptr_bit_buf, 1))
      {
        break;
      }
      head++;
    }
    val = (head << lpg_param) + ia_core_coder_read_bits_buf(ptr_bit_buf, lpg_param);
    memset(&mtx[mtx_idx], 0, val * sizeof(mtx[0]));
    mtx_idx += val;

    if (mtx_idx < cnt)
    {
      mtx[mtx_idx] = 1;
      mtx_idx++;
    }
  } while (mtx_idx < cnt);
}

/**
 *  impeghd_decode_eq_config
 *
 *  \brief Decode Equalizer configuration
 *
 *  \param [in,out]   ptr_bit_buf Pointer to bit buffer structure
 *  \param [out]      ptr_eq_conf Pointer to equalizer config structure
 *  \param [in]       num_in_ch   Number of input channels
 *  \param [in]       ptr_scratch Pointer to scratch buffer
 *
 *  \return Error if any
 *
 */
static IA_ERRORCODE impeghd_decode_eq_config(ia_bit_buf_struct *ptr_bit_buf,
                                             ia_eq_config *ptr_eq_conf, WORD32 num_in_ch,
                                             pWORD8 ptr_scratch)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 eq_prec_lvl, sg_prec_lvl, eq_extn_rng;
  WORD32 eq, filt, freq, ch;

  ia_core_coder_read_escape_value(ptr_bit_buf, &ptr_eq_conf->num_eq, 3, 5, 0);
  ptr_eq_conf->num_eq += 1;
  ptr_eq_conf->eq_params = (ia_eq_params *)ptr_scratch;
  ptr_scratch += ptr_eq_conf->num_eq * sizeof(ia_eq_params);

  eq_prec_lvl = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
  sg_prec_lvl = ia_min_int(eq_prec_lvl + 1, 3);
  eq_extn_rng = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  for (eq = 0; eq < (WORD32)ptr_eq_conf->num_eq; eq++)
  {
    WORD32 last_cntr_freq_p10 = 0;
    WORD32 last_cntr_freq_ld2 = 10;
    WORD32 max_cntr_freq_ld2 = 99;
    WORD32 scl_gain_idx;
    FLOAT32 scl_gain;

    ia_core_coder_read_escape_value(ptr_bit_buf, &ptr_eq_conf->eq_params[eq].num_pk_filt, 2, 4,
                                    0);
    ptr_eq_conf->eq_params[eq].num_pk_filt += 1;
    ptr_eq_conf->eq_params[eq].pk_filt = (ia_eq_pk_filt *)ptr_scratch;
    ptr_scratch += ptr_eq_conf->eq_params[eq].num_pk_filt * sizeof(ia_eq_pk_filt);

    for (filt = 0; filt < (WORD32)ptr_eq_conf->eq_params[eq].num_pk_filt; filt++)
    {
      WORD32 cntr_freq, cntr_freq_p10, cntr_freq_ld2, q_fac_idx, cntr_gain_idx;
      FLOAT32 quality_fac, cntr_gain;

      /* Frequency */
      cntr_freq_p10 =
          impeghd_read_range(ptr_bit_buf, 4 - last_cntr_freq_p10) + last_cntr_freq_p10;
      if (cntr_freq_p10 > last_cntr_freq_p10)
      {
        last_cntr_freq_ld2 = 10;
      }
      if (cntr_freq_p10 == 3)
      {
        max_cntr_freq_ld2 = 24;
      }
      cntr_freq_ld2 =
          impeghd_read_range(ptr_bit_buf, 1 + max_cntr_freq_ld2 - last_cntr_freq_ld2) +
          last_cntr_freq_ld2;
      cntr_freq = cntr_freq_ld2;
      for (freq = 0; freq < cntr_freq_p10; freq++)
      {
        cntr_freq = cntr_freq * 10;
      }
      last_cntr_freq_p10 = cntr_freq_p10;
      last_cntr_freq_ld2 = cntr_freq_ld2;
      ptr_eq_conf->eq_params[eq].pk_filt[filt].freq = (FLOAT32)cntr_freq;

      /* Quality factor */
      q_fac_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 5);
      if (q_fac_idx <= 19)
      {
        quality_fac = ia_mul_flt(0.05f, (FLOAT32)(q_fac_idx + 1));
      }
      else
      {
        WORD32 q_fac_ext = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
        quality_fac =
            ia_add_flt(1.0f, ia_mul_flt(0.1f, (FLOAT32)((q_fac_idx - 20) * 8 + q_fac_ext + 1)));
      }
      ptr_eq_conf->eq_params[eq].pk_filt[filt].q_fac = quality_fac;

      /* Centre gain */
      cntr_gain_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 4 + eq_extn_rng + eq_prec_lvl);
      cntr_gain = ia_eq_min_ranges[eq_extn_rng][eq_prec_lvl] +
                  ia_eq_precisions[eq_prec_lvl] * cntr_gain_idx;
      ptr_eq_conf->eq_params[eq].pk_filt[filt].gain = cntr_gain;
    }

    /* Scaling gain*/
    scl_gain_idx = ia_core_coder_read_bits_buf(ptr_bit_buf, 4 + eq_extn_rng + sg_prec_lvl);
    scl_gain =
        ia_eq_min_ranges[eq_extn_rng][sg_prec_lvl] + ia_eq_precisions[sg_prec_lvl] * scl_gain_idx;
    ptr_eq_conf->eq_params[eq].global_gain = scl_gain;
  }

  ptr_eq_conf->eq_map = (WORD32 *)ptr_scratch;

  for (ch = 0; ch < num_in_ch; ch++)
  {
    if (1 == ia_core_coder_read_bits_buf(ptr_bit_buf, 1))
    {
      ptr_eq_conf->eq_map[ch] = impeghd_read_range(ptr_bit_buf, ptr_eq_conf->num_eq) + 1;
    }
    else
    {
      ptr_eq_conf->eq_map[ch] = 0;
    }
  }

  return err_code;
}

/**
 *  impeghd_peak_filter
 *
 *  \brief Calculate peak filter value
 *
 *  \param [in]   filt  Pointer to PK filter structure
 *  \param [in]   gain  Global Gain value
 *  \param [in]   freq  Centre frequency
 *
 *  \return peak filter value
 *
 */
static FLOAT32 impeghd_peak_filter(ia_eq_pk_filt *filt, FLOAT32 gain, FLOAT32 freq)
{
  FLOAT32 val, gain_lin;
  FLOAT32 frq_p_4, pk_frq_p_4, frq_pk_frq_sq, q_fac_sq;
  FLOAT32 tmp1, tmp2;

  gain_lin = (FLOAT32)pow(10.0f, (FLOAT32)fabs(filt->gain) / 20.f);
  gain_lin = ia_mul_flt(gain_lin, gain_lin);

  frq_p_4 = (FLOAT32)pow(freq, 4);
  pk_frq_p_4 = (FLOAT32)pow(filt->freq, 4);
  q_fac_sq = ia_mul_flt(filt->q_fac, filt->q_fac);
  frq_pk_frq_sq = ia_mul_flt(ia_mul_flt(filt->freq, filt->freq), ia_mul_flt(freq, freq));
  tmp1 = frq_p_4 + (1.0f / q_fac_sq - 2.0f) * frq_pk_frq_sq + pk_frq_p_4;
  tmp2 = frq_p_4 + (gain_lin / q_fac_sq - 2.0f) * frq_pk_frq_sq + pk_frq_p_4;

  if (ia_lt_flt(filt->gain, 0))
  {
    val = tmp1 / tmp2;
  }
  else
  {
    val = tmp2 / tmp1;
  }

  return (FLOAT32)ia_sqrt_flt(val) * (FLOAT32)pow(10.0f, gain / 20.0f);
}

/**
 *  impeghd_generate_gain_table
 *
 *  \brief Generate gain table
 *
 *  \param [in,out]   gain_coder  Pointer to gain coder structure
 *
 *  \return Error if any
 *
 */
static IA_ERRORCODE impeghd_generate_gain_table(ia_dmx_mtx_gain_coder_t *gain_coder)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 idx, prc, tab_sz = 0;
  FLOAT32 s, step, prev_step;

  if (gain_coder->precision_level > 2)
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_PRECISION_LVL;
  }

  for (idx = 0; idx >= gain_coder->min_gain; idx -= 3)
  {
    gain_coder->gain_tab[tab_sz] = (FLOAT32)idx;
    tab_sz++;
  }

  for (idx = 3; idx <= gain_coder->max_gain; idx += 3)
  {
    gain_coder->gain_tab[tab_sz] = (FLOAT32)idx;
    tab_sz++;
  }

  for (idx = 0; idx >= gain_coder->min_gain; --idx)
  {
    if (idx % 3 != 0)
    {
      gain_coder->gain_tab[tab_sz] = (FLOAT32)idx;
      tab_sz++;
    }
  }

  for (idx = 1; idx <= gain_coder->max_gain; ++idx)
  {
    if (idx % 3 != 0)
    {
      gain_coder->gain_tab[tab_sz] = (FLOAT32)idx;
      tab_sz++;
    }
  }

  prev_step = 1.0f;
  for (prc = 1; prc <= gain_coder->precision_level; ++prc)
  {
    step = 1.0f / (FLOAT32)(1 << prc);

    for (s = 0; s >= gain_coder->min_gain; s -= step)
    {
      if (fmod(s, prev_step) != 0.0f)
      {
        gain_coder->gain_tab[tab_sz] = s;
        tab_sz++;
      }
    }
    for (s = step; s <= gain_coder->max_gain; s += step)
    {
      if (fmod(s, prev_step) != 0.0f)
      {
        gain_coder->gain_tab[tab_sz] = s;
        tab_sz++;
      }
    }

    prev_step = step;
  }

  gain_coder->gain_tab[tab_sz] = IA_DMX_MTX_GAIN_ZERO;
  tab_sz++;
  gain_coder->gain_tab_size = tab_sz;

  if (tab_sz !=
      ((gain_coder->max_gain - gain_coder->min_gain) << gain_coder->precision_level) + 2)
  {
    err_code = IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_GAIN_TABLE_SIZE;
  }
  return err_code;
}

/**
 *  impeghd_decode_downmix_matrix
 *
 *  \brief Parse, decode Downmix matrix config and Equalizer config. Update dowmix matrix.
 *
 *  \param [in,out]   fc_params   Pointer to format converter params structure
 *  \param [in,out]   params      Pointer to dowmix matrix params structure
 *  \param [in,out]   ptr_bit_buf Pointer to bit buffer structure
 *  \param [in]       ptr_scratch Pointer to scratch buffer
 *
 *  \return Error if any
 *
 */
IA_ERRORCODE impeghd_decode_downmix_matrix(ia_format_conv_param *fc_params,
                                           ia_dmx_mtx_params *params,
                                           ia_bit_buf_struct *ptr_bit_buf, pWORD8 ptr_scratch)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 equalizer_present;
  WORD32 precision_level;
  WORD32 max_gain, min_gain;
  WORD32 ch, i, j;
  WORD32 num_in_compact, num_out_compact;
  WORD32 is_all_seperable;
  WORD32 is_all_symmetric;
  WORD32 mix_lfe_to_lfe;
  WORD32 raw_code_comp_mtx;
  WORD32 full_assym_inp;
  WORD32 raw_code_non_zeros;
  WORD32 idx;
  WORD32 num_in_ch = params->num_in_ch;
  WORD32 num_out_ch = params->num_out_ch;

  WORD32 *comp_tmplt = NULL;

  ia_dmx_mtx_scr_t *pscr = (ia_dmx_mtx_scr_t *)ptr_scratch;
  ptr_scratch += sizeof(*pscr);

  memset(pscr->is_seperable, 0, MAX_NUM_SPEAKERS * sizeof(pscr->is_seperable[0]));
  memset(pscr->is_symmetric, 0, MAX_NUM_SPEAKERS * sizeof(pscr->is_symmetric[0]));
  memset(pscr->comp_dmx_mtx, 0,
         MAX_NUM_SPEAKERS * MAX_NUM_SPEAKERS * sizeof(pscr->comp_dmx_mtx[0][0]));

  if (params->spk_layout->spk_layout_type == 0)
  {
    WORD32 spk_idx = params->spk_layout->cicp_spk_layout_idx;
    WORD32 ch_cnt = impgehd_cicp_get_num_ls[spk_idx];
    const WORD32 *speaker_table = ia_cicp_idx_ls_set_map_tbl[spk_idx];
    for (ch = 0; ch < ch_cnt; ch++)
    {
      idx = speaker_table[ch];
      pscr->inp_spk_conf[ch].azimuth = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_azimuth;
      pscr->inp_spk_conf[ch].elevation = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_elevation;
      pscr->inp_spk_conf[ch].is_lfe = ia_cicp_ls_geo_tbls[idx].lfe_flag;
    }
  }
  else if (params->spk_layout->spk_layout_type == 1)
  {
    WORD32 ch_cnt = params->spk_layout->num_speakers;
    for (ch = 0; ch < ch_cnt; ch++)
    {
      idx = params->spk_layout->cicp_spk_idx[ch];
      pscr->inp_spk_conf[ch].azimuth = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_azimuth;
      pscr->inp_spk_conf[ch].elevation = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_elevation;
      pscr->inp_spk_conf[ch].is_lfe = ia_cicp_ls_geo_tbls[idx].lfe_flag;
    }
  }
  else if (params->spk_layout->spk_layout_type == 2)
  {
    /* Flexible speaker configuration */
  }
  else
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_UNSUPPORTED_SPK_LAYOUT;
  }

  {
    WORD32 spk_idx = params->cicp_out_idx;
    const WORD32 *speaker_table = ia_cicp_idx_ls_set_map_tbl[spk_idx];
    for (ch = 0; ch < num_out_ch; ch++)
    {
      idx = speaker_table[ch];
      pscr->out_spk_conf[ch].azimuth = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_azimuth;
      pscr->out_spk_conf[ch].elevation = (WORD16)ia_cicp_ls_geo_tbls[idx].ls_elevation;
      pscr->out_spk_conf[ch].is_lfe = ia_cicp_ls_geo_tbls[idx].lfe_flag;
    }
  }

  equalizer_present = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (1 == equalizer_present)
  {
    err_code = impeghd_decode_eq_config(ptr_bit_buf, &pscr->eq_conf, num_in_ch, ptr_scratch);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return err_code;
    }
  }

  precision_level = ia_core_coder_read_bits_buf(ptr_bit_buf, 2);
  ia_core_coder_read_escape_value(ptr_bit_buf, (UWORD32 *)&max_gain, 3, 4, 0);
  ia_core_coder_read_escape_value(ptr_bit_buf, (UWORD32 *)&min_gain, 4, 5, 0);
  min_gain = -(min_gain + 1);

  impeghd_convert_to_compact_config(num_in_ch, pscr->inp_spk_conf, &num_in_compact,
                                    pscr->ptr_inp_spk_comp_conf);
  impeghd_convert_to_compact_config(num_out_ch, pscr->out_spk_conf, &num_out_compact,
                                    pscr->ptr_out_spk_comp_conf);

  is_all_seperable = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (0 == is_all_seperable)
  {
    for (ch = 0; ch < num_out_compact; ch++)
    {
      if (pscr->ptr_out_spk_comp_conf[ch]->pair_type == DMX_SPK_SYMMETRIC)
      {
        pscr->is_seperable[ch] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
      }
    }
  }
  else
  {
    for (ch = 0; ch < num_out_compact; ch++)
    {
      if (pscr->ptr_out_spk_comp_conf[ch]->pair_type == DMX_SPK_SYMMETRIC)
      {
        pscr->is_seperable[ch] = 1;
      }
    }
  }

  is_all_symmetric = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (0 == is_all_symmetric)
  {
    for (ch = 0; ch < num_out_compact; ch++)
    {
      pscr->is_symmetric[ch] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    }
  }
  else
  {
    for (ch = 0; ch < num_out_compact; ch++)
    {
      pscr->is_symmetric[ch] = 1;
    }
  }

  mix_lfe_to_lfe = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  raw_code_comp_mtx = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  if (1 == raw_code_comp_mtx)
  {
    for (i = 0; i < num_in_compact; i++)
    {
      for (j = 0; j < num_out_compact; j++)
      {
        if (0 == mix_lfe_to_lfe ||
            (pscr->ptr_inp_spk_comp_conf[i]->is_lfe == pscr->ptr_out_spk_comp_conf[j]->is_lfe))
        {
          pscr->comp_dmx_mtx[i][j] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }
        else
        {
          pscr->comp_dmx_mtx[i][j] = 0;
        }
      }
    }
  }
  else
  {
    WORD32 use_comp_tmplt = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
    WORD32 total_cnt, cnt;

    if (mix_lfe_to_lfe)
    {
      WORD32 num_in_lfe_compact = 0, num_out_lfe_compact = 0;
      for (ch = 0; ch < num_in_compact; ch++)
      {
        if (pscr->ptr_inp_spk_comp_conf[ch]->is_lfe == 1)
          num_in_lfe_compact++;
      }
      for (ch = 0; ch < num_out_compact; ch++)
      {
        if (pscr->ptr_out_spk_comp_conf[ch]->is_lfe == 1)
          num_out_lfe_compact++;
      }
      total_cnt = (num_in_compact - num_in_lfe_compact) * (num_out_compact - num_out_lfe_compact);
    }
    else
    {
      total_cnt = num_in_compact * num_out_compact;
    }

    if (use_comp_tmplt)
    {
      if ((params->cicp_in_idx == -1) || (params->cicp_out_idx == -1))
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_CICP_IDX;
      }
      impeghd_find_comp_tmplt(params->cicp_in_idx, params->cicp_out_idx, comp_tmplt);
      if (comp_tmplt == NULL)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_NO_COMP_TMPLT;
      }
    }

    impeghd_decode_flat_compact_mtx(ptr_bit_buf, pscr->flat_comp_mtx, total_cnt);

    cnt = 0;
    for (i = 0; i < num_in_compact; i++)
    {
      for (j = 0; j < num_out_compact; j++)
      {
        if (mix_lfe_to_lfe && pscr->ptr_inp_spk_comp_conf[i]->is_lfe &&
            pscr->ptr_out_spk_comp_conf[j]->is_lfe)
        {
          pscr->comp_dmx_mtx[i][j] = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
        }
        else if (mix_lfe_to_lfe && (pscr->ptr_inp_spk_comp_conf[i]->is_lfe ^
                                    pscr->ptr_out_spk_comp_conf[j]->is_lfe))
        {
          pscr->comp_dmx_mtx[i][j] = 0;
        }
        else
        {
          pscr->comp_dmx_mtx[i][j] = pscr->flat_comp_mtx[cnt++];
          if (use_comp_tmplt)
          {
            pscr->comp_dmx_mtx[i][j] ^= comp_tmplt[i * num_out_compact + j];
          }
        }
      }
    }
    if (cnt != total_cnt)
    {
      return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_NUM_CONFS;
    }
  }

  full_assym_inp = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);
  raw_code_non_zeros = ia_core_coder_read_bits_buf(ptr_bit_buf, 1);

  pscr->gain_coder.min_gain = min_gain;
  pscr->gain_coder.max_gain = max_gain;
  pscr->gain_coder.precision_level = precision_level;
  pscr->gain_coder.raw_code_non_zeros = raw_code_non_zeros;
  pscr->gain_coder.gain_lg_param = -1;
  pscr->gain_coder.history_cnt = 0;
  pscr->gain_coder.gain_tab_size = 0;

  if (0 == raw_code_non_zeros)
  {
    pscr->gain_coder.gain_lg_param = ia_core_coder_read_bits_buf(ptr_bit_buf, 3);
    impeghd_generate_gain_table(&pscr->gain_coder);
  }

  for (i = 0; i < num_in_compact; i++)
  {
    WORD32 in_type = pscr->ptr_inp_spk_comp_conf[i]->pair_type;
    assert(in_type != DMX_SPK_NONE);

    for (j = 0; j < num_out_compact; j++)
    {
      WORD32 out_type = pscr->ptr_out_spk_comp_conf[j]->pair_type;
      WORD32 in_org_pos = pscr->ptr_inp_spk_comp_conf[i]->original_position;
      WORD32 out_org_pos = pscr->ptr_out_spk_comp_conf[j]->original_position;
      assert(out_type != DMX_SPK_NONE);

      if (in_type != DMX_SPK_SYMMETRIC && out_type != DMX_SPK_SYMMETRIC)
      {
        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] = IA_DMX_MTX_GAIN_ZERO;
        if (0 == pscr->comp_dmx_mtx[i][j])
          continue;

        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] =
            impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
      }
      else if (in_type != DMX_SPK_SYMMETRIC)
      {
        WORD32 out_org_pos_sym =
            pscr->ptr_out_spk_comp_conf[j]->symmetric_pair->original_position;
        WORD32 use_full = (in_type == DMX_SPK_SINGLE) && full_assym_inp;
        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] = IA_DMX_MTX_GAIN_ZERO;
        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym] = IA_DMX_MTX_GAIN_ZERO;
        if (0 == pscr->comp_dmx_mtx[i][j])
          continue;

        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] =
            impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        if (pscr->is_symmetric[j] && !use_full)
        {
          params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym] =
              params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos];
        }
        else
        {
          params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        }
      }
      else if (out_type != DMX_SPK_SYMMETRIC)
      {
        WORD32 in_org_pos_sym = pscr->ptr_inp_spk_comp_conf[i]->symmetric_pair->original_position;
        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] = IA_DMX_MTX_GAIN_ZERO;
        params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos] = IA_DMX_MTX_GAIN_ZERO;
        if (0 == pscr->comp_dmx_mtx[i][j])
          continue;

        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] =
            impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        if (pscr->is_symmetric[j])
        {
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos] =
              params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos];
        }
        else
        {
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        }
      }
      else
      {
        WORD32 in_org_pos_sym = pscr->ptr_inp_spk_comp_conf[i]->symmetric_pair->original_position;
        WORD32 out_org_pos_sym =
            pscr->ptr_out_spk_comp_conf[j]->symmetric_pair->original_position;
        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] = IA_DMX_MTX_GAIN_ZERO;
        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym] = IA_DMX_MTX_GAIN_ZERO;
        params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos] = IA_DMX_MTX_GAIN_ZERO;
        params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos_sym] = IA_DMX_MTX_GAIN_ZERO;
        if (0 == pscr->comp_dmx_mtx[i][j])
          continue;

        params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos] =
            impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        if (pscr->is_seperable[j] && pscr->is_symmetric[j])
        {
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos_sym] =
              params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos];
        }
        else if (!pscr->is_seperable[j] && pscr->is_symmetric[j])
        {
          params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos] =
              params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym];
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos_sym] =
              params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos];
        }
        else if (pscr->is_seperable[j] && !pscr->is_symmetric[j])
        {
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos_sym] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        }
        else
        {
          params->dmx_mtx[in_org_pos * num_out_ch + out_org_pos_sym] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
          params->dmx_mtx[in_org_pos_sym * num_out_ch + out_org_pos_sym] =
              impeghd_decode_gain(ptr_bit_buf, &pscr->gain_coder);
        }
      }
    }
  }

  if (precision_level > 2)
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_PRECISION_LVL;
  }

  for (i = 0; i < num_in_ch; i++)
  {
    for (j = 0; j < num_out_ch; j++)
    {
      FLOAT32 gain = params->dmx_mtx[i * params->num_out_ch + j];
      if (gain != IA_DMX_MTX_GAIN_ZERO)
      {
        params->dmx_mtx[i * params->num_out_ch + j] = (FLOAT32)pow(10.0f, gain / 20.0f);
      }
      else
      {
        params->dmx_mtx[i * params->num_out_ch + j] = 0.0f;
      }
    }
  }

  if (equalizer_present)
  {
    WORD32 eq, band, filt, eq_idx, fft_band, fft_stop_band;
    FLOAT32 freq;
    if (pscr->eq_conf.num_eq > IA_MAX_NUM_EQ)
    {
      return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_NUM_EQS;
    }

    for (eq = 0; eq < (WORD32)pscr->eq_conf.num_eq; eq++)
    {
      if (pscr->eq_conf.eq_map[eq] > (WORD32)pscr->eq_conf.num_eq)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_EQ_CONF;
      }
    }

    for (eq = 0; eq < (WORD32)pscr->eq_conf.num_eq; eq++)
    {
      for (band = 0; band < IA_DMX_NUM_ERB_BANDS; band++)
      {
        freq = (FLOAT32)fabs(ia_nrm_stft_erb_58[band]) * (FLOAT32)params->samp_freq / 2.0f;
        pscr->eq_gains[band][eq] =
            impeghd_peak_filter(&pscr->eq_conf.eq_params[eq].pk_filt[0],
                                pscr->eq_conf.eq_params[eq].global_gain, freq);
        for (filt = 1; filt < (WORD32)pscr->eq_conf.eq_params[eq].num_pk_filt; filt++)
        {
          pscr->eq_gains[band][eq] *=
              impeghd_peak_filter(&pscr->eq_conf.eq_params[eq].pk_filt[filt], 0, freq);
        }
      }
    }

    for (i = 0; i < num_in_ch; i++)
    {
      eq_idx = pscr->eq_conf.eq_map[i];
      if (eq_idx > 0)
      {
        for (j = 0; j < num_out_ch; j++)
        {
          fft_band = 0;
          /* Check for converter mode */
          for (band = 0; band < IA_DMX_NUM_ERB_BANDS; band++)
          {
            fft_stop_band = ia_erb_freq_idx_256_58[band];
            while (fft_band < fft_stop_band)
            {
              fc_params->downmix_mat_freq[fft_band][i][j] =
                  pscr->eq_gains[band][eq_idx - 1] * params->dmx_mtx[i * num_out_ch + j];
              fft_band++;
            }
          }
        }
      }
      else
      {
        for (j = 0; j < num_out_ch; j++)
        {
          for (band = 0; band < FC_BANDS; band++)
          {
            fc_params->downmix_mat_freq[band][i][j] = params->dmx_mtx[i * num_out_ch + j];
          }
        }
      }
    }
  }
  else
  {
    WORD32 band;
    for (band = 0; band < FC_BANDS; band++)
    {
      for (i = 0; i < num_in_ch; i++)
      {
        for (j = 0; j < num_out_ch; j++)
        {
          fc_params->downmix_mat_freq[band][i][j] = params->dmx_mtx[i * params->num_out_ch + j];
        }
      }
    }
  }

  return err_code;
}
/** @} */ /* End of DmxMtxProc */
