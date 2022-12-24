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
#include <string.h>
#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"
#include "ia_core_coder_apicmd_standards.h"
#include "ia_core_coder_api_defs.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_definitions.h"
#include "impeghd_memory_standards.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "impeghd_multichannel.h"
#include "impeghd_multichannel_rom.h"

#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "ia_core_coder_vec_baisc_ops.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_func_def.h"

/**
 * @defgroup MCTProc Core Decoder MCT processing
 * @ingroup  MCTProc
 * @brief Core Decoder MCT processing
 *
 * @{
 */

/**
 *  impeghd_mc_stereofilling_add
 *
 *  \brief Performs multichannel audio filling
 *
 *  \param [out] ptr_coef          Pointer to coefficient
 *  \param [in]  ptr_dmx_prev      Pointer to previous downmix
 *  \param [in]  ptr_scale_factors Pointer to scale factors array
 *  \param [in]  total_sfb         Total scale factor bands
 *  \param [in]  group_len         Group length
 *  \param [in]  bins_per_sbk      Bin per scale factor band
 *  \param [in]  ptr_sbk_sfb_top   Pointer to scale factor band table
 *
 *
 *
 */
VOID impeghd_mc_stereofilling_add(FLOAT32 *ptr_coef, FLOAT32 *ptr_dmx_prev,
                                  FLOAT32 *ptr_scale_factors, const WORD32 total_sfb,
                                  const WORD32 group_len, const WORD32 bins_per_sbk,
                                  const WORD16 *ptr_sbk_sfb_top)
{
  WORD32 group_win, tab_idx, sfb;
  FLOAT32 *coeff_float_arr, *dmx_prev_float_arr, scf_temp, factor, target_energy, res_energy;

  coeff_float_arr = ptr_coef;
  dmx_prev_float_arr = ptr_dmx_prev;

  /* Start loop from 1 to avoid need for special handling of band 0 */
  for (sfb = 1; sfb < total_sfb; sfb++)
  {
    if (ia_lt_flt(1e-8f, ptr_scale_factors[sfb]))
    {
      scf_temp = 4.0f / ptr_scale_factors[sfb];
      factor = 1e-8f;
      target_energy = 0.0f;
      res_energy = 0.0f;

      for (group_win = 0; group_win < group_len; group_win++)
      {
        const WORD32 offset = group_win * bins_per_sbk;

        /* Amplify empty band to a maximum of target energy per bin, measure energy of delayed
         * downmix */
        for (tab_idx = ptr_sbk_sfb_top[sfb - 1]; tab_idx < ptr_sbk_sfb_top[sfb]; tab_idx++)
        {
          factor += ia_mul_flt(dmx_prev_float_arr[offset + tab_idx],
                               dmx_prev_float_arr[offset + tab_idx]);
          coeff_float_arr[offset + tab_idx] =
              ia_mul_flt(coeff_float_arr[offset + tab_idx], scf_temp);
          res_energy +=
              ia_mul_flt(coeff_float_arr[offset + tab_idx], coeff_float_arr[offset + tab_idx]);
        }
        target_energy += (FLOAT32)(ptr_sbk_sfb_top[sfb] - ptr_sbk_sfb_top[sfb - 1]);
      }

      if (ia_lt_flt(res_energy, target_energy))
      {
        factor = (FLOAT32)ia_sqrt_flt(ia_sub_flt(target_energy, res_energy) / factor);
        factor = ia_min_flt(factor, 10.0f);
        res_energy = 0.0f;

        for (group_win = 0; group_win < group_len; group_win++)
        {
          const WORD32 offset = group_win * bins_per_sbk;
          for (tab_idx = ptr_sbk_sfb_top[sfb - 1]; tab_idx < ptr_sbk_sfb_top[sfb]; tab_idx++)
          {
            coeff_float_arr[offset + tab_idx] +=
                ia_mul_flt(dmx_prev_float_arr[offset + tab_idx], factor);
            res_energy +=
                ia_mul_flt(coeff_float_arr[offset + tab_idx], coeff_float_arr[offset + tab_idx]);
          }
        }

        /* Target energy isn't reached, correct the band */
        if (!ia_eq_flt(res_energy, target_energy) && ia_lt_flt(0.0f, res_energy))
        {
          factor = (FLOAT32)ia_sqrt_flt(target_energy / ia_add_flt(res_energy, 1e-8f));
          factor = ia_min_flt(factor, 10.0f);

          for (group_win = 0; group_win < group_len; group_win++)
          {
            const WORD32 offset = group_win * bins_per_sbk;
            for (tab_idx = ptr_sbk_sfb_top[sfb - 1]; tab_idx < ptr_sbk_sfb_top[sfb]; tab_idx++)
            {
              coeff_float_arr[offset + tab_idx] =
                  ia_mul_flt(coeff_float_arr[offset + tab_idx], factor);
            }
          }
        }
      }

      for (group_win = 0; group_win < group_len; group_win++)
      {
        const WORD32 offset = group_win * bins_per_sbk;
        for (tab_idx = ptr_sbk_sfb_top[sfb - 1]; tab_idx < ptr_sbk_sfb_top[sfb]; tab_idx++)
        {
          coeff_float_arr[offset + tab_idx] =
              ia_mul_flt(coeff_float_arr[offset + tab_idx], ptr_scale_factors[sfb]);
        }
      }
    }
  }
  return;
}

/**
 *  impeghd_mc_decode_huffman
 *
 *  \brief Performs multichannel Huffman decoding
 *
 *  \param [in] ptr_it_bit_buff Pointer to bit buffer handle
 *  \param [in] ptr_ctab        Pointer to Huffman code table
 *  \param [in] ptr_ltab        Pointer to Huffman length table
 *  \param [in] limit           Index limit
 *
 *  \return WORD32              Huffman table index
 *
 */
static WORD32 impeghd_mc_decode_huffman(ia_bit_buf_struct *ptr_it_bit_buff,
                                        const WORD32 *ptr_ctab, const WORD32 *ptr_ltab,
                                        WORD32 limit)
{
  WORD32 n_bits = 0, tab_idx = 0, new_bit, bs_val = 0;
  UWORD8 *ptr_bit_buf_base, *ptr_bit_buf_end, *ptr_read_next;
  UWORD32 bit_pos, cnt_bits;

  ptr_bit_buf_base = ptr_it_bit_buff->ptr_bit_buf_base;
  ptr_bit_buf_end = ptr_it_bit_buff->ptr_bit_buf_end;
  ptr_read_next = ptr_it_bit_buff->ptr_read_next;
  bit_pos = (UWORD32)ptr_it_bit_buff->bit_pos;
  cnt_bits = (UWORD32)ptr_it_bit_buff->cnt_bits;

  while (1)
  {
    n_bits = 0;
    bs_val = 0;
    while ((ptr_it_bit_buff->cnt_bits > 0) && (n_bits < ptr_ltab[tab_idx]))
    {
      new_bit = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 1);
      bs_val = (bs_val << 1) | new_bit;
      n_bits++;
    }

    if (ptr_ctab[tab_idx] != bs_val)
    {
      ptr_it_bit_buff->ptr_bit_buf_base = ptr_bit_buf_base;
      ptr_it_bit_buff->ptr_bit_buf_end = ptr_bit_buf_end;
      ptr_it_bit_buff->ptr_read_next = ptr_read_next;
      ptr_it_bit_buff->bit_pos = (UWORD32)bit_pos;
      ptr_it_bit_buff->cnt_bits = (UWORD32)cnt_bits;
    }
    else
    {
      /* Index found */
      break;
    }

    if (++tab_idx >= limit)
    {
      return -1;
    }
  }
  return tab_idx;
}

/**
 *  impeghd_mc_init
 *
 *  \brief Function to initialize multichannel decoding
 *
 *  \param [out] pstr_usac_dec_config Pointer to USAC decoder config handle
 *  \param [in]  xmpeghd_jmp_buf      Pointer to buffer used by setjmp
 *  \param [in]  num_ch               Number of channel
 *  \param [in]  start_ch             Start channel
 *  \param [in]  start_element        Payload start length index
 *  \param [in]  mct_cnt              Multichannel data count
 *
 *  \return IA_ERRORCODE              Processing error if any
 *
 */
IA_ERRORCODE impeghd_mc_init(ia_usac_decoder_config_struct *pstr_usac_dec_config,
                             jmp_buf *xmpeghd_jmp_buf, WORD32 num_ch, WORD32 start_ch,
                             WORD32 start_element, WORD32 mct_cnt)
{
  IA_ERRORCODE error = IA_MPEGH_DEC_NO_ERROR;
  WORD32 chan;
  UWORD8 *ptr_bitbuf, bitbuf_len;
  ia_bit_buf_struct it_bit_buff;
  ia_multichannel_data *ptr_mc_data = &pstr_usac_dec_config->ia_mcdata[mct_cnt];

  ptr_bitbuf = pstr_usac_dec_config->usac_ext_ele_payload_buf[start_element];
  bitbuf_len = (UWORD8)pstr_usac_dec_config->usac_ext_ele_payload_len[start_element];
  ia_core_coder_create_init_bit_buf(&it_bit_buff, ptr_bitbuf, bitbuf_len);

  it_bit_buff.xmpeghd_jmp_buf = xmpeghd_jmp_buf;
  ptr_mc_data->use_tool = 1;
  ptr_mc_data->start_channel = start_ch;
  ptr_mc_data->start_element = start_element;

  for (chan = 0; chan < num_ch; chan++)
  {
    ptr_mc_data->channel_mask[chan + start_ch] = ia_core_coder_read_bits_buf(&it_bit_buff, 1);
    if (ptr_mc_data->channel_mask[chan + start_ch] > 0)
    {
      if (ID_USAC_LFE == pstr_usac_dec_config->usac_element_type[start_element])
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_CH_MASK_LFE;
      }
      else
      {
        ptr_mc_data->channel_map[ptr_mc_data->num_ch_to_apply + start_ch] = chan + start_ch;
        ptr_mc_data->num_ch_to_apply++;
      }
    }
  }
  /*the number of all non-zero entries in mctChanMask[] shall be greater than or equal to 2*/
  if (ptr_mc_data->num_ch_to_apply < 2)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_MCT_ENTRIES;
  }
  return error;
}

/**
 *  impeghd_mc_parse_ch_pair
 *
 *  \brief Parse channel pair of multichannel
 *
 *  \param [out] code_pair       Channel pair code
 *  \param [in]  num_ch_to_apply Number of channel to apply pair
 *  \param [in]  ptr_it_bit_buff Pointer to bit buffer handle
 *
 *
 *
 */
static IA_ERRORCODE impeghd_mc_parse_ch_pair(WORD32 code_pair[2], WORD32 num_ch_to_apply,
                                             ia_bit_buf_struct *ptr_it_bit_buff)
{
  WORD32 chan0, chan1, pair_index = 0, pair_counter = 0, num_bits = 1, max_num_pair_index, index;

  if (code_pair == NULL || ptr_it_bit_buff == NULL)
  {
    return IA_MPEGH_DEC_INIT_FATAL_DEC_INIT_FAIL;
  }

  max_num_pair_index = ((num_ch_to_apply * (num_ch_to_apply - 1)) >> 1) - 1;
  if (max_num_pair_index < 0)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_MCT_ENTRIES;
  }
  index = max_num_pair_index;
  while (index >>= 1)
  {
    ++num_bits;
  }
  pair_index = ia_core_coder_read_bits_buf(ptr_it_bit_buff, num_bits);

  /*channelPairIndex shall have a value between and including 0 and
   * nMCTChannels·(nMCTChannels−1)/2−1,*/
  if (pair_index > max_num_pair_index)
  {
    return IA_MPEGH_DEC_INIT_FATAL_INVALID_CH_PAIR_IDX;
  }

  for (chan1 = 1; chan1 < num_ch_to_apply; chan1++)
  {
    for (chan0 = 0; chan0 < chan1; chan0++)
    {
      if (pair_counter == pair_index)
      {
        code_pair[0] = chan0;
        code_pair[1] = chan1;
        return IA_MPEGH_DEC_NO_ERROR;
      }
      else
      {
        pair_counter++;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impeghd_mc_parse
 *
 *  \brief Function to parse multichannel data
 *
 *  \param [out] ptr_mc_data     Pointer to multichannel data
 *  \param [in]  ptr_bitbuf      Pointer to bit buffer
 *  \param [in]  bitbuf_len      Bit buffer length
 *  \param [in]  indep_flag      Independecy flag
 *  \param [in]  ptr_sfb_info    Pointer to scale factor band information
 *  \param [in]  ptr_dec_bit_buf Pointer to decoder bit buf pointer
 *
 *  \return IA_ERRORCODE         Processing error if any
 *
 */
IA_ERRORCODE impeghd_mc_parse(ia_multichannel_data *ptr_mc_data, UWORD8 *ptr_bitbuf,
                              UWORD32 bitbuf_len, const WORD32 indep_flag,
                              ia_sfb_info *ptr_sfb_info[MAX_TIME_CHANNELS],
                              ia_bit_buf_struct *ptr_dec_bit_buf)
{
  IA_ERRORCODE error = IA_MPEGH_DEC_NO_ERROR;
  WORD32 box, band, default_val, channel_map, last_val = 0, new_coeff, bands_per_win;
  ia_bit_buf_struct it_bit_buff;

  ia_core_coder_create_init_bit_buf(&it_bit_buff, ptr_bitbuf, bitbuf_len);
  it_bit_buff.xmpeghd_jmp_buf = ptr_dec_bit_buf->xmpeghd_jmp_buf;

  ptr_mc_data->stereo_filling = ia_core_coder_read_bits_buf(&it_bit_buff, 1);
  ptr_mc_data->signal_type = ia_core_coder_read_bits_buf(&it_bit_buff, 1);
  default_val = (ptr_mc_data->signal_type == 0) ? DEFAULT_ALPHA : DEFAULT_BETA;
  ptr_mc_data->keep_tree = (indep_flag > 0) ? 0 : ia_core_coder_read_bits_buf(&it_bit_buff, 1);

  if (indep_flag > 0)
  {
    for (box = 0; box < MAX_NUM_MC_BOXES; box++)
    {
      ptr_mc_data->pair_coeffq_fb_prev[box] = default_val;
      for (band = 0; band < MAX_NUM_MC_BANDS; band++)
      {
        ptr_mc_data->pair_coeffq_sfb_prev[box][band] = default_val;
      }
    }
  }
  if (ptr_mc_data->keep_tree == 0)
  {
    ia_core_coder_read_escape_value(&it_bit_buff, (UWORD32 *)&ptr_mc_data->num_pairs, 5, 8, 16);
  }
  if (ptr_mc_data->num_pairs > MAX_NUM_MC_BOXES)
  {
    return IA_MPEGH_DEC_EXE_FATAL_MCT_PROCESS_FAIL;
  }

  for (box = ptr_mc_data->num_pairs; box < MAX_NUM_MC_BOXES; box++)
  {
    ptr_mc_data->pair_coeffq_fb_prev[box] = default_val;
    for (band = 0; band < MAX_NUM_MC_BANDS; band++)
    {
      ptr_mc_data->pair_coeffq_sfb_prev[box][band] = default_val;
    }
  }

  for (box = 0; box < ptr_mc_data->num_pairs; box++)
  {
    ptr_mc_data->stereo_filling_flag[box] =
        (ptr_mc_data->stereo_filling) ? ia_core_coder_read_bits_buf(&it_bit_buff, 1) : 0;
    if (ptr_mc_data->signal_type == 0 || ptr_mc_data->signal_type == 1)
    {
      last_val = 0;
      if (ptr_mc_data->keep_tree == 0)
      {
        error = impeghd_mc_parse_ch_pair(ptr_mc_data->code_pairs[box],
                                         ptr_mc_data->num_ch_to_apply, &it_bit_buff);
        if (error != IA_MPEGH_DEC_NO_ERROR)
        {
          return error;
        }
      }
      channel_map =
          ptr_mc_data->channel_map[ptr_mc_data->start_channel + ptr_mc_data->code_pairs[box][0]];
      if (ptr_sfb_info[channel_map] == NULL)
      {
        return IA_MPEGH_DEC_EXE_FATAL_MCT_PROCESS_FAIL;
      }
      if ((ptr_mc_data->win_seq_is_long_prev[box] != 0 &&
           ptr_sfb_info[channel_map]->islong == 0) ||
          (ptr_mc_data->win_seq_is_long_prev[box] == 0 && ptr_sfb_info[channel_map]->islong != 0))
      {
        ptr_mc_data->pair_coeffq_fb_prev[box] = default_val;
        for (band = 0; band < MAX_NUM_MC_BANDS; band++)
        {
          ptr_mc_data->pair_coeffq_sfb_prev[box][band] = default_val;
        }
      }
      ptr_mc_data->win_seq_is_long_prev[box] =
          ptr_sfb_info[ptr_mc_data->channel_map[ptr_mc_data->start_channel +
                                                ptr_mc_data->code_pairs[box][0]]]
              ->islong;
      ptr_mc_data->mask_flag[box] = ia_core_coder_read_bits_buf(&it_bit_buff, 1);
      ptr_mc_data->bandwise_coeff_flag[box] = ia_core_coder_read_bits_buf(&it_bit_buff, 1);

      if (ptr_mc_data->bandwise_coeff_flag[box] || ptr_mc_data->mask_flag[box])
      {
        WORD32 is_short = ia_core_coder_read_bits_buf(&it_bit_buff, 1);
        ptr_mc_data->num_mask_band[box] = ia_core_coder_read_bits_buf(&it_bit_buff, 5);
        if (is_short)
        {
          ptr_mc_data->num_mask_band[box] *= 8;
        }
      }
      else
      {
        ptr_mc_data->num_mask_band[box] = MAX_NUM_MC_BANDS;
      }
      if (ptr_mc_data->num_mask_band[box] > MAX_NUM_MC_BANDS)
      {
        return IA_MPEGH_DEC_EXE_FATAL_MAX_NUM_MC_BANDS_EXCEEDED;
      }

      if (ptr_mc_data->mask_flag[box])
      {
        for (band = 0; band < ptr_mc_data->num_mask_band[box]; band++)
        {
          ptr_mc_data->mask[box][band] = ia_core_coder_read_bits_buf(&it_bit_buff, 1);
        }
      }
      else
      {
        for (band = 0; band < ptr_mc_data->num_mask_band[box]; band++)
        {
          ptr_mc_data->mask[box][band] = 1;
        }
      }
      ptr_mc_data->pred_dir[box] =
          (ptr_mc_data->signal_type == 0) ? ia_core_coder_read_bits_buf(&it_bit_buff, 1) : 0;
      ptr_mc_data->b_delta_time[box] =
          (indep_flag > 0) ? 0 : ia_core_coder_read_bits_buf(&it_bit_buff, 1);

      /*mct_delta_time mct_delta_time shall be 0 if MCTSignalingType of the previous and the
      current frame changes between one of the following values: 0 and 1, 0 and 3, 1 and 2, or
      2 and 3 (and viceversa)*/

      if ((((ptr_mc_data->prev_signal_type == 0) && (ptr_mc_data->signal_type == 1)) ||
           ((ptr_mc_data->prev_signal_type == 0) && (ptr_mc_data->signal_type == 3)) ||
           ((ptr_mc_data->prev_signal_type == 1) && (ptr_mc_data->signal_type == 2)) ||
           ((ptr_mc_data->prev_signal_type == 1) && (ptr_mc_data->signal_type == 0)) ||
           ((ptr_mc_data->prev_signal_type == 2) && (ptr_mc_data->signal_type == 3)) ||
           ((ptr_mc_data->prev_signal_type == 2) && (ptr_mc_data->signal_type == 1)) ||
           ((ptr_mc_data->prev_signal_type == 3) && (ptr_mc_data->signal_type == 0)) ||
           ((ptr_mc_data->prev_signal_type == 3) && (ptr_mc_data->signal_type == 2))) &&
          (ptr_mc_data->b_delta_time[box] != 0))
      {
        return IA_MPEGH_DEC_INIT_FATAL_INVALID_DELTA_TIME;
      }

      /* Bands per window */
      if (ptr_sfb_info[ptr_mc_data->channel_map[ptr_mc_data->start_channel +
                                                ptr_mc_data->code_pairs[box][0]]]
              ->islong)
      {
        bands_per_win = ptr_mc_data->num_mask_band[box];
      }
      else
      {
        bands_per_win = ptr_mc_data->num_mask_band[box] >> 3;
      }

      if (ptr_mc_data->bandwise_coeff_flag[box] == 0)
      {
        last_val = (ptr_mc_data->b_delta_time[box] > 0) ? ptr_mc_data->pair_coeffq_fb_prev[box]
                                                        : default_val;
        if (ptr_mc_data->signal_type == 0)
        {
          new_coeff =
              last_val -
              impeghd_mc_decode_huffman(&it_bit_buff, impeghd_mc_huff_code_tbl_scf,
                                        impeghd_mc_huff_len_tbl_scf, CODE_BOOK_ALPHA_LAV) +
              60;
        }
        else
        {
          new_coeff = last_val + impeghd_mc_decode_huffman(
                                     &it_bit_buff, impeghd_mc_huff_code_tbl_angle,
                                     impeghd_mc_huff_len_tbl_angle, CODE_BOOK_BETA_LAV);
          if (new_coeff >= CODE_BOOK_BETA_LAV)
          {
            new_coeff -= CODE_BOOK_BETA_LAV;
          }
        }
        for (band = 0; band < ptr_mc_data->num_mask_band[box]; band++)
        {
          ptr_mc_data->pair_coeffq_sfb[box][band] = new_coeff;
          if (bands_per_win != 0)
          {
            ptr_mc_data->pair_coeffq_sfb_prev[box][band % bands_per_win] =
                (ptr_mc_data->mask[box][band] > 0) ? new_coeff : default_val;
          }
        }
        ptr_mc_data->pair_coeffq_fb_prev[box] = new_coeff;
      }
      else
      {
        for (band = 0; band < ptr_mc_data->num_mask_band[box]; band++)
        {
          if (bands_per_win != 0)
          {
            if (ptr_mc_data->b_delta_time[box] > 0)
            {
              last_val = ptr_mc_data->pair_coeffq_sfb_prev[box][band % bands_per_win];
            }
            else if ((band % bands_per_win) == 0)
            {
              last_val = default_val;
            }
          }
          if (ptr_mc_data->mask[box][band] > 0)
          {
            if (ptr_mc_data->signal_type == 0)
            {
              new_coeff =
                  last_val -
                  impeghd_mc_decode_huffman(&it_bit_buff, impeghd_mc_huff_code_tbl_scf,
                                            impeghd_mc_huff_len_tbl_scf, CODE_BOOK_ALPHA_LAV) +
                  60;
            }
            else
            {
              new_coeff = last_val + impeghd_mc_decode_huffman(
                                         &it_bit_buff, impeghd_mc_huff_code_tbl_angle,
                                         impeghd_mc_huff_len_tbl_angle, CODE_BOOK_BETA_LAV);
              if (new_coeff >= CODE_BOOK_BETA_LAV)
              {
                new_coeff = new_coeff % CODE_BOOK_BETA_LAV;
              }
              if (new_coeff < 0)
              {
                new_coeff = 0;
              }
            }
            ptr_mc_data->pair_coeffq_sfb[box][band] = new_coeff;
            if (bands_per_win != 0)
            {
              ptr_mc_data->pair_coeffq_sfb_prev[box][band % bands_per_win] = new_coeff;
            }
            last_val = new_coeff;
          }
          else
          {
            if (bands_per_win != 0)
            {
              ptr_mc_data->pair_coeffq_sfb_prev[box][band % bands_per_win] = default_val;
            }
          }
        }
        ptr_mc_data->pair_coeffq_fb_prev[box] = default_val;
      }
      for (band = bands_per_win; band < MAX_NUM_MC_BANDS; band++)
      {
        ptr_mc_data->pair_coeffq_sfb_prev[box][band] = default_val;
      }
    }
  }
  for (; box < MAX_NUM_MC_BOXES; box++)
  {
    ptr_mc_data->code_pairs[box][0] = ptr_mc_data->code_pairs[box][1] = 0;
  }

  ptr_mc_data->prev_signal_type = ptr_mc_data->signal_type;
  return error;
}

/**
 *  impeghd_apply_mc_inverse_rotation
 *
 *  \brief Function to apply multichannel inverse rotation
 *
 *  \param [in]  ptr_data_l  Pointer to left channel data
 *  \param [in]  ptr_data_r  Pointer to right channel data
 *  \param [out] ptr_dmx     Pointer to downmix
 *  \param [in]  alpha_idx   Cosine of alpha table index
 *  \param [in]  num_samples Number of samples
 *
 *
 *
 */
static VOID impeghd_apply_mc_inverse_rotation(FLOAT32 *ptr_data_l, FLOAT32 *ptr_data_r,
                                              FLOAT32 *ptr_dmx, WORD32 alpha_idx,
                                              WORD32 num_samples)
{
  WORD32 samp;
  FLOAT32 temp;
  for (samp = 0; samp < num_samples; samp++)
  {
    temp = 0.0f;
    if (ptr_data_l)
    {
      temp += ia_mul_flt((FLOAT32)ptr_data_l[samp], impeghd_mc_index_to_cos_alpha[alpha_idx]);
      ptr_dmx[samp] = (FLOAT32)temp;
    }
    if (ptr_data_r)
    {
      temp += ia_mul_flt((FLOAT32)ptr_data_r[samp], impeghd_mc_index_to_sin_alpha[alpha_idx]);
      ptr_dmx[samp] = (FLOAT32)temp;
    }
  }
  return;
}

/**
 *  impeghd_mc_get_prev_dmx
 *
 *  \brief Function to get previous downmix of multichannel
 *
 *  \param [out] ptr_mc_data       Pointer to multichannel data
 *  \param [in]  ptr_prev_spec1    Pointer to previous left data
 *  \param [in]  ptr_prev_spec2    Pointer to previous right data
 *  \param [out] ptr_prev_dmx      Pointer to previous downmix
 *  \param [in]  bands_per_window  Bands per window
 *  \param [in]  ptr_mask          Pointer to mask array
 *  \param [in]  ptr_coeff_sfb_idx Pointer to scale factor band coefficient index
 *  \param [in]  num_samples       Number of samples
 *  \param [in]  pair              Pair
 *  \param [in]  total_sfb         Total scale factor band
 *  \param [in]  ptr_sfb_offset    Pointer to scale factor band offset
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impeghd_mc_get_prev_dmx(ia_multichannel_data *ptr_mc_data, FLOAT32 *ptr_prev_spec1,
                                     FLOAT32 *ptr_prev_spec2, FLOAT32 *ptr_prev_dmx,
                                     const WORD32 bands_per_window, const WORD32 *ptr_mask,
                                     const WORD32 *ptr_coeff_sfb_idx, const WORD32 num_samples,
                                     const WORD32 pair, const WORD32 total_sfb,
                                     const WORD16 *ptr_sfb_offset)
{
  WORD32 sfb = -1, start_line, stop_line;
  IA_ERRORCODE error = IA_MPEGH_DEC_NO_ERROR;

  if (ptr_mc_data->signal_type == 0)
  {
    WORD32 samp;
    if ((ptr_prev_spec1 != NULL) && (ptr_prev_spec2 != NULL))
    {
      if (!ptr_mc_data->pred_dir[pair])
      {
        for (samp = 0; samp < num_samples; samp++)
        {
          ptr_prev_dmx[samp] = (FLOAT32)ia_mul_flt(
              0.5f, (FLOAT32)ia_add_flt(ptr_prev_spec1[samp], ptr_prev_spec2[samp]));
        }
      }
      else
      {
        for (samp = 0; samp < num_samples; samp++)
        {
          ptr_prev_dmx[samp] = (FLOAT32)ia_mul_flt(
              0.5f, (FLOAT32)ia_sub_flt(ptr_prev_spec1[samp], ptr_prev_spec2[samp]));
        }
      }
    }
  }
  else if (!ptr_mc_data->bandwise_coeff_flag[pair] && !ptr_mc_data->mask_flag[pair])
  {
    impeghd_apply_mc_inverse_rotation(ptr_prev_spec1, ptr_prev_spec2, ptr_prev_dmx,
                                      ptr_coeff_sfb_idx[0], num_samples);
  }
  else
  {
    WORD32 band;
    FLOAT32 *prev_spec1_start, *prev_spec2_start;
    for (band = 0; band < MAX_NUM_MC_BANDS; band++)
    {
      const WORD32 rot_angle = (ptr_mask[band] && band < bands_per_window)
                                   ? ptr_coeff_sfb_idx[band]
                                   : (CODE_BOOK_BETA_LAV >> 1);

      start_line = (sfb < 0) ? 0 : ptr_sfb_offset[sfb];
      if (sfb + 1 >= total_sfb)
      {
        return IA_MPEGH_DEC_EXE_FATAL_SFB_EXCEEDED_MAX;
      }
      stop_line = (sfb + 2 < total_sfb) ? ptr_sfb_offset[sfb + 2] : ptr_sfb_offset[sfb + 1];
      prev_spec1_start = (ptr_prev_spec1) ? &ptr_prev_spec1[start_line] : NULL;
      prev_spec2_start = (ptr_prev_spec2) ? &ptr_prev_spec2[start_line] : NULL;

      impeghd_apply_mc_inverse_rotation(prev_spec1_start, prev_spec2_start,
                                        &ptr_prev_dmx[start_line], rot_angle,
                                        stop_line - start_line);
      sfb += 2;
      if (sfb >= total_sfb)
      {
        break;
      }
    }
  }
  return error;
}

/**
 *  impeghd_apply_mc_rotation
 *
 *  \brief Function to apply multichannel rotation
 *
 *  \param [in,out] ptr_dmx     Pointer to downmix
 *  \param [in,out] ptr_res     Pointer to Usac data channel two coefficient array
 *  \param [in]     alpha_idx   Cosine of alpha table index
 *  \param [in]     num_samples Number of samples
 *
 *
 *
 */
static VOID impeghd_apply_mc_rotation(FLOAT32 *ptr_dmx, FLOAT32 *ptr_res, WORD32 alpha_idx,
                                      WORD32 num_samples)
{
  WORD32 samp;
  FLOAT32 val_l, val_r, temp_dmx, temp_res;

  for (samp = 0; samp < num_samples; samp++)
  {
    temp_dmx = (FLOAT32)ptr_dmx[samp];
    temp_res = (FLOAT32)ptr_res[samp];

    val_l = ia_msu_flt(ia_mul_flt(temp_dmx, impeghd_mc_index_to_cos_alpha[alpha_idx]), temp_res,
                       impeghd_mc_index_to_sin_alpha[alpha_idx]);
    val_r = ia_mac_flt(ia_mul_flt(temp_dmx, impeghd_mc_index_to_sin_alpha[alpha_idx]), temp_res,
                       impeghd_mc_index_to_cos_alpha[alpha_idx]);

    ptr_dmx[samp] = val_l;
    ptr_res[samp] = val_r;
  }
  return;
}

/**
 *  impeghd_apply_mc_prediction
 *
 *  \brief Function to apply multichannel prediction
 *
 *  \param [in,out] ptr_dmx     Pointer to downmix
 *  \param [in,out] ptr_res     Pointer to Usac data channel two coefficient array
 *  \param [in]     alpha_q     Alpha value
 *  \param [in]     num_samples Number of samples
 *  \param [in]     pre_dir     Prediction direction
 *
 *
 *
 */
static VOID impeghd_apply_mc_prediction(FLOAT32 *ptr_dmx, FLOAT32 *ptr_res, WORD32 alpha_q,
                                        WORD32 num_samples, WORD32 pre_dir)
{
  WORD32 samp;
  FLOAT32 val_l, val_r, alpha, dmx_float, res_float;

  alpha = alpha_q * 0.1f;
  for (samp = 0; samp < num_samples; samp++)
  {
    dmx_float = ptr_dmx[samp];
    res_float = ptr_res[samp];

    val_l = ia_add_flt(ia_mac_flt(dmx_float, alpha, dmx_float), res_float);
    val_r = (pre_dir == 0)
                ? ia_sub_flt(ia_msu_flt(dmx_float, alpha, dmx_float), res_float)
                : ia_add_flt(ia_mac_flt(ia_negate_flt(dmx_float), alpha, dmx_float), res_float);

    ptr_dmx[samp] = val_l;
    ptr_res[samp] = val_r;
  }
  return;
}

/**
 *  impeghd_mc_save_prev
 *
 *  \brief Saves previous multichannel specification
 *
 *  \param [out] ptr_mc_data    Pointer to multichannel data
 *  \param [in]  ptr_spec       Pointer to specification
 *  \param [in]  ch             Channel index
 *  \param [in]  num_windows    Number of windows
 *  \param [in]  bins_per_sbk   Bin per scale factor band
 *  \param [in]  zero_spec_save Zero specification flag
 *
 *
 *
 */
VOID impeghd_mc_save_prev(ia_multichannel_data *ptr_mc_data, FLOAT32 *ptr_spec, const WORD32 ch,
                          const WORD32 num_windows, const WORD32 bins_per_sbk,
                          const WORD32 zero_spec_save)
{
  WORD32 offset = 0, win;

  if (!zero_spec_save)
  {
    for (win = 0; win < num_windows; win++)
    {
      ia_core_coder_mem_cpy(&ptr_spec[offset], &ptr_mc_data->prev_out_spec[ch][offset],
                            bins_per_sbk);
      offset += bins_per_sbk;
    }
  }
  else
  {
    for (win = 0; win < num_windows; win++)
    {
      ia_core_coder_memset(&ptr_mc_data->prev_out_spec[ch][offset], bins_per_sbk);
      offset += bins_per_sbk;
    }
  }
}

/**
 *  impeghd_mc_process
 *
 *  \brief Process multichannel
 *
 *  \param [out]    ptr_mc_data       Pointer to multichannel data
 *  \param [in,out] ptr_dmx           Pointer to downmix
 *  \param [in,out] ptr_res           Pointer to Usac data channel two coefficient array
 *  \param [in]     ptr_coeff_sfb_idx Pointer to scale factor band coefficient index
 *  \param [in]     ptr_mask          Pointer to mask array
 *  \param [in]     bands_per_window  Bands per window
 *  \param [in]     total_sfb         Total scale factor band
 *  \param [in]     pair              Pair
 *  \param [in]     ptr_sfb_offset    Pointer to scale factor band offset
 *  \param [in]     num_samples       Number of samples
 *
 *
 *
 */
VOID impeghd_mc_process(ia_multichannel_data *ptr_mc_data, FLOAT32 *ptr_dmx, FLOAT32 *ptr_res,
                        WORD32 *ptr_coeff_sfb_idx, WORD32 *ptr_mask, WORD32 bands_per_window,
                        WORD32 total_sfb, WORD32 pair, const WORD16 *ptr_sfb_offset,
                        WORD32 num_samples)
{
  /* note: ptr_sfb_offset does not start at zero */
  WORD32 band, start_line, stop_line, sfb = -1;
  if (!ptr_mc_data->use_tool)
  {
    return;
  }

  /* apply fullband prediction box */
  if (!ptr_mc_data->bandwise_coeff_flag[pair] && !ptr_mc_data->mask_flag[pair])
  {
    if (ptr_mc_data->signal_type == 0)
    {
      impeghd_apply_mc_prediction(ptr_dmx, ptr_res, ptr_coeff_sfb_idx[0], num_samples,
                                  ptr_mc_data->pred_dir[pair]);
    }
    else
    {
      impeghd_apply_mc_rotation(ptr_dmx, ptr_res, ptr_coeff_sfb_idx[0], num_samples);
    }
  }
  else
  {
    /* Apply bandwise prediction box */
    for (band = 0; band < bands_per_window; band++)
    {
      if (ptr_mask[band] == 1)
      {
        start_line = (sfb < 0) ? 0 : ptr_sfb_offset[sfb];
        stop_line = (sfb + 2 < total_sfb) ? ptr_sfb_offset[sfb + 2] : ptr_sfb_offset[sfb + 1];

        if (ptr_mc_data->signal_type == 0)
        {
          impeghd_apply_mc_prediction(&ptr_dmx[start_line], &ptr_res[start_line],
                                      ptr_coeff_sfb_idx[band], stop_line - start_line,
                                      ptr_mc_data->pred_dir[pair]);
        }
        else
        {
          impeghd_apply_mc_rotation(&ptr_dmx[start_line], &ptr_res[start_line],
                                    ptr_coeff_sfb_idx[band], stop_line - start_line);
        }
      }
      sfb += 2;

      /* break condition */
      if (sfb >= total_sfb - 1)
      {
        break;
      }
    }
  }
}
/** @} */ /* End of MCTProc */