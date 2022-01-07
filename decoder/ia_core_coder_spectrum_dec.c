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
#include <stdio.h>
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include <impeghd_intrinsics_flt.h>
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
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
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_igf_dec.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_mpeghd_showbits_32
 *
 *  \brief Show 32bits in bitstream
 *
 *  \param [in]    ptr_read_next  Buffer pointer to read next bit
 *  \param [in]    cnt_bits    Number of bits to be read
 *  \param [in]    increment    increment with which the buffer pointer has to be
 * placed in reading bitstream
 *
 *  \return UWORD32
 *
 */
static UWORD32 ia_core_coder_mpeghd_showbits_32(UWORD8 *ptr_read_next, WORD32 cnt_bits,
                                                WORD32 *increment)
{
  UWORD8 *v = ptr_read_next;
  UWORD32 b = 0;
  WORD32 i;
  WORD32 bumped = 0;

  for (i = 0; i < 4; i++)
  {
    b = b << 8;
    if (cnt_bits > 0)
    {
      b = b | *v;
      v++;
      bumped++;
    }
    cnt_bits -= 8;
  }
  if (increment != NULL)
  {
    *increment = bumped;
  }
  return b;
}

/**
 *  ia_core_coder_calc_grp_offset
 *
 *  \brief Calculate group offset
 *
 *  \param [in]    ptr_sfb_info    sfb info structure
 *  \param [in]    group        sfb group buffer pointer
 *
 *
 *
 */
VOID ia_core_coder_calc_grp_offset(ia_sfb_info_struct *ptr_sfb_info, pUWORD8 group)
{
  WORD32 group_offset;
  WORD32 group_idx;
  WORD32 ia_core_coder_drc_offset;
  WORD16 *group_offset_p;
  WORD32 sfb, len;
  if (!ptr_sfb_info->islong)
  {
    group_offset = 0;
    group_idx = 0;
    do
    {
      ptr_sfb_info->group_len[group_idx] = group[group_idx] - (WORD16)group_offset;
      group_offset = group[group_idx];
      group_idx++;
    } while (group_offset < 8);
    ptr_sfb_info->num_groups = group_idx;
    group_offset_p = ptr_sfb_info->sfb_idx_tbl;
    ia_core_coder_drc_offset = 0;
    for (group_idx = 0; group_idx < ptr_sfb_info->num_groups; group_idx++)
    {
      len = ptr_sfb_info->group_len[group_idx];
      for (sfb = 0; sfb < ptr_sfb_info->sfb_per_sbk; sfb++)
      {
        ia_core_coder_drc_offset += ptr_sfb_info->sfb_width[sfb] * len;
        *group_offset_p++ = (WORD16)ia_core_coder_drc_offset;
      }
    }
  }
}

/**
 *  ia_core_coder_read_tns_tcx
 *
 *  \brief Read tns data and fill tns frame info structure in tcx coding
 *
 *  \param [in]    sfb_per_sbk        scale factor band per subblock
 *  \param [in,out]  pstr_tns_frame_info    tns frame info structure
 *  \param [in]    it_bit_buff        bit stream buffer
 *  \param [in]    sbk_index        subblock index
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_read_tns_tcx(WORD32 sfb_per_sbk,
                                        ia_tns_frame_info_struct *pstr_tns_frame_info,
                                        ia_bit_buf_struct *it_bit_buff, WORD32 sbk_index)
{
  WORD32 j, k, top, coef_res, resolution, compress;
  WORD32 i;
  WORD16 *sp, tmp, s_mask, n_mask;
  ia_tns_filter_struct *pstr_tns_filt;
  ia_tns_info_struct *pstr_tns_info;
  static const WORD16 sgn_mask[] = {0x0, 0x0, 0x2, 0x4, 0x8};
  static const WORD16 neg_mask[] = {0x0, 0x0, (WORD16)(0xfffc), (WORD16)(0xfff8),
                                    (WORD16)(0xfff0)};

  pstr_tns_frame_info->n_subblocks = 1;

  for (i = sbk_index; i < sbk_index + pstr_tns_frame_info->n_subblocks; i++)
  {
    pstr_tns_info = &pstr_tns_frame_info->str_tns_info[i];
    pstr_tns_info->n_filt = ia_core_coder_read_bits_buf(it_bit_buff, TNS_SHORT_NUM_FILT_BITS);
    if (!pstr_tns_info->n_filt)
      continue;

    pstr_tns_info->coef_res = coef_res = ia_core_coder_read_bits_buf(it_bit_buff, 1) + 3;
    top = sfb_per_sbk;
    pstr_tns_filt = &pstr_tns_info->str_filter[0];

    for (j = pstr_tns_info->n_filt; j > 0; j--)
    {
      pstr_tns_filt->stop_band = top;
      top = pstr_tns_filt->start_band =
          top - ia_core_coder_read_bits_buf(it_bit_buff, TNS_SHORT_START_BAND_BITS);
      if (top <= 0)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_START_BAND;
      }
      pstr_tns_filt->order = ia_core_coder_read_bits_buf(it_bit_buff, TNS_SHORT_ORDER_BITS);

      if (pstr_tns_filt->order)
      {
        pstr_tns_filt->direction = ia_core_coder_read_bits_buf(it_bit_buff, 1);
        compress = ia_core_coder_read_bits_buf(it_bit_buff, 1);
        resolution = coef_res - compress;
        s_mask = sgn_mask[resolution];
        n_mask = neg_mask[resolution];
        sp = pstr_tns_filt->coef;

        for (k = pstr_tns_filt->order; k > 0; k--)
        {
          tmp = (WORD16)ia_core_coder_read_bits_buf(it_bit_buff, resolution);
          if (tmp & s_mask)
          {
            *sp++ = tmp | n_mask;
          }
          else
          {
            *sp++ = tmp;
          }
        }
      }

      pstr_tns_filt++;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_read_tns_u
 *
 *  \brief Read tns data and fill pstr_tns_frame_info structure
 *
 *  \param [in]    ptr_sfb_info      sfb info structure
 *  \param [in]    pstr_tns_frame_info    tns frame info structure
 *  \param [in]    it_bit_buff        bitstream buffer
 *
 *  \return IA_ERRORCODE error if any
 *
 */
IA_ERRORCODE ia_core_coder_read_tns_u(ia_sfb_info_struct *ptr_sfb_info,
                                      ia_tns_frame_info_struct *pstr_tns_frame_info,
                                      ia_bit_buf_struct *it_bit_buff)
{
  WORD32 j, k, top, coef_res, resolution, compress, i;
  WORD16 *sp, tmp, s_mask, n_mask;
  ia_tns_filter_struct *pstr_tns_filt;
  ia_tns_info_struct *pstr_tns_info;
  static const WORD16 sgn_mask[] = {0x0, 0x0, 0x2, 0x4, 0x8};
  static const WORD16 neg_mask[] = {0x0, 0x0, (WORD16)0xfffc, (WORD16)0xfff8, (WORD16)0xfff0};
  WORD16 num_filt_bits;
  WORD16 start_band_bits;
  WORD16 order_bits;
  if (ptr_sfb_info == NULL)
    return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_SFB_INFO;
  pstr_tns_frame_info->n_subblocks = ptr_sfb_info->max_win_len;

  if (ptr_sfb_info->islong)
  {
    num_filt_bits = TNS_LONG_NUM_FILT_BITS;
    start_band_bits = TNS_LONG_START_BAND_BITS;
    order_bits = TNS_LONG_ORDER_BITS;
  }
  else
  {
    num_filt_bits = TNS_SHORT_NUM_FILT_BITS;
    start_band_bits = TNS_SHORT_START_BAND_BITS;
    order_bits = TNS_SHORT_ORDER_BITS;
  }

  for (i = 0; i < pstr_tns_frame_info->n_subblocks; i++)
  {
    pstr_tns_info = &pstr_tns_frame_info->str_tns_info[i];
    pstr_tns_info->n_filt = ia_core_coder_read_bits_buf(it_bit_buff, num_filt_bits);
    if (!pstr_tns_info->n_filt)
      continue;

    pstr_tns_info->coef_res = coef_res = ia_core_coder_read_bits_buf(it_bit_buff, 1) + 3;
    top = ptr_sfb_info->sfb_per_sbk;
    pstr_tns_filt = &pstr_tns_info->str_filter[0];

    for (j = pstr_tns_info->n_filt; j > 0; j--)
    {
      pstr_tns_filt->stop_band = top;
      top = pstr_tns_filt->start_band =
          top - ia_core_coder_read_bits_buf(it_bit_buff, start_band_bits);
      if (top <= 0)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_START_BAND;
      }
      pstr_tns_filt->order = ia_core_coder_read_bits_buf(it_bit_buff, order_bits);

      if (pstr_tns_filt->order)
      {
        pstr_tns_filt->direction = ia_core_coder_read_bits_buf(it_bit_buff, 1);
        compress = ia_core_coder_read_bits_buf(it_bit_buff, 1);
        resolution = coef_res - compress;
        s_mask = sgn_mask[resolution];
        n_mask = neg_mask[resolution];
        sp = pstr_tns_filt->coef;

        for (k = pstr_tns_filt->order; k > 0; k--)
        {
          tmp = (WORD16)ia_core_coder_read_bits_buf(it_bit_buff, resolution);
          if (tmp & s_mask)
          {
            *sp++ = tmp | n_mask;
          }
          else
          {
            *sp++ = tmp;
          }
        }
      }

      pstr_tns_filt++;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_scale_factor_data
 *
 *  \brief Update scale factor codebook
 *
 *  \param [in]    info      sfb info structure
 *  \param [in]    tot_sfb      total sfb per subblock
 *  \param [in]    max_sfb      maximum sfb buf pointers
 *  \param [in]    sfb_per_sbk    sfb per sub block
 *  \param [in]    ptr_code_book  pointer to code book
 *
 *
 *
 */
static VOID ia_core_coder_scale_factor_data(ia_sfb_info_struct *info, WORD32 tot_sfb,
                                            WORD32 max_sfb, WORD32 sfb_per_sbk,
                                            WORD8 *ptr_code_book)
{
  WORD band;
  WORD8 *ptr_codebook = ptr_code_book;
  WORD8 *loc_ptr_codebook = ptr_codebook;
  WORD32 win_len = info->max_win_len;

  memset(ptr_codebook, 0, 128);

  band = 0;
  while (band < tot_sfb || win_len != 0)
  {
    band = band + sfb_per_sbk;
    memset(loc_ptr_codebook, ESC_HCB, max_sfb);
    ptr_codebook += 16;
    loc_ptr_codebook = ptr_codebook;
    win_len--;
  }
  return;
}

/**
 *  ia_core_coder_win_seq_select
 *
 *  \brief Win sequence selection logic based on past window
 *
 *  \param [in,out]    window_sequence_curr  Present window sequence
 *  \param [in]      window_sequence_last  Past window sequence
 *
 *  \return WORD32
 *
 */
WORD32
ia_core_coder_win_seq_select(WORD32 window_sequence_curr, WORD32 window_sequence_last)
{
  WORD32 window_sequence;

  switch (window_sequence_curr)
  {
  case LONG_STOP_SEQUENCE:
    window_sequence = LONG_STOP_SEQUENCE;
    break;

  case EIGHT_SHORT_SEQUENCE:
    window_sequence = EIGHT_SHORT_SEQUENCE;
    break;

  case LONG_START_SEQUENCE:
    if ((window_sequence_last == LONG_START_SEQUENCE) ||
        (window_sequence_last == EIGHT_SHORT_SEQUENCE) ||
        (window_sequence_last == STOP_START_SEQUENCE))
    {
      window_sequence = STOP_START_SEQUENCE;
    }
    else
    {
      window_sequence = LONG_START_SEQUENCE;
    }
    break;

  case ONLY_LONG_SEQUENCE:
    window_sequence = ONLY_LONG_SEQUENCE;
    break;

  default:
    return -1;
  }

  return window_sequence;
}

/**
 *  ia_core_coder_section_data
 *
 *  \brief Section data
 *
 *  \param [in,out] usac_data     USAC data structure
 *  \param [in]     it_bit_buff   bit stream buffer
 *  \param [in]     info          sfb info structure
 *  \param [in]     global_gain   global gain from bitstream
 *  \param [in,out] factors       scale factor buffer pointer
 *  \param [in]     groups        scale factor group buffer
 *  \param [in]     ptr_code_book Codebook pointer
 *
 *
 *
 */
IA_ERRORCODE ia_core_coder_section_data(ia_usac_data_struct *usac_data,
                                        ia_bit_buf_struct *it_bit_buff, ia_sfb_info_struct *info,
                                        WORD16 global_gain, pWORD16 factors, pUWORD8 groups,
                                        WORD8 *ptr_code_book)
{
  WORD32 band;
  WORD16 position = 0;
  WORD32 group;
  WORD16 factor = global_gain;
  WORD8 *loc_codebook_ptr;
  WORD16 *ptr_scale_fac, *loc_ptr_scale_fac;
  WORD16 norm_val;
  WORD32 window_grps, trans_sfb;
  WORD16 index, length;
  const UWORD16 *hscf = usac_data->huffman_code_book_scl;
  const UWORD32 *idx_tab = usac_data->huffman_code_book_scl_index;

  WORD32 start_bit_pos = it_bit_buff->bit_pos;
  UWORD8 *start_read_pos = it_bit_buff->ptr_read_next;
  UWORD8 *ptr_read_next = it_bit_buff->ptr_read_next;
  WORD32 bit_pos = 7 - it_bit_buff->bit_pos;
  WORD32 is_first_group = 1;

  WORD32 bb = 0;
  WORD32 increment;
  WORD32 read_word =
      ia_core_coder_mpeghd_showbits_32(ptr_read_next, it_bit_buff->cnt_bits, &increment);
  ptr_read_next = it_bit_buff->ptr_read_next + increment;

  trans_sfb = info->sfb_per_sbk;
  loc_ptr_scale_fac = factors;
  window_grps = info->max_win_len;
  memset(factors, 0, MAXBANDS);

  for (group = 0; group < window_grps;)
  {
    loc_codebook_ptr = &ptr_code_book[group * 16];
    ptr_scale_fac = loc_ptr_scale_fac;
    group = *groups++;
    for (band = trans_sfb - 1; band >= 0; band--)
    {
      WORD32 cb_num = *loc_codebook_ptr++;

      if ((band == trans_sfb - 1) && (is_first_group == 1))
      {
        *loc_ptr_scale_fac = factor;

        /*decoded scalefactors sf[g][sfb] are within the range of zero to 255, both inclusive*/
        if (*loc_ptr_scale_fac > 255)
        {
          return IA_MPEGH_DEC_EXE_FATAL_INVALID_SF;
        }
        loc_ptr_scale_fac++;
        continue;
      }

      if (cb_num == ZERO_HCB)
        *loc_ptr_scale_fac++ = 0;
      else
      {
        WORD32 pns_band;
        WORD16 curr_energy = 0;

        UWORD32 read_word1;

        read_word1 = read_word << bit_pos;

        ia_core_coder_huffman_decode(read_word1, &index, &length, hscf, idx_tab);

        bit_pos += length;
        ia_core_coder_mpeghd_read_byte_corr1(&ptr_read_next, &bit_pos, &read_word,
                                             it_bit_buff->ptr_bit_buf_end);
        norm_val = index - 60;

        if (cb_num < NOISE_HCB)
        {
          factor = factor + norm_val;
          *loc_ptr_scale_fac = factor;
        }
        else if (cb_num == NOISE_HCB)
        {
          curr_energy += norm_val;
          pns_band = (group << 4) + trans_sfb - band - 1;
          loc_ptr_scale_fac[pns_band] = curr_energy;
        }
        else
        {
          position = position + norm_val;
          *loc_ptr_scale_fac = -position;
        }

        /*decoded scalefactors sf[g][sfb] are within the range of zero to 255, both inclusive*/
        if (*loc_ptr_scale_fac > 255)
        {
          return IA_MPEGH_DEC_EXE_FATAL_INVALID_SF;
        }
        loc_ptr_scale_fac++;
      }
    }
    is_first_group = 0;

    if (!(info->islong))
    {
      for (bb++; bb < group; bb++)
      {
        memcpy(&ptr_scale_fac[trans_sfb], ptr_scale_fac, trans_sfb * sizeof(ptr_scale_fac[0]));
        loc_ptr_scale_fac += trans_sfb;
        ptr_scale_fac += trans_sfb;
      }
    }
  }
  ptr_read_next = ptr_read_next - increment;
  ia_core_coder_mpeghd_read_byte_corr1(&ptr_read_next, &bit_pos, &read_word,
                                       it_bit_buff->ptr_bit_buf_end);

  it_bit_buff->ptr_read_next = ptr_read_next;

  it_bit_buff->bit_pos = 7 - bit_pos;
  {
    WORD32 bits_consumed;
    bits_consumed = (WORD32)((it_bit_buff->ptr_read_next - start_read_pos) << 3) +
                    (start_bit_pos - it_bit_buff->bit_pos);
    it_bit_buff->cnt_bits -= bits_consumed;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_fd_channel_stream
 *
 *  \brief Parse and analyse FD related parameters
 *
 *  \param [in,out]  usac_data        USAC data structure
 *  \param [in]    pstr_core_coder      corecoder data struct
 *  \param [in]    max_sfb          max sfb buffer
 *  \param [in]    window_sequence_last  Previous window sequence
 *  \param [in]    chn            nth channel in total number of
 * core
 * coder channels
 *  \param [in]    noise_filling      noise filling config structure
 *  \param [in]    igf_config        IGF config structure
 *  \param [in]    ch            Channel under processing
 *  \param [in]    it_bit_buff        Bit stream buffer
 *  \param [in]    id            element id
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE
ia_core_coder_fd_channel_stream(ia_usac_data_struct *usac_data,
                                ia_usac_tmp_core_coder_struct *pstr_core_coder, UWORD8 *max_sfb,
                                WORD32 window_sequence_last, WORD32 chn, WORD32 noise_filling,
                                ia_usac_igf_config_struct *igf_config, WORD32 ch,
                                ia_bit_buf_struct *it_bit_buff, WORD32 id

                                )

{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 tot_sfb;
  WORD32 noise_level = 0;
  WORD32 arith_reset_flag;
  WORD32 igf_num_tiles;

  WORD32 arth_size;
  WORD16 global_gain;
  WORD32 max_spec_coefficients;
  WORD32 igf_all_zero = 0;
  WORD32 max_noise_sfb = 0;
  WORD32 fdp_spacing_idx = -1;

  WORD32 *fac_data;
  ia_sfb_info_struct *info;

  WORD8 *ptr_code_book = (WORD8 *)&usac_data->scratch_int_buf[0];

  global_gain = (WORD16)ia_core_coder_read_bits_buf(it_bit_buff, 8);

  if (noise_filling)
  {
    noise_level = ia_core_coder_read_bits_buf(it_bit_buff, 8);
  }
  else
  {
    noise_level = 0;
  }

  igf_config->igf_stereo_filling = 0;

  if (pstr_core_coder->common_window)
  {
    if (ch == 0)
    {
      if ((noise_level > 0) && (noise_level < 32) &&
          ((usac_data->window_sequence[chn] == STOP_START_SEQUENCE) ||
           (usac_data->window_sequence[chn] == LONG_START_SEQUENCE)))
      {
        noise_level = noise_level << 3;
      }
    }
    else
    {
      if ((noise_level > 0) && (noise_level < 32))
      {
        igf_config->igf_stereo_filling = 1;
        noise_level = noise_level << 3;
      }
    }
  }
  else
  {
    err_code = ia_core_coder_ics_info(usac_data, chn, max_sfb, it_bit_buff, window_sequence_last);

    if (err_code != IA_MPEGH_DEC_NO_ERROR)
      return err_code;
    if ((noise_level > 0) && (noise_level < 32) &&
        ((usac_data->window_sequence[chn] == STOP_START_SEQUENCE) ||
         (usac_data->window_sequence[chn] == LONG_START_SEQUENCE)))
    {
      noise_level = noise_level << 3;
    }
  }

  info = usac_data->pstr_sfb_info[chn];
  if (*max_sfb == 0)
  {
    tot_sfb = 0;
  }
  else
  {
    i = 0;
    tot_sfb = info->sfb_per_sbk;

    while (usac_data->group_dis[chn][i++] < info->max_win_len)
    {
      tot_sfb += info->sfb_per_sbk;
    }
  }

  ia_core_coder_scale_factor_data(info, tot_sfb, *max_sfb, info->sfb_per_sbk, ptr_code_book);

  if ((it_bit_buff->ptr_read_next > it_bit_buff->ptr_bit_buf_end - 3) &&
      (it_bit_buff->size == it_bit_buff->max_size))
  {
    return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;
  }

  if (*max_sfb > 0)
  {
    max_spec_coefficients = info->sfb_idx_tbl[*max_sfb - 1] / info->group_len[0];
  }
  else
  {
    max_spec_coefficients = 0;
  }

  if (!pstr_core_coder->common_ltpf)
  {
    usac_data->str_tddec[chn]->ltpf_data.data_present =
        ia_core_coder_read_bits_buf(it_bit_buff, 1);
    if (usac_data->str_tddec[chn]->ltpf_data.data_present)
    {
      if (1 == usac_data->is_base_line_profile_3b)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_PROFILE_CONFIG;
      }
      usac_data->str_tddec[chn]->ltpf_data.pitch_lag_idx =
          ia_core_coder_read_bits_buf(it_bit_buff, 9);
      usac_data->str_tddec[chn]->ltpf_data.gain_idx = ia_core_coder_read_bits_buf(it_bit_buff, 2);
    }
  }
  if (info->islong && !usac_data->usac_independency_flg)
  {
    if (ia_core_coder_read_bits_buf(it_bit_buff, 1)) /* fdp_data_present */
    {
      if (1 == usac_data->is_base_line_profile_3b)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_PROFILE_CONFIG;
      }
      fdp_spacing_idx = ia_core_coder_read_bits_buf(it_bit_buff, 8);
    }
  }

  if (usac_data->usac_independency_flg)
    usac_data->prev_aliasing_symmetry[chn] = (WORD16)ia_core_coder_read_bits_buf(it_bit_buff, 1);
  else
    usac_data->prev_aliasing_symmetry[chn] = usac_data->curr_aliasing_symmetry[chn];

  usac_data->curr_aliasing_symmetry[chn] = (WORD16)ia_core_coder_read_bits_buf(it_bit_buff, 1);

  err_code = ia_core_coder_section_data(usac_data, it_bit_buff, info, global_gain,
                                        usac_data->factors[chn], usac_data->group_dis[chn],
                                        ptr_code_book);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
    return err_code;

  max_noise_sfb = *max_sfb;
  if (igf_config->igf_active)
  {
    WORD8 win_type = (usac_data->window_sequence[chn] == EIGHT_SHORT_SEQUENCE);
    WORD32 igf_sfb_start;
    igf_all_zero = ia_core_coder_get_igf_levels(it_bit_buff, usac_data, igf_config,
                                                info->num_groups, chn, win_type, win_type);

    if (igf_all_zero == -1)
    {
      return IA_MPEGH_DEC_INIT_FATAL_INVALID_IGF_ALL_ZERO;
    }

    if (!igf_all_zero)
    {
      ia_core_coder_get_igf_data(it_bit_buff, usac_data, igf_config, chn, win_type, win_type);
    }

    igf_sfb_start = (usac_data->window_sequence[chn] == EIGHT_SHORT_SEQUENCE)
                        ? igf_config->igf_grid[1].igf_sfb_start
                        : igf_config->igf_grid[0].igf_sfb_start;

    max_noise_sfb = ia_min_int(*max_sfb, igf_sfb_start);
  }

  if (pstr_core_coder->tns_data_present[ch] == 0)
  {
    WORD32 sub_blk;
    usac_data->pstr_tns[chn]->n_subblocks = info->max_win_len;
    for (sub_blk = 0; sub_blk < usac_data->pstr_tns[chn]->n_subblocks; sub_blk++)
    {
      usac_data->pstr_tns[chn]->str_tns_info[sub_blk].n_filt = 0;
    }
  }

  if (pstr_core_coder->tns_data_present[ch] == 1)
    err_code = ia_core_coder_read_tns_u(info, usac_data->pstr_tns[chn], it_bit_buff);
  if (err_code != IA_MPEGH_DEC_NO_ERROR)
    return err_code;

  if (usac_data->usac_independency_flg)
    arith_reset_flag = 1;
  else
    arith_reset_flag = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  switch (usac_data->window_sequence[chn])
  {
  case EIGHT_SHORT_SEQUENCE:
    arth_size = usac_data->ccfl / 8;
    break;
  default:
    arth_size = usac_data->ccfl;
    break;
  }

  if (pstr_core_coder->core_mode[0] == CORE_MODE_FD &&
      pstr_core_coder->core_mode[1] == CORE_MODE_FD && pstr_core_coder->common_window == 1 &&
      id == 1)
  {
    ia_core_coder_cplx_prev_mdct_dmx(
        usac_data->pstr_sfb_info[chn - ch], usac_data->coef_save_float[chn - ch],
        usac_data->coef_save_float[(chn - ch) + 1], usac_data->dmx_re_prev[chn - ch],
        pstr_core_coder->pred_dir,
        usac_data->usac_independency_flg || usac_data->td_frame_prev[chn - ch] ||
            usac_data->td_frame_prev[chn - ch + 1]);
  }

  if (info->islong)
  {
    igf_num_tiles = igf_config->igf_grid[0].igf_num_tiles;
  }
  else
  {
    igf_num_tiles = igf_config->igf_grid[1].igf_num_tiles;
  }

  err_code = ia_core_coder_ac_spectral_data(
      usac_data, max_spec_coefficients, noise_level, arth_size, it_bit_buff, *max_sfb,
      max_noise_sfb, igf_config, igf_num_tiles, igf_all_zero, arith_reset_flag, noise_filling,
      chn, ch, fdp_spacing_idx);

  if (err_code != IA_MPEGH_DEC_NO_ERROR)
    return err_code;

  usac_data->fac_data_present[chn] = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (usac_data->fac_data_present[chn])
  {
    WORD32 fac_len;
    if ((usac_data->window_sequence[chn]) == EIGHT_SHORT_SEQUENCE)
    {
      fac_len = (usac_data->ccfl) >> 4;
    }
    else
    {
      fac_len = (usac_data->ccfl) >> 3;
    }

    fac_data = usac_data->fac_data[chn];
    fac_data[0] = ia_core_coder_read_bits_buf(it_bit_buff, 7);
    ia_core_coder_fac_decoding(fac_len, 0, &fac_data[1], it_bit_buff);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */