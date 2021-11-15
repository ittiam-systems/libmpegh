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
#include "impeghd_error_codes.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_rom.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup DRCProcessing DRCProcessing
 * @ingroup  DRCProcessing
 * @brief DRC Decoder processing
 *
 * @{
 */

/**
 *  impd_drc_compute_filt_coeff
 *
 *  \brief Computes filter coefficients
 *
 *  \param [out] pstr_lp_filt  Pointer to low pass filter coefficient structure
 *  \param [out] pstr_hp_filt  Pointer to high pass filter coefficient structure
 *  \param [out] pstr_ap_filt  Pointer to all pass filter coefficient structure
 *  \param [in] filter_type  filter type
 *  \param [in] co_freq_idx  cross over frequency index
 *
 *  \return VOID
 *
 */
static VOID impd_drc_compute_filt_coeff(ia_iir_filter_struct *pstr_lp_filt,
                                        ia_iir_filter_struct *pstr_hp_filt,
                                        ia_iir_filter_struct *pstr_ap_filt, WORD32 filter_type,
                                        WORD32 co_freq_idx)
{
  FLOAT32 gamma = ia_drc_normal_cross_freq[co_freq_idx].gamma;
  FLOAT32 delta = ia_drc_normal_cross_freq[co_freq_idx].delta;

  switch (filter_type)
  {
  case 0:
  case 2:
    pstr_lp_filt->a0 = 1.0f;
    pstr_lp_filt->a1 = ia_mul_flt(2.0f, ia_sub_flt(gamma, delta));
    pstr_lp_filt->a2 = ia_sub_flt(ia_mul_flt(2.0f, ia_add_flt(gamma, delta)), 1.0f);
    pstr_lp_filt->b0 = gamma;
    pstr_lp_filt->b1 = ia_mul_flt(2.0f, gamma);
    pstr_lp_filt->b2 = gamma;

    pstr_hp_filt->a0 = 1.0f;
    pstr_hp_filt->a1 = pstr_lp_filt->a1;
    pstr_hp_filt->a2 = pstr_lp_filt->a2;
    pstr_hp_filt->b0 = delta;
    pstr_hp_filt->b1 = ia_mul_flt(-2.0f, delta);
    pstr_hp_filt->b2 = delta;

    if (2 == filter_type)
    {
      pstr_ap_filt->a0 = 1.0f;
      pstr_ap_filt->a1 = ia_mul_flt(2.0f, ia_sub_flt(gamma, delta));
      pstr_ap_filt->a2 = ia_sub_flt(ia_mul_flt(2.0f, ia_add_flt(gamma, delta)), 1.0f);
      pstr_ap_filt->b0 = pstr_ap_filt->a2;
      pstr_ap_filt->b1 = pstr_ap_filt->a1;
      pstr_ap_filt->b2 = pstr_ap_filt->a0;
    }
    break;
  case 1:
    pstr_ap_filt->a0 = 1.0f;
    pstr_ap_filt->a1 = ia_mul_flt(2.0f, ia_sub_flt(gamma, delta));
    pstr_ap_filt->a2 = ia_sub_flt(ia_mul_flt(2.0f, ia_add_flt(gamma, delta)), 1.0f);
    pstr_ap_filt->b0 = pstr_ap_filt->a2;
    pstr_ap_filt->b1 = pstr_ap_filt->a1;
    pstr_ap_filt->b2 = pstr_ap_filt->a0;
    break;
  }

  return;
}
/**
 *  impd_drc_initialize_filt_bank
 *
 *  \brief Initialise filter bank
 *
 *  \param [out] pstr_drc_filter_bank  Pointer to drc filter bank structure
 *  \param [in] pstr_gain_params  Pointer to gain params structure
 *  \param [in] num_sb  no of subbands
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_initialize_filt_bank(ia_drc_filter_bank_struct *pstr_drc_filter_bank,
                                                  ia_drc_gain_params_struct *pstr_gain_params,
                                                  WORD32 num_sb)
{
  ia_two_band_filt_struct *str_two_band_filt_bank;
  ia_three_band_filt_struct *str_three_band_filt_bank;
  ia_four_band_filt_struct *str_four_band_filt_bank;
  pstr_drc_filter_bank->complexity = 0;
  pstr_drc_filter_bank->num_bands = num_sb;

  switch (num_sb)
  {
  case 1:
    return IA_MPEGH_DEC_NO_ERROR;
    break;
  case 2:
    pstr_drc_filter_bank->complexity = 8;
    str_two_band_filt_bank = &pstr_drc_filter_bank->str_two_band_filt_bank;
    impd_drc_compute_filt_coeff(&(str_two_band_filt_bank->str_lp),
                                &(str_two_band_filt_bank->str_hp), NULL, 0,
                                pstr_gain_params[1].crossover_freq_idx);
    break;
  case 3:
    pstr_drc_filter_bank->complexity = 18;
    str_three_band_filt_bank = &pstr_drc_filter_bank->str_three_band_filt_bank;
    impd_drc_compute_filt_coeff(
        &(str_three_band_filt_bank->str_lp_stage_2), &(str_three_band_filt_bank->str_hp_stage_2),
        &(str_three_band_filt_bank->str_ap_stage_2), 2, pstr_gain_params[1].crossover_freq_idx);
    impd_drc_compute_filt_coeff(&(str_three_band_filt_bank->str_lp_stage_1),
                                &(str_three_band_filt_bank->str_hp_stage_1), NULL, 0,
                                pstr_gain_params[2].crossover_freq_idx);
    break;
  case 4:
    pstr_drc_filter_bank->complexity = 28;
    str_four_band_filt_bank = &pstr_drc_filter_bank->str_four_band_filt_bank;
    impd_drc_compute_filt_coeff(&(str_four_band_filt_bank->str_lp_stage_3_low),
                                &(str_four_band_filt_bank->str_hp_stage_3_low),
                                &(str_four_band_filt_bank->str_ap_stage_2_high), 2,
                                pstr_gain_params[1].crossover_freq_idx);
    impd_drc_compute_filt_coeff(&(str_four_band_filt_bank->str_lp_stage_1),
                                &(str_four_band_filt_bank->str_hp_stage_1), NULL, 0,
                                pstr_gain_params[2].crossover_freq_idx);
    impd_drc_compute_filt_coeff(&(str_four_band_filt_bank->str_lp_stage_3_high),
                                &(str_four_band_filt_bank->str_hp_stage_3_high),
                                &(str_four_band_filt_bank->str_ap_stage_2_low), 2,
                                pstr_gain_params[3].crossover_freq_idx);
    break;
  default:
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_NUM_SUBBAND;
    break;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_all_filter_banks
 *
 *  \brief Initialise all filter banks
 *
 *  \param [out] pstr_ia_filter_bank  Pointer to filter banks structure
 *  \param [in] pstr_coeffs_uni_drc  Pointer to uni drc coefficients structure
 *  \param [in] pstr_drc_instructions  Pointer to drc instructions structure
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_init_all_filter_banks(ia_filter_banks_struct *pstr_ia_filter_bank,
                                            ia_drc_uni_drc_coeffs_struct *pstr_coeffs_uni_drc,
                                            ia_drc_instructions_struct *pstr_drc_instructions)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  UWORD32 ch_grp;
  WORD32 band_cnt, idx, grp, cnt, cnt1, co_freq_idx;
  WORD32 casc_cross_idx[CHANNEL_GROUP_COUNT_MAX + 1][CHANNEL_GROUP_COUNT_MAX * 3];
  WORD32 grp_cnt[CHANNEL_GROUP_COUNT_MAX + 1];
  WORD32 found_match = 0, num_filter;
  WORD32 num_ch_in_groups = 0;
  WORD32 num_ph_align_ch_grps = pstr_drc_instructions->num_drc_ch_groups;
  memset(casc_cross_idx, 0, sizeof(casc_cross_idx));
  memset(grp_cnt, 0, sizeof(grp_cnt));

  for (ch_grp = 0; ch_grp < (UWORD32)pstr_drc_instructions->num_drc_ch_groups; ch_grp++)
  {
    num_ch_in_groups += pstr_drc_instructions->num_chan_per_ch_group[ch_grp];
  }

  if (pstr_drc_instructions->audio_num_chan > num_ch_in_groups)
  {
    num_ph_align_ch_grps++;
  }

  pstr_ia_filter_bank->num_filt_banks = pstr_drc_instructions->num_drc_ch_groups;
  pstr_ia_filter_bank->num_ph_align_ch_grps = num_ph_align_ch_grps;

  if (pstr_coeffs_uni_drc != NULL)
  {
    for (ch_grp = 0; ch_grp < (UWORD32)pstr_drc_instructions->num_drc_ch_groups; ch_grp++)
    {
      if (0 <= pstr_drc_instructions->gain_set_index_for_channel_group[ch_grp])
      {
        err = impd_drc_initialize_filt_bank(
            &(pstr_ia_filter_bank->str_drc_filter_bank[ch_grp]),
            pstr_coeffs_uni_drc
                ->gain_set_params[pstr_drc_instructions->gain_set_index_for_channel_group[ch_grp]]
                .gain_params,
            pstr_coeffs_uni_drc
                ->gain_set_params[pstr_drc_instructions->gain_set_index_for_channel_group[ch_grp]]
                .band_count);

        if (err != IA_MPEGH_DEC_NO_ERROR)
          return (err);
      }
    }
  }
  else
  {
    pstr_ia_filter_bank->str_drc_filter_bank->num_bands = 1;
  }

  if (pstr_coeffs_uni_drc != NULL)
  {
    for (ch_grp = 0; ch_grp < (UWORD32)pstr_drc_instructions->num_drc_ch_groups; ch_grp++)
    {
      if (0 <= pstr_drc_instructions->gain_set_index_for_channel_group[ch_grp])
      {
        for (band_cnt = 1;
             band_cnt < pstr_coeffs_uni_drc
                            ->gain_set_params[pstr_drc_instructions
                                                  ->gain_set_index_for_channel_group[ch_grp]]
                            .band_count;
             band_cnt++)
        {
          co_freq_idx = pstr_coeffs_uni_drc
                            ->gain_set_params[pstr_drc_instructions
                                                  ->gain_set_index_for_channel_group[ch_grp]]
                            .gain_params[band_cnt]
                            .crossover_freq_idx;
          for (grp = 0; grp < num_ph_align_ch_grps; grp++)
          {
            if (grp != (WORD32)ch_grp)
            {
              casc_cross_idx[grp][grp_cnt[grp]] = co_freq_idx;
              grp_cnt[grp]++;
              if (grp_cnt[grp] > CHANNEL_GROUP_COUNT_MAX * 3)
              {
                return IA_MPEGD_DRC_INIT_FATAL_CH_GROUP_COUNT_EXCEEDED;
              }
            }
          }
        }
      }
    }
  }

  idx = 0;
  while (grp_cnt[0] > idx)
  {
    co_freq_idx = casc_cross_idx[0][idx];
    found_match = 0;
    for (ch_grp = 1; ch_grp < (UWORD32)num_ph_align_ch_grps; ch_grp++)
    {
      found_match = 0;
      for (grp = 0; grp < grp_cnt[ch_grp]; grp++)
      {
        if (co_freq_idx == casc_cross_idx[ch_grp][grp])
        {
          found_match = 1;
          break;
        }
      }
      if (0 == found_match)
        break;
    }
    if (0 == found_match)
    {
      idx++;
    }
    else
    {
      for (ch_grp = 0; ch_grp < (UWORD32)num_ph_align_ch_grps; ch_grp++)
      {
        for (cnt = 0; cnt < grp_cnt[ch_grp]; cnt++)
        {
          if (co_freq_idx == casc_cross_idx[ch_grp][cnt])
          {
            for (cnt1 = cnt + 1; cnt1 < grp_cnt[ch_grp]; cnt1++)
            {
              casc_cross_idx[ch_grp][cnt1 - 1] = casc_cross_idx[ch_grp][cnt1];
            }
            grp_cnt[ch_grp]--;
            break;
          }
        }
      }
      idx = 0;
    }
  }

  for (ch_grp = 0; ch_grp < (UWORD32)num_ph_align_ch_grps; ch_grp++)
  {
    num_filter = grp_cnt[ch_grp];
    if (num_filter > 0)
    {
      for (idx = 0; idx < num_filter; idx++)
      {
        impd_drc_compute_filt_coeff(NULL, NULL,
                                    &(pstr_ia_filter_bank->str_drc_filter_bank[ch_grp]
                                          .str_all_pass_cascade.str_ap_cascade_filt[idx]
                                          .str_ap_stage),
                                    1, casc_cross_idx[ch_grp][idx]);
        pstr_ia_filter_bank->str_drc_filter_bank[ch_grp].complexity += 2;
      }
      pstr_ia_filter_bank->str_drc_filter_bank[ch_grp].str_all_pass_cascade.num_filter =
          num_filter;
    }
  }
  for (grp = 0; grp < num_ph_align_ch_grps; grp++)
  {
    pstr_ia_filter_bank->complexity +=
        (pstr_drc_instructions->num_drc_ch_groups *
         pstr_ia_filter_bank->str_drc_filter_bank[ch_grp].complexity);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_iir_second_order_all_pass_filter
 *
 *  \brief IIR second order all pass filter function
 *
 *  \param [out] output  output buffer
 *  \param [in/out] filter  Pointer to iir filter structure
 *  \param [in] ch_idx  channel index
 *  \param [in] frame_len window length
 *  \param [in] input input buffer
 *
  *  \return VOID
 *
 */
static VOID impd_drc_iir_second_order_all_pass_filter(FLOAT32 *output,
                                                      ia_iir_filter_struct *pstr_filter,
                                                      WORD32 ch_idx, WORD32 frame_len,
                                                      FLOAT32 *input)
{
  FLOAT32 temp;
  FLOAT32 a1 = pstr_filter->a1;
  FLOAT32 a2 = pstr_filter->a2;
  FLOAT32 b0 = pstr_filter->b0;
  FLOAT32 b1 = pstr_filter->b1;
  FLOAT32 b2 = pstr_filter->b2;

  FLOAT32 st1 = pstr_filter->x_p[ch_idx * 2];
  FLOAT32 st2 = pstr_filter->y_p[ch_idx * 2];

  for (WORD32 idx = frame_len - 1; idx >= 0; idx--)
  {
    temp = input[idx];
    output[idx] = ia_add_flt(ia_mul_flt(b0, temp), st1);
    st1 = ia_add_flt(ia_sub_flt(ia_mul_flt(b1, temp), ia_mul_flt(a1, output[idx])), st2);
    st2 = ia_sub_flt(ia_mul_flt(b2, temp), ia_mul_flt(a2, output[idx]));
  }
  pstr_filter->x_p[ch_idx * 2] = st1;
  pstr_filter->y_p[ch_idx * 2] = st2;

  return;
}

/**
 *  impd_drc_apply_low_high_filter
 *
 *  \brief Low high filter function
 *
 *  \param [out] output  output buffer
 *  \param [in/out] pstr_lp_filt  Pointer to iir lowpass filter structure
 *  \param [in/out] pstr_hp_filt  Pointer to iir highpass filter structure
 *  \param [in] ch_idx  channel index
 *  \param [in] frame_len window length
 *  \param [in] input input buffer
 *
 *  \return VOID
 *
 */
VOID impd_drc_apply_low_high_filter(FLOAT32 *output[], ia_iir_filter_struct *pstr_lp_filt,
                                    ia_iir_filter_struct *pstr_hp_filt, WORD32 ch_idx,
                                    WORD32 frame_len, FLOAT32 *input)
{
  FLOAT32 temp, temp1;
  FLOAT32 a1_l = pstr_lp_filt->a1;
  FLOAT32 a2_l = pstr_lp_filt->a2;
  FLOAT32 b0_l = pstr_lp_filt->b0;
  FLOAT32 b1_l = pstr_lp_filt->b1;
  FLOAT32 b2_l = pstr_lp_filt->b2;

  FLOAT32 st1_l = pstr_lp_filt->x_p[ch_idx * 2 + 0];
  FLOAT32 st2_l = pstr_lp_filt->x_p[ch_idx * 2 + 1];
  FLOAT32 st3_l = pstr_lp_filt->y_p[ch_idx * 2 + 0];
  FLOAT32 st4_l = pstr_lp_filt->y_p[ch_idx * 2 + 1];

  FLOAT32 a1_h = pstr_hp_filt->a1;
  FLOAT32 a2_h = pstr_hp_filt->a2;
  FLOAT32 b0_h = pstr_hp_filt->b0;
  FLOAT32 b1_h = pstr_hp_filt->b1;
  FLOAT32 b2_h = pstr_hp_filt->b2;

  FLOAT32 st1_h = pstr_hp_filt->x_p[ch_idx * 2 + 0];
  FLOAT32 st2_h = pstr_hp_filt->x_p[ch_idx * 2 + 1];
  FLOAT32 st3_h = pstr_hp_filt->y_p[ch_idx * 2 + 0];
  FLOAT32 st4_h = pstr_hp_filt->y_p[ch_idx * 2 + 1];

  FLOAT32 *out_low = output[0];
  FLOAT32 *out_high = output[1];

  for (WORD32 idx = 0; idx < frame_len; idx++)
  {
    temp1 = input[idx];
    temp = ia_mac_flt(st1_l, b0_l, temp1);
    st1_l = ia_add_flt(ia_msu_flt(ia_mul_flt(b1_l, temp1), a1_l, temp), st2_l);
    st2_l = ia_msu_flt(ia_mul_flt(b2_l, temp1), a2_l, temp);

    out_low[idx] = ia_mac_flt(st3_l, b0_l, temp);
    st3_l = ia_add_flt(ia_msu_flt(ia_mul_flt(b1_l, temp), a1_l, out_low[idx]), st4_l);
    st4_l = ia_msu_flt(ia_mul_flt(b2_l, temp), a2_l, out_low[idx]);

    temp = ia_mac_flt(st1_h, b0_h, temp1);
    st1_h = ia_add_flt(ia_msu_flt(ia_mul_flt(b1_h, temp1), a1_h, temp), st2_h);
    st2_h = ia_msu_flt(ia_mul_flt(b2_h, temp1), a2_h, temp);

    out_high[idx] = ia_mac_flt(st3_h, b0_h, temp);
    st3_h = ia_add_flt(ia_msu_flt(ia_mul_flt(b1_h, temp), a1_h, out_high[idx]), st4_h);
    st4_h = ia_msu_flt(ia_mul_flt(b2_h, temp), a2_h, out_high[idx]);
  }
  pstr_lp_filt->x_p[ch_idx * 2 + 0] = st1_l;
  pstr_lp_filt->x_p[ch_idx * 2 + 1] = st2_l;
  pstr_lp_filt->y_p[ch_idx * 2 + 0] = st3_l;
  pstr_lp_filt->y_p[ch_idx * 2 + 1] = st4_l;

  pstr_hp_filt->x_p[ch_idx * 2 + 0] = st1_h;
  pstr_hp_filt->x_p[ch_idx * 2 + 1] = st2_h;
  pstr_hp_filt->y_p[ch_idx * 2 + 0] = st3_h;
  pstr_hp_filt->y_p[ch_idx * 2 + 1] = st4_h;

  return;
}

/**
 *  impd_drc_two_band_filter_process
 *
 *  \brief two band filter process function
 *
 *  \param [out] output  output buffer
 *  \param [in/out] str_two_band_filt_bank  Pointer to two band filter structure
 *  \param [in] ch_idx  channel index
 *  \param [in] frame_len window length
 *  \param [in] input input buffer
 *
 *  \return VOID
 *
 */
VOID impd_drc_two_band_filter_process(FLOAT32 *output[],
                                      ia_two_band_filt_struct *str_two_band_filt_bank,
                                      WORD32 ch_idx, WORD32 frame_len, FLOAT32 *input)
{
  ia_iir_filter_struct *pstr_lp_filt = &str_two_band_filt_bank->str_lp;
  ia_iir_filter_struct *pstr_hp_filt = &str_two_band_filt_bank->str_hp;

  impd_drc_apply_low_high_filter(output, pstr_lp_filt, pstr_hp_filt, ch_idx, frame_len, input);
  return;
}

/**
 *  impd_drc_three_band_filter_process
 *
 *  \brief three band filter process function
 *
 *  \param [out] output  output buffer
 *  \param [in/out] str_three_band_filt_bank  Pointer to three band filter structure
 *  \param [in] ch_idx  channel index
 *  \param [in] frame_len window length
 *  \param [in] input input buffer
 *
 *  \return VOID
 *
 */
VOID impd_drc_three_band_filter_process(FLOAT32 *output[],
                                        ia_three_band_filt_struct *str_three_band_filt_bank,
                                        WORD32 ch_idx, WORD32 size, FLOAT32 *input)
{
  ia_iir_filter_struct *pstr_ap_filt;
  ia_iir_filter_struct *pstr_lp_filt = &str_three_band_filt_bank->str_lp_stage_1;
  ia_iir_filter_struct *pstr_hp_filt = &str_three_band_filt_bank->str_hp_stage_1;
  FLOAT32 *out1[2];
  out1[0] = output[0];
  out1[1] = output[1];

  impd_drc_apply_low_high_filter(out1, pstr_lp_filt, pstr_hp_filt, ch_idx, size, input);

  pstr_ap_filt = &str_three_band_filt_bank->str_ap_stage_2;

  impd_drc_iir_second_order_all_pass_filter(output[2], pstr_ap_filt, ch_idx, size, out1[1]);

  pstr_lp_filt = &str_three_band_filt_bank->str_lp_stage_2;
  pstr_hp_filt = &str_three_band_filt_bank->str_hp_stage_2;

  impd_drc_apply_low_high_filter(out1, pstr_lp_filt, pstr_hp_filt, ch_idx, size, out1[0]);

  return;
}

/**
 *  impd_drc_four_band_filter_process
 *
 *  \brief four band filter process function
 *
 *  \param [out] output  output buffer
 *  \param [in/out] str_four_band_filt_bank  Pointer to four band filter structure
 *  \param [in] ch_idx  channel index
 *  \param [in] win_size window length
 *  \param [in] input input buffer
 *
 *  \return VOID
 *
 */
VOID impd_drc_four_band_filter_process(FLOAT32 *output[],
                                       ia_four_band_filt_struct *str_four_band_filt_bank,
                                       WORD32 ch_idx, WORD32 win_size, FLOAT32 *input)
{
  ia_iir_filter_struct *pstr_ap_filt;
  ia_iir_filter_struct *pstr_lp_filt = &str_four_band_filt_bank->str_lp_stage_1;
  ia_iir_filter_struct *pstr_hp_filt = &str_four_band_filt_bank->str_hp_stage_1;
  FLOAT32 *out1[2];
  FLOAT32 *out2[2];

  out1[0] = output[0];
  out1[1] = output[1];
  out2[0] = output[2];
  out2[1] = output[3];

  impd_drc_apply_low_high_filter(out1, pstr_lp_filt, pstr_hp_filt, ch_idx, win_size, input);

  pstr_ap_filt = &str_four_band_filt_bank->str_ap_stage_2_low;

  impd_drc_iir_second_order_all_pass_filter(out1[0], pstr_ap_filt, ch_idx, win_size, out1[0]);

  pstr_ap_filt = &str_four_band_filt_bank->str_ap_stage_2_high;

  impd_drc_iir_second_order_all_pass_filter(out2[0], pstr_ap_filt, ch_idx, win_size, out1[1]);

  pstr_lp_filt = &str_four_band_filt_bank->str_lp_stage_3_low;
  pstr_hp_filt = &str_four_band_filt_bank->str_hp_stage_3_low;

  impd_drc_apply_low_high_filter(out1, pstr_lp_filt, pstr_hp_filt, ch_idx, win_size, out1[0]);

  pstr_lp_filt = &str_four_band_filt_bank->str_lp_stage_3_high;
  pstr_hp_filt = &str_four_band_filt_bank->str_hp_stage_3_high;

  impd_drc_apply_low_high_filter(out2, pstr_lp_filt, pstr_hp_filt, ch_idx, win_size, out2[0]);

  return;
}

/**
 *  impd_drc_all_pass_cascade_process
 *
 *  \brief all pass cascade process function
 *
 *  \param [in/out] str_all_pass_cascade  Pointer to all pass cascade structure
 *  \param [in] ch_idx  channel index
 *  \param [in] win_size window length
 *  \param [in] input input buffer
 *
 *  \return VOID
 *
 */
VOID impd_drc_all_pass_cascade_process(ia_all_pass_cascade_struct *str_all_pass_cascade,
                                       WORD32 ch_idx, WORD32 win_size, FLOAT32 *input)
{
  for (WORD32 idx = str_all_pass_cascade->num_filter; idx >= 0; idx--)
  {
    impd_drc_iir_second_order_all_pass_filter(
        input, &(str_all_pass_cascade->str_ap_cascade_filt[idx].str_ap_stage), ch_idx, win_size,
        input);
  }

  return;
}
/** @} */ /* End of DRCProcessing */
