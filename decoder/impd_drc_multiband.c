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

#include <impeghd_type_def.h>
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_multi_band.h"
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
 *  impd_drc_generate_slope
 *
 *  \brief Generate slope
 *
 *  \param [out] response  response
 *  \param [in] sub_bands_count  no of subbands
 *  \param [in] fcenter_norm_sb  Center frequency of subband
 *  \param [in] fcross_norm_l Normalized crossover frequency low
 *  \param [in] fcross_norm_h Normalized crossover frequency high
 *
 *
 *
 */
static VOID impd_drc_generate_slope(FLOAT32 *response, WORD32 sub_bands_count,
                                    FLOAT32 *fcenter_norm_sb, FLOAT32 fcross_norm_l,
                                    FLOAT32 fcross_norm_h)
{
  WORD32 band;

  FLOAT32 norm = ia_mul_flt(ia_mul_flt(0.05f, FILT_SLOPE), INV_LOG10_2);
  FLOAT32 pow_val1 = (FLOAT32)pow(fcross_norm_l, norm);
  FLOAT32 pow_val2 = (FLOAT32)pow(fcross_norm_h, norm);

  for (band = 0; band < sub_bands_count; band++)
  {
    if (fcross_norm_l > fcenter_norm_sb[band])
    {
      response[band] =
          pow_val1 / (FLOAT32)(ia_drc_impd_drc_table[(WORD32)(fcenter_norm_sb[band] * 512)]);
    }
    else if (fcross_norm_h > fcenter_norm_sb[band])
    {
      response[band] = 1.0f;
    }
    else
    {
      response[band] = (ia_drc_impd_drc_table[(WORD32)(fcenter_norm_sb[band] * 512)]) / pow_val2;
    }
  }
  return;
}

/**
 *  impd_drc_generate_overlap_weights
 *
 *  \brief Generate overlap weights
 *
 *  \param [out] pstr_group_overlap_params  Pointer to overlap params structure
 *  \param [in] pstr_p_loc_drc_coefficients_uni_drc  Pointer to uni drc coefficients structure
 *  \param [in] pstr_drc_instruction_str  Pointer to drc instructions structure
 *  \param [in] sb_domain_mode  subband domain mode
 *  \param [in] index  Array index
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_generate_overlap_weights(
    ia_drc_group_overlap_params_struct *pstr_group_overlap_params,
    ia_drc_uni_drc_coeffs_struct *pstr_p_loc_drc_coefficients_uni_drc,
    ia_drc_instructions_struct *pstr_drc_instruction_str, WORD32 sb_domain_mode, WORD32 index)
{
  ia_drc_gain_params_struct *gain_params;
  FLOAT32 fcenter_norm_sb[AUDIO_CODEC_SUBBAND_COUNT_MAX];
  FLOAT32 w_norm[AUDIO_CODEC_SUBBAND_COUNT_MAX];
  FLOAT32 fcross_norm_l, fcross_norm_h;
  WORD32 dec_sb_count = AUDIO_CODEC_SUBBAND_COUNT_QMF64;
  WORD32 band, cnt, start_sb_index = 0, stop_sb_index = 0;
  WORD32 drc_bands_count, drc_band_type;

  drc_bands_count = pstr_drc_instruction_str->band_count_of_ch_group[index];
  drc_band_type =
      pstr_p_loc_drc_coefficients_uni_drc
          ->gain_set_params[pstr_drc_instruction_str->gain_set_index_for_channel_group[index]]
          .drc_band_type;
  gain_params =
      pstr_p_loc_drc_coefficients_uni_drc
          ->gain_set_params[pstr_drc_instruction_str->gain_set_index_for_channel_group[index]]
          .gain_params;

  if (sb_domain_mode == SUBBAND_DOMAIN_MODE_STFT256)
  {
    for (cnt = 0; cnt < AUDIO_CODEC_SUBBAND_COUNT_STFT256; cnt++)
    {
      fcenter_norm_sb[cnt] = ((cnt) / (2.0f * AUDIO_CODEC_SUBBAND_COUNT_STFT256));
    }
    dec_sb_count = AUDIO_CODEC_SUBBAND_COUNT_STFT256;
  }
  else
  {
    return IA_MPEGD_DRC_UNSUPPORTED_SUBBAND_DOMAIN_MODE;
  }

  if (drc_band_type != 1)
  {
    start_sb_index = 0;
    for (band = 0; band < drc_bands_count; band++)
    {
      if (band == drc_bands_count - 1)
      {
        stop_sb_index = dec_sb_count - 1;
      }
      else
      {
        stop_sb_index = gain_params[band + 1].start_subband_index - 1;
      }

      for (cnt = 0; cnt < dec_sb_count; cnt++)
      {
        if (cnt >= start_sb_index && cnt <= stop_sb_index)
        {
          pstr_group_overlap_params->str_band_overlap_params[band].overlap_weight[cnt] = 1.0;
        }
        else
        {
          pstr_group_overlap_params->str_band_overlap_params[band].overlap_weight[cnt] = 0;
        }
      }
      start_sb_index = stop_sb_index + 1;
    }
  }
  else
  {
    fcross_norm_l = 0;
    for (band = 0; band < drc_bands_count; band++)
    {
      if (band == drc_bands_count - 1)
      {
        fcross_norm_h = 0.5f;
      }
      else
      {

        fcross_norm_h =
            ia_drc_normal_cross_freq[gain_params[band + 1].crossover_freq_idx].freq_cross_norm;
      }

      impd_drc_generate_slope(
          pstr_group_overlap_params->str_band_overlap_params[band].overlap_weight, dec_sb_count,
          fcenter_norm_sb, fcross_norm_l, fcross_norm_h);

      fcross_norm_l = fcross_norm_h;
    }
    for (cnt = 0; cnt < dec_sb_count; cnt++)
    {
      w_norm[cnt] = pstr_group_overlap_params->str_band_overlap_params[0].overlap_weight[cnt];
      for (band = 1; band < drc_bands_count; band++)
      {
        w_norm[cnt] +=
            pstr_group_overlap_params->str_band_overlap_params[band].overlap_weight[cnt];
      }
    }

    for (cnt = 0; cnt < dec_sb_count; cnt++)
    {
      for (band = 0; band < drc_bands_count; band++)
      {
        pstr_group_overlap_params->str_band_overlap_params[band].overlap_weight[cnt] =
            ((FLOAT32)pstr_group_overlap_params->str_band_overlap_params[band]
                 .overlap_weight[cnt] /
             w_norm[cnt]);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_overlap_weight
 *
 *  \brief Initialize overlap weights
 *
 *  \param [out] pstr_overlap_params  Pointer to overlap params structure
 *  \param [in] pstr_p_loc_drc_coefficients_uni_drc  Pointer to uni drc coefficients structure
 *  \param [in] pstr_drc_instruction_str  Pointer to drc instructions structure
 *  \param [in] sb_domain_mode  subband domain mode
 *
 *  \return IA_ERRORCODE error if any
 *
 */
IA_ERRORCODE
impd_drc_init_overlap_weight(ia_drc_overlap_params_struct *pstr_overlap_params,
                             ia_drc_uni_drc_coeffs_struct *pstr_p_loc_drc_coefficients_uni_drc,
                             ia_drc_instructions_struct *pstr_drc_instruction_str,
                             WORD32 sb_domain_mode)
{
  WORD32 grp;
  IA_ERRORCODE error = IA_MPEGH_DEC_NO_ERROR;
  for (grp = 0; grp < pstr_drc_instruction_str->num_drc_ch_groups; grp++)
  {
    if (pstr_drc_instruction_str->band_count_of_ch_group[grp] > 1)
    {
      error = impd_drc_generate_overlap_weights(
          &(pstr_overlap_params->str_grp_overlap_params[grp]),
          pstr_p_loc_drc_coefficients_uni_drc, pstr_drc_instruction_str, sb_domain_mode, grp);
      if (error)
      {
        return error;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCProcessing */