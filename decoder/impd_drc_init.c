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
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_init.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_gain_dec.h"
#include "impd_drc_multi_band.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup DRCProcessing DRCProcessing
 * @ingroup  DRCProcessing
 * @brief DRC Decoder processing
 *
 * @{
 */

/**
 *  impd_drc_init_bitstream_config
 *
 *  \brief DRC bitstream config structure initialization
 *
 *  \param [out] pstr_drc_config  Pointer to DRC bitstream config structure
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impd_drc_init_bitstream_config(ia_drc_config *pstr_drc_config)
{
  WORD32 idx;
  for (idx = 0; idx < 4; idx++)
  {
    pstr_drc_config->str_drc_instruction_str[idx].drc_set_id = idx + 1;
    pstr_drc_config->str_drc_instruction_str[idx].drc_location = 1;
    pstr_drc_config->str_drc_instruction_str[idx].dwnmix_id_count = 1;
    pstr_drc_config->str_drc_instruction_str[idx].drc_set_target_loudness_value_lower = -63;
    pstr_drc_config->str_drc_instruction_str[idx].num_drc_ch_groups = 0;
    pstr_drc_config->str_drc_instruction_str[idx].band_count_of_ch_group[0] = 1;
    pstr_drc_config->str_drc_instruction_str[idx].band_count_of_ch_group[1] = 1;
    pstr_drc_config->str_drc_instruction_str[idx].gain_interpolation_type_for_channel_group[0] =
        1;
    pstr_drc_config->str_drc_instruction_str[idx].gain_interpolation_type_for_channel_group[1] =
        1;
    pstr_drc_config->str_drc_instruction_str[idx].time_delta_min_for_channel_group[0] = 32;
    pstr_drc_config->str_drc_instruction_str[idx].time_delta_min_for_channel_group[1] = 32;
    pstr_drc_config->str_drc_instruction_str[idx].channel_group_of_ch[1] = 1;
    pstr_drc_config->str_drc_instruction_str[idx].gain_element_count = 2;
  }
  pstr_drc_config->str_drc_instruction_str[0].drc_set_effect = 1;
  pstr_drc_config->str_drc_instruction_str[0].gain_set_index[1] =
      pstr_drc_config->str_drc_instruction_str[0].gain_set_index_for_channel_group[1] = 1;

  pstr_drc_config->str_drc_instruction_str[1].drc_set_effect = 2;
  pstr_drc_config->str_drc_instruction_str[1].gain_set_index[0] =
      pstr_drc_config->str_drc_instruction_str[1].gain_set_index_for_channel_group[0] = 1;
  pstr_drc_config->str_drc_instruction_str[1].gain_set_index[1] =
      pstr_drc_config->str_drc_instruction_str[1].gain_set_index_for_channel_group[1] = 2;

  pstr_drc_config->str_drc_instruction_str[2].drc_set_effect = 4;
  pstr_drc_config->str_drc_instruction_str[2].gain_set_index[0] =
      pstr_drc_config->str_drc_instruction_str[2].gain_set_index_for_channel_group[0] = 2;
  pstr_drc_config->str_drc_instruction_str[2].gain_set_index[1] =
      pstr_drc_config->str_drc_instruction_str[2].gain_set_index_for_channel_group[1] = 3;

  pstr_drc_config->str_drc_instruction_str[3].drc_set_effect = 32;
  pstr_drc_config->str_drc_instruction_str[3].gain_set_index[0] =
      pstr_drc_config->str_drc_instruction_str[3].gain_set_index_for_channel_group[0] = 3;
  pstr_drc_config->str_drc_instruction_str[3].gain_set_index[1] =
      pstr_drc_config->str_drc_instruction_str[3].gain_set_index_for_channel_group[1] = 0;

  pstr_drc_config->str_drc_instruction_str[4].drc_set_id = -1;
  pstr_drc_config->str_drc_instruction_str[4].dwnmix_id_count = 1;

  pstr_drc_config->str_drc_config_ext.drc_config_ext_type[0] = 2;
  pstr_drc_config->str_drc_config_ext.ext_bit_size[0] = 345;

  pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].version = 1;
  pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].drc_location = 1;
  pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].gain_set_count = 4;

  for (idx = 0; idx < pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].gain_set_count;
       idx++)
  {
    pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].gain_set_params[idx].gain_interp_type =
        1;
    pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[0].gain_set_params[idx].band_count = 1;
  }

  pstr_drc_config->sampling_rate = 0;
  pstr_drc_config->channel_layout.base_channel_count = 2;

  pstr_drc_config->drc_dwnmix_instructions_count = 0;
  pstr_drc_config->drc_coefficients_drc_count = 1;
  pstr_drc_config->drc_instructions_uni_drc_count = 4;
  pstr_drc_config->drc_instructions_count_plus = 5;
  pstr_drc_config->drc_description_basic_present = 0;
  pstr_drc_config->drc_coefficients_basic_count = 0;
  pstr_drc_config->drc_instructions_basic_count = 0;
  pstr_drc_config->drc_config_ext_present = 1;
  pstr_drc_config->apply_drc = 0;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_bitstream_dec
 *
 *  \brief DRC bitstream dec structure initialization
 *
 *  \param [out] pstr_drc_bs_dec  Pointer to DRC bitstream dec structure
 *  \param [in] sample_rate  sample rate
 *  \param [in] frame_size  framesize
 *  \param [in] delay_mode  delay mode
 *  \param [in] lfe_channel_map_count  lfe channel map count
 *  \param [in] lfe_channel_map  channel map value
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impd_drc_init_bitstream_dec(ia_drc_bits_dec_struct *pstr_drc_bs_dec,
                                         WORD32 sample_rate, WORD32 frame_size, WORD32 delay_mode,
                                         WORD32 lfe_channel_map_count, WORD32 *lfe_channel_map)
{
  ia_drc_params_bs_dec_struct *pstr_ia_drc_params = &pstr_drc_bs_dec->ia_drc_params_struct;
  WORD32 cnt;

  if (sample_rate < MIN_DRC_SAMP_FREQ)
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_SAMP_FREQ;
  }

  if ((frame_size < 1) || (frame_size > AUDIO_CODEC_FRAME_SIZE_MAX) ||
      (frame_size < 0.001f * sample_rate))
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_FRAME_SIZE;
  }

  if (lfe_channel_map_count > MAX_CHANNEL_COUNT)
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_CH_COUNT;
  }

  pstr_ia_drc_params->drc_frame_size = frame_size;
  pstr_ia_drc_params->delay_mode = delay_mode;
  pstr_ia_drc_params->delta_tmin_def = impd_drc_get_delta_tmin(sample_rate);
  if (pstr_ia_drc_params->delta_tmin_def > pstr_ia_drc_params->drc_frame_size)
  {
    return IA_MPEGD_DRC_INIT_FATAL_INVALID_DELTA_TMIN;
  }

  pstr_ia_drc_params->num_gain_values_max_default =
      pstr_ia_drc_params->drc_frame_size / pstr_ia_drc_params->delta_tmin_def;

  impd_drc_init_tbls(&pstr_drc_bs_dec->tables_default,
                     pstr_ia_drc_params->num_gain_values_max_default);

  if (lfe_channel_map_count < 0)
  {
    pstr_ia_drc_params->lfe_channel_map_count = -1;

    for (cnt = 0; cnt < MAX_CHANNEL_COUNT; cnt++)
    {
      pstr_ia_drc_params->lfe_channel_map[cnt] = 0;
    }
  }
  else
  {
    pstr_ia_drc_params->lfe_channel_map_count = lfe_channel_map_count;

    if (lfe_channel_map != NULL)
    {
      for (cnt = 0; cnt < lfe_channel_map_count; cnt++)
      {
        pstr_ia_drc_params->lfe_channel_map[cnt] = lfe_channel_map[cnt];
      }
    }
    else
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_params
 *
 *  \brief DRC params structure initialization
 *
 *  \param [out] pstr_ia_drc_params  Pointer to DRC params structure
 *  \param [in] frame_size  framesize
 *  \param [in] sample_rate  sample rate
 *  \param [in] gain_delay_samples  gain delay samples
 *  \param [in] delay_mode  delay mode
 *  \param [in] sub_band_domain_mode  subband domain mode
 *
 *  \return IA_ERRORCODE Error
 *
 */
static IA_ERRORCODE impd_drc_init_params(ia_drc_params_struct *pstr_ia_drc_params,
                                         WORD32 frame_size, WORD32 sample_rate,
                                         WORD32 gain_delay_samples, WORD32 delay_mode,
                                         WORD32 sub_band_domain_mode)
{
  WORD32 cnt;
  if (sample_rate < MIN_DRC_SAMP_FREQ)
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_SAMP_FREQ;
  }

  pstr_ia_drc_params->sample_rate = sample_rate;

  if ((frame_size < 1) || (frame_size > AUDIO_CODEC_FRAME_SIZE_MAX) ||
      (frame_size < 0.001f * sample_rate))
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_FRAME_SIZE;
  }

  pstr_ia_drc_params->drc_frame_size = frame_size;

  if ((delay_mode != DELAY_MODE_REGULAR) && (delay_mode != DELAY_MODE_LOW))
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_DELAY_MODE;
  }

  pstr_ia_drc_params->delay_mode = delay_mode;

  if ((gain_delay_samples > MAX_SIGNAL_DELAY) || (gain_delay_samples < 0))
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_DELAY_SAMPLES;
  }
  else
  {
    pstr_ia_drc_params->gain_delay_samples = gain_delay_samples;
  }

  pstr_ia_drc_params->drc_set_counter = 0;
  pstr_ia_drc_params->multiband_sel_drc_idx = -1;

  for (cnt = 0; cnt < SEL_DRC_COUNT; cnt++)
  {
    pstr_ia_drc_params->sel_drc_array[cnt].drc_instrns_idx = -1;
    pstr_ia_drc_params->sel_drc_array[cnt].downmix_instrns_idx = -1;
    pstr_ia_drc_params->sel_drc_array[cnt].drc_coeff_idx = -1;
  }

  pstr_ia_drc_params->delta_tmin_def = impd_drc_get_delta_tmin(sample_rate);

  if (pstr_ia_drc_params->delta_tmin_def > pstr_ia_drc_params->drc_frame_size)
  {
    return IA_MPEGD_DRC_INIT_FATAL_INVALID_DELTA_TMIN;
  }

  if ((sub_band_domain_mode >= SUBBAND_DOMAIN_MODE_OFF) &&
      (sub_band_domain_mode <= SUBBAND_DOMAIN_MODE_STFT256))
  {
    pstr_ia_drc_params->sub_band_domain_mode = sub_band_domain_mode;
  }
  else
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_SUBBAND_DOMAIN_MODE;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_update_effect_type_request
 *
 *  \brief Update effect type request
 *
 *  \param [out] num_drc_eff_type_req  Number of requests
 *  \param [out] drc_eff_type_req  effect type request
 *  \param [in] drc_eff_type_req_plus_fallbacks  drc_eff_type_req_plus_fallbacks
 *
 *
 *
 */
VOID impd_drc_update_effect_type_request(WORD8 *num_drc_eff_type_req, WORD8 *drc_eff_type_req,
                                         UWORD64 drc_eff_type_req_plus_fallbacks)
{
  for (WORD32 idx = 0; idx < 8; idx++)
  {
    if (idx == 0 || (drc_eff_type_req_plus_fallbacks & 15) != 0)
    {
      drc_eff_type_req[idx] = drc_eff_type_req_plus_fallbacks & 15;
      drc_eff_type_req_plus_fallbacks >>= 4;
      *num_drc_eff_type_req += 1;
    }
    else
    {
      break;
    }
  }
  return;
}

/**
 *  impd_drc_select_coefficients
 *
 *  \brief Select DRC coefficients
 *
 *  \param [out] ppstr_drc_coefficients  Pointer to DRC coefficients array
 *  \param [out] drc_coefficients_selected  Pointer to DRC coefficients selected
 *  \param [out] pstr_drc_config  Pointer to DRC config structure
 *
 *  \return IA_ERRORCODE Error
 *
 */
static IA_ERRORCODE
impd_drc_select_coefficients(ia_drc_uni_drc_coeffs_struct **ppstr_drc_coefficients,
                             WORD32 *drc_coefficients_selected, ia_drc_config *pstr_drc_config)
{
  WORD32 cnt;
  WORD32 coeff0 = -1;

  for (cnt = 0; cnt < pstr_drc_config->drc_coefficients_drc_count; cnt++)
  {
    if (pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt].drc_location == 1)
    {
      if (pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt].version == 0)
      {
        coeff0 = cnt;
        *drc_coefficients_selected = coeff0;
      }
      else
      {
        *drc_coefficients_selected = cnt;
      }
    }
  }

  if (coeff0 >= 0)
  {
    *ppstr_drc_coefficients = &(pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[coeff0]);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_set_channel_group_info
 *
 *  \brief Set channel group info
 *
 *  \param [in,out] pstr_drc_instructions_uni_drc  Pointer to unidrc instruction struct
 *
 *
 *
 */
static VOID
impd_drc_set_channel_group_info(ia_drc_instructions_struct *pstr_drc_instructions_uni_drc)
{
  WORD32 ch, grp;
  if ((pstr_drc_instructions_uni_drc->downmix_id[0] == ID_FOR_ANY_DOWNMIX) ||
      (pstr_drc_instructions_uni_drc->dwnmix_id_count > 1))
  {
    WORD32 idx = pstr_drc_instructions_uni_drc->gain_set_index[0];
    for (ch = 0; ch < pstr_drc_instructions_uni_drc->audio_num_chan; ch++)
    {
      pstr_drc_instructions_uni_drc->channel_group_of_ch[ch] = (idx >= 0) ? 0 : -1;
    }
  }

  for (grp = 0; grp < pstr_drc_instructions_uni_drc->num_drc_ch_groups; grp++)
  {
    pstr_drc_instructions_uni_drc->num_chan_per_ch_group[grp] = 0;
    for (ch = 0; ch < pstr_drc_instructions_uni_drc->audio_num_chan; ch++)
    {
      if (pstr_drc_instructions_uni_drc->channel_group_of_ch[ch] == grp)
      {
        pstr_drc_instructions_uni_drc->num_chan_per_ch_group[grp]++;
      }
    }
  }
}

/**
 *  impd_drc_set_multiband
 *
 *  \brief Set multiband info
 *
 *  \param [in,out] pstr_drc_instructions_uni_drc  Pointer to unidrc instruction struct
 *  \param [out] selected_drc_is_multiband  variable to update if multiband
 *  \param [in] pstr_ia_drc_params  Pointer to drc params struct
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE
impd_drc_set_multiband(ia_drc_instructions_struct *pstr_drc_instructions_uni_drc,
                       WORD32 *selected_drc_is_multiband,
                       ia_drc_params_struct *pstr_ia_drc_params)
{
  WORD32 ch, grp;
  if (pstr_drc_instructions_uni_drc->drc_set_effect &
      (IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF))
  {
    pstr_drc_instructions_uni_drc->multiband_audio_sig_count =
        pstr_drc_instructions_uni_drc->audio_num_chan;
  }
  else
  {
    pstr_drc_instructions_uni_drc->multiband_audio_sig_count = 0;
    for (ch = 0; ch < pstr_drc_instructions_uni_drc->audio_num_chan; ch++)
    {
      grp = pstr_drc_instructions_uni_drc->channel_group_of_ch[ch];
      if (grp < 0)
      {
        pstr_drc_instructions_uni_drc->multiband_audio_sig_count++;
      }
      else
      {
        pstr_drc_instructions_uni_drc->multiband_audio_sig_count +=
            pstr_drc_instructions_uni_drc->band_count_of_ch_group[grp];
      }
    }
  }

  for (grp = 0; grp < pstr_drc_instructions_uni_drc->num_drc_ch_groups; grp++)
  {
    if (pstr_drc_instructions_uni_drc->band_count_of_ch_group[grp] > 1)
    {
      if (pstr_ia_drc_params->multiband_sel_drc_idx != -1)
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      }
      *selected_drc_is_multiband = 1;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_selected_drc_set
 *
 *  \brief Initialise selected drc set
 *
 *  \param [in,out] pstr_ia_drc_params  Pointer to DRC params structure
 *  \param [in,out] pstr_ia_filter_banks  Pointer to filter banks structure
 *  \param [in,out] pstr_overlap_params  Pointer to overlap params structure
 *  \param [in] audio_num_chan  No of audio channels
 *  \param [in] drc_set_id_selected  drc set id selected
 *  \param [in] downmix_id_selected  downmix id selected
 *  \param [in] pstr_drc_config  Pointer to DRC config structure
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impd_drc_init_selected_drc_set(ia_drc_params_struct *pstr_ia_drc_params,
                                            ia_filter_banks_struct *pstr_ia_filter_banks,
                                            ia_drc_overlap_params_struct *pstr_overlap_params,
                                            WORD32 audio_num_chan, WORD32 drc_set_id_selected,
                                            WORD32 downmix_id_selected,
                                            ia_drc_config *pstr_drc_config)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 cnt;
  WORD32 channel_count = 0;
  WORD32 drc_instructions_selected = -1;
  WORD32 downmix_instructions_selected = -1;
  WORD32 drc_coefficients_selected = -1;
  WORD32 *selected_drc_is_multiband = &pstr_drc_config->multi_band_present;
  *selected_drc_is_multiband = 0;

  ia_drc_instructions_struct *pstr_drc_instructions_uni_drc = NULL;
  ia_drc_uni_drc_coeffs_struct *drc_coefficients_uni_drc = NULL;

  for (cnt = 0; cnt < pstr_drc_config->drc_instructions_count_plus; cnt++)
  {
    if (pstr_drc_config->str_drc_instruction_str[cnt].drc_set_id == drc_set_id_selected)
      break;
  }
  if (cnt == pstr_drc_config->drc_instructions_count_plus)
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR;
  }
  drc_instructions_selected = cnt;
  pstr_ia_drc_params->sel_drc_array[pstr_ia_drc_params->drc_set_counter].drc_instrns_idx =
      drc_instructions_selected;
  pstr_drc_instructions_uni_drc =
      &(pstr_drc_config->str_drc_instruction_str[drc_instructions_selected]);

  if (pstr_drc_config->drc_coefficients_drc_count &&
      pstr_drc_config->str_p_loc_drc_coefficients_uni_drc->drc_frame_size_present)
  {
    if (pstr_drc_config->str_p_loc_drc_coefficients_uni_drc->drc_frame_size !=
        pstr_ia_drc_params->drc_frame_size)
    {
      return IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR;
    }
  }

  switch (downmix_id_selected)
  {
  case ID_FOR_BASE_LAYOUT:
    channel_count = pstr_drc_config->channel_layout.base_channel_count;
    break;
  case ID_FOR_ANY_DOWNMIX:
    channel_count = audio_num_chan;
    break;
  default:
    for (cnt = 0; cnt < pstr_drc_config->ia_mpegh3da_dwnmix_instructions_count; cnt++)
    {
      if (pstr_drc_config->mpegh3da_dwnmix_instructions[cnt].downmix_id == downmix_id_selected)
        break;
    }
    if (cnt == pstr_drc_config->ia_mpegh3da_dwnmix_instructions_count)
    {
      return (IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR);
    }
    channel_count = pstr_drc_config->mpegh3da_dwnmix_instructions[cnt].target_channel_count;

    downmix_instructions_selected = cnt;
    break;
  }
  pstr_ia_drc_params->sel_drc_array[pstr_ia_drc_params->drc_set_counter].downmix_instrns_idx =
      downmix_instructions_selected;

  pstr_drc_instructions_uni_drc->audio_num_chan = channel_count;

  if (pstr_drc_instructions_uni_drc->drc_set_id <= 0)
  {
    drc_coefficients_selected = 0;
  }
  else
  {
    err = impd_drc_select_coefficients(&drc_coefficients_uni_drc, &drc_coefficients_selected,
                                       pstr_drc_config);
    if (err)
      return err;
  }

  pstr_ia_drc_params->sel_drc_array[pstr_ia_drc_params->drc_set_counter].drc_coeff_idx =
      drc_coefficients_selected;

  impd_drc_set_channel_group_info(pstr_drc_instructions_uni_drc);
  err = impd_drc_set_multiband(pstr_drc_instructions_uni_drc, selected_drc_is_multiband,
                               pstr_ia_drc_params);
  if (err)
    return err;

  if (*selected_drc_is_multiband == 1)
  {
    pstr_ia_drc_params->multiband_sel_drc_idx = pstr_ia_drc_params->drc_set_counter;
    pstr_ia_drc_params->sub_band_domain_mode = SUBBAND_DOMAIN_MODE_STFT256;
    pstr_ia_drc_params->gain_delay_samples = DELAY_SAMPLES;

    err = impd_drc_init_all_filter_banks(
        pstr_ia_filter_banks, drc_coefficients_uni_drc,
        &(pstr_drc_config->str_drc_instruction_str[drc_instructions_selected]));
    if (err)
      return (err);

    err = impd_drc_init_overlap_weight(
        pstr_overlap_params, drc_coefficients_uni_drc,
        &(pstr_drc_config->str_drc_instruction_str[drc_instructions_selected]),
        pstr_ia_drc_params->sub_band_domain_mode);
    if (err)
      return (err);
  }

  pstr_ia_drc_params->drc_set_counter++;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_decode
 *
 *  \brief DRC decoder initialization
 *
 *  \param [out] pstr_drc_gain_dec  Pointer to DRC gain decoder structure
 *  \param [in] frame_size  framesize
 *  \param [in] sample_rate  sample rate
 *  \param [in] gain_delay_samples  gain delay samples
 *  \param [in] delay_mode  delay mode
 *  \param [in] sub_band_domain_mode  subband domain mode
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impd_drc_init_decode(ia_drc_gain_dec_struct *pstr_drc_gain_dec, WORD32 frame_size,
                                  WORD32 sample_rate, WORD32 gain_delay_samples,
                                  WORD32 delay_mode, WORD32 sub_band_domain_mode)
{
  return impd_drc_init_params(&pstr_drc_gain_dec->ia_drc_params_struct, frame_size, sample_rate,
                              gain_delay_samples, delay_mode, sub_band_domain_mode);
}

/**
 *  impd_drc_init_decode_post_config
 *
 *  \brief Initialise selected drc set
 *
 *  \param [in,out] pstr_drc_gain_dec  Pointer to DRC gain dec structure
 *  \param [in] audio_num_chan  No of audio channels
 *  \param [in] drc_set_id_processed  drc set id processed
 *  \param [in] downmix_id_processed  downmix id processed
 *  \param [in] num_sets_processed  no of sets processed
 *  \param [in] channel_offset  channel offset
 *  \param [in] num_ch_process  no of channels
 *  \param [in] pstr_drc_config  Pointer to DRC config structure
 *  \param [in] mem_ptr  Memory pointer
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impd_drc_init_decode_post_config(ia_drc_gain_dec_struct *pstr_drc_gain_dec,
                                              WORD32 audio_num_chan, WORD32 *drc_set_id_processed,
                                              WORD32 *downmix_id_processed,
                                              WORD32 num_sets_processed, WORD32 channel_offset,
                                              WORD32 num_ch_process,
                                              ia_drc_config *pstr_drc_config, pVOID *mem_ptr)
{

  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 count, max_multiband_audio_sig_count = 0;
  ia_drc_params_struct *p_drc_params_struct = &pstr_drc_gain_dec->ia_drc_params_struct;

  for (count = num_sets_processed - 1; count >= 0; count--)
  {
    err_code = impd_drc_init_selected_drc_set(
        p_drc_params_struct, &pstr_drc_gain_dec->ia_filter_banks_struct,
        &pstr_drc_gain_dec->str_overlap_params, audio_num_chan, drc_set_id_processed[count],
        downmix_id_processed[count], pstr_drc_config);
    if (err_code != IA_MPEGH_DEC_NO_ERROR)
    {
      return (err_code);
    }
  }

  pstr_drc_gain_dec->audio_num_chan = audio_num_chan;

  for (WORD32 cnt = SEL_DRC_COUNT - 1; cnt >= 0; cnt--)
  {
    if (0 <= p_drc_params_struct->sel_drc_array[cnt].drc_instrns_idx)
    {
      ia_drc_instructions_struct *drc_instruction_str = &(
          pstr_drc_config
              ->str_drc_instruction_str[p_drc_params_struct->sel_drc_array[cnt].drc_instrns_idx]);
      if (0 < drc_instruction_str->gain_element_count)
      {
        pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt].pstr_buf_interp =
            (ia_drc_interp_buf_struct *)*mem_ptr;
        *mem_ptr = (pVOID)(
            (SIZE_T)*mem_ptr +
            drc_instruction_str->gain_element_count * sizeof(ia_drc_interp_buf_struct) + 32);
        pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt].buf_interp_cnt =
            drc_instruction_str->gain_element_count;
        for (count = pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt].buf_interp_cnt - 1;
             count >= 0; count--)
        {

          pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt]
              .pstr_buf_interp[count]
              .str_node.loc_gain_db = 0;
          pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt]
              .pstr_buf_interp[count]
              .str_node.slope = 0;
          pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt]
              .pstr_buf_interp[count]
              .str_node.time = 0;
          pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt]
              .pstr_buf_interp[count]
              .prev_node.time = -1;

          for (WORD32 j = 2 * AUDIO_CODEC_FRAME_SIZE_MAX + MAX_SIGNAL_DELAY - 1; j >= 0; j--)
          {

            pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[cnt]
                .pstr_buf_interp[count]
                .lpcm_gains[j] = 1.0f;
          }
        }
      }
    }
  }

  if (-1 != channel_offset && -1 != num_ch_process)
  {
    pstr_drc_gain_dec->ia_drc_params_struct.channel_offset = channel_offset;
    pstr_drc_gain_dec->ia_drc_params_struct.num_ch_process = num_ch_process;
  }
  else
  {
    pstr_drc_gain_dec->ia_drc_params_struct.channel_offset = 0;
    pstr_drc_gain_dec->ia_drc_params_struct.num_ch_process = -1;
  }

  for (count = p_drc_params_struct->drc_set_counter - 1; count >= 0; count--)
  {
    ia_drc_instructions_struct *drc_instruction_str = &(
        pstr_drc_config
            ->str_drc_instruction_str[p_drc_params_struct->sel_drc_array[count].drc_instrns_idx]);
    max_multiband_audio_sig_count =
        ia_max_int(max_multiband_audio_sig_count, drc_instruction_str->multiband_audio_sig_count);
  }

  pstr_drc_gain_dec->audio_band_buffer.frame_size = p_drc_params_struct->drc_frame_size;
  pstr_drc_gain_dec->audio_band_buffer.multiband_audio_sig_count = max_multiband_audio_sig_count;
  pstr_drc_gain_dec->audio_band_buffer.non_interleaved_audio = *mem_ptr;

  *mem_ptr = (pVOID)((SIZE_T)*mem_ptr + (max_multiband_audio_sig_count * sizeof(FLOAT32 *)));

  for (count = max_multiband_audio_sig_count - 1; count >= 0; count--)
  {
    pstr_drc_gain_dec->audio_band_buffer.non_interleaved_audio[count] = *mem_ptr;
    *mem_ptr =
        (pVOID)((SIZE_T)*mem_ptr + (p_drc_params_struct->drc_frame_size * sizeof(FLOAT32)));
  }

  return err_code;
}
/** @} */ /* End of DRCProcessing */