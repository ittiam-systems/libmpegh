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
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_error_codes.h"
#include "impeghd_mhas_parse.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_parser.h"
#include "impd_drc_rom.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup DRCConfigPayloadParse DRCConfigPayloadParse
 * @ingroup  DRCConfigPayloadParse
 * @brief DRC Config payload parsing
 *
 * @{
 */

/**
 *  impd_drc_gen_instructions_for_drc_off
 *
 *  \brief Function to generate instructions
 *
 *  \param [out]  pstr_drc_config   Pointer to drc config structure
 *
 *
 *
 */
static VOID impd_drc_gen_instructions_for_drc_off(ia_drc_config *pstr_drc_config)
{
  ia_drc_instructions_struct *pstr_drc_instruction;
  WORD32 instrn_cnt, cnt, id;
  id = -1;

  pstr_drc_config->drc_dwnmix_instructions_count = 1;
  pstr_drc_config->dwnmix_instructions[0].downmix_coefficients_present = 0;
  pstr_drc_config->dwnmix_instructions[0].downmix_id = -1;
  pstr_drc_config->dwnmix_instructions[0].target_channel_count = 0;
  pstr_drc_config->dwnmix_instructions[0].target_layout = 0;

  cnt = pstr_drc_config->drc_instructions_uni_drc_count;

  pstr_drc_instruction = &(pstr_drc_config->str_drc_instruction_str[cnt]);
  memset(pstr_drc_instruction, 0, sizeof(ia_drc_instructions_struct));
  pstr_drc_instruction->drc_set_id = id;
  id--;

  pstr_drc_instruction->depends_on_drc_set_present = 0;
  pstr_drc_instruction->drc_apply_to_dwnmix = 0;
  pstr_drc_instruction->downmix_id[0] = ID_FOR_BASE_LAYOUT;
  pstr_drc_instruction->dwnmix_id_count = 1;
  pstr_drc_instruction->gain_element_count = 0;
  pstr_drc_instruction->no_independent_use = 0;

  for (instrn_cnt = 1; instrn_cnt < pstr_drc_config->drc_dwnmix_instructions_count + 1;
       instrn_cnt++)
  {
    pstr_drc_instruction = &(pstr_drc_config->str_drc_instruction_str[cnt + instrn_cnt]);
    memset(pstr_drc_instruction, 0, sizeof(ia_drc_instructions_struct));
    pstr_drc_instruction->drc_set_id = id;
    id--;

    pstr_drc_instruction->depends_on_drc_set_present = 0;
    pstr_drc_instruction->drc_apply_to_dwnmix = 0;
    pstr_drc_instruction->downmix_id[0] =
        pstr_drc_config->dwnmix_instructions[instrn_cnt - 1].downmix_id;
    pstr_drc_instruction->dwnmix_id_count = 1;
    pstr_drc_instruction->gain_element_count = 0;
    pstr_drc_instruction->no_independent_use = 0;
  }
  pstr_drc_config->drc_instructions_count_plus = pstr_drc_config->drc_instructions_uni_drc_count +
                                                 pstr_drc_config->drc_dwnmix_instructions_count +
                                                 1;
  return;
}

/**
 *  impd_drc_parse_config_ext
 *
 *  \brief Function to parse drc config extension payload
 *
 *  \param [out]  pstr_drc_config_ext Pointer to drc config ext structure
 *  \param [in]   pstr_it_bit_buff    Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_parse_config_ext(ia_drc_config_ext *pstr_drc_config_ext,
                                              ia_bit_buf_struct *pstr_it_bit_buff)
{
  WORD32 size, idx = 0;
  WORD32 bit_size_len, bit_size_ext, bit_size;

  pstr_drc_config_ext->drc_config_ext_type[idx] =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

  while (UNIDRCCONFEXT_TERM != pstr_drc_config_ext->drc_config_ext_type[idx])
  {
    if ((EXT_COUNT_MAX - 1) <= idx)
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }
    bit_size_len = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

    bit_size_ext = bit_size_len + 4;

    bit_size = ia_core_coder_read_bits_buf(pstr_it_bit_buff, bit_size_ext);
    pstr_drc_config_ext->ext_bit_size[idx] = bit_size + 1;

    for (size = 0; size < pstr_drc_config_ext->ext_bit_size[idx]; size++)
    {
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    }

    idx++;
    pstr_drc_config_ext->drc_config_ext_type[idx] =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_gen_instructions_derived_data
 *
 *  \brief Function to generate drc instructions derived data
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instruction structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  pstr_ia_drc_params   Pointer to drc params bitstreamdec structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_gen_instructions_derived_data(ia_drc_instructions_struct *pstr_drc_instruction,
                                       ia_drc_config *pstr_drc_config,
                                       ia_drc_params_bs_dec_struct *pstr_ia_drc_params)
{
  ia_drc_uni_drc_coeffs_struct *pstr_p_loc_drc_coefficients_uni_drc = NULL;
  WORD32 gain_element_cnt = 0;
  WORD32 cnt, grp, seq;

  for (cnt = 0; cnt < pstr_drc_config->drc_coefficients_drc_count; cnt++)
  {
    if (pstr_drc_instruction->drc_location ==
        pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt].drc_location)
    {
      break;
    }
  }
  if (cnt == pstr_drc_config->drc_coefficients_drc_count)
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR;
  }
  pstr_p_loc_drc_coefficients_uni_drc =
      &(pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt]);

  for (grp = 0; grp < pstr_drc_instruction->num_drc_ch_groups; grp++)
  {
    seq = pstr_drc_instruction->gain_set_index_for_channel_group[grp];
    if (seq >= pstr_p_loc_drc_coefficients_uni_drc->gain_set_count)
    {
      return IA_MPEGD_DRC_INIT_FATAL_INVALID_GAIN_SET_INDEX;
    }

    pstr_drc_instruction->gain_interpolation_type_for_channel_group[grp] =
        pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[seq].gain_interp_type;

    if (0 == pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[seq].time_delt_min_flag)
    {
      pstr_drc_instruction->time_delta_min_for_channel_group[grp] =
          pstr_ia_drc_params->delta_tmin_def;
    }
    else
    {
      pstr_drc_instruction->time_delta_min_for_channel_group[grp] =
          pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[seq].time_delt_min_val;
    }

    pstr_drc_instruction->time_alignment_for_channel_group[grp] =
        pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[seq].time_alignment;
  }

  if ((IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF) &
      pstr_drc_instruction->drc_set_effect)
  {
    pstr_drc_instruction->gain_element_count = pstr_drc_instruction->num_drc_ch_groups;
  }
  else
  {
    for (grp = 0; grp < pstr_drc_instruction->num_drc_ch_groups; grp++)
    {
      {
        WORD32 band_cnt;
        seq = pstr_drc_instruction->gain_set_index_for_channel_group[grp];
        band_cnt = pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[seq].band_count;
        pstr_drc_instruction->band_count_of_ch_group[grp] = band_cnt;
        gain_element_cnt += band_cnt;
      }
    }
    pstr_drc_instruction->gain_element_count = gain_element_cnt;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_decode_method_value
 *
 *  \brief Function to decode method value
 *
 *  \param [out]  method_value   method value
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  method_defn   method definition
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_decode_method_value(FLOAT32 *method_value,
                                                 ia_bit_buf_struct *pstr_it_bit_buff,
                                                 WORD32 method_defn)
{
  WORD32 temp;
  FLOAT32 value;

  switch (method_defn)
  {
  case IA_DRC_METHOD_DEFINITION_SHORT_TERM_LOUDNESS:
    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);
    value = -116.f + temp * 0.5f;
    break;
  case IA_DRC_METHOD_DEFINITION_ROOM_TYPE:
    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 2);
    value = (FLOAT32)temp;
    break;
  case IA_DRC_METHOD_DEFINITION_MIXING_LEVEL:
    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 5);
    value = temp + 80.0f;
    break;
  case IA_DRC_METHOD_DEFINITION_LOUDNESS_RANGE:
    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);

    if (0 == temp)
      value = 0.0f;
    else if (128 >= temp)
      value = temp * 0.25f;
    else if (204 >= temp)
      value = 0.5f * temp - 32.0f;
    else
      value = temp - 134.0f;
    break;
  case IA_DRC_METHOD_DEFINITION_SHORT_TERM_LOUDNESS_MAX:
  case IA_DRC_METHOD_DEFINITION_MOMENTARY_LOUDNESS_MAX:
  case IA_DRC_METHOD_DEFINITION_MAX_OF_LOUDNESS_RANGE:
  case IA_DRC_METHOD_DEFINITION_ANCHOR_LOUDNESS:
  case IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS:
  case IA_DRC_METHOD_DEFINITION_UNKNOWN_OTHER:
    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);
    value = -57.75f + temp * 0.25f;
    break;
  default:
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_METHOD_DEFINITION;
    break;
  }
  *method_value = value;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_loudness_measure
 *
 *  \brief Function to parse loudness measure
 *
 *  \param [in,out]  pstr_loudness_measure   Pointer to loudness measure structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_loudness_measure(ia_drc_loudness_measure_struct *pstr_loudness_measure,
                                ia_bit_buf_struct *pstr_it_bit_buff)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 temp;

  pstr_loudness_measure->method_def = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

  err_code = impd_drc_decode_method_value(&(pstr_loudness_measure->method_value),
                                          pstr_it_bit_buff, pstr_loudness_measure->method_def);
  if (err_code)
    return err_code;

  temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

  pstr_loudness_measure->measurement_system = (temp >> 2) & 0xf;
  if (IA_DRC_MEASUREMENT_SYSTEM_RESERVED_E < pstr_loudness_measure->measurement_system)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }

  pstr_loudness_measure->reliability = temp & 3;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_loudness_info
 *
 *  \brief Function to parse loudness info
 *
 *  \param [out]  pstr_loudness_info   Pointer to loudness info structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_parse_loudness_info(ia_drc_loudness_info_struct *pstr_loudness_info,
                                                 ia_bit_buf_struct *pstr_it_bit_buff)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 sample_peak_level, true_peak_level, cnt, temp;

  pstr_loudness_info->drc_set_id = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

  temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);

  pstr_loudness_info->downmix_id = (temp >> 1) & 0x7f;
  pstr_loudness_info->sample_peak_level_present = temp & 1;

  if (pstr_loudness_info->sample_peak_level_present)
  {
    sample_peak_level = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 12);

    if (0 != sample_peak_level)
    {
      pstr_loudness_info->sample_peak_level = (20.0f - sample_peak_level * 0.03125f);
    }
    else
    {
      pstr_loudness_info->sample_peak_level_present = 0;
      pstr_loudness_info->sample_peak_level = 0;
    }
  }

  pstr_loudness_info->true_peak_level_present = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  if (pstr_loudness_info->true_peak_level_present)
  {
    true_peak_level = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 12);

    if (0 != true_peak_level)
    {
      pstr_loudness_info->true_peak_level = (20.0f - true_peak_level * 0.03125f);
    }
    else
    {
      pstr_loudness_info->true_peak_level_present = 0;
      pstr_loudness_info->true_peak_level = 0;
    }

    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);
    pstr_loudness_info->true_peak_level_measurement_system = (temp >> 2) & 0xf;
    pstr_loudness_info->true_peak_level_reliability = temp & 3;
  }

  pstr_loudness_info->measurement_count = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

  for (cnt = 0; cnt < pstr_loudness_info->measurement_count; cnt++)
  {
    err_code = impd_drc_parse_loudness_measure(&(pstr_loudness_info->loudness_measure[cnt]),
                                               pstr_it_bit_buff);
    if (err_code)
      return (err_code);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_loudness_info_set_ext
 *
 *  \brief Function to parse loudness info set extension payload
 *
 *  \param [in,out]  pstr_loudness_info_set   Pointer to loudness info set structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_loudness_info_set_ext(ia_drc_loudness_info_set_struct *pstr_loudness_info_set,
                                     ia_bit_buf_struct *pstr_it_bit_buff)
{
  WORD32 idx = 0;
  WORD32 bit_size_len, bit_size_ext, bit_size;

  pstr_loudness_info_set->str_loudness_info_set_ext.loudness_info_set_ext_type[idx] =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

  while (UNIDRCLOUDEXT_TERM !=
         pstr_loudness_info_set->str_loudness_info_set_ext.loudness_info_set_ext_type[idx])
  {
    bit_size_len = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);
    bit_size_ext = bit_size_len + 4;

    bit_size = ia_core_coder_read_bits_buf(pstr_it_bit_buff, bit_size_ext);

    if (idx >= (EXT_COUNT_MAX - 1))
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }

    pstr_loudness_info_set->str_loudness_info_set_ext.ext_bit_size[idx] = bit_size + 1;

    for (WORD32 size = 0;
         size < pstr_loudness_info_set->str_loudness_info_set_ext.ext_bit_size[idx]; size++)
    {
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    }
    idx++;
    pstr_loudness_info_set->str_loudness_info_set_ext.loudness_info_set_ext_type[idx] =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_mpegh3da_parse_loudness_info_set
 *
 *  \brief Function to parse loudness info set
 *
 *  \param [out]  pstr_loudness_info_set   Pointer to loudness info set structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
IA_ERRORCODE
impd_drc_mpegh3da_parse_loudness_info_set(ia_drc_loudness_info_set_struct *pstr_loudness_info_set,
                                          ia_bit_buf_struct *pstr_it_bit_buff,
                                          ia_mae_audio_scene_info *pstr_mae_asi)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 cnt;

  pstr_loudness_info_set->loudness_info_count = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

  for (cnt = 0; cnt < pstr_loudness_info_set->loudness_info_count; cnt++)
  {
    pstr_loudness_info_set->loudness_info[cnt].loudness_info_type =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 2);
    switch (pstr_loudness_info_set->loudness_info[cnt].loudness_info_type)
    {
    case 3:
      pstr_loudness_info_set->loudness_info[cnt].mae_group_preset_id =
          ia_core_coder_read_bits_buf(pstr_it_bit_buff, 5);
      break;
    case 2:
    case 1:
      pstr_loudness_info_set->loudness_info[cnt].mae_group_id =
          ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
      break;
    default:
      pstr_loudness_info_set->loudness_info[cnt].mae_group_id = -1;
      pstr_loudness_info_set->loudness_info[cnt].mae_group_preset_id = -1;
      break;
    }

    err_code = impd_drc_parse_loudness_info(&(pstr_loudness_info_set->loudness_info[cnt]),
                                            pstr_it_bit_buff);
    if (err_code)
      return (err_code);
  }

  pstr_loudness_info_set->loudness_info_album_count = 0;
  pstr_loudness_info_set->loudness_info_album_present =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  if (pstr_loudness_info_set->loudness_info_album_present)
  {
    pstr_loudness_info_set->loudness_info_album_count =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

    for (cnt = 0; cnt < pstr_loudness_info_set->loudness_info_album_count; cnt++)
    {
      pstr_loudness_info_set->loudness_info[cnt].loudness_info_type = 0;
      err_code = impd_drc_parse_loudness_info(
          &(pstr_loudness_info_set->str_loudness_info_album[cnt]), pstr_it_bit_buff);
      if (err_code)
        return (err_code);
    }
  }

  pstr_loudness_info_set->loudness_info_set_ext_present =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  if (pstr_loudness_info_set->loudness_info_set_ext_present)
  {
    err_code = impd_drc_parse_loudness_info_set_ext(pstr_loudness_info_set, pstr_it_bit_buff);
    if (err_code)
      return (err_code);
  }

  return err_code;
}

/**
 *  impd_drc_mpegh3da_parse_drc_channel_layout
 *
 *  \brief Function to parse drc channel layout
 *
 *  \param [out] pstr_channel_layout   Pointer to channel layout structure
 *  \param [in]  pstr_it_bit_buff      Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_mpegh3da_parse_drc_channel_layout(ia_drc_channel_layout_struct *pstr_channel_layout,
                                           ia_bit_buf_struct *pstr_it_bit_buff)
{
  pstr_channel_layout->base_channel_count = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
  pstr_channel_layout->layout_signaling_present = 0;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_gain_set_params
 *
 *  \brief Function to parse gain set params
 *
 *  \param [out]  pstr_gain_set_params   Pointer to gain set params structure
 *  \param [out]  gain_seq_index   gain sequence index
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  version   version
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_gain_set_params(ia_drc_gain_set_params_struct *pstr_gain_set_params,
                               WORD8 *gain_seq_index, ia_bit_buf_struct *pstr_it_bit_buff,
                               WORD32 version)
{
  WORD32 cnt, temp;

  temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);
  pstr_gain_set_params->full_frame = (temp >> 2) & 1;
  pstr_gain_set_params->gain_coding_profile = (temp >> 4) & 3;
  pstr_gain_set_params->gain_interp_type = (temp >> 3) & 1;
  pstr_gain_set_params->time_alignment = (temp >> 1) & 1;
  pstr_gain_set_params->time_delt_min_flag = temp & 1;

  if (pstr_gain_set_params->time_delt_min_flag)
  {
    WORD16 time_delta_min;
    time_delta_min = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 11);
    pstr_gain_set_params->time_delt_min_val = time_delta_min + 1;
  }

  if (IA_DRC_GAIN_CODING_PROFILE_CONSTANT != pstr_gain_set_params->gain_coding_profile)
  {
    pstr_gain_set_params->band_count = (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

    if (BAND_COUNT_MAX < pstr_gain_set_params->band_count)
    {
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
    }

    if (1 < pstr_gain_set_params->band_count)
    {
      pstr_gain_set_params->drc_band_type =
          (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    }

    for (cnt = 0; cnt < pstr_gain_set_params->band_count; cnt++)
    {
      if (0 != version)
      {
        WORD32 indexPresent;
        indexPresent = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
        if (!indexPresent)
        {
          *gain_seq_index = (*gain_seq_index) + 1;
        }
        else
        {
          WORD8 bs_idx;
          bs_idx = (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);
          *gain_seq_index = bs_idx;
        }
      }
      else
      {
        *gain_seq_index = (*gain_seq_index) + 1;
      }

      if (SEQUENCE_COUNT_MAX <= *gain_seq_index)
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      }

      pstr_gain_set_params->gain_params[cnt].gain_seq_idx = *gain_seq_index;
      if (0 == version)
      {
        pstr_gain_set_params->gain_params[cnt].drc_characteristic =
            (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
      }
    }
    if (!pstr_gain_set_params->drc_band_type)
    {
      for (cnt = 1; cnt < pstr_gain_set_params->band_count; cnt++)
      {
        pstr_gain_set_params->gain_params[cnt].start_subband_index =
            (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 10);
      }
    }
    else
    {
      for (cnt = 1; cnt < pstr_gain_set_params->band_count; cnt++)
      {
        pstr_gain_set_params->gain_params[cnt].crossover_freq_idx =
            (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);
      }
    }
  }
  else
  {
    pstr_gain_set_params->band_count = 1;
    *gain_seq_index = (*gain_seq_index) + 1;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_coeff
 *
 *  \brief Function to parse drc coefficients
 *
 *  \param [out]  pstr_p_loc_drc_coefficients_uni_drc   Pointer to drc coefficients structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  version   version
 *  \param [in]  pstr_ia_drc_params   Pointer to drc params structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_coeff(ia_drc_uni_drc_coeffs_struct *pstr_p_loc_drc_coefficients_uni_drc,
                     ia_bit_buf_struct *pstr_it_bit_buff, WORD32 version,
                     ia_drc_params_bs_dec_struct *pstr_ia_drc_params)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 cnt, temp;
  WORD8 drc_frame_size, gain_seq_index = -1;

  pstr_p_loc_drc_coefficients_uni_drc->version = (WORD8)version;
  if (0 == version)
  {
    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 5);
    pstr_p_loc_drc_coefficients_uni_drc->drc_frame_size_present = temp & 1;

    /*drcFrameSizePresent shall	be	set	to	0*/
    if (pstr_p_loc_drc_coefficients_uni_drc->drc_frame_size_present != 0)
    {
      return IA_MPEGD_DRC_INIT_FATAL_INVALID_DRC_PARAM_FOR_LC_PROFILE;
    }
    pstr_p_loc_drc_coefficients_uni_drc->drc_location = (temp >> 1) & 0xf;

    if (1 == pstr_p_loc_drc_coefficients_uni_drc->drc_frame_size_present)
    {
      drc_frame_size = (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 15);

      pstr_p_loc_drc_coefficients_uni_drc->drc_frame_size = drc_frame_size + 1;
    }

    pstr_p_loc_drc_coefficients_uni_drc->gain_set_count =
        (WORD8)ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

    for (cnt = 0; cnt < pstr_p_loc_drc_coefficients_uni_drc->gain_set_count; cnt++)
    {
      err_code = impd_drc_parse_gain_set_params(
          &(pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt]), &gain_seq_index,
          pstr_it_bit_buff, version);

      if (err_code)
        return (err_code);

      /*timeDeltaMinPresent shall	be	set	to	0 and
       * gainInterpolationType	shall	be	set	to	1*/
      if ((pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].time_delt_min_flag != 0) ||
          (pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].gain_interp_type != 1))
      {
        return IA_MPEGD_DRC_INIT_FATAL_INVALID_DRC_PARAM_FOR_LC_PROFILE;
      }

      if (pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].time_delt_min_flag)
      {
        if (pstr_ia_drc_params->drc_frame_size <
            pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].time_delt_min_val)
        {
          return (IA_MPEGD_DRC_INIT_NONFATAL_PARAM_ERROR);
        }
        pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].num_gain_max_values =
            (WORD8)(pstr_ia_drc_params->drc_frame_size /
                    pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].time_delt_min_val);

        impd_drc_init_tbls(
            &(pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].str_tables),
            pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[cnt].num_gain_max_values);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_target_loudness
 *
 *  \brief Function to parse target loudness
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_target_loudness(ia_drc_instructions_struct *pstr_drc_instruction,
                               ia_bit_buf_struct *pstr_it_bit_buff)
{
  pstr_drc_instruction->drc_set_target_loudness_present =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  pstr_drc_instruction->drc_set_target_loudness_value_upper = 0;
  pstr_drc_instruction->drc_set_target_loudness_value_lower = -63;

  if (1 == pstr_drc_instruction->drc_set_target_loudness_present)
  {
    WORD32 drc_set_target_loudness_value_upper, drc_set_target_loudness_value_lower;
    drc_set_target_loudness_value_upper = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

    pstr_drc_instruction->drc_set_target_loudness_value_upper =
        drc_set_target_loudness_value_upper - 63;
    pstr_drc_instruction->drc_set_target_loudness_value_lower_present =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    if (1 == pstr_drc_instruction->drc_set_target_loudness_value_lower_present)
    {
      drc_set_target_loudness_value_lower = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);
      pstr_drc_instruction->drc_set_target_loudness_value_lower =
          drc_set_target_loudness_value_lower - 63;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_update_uni_drc_coeff
 *
 *  \brief Function to update uni drc coefficients
 *
 *  \param [out]  ppstr_p_loc_drc_coefficients_uni_drc   Pointer to store uni drc coefficients
 * structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  location   drc location
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_update_uni_drc_coeff(ia_drc_uni_drc_coeffs_struct **ppstr_p_loc_drc_coefficients_uni_drc,
                              ia_drc_config *pstr_drc_config, WORD32 location)
{
  WORD32 cnt, c0 = -1;

  for (cnt = 0; cnt < pstr_drc_config->drc_coefficients_drc_count; cnt++)
  {
    if ((location == pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt].drc_location) &&
        (0 == pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt].version))
    {
      c0 = cnt;
    }
  }

  if (c0 < 0)
  {
    return IA_MPEGD_DRC_INIT_FATAL_UNEXPECTED_ERROR;
  }
  else
  {
    *ppstr_p_loc_drc_coefficients_uni_drc =
        &(pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[c0]);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_downmix_id
 *
 *  \brief Function to parse downmix id
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  version   version
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_parse_downmix_id(ia_drc_instructions_struct *pstr_drc_instruction,
                                              ia_bit_buf_struct *pstr_it_bit_buff, WORD32 version)
{
  WORD32 additional_dmix_id_flag, additional_dmix_id_cnt;
  {
    pstr_drc_instruction->downmix_id[0] = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
    if (0 == version)
    {
      if (0 != pstr_drc_instruction->downmix_id[0])
      {
        pstr_drc_instruction->drc_apply_to_dwnmix = 1;
      }
      else
      {
        pstr_drc_instruction->drc_apply_to_dwnmix = 0;
      }
    }
    additional_dmix_id_flag = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

    if (!additional_dmix_id_flag)
    {
      pstr_drc_instruction->dwnmix_id_count = 1;
    }
    else
    {
      additional_dmix_id_cnt = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 3);
      for (WORD32 cnt = 0; cnt < additional_dmix_id_cnt; cnt++)
      {
        pstr_drc_instruction->downmix_id[cnt + 1] =
            ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
      }
      pstr_drc_instruction->dwnmix_id_count = 1 + additional_dmix_id_cnt;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_update_ch_count
 *
 *  \brief Function to update channel count
 *
 *  \param [out]  ch_cnt   channel count
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_update_ch_count(WORD32 *ch_cnt,
                                             ia_drc_instructions_struct *pstr_drc_instruction,
                                             ia_drc_config *pstr_drc_config)
{
  WORD32 cnt;
  if ((ID_FOR_BASE_LAYOUT != pstr_drc_instruction->downmix_id[0]) &&
      (ID_FOR_ANY_DOWNMIX != pstr_drc_instruction->downmix_id[0]) &&
      (1 == pstr_drc_instruction->dwnmix_id_count))
  {
    for (cnt = 0; cnt < pstr_drc_config->ia_mpegh3da_dwnmix_instructions_count; cnt++)
    {
      if (pstr_drc_instruction->downmix_id[0] ==
          pstr_drc_config->mpegh3da_dwnmix_instructions[cnt].downmix_id)
        break;
    }
    if (cnt == pstr_drc_config->ia_mpegh3da_dwnmix_instructions_count)
    {
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
    }
    *ch_cnt = pstr_drc_config->mpegh3da_dwnmix_instructions[cnt].target_channel_count;
  }
  else if (((ID_FOR_ANY_DOWNMIX == pstr_drc_instruction->downmix_id[0]) ||
            (1 < pstr_drc_instruction->dwnmix_id_count)))
  {
    *ch_cnt = 1;
  }

  if (MAX_CHANNEL_COUNT < *ch_cnt)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_decode_gain_modifiers
 *
 *  \brief Function to decode gain modifiers
 *
 *  \param [in,out]  pstr_gain_modifiers   Pointer to gain modifiers structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  band_cnt   band count
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_decode_gain_modifiers(ia_drc_gain_modifiers_struct *pstr_gain_modifiers,
                               ia_bit_buf_struct *pstr_it_bit_buff, WORD32 band_cnt)
{
  WORD32 sign, tmp;
  WORD32 band, gain_scaling_flag, gain_off_flag;
  FLOAT32 attn_scale_factor = 1.0f, amplfn_scale_factor = 1.0f, gain_off = 0.0f;

  gain_scaling_flag = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  if (gain_scaling_flag)
  {
    tmp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);

    attn_scale_factor = (((tmp >> 4) & 0xf) * 0.125f);
    amplfn_scale_factor = ((tmp & 0xf) * 0.125f);
  }

  gain_off_flag = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
  if (gain_off_flag)
  {
    tmp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

    sign = ((tmp >> 5) & 1);
    gain_off = (1 + (tmp & 0x1f)) * 0.25f;

    if (sign)
    {
      gain_off = -gain_off;
    }
  }
  for (band = band_cnt - 1; band >= 0; band--)
  {
    pstr_gain_modifiers->gain_scaling_flag[band] = gain_scaling_flag;
    pstr_gain_modifiers->attn_scale_factor[band] = attn_scale_factor;
    pstr_gain_modifiers->amplfn_scale_factor[band] = amplfn_scale_factor;
    pstr_gain_modifiers->gain_offset_flag[band] = gain_off_flag;
    pstr_gain_modifiers->gain_offset[band] = gain_off;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_instrns_effect_not_self_other
 *
 *  \brief Function to parse drc instructions for effect type other than self and other
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [out]  ch_cnt   channel count
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  unique_idx   unique index
 *  \param [in]  pstr_p_loc_drc_coefficients_uni_drc   Pointer to uni drc coefficients structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_parse_instrns_effect_not_self_other(
    ia_drc_instructions_struct *pstr_drc_instruction, WORD32 *ch_cnt,
    ia_bit_buf_struct *pstr_it_bit_buff, ia_drc_config *pstr_drc_config, WORD32 *unique_idx,
    ia_drc_uni_drc_coeffs_struct *pstr_p_loc_drc_coefficients_uni_drc)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ch, grp;
  err_code = impd_drc_update_ch_count(ch_cnt, pstr_drc_instruction, pstr_drc_config);
  if (err_code)
    return (err_code);

  ch = 0;
  while (ch < *ch_cnt)
  {
    WORD32 bs_gain_set_idx;
    WORD32 repeat_gain_set_idx, repeat_gain_set_idx_cnt, temp;

    temp = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
    bs_gain_set_idx = (temp >> 1) & 0x7f;
    repeat_gain_set_idx = temp & 1;

    if (GAIN_SET_COUNT_MAX < bs_gain_set_idx)
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }

    pstr_drc_instruction->gain_set_index[ch] = bs_gain_set_idx - 1;
    ch++;

    if (1 == repeat_gain_set_idx)
    {
      repeat_gain_set_idx_cnt = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 5);
      repeat_gain_set_idx_cnt += 1;

      if (MAX_CHANNEL_COUNT < (ch + repeat_gain_set_idx_cnt))
      {
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      }

      for (WORD32 idx = 0; idx < repeat_gain_set_idx_cnt; idx++)
      {
        pstr_drc_instruction->gain_set_index[ch + idx] = bs_gain_set_idx - 1;
      }
      ch += repeat_gain_set_idx_cnt;
    }
  }
  if (ch > *ch_cnt)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }

  grp = 0;
  if ((ID_FOR_ANY_DOWNMIX == pstr_drc_instruction->downmix_id[0]) ||
      (1 < pstr_drc_instruction->dwnmix_id_count))
  {
    WORD32 index = pstr_drc_instruction->gain_set_index[0];
    if (0 <= index)
    {
      unique_idx[0] = index;
      grp = 1;
    }
  }
  else
  {
    for (ch = 0; ch < *ch_cnt; ch++)
    {
      WORD32 index = pstr_drc_instruction->gain_set_index[ch];
      WORD32 match = 0;
      if (0 > index)
      {
        pstr_drc_instruction->channel_group_of_ch[ch] = -1;
      }
      else
      {
        for (WORD32 n = 0; n < grp; n++)
        {
          if (index == unique_idx[n])
          {
            match = 1;
            pstr_drc_instruction->channel_group_of_ch[ch] = n;
            break;
          }
        }
        if (0 == match)
        {
          unique_idx[grp] = index;
          pstr_drc_instruction->channel_group_of_ch[ch] = grp;
          grp++;
        }
      }
    }
  }

  pstr_drc_instruction->num_drc_ch_groups = grp;

  if (ia_min_flt(CHANNEL_GROUP_COUNT_MAX, MAX_CHANNEL_COUNT) <
      pstr_drc_instruction->num_drc_ch_groups)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNSUPPORTED_CH_GROUPS;
  }
  for (grp = 0; grp < pstr_drc_instruction->num_drc_ch_groups; grp++)
  {
    WORD32 band_cnt;

    if (0 > unique_idx[grp])
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }
    pstr_drc_instruction->gain_set_index_for_channel_group[grp] = unique_idx[grp];

    if (NULL != pstr_p_loc_drc_coefficients_uni_drc &&
        unique_idx[grp] < pstr_p_loc_drc_coefficients_uni_drc->gain_set_count)
    {
      band_cnt = pstr_p_loc_drc_coefficients_uni_drc->gain_set_params[unique_idx[grp]].band_count;
    }
    else
    {
      band_cnt = 1;
    }

    err_code = impd_drc_decode_gain_modifiers(
        &(pstr_drc_instruction->str_gain_modifiers_of_ch_group[grp]), pstr_it_bit_buff, band_cnt);
    if (err_code)
      return (err_code);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_decode_ducking_scaling
 *
 *  \brief Function to decode ducking scale factor
 *
 *  \param [out]  ducking_scaling_flag   Flag to update if ducking scalefactor present
 *  \param [out]  ducking_scaling   ducking scalefactor
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE impd_drc_decode_ducking_scaling(WORD32 *ducking_scaling_flag,
                                                    FLOAT32 *ducking_scaling,
                                                    ia_bit_buf_struct *pstr_it_bit_buff)
{
  WORD32 sigma, mu;

  *ducking_scaling_flag = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  if (*ducking_scaling_flag != 0)
  {
    WORD32 scale_factor = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

    *ducking_scaling_flag = 1;

    sigma = scale_factor >> 3;
    mu = scale_factor & 0x7;
    if (sigma != 0)
    {
      *ducking_scaling = 1.0f - 0.125f * (1.0f + mu);
    }
    else
    {
      *ducking_scaling = 1.0f + 0.125f * (1.0f + mu);
    }
  }
  else
  {
    *ducking_scaling = 1.0f;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_update_ch_group_info_duck_self
 *
 *  \brief Function to parse channel group info for effect type duck self
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  unique_scaling   unique scaling
 *  \param [in]  unique_idx   unique index
 *  \param [in]  ch_cnt   channel count
 *
 *
 *
 */
static VOID
impd_drc_update_ch_group_info_duck_self(ia_drc_instructions_struct *pstr_drc_instruction,
                                        FLOAT32 *unique_scaling, WORD32 *unique_idx,
                                        WORD32 ch_cnt)
{
  WORD32 match, idx, grp = 0;
  FLOAT32 factor;

  for (WORD32 ch = 0; ch < ch_cnt; ch++)
  {
    match = 0;
    idx = pstr_drc_instruction->gain_set_index[ch];
    factor = pstr_drc_instruction->str_ducking_modifiers_for_channel[ch].ducking_scaling;

    if (0 > idx)
    {
      pstr_drc_instruction->channel_group_of_ch[ch] = -1;
    }
    else
    {
      for (WORD32 n = 0; n < grp; n++)
      {
        if ((factor == unique_scaling[n]) && (idx == unique_idx[n]))
        {
          match = 1;
          pstr_drc_instruction->channel_group_of_ch[ch] = n;
          break;
        }
      }
      if (0 == match)
      {
        unique_idx[grp] = idx;
        unique_scaling[grp] = factor;
        pstr_drc_instruction->channel_group_of_ch[ch] = grp;
        grp++;
      }
    }
  }
  pstr_drc_instruction->num_drc_ch_groups = grp;
}

/**
 *  impd_drc_update_ch_group_info_duck_other
 *
 *  \brief Function to parse channel group info for effect type duck other
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  unique_scaling   unique scaling
 *  \param [in]  unique_idx   unique index
 *  \param [in]  ch_cnt   channel count
 *  \param [in]  ducking_seq   ducking_sequence
 *
 *  \return IA_ERRORCODE error code
 *
 */
static IA_ERRORCODE
impd_drc_update_ch_group_info_duck_other(ia_drc_instructions_struct *pstr_drc_instruction,
                                         FLOAT32 *unique_scaling, WORD32 *unique_idx,
                                         WORD32 ch_cnt, WORD32 *ducking_seq)
{
  WORD32 match, idx, grp = 0;
  FLOAT32 factor;
  for (WORD32 ch = 0; ch < ch_cnt; ch++)
  {
    match = 0;
    idx = pstr_drc_instruction->gain_set_index[ch];
    factor = pstr_drc_instruction->str_ducking_modifiers_for_channel[ch].ducking_scaling;

    if (idx >= 0)
    {
      if ((idx != *ducking_seq) && (0 < *ducking_seq))
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNSUPPORTED_DUCKING_SEQUENCE;
      }
      *ducking_seq = idx;
      pstr_drc_instruction->channel_group_of_ch[ch] = -1;
    }
    else
    {
      for (WORD32 n = 0; n < grp; n++)
      {
        if (factor == unique_scaling[n])
        {
          match = 1;
          pstr_drc_instruction->channel_group_of_ch[ch] = n;
          break;
        }
      }
      if (0 == match)
      {
        unique_idx[grp] = idx;
        unique_scaling[grp] = factor;
        pstr_drc_instruction->channel_group_of_ch[ch] = grp;
        grp++;
      }
    }
  }
  pstr_drc_instruction->num_drc_ch_groups = grp;
  if (-1 == *ducking_seq)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNSUPPORTED_DUCKING_SEQUENCE;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_instrns_effect_self_other
 *
 *  \brief Function to parse drc instructions for effect type self and other
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [out]  ch_cnt   channel count
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  unique_scaling   unique scaling
 *  \param [in]  unique_idx   unique index
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_instrns_effect_self_other(ia_drc_instructions_struct *pstr_drc_instruction,
                                         WORD32 *ch_cnt, ia_bit_buf_struct *pstr_it_bit_buff,
                                         FLOAT32 *unique_scaling, WORD32 *unique_idx)
{
  WORD32 ch = 0, grp = 0;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ducking_seq;
  WORD32 repeat_params, repeat_params_cnt;

  while (ch < *ch_cnt)
  {
    WORD32 bs_gain_set_idx;
    bs_gain_set_idx = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

    if (GAIN_SET_COUNT_MAX < bs_gain_set_idx)
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }

    pstr_drc_instruction->gain_set_index[ch] = bs_gain_set_idx - 1;
    err_code = impd_drc_decode_ducking_scaling(
        &(pstr_drc_instruction->str_ducking_modifiers_for_channel[ch].ducking_scaling_flag),
        &(pstr_drc_instruction->str_ducking_modifiers_for_channel[ch].ducking_scaling),
        pstr_it_bit_buff);
    if (err_code)
    {
      return err_code;
    }
    ch++;

    repeat_params = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

    if (1 == repeat_params)
    {
      repeat_params_cnt = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 5);
      repeat_params_cnt += 1;
      if (MAX_CHANNEL_COUNT < (ch + repeat_params_cnt))
      {
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      }

      for (WORD32 idx = 0; idx < repeat_params_cnt; idx++)
      {
        pstr_drc_instruction->gain_set_index[ch + idx] =
            pstr_drc_instruction->gain_set_index[ch + idx - 1];
        pstr_drc_instruction->str_ducking_modifiers_for_channel[ch + idx] =
            pstr_drc_instruction->str_ducking_modifiers_for_channel[ch + idx - 1];
      }
      ch += repeat_params_cnt;
    }
  }
  if (ch > *ch_cnt)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }

  ducking_seq = -1;
  if (IA_DRC_EFFECT_BIT_DUCK_SELF & pstr_drc_instruction->drc_set_effect)
  {
    impd_drc_update_ch_group_info_duck_self(pstr_drc_instruction, unique_scaling, unique_idx,
                                            *ch_cnt);
  }
  else if (IA_DRC_EFFECT_BIT_DUCK_OTHER & pstr_drc_instruction->drc_set_effect)
  {
    err_code = impd_drc_update_ch_group_info_duck_other(pstr_drc_instruction, unique_scaling,
                                                        unique_idx, *ch_cnt, &ducking_seq);
    if (err_code)
      return err_code;
  }

  if (MAX_CHANNEL_COUNT < pstr_drc_instruction->num_drc_ch_groups)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNSUPPORTED_CH_GROUPS;
  }

  for (grp = 0; grp < pstr_drc_instruction->num_drc_ch_groups; grp++)
  {
    WORD32 set = (IA_DRC_EFFECT_BIT_DUCK_OTHER & pstr_drc_instruction->drc_set_effect)
                     ? ducking_seq
                     : unique_idx[grp];
    if (0 > set)
    {
      return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    }
    pstr_drc_instruction->gain_set_index_for_channel_group[grp] = set;

    pstr_drc_instruction->str_ducking_modifiers_for_channel_group[grp].ducking_scaling =
        (FLOAT32)unique_scaling[grp];

    if (1.0f == unique_scaling[grp])
    {
      pstr_drc_instruction->str_ducking_modifiers_for_channel_group[grp].ducking_scaling_flag = 0;
    }
    else
    {
      pstr_drc_instruction->str_ducking_modifiers_for_channel_group[grp].ducking_scaling_flag = 1;
    }
    pstr_drc_instruction->band_count_of_ch_group[grp] = 1;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_drc_instructions
 *
 *  \brief Function to parse drc instructions
 *
 *  \param [out]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  version   version
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
static IA_ERRORCODE
impd_drc_parse_drc_instructions(ia_drc_instructions_struct *pstr_drc_instruction,
                                ia_bit_buf_struct *pstr_it_bit_buff, WORD32 version,
                                ia_drc_config *pstr_drc_config)
{
  ia_drc_uni_drc_coeffs_struct *pstr_p_loc_drc_coefficients_uni_drc = NULL;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ch, peak_lim_target;
  WORD32 ch_cnt;
  WORD32 unique_idx[MAX_CHANNEL_COUNT];
  FLOAT32 unique_scaling[MAX_CHANNEL_COUNT];

  pstr_drc_instruction->drc_set_id = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

  if (DRC_INSTRUCTIONS_COUNT_MAX <= pstr_drc_instruction->drc_set_id)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
  }

  pstr_drc_instruction->drc_location = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 4);

  err_code = impd_drc_parse_downmix_id(pstr_drc_instruction, pstr_it_bit_buff, version);
  if (err_code)
    return err_code;

  pstr_drc_instruction->drc_set_effect = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 16);

  if (0 == ((IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF) &
            pstr_drc_instruction->drc_set_effect))
  {
    pstr_drc_instruction->limiter_peak_target_present =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    if (pstr_drc_instruction->limiter_peak_target_present)
    {
      peak_lim_target = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 8);
      pstr_drc_instruction->limiter_peak_target = (-peak_lim_target * 0.125f);
    }
  }

  err_code = impd_drc_parse_target_loudness(pstr_drc_instruction, pstr_it_bit_buff);
  if (err_code)
    return err_code;

  pstr_drc_instruction->depends_on_drc_set_present =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  /*dependsOnDrcSetPresent	shall	be	set	to	0	for
   * drcInstructionsUniDrc()
   * with	downmixId	==	0*/
  if ((pstr_drc_instruction->downmix_id[0] == 0) &&
      (pstr_drc_instruction->depends_on_drc_set_present != 0))
  {
    return IA_MPEGD_DRC_INIT_FATAL_INVALID_DRC_PARAM_FOR_LC_PROFILE;
  }

  pstr_drc_instruction->no_independent_use = 0;
  if (!pstr_drc_instruction->depends_on_drc_set_present)
  {
    pstr_drc_instruction->no_independent_use = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
  }
  else
  {
    pstr_drc_instruction->depends_on_drc_set = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);
  }

  err_code = impd_drc_update_uni_drc_coeff(&pstr_p_loc_drc_coefficients_uni_drc, pstr_drc_config,
                                           pstr_drc_instruction->drc_location);
  if (err_code)
    return (err_code);

  ch_cnt = pstr_drc_config->channel_layout.base_channel_count;

  if (MAX_CHANNEL_COUNT < ch_cnt)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }
  for (ch = 0; ch < MAX_CHANNEL_COUNT; ch++)
  {
    unique_idx[ch] = -10;
    unique_scaling[ch] = -10.0f;
  }

  if (!((IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF) &
        pstr_drc_instruction->drc_set_effect))
  {
    err_code = impd_drc_parse_instrns_effect_not_self_other(
        pstr_drc_instruction, &ch_cnt, pstr_it_bit_buff, pstr_drc_config, unique_idx,
        pstr_p_loc_drc_coefficients_uni_drc);
    if (err_code)
      return (err_code);
  }
  else
  {
    err_code = impd_drc_parse_instrns_effect_self_other(
        pstr_drc_instruction, &ch_cnt, pstr_it_bit_buff, unique_scaling, unique_idx);
    if (err_code)
      return (err_code);
  }

  /*nDrcChannelGroups	shall	be	restricted	to	1	for
   * drcInstructionsUniDrc()
   * with	downmixId	!=	0*/
  if ((pstr_drc_instruction->downmix_id[0] != 0) &&
      (pstr_drc_instruction->num_drc_ch_groups != 1))
  {
    return IA_MPEGD_DRC_INIT_FATAL_INVALID_DRC_PARAM_FOR_LC_PROFILE;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_parse_config
 *
 *  \brief Function to parse drc config
 *
 *  \param [out]  pstr_drc_config   Pointer to drc config structure
 *  \param [out]  pstr_loudness_info_set   Pointer to loudness info set structure
 *  \param [in]  pstr_it_bit_buff   Pointer to bit buffer structure
 *  \param [in]  pstr_ia_drc_params   Pointer to drc params bitstreamdec structure
 *
 *  \return IA_ERRORCODE errorcode
 *
 */
IA_ERRORCODE
impd_drc_parse_config(ia_drc_config *pstr_drc_config,
                      ia_drc_loudness_info_set_struct *pstr_loudness_info_set,
                      ia_bit_buf_struct *pstr_it_bit_buff,
                      ia_drc_params_bs_dec_struct *pstr_ia_drc_params,
                      ia_mae_audio_scene_info *pstr_mae_asi)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 cnt, version = 0;

  /* sampling frequency from mpegh3daconfig() */
  pstr_drc_config->drc_coefficients_drc_count = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 3);

  if (DRC_COEFF_COUNT_MAX < pstr_drc_config->drc_coefficients_drc_count)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_COEFF_MAX_EXCEEDED;
  }

  pstr_drc_config->drc_instructions_uni_drc_count =
      ia_core_coder_read_bits_buf(pstr_it_bit_buff, 6);

  if (DRC_INSTRUCTIONS_COUNT_MAX < pstr_drc_config->drc_instructions_uni_drc_count)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_DRC_INSTR_MAX_EXCEEDED;
  }

  err_code = impd_drc_mpegh3da_parse_drc_channel_layout(&pstr_drc_config->channel_layout,
                                                        pstr_it_bit_buff);

  for (cnt = 0; cnt < pstr_drc_config->drc_coefficients_drc_count; cnt++)
  {
    err_code = impd_drc_parse_coeff(&(pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[cnt]),
                                    pstr_it_bit_buff, version, pstr_ia_drc_params);
    if (err_code)
      return (err_code);
  }

  for (cnt = 0; cnt < pstr_drc_config->drc_instructions_uni_drc_count; cnt++)
  {
    pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type =
        ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
    if (0 == pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type)
    {
      pstr_drc_config->str_drc_instruction_str[cnt].mae_group_id = -1;
      pstr_drc_config->str_drc_instruction_str[cnt].mae_group_preset_id = -1;
    }
    else
    {
      pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type =
          ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);
      pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type =
          pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type | (1 << 1);
      if (3 == pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type)
      {
        pstr_drc_config->str_drc_instruction_str[cnt].mae_group_preset_id =
            ia_core_coder_read_bits_buf(pstr_it_bit_buff, 5);
      }
      else if (2 == pstr_drc_config->str_drc_instruction_str[cnt].drc_instructions_type)
      {
        pstr_drc_config->str_drc_instruction_str[cnt].mae_group_id =
            ia_core_coder_read_bits_buf(pstr_it_bit_buff, 7);
      }
    }

    err_code = impd_drc_parse_drc_instructions(&(pstr_drc_config->str_drc_instruction_str[cnt]),
                                               pstr_it_bit_buff, version, pstr_drc_config);
    if (err_code)
      return (err_code);
  }

  pstr_drc_config->drc_config_ext_present = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  if (1 == pstr_drc_config->drc_config_ext_present)
  {
    err_code =
        impd_drc_parse_config_ext(&(pstr_drc_config->str_drc_config_ext), pstr_it_bit_buff);
    if (err_code)
      return (err_code);
  }

  pstr_drc_config->loudness_infoset_present = ia_core_coder_read_bits_buf(pstr_it_bit_buff, 1);

  /*loudnessInfoSetPresent	within	mpegh3daUniDrcConfig()	shall	be	set	to 0*/
  if (pstr_drc_config->loudness_infoset_present != 0)
  {
    return IA_MPEGD_DRC_INIT_FATAL_INVALID_DRC_PARAM_FOR_LC_PROFILE;
  }

  if (1 == pstr_drc_config->loudness_infoset_present)
  {
    err_code = impd_drc_mpegh3da_parse_loudness_info_set(pstr_loudness_info_set, pstr_it_bit_buff,
                                                         pstr_mae_asi);
    if (err_code)
      return (err_code);
  }

  for (cnt = 0; cnt < pstr_drc_config->drc_instructions_uni_drc_count; cnt++)
  {
    err_code = impd_drc_gen_instructions_derived_data(
        &(pstr_drc_config->str_drc_instruction_str[cnt]), pstr_drc_config, pstr_ia_drc_params);
    if (err_code)
      return (err_code);
  }

  if (DRC_INSTRUCTIONS_COUNT_MAX <= (pstr_drc_config->drc_instructions_uni_drc_count +
                                     pstr_drc_config->drc_dwnmix_instructions_count))
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_DRC_INSTR_MAX_EXCEEDED);
  }
  impd_drc_gen_instructions_for_drc_off(pstr_drc_config);
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCConfigPayloadParse */
