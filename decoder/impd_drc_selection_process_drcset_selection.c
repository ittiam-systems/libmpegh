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
#include "impeghd_error_codes.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_interface.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_rom.h"
#include "impd_drc_selection_process.h"
#include "impd_drc_sel_proc_drc_set_sel.h"
#include "impd_drc_loudness_control.h"
#include "impeghd_intrinsics_flt.h"
#include <ia_core_coder_basic_ops32.h>
#include "ia_core_coder_constants.h"

/**
 * @defgroup DRCControlParams DRCControlParams
 * @ingroup  DRCControlParams
 * @brief DRC Control parameter setting
 *
 * @{
 */

static const WORD32 effect_types_request_table[] = {
    IA_DRC_EFFECT_BIT_NIGHT,    IA_DRC_EFFECT_BIT_NOISY,   IA_DRC_EFFECT_BIT_LIMITED,
    IA_DRC_EFFECT_BIT_LOWLEVEL, IA_DRC_EFFECT_BIT_DIALOG,  IA_DRC_EFFECT_BIT_GENERAL_COMPR,
    IA_DRC_EFFECT_BIT_EXPAND,   IA_DRC_EFFECT_BIT_ARTISTIC};

/**
 *
 *	impd_drc_validate_requested_drc_feature
 *  \brief Validation of requested DRC feature
 *
 *  \param [in/out] pstr_drc_sel_proc_params_struct
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE impd_drc_validate_requested_drc_feature(
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct)
{
  WORD32 i, j;

  for (i = 0; i < pstr_drc_sel_proc_params_struct->num_drc_feature_requests; i++)
  {
    switch (pstr_drc_sel_proc_params_struct->drc_feature_req_type[i])
    {
    case MATCH_DYNAMIC_RANGE:
    case MATCH_DRC_CHARACTERISTIC:
      break;
    case MATCH_EFFECT_TYPE:
      for (j = 0; j < pstr_drc_sel_proc_params_struct->desired_num_drc_effects_of_requested[i];
           j++)
      {
        if (pstr_drc_sel_proc_params_struct->requested_drc_effect_type[i][j] ==
            EFFECT_TYPE_REQUESTED_NONE)
        {
          if (pstr_drc_sel_proc_params_struct->desired_num_drc_effects_of_requested[i] > 1)
          {
            return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
          }
        }
      }
      break;
    default:
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      break;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_get_dependent_drc_instructions
 *  \brief Initialize get dependent DRC indtruction
 *
 *  \param [in] drc_config
 *  \param [in] str_drc_instruction_str
 *  \param [out] drc_instructions_dependent
 *
 *  \return error IA_ERRORCODE if any
 *
 */
static IA_ERRORCODE impd_drc_get_dependent_drc_instructions(
    const ia_drc_config *drc_config, const ia_drc_instructions_struct *pstr_drc_instruction_str,
    ia_drc_instructions_struct **drc_instructions_dependent)
{
  WORD32 j;
  ia_drc_instructions_struct *dependent_drc = NULL;
  for (j = 0; j < drc_config->drc_instructions_uni_drc_count; j++)
  {
    dependent_drc = (ia_drc_instructions_struct *)&(drc_config->str_drc_instruction_str[j]);
    if (dependent_drc->drc_set_id == pstr_drc_instruction_str->depends_on_drc_set)
    {
      break;
    }
  }
  if (j >= drc_config->drc_instructions_uni_drc_count)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }
  if (dependent_drc->depends_on_drc_set_present == 1)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }
  *drc_instructions_dependent = dependent_drc;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_select_drcs_without_compr_effects
 *  \brief Select drc without compr effects
 *
 *  \param [in/out] pstr_drc_config
 *  \param [out] match_found_flag
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info

 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_drc_select_drcs_without_compr_effects(
    ia_drc_config *pstr_drc_config, WORD32 *match_found_flag, WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  WORD32 i, k, n;
  WORD32 effect_types_request_table_size;
  WORD32 match;
  ia_drc_selection_candidate_info_struct
      selection_candidate_info_step_2[SELECTION_CANDIDATE_COUNT_MAX];
  ia_drc_instructions_struct *str_drc_instruction_str;
  WORD32 selection_candidate_step_2_count = 0;

  effect_types_request_table_size = sizeof(effect_types_request_table) / sizeof(WORD32);
  k = 0;
  for (i = 0; i < *selection_candidate_count; i++)
  {
    str_drc_instruction_str =
        &(pstr_drc_config->str_drc_instruction_str[selection_candidate_info[i].drc_instrns_idx]);

    match = 1;
    for (n = 0; n < effect_types_request_table_size; n++)
    {
      if ((str_drc_instruction_str->drc_set_effect & effect_types_request_table[n]) != 0x0)
      {
        match = 0;
      }
    }
    if (match == 1)
    {
      if (k >= SELECTION_CANDIDATE_COUNT_MAX)
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      }

      memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
             sizeof(ia_drc_selection_candidate_info_struct));
      k++;
    }
  }
  selection_candidate_step_2_count = k;

  if (selection_candidate_step_2_count <= 0)
  {
    *match_found_flag = 0;
  }
  else
  {
    *match_found_flag = 1;
    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      memcpy(&selection_candidate_info[i], &selection_candidate_info_step_2[i],
             sizeof(ia_drc_selection_candidate_info_struct));
      *selection_candidate_count = selection_candidate_step_2_count;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_match_effect_type_attempt
 *  \brief Match effect type
 *
 *  \param [in/out] pstr_drc_config
 *  \param [in] requested_effect_type
 *  \param [in] state_requested
 *  \param [out] match_found_flag
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
static IA_ERRORCODE impd_drc_match_effect_type_attempt(
    ia_drc_config *pstr_drc_config, WORD32 requested_effect_type, WORD32 state_requested,
    WORD32 *match_found_flag, WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  WORD32 i, k;
  WORD32 effect_bit_idx;
  WORD32 selection_candidate_step_2_count = 0;
  IA_ERRORCODE err;
  ia_drc_selection_candidate_info_struct
      selection_candidate_info_step_2[SELECTION_CANDIDATE_COUNT_MAX];
  ia_drc_instructions_struct *str_drc_instruction_str;
  ia_drc_instructions_struct *drc_instructions_dependent;
  if ((requested_effect_type > EFFECT_TYPE_REQUESTED_NONE) &&
      (requested_effect_type < EFFECT_TYPE_REQUESTED_COUNT))
  {
    effect_bit_idx = 1 << (requested_effect_type - 1);
  }
  else if (requested_effect_type == EFFECT_TYPE_REQUESTED_NONE)
  {
    effect_bit_idx = -1;
  }
  else
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }

  if (effect_bit_idx != IA_DRC_EFFECT_BIT_NONE)
  {

    k = 0;
    for (i = 0; i < *selection_candidate_count; i++)
    {
      str_drc_instruction_str = &(
          pstr_drc_config->str_drc_instruction_str[selection_candidate_info[i].drc_instrns_idx]);
      if (str_drc_instruction_str->depends_on_drc_set_present != 1)
      {
        if (state_requested != 1)
        {
          if ((str_drc_instruction_str->drc_set_effect & effect_bit_idx) == 0x0)
          {
            if (k >= SELECTION_CANDIDATE_COUNT_MAX)
              return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
            memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k++;
          }
        }
        else
        {
          if ((str_drc_instruction_str->drc_set_effect & effect_bit_idx) != 0x0)
          {
            if (k >= SELECTION_CANDIDATE_COUNT_MAX)
              return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
            memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k++;
          }
        }
      }
      else
      {
        err = impd_drc_get_dependent_drc_instructions(pstr_drc_config, str_drc_instruction_str,
                                                      &drc_instructions_dependent);
        if (err)
          return (err);

        if (state_requested != 1)
        {
          if (((str_drc_instruction_str->drc_set_effect & effect_bit_idx) == 0x0) &&
              ((drc_instructions_dependent->drc_set_effect & effect_bit_idx) == 0x0))
          {
            if (k >= SELECTION_CANDIDATE_COUNT_MAX)
              return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
            memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k++;
          }
        }
        else
        {
          if (((str_drc_instruction_str->drc_set_effect & effect_bit_idx) != 0x0) ||
              ((drc_instructions_dependent->drc_set_effect & effect_bit_idx) != 0x0))
          {
            if (k >= SELECTION_CANDIDATE_COUNT_MAX)
              return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
            memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k++;
          }
        }
      }
    }
    selection_candidate_step_2_count = k;

    if (selection_candidate_step_2_count <= 0)
    {
      *match_found_flag = 0;
    }
    else
    {

      *match_found_flag = 1;
      for (i = 0; i < selection_candidate_step_2_count; i++)
      {
        *selection_candidate_count = selection_candidate_step_2_count;
        memcpy(&selection_candidate_info[i], &selection_candidate_info_step_2[i],
               sizeof(ia_drc_selection_candidate_info_struct));
      }
    }
  }
  else
  {
    err = impd_drc_select_drcs_without_compr_effects(
        pstr_drc_config, match_found_flag, selection_candidate_count, selection_candidate_info);
    if (err)
      return (err);
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_match_effect_types
 *  \brief Matchin effect type
 *
 *  \param [in/out] pstr_drc_config
 *  \param [in] effect_type_requested_total_count
 *  \param [in] effect_type_requested_desired_count
 *  \param [in] requested_effect_type
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE
impd_drc_match_effect_types(ia_drc_config *pstr_drc_config,
                            WORD32 effect_type_requested_total_count,
                            WORD32 effect_type_requested_desired_count,
                            WORD8 *requested_effect_type, WORD32 *selection_candidate_count,
                            ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  IA_ERRORCODE err;
  WORD32 state_requested;
  WORD32 desired_effect_type_found = 0;
  WORD32 match_found_flag = 0;
  WORD32 k = 0;
  while (k < effect_type_requested_desired_count)
  {
    state_requested = 1;
    err = impd_drc_match_effect_type_attempt(pstr_drc_config, requested_effect_type[k],
                                             state_requested, &match_found_flag,
                                             selection_candidate_count, selection_candidate_info);
    if (err)
    {
      return (err);
    }

    if (match_found_flag)
    {
      desired_effect_type_found = 1;
    }

    k++;
  }
  if (desired_effect_type_found == 0)
  {
    while ((k < effect_type_requested_total_count) && (match_found_flag == 0))
    {
      state_requested = 1;
      err = impd_drc_match_effect_type_attempt(
          pstr_drc_config, requested_effect_type[k], state_requested, &match_found_flag,
          selection_candidate_count, selection_candidate_info);
      if (err)
      {
        return (err);
      }
      k++;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_match_dynamic_range
 *  \brief Match dymanic range
 *
 *  \param [in/out] pstr_drc_config
 *  \param [in] effect_type_requested_total_count
 *  \param [in] pstr_drc_sel_proc_params_struct
 *  \param [in] num_drc_requests
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE
impd_drc_match_dynamic_range(ia_drc_config *pstr_drc_config,
                             ia_drc_loudness_info_set_struct *pstr_loudness_info,
                             ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
                             WORD32 num_drc_requests, WORD32 *selection_candidate_count,
                             ia_drc_selection_candidate_info_struct *selection_candidate_info)
{

  IA_ERRORCODE err;
  WORD32 i, k;
  WORD32 lp_avg_present_val;
  FLOAT32 lp_avg_val;
  WORD32 selected[DRC_INSTRUCTIONS_COUNT_MAX];
  FLOAT32 deviation_min = 1000.0f;
  WORD32 album_mode = pstr_drc_sel_proc_params_struct->album_mode;
  ia_drc_instructions_struct *str_drc_instruction_str;

  FLOAT32 dynamic_range_requested =
      pstr_drc_sel_proc_params_struct->requested_dyn_range_value[num_drc_requests];

  FLOAT32 dynamic_range_min_requested =
      pstr_drc_sel_proc_params_struct->requested_dyn_range_min_val[num_drc_requests];

  FLOAT32 dynamic_range_max_requested =
      pstr_drc_sel_proc_params_struct->requested_dyn_range_max_val[num_drc_requests];

  WORD32 dynamic_range_measurement_type =
      pstr_drc_sel_proc_params_struct->requested_dyn_range_measur_type[num_drc_requests];

  WORD32 requested_dyn_range_range_flag =
      pstr_drc_sel_proc_params_struct->requested_dyn_range_range_flag[num_drc_requests];

  WORD8 *requested_dwnmix_id = pstr_drc_sel_proc_params_struct->requested_dwnmix_id;

  k = 0;
  for (i = 0; i < *selection_candidate_count; i++)
  {
    str_drc_instruction_str =
        &(pstr_drc_config->str_drc_instruction_str[selection_candidate_info[i].drc_instrns_idx]);

    err = impd_drc_loudness_pk_to_avg_info(
        &lp_avg_val, &lp_avg_present_val, pstr_loudness_info, str_drc_instruction_str,
        requested_dwnmix_id[selection_candidate_info[i].downmix_id_request_index],
        dynamic_range_measurement_type, album_mode);
    if (err)
    {
      return (err);
    }

    if (lp_avg_present_val == 1)
    {
      if (requested_dyn_range_range_flag != 1)
      {

        FLOAT32 deviation = (FLOAT32)fabs((FLOAT64)(dynamic_range_requested - lp_avg_val));
        if (deviation_min >= deviation)
        {
          if (deviation_min > deviation)
          {
            deviation_min = deviation;
            k = 0;
          }
          if (k >= DRC_INSTRUCTIONS_COUNT_MAX)
            return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
          selected[k] = i;
          k++;
        }
      }
      else
      {

        if ((lp_avg_val >= dynamic_range_min_requested) &&
            (lp_avg_val <= dynamic_range_max_requested))
        {
          if (k >= DRC_INSTRUCTIONS_COUNT_MAX)
            return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
          selected[k] = i;
          k++;
        }
      }
    }
  }
  if (k > 0)
  {
    for (i = 0; i < k; i++)
    {
      memcpy(&selection_candidate_info[i], &selection_candidate_info[selected[i]],
             sizeof(ia_drc_selection_candidate_info_struct));
    }
    *selection_candidate_count = k;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_match_drc_characteristic_attempt
 *  \brief Match DRC characterstic attempt
 *
 *  \param [in/out] pstr_drc_config
 *  \param [in] requested_drc_characteristic
 *  \param [out] match_found_flag
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_match_drc_characteristic_attempt(
    ia_drc_config *pstr_drc_config, WORD32 requested_drc_characteristic, WORD32 *match_found_flag,
    WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  WORD32 i, k, n, b, m;
  WORD32 ref_count;
  WORD32 drc_characteristic;
  WORD32 drc_characteristic_request_1;
  WORD32 drc_characteristic_request_2;
  WORD32 drc_characteristic_request_3;
  FLOAT32 match_count;
  ia_drc_instructions_struct *str_drc_instruction_str = NULL;
  ia_drc_uni_drc_coeffs_struct *str_p_loc_drc_coefficients_uni_drc = NULL;
  ia_drc_gain_set_params_struct *gain_set_params = NULL;
  *match_found_flag = 0;

  if (requested_drc_characteristic < 1)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }
  if (requested_drc_characteristic >= 12)
  {
    drc_characteristic_request_1 = requested_drc_characteristic;
    drc_characteristic_request_2 = -1;
    drc_characteristic_request_3 = -1;
  }
  else
  {
    drc_characteristic_request_1 =
        ia_drc_characteristic_order_default[requested_drc_characteristic - 1][0];
    drc_characteristic_request_2 =
        ia_drc_characteristic_order_default[requested_drc_characteristic - 1][1];
    drc_characteristic_request_3 =
        ia_drc_characteristic_order_default[requested_drc_characteristic - 1][2];
  }

  if (pstr_drc_config->drc_coefficients_drc_count)
  {
    for (i = 0; i < pstr_drc_config->drc_coefficients_drc_count; i++)
    {
      str_p_loc_drc_coefficients_uni_drc =
          &(pstr_drc_config->str_p_loc_drc_coefficients_uni_drc[i]);
      if (str_p_loc_drc_coefficients_uni_drc->drc_location == LOCATION_SELECTED)
        break;
    }

    if (i == pstr_drc_config->drc_coefficients_drc_count)
    {
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
    }
  }

  if (str_p_loc_drc_coefficients_uni_drc == NULL)
  {
    return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_UNI_DRC_COEF;
  }

  n = 0;
  for (i = 0; i < *selection_candidate_count; i++)
  {
    ref_count = 0;
    match_count = 0;

    str_drc_instruction_str =
        &(pstr_drc_config->str_drc_instruction_str[selection_candidate_info[i].drc_instrns_idx]);
    for (k = 0; k < str_drc_instruction_str->num_drc_ch_groups; k++)
    {
      gain_set_params =
          &(str_p_loc_drc_coefficients_uni_drc
                ->gain_set_params[str_drc_instruction_str->gain_set_index_for_channel_group[k]]);
      if (gain_set_params == NULL)
      {
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      }
      for (b = 0; b < gain_set_params->band_count; b++)
      {
        ref_count++;
        drc_characteristic = gain_set_params->gain_params[b].drc_characteristic;
        if (drc_characteristic == drc_characteristic_request_3)
        {
          match_count += 0.5f;
        }
        else if (drc_characteristic == drc_characteristic_request_2)
        {
          match_count += 0.75f;
        }
        else if (drc_characteristic == drc_characteristic_request_1)
        {
          match_count += 1.0f;
        }
      }
    }
    if (str_drc_instruction_str->depends_on_drc_set_present == 1)
    {
      WORD32 depends_on_drc_set = str_drc_instruction_str->depends_on_drc_set;
      for (m = 0; m < pstr_drc_config->drc_instructions_uni_drc_count; m++)
      {
        if (pstr_drc_config->str_drc_instruction_str[m].drc_set_id == depends_on_drc_set)
          break;
      }
      if (m == pstr_drc_config->drc_instructions_uni_drc_count)
      {
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      }
      str_drc_instruction_str = &(pstr_drc_config->str_drc_instruction_str[m]);
      if ((str_drc_instruction_str->drc_set_effect &
           (IA_DRC_EFFECT_BIT_FADE | IA_DRC_EFFECT_BIT_DUCK_OTHER |
            IA_DRC_EFFECT_BIT_DUCK_SELF)) == 0)
      {
        if (str_drc_instruction_str->drc_set_effect != IA_DRC_EFFECT_BIT_CLIPPING)
        {
          for (k = 0; k < str_drc_instruction_str->num_drc_ch_groups; k++)
          {
            gain_set_params =
                &(str_p_loc_drc_coefficients_uni_drc->gain_set_params
                      [str_drc_instruction_str->gain_set_index_for_channel_group[k]]);
            if (gain_set_params == NULL)
            {
              return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
            }
            for (b = 0; b < gain_set_params->band_count; b++)
            {
              ref_count++;
              drc_characteristic = gain_set_params->gain_params[b].drc_characteristic;
              if (drc_characteristic == drc_characteristic_request_3)
              {
                match_count += 0.5f;
              }
              else if (drc_characteristic == drc_characteristic_request_2)
              {
                match_count += 0.75f;
              }
              else if (drc_characteristic == drc_characteristic_request_1)
              {
                match_count += 1.0f;
              }
            }
          }
        }
      }
    }
    if ((ref_count > 0) && (((FLOAT32)match_count) > 0.5f * ref_count))
    {
      if (n >= SELECTION_CANDIDATE_COUNT_MAX)
        return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
      memcpy(&selection_candidate_info[n], &selection_candidate_info[i],
             sizeof(ia_drc_selection_candidate_info_struct));
      n++;
    }
  }
  if (n > 0)
  {
    *selection_candidate_count = n;
    *match_found_flag = 1;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_match_drc_characteristic
 *  \brief Match DRC characterstics
 *
 *  \param [in/out] pstr_drc_config
 *  \param [in] requested_drc_characteristic
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_drc_match_drc_characteristic(
    ia_drc_config *pstr_drc_config, WORD32 requested_drc_characteristic,
    WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  WORD32 k;
  WORD32 match_found_flag = 0;
  WORD32 drc_characteristic_order_count =
      sizeof(ia_drc_characteristic_order_default[requested_drc_characteristic]) / sizeof(WORD32);
  const WORD32 *drc_characteristic_order =
      ia_drc_characteristic_order_default[requested_drc_characteristic - 1];
  IA_ERRORCODE err;
  k = 0;
  while ((k < drc_characteristic_order_count) && (match_found_flag == 0) &&
         (drc_characteristic_order[k] > 0))
  {
    err = impd_match_drc_characteristic_attempt(pstr_drc_config, drc_characteristic_order[k],
                                                &match_found_flag, selection_candidate_count,
                                                selection_candidate_info);
    if (err)
    {
      return (err);
    }
    k++;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_select_drc_coeff3
 *  \brief Select DRC coef
 *
 *  \param [in/out] pstr_drc_config
 *  \param [ouy] requested_drc_characteristic
 *
 *  \return
 *
 */
static VOID
impd_select_drc_coeff3(ia_drc_config *drc_config,
                       ia_drc_uni_drc_coeffs_struct **pstr_p_loc_drc_coefficients_uni_drc)
{
  WORD32 n;
  WORD32 cV0 = -1;
  for (n = 0; n < drc_config->drc_coefficients_drc_count; n++)
  {
    if (drc_config->str_p_loc_drc_coefficients_uni_drc[n].drc_location == 1 &&
        drc_config->str_p_loc_drc_coefficients_uni_drc[n].version == 0)
    {
      {
        cV0 = n;
      }
    }
  }
  if (cV0 < 0)
  {
    *pstr_p_loc_drc_coefficients_uni_drc = NULL;
  }
  else
  {
    *pstr_p_loc_drc_coefficients_uni_drc = &(drc_config->str_p_loc_drc_coefficients_uni_drc[cV0]);
  }

  return;
}

/**
 *
 *	impd_drc_set_pre_selection
 *  \brief Preslect DRC preset
 *
 *  \param [in/out] pstr_drc_sel_proc_params_struct
 *  \param [in/out] pstr_drc_config
 *  \param [in] pstr_loudness_info
 *  \param [in] restrict_to_drc_with_album_loudness
 *  \param [out] selection_candidate_count
 *  \param [out] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_drc_set_pre_selection(
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
    ia_drc_config *pstr_drc_config, ia_drc_loudness_info_set_struct *pstr_loudness_info,
    WORD32 restrict_to_drc_with_album_loudness, WORD32 *selection_candidate_count,
    ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  IA_ERRORCODE err;
  WORD8 *requested_dwnmix_id = pstr_drc_sel_proc_params_struct->requested_dwnmix_id;
  WORD32 i, j, k, l, d, n, cnt;
  WORD32 omit_preselection_based_on_requested_group_id;
  WORD32 selection_candidate_step_2_count;
  WORD32 peak_info_count;
  WORD32 loudness_drc_set_id_requested;
  WORD32 eq_set_id_loudness[16];
  WORD32 eq_set_id_Peak[16];
  WORD32 explicit_peak_information_present[16];
  WORD32 num_compression_eq_id[16];
  WORD32 downmix_id_match = 0;
  WORD32 num_compression_eq_count = 0;
  WORD32 loudness_info_count = 0;
  WORD32 num_downmix_id_requests = pstr_drc_sel_proc_params_struct->num_downmix_id_requests;
  WORD32 loudness_deviation_max = pstr_drc_sel_proc_params_struct->loudness_deviation_max;
  FLOAT32 adjustment;
  FLOAT32 loud_norm_gain_db[16];
  FLOAT32 loudness[16];
  FLOAT32 signal_peak_level[16];
  FLOAT32 output_peak_level_min = 1000.0f;
  FLOAT32 output_peak_level_max = pstr_drc_sel_proc_params_struct->output_peak_level_max;
  ia_drc_uni_drc_coeffs_struct *str_p_loc_drc_coefficients_uni_drc = NULL;
  ia_drc_instructions_struct *str_drc_instruction_str = NULL;
  ia_drc_selection_candidate_info_struct
      selection_candidate_info_step_2[SELECTION_CANDIDATE_COUNT_MAX];
  memset(&selection_candidate_info_step_2[0], 0,
         SELECTION_CANDIDATE_COUNT_MAX * sizeof(ia_drc_selection_candidate_info_struct));
  impd_select_drc_coeff3(pstr_drc_config, &str_p_loc_drc_coefficients_uni_drc);
  k = 0;
  for (d = 0; d < num_downmix_id_requests; d++)
  {
    for (i = 0; i < pstr_drc_config->drc_instructions_count_plus; i++)
    {
      downmix_id_match = 0;
      str_drc_instruction_str = &(pstr_drc_config->str_drc_instruction_str[i]);

      for (j = 0; j < str_drc_instruction_str->dwnmix_id_count; j++)
      {
        if (((str_drc_instruction_str->downmix_id[j] == ID_FOR_BASE_LAYOUT) &&
             (str_drc_instruction_str->drc_set_id > 0)) ||
            (str_drc_instruction_str->downmix_id[j] == requested_dwnmix_id[d]) ||
            (str_drc_instruction_str->downmix_id[j] == ID_FOR_ANY_DOWNMIX))
        {
          downmix_id_match = 1;
        }
      }
      if (downmix_id_match == 1)
      {
        if (pstr_drc_sel_proc_params_struct->dynamic_range_control_on != 1)
        {
          if (str_drc_instruction_str->drc_set_id < 0)
          {
            err = impd_drc_init_loudness_control(
                &loudness_info_count, eq_set_id_loudness, loud_norm_gain_db, loudness,
                pstr_drc_sel_proc_params_struct, pstr_loudness_info, requested_dwnmix_id[d],
                str_drc_instruction_str, num_compression_eq_count, num_compression_eq_id);
            if (err)
              return (err);

            impd_drc_signal_peak_level_info(&peak_info_count, eq_set_id_Peak, signal_peak_level,
                                            explicit_peak_information_present, pstr_drc_config,
                                            pstr_loudness_info, str_drc_instruction_str,
                                            requested_dwnmix_id[d],
                                            pstr_drc_sel_proc_params_struct->album_mode,
                                            num_compression_eq_count, num_compression_eq_id);
            for (l = 0; l < loudness_info_count; l++)
            {
              WORD32 match_found_flag = 0;
              WORD32 p;
              if (k >= SELECTION_CANDIDATE_COUNT_MAX)
                return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
              for (p = 0; p < peak_info_count; p++)
              {
                if (eq_set_id_Peak[p] == eq_set_id_loudness[l])
                {
                  match_found_flag = 1;
                  break;
                }
              }
              if (match_found_flag == 1)
              {
                adjustment =
                    ia_max_flt(0,
                               signal_peak_level[p] + loud_norm_gain_db[l] -
                                   pstr_drc_sel_proc_params_struct->output_peak_level_max);
                adjustment =
                    ia_min_flt(adjustment, (FLOAT32)ia_max_int(0, loudness_deviation_max));

                selection_candidate_info[k].loud_norm_db_gain_adjust =
                    (loud_norm_gain_db[l] - adjustment);

                selection_candidate_info[k].loud_norm_db_gain_adjust =
                    ia_min_flt(selection_candidate_info[k].loud_norm_db_gain_adjust,
                               pstr_drc_sel_proc_params_struct->loudness_norm_gain_db_max);
                pstr_drc_config->loudness_gain_db =
                    selection_candidate_info[k].loud_norm_db_gain_adjust;

                selection_candidate_info[k].output_peak_level =
                    signal_peak_level[p] + selection_candidate_info[k].loud_norm_db_gain_adjust;
                if (loudness[l] == UNDEFINED_LOUDNESS_VALUE)
                {
                  selection_candidate_info[k].output_loudness = UNDEFINED_LOUDNESS_VALUE;
                }
                else
                {
                  selection_candidate_info[k].output_loudness =
                      loudness[l] + selection_candidate_info[k].loud_norm_db_gain_adjust;
                }
                selection_candidate_info[k].drc_instrns_idx = i;
                selection_candidate_info[k].downmix_id_request_index = d;
                selection_candidate_info[k].eq_set_id = eq_set_id_loudness[l];
                if (explicit_peak_information_present[p] != 1)
                {
                  selection_candidate_info[k].selection_flags = 0;
                }
                else
                {
                  selection_candidate_info[k].selection_flags =
                      IA_SELECTION_FLAG_EXPLICIT_PEAK_INFO_PRESENT;
                }
                k++;
              }
            }
          }
        }
        else
        {
          if ((str_drc_instruction_str->drc_set_effect != 0 ||
               str_drc_instruction_str->drc_set_id < 0) &&
              (((str_drc_instruction_str->depends_on_drc_set_present == 0) &&
                (str_drc_instruction_str->no_independent_use == 0)) ||
               (str_drc_instruction_str->depends_on_drc_set_present == 1)) &&
              (str_drc_instruction_str->drc_set_effect != IA_DRC_EFFECT_BIT_FADE) &&
              (str_drc_instruction_str->drc_set_effect != IA_DRC_EFFECT_BIT_DUCK_OTHER) &&
              (str_drc_instruction_str->drc_set_effect != IA_DRC_EFFECT_BIT_DUCK_SELF))
          {
            WORD32 drc_is_permitted = 1;
            if (str_drc_instruction_str->depends_on_drc_set > 0)
            {
              ia_drc_instructions_struct *drc_inst_uni_drc_dependent;

              for (cnt = 0; cnt < pstr_drc_config->drc_instructions_uni_drc_count; cnt++)
              {
                if (str_drc_instruction_str->depends_on_drc_set ==
                    pstr_drc_config->str_drc_instruction_str[cnt].drc_set_id)
                  break;
              }
              if (cnt == pstr_drc_config->drc_instructions_uni_drc_count)
              {
                return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
              }
              drc_inst_uni_drc_dependent = &pstr_drc_config->str_drc_instruction_str[cnt];

              if (err)
                return (err);
              if (str_p_loc_drc_coefficients_uni_drc != NULL)
              {
                for (j = 0; j < str_drc_instruction_str->num_drc_ch_groups; j++)
                {
                  ia_drc_gain_set_params_struct *gain_set_params =
                      &(str_p_loc_drc_coefficients_uni_drc->gain_set_params
                            [drc_inst_uni_drc_dependent->gain_set_index_for_channel_group[j]]);
                  if (gain_set_params->band_count >
                      pstr_drc_sel_proc_params_struct->num_bands_supported)
                  {
                    drc_is_permitted = 0;
                  }
                }
              }
            }

            if (str_p_loc_drc_coefficients_uni_drc != NULL)
            {
              for (j = 0; j < str_drc_instruction_str->num_drc_ch_groups; j++)
              {
                ia_drc_gain_set_params_struct *gain_set_params =
                    &(str_p_loc_drc_coefficients_uni_drc->gain_set_params
                          [str_drc_instruction_str->gain_set_index_for_channel_group[j]]);
                if (gain_set_params->band_count >
                    pstr_drc_sel_proc_params_struct->num_bands_supported)
                {
                  drc_is_permitted = 0;
                }
              }
            }

            if (drc_is_permitted == 1)
            {
              err = impd_drc_init_loudness_control(
                  &loudness_info_count, eq_set_id_loudness, loud_norm_gain_db, loudness,
                  pstr_drc_sel_proc_params_struct, pstr_loudness_info, requested_dwnmix_id[d],
                  str_drc_instruction_str, num_compression_eq_count, num_compression_eq_id);

              if (err)
              {
                return (err);
              }

              if (loudness_info_count > MAX_LOUDNESS_INFO_COUNT)
              {
                return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
              }

              impd_drc_signal_peak_level_info(&peak_info_count, eq_set_id_Peak, signal_peak_level,
                                              explicit_peak_information_present, pstr_drc_config,
                                              pstr_loudness_info, str_drc_instruction_str,
                                              requested_dwnmix_id[d],
                                              pstr_drc_sel_proc_params_struct->album_mode,
                                              num_compression_eq_count, num_compression_eq_id);

              for (l = 0; l < loudness_info_count; l++)
              {
                WORD32 match_found_flag = 0;
                WORD32 p;
                if (k >= SELECTION_CANDIDATE_COUNT_MAX)
                {
                  return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
                }
                selection_candidate_info[k].loud_norm_db_gain_adjust = loud_norm_gain_db[l];

                selection_candidate_info[k].loud_norm_db_gain_adjust =
                    ia_min_flt(selection_candidate_info[k].loud_norm_db_gain_adjust,
                               pstr_drc_sel_proc_params_struct->loudness_norm_gain_db_max);

                pstr_drc_config->loudness_gain_db =
                    selection_candidate_info[k].loud_norm_db_gain_adjust;

                if (loudness[l] == UNDEFINED_LOUDNESS_VALUE)
                {
                  selection_candidate_info[k].output_loudness = UNDEFINED_LOUDNESS_VALUE;
                }
                else
                {
                  selection_candidate_info[k].output_loudness = ia_add_flt(
                      loudness[l], selection_candidate_info[k].loud_norm_db_gain_adjust);
                }

                for (p = 0; p < peak_info_count; p++)
                {
                  if (eq_set_id_Peak[p] == eq_set_id_loudness[l])
                  {
                    match_found_flag = 1;
                    break;
                  }
                }
                if (match_found_flag == 0)
                {
                  selection_candidate_info[k].output_peak_level =
                      selection_candidate_info[k].loud_norm_db_gain_adjust;
                }
                else
                {
                  selection_candidate_info[k].output_peak_level = ia_add_flt(
                      signal_peak_level[p], selection_candidate_info[k].loud_norm_db_gain_adjust);
                }
                selection_candidate_info[k].drc_instrns_idx = i;
                selection_candidate_info[k].downmix_id_request_index = d;
                selection_candidate_info[k].eq_set_id = eq_set_id_loudness[l];
                if (explicit_peak_information_present[p] != 1)
                {
                  selection_candidate_info[k].selection_flags = 0;
                }
                else
                {
                  selection_candidate_info[k].selection_flags =
                      IA_SELECTION_FLAG_EXPLICIT_PEAK_INFO_PRESENT;
                }
                if (str_drc_instruction_str->drc_set_target_loudness_present &&
                    ((pstr_drc_sel_proc_params_struct->loudness_normalization_on &&
                      str_drc_instruction_str->drc_set_target_loudness_value_upper >=
                          (FLOAT32)pstr_drc_sel_proc_params_struct->target_loudness &&
                      str_drc_instruction_str->drc_set_target_loudness_value_lower <
                          (FLOAT32)pstr_drc_sel_proc_params_struct->target_loudness) ||
                     !pstr_drc_sel_proc_params_struct->loudness_normalization_on))

                {
                  selection_candidate_info[k].selection_flags |=
                      IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH;
                  if (!explicit_peak_information_present[p])
                  {
                    if (!pstr_drc_sel_proc_params_struct->loudness_normalization_on)
                    {
                      selection_candidate_info[k].output_peak_level = 0;
                    }
                    else
                    {

                      selection_candidate_info[k].output_peak_level =
                          pstr_drc_sel_proc_params_struct->target_loudness -
                          str_drc_instruction_str->drc_set_target_loudness_value_upper;
                    }
                  }
                }
                if ((selection_candidate_info[k].selection_flags &
                         (IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH |
                          IA_SELECTION_FLAG_EXPLICIT_PEAK_INFO_PRESENT) ||
                     !str_drc_instruction_str->drc_set_target_loudness_present))
                {
                  k++;
                }
              }
            }
          }
        }
      }
    }
  }
  if (k > SELECTION_CANDIDATE_COUNT_MAX)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
  }
  *selection_candidate_count = k;
  if (pstr_drc_sel_proc_params_struct->dynamic_range_control_on == 1)
  {
    n = 0;
    for (k = 0; k < *selection_candidate_count; k++)
    {
      if (!((selection_candidate_info[k].selection_flags &
             IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH) &&
            !(selection_candidate_info[k].selection_flags &
              IA_SELECTION_FLAG_EXPLICIT_PEAK_INFO_PRESENT)))
      {
        if (selection_candidate_info[k].output_peak_level <= output_peak_level_max)
        {
          memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                 sizeof(ia_drc_selection_candidate_info_struct));
          n++;
        }

        if (selection_candidate_info[k].output_peak_level < output_peak_level_min)

        {
          str_drc_instruction_str =
              &(pstr_drc_config
                    ->str_drc_instruction_str[selection_candidate_info[k].drc_instrns_idx]);
          if (str_drc_instruction_str->drc_instructions_type == 2)
          {

            for (i = 0; i < pstr_drc_sel_proc_params_struct->num_group_ids_requested; i++)
            {
              if (str_drc_instruction_str->mae_group_id ==
                  pstr_drc_sel_proc_params_struct->group_id_requested[i])
              {

                output_peak_level_min = selection_candidate_info[k].output_peak_level;

                break;
              }
            }
          }
          else if (str_drc_instruction_str->drc_instructions_type == 3)
          {
            for (i = 0; i < pstr_drc_sel_proc_params_struct->num_group_preset_ids_requested; i++)
            {
              if (str_drc_instruction_str->mae_group_preset_id ==
                  pstr_drc_sel_proc_params_struct->group_preset_id_requested[i])
              {

                output_peak_level_min = selection_candidate_info[k].output_peak_level;

                break;
              }
            }
          }
          else
          {

            output_peak_level_min = selection_candidate_info[k].output_peak_level;
          }
        }
      }
      else
      {
        memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
               sizeof(ia_drc_selection_candidate_info_struct));
        n++;
      }
    }
    selection_candidate_step_2_count = n;
    if (selection_candidate_step_2_count == 0)
    {
      n = 0;
      for (k = 0; k < *selection_candidate_count; k++)
      {
        if ((selection_candidate_info[k].selection_flags &
             IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH) &&
            (selection_candidate_info[k].selection_flags &
             IA_SELECTION_FLAG_EXPLICIT_PEAK_INFO_PRESENT))
        {
          memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                 sizeof(ia_drc_selection_candidate_info_struct));
          n++;
        }
      }
      selection_candidate_step_2_count = n;
      if (n == 0)
      {
        for (k = 0; k < *selection_candidate_count; k++)
        {
          if (selection_candidate_info_step_2[k].output_peak_level <
              (FLOAT32)output_peak_level_min + 1.0f)
          {
            memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                   sizeof(ia_drc_selection_candidate_info_struct));
            adjustment = ia_max_flt(0,
                                    selection_candidate_info_step_2[n].output_peak_level -
                                        (output_peak_level_max));
            adjustment = ia_min_flt(adjustment, (FLOAT32)ia_max_int(0, loudness_deviation_max));
            selection_candidate_info_step_2[n].loud_norm_db_gain_adjust -= adjustment;
            selection_candidate_info_step_2[n].output_peak_level -= adjustment;
            selection_candidate_info_step_2[n].output_loudness -= adjustment;
            n++;
          }
        }
        selection_candidate_step_2_count = n;
      }
    }

    n = 0;
    omit_preselection_based_on_requested_group_id = 0;
    for (k = 0; k < selection_candidate_step_2_count; k++)
    {
      str_drc_instruction_str = &(
          pstr_drc_config->str_drc_instruction_str[selection_candidate_info[k].drc_instrns_idx]);
      if (str_drc_instruction_str->drc_instructions_type == 3)
      {

        for (i = 0; i < pstr_drc_sel_proc_params_struct->num_group_preset_ids_requested; i++)
        {
          if (str_drc_instruction_str->mae_group_preset_id ==
              pstr_drc_sel_proc_params_struct->group_preset_id_requested[i])
          {
            memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                   sizeof(ia_drc_selection_candidate_info_struct));
            omit_preselection_based_on_requested_group_id = 1;
            n++;
          }
        }
      }
      else if (str_drc_instruction_str->drc_instructions_type == 0 ||
               str_drc_instruction_str->drc_instructions_type == 2)
      {
        memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
               sizeof(ia_drc_selection_candidate_info_struct));
        n++;
      }
    }
    selection_candidate_step_2_count = n;
    if (omit_preselection_based_on_requested_group_id != 1)
    {
      n = 0;
      for (k = 0; k < selection_candidate_step_2_count; k++)
      {
        str_drc_instruction_str =
            &(pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info[k].drc_instrns_idx]);
        if (str_drc_instruction_str->drc_instructions_type == 0)
        {
          memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                 sizeof(ia_drc_selection_candidate_info_struct));
          n++;
        }
        else if (str_drc_instruction_str->drc_instructions_type == 2)
        {
          for (i = 0; i < pstr_drc_sel_proc_params_struct->num_group_ids_requested; i++)
          {
            if (str_drc_instruction_str->mae_group_id ==
                pstr_drc_sel_proc_params_struct->group_id_requested[i])
            {
              memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                     sizeof(ia_drc_selection_candidate_info_struct));
              n++;
            }
          }
        }
      }
      selection_candidate_step_2_count = n;
    }
    else
    {
      n = 0;
      for (k = 0; k < selection_candidate_step_2_count; k++)
      {
        str_drc_instruction_str =
            &(pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info[k].drc_instrns_idx]);
        if (str_drc_instruction_str->drc_instructions_type == 0 ||
            str_drc_instruction_str->drc_instructions_type == 3)
        {
          memcpy(&selection_candidate_info_step_2[n], &selection_candidate_info[k],
                 sizeof(ia_drc_selection_candidate_info_struct));
          n++;
        }
      }
      selection_candidate_step_2_count = n;
    }

    for (n = 0; n < selection_candidate_step_2_count; n++)
    {
      memcpy(&selection_candidate_info[n], &selection_candidate_info_step_2[n],
             sizeof(ia_drc_selection_candidate_info_struct));
    }
    *selection_candidate_count = selection_candidate_step_2_count;
  }

  if (restrict_to_drc_with_album_loudness == 1)
  {
    j = 0;
    for (k = 0; k < *selection_candidate_count; k++)
    {
      loudness_drc_set_id_requested = ia_max_int(
          0,
          pstr_drc_config->str_drc_instruction_str[selection_candidate_info[k].drc_instrns_idx]
              .drc_set_id);
      for (n = 0; n < pstr_loudness_info->loudness_info_album_count; n++)
      {
        if (loudness_drc_set_id_requested ==
            pstr_loudness_info->str_loudness_info_album[n].drc_set_id)
        {
          if (j >= SELECTION_CANDIDATE_COUNT_MAX)
          {
            return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
          }
          memcpy(&selection_candidate_info[j], &selection_candidate_info[k],
                 sizeof(ia_drc_selection_candidate_info_struct));
          j++;
          break;
        }
      }
    }
    *selection_candidate_count = j;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_set_final_selection
 *  \brief DRC set final select
 *
 *  \param [in/out] pstr_drc_config
 *  \param [in/out] pstr_drc_sel_proc_params_struct
 *  \param [in] selection_candidate_count
 *  \param [in] selection_candidate_info
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE
impd_drc_set_final_selection(ia_drc_config *pstr_drc_config,
                             ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
                             WORD32 *selection_candidate_count,
                             ia_drc_selection_candidate_info_struct *selection_candidate_info)
{
  IA_ERRORCODE err;
  WORD32 k, i, n;
  WORD32 selection_candidate_step_2_count;
  WORD32 num_members_group_preset_id = 0;
  WORD32 max_num_members_group_preset_ids = 0;
  WORD32 mae_group_preset_id_lowest = 1000;
  WORD32 drc_set_id_max;
  WORD32 effect_count, effect_count_min;
  WORD32 effect_types_request_table_size;
  WORD32 drc_set_target_loudness_val_upper_min;
  FLOAT32 output_level_max;
  FLOAT32 output_level_min;
  ia_drc_instructions_struct *str_drc_instruction_str;
  ia_drc_instructions_struct *drc_instructions_dependent;
  ia_drc_selection_candidate_info_struct
      selection_candidate_info_step_2[SELECTION_CANDIDATE_COUNT_MAX] = {{0}};

  k = 0;
  output_level_min = 10000.0f;
  for (i = 0; i < *selection_candidate_count; i++)
  {
    if (output_level_min >= selection_candidate_info[i].output_peak_level)
    {
      if (output_level_min > selection_candidate_info[i].output_peak_level)
      {
        output_level_min = selection_candidate_info[i].output_peak_level;
        k = 0;
      }
      memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
             sizeof(ia_drc_selection_candidate_info_struct));
      k++;
    }
  }
  selection_candidate_step_2_count = k;

  if (output_level_min <= 0)
  {
    selection_candidate_step_2_count = *selection_candidate_count;
    k = 0;
    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      if (selection_candidate_info[i].output_peak_level <= 0)
      {
        memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info[i],
               sizeof(ia_drc_selection_candidate_info_struct));
        k++;
      }
    }
    selection_candidate_step_2_count = k;

    /* final MPEG-H selection based on groupPresetId and groupId */
    k = 0;
    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      str_drc_instruction_str =
          &(pstr_drc_config
                ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
      if (str_drc_instruction_str->drc_instructions_type == 3)
      {
        k++;
      }
    }
    if (k == 0)
    {
      k = 0;
      for (i = 0; i < selection_candidate_step_2_count; i++)
      {
        str_drc_instruction_str =
            &(pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
        if (str_drc_instruction_str->drc_instructions_type == 2)
        {
          k++;
        }
      }
      if (k != 0)
      {
        k = 0;
        for (i = 0; i < selection_candidate_step_2_count; i++)
        {
          str_drc_instruction_str = &(
              pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
          if (str_drc_instruction_str->drc_instructions_type == 2)
          {
            memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k++;
          }
        }
        selection_candidate_step_2_count = k;
      }
    }
    else
    {

      k = 0;
      for (i = 0; i < selection_candidate_step_2_count; i++)
      {
        str_drc_instruction_str =
            &(pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
        if (str_drc_instruction_str->drc_instructions_type == 3)
        {
          if (pstr_drc_sel_proc_params_struct->group_preset_id_requested_preference ==
              str_drc_instruction_str->mae_group_preset_id)
          {
            memcpy(&selection_candidate_info_step_2[0], &selection_candidate_info_step_2[i],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k = 1;
            break;
          }
        }
      }
      if (k == 0)
      {
        for (i = 0; i < selection_candidate_step_2_count; i++)
        {
          str_drc_instruction_str = &(
              pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
          if (str_drc_instruction_str->drc_instructions_type == 3)
          {
            for (n = 0; n < pstr_drc_sel_proc_params_struct->num_group_preset_ids_requested; n++)
            {
              if (pstr_drc_sel_proc_params_struct->group_preset_id_requested[n] ==
                  str_drc_instruction_str->mae_group_preset_id)
              {
                num_members_group_preset_id =
                    pstr_drc_sel_proc_params_struct->num_members_group_preset_ids_requested[n];
                break;
              }
            }
            if (max_num_members_group_preset_ids <= num_members_group_preset_id)
            {
              if (max_num_members_group_preset_ids < num_members_group_preset_id)
              {
                max_num_members_group_preset_ids = num_members_group_preset_id;
                k = 0;
              }
              memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
                     sizeof(ia_drc_selection_candidate_info_struct));
              k++;
            }
          }
        }
      }
      if (k > 1)
      {
        for (n = 0; n < k; n++)
        {
          str_drc_instruction_str = &(
              pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info_step_2[n].drc_instrns_idx]);
          if (mae_group_preset_id_lowest > str_drc_instruction_str->mae_group_preset_id)
          {
            mae_group_preset_id_lowest = str_drc_instruction_str->mae_group_preset_id;
            memcpy(&selection_candidate_info_step_2[0], &selection_candidate_info_step_2[n],
                   sizeof(ia_drc_selection_candidate_info_struct));
            k = 1;
          }
        }
      }
      selection_candidate_step_2_count = k;
    }

    k = 0;
    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      str_drc_instruction_str =
          &(pstr_drc_config
                ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
      for (n = 0; n < str_drc_instruction_str->dwnmix_id_count; n++)
      {
        if (pstr_drc_sel_proc_params_struct->requested_dwnmix_id
                [selection_candidate_info_step_2[i].downmix_id_request_index] ==
            str_drc_instruction_str->downmix_id[n])
        {
          if (k >= SELECTION_CANDIDATE_COUNT_MAX)
            return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
          memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
                 sizeof(ia_drc_selection_candidate_info_struct));
          k++;
        }
      }
    }
    if (k > 0)
    {
      selection_candidate_step_2_count = k;
    }

    effect_types_request_table_size = sizeof(effect_types_request_table) / sizeof(WORD32);
    effect_count_min = 100;
    k = 0;
    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      str_drc_instruction_str =
          &(pstr_drc_config
                ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
      effect_count = 0;
      if (str_drc_instruction_str->depends_on_drc_set_present != 1)
      {
        for (n = 0; n < effect_types_request_table_size; n++)
        {
          if (effect_types_request_table[n] != IA_DRC_EFFECT_BIT_GENERAL_COMPR)
          {
            if ((str_drc_instruction_str->drc_set_effect & effect_types_request_table[n]) != 0x0)
            {
              effect_count++;
            }
          }
        }
      }
      else
      {
        if (pstr_drc_config->drc_instructions_uni_drc_count > 0)
        {
          err = impd_drc_get_dependent_drc_instructions(pstr_drc_config, str_drc_instruction_str,
                                                        &drc_instructions_dependent);
          if (err)
            return (err);

          for (n = 0; n < effect_types_request_table_size; n++)
          {
            if (effect_types_request_table[n] != IA_DRC_EFFECT_BIT_GENERAL_COMPR)
            {
              if (((str_drc_instruction_str->drc_set_effect & effect_types_request_table[n]) !=
                   0x0) ||
                  ((drc_instructions_dependent->drc_set_effect & effect_types_request_table[n]) !=
                   0x0))
              {
                effect_count++;
              }
            }
          }
        }
      }
      if (effect_count_min >= effect_count)
      {
        if (effect_count_min > effect_count)
        {
          effect_count_min = effect_count;
          k = 0;
        }
        memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
               sizeof(ia_drc_selection_candidate_info_struct));
        k++;
      }
    }
    selection_candidate_step_2_count = k;

    drc_set_target_loudness_val_upper_min = 100;
    k = 0;
    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      if (selection_candidate_info_step_2[i].selection_flags &
          IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH)
      {
        k++;
      }
    }
    if (k != 0 && k != selection_candidate_step_2_count)
    {
      k = 0;
      for (i = 0; i < selection_candidate_step_2_count; i++)
      {
        if (!(selection_candidate_info_step_2[i].selection_flags &
              IA_SELECTION_FLAG_DRC_TARGET_LOUDNESS_MATCH))
        {
          memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
                 sizeof(ia_drc_selection_candidate_info_struct));
          k++;
        }
      }
      selection_candidate_step_2_count = k;
    }
    else if (k == selection_candidate_step_2_count)
    {
      k = 0;
      for (i = 0; i < selection_candidate_step_2_count; i++)
      {
        str_drc_instruction_str =
            &(pstr_drc_config
                  ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
        if (str_drc_instruction_str->drc_set_target_loudness_present != 1)
        {
          return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
        }
        if (drc_set_target_loudness_val_upper_min >=
            str_drc_instruction_str->drc_set_target_loudness_value_upper)
        {
          if (drc_set_target_loudness_val_upper_min >
              str_drc_instruction_str->drc_set_target_loudness_value_upper)
          {
            drc_set_target_loudness_val_upper_min =
                str_drc_instruction_str->drc_set_target_loudness_value_upper;
            k = 0;
          }
          memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
                 sizeof(ia_drc_selection_candidate_info_struct));
          k++;
        }
      }
      selection_candidate_step_2_count = k;
    }

    k = 0;
    output_level_max = -1000.0f;

    for (i = 0; i < selection_candidate_step_2_count; i++)
    {
      if ((selection_candidate_info_step_2[i].output_peak_level <= 0.0f) &&
          (output_level_max <= selection_candidate_info_step_2[i].output_peak_level))
      {
        if (output_level_max < selection_candidate_info_step_2[i].output_peak_level)
        {
          k = 0;
        }
        memcpy(&selection_candidate_info_step_2[k], &selection_candidate_info_step_2[i],
               sizeof(ia_drc_selection_candidate_info_struct));
        k++;
        output_level_max = selection_candidate_info_step_2[i].output_peak_level;
      }
    }
    selection_candidate_step_2_count = k;
  }

  drc_set_id_max = -1000;
  for (i = 0; i < selection_candidate_step_2_count; i++)
  {
    str_drc_instruction_str =
        &(pstr_drc_config
              ->str_drc_instruction_str[selection_candidate_info_step_2[i].drc_instrns_idx]);
    if (drc_set_id_max < str_drc_instruction_str->drc_set_id)
    {
      drc_set_id_max = str_drc_instruction_str->drc_set_id;
      memcpy(&selection_candidate_info_step_2[0], &selection_candidate_info_step_2[i],
             sizeof(ia_drc_selection_candidate_info_struct));
    }
  }
  memcpy(&selection_candidate_info[0], &selection_candidate_info_step_2[0],
         sizeof(ia_drc_selection_candidate_info_struct));
  *selection_candidate_count = 1;

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCControlParams */