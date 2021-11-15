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
#include <stdlib.h>
#include <string.h>
#include <impeghd_type_def.h>
#include "impeghd_error_codes.h"
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_interface.h"
#include "impd_drc_struct.h"
#include "impd_drc_selection_process.h"
#include "impd_drc_sel_proc_drc_set_sel.h"
#include "ia_core_coder_constants.h"

/**
 * @defgroup DRCControlParams DRCControlParams
 * @ingroup  DRCControlParams
 * @brief DRC Control parameter setting
 *
 * @{
 */

/**
 *
 *	impd_drc_uni_selction_proc_init
 *  \brief Initialize drc uni selection process
 *
 *  \param [in/out] ia_drc_sel_pro_struct structure to be initialized
 *  \param [in] pstr_drc_sel_proc_params_struct
 *  \param [in] pstr_drc_interface
 *  \param [in] subband_domain_mode
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE
impd_drc_uni_selction_proc_init(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
                                ia_drc_interface_struct *pstr_drc_interface,
                                WORD32 subband_domain_mode)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;

  if (pstr_drc_uni_sel_proc == NULL)
  {
    return 1;
  }

  if (pstr_drc_uni_sel_proc->first_frame == 1)
  {
    err = impd_drc_sel_proc_init_dflt(pstr_drc_uni_sel_proc);
    if (err)
    {
      return (err);
    }
  }

  err = impd_drc_sel_proc_init_sel_proc_params(pstr_drc_uni_sel_proc,
                                               pstr_drc_sel_proc_params_struct);
  if (err)
  {
    return (err);
  }

  pstr_drc_uni_sel_proc->subband_domain_mode = subband_domain_mode;
  impd_drc_sel_proc_init_interface_params(pstr_drc_uni_sel_proc, pstr_drc_interface);
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_mpegh_params_set_sel_process
 *  \brief Sets select process
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *  \param [in] num_group_ids_requested
 *  \param [in] group_id_requested
 *  \param [in] num_group_preset_ids_requested
 *  \param [in] num_members_group_preset_ids_requested
 *  \param [in] group_preset_id_requested_preference
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_drc_mpegh_params_set_sel_process(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                                   WORD8 num_group_ids_requested,
                                                   WORD8 *group_id_requested,
                                                   WORD8 num_group_preset_ids_requested,
                                                   WORD8 *group_preset_id_requested,
                                                   WORD8 *num_members_group_preset_ids_requested,
                                                   WORD8 group_preset_id_requested_preference)
{
  WORD32 i;

  if (num_group_ids_requested !=
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_ids_requested)
  {
    pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_ids_requested =
        num_group_ids_requested;
  }
  for (i = 0; i < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_ids_requested; i++)
  {
    if (group_id_requested[i] !=
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_id_requested[i])
    {
      pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_id_requested[i] =
          group_id_requested[i];
    }
  }

  if (num_group_preset_ids_requested !=
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_preset_ids_requested)
  {
    pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_preset_ids_requested =
        num_group_preset_ids_requested;
  }

  for (i = 0;
       i < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_preset_ids_requested; i++)
  {
    if (group_preset_id_requested[i] !=
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_preset_id_requested[i])
    {
      pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_preset_id_requested[i] =
          group_preset_id_requested[i];
    }
    if (num_members_group_preset_ids_requested[i] !=
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
            .num_members_group_preset_ids_requested[i])
    {
      pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
          .num_members_group_preset_ids_requested[i] = num_members_group_preset_ids_requested[i];
    }
  }
  if (group_preset_id_requested_preference !=
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_preset_id_requested_preference)
  {
    pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_preset_id_requested_preference =
        group_preset_id_requested_preference;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_select_drc_set
 *  \brief Select DRC set
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *  \param [in] drc_set_id_selected
 *
 *  \return IA_ERRORCODE error if any
 *
 */
static IA_ERRORCODE impd_select_drc_set(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                        WORD32 *drc_set_id_selected)
{
  WORD32 i;
  IA_ERRORCODE err;
  WORD32 selection_candidate_count = 0;
  WORD32 restrict_to_drc_with_album_loudness = 1;
  ia_drc_config *pstr_drc_config = pstr_drc_uni_sel_proc->drc_config;
  ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct =
      &pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params;
  ia_drc_loudness_info_set_struct *pstr_loudness_info = &pstr_drc_uni_sel_proc->loudness_info_set;
  ia_drc_selection_candidate_info_struct selection_candidate_info[SELECTION_CANDIDATE_COUNT_MAX];

  if (pstr_drc_sel_proc_params_struct->album_mode == 0)
  {
    restrict_to_drc_with_album_loudness = 0;
  }

  while (!selection_candidate_count)
  {
    err = impd_drc_set_pre_selection(pstr_drc_sel_proc_params_struct, pstr_drc_config,
                                     pstr_loudness_info, restrict_to_drc_with_album_loudness,
                                     &selection_candidate_count, selection_candidate_info);
    if (err)
    {
      return err;
    }

    if (selection_candidate_count == 0)
    {
      if (restrict_to_drc_with_album_loudness != 1)
      {
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      }
      else
      {
        restrict_to_drc_with_album_loudness = 0;
        continue;
      }
    }

    err = impd_drc_validate_requested_drc_feature(pstr_drc_sel_proc_params_struct);
    if (err)
    {
      return (err);
    }
    if (pstr_drc_sel_proc_params_struct->dynamic_range_control_on == 1)
    {
      if (pstr_drc_sel_proc_params_struct->num_drc_feature_requests <= 0)
      {
        WORD32 match_found_flag = 0;

        err = impd_drc_select_drcs_without_compr_effects(pstr_drc_config, &match_found_flag,
                                                         &selection_candidate_count,
                                                         selection_candidate_info);
        if (err)
          return (err);

        if (match_found_flag == 0)
        {
          WORD32 requested_num_drc_effects = 5;
          WORD32 desired_num_drc_effects_of_requested = 1;
          WORD8 requested_drc_effect_type[5] = {
              EFFECT_TYPE_REQUESTED_GENERAL_COMPR, EFFECT_TYPE_REQUESTED_NIGHT,
              EFFECT_TYPE_REQUESTED_NOISY, EFFECT_TYPE_REQUESTED_LIMITED,
              EFFECT_TYPE_REQUESTED_LOWLEVEL};

          err = impd_drc_match_effect_types(
              pstr_drc_config, requested_num_drc_effects, desired_num_drc_effects_of_requested,
              requested_drc_effect_type, &selection_candidate_count, selection_candidate_info);
          if (err)
            return (err);
        }
      }
      else
      {
        for (i = 0; i < pstr_drc_sel_proc_params_struct->num_drc_feature_requests; i++)
        {
          switch (pstr_drc_sel_proc_params_struct->drc_feature_req_type[i])
          {
          case MATCH_DYNAMIC_RANGE:
            err = impd_drc_match_dynamic_range(
                pstr_drc_config, pstr_loudness_info, pstr_drc_sel_proc_params_struct, i,
                &selection_candidate_count, selection_candidate_info);
            if (err)
              return (err);
            break;
          case MATCH_DRC_CHARACTERISTIC:
            err = impd_drc_match_drc_characteristic(
                pstr_drc_config, pstr_drc_sel_proc_params_struct->requested_drc_characteristic[i],
                &selection_candidate_count, selection_candidate_info);
            if (err)
              return (err);
            break;
          case MATCH_EFFECT_TYPE:
            err = impd_drc_match_effect_types(
                pstr_drc_config, pstr_drc_sel_proc_params_struct->requested_num_drc_effects[i],
                pstr_drc_sel_proc_params_struct->desired_num_drc_effects_of_requested[i],
                pstr_drc_sel_proc_params_struct->requested_drc_effect_type[i],
                &selection_candidate_count, selection_candidate_info);
            if (err)
              return (err);
            break;
          default:
            return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
            break;
          }
        }
      }

      if (selection_candidate_count <= 0)
      {
        selection_candidate_count = 0;
        {
          return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
        }
      }
      else
      {

        err = impd_drc_set_final_selection(pstr_drc_config, pstr_drc_sel_proc_params_struct,
                                           &selection_candidate_count, selection_candidate_info);
        if (err)
        {
          return (err);
        }
      }
    }

    if (selection_candidate_count == 0)
    {
      if (restrict_to_drc_with_album_loudness != 1)
      {
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      }
      else
      {
        restrict_to_drc_with_album_loudness = 0;
      }
    }
  }
  *drc_set_id_selected =
      pstr_drc_config->str_drc_instruction_str[selection_candidate_info[0].drc_instrns_idx]
          .drc_set_id;
  if (selection_candidate_count > 0)
  {
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.loud_norm_gain_db =
        selection_candidate_info[0].loud_norm_db_gain_adjust;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.output_peak_level_db =
        selection_candidate_info[0].output_peak_level;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.output_loudness =
        selection_candidate_info[0].output_loudness;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.active_downmix_id =
        pstr_drc_sel_proc_params_struct
            ->requested_dwnmix_id[selection_candidate_info[0].downmix_id_request_index];
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_get_selected_drc_set
 *  \brief gets selected drc set
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *  \param [in] drc_set_id_selected
 *
 *  \return IA_ERRORCODE error if any
 *
 */
static IA_ERRORCODE impd_get_selected_drc_set(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                              WORD32 drc_set_id_selected)
{
  WORD32 n;
  for (n = 0; n < pstr_drc_uni_sel_proc->drc_config->drc_instructions_count_plus; n++)
  {
    if (drc_set_id_selected ==
        pstr_drc_uni_sel_proc->drc_config->str_drc_instruction_str[n].drc_set_id)
      break;
  }
  if (pstr_drc_uni_sel_proc->drc_config->drc_instructions_count_plus == n)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }

  pstr_drc_uni_sel_proc->drc_inst_index_sel = (WORD8)n;
  pstr_drc_uni_sel_proc->drc_instrns_idx[0] = pstr_drc_uni_sel_proc->drc_inst_index_sel;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_get_dependent_drc_set
 *  \brief Get dependent DRC set
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *
 *  \return IA_ERRORCODE erro if any
 *
 */
static IA_ERRORCODE impd_get_dependent_drc_set(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc)
{
  ia_drc_instructions_struct *pstr_drc_instruction_str = NULL;
  pstr_drc_instruction_str =
      &(pstr_drc_uni_sel_proc->drc_config
            ->str_drc_instruction_str[pstr_drc_uni_sel_proc->drc_inst_index_sel]);

  if (pstr_drc_instruction_str->depends_on_drc_set_present != 1)
  {
    pstr_drc_uni_sel_proc->drc_instrns_idx[1] = -1;
  }
  else
  {
    WORD8 n;
    WORD32 drc_dependent_set_id = pstr_drc_instruction_str->depends_on_drc_set;

    for (n = 0; n < pstr_drc_uni_sel_proc->drc_config->drc_instructions_count_plus; n++)
    {
      if (pstr_drc_uni_sel_proc->drc_config->str_drc_instruction_str[n].drc_set_id ==
          drc_dependent_set_id)
        break;
    }
    if (pstr_drc_uni_sel_proc->drc_config->drc_instructions_count_plus == n)
    {
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
    }
    pstr_drc_uni_sel_proc->drc_instrns_idx[1] = n;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_get_fading_drc_set
 *  \brief Get fadding DRC set
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *
 *  \return IA_ERRORCODE error if any
 *
 */
static IA_ERRORCODE impd_get_fading_drc_set(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc)
{
  pstr_drc_uni_sel_proc->drc_instrns_idx[2] = -1;
  if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.album_mode == 0)
  {
    WORD32 n;
    ia_drc_instructions_struct *pstr_drc_instruction_str = NULL;
    for (n = 0; n < pstr_drc_uni_sel_proc->drc_config->drc_instructions_uni_drc_count; n++)
    {
      pstr_drc_instruction_str = &(pstr_drc_uni_sel_proc->drc_config->str_drc_instruction_str[n]);

      if (pstr_drc_instruction_str->drc_set_effect & IA_DRC_EFFECT_BIT_FADE)
      {
        if (pstr_drc_instruction_str->downmix_id[0] == ID_FOR_ANY_DOWNMIX)
        {
          if (pstr_drc_instruction_str->drc_instructions_type == 3)
          {
            WORD32 m;
            for (m = 0; m < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                                .num_group_preset_ids_requested;
                 m++)
            {
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .group_preset_id_requested[m] ==
                  pstr_drc_instruction_str->mae_group_preset_id)
              {
                pstr_drc_uni_sel_proc->drc_instrns_idx[2] = (WORD8)n;
                break;
              }
            }
          }
          else if (pstr_drc_instruction_str->drc_instructions_type == 2)
          {
            WORD32 m;
            for (m = 0;
                 m < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_ids_requested;
                 m++)
            {
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_id_requested[m] ==
                  pstr_drc_instruction_str->mae_group_id)
              {
                pstr_drc_uni_sel_proc->drc_instrns_idx[2] = (WORD8)n;
                break;
              }
            }
          }
          else if (pstr_drc_instruction_str->drc_instructions_type == 0)
          {
            pstr_drc_uni_sel_proc->drc_instrns_idx[2] = (WORD8)n;
          }
        }
        else
        {
          return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_get_ducking_drc_set
 *  \brief Get duckling DRC set
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *
 *  \return error IA_ERRORCODE if any
 *
 */
static IA_ERRORCODE impd_get_ducking_drc_set(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc)
{
  WORD32 drc_instrns_idx;
  WORD32 n, k;
  ia_drc_instructions_struct *pstr_drc_instruction_str;

  pstr_drc_uni_sel_proc->drc_instrns_idx[3] = -1;
  drc_instrns_idx = -1;
  pstr_drc_instruction_str = NULL;

  if (drc_instrns_idx == -1)
  {
    for (n = 0; n < pstr_drc_uni_sel_proc->drc_config->drc_instructions_uni_drc_count; n++)
    {
      pstr_drc_instruction_str = &(pstr_drc_uni_sel_proc->drc_config->str_drc_instruction_str[n]);

      if (pstr_drc_instruction_str->drc_set_effect &
          (IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF))
      {
        for (k = 0; k < pstr_drc_instruction_str->dwnmix_id_count; k++)
        {
          if (pstr_drc_instruction_str->downmix_id[k] == ID_FOR_BASE_LAYOUT)
          {
            if (pstr_drc_instruction_str->drc_instructions_type == 3)
            {

              WORD32 m;
              for (m = 0; m < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                                  .num_group_preset_ids_requested;
                   m++)
              {
                if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                        .group_preset_id_requested[m] ==
                    pstr_drc_instruction_str->mae_group_preset_id)
                {
                  drc_instrns_idx = n;
                  break;
                }
              }
            }
            else if (pstr_drc_instruction_str->drc_instructions_type == 2)
            {
              WORD32 m;
              for (m = 0;
                   m < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_ids_requested;
                   m++)
              {
                if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.group_id_requested[m] ==
                    pstr_drc_instruction_str->mae_group_id)
                {
                  drc_instrns_idx = n;
                  break;
                }
              }
            }
            else if (pstr_drc_instruction_str->drc_instructions_type == 0)
            {
              drc_instrns_idx = n;
            }
          }
        }
      }
    }
  }
  if (drc_instrns_idx > -1)
  {
    pstr_drc_uni_sel_proc->drc_instrns_idx[2] = -1;
    pstr_drc_uni_sel_proc->drc_instrns_idx[3] = (WORD8)drc_instrns_idx;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_sel_downmix_matrix
 *  \brief Select down mix matrix
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *  \param [in] pstr_drc_config
 *
 *  \return
 *
 */
static VOID impd_sel_downmix_matrix(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                    ia_drc_config *pstr_drc_config)
{
  WORD32 i, j, n;
  pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.target_layout = -1;
  pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.downmix_matrix_present = 0;
  pstr_drc_uni_sel_proc->downmix_inst_index_sel = -1;
  pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.base_channel_count =
      pstr_drc_config->channel_layout.base_channel_count;
  pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.target_channel_count =
      pstr_drc_config->channel_layout.base_channel_count;

  if (pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.active_downmix_id != 0)
  {
    for (n = 0; n < pstr_drc_config->ia_mpegh3da_dwnmix_instructions_count; n++)
    {
      ia_drc_downmix_instructions_struct *dwnmix_instructions =
          &(pstr_drc_config->mpegh3da_dwnmix_instructions[n]);

      if (pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.active_downmix_id ==
          dwnmix_instructions->downmix_id)
      {
        pstr_drc_uni_sel_proc->downmix_inst_index_sel = (WORD8)n;
        pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.target_channel_count =
            dwnmix_instructions->target_channel_count;
        pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.target_layout =
            dwnmix_instructions->target_layout;
        if (dwnmix_instructions->downmix_coefficients_present)
        {
          for (i = 0; i < pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.base_channel_count; i++)
          {
            for (j = 0; j < pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.target_channel_count;
                 j++)
            {
              pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.downmix_matrix[i][j] =
                  dwnmix_instructions->downmix_coefficient
                      [i + j * pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.base_channel_count];
            }
          }
          pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.downmix_matrix_present = 1;
        }
        break;
      }
    }
  }
  return;
}

/**
 *
 *	impd_drc_map_target_config_req_downmix_id
 *  \brief Mapping tarquired configuration downmix id
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *  \param [in] pstr_drc_config

 *
 *  \return error IA_ERRORCODE if any
 *
 */
static IA_ERRORCODE
impd_drc_map_target_config_req_downmix_id(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                          ia_drc_config *pstr_drc_config)
{
  WORD32 i, dwnmix_instructions_count;
  WORD32 target_ch_count_prelim =
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_channel_count;

  pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 0;
  switch (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type)
  {
  case 2:
    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_ch_count ==
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_channel_count)
    {
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0] = 0;
    }
    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests == 0)
    {
      dwnmix_instructions_count =
          pstr_drc_uni_sel_proc->drc_config->drc_dwnmix_instructions_count;
      for (i = 0; i < dwnmix_instructions_count; i++)
      {
        ia_drc_downmix_instructions_struct *dwnmix_instructions =
            &(pstr_drc_config->dwnmix_instructions[i]);
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_ch_count ==
            dwnmix_instructions->target_channel_count)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id
              [pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests] =
              dwnmix_instructions->downmix_id;
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests += 1;
          target_ch_count_prelim = dwnmix_instructions->target_channel_count;
        }
      }
    }

    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests == 0)
    {
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0] = 0;
    }
    break;

  case 1:
    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_layout ==
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_layout)
    {
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0] = 0;
    }
    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests == 0)
    {
      dwnmix_instructions_count =
          pstr_drc_uni_sel_proc->drc_config->drc_dwnmix_instructions_count;
      for (i = 0; i < dwnmix_instructions_count; i++)
      {
        ia_drc_downmix_instructions_struct *dwnmix_instructions =
            &(pstr_drc_config->dwnmix_instructions[i]);
        if (dwnmix_instructions->target_layout ==
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_layout)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id
              [pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests] =
              dwnmix_instructions->downmix_id;
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests += 1;
          target_ch_count_prelim = dwnmix_instructions->target_channel_count;
        }
      }
    }

    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests == 0)
    {
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0] = 0;
    }
    break;

  case 0:
    if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests == 0)
    {
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 1;
      pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0] = 0;
    }
    break;

  default:
    return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
    break;
  }
  pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_ch_count_prelim =
      target_ch_count_prelim;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_uni_sel_proc_process
 *  \brief Process selected uni DRC process
 *
 *  \param [in/out] pstr_drc_uni_sel_proc
 *  \param [in] pstr_drc_config
 *  \param [in] pstr_loudness_info
 *  \param [out] hia_drc_sel_proc_output_struct
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE
impd_drc_uni_sel_proc_process(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                              ia_drc_config *pstr_drc_config,
                              ia_drc_loudness_info_set_struct *pstr_loudness_info,
                              ia_drc_sel_proc_output_struct *hia_drc_sel_proc_output_struct)
{
  WORD32 i, drc_set_id_selected, activeDrcSetIndex;
  IA_ERRORCODE err;

  if (pstr_drc_config != NULL)
  {
    if (!((pstr_drc_uni_sel_proc->drc_config == NULL) ||
          (memcmp(pstr_drc_uni_sel_proc->drc_config, pstr_drc_config, sizeof(ia_drc_config)))))
    {
      pstr_drc_uni_sel_proc->drc_config_flag = 0;
    }
    else
    {
      pstr_drc_uni_sel_proc->drc_config = pstr_drc_config;
      pstr_drc_uni_sel_proc->drc_config_flag = 1;

      if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_channel_count !=
          pstr_drc_uni_sel_proc->drc_config->channel_layout.base_channel_count)
      {
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_channel_count =
            pstr_drc_uni_sel_proc->drc_config->channel_layout.base_channel_count;
      }
      if (pstr_drc_uni_sel_proc->drc_config->channel_layout.layout_signaling_present == 1 &&
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_layout !=
              pstr_drc_uni_sel_proc->drc_config->channel_layout.defined_layout)
      {
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_layout =
            pstr_drc_uni_sel_proc->drc_config->channel_layout.defined_layout;
      }
    }
  }
  if (pstr_loudness_info != NULL)
  {
    WORD32 mem_check = memcmp(&pstr_drc_uni_sel_proc->loudness_info_set, pstr_loudness_info,
                              sizeof(ia_drc_loudness_info_set_struct));
    if (mem_check == 0)
    {
      pstr_drc_uni_sel_proc->loudness_info_set_flag = 0;
    }
    else
    {
      pstr_drc_uni_sel_proc->loudness_info_set = *pstr_loudness_info;
      pstr_drc_uni_sel_proc->loudness_info_set_flag = 1;
    }
  }

  if ((pstr_drc_uni_sel_proc->drc_config_flag &&
       pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type != 0) ||
      (pstr_drc_uni_sel_proc->sel_proc_request_flag &&
       pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type != 0) ||
      (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type == 0 &&
       pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests == 0))
  {
    err = impd_drc_map_target_config_req_downmix_id(pstr_drc_uni_sel_proc,
                                                    pstr_drc_uni_sel_proc->drc_config);
    if (err)
    {
      return (err);
    }
  }

  if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0] == 0)
  {
    pstr_drc_uni_sel_proc->drc_config->dwnmix_instructions[0].downmix_id = -1;
    pstr_drc_uni_sel_proc->drc_config
        ->str_drc_instruction_str[pstr_drc_uni_sel_proc->drc_config->drc_instructions_count_plus -
                                  1]
        .downmix_id[0] = -1;
  }
  else
  {
    pstr_drc_uni_sel_proc->drc_config->dwnmix_instructions[0].downmix_id =
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0];
    pstr_drc_uni_sel_proc->drc_config
        ->str_drc_instruction_str[pstr_drc_uni_sel_proc->drc_config->drc_instructions_count_plus -
                                  1]
        .downmix_id[0] =
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[0];
  }

  if (pstr_drc_uni_sel_proc->drc_config_flag || pstr_drc_uni_sel_proc->loudness_info_set_flag ||
      pstr_drc_uni_sel_proc->sel_proc_request_flag)
  {
    err = impd_select_drc_set(pstr_drc_uni_sel_proc, &drc_set_id_selected);
    if (err)
    {
      return (err);
    }
    err = impd_get_selected_drc_set(pstr_drc_uni_sel_proc, drc_set_id_selected);
    if (err)
    {
      return (err);
    }

    err = impd_get_dependent_drc_set(pstr_drc_uni_sel_proc);
    if (err)
    {
      return (err);
    }
    err = impd_get_fading_drc_set(pstr_drc_uni_sel_proc);
    if (err)
    {
      return (err);
    }
    err = impd_get_ducking_drc_set(pstr_drc_uni_sel_proc);
    if (err)
    {
      return (err);
    }

    activeDrcSetIndex = 0;
    for (i = SUB_DRC_COUNT - 1; i >= 0; i--)
    {
      WORD32 drc_instrns_idx = pstr_drc_uni_sel_proc->drc_instrns_idx[i];
      ia_drc_instructions_struct *pstr_drc_instruction_str;
      if (drc_instrns_idx < 0)
        continue;

      pstr_drc_instruction_str =
          &(pstr_drc_uni_sel_proc->drc_config->str_drc_instruction_str[drc_instrns_idx]);

      if (pstr_drc_instruction_str->drc_set_id > 0)
      {
        pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.sel_drc_set_ids[activeDrcSetIndex] =
            pstr_drc_instruction_str->drc_set_id;

        if (!((i == 3) && (pstr_drc_instruction_str->drc_set_effect &
                           (IA_DRC_EFFECT_BIT_DUCK_SELF | IA_DRC_EFFECT_BIT_DUCK_OTHER))))
        {

          if (pstr_drc_instruction_str->drc_apply_to_dwnmix != 1)
          {
            pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.sel_downmix_ids[activeDrcSetIndex] = 0;
          }
          else
          {
            pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.sel_downmix_ids[activeDrcSetIndex] =
                pstr_drc_instruction_str->downmix_id[0];
          }
        }
        else
        {
          pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.sel_downmix_ids[activeDrcSetIndex] = 0;
        }

        activeDrcSetIndex++;
      }
    }
    if (activeDrcSetIndex > 3)
    {
      pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.num_sel_drc_sets = -1;
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
    }
    else
    {
      pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.num_sel_drc_sets = activeDrcSetIndex;
    }

    impd_sel_downmix_matrix(pstr_drc_uni_sel_proc, pstr_drc_uni_sel_proc->drc_config);

    pstr_drc_uni_sel_proc->sel_proc_request_flag = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.boost =
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.boost;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.compress =
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.compress;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.drc_characteristic_target =
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_characteristic_target;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.loud_norm_gain_db +=
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_norm_gain_modification_db;

    WORD32 m;
    ia_drc_loudness_info_struct *loudness_info = NULL;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.group_id_loudness_count = 0;
    for (i = 0; i < pstr_drc_uni_sel_proc->loudness_info_set.loudness_info_count; i++)
    {
      if ((ID_FOR_BASE_LAYOUT ==
           pstr_drc_uni_sel_proc->loudness_info_set.loudness_info[i].downmix_id) &&
          (pstr_drc_uni_sel_proc->loudness_info_set.loudness_info[i].drc_set_id == 0) &&
          (pstr_drc_uni_sel_proc->loudness_info_set.loudness_info[i].loudness_info_type == 1))
      {
        loudness_info = &pstr_drc_uni_sel_proc->loudness_info_set.loudness_info[i];
        for (m = 0; m < loudness_info->measurement_count; m++)
        {
          if ((loudness_info->loudness_measure[m].method_def ==
               IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS) ||
              (loudness_info->loudness_measure[m].method_def ==
               IA_DRC_METHOD_DEFINITION_ANCHOR_LOUDNESS))
          {
            pstr_drc_uni_sel_proc->uni_drc_sel_proc_output
                .group_id[pstr_drc_uni_sel_proc->uni_drc_sel_proc_output
                              .group_id_loudness_count] = loudness_info->mae_group_id;
            pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.group_id_loudness
                [pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.group_id_loudness_count] =

                loudness_info->loudness_measure[m].method_value;

            pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.group_id_loudness_count += 1;
            break;
          }
        }
      }
    }
  }
  *hia_drc_sel_proc_output_struct = pstr_drc_uni_sel_proc->uni_drc_sel_proc_output;

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCControlParams */