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
 *  impd_drc_sel_proc_init_dflt
 *
 *  \brief Set ia_drc_sel_pro_struct structure to default value
 *
 *  \param [in,out] pstr_drc_uni_sel_proc Pointer to drc uni selection process structure
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_drc_sel_proc_init_dflt(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc)
{

  if (pstr_drc_uni_sel_proc == NULL)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_SEL_PROCESS_INIT_FAIL;
  }
  else
  {
    pstr_drc_uni_sel_proc->first_frame = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.album_mode = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_normalization_on = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_deviation_max = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_norm_gain_modification_db = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.output_peak_level_max = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_characteristic_target = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_ids_requested = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_group_preset_ids_requested = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_drc_feature_requests = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.output_peak_level_db = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.loud_norm_gain_db = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.num_sel_drc_sets = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.active_downmix_id = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.base_channel_count = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.target_channel_count = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.downmix_matrix_present = 0;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.group_id_loudness_count = 0;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_method =
        IA_DRC_USER_METHOD_DEFINITION_DEFAULT;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_system =
        IA_DRC_USER_MEASUREMENT_SYSTEM_DEFAULT;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_pre_proc =
        IA_DRC_USER_LOUDNESS_PREPROCESSING_DEFAULT;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_channel_count = -1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.base_layout = -1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.peak_limiter = 1;
    pstr_drc_uni_sel_proc->drc_inst_index_sel = -1;
    pstr_drc_uni_sel_proc->drc_coef_index_sel = -1;
    pstr_drc_uni_sel_proc->downmix_inst_index_sel = -1;
    pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_loudness = -24.0f;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.device_cut_off_frequency = 500;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_norm_gain_db_max =
        IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.dynamic_range_control_on = 1;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_bands_supported = 4;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.boost = 1.0f;
    pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.compress = 1.0f;
    pstr_drc_uni_sel_proc->uni_drc_sel_proc_output.output_loudness = UNDEFINED_LOUDNESS_VALUE;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *  impd_drc_sel_proc_init_sel_proc_params
 *  \brief initalize selected structure parameters
 *
 *  \param [in,out] pstr_drc_uni_sel_proc structure to be initalized
 *  \param [in]  pstr_drc_sel_proc_params_struct sructure to copy the values
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE impd_drc_sel_proc_init_sel_proc_params(
    ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct)
{
  if (pstr_drc_uni_sel_proc != NULL)
  {
    if (pstr_drc_sel_proc_params_struct != NULL)
    {
      if (memcmp(&pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params,
                 pstr_drc_sel_proc_params_struct, sizeof(ia_drc_sel_proc_params_struct)))
      {
        pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
        pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params = *pstr_drc_sel_proc_params_struct;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *
 *	impd_drc_sel_proc_init_interface_params
 *
 *  \brief Initialize interface parameter
 *
 *  \param [in,out] pstr_drc_uni_sel_proc Pointer to drc uni selction process structure
 *  \param [in] pstr_drc_interface        Pointer to drc interface structure
 *
 *
 *
 */
VOID impd_drc_sel_proc_init_interface_params(ia_drc_sel_pro_struct *pstr_drc_uni_sel_proc,
                                             ia_drc_interface_struct *pstr_drc_interface)
{
  WORD32 i, j;

  if (pstr_drc_uni_sel_proc != NULL)
  {
    if (pstr_drc_interface != NULL)
    {
      if (pstr_drc_interface->loudness_norm_parameter_interface_flag)
      {
        if (pstr_drc_interface->loudness_norm_param_interface.change_device_cut_off_freq)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.device_cut_off_frequency !=
              pstr_drc_interface->loudness_norm_param_interface.device_cut_off_frequency)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.device_cut_off_frequency =
                pstr_drc_interface->loudness_norm_param_interface.device_cut_off_frequency;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface.change_loudness_deviation_max)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_deviation_max !=
              pstr_drc_interface->loudness_norm_param_interface.loudness_deviation_max)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_deviation_max =
                pstr_drc_interface->loudness_norm_param_interface.loudness_deviation_max;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface.change_loudness_measur_method)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_method !=
              pstr_drc_interface->loudness_norm_param_interface.loudness_measurement_method)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_method =
                pstr_drc_interface->loudness_norm_param_interface.loudness_measurement_method;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface.change_loudness_measur_pre_proc)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_pre_proc !=
              pstr_drc_interface->loudness_norm_param_interface.loudness_measurement_pre_proc)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_pre_proc =
                pstr_drc_interface->loudness_norm_param_interface.loudness_measurement_pre_proc;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface.change_loudness_measur_system)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_system !=
              pstr_drc_interface->loudness_norm_param_interface.loudness_measurement_system)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_measurement_system =
                pstr_drc_interface->loudness_norm_param_interface.loudness_measurement_system;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface.change_loudness_norm_gain_db_max)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_norm_gain_db_max !=
              pstr_drc_interface->loudness_norm_param_interface.loudness_norm_gain_db_max)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_norm_gain_db_max =
                pstr_drc_interface->loudness_norm_param_interface.loudness_norm_gain_db_max;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface
                .change_loudness_norm_gain_modification_db)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                  .loudness_norm_gain_modification_db !=
              pstr_drc_interface->loudness_norm_param_interface
                  .loudness_norm_gain_modification_db)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                .loudness_norm_gain_modification_db =
                pstr_drc_interface->loudness_norm_param_interface
                    .loudness_norm_gain_modification_db;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->loudness_norm_param_interface.change_output_peak_level_max)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.output_peak_level_max !=
              pstr_drc_interface->loudness_norm_param_interface.output_peak_level_max)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.output_peak_level_max =
                pstr_drc_interface->loudness_norm_param_interface.output_peak_level_max;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.peak_limiter !=
            pstr_drc_interface->loudness_norm_param_interface.peak_limiter)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.peak_limiter =
              pstr_drc_interface->loudness_norm_param_interface.peak_limiter;
          pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
        }
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.album_mode !=
            pstr_drc_interface->loudness_norm_param_interface.album_mode)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.album_mode =
              pstr_drc_interface->loudness_norm_param_interface.album_mode;
          pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
        }
      }

      if (pstr_drc_interface->loudness_norm_ctrl_interface_flag)
      {
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_normalization_on)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_loudness !=
              pstr_drc_interface->loudness_norm_ctrl_interface.target_loudness)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_loudness =
                pstr_drc_interface->loudness_norm_ctrl_interface.target_loudness;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_normalization_on !=
            pstr_drc_interface->loudness_norm_ctrl_interface.loudness_normalization_on)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.loudness_normalization_on =
              pstr_drc_interface->loudness_norm_ctrl_interface.loudness_normalization_on;
          pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
        }
      }

      if (pstr_drc_interface->drc_parameter_interface_flag)
      {
        if (pstr_drc_interface->drc_parameter_interface.change_drc_characteristic_target)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_characteristic_target !=
              pstr_drc_interface->drc_parameter_interface.drc_characteristic_target)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_characteristic_target =
                pstr_drc_interface->drc_parameter_interface.drc_characteristic_target;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->drc_parameter_interface.change_compress)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.compress !=
              pstr_drc_interface->drc_parameter_interface.compress)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.compress =
                pstr_drc_interface->drc_parameter_interface.compress;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
        if (pstr_drc_interface->drc_parameter_interface.change_boost)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.boost !=
              pstr_drc_interface->drc_parameter_interface.boost)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.boost =
                pstr_drc_interface->drc_parameter_interface.boost;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
        }
      }

      if (pstr_drc_interface->drc_interface_flag)
      {
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.dynamic_range_control_on)
        {
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_drc_feature_requests !=
              pstr_drc_interface->drc_ctrl_interface.num_drc_feature_requests)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_drc_feature_requests =
                pstr_drc_interface->drc_ctrl_interface.num_drc_feature_requests;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
          for (i = 0;
               i < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_drc_feature_requests;
               i++)
          {
            if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_feature_req_type[i] !=
                pstr_drc_interface->drc_ctrl_interface.drc_feature_req_type[i])
            {
              pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_feature_req_type[i] =
                  pstr_drc_interface->drc_ctrl_interface.drc_feature_req_type[i];
              pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
            }
            switch (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.drc_feature_req_type[i])
            {
            case 0:
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_num_drc_effects[i] !=
                  pstr_drc_interface->drc_ctrl_interface.requested_num_drc_effects[i])
              {
                pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_num_drc_effects[i] =
                    pstr_drc_interface->drc_ctrl_interface.requested_num_drc_effects[i];
                pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
              }
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .desired_num_drc_effects_of_requested[i] !=
                  pstr_drc_interface->drc_ctrl_interface.desired_num_drc_effects_of_requested[i])
              {
                pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                    .desired_num_drc_effects_of_requested[i] =
                    pstr_drc_interface->drc_ctrl_interface
                        .desired_num_drc_effects_of_requested[i];
                pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
              }
              for (j = 0; j < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                                  .requested_num_drc_effects[i];
                   j++)
              {
                if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                        .requested_drc_effect_type[i][j] !=
                    pstr_drc_interface->drc_ctrl_interface.requested_drc_effect_type[i][j])
                {
                  pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_drc_effect_type[i][j] =
                      pstr_drc_interface->drc_ctrl_interface.requested_drc_effect_type[i][j];
                  pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
                }
              }
              break;
            case 1:
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_dyn_range_measur_type[i] !=
                  pstr_drc_interface->drc_ctrl_interface.requested_dyn_rng_measurement_type[i])
              {
                pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                    .requested_dyn_range_measur_type[i] =
                    pstr_drc_interface->drc_ctrl_interface.requested_dyn_rng_measurement_type[i];
                pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
              }
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_dyn_range_range_flag[i] !=
                  pstr_drc_interface->drc_ctrl_interface
                      .requested_dyn_range_is_single_val_flag[i])
              {
                pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                    .requested_dyn_range_range_flag[i] =
                    pstr_drc_interface->drc_ctrl_interface
                        .requested_dyn_range_is_single_val_flag[i];
                pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
              }
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_dyn_range_range_flag[i] == 0)
              {
                if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                        .requested_dyn_range_value[i] !=
                    pstr_drc_interface->drc_ctrl_interface.requested_dyn_range_value[i])
                {
                  pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_dyn_range_value[i] =
                      pstr_drc_interface->drc_ctrl_interface.requested_dyn_range_value[i];
                  pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
                }
              }
              else
              {
                if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                        .requested_dyn_range_min_val[i] !=
                    pstr_drc_interface->drc_ctrl_interface.requested_dyn_range_min_val[i])
                {
                  pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_dyn_range_min_val[i] =
                      pstr_drc_interface->drc_ctrl_interface.requested_dyn_range_min_val[i];
                  pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
                }
                if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                        .requested_dyn_range_max_val[i] !=
                    pstr_drc_interface->drc_ctrl_interface.requested_dyn_range_max_val[i])
                {
                  pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_dyn_range_max_val[i] =
                      pstr_drc_interface->drc_ctrl_interface.requested_dyn_range_max_val[i];
                  pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
                }
              }
              break;
            case 2:
              if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                      .requested_drc_characteristic[i] !=
                  pstr_drc_interface->drc_ctrl_interface.requested_drc_characteristic[i])
              {
                pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params
                    .requested_drc_characteristic[i] =
                    pstr_drc_interface->drc_ctrl_interface.requested_drc_characteristic[i];
                pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
              }
              break;
            }
          }
        }
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.dynamic_range_control_on !=
            pstr_drc_interface->drc_ctrl_interface.dynamic_range_control_on)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.dynamic_range_control_on =
              pstr_drc_interface->drc_ctrl_interface.dynamic_range_control_on;
          pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          if (!pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.dynamic_range_control_on)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_drc_feature_requests = 0;
          }
        }
      }

      if (pstr_drc_interface->system_interface_flag)
      {
        if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type !=
            pstr_drc_interface->system_interface.target_config_request_type)
        {
          pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type =
              pstr_drc_interface->system_interface.target_config_request_type;
          pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
        }
        switch (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.target_config_request_type)
        {
        case 0:
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests !=
              pstr_drc_interface->system_interface.num_downmix_id_requests)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests =
                pstr_drc_interface->system_interface.num_downmix_id_requests;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
          for (i = 0;
               i < pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.num_downmix_id_requests;
               i++)
          {
            if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[i] !=
                pstr_drc_interface->system_interface.requested_dwnmix_id[i])
            {
              pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_dwnmix_id[i] =
                  pstr_drc_interface->system_interface.requested_dwnmix_id[i];
              pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
            }
          }
          break;
        case 1:
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_layout !=
              pstr_drc_interface->system_interface.requested_target_layout)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_layout =
                pstr_drc_interface->system_interface.requested_target_layout;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
          break;
        case 2:
          if (pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_ch_count !=
              pstr_drc_interface->system_interface.requested_target_ch_count)
          {
            pstr_drc_uni_sel_proc->str_uni_drc_sel_proc_params.requested_target_ch_count =
                pstr_drc_interface->system_interface.requested_target_ch_count;
            pstr_drc_uni_sel_proc->sel_proc_request_flag = 1;
          }
          break;
        }
      }
    }
  }

  return;
}
/** @} */ /* End of DRCControlParams */