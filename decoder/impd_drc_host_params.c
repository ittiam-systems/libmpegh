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
#include "impeghd_error_codes.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_interface.h"
#include "impd_drc_struct.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_rom.h"
#include "impd_drc_selection_process.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup DRCProcessing DRCProcessing
 * @ingroup  DRCProcessing
 * @brief DRC Decoder processing
 *
 * @{
 */

/**
 *  impd_drc_set_def_params_sel_process
 *
 *  \brief Set default params in selection process params structure
 *
 *  \param [out] pstr_drc_sel_proc_params  Pointer to DRC selection process params structure
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE
impd_drc_set_def_params_sel_process(ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params)
{
  pstr_drc_sel_proc_params->album_mode = 0;
  pstr_drc_sel_proc_params->base_channel_count = -1;
  pstr_drc_sel_proc_params->base_layout = -1;
  pstr_drc_sel_proc_params->boost = 1.0f;
  pstr_drc_sel_proc_params->compress = 1.0f;
  pstr_drc_sel_proc_params->device_cut_off_frequency = 500;
  pstr_drc_sel_proc_params->drc_characteristic_target = 0;
  pstr_drc_sel_proc_params->dynamic_range_control_on = 0;

  pstr_drc_sel_proc_params->loudness_deviation_max = 0;

  pstr_drc_sel_proc_params->loudness_measurement_method = IA_DRC_USER_METHOD_DEFINITION_DEFAULT;
  pstr_drc_sel_proc_params->loudness_measurement_system = IA_DRC_USER_MEASUREMENT_SYSTEM_DEFAULT;
  pstr_drc_sel_proc_params->loudness_measurement_pre_proc =
      IA_DRC_USER_LOUDNESS_PREPROCESSING_DEFAULT;
  pstr_drc_sel_proc_params->loudness_norm_gain_db_max =
      IA_DRC_LOUDNESS_NORMALIZATION_GAIN_MAX_DEFAULT;
  pstr_drc_sel_proc_params->loudness_norm_gain_modification_db = 0;
  pstr_drc_sel_proc_params->loudness_normalization_on = 0;

  pstr_drc_sel_proc_params->num_bands_supported = 4;
  pstr_drc_sel_proc_params->num_downmix_id_requests = 0;
  pstr_drc_sel_proc_params->num_drc_feature_requests = 0;
  pstr_drc_sel_proc_params->num_group_ids_requested = 0;
  pstr_drc_sel_proc_params->num_group_preset_ids_requested = 0;
  pstr_drc_sel_proc_params->output_peak_level_max = 0;
  pstr_drc_sel_proc_params->peak_limiter = 1;
  pstr_drc_sel_proc_params->target_config_request_type = 0;
  pstr_drc_sel_proc_params->target_loudness = -24.0f;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_set_custom_params
 *
 *  \brief Set custom params in selection process params structure
 *
 *  \param [out] pstr_drc_sel_proc_params  Pointer to DRC selection process params structure
 *  \param [in] control_parameter_index  control parameter index
 *
 *  \return IA_ERRORCODE Error
 *
 */
IA_ERRORCODE impd_drc_set_custom_params(const WORD32 control_parameter_index,
                                        ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params)
{
  WORD32 num_requests, num_effects;

  const ia_drc_loc_drc_interface_struct *drc_ctrl_interface =
      &(ia_drc_loc_dyn_range_ctrl_interface[control_parameter_index - 1]);
  const ia_drc_loc_drc_parameter_interface_struct *drc_parameter_interface =
      &(ia_drc_loc_drc_parameter_interface[control_parameter_index - 1]);
  const ia_drc_loc_loudness_norm_ctrl_interface_struct *loudness_norm_ctrl_interface =
      &(ia_drc_loc_loudness_norm_ctrl_interface[control_parameter_index - 1]);
  const ia_drc_loc_loudness_norm_param_interface_struct *loudness_norm_param_interface =
      &(ia_drc_loc_loudness_norm_param_interface[control_parameter_index - 1]);
  const ia_drc_loc_mpegh_parameter_interface_struct *mpegh_parameter_interface =
      &(ia_drc_loc_mpegh_parameter_interface[control_parameter_index - 1]);
  const ia_drc_loc_requested_drc_effect_struct *requested_drc_effect_type =
      &(ia_drc_loc_requested_drc_effect_type_str[control_parameter_index - 1]);
  const ia_drc_loc_sys_interface_struct *system_interface =
      &(ia_drc_loc_sys_interface[control_parameter_index - 1]);

  pstr_drc_sel_proc_params->album_mode = loudness_norm_param_interface->album_mode;
  pstr_drc_sel_proc_params->boost = drc_parameter_interface->boost;
  pstr_drc_sel_proc_params->compress = drc_parameter_interface->compress;
  pstr_drc_sel_proc_params->device_cut_off_frequency =
      loudness_norm_param_interface->device_cut_off_frequency;
  pstr_drc_sel_proc_params->drc_characteristic_target =
      drc_parameter_interface->drc_characteristic_target;
  pstr_drc_sel_proc_params->dynamic_range_control_on =
      drc_ctrl_interface->dynamic_range_control_on;
  pstr_drc_sel_proc_params->group_preset_id_requested_preference =
      mpegh_parameter_interface->group_preset_id_requested_preference;

  pstr_drc_sel_proc_params->loudness_deviation_max =
      loudness_norm_param_interface->loudness_deviation_max;
  pstr_drc_sel_proc_params->loudness_deviation_max =
      ia_max_int(0, pstr_drc_sel_proc_params->loudness_deviation_max);
  pstr_drc_sel_proc_params->loudness_measurement_method =
      loudness_norm_param_interface->loudness_measurement_method;
  pstr_drc_sel_proc_params->loudness_measurement_system =
      loudness_norm_param_interface->loudness_measurement_system;
  pstr_drc_sel_proc_params->loudness_measurement_pre_proc =
      loudness_norm_param_interface->loudness_measurement_pre_proc;
  pstr_drc_sel_proc_params->loudness_normalization_on =
      loudness_norm_ctrl_interface->loudness_normalization_on;
  pstr_drc_sel_proc_params->loudness_norm_gain_db_max =
      loudness_norm_param_interface->loudness_norm_gain_db_max;
  pstr_drc_sel_proc_params->loudness_norm_gain_db_max =
      ia_max_flt(0, pstr_drc_sel_proc_params->loudness_norm_gain_db_max);
  pstr_drc_sel_proc_params->loudness_norm_gain_modification_db =
      loudness_norm_param_interface->loudness_norm_gain_modification_db;

  pstr_drc_sel_proc_params->num_drc_feature_requests =
      drc_ctrl_interface->num_drc_feature_requests;
  pstr_drc_sel_proc_params->num_group_ids_requested =
      mpegh_parameter_interface->num_group_ids_requested;
  pstr_drc_sel_proc_params->num_group_preset_ids_requested =
      mpegh_parameter_interface->num_group_preset_ids_requested;
  pstr_drc_sel_proc_params->output_peak_level_max =
      loudness_norm_param_interface->output_peak_level_max;
  pstr_drc_sel_proc_params->peak_limiter = loudness_norm_param_interface->peak_limiter;
  pstr_drc_sel_proc_params->target_loudness = loudness_norm_ctrl_interface->target_loudness;

  pstr_drc_sel_proc_params->target_config_request_type =
      system_interface->target_config_request_type;
  switch (system_interface->target_config_request_type)
  {
  case 2:
    pstr_drc_sel_proc_params->requested_target_ch_count =
        system_interface->requested_target_ch_count;
    break;
  case 1:
    pstr_drc_sel_proc_params->requested_target_layout = system_interface->requested_target_layout;
    break;
  case 0:
  default:
    pstr_drc_sel_proc_params->num_downmix_id_requests = system_interface->num_downmix_id_requests;
    for (num_requests = system_interface->num_downmix_id_requests - 1; num_requests >= 0;
         num_requests--)
    {
      pstr_drc_sel_proc_params->requested_dwnmix_id[num_requests] =
          system_interface->requested_dwnmix_id[num_requests];
    }
    break;
  }

  for (num_requests = mpegh_parameter_interface->num_group_ids_requested - 1; num_requests >= 0;
       num_requests--)
  {
    pstr_drc_sel_proc_params->group_id_requested[num_requests] =
        mpegh_parameter_interface->group_id_requested[num_requests];
  }

  for (num_requests = mpegh_parameter_interface->num_group_preset_ids_requested - 1;
       num_requests >= 0; num_requests--)
  {
    pstr_drc_sel_proc_params->group_preset_id_requested[num_requests] =
        mpegh_parameter_interface->group_preset_id_requested[num_requests];
    pstr_drc_sel_proc_params->num_members_group_preset_ids_requested[num_requests] =
        mpegh_parameter_interface->num_members_group_preset_ids_requested[num_requests];
  }

  for (num_requests = drc_ctrl_interface->num_drc_feature_requests - 1; num_requests >= 0;
       num_requests--)
  {
    pstr_drc_sel_proc_params->drc_feature_req_type[num_requests] =
        drc_ctrl_interface->drc_feature_req_type[num_requests];
    switch (drc_ctrl_interface->drc_feature_req_type[num_requests])
    {
    case MATCH_DRC_CHARACTERISTIC:
      pstr_drc_sel_proc_params->requested_drc_characteristic[num_requests] =
          drc_ctrl_interface->requested_drc_characteristic;
      break;
    case MATCH_DYNAMIC_RANGE:
      pstr_drc_sel_proc_params->requested_dyn_range_measur_type[num_requests] =
          drc_ctrl_interface->requested_dyn_rng_measurement_type;
      pstr_drc_sel_proc_params->requested_dyn_range_range_flag[num_requests] =
          drc_ctrl_interface->requested_dyn_range_is_single_val_flag;
      pstr_drc_sel_proc_params->requested_dyn_range_value[num_requests] =
          drc_ctrl_interface->requested_dyn_range_value;
      pstr_drc_sel_proc_params->requested_dyn_range_min_val[num_requests] =
          drc_ctrl_interface->requested_dyn_range_min_val;
      pstr_drc_sel_proc_params->requested_dyn_range_max_val[num_requests] =
          drc_ctrl_interface->requested_dyn_range_max_val;
      break;
    case MATCH_EFFECT_TYPE:
      pstr_drc_sel_proc_params->requested_num_drc_effects[num_requests] =
          requested_drc_effect_type->requested_num_drc_effects;
      pstr_drc_sel_proc_params->desired_num_drc_effects_of_requested[num_requests] =
          requested_drc_effect_type->desired_num_drc_effects_of_requested;
      for (num_effects = requested_drc_effect_type->requested_num_drc_effects - 1;
           num_effects >= 0; num_effects--)
      {
        pstr_drc_sel_proc_params->requested_drc_effect_type[num_requests][num_effects] =
            requested_drc_effect_type->requested_drc_effect_type[num_effects];
      }
      break;
    default:
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCProcessing */