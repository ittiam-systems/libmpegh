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
 *  impd_drc_check_if_match
 *
 *  \brief Function to check if match found
 *
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_ia_loudness_info   Pointer to loudness info structure
 *
 *  \return WORD32 match found flag
 *
 */
static WORD32 impd_drc_check_if_match(ia_drc_instructions_struct *pstr_drc_instruction,
                                      ia_drc_loudness_info_struct *pstr_ia_loudness_info)
{
  WORD32 found_match = 0;
  if ((3 == pstr_drc_instruction->drc_instructions_type &&
       3 == pstr_ia_loudness_info->loudness_info_type &&
       pstr_drc_instruction->mae_group_preset_id == pstr_ia_loudness_info->mae_group_preset_id) ||
      (2 == pstr_drc_instruction->drc_instructions_type &&
       2 == pstr_ia_loudness_info->loudness_info_type &&
       pstr_drc_instruction->mae_group_id == pstr_ia_loudness_info->mae_group_id) ||
      (0 == pstr_drc_instruction->drc_instructions_type &&
       0 == pstr_ia_loudness_info->loudness_info_type))
  {
    found_match = 1;
  }
  return found_match;
}

/**
 *  impd_drc_signal_peak_level_info
 *
 *  \brief Function to get peak level info
 *
 *  \param [out]  peak_info_count   peakinfo count
 *  \param [out]  eq_set_id   drc set id
 *  \param [out]  sig_pk_level   signal peak level
 *  \param [out]  explicit_peak_info_present   peak info present
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  pstr_loudness_info   Pointer to loudness info structure
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  album_mode   album mode
 *  \param [in]  num_compression_eq_count   no of compression count
 *  \param [in]  num_compression_eq_id   compression id
 *
 *  \return VOID
 *
 */
VOID impd_drc_signal_peak_level_info(WORD32 *peak_info_count, WORD32 eq_set_id[],
                                     FLOAT32 sig_pk_level[], WORD32 explicit_peak_info_present[],
                                     ia_drc_config *pstr_drc_config,
                                     ia_drc_loudness_info_set_struct *pstr_loudness_info,
                                     ia_drc_instructions_struct *pstr_drc_instruction,
                                     WORD32 req_dwnmix_id, WORD32 album_mode,
                                     WORD32 num_compression_eq_count,
                                     WORD32 *num_compression_eq_id)
{
  ia_drc_loudness_info_struct *pstr_ia_loudness_info;
  WORD32 cnt1, cnt2, cnt3, mode;
  WORD32 pre_lim_cnt;
  WORD32 peak_cnt = 0;
  WORD32 loudness_info_cnt = 0;
  WORD32 drc_set_id_req = pstr_drc_instruction->drc_set_id;
  WORD32 loudness_drc_set_id_req;
  WORD32 found_match = 0;
  FLOAT32 signal_peak_level_temp;

  eq_set_id[0] = 0;
  sig_pk_level[0] = 0;
  explicit_peak_info_present[0] = 0;

  cnt2 = 0;
  if (0 > drc_set_id_req)
  {
    for (cnt2 = 0; cnt2 < num_compression_eq_count; cnt2++)
    {
      eq_set_id[cnt2] = num_compression_eq_id[cnt2];
      sig_pk_level[cnt2] = 0;
      explicit_peak_info_present[cnt2] = 0;
    }
  }
  eq_set_id[cnt2] = 0;
  sig_pk_level[cnt2] = 0;
  explicit_peak_info_present[cnt2] = 0;
  cnt2++;

  pre_lim_cnt = cnt2;

  if (0 <= drc_set_id_req)
  {
    loudness_drc_set_id_req = drc_set_id_req;
  }
  else
  {
    loudness_drc_set_id_req = 0;
  }

  if (album_mode != 1)
  {
    mode = 0;
    loudness_info_cnt = pstr_loudness_info->loudness_info_count;
  }
  else
  {
    mode = 1;
    loudness_info_cnt = pstr_loudness_info->loudness_info_album_count;
  }

  for (cnt3 = 0; cnt3 < loudness_info_cnt; cnt3++)
  {
    if (mode != 1)
    {
      pstr_ia_loudness_info = &(pstr_loudness_info->loudness_info[cnt3]);
    }
    else
    {
      pstr_ia_loudness_info = &(pstr_loudness_info->str_loudness_info_album[cnt3]);
    }

    if (req_dwnmix_id == pstr_ia_loudness_info->downmix_id &&
        loudness_drc_set_id_req == pstr_ia_loudness_info->drc_set_id)
    {
      if (pstr_ia_loudness_info->true_peak_level_present)
      {
        eq_set_id[peak_cnt] = 0;
        sig_pk_level[peak_cnt] = pstr_ia_loudness_info->true_peak_level;
        explicit_peak_info_present[peak_cnt] = 1;

        if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
        {
          found_match = 1;
          peak_cnt++;
        }
      }
      if (0 == found_match)
      {
        if (pstr_ia_loudness_info->sample_peak_level_present)
        {
          eq_set_id[peak_cnt] = 0;

          sig_pk_level[peak_cnt] = pstr_ia_loudness_info->sample_peak_level;

          explicit_peak_info_present[peak_cnt] = 1;

          if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
          {
            found_match = 1;
            peak_cnt++;
          }
        }
      }
    }
  }
  if (0 == found_match)
  {
    for (cnt3 = 0; cnt3 < loudness_info_cnt; cnt3++)
    {
      if (mode != 1)
      {
        pstr_ia_loudness_info = &(pstr_loudness_info->loudness_info[cnt3]);
      }
      else
      {
        pstr_ia_loudness_info = &(pstr_loudness_info->str_loudness_info_album[cnt3]);
      }

      if (ID_FOR_ANY_DRC == pstr_ia_loudness_info->drc_set_id &&
          req_dwnmix_id == pstr_ia_loudness_info->downmix_id)
      {
        if (pstr_ia_loudness_info->true_peak_level_present)
        {
          eq_set_id[peak_cnt] = 0;

          sig_pk_level[peak_cnt] = pstr_ia_loudness_info->true_peak_level;

          explicit_peak_info_present[peak_cnt] = 1;

          if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
          {
            found_match = 1;
            peak_cnt++;
          }
        }
        if (0 == found_match)
        {
          if (pstr_ia_loudness_info->sample_peak_level_present)
          {
            eq_set_id[peak_cnt] = 0;

            sig_pk_level[peak_cnt] = pstr_ia_loudness_info->sample_peak_level;

            explicit_peak_info_present[peak_cnt] = 1;
            if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
            {
              found_match = 1;
              peak_cnt++;
            }
          }
        }
      }
    }
  }
  if (0 == found_match)
  {
    for (cnt1 = 0; cnt1 < pstr_drc_instruction->dwnmix_id_count; cnt1++)
    {
      if ((req_dwnmix_id == pstr_drc_instruction->downmix_id[0] ||
           ID_FOR_ANY_DOWNMIX == pstr_drc_instruction->downmix_id[0]) &&
          (pstr_drc_instruction->limiter_peak_target_present))
      {
        eq_set_id[peak_cnt] = 0;
        sig_pk_level[peak_cnt] = pstr_drc_instruction->limiter_peak_target;
        explicit_peak_info_present[peak_cnt] = 1;
        found_match = 1;
        peak_cnt++;
      }
    }
  }
  if (0 == found_match)
  {
    for (cnt1 = 1; cnt1 < pstr_drc_instruction->dwnmix_id_count; cnt1++)
    {
      if ((req_dwnmix_id == pstr_drc_instruction->downmix_id[cnt1]) &&
          (pstr_drc_instruction->limiter_peak_target_present))
      {
        {
          eq_set_id[peak_cnt] = 0;
          sig_pk_level[peak_cnt] = pstr_drc_instruction->limiter_peak_target;
          explicit_peak_info_present[peak_cnt] = 1;
          found_match = 1;
          peak_cnt++;
        }
      }
    }
  }
  if (0 == found_match)
  {
    if (ID_FOR_BASE_LAYOUT != req_dwnmix_id)
    {
      signal_peak_level_temp = 0;
      for (cnt3 = 0; cnt3 < loudness_info_cnt; cnt3++)
      {
        if (mode != 1)
        {
          pstr_ia_loudness_info = &(pstr_loudness_info->loudness_info[cnt3]);
        }
        else
        {
          pstr_ia_loudness_info = &(pstr_loudness_info->str_loudness_info_album[cnt3]);
        }

        if (loudness_drc_set_id_req == pstr_ia_loudness_info->drc_set_id &&
            ID_FOR_BASE_LAYOUT == pstr_ia_loudness_info->downmix_id)
        {
          if (pstr_ia_loudness_info->true_peak_level_present)
          {
            eq_set_id[peak_cnt] = 0;
            sig_pk_level[peak_cnt] =
                pstr_ia_loudness_info->true_peak_level + signal_peak_level_temp;
            explicit_peak_info_present[peak_cnt] = 0;

            if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
            {
              found_match = 1;
              peak_cnt++;
            }
          }
          if (0 == found_match)
          {
            if (pstr_ia_loudness_info->sample_peak_level_present)
            {
              eq_set_id[peak_cnt] = 0;
              sig_pk_level[peak_cnt] =
                  pstr_ia_loudness_info->sample_peak_level + signal_peak_level_temp;
              explicit_peak_info_present[peak_cnt] = 0;

              if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
              {
                found_match = 1;
                peak_cnt++;
              }
            }
          }
        }
      }
      if (0 == found_match)
      {
        for (cnt3 = 0; cnt3 < loudness_info_cnt; cnt3++)
        {
          if (mode != 1)
          {
            pstr_ia_loudness_info = &(pstr_loudness_info->loudness_info[cnt3]);
          }
          else
          {
            pstr_ia_loudness_info = &(pstr_loudness_info->str_loudness_info_album[cnt3]);
          }

          if (ID_FOR_ANY_DRC == pstr_ia_loudness_info->drc_set_id &&
              ID_FOR_BASE_LAYOUT == pstr_ia_loudness_info->downmix_id)
          {
            if (pstr_ia_loudness_info->true_peak_level_present)
            {
              eq_set_id[peak_cnt] = 0;
              sig_pk_level[peak_cnt] =
                  pstr_ia_loudness_info->true_peak_level + signal_peak_level_temp;
              explicit_peak_info_present[peak_cnt] = 0;

              if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
              {
                found_match = 1;
                peak_cnt++;
              }
            }
            if ((0 == found_match) && (pstr_ia_loudness_info->sample_peak_level_present))
            {
              eq_set_id[peak_cnt] = 0;

              sig_pk_level[peak_cnt] =
                  pstr_ia_loudness_info->sample_peak_level + signal_peak_level_temp;

              explicit_peak_info_present[peak_cnt] = 0;

              if (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info))
              {
                found_match = 1;
                peak_cnt++;
              }
            }
          }
        }
      }
      if (0 == found_match)
      {
        ia_drc_instructions_struct *drc_instructions_drc_temp;
        for (cnt3 = 0; cnt3 < pstr_drc_config->drc_instructions_count_plus; cnt3++)
        {
          drc_instructions_drc_temp = &pstr_drc_config->str_drc_instruction_str[cnt3];
          if ((loudness_drc_set_id_req == drc_instructions_drc_temp->drc_set_id) &&
              (ID_FOR_BASE_LAYOUT == drc_instructions_drc_temp->downmix_id[0]) &&
              (drc_instructions_drc_temp->limiter_peak_target_present))
          {
            eq_set_id[peak_cnt] = -1;
            sig_pk_level[peak_cnt] =
                drc_instructions_drc_temp->limiter_peak_target + signal_peak_level_temp;
            explicit_peak_info_present[peak_cnt] = 0;
            found_match = 1;
            peak_cnt++;
          }
        }
      }
    }
  }
  (void)found_match;
  if (peak_cnt <= 0)
  {
    *peak_info_count = pre_lim_cnt;
  }
  else
  {
    *peak_info_count = peak_cnt;
  }
  return;
}

/**
 *  impd_drc_extract_loudness_peak_to_avg_info
 *
 *  \brief Function to extract loudness peak to average information
 *
 *  \param [out]  loudness_pk_2_avg_val   loudness peak to average value
 *  \param [out]  loudness_pk_2_avg_val_flag   flag to update if loudness peak to average value
 * present
 *  \param [in]  pstr_loudness_info   Pointer to loudness info structure
 *  \param [in]  dyn_range_measure_type   dynamic range measurement type
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_extract_loudness_peak_to_avg_info(
    FLOAT32 *loudness_pk_2_avg_val, WORD32 *loudness_pk_2_avg_val_flag,
    ia_drc_loudness_info_struct *pstr_ia_loudness_info, WORD32 dyn_range_measure_type)
{
  ia_drc_loudness_measure_struct *pstr_loudness_measure = NULL;
  FLOAT32 prog_loudness = 0.0f;
  FLOAT32 pk_loudness = 0.0f;
  WORD32 prog_loudness_flag = 0;
  WORD32 pk_loudness_flag = 0;
  WORD32 match_measure_prog_loudness = 0;
  WORD32 match_measure_pk_loudness = 0;

  for (WORD32 cnt = 0; cnt < pstr_ia_loudness_info->measurement_count; cnt++)
  {
    pstr_loudness_measure = &(pstr_ia_loudness_info->loudness_measure[cnt]);
    if ((IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS == pstr_loudness_measure->method_def) &&
        (ia_drc_measurement_method_prog_loudness_tbl[pstr_loudness_measure->measurement_system] >
         match_measure_prog_loudness))
    {
      match_measure_prog_loudness =
          ia_drc_measurement_method_prog_loudness_tbl[pstr_loudness_measure->measurement_system];
      prog_loudness = (FLOAT32)pstr_loudness_measure->method_value;
      prog_loudness_flag = 1;
    }

    switch (dyn_range_measure_type)
    {
    case IA_DRC_TOP_OF_LOUDNESS_RANGE_TO_AVG:
    {
      if ((pstr_loudness_measure->method_def == IA_DRC_METHOD_DEFINITION_MAX_OF_LOUDNESS_RANGE) &&
          (ia_drc_measurement_method_peak_loudness_tbl[pstr_loudness_measure
                                                           ->measurement_system] >
           match_measure_pk_loudness))
      {
        match_measure_pk_loudness =
            ia_drc_measurement_method_peak_loudness_tbl[pstr_loudness_measure
                                                            ->measurement_system];
        pk_loudness = (FLOAT32)pstr_loudness_measure->method_value;
        pk_loudness_flag = 1;
      }
      break;
    }
    case IA_DRC_MOMENTARY_LOUDNESS_TO_AVG:
    {
      if ((pstr_loudness_measure->method_def ==
           IA_DRC_METHOD_DEFINITION_MOMENTARY_LOUDNESS_MAX) &&
          (ia_drc_measurement_method_peak_loudness_tbl[pstr_loudness_measure
                                                           ->measurement_system] >
           match_measure_pk_loudness))
      {
        match_measure_pk_loudness =
            ia_drc_measurement_method_peak_loudness_tbl[pstr_loudness_measure
                                                            ->measurement_system];
        pk_loudness = (FLOAT32)pstr_loudness_measure->method_value;
        pk_loudness_flag = 1;
      }
      break;
    }
    case IA_DRC_SHORT_TERM_LOUDNESS_TO_AVG:
    {
      if ((pstr_loudness_measure->method_def ==
           IA_DRC_METHOD_DEFINITION_SHORT_TERM_LOUDNESS_MAX) &&
          (ia_drc_measurement_method_peak_loudness_tbl[pstr_loudness_measure
                                                           ->measurement_system] >
           match_measure_pk_loudness))
      {
        match_measure_pk_loudness =
            ia_drc_measurement_method_peak_loudness_tbl[pstr_loudness_measure
                                                            ->measurement_system];
        pk_loudness = (FLOAT32)pstr_loudness_measure->method_value;
        pk_loudness_flag = 1;
      }
    }
    break;
    default:
      return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
      break;
    }
  }
  if ((1 == pk_loudness_flag) && (1 == prog_loudness_flag))
  {
    *loudness_pk_2_avg_val_flag = 1;
    *loudness_pk_2_avg_val = pk_loudness - prog_loudness;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
 *  impd_drc_loudness_pk_to_avg_info
 *
 *  \brief Function to update loudness peak to average information
 *
 *  \param [out]  loudness_pk_2_avg_val   loudness peak to average value
 *  \param [out]  loudness_pk_2_avg_val_flag   flag to update if loudness peak to average value
 * present
 *  \param [in]  pstr_loudness_info   Pointer to loudness info structure
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  dyn_range_measure_type   dynamic range measurement type
 *  \param [in]  album_mode   album mode
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_loudness_pk_to_avg_info(FLOAT32 *loudness_pk_2_avg_val,
                                              WORD32 *loudness_pk_2_avg_val_flag,
                                              ia_drc_loudness_info_set_struct *pstr_loudness_info,
                                              ia_drc_instructions_struct *pstr_drc_instruction,
                                              WORD32 req_dwnmix_id, WORD32 dyn_range_measure_type,
                                              WORD32 album_mode)
{
  IA_ERRORCODE err_code;
  WORD32 cnt;
  WORD32 drc_set_id = ia_max_int(pstr_drc_instruction->drc_set_id, 0);

  *loudness_pk_2_avg_val_flag = 0;

  if (1 == album_mode)
  {
    for (cnt = pstr_loudness_info->loudness_info_album_count - 1; cnt >= 0; cnt--)
    {
      ia_drc_loudness_info_struct *pstr_ia_loudness_info =
          &(pstr_loudness_info->str_loudness_info_album[cnt]);
      if ((drc_set_id == pstr_ia_loudness_info->drc_set_id) &&
          (req_dwnmix_id == pstr_ia_loudness_info->downmix_id) &&
          (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info)))
      {
        err_code = impd_drc_extract_loudness_peak_to_avg_info(
            loudness_pk_2_avg_val, loudness_pk_2_avg_val_flag, pstr_ia_loudness_info,
            dyn_range_measure_type);
        if (err_code)
          return (err_code);
      }
    }
  }
  if (0 == *loudness_pk_2_avg_val_flag)
  {
    for (cnt = pstr_loudness_info->loudness_info_count - 1; cnt >= 0; cnt--)
    {
      ia_drc_loudness_info_struct *pstr_ia_loudness_info =
          &(pstr_loudness_info->loudness_info[cnt]);
      if ((drc_set_id == pstr_ia_loudness_info->drc_set_id) &&
          (req_dwnmix_id == pstr_ia_loudness_info->downmix_id) &&
          (impd_drc_check_if_match(pstr_drc_instruction, pstr_ia_loudness_info)))
      {
        err_code = impd_drc_extract_loudness_peak_to_avg_info(
            loudness_pk_2_avg_val, loudness_pk_2_avg_val_flag, pstr_ia_loudness_info,
            dyn_range_measure_type);
        if (err_code)
          return (err_code);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_chk_loudness_info_present
 *
 *  \brief Function to check if loudness information present
 *
 *  \param [out]  loudness_info_flag   flag to update if loudness info is present
 *  \param [in]  pstr_loudness_info   Pointer to loudness info structure
 *
 *  \return WORD32 loudness_info_flag
 *
 */
static WORD32
impd_drc_chk_loudness_info_present(ia_drc_loudness_info_struct *pstr_ia_loudness_info)
{
  WORD32 loudness_info_flag = 0;
  for (WORD32 cnt = 0; cnt < pstr_ia_loudness_info->measurement_count; cnt++)
  {
    if ((IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS ==
         pstr_ia_loudness_info->loudness_measure[cnt].method_def) ||
        (IA_DRC_METHOD_DEFINITION_ANCHOR_LOUDNESS ==
         pstr_ia_loudness_info->loudness_measure[cnt].method_def))
    {
      loudness_info_flag = 1;
    }
  }
  return loudness_info_flag;
}

/**
 *  impd_drc_check_loudness_info
 *
 *  \brief Function to check loudness information
 *
 *  \param [out]  loudness_info_matching   Pointer to loudness info structure
 *  \param [out]  info_cnt   info count
 *  \param [in]  pstr_ia_loudness_info   Pointer to loudness info structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  drc_set_id_req   drc set id requested
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_drc_sel_proc_params_struct   Pointer to drc selection process params
 * structure
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_check_loudness_info(
    ia_drc_loudness_info_struct *loudness_info_matching[], WORD32 *info_cnt,
    WORD32 loudness_info_cnt, ia_drc_loudness_info_struct *pstr_ia_loudness_info,
    WORD32 req_dwnmix_id, WORD32 drc_set_id_req, ia_drc_instructions_struct *pstr_drc_instruction,
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct)
{
  WORD32 cnt;
  WORD32 group_id_req = 0;
  WORD32 group_preset_id_req = 0;

  if (1 != pstr_drc_sel_proc_params_struct->dynamic_range_control_on)
  {
    if (0 > pstr_drc_instruction->drc_set_id)
    {
      group_id_req = -1;
      group_preset_id_req = -1;
    }
  }
  else
  {
    switch (pstr_drc_instruction->drc_instructions_type)
    {
    case 2:
      group_id_req = pstr_drc_instruction->mae_group_id;
      group_preset_id_req = -1;
      break;
    case 3:
      group_id_req = -1;
      group_preset_id_req = pstr_drc_instruction->mae_group_preset_id;
      break;
    default:
      group_id_req = -1;
      group_preset_id_req = -1;
      break;
    }
  }

  for (cnt = 0; cnt < loudness_info_cnt; cnt++)
  {
    if ((req_dwnmix_id == pstr_ia_loudness_info[cnt].downmix_id) &&
        (drc_set_id_req == pstr_ia_loudness_info[cnt].drc_set_id) &&
        (impd_drc_chk_loudness_info_present(&(pstr_ia_loudness_info[cnt]))))
    {
      loudness_info_matching[*info_cnt] = &(pstr_ia_loudness_info[cnt]);
      (*info_cnt)++;
    }
  }

  {
    WORD32 cnt1 = 0, cnt2 = 0, l = 0, m = 0;
    WORD32 largest_grp_preset_id = 0;
    WORD32 loud_info_type3_match[LOUDNESS_INFO_COUNT_MAX];
    WORD32 loud_info_type3_match_num_members[LOUDNESS_INFO_COUNT_MAX];
    WORD32 loud_info_type2_match[LOUDNESS_INFO_COUNT_MAX];
    WORD32 loud_info_type0_match[LOUDNESS_INFO_COUNT_MAX];
    WORD32 max_num_members_grp_preset_ids = 0;
    WORD32 max_grp_id = 0;

    for (cnt = 0; cnt < *info_cnt; cnt++)
    {
      switch (loudness_info_matching[cnt]->loudness_info_type)
      {
      case 0:
        loud_info_type0_match[m] = cnt;
        m++;
        break;
      case 2:
        for (cnt1 = pstr_drc_sel_proc_params_struct->num_group_ids_requested - 1; cnt1 >= 0;
             cnt1--)
        {
          if (loudness_info_matching[cnt]->mae_group_id ==
              pstr_drc_sel_proc_params_struct->group_id_requested[cnt1])
          {
            loud_info_type2_match[l] = cnt;
            l++;
          }
        }
        break;
      case 3:
        for (cnt1 = pstr_drc_sel_proc_params_struct->num_group_preset_ids_requested - 1;
             cnt1 >= 0; cnt1--)
        {
          if (loudness_info_matching[cnt]->mae_group_preset_id ==
              pstr_drc_sel_proc_params_struct->group_preset_id_requested[cnt1])
          {
            loud_info_type3_match[cnt2] = cnt;
            loud_info_type3_match_num_members[cnt2] =
                pstr_drc_sel_proc_params_struct->num_members_group_preset_ids_requested[cnt1];
            cnt2++;
          }
        }
        break;
      }
    }
    *info_cnt = 0;

    if (0 != cnt2)
    {
      if (-1 != group_preset_id_req)
      {
        for (cnt = 0; cnt < cnt2; cnt++)
        {
          if (loudness_info_matching[loud_info_type3_match[cnt]]->mae_group_preset_id ==
              group_preset_id_req)
          {
            loudness_info_matching[*info_cnt] =
                loudness_info_matching[loud_info_type3_match[cnt]];
            (*info_cnt)++;
          }
        }
      }

      if (0 == *info_cnt &&
          -1 != pstr_drc_sel_proc_params_struct->group_preset_id_requested_preference)
      {
        for (cnt = 0; cnt < cnt2; cnt++)
        {
          if (loudness_info_matching[loud_info_type3_match[cnt]]->mae_group_preset_id ==
              pstr_drc_sel_proc_params_struct->group_preset_id_requested_preference)
          {
            loudness_info_matching[*info_cnt] =
                loudness_info_matching[loud_info_type3_match[cnt]];
            (*info_cnt)++;
          }
        }
      }

      if (0 == *info_cnt)
      {
        for (cnt = 0; cnt < cnt2; cnt++)
        {
          if (loud_info_type3_match_num_members[cnt] >= max_num_members_grp_preset_ids)
          {
            if (loud_info_type3_match_num_members[cnt] > max_num_members_grp_preset_ids)
            {
              max_num_members_grp_preset_ids = loud_info_type3_match_num_members[cnt];
              *info_cnt = 0;
            }

            if (loudness_info_matching[loud_info_type3_match[cnt]]->mae_group_preset_id >=
                largest_grp_preset_id)
            {
              if (loudness_info_matching[loud_info_type3_match[cnt]]->mae_group_preset_id >
                  largest_grp_preset_id)
              {
                largest_grp_preset_id =
                    loudness_info_matching[loud_info_type3_match[cnt]]->mae_group_preset_id;
                *info_cnt = 0;
              }
              loudness_info_matching[*info_cnt] =
                  loudness_info_matching[loud_info_type3_match[cnt]];
              (*info_cnt)++;
            }
          }
        }
      }
    }

    if (0 == *info_cnt && 0 != l)
    {
      if (-1 != group_id_req)
      {
        for (cnt = 0; cnt < l; cnt++)
        {
          if (loudness_info_matching[loud_info_type2_match[cnt]]->mae_group_id == group_id_req)
          {
            loudness_info_matching[*info_cnt] =
                loudness_info_matching[loud_info_type2_match[cnt]];
            (*info_cnt)++;
          }
        }
      }

      if (0 == *info_cnt)
      {
        for (cnt = 0; cnt < l; cnt++)
        {
          if (loudness_info_matching[loud_info_type2_match[cnt]]->mae_group_id >= max_grp_id)
          {
            if (loudness_info_matching[loud_info_type2_match[cnt]]->mae_group_id > max_grp_id)
            {
              max_grp_id = loudness_info_matching[loud_info_type2_match[cnt]]->mae_group_id;
              *info_cnt = 0;
            }
            loudness_info_matching[*info_cnt] =
                loudness_info_matching[loud_info_type2_match[cnt]];
            (*info_cnt)++;
          }
        }
      }
    }

    if (0 == *info_cnt && 0 != m)
    {
      for (cnt = 0; cnt < m; cnt++)
      {
        loudness_info_matching[*info_cnt] = loudness_info_matching[loud_info_type0_match[cnt]];
        (*info_cnt)++;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_check_loudness_payload
 *
 *  \brief Function to check loudness payload
 *
 *  \param [out]  loudness_info_matching   Pointer to loudness info structure
 *  \param [out]  info_cnt   info count
 *  \param [in]  pstr_ia_loudness_info   Pointer to loudness info structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  pstr_drc_sel_proc_params_struct   Pointer to drc selection process params
 * structure
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_check_loudness_payload(
    ia_drc_loudness_info_struct *loudness_info_matching[], WORD32 *info_cnt,
    WORD32 loudness_info_cnt, ia_drc_loudness_info_struct *pstr_ia_loudness_info,
    WORD32 req_dwnmix_id, ia_drc_instructions_struct *pstr_drc_instruction,
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  if (0 > pstr_drc_instruction->drc_set_id)
  {
    pstr_drc_instruction->drc_set_id = ID_FOR_NO_DRC;
  }
  err_code = impd_drc_check_loudness_info(
      loudness_info_matching, info_cnt, loudness_info_cnt, pstr_ia_loudness_info, req_dwnmix_id,
      pstr_drc_instruction->drc_set_id, pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(loudness_info_matching, info_cnt, loudness_info_cnt,
                                          pstr_ia_loudness_info, ID_FOR_ANY_DOWNMIX,
                                          pstr_drc_instruction->drc_set_id, pstr_drc_instruction,
                                          pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(loudness_info_matching, info_cnt, loudness_info_cnt,
                                          pstr_ia_loudness_info, req_dwnmix_id, ID_FOR_ANY_DRC,
                                          pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(loudness_info_matching, info_cnt, loudness_info_cnt,
                                          pstr_ia_loudness_info, req_dwnmix_id, ID_FOR_NO_DRC,
                                          pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(
      loudness_info_matching, info_cnt, loudness_info_cnt, pstr_ia_loudness_info,
      ID_FOR_ANY_DOWNMIX, ID_FOR_ANY_DRC, pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(
      loudness_info_matching, info_cnt, loudness_info_cnt, pstr_ia_loudness_info,
      ID_FOR_ANY_DOWNMIX, ID_FOR_NO_DRC, pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(loudness_info_matching, info_cnt, loudness_info_cnt,
                                          pstr_ia_loudness_info, ID_FOR_BASE_LAYOUT,
                                          pstr_drc_instruction->drc_set_id, pstr_drc_instruction,
                                          pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(
      loudness_info_matching, info_cnt, loudness_info_cnt, pstr_ia_loudness_info,
      ID_FOR_BASE_LAYOUT, ID_FOR_ANY_DRC, pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;
  err_code = impd_drc_check_loudness_info(
      loudness_info_matching, info_cnt, loudness_info_cnt, pstr_ia_loudness_info,
      ID_FOR_BASE_LAYOUT, ID_FOR_NO_DRC, pstr_drc_instruction, pstr_drc_sel_proc_params_struct);
  if (err_code || *info_cnt)
    goto match_end;

match_end:
  return (err_code);
}

/**
 *  impd_drc_find_overall_loudness_info
 *
 *  \brief Function to find overall loudness info
 *
 *  \param [out]  loudness_info_matching   Pointer to loudness info structure
 *  \param [out]  info_cnt   info count
 *  \param [out]  overall_loudness_info_flag   overall loudness info present flag
 *  \param [in]  pstr_drc_sel_proc_params_struct   Pointer to drc selection process params
 * structure
 *  \param [in]  pstr_loudness_info   Pointer to loudness info set structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE
impd_drc_find_overall_loudness_info(
    ia_drc_loudness_info_struct *loudness_info_matching[], WORD32 *info_cnt,
    WORD32 *overall_loudness_info_flag,
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
    ia_drc_loudness_info_set_struct *pstr_loudness_info, WORD32 req_dwnmix_id,
    ia_drc_instructions_struct *pstr_drc_instruction)
{
  IA_ERRORCODE err_code;
  *info_cnt = 0;

  if (1 == pstr_drc_sel_proc_params_struct->album_mode)
  {
    err_code = impd_drc_check_loudness_payload(
        loudness_info_matching, info_cnt, pstr_loudness_info->loudness_info_album_count,
        pstr_loudness_info->str_loudness_info_album, req_dwnmix_id, pstr_drc_instruction,
        pstr_drc_sel_proc_params_struct);
    if (err_code)
      return (err_code);
  }
  if (0 == *info_cnt)
  {
    err_code = impd_drc_check_loudness_payload(
        loudness_info_matching, info_cnt, pstr_loudness_info->loudness_info_count,
        pstr_loudness_info->loudness_info, req_dwnmix_id, pstr_drc_instruction,
        pstr_drc_sel_proc_params_struct);
    if (err_code)
      return (err_code);
  }
  *overall_loudness_info_flag = (*info_cnt > 0);
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_high_pass_loudness_adjust_info
 *
 *  \brief Function to find high pass loudness adjust info
 *
 *  \param [out]  loudness_hp_adjust_flag   flag to update if loudness highpass adjust info
 * present
 *  \param [out]  loudness_hp_adjust_info   loudness highpass adjust info
 *  \param [in]  pstr_ia_loudness_info   Pointer to loudness info structure
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE
impd_drc_high_pass_loudness_adjust_info(WORD32 *loudness_hp_adjust_flag,
                                        FLOAT32 *loudness_hp_adjust_info,
                                        ia_drc_loudness_info_struct *pstr_ia_loudness_info)
{
  WORD32 cnt1, cnt2;
  *loudness_hp_adjust_flag = 0;
  *loudness_hp_adjust_info = 0;

  for (cnt1 = 0; cnt1 < pstr_ia_loudness_info->measurement_count; cnt1++)
  {
    if (IA_DRC_MEASUREMENT_SYSTEM_BS_1770_4_PRE_PROCESSING ==
        pstr_ia_loudness_info->loudness_measure[cnt1].measurement_system)
    {
      for (cnt2 = 0; cnt2 < pstr_ia_loudness_info->measurement_count; cnt2++)
      {
        if ((IA_DRC_MEASUREMENT_SYSTEM_BS_1770_4 ==
             pstr_ia_loudness_info->loudness_measure[cnt2].measurement_system) &&
            (pstr_ia_loudness_info->loudness_measure[cnt1].method_def ==
             pstr_ia_loudness_info->loudness_measure[cnt2].method_def))
        {
          *loudness_hp_adjust_flag = 1;
          *loudness_hp_adjust_info = (pstr_ia_loudness_info->loudness_measure[cnt1].method_value -
                                      pstr_ia_loudness_info->loudness_measure[cnt2].method_value);
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_find_high_pass_loudness_adjust
 *
 *  \brief Function to find high pass loudness adjust
 *
 *  \param [out]  loudness_hp_adjust_flag   flag to update if loudness highpass adjust info
 * present
 *  \param [out]  loudness_hp_adjust_info   loudness highpass adjust info
 *  \param [in]  pstr_loudness_info   Pointer to loudness info structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  drc_set_id_req   drc set id requested
 *  \param [in]  album_mode   album mode
 *  \param [in]  device_cutoff_freq   device cutoff frequency
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_find_high_pass_loudness_adjust(
    WORD32 *loudness_hp_adjust_flag, FLOAT32 *loudness_hp_adjust_info,
    ia_drc_loudness_info_set_struct *pstr_loudness_info, WORD32 req_dwnmix_id,
    WORD32 drc_set_id_req, WORD32 album_mode, FLOAT32 device_cutoff_freq)
{
  IA_ERRORCODE err_code;
  WORD32 cnt;
  WORD32 loudness_drc_set_id_req;

  if (drc_set_id_req >= 0)
  {
    loudness_drc_set_id_req = drc_set_id_req;
  }
  else
  {
    loudness_drc_set_id_req = 0;
  }

  *loudness_hp_adjust_flag = 0;

  if (1 == album_mode)
  {
    for (cnt = 0; cnt < pstr_loudness_info->loudness_info_album_count; cnt++)
    {
      if (((ID_FOR_ANY_DOWNMIX == pstr_loudness_info->str_loudness_info_album[cnt].downmix_id) ||
           (req_dwnmix_id == pstr_loudness_info->str_loudness_info_album[cnt].downmix_id)) &&
          (pstr_loudness_info->str_loudness_info_album[cnt].drc_set_id ==
           loudness_drc_set_id_req))
      {
        err_code = impd_drc_high_pass_loudness_adjust_info(
            loudness_hp_adjust_flag, loudness_hp_adjust_info,
            &(pstr_loudness_info->loudness_info[cnt]));
        if (err_code)
          return (err_code);
      }
    }
  }
  if (0 == *loudness_hp_adjust_flag)
  {
    for (cnt = 0; cnt < pstr_loudness_info->loudness_info_count; cnt++)
    {
      if (((ID_FOR_ANY_DOWNMIX == pstr_loudness_info->loudness_info[cnt].downmix_id) ||
           (req_dwnmix_id == pstr_loudness_info->loudness_info[cnt].downmix_id)) &&
          (pstr_loudness_info->loudness_info[cnt].drc_set_id == loudness_drc_set_id_req))
      {
        err_code = impd_drc_high_pass_loudness_adjust_info(
            loudness_hp_adjust_flag, loudness_hp_adjust_info,
            &(pstr_loudness_info->loudness_info[cnt]));
        if (err_code)
          return (err_code);
      }
    }
  }
  if (0 == *loudness_hp_adjust_flag)
  {
    for (cnt = 0; cnt < pstr_loudness_info->loudness_info_count; cnt++)
    {
      if ((ID_FOR_BASE_LAYOUT == pstr_loudness_info->loudness_info[cnt].downmix_id) &&
          (pstr_loudness_info->loudness_info[cnt].drc_set_id == loudness_drc_set_id_req))
      {
        err_code = impd_drc_high_pass_loudness_adjust_info(
            loudness_hp_adjust_flag, loudness_hp_adjust_info,
            &(pstr_loudness_info->loudness_info[cnt]));
        if (err_code)
          return (err_code);
      }
    }
  }
  if (0 == *loudness_hp_adjust_flag)
  {
    for (cnt = 0; cnt < pstr_loudness_info->loudness_info_count; cnt++)
    {
      if ((ID_FOR_BASE_LAYOUT == pstr_loudness_info->loudness_info[cnt].downmix_id) &&
          (0 == pstr_loudness_info->loudness_info[cnt].drc_set_id))
      {
        err_code = impd_drc_high_pass_loudness_adjust_info(
            loudness_hp_adjust_flag, loudness_hp_adjust_info,
            &(pstr_loudness_info->loudness_info[cnt]));
        if (err_code)
          return (err_code);
      }
    }
  }
  if (*loudness_hp_adjust_flag)
  {
    *loudness_hp_adjust_info *= (ia_max_flt(20, ia_min_flt(500, device_cutoff_freq)) - 20) / 480;
  }
  else
  {
    *loudness_hp_adjust_info = 0;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_init_loudness_control
 *
 *  \brief Initialise loudness control
 *
 *  \param [out]  loudness_info_cnt   loudness info cnt
 *  \param [out]  eq_set_id   drc set id
 *  \param [out]  ln_gain_db   loudness normalization gain
 *  \param [out]  loudness   loudness
 *  \param [in]  pstr_drc_sel_proc_params_struct   Pointer to drc selection process params
 * structure
 *  \param [in]  pstr_loudness_info   Pointer to loudness info structure
 *  \param [in]  req_dwnmix_id   requested dwnmix id
 *  \param [in]  pstr_drc_instruction   Pointer to drc instructions structure
 *  \param [in]  num_compression_eq_count   no of compression eq count
 *  \param [in]  num_compression_eq_id   no of compression eq id
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_init_loudness_control(
    WORD32 *loudness_info_cnt, WORD32 eq_set_id[], FLOAT32 ln_gain_db[], FLOAT32 loudness[],
    ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
    ia_drc_loudness_info_set_struct *pstr_loudness_info, WORD32 req_dwnmix_id,
    ia_drc_instructions_struct *pstr_drc_instruction, WORD32 num_compression_eq_count,
    WORD32 *num_compression_eq_id)
{
  IA_ERRORCODE err_code;
  WORD32 cnt, info_cnt = 0, pre_lim_cnt;
  WORD32 loudness_hp_adjust_flag;
  WORD32 overall_loudness_info_flag;
  FLOAT32 pre_process_adjust;

  cnt = 0;
  if (0 > pstr_drc_instruction->drc_set_id)
  {
    memset(ln_gain_db, 0, num_compression_eq_count);
    for (cnt = 0; cnt < num_compression_eq_count; cnt++)
    {
      loudness[cnt] = UNDEFINED_LOUDNESS_VALUE;
      eq_set_id[cnt] = num_compression_eq_id[cnt];
    }
  }
  if (MAX_NUM_COMPRESSION_EQ <= cnt)
  {
    return IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR;
  }

  loudness[cnt] = UNDEFINED_LOUDNESS_VALUE;
  ln_gain_db[cnt] = 0;
  eq_set_id[cnt] = 0;
  cnt++;

  pre_lim_cnt = cnt;

  if (1 == pstr_drc_sel_proc_params_struct->loudness_normalization_on)
  {
    WORD32 cnt1;
    ia_drc_loudness_info_struct *pstr_ia_loudness_info[16];
    err_code = impd_drc_find_overall_loudness_info(
        pstr_ia_loudness_info, &info_cnt, &overall_loudness_info_flag,
        pstr_drc_sel_proc_params_struct, pstr_loudness_info, req_dwnmix_id, pstr_drc_instruction);
    if (err_code)
      return (err_code);

    if (1 == overall_loudness_info_flag)
    {
      WORD32 req_method_def = IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS;
      WORD32 other_method_def = IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS;
      WORD32 req_preprocessing = 0;
      WORD32 match_measure;
      const WORD32 *system_bonus = ia_drc_measurement_system_def_tbl;
      FLOAT32 method_value = 0;

      switch (pstr_drc_sel_proc_params_struct->loudness_measurement_method)
      {
      case IA_DRC_USER_METHOD_DEFINITION_ANCHOR_LOUDNESS:
        req_method_def = IA_DRC_METHOD_DEFINITION_ANCHOR_LOUDNESS;
        other_method_def = IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS;
        break;
      case IA_DRC_USER_METHOD_DEFINITION_DEFAULT:
      case IA_DRC_USER_METHOD_DEFINITION_PROGRAM_LOUDNESS:
        req_method_def = IA_DRC_METHOD_DEFINITION_PROGRAM_LOUDNESS;
        other_method_def = IA_DRC_METHOD_DEFINITION_ANCHOR_LOUDNESS;
        break;
      default:
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
        break;
      }

      switch (pstr_drc_sel_proc_params_struct->loudness_measurement_system)
      {
      case IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_E:
        system_bonus = ia_drc_measurement_system_rms_e_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_D:
        system_bonus = ia_drc_measurement_system_rms_d_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_C:
        system_bonus = ia_drc_measurement_system_rms_c_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_B:
        system_bonus = ia_drc_measurement_system_rms_b_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_RESERVED_A:
        system_bonus = ia_drc_measurement_system_rms_a_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_EXPERT_PANEL:
        system_bonus = ia_drc_measurement_system_expert_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_USER:
        system_bonus = ia_drc_measurement_system_user_tbl;
        break;
      case IA_DRC_USER_MEASUREMENT_SYSTEM_DEFAULT:
      case IA_DRC_USER_MEASUREMENT_SYSTEM_BS_1770_4:
        system_bonus = ia_drc_ameasurement_system_bs1770_3_tbl;
        break;
      default:
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
        break;
      }

      switch (pstr_drc_sel_proc_params_struct->loudness_measurement_pre_proc)
      {
      case IA_DRC_USER_LOUDNESS_PREPROCESSING_HIGHPASS:
        req_preprocessing = 1;
        break;
      case IA_DRC_USER_LOUDNESS_PREPROCESSING_DEFAULT:
      case IA_DRC_USER_LOUDNESS_PREPROCESSING_OFF:
        req_preprocessing = 0;
        break;
      default:
        return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
        break;
      }

      for (cnt = 0; cnt < info_cnt; cnt++)
      {
        match_measure = -1;
        for (cnt1 = 0; cnt1 < pstr_ia_loudness_info[cnt]->measurement_count; cnt1++)
        {
          ia_drc_loudness_measure_struct *pstr_loudness_measure =
              &(pstr_ia_loudness_info[cnt]->loudness_measure[cnt1]);
          if (req_method_def == pstr_loudness_measure->method_def &&
              match_measure < system_bonus[pstr_loudness_measure->measurement_system])
          {

            method_value = pstr_loudness_measure->method_value;
            match_measure = system_bonus[pstr_loudness_measure->measurement_system];
          }
        }
        if (-1 == match_measure)
        {
          for (cnt1 = 0; cnt1 < pstr_ia_loudness_info[cnt]->measurement_count; cnt1++)
          {
            ia_drc_loudness_measure_struct *pstr_loudness_measure =
                &(pstr_ia_loudness_info[cnt]->loudness_measure[cnt1]);
            if (other_method_def == pstr_loudness_measure->method_def &&
                match_measure < system_bonus[pstr_loudness_measure->measurement_system])
            {

              method_value = pstr_loudness_measure->method_value;
              match_measure = system_bonus[pstr_loudness_measure->measurement_system];
            }
          }
        }

        if (1 == req_preprocessing)
        {
          err_code = impd_drc_find_high_pass_loudness_adjust(
              &loudness_hp_adjust_flag, &pre_process_adjust, pstr_loudness_info, req_dwnmix_id,
              pstr_drc_instruction->drc_set_id, pstr_drc_sel_proc_params_struct->album_mode,
              (FLOAT32)pstr_drc_sel_proc_params_struct->device_cut_off_frequency);
          if (err_code)
            return (err_code);

          if (0 == loudness_hp_adjust_flag)
          {
            pre_process_adjust = -2.0f;
          }
          method_value += pre_process_adjust;
        }

        ln_gain_db[cnt] = pstr_drc_sel_proc_params_struct->target_loudness - method_value;
        loudness[cnt] = method_value;
        eq_set_id[cnt] = 0;
      }
    }
  }
  if (0 >= info_cnt)
  {
    *loudness_info_cnt = pre_lim_cnt;
  }
  else
  {
    *loudness_info_cnt = info_cnt;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of DRCProcessing */