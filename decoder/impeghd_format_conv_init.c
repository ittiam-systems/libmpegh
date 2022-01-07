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
#include <stdlib.h>
#include <stdio.h>
#include <impeghd_type_def.h>
#include "impeghd_hoa_common_values.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impd_drc_common.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_config.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry_rom.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_rom.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"

/**
 * @defgroup FmtConvInit Format converter intialization
 * @ingroup  FmtConvInit
 * @brief Format converter intialization
 *
 * @{
 */

/**
 *  impeghd_format_conv_dmx_init
 *
 *  \brief Format conversion downmix initialization
 *
 *  \param [in,out] active_dmx          Pointer to format convert downmix structure
 *  \param [in]     format_conv_params  Pointer to format convert params structure
 *
 *  \return IA_ERRORCODE                      Error
 *
 */
IA_ERRORCODE impeghd_format_conv_dmx_init(ia_format_conv_dmx_state *active_dmx,
                                          ia_format_conv_param *format_conv_params)
{
  FLOAT32(*downmix_mat)
  [FC_BANDS][FC_MAX_CHANNELS][FC_MAX_CHANNELS] = &format_conv_params->downmix_mat_freq;
  WORD32 num_in_ch = format_conv_params->num_in_ch;
  WORD32 num_out_ch = format_conv_params->num_out_ch;
  WORD32 out_ch, in_ch, band;
  active_dmx->num_in_ch = num_in_ch;
  active_dmx->num_out_ch = num_out_ch;

  for (out_ch = 0; out_ch < active_dmx->num_out_ch; out_ch++)
  {
    for (in_ch = 0; in_ch < active_dmx->num_in_ch; in_ch++)
    {
      for (band = 0; band < FC_BANDS; band++)
      {
        active_dmx->downmix_mat[out_ch][in_ch][band] = (*downmix_mat)[band][in_ch][out_ch];
      }
    }
  }
  return 0;
}

/**
 *  impeghd_m_channel
 *
 *  \brief Identify the channel format
 *
 *  \param [in] channel          Channel to be searched
 *  \return WORD8
 */
static WORD8 impeghd_m_channel(const WORD32 channel)
{
  if ((channel >= CH_M_000) && (channel <= CH_M_180))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/**
 *  impeghd_u_channel
 *
 *  \brief Identify the channel format
 *
 *  \param [in] channel          Channel to be searched
 *  \return WORD8
 *
 */
static WORD8 impeghd_u_channel(const WORD32 channel)
{
  if ((channel >= CH_U_000) && (channel <= CH_U_180))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}
/**
 *  impeghd_format_conv_peak_filter_neg_gain
 *
 *  \brief Calculate peak filter value with negative gain
 *
 *  \param [in] ptr_peak_filter_params  Pointer to filter params array
 *  \param [in] frequency               Filter cut off frequency
 *  \param [in] strength                1st equaliser strength
 *  \param [in] strength_1               2nd equaliser strength
 *
 *  \return FLOAT32                 Peak filter value
 *
 */
static FLOAT32 impeghd_format_conv_peak_filter_neg_gain(const FLOAT32 *ptr_peak_filter_params,
                                                        FLOAT32 frequency, const FLOAT32 strength,
                                                        const FLOAT32 strength_1)
{

  const FLOAT32 frequency_peak = ptr_peak_filter_params[0];
  const FLOAT32 factor_q = ptr_peak_filter_params[1];
  const FLOAT32 peak_gain = strength * ptr_peak_filter_params[2];
  const FLOAT32 gain_in = strength_1 * ptr_peak_filter_params[3];
  FLOAT32 linear_gain, gain;
  FLOAT32 fc_pow_4 = frequency * frequency * frequency * frequency;
  FLOAT32 q_fac_pow_2 = (factor_q * factor_q);
  FLOAT32 peak_sqxfc_sq = frequency_peak * frequency_peak * frequency * frequency;
  FLOAT32 peak_fre_pow_4 = frequency_peak * frequency_peak * frequency_peak * frequency_peak;
  FLOAT32 temp;
  FLOAT32 sq_linear_gain;
  linear_gain = (FLOAT32)pow(10.0f, (FLOAT32)ia_fabs_flt(peak_gain) / 20.0f);
  sq_linear_gain = linear_gain * linear_gain;
  temp = fc_pow_4 + peak_fre_pow_4;

  gain = ia_add_flt(temp, ia_mul_flt((1.0f / q_fac_pow_2 - 2.0f), peak_sqxfc_sq)) /
         ia_add_flt(temp, ia_mul_flt((sq_linear_gain / q_fac_pow_2 - 2.0f), peak_sqxfc_sq));
  // 10.3.4.6.4 Derivation of equalizer gains GEQ in spec ISO/IEC 23008-3
  return ia_mul_flt((FLOAT32)ia_sqrt_flt(gain), (FLOAT32)pow(10.0f, gain_in / 20.0f));
}

/**
 *  impeghd_fc_filter_peak
 *
 *  \brief Calculate peak filter value
 *
 *  \param [in] params      Pointer to filter params array
 *  \param [in] freq        Filter cut off frequency
 *
 *  \return FLOAT32         Peak filter value
 *
 */
static FLOAT32 impeghd_fc_filter_peak(const FLOAT32 *params, FLOAT32 freq)
{
  const FLOAT32 factor_q = params[1];
  const FLOAT32 gain_in = params[3];
  const FLOAT32 peak_gain = params[2];
  const FLOAT32 frequency_peak = params[0];
  const FLOAT32 frequency = freq;
  FLOAT32 temp;
  FLOAT32 sq_linear_gain;
  FLOAT32 fc_pow_4 = frequency * frequency * frequency * frequency;
  FLOAT32 linear_gain, gain;
  FLOAT32 peak_sqxfc_sq = frequency_peak * frequency_peak * frequency * frequency;
  FLOAT32 q_fac_pow_2 = (factor_q * factor_q);
  linear_gain = (FLOAT32)pow(10.0f, (FLOAT32)ia_fabs_flt(peak_gain) / 20.0f);
  FLOAT32 peak_fre_pow_4 = frequency_peak * frequency_peak * frequency_peak * frequency_peak;
  temp = fc_pow_4 + peak_fre_pow_4;
  sq_linear_gain = linear_gain * linear_gain;
  gain = ia_add_flt(temp, ia_mul_flt((sq_linear_gain / q_fac_pow_2 - 2.0f), peak_sqxfc_sq)) /
         (ia_add_flt(temp, ia_mul_flt((1.0f / q_fac_pow_2 - 2.0f), peak_sqxfc_sq)));
  // 10.3.4.6.4 Derivation of equalizer gains GEQ in spec ISO/IEC 23008-3
  return ia_mul_flt((FLOAT32)ia_sqrt_flt(gain), (FLOAT32)pow(10.0f, gain_in / 20.0f));
}
/**
 *  impeghd_fc_find_channel
 *
 *  \brief Search channel for format conversion
 *
 *  \param [in] ch          Channel to be searched
 *  \param [in] channels    Pointer to channels array
 *  \param [in] nchan       Number of channels
 *
 *  \return WORD32          Matching channel index
 *
 */
static WORD32 impeghd_fc_find_channel(const WORD32 ch, const WORD32 nchan, const WORD32 *channels)
{
  WORD32 chanl;
  WORD32 invl_channel = -1;
  for (chanl = 0; chanl < nchan; chanl++)
  {
    if (channels[chanl] == ch)
    {
      return chanl;
    }
  }
  return invl_channel;
}

/**
 *  impeghd_fc_eq_compute
 *
 *  \brief Compute equalize value
 *
 *  \param [in]  samp_rate  Sample rate
 *  \param [in] pstr_params  Pointer to format convert state structure
 *
 *  \return IA_ERRORCODE error_code
 *
 */

static IA_ERRORCODE impeghd_fc_eq_compute(const WORD32 samp_rate,
                                          ia_format_conv_data_state *pstr_params)
{
  WORD32 rules;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 sr_flag = 1;
  WORD32 fs = (WORD32)samp_rate;
  const FLOAT32 *ptr_filter_param;
  FLOAT32 *ptr_eq;
  const FLOAT32 *ptr_frequency, *ptr_ft;
  for (i = 0; i < 8; i++)
  {
    if (ia_sampling_rate_tbl[i] == fs)
    {
      sr_flag = 0;
      break;
    }
  }
  if (sr_flag)
    return IA_MPEGH_DEC_INIT_FATAL_SAMP_FREQ_NOT_SUPPORTED;
  ptr_frequency = ia_eq_freq_fc[i];
  ptr_ft = ptr_frequency;
  for (rules = RULE_EQ1; rules <= RULE_EQ5; rules++)
  {
    ptr_eq = pstr_params->eq[rules - 1];
    ptr_frequency = ptr_ft;
    ptr_filter_param = &ia_peak_filter_params_fc[rules - 1][0];
    for (i = 0; i < FC_ERB_BANDS; i++)
    {
      ptr_eq[i] = 1.0f;
      if ((RULE_EQ4 == rules) || (RULE_EQ3 == rules))
      {
        ptr_eq[i] *= impeghd_fc_filter_peak(&ptr_filter_param[0], *ptr_frequency);
        ptr_eq[i] *= impeghd_fc_filter_peak(&ptr_filter_param[4], *ptr_frequency);
      }
      if (RULE_EQ3 == rules)
        ptr_filter_param = &ia_peak_filter_params_fc[2][4];
      if (RULE_EQ4 != rules)
        ptr_eq[i] *=
            impeghd_format_conv_peak_filter_neg_gain(ptr_filter_param, *ptr_frequency, 1, 1);
      ptr_frequency++;
    }
  }
  return err_code;
}
/**
 *  impeghd_format_conv_compute_eq_dev
 *
 *  \brief Compute equalize value for the deviation for speaker specification
 *
 *  \param [in]  samp_rate  Sample rate
 *  \param [in,out] pstr_params  Pointer to format convert state structure
 *  \param [in]  eq_1_idx  1st equaliser index
 *  \param [in]  eq_2_idx  2nd equaliser index
 *  \param [in]  eq_1_strength  1st equaliser strength
 *  \param [in]  eq_2_strength  2nd equaliser strength
 *
 *  \return IA_ERRORCODE err_code
 *
 */

static IA_ERRORCODE
impeghd_format_conv_compute_eq_dev(const WORD32 samp_rate, ia_format_conv_data_state *pstr_params,
                                   const WORD32 eq_1_idx, const WORD32 eq_2_idx,
                                   const FLOAT32 eq_1_strength, const FLOAT32 eq_2_strength)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 sr_flag = 1;
  WORD32 fs = (WORD32)samp_rate;
  FLOAT32 *ptr_eq;
  const FLOAT32 *fc;
  const FLOAT32 *ptr_filter_param;

  for (i = 0; i < 12; i++)
  {
    if (ia_sampling_rate_tbl[i] == fs)
    {
      sr_flag = 0;
      break;
    }
  }
  if (sr_flag)
    return IA_MPEGH_DEC_INIT_FATAL_SAMP_FREQ_NOT_SUPPORTED;
  fc = ia_eq_freq_fc[i];
  ptr_eq = pstr_params->eq[0];
  for (i = 0; i < FC_ERB_BANDS; i++)
  {
    if (eq_1_idx == RULE_EQ1)
    {

      ptr_filter_param = &ia_peak_filter_params_fc[eq_1_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(12000.0f, 0.3f,
                                                                    // -2.0f*eq_1_strength,
                                                                    // 1.0f*eq_1_strength, fc);
    }
    if (eq_2_idx == RULE_EQ1)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_2_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(12000.0f, 0.3f,
                                                                    // -2.0f*eq_2_strength,
                                                                    // 1.0f*eq_2_strength, fc);
    }

    /* EQ2: for mixing surround height to surround horizontal */

    if (eq_1_idx == RULE_EQ2)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_1_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(12000.0f, 0.3f,
                                                                    // -3.5f*eq_1_strength,
                                                                    // 1.0f*eq_1_strength, fc);
    }
    if (eq_2_idx == RULE_EQ2)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_2_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(12000.0f, 0.3f,
                                                                    // -3.5f*eq_2_strength,
                                                                    // 1.0f*eq_2_strength, fc);
    }

    /* EQ3: for mixing top to height loudspeakers */

    if (eq_1_idx == RULE_EQ3)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_1_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(200.0f, 0.3f,
                                                                    // -6.5f*eq_1_strength,
                                                                    // 0.7f*eq_1_strength, fc);
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(1300.0f, 0.5f,
                                                                    // 1.8f*eq_1_strength,
                                                                    // 0.0f*eq_1_strength, fc);
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(600.0f, 1.0f,
                                                                    // 2.0f*eq_1_strength,
                                                                    // 0.0f*eq_1_strength, fc);
    }
    if (eq_2_idx == RULE_EQ3)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_2_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(200.0f, 0.3f,
                                                                    // -6.5f*eq_2_strength,
                                                                    // 0.7f*eq_2_strength, fc);
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(1300.0f, 0.5f,
                                                                    // 1.8f*eq_2_strength,
                                                                    // 0.0f*eq_2_strength, fc);
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(600.0f, 1.0f,
                                                                    // 2.0f*eq_2_strength,
                                                                    // 0.0f*eq_2_strength, fc);
    }

    /* EQ4: for mixing top to horizontal loudspeakers */

    if (eq_1_idx == RULE_EQ4)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_1_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(5000.0f, 1.0f,
                                                                    // 4.5f*eq_1_strength,
                                                                    // -3.1f*eq_1_strength, fc);
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // peak_filter(1100.0f, 0.8f,
                                                                    // 1.8f*eq_1_strength,
                                                                    // 0.0f*eq_1_strength, fc);
    }
    if (eq_2_idx == RULE_EQ4)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_2_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(5000.0f, 1.0f,
                                                                    // 4.5f*eq_2_strength,
                                                                    // -3.1f*eq_2_strength, fc);
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength, 1 - eq_2_strength); // peak_filter(1100.0f, 0.8f,
                                                                    // 1.8f*eq_2_strength,
                                                                    // 0.0f*eq_2_strength, fc);
    }

    /* EQ5: for mixing M to U for rand5 */

    if (eq_1_idx == RULE_EQ5)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_1_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_1_strength, 1 - eq_1_strength); // eq[i] *= peak_filter(35.0f,
                                                                    // 0.25f, -1.3f*eq_1_strength,
                                                                    // 1.0f*eq_1_strength, fc);
    }
    if (eq_2_idx == RULE_EQ5)
    {
      ptr_filter_param = &ia_peak_filter_params_fc[eq_2_idx - 1][0];
      ptr_eq[i] *= impeghd_format_conv_peak_filter_neg_gain(
          ptr_filter_param, *fc, eq_2_strength,
          1 - eq_2_strength); // ptr_eq[i] *= peak_filter(35.0f,
                              // 0.25f, -1.3f*eq_2_strength,
                              // 1.0f*eq_2_strength, fc);
    }
  }
  return err_code;
}

/**
 *  impeghd_format_conv_init_data
 *
 *  \brief Format conversion data initialization
 *
 *  \param [in] ptr_scratch              Pointer to scratch memory
 *  \param [in,out] pstr_azi_elev_array  Pointer to azimuth elevation array
 *  \param [in] params                   Pointer to format converter param structure
 *  \param [in] ref_spk_layout           Pointer to 3d speaker config structure
 *  \return IA_ERRORCODE          Error
 *
 */
IA_ERRORCODE impeghd_format_conv_init_data(FLOAT32 *ptr_scratch,
                                           ia_cicp_ls_geo_str *pstr_azi_elev_array,
                                           ia_format_conv_param *params,
                                           ia_speaker_config_3d *ref_spk_layout)
{
  ia_cicp_ls_geo_str *pstr_azi_ele[CICP_MAX_CH];
  ia_format_conv_data_state *pstr_params = params->data_state;
  WORD32 ptr, j, erb;
  UWORD32 channel;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 invalid_order = 0, k;
  WORD32 *ptr_io_src, *ptr_io_dst;
  WORD32 *ptr_io_proc;
  WORD32 name_chn;
  const WORD32 sampling_freq = params->samp_rate;
  WORD32 num_in_ch = params->num_in_ch;
  WORD32 *ptr_in_ch_format = params->in_ch;
  const WORD32 *ptr_out_ch_format = params->out_ch;
  WORD32 start_chn;
  WORD32 idx, idx2;
  WORD32 input_channel_id, output_channel_idx;
  UWORD32 out_chn_num = params->num_out_ch, i;
  WORD32 num_chanl, chnl_u;
  WORD32 *ptr_dst_params = (params->data_state)->in_out_dst;
  WORD32 *ptr_src_params = (params->data_state)->in_out_src;
  WORD32 *ptr_proc_params = (params->data_state)->in_out_proc;
  FLOAT32 *ptr_az_ele_dev = ptr_scratch;
  FLOAT32 *ptr_gain_params = (params->data_state)->in_out_gain;
  FLOAT32 azi_a, azi_b, azi_target, mn, alpha1 = 0.0f, alpha0 = 0.0f, center;
  FLOAT32 a1, a2, nrm;
  FLOAT32 *ptr_io_gain;
  FLOAT32 gain;

  ptr_io_src = params->init_scratch.in_out_src;
  ptr_io_gain = params->init_scratch.in_out_gain;
  ptr_io_dst = params->init_scratch.in_out_dst;
  ptr_io_proc = params->init_scratch.in_out_proc;
  if (ref_spk_layout->spk_layout_type == 2)
  {
    for (i = 0; i < USED_CHANNEL; i++)
    {
      name_chn = ia_channel_names[i];
      pstr_azi_ele[name_chn] = &pstr_azi_elev_array[name_chn];
      pstr_azi_ele[name_chn]->ls_azimuth = ia_cicp_ls_geo_tbls[name_chn].ls_azimuth;
      pstr_azi_ele[name_chn]->ls_elevation = ia_cicp_ls_geo_tbls[name_chn].ls_elevation;
    }
    for (i = 0; i < out_chn_num; i++)
    {
      ptr_az_ele_dev[2 * i] = ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].az_angle_idx -
                              pstr_azi_ele[i]->ls_azimuth;
      ptr_az_ele_dev[2 * i + 1] =
          ref_spk_layout->str_flex_spk.str_flex_spk_descr[i].el_angle_idx -
          pstr_azi_ele[i]->ls_elevation;
      if ((fabs(ptr_az_ele_dev[2 * i]) > 35) || (fabs(ptr_az_ele_dev[2 * i + 1]) > 55))
      {
        invalid_order = 1;
        break;
      }
    }
    for (i = 0; i < out_chn_num; i++)
    {
      pstr_azi_ele[ptr_out_ch_format[i]]->ls_azimuth += ptr_az_ele_dev[2 * i];
      pstr_azi_ele[ptr_out_ch_format[i]]->ls_elevation += ptr_az_ele_dev[2 * i + 1];
    }
    for (i = 0; i < out_chn_num; i++)
    {
      if (invalid_order == 1)
        break;
      /* height channel? */
      if ((pstr_azi_ele[i]->ls_elevation < 0) || (pstr_azi_ele[i]->ls_elevation == 0) ||
          (pstr_azi_ele[i]->ls_elevation > 0))
      {
        /* compare angles between channel pairs */
        for (channel = 0; channel < out_chn_num; channel++)
        {
          if (channel == i)
          {
            continue;
          }
          if (pstr_azi_ele[ptr_out_ch_format[i]]->ls_azimuth <
              pstr_azi_ele[ptr_out_ch_format[channel]]->ls_azimuth)
          {
            /* ordering of randomized angles different? */
            if (pstr_azi_ele[ptr_out_ch_format[i]]->ls_azimuth >=
                pstr_azi_ele[ptr_out_ch_format[channel]]->ls_azimuth)
            {
              invalid_order = 1;
              break;
            }
          }
        }
      }
    }
    /* check that angle between any loudspeaker pair is not too small */
    for (i = 0; i < out_chn_num; i++)
    {
      if (invalid_order == 1)
        break;
      FLOAT32 azi, ele, angle;
      if ((ptr_out_ch_format[i] == CH_LFE1) || (ptr_out_ch_format[i] == CH_LFE2))
      { /* ignore LFEs */
        continue;
      }
      /* compare angles between channel pairs */
      for (channel = 0; channel < out_chn_num; channel++)
      {
        if ((ptr_out_ch_format[channel] == CH_LFE1) || (ptr_out_ch_format[channel] == CH_LFE2))
        { /* ignore LFEs */
          continue;
        }
        if (channel == i)
        {
          continue;
        }
        /* compute angle between loudspeaker pair */
        azi = pstr_azi_ele[ptr_out_ch_format[i]]->ls_azimuth -
              pstr_azi_ele[ptr_out_ch_format[channel]]->ls_azimuth;
        ele = pstr_azi_ele[ptr_out_ch_format[i]]->ls_elevation -
              pstr_azi_ele[ptr_out_ch_format[channel]]->ls_elevation;
        azi = (FLOAT32)azi * (FLOAT32)M_PI / 180.0f;
        ele = (FLOAT32)ele * (FLOAT32)M_PI / 180.0f;
        angle = (FLOAT32)acos(cos(azi) * cos(ele)) * 180.0f / (FLOAT32)M_PI;
        if (fabs(angle) < 15)
        {
          invalid_order = 1;
          break;
        }
      }
      channel = 0;
      while (ia_ele_ordering[channel][0] >= 0)
      {
        if (invalid_order == 1)
          break;
        FLOAT32 ele_1, ele_2, ele_3;
        WORD32 count;
        ele_1 = -1e20f;
        ele_2 = 1e20f;
        ele_3 = 1e20f;
        count = 0;
        for (i = 0; i < out_chn_num; i++)
        {
          if ((ia_ele_ordering[channel][0] == ptr_out_ch_format[i]) &&
              (ptr_out_ch_format[i] != CH_EMPTY))
          {
            ele_1 = pstr_azi_ele[ptr_out_ch_format[i]]->ls_elevation;
            count++;
          }
          if ((ia_ele_ordering[channel][1] == ptr_out_ch_format[i]) &&
              (ptr_out_ch_format[i] != CH_EMPTY))
          {
            ele_2 = pstr_azi_ele[ptr_out_ch_format[i]]->ls_elevation;
            count++;
          }
          if ((ia_ele_ordering[channel][2] == ptr_out_ch_format[i]) &&
              (ptr_out_ch_format[i] != CH_EMPTY))
          {
            ele_3 = pstr_azi_ele[ptr_out_ch_format[i]]->ls_elevation;
            count++;
          }
        }
        if (ele_2 == 1e20f) /* ensure correct ordering if ele_2 has not been set */
          ele_2 = 0.5f * (ele_1 + ele_3);
        if ((count > 1) && !((ele_1 < ele_2) && (ele_2 < ele_3)))
        {
          invalid_order = 1;
          break;
        }
        channel++;
      }
    }
  }
  if ((invalid_order == 1) || (ref_spk_layout->spk_layout_type == 0))
  {
    for (i = 0; i < USED_CHANNEL; i++)
    {
      name_chn = ia_channel_names[i];
      pstr_azi_ele[name_chn] = (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[name_chn];
    }
    ref_spk_layout->spk_layout_type = 0;
  }
  if (ref_spk_layout->spk_layout_type == 1)
  {
    for (i = 0; i < out_chn_num; i++)
    {
      pstr_azi_ele[i] =
          (ia_cicp_ls_geo_str *)&ia_cicp_ls_geo_tbls[ref_spk_layout->cicp_spk_idx[i]];
    }
  }
  channel = 0;
  for (i = 0; i < (UWORD32)num_in_ch; i++)
  {
    input_channel_id = ptr_in_ch_format[i];
    output_channel_idx =
        impeghd_fc_find_channel(input_channel_id, out_chn_num, ptr_out_ch_format);
    if (input_channel_id == CH_EMPTY)
    {
      ptr_io_src[channel] = i;
      ptr_io_gain[channel] = 0.0f;
      ptr_io_dst[channel] = 0;
      ptr_io_proc[channel] = 0;
      channel++;
      continue;
    }
    if (output_channel_idx < 0)
    {
      start_chn = channel;
      k = 0;
      while (ia_dmx_rules_fc[k][0] != -1)
      {

        if (ia_dmx_rules_fc[k][0] == input_channel_id)
        {
          if (ia_dmx_rules_fc[k][4] == RULE_TOP2ALLM) // Not Entering IF statement
          {
            num_chanl = 0;
            for (chnl_u = CH_M_000; chnl_u <= CH_M_180; chnl_u++)
            {
              if (impeghd_fc_find_channel(chnl_u, out_chn_num, ptr_out_ch_format) != -1)
                num_chanl++;
            }
            if (num_chanl > 0) // Max value can be 9
            {
              gain = 1.0f / (FLOAT32)ia_sqrt_flt(num_chanl);
              for (chnl_u = CH_M_000; chnl_u <= CH_M_180; chnl_u++)
              {
                ptr_io_src[channel] = i;
                ptr_io_dst[channel] =
                    impeghd_fc_find_channel(chnl_u, out_chn_num, ptr_out_ch_format);
                ptr_io_gain[channel] =
                    ia_mul_flt(ia_mul_flt(0.01f, (FLOAT32)ia_dmx_rules_fc[k][3]), gain);
                ptr_io_proc[channel] = ia_dmx_rules_fc[k][7];
                if (ptr_io_dst[channel] != -1)
                  channel++;
              }
              break;
            }
            k++;
            continue;
          }
          if (ia_dmx_rules_fc[k][4] == RULE_TOP2ALLU) // Not entering the IF statement
          {
            num_chanl = 0;
            for (chnl_u = CH_U_000; chnl_u <= CH_U_180; chnl_u++)
            {
              if (impeghd_fc_find_channel(chnl_u, out_chn_num, ptr_out_ch_format) != -1)
                num_chanl++;
            }
            if (num_chanl > 0) // Max value can be 9
            {
              gain = 1.0f / (FLOAT32)ia_sqrt_flt(num_chanl);
              for (chnl_u = CH_U_000; chnl_u <= CH_U_180; chnl_u++)
              {
                ptr_io_src[channel] = i;
                ptr_io_dst[channel] =
                    impeghd_fc_find_channel(chnl_u, out_chn_num, ptr_out_ch_format);
                ptr_io_gain[channel] =
                    ia_mul_flt(ia_mul_flt(0.01f, (FLOAT32)ia_dmx_rules_fc[k][3]), gain);
                ptr_io_proc[channel] = ia_dmx_rules_fc[k][7];
                if (ptr_io_dst[channel] != -1)
                  channel++;
              }
              break;
            }
            k++;
            continue;
          }

          idx = impeghd_fc_find_channel(ia_dmx_rules_fc[k][1], out_chn_num, ptr_out_ch_format);
          idx2 = idx;
          if (ia_dmx_rules_fc[k][2] != -1)
          {
            idx2 = impeghd_fc_find_channel(ia_dmx_rules_fc[k][2], out_chn_num, ptr_out_ch_format);
          }
          if ((idx >= 0) && (idx2 >= 0))
          {
            if ((ia_dmx_rules_fc[k][4] == RULE_PANNING) ||
                (ia_dmx_rules_fc[k][4] == RULE_AUTOPAN))
            {
              azi_b = pstr_azi_ele[ptr_out_ch_format[idx2]]->ls_azimuth;
              azi_a = pstr_azi_ele[ptr_out_ch_format[idx]]->ls_azimuth;

              azi_target = pstr_azi_ele[input_channel_id]->ls_azimuth;
              mn = (FLOAT32)ia_min_flt((FLOAT32)ia_min_flt(azi_a, azi_b), azi_target);
              azi_b = ia_sub_flt(azi_b, mn);
              azi_a = ia_sub_flt(azi_a, mn);

              azi_target = ia_sub_flt(azi_target, mn);
              if (ia_dmx_rules_fc[k][4] == RULE_AUTOPAN)
              {
                center = ia_mul_flt(0.5f, ia_add_flt(azi_a, azi_b));
                alpha0 = ia_mul_flt(0.5f, (FLOAT32)ia_fabs_flt(ia_sub_flt(azi_a, azi_b)));
                alpha1 = ia_sub_flt(center, azi_target);
                if (azi_a > azi_b)
                {
                  alpha1 = -alpha1;
                }
              }
              else if (ia_dmx_rules_fc[k][4] == RULE_PANNING)
              {
                alpha1 = (FLOAT32)ia_dmx_rules_fc[k][6];
                alpha0 = (FLOAT32)ia_dmx_rules_fc[k][5];
              }
              alpha1 = ia_mul_flt(alpha1, (FLOAT32)M_PI / 180.0f);
              alpha0 = ia_mul_flt(alpha0, (FLOAT32)M_PI / 180.0f);
              a2 = 1.0f;
              a1 = ia_mul_flt(a2,
                              (ia_add_flt((FLOAT32)tan(alpha0), (FLOAT32)tan(alpha1)) + 1e-10f)) /
                   (ia_sub_flt((FLOAT32)tan(alpha0), (FLOAT32)tan(alpha1)) + 1e-10f);

              nrm = 1.0f / (FLOAT32)ia_sqrt_flt(a1 * a1 + a2 * a2);
              a2 = ia_mul_flt(a2, nrm);
              a1 = ia_mul_flt(a1, nrm);
              ptr_io_dst[channel] = idx;
              ptr_io_src[channel] = i;
              ptr_io_proc[channel] = ia_dmx_rules_fc[k][7];
              ptr_io_gain[channel] =
                  ia_mul_flt(ia_mul_flt(0.01f, (FLOAT32)ia_dmx_rules_fc[k][3]), a1);

              channel = channel + 1;
              ptr_io_src[channel] = i;
              ptr_io_gain[channel] = 0.01f * ia_dmx_rules_fc[k][3] * a2;
              ptr_io_dst[channel] = idx2;

              ptr_io_proc[channel] = ia_dmx_rules_fc[k][7];
              channel = channel + 1;
            }
            else
            {
              ptr_io_src[channel] = i;
              ptr_io_gain[channel] = ia_mul_flt(0.01f, (FLOAT32)ia_dmx_rules_fc[k][3]);
              ptr_io_dst[channel] = idx;
              ptr_io_proc[channel] = ia_dmx_rules_fc[k][7];
              channel = channel + 1;
              if (ia_dmx_rules_fc[k][2] != -1)
              {
                idx = impeghd_fc_find_channel(ia_dmx_rules_fc[k][2], out_chn_num,
                                              ptr_out_ch_format);
                if (idx != -1) // Not entering IF
                {

                  ptr_io_gain[channel] = ia_mul_flt(0.01f, (FLOAT32)ia_dmx_rules_fc[k][3]);
                  ptr_io_dst[channel] = idx;
                  ptr_io_src[channel] = i;
                  ptr_io_proc[channel] = ia_dmx_rules_fc[k][7];
                  channel = channel + 1;
                }
              }
            }
            break;
          }
        }
        k++;
      }
      if (start_chn == channel)
      {
        return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_CHANNEL_NUM;
      }
    }
    else
    {
      ptr_io_src[channel] = i;
      ptr_io_dst[channel] = output_channel_idx;
      ptr_io_gain[channel] = 1.0f;
      ptr_io_proc[channel] = 0;
      channel++;
    }
  }
  ptr_io_gain[channel] = -1;
  ptr_io_src[channel] = -1;
  ptr_io_proc[channel] = -1;
  ptr_io_dst[channel] = -1;
  ptr = 0;
  for (i = 0; i < out_chn_num; i++)
  {
    channel = 0;
    while (ptr_io_src[channel] >= 0)
    {
      if (i == ptr_io_dst[channel])
      {
        pstr_params->in_out_gain[ptr] = ptr_io_gain[channel];
        pstr_params->in_out_dst[ptr] = ptr_io_dst[channel];
        pstr_params->in_out_proc[ptr] = ptr_io_proc[channel];
        pstr_params->in_out_src[ptr] = ptr_io_src[channel];
        ptr++;
      }
      channel++;
    }
  }
  if (channel != ptr)
  {
    exit(-1);
  }
  if (ref_spk_layout->spk_layout_type == 2)
  {
    ptr_io_src[channel] = -1;
    ptr_io_dst[channel] = -1;
    ptr_io_gain[channel] = -1;
    ptr_io_proc[channel] = -1;
    WORD32 rand_eq_index = RULE_REQ - 1;
    i = 0;
    while (ptr_io_src[i] >= 0)
    {
      /* test if horizontal outformat channel */
      if (impeghd_m_channel(ptr_out_ch_format[ptr_io_dst[i]]))
      {
        /* test if this channel is now height */
        if ((pstr_azi_ele[ptr_out_ch_format[ptr_io_dst[i]]]->ls_elevation > 0.0f) &&
            (pstr_azi_ele[ptr_out_ch_format[ptr_io_dst[i]]]->ls_elevation <= 60.0f))
        {
          /* gain/EQ fading constant ("degree of height") */
          FLOAT32 nrm_height;
          nrm_height =
              (FLOAT32)fmin(pstr_azi_ele[ptr_out_ch_format[ptr_io_dst[i]]]->ls_elevation, 35.0f) /
              35.0f;
          /* test if corresponding informat channel is height channel */
          if (impeghd_u_channel(ptr_in_ch_format[ptr_io_src[i]]))
          {
            /* as height increases interpolate to gain 0dB and no eq */
            /* gain compensation by modification of dmx matrix */
            ptr_io_gain[i] =
                nrm_height * ptr_io_gain[i] / (0.01f * 85) + (1.0f - nrm_height) * ptr_io_gain[i];
            if (nrm_height == 1.0)
            {
              ptr_io_proc[i] = RULE_NOPROC;
            }
            else
            {
              impeghd_format_conv_compute_eq_dev(sampling_freq, pstr_params, RULE_NOPROC,
                                                 ptr_io_proc[i], nrm_height, 1 - nrm_height);
              // compute_eq(bands_nrm, nbands, sfreq_Hz, RULE_NOPROC, nrm_height,
              // params->ptr_io_proc[i], 1.0f - nrm_height, params->eq[rand_eq_index]);
              ptr_io_proc[i] = rand_eq_index;
              rand_eq_index++;
            }
          }
          else if (impeghd_m_channel(ptr_in_ch_format[ptr_io_src[i]]))
          {
            /* set EQ to EQ5 */
            /* as height increases interpolated current eq to eq5 */
            if (nrm_height == 1.0)
            {
              ptr_io_proc[i] = RULE_EQ5;
            }
            else
            {
              impeghd_format_conv_compute_eq_dev(sampling_freq, pstr_params, RULE_EQ5,
                                                 ptr_io_proc[i], nrm_height, 1 - nrm_height);
              // compute_eq(bands_nrm, nbands, sfreq_Hz, RULE_EQ5, nrm_height,
              // params->ptr_io_proc[i], 1.0f - nrm_height, params->eq[rand_eq_index]);
              ptr_io_proc[i] = rand_eq_index;
              rand_eq_index++;
            }
          }
        }
      }
      i++;
    }
  }
  pstr_params->in_out_gain[channel] = -1;
  pstr_params->in_out_src[channel] = -1;
  pstr_params->in_out_proc[channel] = -1;
  pstr_params->in_out_dst[channel] = -1;

  err_code = impeghd_fc_eq_compute(sampling_freq, pstr_params);
  if (err_code)
    return err_code;

  /* downmix matrix */

  i = 0;

  while (ptr_src_params[i] >= 0)
  {

    FLOAT32 param_gain = ptr_gain_params[i];
    WORD32 d = ptr_dst_params[i];
    WORD32 s = ptr_src_params[i];

    params->downmix_mat[s][d] = param_gain;
    i++;
  }

  i = 0;

  while (ptr_src_params[i] >= 0)
  {
    FLOAT32 param_gain = ptr_gain_params[i];
    WORD32 d = ptr_dst_params[i];
    WORD32 s = ptr_src_params[i];
    if (ptr_proc_params[i] <= 0)
    {
      for (j = 0; j < FC_BANDS; j++)
      {
        params->downmix_mat_freq[j][s][d] = param_gain;
      }
    }
    else
    {
      FLOAT32 *ptr_eq = params->data_state->eq[ptr_proc_params[i] - 1];
      j = 0;
      for (erb = 0; erb < FC_ERB_BANDS; erb++)
      {
        while (j < ia_erb_idx_freq_fc[erb])
        {
          params->downmix_mat_freq[j][s][d] = ia_mul_flt(ptr_eq[erb], param_gain);
          j++;
        }
      }
      params->downmix_mat_freq[j][s][d] = ia_mul_flt(ptr_eq[erb - 1], param_gain);
    }
    i++;
  }
  return (0);
}

/**
 *  impeghd_format_conv_post_proc_dmx_mtx
 *
 *  \brief Post process gains to Downmix matrix
 *  \param [in,out] params        pointer to format converter param structure
 *  \return IA_ERRORCODE          Error
 *
 */

IA_ERRORCODE impeghd_format_conv_post_proc_dmx_mtx(ia_format_conv_param *params)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 num_in_chan = params->num_in_ch;
  WORD32 num_out_chan = params->num_out_ch;
  WORD32 in_ch, out_ch, band;

  FLOAT32 energy, mod_energy, norm_fac, max_gain, threshold = 0.3f;

  for (in_ch = 0; in_ch < num_in_chan; in_ch++)
  {
    max_gain = 0;
    for (out_ch = 0; out_ch < num_out_chan; out_ch++)
    {
      max_gain = ia_max_flt(max_gain, params->downmix_mat[in_ch][out_ch]);
    }
    if (ia_lteq_flt(max_gain, threshold))
      continue;

    energy = 0;
    for (out_ch = 0; out_ch < num_out_chan; out_ch++)
    {
      energy = ia_mac_flt(energy, params->downmix_mat[in_ch][out_ch],
                          params->downmix_mat[in_ch][out_ch]);
    }

    for (out_ch = 0; out_ch < num_out_chan; out_ch++)
    {
      if (ia_lteq_flt(params->downmix_mat[in_ch][out_ch], threshold))
      {
        params->downmix_mat[in_ch][out_ch] = 0;
      }
    }

    mod_energy = 0;
    for (out_ch = 0; out_ch < num_out_chan; out_ch++)
    {
      mod_energy = ia_mac_flt(mod_energy, params->downmix_mat[in_ch][out_ch],
                              params->downmix_mat[in_ch][out_ch]);
    }

    if (mod_energy != 0)
    {
      norm_fac = (FLOAT32)ia_sqrt_flt(energy / mod_energy);
    }
    else
    {
      norm_fac = 1.0f;
    }
    for (out_ch = 0; out_ch < num_out_chan; out_ch++)
    {
      params->downmix_mat[in_ch][out_ch] = params->downmix_mat[in_ch][out_ch] * norm_fac;
      for (band = 0; band < FC_BANDS; band++)
      {
        params->downmix_mat_freq[band][in_ch][out_ch] = params->downmix_mat[in_ch][out_ch];
      }
    }
  }
  return err_code;
}

/**
 *  impeghd_format_conv_map_lfes
 *
 *  \brief Post process gains to Downmix matrix
 *  \param [in,out] params                pointer to format converter param structure
 *  \param [in]     pp_cicp_in_geometry   Pointer input channel configuration
 *  \param [in]     pp_cicp_out_geometry  Pointer input channel configuration
 *  \return IA_ERRORCODE          Error
 *
 */
IA_ERRORCODE impeghd_format_conv_map_lfes(ia_format_conv_param *params,
                                          const ia_cicp_ls_geo_str **pp_cicp_in_geometry,
                                          const ia_cicp_ls_geo_str **pp_cicp_out_geometry)
{
  WORD32 out_ch, in_ch, ch_idx;
  WORD32 num_tgt_lfes = 0;

  WORD32 tgt_lfe_ch_idxs[MAX_NUM_CHANNELS];

  for (out_ch = 0; out_ch < params->num_out_ch; out_ch++)
  {
    if (1 == pp_cicp_out_geometry[out_ch]->lfe_flag)
    {
      tgt_lfe_ch_idxs[num_tgt_lfes] = out_ch;
      num_tgt_lfes++;
    }
  }

  if (num_tgt_lfes > 0)
  {
    for (in_ch = 0; in_ch < params->num_in_ch; in_ch++)
    {
      if (1 == pp_cicp_in_geometry[in_ch]->lfe_flag)
      {
        /* As there can be a maximum of 1 LFE channel */
        /* We will directly use the channel index of  */
        /* LFE for setting downmix matrix flag        */

        for (ch_idx = 0; ch_idx < params->num_out_ch; ch_idx++)
        {
          params->downmix_mat[in_ch][ch_idx] = 0;
        }
        params->downmix_mat[in_ch][tgt_lfe_ch_idxs[0]] = 1.0f;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of FmtConvInit */