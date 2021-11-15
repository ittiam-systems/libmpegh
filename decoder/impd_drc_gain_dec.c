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
#include "impd_drc_gain_dec.h"
#include "impd_drc_multi_band.h"
#include "impd_drc_process_audio.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup DRCProcessing DRCProcessing
 * @ingroup  DRCProcessing
 * @brief DRC Decoder processing
 *
 * @{
 */

/**
 *  impd_drc_conv_gain_db_to_linear_domain
 *
 *  \brief Convert gain and slope to linear domain
 *
 *  \param [out]  out_gain_lin   output linear gain
 *  \param [out]  out_slope_lin   output linear slope
 *  \param [in]  pstr_interp_params   Pointer to interpolation param structure
 *  \param [in]  drc_band   drc band count
 *  \param [in]  in_gain_db   input gain in db
 *  \param [in]  in_slope_db   input slope in db
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE
impd_drc_conv_gain_db_to_linear_domain(FLOAT32 *out_gain_lin, FLOAT32 *out_slope_lin,
                                       ia_drc_interp_params_struct *pstr_interp_params,
                                       WORD32 drc_band, FLOAT32 in_gain_db, FLOAT32 in_slope_db)

{
  ia_drc_gain_modifiers_struct *pstr_gain_modifiers = pstr_interp_params->pstr_gain_modifiers;
  FLOAT32 gain_ratio = 1.0;

  if (pstr_interp_params->gain_modification_flag)
  {
    if (in_gain_db >= 0.0f)
    {
      gain_ratio = ia_mul_flt(gain_ratio, pstr_interp_params->boost);
    }
    else
    {
      gain_ratio = ia_mul_flt(gain_ratio, pstr_interp_params->compress);
    }
  }

  if (1 == pstr_gain_modifiers->gain_scaling_flag[drc_band])
  {
    if (in_gain_db >= 0.0)
    {
      gain_ratio = ia_mul_flt(gain_ratio, pstr_gain_modifiers->amplfn_scale_factor[drc_band]);
    }
    else
    {
      gain_ratio = ia_mul_flt(gain_ratio, pstr_gain_modifiers->attn_scale_factor[drc_band]);
    }
  }

  if ((1 == pstr_interp_params->ducking_flag) &&
      (1 == pstr_interp_params->pstr_ducking_modifiers->ducking_scaling_flag))
  {
    gain_ratio =
        ia_mul_flt(gain_ratio, pstr_interp_params->pstr_ducking_modifiers->ducking_scaling);
  }

  {
    *out_gain_lin = (FLOAT32)pow(2.0, (FLOAT64)(ia_mul_flt(gain_ratio, in_gain_db / 6.0f)));

    *out_slope_lin = ia_mul_flt(ia_mul_flt(SLOPE_FACTOR_DB_TO_LINEAR, gain_ratio),
                                ia_mul_flt(*out_gain_lin, in_slope_db));

    if (1 == pstr_gain_modifiers->gain_offset_flag[drc_band])
    {
      *out_gain_lin *=
          (FLOAT32)pow(2.0, (FLOAT64)(pstr_gain_modifiers->gain_offset[drc_band] / 6.0f));
    }

    if ((1 == pstr_interp_params->clipping_flag) &&
        (1 == pstr_interp_params->limiter_peak_target_present))
    {
      *out_gain_lin *= (FLOAT32)pow(2.0,
                                    ia_max_flt(0.0,
                                               -pstr_interp_params->limiter_peak_target -
                                                   pstr_interp_params->loud_norm_gain_db) /
                                        6.0f);

      if (*out_gain_lin >= 1.0f)
      {
        *out_gain_lin = 1.0f;
        *out_slope_lin = 0;
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_interpolate_drc_gain
 *
 *  \brief Interpolate drc gain
 *
 *  \param [out]  interp_out   interpolated output
 *  \param [in]  pstr_interp_params   Pointer to interpolation param structure
 *  \param [in]  drc_band   drc band count
 *  \param [in]  gain_step_td   gain step time domain
 *  \param [in]  gain_prev   previous gain
 *  \param [in]  gain_cur   gain
 *  \param [in]  slope_prev  previous slope
 *  \param [in]  slope_cur  slope
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_interpolate_drc_gain(FLOAT32 *interp_out,
                                                  ia_drc_interp_params_struct *pstr_interp_params,
                                                  WORD32 drc_band, WORD32 gain_step_td,
                                                  FLOAT32 gain_prev, FLOAT32 gain_cur,
                                                  FLOAT32 slope_prev, FLOAT32 slope_cur)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  FLOAT32 a;
  FLOAT32 slope_prev_lin, slope_cur_lin;
  FLOAT32 gain_prev_lin, gain_cur_lin;
  FLOAT32 node_insert_float;
  WORD32 node_insert;
  WORD32 node;
  WORD32 cubic_interpoln = 1;

  if (gain_step_td <= 0)
  {
    return (IA_MPEGD_DRC_INIT_NONFATAL_UNEXPECTED_ERROR);
  }

  err_code = impd_drc_conv_gain_db_to_linear_domain(
      &gain_prev_lin, &slope_prev_lin, pstr_interp_params, drc_band, gain_prev, slope_prev);
  if (err_code)
    return (err_code);
  err_code = impd_drc_conv_gain_db_to_linear_domain(
      &gain_cur_lin, &slope_cur_lin, pstr_interp_params, drc_band, gain_cur, slope_cur);
  if (err_code)
    return (err_code);

  if (IA_DRC_GAIN_INTERPOLATION_TYPE_SPLINE == pstr_interp_params->gain_interp_type)
  {
    slope_prev_lin = slope_prev_lin / (FLOAT32)pstr_interp_params->delta_tmin;
    slope_cur_lin = slope_cur_lin / (FLOAT32)pstr_interp_params->delta_tmin;

    if ((FLOAT32)fabs((FLOAT64)slope_prev_lin) > (FLOAT32)fabs((FLOAT64)slope_cur_lin))
    {
      node_insert_float = ia_mul_flt(
          2.0f,
          (ia_sub_flt(ia_sub_flt(gain_cur_lin, gain_prev_lin), (slope_cur_lin * gain_step_td))) /
              (slope_prev_lin - slope_cur_lin));
      node_insert = (WORD32)(0.5f + node_insert_float);

      if ((node_insert >= 0) && (node_insert < gain_step_td))
      {
        cubic_interpoln = 0;

        interp_out[0] = gain_prev_lin;
        interp_out[gain_step_td] = gain_cur_lin;

        a = ia_mul_flt(0.5f, ia_sub_flt(slope_cur_lin, slope_prev_lin) / node_insert_float);
        for (node = 1; node < node_insert; node++)
        {
          interp_out[node] =
              ia_add_flt(ia_mul_flt((ia_add_flt(ia_mul_flt(a, (FLOAT32)node), slope_prev_lin)),
                                    (FLOAT32)node),
                         gain_prev_lin);
          interp_out[node] = ia_max_flt(0.0f, interp_out[node]);
        }
        for (; node < gain_step_td; node++)
        {
          interp_out[node] =
              ia_add_flt(ia_mul_flt(slope_cur_lin, (FLOAT32)(node - gain_step_td)), gain_cur_lin);
        }
      }
    }

    if (cubic_interpoln == 1)
    {
      FLOAT32 k1, k2, band;
      FLOAT32 gain_step_inv = 1.0f / (FLOAT32)gain_step_td;
      FLOAT32 gain_step_inv_sq = ia_mul_flt(gain_step_inv, gain_step_inv);

      k1 = ia_mul_flt(ia_sub_flt(gain_cur_lin, gain_prev_lin), gain_step_inv_sq);
      k2 = ia_add_flt(slope_cur_lin, slope_prev_lin);

      a = ia_mul_flt(gain_step_inv,
                     (ia_sub_flt(ia_mul_flt(gain_step_inv, k2), ia_mul_flt(2.0f, k1))));
      band = ia_sub_flt(ia_mul_flt(3.0f, k1),
                        ia_mul_flt(gain_step_inv, ia_add_flt(k2, slope_prev_lin)));

      interp_out[0] = gain_prev_lin;
      interp_out[gain_step_td] = gain_cur_lin;

      for (node = 1; node < gain_step_td; node++)
      {
        interp_out[node] = ia_add_flt(
            ia_mul_flt(ia_add_flt(ia_mul_flt(ia_add_flt(ia_mul_flt(a, (FLOAT32)node), band),
                                             (FLOAT32)node),
                                  slope_prev_lin),
                       (FLOAT32)node),
            gain_prev_lin);
        interp_out[node] = ia_max_flt(0.0f, interp_out[node]);
      }
    }
  }
  else if (fabs((FLOAT64)slope_prev_lin) < fabs((FLOAT64)slope_cur_lin))
  {
    node_insert_float = ia_mul_flt(
        2.0f,
        (ia_add_flt(ia_sub_flt(gain_prev_lin, gain_cur_lin), (slope_prev_lin * gain_step_td))) /
            ia_sub_flt(slope_prev_lin, slope_cur_lin));
    node_insert_float = gain_step_td - node_insert_float;
    node_insert = (WORD32)(0.5f + node_insert_float);

    if ((node_insert >= 0) && (node_insert < gain_step_td))
    {

      interp_out[0] = gain_prev_lin;
      interp_out[gain_step_td] = gain_cur_lin;

      for (node = 1; node < node_insert; node++)
      {
        interp_out[node] = ia_add_flt(ia_mul_flt(slope_prev_lin, (FLOAT32)node), gain_prev_lin);
      }
      a = ia_sub_flt(slope_cur_lin, slope_prev_lin) / (2.0f * (gain_step_td - node_insert_float));

      for (; node < gain_step_td; node++)
      {
        interp_out[node] =
            ia_add_flt(ia_mul_flt((ia_add_flt(ia_mul_flt(a, (FLOAT32)(gain_step_td - node)),
                                              (-slope_cur_lin))),
                                  (FLOAT32)(gain_step_td - node)),
                       gain_cur_lin);
        interp_out[node] = ia_max_flt(0.0f, interp_out[node]);
      }
    }
  }
  else
  {

    a = ia_sub_flt(gain_cur_lin, gain_prev_lin) / (FLOAT32)gain_step_td;
    interp_out[0] = gain_prev_lin;
    interp_out[gain_step_td] = gain_cur_lin;
    for (node = 1; node < gain_step_td; node++)
    {
      interp_out[node] = ia_add_flt(ia_mul_flt(a, (FLOAT32)node), gain_prev_lin);
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_advance_buf
 *
 *  \brief Advance interpolation buffer
 *
 *  \param [in/out]  pstr_gain_buf   Poiter to gain buffer structure
 *  \param [in]  frame_size   framesize
 *
 *  \return VOID
 *
 */
static VOID impd_drc_advance_buf(ia_drc_gain_buffer_struct *pstr_gain_buf, WORD32 frame_size)
{
  ia_drc_interp_buf_struct *pstr_buf_interp;
  WORD32 cnt;

  for (cnt = pstr_gain_buf->buf_interp_cnt - 1; cnt >= 0; cnt--)
  {
    pstr_buf_interp = &(pstr_gain_buf->pstr_buf_interp[cnt]);
    pstr_buf_interp->prev_node = pstr_buf_interp->str_node;
    pstr_buf_interp->prev_node.time -= frame_size;
    memmove(pstr_buf_interp->lpcm_gains, pstr_buf_interp->lpcm_gains + frame_size,
            sizeof(FLOAT32) * (frame_size + MAX_SIGNAL_DELAY));
  }
  return;
}

/**
 *  impd_drc_concatenate_segments
 *
 *  \brief Concatenate interpolation segments
 *
 *  \param [in/out]  pstr_buf_interp   Poiter to buffer interpolation structure
 *  \param [in]  frame_size   framesize
 *  \param [in]  drc_band   drc band count
 *  \param [in]  pstr_interp_params   Pointer to interpolation param structure
 *  \param [in]  pstr_spline_nodes   Pointer to spline nodes structure
 *
 *  \return IA_ERRORCODE error
 *
 */
static IA_ERRORCODE impd_drc_concatenate_segments(ia_drc_interp_buf_struct *pstr_buf_interp,
                                                  WORD32 frame_size, WORD32 drc_band,
                                                  ia_drc_interp_params_struct *pstr_interp_params,
                                                  ia_drc_spline_nodes_struct *pstr_spline_nodes)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 time_prev, duration_seg, node;
  FLOAT32 loc_gain_db = 0.0f, prev_gain_db, slope = 0.0f, slope_prev;

  prev_gain_db = pstr_buf_interp->prev_node.loc_gain_db;
  slope_prev = pstr_buf_interp->prev_node.slope;
  time_prev = pstr_buf_interp->prev_node.time;

  for (node = 0; node < pstr_spline_nodes->no_of_nodes; node++)
  {
    duration_seg = pstr_spline_nodes->str_node[node].time - time_prev;
    loc_gain_db = pstr_spline_nodes->str_node[node].loc_gain_db;
    slope = pstr_spline_nodes->str_node[node].slope;

    err_code = impd_drc_interpolate_drc_gain(
        pstr_buf_interp->lpcm_gains + MAX_SIGNAL_DELAY + frame_size + time_prev,
        pstr_interp_params, drc_band, duration_seg, prev_gain_db, loc_gain_db, slope_prev, slope);
    if (err_code)
      return (err_code);

    prev_gain_db = loc_gain_db;
    slope_prev = slope;
    time_prev = pstr_spline_nodes->str_node[node].time;
  }

  pstr_buf_interp->str_node.loc_gain_db = loc_gain_db;
  pstr_buf_interp->str_node.slope = slope;
  pstr_buf_interp->str_node.time = time_prev;
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_get_gain
 *
 *  \brief Get gain function
 *
 *  \param [in/out]  pstr_drc_gain_buffers   Poiter to drc gain buffers
 *  \param [in]  pstr_drc_gain_dec   Pointer to drc gain decode structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  pstr_drc_gain   Pointer to drc gain structure
 *  \param [in]  compress_fac   compression factor
 *  \param [in]  boost_fac   boost factor
 *  \param [in]  characteristic_idx   characteristic index
 *  \param [in]  ln_gain_db   loudness normalization gain in db
 *  \param [in]  sel_drc_index   drc index
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE
impd_drc_get_gain(ia_drc_gain_buffers_struct *pstr_drc_gain_buffers,
                  ia_drc_gain_dec_struct *pstr_drc_gain_dec, ia_drc_config *pstr_drc_config,
                  ia_drc_gain_struct *pstr_drc_gain, FLOAT32 compress_fac, FLOAT32 boost_fac,
                  WORD32 characteristic_idx, FLOAT32 ln_gain_db, WORD32 sel_drc_index)
{
  ia_drc_params_struct *pstr_ia_drc_params_struct = &(pstr_drc_gain_dec->ia_drc_params_struct);
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 drc_instrns_idx =
      pstr_ia_drc_params_struct->sel_drc_array[sel_drc_index].drc_instrns_idx;

  if (drc_instrns_idx >= 0)
  {
    ia_drc_interp_params_struct str_interp_params = {0};
    ia_drc_instructions_struct *pstr_drc_instruction_str =
        &(pstr_drc_config->str_drc_instruction_str[drc_instrns_idx]);

    WORD32 drc_set_eff = pstr_drc_instruction_str->drc_set_effect;
    WORD32 gain_element_idx;
    WORD32 num_drc_ch_grps = pstr_drc_instruction_str->num_drc_ch_groups;

    str_interp_params.boost = boost_fac;
    str_interp_params.characteristic_idx = characteristic_idx;
    str_interp_params.compress = compress_fac;
    str_interp_params.limiter_peak_target = pstr_drc_instruction_str->limiter_peak_target;
    str_interp_params.limiter_peak_target_present =
        pstr_drc_instruction_str->limiter_peak_target_present;
    str_interp_params.loud_norm_gain_db = ln_gain_db;

    if ((IA_DRC_EFFECT_BIT_FADE != drc_set_eff) &&
        (((IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF) & drc_set_eff) == 0) &&
        (IA_DRC_EFFECT_BIT_CLIPPING != drc_set_eff))
    {
      str_interp_params.gain_modification_flag = 1;
    }
    else
    {
      str_interp_params.gain_modification_flag = 0;
    }
    if ((IA_DRC_EFFECT_BIT_DUCK_OTHER | IA_DRC_EFFECT_BIT_DUCK_SELF) & drc_set_eff)
    {
      str_interp_params.ducking_flag = 1;
    }
    else
    {
      str_interp_params.ducking_flag = 0;
    }
    if (IA_DRC_EFFECT_BIT_CLIPPING == drc_set_eff)
    {
      str_interp_params.clipping_flag = 1;
    }
    else
    {
      str_interp_params.clipping_flag = 0;
    }

    impd_drc_advance_buf(&(pstr_drc_gain_buffers->pstr_gain_buf[sel_drc_index]),
                         pstr_ia_drc_params_struct->drc_frame_size);

    gain_element_idx = 0;
    for (WORD32 grp = 0; grp < num_drc_ch_grps; grp++)
    {
      WORD32 gainSet = pstr_drc_instruction_str->gain_set_index_for_channel_group[grp];
      WORD32 num_drc_bands = pstr_drc_instruction_str->band_count_of_ch_group[grp];

      str_interp_params.delta_tmin =
          pstr_drc_instruction_str->time_delta_min_for_channel_group[grp];
      str_interp_params.gain_interp_type =
          pstr_drc_instruction_str->gain_interpolation_type_for_channel_group[grp];
      str_interp_params.pstr_ducking_modifiers =
          &(pstr_drc_instruction_str->str_ducking_modifiers_for_channel_group[grp]);
      str_interp_params.pstr_gain_modifiers =
          &(pstr_drc_instruction_str->str_gain_modifiers_of_ch_group[grp]);

      for (WORD32 band = 0; band < num_drc_bands; band++)
      {
        err_code = impd_drc_concatenate_segments(
            &(pstr_drc_gain_buffers->pstr_gain_buf[sel_drc_index]
                  .pstr_buf_interp[gain_element_idx]),
            pstr_ia_drc_params_struct->drc_frame_size, band, &str_interp_params,
            &(pstr_drc_gain->drc_gain_sequence[gainSet].str_spline_nodes[band]));
        if (err_code)
          return (err_code);
        gain_element_idx++;
      }
    }
  }
  return err_code;
}
/** @} */ /* End of DRCProcessing */
