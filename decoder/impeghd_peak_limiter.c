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

#include <impeghd_type_def.h>
#include "ia_core_coder_cnst.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_peak_limiter_struct_def.h"

/**
 * @defgroup PeakLmtProc Peak Limiter Processing
 * @ingroup  PeakLmtProc
 * @brief Peak Limiter Processing
 *
 * @{
 */

/**
*  impeghd_cpy_delay_samples
*
*  \brief copy to delay samples buffer
*
*  \param [in] frame_size frame size
*  \param [in] delay_in_idx index to delay input
*  \param [in] channel_num Channel number
*  \param [in] ptr_delayed_in pointer to delay input buffer
*  \param [out] ptr_samples pointer to sample buffer
*  \param [in] attack_time_samples attack constant
*  \return VOID
*
*/
static VOID impeghd_cpy_delay_samples(UWORD32 frame_size, UWORD32 *delay_in_idx,
                                      UWORD32 channel_num, FLOAT32 *ptr_delayed_in,
                                      FLOAT32 *ptr_samples, UWORD32 attack_time_samples)
{
  FLOAT32 tmp;
  UWORD32 i, j;
  for (i = 0; i < frame_size; i++)
  {

    for (j = 0; j < channel_num; j++)
    {
      tmp = ptr_delayed_in[*delay_in_idx * channel_num + j];
      ptr_delayed_in[*delay_in_idx * channel_num + j] = ptr_samples[i * channel_num + j];
      ptr_samples[i * channel_num + j] = tmp;
    }

    (*delay_in_idx)++;
    if (*delay_in_idx >= attack_time_samples)
      *delay_in_idx = 0;
  }

  return;
}

/**
*  impeghd_cpy_delay_samples_with_gain
*
*  \brief copy to  samples buffer
*  \param [in] i index to frame length
*  \param [in] delay_in_idx index to delay input
*  \param [in] channel_num Channel number
*  \param [in] ptr_delayed_in pointer to delay input buffer
*  \param [out] ptr_samples pointer to sample buffer
*  \param [in] attack_time_samples attack constant
*  \param [in] limit_threshold limit threshold value
*  \return VOID
*
*/
static VOID impeghd_cpy_delay_samples_with_gain(UWORD32 i, UWORD32 *delay_in_idx,
                                                UWORD32 channel_num, FLOAT32 gain,
                                                FLOAT32 *ptr_delayed_in, FLOAT32 *ptr_samples,
                                                UWORD32 attack_time_samples,
                                                FLOAT32 limit_threshold)
{
  FLOAT32 tmp;
  UWORD32 j;

  for (j = 0; j < channel_num; j++)
  {
    // qfactor = 15;
    tmp = ptr_delayed_in[*delay_in_idx * channel_num + j];
    ptr_delayed_in[*delay_in_idx * channel_num + j] = ptr_samples[i * channel_num + j];
    tmp = ia_mul_flt(tmp, gain);
    if (ia_lt_flt(limit_threshold, tmp))
      tmp = limit_threshold;
    else if (ia_lt_flt(tmp, ia_negate_flt(limit_threshold)))
      tmp = ia_negate_flt(limit_threshold);

    ptr_samples[i * channel_num + j] = tmp;
  }

  (*delay_in_idx)++;
  if (*delay_in_idx >= attack_time_samples)
    *delay_in_idx = 0;
  return;
}

/**
*  impeghd_peak_limiter_init
*
*  \brief Peak Limiter initialization
*
*  \param [in/out] pstr_peak_limiter Pointer to peak_limiter struct
*  \param [in] channel_num Number of ouptut channels
*  \param [in] sample_rate Sampling rate value
*  \param [in] ptr_buff Peak limiter buffer of size PEAK_LIM_SIZE
*
*  \return IA_ERRORCODE
*
*/
IA_ERRORCODE impeghd_peak_limiter_init(ia_peak_limiter_struct *pstr_peak_limiter,
                                       UWORD32 channel_num, UWORD32 sample_rate,
                                       FLOAT32 *ptr_buff)
{
  UWORD32 attack_time;

  attack_time = (UWORD32)(DEFAULT_ATTACK_TIME_MS * sample_rate / 1000);

  if (attack_time < 1)
    return IA_MPEGH_DEC_NO_ERROR;

  pstr_peak_limiter->max_buf = ptr_buff;
  pstr_peak_limiter->max_idx = 0;
  pstr_peak_limiter->cir_buf_pnt = 0;
  pstr_peak_limiter->delayed_input = ptr_buff + 1 + attack_time;
  pstr_peak_limiter->limit_threshold = ia_mul_flt(LIM_DEFAULT_THRESHOLD, 32768.0f);
  pstr_peak_limiter->num_channels = channel_num;
  pstr_peak_limiter->sample_rate = sample_rate;
  pstr_peak_limiter->min_gain = 1.0f;
  pstr_peak_limiter->limiter_on = 1;
  pstr_peak_limiter->pre_smoothed_gain = 1.0f;
  pstr_peak_limiter->gain_modified = 1.0f;
  pstr_peak_limiter->delayed_input_index = 0;
  pstr_peak_limiter->attack_time = DEFAULT_ATTACK_TIME_MS;
  pstr_peak_limiter->release_time = DEFAULT_RELEASE_TIME_MS;
  pstr_peak_limiter->attack_time_samples = attack_time;
  pstr_peak_limiter->attack_constant = (FLOAT32)pow(0.1, 1.0 / (attack_time + 1));
  pstr_peak_limiter->release_constant =
      (FLOAT32)pow(0.1, 1.0 / (DEFAULT_RELEASE_TIME_MS * sample_rate / 1000 + 1));

  return IA_MPEGH_DEC_NO_ERROR;
}
/**
*  impeghd_peak_limiter_process
*
*  \brief Peak Limiter process
*
*  \param [in/out] pstr_peak_limiter
*  \param [in] ptr_samples
*  \param [in] frame_size
*
*  \return VOID
*
*/
VOID impeghd_peak_limiter_process(ia_peak_limiter_struct *pstr_peak_limiter, FLOAT32 *ptr_samples,
                                  UWORD32 frame_size)

{
  UWORD32 i, j;
  UWORD32 channel_num = pstr_peak_limiter->num_channels;
  UWORD32 attack_time_samples = pstr_peak_limiter->attack_time_samples;
  UWORD32 delay_in_idx = pstr_peak_limiter->delayed_input_index;
  FLOAT32 maximum, tmp, gain, min_gain = 1;
  FLOAT32 attack_constant = pstr_peak_limiter->attack_constant;
  FLOAT32 release_constant = pstr_peak_limiter->release_constant;
  FLOAT32 limit_threshold = pstr_peak_limiter->limit_threshold;
  FLOAT32 *ptr_max_buf = pstr_peak_limiter->max_buf;
  FLOAT32 gain_modified = pstr_peak_limiter->gain_modified;
  FLOAT32 *ptr_delayed_in = pstr_peak_limiter->delayed_input;
  FLOAT32 pre_smoothed_gain = pstr_peak_limiter->pre_smoothed_gain;
  if (!pstr_peak_limiter->limiter_on && ((FLOAT32)pre_smoothed_gain >= 1.0f))
  {
    impeghd_cpy_delay_samples(frame_size, &delay_in_idx, channel_num, ptr_delayed_in, ptr_samples,
                              attack_time_samples);
  }
  else
  {
    for (i = 0; i < frame_size; i++)
    {
      tmp = 0.0f;
      for (j = 0; j < channel_num; j++)
      {
        tmp = ia_max_flt(tmp, (FLOAT32)ia_fabs_flt(ptr_samples[i * channel_num + j]));
      }
      ptr_max_buf[pstr_peak_limiter->cir_buf_pnt] = tmp;
      if (pstr_peak_limiter->max_idx == pstr_peak_limiter->cir_buf_pnt)
      {
        pstr_peak_limiter->max_idx = 0;
        for (j = 1; j < (attack_time_samples + 1); j++)
        {
          if (ia_lt_flt(ptr_max_buf[pstr_peak_limiter->max_idx], ptr_max_buf[j]))
            pstr_peak_limiter->max_idx = j;
        }
      }
      else if (ia_lteq_flt(ptr_max_buf[pstr_peak_limiter->max_idx], tmp))
      {
        pstr_peak_limiter->max_idx = pstr_peak_limiter->cir_buf_pnt;
      }

      pstr_peak_limiter->cir_buf_pnt++;
      if (pstr_peak_limiter->cir_buf_pnt == (attack_time_samples + 1))
        pstr_peak_limiter->cir_buf_pnt = 0;

      maximum = ptr_max_buf[pstr_peak_limiter->max_idx];

      if (limit_threshold >= maximum)
      {
        gain = 1;
      }
      else
      {
        gain = limit_threshold / maximum;
      }
      if (ia_lt_flt(gain, pre_smoothed_gain))
      {
        gain_modified = ia_min_flt(
            gain_modified,
            ia_mul_flt(ia_msu_flt(gain, 0.1f, (FLOAT32)pre_smoothed_gain), 1.11111111f));
      }
      else
      {
        gain_modified = gain;
      }
      if (gain_modified >= pre_smoothed_gain)
      {
        pre_smoothed_gain = ia_mac_flt(gain_modified, release_constant,
                                       ia_sub_flt(pre_smoothed_gain, gain_modified));
      }
      else
      {
        pre_smoothed_gain = ia_mac_flt(gain_modified, attack_constant,
                                       ia_sub_flt(pre_smoothed_gain, gain_modified));
        pre_smoothed_gain = ia_max_flt(pre_smoothed_gain, gain);
      }
      gain = (FLOAT32)pre_smoothed_gain;
      impeghd_cpy_delay_samples_with_gain(i, &delay_in_idx, channel_num, gain, ptr_delayed_in,
                                          ptr_samples, attack_time_samples, limit_threshold);

      if (ia_lt_flt(gain, min_gain))
        min_gain = gain;
    }
  }
  pstr_peak_limiter->gain_modified = gain_modified;
  pstr_peak_limiter->delayed_input_index = delay_in_idx;
  pstr_peak_limiter->pre_smoothed_gain = pre_smoothed_gain;
  pstr_peak_limiter->min_gain = min_gain;

  return;
}
/** @} */ /* End of PeakLmtProc */