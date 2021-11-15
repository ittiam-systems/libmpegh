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
#include <string.h>

#include <impeghd_type_def.h>
#include "impeghd_intrinsics_flt.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_com.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_avq_rom.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_func_def.h"

#define F_PIT_SHARP 0.85F

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_acelp_pitch_sharpening
 *
 *  \brief Pitch sharpening function.
 *
 *  \param [in/out] x       Pointer to input buffer.
 *  \param [in]     pit_lag Pitch lag value.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_acelp_pitch_sharpening(FLOAT32 *x, WORD32 pit_lag)
{
  WORD32 idx;
  for (idx = pit_lag; idx < LEN_SUBFR; idx++)
  {
    x[idx] = ia_mac_flt(x[idx], x[idx - pit_lag], F_PIT_SHARP);
  }
  return;
}

/**
 *  ia_core_coder_acelp_decode_1sp_per_track
 *
 *  \brief Function for Decoding one signed pulse per track
 *
 *  \param [in]  idx_1p       Index of signed pulse.
 *  \param [in]  M            Number of bits per track.
 *  \param [in]  drc_offset   Offset.
 *  \param [in]  track        Track index.
 *  \param [out] ptr_code_vec Pointer to code vector.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_acelp_decode_1sp_per_track(WORD32 idx_1p, WORD32 M, WORD32 drc_offset,
                                                     WORD32 track, FLOAT32 *ptr_code_vec)
{
  WORD32 sign_index, mask, m, sp_pos;
  mask = ((1 << M) - 1);
  sp_pos = (idx_1p & mask) + drc_offset;
  sign_index = ((idx_1p >> M) & 1);
  m = (sp_pos << 2) + track;

  ptr_code_vec[m] =
      (sign_index == 1) ? ia_sub_flt(ptr_code_vec[m], 1.0f) : ia_add_flt(ptr_code_vec[m], 1.0f);
  return;
}

/**
 *  ia_core_coder_acelp_decode_2sp_per_track
 *
 *  \brief Function for Decoding two signed pulse per track
 *
 *  \param [in]  idx_2p       Index of signed pulse.
 *  \param [in]  M            Number of bits per track.
 *  \param [in]  drc_offset   Offset.
 *  \param [in]  track        Track index.
 *  \param [out] ptr_code_vec Pointer to code vector.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_acelp_decode_2sp_per_track(WORD32 idx_2p, WORD32 M, WORD32 drc_offset,
                                                     WORD32 track, FLOAT32 *ptr_code_vec)
{
  WORD32 sign_index, mask, m0, m1;
  WORD32 sp_pos[2];

  mask = ((1 << M) - 1);
  sp_pos[0] = (((idx_2p >> M) & mask) + drc_offset);
  sp_pos[1] = ((idx_2p & mask) + drc_offset);
  sign_index = (idx_2p >> 2 * M) & 1;

  m0 = (sp_pos[0] << 2) + track;
  m1 = (sp_pos[1] << 2) + track;

  if ((sp_pos[1] - sp_pos[0]) >= 0)
  {
    if (sign_index != 1)
    {
      ptr_code_vec[m0] = ia_add_flt(ptr_code_vec[m0], 1.0f);
      ptr_code_vec[m1] = ia_add_flt(ptr_code_vec[m1], 1.0f);
    }
    else
    {
      ptr_code_vec[m0] = ia_sub_flt(ptr_code_vec[m0], 1.0f);
      ptr_code_vec[m1] = ia_sub_flt(ptr_code_vec[m1], 1.0f);
    }
  }
  else
  {
    if (sign_index != 1)
    {
      ptr_code_vec[m0] = ia_add_flt(ptr_code_vec[m0], 1.0f);
      ptr_code_vec[m1] = ia_sub_flt(ptr_code_vec[m1], 1.0f);
    }
    else
    {
      ptr_code_vec[m0] = ia_sub_flt(ptr_code_vec[m0], 1.0f);
      ptr_code_vec[m1] = ia_add_flt(ptr_code_vec[m1], 1.0f);
    }
  }
  return;
}

/**
 *  ia_core_coder_acelp_decode_3sp_per_track
 *
 *  \brief Function for Decoding three signed pulses per track
 *
 *  \param [in]  idx_3p       Index of signed pulse.
 *  \param [in]  M            Number of bits per track.
 *  \param [in]  drc_offset   Offset.
 *  \param [in]  track        Track index.
 *  \param [out] ptr_code_vec Pointer to code vector.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_acelp_decode_3sp_per_track(WORD32 idx_3p, WORD32 M, WORD32 drc_offset,
                                                     WORD32 track, FLOAT32 *ptr_code_vec)
{
  WORD32 j, mask, idx_2p, idx_1p;

  mask = ((1 << (2 * M - 1)) - 1);
  idx_2p = idx_3p & mask;
  j = drc_offset;
  if (((idx_3p >> ((2 * M) - 1)) & 1) == 1)
  {
    j += (1 << (M - 1));
  }
  ia_core_coder_acelp_decode_2sp_per_track(idx_2p, M - 1, j, track, ptr_code_vec);
  mask = ((1 << (M + 1)) - 1);
  idx_1p = (idx_3p >> 2 * M) & mask;
  ia_core_coder_acelp_decode_1sp_per_track(idx_1p, M, drc_offset, track, ptr_code_vec);
  return;
}

/**
 *  ia_core_coder_d_acelp_decode_4sp_per_track_section
 *
 *  \brief Function for Decoding four signed pulses per track
 *
 *  \param [in]  index        Index of signed pulse.
 *  \param [in]  drc_offset   Offset.
 *  \param [in]  track        Track index.
 *  \param [out] ptr_code_vec Pointer to code vector.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_d_acelp_decode_4sp_per_track_section(WORD32 index, WORD32 drc_offset,
                                                               WORD32 track,
                                                               FLOAT32 *ptr_code_vec)
{
  WORD32 j, idx_2p;

  idx_2p = index & 31;
  j = drc_offset;
  if (((index >> 5) & 1) == 1)
  {
    j += 4;
  }
  ia_core_coder_acelp_decode_2sp_per_track(idx_2p, 2, j, track, ptr_code_vec);
  idx_2p = (index >> 6) & 127;
  ia_core_coder_acelp_decode_2sp_per_track(idx_2p, 3, drc_offset, track, ptr_code_vec);
  return;
}

/**
 *  ia_core_coder_acelp_decode_4sp_per_track
 *
 *  \brief Function for Decoding four signed pulses per track
 *
 *  \param [in]  idx_4p       Index of signed pulse.
 *  \param [in]  track        Track index.
 *  \param [out] ptr_code_vec Pointer to code vector.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_acelp_decode_4sp_per_track(WORD32 idx_4p, WORD32 track,
                                                     FLOAT32 *ptr_code_vec)
{
  WORD32 idx_1p, idx_2p, idx_3p;

  switch ((idx_4p >> 14) & 3)
  {
  case 0:
    if (((idx_4p >> 13) & 1) != 0)
    {
      ia_core_coder_d_acelp_decode_4sp_per_track_section(idx_4p, 8, track, ptr_code_vec);
    }
    else
    {
      ia_core_coder_d_acelp_decode_4sp_per_track_section(idx_4p, 0, track, ptr_code_vec);
    }
    break;
  case 1:
    idx_1p = idx_4p >> 10;
    ia_core_coder_acelp_decode_1sp_per_track(idx_1p, 3, 0, track, ptr_code_vec);
    ia_core_coder_acelp_decode_3sp_per_track(idx_4p, 3, 8, track, ptr_code_vec);
    break;
  case 2:
    idx_2p = idx_4p >> 7;
    ia_core_coder_acelp_decode_2sp_per_track(idx_2p, 3, 0, track, ptr_code_vec);
    ia_core_coder_acelp_decode_2sp_per_track(idx_4p, 3, 8, track, ptr_code_vec);
    break;
  case 3:
    idx_3p = idx_4p >> 4;
    ia_core_coder_acelp_decode_3sp_per_track(idx_3p, 3, 0, track, ptr_code_vec);
    ia_core_coder_acelp_decode_1sp_per_track(idx_4p, 3, 8, track, ptr_code_vec);
    break;
  }
  return;
}

/**
 *  ia_core_coder_d_acelp_add_pulse
 *
 *  \brief Adds processing based on signed bit information.
 *
 *  \param [in]  ptr_pos      Pointer to position buffer.
 *  \param [in]  nb_pulse     Number of pulses.
 *  \param [in]  track        Track value.
 *  \param [out] ptr_code_vec Pointer to code vector.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_d_acelp_add_pulse(WORD32 *ptr_pos, WORD32 nb_pulse, WORD32 track,
                                            FLOAT32 *ptr_code_vec)
{
  WORD32 idx, idx1;
  for (idx = 0; idx < nb_pulse; idx++)
  {
    idx1 = ((ptr_pos[idx] & (16 - 1)) << 2) + track;
    ptr_code_vec[idx1] = ((ptr_pos[idx] & 16) == 0) ? ia_add_flt(ptr_code_vec[idx1], 1.0f)
                                                    : ia_sub_flt(ptr_code_vec[idx1], 1.0f);
  }
  return;
}

/**
 *  ia_core_coder_d_acelp_decode_1p_n1
 *
 *  \brief Helper function for decoding single pulse per track.
 *
 *  \param [in]  index      Index Value.
 *  \param [in]  N          Number of bits.
 *  \param [in]  drc_offset Offset value.
 *  \param [out] ptr_pos    Pointer to position values.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_d_acelp_decode_1p_n1(WORD32 index, WORD32 N, WORD32 drc_offset,
                                               WORD32 *ptr_pos)
{
  WORD32 idx, pos1, mask;

  mask = ((1 << N) - 1);
  pos1 = ((index & mask) + drc_offset);
  idx = ((index >> N) & 1);
  if (idx == 1)
  {
    pos1 += 16;
  }
  ptr_pos[0] = pos1;
  return;
}

/**
 *  ia_core_coder_acelp_decode_pulses_per_track
 *
 *  \brief Decodes pulses per track information.
 *
 *  \param [in]  ptr_cb_index   Pointer to codebook index.
 *  \param [in]  code_bits      Code bits.
 *  \param [out] ptr_code_vec   Pointer to code vector.
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_acelp_decode_pulses_per_track(WORD32 *ptr_cb_index, const WORD16 code_bits,
                                                 FLOAT32 *ptr_code_vec)
{
  WORD32 track_idx, index, drc_offset, pos[6], idx;
  ia_core_coder_memset(ptr_code_vec, 64);

  switch (code_bits)
  {
  case 12:
    for (track_idx = 0; track_idx < 4; track_idx += 2)
    {
      drc_offset = ptr_cb_index[2 * (track_idx >> 1)];
      index = ptr_cb_index[2 * (track_idx >> 1) + 1];
      ia_core_coder_d_acelp_decode_1p_n1(index, 4, 0, pos);
      ia_core_coder_d_acelp_add_pulse(pos, 1, 2 * drc_offset + (track_idx >> 1), ptr_code_vec);
    }
    break;
  case 16:
    idx = 0;
    drc_offset = ptr_cb_index[idx++];
    drc_offset = (drc_offset == 0) ? 1 : 3;
    for (track_idx = 0; track_idx < 4; track_idx++)
    {
      if (track_idx != drc_offset)
      {
        index = ptr_cb_index[idx++];
        ia_core_coder_d_acelp_decode_1p_n1(index, 4, 0, pos);
        ia_core_coder_d_acelp_add_pulse(pos, 1, track_idx, ptr_code_vec);
      }
    }
    break;
  case 20:
    for (track_idx = 0; track_idx < 4; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_1sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    break;
  case 28:
    for (track_idx = 0; track_idx < 2; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_2sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    for (track_idx = 2; track_idx < 4; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_1sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    break;
  case 36:
    for (track_idx = 0; track_idx < 4; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_2sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    break;
  case 44:
    for (track_idx = 0; track_idx < 2; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_3sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    for (track_idx = 2; track_idx < 4; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_2sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    break;
  case 52:
    for (track_idx = 0; track_idx < 4; track_idx++)
    {
      index = ptr_cb_index[track_idx];
      ia_core_coder_acelp_decode_3sp_per_track(index, 4, 0, track_idx, ptr_code_vec);
    }
    break;
  case 64:
    for (track_idx = 0; track_idx < 4; track_idx++)
    {
      index = ((ptr_cb_index[track_idx] << 14) + ptr_cb_index[track_idx + 4]);
      ia_core_coder_acelp_decode_4sp_per_track(index, track_idx, ptr_code_vec);
    }
    break;
  default:
    break;
  }
  return;
}

/**
 *  ia_core_coder_acelp_decode_gains
 *
 *  \brief Decodes adaptive and innovative code book gains.
 *
 *  \param [in]  index             Index value.
 *  \param [in]  ptr_code_vec      Pointer to Code vector.
 *  \param [out] ptr_pitch_gain    Pointer to gain buffer.
 *  \param [out] ptr_codebook_gain Pointer to code book gain values.
 *  \param [in]  mean_exc_energy   Mean excitation energy.
 *  \param [out] ptr_energy        Pointer to average innovation energy computed.
 *
 *  \return VOID
 *
 */
static void ia_core_coder_acelp_decode_gains(WORD32 index, FLOAT32 *ptr_code_vec,
                                             FLOAT32 *ptr_pitch_gain, FLOAT32 *ptr_codebook_gain,
                                             FLOAT32 mean_exc_energy, FLOAT32 *ptr_energy)
{
  WORD32 idx;
  FLOAT32 avg_innov_energy = 0.01f, est_gain;
  const FLOAT32 *gain_table = ia_core_coder_int_leave_gain_table;

  for (idx = 0; idx < LEN_SUBFR; idx++)
  {
    avg_innov_energy = ia_mac_flt(avg_innov_energy, ptr_code_vec[idx], ptr_code_vec[idx]);
  }
  *ptr_energy = avg_innov_energy;
  avg_innov_energy =
      (FLOAT32)ia_mul_flt(10.0, (FLOAT32)log10(avg_innov_energy / (FLOAT32)LEN_SUBFR));
  est_gain = ia_sub_flt(mean_exc_energy, avg_innov_energy);
  est_gain = (FLOAT32)pow(10.0, ia_mul_double_flt(0.05, est_gain));
  *ptr_pitch_gain = gain_table[index * 2];
  *ptr_codebook_gain = ia_mul_flt(gain_table[index * 2 + 1], est_gain);

  return;
}

/**
 *  ia_core_coder_cb_exc_calc
 *
 *  \brief Codebook excitation calculation.
 *
 *  \param [out] ptr_xcitation_curr Pointer to current excitation vector.
 *  \param [in]  pitch_lag          Pitch lag value.
 *  \param [in]  frac               Fraction value.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_cb_exc_calc(FLOAT32 *ptr_xcitation_curr, WORD32 pitch_lag, WORD32 frac)
{
  WORD32 idx1, idx2;
  FLOAT32 s, *x0, *x1, *x2;
  const FLOAT32 *c1, *c2;

  x0 = &ptr_xcitation_curr[-pitch_lag];
  frac = -frac;
  if (frac < 0)
  {
    frac += UP_SAMP;
    x0--;
  }
  for (idx1 = 0; idx1 < LEN_SUBFR + 1; idx1++)
  {
    x1 = x0++;
    x2 = x1 + 1;
    c1 = &ia_core_coder_interpol_filt[frac];
    c2 = &ia_core_coder_interpol_filt[UP_SAMP - frac];
    s = 0.0;
    for (idx2 = 0; idx2 < INTER_LP_FIL_ORDER; idx2++, c1 += UP_SAMP, c2 += UP_SAMP)
    {
      s = ia_mac_flt(ia_mac_flt(s, (*x1--), (*c1)), (*x2++), (*c2));
    }
    ptr_xcitation_curr[idx1] = s;
  }
  return;
}

/**
 *  ia_core_coder_acelp_alias_cnx
 *
 *  \brief Forward alias cancellation
 *
 *  \param [in/out] ptr_usac_data      Pointer to USAC data structure.
 *  \param [in/out] pstr_td_frame_data Pointer to TD frame data.
 *  \param [in]     k                  Index value.
 *  \param [in/out] lp_filt_coeff      Pointer to filter coefficients.
 *  \param [in]     stability_factor   Factor value used by processing algorithm.
 *  \param [in/out] ptr_st             Pointer to LPD decoder state structure.
 *  \param [in/out] ptr_td_config      Pointer to TD config structure.
 *  \param [in/out] ptr_synth_fb       Pointer to synthesis buffer.
 *  \param [in/out] ptr_pitch_buffer   Pointer to pitch buffer.
 *  \param [in/out] ptr_voice_factors  Pointer to voice factors buffer.
 *  \param [in]     ptr_temp_scratch   Pointer to scratch buffer used for processing.
 *
 *  \return IA_ERRORCODE                     Processing error if any.
 *
 */
IA_ERRORCODE ia_core_coder_acelp_alias_cnx(ia_usac_data_struct *ptr_usac_data,
                                           ia_td_frame_data_struct *pstr_td_frame_data, WORD32 k,
                                           FLOAT32 *lp_filt_coeff, FLOAT32 stability_factor,
                                           ia_usac_lpd_decoder_handle ptr_st,
                                           ia_usac_td_config_handle ptr_td_config,
                                           FLOAT32 *ptr_synth_fb, FLOAT32 *ptr_pitch_buffer,
                                           FLOAT32 *ptr_voice_factors, FLOAT32 *ptr_temp_scratch)
{
  WORD32 idx, subfr_idx;
  WORD32 pitch_lag, pitch_lag_frac, index, pitch_flag, pitch_lag_max;
  WORD32 pitch_lag_min = 0;
  FLOAT32 tmp, pitch_gain, gain_code, voicing_factor, r_v, innov_energy, pitch_energy,
      mean_ener_code;
  FLOAT32 gain_smooth, gain_code0, cpe;
  FLOAT32 gain_smooth_factor;
  FLOAT32 *ptr_lp_filt_coeff;
  WORD32 pitch_min;
  WORD32 pitch_fr2;
  WORD32 pitch_fr1;
  WORD32 pitch_max;
  WORD32 subfr_nb = 0;
  const WORD16 num_codebits_table[8] = {20, 28, 36, 44, 52, 64, 12, 16};
  WORD32 len_residual;
  WORD32 len_subfr = ptr_td_config->len_subfrm;
  WORD32 fac_length_fb = 0, delay_fb = 0, delay = 0;
  FLOAT32 err_bwe = 0.0f;

  FLOAT32 *code = ptr_temp_scratch;
  FLOAT32 *synth_temp = code + LEN_SUBFR;
  FLOAT32 *post_process_exc = synth_temp + FAC_LENGTH + 16;
  FLOAT32 *x = post_process_exc + LEN_SUBFR;
  FLOAT32 *xn2 = x + FAC_LENGTH;
  WORD32 *int_x = (WORD32 *)(xn2 + 2 * FAC_LENGTH + 16);
  FLOAT32 *xn2_fb = (FLOAT32 *)int_x + FAC_LENGTH;
  FLOAT32 *xn2_tmp =
      xn2_fb + 2 * 2 * FAC_LENGTH + 2 * L_FILT_MAX; //[FAC_LENGTH + 2 * L_FILT_MAX];
  FLOAT32 *fft_scratch = xn2_tmp + (FAC_LENGTH + 2 * L_FILT_MAX);
  WORD32 fac_length;
  FLOAT32 *ptr_scratch = &ptr_usac_data->scratch_buffer_float[0];
  WORD32 loop_count = 0;
  WORD32 core_mode = pstr_td_frame_data->acelp_core_mode;
  FLOAT32 *synth_signal =
      &ptr_usac_data
           ->synth_buf[len_subfr * k + MAX_PITCH +
                       (((ptr_td_config->num_frame * ptr_td_config->num_subfrm) >> 1) - 1) *
                           LEN_SUBFR];
  FLOAT32 *xcitation_curr =
      &ptr_usac_data->exc_buf[len_subfr * k + MAX_PITCH + (INTER_LP_FIL_ORDER + 1)];
  FLOAT32 *ptr_pitch_gain =
      &ptr_usac_data
           ->pitch_gain[k * ptr_td_config->num_subfrm +
                        (((ptr_td_config->num_frame * ptr_td_config->num_subfrm) >> 1) - 1)];
  WORD32 *ptr_pitch =
      &ptr_usac_data->pitch[k * ptr_td_config->num_subfrm +
                            (((ptr_td_config->num_frame * ptr_td_config->num_subfrm) >> 1) - 1)];
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  fac_length = len_subfr >> 1;

  if (ptr_td_config->full_band_lpd)
  {
    fac_length_fb = ptr_td_config->fac_fb * fac_length;
    delay_fb = ptr_st->td_resamp_delay;
    delay = delay_fb / ptr_td_config->fac_fb;
  }
  if (ptr_st->mode_prev > 0)
  {
    for (idx = 0; idx < fac_length; idx++)
    {
      x[idx] =
          ia_mul_flt(ptr_st->fac_gain, (FLOAT32)pstr_td_frame_data->fac[k * FAC_LENGTH + idx]);
    }
    for (idx = 0; idx < (fac_length >> 2); idx++)
    {
      x[idx] = ia_mul_flt(x[idx], ptr_st->fac_fd_data[idx]);
    }

    err = ia_core_coder_acelp_mdct(x, xn2 + fac_length, fac_length, ptr_scratch, fft_scratch);
    if (err != IA_MPEGH_DEC_NO_ERROR)
      return err;

    ia_core_coder_vec_cnst_mul((2.0f / (FLOAT32)fac_length), xn2 + fac_length, xn2 + fac_length,
                               fac_length);

    ia_core_coder_memset(xn2, fac_length);

    ia_core_coder_lpc_wt_synthesis_tool(ptr_st->lp_flt_coeff_a_prev, xn2 + fac_length,
                                        fac_length);

    if (ptr_td_config->full_band_lpd)
    {
      /* resample FAC for FB */
      ia_core_coder_memset(xn2_fb, ptr_td_config->fac_fb * 2 * FAC_LENGTH * delay_fb);
      ia_core_coder_mem_cpy(xn2 + fac_length, xn2_tmp, fac_length);
      ia_core_coder_memset(xn2_tmp + fac_length, delay);

      ia_core_coder_td_resampler(xn2_tmp, (WORD16)fac_length, ptr_td_config->fscale,
                                 xn2_fb + fac_length_fb, ptr_td_config->fscale_full_band, NULL, 0,
                                 ptr_temp_scratch);
    }
    for (idx = 0; idx < 2 * fac_length; idx++)
    {
      xn2[idx] = ia_add_flt(xn2[idx], synth_signal[idx - (2 * fac_length)]);
    }

    if (ptr_td_config->full_band_lpd)
    {
      for (idx = 0; idx < 2 * fac_length_fb; idx++)
      {
        xn2_fb[idx] = ia_add_flt(xn2_fb[idx], ptr_synth_fb[idx - (2 * fac_length_fb)]);
      }
    }

    ia_core_coder_mem_cpy(xn2 + fac_length, synth_signal - fac_length, fac_length);

    if (ptr_td_config->full_band_lpd)
    {
      /* update past synthesis for FB */
      ia_core_coder_mem_cpy(xn2_fb + fac_length_fb + delay_fb,
                            ptr_synth_fb - fac_length_fb + delay_fb, fac_length_fb - delay_fb);

      /* fade at transition (necessary due to memoryless resampling */
      for (idx = 0; idx < delay_fb; idx++)
      {
        ptr_synth_fb[-fac_length_fb + idx] =
            ia_mac_flt(ia_mul_flt(((FLOAT32)(delay_fb - idx) / (FLOAT32)delay_fb),
                                  ptr_synth_fb[-fac_length_fb + idx]),
                       ((FLOAT32)idx / (FLOAT32)delay_fb), xn2_fb[fac_length_fb + idx]);
      }
    }

    tmp = 0.0;
    ia_core_coder_preemphsis_tool(xn2, PREEMPH_FILT_FAC, 2 * fac_length, tmp);

    ptr_lp_filt_coeff = ptr_st->lp_flt_coeff_a_prev;
    len_residual = fac_length % LEN_SUBFR;
    if (len_residual != 0)
    {
      ia_core_coder_residual_tool(ptr_lp_filt_coeff, &xn2[fac_length],
                                  &xcitation_curr[fac_length - (2 * fac_length)], len_residual, 1,
                                  17);
      ptr_lp_filt_coeff += (ORDER + 1);
    }

    loop_count = (fac_length + len_residual) / LEN_SUBFR;
    ia_core_coder_residual_tool(ptr_lp_filt_coeff, &xn2[fac_length + len_residual],
                                &xcitation_curr[len_residual - fac_length], LEN_SUBFR, loop_count,
                                17);
  }

  for (idx = 0; idx < ORDER; idx++)
  {
    synth_temp[idx] =
        ia_msu_flt(synth_signal[idx - ORDER], PREEMPH_FILT_FAC, synth_signal[idx - ORDER - 1]);
  }

  idx = (((ptr_td_config->fscale * TMIN) + (FSCALE_DENOM >> 1)) / FSCALE_DENOM) - TMIN;
  pitch_min = TMIN + idx;
  pitch_fr2 = TFR2 - idx;
  pitch_fr1 = TFR1;
  pitch_max = TMAX + (6 * idx);

  mean_ener_code = ia_mac_flt(18.0f, ((FLOAT32)pstr_td_frame_data->mean_energy[k]), 12.0f);

  ptr_lp_filt_coeff = lp_filt_coeff;
  for (subfr_idx = 0; subfr_idx < len_subfr; subfr_idx += LEN_SUBFR)
  {
    pitch_flag = subfr_idx;
    if ((len_subfr == 256) && (subfr_idx == (2 * LEN_SUBFR)))
    {
      pitch_flag = 0;
    }
    index = pstr_td_frame_data->acb_index[k * 4 + subfr_nb];

    if (pitch_flag != 0)
    {
      pitch_lag = pitch_lag_min + (index >> 2);
      pitch_lag_frac = index - (pitch_lag - pitch_lag_min) * 4;
    }
    else
    {
      if (index < (pitch_fr2 - pitch_min) * 4)
      {
        pitch_lag = pitch_min + (index >> 2);
        pitch_lag_frac = index - (pitch_lag - pitch_min) * 4;
      }
      else if (index < ((pitch_fr2 - pitch_min) * 4 + (pitch_fr1 - pitch_fr2) * 2))
      {
        index -= (pitch_fr2 - pitch_min) * 4;
        pitch_lag = pitch_fr2 + (index >> 1);
        pitch_lag_frac = index - (pitch_lag - pitch_fr2) * 2;
        pitch_lag_frac *= 2;
      }
      else
      {
        pitch_lag =
            index + pitch_fr1 - ((pitch_fr2 - pitch_min) * 4) - ((pitch_fr1 - pitch_fr2) * 2);
        pitch_lag_frac = 0;
      }
      pitch_lag_min = pitch_lag - 8;
      if (pitch_lag_min < pitch_min)
      {
        pitch_lag_min = pitch_min;
      }

      pitch_lag_max = pitch_lag_min + 15;
      if (pitch_lag_max > pitch_max)
      {
        pitch_lag_max = pitch_max;
        pitch_lag_min = pitch_lag_max - 15;
      }
    }

    if (ptr_td_config->full_band_lpd)
    {
      ia_core_coder_mix_past_exc(&ptr_st->tbe_dec_data, &err_bwe, subfr_idx, pitch_lag,
                                 pitch_lag_frac);
      ptr_pitch_buffer[subfr_idx / LEN_SUBFR] =
          (FLOAT32)ia_add_flt((FLOAT32)pitch_lag, (FLOAT32)(pitch_lag_frac / 4.0));
    }

    ia_core_coder_cb_exc_calc(&xcitation_curr[subfr_idx], pitch_lag, pitch_lag_frac);

    if (pstr_td_frame_data->ltp_filtering_flag[k * 4 + subfr_nb] == 0)
    {
      for (idx = 0; idx < LEN_SUBFR; idx++)
      {
        code[idx] = (FLOAT32)ia_mac_flt(
            ia_mac_flt(ia_mul_flt((FLOAT32)0.18, xcitation_curr[idx - 1 + subfr_idx]),
                       (FLOAT32)0.64, xcitation_curr[idx + subfr_idx]),
            (FLOAT32)0.18, xcitation_curr[idx + 1 + subfr_idx]);
      }

      ia_core_coder_mem_cpy(code, &xcitation_curr[subfr_idx], LEN_SUBFR);
    }

    ia_core_coder_acelp_decode_pulses_per_track(
        &(pstr_td_frame_data->icb_index[k * 4 + subfr_nb][0]), num_codebits_table[core_mode],
        code);

    tmp = 0.0;
    ia_core_coder_preemphsis_tool(code, TILT_CODE, LEN_SUBFR, tmp);
    idx = pitch_lag;
    if (pitch_lag_frac > 2)
    {
      idx++;
    }
    if (idx >= 0)
    {
      ia_core_coder_acelp_pitch_sharpening(code, idx);
    }

    index = pstr_td_frame_data->gains[k * 4 + subfr_nb];

    ia_core_coder_acelp_decode_gains(index, code, &pitch_gain, &gain_code, mean_ener_code,
                                     &innov_energy);

    pitch_energy = 0.0;
    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      pitch_energy = ia_mac_flt(pitch_energy, xcitation_curr[idx + subfr_idx],
                                xcitation_curr[idx + subfr_idx]);
    }

    pitch_energy = ia_mul_flt(pitch_energy, ia_mul_flt(pitch_gain, pitch_gain));
    /* energy of innovative code excitation */
    innov_energy = 0.0;
    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      innov_energy = ia_mac_flt(innov_energy, code[idx], code[idx]);
    }
    innov_energy = ia_mul_flt(innov_energy, ia_mul_flt(gain_code, gain_code));

    r_v = (FLOAT32)(ia_sub_flt(pitch_energy, innov_energy) /
                    ia_add_flt(pitch_energy, ia_add_flt(innov_energy, (FLOAT32)0.01)));

    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      post_process_exc[idx] = ia_mul_flt(pitch_gain, xcitation_curr[idx + subfr_idx]);
      xcitation_curr[idx + subfr_idx] = ia_mac_flt(
          ia_mul_flt(pitch_gain, xcitation_curr[idx + subfr_idx]), gain_code, code[idx]);
    }

    if (ptr_td_config->full_band_lpd)
    {
      ia_core_coder_tbe_prep_exc(&ptr_st->tbe_dec_data, subfr_idx, pitch_gain, gain_code, code,
                                 r_v, &ptr_voice_factors[subfr_idx / LEN_SUBFR]);
    }
    idx = pitch_lag;
    if (pitch_lag_frac > 2)
    {
      idx++;
    }

    if (idx > pitch_max)
    {
      idx = pitch_max;
    }

    *ptr_pitch++ = idx;
    *ptr_pitch_gain++ = pitch_gain;

    voicing_factor = (FLOAT32)ia_mul_flt(0.5, ia_sub_flt(1.0, r_v));
    gain_smooth_factor = ia_mul_flt(stability_factor, voicing_factor);
    gain_code0 = gain_code;
    if (!ia_lt_flt(gain_code0, ptr_st->gain_threshold))
    {
      gain_code0 = (FLOAT32)(gain_code0 / 1.19);
      if (ia_lt_flt(gain_code0, ptr_st->gain_threshold))
      {
        gain_code0 = ptr_st->gain_threshold;
      }
    }
    else
    {
      gain_code0 = (FLOAT32)ia_mul_flt(gain_code0, (FLOAT32)1.19);
      if (ia_lt_flt(ptr_st->gain_threshold, gain_code0))
      {
        gain_code0 = ptr_st->gain_threshold;
      }
    }
    ptr_st->gain_threshold = gain_code0;
    gain_smooth = (FLOAT32)ia_mac_flt(ia_mul_flt(gain_smooth_factor, gain_code0),
                                      ia_sub_flt(1.0, gain_smooth_factor), gain_code);
    for (idx = 0; idx < LEN_SUBFR; idx++)
    {
      code[idx] *= gain_smooth;
    }

    cpe = (FLOAT32)ia_mul_flt(0.125, ia_add_flt(1.0, r_v));

    post_process_exc[0] = ia_add_flt(post_process_exc[0], ia_msu_flt(code[0], cpe, code[1]));

    for (idx = 1; idx < LEN_SUBFR - 1; idx++)
    {
      post_process_exc[idx] =
          ia_add_flt(post_process_exc[idx],
                     ia_msu_flt(code[idx], cpe, ia_add_flt(code[idx - 1], code[idx + 1])));
    }

    post_process_exc[LEN_SUBFR - 1] =
        ia_add_flt(post_process_exc[LEN_SUBFR - 1],
                   ia_msu_flt(code[LEN_SUBFR - 1], cpe, code[LEN_SUBFR - 2]));

    ia_core_coder_synthesis_tool(ptr_lp_filt_coeff, post_process_exc, &synth_signal[subfr_idx],
                                 LEN_SUBFR, synth_temp, ptr_temp_scratch);
    ia_core_coder_mem_cpy(&synth_signal[subfr_idx + LEN_SUBFR - ORDER], synth_temp, ORDER);

    ptr_lp_filt_coeff += (ORDER + 1);
    subfr_nb++;
  }

  ia_core_coder_deemphsis_tool(synth_signal, len_subfr, synth_signal[-1]);

  ia_core_coder_memset(synth_temp + 16, FAC_LENGTH);
  ia_core_coder_synthesis_tool1(ptr_lp_filt_coeff, synth_temp + 16, FAC_LENGTH);

  ptr_lp_filt_coeff -= (2 * (ORDER + 1));
  ia_core_coder_mem_cpy(ptr_lp_filt_coeff, ptr_st->lp_flt_coeff_a_prev, (2 * (ORDER + 1)));
  ia_core_coder_mem_cpy(synth_signal + len_subfr - (1 + fac_length), ptr_st->exc_prev,
                        (1 + fac_length));
  ia_core_coder_mem_cpy(synth_temp + 16, ptr_st->exc_prev + 1 + fac_length, fac_length);
  ia_core_coder_deemphsis_tool(ptr_st->exc_prev + 1 + fac_length, fac_length,
                               synth_signal[len_subfr - 1]);

  return err;
}
/** @} */ /* End of CoreDecProc */