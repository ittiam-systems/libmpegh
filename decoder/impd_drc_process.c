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
 *  impd_drc_apply_gains_and_add
 *
 *  \brief Apply gains to audio frame and add
 *
 *  \param [out] pstr_drc_gain_dec  Pointer to gain dec structure
 *  \param [out] channel_audio  channel audio output
 *  \param [in] pstr_drc_instruction_arr  Pointer to instructions structure
 *  \param [in] impd_apply_gains  apply gain flag
 *  \param [in] sel_drc_idx  drc index
 *
 *  \return VOID
 *
 */
static VOID impd_drc_apply_gains_and_add(ia_drc_gain_dec_struct *pstr_drc_gain_dec,
                                         FLOAT32 *channel_audio[],
                                         ia_drc_instructions_struct *pstr_drc_instruction_arr,
                                         WORD32 impd_apply_gains, WORD32 sel_drc_idx)
{
  ia_drc_params_struct *pstr_ia_drc_params = &pstr_drc_gain_dec->ia_drc_params_struct;
  ia_drc_gain_buffer_struct *pstr_gain_buf =
      &(pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[sel_drc_idx]);

  WORD32 num_ch, num_band, num_ch_grp, fr_size;
  WORD32 offset = 0, sig_idx = 0;
  WORD32 gain_idx_grp[CHANNEL_GROUP_COUNT_MAX];
  WORD32 sig_idx_ch[MAX_CHANNEL_COUNT];
  const WORD32 drc_instrns_idx =
      pstr_drc_gain_dec->ia_drc_params_struct.sel_drc_array[sel_drc_idx].drc_instrns_idx;
  FLOAT32 sum;
  FLOAT32 *lpcm_gain;
  FLOAT32 **deinterleaved_audio = pstr_drc_gain_dec->audio_band_buffer.non_interleaved_audio;
  ia_drc_instructions_struct *pstr_drc_instruction = &(pstr_drc_instruction_arr[drc_instrns_idx]);

  if (0 <= drc_instrns_idx)
  {
    pstr_drc_instruction = &(pstr_drc_instruction_arr[drc_instrns_idx]);
    {
      if (0 < pstr_drc_instruction->drc_set_id)
      {

        if (DELAY_MODE_LOW == pstr_ia_drc_params->delay_mode)
        {
          offset = pstr_ia_drc_params->drc_frame_size;
        }
        gain_idx_grp[0] = 0;

        for (num_ch_grp = 0; num_ch_grp < pstr_drc_instruction->num_drc_ch_groups - 1;
             num_ch_grp++)
        {
          gain_idx_grp[num_ch_grp + 1] =
              gain_idx_grp[num_ch_grp] + pstr_drc_instruction->band_count_of_ch_group[num_ch_grp];
        }

        sig_idx_ch[0] = 0;
        for (num_ch = 0; num_ch < pstr_drc_instruction->audio_num_chan - 1; num_ch++)
        {
          if (pstr_drc_instruction->channel_group_of_ch[num_ch] >= 0)
          {
            sig_idx_ch[num_ch + 1] =
                sig_idx_ch[num_ch] +
                pstr_drc_instruction
                    ->band_count_of_ch_group[pstr_drc_instruction->channel_group_of_ch[num_ch]];
          }
          else
          {
            sig_idx_ch[num_ch + 1] = sig_idx_ch[num_ch] + 1;
          }
        }

        for (num_ch_grp = pstr_drc_instruction->num_drc_ch_groups - 1; num_ch_grp >= 0;
             num_ch_grp--)
        {
          for (num_band = pstr_drc_instruction->band_count_of_ch_group[num_ch_grp] - 1;
               num_band >= 0; num_band--)
          {

            lpcm_gain =
                pstr_gain_buf->pstr_buf_interp[gain_idx_grp[num_ch_grp] + num_band].lpcm_gains +
                MAX_SIGNAL_DELAY - pstr_ia_drc_params->gain_delay_samples + offset;

            {

              if (1 != impd_apply_gains)
              {
                for (num_ch = pstr_ia_drc_params->channel_offset;
                     num_ch <
                     pstr_ia_drc_params->channel_offset + pstr_ia_drc_params->num_ch_process;
                     num_ch++)

                {
                  if (num_ch_grp == pstr_drc_instruction->channel_group_of_ch[num_ch])
                  {
                    sig_idx = sig_idx_ch[num_ch] + num_band;

                    for (fr_size = pstr_ia_drc_params->drc_frame_size - 1; fr_size >= 0;
                         fr_size--)
                    {
                      deinterleaved_audio[sig_idx][fr_size] = (FLOAT32)lpcm_gain[fr_size];
                    }
                  }
                }
              }
              else
              {
                for (num_ch = pstr_ia_drc_params->channel_offset;
                     num_ch <
                     pstr_ia_drc_params->channel_offset + pstr_ia_drc_params->num_ch_process;
                     num_ch++)

                {
                  if (num_ch_grp == pstr_drc_instruction->channel_group_of_ch[num_ch])
                  {
                    sig_idx = sig_idx_ch[num_ch] + num_band;

                    for (fr_size = pstr_ia_drc_params->drc_frame_size - 1; fr_size >= 0;
                         fr_size--)
                    {
                      deinterleaved_audio[sig_idx][fr_size] =
                          ia_mul_flt(deinterleaved_audio[sig_idx][fr_size], lpcm_gain[fr_size]);
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  sig_idx = 0;

  if (0 >= pstr_drc_instruction->drc_set_id)
  {
    for (num_ch = pstr_ia_drc_params->num_ch_process - 1; num_ch >= 0; num_ch--)
    {
      for (fr_size = pstr_ia_drc_params->drc_frame_size - 1; fr_size >= 0; fr_size--)
      {
        channel_audio[num_ch][fr_size] = deinterleaved_audio[num_ch][fr_size];
      }
    }
  }
  else
  {
    for (num_ch = pstr_ia_drc_params->channel_offset;
         num_ch < pstr_ia_drc_params->channel_offset + pstr_ia_drc_params->num_ch_process;
         num_ch++)
    {
      num_ch_grp = pstr_drc_instruction->channel_group_of_ch[num_ch];
      if (num_ch_grp < 0)
      {
        sig_idx++;
      }
      else
      {
        for (fr_size = pstr_ia_drc_params->drc_frame_size - 1; fr_size >= 0; fr_size--)
        {
          sum = 0.0f;
          for (num_band = pstr_drc_instruction->band_count_of_ch_group[num_ch_grp] - 1;
               num_band >= 0; num_band--)
          {
            sum = ia_add_flt(sum, (deinterleaved_audio[sig_idx + num_band][fr_size]));
          }
          channel_audio[num_ch - pstr_ia_drc_params->channel_offset][fr_size] = sum;
        }
        sig_idx += pstr_drc_instruction->band_count_of_ch_group[num_ch_grp];
      }
    }
  }

  return;
}

/**
 *  impd_drc_apply_gains_subband
 *
 *  \brief Apply DRC gains to audio frame
 *
 *  \param [out] deinterleaved_audio_re  audio real part
 *  \param [out] deinterleaved_audio_im  audio imaginary part
 *  \param [in] pstr_drc_instruction_arr  Pointer to instructions structure
 *  \param [in] pstr_drc_gain_dec  Pointer to drc gain structure
 *  \param [in] sel_drc_idx  drc index
 *
*  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE
impd_drc_apply_gains_subband(FLOAT32 *deinterleaved_audio_re[], FLOAT32 *deinterleaved_audio_im[],
                             ia_drc_instructions_struct *pstr_drc_instruction_arr,
                             ia_drc_gain_dec_struct *pstr_drc_gain_dec, WORD32 sel_drc_idx)
{
  ia_drc_instructions_struct *pstr_drc_instruction;
  ia_drc_params_struct *pstr_ia_drc_params = &pstr_drc_gain_dec->ia_drc_params_struct;
  ia_drc_gain_buffer_struct *pstr_gain_buf =
      &(pstr_drc_gain_dec->drc_gain_buffers.pstr_gain_buf[sel_drc_idx]);
  ia_drc_overlap_params_struct *pstr_overlap_params = &pstr_drc_gain_dec->str_overlap_params;

  WORD32 num_ch, num_band, num_ch_grp, size, cnt;
  WORD32 gain_idx_grp[CHANNEL_GROUP_COUNT_MAX];
  WORD32 offset = 0, sig_idx = 0;
  WORD32 drc_frame_size_sb = 0;
  WORD32 num_dec_sb = 0;
  WORD32 ds_fac = 0;
  const WORD32 drc_instrns_idx =
      pstr_drc_gain_dec->ia_drc_params_struct.sel_drc_array[sel_drc_idx].drc_instrns_idx;

  FLOAT32 *lpcm_gain;
  FLOAT32 gain_sb, gain_lr;

  switch (pstr_ia_drc_params->sub_band_domain_mode)
  {
  case SUBBAND_DOMAIN_MODE_STFT256:
    num_dec_sb = AUDIO_CODEC_SUBBAND_COUNT_STFT256;
    ds_fac = AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR_STFT256;
    break;
  case SUBBAND_DOMAIN_MODE_QMF71:
    num_dec_sb = AUDIO_CODEC_SUBBAND_COUNT_QMF71;
    ds_fac = AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR_QMF71;
    break;
  case SUBBAND_DOMAIN_MODE_QMF64:
    num_dec_sb = AUDIO_CODEC_SUBBAND_COUNT_QMF64;
    ds_fac = AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR_QMF64;
    break;
  default:
    return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_SUBBAND_DOMAIN_MODE;
    break;
  }
  drc_frame_size_sb = pstr_ia_drc_params->drc_frame_size / ds_fac;

  if (0 <= drc_instrns_idx)
  {
    pstr_drc_instruction = &(pstr_drc_instruction_arr[drc_instrns_idx]);
    {
      if (0 < pstr_drc_instruction->drc_set_id)
      {
        if (DELAY_MODE_LOW == pstr_ia_drc_params->delay_mode)
        {
          offset = pstr_ia_drc_params->drc_frame_size;
        }
        gain_idx_grp[0] = 0;
        for (num_ch_grp = 0; num_ch_grp < pstr_drc_instruction->num_drc_ch_groups - 1;
             num_ch_grp++)
        {
          gain_idx_grp[num_ch_grp + 1] =
              gain_idx_grp[num_ch_grp] +
              pstr_drc_instruction->band_count_of_ch_group
                  [num_ch_grp]; /* index of first gain sequence
                                                                                 in channel
                                   group */
        }

        if (-1 == pstr_ia_drc_params->num_ch_process)
          pstr_ia_drc_params->num_ch_process = pstr_drc_instruction->audio_num_chan;

        if ((pstr_drc_instruction->audio_num_chan) <
            (pstr_ia_drc_params->channel_offset + pstr_ia_drc_params->num_ch_process))
        {
          return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_CH_COUNT;
        }

        for (num_ch = pstr_ia_drc_params->channel_offset;
             num_ch < pstr_ia_drc_params->channel_offset + pstr_ia_drc_params->num_ch_process;
             num_ch++)
        {
          num_ch_grp = pstr_drc_instruction->channel_group_of_ch[num_ch];
          if (0 <= num_ch_grp)
          {
            for (size = drc_frame_size_sb - 1; size >= 0; size--)
            {
              if (1 >= pstr_drc_instruction->band_count_of_ch_group[num_ch_grp])
              { /* single-band DRC */

                lpcm_gain = pstr_gain_buf->pstr_buf_interp[gain_idx_grp[num_ch_grp]].lpcm_gains +
                            MAX_SIGNAL_DELAY - pstr_ia_drc_params->gain_delay_samples + offset;

                /* get gain for this timeslot by downsampling */
                gain_sb = lpcm_gain[(size * ds_fac + (ds_fac - 1) / 2)];
                for (cnt = 0; cnt < num_dec_sb; cnt++)
                {

                  deinterleaved_audio_re[sig_idx][size * num_dec_sb + cnt] *= (FLOAT32)gain_sb;
                  deinterleaved_audio_im[sig_idx][size * num_dec_sb + cnt] *= (FLOAT32)gain_sb;
                }
              }
              else
              {
                for (cnt = num_dec_sb - 1; cnt >= 0; cnt--)
                {
                  gain_sb = 0;
                  for (num_band = 0;
                       num_band < pstr_drc_instruction->band_count_of_ch_group[num_ch_grp];
                       num_band++)
                  {
                    lpcm_gain =
                        pstr_gain_buf->pstr_buf_interp[gain_idx_grp[num_ch_grp] + num_band]
                            .lpcm_gains +
                        MAX_SIGNAL_DELAY - pstr_ia_drc_params->gain_delay_samples + offset;

                    gain_lr = lpcm_gain[(size * ds_fac + (ds_fac - 1) / 2)];
                    gain_sb = ia_add_flt(
                        gain_sb,
                        ia_mul_flt(pstr_overlap_params->str_grp_overlap_params[num_ch_grp]
                                       .str_band_overlap_params[num_band]
                                       .overlap_weight[cnt],
                                   gain_lr));
                  }
                  deinterleaved_audio_re[sig_idx][size * num_dec_sb + cnt] *= gain_sb;
                  if (SUBBAND_DOMAIN_MODE_STFT256 != pstr_ia_drc_params->sub_band_domain_mode)
                  {
                    deinterleaved_audio_im[sig_idx][size * num_dec_sb + cnt] *= gain_sb;
                  }
                  else
                  {
                    if (0 != cnt)
                      deinterleaved_audio_im[sig_idx][size * num_dec_sb + cnt] *= gain_sb;
                    if (cnt == (num_dec_sb - 1))
                      deinterleaved_audio_im[sig_idx][size * num_dec_sb + 0] *= gain_sb;
                  }
                }
              }
            }
          }
          sig_idx++;
        }
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_filter_banks_process
 *
 *  \brief Process filterbanks
 *
 *  \param [out] audio_io_buf  audio buffer
 *  \param [in] pstr_drc_instruction_arr  Pointer to instructions structure
 *  \param [in] pstr_drc_gain_dec  Pointer to drc gain structure
 *  \param [in] sel_drc_idx  drc index
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_filter_banks_process(FLOAT32 *audio_io_buf[],
                                           ia_drc_instructions_struct *pstr_drc_instruction_arr,
                                           ia_drc_gain_dec_struct *pstr_drc_gain_dec,
                                           WORD32 sel_drc_idx)
{
  ia_drc_filter_bank_struct *str_drc_filter_bank;
  ia_drc_instructions_struct *pstr_drc_instruction;
  ia_drc_params_struct *pstr_ia_drc_params = &pstr_drc_gain_dec->ia_drc_params_struct;
  ia_drc_audio_band_buffer_struct *audio_band_buffer = &pstr_drc_gain_dec->audio_band_buffer;
  ia_filter_banks_struct *ia_filter_banks_struct = &pstr_drc_gain_dec->ia_filter_banks_struct;

  WORD32 passThru;
  WORD32 num_ch, num_ch_grp, e = 0, fr_size, no_of_bands;
  WORD32 drc_frame_size = pstr_ia_drc_params->drc_frame_size;
  const WORD32 drc_instrns_idx =
      pstr_drc_gain_dec->ia_drc_params_struct.sel_drc_array[sel_drc_idx].drc_instrns_idx;

  FLOAT32 *audio_input;
  FLOAT32 **audio_output;

  if (pstr_drc_gain_dec->ia_drc_params_struct.multiband_sel_drc_idx != sel_drc_idx)
  {
    passThru = 1;
  }
  else
  {
    passThru = 0;
  }

  if (drc_instrns_idx < 0)
  {
    return IA_MPEGD_DRC_EXE_FATAL_INVALID_INDEX;
  }

  pstr_drc_instruction = &(pstr_drc_instruction_arr[drc_instrns_idx]);

  for (num_ch = pstr_ia_drc_params->channel_offset;
       num_ch < pstr_ia_drc_params->channel_offset + pstr_ia_drc_params->num_ch_process; num_ch++)
  {
    str_drc_filter_bank = NULL;
    audio_input = audio_io_buf[num_ch - pstr_ia_drc_params->channel_offset];
    audio_output = &(audio_band_buffer->non_interleaved_audio[e]);

    if ((0 <= drc_instrns_idx) && (0 == passThru))
    {
      if (0 <= pstr_drc_instruction->drc_set_id)
      {
        num_ch_grp = pstr_drc_instruction->channel_group_of_ch[num_ch];
        if (-1 != num_ch_grp)
        {
          no_of_bands = pstr_drc_instruction->band_count_of_ch_group[num_ch_grp];
          str_drc_filter_bank = &(ia_filter_banks_struct->str_drc_filter_bank[num_ch_grp]);
        }
        else
        {
          no_of_bands = 1;
          str_drc_filter_bank =
              &(ia_filter_banks_struct
                    ->str_drc_filter_bank[pstr_drc_instruction->num_drc_ch_groups]);
        }
        if (str_drc_filter_bank->str_all_pass_cascade.num_filter > 0)
        {
          impd_drc_all_pass_cascade_process(&str_drc_filter_bank->str_all_pass_cascade, num_ch,
                                            drc_frame_size, (FLOAT32 *)audio_input);
        }
      }
      else
      {
        no_of_bands = 1;
      }
    }
    else
    {
      no_of_bands = 1;
    }
    switch (no_of_bands)
    {
    case 4:
      impd_drc_four_band_filter_process((FLOAT32 **)audio_output,
                                        &str_drc_filter_bank->str_four_band_filt_bank, num_ch,
                                        drc_frame_size, (FLOAT32 *)audio_input);
      e += 4;
      break;
    case 3:
      impd_drc_three_band_filter_process((FLOAT32 **)audio_output,
                                         &str_drc_filter_bank->str_three_band_filt_bank, num_ch,
                                         drc_frame_size, (FLOAT32 *)audio_input);
      e += 3;
      break;
    case 2:
      impd_drc_two_band_filter_process((FLOAT32 **)audio_output,
                                       &str_drc_filter_bank->str_two_band_filt_bank, num_ch,
                                       drc_frame_size, (FLOAT32 *)audio_input);
      e += 2;
      break;
    case 1:
      for (fr_size = 0; fr_size < drc_frame_size; fr_size++)
      {

        audio_output[0][fr_size] = audio_input[fr_size];
      }
      e++;
      break;
    default:
      return (IA_MPEGD_DRC_INIT_NONFATAL_PARAM_ERROR);
      break;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  impd_drc_td_process
 *
 *  \brief DRC time domain process function
 *
 *  \param [in/out]  audio_in_out_buf   audio buffer
 *  \param [in/out]  pstr_drc_gain_dec   Pointer to drc gain decode structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  pstr_drc_gain   Pointer to drc gain structure
 *  \param [in]  ln_gain_db   loudness normalization gain in db
 *  \param [in]  boost_fac   boost_fac factor
 *  \param [in]  compress_fac   compression factor
 *  \param [in]  drc_characteristic_target   characteristic target
 *
 *  \return IA_ERRORCODE error
 *
 */

IA_ERRORCODE impd_drc_td_process(FLOAT32 *audio_in_out_buf[],
                                 ia_drc_gain_dec_struct *pstr_drc_gain_dec,
                                 ia_drc_config *pstr_drc_config,
                                 ia_drc_gain_struct *pstr_drc_gain, FLOAT32 ln_gain_db,
                                 FLOAT32 boost_fac, FLOAT32 compress_fac,
                                 WORD32 drc_characteristic_target)
{
  ia_drc_instructions_struct *pstr_drc_instruction = pstr_drc_config->str_drc_instruction_str;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 sel_drc_idx;

  if (pstr_drc_config->apply_drc)
  {
    for (sel_drc_idx = pstr_drc_gain_dec->ia_drc_params_struct.drc_set_counter - 1;
         sel_drc_idx >= 0; sel_drc_idx--)
    {
      err_code = impd_drc_get_gain(&pstr_drc_gain_dec->drc_gain_buffers, pstr_drc_gain_dec,
                                   pstr_drc_config, pstr_drc_gain, compress_fac, boost_fac,
                                   drc_characteristic_target, ln_gain_db, sel_drc_idx);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return (err_code);
      }
    }

    {
      for (sel_drc_idx = pstr_drc_gain_dec->ia_drc_params_struct.drc_set_counter - 1;
           sel_drc_idx >= 0; sel_drc_idx--)
      {
        {
          ia_drc_instructions_struct *drc_instructions_temp =
              &(pstr_drc_instruction[pstr_drc_gain_dec->ia_drc_params_struct
                                         .sel_drc_array[sel_drc_idx]
                                         .drc_instrns_idx]);
          if (-1 == pstr_drc_gain_dec->ia_drc_params_struct.num_ch_process)
          {
            pstr_drc_gain_dec->ia_drc_params_struct.num_ch_process =
                drc_instructions_temp->audio_num_chan;
          }

          if ((drc_instructions_temp->audio_num_chan) <
              (pstr_drc_gain_dec->ia_drc_params_struct.channel_offset +
               pstr_drc_gain_dec->ia_drc_params_struct.num_ch_process))
          {
            return IA_MPEGD_DRC_INIT_FATAL_UNSUPPORTED_CH_COUNT;
          }
        }
        err_code = impd_drc_filter_banks_process(audio_in_out_buf, pstr_drc_instruction,
                                                 pstr_drc_gain_dec, sel_drc_idx);

        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return (err_code);
        }

        impd_drc_apply_gains_and_add(pstr_drc_gain_dec, audio_in_out_buf, pstr_drc_instruction, 1,
                                     sel_drc_idx);
      }
    }
  }
  return err_code;
}

/**
 *  impd_drc_fd_process
 *
 *  \brief DRC frequency domain process function
 *
 *  \param [in/out]  audio_real_buff   audio buffer real
 *  \param [in/out]  audio_imag_buff   audio buffer imag
 *  \param [in/out]  pstr_drc_gain_dec   Pointer to drc gain decode structure
 *  \param [in]  pstr_drc_config   Pointer to drc config structure
 *  \param [in]  pstr_drc_gain   Pointer to drc gain structure
 *  \param [in]  ln_gain_db   loudness normalization gain in db
 *  \param [in]  boost_fac   boost_fac factor
 *  \param [in]  compress_fac   compression factor
 *  \param [in]  drc_characteristic_target   characteristic target
 *
 *  \return IA_ERRORCODE error
 *
 */
IA_ERRORCODE impd_drc_fd_process(FLOAT32 *audio_real_buff[], FLOAT32 *audio_imag_buff[],
                                 ia_drc_gain_dec_struct *pstr_drc_gain_dec,
                                 ia_drc_config *pstr_drc_config,
                                 ia_drc_gain_struct *pstr_drc_gain, FLOAT32 ln_gain_db,
                                 FLOAT32 boost_fac, FLOAT32 compress_fac,
                                 WORD32 drc_characteristic_target)
{
  ia_drc_instructions_struct *pstr_drc_instruction = pstr_drc_config->str_drc_instruction_str;
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 sel_drc_idx;

  if (pstr_drc_config->apply_drc)
  {
    for (sel_drc_idx = pstr_drc_gain_dec->ia_drc_params_struct.drc_set_counter - 1;
         sel_drc_idx >= 0; sel_drc_idx--)
    {
      err_code = impd_drc_get_gain(&pstr_drc_gain_dec->drc_gain_buffers, pstr_drc_gain_dec,
                                   pstr_drc_config, pstr_drc_gain, compress_fac, boost_fac,
                                   drc_characteristic_target, ln_gain_db, sel_drc_idx);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return (err_code);
      }
    }

    {
      for (sel_drc_idx = pstr_drc_gain_dec->ia_drc_params_struct.drc_set_counter - 1;
           sel_drc_idx >= 0; sel_drc_idx--)
      {
        err_code =
            impd_drc_apply_gains_subband(audio_real_buff, audio_imag_buff, pstr_drc_instruction,
                                         pstr_drc_gain_dec, sel_drc_idx);
        if (err_code != IA_MPEGH_DEC_NO_ERROR)
        {
          return (err_code);
        }
      }
    }
  }
  return err_code;
}

/** @} */ /* End of DRCProcessing */