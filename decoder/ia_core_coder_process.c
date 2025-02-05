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
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"
#include "ia_core_coder_acelp_info.h"

#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_igf_data_struct.h"
#include "impeghd_memory_standards.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct_def.h"
#include "impeghd_hoa_dec_struct.h"
#include "impeghd_error_codes.h"
#include <impeghd_fft_ifft.h>
#include "impeghd_mhas_parse.h"
#include "impeghd_multichannel.h"
#include "impeghd_tbe_dec.h"
#include "impeghd_binaural.h"
#include "impeghd_ele_interaction_intrfc.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_data_types.h"
#include "impeghd_hoa_frame_params.h"
#include "impeghd_hoa_nfc_filtering.h"
#include "impeghd_hoa_space_positions.h"
#include "impeghd_hoa_simple_mtrx.h"
#include "impeghd_hoa_render_mtrx.h"
#include "impeghd_hoa_renderer.h"
#include "impeghd_hoa_spatial_decoder_struct.h"
#include "impeghd_hoa_spatial_decoder.h"
#include "impeghd_hoa_decoder.h"
#include "impeghd_3d_vec_struct_def.h"
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_oam_dec_defines.h"
#include "impeghd_oam_dec_struct_def.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_obj_ren_dec_struct_def.h"
#include "impeghd_uni_drc_struct.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_func_def.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_rom.h"
#include "impeghd_format_conv_data.h"
#include "impeghd_peak_limiter_struct_def.h"
#include "impeghd_resampler.h"
#include "ia_core_coder_struct.h"
#include "ia_core_coder_create.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

#define IA_CC_PROC_ESC_VAL_BITS_8 255
/**
 *  impeghd_format_conv_active_dmx_stft_process
 *
 *  \brief Format conversion downmix processing
 *
 *  \param [in,out] active_dmx  Pointer to format convert downmix structure
 *  \param [in]     in_buf      Pointer to input buffer array
 *  \param [out]    out_buf     Pointer to output buffer array
 *
 *  \return IA_ERRORCODE              Error
 *
 */
static IA_ERRORCODE
impeghd_format_conv_active_dmx_stft_process(ia_format_conv_dmx_state *active_dmx,
                                            const FLOAT32 **in_buf, FLOAT32 **out_buf)
{
  WORD32 erb, ch_in, ch_out, fft_band, fft_band_stop;
  UWORD32 band_idx;
  FLOAT32 eq;
  FLOAT32 temp;
  const WORD32 *erb_freq_index = &ia_erb_idx_freq_fc[0];
  FLOAT32 *target_energy, *target_energy_prev;
  FLOAT32 *realized_energy, *realized_energy_prev;
  WORD32 num_out_ch;
  FLOAT32 *out_buf_ch;
  const FLOAT32 *in_buf_ch;
  FLOAT32 t_energy;
  FLOAT32 r_energy;
  FLOAT32 **dmix_mat_cho;
  FLOAT32 *dmix_mat_chi;

  num_out_ch = active_dmx->num_out_ch;

  /* target energy and realized signal */
  for (ch_out = 0; ch_out < num_out_ch; ch_out++)
  {
    memset(&out_buf[ch_out][0], 0, FC_STFT_FRAMEx2 * sizeof(FLOAT32));
    memset(&active_dmx->target_energy[ch_out][0], 0, FC_ERB_BANDS * sizeof(FLOAT32));
    memset(&active_dmx->realized_energy[ch_out][0], 0, FC_ERB_BANDS * sizeof(FLOAT32));
    target_energy = active_dmx->target_energy[ch_out];
    out_buf_ch = out_buf[ch_out];
    dmix_mat_cho = active_dmx->downmix_mat[ch_out];
    for (ch_in = 0; ch_in < active_dmx->num_in_ch; ch_in++)
    {
      in_buf_ch = in_buf[ch_in];
      dmix_mat_chi = dmix_mat_cho[ch_in];
      if (dmix_mat_chi[0])
      {
        temp = ia_mul_flt(dmix_mat_chi[0], in_buf_ch[0]);
        out_buf_ch[0] = ia_add_flt(out_buf_ch[0], temp);
        target_energy[0] = ia_mac_flt(target_energy[0], temp, temp);

        temp = ia_mul_flt(dmix_mat_chi[FC_BANDS_1], in_buf_ch[1]);
        out_buf_ch[1] = ia_add_flt(out_buf_ch[1], temp);
        target_energy[FC_ERB_BANDS_MINUS1] =
            ia_mac_flt(target_energy[FC_ERB_BANDS_MINUS1], temp, temp);

        fft_band = 1;
        for (erb = 1; erb < FC_ERB_BANDS; erb++)
        {
          fft_band_stop = erb_freq_index[erb];
          t_energy = target_energy[erb];
          while (fft_band < fft_band_stop)
          {
            band_idx = fft_band * 2;
            temp = ia_mul_flt(dmix_mat_chi[fft_band], in_buf_ch[band_idx]);
            out_buf_ch[band_idx] = ia_add_flt(temp, out_buf_ch[band_idx]);
            t_energy = ia_mac_flt(t_energy, temp, temp);

            band_idx++;
            temp = ia_mul_flt(dmix_mat_chi[fft_band], in_buf_ch[band_idx]);
            out_buf_ch[band_idx] = ia_add_flt(temp, out_buf_ch[band_idx]);
            t_energy = ia_mac_flt(t_energy, temp, temp);

            fft_band++;
          }
          target_energy[erb] = t_energy;
        }
      }
    }
  }
  /*realized energy */

  for (ch_out = 0; ch_out < num_out_ch; ch_out++)
  {
    out_buf_ch = out_buf[ch_out];
    realized_energy = active_dmx->realized_energy[ch_out];
    realized_energy[0] = ia_mul_flt(out_buf_ch[0], out_buf_ch[0]);
    realized_energy[FC_ERB_BANDS_MINUS1] = ia_mul_flt(out_buf_ch[1], out_buf_ch[1]);

    fft_band = 1;

    target_energy = active_dmx->target_energy[ch_out];
    target_energy_prev = active_dmx->target_energy_prev[ch_out];
    realized_energy_prev = active_dmx->realized_energy_prev[ch_out];

    for (erb = 0; erb < FC_ERB_BANDS; erb++)
    {
      fft_band_stop = erb_freq_index[erb];
      r_energy = realized_energy[erb];
      while (fft_band < fft_band_stop)
      {
        band_idx = fft_band * 2;
        r_energy = ia_mac_flt(r_energy, out_buf_ch[band_idx], out_buf_ch[band_idx]);
        band_idx++;
        r_energy = ia_mac_flt(r_energy, out_buf_ch[band_idx], out_buf_ch[band_idx]);
        fft_band++;
      }
      target_energy[erb] = ia_add_flt(ia_mul_flt(FC_ALPHA, target_energy[erb]),
                                      ia_mul_flt(FC_BETA, target_energy_prev[erb]));
      realized_energy[erb] = ia_add_flt(ia_mul_flt(FC_ALPHA, r_energy),
                                        ia_mul_flt(FC_BETA, realized_energy_prev[erb]));
    }
    memcpy(target_energy_prev, target_energy, sizeof(FLOAT32) * FC_ERB_BANDS);
    memcpy(realized_energy_prev, realized_energy, sizeof(FLOAT32) * FC_ERB_BANDS);

    t_energy = target_energy[0];
    r_energy = realized_energy[0];
    eq = (FLOAT32)ia_sqrt_flt(t_energy / (FC_EPSILON + r_energy));
    IMPEGHD_CLIP(eq, FC_EQ_MAX, FC_EQ_MIN);
    out_buf_ch[0] *= eq;

    t_energy = target_energy[FC_ERB_BANDS_MINUS1];
    r_energy = realized_energy[FC_ERB_BANDS_MINUS1];
    eq = (FLOAT32)ia_sqrt_flt(t_energy / (FC_EPSILON + r_energy));
    IMPEGHD_CLIP(eq, FC_EQ_MAX, FC_EQ_MIN);
    out_buf_ch[1] *= eq;
    fft_band = 1;
    for (erb = 1; erb < FC_ERB_BANDS; erb++)
    {
      eq = (FLOAT32)ia_sqrt_flt(target_energy[erb] / (FC_EPSILON + realized_energy[erb]));
      IMPEGHD_CLIP(eq, FC_EQ_MAX, FC_EQ_MIN);
      fft_band_stop = erb_freq_index[erb];
      while (fft_band < fft_band_stop)
      {
        band_idx = fft_band * 2;
        out_buf_ch[band_idx] = ia_mul_flt(out_buf_ch[band_idx], eq);
        band_idx++;
        out_buf_ch[band_idx] = ia_mul_flt(eq, out_buf_ch[band_idx]);
        fft_band++;
      }
    }
  }
  return 0;
}

/**
 *  ia_core_coder_read_ext_element
 *
 *  \brief Read USAC extension element
 *
 *  \param [in]    usac_ext_ele_default_length  USAC extension payload length
 *  \param [in]    usac_ext_ele_payload_frag    USAC extension payload fragment
 *  \param [in]    it_bit_buff                  bit stream buffer
 *  \param [in,out]pstr_usac_dec_config         USAC Decoder configuration
 *  \param [in]    elem_idx                     Element index in the
 * total
 * number of elements
 *
 *  \return IA_ERRORCODE
 *
 */
static IA_ERRORCODE
ia_core_coder_read_ext_element(UWORD32 usac_ext_ele_default_length,
                               UWORD32 usac_ext_ele_payload_frag, ia_bit_buf_struct *it_bit_buff,
                               ia_usac_decoder_config_struct *pstr_usac_dec_config,
                               WORD32 elem_idx)
{
  UWORD32 usac_ext_element_present;
  UWORD32 usac_ext_element_use_dft_length;
  UWORD32 pay_load_length;
  WORD32 i;
  usac_ext_element_present = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (usac_ext_element_present)
  {
    usac_ext_element_use_dft_length = ia_core_coder_read_bits_buf(it_bit_buff, 1);

    if (0 == usac_ext_element_use_dft_length)
    {
      pay_load_length = ia_core_coder_read_bits_buf(it_bit_buff, 8);

      if (pay_load_length == IA_CC_PROC_ESC_VAL_BITS_8)
      {
        pay_load_length = ia_core_coder_read_bits_buf(it_bit_buff, 16);
        pay_load_length = (UWORD32)((WORD32)pay_load_length + IA_CC_PROC_ESC_VAL_BITS_8 - 2);
      }
    }
    else
    {
      pay_load_length = usac_ext_ele_default_length;
    }
    if ((it_bit_buff->cnt_bits >> 3) < (WORD32)pay_load_length)
      return IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES;
    if (pay_load_length > 0)
    {
      if (usac_ext_ele_payload_frag)
        ia_core_coder_skip_bits_buf(it_bit_buff, 2);

      if (pstr_usac_dec_config->usac_ext_ele_payload_present[elem_idx])
      {
        if (pstr_usac_dec_config->ia_ext_ele_payload_type[elem_idx] == ID_EXT_ELE_UNI_DRC)
        {
          WORD32 preroll_counter = pstr_usac_dec_config->preroll_counter;
          WORD32 payload_buffer_offeset = 0;
          for (i = 0; i < preroll_counter; i++)
            payload_buffer_offeset +=
                pstr_usac_dec_config->usac_ext_gain_payload_len[i] * sizeof(WORD8);
          if ((pay_load_length + payload_buffer_offeset) >
              (MAX_AUDIO_PREROLLS * MAX_EXT_ELE_PAYLOAD))
            return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;

          for (i = 0; i < ((WORD32)pay_load_length); i++)
          {
            pstr_usac_dec_config->usac_ext_gain_payload_buf[elem_idx][i] =
                (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 8);
          }
          pstr_usac_dec_config->usac_ext_gain_payload_len[elem_idx] += pay_load_length;
        }
        else
        {
          if ((pay_load_length) > (MAX_AUDIO_PREROLLS * MAX_EXT_ELE_PAYLOAD))
            return IA_MPEGH_DEC_EXE_FATAL_DECODE_FRAME_ERROR;

          for (i = 0; i < ((WORD32)pay_load_length); i++)
          {
            pstr_usac_dec_config->usac_ext_gain_payload_buf[elem_idx][i] =
                (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 8);
          }
          pstr_usac_dec_config->usac_ext_gain_payload_len[elem_idx] = pay_load_length;
        }
      }
      else
      {

        if (it_bit_buff->cnt_bits < (WORD32)(pay_load_length << 3))
          longjmp(*(it_bit_buff->xmpeghd_jmp_buf),
                  IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES);

        it_bit_buff->ptr_read_next = it_bit_buff->ptr_read_next + pay_load_length;
        it_bit_buff->cnt_bits = it_bit_buff->cnt_bits - (WORD32)(pay_load_length << 3);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/**
*  impeghd_format_conv_earcon_process
*
*  \brief Format coverter processing involving active_dmx processing in freq domain
*
*  \param [in,out]  p_obj_mpegh_dec Pointer to decoder api structure
*  \param [in,out]  pstr_pcm_data   Pointer to pcm data config structure
*  \param [in]      scratch_buf     Scratch buffer for internal processing
*
*  \return IA_ERRORCODE error if any
*
*/
IA_ERRORCODE impeghd_format_conv_earcon_process(ia_mpegh_dec_api_struct *p_obj_mpegh_dec,
                                                ia_pcm_data_config *pstr_pcm_data,
                                                FLOAT32 *scratch_buf)
{
  ia_format_conv_state_struct *pstr_fc_state;
  ia_format_conv_dmx_state *pstr_stft_dmx_active = NULL;
  FLOAT32 **ptr_in_stft_buf;
  FLOAT32 **ptr_out_stft_buf;
  FLOAT32 *ptr_in_time_buf;
  FLOAT32 *ptr_sample_time;
  FLOAT32 val;
  const FLOAT32 *ptr_window_stft = ia_core_coder_sine_window256;
  FLOAT32 *ptr_out_re_stft, *ptr_out_im_stft;
  FLOAT32 *ptr_scratch;
  FLOAT32 *ptr_stft_in_buf, *ptr_stft_out_buf;
  FLOAT32 *ptr_inp_stft_buf[FC_OUT_MAX_CH];
  FLOAT32 *ptr_outp_stft_buf[FC_OUT_MAX_CH];
  WORD32 len_fft_stft = FC_STFT_FRAMEx2;
  WORD32 num_chn_in;
  WORD32 num_chn_out;
  WORD32 idx, ids, idj, idz;
  IA_ERRORCODE error_id = IA_MPEGH_DEC_NO_ERROR;
  UWORD32 idx_real, idx_imag;

  pstr_fc_state = &p_obj_mpegh_dec->p_state_mpeghd->state_format_conv;
  if (pstr_fc_state->fc_params == NULL)
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_PARAM;
  num_chn_in = pstr_fc_state->fc_params->num_in_ch;
  num_chn_out = pstr_fc_state->fc_params->num_out_ch;
  pstr_stft_dmx_active = pstr_fc_state->fc_params->active_dmx_stft;
  ptr_in_stft_buf = pstr_fc_state->stft_in_buf;
  ptr_out_stft_buf = pstr_fc_state->stft_out_buf;
  if (pstr_fc_state->fc_params->num_out_ch <= 0 || pstr_fc_state->fc_params->num_in_ch <= 0)
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_CHANNEL_NUM;
  }
  ptr_out_re_stft = scratch_buf;
  ptr_out_im_stft = ptr_out_re_stft + len_fft_stft;
  ptr_stft_in_buf = ptr_out_im_stft + len_fft_stft;
  ptr_stft_out_buf = ptr_stft_in_buf + (num_chn_in * len_fft_stft * sizeof(FLOAT32));
  ptr_scratch = ptr_stft_out_buf + (num_chn_out * len_fft_stft * sizeof(FLOAT32));

  for (idx = 0; idx < num_chn_in; idx++)
  {
    ptr_inp_stft_buf[idx] = ptr_stft_in_buf + (idx * len_fft_stft);
  }

  for (idx = 0; idx < num_chn_out; idx++)
  {
    ptr_outp_stft_buf[idx] = ptr_stft_out_buf + (idx * len_fft_stft);
  }

  for (idj = 0; idj < FC_STFT_SLOTS; idj++)
  {
    idz = idj * FC_STFT_FRAME;
    /* Transform into frequency domain*/
    for (idx = 0; idx < num_chn_in; idx++)
    {
      ptr_in_time_buf = ptr_in_stft_buf[idx];
      ptr_sample_time = &pstr_pcm_data->pcm_sample[idx][idz];
      for (ids = 0; ids < FC_STFT_FRAME; ids++)
      {
        val = ptr_sample_time[ids];
        ptr_out_re_stft[ids] = ia_mul_flt(ptr_in_time_buf[ids], ptr_window_stft[ids]);
        ptr_out_re_stft[FC_STFT_FRAME + ids] =
            ia_mul_flt(ptr_window_stft[FC_STFT_FRAME_1 - ids], val);
        ptr_in_time_buf[ids] = val;
      }
      memset(ptr_out_im_stft, 0, len_fft_stft * sizeof(FLOAT32));
      impeghd_rad2_cplx_ifft(ptr_out_re_stft, ptr_out_im_stft, len_fft_stft, ptr_scratch);

      ptr_inp_stft_buf[idx][0] = ptr_out_re_stft[0];
      ptr_inp_stft_buf[idx][1] = ptr_out_re_stft[len_fft_stft / 2];

      for (ids = 1; ids < len_fft_stft / 2; ids++)
      {
        idx_real = 2 * ids;
        idx_imag = 2 * ids + 1;
        ptr_inp_stft_buf[idx][idx_real] = ptr_out_re_stft[ids];
        ptr_inp_stft_buf[idx][idx_imag] = ptr_out_im_stft[ids];
      }
    }
    error_id = impeghd_format_conv_active_dmx_stft_process(
        pstr_stft_dmx_active, (const FLOAT32 **)ptr_inp_stft_buf, ptr_outp_stft_buf);

    for (idx = 0; idx < num_chn_out; idx++)
    {
      ptr_out_re_stft[0] = ptr_outp_stft_buf[idx][0];
      ptr_out_im_stft[0] = 0;
      ptr_out_re_stft[len_fft_stft / 2] = ptr_outp_stft_buf[idx][1];
      ptr_out_im_stft[len_fft_stft / 2] = 0;

      for (ids = 1; ids < len_fft_stft / 2; ids++)
      {
        idx_real = 2 * ids;
        idx_imag = idx_real + 1;
        ptr_out_re_stft[ids] = ptr_outp_stft_buf[idx][idx_real];
        ptr_out_re_stft[len_fft_stft - ids] = ptr_out_re_stft[ids];
        ptr_out_im_stft[ids] = ptr_outp_stft_buf[idx][idx_imag];
        ptr_out_im_stft[len_fft_stft - ids] = -ptr_out_im_stft[ids];
      }

      impeghd_rad2_cplx_fft(ptr_out_re_stft, ptr_out_im_stft, len_fft_stft, ptr_scratch);

      ptr_sample_time = &pstr_pcm_data->pcm_sample[idx][idz];
      for (ids = 0; ids < FC_STFT_FRAME; ids++)
      {
        /* overlap add */
        ptr_sample_time[ids] = ia_mac_flt(
            ia_mul_flt((ptr_out_re_stft[ids] / ((FLOAT32)len_fft_stft)), ptr_window_stft[ids]),
            ptr_out_stft_buf[idx][ids], ptr_window_stft[FC_STFT_FRAME_1 - ids]);
        ptr_out_stft_buf[idx][ids] =
            ptr_out_re_stft[FC_STFT_FRAME + ids] / ((FLOAT32)len_fft_stft);
      }
    }
  }
  return error_id;
}
/**
*  impeghd_domain_switcher_process
*
*  \brief Domain processing involving active_dmx processing in freq domain
*
*  \param [in,out]  p_obj_mpegh_dec Decoder api structure
*  \param [in,out]  num_chn_out      Output num of channel
*  \param [in]    ds_t2f        Flag for Domain switcher time to frequency
* domain conversion
*  \param [in]    scratch_buf      Scratch buffer for internal processing
*
*  \return IA_ERRORCODE error if any
*
*/
IA_ERRORCODE
impeghd_domain_switcher_process(ia_mpegh_dec_api_struct *p_obj_mpegh_dec, WORD32 *num_chn_out,
                                UWORD32 ds_t2f, FLOAT32 *scratch_buf)
{
  ia_mpegh_dec_state_struct *pstr_mpegh_dec = p_obj_mpegh_dec->p_state_mpeghd;
  ia_dec_data_struct *pstr_dec_data = (ia_dec_data_struct *)pstr_mpegh_dec->pstr_dec_data;
  ia_usac_data_struct *pstr_usac_data = &(pstr_dec_data->str_usac_data);
  ia_ds_state_struct *pstr_ds_state;
  FLOAT32 **ptr_in_stft_buf;
  FLOAT32 **ptr_out_stft_buf;
  FLOAT32 *ptr_in_time_buf;
  FLOAT32 *ptr_sample_time, *ptr_stft_time_sample;
  FLOAT32 val;
  const FLOAT32 *ptr_window_stft = ia_core_coder_sine_window256;
  FLOAT32 *ptr_out_re_stft, *ptr_out_im_stft;
  FLOAT32 *ptr_scratch;
  FLOAT32 *ptr_stft_in_buf, *ptr_stft_out_buf;
  FLOAT32 *ptr_inp_stft_buf[FC_OUT_MAX_CH];
  WORD32 t2f = ds_t2f;
  WORD32 idx, ids, idj, idz, l;
  IA_ERRORCODE error_id = IA_MPEGH_DEC_NO_ERROR;
  WORD32 len_fft_stft = FC_STFT_FRAMEx2;
  WORD32 num_chn_in;
  ptr_out_re_stft = scratch_buf;
  ptr_out_im_stft = ptr_out_re_stft + len_fft_stft;
  ptr_stft_in_buf = ptr_out_im_stft + len_fft_stft;
  pstr_ds_state = &p_obj_mpegh_dec->p_state_mpeghd->state_domain_switcher;

  if (pstr_ds_state->ds_params == NULL)
  {
    return IA_MPEGH_DEC_EXE_FATAL_DOMAIN_SWITCHER_PROCESS_FAILED;
  }

  num_chn_in = pstr_ds_state->ds_params->num_in_ch;
  *num_chn_out = pstr_ds_state->ds_params->num_out_ch;
  ptr_in_stft_buf = pstr_ds_state->stft_in_buf;
  ptr_out_stft_buf = pstr_ds_state->stft_out_buf;

  ptr_stft_out_buf = ptr_stft_in_buf + (num_chn_in * len_fft_stft * sizeof(FLOAT32));
  ptr_scratch = ptr_stft_out_buf + (*num_chn_out * len_fft_stft * sizeof(FLOAT32));

  for (idx = 0; idx < num_chn_in; idx++)
  {
    ptr_inp_stft_buf[idx] = ptr_stft_in_buf + (idx * len_fft_stft);
  }

  for (idj = 0; idj < FC_STFT_SLOTS; idj++)
  {
    idz = idj * FC_STFT_FRAME;
    if (t2f)
    {
      /* Transform into frequency domain*/
      for (idx = 0; idx < num_chn_in; idx++)
      {
        ptr_in_time_buf = ptr_in_stft_buf[idx];
        ptr_sample_time = &pstr_usac_data->time_sample_vector[idx][idz];
        for (ids = 0; ids < FC_STFT_FRAME; ids++)
        {
          val = ptr_sample_time[ids];
          ptr_out_re_stft[ids] = ia_mul_flt(ptr_in_time_buf[ids], ptr_window_stft[ids]);
          ptr_out_re_stft[FC_STFT_FRAME + ids] =
              ia_mul_flt(ptr_window_stft[FC_STFT_FRAME_1 - ids], val);
          ptr_in_time_buf[ids] = val;
        }
        memset(ptr_out_im_stft, 0, len_fft_stft * sizeof(FLOAT32));

        {
          impeghd_rad2_cplx_fft(ptr_out_re_stft, ptr_out_im_stft, len_fft_stft, ptr_scratch);
        }

        ptr_inp_stft_buf[idx][0] = ptr_out_re_stft[0];
        ptr_inp_stft_buf[idx][1] = ptr_out_re_stft[len_fft_stft / 2];

        ptr_stft_time_sample = &pstr_usac_data->time_sample_stft[idx][idz];
        l = 2 * len_fft_stft;
        ptr_stft_time_sample[0] = ptr_out_re_stft[0];
        ptr_stft_time_sample[l] = ptr_out_re_stft[len_fft_stft / 2];
        for (ids = 1; ids < FC_STFT_FRAME; ids++)
        {
          ptr_stft_time_sample[ids] = ptr_out_re_stft[ids];
          ptr_stft_time_sample[l + ids] = ptr_out_im_stft[ids];
        }
      }
    }
    else
    {
      for (idx = 0; idx < *num_chn_out; idx++)
      {

        ptr_stft_time_sample = &pstr_usac_data->time_sample_stft[idx][idz];
        l = 2 * len_fft_stft;
        ptr_out_re_stft[0] = ptr_stft_time_sample[0];
        ptr_out_re_stft[len_fft_stft / 2] = ptr_stft_time_sample[l];
        ptr_out_im_stft[0] = 0;
        ptr_out_im_stft[len_fft_stft / 2] = 0;

        for (ids = 1; ids < len_fft_stft / 2; ids++)
        {
          ptr_out_re_stft[ids] = ptr_stft_time_sample[ids];
          ptr_out_re_stft[len_fft_stft - ids] = ptr_out_re_stft[ids];
          ptr_out_im_stft[ids] = ptr_stft_time_sample[l + ids];
          ptr_out_im_stft[len_fft_stft - ids] = -ptr_out_im_stft[ids];
        }

        /*inverse fft*/

        impeghd_rad2_cplx_ifft(ptr_out_re_stft, ptr_out_im_stft, len_fft_stft, ptr_scratch);
        ptr_sample_time = &pstr_usac_data->time_sample_vector[idx][idz];
        for (ids = 0; ids < FC_STFT_FRAME; ids++)
        {
          /* overlap add */
          ptr_sample_time[ids] = ia_mac_flt(
              ia_mul_flt((ptr_out_re_stft[ids] / ((FLOAT32)len_fft_stft)), ptr_window_stft[ids]),
              ptr_out_stft_buf[idx][ids], ptr_window_stft[FC_STFT_FRAME_1 - ids]);
          ptr_out_stft_buf[idx][ids] =
              ptr_out_re_stft[FC_STFT_FRAME + ids] / ((FLOAT32)len_fft_stft);
        }
      }
    }
  }

  p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes = 1024 * *num_chn_out * sizeof(FLOAT32);
  return error_id;
}
/**
*  impeghd_format_converter_process
*
*  \brief Format coverter processing involving active_dmx processing in freq domain
*
*  \param [in,out]  p_obj_mpegh_dec Decoder api structure
*  \param [in,out]  num_chn_out      Output num of channel
*  \param [in]    scratch_buf      Scratch buffer for internal processing
*
*  \return IA_ERRORCODE error if any
*
*/
IA_ERRORCODE
impeghd_format_converter_process(ia_mpegh_dec_api_struct *p_obj_mpegh_dec, WORD32 *num_chn_out,
                                 FLOAT32 *scratch_buf)
{
  ia_format_conv_state_struct *pstr_fc_state;
  ia_format_conv_dmx_state *pstr_stft_dmx_active = NULL;
  ia_mpegh_dec_state_struct *pstr_mpegh_dec = p_obj_mpegh_dec->p_state_mpeghd;
  ia_dec_data_struct *pstr_dec_data = (ia_dec_data_struct *)pstr_mpegh_dec->pstr_dec_data;
  ia_usac_data_struct *pstr_usac_data = &(pstr_dec_data->str_usac_data);
  WORD32 len_fft_stft = FC_STFT_FRAMEx2;
  WORD32 num_chn_in;
  WORD32 idx, ids, idj, idz;
  IA_ERRORCODE error_id = IA_MPEGH_DEC_NO_ERROR;
  UWORD32 idx_real, idx_imag;
  FLOAT32 *ptr_sample_time;
  FLOAT32 val;
  const FLOAT32 *ptr_window_stft = ia_core_coder_sine_window256;
  FLOAT32 *ptr_out_re_stft, *ptr_out_im_stft;
  FLOAT32 *ptr_scratch;
  FLOAT32 *ptr_stft_in_buf, *ptr_stft_out_buf;
  FLOAT32 *ptr_inp_stft_buf[FC_OUT_MAX_CH];
  FLOAT32 *ptr_outp_stft_buf[FC_OUT_MAX_CH];

  ptr_out_re_stft = scratch_buf;
  ptr_out_im_stft = ptr_out_re_stft + len_fft_stft;
  ptr_stft_in_buf = ptr_out_im_stft + len_fft_stft;
  pstr_fc_state = &p_obj_mpegh_dec->p_state_mpeghd->state_format_conv;
  if (pstr_fc_state->fc_params == NULL)
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_PARAM;
  FLOAT32 **ptr_in_stft_buf;
  FLOAT32 **ptr_out_stft_buf;
  FLOAT32 *ptr_in_time_buf;
  num_chn_in = pstr_fc_state->fc_params->num_in_ch;
  *num_chn_out = pstr_fc_state->fc_params->num_out_ch;
  pstr_stft_dmx_active = pstr_fc_state->fc_params->active_dmx_stft;
  ptr_in_stft_buf = pstr_fc_state->stft_in_buf;
  ptr_out_stft_buf = pstr_fc_state->stft_out_buf;
  ptr_stft_out_buf = ptr_stft_in_buf + (num_chn_in * len_fft_stft * sizeof(FLOAT32));
  ptr_scratch = ptr_stft_out_buf + (*num_chn_out * len_fft_stft * sizeof(FLOAT32));

  if (pstr_fc_state->fc_params->num_out_ch <= 0 || pstr_fc_state->fc_params->num_in_ch <= 0)
  {
    return IA_MPEGH_FORMAT_CONV_INIT_FATAL_INVALID_CHANNEL_NUM;
  }
  for (idx = 0; idx < *num_chn_out; idx++)
  {
    ptr_outp_stft_buf[idx] = ptr_stft_out_buf + (idx * len_fft_stft);
  }

  for (idx = 0; idx < num_chn_in; idx++)
  {
    ptr_inp_stft_buf[idx] = ptr_stft_in_buf + (idx * len_fft_stft);
  }

  for (idj = 0; idj < FC_STFT_SLOTS; idj++)
  {
    idz = idj * FC_STFT_FRAME;
    /* Transform into frequency domain*/
    for (idx = 0; idx < num_chn_in; idx++)
    {
      ptr_in_time_buf = ptr_in_stft_buf[idx];
      ptr_sample_time = &pstr_usac_data->time_sample_vector[idx][idz];
      for (ids = 0; ids < FC_STFT_FRAME; ids++)
      {
        val = ptr_sample_time[ids];
        ptr_out_re_stft[ids] = ia_mul_flt(ptr_in_time_buf[ids], ptr_window_stft[ids]);
        ptr_out_re_stft[FC_STFT_FRAME + ids] =
            ia_mul_flt(ptr_window_stft[FC_STFT_FRAME_1 - ids], val);
        ptr_in_time_buf[ids] = val;
      }
      memset(ptr_out_im_stft, 0, len_fft_stft * sizeof(FLOAT32));
      impeghd_rad2_cplx_ifft(ptr_out_re_stft, ptr_out_im_stft, len_fft_stft, ptr_scratch);

      ptr_inp_stft_buf[idx][0] = ptr_out_re_stft[0];
      ptr_inp_stft_buf[idx][1] = ptr_out_re_stft[len_fft_stft / 2];

      for (ids = 1; ids < len_fft_stft / 2; ids++)
      {
        idx_real = 2 * ids;
        idx_imag = 2 * ids + 1;
        ptr_inp_stft_buf[idx][idx_real] = ptr_out_re_stft[ids];
        ptr_inp_stft_buf[idx][idx_imag] = ptr_out_im_stft[ids];
      }
    }

    error_id = impeghd_format_conv_active_dmx_stft_process(
        pstr_stft_dmx_active, (const FLOAT32 **)ptr_inp_stft_buf, ptr_outp_stft_buf);

    for (idx = 0; idx < *num_chn_out; idx++)
    {
      ptr_out_re_stft[0] = ptr_outp_stft_buf[idx][0];
      ptr_out_im_stft[0] = 0;
      ptr_out_re_stft[len_fft_stft / 2] = ptr_outp_stft_buf[idx][1];
      ptr_out_im_stft[len_fft_stft / 2] = 0;

      for (ids = 1; ids < len_fft_stft / 2; ids++)
      {
        idx_real = 2 * ids;
        idx_imag = idx_real + 1;
        ptr_out_re_stft[ids] = ptr_outp_stft_buf[idx][idx_real];
        ptr_out_re_stft[len_fft_stft - ids] = ptr_out_re_stft[ids];
        ptr_out_im_stft[ids] = ptr_outp_stft_buf[idx][idx_imag];
        ptr_out_im_stft[len_fft_stft - ids] = -ptr_out_im_stft[ids];
      }

      /*inverse fft*/
      impeghd_rad2_cplx_fft(ptr_out_re_stft, ptr_out_im_stft, len_fft_stft, ptr_scratch);

      ptr_sample_time = &pstr_usac_data->time_sample_vector[idx][idz];
      for (ids = 0; ids < FC_STFT_FRAME; ids++)
      {
        /* overlap add */
        ptr_sample_time[ids] = ia_mac_flt(
            ia_mul_flt((ptr_out_re_stft[ids] / ((FLOAT32)len_fft_stft)), ptr_window_stft[ids]),
            ptr_out_stft_buf[idx][ids], ptr_window_stft[FC_STFT_FRAME_1 - ids]);
        ptr_out_stft_buf[idx][ids] =
            ptr_out_re_stft[FC_STFT_FRAME + ids] / ((FLOAT32)len_fft_stft);
      }
    }
  }

  p_obj_mpegh_dec->p_state_mpeghd->ui_out_bytes = 1024 * *num_chn_out * sizeof(FLOAT32);
  return error_id;
}

/**
*  ia_core_coder_usac_process
*
*  \brief Performs USAC data parsing and processing
*
*  \param [in,out]  pstr_dec_data dec data struct
*  \param [in,out]  codec_handle  dec state struct
*
*  \return IA_ERRORCODE
*
*/
IA_ERRORCODE ia_core_coder_usac_process(ia_dec_data_struct *pstr_dec_data, VOID *codec_handle)
{
  IA_ERRORCODE err = IA_MPEGH_DEC_NO_ERROR;
  WORD32 ele_id = 0;
  WORD32 mct_cnt = 0;
  ia_mpegh_dec_state_struct *p_state_mpegh_dec = (ia_mpegh_dec_state_struct *)codec_handle;

  ia_usac_data_struct *pstr_usac_data = &(pstr_dec_data->str_usac_data);
  ia_bit_buf_struct *it_bit_buff = &pstr_dec_data->dec_bit_buf;

  ia_frame_data_struct *fd = &(pstr_dec_data->str_frame_data);

  ia_usac_config_struct *pstr_usac_config = &(fd->str_audio_specific_config.str_usac_config);
  ia_usac_decoder_config_struct *pstr_usac_dec_config =
      &(fd->str_audio_specific_config.str_usac_config.str_usac_dec_config);

  WORD16 nr_core_coder_channels = 0;
  WORD32 ch_offset = 0;

  WORD32 elem_idx = 0;
  WORD32 num_ch_out = 0;
  WORD32 num_elements = pstr_usac_dec_config->num_elements;
  ia_usac_tmp_core_coder_struct str_tmp_core_coder[MAX_NUM_CHANNELS] = {{0}};
  ia_usac_tmp_core_coder_struct *pstr_core_coder;
  WORD32 ch_cnt = 0;
  WORD32 sig_group, i;

  pstr_usac_data->is_base_line_profile_3b = p_state_mpegh_dec->is_base_line_profile_3b;

  for (i = 0; i < MAX_NUM_CHANNELS; i++)
  {
    ia_core_coder_memset(pstr_usac_data->time_sample_vector[i],
                         (1024 * sizeof(pstr_usac_data->time_sample_vector[i][0])) /
                             sizeof(FLOAT32));
  }
  pstr_usac_data->usac_independency_flg = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  for (elem_idx = 0; elem_idx < num_elements; elem_idx++)
  {
    WORD32 stereo_config_index =
        pstr_usac_config->str_usac_dec_config.str_usac_element_config[elem_idx]
            .stereo_config_index;
    switch (ele_id = pstr_usac_dec_config->usac_element_type[elem_idx])
    {
    case ID_USAC_SCE:
      nr_core_coder_channels = 1;
      num_ch_out += 1;

      if ((ch_offset >= MAX_NUM_CHANNELS) || (num_ch_out > MAX_NUM_CHANNELS))
        return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_CHANNELS;

      if (pstr_usac_dec_config->ele_length_present == 1)
      {
        ia_core_coder_skip_bits_buf(it_bit_buff, 16);
      }
      pstr_core_coder = &str_tmp_core_coder[ch_cnt];
      err = ia_core_coder_data(ele_id, pstr_usac_data, elem_idx, ch_offset, it_bit_buff,
                               nr_core_coder_channels, pstr_usac_dec_config, pstr_core_coder);
      ch_cnt++;
      if (err != IA_MPEGH_DEC_NO_ERROR)
        return err;

      ch_offset += nr_core_coder_channels;
      break;

    case ID_USAC_CPE:
      if (1 == stereo_config_index)
      {
        nr_core_coder_channels = 1;
      }
      else
      {
        nr_core_coder_channels = 2;
      }

      num_ch_out += 2;

      if ((ch_offset >= MAX_NUM_CHANNELS) || (num_ch_out > MAX_NUM_CHANNELS))
        return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_CHANNELS;

      if (pstr_usac_dec_config->ele_length_present == 1)
      {
        ia_core_coder_skip_bits_buf(it_bit_buff, 16);
      }
      pstr_core_coder = &str_tmp_core_coder[ch_cnt];
      err = ia_core_coder_data(ele_id, pstr_usac_data, elem_idx, ch_offset, it_bit_buff,
                               nr_core_coder_channels, pstr_usac_dec_config, pstr_core_coder);
      ch_cnt++;
      if (err != IA_MPEGH_DEC_NO_ERROR)
        return err;

      ch_offset += nr_core_coder_channels;
      break;

    case ID_USAC_LFE:
      nr_core_coder_channels = 1;
      num_ch_out += 1;

      if ((ch_offset >= MAX_NUM_CHANNELS) || (num_ch_out > MAX_NUM_CHANNELS))
        return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_NUM_CHANNELS;

      if (pstr_usac_dec_config->ele_length_present == 1)
      {
        ia_core_coder_skip_bits_buf(it_bit_buff, 16);
      }
      pstr_core_coder = &str_tmp_core_coder[ch_cnt];
      err = ia_core_coder_data(ele_id, pstr_usac_data, elem_idx, ch_offset, it_bit_buff,
                               nr_core_coder_channels, pstr_usac_dec_config, pstr_core_coder);
      ch_cnt++;
      if (err != IA_MPEGH_DEC_NO_ERROR)
        return err;

      ch_offset += nr_core_coder_channels;
      break;

    case ID_USAC_EXT:
    {
      ia_usac_dec_element_config_struct *pusac_element_config =
          &pstr_usac_dec_config->str_usac_element_config[elem_idx];
      err = ia_core_coder_read_ext_element(pusac_element_config->usac_ext_eleme_def_len,
                                           pusac_element_config->usac_ext_elem_pld_frag,
                                           it_bit_buff, pstr_usac_dec_config, elem_idx);

      if (err != IA_MPEGH_DEC_NO_ERROR)
        return err;

      break;
    }

    default:

      return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_ELEM_INDEX;

      break;
    }
  }

  for (elem_idx = 0; elem_idx < num_elements; elem_idx++)
  {
    if (pstr_usac_dec_config->ia_ext_ele_payload_type[elem_idx] == ID_MPEGH_EXT_ELE_MCT)
    {
      err = impeghd_mc_parse(&pstr_usac_dec_config->ia_mcdata[mct_cnt],
                             pstr_usac_dec_config->usac_ext_gain_payload_buf[elem_idx],
                             pstr_usac_dec_config->usac_ext_gain_payload_len[elem_idx],
                             pstr_usac_data->usac_independency_flg,
                             pstr_usac_dec_config->ia_sfb_info, &pstr_dec_data->dec_bit_buf);
      if (err != IA_MPEGH_DEC_NO_ERROR)
      {
        return err;
      }
      mct_cnt++;
      for (i = 0; i < MAX_NUM_CHANNELS; i++)
      {
        pstr_usac_dec_config->ia_mcdata[mct_cnt].prev_out_spec[i] =
            &pstr_usac_data->mct_prev_out_spec[i][0];
      }
    }
  }
  for (sig_group = 0; sig_group < mct_cnt; sig_group++)
  {
    for (i = 0; i < pstr_usac_dec_config->ia_mcdata[sig_group].num_pairs; i++)
    {
      WORD32 mct_band_offset = 0, mct_bands_per_win;
      WORD32 win, group, groupwin, ch1, ch2;
      FLOAT32 *dmx = NULL, *res = NULL;
      WORD32 igf_cnt;
      ia_multichannel_data *mct = &pstr_usac_dec_config->ia_mcdata[sig_group];
      WORD32 win_tot = 0;
      WORD32 zero_prev_out_spec1, zero_prev_out_spec2;
      WORD32 no_frame_memory;
      ch1 = mct->channel_map[mct->code_pairs[i][0] + mct->start_channel];
      ch2 = mct->channel_map[mct->code_pairs[i][1] + mct->start_channel];
      if (pstr_usac_dec_config->ia_sfb_info[ch1] == NULL ||
          pstr_usac_dec_config->ia_sfb_info[ch2] == NULL)
        return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_SFB_INFO;
      mct_bands_per_win = pstr_usac_dec_config->ia_sfb_info[ch1]->islong
                              ? mct->num_mask_band[i]
                              : mct->num_mask_band[i] / 8;

      if (pstr_usac_dec_config->ia_sfb_info[ch1]->islong !=
          pstr_usac_dec_config->ia_sfb_info[ch2]->islong)
      {
        return IA_MPEGH_DEC_EXE_FATAL_INVALID_WINSHAPE_COMB;
      }

      zero_prev_out_spec1 = ((pstr_usac_data->window_sequence[ch1] == EIGHT_SHORT_SEQUENCE) !=
                             (pstr_usac_data->window_sequence_last[ch1] == EIGHT_SHORT_SEQUENCE));
      zero_prev_out_spec2 = ((pstr_usac_data->window_sequence[ch2] == EIGHT_SHORT_SEQUENCE) !=
                             (pstr_usac_data->window_sequence_last[ch2] == EIGHT_SHORT_SEQUENCE));
      no_frame_memory =
          (pstr_usac_data->usac_independency_flg || pstr_usac_data->td_frame_prev[ch1] ||
           pstr_usac_data->td_frame_prev[ch2]);

      if (mct->stereo_filling_flag[i])
      {
        FLOAT32 *prev_dmx = pstr_usac_data->ptr_fft_scratch;
        memset(prev_dmx, 0, LN2 * sizeof(FLOAT32));
        FLOAT32 *prevSpec1 = mct->prev_out_spec[mct->code_pairs[i][0] + mct->start_channel];
        FLOAT32 *prevSpec2 = mct->prev_out_spec[mct->code_pairs[i][1] + mct->start_channel];
        WORD32 sfb;

        for (win = 0, group = 0; group < pstr_usac_dec_config->ia_sfb_info[ch2]->num_groups;
             group++)
        {
          const WORD32 band_offset =
              win_tot * pstr_usac_dec_config->ia_sfb_info[ch2]->sfb_per_sbk;
          for (groupwin = 0; groupwin < pstr_usac_dec_config->ia_sfb_info[ch2]->group_len[group];
               groupwin++, win++)
          {
            err = impeghd_mc_get_prev_dmx(
                mct, (zero_prev_out_spec1 || no_frame_memory) ? NULL : prevSpec1,
                (zero_prev_out_spec2 || no_frame_memory) ? NULL : prevSpec2,
                prev_dmx + win * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk,
                mct_bands_per_win, mct->mask[i] + mct_band_offset,
                mct->pair_coeffq_sfb_prev[i] + mct_band_offset,
                pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk, i,
                pstr_usac_dec_config->ia_sfb_info[ch2]->sfb_per_sbk,
                pstr_usac_dec_config->ia_sfb_info[ch2]->ptr_sfb_tbl);
            if (err != IA_MPEGH_DEC_NO_ERROR)
            {
              return err;
            }

            mct_band_offset += mct_bands_per_win;
            if (prevSpec1)
            {
              prevSpec1 += pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk;
            }
            if (prevSpec2)
            {
              prevSpec2 += pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk;
            }
          }

          impeghd_mc_stereofilling_add(
              pstr_usac_data->coef[ch2] +
                  win_tot * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk,
              &prev_dmx[win_tot * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk],
              &pstr_usac_data->scale_factors[ch2][band_offset],
              pstr_usac_dec_config->ia_sfb_info[ch2]->sfb_per_sbk,
              pstr_usac_dec_config->ia_sfb_info[ch2]->group_len[group],
              pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk,
              pstr_usac_dec_config->ia_sfb_info[ch2]->ptr_sfb_tbl);

          for (igf_cnt = 0; igf_cnt < 4; igf_cnt++)
          {
            impeghd_mc_stereofilling_add(
                &pstr_usac_data->igf_dec_data[ch2].igf_input_spec[MAX_IGF_LEN * igf_cnt] +
                    win_tot * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk,
                &prev_dmx[win_tot * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk],
                &pstr_usac_data->scale_factors[ch2][band_offset],
                pstr_usac_dec_config->ia_sfb_info[ch2]->sfb_per_sbk,
                pstr_usac_dec_config->ia_sfb_info[ch2]->group_len[group],
                pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk,
                pstr_usac_dec_config->ia_sfb_info[ch2]->ptr_sfb_tbl);
          }

          for (sfb = 0; sfb < pstr_usac_dec_config->ia_sfb_info[ch2]->sfb_per_sbk; sfb++)
          {
            pstr_usac_data->scale_factors[ch2][band_offset + sfb] = 0.0f;
          }

          win_tot += pstr_usac_dec_config->ia_sfb_info[ch2]->group_len[group];
        }
      }

      mct_band_offset = 0;

      for (win = 0, group = 0; group < pstr_usac_dec_config->ia_sfb_info[ch1]->num_groups;
           group++)
      {
        for (groupwin = 0; groupwin < pstr_usac_dec_config->ia_sfb_info[ch1]->group_len[group];
             groupwin++, win++)
        {
          dmx = pstr_usac_data->coef[ch1] +
                win * pstr_usac_dec_config->ia_sfb_info[ch1]->bins_per_sbk; // [win];
          res = pstr_usac_data->coef[ch2] +
                win * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk; // [win];
          impeghd_mc_process(mct, dmx, res, mct->pair_coeffq_sfb[i] + mct_band_offset,
                             mct->mask[i] + mct_band_offset, mct_bands_per_win,
                             pstr_usac_dec_config->ia_sfb_info[ch1]->sfb_per_sbk, i,
                             pstr_usac_dec_config->ia_sfb_info[ch1]->ptr_sfb_tbl,
                             pstr_usac_dec_config->ia_sfb_info[ch1]->bins_per_sbk);

          for (igf_cnt = 0; igf_cnt < 4; igf_cnt++)
          {
            dmx = &pstr_usac_data->igf_dec_data[ch1].igf_input_spec[MAX_IGF_LEN * igf_cnt] +
                  win * pstr_usac_dec_config->ia_sfb_info[ch1]->bins_per_sbk;
            res = &pstr_usac_data->igf_dec_data[ch2].igf_input_spec[MAX_IGF_LEN * igf_cnt] +
                  win * pstr_usac_dec_config->ia_sfb_info[ch2]->bins_per_sbk;

            impeghd_mc_process(mct, dmx, res, mct->pair_coeffq_sfb[i] + mct_band_offset,
                               mct->mask[i] + mct_band_offset, mct_bands_per_win,
                               pstr_usac_dec_config->ia_sfb_info[ch1]->sfb_per_sbk, i,
                               pstr_usac_dec_config->ia_sfb_info[ch1]->ptr_sfb_tbl,
                               pstr_usac_dec_config->ia_sfb_info[ch1]->bins_per_sbk);
          }
          mct_band_offset += mct_bands_per_win;
        }
      }
    }
  }
  for (sig_group = 0; sig_group < mct_cnt; sig_group++)
  {
    ia_multichannel_data *mct = &pstr_usac_dec_config->ia_mcdata[sig_group];
    WORD32 ch;
    for (ch = 0; ch < mct->num_ch_to_apply; ch++)
    {
      const WORD32 chIdx = mct->channel_map[ch + mct->start_channel];
      const ia_sfb_info *sfb_info_element = pstr_usac_dec_config->ia_sfb_info[chIdx];
      if (pstr_usac_dec_config->ia_sfb_info[chIdx] == NULL)
        return IA_MPEGH_DEC_EXE_FATAL_NULL_PTR_SFB_INFO;
      mct->prev_out_spec[ch] = &pstr_usac_data->mct_prev_out_spec[ch][0];
      impeghd_mc_save_prev(mct, pstr_usac_data->coef[chIdx], ch, sfb_info_element->max_win_len,
                           sfb_info_element->bins_per_sbk, 0);
    }
  }

  ch_cnt = 0;
  ch_offset = 0;
  /* process elements */
  for (elem_idx = 0; elem_idx < num_elements; elem_idx++)
  {
    WORD32 stereo_config_index =
        pstr_usac_config->str_usac_dec_config.str_usac_element_config[elem_idx]
            .stereo_config_index;
    WORD32 num_ch_per_ele = 0;

    switch (pstr_usac_dec_config->usac_element_type[elem_idx])
    {
    case ID_USAC_SCE:
    case ID_USAC_LFE:
      pstr_core_coder = &str_tmp_core_coder[ch_cnt];

      err = ia_core_coder_data_process(pstr_usac_data, elem_idx, ch_offset, 1, pstr_core_coder);
      if (err != IA_MPEGH_DEC_NO_ERROR)
        return err;
      ch_offset += 1;
      ch_cnt++;
      break;
    case ID_USAC_CPE:
      if (1 == stereo_config_index)
      {
        num_ch_per_ele = 1;
      }
      else
      {
        num_ch_per_ele = 2;
      }
      pstr_core_coder = &str_tmp_core_coder[ch_cnt];

      err = ia_core_coder_data_process(pstr_usac_data, elem_idx, ch_offset, num_ch_per_ele,
                                       pstr_core_coder);
      if (err != IA_MPEGH_DEC_NO_ERROR)
        return err;
      ch_offset += num_ch_per_ele;
      ch_cnt++;
      break;
    default:
      break;
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */