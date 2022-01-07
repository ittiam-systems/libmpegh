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
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_dec.h"
#include "ia_core_coder_apicmd_standards.h"
#include "ia_core_coder_api_defs.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_channel.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_definitions.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "impeghd_memory_standards.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_func_def.h"

#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"

#include "impeghd_binaural.h"
#include "impeghd_binaural_filter_design.h"
#include "impeghd_binaural_renderer.h"
#include "impeghd_cicp_defines.h"

#include "impeghd_cicp_struct_def.h"
#include "impeghd_obj_ren_dec_defines.h"
#include "impeghd_ren_interface_utils.h"

/**
 * @defgroup BinauralProcess Binaural renderer process
 * @ingroup  BinauralProcess
 * @brief Binaural renderer process
 *
 * @{
 */

/**
*  impeghd_fft_complex_to_perm_fmt_i
*
*  \brief Helper function to re-arrange spectral data.
*
*  \param [in,out] ptr_xr      Pointer to real part of input data.
*  \param [in,out] ptr_xi      Pointer to imaginary part of input data.
*  \param [in]  fft_size    FFT size information.
*  \param [in]  ptr_scratch Pointer to scratch buffer.
*
*
*
*/
static VOID impeghd_fft_complex_to_perm_fmt_i(FLOAT32 *ptr_xr, FLOAT32 *ptr_xi, WORD32 fft_size,
                                              FLOAT32 *ptr_scratch)
{
  WORD32 i;
  WORD32 nyquist_ind;
  memmove(ptr_scratch, ptr_xr, sizeof(ptr_xr[0]) * fft_size);
  nyquist_ind = fft_size / 2;
  ptr_xr[0] = ptr_scratch[0];
  ptr_xr[1] = ptr_scratch[nyquist_ind];
  for (i = 1; i < nyquist_ind; i++)
  {
    ptr_xr[2 * i] = ptr_scratch[i];
    ptr_xr[2 * i + 1] = -1 * ptr_xi[i];
  }
}

/**
*  impeghd_br_find_geometry_in_speaker_config_3d
*
*  \brief Helper function to get speaker geometry from speaker config data.
*
*  \param [in]  ptr_geometry          Pointer to channel/ls geometry structure.
*  \param [in]  ptr_speaker_config_3d Pointer to speaker config data structure.
*  \param [out] ind                   Pointer to variable carrying index of the appropriate
* speaker.
*
*  \return error IA_ERRORCODE if any
*
*/
static IA_ERRORCODE impeghd_br_find_geometry_in_speaker_config_3d(
    ia_ch_geometry *ptr_geometry, ia_interface_speaker_config_3d *ptr_speaker_config_3d,
    pWORD32 ind)
{
  UWORD32 spk;
  *ind = -1;

  for (spk = 0; spk < ptr_speaker_config_3d->num_speakers; spk++)
  {

    if ((ptr_geometry->az == ptr_speaker_config_3d->geometry[spk].az) &&
        (ptr_geometry->el == ptr_speaker_config_3d->geometry[spk].el) &&
        (ptr_geometry->lfe == ptr_speaker_config_3d->geometry[spk].lfe))
    {
      *ind = (WORD32)spk;
      break;
    }
  }

  if ((*ind == -1) && (ptr_geometry->lfe == 1))
  {
    for (spk = 0; spk < ptr_speaker_config_3d->num_speakers; spk++)
    {
      if (ptr_geometry->lfe == ptr_speaker_config_3d->geometry[spk].lfe)
      {
        *ind = (WORD32)spk;
        break;
      }
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_binaural_struct_init
*
*  \brief Helper function to initialize binaural structure data.
*
*  \param [in]  direct_len     Direct Filter length information.
*  \param [in]  diffuse_len    Diffuse block length information.
*  \param [in]  diffuse_blocks Number of diffuse blocks.
*  \param [in]  num_input      Number of input channels.
*  \param [in]  num_output     Number of output channels.
*  \param [out] ptr_binaural   Pointer to binaural renderer persistent structure.
*
*  \return error IA_ERRORCODE if any.
*
*/
static IA_ERRORCODE impeghd_binaural_struct_init(WORD32 direct_len, WORD32 diffuse_len,
                                                 WORD32 diffuse_blocks, WORD32 num_input,
                                                 WORD32 num_output,
                                                 ia_binaural_ren_pers_mem_str *ptr_binaural)
{
  WORD32 len, n;
  WORD32 fft_min_size;
  n = 1;
  if (diffuse_len > BINAURAL_DIF_MAXLEN || direct_len > BINAURAL_DIR_MAXLEN)
  {
    return IA_MPEGH_BINAURAL_INIT_FATAL_UNSUPPORTED_BLOCK_LENGTH;
  }
  do
  {
    fft_min_size = (1 << n);
    n++;
  } while (!(fft_min_size >= 2 * direct_len));

  ptr_binaural->write_idx = 0;
  ptr_binaural->filter_size = 0;
  ptr_binaural->fft_size = fft_min_size;
  ptr_binaural->num_input = num_input;
  ptr_binaural->num_output = num_output;
  len = ptr_binaural->fft_size;

  if (diffuse_blocks < 0)
  {
    diffuse_blocks = 2 * diffuse_len / ptr_binaural->fft_size;
    if (len % diffuse_len != 0)
    {
      diffuse_blocks++;
    }
  }
  ptr_binaural->processed_samples = 0;
  ptr_binaural->output_samples_size = 0;
  ptr_binaural->memory_index = 0;
  ptr_binaural->ifft_len = ptr_binaural->fft_size;
  ptr_binaural->fft_len = ptr_binaural->fft_size;

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_binaural_struct_set
*
*  \brief Helper function to set binaural structure data.
*
*  \param [in]  ptr_td_binaural_renderer_param			Pointer to binaural memory
* structure
*  \param [in]  ptr_scratch_buf
* Pointer to scratch.
*  \param [out] ptr_binaural
* Pointer to binaural data structure.
*
*
*
*/
static VOID
impeghd_binaural_struct_set(ia_binaural_ren_pers_mem_str *ptr_binaural, WORD8 *ptr_scratch_buf,
                            ia_td_binaural_ren_param_str *ptr_td_binaural_renderer_param)
{
  WORD32 inp, out, block;
  FLOAT32 *fft_ifft_scratch, *complex_scratch, *ptr_xi;
  ia_binaural_scratch *pstr_scratch = (ia_binaural_scratch *)ptr_scratch_buf;
  fft_ifft_scratch = pstr_scratch->fft_ifft_scratch;
  complex_scratch = pstr_scratch->complex_scratch;
  ptr_xi = pstr_scratch->xi;
  ptr_binaural->diffuse_blocks = ptr_td_binaural_renderer_param->num_diffuse_block;
  ptr_binaural->direct_size = ptr_td_binaural_renderer_param->len_direct;
  ptr_binaural->diffuse_size = ptr_td_binaural_renderer_param->len_direct;
  ptr_binaural->filter_size = ptr_binaural->fft_size / 2;
  for (out = 0; out < ptr_binaural->num_output; out++)
  {
    for (inp = 0; inp < ptr_binaural->num_input; inp++)
    {
      memmove(ptr_binaural->taps_direct[out][inp],
              ptr_td_binaural_renderer_param->ptr_taps_direct[out][inp],
              ptr_binaural->filter_size *
                  sizeof(ptr_td_binaural_renderer_param->ptr_taps_direct[out][inp][0]));
      memset(&ptr_binaural->taps_direct[out][inp][ptr_binaural->filter_size], 0,
             (ptr_binaural->fft_size - ptr_binaural->filter_size) *
                 sizeof(ptr_binaural->taps_direct[out][inp][ptr_binaural->filter_size]));
      ia_core_coder_memset(ptr_xi, ptr_binaural->fft_size);
      impeghd_cplx_ifft_8k(ptr_binaural->taps_direct[out][inp], ptr_xi, fft_ifft_scratch,
                           ptr_binaural->ifft_len);
      impeghd_fft_complex_to_perm_fmt_i((ptr_binaural->taps_direct[out][inp]), ptr_xi,
                                        ptr_binaural->fft_size, complex_scratch);
      ia_core_coder_memset(ptr_binaural->input_mem_buf[inp], ptr_binaural->fft_size);
    }
    ia_core_coder_memset(ptr_binaural->output_mem_buf[out], ptr_binaural->fft_size);
  }

  for (block = 0; block < ptr_binaural->diffuse_blocks; block++)
  {
    for (out = 0; out < ptr_binaural->num_output; out++)
    {
      memmove(ptr_binaural->taps_diffuse[block][out],
              ptr_td_binaural_renderer_param->ptr_taps_diffuse[block][out],
              sizeof(ptr_td_binaural_renderer_param->ptr_taps_diffuse[block][out][0]) *
                  ptr_binaural->diffuse_size);
      memset(&ptr_binaural->taps_diffuse[block][out][ptr_binaural->filter_size], 0,
             sizeof(ptr_binaural->taps_diffuse[block][out][ptr_binaural->filter_size]) *
                 (ptr_binaural->fft_size - ptr_binaural->filter_size));
      ia_core_coder_memset(ptr_xi, ptr_binaural->fft_size);
      impeghd_cplx_ifft_8k(ptr_binaural->taps_diffuse[block][out], ptr_xi, fft_ifft_scratch,
                           ptr_binaural->ifft_len);
      impeghd_fft_complex_to_perm_fmt_i((ptr_binaural->taps_diffuse[block][out]), ptr_xi,
                                        ptr_binaural->fft_size, complex_scratch);
    }
  }

  for (block = 0; block < ptr_binaural->diffuse_blocks; block++)
  {
    for (out = 0; out < ptr_binaural->num_output; out++)
    {
      ptr_td_binaural_renderer_param->ptr_diffuse_fc[out][block] =
          ia_min_flt(ptr_td_binaural_renderer_param->ptr_diffuse_fc[out][block], 1.0f);
      ptr_binaural->diffuse_process_size[block][out] = (WORD32)(
          ptr_td_binaural_renderer_param->ptr_diffuse_fc[out][block] * ptr_binaural->filter_size);
    }
  }
  for (out = 0; out < ptr_binaural->num_output; out++)
  {
    for (inp = 0; inp < ptr_binaural->num_input; inp++)
    {
      ptr_td_binaural_renderer_param->ptr_direct_fc[out][inp] =
          ia_min_flt(ptr_td_binaural_renderer_param->ptr_direct_fc[out][inp], 1.0f);
      ptr_binaural->direct_process_size[out][inp] = (WORD32)(
          ptr_td_binaural_renderer_param->ptr_direct_fc[out][inp] * ptr_binaural->filter_size);
    }
  }
  for (inp = 0; inp < ptr_binaural->num_input; inp++)
  {
    ptr_binaural->diffuse_weight[inp] =
        ptr_td_binaural_renderer_param->ptr_inv_diffuse_weight[inp];
  }

  ptr_binaural->spec_mul_size = 2 * ptr_binaural->filter_size;
  ptr_binaural->processed_samples = ptr_binaural->fft_size - ptr_binaural->filter_size;
  ptr_binaural->output_samples_size = ptr_binaural->processed_samples;
  ptr_binaural->memory_index = 0;
}

/**
*  impeghd_binaural_renderer_init
*
*  \brief TD Binaural renderer module initalization.
*
*  \param [in,out] binaural_info_handle Pointer to binaural renderer data structure.
*  \param [in]  wav_param            Pointer to input stream properties structure.
*  \param [in]  binaural_handle      Pointer to binaural persistent structure.
*
*  \return error IA_ERRORCODE if any
*
*/

IA_ERRORCODE impeghd_binaural_renderer_init(ia_binaural_renderer *binaural_info_handle,
                                            ia_binaural_in_stream_cfg_str *wav_param,
                                            ia_binaural_render_struct *binaural_handle)
{
  IA_ERRORCODE error;
  WORD32 i, j;
  WORD32 num_ch_expected = 0, num_lfe_expected = 0;
  WORD32 returned_code_8s, ind = 0;
  WORD8 *ptr_scratch = (WORD8 *)binaural_info_handle->ptr_scratch;
  ia_binaural_scratch *pstr_scratch = (ia_binaural_scratch *)ptr_scratch;
  ia_binaural_representation_str *ptr_binaural_rep = NULL;
  ia_td_binaural_ren_param_str *ptr_td_binaural_renderer_param = NULL;
  ia_ch_geometry geometry_expected[CICP2GEOMETRY_MAX_LOUDSPEAKERS_BRIR] = {{0}};

  if (wav_param->cicp_spk_idx > 0)
  {
    error = impeghd_fill_ls_geom_frm_cicp_spk_idx(wav_param->cicp_spk_idx, &geometry_expected[0],
                                                  &num_ch_expected, &num_lfe_expected);
    if (IA_MPEGH_DEC_NO_ERROR != error)
    {
      return error;
    }
  }
  for (i = 0; i < binaural_info_handle->num_binaural_representation; i++)
  {
    ptr_binaural_rep = binaural_info_handle->ptr_binaural_rep[i];
    if ((ptr_binaural_rep->binaural_data_format_id == 2) &&
        (ptr_binaural_rep->brir_sampling_frequency == (UWORD32)wav_param->fs_input))
    {
      if (ptr_binaural_rep->is_hoa_data == 0)
      {
        for (j = 0; j < wav_param->num_speaker_expected; j++)
        {
          ind = -1;
          impeghd_br_find_geometry_in_speaker_config_3d(
              &geometry_expected[j], &(ptr_binaural_rep->setup_spk_config_3d), &ind);
          if (ind != -1)
          {
            ptr_td_binaural_renderer_param = &ptr_binaural_rep->td_binaural_ren_param;
            binaural_handle->channel_map[j] = ind;
          }
          else
          {
            ptr_td_binaural_renderer_param = NULL;
            break;
          }
        }
        if (ptr_td_binaural_renderer_param != NULL)
        {
          break;
        }
      }
      else
      {
        ptr_td_binaural_renderer_param = &ptr_binaural_rep->td_binaural_ren_param;
        for (j = 0; j < wav_param->num_speaker_expected; j++)
        {
          binaural_handle->channel_map[j] = j;
        }
        break;
      }
    }
  }

  if (ptr_td_binaural_renderer_param == NULL)
  {
    return IA_MPEGH_BINAURAL_INIT_FATAL_INIT_FAIL;
  }

  for (ind = 0; ind < BINAURAL_NB_OUTPUT; ind++)
  {
    binaural_handle->ptr_binaural_signal_out_renderer[ind] =
        &binaural_handle->binaural_signal_out_renderer[ind][0];
  }
  for (ind = 0; ind < ptr_td_binaural_renderer_param->num_channel; ind++)
  {
    binaural_handle->ptr_binaural_signal_in_renderer[ind] =
        &binaural_handle->binaural_signal_in_renderer[ind][0];
  }

  returned_code_8s = impeghd_binaural_struct_init(
      ptr_td_binaural_renderer_param->len_direct, ptr_td_binaural_renderer_param->len_direct,
      ptr_td_binaural_renderer_param->num_diffuse_block,
      ptr_td_binaural_renderer_param->num_channel, BINAURAL_NB_OUTPUT,
      &(binaural_handle->str_binaural));

  if (returned_code_8s < 0)
  {
    return IA_MPEGH_BINAURAL_INIT_FATAL_INIT_FAIL;
  }

  impeghd_binaural_struct_set(&(binaural_handle->str_binaural), ptr_scratch,
                              ptr_td_binaural_renderer_param);

  binaural_handle->ptr_work = pstr_scratch->work_buffer;
  ia_core_coder_memset(binaural_handle->ptr_work, binaural_handle->str_binaural.filter_size);

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_fft_perm_fmt_to_complex_i
*
*  \brief Helper function to re-arrange spectral data.
*
*  \param [in,out] ptr_x_r      Pointer to real part of input data.
*  \param [in,out] ptr_x_im     Pointer to imaginary part of input data.
*  \param [in]  fft_len         FFT size information.
*  \param [in]  ptr_scratch_buf Pointer to scratch buffer.
*
*
*
*/
VOID impeghd_fft_perm_fmt_to_complex_i(FLOAT32 *ptr_x_r, FLOAT32 *ptr_x_im, WORD32 fft_len,
                                       WORD8 *ptr_scratch_buf)
{
  WORD32 ind;
  WORD32 nyquist_ind;
  FLOAT32 *ptr_temp = NULL;
  ia_binaural_scratch *pstr_scratch = (ia_binaural_scratch *)ptr_scratch_buf;

  if (!ptr_x_r)
  {
    return;
  }
  ptr_temp = pstr_scratch->complex_scratch;
  memmove(ptr_temp, ptr_x_r, fft_len * sizeof(*ptr_x_r));
  nyquist_ind = fft_len / 2;
  ptr_x_r[nyquist_ind] = ptr_temp[1];
  ptr_x_r[0] = ptr_temp[0];

  for (ind = 1; ind < nyquist_ind; ind++)
  {
    ptr_x_im[ind] = -1 * ptr_temp[2 * ind + 1];
    ptr_x_im[fft_len - ind] = -1 * ptr_x_im[ind];
    ptr_x_r[ind] = ptr_temp[2 * ind];
    ptr_x_r[fft_len - ind] = ptr_x_r[ind];
  }
}

/**
*  impeghd_binaural_mul_add_perm
*
*  \brief Helper function to carry out arithmetic operations on data.
*
*  \param [in]  ptr_src1c    Pointer to 1st source buffer.
*  \param [in]  ptr_src2c    Pointer to 2nd source buffer.
*  \param [out] ptr_dstc     Pointer to destination buffer.
*  \param [in]  len          Data length information.
*
*  \return error IA_ERRORCODE if any
*
*/
IA_ERRORCODE impeghd_binaural_mul_add_perm(const FLOAT32 *ptr_src1c, const FLOAT32 *ptr_src2c,
                                           FLOAT32 *ptr_dstc, WORD32 len)
{
  WORD32 i;

  *ptr_dstc = ia_mac_flt(*ptr_dstc, *ptr_src1c, *ptr_src2c);
  ptr_src1c++;
  ptr_src2c++;
  ptr_dstc++;

  *ptr_dstc = ia_mac_flt(*ptr_dstc, *ptr_src1c, *ptr_src2c);
  ptr_src1c++;
  ptr_src2c++;
  ptr_dstc++;
  for (i = 1; i < len; i++)
  {
    *ptr_dstc = ia_mac_flt(*ptr_dstc, *ptr_src1c, *ptr_src2c);
    *ptr_dstc = ia_msu_flt(*ptr_dstc, ptr_src1c[1], ptr_src2c[1]);
    ptr_dstc++;

    *ptr_dstc = ia_mac_flt(*ptr_dstc, *ptr_src1c++, ptr_src2c[1]);
    *ptr_dstc = ia_mac_flt(*ptr_dstc, *ptr_src1c, *ptr_src2c);
    ptr_src1c++;
    ptr_src2c++;
    ptr_src2c++;
    ptr_dstc++;
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  impeghd_binaural_struct_process_ld_ola
*
*  \brief Carries out low delay binaural renderer overlap add processing.
*
*  \param [in]  ptr_src      Pointer to real part of input data.
*  \param [out] ptr_dst      Pointer to imaginary part of input data.
*  \param [in]  samples_len  Samples length information.
*  \param [in]  ptr_work     Pointer to interim buffer to store data.
*  \param [in,out] ptr_binaural Pointer to binaural persistent memory structure.
*
*  \return no of sample processed
*
*/
WORD32 impeghd_binaural_struct_process_ld_ola(FLOAT32 **ptr_src, FLOAT32 **ptr_dst,
                                              WORD32 samples_len, FLOAT32 *ptr_work,
                                              ia_binaural_ren_pers_mem_str *ptr_binaural,
                                              FLOAT32 *ptr_scratch_buf)
{
  WORD32 idx;
  WORD32 len = 0, j, k;
  WORD32 inp, out;
  WORD32 n_in_samps;
  WORD32 n_out_samps = 0;
  FLOAT32 *ptr_x_im = NULL;
  FLOAT32 val = 0;
  FLOAT32 *fft_ifft_scratch, *complex_scratch;
  ia_binaural_scratch *pstr_scratch = (ia_binaural_scratch *)ptr_scratch_buf;
  fft_ifft_scratch = pstr_scratch->fft_ifft_scratch;
  complex_scratch = pstr_scratch->complex_scratch;
  if (samples_len + ptr_binaural->memory_index >= ptr_binaural->processed_samples)
  {
    idx = 0;
    n_in_samps = samples_len + ptr_binaural->memory_index;
    ptr_x_im = pstr_scratch->xi;

    while (n_in_samps >= ptr_binaural->processed_samples)
    {
      len =
          (WORD32)(n_in_samps > ptr_binaural->processed_samples ? ptr_binaural->processed_samples
                                                                : n_in_samps);

      len = len - ptr_binaural->memory_index;

      for (inp = 0; inp < ptr_binaural->num_input; inp++)
      {
        memmove(&ptr_binaural->input_mem_buf[inp][ptr_binaural->memory_index], &ptr_src[inp][idx],
                len * sizeof(ptr_src[inp][idx]));
      }
      ia_core_coder_memset(&ptr_binaural->input_mem_buf[0][ptr_binaural->processed_samples],
                           ptr_binaural->filter_size);
      ia_core_coder_memset(
          &ptr_binaural->prev_in_buf[ptr_binaural->write_idx][ptr_binaural->processed_samples],
          ptr_binaural->filter_size);
      ia_core_coder_memset(ptr_x_im, ptr_binaural->fft_size);
      impeghd_cplx_ifft_8k(ptr_binaural->input_mem_buf[0], ptr_x_im, (FLOAT32 *)fft_ifft_scratch,
                           ptr_binaural->ifft_len);
      impeghd_fft_complex_to_perm_fmt_i((ptr_binaural->input_mem_buf[0]), ptr_x_im,
                                        ptr_binaural->fft_size, complex_scratch);

      k = ptr_binaural->write_idx;
      for (j = 0; j < ptr_binaural->spec_mul_size; j++)
      {
        ptr_binaural->prev_in_buf[k][j] =
            ptr_binaural->input_mem_buf[0][j] * ptr_binaural->diffuse_weight[0];
      }

      for (inp = 1; inp < ptr_binaural->num_input; inp++)
      {
        ia_core_coder_memset(&ptr_binaural->input_mem_buf[inp][ptr_binaural->processed_samples],
                             ptr_binaural->filter_size);
        ia_core_coder_memset(ptr_x_im, ptr_binaural->fft_size);

        impeghd_cplx_ifft_8k(ptr_binaural->input_mem_buf[inp], ptr_x_im,
                             (FLOAT32 *)fft_ifft_scratch, ptr_binaural->ifft_len);
        impeghd_fft_complex_to_perm_fmt_i((ptr_binaural->input_mem_buf[inp]), ptr_x_im,
                                          ptr_binaural->fft_size, complex_scratch);

        k = ptr_binaural->write_idx;
        val = ptr_binaural->diffuse_weight[inp];
        for (j = 0; j < ptr_binaural->spec_mul_size; j++)
        {
          ptr_binaural->prev_in_buf[k][j] += (ptr_binaural->input_mem_buf[inp][j] * val);
        }
      }

      for (out = 0; out < ptr_binaural->num_output; out++)
      {
        memmove(ptr_work, &ptr_binaural->output_mem_buf[out][ptr_binaural->processed_samples],
                ptr_binaural->filter_size * sizeof(ptr_work[0]));
        ia_core_coder_memset(ptr_binaural->output_mem_buf[out], ptr_binaural->fft_size);

        for (inp = 0; inp < ptr_binaural->num_input; inp++)
        {
          impeghd_binaural_mul_add_perm(
              ptr_binaural->taps_direct[out][inp], ptr_binaural->input_mem_buf[inp],
              ptr_binaural->output_mem_buf[out], ptr_binaural->direct_process_size[out][inp]);
        }

        k = (ptr_binaural->write_idx + ptr_binaural->diffuse_blocks) %
            (ptr_binaural->diffuse_blocks + 1);

        for (j = 0; j < ptr_binaural->diffuse_blocks; j++)
        {

          impeghd_binaural_mul_add_perm(
              ptr_binaural->taps_diffuse[j][out], ptr_binaural->prev_in_buf[k],
              ptr_binaural->output_mem_buf[out], ptr_binaural->diffuse_process_size[j][out]);

          k = (k + ptr_binaural->diffuse_blocks) % (ptr_binaural->diffuse_blocks + 1);
        }

        impeghd_fft_perm_fmt_to_complex_i((ptr_binaural->output_mem_buf[out]), ptr_x_im,
                                          ptr_binaural->fft_size, (WORD8 *)ptr_scratch_buf);
        impeghd_cplx_fft_16k(ptr_binaural->output_mem_buf[out], ptr_x_im,
                             (FLOAT32 *)fft_ifft_scratch, ptr_binaural->fft_len);

        val = 1.f / ptr_binaural->fft_size;
        for (j = 0; j < ptr_binaural->fft_size; j++)
        {
          ptr_binaural->output_mem_buf[out][j] =
              ia_mul_flt(ptr_binaural->output_mem_buf[out][j], val);
        }
        for (j = 0; j < ptr_binaural->filter_size; j++)
        {
          ptr_binaural->output_mem_buf[out][j] =
              ia_add_flt(ptr_binaural->output_mem_buf[out][j], ptr_work[j]);
        }
      }

      ptr_binaural->write_idx =
          (ptr_binaural->write_idx + 1) % (ptr_binaural->diffuse_blocks + 1);

      for (out = 0; out < ptr_binaural->num_output; out++)
      {
        memmove(&ptr_dst[out][n_out_samps], ptr_binaural->output_mem_buf[out],
                ptr_binaural->processed_samples * sizeof(ptr_dst[0][0]));
      }

      n_out_samps += ptr_binaural->processed_samples;
      n_in_samps = n_in_samps - ptr_binaural->processed_samples;
      samples_len = samples_len - len;
      idx = idx + len;
      ptr_binaural->memory_index = 0;
    }

    for (inp = 0; inp < ptr_binaural->num_input; inp++)
    {
      memmove(ptr_binaural->input_mem_buf[inp], &ptr_src[inp][idx],
              samples_len * sizeof(ptr_src[0][0]));
    }

    ptr_binaural->memory_index = (WORD32)samples_len;
    return n_out_samps;
  }
  else
  {
    for (inp = 0; inp < ptr_binaural->num_input; inp++)
    {
      memmove(&ptr_binaural->input_mem_buf[inp][ptr_binaural->memory_index], ptr_src[inp],
              sizeof(ptr_src[inp][0]) * samples_len);
    }
    ptr_binaural->memory_index = ptr_binaural->memory_index + (WORD32)samples_len;
    return n_out_samps;
  }
}

/** @} */ /* End of BinauralProcess */