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
#include "impeghd_error_codes.h"
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_dec.h"
#include "ia_core_coder_api_defs.h"
#include "ia_core_coder_apicmd_standards.h"

#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_channel.h"
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
#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_cicp_2_geometry.h"
#include <impeghd_fft_ifft.h>
#include "impeghd_obj_ren_dec_defines.h"

#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup BinauralProcess Binaural renderer process
 * @ingroup  BinauralProcess
 * @brief Binaural renderer process
 *
 * @{
 */

/**
*  impeghd_compute_filter_params
*
*  \brief TD Binaural renderer filter params are computed.
*
*  \param [i/o] ptr_param       Pointer to TD binaural renderer params structure.
*  \param [in]  ptr_fir_data    Pointer to Binaural FIR data.
*  \param [in]  n_brir_pairs    Number of BRIR pairs information.
*  \param [in]  ptr_scratch_buf Pointer to scratch buffer.
*
*  \return error IA_ERRORCODE if any
*
*/
IA_ERRORCODE impeghd_compute_filter_params(ia_td_binaural_ren_param_str *ptr_param,
                                           ia_binaural_fir_data_str *ptr_fir_data,
                                           UWORD32 n_brir_pairs, FLOAT32 *ptr_scratch_buf)
{
  WORD32 i_ear, ch, tap;
  WORD32 i, ind_begin_min_i, j, ind_direct_max_i;
  WORD32 len_direct, len_diffuse, ind_diffuse;
  WORD32 ind_diffuse_max_i, diffuse_min_length, num_diffuse_block;
  WORD32 fft_size, ind_start, i_block, ind_nyquist, ind_freq;
  WORD32 ind_freq_max_i, cpy_len, len_win;
  WORD32 num_channel_with_zero_energy;
  FLOAT32 energy, energy_max_i, threshold, cse_inst, cse_inst_inv, cse_inst_inv_freq;
  FLOAT32 *ptr_diffuse_filter = NULL;
  FLOAT32 *ptr_energy_diffuse_filter[2];
  FLOAT32 *ptr_diffuse_weight[2];
  FLOAT32 *ptr_spectrum_r, *ptr_spectrum_i;
  FLOAT32 *ptr_spectrum_mag, *ptr_diffuse_block;
  FLOAT32 *fft_ifft_scratch;
  ia_binaural_scratch *pstr_scratch = (ia_binaural_scratch *)ptr_scratch_buf;
  ptr_spectrum_r = pstr_scratch->spectrum_r;
  ptr_spectrum_i = pstr_scratch->spectrum_i;
  ptr_spectrum_mag = pstr_scratch->spectrum_mag;
  ptr_diffuse_block = pstr_scratch->spectrum_block;
  fft_ifft_scratch = pstr_scratch->fft_ifft_scratch;
  ptr_energy_diffuse_filter[0] = pstr_scratch->diffusion_filter_0;
  ptr_energy_diffuse_filter[1] = pstr_scratch->diffusion_filter_1;
  ptr_diffuse_weight[0] = pstr_scratch->diffuse_weigh_0;
  ptr_diffuse_weight[1] = pstr_scratch->diffuse_weigh_1;
  if (ptr_fir_data->n_taps <= 1024)
  {
    len_direct = 1;
    ptr_param->num_channel = n_brir_pairs;
    ptr_param->begin_delay = 0;

    while (len_direct < ptr_fir_data->n_taps)
    {
      len_direct = len_direct * 2;
    }
    ptr_param->len_direct = len_direct;

    for (ch = 0; ch < ptr_param->num_channel; ++ch)
    {
      for (i_ear = 0; i_ear < 2; ++i_ear)
      {
        ptr_param->ptr_direct_fc[i_ear][ch] = 1;
        ia_core_coder_memset(ptr_param->ptr_taps_direct[i_ear][ch], ptr_param->len_direct);
        ia_core_coder_mem_cpy(ptr_fir_data->taps[i_ear][ch],
                              ptr_param->ptr_taps_direct[i_ear][ch], ptr_fir_data->n_taps);
      }
    }
    ptr_param->num_diffuse_block = 0;
    memset(ptr_param->ptr_diffuse_fc, 0, 2 * MAX_NUM_DIFFUSE_BLOCKS * sizeof(FLOAT32));
    memset(ptr_param->ptr_taps_diffuse, 0,
           2 * MAX_NUM_DIFFUSE_BLOCKS * MAX_LENGTH_DIRECT_FILTER * sizeof(FLOAT32));
    ia_core_coder_memset(ptr_param->ptr_inv_diffuse_weight, ptr_param->num_channel);

    return IA_MPEGH_DEC_NO_ERROR;
  }
  num_channel_with_zero_energy = 0;
  energy_max_i = 0;
  for (i_ear = 0; i_ear < 2; ++i_ear)
  {
    for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
    {
      energy = 0;
      for (tap = 0; tap < ptr_fir_data->n_taps; ++tap)
      {
        energy = ia_mac_flt(energy, ptr_fir_data->taps[i_ear][ch][tap],
                            ptr_fir_data->taps[i_ear][ch][tap]);
      }
      if (energy > energy_max_i)
      {
        energy_max_i = energy;
      }
      if (energy == 0)
      {
        num_channel_with_zero_energy++;
      }
    }
  }
  num_channel_with_zero_energy = num_channel_with_zero_energy >> 1;
  if (num_channel_with_zero_energy == (WORD32)n_brir_pairs)
  {
    return IA_MPEGH_BINAURAL_EXE_FATAL_COMPUTE_FILTER_PARAMETER_FAIL;
  }
  ind_begin_min_i = ptr_fir_data->n_taps - 1;
  threshold = (FLOAT32)pow(10., ENERGY_THRESHOLD / 10);
  for (i_ear = 0; i_ear < 2; ++i_ear)
  {
    for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
    {
      cse_inst = 0;
      i = 0;
      while (i < ptr_fir_data->n_taps)
      {
        cse_inst = ia_mac_flt(cse_inst, ptr_fir_data->taps[i_ear][ch][i],
                              ptr_fir_data->taps[i_ear][ch][i]);
        if (cse_inst / energy_max_i > threshold)
        {
          break;
        }
        ++i;
      }
      if (i < ind_begin_min_i)
      {
        ind_begin_min_i = i;
      }
    }
  }
  ind_direct_max_i = 0;
  for (i_ear = 0; i_ear < 2; ++i_ear)
  {
    for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
    {
      energy = 0;
      for (tap = ind_begin_min_i; tap < ptr_fir_data->n_taps; ++tap)
      {
        energy = ia_mac_flt(energy, ptr_fir_data->taps[i_ear][ch][tap],
                            ptr_fir_data->taps[i_ear][ch][tap]);
      }
      if (energy <= 0)
      {
        j = 0;
      }
      else
      {
        threshold = (FLOAT32)pow(10., DIRECT_ENERGY_THRESH / 10.) * energy;
        j = ptr_fir_data->n_taps - 1;
        cse_inst_inv = 0;
        while (j >= 0)
        {
          cse_inst_inv = ia_mac_flt(cse_inst_inv, ptr_fir_data->taps[i_ear][ch][j],
                                    ptr_fir_data->taps[i_ear][ch][j]);
          if (cse_inst_inv >= threshold)
            break;
          --j;
        }
      }
      if (j > ind_direct_max_i)
      {
        ind_direct_max_i = j;
      }
    }
  }
  ind_direct_max_i -= ind_begin_min_i;
  len_direct = 1;
  while (len_direct < ind_direct_max_i)
  {
    len_direct = len_direct * 2;
  }
  if (len_direct > MAX_LENGTH_DIRECT_FILTER)
  {
    len_direct = MAX_LENGTH_DIRECT_FILTER;
  }

  if (len_direct < MIN_LENGTH_DIRECT_FILTER)
  {
    return IA_MPEGH_BINAURAL_EXE_FATAL_UNSUPPORTED_FILTER_LENGTH;
  }

  fft_size = len_direct;
  ind_nyquist = fft_size / 2;
  for (i_ear = 0; i_ear < 2; ++i_ear)
  {
    for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
    {
      ind_start = ind_begin_min_i;
      if (ind_start + fft_size - 1 >= ptr_fir_data->n_taps)
      {
        ia_core_coder_memset(ptr_spectrum_r, fft_size);
        ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ind_start], ptr_spectrum_r,
                              (ptr_fir_data->n_taps - ind_start));
      }
      else
      {
        ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ind_start], ptr_spectrum_r,
                              fft_size);
      }
      ia_core_coder_memset(ptr_spectrum_i, fft_size);
      impeghd_cplx_ifft_8k(ptr_spectrum_r, ptr_spectrum_i, fft_ifft_scratch, len_direct);

      for (int f = 0; f < fft_size; f++)
      {
        ptr_spectrum_mag[f] = (FLOAT32)ia_sqrt_flt(ptr_spectrum_r[f] * ptr_spectrum_r[f] +
                                                   ptr_spectrum_i[f] * ptr_spectrum_i[f]);
      }

      energy = 0;
      for (tap = 0; tap <= ind_nyquist; ++tap)
      {
        energy = ia_mac_flt(energy, ptr_spectrum_mag[tap], ptr_spectrum_mag[tap]);
      }
      if (energy <= 0)
      {
        ind_freq = 0;
      }
      else
      {
        threshold = (FLOAT32)pow(10., DIRECT_FC_THRESH / 10.) * energy;
        ind_freq = ind_nyquist;
        cse_inst_inv_freq = 0;
        while (ind_freq >= 0)
        {
          cse_inst_inv_freq = ia_mac_flt(cse_inst_inv_freq, ptr_spectrum_mag[ind_freq],
                                         ptr_spectrum_mag[ind_freq]);
          if (cse_inst_inv_freq >= threshold)
            break;
          --ind_freq;
        }
      }
      ptr_param->ptr_direct_fc[i_ear][ch] = (FLOAT32)ind_freq / ind_nyquist;
      ptr_param->ptr_direct_fc[i_ear][ch] =
          (FLOAT32)ia_ceil_flt(ptr_param->ptr_direct_fc[i_ear][ch] * 6.f) / 6.f;
    }
  }

  if (len_direct >= ptr_fir_data->n_taps)
  {
    num_diffuse_block = 0;
  }
  else
  {
    ind_diffuse_max_i = 0;
    for (i_ear = 0; i_ear < 2; ++i_ear)
    {
      for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
      {
        energy = 0;
        for (tap = ind_begin_min_i; tap < ptr_fir_data->n_taps; ++tap)
        {
          energy = ia_mac_flt(energy, ptr_fir_data->taps[i_ear][ch][tap],
                              ptr_fir_data->taps[i_ear][ch][tap]);
        }
        if (energy != 0)
        {
          threshold = ia_mul_flt((FLOAT32)pow(10., DIFFUSE_ENERGY_THRESH / 10.), energy);

          ind_diffuse = ptr_fir_data->n_taps - 1;
          cse_inst_inv = 0;
          while (ind_diffuse >= 0)
          {
            cse_inst_inv = ia_mac_flt(cse_inst_inv, ptr_fir_data->taps[i_ear][ch][ind_diffuse],
                                      ptr_fir_data->taps[i_ear][ch][ind_diffuse]);
            if (cse_inst_inv >= threshold)
            {
              break;
            }
            --ind_diffuse;
          }
        }
        else
        {
          ind_diffuse = 0;
        }
        if (ind_diffuse > ind_diffuse_max_i)
        {
          ind_diffuse_max_i = ind_diffuse;
        }
      }
    }
    diffuse_min_length = ind_diffuse_max_i - (ind_begin_min_i + len_direct - 1);
    num_diffuse_block = 0;
    while (num_diffuse_block * len_direct < diffuse_min_length)
      ++num_diffuse_block;

    if (num_diffuse_block > MAX_NUM_DIFFUSE_BLOCKS)
    {
      num_diffuse_block = MAX_NUM_DIFFUSE_BLOCKS;
    }
    if (num_diffuse_block > 0)
    {
      fft_size = len_direct;
      ind_nyquist = fft_size / 2;
      for (i_block = 0; i_block < num_diffuse_block; ++i_block)
      {
        ind_freq_max_i = 0;
        for (i_ear = 0; i_ear < 2; ++i_ear)
        {
          for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
          {
            ind_start = ind_begin_min_i + (i_block + 1) * fft_size;
            if (ind_start + fft_size - 1 >= ptr_fir_data->n_taps)
            {
              ia_core_coder_memset(ptr_diffuse_block, fft_size);
              ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ind_start], ptr_diffuse_block,
                                    (ptr_fir_data->n_taps - ind_start));
            }
            else
            {
              ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ind_start], ptr_diffuse_block,
                                    fft_size);
            }

            ptr_spectrum_r = ptr_diffuse_block;
            ia_core_coder_memset(ptr_spectrum_i, fft_size);
            impeghd_cplx_ifft_8k(ptr_spectrum_r, ptr_spectrum_i, fft_ifft_scratch, len_direct);

            for (int f = 0; f < fft_size; f++)
            {
              ptr_spectrum_mag[f] = (FLOAT32)ia_sqrt_flt(ptr_spectrum_r[f] * ptr_spectrum_r[f] +
                                                         ptr_spectrum_i[f] * ptr_spectrum_i[f]);
            }

            energy = 0; /* Recompute filter energy (to Nyquist freq only) */
            for (tap = 0; tap <= ind_nyquist; ++tap)
            {
              energy = ia_mac_flt(energy, ptr_spectrum_mag[tap], ptr_spectrum_mag[tap]);
            }
            if (energy <= 0)
            {
              ind_freq = 0;
            }
            else
            {
              threshold = ia_mul_flt(powf(10., DIFFUSE_FC_THRESH / 10.), energy);
              ind_freq = ind_nyquist;
              cse_inst_inv_freq = 0;
              while (ind_freq >= 0)
              {
                cse_inst_inv_freq = ia_mac_flt(cse_inst_inv_freq, ptr_spectrum_mag[ind_freq],
                                               ptr_spectrum_mag[ind_freq]);
                if (cse_inst_inv_freq >= threshold)
                  break;
                --ind_freq;
              }
            }

            if (ind_freq > ind_freq_max_i)
            {
              ind_freq_max_i = ind_freq;
            }
          }
          ptr_param->ptr_diffuse_fc[i_ear][i_block] = (FLOAT32)ind_freq_max_i / ind_nyquist;
          if (ptr_param->ptr_diffuse_fc[i_ear][i_block] != 0)
          {
            ptr_param->ptr_diffuse_fc[i_ear][i_block] =
                (FLOAT32)ia_ceil_flt(ptr_param->ptr_diffuse_fc[i_ear][i_block] * 6.f) / 6.f;
          }
          else
          {
            ptr_param->ptr_diffuse_fc[i_ear][i_block] = 1.f / 6.f;
          }
          ind_freq_max_i = 0;
        }
      }
    }
    if (num_diffuse_block > 0)
    {
      len_diffuse = num_diffuse_block * len_direct;
      ptr_diffuse_filter = pstr_scratch->diffuse_filter;
      for (i_ear = 0; i_ear < 2; ++i_ear)
      {

        ia_core_coder_memset(pstr_scratch->diffusion_filter_wmc[i_ear], len_diffuse);
      }
      for (i_ear = 0; i_ear < 2; ++i_ear)
      {
        for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
        {
          ind_start = ind_begin_min_i + len_direct;
          if (ind_start + len_diffuse - 1 >= ptr_fir_data->n_taps)
          {
            ia_core_coder_memset(ptr_diffuse_filter, len_diffuse);
            ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ind_start], ptr_diffuse_filter,
                                  (ptr_fir_data->n_taps - ind_start));
          }
          else
          {
            ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ind_start], ptr_diffuse_filter,
                                  len_diffuse);
          }
          len_win = 512;
          if (len_diffuse >= len_win)
          {
            for (tap = 0; tap < len_win; ++tap)
            {
              ptr_diffuse_filter[len_diffuse - 1 - tap] =
                  ia_mul_flt(ptr_diffuse_filter[len_diffuse - 1 - tap],
                             (FLOAT32)(cos(PI * (1 - ((FLOAT32)tap + 1.) / len_win)) + 1) / 2);
            }
          }
          energy = 0;
          for (tap = 0; tap < len_diffuse; ++tap)
          {
            energy = ia_mac_flt(energy, ptr_diffuse_filter[tap], ptr_diffuse_filter[tap]);
          }
          ptr_energy_diffuse_filter[i_ear][ch] = energy;

          if (ptr_energy_diffuse_filter[i_ear][ch] > 0)
          {
            for (tap = 0; tap < len_diffuse; ++tap)
            {
              pstr_scratch->diffusion_filter_wmc[i_ear][tap] =
                  ia_add_flt(pstr_scratch->diffusion_filter_wmc[i_ear][tap],
                             (FLOAT32)(ptr_diffuse_filter[tap] / ia_sqrt_flt(energy) /
                                       (n_brir_pairs - num_channel_with_zero_energy)));
            }
          }
        }
      }

      for (i_ear = 0; i_ear < 2; ++i_ear)
      {
        energy = 0;
        for (tap = 0; tap < len_diffuse; ++tap)
        {
          energy = ia_mac_flt(energy, pstr_scratch->diffusion_filter_wmc[i_ear][tap],
                              pstr_scratch->diffusion_filter_wmc[i_ear][tap]);
        }

        for (ch = 0; ch < (WORD32)n_brir_pairs; ++ch)
        {
          if (ptr_energy_diffuse_filter[i_ear][ch] <= 0)
          {
            ptr_diffuse_weight[i_ear][ch] = -1;
          }
          else
          {
            ptr_diffuse_weight[i_ear][ch] =
                (FLOAT32)ia_sqrt_flt(energy / ptr_energy_diffuse_filter[i_ear][ch]);
          }
        }
      }
      for (i_ear = 0; i_ear < 2; ++i_ear)
      {
        for (tap = 0; tap < len_diffuse; ++tap)
        {
          pstr_scratch->diffusion_filter_wmc[i_ear][tap] *= 0.501187233627272f;
        }
      }
    }
  }
  ptr_param->begin_delay = ind_begin_min_i;
  ptr_param->len_direct = len_direct;
  ptr_param->num_channel = n_brir_pairs;
  ptr_param->num_diffuse_block = num_diffuse_block;
  cpy_len = ptr_fir_data->n_taps < ptr_param->len_direct
                ? ptr_fir_data->n_taps
                : ptr_param->len_direct; /* take min of the 2 len */
  for (i_ear = 0; i_ear < 2; ++i_ear)
  {
    for (ch = 0; ch < ptr_param->num_channel; ++ch)
    {
      ia_core_coder_memset(ptr_param->ptr_taps_direct[i_ear][ch], ptr_param->len_direct);
      ia_core_coder_mem_cpy(&ptr_fir_data->taps[i_ear][ch][ptr_param->begin_delay],
                            ptr_param->ptr_taps_direct[i_ear][ch], cpy_len);
    }
  }

  if (num_diffuse_block <= 0)
  {
    memset(ptr_param->ptr_diffuse_fc, 0, 2 * MAX_NUM_DIFFUSE_BLOCKS * sizeof(FLOAT32));
    memset(ptr_param->ptr_taps_diffuse, 0,
           2 * MAX_NUM_DIFFUSE_BLOCKS * MAX_LENGTH_DIRECT_FILTER * sizeof(FLOAT32));
    ia_core_coder_memset(ptr_param->ptr_inv_diffuse_weight, ptr_param->num_channel);
  }
  else
  {
    for (ch = 0; ch < ptr_param->num_channel; ++ch)
    {
      if (ptr_diffuse_weight[0][ch] == -1 || ptr_diffuse_weight[1][ch] == -1)
      {
        ptr_param->ptr_inv_diffuse_weight[ch] = 0;
      }
      else
      {
        ptr_param->ptr_inv_diffuse_weight[ch] =
            2.f / ia_add_flt(ptr_diffuse_weight[0][ch], ptr_diffuse_weight[1][ch]);
      }
    }
    for (i_ear = 0; i_ear < 2; ++i_ear)
    {
      for (i_block = 0; i_block < ptr_param->num_diffuse_block; ++i_block)
      {
        ind_start = i_block * ptr_param->len_direct;
        ia_core_coder_mem_cpy(&pstr_scratch->diffusion_filter_wmc[i_ear][ind_start],
                              ptr_param->ptr_taps_diffuse[i_block][i_ear], ptr_param->len_direct);
      }
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/** @} */ /* End of BinauralProcess */