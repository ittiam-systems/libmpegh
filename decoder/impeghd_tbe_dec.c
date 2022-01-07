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
#include <string.h>

#include <impeghd_type_def.h>
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_tbe_dec.h"
#include "impeghd_tbe_rom.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_config.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_tbe_data
 *
 *  \brief Function to read TD-bandwidth extenstion bit stream data.
 *
 *  \param [out] tbe_dec_data    Pointer to TBE dec data structure.
 *  \param [in]  ptr_it_bit_buff Pointer to bit buffer structure.
 *  \param [in]  tbe_frame_class TBE frame class type.
 *
 *
 *
 */
VOID ia_core_coder_tbe_data(ia_usac_tbe_data_handle tbe_dec_data,
                            ia_bit_buf_struct *ptr_it_bit_buff, WORD32 tbe_frame_class)
{
  WORD32 sf_idx;
  ia_usac_tbe_bitstream_handle bs_handle;

  sf_idx = (1 == tbe_frame_class) ? 1 : 0;
  bs_handle = &tbe_dec_data->bit_stream[sf_idx];
  bs_handle->harm_ext_mode = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 1);
  bs_handle->frame_gain_idx = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 5);
  bs_handle->sub_gain_idx = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 5);
  bs_handle->lsf_idx[0] = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 7);
  bs_handle->lsf_idx[1] = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 7);

  if (tbe_dec_data->bit_stream[sf_idx].harm_ext_mode != 0)
  {
    bs_handle->nl_config = 1;
  }
  else
  {
    bs_handle->hr_config = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 1);
    bs_handle->nl_config = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 1);
    bs_handle->idx_mix_config = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 2);

    if (bs_handle->hr_config != 1)
    {
      bs_handle->idx_shb_exc_resp[0] = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 7);
      bs_handle->idx_shb_exc_resp[1] = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 4);
    }
    else
    {
      bs_handle->idx_shb_fr_gain = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 6);
      bs_handle->idx_res_sub_gains = ia_core_coder_read_bits_buf(ptr_it_bit_buff, 5);
    }
  }

  return;
}

/**
 *  ia_core_coder_fir_filter
 *
 *  \brief Function for FIR Filter implementation.
 *
 *  \param [in,out] ptr_input       Pointer to Input / Output buffer.
 *  \param [in]     ptr_filt_coeff  Pointer to filter coefficient array.
 *  \param [in]     ptr_filter_mem  Pointer to filter states.
 *  \param [in]     order           Order of filter.
 *  \param [in]     length          Length of Input buffer.
 *
 *
 *
 */
static VOID ia_core_coder_fir_filter(FLOAT32 *ptr_input, FLOAT32 *ptr_filt_coeff,
                                     FLOAT32 *ptr_filter_mem, WORD32 order, WORD32 length)
{
  WORD32 len, order_idx;
  FLOAT32 temp, *ptr_mem, *ptr_mem_prev, *ptr_output;
  const FLOAT32 *ptr_coeff;

  ptr_output = ptr_input;
  for (len = 0; len < length; len++)
  {
    temp = *ptr_input;
    *ptr_output = ia_mul_flt(*ptr_filt_coeff, (*ptr_input++));
    ptr_coeff = ptr_filt_coeff + order;
    ptr_mem = ptr_filter_mem + order - 1;
    ptr_mem_prev = ptr_mem - 1;
    for (order_idx = order; order_idx > 1; order_idx--)
    {
      *ptr_output += ia_mul_flt((*ptr_coeff--), *ptr_mem);
      *ptr_mem-- = *ptr_mem_prev--;
    }
    *ptr_output++ += ia_mul_flt(*ptr_coeff, *ptr_mem);
    *ptr_filter_mem = temp;
  }

  return;
}

/**
 *  ia_core_coder_tbe_synth_plus
 *
 *  \brief Function for TBE Highband LP synthesis.
 *
 *  \param [in]  ptr_coeff         Pointer to coefficients buffer used during processing.
 *  \param [in]  length            Length parameter used by synthesis block.
 *  \param [in]  ptr_input         Pointer to input buffer.
 *  \param [out] ptr_output        Pointer to output buffer.
 *  \param [in]  ptr_mem           Pointer to states memory.
 *  \param [in]  ptr_synth_scratch Pointer to sratch memory used for intermediate storage.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_synth_plus(FLOAT32 *ptr_coeff, WORD32 length, FLOAT32 *ptr_input,
                                         FLOAT32 *ptr_output, FLOAT32 *ptr_mem,
                                         FLOAT32 *ptr_synth_scratch)
{
  WORD32 len, idx;
  FLOAT32 *buf, s, *yy;

  buf = ptr_synth_scratch;
  ia_core_coder_mem_cpy(ptr_mem, buf, length);
  yy = &buf[length];
  for (len = 0; len < LEN_SUBFR; len++)
  {
    s = ptr_input[len];
    for (idx = 1; idx <= length; idx++)
    {
      s = ia_msu_flt(s, ptr_coeff[idx], yy[len - idx]);
    }
    yy[len] = s;
    ptr_output[len] = s;
  }
  ia_core_coder_mem_cpy(&yy[LEN_SUBFR - length], ptr_mem, length);

  return;
}

/**
 *  ia_core_coder_tbe_random_sign
 *
 *  \brief Helper function to calculate random sign value.
 *
 *  \param [in] ptr_seed            Pointer to seed value used by the function.
 *  \param [in] ptr_scaled_abs_seed Pointer to scaled absolute seed value used by the function.
 *
 *  \return WORD32                  1 or -1 based on the calculation.
 *
 */
static WORD32 ia_core_coder_tbe_random_sign(WORD16 *ptr_seed, WORD16 *ptr_scaled_abs_seed)
{
  WORD32 sign = 1;

  *ptr_seed = (WORD16)(*ptr_seed * RAND_SIGN_CONST_1 + RAND_SIGN_CONST_2);
  *ptr_scaled_abs_seed = (WORD16)ia_fabs_flt(((FLOAT32)(*ptr_seed) * RAND_SIGN_CONST_3));
  if (*ptr_seed < 0)
  {
    sign = -1;
  }

  return sign;
}

/**
 *  ia_core_coder_tbe_gamma_shaping
 *
 *  \brief Helper function that applies gain(in geometric progression) and does shaping of
 * ptr_input.
 *
 *  \param [in]  ptr_input  Pointer to input buffer.
 *  \param [out] ptr_output Pointer to output buffer.
 *  \param [in]  gamma      Step value used by shaping algo.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_gamma_shaping(FLOAT32 *ptr_input, FLOAT32 *ptr_output,
                                            FLOAT32 gamma)
{
  WORD32 order_idx;
  FLOAT32 gain;

  ptr_output[0] = ptr_input[0];
  gain = gamma;
  for (order_idx = 1; order_idx <= LPC_ORDER_TBE; order_idx++)
  {
    ptr_output[order_idx] = ia_mul_flt(gain, ptr_input[order_idx]);
    gain = ia_mul_flt(gain, gamma);
  }

  return;
}

/**
 *  ia_core_coder_tbe_gen_random_vec
 *
 *  \brief Helper function to generate random vector.
 *
 *  \param [out] ptr_output Pointer to output buffer.
 *  \param [in]  length     Length of output buffer.
 *  \param [in]  ptr_seed   Seed value used by the processing function.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_gen_random_vec(FLOAT32 *ptr_output, WORD32 length, WORD16 *ptr_seed)
{
  WORD32 len;
  WORD16 idx1, idx2, tmp;
  FLOAT32 gain1 = RAND_VEC_GAIN_1, gain2 = RAND_VEC_GAIN_2;

  ia_core_coder_tbe_random_sign(&ptr_seed[0], &idx1);
  ia_core_coder_tbe_random_sign(&ptr_seed[1], &idx2);

  while (idx2 == idx1)
  {
    ia_core_coder_tbe_random_sign(&ptr_seed[1], &idx2);
  }
  if (ia_core_coder_tbe_random_sign(&ptr_seed[0], &tmp) < 0)
  {
    gain1 = ia_negate_flt(gain1);
  }
  if (ia_core_coder_tbe_random_sign(&ptr_seed[1], &tmp) < 0)
  {
    gain2 = ia_negate_flt(gain2);
  }
  for (len = 0; len < length; len++)
  {
    idx1 &= 0x00ff;
    idx2 &= 0x00ff;
    ptr_output[len] = ia_mac_flt(ia_mul_flt(gain1, ia_core_coder_gaus_dico[idx1]), gain2,
                                 ia_core_coder_gaus_dico[idx2]);
    idx1++;
    idx2++;
  }

  return;
}

/**
 *  ia_core_coder_tbe_calc_energy
 *
 *  \brief Helper function to calculate input energy.
 *
 *  \param [in] ptr_input Pointer to input buffer.
 *  \param [in] length    Length of the input.
 *
 *  \return WORD32        Input signal energy value.
 *
 */
static FLOAT32 ia_core_coder_tbe_calc_energy(FLOAT32 *ptr_input, WORD32 length)
{
  WORD32 len;
  FLOAT32 ener = 0.0f;

  for (len = 0; len < length; len++)
  {
    ener += ia_mul_flt(ptr_input[len], ptr_input[len]);
  }

  return ener;
}

/**
 *  ia_core_coder_tbe_find_mean
 *
 *  \brief Helper function to find mean of input signal values.
 *
 *  \param [in] ptr_input Pointer to input buffer.
 *  \param [in] length    Buffer length.
 *
 *  \return WORD32        Mean value of the input signal.
 *
 */
static FLOAT32 ia_core_coder_tbe_find_mean(FLOAT32 *ptr_input, WORD32 length)
{
  FLOAT32 mean = 0.0f;
  WORD32 len;

  for (len = 0; len < length; len++)
  {
    mean = ia_add_flt(mean, ptr_input[len]);
  }
  mean /= (FLOAT32)length;

  return mean;
}

/**
 *  ia_core_coder_tbe_lsp_lsf_conv
 *
 *  \brief Function to LSP to LSF and vice-versa conversion.
 *
 *  \param [in]  ptr_in   Pointer to line spectral pairs buffer.
 *  \param [out] ptr_out  Pointer to line spectral frequencies buffer.
 *  \param [in]  conv_dir Flag that indicates LSP to LSF conversion or vice versa.
 *
 *
 *
 */

static VOID ia_core_coder_tbe_lsp_lsf_conv(FLOAT32 *ptr_in, FLOAT32 *ptr_out, WORD32 conv_dir)
{
  WORD32 order_idx;

  if (conv_dir != 1)
  {
    for (order_idx = 0; order_idx < LPC_ORDER_TBE; order_idx++)
    {
      ptr_out[order_idx] =
          (FLOAT32)ia_mul_double_flt(ONEBYPIx2, (FLOAT32)acos(ptr_in[order_idx]));
    }
  }
  else
  {
    for (order_idx = 0; order_idx < LPC_ORDER_TBE; order_idx++)
    {
      ptr_out[order_idx] = (FLOAT32)cos(ptr_in[order_idx] * PIx2);
    }
  }

  return;
}

/**
 *  ia_core_coder_tbe_shift_lsf
 *
 *  \brief Helper function to apply shift on LSF.
 *
 *  \param [in,out] ptr_lsf Pointer to LSF buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_shift_lsf(FLOAT32 *ptr_lsf)
{
  WORD32 order_idx, exit = 0;
  FLOAT32 delta = 0.0f;

  do
  {
    exit = 1;
    delta = ptr_lsf[0];
    if (ia_lt_flt(delta, TBE_DELTA))
    {
      exit = 0;
      delta = ia_sub_flt(delta, TBE_DELTA_DEV);
      ptr_lsf[0] = ia_sub_flt(ptr_lsf[0], delta);
    }
    for (order_idx = 1; order_idx < LPC_ORDER_TBE; order_idx++)
    {
      delta = ia_sub_flt(ptr_lsf[order_idx], ptr_lsf[order_idx - 1]);
      if (ia_lt_flt(delta, TBE_DELTA))
      {
        exit = 0;
        delta = ia_sub_flt(delta, TBE_DELTA_DEV);
        delta = ia_mul_flt(delta, 0.5f);
        ptr_lsf[order_idx - 1] = ia_add_flt(ptr_lsf[order_idx - 1], delta);
        ptr_lsf[order_idx] = ia_sub_flt(ptr_lsf[order_idx], delta);
      }
    }
    delta = ia_sub_flt(0.5f, ptr_lsf[LPC_ORDER_TBE_MINUS1]);
    if (ia_lt_flt(delta, TBE_DELTA))
    {
      exit = 0;
      delta = ia_sub_flt(delta, TBE_DELTA_DEV);
      ptr_lsf[LPC_ORDER_TBE - 1] = ia_add_flt(ptr_lsf[LPC_ORDER_TBE - 1], delta);
    }
  } while (exit == 0);

  return;
}

/**
 *  ia_core_coder_tbe_interpol
 *
 *  \brief Function to perform interpolation.
 *
 *  \param [in]  ptr_in    Pointer to input buffer.
 *  \param [in]  ptr_state Pointer to interpolation states.
 *  \param [in]  length    Length of the output.
 *  \param [out] ptr_out   Pointer to output buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_interpol(FLOAT32 *ptr_in, FLOAT32 *ptr_state, WORD32 length,
                                       FLOAT32 *ptr_out)
{
  WORD32 len, num;
  FLOAT32 tmp[AP_NUM - 1] = {0.0f};

  for (len = 0; len < length; len++)
  {
    tmp[0] = ia_mac_flt(ptr_state[0], ia_core_coder_ap_filter2[0], ptr_in[len]);
    ptr_state[0] = ia_msu_flt(ptr_in[len], ia_core_coder_ap_filter2[0], tmp[0]);

    for (num = 1; num < AP_NUM - 1; num++)
    {
      tmp[num] = ia_mac_flt(ptr_state[num], ia_core_coder_ap_filter2[num], tmp[num - 1]);
      ptr_state[num] = ia_msu_flt(tmp[num - 1], ia_core_coder_ap_filter2[num], tmp[num]);
    }
    ptr_out[2 * len + 1] =
        ia_mac_flt(ptr_state[AP_NUM - 1], ia_core_coder_ap_filter2[AP_NUM - 1], tmp[AP_NUM - 2]);
    ptr_state[AP_NUM - 1] =
        ia_msu_flt(tmp[AP_NUM - 2], ia_core_coder_ap_filter2[AP_NUM - 1], ptr_out[2 * len + 1]);
  }

  for (len = 0; len < length; len++)
  {
    tmp[0] = ia_mac_flt(ptr_state[AP_NUM], ia_core_coder_ap_filter1[0], ptr_in[len]);
    ptr_state[AP_NUM] = ia_msu_flt(ptr_in[len], ia_core_coder_ap_filter1[0], tmp[0]);

    for (num = 1; num < AP_NUM - 1; num++)
    {
      tmp[num] = ia_mac_flt(ptr_state[AP_NUM + num], ia_core_coder_ap_filter1[num], tmp[num - 1]);
      ptr_state[AP_NUM + num] = ia_msu_flt(tmp[num - 1], ia_core_coder_ap_filter1[num], tmp[num]);
    }
    ptr_out[2 * len] = ia_mac_flt(ptr_state[(2 * AP_NUM) - 1],
                                  ia_core_coder_ap_filter1[AP_NUM - 1], tmp[AP_NUM - 2]);
    ptr_state[(2 * AP_NUM) - 1] =
        ia_msu_flt(tmp[AP_NUM - 2], ia_core_coder_ap_filter1[AP_NUM - 1], ptr_out[2 * len]);
  }

  return;
}

/**
 *  ia_core_coder_tbe_nonlinearity
 *
 *  \brief Function for TBE non linear processing.
 *
 *  \param [in]  bit_stream               Pointer to USAC bitstream handle.
 *  \param [in]  ptr_input                Pointer to input buffer.
 *  \param [out] ptr_output               Pointer to output buffer.
 *  \param [in]  ptr_old_tbe_exc_extended Pointer to previous TBE excitation data.
 *  \param [in]  ptr_scratch              Pointer to scratch buffer.
 *  \param [in]  length                   Pointer to TBE data length.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_nonlinearity(ia_usac_tbe_bitstream_handle bit_stream,
                                           FLOAT32 *ptr_input, FLOAT32 *ptr_output,
                                           FLOAT32 *ptr_old_tbe_exc_extended,
                                           FLOAT32 *ptr_scratch, WORD32 length)
{
  WORD32 len, nl_level;
  FLOAT32 *p_out, in_energy, out_energy, temp, *temp_buff, *temp_buff1;

  temp_buff = ptr_scratch;
  temp_buff1 = temp_buff + SCRATCH_IDX_NL_LEVEL_2;
  nl_level = bit_stream->nl_config;
  if (bit_stream->harm_ext_mode == 1)
  {
    nl_level = 0;
    if ((bit_stream->sub_gain_idx & 1) == 1)
    {
      nl_level = 1;
    }
  }
  if (bit_stream->nl_config == 0 && bit_stream->idx_mix_config <= 1 &&
      bit_stream->harm_ext_mode == 0)
  {
    nl_level = 2;
  }

  p_out = ptr_output + NL_BUFF_OFFSET;
  ia_core_coder_mem_cpy(ptr_old_tbe_exc_extended, ptr_output, NL_BUFF_OFFSET);

  switch (nl_level)
  {
  case 0:
  {
    in_energy = ia_core_coder_tbe_calc_energy(ptr_input, length);
    for (len = 0; len < length; len++)
    {
      *p_out++ = ia_mul_flt(ia_mul_flt(ptr_input[len], ptr_input[len]),
                            (ptr_input[len] > 0 ? 1.0f : -1.0f));
    }
    out_energy = ia_core_coder_tbe_calc_energy(ptr_output + NL_BUFF_OFFSET, length);

    temp = 0.0f;
    if (!ia_eq_flt(out_energy, 0.0f))
    {
      temp = (FLOAT32)ia_sqrt_flt(in_energy / out_energy);
    }

    p_out = ptr_output + NL_BUFF_OFFSET;
    for (len = 0; len < length; len++)
    {
      *p_out = ia_mul_flt(*p_out, temp);
      p_out++;
    }
    break;
  }
  case 1:
  {
    for (len = 0; len < length; len++)
    {
      *p_out++ = (FLOAT32)ia_fabs_flt(ptr_input[len]);
    }
    break;
  }
  case 2:
  {
    in_energy = ia_core_coder_tbe_calc_energy(ptr_input, length);
    for (len = 0; len < length; len++)
    {
      *p_out++ = ia_mul_flt(ia_mul_flt(ptr_input[len], ptr_input[len]),
                            (ptr_input[len] > 0 ? 1.0f : -1.0f));
    }
    out_energy = ia_core_coder_tbe_calc_energy(ptr_output + NL_BUFF_OFFSET, length);

    temp = 0.0f;
    if (!ia_eq_flt(out_energy, 0.0f))
    {
      temp = (FLOAT32)ia_sqrt_flt(in_energy / out_energy);
    }

    p_out = ptr_output + NL_BUFF_OFFSET;
    *p_out = ia_mul_flt(*p_out, temp);
    temp_buff[0] = p_out[0];
    p_out++;

    *p_out = ia_mul_flt(*p_out, temp);
    temp_buff[1] = ia_add_flt((*p_out), ia_msu_flt((2 * p_out[-1]), 0.94f, temp_buff[0]));
    p_out++;

    for (len = 2; len < length; len++)
    {
      *p_out *= temp;
      temp_buff[len] = ia_add_flt(ia_add_flt(p_out[0], (2 * p_out[-1])),
                                  ia_msu_flt(ia_msu_flt(p_out[-2], 0.94f, temp_buff[len - 1]),
                                             0.33f, temp_buff[len - 2]));
      p_out++;
    }

    len = 0;
    p_out = ptr_output + NL_BUFF_OFFSET;
    temp_buff1[len] = ((FLOAT32)ia_fabs_flt(ptr_input[len]));
    *p_out++ = ia_mac_flt(ia_mul_flt(VAL_A, temp_buff[len]), VAL_B, temp_buff1[len]);
    len++;

    temp_buff1[len] = ia_msu_flt(ia_mac_flt((FLOAT32)ia_fabs_flt(ptr_input[len]), -2.0f,
                                            ((FLOAT32)ia_fabs_flt(ptr_input[len - 1]))),
                                 0.94f, temp_buff1[len - 1]);
    *p_out++ = ia_mac_flt(ia_mul_flt(VAL_A, temp_buff[len]), VAL_B, temp_buff1[len]);
    len++;

    for (len = 2; len < length; len++)
    {
      temp_buff1[len] = ia_msu_flt(
          ia_msu_flt(ia_add_flt(ia_mac_flt(((FLOAT32)ia_fabs_flt(ptr_input[len])), -2.0f,
                                           ((FLOAT32)ia_fabs_flt(ptr_input[len - 1]))),
                                ((FLOAT32)ia_fabs_flt(ptr_input[len - 2]))),
                     0.94f, temp_buff1[len - 1]),
          0.33f, temp_buff1[len - 2]);
      *p_out++ = ia_mac_flt(ia_mul_flt(VAL_A, temp_buff[len]), VAL_B, temp_buff1[len]);
    }
    break;
  }
  default:
  {
    /* For invalid cases */
    break;
  }
  }

  ia_core_coder_mem_cpy(ptr_output + 2 * LEN_FRAME, ptr_old_tbe_exc_extended, NL_BUFF_OFFSET);
  return;
}

/**
 *  ia_core_coder_tbe_conv_lsf_kernel
 *
 *  \brief Helper function used by LSF 2 LPC converter function.
 *
 *  \param [in,out] ptr_set_a Pointer to intermediate buffer set ptr_coeff.
 *  \param [in,out] ptr_set_b Pointer to intermediate buffer set b.
 *  \param [in]     ptr_p     Pointer to even parts of LSP.
 *  \param [in]     ptr_q     Pointer to odd parts of LSP.
 *  \param [in]     length    Order of the data to be processed.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_conv_lsf_kernel(FLOAT32 *ptr_set_a, FLOAT32 *ptr_set_b,
                                              FLOAT32 *ptr_p, FLOAT32 *ptr_q, WORD32 length)
{
  WORD32 len;
  FLOAT32 *set_a1, *set_a2, *set_a3, *set_b1, *set_b2, *set_b3;

  set_a1 = &ptr_set_a[0];
  set_a2 = &ptr_set_a[LPC_ORDER_TBE];
  set_a3 = &ptr_set_a[2 * LPC_ORDER_TBE];
  set_b1 = &ptr_set_b[0];
  set_b2 = &ptr_set_b[LPC_ORDER_TBE];
  set_b3 = &ptr_set_b[2 * LPC_ORDER_TBE];

  for (len = 0; len < length; len++)
  {
    set_a1[len + 1] =
        ia_add_flt(ia_msu_flt(set_a1[len], (2 * ptr_p[len]), set_a2[len]), set_a3[len]);
    set_b1[len + 1] =
        ia_add_flt(ia_msu_flt(set_b1[len], (2 * ptr_q[len]), set_b2[len]), set_b3[len]);
    set_a3[len] = set_a2[len];
    set_a2[len] = set_a1[len];
    set_b3[len] = set_b2[len];
    set_b2[len] = set_b1[len];
  }

  return;
}

/**
 *  ia_core_coder_tbe_lsf2lpc_conv
 *
 *  \brief Function for TBE LSF to LPC converter.
 *
 *  \param [out] ptr_lpc Pointer to LPC buffer.
 *  \param [in]  ptr_lsf Pointer to LSF buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_lsf2lpc_conv(FLOAT32 *ptr_lpc, FLOAT32 *ptr_lsf)
{
  WORD32 order_idx, order_rs1;
  FLOAT32 tmp, *plpc, p[LPC_ORDER_TBE] = {0.0f}, q[LPC_ORDER_TBE] = {0.0f},
                      set_a[3][LPC_ORDER_TBE], set_b[3][LPC_ORDER_TBE];

  ia_core_coder_memset(&set_a[0][0], sizeof(set_a) / sizeof(set_a[0][0]));
  ia_core_coder_memset(&set_b[0][0], sizeof(set_b) / sizeof(set_b[0][0]));

  if (ia_lteq_flt(0.5f, ptr_lsf[LPC_ORDER_TBE - 1]) || ia_lteq_flt(ptr_lsf[0], 0.0f))
  {
    if (ia_lteq_flt(ptr_lsf[0], 0.0f))
    {
      ptr_lsf[0] = 0.022f;
    }
    if (ia_lteq_flt(0.5f, ptr_lsf[LPC_ORDER_TBE - 1]))
    {
      ptr_lsf[LPC_ORDER_TBE - 1] = 0.499f;
    }
    tmp = ia_mul_flt(ia_sub_flt(ptr_lsf[LPC_ORDER_TBE - 1], ptr_lsf[0]),
                     ia_core_coder_recip_order[LPC_ORDER_TBE]);
    for (order_idx = 1; order_idx < LPC_ORDER_TBE; order_idx++)
    {
      ptr_lsf[order_idx] = ia_add_flt(ptr_lsf[order_idx - 1], tmp);
    }
  }

  order_rs1 = LPC_ORDER_TBE >> 1;
  for (order_idx = 0; order_idx < order_rs1; order_idx++)
  {
    p[order_idx] = (FLOAT32)cos(ia_mul_flt(6.2832f, ptr_lsf[2 * order_idx]));
    q[order_idx] = (FLOAT32)cos(ia_mul_flt(6.2832f, ptr_lsf[2 * order_idx + 1]));
  }

  set_a[0][0] = set_b[0][0] = 0.25f;
  ia_core_coder_tbe_conv_lsf_kernel(&set_a[0][0], &set_b[0][0], p, q, order_rs1);

  set_a[0][0] = 0.25f;
  set_b[0][0] = ia_negate_flt(0.25f);
  ia_core_coder_tbe_conv_lsf_kernel(&set_a[0][0], &set_b[0][0], p, q, order_rs1);

  plpc = &(ptr_lpc[1]);
  plpc[0] = ia_mul_flt(2, ia_add_flt(set_a[0][order_rs1], set_b[0][order_rs1]));
  for (order_idx = 2; order_idx <= LPC_ORDER_TBE; order_idx++)
  {
    set_a[0][0] = set_b[0][0] = 0.0f;
    ia_core_coder_tbe_conv_lsf_kernel(&set_a[0][0], &set_b[0][0], p, q, order_rs1);
    plpc[order_idx - 1] = ia_mul_flt(2.0f, ia_add_flt(set_a[0][order_rs1], set_b[0][order_rs1]));
  }

  return;
}

/**
 *  ia_core_coder_tbe_decim_allpass_steep
 *
 *  \brief Function to decimate using all pass filter used by TBE module.
 *
 *  \param [in]  ptr_in    Pointer to input buffer.
 *  \param [in]  ptr_state Pointer to buffer states.
 *  \param [in]  length    Length of the input buffer.
 *  \param [out] ptr_out   Pointer to output buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_decim_allpass_steep(FLOAT32 *ptr_in, FLOAT32 *ptr_state,
                                                  WORD32 length, FLOAT32 *ptr_out)
{
  WORD32 num, len;
  FLOAT32 temp[AP_NUM];

  for (len = 0; len < (length >> 1); len++)
  {
    temp[0] = ia_mac_flt(ptr_state[0], ia_core_coder_ap_filter1[0], ptr_in[2 * len]);
    ptr_state[0] = ia_msu_flt(ptr_in[2 * len], ia_core_coder_ap_filter1[0], temp[0]);
    for (num = 1; num < AP_NUM - 1; num++)
    {
      temp[num] = ia_mac_flt(ptr_state[num], ia_core_coder_ap_filter1[num], temp[num - 1]);
      ptr_state[num] = ia_msu_flt(temp[num - 1], ia_core_coder_ap_filter1[num], temp[num]);
    }
    ptr_out[len] =
        ia_mac_flt(ptr_state[AP_NUM - 1], ia_core_coder_ap_filter1[AP_NUM - 1], temp[AP_NUM - 2]);
    ptr_state[AP_NUM - 1] =
        ia_msu_flt(temp[AP_NUM - 2], ia_core_coder_ap_filter1[AP_NUM - 1], ptr_out[len]);
  }

  temp[0] = ia_mac_flt(ptr_state[AP_NUM], ia_core_coder_ap_filter2[0], ptr_state[2 * AP_NUM]);
  ptr_state[AP_NUM] = ia_msu_flt(ptr_state[2 * AP_NUM], ia_core_coder_ap_filter2[0], temp[0]);
  for (num = 1; num < AP_NUM - 1; num++)
  {
    temp[num] = ia_mac_flt(ptr_state[AP_NUM + num], ia_core_coder_ap_filter2[num], temp[num - 1]);
    ptr_state[AP_NUM + num] = ia_msu_flt(temp[num - 1], ia_core_coder_ap_filter2[num], temp[num]);
  }

  temp[AP_NUM - 1] = ia_mac_flt(ptr_state[2 * AP_NUM - 1], ia_core_coder_ap_filter2[AP_NUM - 1],
                                temp[AP_NUM - 2]);
  ptr_state[2 * AP_NUM - 1] =
      ia_msu_flt(temp[AP_NUM - 2], ia_core_coder_ap_filter2[AP_NUM - 1], temp[AP_NUM - 1]);
  ptr_out[0] = (FLOAT32)ia_mul_flt(ia_add_flt(ptr_out[0], temp[AP_NUM - 1]), 0.5);
  for (len = 1; len < length / 2; len++)
  {
    temp[0] = ia_mac_flt(ptr_state[AP_NUM], ia_core_coder_ap_filter2[0], ptr_in[2 * len - 1]);
    ptr_state[AP_NUM] = ia_msu_flt(ptr_in[2 * len - 1], ia_core_coder_ap_filter2[0], temp[0]);
    for (num = 1; num < AP_NUM - 1; num++)
    {
      temp[num] =
          ia_mac_flt(ptr_state[AP_NUM + num], ia_core_coder_ap_filter2[num], temp[num - 1]);
      ptr_state[AP_NUM + num] =
          ia_msu_flt(temp[num - 1], ia_core_coder_ap_filter2[num], temp[num]);
    }
    temp[AP_NUM - 1] = ia_mac_flt(ptr_state[2 * AP_NUM - 1], ia_core_coder_ap_filter2[AP_NUM - 1],
                                  temp[AP_NUM - 2]);
    ptr_state[2 * AP_NUM - 1] =
        ia_msu_flt(temp[AP_NUM - 2], ia_core_coder_ap_filter2[AP_NUM - 1], temp[AP_NUM - 1]);
    ptr_out[len] = (FLOAT32)ia_mul_flt(ia_add_flt(ptr_out[len], temp[AP_NUM - 1]), 0.5);
  }
  ptr_state[2 * AP_NUM] = ptr_in[length - 1];

  return;
}

/**
 *  ia_core_coder_tbe_gen_nonlinear_exc
 *
 *  \brief Function for Non linear excitation generation.
 *
 *  \param [in]  ptr_tbe_exc_extended Pointer to tbe excitation buffer.
 *  \param [in]  ptr_exc              Pointer to excitation buffer.
 *  \param [out] ptr_scratch          Pointer to scratch buffer.
 *  \param [in]  ptr_mem_gen_exc      Pointer to excitation memory.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_gen_nonlinear_exc(FLOAT32 *ptr_tbe_exc_extended, FLOAT32 *ptr_exc,
                                                FLOAT32 *ptr_scratch, FLOAT32 *ptr_mem_gen_exc)
{
  WORD32 len;
  FLOAT32 *exc_nl = ptr_scratch;

  for (len = 0; len < 2 * LEN_TBE_FRAME; len++)
  {
    exc_nl[len] = (len & 1) ? (ptr_tbe_exc_extended[len]) : (-ptr_tbe_exc_extended[len]);
  }

  ia_core_coder_tbe_decim_allpass_steep(exc_nl, ptr_mem_gen_exc, 2 * LEN_TBE_FRAME, ptr_exc);
  return;
}

/**
 *  ia_core_coder_tbe_auto_corr
 *
 *  \brief TBE auto-correlation function.
 *
 *  \param [in] ptr_input   Pointer to input buffer.
 *  \param [in] ptr_acf     Pointer to buffer to store autocorrelation values for different lags.
 *  \param [in] order       Max lag for computing correlation values.
 *  \param [in] length      Length of input buffer.
 *  \param [in] ptr_window  Pointer to windowing coefficients.
 *  \param [in] ptr_scratch Pointer to scratch buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_auto_corr(FLOAT32 *ptr_input, FLOAT32 *ptr_acf, WORD32 order,
                                        WORD32 length, const FLOAT32 *ptr_window,
                                        FLOAT32 *ptr_scratch)
{
  WORD32 len, lag, len_rs1;
  FLOAT32 *temp = ptr_scratch;

  len_rs1 = length >> 1;
  for (len = 0; len < len_rs1; len++)
  {
    temp[len] = ia_mul_flt(ptr_input[len], ptr_window[len]);
    temp[len + len_rs1] = ia_mul_flt(ptr_input[len + len_rs1], ptr_window[len_rs1 - 1 - len]);
  }

  for (lag = 0; lag <= order; lag++)
  {
    ptr_acf[lag] = ia_mul_flt(temp[0], temp[lag]);
    for (len = 1; len < length - lag; len++)
    {
      ptr_acf[lag] += ia_mul_flt(temp[len], temp[lag + len]);
    }
  }

  return;
}

/**
 *  ia_core_coder_tbe_levinson_durbin
 *
 *  \brief Function with Levinson Durbin implementation used by TBE.
 *
 *  \param [out] ptr_lpc Pointer to LPC coefficients buffer.
 *  \param [in]  ptr_acf Pointer to autocorrelation buffer.
 *  \param [in]  order   Auto correlation order - max lag/delay value.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_levinson_durbin(FLOAT32 *ptr_lpc, FLOAT32 *ptr_acf, WORD32 order)
{
  WORD32 order_idx, idx;
  FLOAT32 tmp[24] = {0.0f}, acc, mem, norm;

  tmp[0] = (-ptr_acf[1]) / ptr_acf[0];
  ptr_lpc[0] = 1.0f;
  ptr_lpc[1] = tmp[0];
  norm = ia_mac_flt(ptr_acf[0], ptr_acf[1], tmp[0]);

  for (order_idx = 2; order_idx <= order; order_idx++)
  {
    acc = 0.0f;
    for (idx = 0; idx < order_idx; idx++)
    {
      acc += ia_mul_flt(ptr_acf[order_idx - idx], ptr_lpc[idx]);
    }

    tmp[order_idx - 1] = (-acc) / norm;
    for (idx = 1; idx <= order_idx / 2; idx++)
    {
      mem = ia_mac_flt(ptr_lpc[idx], tmp[order_idx - 1], ptr_lpc[order_idx - idx]);
      ptr_lpc[order_idx - idx] += ia_mul_flt(tmp[order_idx - 1], ptr_lpc[idx]);
      ptr_lpc[idx] = mem;
    }

    ptr_lpc[order_idx] = tmp[order_idx - 1];
    norm += ia_mul_flt(tmp[order_idx - 1], acc);
    if (ia_lteq_flt(norm, 0.0f))
    {
      norm = 0.01f;
    }
  }

  return;
}

/**
 *  ia_core_coder_tbe_gen_nmod_exc
 *
 *  \brief Function to calculate high band excitation.
 *
 *  \param [in]     ptr_exc_whtnd     Pointer to excitation buffer.
 *  \param [in]     ptr_voice_factors Pointer to buffer representing voice factors.
 *  \param [in,out] ptr_mem_csfilt    Pointer to filter states buffer.
 *  \param [in]     ptr_tbe_seed      Pointer to TBE seed value buffer.
 *  \param [in]     ptr_lpc_sf        Pointer to LPC parameter scale factors buffer.
 *  \param [in,out] ptr_wn_ana_mem    Pointer to analysis window memory.
 *  \param [in,out] ptr_wn_syn_mem    Pointer to synthesis window memory.
 *  \param [in,out] ptr_white_exc     Pointer to white excitation data.
 *  \param [in]     ptr_scratch       Pointer to scratch buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_gen_nmod_exc(FLOAT32 *ptr_exc_whtnd, FLOAT32 *ptr_voice_factors,
                                           FLOAT32 *ptr_mem_csfilt, WORD16 *ptr_tbe_seed,
                                           FLOAT32 *ptr_lpc_sf, FLOAT32 *ptr_wn_ana_mem,
                                           FLOAT32 *ptr_wn_syn_mem, FLOAT32 *ptr_white_exc,
                                           FLOAT32 *ptr_scratch)
{
  WORD32 num, len;
  FLOAT32 var_env_shape, csfilt_num2, csfilt_den2, *wn_lpc_temp, *exc_temp, *exc_noisy_env,
      *ptr_synth_scratch;

  wn_lpc_temp = ptr_scratch;
  exc_temp = wn_lpc_temp + LPC_ORDER_TBE + 1;
  exc_noisy_env = exc_temp + LEN_TBE_FRAME;
  ptr_synth_scratch = exc_noisy_env + LEN_TBE_FRAME;
  var_env_shape = ia_core_coder_tbe_find_mean(ptr_voice_factors, 4);
  var_env_shape = ia_msu_flt(ENV_SHAPE_CONST_1, ENV_SHAPE_CONST_2, var_env_shape);
  var_env_shape = ia_min_flt(ia_max_flt(var_env_shape, ENV_SHAPE_CONST_3), ENV_SHAPE_CONST_4);
  csfilt_num2 = ia_sub_flt(1.0f, var_env_shape);
  csfilt_den2 = ia_negate_flt(var_env_shape);
  for (len = 0; len < LEN_TBE_FRAME; len++)
  {
    exc_temp[len] = (FLOAT32)(ia_fabs_flt(ptr_exc_whtnd[len]));
  }

  if (ia_eq_flt(*ptr_mem_csfilt, 0))
  {
    FLOAT32 tmp_scale = 0;
    for (len = 0; len < LEN_SUBFR_TBE / 4; len++)
    {
      tmp_scale = ia_add_flt(tmp_scale, exc_temp[len]);
    }
    *ptr_mem_csfilt = ia_mul_flt(var_env_shape, (tmp_scale / (LEN_SUBFR_TBE / 4)));
  }

  for (len = 0; len < LEN_TBE_FRAME; len++)
  {
    exc_noisy_env[len] = ia_mac_flt(*ptr_mem_csfilt, csfilt_num2, exc_temp[len]);
    *ptr_mem_csfilt = ia_mul_flt(ia_negate_flt(csfilt_den2), exc_noisy_env[len]);
  }
  ia_core_coder_tbe_gen_random_vec(ptr_white_exc, 256, ptr_tbe_seed);

  for (num = 0; num < NUM_TBE_FRAME; num++)
  {
    ia_core_coder_tbe_gamma_shaping(ptr_lpc_sf + num * (LPC_ORDER_TBE + 1), wn_lpc_temp, 0.55f);
    ia_core_coder_fir_filter(ptr_white_exc + num * LEN_SUBFR, wn_lpc_temp, ptr_wn_ana_mem,
                             LPC_ORDER_TBE, LEN_SUBFR);
    ia_core_coder_tbe_gamma_shaping(ptr_lpc_sf + num * (LPC_ORDER_TBE + 1), wn_lpc_temp, 0.7f);
    ia_core_coder_tbe_synth_plus(wn_lpc_temp, LPC_ORDER_TBE, ptr_white_exc + num * LEN_SUBFR,
                                 ptr_white_exc + num * LEN_SUBFR, ptr_wn_syn_mem,
                                 ptr_synth_scratch);
  }

  for (len = 0; len < LEN_TBE_FRAME; len++)
  {
    ptr_white_exc[len] = ia_mul_flt(ptr_white_exc[len], exc_noisy_env[len]);
  }

  return;
}

/**
 *  ia_core_coder_tbe_gen_exc
 *
 *  \brief Function to generate TBE excitation value from white excitation values.
 *
 *  \param [out] ptr_exc_whtnd     Pointer to excitation output buffer.
 *  \param [in]  ptr_white_exc     Pointer to white excitation buffer.
 *  \param [in]  ptr_voice_factors Pointer to voice factors buffer used for calculation.
 *  \param [in]  pow1              Scalar power factor used in calculation of excitation.
 *  \param [in]  pow2              Scalar power factor used in calculation of excitation.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_gen_exc(FLOAT32 *ptr_exc_whtnd, FLOAT32 *ptr_white_exc,
                                      FLOAT32 *ptr_voice_factors, FLOAT32 pow1, FLOAT32 pow2)
{
  WORD32 frame, len, idx;
  FLOAT32 temp1, temp2;

  idx = 0;
  for (frame = 0; frame < NUM_TBE_FRAME; frame++)
  {
    temp1 = (FLOAT32)ia_sqrt_flt(ptr_voice_factors[frame]);
    temp2 = (FLOAT32)ia_sqrt_flt((pow1 * ia_sub_flt(1.0f, ptr_voice_factors[frame])) / pow2);

    for (len = 0; len < LEN_SUBFR_TBE; len++)
    {
      ptr_exc_whtnd[idx] =
          ia_mac_flt(ia_mul_flt(temp1, ptr_exc_whtnd[idx]), temp2, ptr_white_exc[idx]);
      idx++;
    }
  }
  return;
}

/**
 *  ia_core_coder_tbe_lpc_synth
 *
 *  \brief Function for TBE module's LPC synthesis.
 *
 *  \param [in,out] ptr_exc_whtnd        Pointer to excitation buffer.
 *  \param [in]     ptr_lpc_sf           Pointer to LPC coefficients buffer.
 *  \param [in]     ptr_hb_target_energy Pointer to highband buffer.
 *  \param [in]     harm_ext_mode        Harmonic extension mode indicator.
 *  \param [in]     hr_config            High Resolution configuration flag.
 *  \param [in,out] ptr_state_lpc_syn    Pointer to lpc synthesis states.
 *  \param [in]     ptr_excitation       Pointer to excitation buffer.
 *  \param [in]     ptr_scratch          Pointer to scratch buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_lpc_synth(FLOAT32 *ptr_exc_whtnd, FLOAT32 *ptr_lpc_sf,
                                        FLOAT32 *ptr_hb_target_energy, WORD32 harm_ext_mode,
                                        WORD32 hr_config, FLOAT32 *ptr_state_lpc_syn,
                                        FLOAT32 *ptr_excitation, FLOAT32 *ptr_scratch)
{
  WORD32 len;
  FLOAT32 *zero_mem, *syn_ener_sf, *tmp, *ptr_synth_scratch;

  zero_mem = ptr_scratch;
  syn_ener_sf = zero_mem + LPC_ORDER_TBE;
  tmp = syn_ener_sf + 4;
  ptr_synth_scratch = tmp + 80;
  ia_core_coder_memset(zero_mem, LPC_ORDER_TBE);

  if (hr_config != 1 || harm_ext_mode == 1)
  {
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf, LPC_ORDER_TBE, ptr_exc_whtnd, ptr_excitation,
                                 ptr_state_lpc_syn, ptr_synth_scratch);
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 64, ptr_excitation + 64, ptr_state_lpc_syn,
                                 ptr_synth_scratch);
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + 2 * (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 128, ptr_excitation + 128, ptr_state_lpc_syn,
                                 ptr_synth_scratch);
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + 3 * (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 192, ptr_excitation + 192, ptr_state_lpc_syn,
                                 ptr_synth_scratch);
  }
  else
  {
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf, LPC_ORDER_TBE, ptr_exc_whtnd, tmp, zero_mem,
                                 ptr_synth_scratch);
    syn_ener_sf[0] = ia_mul_flt(0.125f, ia_core_coder_tbe_calc_energy(tmp, 64));

    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 64, tmp, zero_mem, ptr_synth_scratch);
    syn_ener_sf[1] = ia_mul_flt(0.125f, ia_core_coder_tbe_calc_energy(tmp, 64));

    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + 2 * (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 128, tmp, zero_mem, ptr_synth_scratch);
    syn_ener_sf[2] = ia_mul_flt(0.125f, ia_core_coder_tbe_calc_energy(tmp, 64));

    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + 3 * (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 192, tmp, zero_mem, ptr_synth_scratch);
    syn_ener_sf[3] = ia_mul_flt(0.125f, ia_core_coder_tbe_calc_energy(tmp, 64));

    tmp[0] = (FLOAT32)(ptr_hb_target_energy[0]) /
             (ia_add_flt(ia_add_flt(syn_ener_sf[0], syn_ener_sf[1]),
                         ia_add_flt(syn_ener_sf[2], syn_ener_sf[3])));
    for (len = 0; len < LEN_TBE_FRAME; len++)
    {
      ptr_exc_whtnd[len] = ia_mul_flt(ptr_exc_whtnd[len], (FLOAT32)ia_sqrt_flt(tmp[0]));
    }

    ia_core_coder_tbe_synth_plus(ptr_lpc_sf, LPC_ORDER_TBE, ptr_exc_whtnd, ptr_excitation,
                                 ptr_state_lpc_syn, ptr_synth_scratch);
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 64, ptr_excitation + 64, ptr_state_lpc_syn,
                                 ptr_synth_scratch);
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + 2 * (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 128, ptr_excitation + 128, ptr_state_lpc_syn,
                                 ptr_synth_scratch);
    ia_core_coder_tbe_synth_plus(ptr_lpc_sf + 3 * (LPC_ORDER_TBE + 1), LPC_ORDER_TBE,
                                 ptr_exc_whtnd + 192, ptr_excitation + 192, ptr_state_lpc_syn,
                                 ptr_synth_scratch);
  }

  return;
}

/**
 *  ia_core_coder_tbe_dequantize_params
 *
 *  \brief Function to de-quantize TBE params.
 *
 *  \param [in]  bit_stream             Pointer to bitstream structure.
 *  \param [out] ptr_q_lsf              Pointer to LSF buffer.
 *  \param [out] ptr_q_subgain          Pointer to sub frame gain buffer.
 *  \param [out] ptr_q_frame_gain       Pointer to frame gain.
 *  \param [out] ptr_q_hb_energy_target Pointer to high band energy target.
 *  \param [out] ptr_q_res_subgain      Pointer to residual subband gain
 *  \param [out] ptr_q_mix_factors      Pointer to mix factors.
 *  \param [out] ptr_q_exc_resp         Pointer to excitation response.
 *
 *
 *
 */
static VOID
ia_core_coder_tbe_dequantize_params(ia_usac_tbe_bitstream_handle bit_stream, FLOAT32 *ptr_q_lsf,
                                    FLOAT32 *ptr_q_subgain, FLOAT32 *ptr_q_frame_gain,
                                    FLOAT32 *ptr_q_hb_energy_target, FLOAT32 *ptr_q_res_subgain,
                                    FLOAT32 *ptr_q_mix_factors, FLOAT32 *ptr_q_exc_resp)
{
  WORD32 order_idx, gain_idx;
  FLOAT32 lsf_diff[LPC_ORDER_TBE], filt_pow;

  if (bit_stream->harm_ext_mode != 0)
  {
    *ptr_q_hb_energy_target = 0;
    *ptr_q_mix_factors = 0;
    ia_core_coder_memset(ptr_q_res_subgain, NUM_TBE_FRAME);
  }
  else
  {
    *ptr_q_mix_factors = ia_mac_flt(0.25f, (FLOAT32)bit_stream->idx_mix_config, 0.25f);
    if (bit_stream->hr_config != 1)
    {
      ptr_q_exc_resp[0] = 1.0f;
      ia_core_coder_mem_cpy(ia_core_coder_tbe_exc_filter_cb1_7b +
                                10 * bit_stream->idx_shb_exc_resp[0],
                            ptr_q_exc_resp, 10);
      ia_core_coder_mem_cpy(ia_core_coder_tbe_exc_filter_cb2_4b +
                                6 * bit_stream->idx_shb_exc_resp[1],
                            ptr_q_exc_resp + 10, 6);

      filt_pow =
          (FLOAT32)ia_sqrt_flt(ia_core_coder_tbe_calc_energy(ptr_q_exc_resp, RESP_ORDER + 1));
      if (filt_pow < 1e-6)
      {
        filt_pow = 1.0f;
      }
      for (order_idx = 0; order_idx < RESP_ORDER + 1; order_idx++)
      {
        ptr_q_exc_resp[order_idx] = ptr_q_exc_resp[order_idx] / filt_pow;
      }
    }
    else
    {
      *ptr_q_hb_energy_target = ia_mul_flt((FLOAT32)bit_stream->idx_shb_fr_gain, 0.0625f);
      *ptr_q_hb_energy_target = (FLOAT32)pow(10.0, *ptr_q_hb_energy_target);
      ia_core_coder_mem_cpy(ia_core_coder_sub_gain_5bit +
                                bit_stream->idx_res_sub_gains * NUM_TBE_FRAME,
                            ptr_q_res_subgain, NUM_TBE_FRAME);
    }
  }

  ia_core_coder_mem_cpy(ia_core_coder_tbe_lsf_cb1_7b + bit_stream->lsf_idx[0] * LPC_ORDER_TBE,
                        ptr_q_lsf, LPC_ORDER_TBE);
  ia_core_coder_mem_cpy(ia_core_coder_tbe_lsf_cb2_7b + bit_stream->lsf_idx[1] * LPC_ORDER_TBE,
                        lsf_diff, LPC_ORDER_TBE);

  for (order_idx = 0; order_idx < LPC_ORDER_TBE; order_idx++)
  {
    ptr_q_lsf[order_idx] = ia_add_flt(ptr_q_lsf[order_idx], lsf_diff[order_idx]);
  }
  ia_core_coder_tbe_shift_lsf(ptr_q_lsf);

  gain_idx = bit_stream->sub_gain_idx * NUM_SUBGAINS;
  ia_core_coder_mem_cpy(&ia_core_coder_sub_gain_5bit[gain_idx], ptr_q_subgain, NUM_SUBGAINS);

  for (order_idx = NUM_SUBFR - 1; order_idx >= 0; order_idx--)
  {
    ptr_q_subgain[order_idx] = ptr_q_subgain[order_idx * NUM_SUBGAINS / NUM_SUBFR];
  }
  *ptr_q_frame_gain = ia_core_coder_gain_frame_5bit[bit_stream->frame_gain_idx];

  return;
}

/**
 *  ia_core_coder_tbe_scale_shaping
 *
 *  \brief Function to do scaling of TBE processing data.
 *
 *  \param [in]  length      Length of the input data.
 *  \param [out] ptr_output  Pointer to output buffer.
 *  \param [in]  ptr_overlap Pointer to overlap buffer.
 *  \param [in]  ptr_subgain Pointer to subframe gain.
 *  \param [in]  frame_gain  Frame gain value.
 *  \param [in]  ptr_subwin  Pointer to subwindown coefficients.
 *  \param [in]  ptr_scratch Pointer to scratch buffer.
 *
 *
 *
 */
static VOID ia_core_coder_tbe_scale_shaping(WORD32 length, FLOAT32 *ptr_output,
                                            FLOAT32 *ptr_overlap, FLOAT32 *ptr_subgain,
                                            FLOAT32 frame_gain, const FLOAT32 *ptr_subwin,
                                            FLOAT32 *ptr_scratch)
{
  WORD32 len, num, idx, l_lahead, l_frame, join_length, num_join;
  FLOAT32 *mod_syn = ptr_scratch;

  l_frame = LEN_TBE_FRAME;
  l_lahead = TBE_LAHEAD;
  num_join = NUM_SUBFR / NUM_SUBGAINS;
  join_length = num_join * length;

  idx = 0;
  for (len = 0; len < length; len++)
  {
    mod_syn[idx] = ia_mul_flt(ia_mul_flt(ptr_output[idx], ptr_subwin[len + 1]), ptr_subgain[0]);
    idx++;
  }
  for (num = 0; num < NUM_SUBGAINS - 1; num++)
  {
    for (len = 0; len < join_length - length; len++)
    {
      mod_syn[idx] = ia_mul_flt(ptr_output[idx], ptr_subgain[num * num_join]);
      idx++;
    }
    for (len = 0; len < length; len++)
    {
      mod_syn[idx] = ia_mul_flt(
          ptr_output[idx],
          ia_mac_flt(ia_mul_flt(ptr_subwin[length - len - 1], ptr_subgain[num * num_join]),
                     ptr_subwin[len + 1], (ptr_subgain[(num + 1) * num_join])));
      idx++;
    }
  }
  for (len = 0; len < join_length - length; len++)
  {
    mod_syn[idx] = ia_mul_flt(ptr_output[idx], ptr_subgain[(NUM_SUBGAINS - 1) * num_join]);
    idx++;
  }
  for (len = 0; len < length; len++)
  {
    mod_syn[idx] =
        ia_mul_flt(ptr_output[idx], ia_mul_flt(ptr_subwin[length - len - 1],
                                               ptr_subgain[(NUM_SUBGAINS - 1) * num_join]));
    idx++;
  }
  for (len = 0; len < l_lahead; len++)
  {
    ptr_output[len] = ia_mul_flt(mod_syn[len], frame_gain);
    ptr_output[len] = ia_add_flt(ptr_output[len], ptr_overlap[len]);
  }
  for (; len < l_frame; len++)
  {
    ptr_output[len] = ia_mul_flt(mod_syn[len], frame_gain);
  }
  for (; len < l_frame + l_lahead; len++)
  {
    ptr_overlap[len - l_frame] = ia_mul_flt(mod_syn[len], frame_gain);
  }

  return;
}

/**
 *  ia_core_coder_reset_tbe_state
 *
 *  \brief Function to reset TBE states.
 *
 *  \param [in,out] tbe_dec_data TBE state structure handle
 *
 *
 *
 */
VOID ia_core_coder_reset_tbe_state(ia_usac_tbe_data_handle tbe_dec_data)
{
  ia_core_coder_memset(tbe_dec_data->tbe_state.gen_synth_state_lsyn_filt_local, (2 * AP_NUM + 1));
  ia_core_coder_memset(tbe_dec_data->tbe_state.mem_csfilt, 2);
  ia_core_coder_memset(tbe_dec_data->tbe_state.mem_gen_exc_filt_down, (2 * AP_NUM + 1));
  ia_core_coder_memset(tbe_dec_data->tbe_state.mem_resp_exc_whtnd, RESP_ORDER);
  ia_core_coder_memset(tbe_dec_data->tbe_state.mem_whtn_filt, LPC_WHTN_ORDER);
  ia_core_coder_memset(tbe_dec_data->tbe_state.old_tbe_exc, TBE_PIT_MAX * 2);
  ia_core_coder_memset(tbe_dec_data->tbe_state.old_tbe_exc_extended, NL_BUFF_OFFSET);
  ia_core_coder_memset(tbe_dec_data->tbe_state.state_lpc_syn, LPC_ORDER_TBE);
  ia_core_coder_memset(tbe_dec_data->tbe_state.state_syn_exc, TBE_LAHEAD);
  ia_core_coder_memset(tbe_dec_data->tbe_state.syn_overlap, TBE_LAHEAD);
  ia_core_coder_memset(tbe_dec_data->tbe_state.wn_ana_mem, LPC_ORDER_TBE);
  ia_core_coder_memset(tbe_dec_data->tbe_state.wn_syn_mem, LPC_ORDER_TBE);

  return;
}

/**
 *  ia_core_coder_tbe_apply
 *
 *  \brief Function to apply time domain bandwidth extension.
 *
 *  \param [in,out] tbe_dec_data         Pointer to TBE decoder data structure.
 *  \param [in]     frame_class          Frame class type.
 *  \param [in]     ptr_tbe_exc_extended Pointer to extended TBE excitation buffer.
 *  \param [in,out] ptr_voice_factors    Pointer to voice factors buffer.
 *  \param [in]     ptr_synth            Pointer to synthesis buffer.
 *  \param [in]     first_frame          First frame flag.
 *  \param [in]     ptr_scratch          Pointer to scratch buffer.
 *
 *
 *
 */
VOID ia_core_coder_tbe_apply(ia_usac_tbe_data_handle tbe_dec_data, WORD32 frame_class,
                             FLOAT32 *ptr_tbe_exc_extended, FLOAT32 *ptr_voice_factors,
                             FLOAT32 *ptr_synth, WORD32 first_frame, FLOAT32 *ptr_scratch)
{
  WORD32 len, order_idx, idx;
  FLOAT32 frame_gain = 0.0f;
  FLOAT32 prev_pow, curr_pow, scale;
  const FLOAT32 *ptr_lsp_interp_coef;
  FLOAT32 hb_energy_target = 0.0f;
  FLOAT32 mix_factors = 0.0f;
  FLOAT32 temp;
  FLOAT32 pow1, pow2;
  FLOAT32 *tbe_exc_buf = &(tbe_dec_data->exc[(TBE_PIT_MAX << 1)]);
  WORD32 tmp;
  WORD32 subfr_idx = (1 == frame_class) ? 1 : 0;
  ia_usac_tbe_bitstream_handle bit_stream = &tbe_dec_data->bit_stream[subfr_idx];

  FLOAT32 *ptr_lsf = ptr_scratch;
  FLOAT32 *sub_gain = ptr_lsf + LPC_ORDER_TBE;
  FLOAT32 *vf_modified = sub_gain + NUM_SUBFR;
  FLOAT32 *lsp_1 = vf_modified + NUM_TBE_FRAME;
  FLOAT32 *lsp_2 = lsp_1 + LPC_ORDER_TBE;
  FLOAT32 *lsp_temp = lsp_2 + LPC_ORDER_TBE;
  FLOAT32 *ptr_lpc_sf = lsp_temp + LPC_ORDER_TBE;

  FLOAT32 *res_sub_gain = ptr_lpc_sf + 4 * (LPC_ORDER_TBE + 1);
  FLOAT32 *exc_resp = res_sub_gain + NUM_TBE_FRAME;
  FLOAT32 *rx = exc_resp + RESP_ORDER + 1;
  FLOAT32 *lpc_whtn = rx + LPC_WHTN_ORDER + 1;

  FLOAT32 *scratch_acf = lpc_whtn + LPC_WHTN_ORDER + 1;
  FLOAT32 *shaped_excitation = scratch_acf + LEN_SUPERFRAME;
  FLOAT32 *ptr_exc_whtnd = shaped_excitation + LEN_TBE_FRAME + TBE_LAHEAD;
  FLOAT32 *ptr_white_exc = ptr_exc_whtnd + LEN_TBE_FRAME;
  FLOAT32 *ptr_tbe_scr_curr = ptr_white_exc + LEN_TBE_FRAME;

  ia_core_coder_mem_cpy(&tbe_dec_data->exc[2 * LEN_FRAME], tbe_dec_data->tbe_state.old_tbe_exc,
                        2 * TBE_PIT_MAX);
  ia_core_coder_tbe_nonlinearity(bit_stream, tbe_exc_buf, ptr_tbe_exc_extended,
                                 tbe_dec_data->tbe_state.old_tbe_exc_extended, ptr_tbe_scr_curr,
                                 2 * LEN_FRAME);
  ia_core_coder_tbe_dequantize_params(bit_stream, ptr_lsf, sub_gain, &frame_gain,
                                      &hb_energy_target, res_sub_gain, &mix_factors, exc_resp);
  ia_core_coder_mem_cpy(ptr_voice_factors, vf_modified, NUM_TBE_FRAME);
  if (ia_core_coder_tbe_find_mean(ptr_voice_factors, 4) > 0.4f)
  {
    for (len = 1; len < NUM_TBE_FRAME; len++)
    {
      vf_modified[len] =
          ia_mac_flt(ia_mul_flt(0.8f, ptr_voice_factors[len]), 0.2f, ptr_voice_factors[len - 1]);
    }
  }

  ia_core_coder_tbe_lsp_lsf_conv(ptr_lsf, lsp_2, 1);
  if (!first_frame)
  {
    ia_core_coder_mem_cpy(tbe_dec_data->lsp_prev_interp, lsp_1, LPC_ORDER_TBE);
  }
  else
  {
    ia_core_coder_mem_cpy(lsp_2, lsp_1, LPC_ORDER_TBE);
  }

  ptr_lsp_interp_coef = ia_core_coder_tbe_interpol_frac;
  for (idx = 0; idx < 4; idx++)
  {
    for (order_idx = 0; order_idx < LPC_ORDER_TBE; order_idx++)
    {
      lsp_temp[order_idx] = ia_mac_flt(ia_mul_flt(lsp_1[order_idx], (*ptr_lsp_interp_coef)),
                                       lsp_2[order_idx], (*(ptr_lsp_interp_coef + 1)));
    }
    ptr_lsp_interp_coef += 2;
    ia_core_coder_tbe_lsp_lsf_conv(lsp_temp, lsp_temp, 0);
    ia_core_coder_tbe_lsf2lpc_conv(ptr_lpc_sf + idx * (LPC_ORDER_TBE + 1), lsp_temp);
    ptr_lpc_sf[idx * (LPC_ORDER_TBE + 1)] = 1.0f;
  }

  ia_core_coder_mem_cpy(lsp_2, tbe_dec_data->lsp_prev_interp, LPC_ORDER_TBE);
  ia_core_coder_mem_cpy(tbe_dec_data->tbe_state.state_syn_exc, shaped_excitation, TBE_LAHEAD);
  ia_core_coder_tbe_gen_nonlinear_exc(ptr_tbe_exc_extended, ptr_exc_whtnd, ptr_tbe_scr_curr,
                                      tbe_dec_data->tbe_state.mem_gen_exc_filt_down);
  if (bit_stream->harm_ext_mode == 0 && bit_stream->hr_config == 0)
  {
    ia_core_coder_fir_filter(ptr_exc_whtnd, exc_resp, tbe_dec_data->tbe_state.mem_resp_exc_whtnd,
                             RESP_ORDER, LEN_TBE_FRAME);
  }
  else
  {
    ia_core_coder_mem_cpy(ptr_exc_whtnd + LEN_TBE_FRAME - RESP_ORDER,
                          tbe_dec_data->tbe_state.mem_resp_exc_whtnd, RESP_ORDER);
  }

  ia_core_coder_tbe_auto_corr(ptr_exc_whtnd, rx, LPC_WHTN_ORDER, LEN_TBE_FRAME,
                              ia_core_coder_tbe_win_flatten, scratch_acf);
  rx[0] = ia_max_flt(rx[0], 1.0e-8f);
  for (order_idx = 0; order_idx <= LPC_WHTN_ORDER; order_idx++)
  {
    rx[order_idx] = ia_mul_flt(rx[order_idx], ia_core_coder_wac[order_idx]);
  }
  rx[0] = ia_add_flt(rx[0], 1.0e-8f);
  ia_core_coder_tbe_levinson_durbin(lpc_whtn, rx, LPC_WHTN_ORDER);
  ia_core_coder_fir_filter(ptr_exc_whtnd, lpc_whtn, tbe_dec_data->tbe_state.mem_whtn_filt,
                           LPC_WHTN_ORDER, LEN_TBE_FRAME);
  if (bit_stream->harm_ext_mode == 0 && bit_stream->hr_config == 1)
  {
    for (len = 0; len < LEN_TBE_FRAME; len++)
    {
      tmp = (len >> 6) & 0xff;
      ptr_exc_whtnd[len] = ia_mul_flt(ptr_exc_whtnd[len], res_sub_gain[tmp]);
    }
  }

  ia_core_coder_tbe_gen_nmod_exc(
      ptr_exc_whtnd, vf_modified, tbe_dec_data->tbe_state.mem_csfilt, tbe_dec_data->tbe_seed,
      ptr_lpc_sf, tbe_dec_data->tbe_state.wn_ana_mem, tbe_dec_data->tbe_state.wn_syn_mem,
      ptr_white_exc, ptr_tbe_scr_curr);

  pow1 = pow2 = 0.00001f;
  for (len = 0; len < LEN_TBE_FRAME; len++)
  {
    pow1 += ia_mul_flt(ptr_exc_whtnd[len], ptr_exc_whtnd[len]);
    pow2 += ia_mul_flt(ptr_white_exc[len], ptr_white_exc[len]);
  }
  if (bit_stream->harm_ext_mode == 0)
  {
    for (len = 0; len < NUM_TBE_FRAME; len++)
    {
      vf_modified[len] = ia_mul_flt(vf_modified[len], mix_factors);
    }
  }

  ia_core_coder_tbe_gen_exc(ptr_exc_whtnd, ptr_white_exc, vf_modified, pow1, pow2);
  ia_core_coder_tbe_lpc_synth(ptr_exc_whtnd, ptr_lpc_sf, &hb_energy_target,
                              bit_stream->harm_ext_mode, bit_stream->hr_config,
                              tbe_dec_data->tbe_state.state_lpc_syn,
                              shaped_excitation + TBE_LAHEAD, ptr_tbe_scr_curr);

  prev_pow = ia_core_coder_tbe_calc_energy(shaped_excitation, TBE_LAHEAD + 10);
  curr_pow = ia_core_coder_tbe_calc_energy(shaped_excitation + TBE_LAHEAD + 10, TBE_LAHEAD + 10);
  if (ia_lt_flt(0.75f, ptr_voice_factors[0]))
  {
    curr_pow = (FLOAT32)ia_mul_flt(curr_pow, (FLOAT32)0.25);
  }

  scale = (prev_pow == 0) ? 0 : (FLOAT32)ia_sqrt_flt(curr_pow / prev_pow);
  for (len = 0; len < TBE_LAHEAD; len++)
  {
    shaped_excitation[len] = ia_mul_flt(shaped_excitation[len], scale);
  }
  for (; len < TBE_LAHEAD + 10; len++)
  {
    temp = (len - (TBE_LAHEAD - 1)) / 10.0f;
    shaped_excitation[len] =
        ia_mul_flt(shaped_excitation[len],
                   ia_mac_flt(ia_mul_flt(temp, 1.0f), ia_sub_flt(1.0f, temp), scale));
  }

  ia_core_coder_mem_cpy(shaped_excitation + LEN_TBE_FRAME, tbe_dec_data->tbe_state.state_syn_exc,
                        TBE_LAHEAD);
  ia_core_coder_tbe_scale_shaping(TBE_OVERLAP_LEN, shaped_excitation,
                                  tbe_dec_data->tbe_state.syn_overlap, sub_gain, frame_gain,
                                  ia_core_coder_sub_win_tbe, ptr_tbe_scr_curr);
  ia_core_coder_tbe_interpol(shaped_excitation,
                             tbe_dec_data->tbe_state.gen_synth_state_lsyn_filt_local,
                             LEN_TBE_FRAME, ptr_tbe_scr_curr);
  for (len = 0; len < 2 * LEN_TBE_FRAME; len++)
  {
    ptr_synth[len] =
        ((len % 2) == 0) ? (ia_negate_flt(ptr_tbe_scr_curr[len])) : (ptr_tbe_scr_curr[len]);
  }
  ia_core_coder_mem_cpy(ptr_synth, tbe_dec_data->synth_prev, 2 * LEN_TBE_FRAME);

  return;
}

/**
 *  ia_core_coder_tbe_gen_transition
 *
 *  \brief TBE transition generator function.
 *
 *  \param [in]  tbe_dec_data Pointer to TBE decoder data structure.
 *  \param [in]  length       Processing data length.
 *  \param [out] ptr_output   Pointer to output buffer.
 *  \param [in]  ptr_scratch  Pointer to scratch buffer.
 *
 *
 *
 */
VOID ia_core_coder_tbe_gen_transition(ia_usac_tbe_data_handle tbe_dec_data, WORD32 length,
                                      FLOAT32 *ptr_output, FLOAT32 *ptr_scratch)
{
  WORD32 len;
  FLOAT32 *syn_overlap, *ptr_input, *old_hb_synth, *state_lsyn_filt_local;

  syn_overlap = ptr_scratch;
  ptr_input = tbe_dec_data->tbe_state.syn_overlap;
  old_hb_synth = tbe_dec_data->synth_prev;
  state_lsyn_filt_local = tbe_dec_data->tbe_state.gen_synth_state_lsyn_filt_local;
  ia_core_coder_tbe_interpol(ptr_input, state_lsyn_filt_local, TBE_OVERLAP_LEN, syn_overlap);

  for (len = 0; len < 2 * TBE_OVERLAP_LEN; len++)
  {
    syn_overlap[len] = ((len % 2) == 0) ? (ia_negate_flt(syn_overlap[len])) : (syn_overlap[len]);
    ptr_output[len] = ia_mac_flt(
        ia_mul_flt(ia_core_coder_subwin_tbe_fb[len], old_hb_synth[2 * LEN_TBE_FRAME - 1 - len]),
        ia_core_coder_subwin_tbe_fb[2 * TBE_OVERLAP_LEN - len], syn_overlap[len]);
  }
  for (; len < length; len++)
  {
    ptr_output[len] = old_hb_synth[2 * LEN_TBE_FRAME - 1 - len];
  }

  return;
}

/**
 *  ia_core_coder_tbe_init
 *
 *  \brief TBE initialization function.
 *
 *  \param [in,out] tbe_dec_data Pointer to TBE decoder data structure.
 *
 *
 *
 */
VOID ia_core_coder_tbe_init(ia_usac_tbe_data_handle tbe_dec_data)
{
  WORD32 idx;

  tbe_dec_data->tbe_seed[0] = 23;
  tbe_dec_data->tbe_seed[1] = 59;

  ia_core_coder_mem_cpy(ia_core_coder_lsp_prev_interp, tbe_dec_data->lsp_prev_interp,
                        LPC_ORDER_TBE);
  ia_core_coder_memset(tbe_dec_data->synth_prev, 2 * LEN_FRAME);
  ia_core_coder_memset(tbe_dec_data->synth_curr, 2 * LEN_FRAME);
  ia_core_coder_reset_tbe_state(tbe_dec_data);

  for (idx = 0; idx <= 1; idx++)
  {
    memset(&tbe_dec_data->bit_stream[idx], 0, sizeof(tbe_dec_data->bit_stream[idx]));
    tbe_dec_data->bit_stream[idx].hr_config = 1;
    tbe_dec_data->bit_stream[idx].nl_config = 1;
  }

  return;
}

/**
 *  ia_core_coder_mix_past_exc
 *
 *  \brief Function to mix past TBE excitation values.
 *
 *  \param [in,out] tbe_dec_data   TBE decoder data structure.
 *  \param [out]    ptr_error      Pointer to error signal.
 *  \param [in]     subfr_idx      Sub-frame index value
 *  \param [in]     pitch_lag      Integer part of pitch lag value.
 *  \param [in]     pitch_lag_frac Fractional part of pitch lag value
 *
 *
 *
 */
VOID ia_core_coder_mix_past_exc(ia_usac_tbe_data_handle tbe_dec_data, FLOAT32 *ptr_error,
                                WORD32 subfr_idx, WORD32 pitch_lag, WORD32 pitch_lag_frac)
{
  WORD32 offset, idx1;
  FLOAT32 *tbe_exc;

  offset = pitch_lag * 2 +
           (WORD32)(ia_add_flt(ia_mul_flt((FLOAT32)pitch_lag_frac, 0.5f), 0.5f) + 4) - 4;
  tbe_exc = &(tbe_dec_data->exc[(subfr_idx << 1) + (TBE_PIT_MAX << 1)]);
  if (subfr_idx == 0)
  {
    ia_core_coder_mem_cpy(tbe_dec_data->tbe_state.old_tbe_exc, tbe_dec_data->exc,
                          2 * TBE_PIT_MAX);
  }

  for (idx1 = 0; idx1 < LEN_SUBFR * 2; idx1++)
  {
    tbe_exc[idx1] = tbe_exc[idx1 - offset + (WORD32)*ptr_error];
  }
  *ptr_error =
      ia_msu_flt(ia_msu_flt(ia_add_flt((*ptr_error), (FLOAT32)offset), (FLOAT32)pitch_lag, 2),
                 0.5f, (FLOAT32)pitch_lag_frac);

  return;
}

/**
 *  ia_core_coder_tbe_prep_exc
 *
 *  \brief Function to compute TBE Excitation data.
 *
 *  \param [in] tbe_dec_data      Pointer to TBE decoder data structure.
 *  \param [in] i_subfr           Index of the subframe.
 *  \param [in] gain_pit          Gain value.
 *  \param [in] gain_code         Gain code
 *  \param [in] ptr_code          Pointer to code buffer.
 *  \param [in] voice_fac         Voice factor value
 *  \param [in] ptr_voice_factors Pointer to voice factors buffer.
 *
 *
 *
 */
VOID ia_core_coder_tbe_prep_exc(ia_usac_tbe_data_handle tbe_dec_data, WORD32 i_subfr,
                                FLOAT32 gain_pit, FLOAT32 gain_code, FLOAT32 *ptr_code,
                                FLOAT32 voice_fac, FLOAT32 *ptr_voice_factors)
{
  WORD32 len;
  FLOAT32 post_interpolate[LEN_SUBFR * 2] = {0.0f}, pre_interpolate[LEN_SUBFR] = {0.0f},
                                       voice_fac_proc, *tbe_exc;

  voice_fac_proc = ia_sub_flt(ia_mul_flt(0.95f, voice_fac), 0.05f);
  tbe_exc = &(tbe_dec_data->exc[(TBE_PIT_MAX << 1)]);
  *ptr_voice_factors = 1.0f / ia_add_flt(1.0f, (FLOAT32)exp(ia_mul_flt(-4.0f, voice_fac_proc)));
  *ptr_voice_factors = ia_min_flt(ia_max_flt(0.0f, *ptr_voice_factors), 1.0f);

  for (len = 0; len < LEN_SUBFR; len++)
  {
    pre_interpolate[len] = ia_mul_flt(gain_code, ptr_code[len]);
  }
  for (len = 0; len < 2 * LEN_SUBFR; len += 2)
  {
    post_interpolate[len] = pre_interpolate[len / 2];
  }
  for (len = 1; len < 2 * LEN_SUBFR - 1; len += 2)
  {
    post_interpolate[len] =
        ia_mul_flt(0.5f, ia_add_flt(post_interpolate[len - 1], post_interpolate[len + 1]));
  }
  post_interpolate[len] = ia_mul_flt(0.5f, pre_interpolate[len / 2]);

  for (len = 0; len < LEN_SUBFR * 2; len++)
  {
    tbe_exc[len + i_subfr * 2] =
        ia_mac_flt(post_interpolate[len], gain_pit, tbe_exc[len + i_subfr * 2]);
  }

  return;
}

/**
 *  ia_core_coder_td_resampler
 *
 *  \brief Function for sample rate converter used by fullband LPD path.
 *
 *  \param [in]     ptr_input      Pointer to input buffer.
 *  \param [in]     num_samples_in Number of input samples.
 *  \param [in]     fs_in          Input signal's sampling frequency.
 *  \param [out]    ptr_output     Pointer to output buffer
 *  \param [in]     fs_out         Output signal's sampling frequency.
 *  \param [in,out] ptr_filter_mem Pointer to resampling filter states.
 *  \param [in]     interp_only    Flag that indicates interpolation processing.
 *  \param [in]     ptr_scratch    Pointer to scratch buffer
 *
 *
 *
 */
VOID ia_core_coder_td_resampler(FLOAT32 *ptr_input, WORD16 num_samples_in, WORD32 fs_in,
                                FLOAT32 *ptr_output, WORD32 fs_out, FLOAT32 *ptr_filter_mem,
                                WORD32 interp_only, FLOAT32 *ptr_scratch)
{
  WORD16 len, samp, step_size, step_idx = 0, num_samples_out;
  FLOAT32 *signal = ptr_scratch;

  if (fs_in >= fs_out)
  {
    step_size = (1 == interp_only) ? 1 : 2;
    num_samples_out = (1 == interp_only) ? num_samples_in : num_samples_in / 2;
    if (ptr_filter_mem)
    {
      ia_core_coder_mem_cpy(ptr_filter_mem, signal, DOWNSAMP_FILT_MEM_SIZE);
    }
    else if (DOWNSAMP_FILT_MEM_SIZE / 2 < num_samples_in)
    {
      for (len = 0; len < DOWNSAMP_FILT_MEM_SIZE / 2; len++)
      {
        signal[DOWNSAMP_FILT_MEM_SIZE / 2 - 1 - len] =
            ia_sub_flt((2 * ptr_input[0]), ptr_input[len + 1]);
        signal[num_samples_in + DOWNSAMP_FILT_MEM_SIZE / 2 + len] =
            ia_sub_flt((2 * ptr_input[num_samples_in - 1]), ptr_input[num_samples_in - 2 - len]);
      }
    }

    if (ptr_filter_mem)
    {
      ia_core_coder_mem_cpy(ptr_input, signal + DOWNSAMP_FILT_MEM_SIZE, num_samples_in);
    }
    else
    {
      ia_core_coder_mem_cpy(ptr_input, signal + DOWNSAMP_FILT_MEM_SIZE / 2, num_samples_in);
    }

    for (samp = 0; samp < num_samples_out; samp++)
    {
      ptr_output[samp] = 0.f;
      for (len = 0; len < DOWNSAMP_FILT_LEN; len++)
      {
        ptr_output[samp] =
            ia_mac_flt(ia_mac_flt(ptr_output[samp], signal[step_idx + DOWNSAMP_FILT_LEN - len],
                                  ia_core_coder_tdr_filter_interp[len]),
                       signal[step_idx + 1 + DOWNSAMP_FILT_LEN + len],
                       ia_core_coder_tdr_filter_interp[len + 1]);
      }
      ptr_output[samp] = ia_mul_flt(ptr_output[samp], 0.5f);
      step_idx += step_size;
    }

    if (0 == interp_only && ptr_filter_mem)
    {
      ia_core_coder_mem_cpy(signal + num_samples_in, ptr_filter_mem, DOWNSAMP_FILT_MEM_SIZE);
    }
  }
  else
  {
    if (ptr_filter_mem)
    {
      ia_core_coder_mem_cpy(ptr_filter_mem, signal, UPSAMP_FILT_MEM_SIZE);
    }
    else if (UPSAMP_FILT_MEM_SIZE / 2 < num_samples_in)
    {
      for (len = 0; len < UPSAMP_FILT_MEM_SIZE / 2; len++)
      {
        signal[UPSAMP_FILT_MEM_SIZE / 2 - 1 - len] =
            ia_sub_flt((2 * ptr_input[0]), ptr_input[len + 1]);
        signal[num_samples_in + UPSAMP_FILT_MEM_SIZE / 2 + len] =
            ia_sub_flt((2 * ptr_input[num_samples_in - 1]), ptr_input[num_samples_in - 2 - len]);
      }
    }

    if (ptr_filter_mem)
    {
      ia_core_coder_mem_cpy(ptr_input, signal + UPSAMP_FILT_MEM_SIZE, num_samples_in);
    }
    else
    {
      ia_core_coder_mem_cpy(ptr_input, signal + UPSAMP_FILT_MEM_SIZE / 2, num_samples_in);
    }

    for (samp = 0; samp < num_samples_in; samp++)
    {
      ptr_output[(samp * 2)] = signal[samp + UPSAMP_FILT_LEN];
      ptr_output[(samp * 2) + 1] = 0.f;
      for (len = 0; len < UPSAMP_FILT_LEN; len++)
      {
        ptr_output[(samp * 2) + 1] = ia_mac_flt(
            ia_mac_flt(ptr_output[(samp * 2) + 1], signal[samp + UPSAMP_FILT_LEN - len],
                       ia_core_coder_tdr_filter_interp[(len * 2) + 1]),
            signal[samp + 1 + UPSAMP_FILT_LEN + len],
            ia_core_coder_tdr_filter_interp[(len * 2) + 1]);
      }
    }

    if (ptr_filter_mem)
    {
      ia_core_coder_mem_cpy(signal + num_samples_in, ptr_filter_mem, UPSAMP_FILT_MEM_SIZE);
    }
  }

  return;
}
/** @} */ /* End of CoreDecProc */
