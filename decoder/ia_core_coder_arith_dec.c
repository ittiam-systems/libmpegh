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
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_info.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_igf_dec.h"
#include <ia_core_coder_basic_ops32.h>

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_arith_map_context
 *
 *  \brief Helper function with arithmetic decoding context mapping.
 *
 *  \param [in]  pres_n           Index of the present code word.
 *  \param [in]  prev_n           Index of the previous code word.
 *  \param [i/o] c_prev           Pointer to previous code word buffer.
 *  \param [i/o] c                Pointer to code word buffer.
 *  \param [in]  arith_reset_flag State reset flag.
 *  \param [in]  ptr_scratch_32   Pointer to scratch buffer.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_arith_map_context(WORD32 pres_n, WORD32 prev_n, WORD8 *c_prev, WORD8 *c,
                                            WORD32 arith_reset_flag, WORD32 *ptr_scratch_32)
{
  WORD32 i, k;
  FLOAT32 ratio;
  ia_arith_dec_scratch *pstr_scratch = (ia_arith_dec_scratch *)ptr_scratch_32;
  WORD32 *p_c_prev_tmp = pstr_scratch->scratch_prev_tmp;
  WORD32 *p_c_tmp = pstr_scratch->scratch_tmp;
  if (!(arith_reset_flag))
  {
    for (i = 2; i < (prev_n / 2 + 4); i++)
    {
      p_c_tmp[i] = (WORD32)c[i];
      p_c_prev_tmp[i] = (WORD32)c_prev[i];
    }

    ratio = (FLOAT32)(prev_n) / (FLOAT32)(pres_n);

    for (i = 0; i < (pres_n / 2); i++)
    {
      k = (WORD32)ia_mul_flt((FLOAT32)(i), ratio);
      c[2 + i] = (WORD8)p_c_tmp[2 + k];
      c_prev[2 + i] = (WORD8)p_c_prev_tmp[2 + k];
    }
    c[(pres_n / 2) + 3] = (WORD8)p_c_tmp[(prev_n / 2) + 3];
    c_prev[(pres_n / 2) + 2] = (WORD8)p_c_prev_tmp[(prev_n / 2) + 2];
  }
  else
  {
    memset(c, 0, 516 * sizeof(WORD8));
    memset(c_prev, 0, 516 * sizeof(WORD8));
  }
}

/**
 *  ia_core_coder_arith_first_symbol
 *
 *  \brief First symbol decoding function.
 *
 *  \param [in]  it_bit_buff Pointer to bit buffer structure.
 *  \param [out] s           Pointer to arithmetic coding state structure.
 *
 *  \return WORD32 Bit count value.
 *
 */
WORD32 ia_core_coder_arith_first_symbol(ia_bit_buf_struct *it_bit_buff, ia_state_arith *s)
{
  WORD32 bit_count = 16;
  register WORD32 val;

  if (it_bit_buff->cnt_bits >= 16)
  {
    val = ia_core_coder_read_bits_buf(it_bit_buff, 16);
  }
  else
  {
    WORD32 shift_value = 16 - it_bit_buff->cnt_bits;
    val = ia_core_coder_read_bits_buf(it_bit_buff, it_bit_buff->cnt_bits);
    val <<= shift_value;
  }
  s->value = val;
  s->high = 65535;
  s->low = 0;
  return bit_count;
}

/**
 *  ia_core_coder_arith_get_pk
 *
 *  \brief Brief description
 *
 *  \param [in] c Code word param.
 *
 *  \return UWORD32
 *
 */
static UWORD32 ia_core_coder_arith_get_pk(UWORD32 c)
{
  WORD32 idx, idx_min, idx_max;
  UWORD32 j;
  idx_min = -1;
  idx_max = ARITH_HASH_TABLE_SIZE - 1;
  while ((idx_max - idx_min) > 1)
  {
    idx = idx_min + ((idx_max - idx_min) >> 1);
    j = ia_core_coder_ari_hash_m[idx];
    if (c > j)
    {
      idx_min = idx;
    }
    else if (c < j)
    {
      idx_max = idx;
    }
    else
    {
      return (UWORD32)ia_core_coder_ari_hash_m_1[idx];
    }
  }
  return (ia_core_coder_ari_lookup_m[idx_max]);
}

/**
 *  ia_core_coder_arith_decode
 *
 *  \brief Arithmetic Decoding.
 *
 *  \param [in]  it_bit_buff Pointer to input buffer handle.
 *  \param [in]  bit_count   Bit count value.
 *  \param [out] m           Pointer to buffer to store decoded symbol value.
 *  \param [i/o] s           Pointer to arithmetic state structure.
 *  \param [in]  cum_freq    Pointer to cumulative frequency table.
 *  \param [in]  cfl         Frame length value.
 *
 *  \return bit count
 *
 */

WORD32 ia_core_coder_arith_decode(ia_bit_buf_struct *it_bit_buff, WORD32 bit_count, WORD32 *m,
                                  ia_state_arith *s, UWORD16 const *cum_freq, WORD32 cfl)
{
  WORD32 low, high, range, value;
  WORD32 symbol;
  WORD32 cumulative;
  UWORD16 const *p;
  UWORD16 const *q;
  value = s->value;
  high = s->high;
  low = s->low;

  range = high - low + 1;
  cumulative = ((((WORD32)(value - low + 1)) << 14) - ((WORD32)1)) / ((WORD32)range);

  if ((it_bit_buff->cnt_bits == 0) && (cumulative <= 0))
  {
    return INVALID_BIT_COUNT;
  }
  p = cum_freq - 1;

  do
  {
    q = p + (cfl >> 1);
    if (*q > cumulative)
    {
      cfl++;
      p = q;
    }
    cfl = cfl >> 1;
  } while (cfl > 1);

  symbol = (WORD32)(p - cum_freq) + 1;

  if (symbol)
  {
    high = low + ((range * cum_freq[symbol - 1]) >> 14) - 1;
  }

  low += ((range * cum_freq[symbol]) >> 14);

  for (;;)
  {
    if (high < 32768)
    {
    }
    else if (low >= 32768)
    {
      value -= 32768;
      low -= 32768;
      high -= 32768;
    }
    else if (low >= 16384 && high < 49152)
    {
      value -= 16384;
      low -= 16384;
      high -= 16384;
    }
    else
    {
      break;
    }
    high += high + 1;
    low += low;

    if (it_bit_buff->cnt_bits <= 0)
    {
      value = (value << 1);
    }
    else
    {
      value = (value << 1) | ia_core_coder_read_bits_buf(it_bit_buff, 1);
    }
    bit_count++;
  }
  s->value = value;
  s->high = high;
  s->low = low;
  *m = symbol;

  return bit_count;
}

/**
 *  ia_core_coder_arith_get_context
 *
 *  \brief Arithmetic Decoding get context function.
 *
 *  \param [in]  c_prev Pointer to previous code word buffer.
 *  \param [in]  c_pres Pointer to present code word buffer.
 *  \param [out] c      Pointer to code word buffer.
 *  \param [in]  i      Index value.
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_arith_get_context(WORD8 *c_prev, WORD8 *c_pres, WORD32 *c_val,
                                              WORD32 i)
{
  WORD32 temp = (WORD32)c_prev[1] << 12;

  *c_val = *c_val >> 4;
  *c_val = *c_val + temp;
  *c_val = (*c_val & 0xFFF0) + c_pres[-1];

  if (i > 3)
  {
    if ((c_pres[-1] + c_pres[-2] + c_pres[-3]) < 5)
    {
      return (*c_val + 0x10000);
    }
  }

  return (*c_val);
}

/**
 *  ia_core_coder_arth_decoding_level2
 *
 *  \brief Arithmetic decoding level 2 function.
 *
 *  \param [in]  it_bit_buff
 *  \param [i/o] c_prev
 *  \param [i/o] c_pres
 *  \param [in]  n
 *  \param [in]  pres_n
 *  \param [out] quant
 *
 *  \return error IA_ERRORCODE if any
 *
 */
static IA_ERRORCODE ia_core_coder_arth_decoding_level2(ia_bit_buf_struct *it_bit_buff,
                                                       WORD8 *c_prev, WORD8 *c_pres, WORD32 n,
                                                       WORD32 pres_n, WORD32 *quant)
{
  WORD32 i, j;
  WORD32 s1;
  WORD32 temp;
  WORD32 bit_count_5;
  WORD32 lev, pki, esc_nb;
  WORD32 a = 0, b = 0;
  WORD32 m = 0, c = 0;
  WORD32 bit_count = 0;

  ia_bit_buf_struct it_bit_buff_temp = {0};
  ia_state_arith as;
  it_bit_buff_temp = *it_bit_buff;

  for (i = 0; i < pres_n; i++)
  {
    c_prev[i] = c_pres[i];
    c_pres[i] = 1;
  }

  bit_count = ia_core_coder_arith_first_symbol(&it_bit_buff_temp, &as);
  c = (WORD32)c_prev[0] << 12;

  for (i = 0; i < n; i++)
  {
    s1 = ia_core_coder_arith_get_context(c_prev + i, c_pres + i, &c, i);

    for (lev = esc_nb = 0;;)
    {
      pki = ia_core_coder_arith_get_pk(s1 + ia_core_coder_esc_nb_offset[esc_nb]);
      bit_count = ia_core_coder_arith_decode(&it_bit_buff_temp, bit_count, &m, &as,
                                             ia_core_coder_ari_cf_m[pki], 17);
      if (bit_count == INVALID_BIT_COUNT)
      {
        return IA_MPEGH_DEC_EXE_FATAL_ARITH_DECODE_FAILED;
      }
      if (m < ARITH_ESCAPE)
      {
        break;
      }
      lev += 1;
      esc_nb = lev;

      if (esc_nb > 7)
      {
        esc_nb = 7;
      }
    }

    if (m != 0)
    {
      a = m & 0x3;
      b = m >> 2;

      for (j = 0; j < lev; j++)
      {
        WORD32 lsbidx = (a == 0) ? 1 : ((b == 0) ? 0 : 2);
        bit_count = ia_core_coder_arith_decode(&it_bit_buff_temp, bit_count, &m, &as,
                                               ia_core_coder_ari_cf_r[lsbidx], 4);
        if (bit_count == INVALID_BIT_COUNT)
        {
          return IA_MPEGH_DEC_EXE_FATAL_ARITH_DECODE_FAILED;
        }
        if (a > (MAX_32 >> 1))
        {
          a = MAX_32;
        }
        else
        {
          a = (a << 1) | (m & 1);
        }

        if (b > (MAX_32 >> 1))
        {
          b = MAX_32;
        }
        else
        {
          b = (b << 1) | ((m >> 1) & 1);
        }
      }
      quant[2 * i + 0] = a;
      quant[2 * i + 1] = b;

      if ((a > (MAX_32 >> 1)) && (b > (MAX_32 >> 1)))
      {
        temp = MAX_32;
      }
      else
      {
        temp = a + b + 1;
      }
      c_pres[i] = (WORD8)temp;
      if (temp > 0xF)
      {
        c_pres[i] = 0xF;
      }
    }
    else
    {
      if (esc_nb > 0)
      {
        break;
      }
      quant[2 * i + 0] = 0;
      quant[2 * i + 1] = 0;
      c_pres[i] = 1;
    }
  }

  bit_count -= 16 - 2;
  if (bit_count > it_bit_buff->cnt_bits)
    return IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES;

  if (bit_count > 0)
  {
    bit_count_5 = bit_count >> 5;
    bit_count_5 = (bit_count_5 * 32) + (bit_count & 31);
    ia_core_coder_skip_bits_buf(it_bit_buff, bit_count_5);
  }

  for (i = 0; i < pres_n; i++)
  {
    WORD32 temp1 = quant[1];
    WORD32 temp0 = quant[0];
    if (temp0)
    {
      m = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      bit_count++;
      if ((m > (MAX_32 >> 1)) || (temp0 == MAX_32))
      {
        m = MAX_32;
        temp0 = MAX_32;
      }
      else
      {
        m = ia_core_coder_mul32_sh_sat(ia_core_coder_shl32_sat(m, 1), temp0, 0);
      }
      temp0 = m - (temp0);
    }

    if (temp1)
    {
      m = ia_core_coder_read_bits_buf(it_bit_buff, 1);
      bit_count++;
      if ((m > (MAX_32 >> 1)) || (temp1 == MAX_32))
      {
        m = MAX_32;
        temp1 = MAX_32;
      }
      else
      {
        m = ia_core_coder_mul32_sh_sat(ia_core_coder_shl32_sat(m, 1), temp1, 0);
      }
      temp1 = m - (temp1);
    }
    *quant++ = temp0;
    *quant++ = temp1;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_apply_scale_factor
 *
 *  \brief Scale factor application.
 *
 *  \param [i/o] coef          Pointer to coef buffer.
 *  \param [in]  fac           Factor value.
 *  \param [in]  pstr_sfb_info Pointer to scale factor band info structure.
 *  \param [in]  win_tot       Length parameter.
 *  \param [in]  sfb           Scalefactor band index.
 *  \param [in]  grp           Group index.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_apply_scale_factor(FLOAT32 *coef, FLOAT32 fac,
                                             ia_sfb_info_struct *pstr_sfb_info, WORD32 win_tot,
                                             WORD32 sfb, WORD32 grp)
{
  WORD32 i;
  WORD32 grp_win, length;
  WORD32 start, win;
  WORD32 drc_offset;
  FLOAT32 *p_coef;
  for (grp_win = 0; grp_win < pstr_sfb_info->group_len[grp]; grp_win++)
  {
    start = (sfb == 0) ? 0 : pstr_sfb_info->ptr_sfb_tbl[sfb - 1];
    win = win_tot + grp_win;
    drc_offset = win * pstr_sfb_info->bins_per_sbk;
    p_coef = &coef[drc_offset + start];
    length = pstr_sfb_info->ptr_sfb_tbl[sfb] - start;

    for (i = 0; i < length; i++)
    {
      p_coef[i] = ia_mul_flt(p_coef[i], fac);
    }
  }
}

/**
 *  ia_core_coder_randomsign_fix
 *
 *  \brief Random sign generator function.
 *
 *  \param [i/o] seed Seed value used to generate
 *
 *  \return sign
 *
 */
static WORD32 ia_core_coder_randomsign_fix(UWORD32 *seed)
{
  WORD32 sign = 0;
  *seed = (UWORD32)(((UWORD64)(*seed) * (UWORD64)69069) + 5);
  if (((*seed) & 0x10000) <= 0)
  {
    sign = +1;
  }
  else
  {
    sign = -1;
  }
  return sign;
}

/**
 *  impeghd_esc_iquant
 *
 *  \brief Inverse quantization function related to IGF data.
 *
 *  \param [i/o] q                Pointer to the buffer that stores quantized values.
 *  \param [i/o] coef             Pointer to coefficient buffer.
 *  \param [in]  igf_config       Pointer to IGF config structure.
 *  \param [in]  igf_all_zero     Flag to indicate if IGF is all zeros.
 *  \param [in]  igf_input_spec_float Input Spectral data.
 *  \param [in]  igf_num_tiles    Number of IGF tiles.
 *  \param [in]  noise_level      Noise level value.
 *  \param [in]  with_noise       Flag to indicate noise presence.
 *  \param [in]  seed_value       Pointer to seed value.
 *  \param [in]  length           Frame length parameter.
 *
 *  \return VOID
 *
 */
static VOID impeghd_esc_iquant(WORD32 *q, FLOAT32 *coef, ia_usac_igf_config_struct *igf_config,
                               WORD32 igf_all_zero, FLOAT32 *igf_input_spec_float,
                               WORD32 igf_num_tiles, FLOAT32 noise_level, WORD32 with_noise,
                               UWORD32 *seed_value, WORD32 length)
{
  WORD32 i, igf_cnt;
  WORD16 flag;

  for (i = 0; i < length; i++)
  {
    flag = 1;

    if (with_noise)
    {
      if (q[i] == 0)
      {
        coef[i] = ia_mul_flt(noise_level, (FLOAT32)ia_core_coder_randomsign_fix(seed_value));
        continue;
      }
    }

    WORD32 local_q = q[i];
    if (local_q < 0)
    {
      local_q = -local_q;
      flag = -1;
    }

    if (local_q >= 8192)
    {
      local_q = 8191;
    }

    if (local_q >= 1024)
    {
      coef[i] = ia_mul_flt((FLOAT32)flag, (float)pow((float)local_q, 4. / 3.));
    }
    else
    {
      coef[i] = ia_mul_flt((FLOAT32)flag, ia_core_coder_pow_table[local_q]);
    }
  }

  if (igf_config->igf_active)
  {
    for (i = 0; i < length; i++)
    {
      for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
      {
        if (!(igf_config->use_inf && q[i] == 0 && with_noise && !igf_all_zero))
        {
          igf_input_spec_float[igf_cnt * MAX_IGF_LEN + i] = coef[i];
        }
        else
        {
          igf_input_spec_float[igf_cnt * MAX_IGF_LEN + i] =
              ia_mul_flt((FLOAT32)ia_core_coder_randomsign_fix(seed_value), noise_level);
        }
      }
    }
  }

  return;
}

/**
*  ia_core_coder_apply_scfs_and_nf
*
*  \brief Apply scalefactors and noise filling.
*
*  \param [in]  noise_filling  Noise filling presence indicator flag.
*  \param [i/o] usac_data      Pointer to USAC data structure.
*  \param [i/o] quant          Pointer to quantization structure.
*  \param [in]  noise_level    Noise level parameter.
*  \param [in]  max_sfb        Max scalefactor band value.
*  \param [in]  max_noise_sfb  Max noise scalefactor band value.
*  \param [in]  igf_config     Pointer to IGF config structure.
*  \param [in]  igf_num_tiles  IGF number of tiles value.
*  \param [in]  igf_all_zero   IGF all zeroes flag.
*  \param [in]  chn            Channel index.
*  \param [in]  ch             Channel offset parameter.
*
*  \return VOID
*
*/
static VOID ia_core_coder_apply_scfs_and_nf(WORD32 noise_filling, ia_usac_data_struct *usac_data,
                                            WORD32 *quant, WORD32 noise_level, UWORD8 max_sfb,
                                            WORD32 max_noise_sfb,
                                            ia_usac_igf_config_struct *igf_config,
                                            WORD32 igf_num_tiles, WORD32 igf_all_zero, WORD32 chn,
                                            WORD32 ch)
{
  WORD32 igf_cnt;
  WORD32 grp = 0, win_tot = 0, sfb = 0;
  WORD32 noise_filling_start_offset = 0;
  WORD32 fac = 0, length = 0;
  UWORD32 *seed_value = &usac_data->seed_value[chn];
  WORD16 *factors = usac_data->factors[chn];
  FLOAT32 fac_float = 0;
  FLOAT32 noise_level_val = 0;
  FLOAT32 *igf_input_spec = usac_data->igf_dec_data[chn].igf_input_spec;
  FLOAT32 *dmx_re_prev = ch ? usac_data->dmx_re_prev[chn - ch] : NULL;
  FLOAT32 *coef_float = &usac_data->coef[chn][0];
  ia_sfb_info_struct *pstr_sfb_info = usac_data->pstr_sfb_info[chn];
  noise_filling_start_offset = (usac_data->ccfl == 768) ? (pstr_sfb_info->islong ? 120 : 15)
                                                        : (pstr_sfb_info->islong ? 160 : 20);

  if (noise_filling)
  {
    noise_level_val = ia_core_coder_pow_14_3[noise_level >> 5];
  }
  memset(igf_input_spec, 0, sizeof(FLOAT32) * MAX_IGF_LEN * 4);

  for (grp = 0; grp < pstr_sfb_info->num_groups; grp++)
  {
    WORD32 grp_win = 0;
    for (sfb = 0; sfb < (WORD32)max_sfb; sfb++)
    {
      WORD32 band_quantized_to_zero = 1;
      WORD32 noise_filling_present = 0;
      WORD32 sfb_offset = win_tot * pstr_sfb_info->sfb_per_sbk;
      fac = (WORD32)(factors[sfb_offset + sfb] - SF_OFFSET);
      for (grp_win = 0; grp_win < pstr_sfb_info->group_len[grp]; grp_win++)
      {
        WORD32 idx = 0;
        WORD32 start = (sfb == 0) ? 0 : pstr_sfb_info->ptr_sfb_tbl[sfb - 1];
        WORD32 win = grp_win + win_tot;
        WORD32 drc_offset = win * pstr_sfb_info->bins_per_sbk;
        for (idx = start; idx < pstr_sfb_info->ptr_sfb_tbl[sfb]; idx++)
        {
          if (quant[drc_offset + idx] != 0)
          {
            band_quantized_to_zero = 0;
            break;
          }
          if (!band_quantized_to_zero)
          {
            break;
          }
        }
      }

      for (grp_win = 0; grp_win < pstr_sfb_info->group_len[grp]; grp_win++)
      {
        WORD32 start = (sfb == 0) ? 0 : pstr_sfb_info->ptr_sfb_tbl[sfb - 1];
        WORD32 win = win_tot + grp_win;
        WORD32 drc_offset = win * pstr_sfb_info->bins_per_sbk;

        if (noise_filling)
        {
          noise_filling_present =
              (start >= noise_filling_start_offset) && noise_filling && sfb < max_noise_sfb;
          if (band_quantized_to_zero && (grp_win == 0))
          {
            fac += ((noise_level & 0x1f) - 16);
          }
        }
        if (grp_win == 0)
        {
          fac_float = (FLOAT32)pow(2.f, ia_mul_flt((FLOAT32)fac, 0.25f));
          usac_data->scale_factors[chn][sfb_offset + sfb] =
              ((start >= noise_filling_start_offset) && (sfb < max_sfb) &&
               band_quantized_to_zero && !igf_config->igf_stereo_filling)
                  ? fac_float
                  : 0.0f;
        }
        length = pstr_sfb_info->ptr_sfb_tbl[sfb] - start;

        impeghd_esc_iquant(&quant[drc_offset + start], &coef_float[drc_offset + start],
                           igf_config, igf_all_zero, &igf_input_spec[drc_offset + start],
                           igf_num_tiles, noise_level_val, noise_filling_present, seed_value,
                           length);
      }

      if (igf_config->igf_active)
      {
        for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
        {
          ia_core_coder_igf_stereofilling(&igf_input_spec[igf_cnt * MAX_IGF_LEN], dmx_re_prev,
                                          pstr_sfb_info, noise_filling, band_quantized_to_zero,
                                          igf_config->igf_stereo_filling, win_tot, sfb,
                                          noise_filling_start_offset, grp, max_noise_sfb);
        }
      }
      ia_core_coder_igf_stereofilling(coef_float, dmx_re_prev, pstr_sfb_info, noise_filling,
                                      band_quantized_to_zero, igf_config->igf_stereo_filling,
                                      win_tot, sfb, noise_filling_start_offset, grp,
                                      max_noise_sfb);

      ia_core_coder_apply_scale_factor(coef_float, fac_float, pstr_sfb_info, win_tot, sfb, grp);

      if (igf_config->igf_active)
      {
        for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
        {
          ia_core_coder_apply_scale_factor(&igf_input_spec[igf_cnt * MAX_IGF_LEN], fac_float,
                                           pstr_sfb_info, win_tot, sfb, grp);
        }
      }
    }

    for (/* sfb */; sfb < pstr_sfb_info->sfb_per_sbk; sfb++)
    {
      usac_data->scale_factors[chn][win_tot * pstr_sfb_info->sfb_per_sbk + sfb] = 0.0f;
    }
    win_tot += pstr_sfb_info->group_len[grp];
  }

  return;
}
/**
 *  ia_core_coder_fd_chn_fdp_decode
 *
 *  \brief Frequency Domain Prediction decoding.
 *
 *  \param [i/o] usac_data Pointer to USAC data structure.
 *  \param [i/o] quant     Pointer to quantization structure.
 *  \param [in]  fdp_spacing_index Spacing index value extracted from bit stream.
 *  \param [in]  pred_bw   Prediction bandwidth.
 *  \param [in]  max_sfb   Max Scalefactor band value.
 *  \param [in]  chn       Processing channel index.
 *
 *  \return VOID
 *
 */
static VOID ia_core_coder_fd_chn_fdp_decode(ia_usac_data_struct *usac_data, WORD32 *quant,
                                            WORD32 fdp_spacing_index, WORD32 pred_bw,
                                            WORD32 max_sfb, WORD32 chn)
{
  WORD32 i = 0;
  WORD32 s1, s2, sfb, harmonic_spacing = 0;
  WORD32 fdp_int[172];
  WORD32 harmonic_idx = -128, compare_idx = 256;
  WORD32 is_eight_short_sequence = !usac_data->pstr_sfb_info[chn]->islong;
  const WORD16 *sfb_offset = usac_data->pstr_sfb_info[chn]->ptr_sfb_tbl - 1;
  WORD16 *quant_scf_curr = usac_data->factors[chn];
  WORD16 *quant_spec_prev2 = usac_data->fdp_quant_spec_prev[chn];
  WORD16 *quant_spec_prev1 = usac_data->fdp_quant_spec_prev[chn] + 172;
  FLOAT32 *coef = usac_data->coef[chn];

  if (!(is_eight_short_sequence || (max_sfb <= 0)))
  {
    if (pred_bw < sfb_offset[max_sfb])
    { /* ISO 23003-3, Table 109 */
      max_sfb = 1;
      while (sfb_offset[max_sfb] < pred_bw)
        max_sfb++;

      if ((fdp_spacing_index >= 0) && (fdp_spacing_index < 256))
      {
        harmonic_spacing = ia_core_coder_harmonic_spacing[fdp_spacing_index];
      }
    }
  }
  else
  {
    max_sfb = 0;
  }

  s2 = 0;
  s1 = 0;

  /* start decoding: for each quantized bin compute predictor estimate */
  for (sfb = 0; sfb < max_sfb; sfb++)
  {
    WORD32 scf16 = quant_scf_curr[sfb] - SF_OFFSET - 21;

    if (scf16 >= 0)
    {
      scf16 = scf16 > 62 ? 65536 : (WORD32)ia_core_coder_fdp_scf[scf16]; /* scf look-up */
    }
    else
    {
      scf16 = 0;
    }

    if (harmonic_spacing)
    { /* FDP active and allowed, compute estimate */
      for (i = (sfb <= 0) ? 0 : sfb_offset[sfb]; i < sfb_offset[sfb + 1]; i++)
      {
        if (ia_abs_int(i * 256 - harmonic_idx) < 384)
        { /* bin is not harmonic */
          const WORD32 reg32 =
              s1 * (WORD32)quant_spec_prev1[i] + s2 * (WORD32)quant_spec_prev2[i];
          fdp_int[i] = (WORD32)(((UWORD32)ia_abs_int(reg32) + 16384) >> 15);
          if (reg32 < 0)
          {
            fdp_int[i] *= -1;
          }
          coef[i] = ia_mac_flt(coef[i], 512.0f, (FLOAT32)fdp_int[i]);
        }
        else
        { /* bin is part of the currently active harmonic line */
          fdp_int[i] = 0;
        }
        if (i * 256 == compare_idx)
        { /* update indices and LPC coeffs */
          harmonic_idx = harmonic_idx + harmonic_spacing;
          compare_idx = harmonic_idx & 255;
          if (compare_idx > 128)
          {
            compare_idx = 256 - compare_idx; /* exploit trigonom. symmetry */
          }
          s2 = ia_core_coder_fdp_s2[compare_idx];
          s1 = ia_core_coder_fdp_s1[compare_idx];
          compare_idx = harmonic_idx >> 8; /* integer unscaled harm. index */
          if ((compare_idx & 1) == 0)
          {
            s1 *= -1; /* negate first LPC coeff for even harm. indices */
          }
          compare_idx = 256 + ((harmonic_idx + 128) >> 8) * 256;
        }
      }
    }

    /* start update: compute integer sum of each line and its estimate */
    for (i = (sfb <= 0) ? 0 : sfb_offset[sfb]; i < sfb_offset[sfb + 1]; i++)
    {
      WORD32 x_int = ia_core_coder_fdp_exp[ia_min_int(ia_abs_int(quant[i]), 181)]; /* look-up */
      x_int = (WORD32)((512 + (UWORD32)x_int * (UWORD32)scf16) >> 10);
      if (quant[i] < 0)
      {
        x_int *= -1;
      }
      if (harmonic_spacing)
      {
        x_int += fdp_int[i]; /* add previously computed FDP estimate */
      }

      quant_spec_prev2[i] = quant_spec_prev1[i];
      quant_spec_prev1[i] = (short)ia_min_int(ia_max_int(x_int, QUANT_SPEC_MIN), QUANT_SPEC_MAX);
    }
  }

  /* finalize update: reset states of currently uncoded spectral lines */
  for (i = ia_min_int(i, pred_bw); i < UNCODED_LINES; i++)
  {
    quant_spec_prev2[i] = quant_spec_prev1[i] = 0;
  }
}

/**
 *  ia_core_coder_ac_spectral_data
 *
 *  \brief Spectral data decoding.
 *
 *  \param [i/o] usac_data             Pointer to USAC data structure.
 *  \param [in]  max_spec_coefficients Maximum spectral coefficients.
 *  \param [in]  noise_level           Noise level parameter.
 *  \param [in]  arith_pres_n          nth arithmetic coded symbol.
 *  \param [in]  it_bit_buff           Pointer to bit buffer structure.
 *  \param [in]  max_sfb               Max scale factor band value.
 *  \param [in]  max_noise_sfb         Max noise scale factor band.
 *  \param [in]  igf_config            Pointer to IGF configuration structure.
 *  \param [in]  igf_num_tiles         IGF number of tiles.
 *  \param [in]  igf_all_zero          IGF all zeroes flag.
 *  \param [in]  arith_reset_flag      Arithmetic decoder state reset flag.
 *  \param [in]  noise_filling         Noise filling presence flag.
 *  \param [in]  chn                   Channel index.
 *  \param [in]  ch                    Channel offset parameter.
 *  \param [in]  fdp_spacing_idx       FDP spacing index bs parameter.
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE ia_core_coder_ac_spectral_data(
    ia_usac_data_struct *usac_data, WORD32 max_spec_coefficients, WORD32 noise_level,
    WORD32 arith_pres_n, ia_bit_buf_struct *it_bit_buff, UWORD8 max_sfb, WORD32 max_noise_sfb,
    ia_usac_igf_config_struct *igf_config, WORD32 igf_num_tiles, WORD32 igf_all_zero,
    WORD32 arith_reset_flag, WORD32 noise_filling, WORD32 chn, WORD32 ch, WORD32 fdp_spacing_idx)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 i;
  WORD32 sbk;
  WORD32 igf_cnt;
  WORD32 fs = ia_core_coder_sample_freq_idx_table[usac_data->sampling_rate_idx];
  WORD32 sbk_sfb_top = 0;
  const WORD32 max_win_len = usac_data->pstr_sfb_info[chn]->max_win_len;
  const WORD32 pred_bw =
      (usac_data->ccfl != 768) && (fs >= 44100) ? 132 : (160 * usac_data->ccfl) / 1024;
  WORD32 *x_ac_dec;
  WORD32 *ptr_scratch_buf;
  WORD8 *c_prev = &usac_data->c_prev[chn][0];
  WORD8 *c_pres = &usac_data->c[chn][0];
  FLOAT32 *igf_input_spec = &(usac_data->igf_dec_data[chn].igf_input_spec[0]);
  ia_arith_dec_scratch *pstr_scratch = (ia_arith_dec_scratch *)usac_data->scratch_int_buf;
  x_ac_dec = pstr_scratch->scratch_x_ac_dec;
  ptr_scratch_buf = pstr_scratch->scratch_map_context;

  if (max_noise_sfb <= 0)
  {
    sbk_sfb_top = usac_data->pstr_sfb_info[chn]->ptr_sfb_tbl[0];
  }
  else
  {
    sbk_sfb_top = usac_data->pstr_sfb_info[chn]->ptr_sfb_tbl[max_noise_sfb - 1];
  }

  memset(x_ac_dec, 0, 1024 * sizeof(WORD32));
  ia_core_coder_arith_map_context(arith_pres_n, usac_data->arith_prev_n[chn], c_prev, c_pres,
                                  arith_reset_flag, ptr_scratch_buf);

  usac_data->arith_prev_n[chn] = arith_pres_n;

  if (max_spec_coefficients <= 0)
  {
    for (i = 0; i < (arith_pres_n / 2); i++)
    {
      c_pres[i + 2] = 1;
    }
  }
  else
  {
    for (sbk = 0; sbk < max_win_len; sbk++)
    {
      err_code = ia_core_coder_arth_decoding_level2(it_bit_buff, c_prev + 2, c_pres + 2,
                                                    max_spec_coefficients / 2, arith_pres_n / 2,
                                                    &x_ac_dec[sbk * arith_pres_n]);
      if (err_code != IA_MPEGH_DEC_NO_ERROR)
      {
        return err_code;
      }

      for (i = max_spec_coefficients / 2; i < arith_pres_n / 2; i++)
      {
        x_ac_dec[sbk * arith_pres_n + 2 * i + 0] = 0;
        x_ac_dec[sbk * arith_pres_n + 2 * i + 1] = 0;
      }
    }
  }

  ia_core_coder_apply_scfs_and_nf(noise_filling, usac_data, x_ac_dec, noise_level, max_sfb,
                                  max_noise_sfb, igf_config, igf_num_tiles, igf_all_zero, chn,
                                  ch);

  if (usac_data->usac_independency_flg)
  {
    memset(&usac_data->fdp_quant_spec_prev[chn], 0, 2 * 172 * sizeof(WORD16));
  }
  ia_core_coder_fd_chn_fdp_decode(usac_data, x_ac_dec, fdp_spacing_idx, pred_bw, max_sfb, chn);

  if (igf_config->igf_active && usac_data->pstr_sfb_info[chn]->islong && (max_noise_sfb > 0) &&
      (fdp_spacing_idx >= 0))
  {
    for (igf_cnt = 0; igf_cnt < igf_num_tiles; igf_cnt++)
    {
      memcpy(&igf_input_spec[igf_cnt * MAX_IGF_LEN], &usac_data->coef[chn][0],
             sizeof(FLOAT32) * ia_min_int(pred_bw, sbk_sfb_top));
    }
  }
  return IA_MPEGH_DEC_NO_ERROR;
}

/**
 *  ia_core_coder_arith_data
 *
 *  \brief Arithmetic data extraction.
 *
 *  \param [i/o] pstr_td_frame_data Pointer to TD frame data structure.
 *  \param [out] x_ac_dec       Pointer to airthmetic decoder data.
 *  \param [in]  usac_data      Pointer to Usac data structure.
 *  \param [in]  it_bit_buff    Pointer to bit buffer structure.
 *  \param [in]  first_tcx_flag First TCX frame flag.
 *  \param [in]  k              Index value.
 *
 *  \return error IA_ERRORCODE if any
 *
 */
IA_ERRORCODE ia_core_coder_arith_data(ia_td_frame_data_struct *pstr_td_frame_data,
                                      WORD32 *x_ac_dec, ia_usac_data_struct *usac_data,
                                      ia_bit_buf_struct *it_bit_buff, WORD32 first_tcx_flag,
                                      WORD32 k)
{
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  WORD32 arith_reset_flag = first_tcx_flag && pstr_td_frame_data->arith_reset_flag;
  WORD32 tcx_size = pstr_td_frame_data->tcx_lg[k];
  WORD32 *arith_prev_n = &usac_data->arith_prev_n[usac_data->present_chan];
  WORD8 *c_prev = &usac_data->c_prev[usac_data->present_chan][0];
  WORD8 *c_pres = &usac_data->c[usac_data->present_chan][0];
  ia_arith_dec_scratch *pstr_scratch = (ia_arith_dec_scratch *)usac_data->ptr_fft_scratch;
  memset(x_ac_dec, 0, tcx_size * sizeof(WORD32));
  ia_core_coder_arith_map_context(tcx_size, *arith_prev_n, c_prev, c_pres, arith_reset_flag,
                                  pstr_scratch->scratch_map_context);

  *arith_prev_n = tcx_size;
  err_code = ia_core_coder_arth_decoding_level2(it_bit_buff, c_prev + 2, c_pres + 2, tcx_size / 2,
                                                tcx_size / 2, x_ac_dec);

  return err_code;
}
/** @} */ /* End of CoreDecProc */