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
#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "impeghd_intrinsics_flt.h"
#include "impeghd_error_codes.h"
#include "ia_core_coder_cnst.h"
#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_constants.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_igf_data_struct.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_stereo_lpd.h"
#include "ia_core_coder_tns_usac.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "impeghd_tbe_dec.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_main.h"
#include "ia_core_coder_arith_dec.h"
#include "ia_core_coder_bit_extract.h"
#include "ia_core_coder_igf_dec.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_igf_get_swb_offset
 *
 *  \brief Calculate swb offset
 *
 *  \param [in]    igf_win_type    igf window type
 *  \param [out]  swb_offset      output swb offset array pointer
 *  \param [in]    pstr_usac_winmap  sfb info structure
 *
 *
 *
 */
VOID ia_core_coder_igf_get_swb_offset(WORD32 igf_win_type, WORD16 *swb_offset,
                                      ia_sfb_info_struct *pstr_usac_winmap)
{
  WORD32 sfb;
  WORD32 swb_shift = 0;
  ia_sfb_info_struct *igf_info = pstr_usac_winmap;

  if (igf_win_type == 2 || igf_win_type == 3)
    swb_shift = (WORD32)igf_win_type;

  for (sfb = 0; sfb < igf_info->sfb_per_sbk; sfb++)
  {
    swb_offset[sfb + 1] = (*(igf_info->ptr_sfb_tbl + sfb)) << swb_shift;
  }
}

/**
 *  ia_core_coder_igf_quant_ctx
 *
 *  \brief Calculates CTX (index to  the  context) containing  the  probability
 * distribution
 *
 *  \param [in]    ctx    index
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_quant_ctx(const WORD32 ctx)
{
  if (ctx > 3)
  {
    return 3;
  }
  else if (ctx < -3)
  {
    return -3;
  }
  else
  {
    return ctx;
  }
}

/**
 *  ia_core_coder_igf_arith_decode_bits
 *
 *  \brief This helper  function uses the USAC arithmetic decoder function to
 *          obtain a  value of length  num_bits from bitstream
 *
 *  \param [in]    it_bit_buf    bit stream buffer
 *  \param [in]    ptr_bit_offset  offset
 *  \param [in,out]  stat      arithmetic decoder state buffer
 *  \param [in]    num_bits    number of bits
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_arith_decode_bits(ia_bit_buf_struct *it_bit_buf,
                                                  WORD32 *ptr_bit_offset, ia_state_arith *stat,
                                                  const WORD32 num_bits)
{
  WORD32 i = 0;
  WORD32 num_bits_read = 0;
  WORD32 bit = 0;
  for (i = num_bits - 1; i >= 0; --i)
  {
    *ptr_bit_offset = ia_core_coder_arith_decode(it_bit_buf, *ptr_bit_offset, &bit, stat,
                                                 ia_core_coder_cf_for_bit, 2);
    num_bits_read = num_bits_read + (bit << i);
  }
  return num_bits_read;
}

/**
 *  ia_core_coder_igf_arith_decode_residual
 *
 *  \brief Is a helper function that returns the decoded residual value, which will be added
 *       to the predicted  value to obtain  the  original value
 *
 *  \param [in]    it_bit_buf      bit stream buffer
 *  \param [in]    ptr_bit_offset    offset
 *  \param [in,out]  stat        arithmetic decoder state buffer
 *  \param [in]    ptr_cum_freq_table  cumulative freq table
 *  \param [in]    table_offset    table offset
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_arith_decode_residual(ia_bit_buf_struct *it_bit_buf,
                                                      WORD32 *ptr_bit_offset,
                                                      ia_state_arith *stat,
                                                      const UWORD16 *ptr_cum_freq_table,
                                                      const WORD32 table_offset)
{
  WORD32 val = 0;
  WORD32 dec_res_val = 0;

  *ptr_bit_offset = ia_core_coder_arith_decode(it_bit_buf, *ptr_bit_offset, &val, stat,
                                               ptr_cum_freq_table, SYMBOLS_IN_TABLE);

  if ((val != SYMBOLS_IN_TABLE - 1) && (val != 0))
  {
    dec_res_val = (val - 1) + MIN_ENC_SEPARATE;
    dec_res_val -= table_offset;
    return dec_res_val;
  }
  else
  {
    WORD32 extra = ia_core_coder_igf_arith_decode_bits(it_bit_buf, ptr_bit_offset, stat, 4);
    if (extra == 15)
    {
      extra = ia_core_coder_igf_arith_decode_bits(it_bit_buf, ptr_bit_offset, stat, 7);
      extra = 15 + extra;
    }

    if (val == 0)
    {
      dec_res_val = (MIN_ENC_SEPARATE - 1) - extra;
    }
    else
    {
      dec_res_val = (MAX_ENC_SEPARATE + 1) + extra;
    }
  }
  dec_res_val -= table_offset;

  return dec_res_val;
}

/**
 *  ia_core_coder_igf_arith_decode
 *
 *  \brief Performs arithmetic decoding of igf bits
 *
 *  \param [in]    it_bit_buf      bit stream buffer
 *  \param [in]    bit_offset      offset
 *  \param [in,out]  stat        arithmetic decoder state buffer
 *  \param [in]    igf_win_type    igf window table
 *  \param [in]    igf_config      igf config structure
 *  \param [in]    igf_dec_data    igf decoder data structure
 *  \param [out]  igf_curr      bufffer  containing  the  IGF
 * levels
 * for  the  current  window
 *
 *
 *
 */
static VOID ia_core_coder_igf_arith_decode(ia_bit_buf_struct *it_bit_buf, WORD32 *bit_offset,
                                           ia_state_arith *stat, WORD32 igf_win_type,
                                           ia_usac_igf_config_struct *igf_config,
                                           ia_usac_igf_dec_data_struct *igf_dec_data,
                                           WORD32 *igf_curr)
{
  WORD32 sfb, i, pred, ctx, ctx_f, ctx_t, inc = 1;
  WORD32 m_igf_sfb_start = igf_config->igf_grid[igf_win_type].igf_sfb_start;
  WORD32 m_igf_sfb_stop = igf_config->igf_grid[igf_win_type].igf_sfb_stop;
  WORD32 t = igf_dec_data->igf_envelope.igf_arith_time_idx;
  WORD32 *igf_prev = igf_dec_data->igf_envelope.igf_levels_prev_fix + m_igf_sfb_start;
  WORD32 igf_prev_d = igf_dec_data->igf_envelope.igf_prev_win_ctx;

  WORD32 length = m_igf_sfb_stop - m_igf_sfb_start;

  if ((0 == igf_win_type) && !igf_config->use_high_res)
  {
    inc = SFE_GROUP_SIZE;
  }

  for (sfb = 0; sfb < length; sfb += inc)
  {
    if (t == 0)
    {
      if (sfb == inc)
      {
        pred = igf_curr[sfb - inc];
        igf_curr[sfb] =
            pred + ia_core_coder_igf_arith_decode_residual(
                       it_bit_buf, bit_offset, stat, ia_core_coder_cf_se_01, CF_OFFSET_SE_01);
      }
      else if (sfb == 0)
      {
        igf_curr[sfb] = ia_core_coder_igf_arith_decode_bits(it_bit_buf, bit_offset, stat, 7);
      }
      else
      {
        pred = igf_curr[sfb - inc];
        ctx = ia_core_coder_igf_quant_ctx(igf_curr[sfb - inc] - igf_curr[sfb - 2 * inc]);
        igf_curr[sfb] =
            pred + ia_core_coder_igf_arith_decode_residual(
                       it_bit_buf, bit_offset, stat, ia_core_coder_cf_se_02[CTX_OFFSET + ctx],
                       ia_core_coder_cf_offset_se_02[CTX_OFFSET + ctx]);
      }
    }
    else if (sfb == 0)
    {
      if (t != 1)
      {
        pred = igf_prev[sfb];
        ctx = ia_core_coder_igf_quant_ctx(igf_prev[sfb] - igf_prev_d);
        igf_curr[sfb] =
            pred + ia_core_coder_igf_arith_decode_residual(
                       it_bit_buf, bit_offset, stat, ia_core_coder_cf_se_20[CTX_OFFSET + ctx],
                       ia_core_coder_cf_offset_se_20[CTX_OFFSET + ctx]);
      }
      else
      {
        pred = igf_prev[sfb];
        igf_curr[sfb] =
            pred + ia_core_coder_igf_arith_decode_residual(
                       it_bit_buf, bit_offset, stat, ia_core_coder_cf_se_10, CF_OFFSET_SE_10);
      }
    }
    else
    {
      pred = igf_prev[sfb] + igf_curr[sfb - inc] - igf_prev[sfb - inc];
      ctx_t = ia_core_coder_igf_quant_ctx(igf_curr[sfb - inc] - igf_prev[sfb - inc]);
      ctx_f = ia_core_coder_igf_quant_ctx(igf_prev[sfb] - igf_prev[sfb - inc]);

      igf_curr[sfb] =
          pred + ia_core_coder_igf_arith_decode_residual(
                     it_bit_buf, bit_offset, stat,
                     ia_core_coder_cf_se_11[CTX_OFFSET + ctx_t][CTX_OFFSET + ctx_f],
                     ia_core_coder_cf_offset_se_11[CTX_OFFSET + ctx_t][CTX_OFFSET + ctx_f]);
    }
    for (i = 1; i < inc; ++i)
    {
      if (sfb + i < length)
      {
        igf_curr[sfb + i] = igf_curr[sfb];
      }
    }
  }
}

/**
 *  ia_core_coder_igf_tnf_calc_par_coef
 *
 *  \brief Calculates LPC par coeff
 *
 *  \param [out]   par_coef pointer to par_coef array
 *  \param [in]    input    acf Input
 *  \param [in]    order    order of filter
 *
 *
 *
 */
static VOID ia_core_coder_igf_tnf_calc_par_coef(FLOAT32 *par_coef, const FLOAT32 *input,
                                                const WORD32 order)
{
  WORD32 i;
  WORD32 j;
  FLOAT32 tmp;
  FLOAT32 tmp2;
  FLOAT32 mem[32] = {0.0f};
  FLOAT32 *const p_mem = &mem[order];
  const FLOAT32 threshold = 0.0000152f;

  ia_core_coder_mem_cpy(input, mem, order);
  ia_core_coder_mem_cpy(&input[1], p_mem, order);

  for (i = 0; i < order; i++)
  {
    tmp = 0;
    if (mem[0] >= threshold)
    {
      tmp = -p_mem[i] / mem[0];
    }

    tmp = ia_min_flt(0.999f, ia_max_flt(-0.999f, tmp));
    par_coef[i] = tmp;

    for (j = i; j < order; j++)
    {
      tmp2 = ia_mac_flt(p_mem[j], tmp, mem[j - i]);
      mem[j - i] = ia_mac_flt(mem[j - i], tmp, p_mem[j]);
      p_mem[j] = tmp2;
    }
  }
}
/**
 *  ia_core_coder_igf_tnf_convert_lpc
 *
 *  \brief Calculates LPC pred coeffs and LPC gain
 *
 *  \param [in]    input    acf Input
 *  \param [in]    order    order of filter
 *  \param [out]   pred_coef  LPC pred coeff
 *
 *
 *
 */
static VOID ia_core_coder_igf_tnf_convert_lpc(const FLOAT32 *input, const WORD32 order,
                                              FLOAT32 *pred_coef)
{
  WORD32 i;
  WORD32 j;
  FLOAT32 tmp;
  FLOAT32 par_coef[16] = {0.0f};

  ia_core_coder_igf_tnf_calc_par_coef(par_coef, input, order);

  pred_coef[0] = 1.0f;
  pred_coef[1] = par_coef[0];

  for (i = 1; i < order; i++)
  {
    for (j = 0; j < (i >> 1); j++)
    {
      tmp = pred_coef[j + 1];
      pred_coef[j + 1] = ia_mac_flt(pred_coef[j + 1], par_coef[i], pred_coef[i - 1 - j + 1]);
      pred_coef[i - 1 - j + 1] = ia_mac_flt(pred_coef[i - 1 - j + 1], par_coef[i], tmp);
    }
    if (i & 1)
    {
      pred_coef[j + 1] = ia_mac_flt(pred_coef[j + 1], par_coef[i], pred_coef[j + 1]);
    }
    pred_coef[i + 1] = par_coef[i];
  }
}

/**
 *  ia_core_coder_igf_tnf_filter
 *
 *  \brief Performs igf tnf filtering
 *
 *  \param [out]   spec       tnf spectral coeff
 *  \param [in]    tnf_len    tnf length
 *  \param [in]    pred_coef  prediction coeff
 *  \param [in]    order      order of filter
 *  \param [in]    pbuf       Scratch buffer for internal processing
 *
 *
 *
 */
static VOID ia_core_coder_igf_tnf_filter(FLOAT32 *spec, const WORD32 tnf_len,
                                         const FLOAT32 *pred_coef, const WORD32 order,
                                         FLOAT32 *pbuf)
{
  WORD32 i;
  WORD32 j;
  if (order > 0)
  {
    for (j = 0; j < tnf_len; j++, pbuf++)
    {
      *pbuf = spec[j];
      for (i = 1; (i < order) && (j >= i); i++)
      {
        spec[j] = ia_add_flt(spec[j], ia_mul_flt(pred_coef[i], pbuf[-i]));
      }
    }
  }
}

/**
 *  ia_core_coder_igf_tnf_norm
 *
 *  \brief Calculates norm values
 *
 *  \param [in,out]    x    tnf spec coeff
 *  \param [in]      n    range
 *
 *  \return WORD64
 *
 */
static FLOAT32 ia_core_coder_igf_tnf_norm(const FLOAT32 *x, const WORD32 n)
{
  WORD32 i;
  FLOAT32 acc = 0.0f;

  for (i = 0; i < n; i++)
  {
    acc = ia_add_flt(acc, ia_mul_flt(x[i], x[i]));
  }
  return acc;
}

/**
 *  ia_core_coder_igf_tnf_autocorrelation
 *
 *  \brief Calculates tnf autocorrelation and returns the value
 *
 *  \param [in]    x  tnf spectral coeff
 *  \param [in]    n  tnf range
 *  \param [in]    lag  lag
 *
 *  \return FLOAT32
 *
 */
static FLOAT32 ia_core_coder_igf_tnf_autocorrelation(const FLOAT32 *x, const WORD32 n,
                                                     const WORD32 lag)
{
  WORD32 i;
  FLOAT32 acc = 0.0f;

  if (n - lag)
  {
    acc = ia_mul_flt(x[0], x[lag]);
  }
  for (i = 1; i < n - lag; i++)
  {
    acc = ia_add_flt(acc, ia_mul_flt(x[i], x[i + lag]));
  }
  return acc;
}

/**
 *  ia_core_coder_igf_tnf_detect
 *
 *  \brief Detects igf tnf and estimates pred coeffs and gain
 *
 *  \param [in]    p_spec    tnf spectral coeff
 *  \param [in]    igf_start  igf start subband
 *  \param [in]    igf_stop  igf start subband
 *  \param [in]    pred_coef  estimate pred coeff
 *  \param [in]    order    order
 *
 *
 *
 */
VOID ia_core_coder_igf_tnf_detect(const FLOAT32 *p_spec, const WORD32 igf_start,
                                  const WORD32 igf_stop, FLOAT32 *pred_coef, WORD32 *order)
{
  WORD32 i;
  WORD32 lag;
  WORD32 start_line;
  WORD32 stop_line;
  WORD32 tnf_range;
  FLOAT32 fac;
  FLOAT32 norms[3] = {0.0f};
  FLOAT32 acf_x[16 + 1] = {0.0f};
  const FLOAT32 threshold = 0.0000000037f;
  const WORD32 igf_range = igf_stop - igf_start;

  for (i = 0; i < 3; i++)
  {
    start_line = igf_start + igf_range * i / 3;
    stop_line = igf_start + igf_range * (i + 1) / 3;
    tnf_range = stop_line - start_line;
    norms[i] = ia_core_coder_igf_tnf_norm(p_spec + start_line, tnf_range);
  }
  for (i = 0; (i < 3) && (norms[i] > threshold); i++)
  {
    start_line = igf_start + igf_range * i / 3;
    stop_line = igf_start + igf_range * (i + 1) / 3;
    tnf_range = stop_line - start_line;
    fac = 1.0f / norms[i];
    for (lag = 1; lag <= 8; lag++)
    {
      acf_x[lag] = ia_add_flt(
          acf_x[lag], ia_mul_flt(fac, ia_mul_flt(ia_core_coder_tnf_acf_win[lag - 1],
                                                 ia_core_coder_igf_tnf_autocorrelation(
                                                     p_spec + start_line, tnf_range, lag))));
    }
  }

  if (i == 3)
  {
    *order = 8;
    acf_x[0] = 3.0f;
    ia_core_coder_igf_tnf_convert_lpc(acf_x, ia_min_int(8, igf_range >> 2), pred_coef);
  }
}

/**
 *  ia_core_coder_tnf_apply
 *
 *  \brief Apply tnf
 *
 *  \param [in,out]  igf_dec_data    igf decoder data
 *  \param [in]    igf_bgn        igf start subband
 *  \param [in]    igf_end        igf stop subband
 *
 *
 *
 */
static VOID ia_core_coder_tnf_apply(ia_usac_igf_dec_data_struct *igf_dec_data, WORD32 igf_bgn,
                                    WORD32 igf_end)
{
  WORD32 order = 0;
  FLOAT32 *pred_coef = igf_dec_data->igf_scratch;
  FLOAT32 *tmp_scratch = pred_coef + 17;
  ia_core_coder_igf_tnf_detect(igf_dec_data->igf_tnf_spec_float, igf_bgn, igf_end, pred_coef,
                               &order);

  ia_core_coder_igf_tnf_filter(&igf_dec_data->igf_tnf_spec_float[igf_bgn], igf_end - igf_bgn,
                               pred_coef, order, tmp_scratch);
}

/**
 *  ia_core_coder_igf_get_igf_min
 *
 *  \brief igf_min is a  subband  index which determine a  minimal  frequency to
 * assign  a source tile range
 *
 *  \param [in]    igf_win_type    igf window type
 *  \param [in]    ccfl        framelength
 *  \param [in]    fs          sampling frequency
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_get_igf_min(WORD32 igf_win_type, WORD32 ccfl, WORD32 fs)
{
  WORD32 igf_min;
  WORD32 bl = ccfl;

  switch (igf_win_type)
  {
  case 2:
    bl = ccfl >> 1;
    break;
  case 1:
    bl = ccfl >> 3;
    break;
  }

  igf_min = (WORD32)((bl * 1125) / (fs >> 1));
  igf_min += igf_min % 2;

  return igf_min;
}

/**
 *  ia_core_coder_igf_decode_whitening_level
 *
 *  \brief Calculate igf whitening level which describe which whitening should be  used
 *
 *  \param [in]    it_bit_buf      bit stream buffer
 *  \param [in]    igf_bitstream    igf bitstream buffer
 *  \param [in]    tile_num      tile index
 *
 *
 *
 */
static VOID ia_core_coder_igf_decode_whitening_level(ia_bit_buf_struct *it_bit_buf,
                                                     ia_usac_igf_bitstream_struct *igf_bitstream,
                                                     WORD32 tile_num)
{
  WORD32 level;

  igf_bitstream->igf_whitening_level[tile_num] = 0;

  level = ia_core_coder_read_bits_buf(it_bit_buf, 1);

  if (level == 1)
  {
    level = ia_core_coder_read_bits_buf(it_bit_buf, 1);

    if (level == 1)
    {
      igf_bitstream->igf_whitening_level[tile_num] += 1;
    }

    igf_bitstream->igf_whitening_level[tile_num] += 1;
  }
}

/**
 *  ia_core_coder_igf_reset_level
 *
 *  \brief Resets/initialize igf coeffs
 *
 *  \param [in]    ptr_igf_env    igf envelope structure
 *
 *
 *
 */
static VOID ia_core_coder_igf_reset_level(ia_usac_igf_envelope_struct *ptr_igf_env)
{
  ia_core_coder_memset(ptr_igf_env->igf_sn_float, NSFB_LONG);
  ia_core_coder_memset(ptr_igf_env->igf_pn_float, NSFB_LONG);
}

/**
 *  ia_core_coder_igf_reset_data
 *
 *  \brief Resets/initialize igf data
 *
 *  \param [out]    igf_dec_data  igf decoder data structure
 *  \param [in]      igf_frame_id  igf frame id
 *
 *
 *
 */
static VOID ia_core_coder_igf_reset_data(ia_usac_igf_dec_data_struct *igf_dec_data,
                                         WORD32 igf_frame_id)
{
  WORD32 tile;
  WORD32 sf_index = 0;

  if (3 == igf_frame_id)
  {
    sf_index = 1;
  }

  for (tile = 0; tile < MAX_IGF_TILES; tile++)
  {
    igf_dec_data->igf_memory.prev_tile_num[tile] = 3;
    igf_dec_data->igf_bitstream[sf_index].igf_tile_num[tile] = 3;
  }

  memset(igf_dec_data->igf_memory.prev_whitening_level, 0,
         MAX_IGF_TILES * sizeof(igf_dec_data->igf_memory.prev_whitening_level[0]));
  memset(igf_dec_data->igf_bitstream[sf_index].igf_whitening_level, 0,
         MAX_IGF_TILES * sizeof(igf_dec_data->igf_bitstream[sf_index].igf_whitening_level[0]));

  igf_dec_data->igf_bitstream[sf_index].igf_is_tnf = 0;
}

/**
 *  ia_core_coder_igf_reset
 *
 *  \brief Igf reset function which resets data and envelope
 *
 *  \param [in,out]  igf_dec_data Pointer to igf dec data structure
 *
 *
 *
 */
static VOID ia_core_coder_igf_reset(ia_usac_igf_dec_data_struct *igf_dec_data)
{
  if (NULL != igf_dec_data)
  {
    ia_core_coder_igf_reset_level(&igf_dec_data->igf_envelope);
    ia_core_coder_igf_reset_data(igf_dec_data, 2);
    ia_core_coder_igf_reset_data(igf_dec_data, 3);
  }
}

/**
 *  ia_core_coder_igf_get_tile_idx
 *
 *  \brief Calculate igf tile index
 *
 *  \param [in]    tb        offset table for scalefactor  bands
 *  \param [in]    igf_bgn      igf start subband
 *  \param [in]    num_tiles    number of igf tiles
 *  \param [in]    tile      Vector of length 4 containing the tile index
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_get_tile_idx(WORD32 tb, WORD32 igf_bgn, WORD32 num_tiles,
                                             WORD32 *tile)
{
  WORD32 tl = 0;
  WORD32 sum = igf_bgn;

  for (tl = 0; tl < num_tiles; tl++)
  {
    sum += tile[tl];
    if (tb < sum)
    {
      break;
    }
  }
  return tl;
}

/**
 *  ia_core_coder_igf_get_num_tiles
 *
 *  \brief Calculate igf number of tiles and return the value
 *
 *  \param [in]      igf_grid      igf grid configuration structure
 *  \param [in]      use_high_res_flag  flag for high/low resolution
 *  \param [in]      swb_offset      offset  table for
 * scalefactor
 * band
 *  \param [in]      p_tile        Vector of length 4 containing the
 * tile
 * index
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_get_num_tiles(ia_usac_igf_grid_config_struct *igf_grid,
                                              WORD8 use_high_res_flag, WORD16 *swb_offset,
                                              WORD32 *p_tile)
{
  WORD32 i, j;
  WORD32 sfb_start = igf_grid->igf_sfb_start;
  WORD32 sfb_stop = igf_grid->igf_sfb_stop;
  WORD32 igf_bgn = swb_offset[sfb_start];
  WORD32 igf_end = swb_offset[sfb_stop];
  WORD32 mem = sfb_start;
  WORD32 igf_range = ia_max_int(8, igf_end - igf_bgn);
  WORD32 sbs = igf_bgn;
  WORD32 num_tiles = 0;
  WORD32 tile[MAX_IGF_TILES] = {0};

  for (i = 0; i < MAX_IGF_TILES; i++)
  {
    tile[i] = igf_range >> 2;
  }

  if (igf_bgn > igf_grid->igf_min && igf_bgn < igf_end)
  {
    for (i = 0; i < MAX_IGF_TILES; i++)
    {
      j = 1;
      while (ia_min_int(sbs + tile[i], igf_end) > swb_offset[j])
      {
        j++;
      }

      if (!use_high_res_flag)
      {
        if (i == 3)
        {
          j = sfb_stop;
        }
        else if ((((j - sfb_start) & 1) == 1) && ((j - mem) > 1) && (i < 3))
        {
          j--;
        }
        mem = j;
      }

      tile[i] = ia_max_int(2, ia_min_int(swb_offset[j] - sbs, igf_end - sbs));

      num_tiles++;
      sbs += tile[i];
      if (sbs == igf_end)
      {
        break;
      }
    }
  }

  memset(&tile[num_tiles], 0, (MAX_IGF_TILES - num_tiles) * sizeof(tile[0]));

  if (p_tile)
  {
    memcpy(p_tile, tile, sizeof(tile));
  }

  return num_tiles;
}

/**
 *  ia_core_coder_igf_get_sb
 *
 *  \brief Calculate igf target subbands
 *
 *  \param [in]    igf_min    subband  index which determine a  minimal  frequency
 *  \param [in]    igf_bgn    igf start subband
 *  \param [in]    tb      swb offset value
 *  \param [in]    num_tiles  number of tiles
 *  \param [in]    tile    Vector of length 4 containing the tile index
 *  \param [in]    tile_idx    tile index
 *
 *  \return WORD32
 *
 */
static WORD32 ia_core_coder_igf_get_sb(WORD32 igf_min, WORD32 igf_bgn, WORD32 tb,
                                       WORD32 num_tiles, WORD32 *tile, WORD8 *tile_idx)
{
  WORD32 i;
  WORD8 sel_tile;
  WORD32 sbs = igf_bgn;
  WORD32 offset = 0;
  WORD32 sb = tb;
  WORD32 src = igf_bgn - igf_min;

  if (src > 0)
  {
    for (i = 0; i < num_tiles; i++)
    {
      if ((tb >= sbs) && (tb < (sbs + tile[i])))
      {
        sel_tile = tile_idx[i];

        offset = (WORD32)(ia_add_flt(2.5f, ia_mul_flt(-0.5f, (FLOAT32)sel_tile)) * tile[i] + sbs -
                          igf_bgn);
        offset += offset % 2;
        break;
      }
      sbs += tile[i];
    }
    sb = tb - offset;
    if (sb < igf_min)
    {
      sb = (igf_min + tb % src);
    }
  }
  return sb;
}
/**
 *  ia_core_coder_igf_get_random_sign
 *
 *  \brief Calulate igf random sign and returns it
 *
 *  \param [in]    seed    seed value
 *
 *  \return FLOAT32
 *
 */
FLOAT32 ia_core_coder_igf_get_random_sign(UWORD32 *seed)
{
  FLOAT32 sign = +1.0f;
  UWORD64 temp;

  temp = ((*seed) * (UWORD64)69069) + 5;
  *seed = (UWORD32)temp;

  if (((*seed) & 0x10000) > 0)
  {
    sign = -1.0f;
  }
  return sign;
}

/**
 *  ia_core_coder_igf_apply_whitening
 *
 *  \brief Apply igf whitening
 *
 *  \param [in]    igf_config      igf config structure
 *  \param [in]    igf_bit_stream    igf bit stream
 *  \param [in,out]  ptr_igf_input_spec  igf input spectrum
 *  \param [in]    swb_offset      offset  table for scalefactor band
 *  \param [in]    igf_win_type    igf window type
 *  \param [in]    ptr_scratch      scartch pointer used for internal
 * processing
 *
 *
 *
 */
static VOID ia_core_coder_igf_apply_whitening(ia_usac_igf_config_struct *igf_config,
                                              ia_usac_igf_bitstream_struct *igf_bit_stream,
                                              FLOAT32 *ptr_igf_input_spec, short *swb_offset,
                                              WORD32 igf_win_type, FLOAT32 *ptr_scratch)
{
  WORD32 e, ix, i, j, n = 0;
  WORD32 num_tiles = igf_config->igf_grid[igf_win_type].igf_num_tiles;
  WORD32 igf_min = igf_config->igf_grid[igf_win_type].igf_min;
  WORD32 stop = swb_offset[igf_config->igf_grid[igf_win_type].igf_sfb_start];
  FLOAT32 *igf_spec_flat = ptr_scratch;
  FLOAT32 *ptr_mdct_float;

  FLOAT32 env;
  FLOAT32 fac = 0.0f;

  if (1 != igf_win_type)
  {
    for (ix = 0; ix < num_tiles; ix++)
    {
      if (0 == igf_bit_stream->igf_whitening_level[ix])
      {
        ptr_mdct_float = &ptr_igf_input_spec[ix * MAX_IGF_LEN];
        for (i = igf_min; i < stop - 3; i++)
        {
          env = 1e-3f;
          for (j = i - 3; j <= i + 3; j++)
          {
            env = ia_add_flt(env, ia_mul_flt(ptr_mdct_float[j], ptr_mdct_float[j]));
          }
          if (ia_lteq_flt(1, env))
          {
            for (e = 0; e < 64; e++)
            {
              if (ia_lt_flt(env, ia_core_coder_pow_2_table[e]))
              {
                n = e - 1;
                break;
              }
            }
          }
          else
          {
            for (e = 0; e < 64; e++)
            {
              if (ia_lteq_flt(ia_core_coder_pow_2_inv_table[e], env))
              {
                n = e;
                break;
              }
            }
          }
          if (n >= 0)
          {
            if (ia_lteq_flt(1, env))
              fac = (FLOAT32)ia_core_coder_pow_igf_whitening[n];
            else
              fac = (FLOAT32)ia_core_coder_pow_neg_igf_whitening[n];
            igf_spec_flat[i] = ia_mul_flt(ptr_mdct_float[i], fac);
          }
          else
          {
            igf_spec_flat[i] = 0;
          }
        }

        for (; i < stop; i++)
        {
          igf_spec_flat[i] = ia_mul_flt(ptr_mdct_float[i], fac);
        }

        ia_core_coder_mem_cpy(&igf_spec_flat[igf_min], &ptr_mdct_float[igf_min], stop - igf_min);
      }
    }
  }
}

/**
 *  ia_core_coder_igf_norm
 *
 *  \brief Normalizes pn and sn vectors
 *
 *  \param [in]    igf_config    igf config structures
 *  \param [in,out]  igf_envelope  igf envelope structure
 *  \param [in]    igf_win_type  igf window type
 *  \param [in]    group_len    group length
 *
 *
 *
 */
static VOID ia_core_coder_igf_norm(ia_usac_igf_config_struct *igf_config,
                                   ia_usac_igf_envelope_struct *igf_envelope, WORD32 igf_win_type,
                                   WORD32 group_len)
{
  WORD32 sfb;
  WORD32 igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;
  WORD32 igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;

  for (sfb = igf_stop_sfb - 1; sfb >= igf_start_sfb; sfb--)
  {
    igf_envelope->igf_sn_float[sfb] /= group_len;
    igf_envelope->igf_pn_float[sfb] /= group_len;
  }
}

/**
 *  ia_core_coder_igf_rescale
 *
 *  \brief Rescale usac spectral coeff based on igf levels
 *
 *  \param [in]    igf_config        igf config structure
 *  \param [in]    igf_win_type      igf window type
 *  \param [in]    ptr_coef        spectral coeff
 *  \param [in]    swb_offset        offset  table for scalefactor band
 *  \param [in]    igf_levels_curr_float  IGF  levels for the current window
 *
 *
 *
 */
static VOID ia_core_coder_igf_rescale(ia_usac_igf_config_struct *igf_config, WORD32 igf_win_type,
                                      FLOAT32 *ptr_coef, WORD16 *swb_offset,
                                      FLOAT32 *igf_levels_curr_float)
{
  WORD32 sfb, bin, width, sfb_high;
  WORD32 sfb_step = 2;
  WORD32 igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;
  WORD32 igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;

  FLOAT32 *ptr_coef_loc;

  if (igf_config->use_high_res || igf_win_type >= 1)
  {
    sfb_step = 1;
  }

  if (igf_win_type > 1)
  {
    for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += sfb_step)
    {
      sfb_high = ia_min_int(sfb + sfb_step, igf_stop_sfb);
      width = (swb_offset[sfb_high] - swb_offset[sfb]);

      ptr_coef_loc = &ptr_coef[swb_offset[sfb]];

      for (bin = 0; bin < width; bin++)
      {
        ptr_coef_loc[bin] = ia_mul_flt(ptr_coef_loc[bin], igf_levels_curr_float[sfb]);
      }
    }
  }
}

/**
 *  ia_core_coder_igf_calc_mono
 *
 *  \brief Calculate igf envelope parameters for mono
 *
 *  \param [in]      igf_config        igf config structure
 *  \param [in,out]    igf_dec_data      igf decoder structure
 *  \param [in]      igf_bit_stream      igf bitstream buffer
 *  \param [in]      ptr_coef        coeff pointer
 *  \param [in]      ptr_igf_input_spec    igf spectrum coeff
 *  \param [in]      swb_offset        offset  table for
 * scalefactor
 * band
 *  \param [in]      seed_value        seed value
 *  \param [in]      igf_win_type      igf window type
 *
 *
 *
 */
static VOID ia_core_coder_igf_calc_mono(ia_usac_igf_config_struct *igf_config,
                                        ia_usac_igf_dec_data_struct *igf_dec_data,
                                        ia_usac_igf_bitstream_struct *igf_bit_stream,
                                        FLOAT32 *ptr_coef, FLOAT32 *ptr_igf_input_spec,
                                        WORD16 *swb_offset, UWORD32 *seed_value,
                                        WORD32 igf_win_type)
{
  WORD32 sfb, bin, width, igf_sb, sfb_high;
  WORD32 sfb_step = 2;
  WORD32 tile[MAX_IGF_TILES] = {0};
  WORD32 num_tiles;
  FLOAT32 tot_energy = 0, value;

  ia_usac_igf_envelope_struct *igf_envelope = &igf_dec_data->igf_envelope;
  WORD32 igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;
  WORD32 igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;
  WORD32 igf_min = igf_config->igf_grid[igf_win_type].igf_min;
  WORD32 igf_bgn = swb_offset[igf_start_sfb];

  if (igf_config->use_high_res || igf_win_type >= 1)
  {
    sfb_step = 1;
  }

  for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += sfb_step)
  {
    sfb_high = ia_min_int(sfb + sfb_step, igf_stop_sfb);
    width = (swb_offset[sfb_high] - swb_offset[sfb]);

    tot_energy = 0;
    for (bin = 0; bin < width; bin++)
    {
      igf_sb = swb_offset[sfb] + bin;
      tot_energy = ia_mac_flt(tot_energy, ptr_coef[igf_sb], ptr_coef[igf_sb]);
    }

    igf_envelope->igf_sn_float[sfb] = ia_add_flt(igf_envelope->igf_sn_float[sfb], tot_energy);

    tot_energy = 0;
    num_tiles = ia_core_coder_igf_get_num_tiles(&igf_config->igf_grid[igf_win_type],
                                                igf_config->use_high_res, swb_offset, tile);

    for (bin = 0; bin < width; bin++)
    {
      if (ptr_coef[swb_offset[sfb] + bin] == 0)
      {
        WORD32 ix = ia_core_coder_igf_get_tile_idx(swb_offset[sfb] + bin,
                                                   swb_offset[igf_start_sfb], num_tiles, tile);
        igf_sb = ia_core_coder_igf_get_sb(igf_min, igf_bgn, swb_offset[sfb] + bin, num_tiles,
                                          tile, igf_bit_stream->igf_tile_num);
        value = ptr_igf_input_spec[ix * MAX_IGF_LEN + igf_sb];

        if ((igf_win_type != 1) && igf_config->use_whitening &&
            (2 == igf_bit_stream->igf_whitening_level[ix]))
        {
          value = ia_core_coder_igf_get_random_sign(seed_value);
          value = ia_mul_flt(value, 2097152.0f);
        }
        tot_energy = ia_add_flt(tot_energy, ia_mul_flt(value, value));
      }
    }
    igf_envelope->igf_pn_float[sfb] = ia_add_flt(igf_envelope->igf_pn_float[sfb], tot_energy);
  }
}

/**
 *  ia_core_coder_igf_calc_stereo
 *
 *  \brief calculate igf stereo
 *
 *  \param [in]        igf_config    igf config structure
 *  \param [in,out]      usac_data    usac data structure
 *  \param [in]        chn        channel under process
 *  \param [in]        group_len    group length
 *  \param [out]      win_offset    window offset
 *  \param [in]        bins_per_sbk  bins per subblock
 *  \param [in]        sf_index    scale factor index
 *  \param [in]        igf_win_type  igf window type
 *  \param [in]        mask      buffer containing igf mask values
 *  \param [in]        swb_offset    offset  table for scalefactor band
 *  \param [in]        seed_value_l  left seed value
 *  \param [in]        seed_value_r  right seed value
 *
 *
 *
 */
static VOID ia_core_coder_igf_calc_stereo(const ia_usac_igf_config_struct *igf_config,
                                          ia_usac_data_struct *usac_data, WORD32 chn,
                                          WORD32 group_len, WORD32 win_offset,
                                          WORD32 bins_per_sbk, WORD32 sf_index,
                                          const WORD32 igf_win_type, const WORD32 *mask,
                                          WORD16 *swb_offset, UWORD32 *seed_value_l,
                                          UWORD32 *seed_value_r)
{
  WORD32 i;
  for (i = 0; i < group_len; i++)
  {
    WORD32 sfb;
    WORD32 bin;
    WORD32 width;
    WORD32 igf_sb;
    WORD32 igf_sb_l;
    WORD32 igf_sb_r;
    FLOAT32 val_l;
    FLOAT32 val_r;
    FLOAT32 energy_l;
    FLOAT32 energy_r;
    WORD32 sfb_step = 2;
    WORD32 sfb_high;
    WORD32 tile_l[4] = {0};
    WORD32 tile_r[4] = {0};
    WORD32 num_tiles_l;
    WORD32 num_tiles_r;
    WORD32 igf_start_sfb = (WORD8)(igf_config->igf_grid[igf_win_type].igf_sfb_start);
    WORD32 igf_stop_sfb = (WORD8)(igf_config->igf_grid[igf_win_type].igf_sfb_stop);
    WORD32 igf_min = (WORD8)(igf_config->igf_grid[igf_win_type].igf_min);
    WORD32 igf_bgn = (WORD32)(swb_offset[igf_start_sfb]);

    ia_usac_igf_dec_data_struct *igf_dec_data_right = &usac_data->igf_dec_data[chn + 1];
    ia_usac_igf_dec_data_struct *igf_dec_data_left = &usac_data->igf_dec_data[chn];

    ia_usac_igf_envelope_struct *igf_env_right = &igf_dec_data_right->igf_envelope;
    ia_usac_igf_envelope_struct *igf_env_left = &igf_dec_data_left->igf_envelope;

    const ia_usac_igf_bitstream_struct *igf_bs_right =
        &igf_dec_data_right->igf_bitstream[sf_index];
    const ia_usac_igf_bitstream_struct *igf_bs_left = &igf_dec_data_left->igf_bitstream[sf_index];

    const FLOAT32 *ptr_coef_right = &usac_data->coef[chn + 1][win_offset * bins_per_sbk];
    const FLOAT32 *ptr_coef_left = &usac_data->coef[chn][win_offset * bins_per_sbk];

    const FLOAT32 *ptr_igf_inp_spec_right =
        &igf_dec_data_right->igf_input_spec[win_offset * bins_per_sbk];
    const FLOAT32 *ptr_igf_inp_spec_left =
        &igf_dec_data_left->igf_input_spec[win_offset * bins_per_sbk];

    if (igf_config->use_high_res || igf_win_type >= 1)
    {
      sfb_step = 1;
    }

    for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += sfb_step)
    {
      sfb_high = ia_min_int(sfb + sfb_step, igf_stop_sfb);
      width = swb_offset[sfb_high] - swb_offset[sfb];
      energy_l = energy_r = 0.0f;

      for (bin = 0; bin < width; bin++)
      {
        igf_sb = swb_offset[sfb] + bin;
        val_r = ptr_coef_right[igf_sb];
        energy_r = ia_add_flt(energy_r, ia_mul_flt(val_r, val_r));
        val_l = ptr_coef_left[igf_sb];
        energy_l = ia_add_flt(energy_l, ia_mul_flt(val_l, val_l));
      }

      igf_env_right->igf_sn_float[sfb] = ia_add_flt(igf_env_right->igf_sn_float[sfb], energy_r);
      igf_env_left->igf_sn_float[sfb] = ia_add_flt(igf_env_left->igf_sn_float[sfb], energy_l);

      energy_l = energy_r = 0.0f;
      num_tiles_r = ia_core_coder_igf_get_num_tiles(
          (ia_usac_igf_grid_config_struct *)&igf_config->igf_grid[igf_win_type],
          igf_config->use_high_res, swb_offset, tile_r);
      num_tiles_l = ia_core_coder_igf_get_num_tiles(
          (ia_usac_igf_grid_config_struct *)&igf_config->igf_grid[igf_win_type],
          igf_config->use_high_res, swb_offset, tile_l);

      for (bin = 0; bin < width; bin++)
      {
        WORD32 ix_r = ia_core_coder_igf_get_tile_idx(
            swb_offset[sfb] + bin, swb_offset[igf_start_sfb], num_tiles_r, tile_r);
        WORD32 ix_l = ia_core_coder_igf_get_tile_idx(
            swb_offset[sfb] + bin, swb_offset[igf_start_sfb], num_tiles_l, tile_l);

        igf_sb_r = ia_core_coder_igf_get_sb(igf_min, igf_bgn, swb_offset[sfb] + bin, num_tiles_r,
                                            tile_r, (WORD8 *)igf_bs_right->igf_tile_num);
        igf_sb_l =
            ia_core_coder_igf_get_sb(igf_min, igf_bgn, (WORD32)(swb_offset[sfb] + bin),
                                     num_tiles_l, tile_l, (WORD8 *)igf_bs_left->igf_tile_num);

        val_r = ptr_igf_inp_spec_right[ix_r * MAX_IGF_LEN + igf_sb_r];
        val_l = ptr_igf_inp_spec_left[ix_l * MAX_IGF_LEN + igf_sb_l];

        if ((igf_win_type != 1) && igf_config->use_whitening)
        {
          if (2 == igf_bs_right->igf_whitening_level[ix_r])
          {
            val_r =
                ia_mul_flt(2097152.f, (FLOAT32)ia_core_coder_igf_get_random_sign(seed_value_r));
          }
          if (2 == igf_bs_left->igf_whitening_level[ix_l])
          {
            val_l =
                ia_mul_flt(2097152.f, (FLOAT32)ia_core_coder_igf_get_random_sign(seed_value_l));
          }
        }

        if (mask[sfb])
        {
          FLOAT32 tmp = val_l;
          val_l = ia_mul_flt(0.5f, ia_add_flt(tmp, val_r));
          val_r = ia_mul_flt(0.5f, ia_sub_flt(tmp, val_r));
        }
        if (ia_eq_flt(ptr_coef_right[swb_offset[sfb] + bin], 0.0f))
        {
          energy_r = ia_add_flt(energy_r, ia_mul_flt(val_r, val_r));
        }
        if (ia_eq_flt(ptr_coef_left[swb_offset[sfb] + bin], 0.0f))
        {
          energy_l = ia_add_flt(energy_l, ia_mul_flt(val_l, val_l));
        }
      }
      igf_env_right->igf_pn_float[sfb] = ia_add_flt(igf_env_right->igf_pn_float[sfb], energy_r);
      igf_env_left->igf_pn_float[sfb] = ia_add_flt(igf_env_left->igf_pn_float[sfb], energy_l);
    }
    win_offset++;
  }
}

/**
 *  ia_core_coder_igf_apply_mono
 *
 *  \brief igf mono processing
 *
 *  \param [in,out]  igf_dec_data      igf decoder data
 *  \param [in]    igf_config        igf config structure
 *  \param [in]    igf_envelop        igf envelope structure
 *  \param [in]    igf_bit_stream      igf bitstream
 *  \param [in,out]  ptr_coef        USAC coeff
 *  \param [in]    ptr_igf_input_spec    igf coeff
 *  \param [in]    swb_offset        offset  table for scalefactor band
 *  \param [in]    seed_value        seed value
 *  \param [in]    igf_win_type      igf window type
 *  \param [in]    grp            group
 *
 *
 *
 */
static VOID ia_core_coder_igf_apply_mono(ia_usac_igf_dec_data_struct *igf_dec_data,
                                         ia_usac_igf_config_struct *igf_config,
                                         ia_usac_igf_envelope_struct *igf_envelop,
                                         ia_usac_igf_bitstream_struct *igf_bit_stream,
                                         FLOAT32 *ptr_coef, FLOAT32 *ptr_igf_input_spec,
                                         WORD16 *swb_offset, UWORD32 *seed_value,
                                         WORD32 igf_win_type, WORD32 grp)
{
  WORD32 ix, sfb, bin, width, igf_sb, sfb_high;
  WORD32 sfb_step = 2;
  WORD32 tile[4] = {0};
  WORD32 num_tiles;
  FLOAT32 ergy_curr, mn, ergy_sn, ergy_pn, value, gn, coef_bin;
  WORD32 igf_sfb_start = igf_config->igf_grid[igf_win_type].igf_sfb_start;
  WORD32 igf_sfb_stop = igf_config->igf_grid[igf_win_type].igf_sfb_stop;
  WORD32 igf_min = igf_config->igf_grid[igf_win_type].igf_min;
  WORD32 igf_bgn = swb_offset[igf_sfb_start];

  if (igf_config->use_tnf && 1 != igf_win_type)
  {
    ia_core_coder_memset(igf_dec_data->igf_tnf_spec_float, MAX_IGF_LEN);
    memset(igf_dec_data->igf_tnf_mask, 1, MAX_IGF_LEN);
  }
  ia_core_coder_igf_rescale(igf_config, igf_win_type, ptr_coef, swb_offset,
                            igf_bit_stream->igf_levels_curr_float[grp]);

  if (igf_config->use_high_res || igf_win_type >= 1)
  {
    sfb_step = 1;
  }

  for (sfb = igf_sfb_start; sfb < igf_sfb_stop; sfb += sfb_step)
  {
    sfb_high = ia_min_int(sfb + sfb_step, igf_sfb_stop);
    width = (swb_offset[sfb_high] - swb_offset[sfb]);

    ergy_curr = igf_bit_stream->igf_levels_curr_float[grp][sfb];
    ergy_pn = igf_envelop->igf_pn_float[sfb];
    ergy_sn = igf_envelop->igf_sn_float[sfb];
    mn = ia_sub_flt(ia_mul_flt(ia_mul_flt(ergy_curr, ergy_curr), (FLOAT32)width), ergy_sn);

    if (ia_lt_flt(0, mn) && ia_lt_flt(0, ergy_pn))
    {
      gn = (FLOAT32)ia_min_flt((FLOAT32)ia_sqrt_flt((FLOAT64)mn / ergy_pn), 10.f);
    }
    else
    {
      gn = 0;
    }
    num_tiles = ia_core_coder_igf_get_num_tiles(&igf_config->igf_grid[igf_win_type],
                                                igf_config->use_high_res, swb_offset, tile);

    for (bin = 0; bin < width; bin++)
    {
      ix = ia_core_coder_igf_get_tile_idx(swb_offset[sfb] + bin, swb_offset[igf_sfb_start],
                                          num_tiles, tile);
      igf_sb = ia_core_coder_igf_get_sb(igf_min, igf_bgn, swb_offset[sfb] + bin, num_tiles, tile,
                                        igf_bit_stream->igf_tile_num);
      value = ptr_igf_input_spec[ix * MAX_IGF_LEN + igf_sb];

      coef_bin = ptr_coef[swb_offset[sfb] + bin];

      if (coef_bin == 0)
      {
        if ((1 != igf_win_type) && igf_config->use_whitening &&
            (2 == igf_bit_stream->igf_whitening_level[ix]))
        {
          value = ia_mul_flt(ia_core_coder_igf_get_random_sign(seed_value), 2097152.0f);
        }
      }

      if (igf_config->use_tnf && (1 != igf_win_type))
      {
        igf_dec_data->igf_tnf_spec_float[swb_offset[sfb] + bin] = ia_mul_flt(gn, value);

        if (coef_bin == 0)
        {
          igf_dec_data->igf_tnf_mask[swb_offset[sfb] + bin] = 0;
        }
      }

      if (coef_bin == 0)
      {
        ptr_coef[swb_offset[sfb] + bin] = ia_mul_flt(gn, value);
      }
    }
  }
}
/**
 *  ia_core_coder_igf_stereo_proc
 *
 *  \brief igf stereo processing in igf apply stereo
 *
 *  \param [in]      igf_config    igf config structure
 *  \param [in]      usac_data    usac data structure
 *  \param [in]      chn        channel under process
 *  \param [in]      group_len    group length
 *  \param [in,out]  win_offset    window offset buffer
 *  \param [in]      bins_per_sbk  bins in subblock
 *  \param [in]      sf_index    scalefactor index
 *  \param [in]      igf_win_type  igf window type
 *  \param [in]      mask      buffer containing igf mask values
 *  \param [in]      swb_offset    offset  table for  scalefactor band
 *  \param [in]      seed_value_l  left seed value
 *  \param [in]      seed_value_r  right seed value
 *  \param [in]      grp        group
 *
 *
 *
 */
static VOID ia_core_coder_igf_stereo_proc(ia_usac_igf_config_struct *igf_config,
                                          ia_usac_data_struct *usac_data, WORD32 chn,
                                          WORD32 group_len, WORD32 *win_offset,
                                          WORD32 bins_per_sbk, WORD32 sf_index,
                                          WORD32 igf_win_type, const WORD32 *mask,
                                          WORD16 *swb_offset, UWORD32 *seed_value_l,
                                          UWORD32 *seed_value_r, WORD32 grp)
{
  WORD32 i;
  for (i = 0; i < group_len; i++)
  {
    WORD32 sfb;
    WORD32 bin;
    WORD32 num_tiles_left, num_tiles_right;
    WORD32 width, sfb_high;
    WORD32 igf_sb_l, igf_sb_r;
    WORD32 sfb_step = 2;

    FLOAT32 de_l, de_r;
    FLOAT32 mn_l, mn_r;
    FLOAT32 sn_l, sn_r;
    FLOAT32 pn_l, pn_r;
    FLOAT32 gn_l, gn_r;
    FLOAT32 val_l, val_r;
    FLOAT32 coef_bin_l, coef_bin_r;

    WORD32 tile_l[4] = {0};
    WORD32 tile_r[4] = {0};

    const WORD32 igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;
    const WORD32 igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;
    const WORD32 igf_min = igf_config->igf_grid[igf_win_type].igf_min;
    const WORD32 igf_bgn = swb_offset[igf_start_sfb];

    ia_usac_igf_dec_data_struct *igf_dec_data_right = &usac_data->igf_dec_data[chn + 1];
    ia_usac_igf_dec_data_struct *igf_dec_data_left = &usac_data->igf_dec_data[chn];

    ia_usac_igf_envelope_struct *igf_env_right = &igf_dec_data_right->igf_envelope;
    ia_usac_igf_envelope_struct *igf_env_left = &igf_dec_data_left->igf_envelope;

    ia_usac_igf_bitstream_struct *igf_bs_right = &igf_dec_data_right->igf_bitstream[sf_index];
    ia_usac_igf_bitstream_struct *igf_bs_left = &igf_dec_data_left->igf_bitstream[sf_index];

    FLOAT32 *ptr_coef_right = &usac_data->coef[chn + 1][*win_offset * bins_per_sbk];
    FLOAT32 *ptr_coef_left = &usac_data->coef[chn][*win_offset * bins_per_sbk];

    FLOAT32 *ptr_igf_inp_spec_right =
        &igf_dec_data_right->igf_input_spec[*win_offset * bins_per_sbk];
    FLOAT32 *ptr_igf_inp_spec_left =
        &igf_dec_data_left->igf_input_spec[*win_offset * bins_per_sbk];

    if (igf_config->use_tnf && 1 != igf_win_type)
    {
      for (bin = 0; bin < MAX_IGF_LEN; bin++)
      {
        igf_dec_data_left->igf_tnf_spec_float[bin] = 0.0f;
        igf_dec_data_right->igf_tnf_spec_float[bin] = 0.0f;
        igf_dec_data_left->igf_tnf_mask[bin] = 1;
        igf_dec_data_right->igf_tnf_mask[bin] = 1;
      }
    }

    ia_core_coder_igf_rescale(igf_config, igf_win_type, ptr_coef_left, (WORD16 *)swb_offset,
                              igf_bs_left->igf_levels_curr_float[grp]);

    ia_core_coder_igf_rescale(igf_config, igf_win_type, ptr_coef_right, (WORD16 *)swb_offset,
                              igf_bs_right->igf_levels_curr_float[grp]);

    if (igf_config->use_high_res || igf_win_type >= 1)
    {
      sfb_step = 1;
    }

    for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += sfb_step)
    {
      sfb_high = ia_min_int(sfb + sfb_step, igf_stop_sfb);
      width = (swb_offset[sfb_high] - swb_offset[sfb]);

      de_r = igf_bs_right->igf_levels_curr_float[grp][sfb];
      sn_r = igf_env_right->igf_sn_float[sfb];
      pn_r = igf_env_right->igf_pn_float[sfb];
      mn_r = ia_sub_flt(ia_mul_flt(ia_mul_flt(de_r, de_r), (FLOAT32)width), sn_r);
      de_l = igf_bs_left->igf_levels_curr_float[grp][sfb];
      sn_l = igf_env_left->igf_sn_float[sfb];
      pn_l = igf_env_left->igf_pn_float[sfb];
      mn_l = ia_sub_flt(ia_mul_flt(ia_mul_flt(de_l, de_l), (FLOAT32)width), sn_l);

      gn_r = gn_l = 0.0f;
      if (ia_lt_flt(0.0f, mn_r) && ia_lt_flt(0.0f, pn_r))
      {
        gn_r = ia_min_flt(10.f, (FLOAT32)ia_sqrt_flt(mn_r / pn_r));
      }
      if (ia_lt_flt(0.0f, mn_l) && ia_lt_flt(0.0f, pn_l))
      {
        gn_l = ia_min_flt(10.f, (FLOAT32)ia_sqrt_flt(mn_l / pn_l));
      }

      num_tiles_left = ia_core_coder_igf_get_num_tiles(
          &igf_config->igf_grid[igf_win_type], igf_config->use_high_res, swb_offset, tile_l);
      num_tiles_right = ia_core_coder_igf_get_num_tiles(
          &igf_config->igf_grid[igf_win_type], igf_config->use_high_res, swb_offset, tile_r);

      for (bin = 0; bin < width; bin++)
      {
        WORD32 ix_r = ia_core_coder_igf_get_tile_idx(
            swb_offset[sfb] + bin, swb_offset[igf_start_sfb], num_tiles_right, tile_r);
        WORD32 ix_l = ia_core_coder_igf_get_tile_idx(
            swb_offset[sfb] + bin, swb_offset[igf_start_sfb], num_tiles_left, tile_l);
        igf_sb_r = ia_core_coder_igf_get_sb(igf_min, igf_bgn, swb_offset[sfb] + bin,
                                            num_tiles_right, tile_r, igf_bs_right->igf_tile_num);
        igf_sb_l = ia_core_coder_igf_get_sb(igf_min, igf_bgn, swb_offset[sfb] + bin,
                                            num_tiles_left, tile_l, igf_bs_left->igf_tile_num);

        val_r = ptr_igf_inp_spec_right[ix_r * MAX_IGF_LEN + igf_sb_r];
        val_l = ptr_igf_inp_spec_left[ix_l * MAX_IGF_LEN + igf_sb_l];

        if (igf_config->use_whitening && (1 != igf_win_type))
        {
          if (2 == igf_bs_right->igf_whitening_level[ix_r])
          {
            val_r = ia_mul_flt(2097152.f, ia_core_coder_igf_get_random_sign(seed_value_r));
          }
          if (2 == igf_bs_left->igf_whitening_level[ix_l])
          {
            val_l = ia_mul_flt(2097152.f, ia_core_coder_igf_get_random_sign(seed_value_l));
          }
        }
        if (mask[sfb])
        {
          FLOAT32 tmp = val_l;
          val_l = ia_mul_flt(0.5f, ia_add_flt(tmp, val_r));
          val_r = ia_mul_flt(0.5f, ia_sub_flt(tmp, val_r));
        }

        coef_bin_r = ptr_coef_right[swb_offset[sfb] + bin];
        coef_bin_l = ptr_coef_left[swb_offset[sfb] + bin];
        if (igf_config->use_tnf && 1 != igf_win_type)
        {
          igf_dec_data_right->igf_tnf_spec_float[swb_offset[sfb] + bin] = ia_mul_flt(gn_r, val_r);
          igf_dec_data_left->igf_tnf_spec_float[swb_offset[sfb] + bin] = ia_mul_flt(gn_l, val_l);

          if (ia_eq_flt(coef_bin_r, 0.0f))
          {
            igf_dec_data_right->igf_tnf_mask[swb_offset[sfb] + bin] = 0;
          }
          if (ia_eq_flt(coef_bin_l, 0.0f))
          {
            igf_dec_data_left->igf_tnf_mask[swb_offset[sfb] + bin] = 0;
          }

          if (mask[sfb])
          {
            igf_dec_data_right->igf_tnf_mask[swb_offset[sfb] + bin] =
                (WORD32)(igf_dec_data_right->igf_tnf_mask[swb_offset[sfb] + bin] ||
                         igf_dec_data_left->igf_tnf_mask[swb_offset[sfb] + bin]);
            igf_dec_data_left->igf_tnf_mask[swb_offset[sfb] + bin] =
                igf_dec_data_right->igf_tnf_mask[swb_offset[sfb] + bin];
          }
        }
        if (ia_eq_flt(coef_bin_r, 0.0f))
        {
          ptr_coef_right[swb_offset[sfb] + bin] = ia_mul_flt(gn_r, val_r);
        }
        if (ia_eq_flt(coef_bin_l, 0.0f))
        {
          ptr_coef_left[swb_offset[sfb] + bin] = ia_mul_flt(gn_l, val_l);
        }
      }
    }
    *win_offset = *win_offset + 1;
  }
}

/**
 *  ia_core_coder_sync_data
 *
 *  \brief Sync right and left data
 *
 *  \param [in,out]    igf_bitstream_l      left igf bitstream structure
 *  \param [in,out]    igf_bitstream_r      right igf bitstream structure
 *
 *
 *
 */
static VOID ia_core_coder_sync_data(ia_usac_igf_bitstream_struct *igf_bitstream_l,
                                    ia_usac_igf_bitstream_struct *igf_bitstream_r)
{
  WORD32 allzero_left;
  WORD32 allzero_right;

  allzero_left = igf_bitstream_l->igf_all_zero;
  allzero_right = igf_bitstream_r->igf_all_zero;

  if (1 == allzero_left && 0 == allzero_right)
  {
    memcpy(igf_bitstream_l->igf_tile_num, igf_bitstream_r->igf_tile_num,
           4 * sizeof(igf_bitstream_l->igf_tile_num[0]));
    memcpy(igf_bitstream_l->igf_whitening_level, igf_bitstream_r->igf_whitening_level,
           4 * sizeof(igf_bitstream_l->igf_whitening_level[0]));
    igf_bitstream_l->igf_is_tnf = igf_bitstream_r->igf_is_tnf;
  }
  else if (0 == allzero_left && 1 == allzero_right)
  {
    memcpy(igf_bitstream_r->igf_tile_num, igf_bitstream_l->igf_tile_num,
           4 * sizeof(igf_bitstream_r->igf_tile_num[0]));
    memcpy(igf_bitstream_r->igf_whitening_level, igf_bitstream_l->igf_whitening_level,
           4 * sizeof(igf_bitstream_r->igf_whitening_level[0]));
    igf_bitstream_r->igf_is_tnf = igf_bitstream_l->igf_is_tnf;
  }
}

/**
 *  ia_core_coder_igf_apply_stereo
 *
 *  \brief Apply igf stereo
 *
 *  \param [in,out]    usac_data      USAC data structure
 *  \param [in]      chn          Channel under process
 *  \param [in]      igf_config      igf config structure
 *  \param [in]      igf_win_type    igf window type
 *  \param [in]      mask        ms used flag
 *  \param [in]      hasmask        igf mask present
 *  \param [in]      num_groups      num of groups
 *  \param [in]      group_len      group length
 *  \param [in]      bins_per_sbk    bins per subblock
 *  \param [in]      sfb_per_sbk      sfb per sub block
 *
 *
 *
 */
VOID ia_core_coder_igf_apply_stereo(ia_usac_data_struct *usac_data, WORD32 chn,
                                    ia_usac_igf_config_struct *igf_config,
                                    const WORD32 igf_win_type, const WORD8 *mask,
                                    const WORD32 hasmask, const WORD32 num_groups,
                                    const WORD16 *group_len, const WORD32 bins_per_sbk,
                                    const WORD32 sfb_per_sbk)
{
  WORD32 grp;
  WORD32 i;
  WORD32 start_win;
  WORD32 allzero_left;
  WORD32 allzero_right;
  WORD32 win_offset = 0;
  WORD16 swb_offset[64] = {0};
  WORD32 st_mask[SFB_NUM_MAX] = {0};
  const WORD32 sf_index = (3 == igf_win_type) ? 1 : 0;

  ia_usac_igf_dec_data_struct *igf_dec_data_left = &usac_data->igf_dec_data[chn];
  ia_usac_igf_dec_data_struct *igf_dec_data_right = &usac_data->igf_dec_data[chn + 1];
  WORD32 win = usac_data->window_sequence[chn];

  UWORD32 seed_l = usac_data->seed_value[chn];
  UWORD32 seed_r = usac_data->seed_value[chn + 1];
  UWORD32 seed_l_1 = usac_data->seed_value[chn];
  UWORD32 seed_r_1 = usac_data->seed_value[chn + 1];

  allzero_left = igf_dec_data_left->igf_bitstream[sf_index].igf_all_zero;
  allzero_right = igf_dec_data_right->igf_bitstream[sf_index].igf_all_zero;

  if ((0 == allzero_left) || (0 == allzero_right))
  {
    ia_core_coder_igf_get_swb_offset(igf_win_type, swb_offset, usac_data->pstr_usac_winmap[win]);

    ia_core_coder_sync_data(&igf_dec_data_left->igf_bitstream[sf_index],
                            &igf_dec_data_right->igf_bitstream[sf_index]);

    if (igf_config->use_whitening && (1 != igf_win_type))
    {
      ia_core_coder_igf_apply_whitening(igf_config, &igf_dec_data_left->igf_bitstream[sf_index],
                                        igf_dec_data_left->igf_input_spec, swb_offset,
                                        igf_win_type, usac_data->ptr_fft_scratch);

      ia_core_coder_igf_apply_whitening(igf_config, &igf_dec_data_right->igf_bitstream[sf_index],
                                        igf_dec_data_right->igf_input_spec, swb_offset,
                                        igf_win_type, usac_data->ptr_fft_scratch);
    }

    for (grp = 0; grp < num_groups; grp++)
    {
      start_win = win_offset;
      ia_core_coder_igf_reset_level(&igf_dec_data_right->igf_envelope);
      ia_core_coder_igf_reset_level(&igf_dec_data_left->igf_envelope);

      memset(st_mask, 0, sfb_per_sbk * sizeof(st_mask[0]));

      if (hasmask == 3)
      {
        for (i = 0; i < sfb_per_sbk; i++)
        {
          st_mask[i] = usac_data->cplx_pred_used[chn][grp][i];
        }
      }
      else if (hasmask > 0)
      {
        for (i = 0; i < sfb_per_sbk; i++)
        {
          st_mask[i] = mask[grp * sfb_per_sbk + i];
        }
      }

      ia_core_coder_igf_calc_stereo(igf_config, usac_data, chn, group_len[grp], win_offset,
                                    bins_per_sbk, sf_index, igf_win_type, st_mask, swb_offset,
                                    &seed_l, &seed_r);
      win_offset += group_len[grp];
      if (group_len[grp] != 1)
      {
        ia_core_coder_igf_norm(igf_config, &igf_dec_data_left->igf_envelope, igf_win_type,
                               group_len[grp]);

        ia_core_coder_igf_norm(igf_config, &igf_dec_data_right->igf_envelope, igf_win_type,
                               group_len[grp]);
      }
      win_offset = start_win;

      ia_core_coder_igf_stereo_proc(igf_config, usac_data, chn, group_len[grp], &win_offset,
                                    bins_per_sbk, sf_index, igf_win_type, st_mask, swb_offset,
                                    &seed_l_1, &seed_r_1, grp);
    }

    usac_data->seed_value[chn + 1] = seed_r;
    usac_data->seed_value[chn] = seed_l;
  }
}

/**
*  ia_core_coder_igf_stereo_pred_data
*
*  \brief Update complex prediction related config structure based on bit stream
*
*  \param [in,out]    usac_data      USAC data structure
*  \param [in]      num_window_groups  num of window groups
*  \param [in]      igf_win_type    igf window type
*  \param [in]      it_bit_buff      bit stream buffer
*  \param [in]      chn          channel under process
*  \param [in]      elem_idx      element index in total num ber of
* elements
*
*
*
*/
VOID ia_core_coder_igf_stereo_pred_data(ia_usac_data_struct *usac_data, WORD32 num_window_groups,
                                        WORD32 igf_win_type, ia_bit_buf_struct *it_bit_buff,
                                        WORD32 chn, WORD32 elem_idx)
{
  ia_huff_code_book_struct *ptr_huff_code_book = &ia_core_coder_book;
  const ia_huff_code_word_struct *ptr_huff_code_word = ptr_huff_code_book->pstr_huff_code_word;
  WORD32 igf_stereo_pred_all;
  WORD32 delta_code_time;
  WORD32 grp, sfb;
  WORD32 dpcm_alpha, last_alpha_q_re;

  WORD32(*alpha_q_re)[SFB_NUM_MAX] = usac_data->alpha_q_re[chn];
  WORD32(*alpha_q_im)[SFB_NUM_MAX] = usac_data->alpha_q_im[chn];
  WORD32 *alpha_q_re_prev = usac_data->alpha_q_re_prev[chn];
  WORD32 *alpha_q_im_prev = usac_data->alpha_q_im_prev[chn];
  UWORD8(*cplx_pred_used)[SFB_NUM_MAX] = usac_data->cplx_pred_used[chn];

  ia_usac_igf_config_struct *ptr_igf_config = &usac_data->igf_config[elem_idx];
  WORD32 igf_start_sfb = ptr_igf_config->igf_grid[igf_win_type].igf_sfb_start;
  WORD32 igf_stop_sfb = ptr_igf_config->igf_grid[igf_win_type].igf_sfb_stop;
  WORD32 *igf_pred_dir = &usac_data->igf_pred_dir[elem_idx];

  igf_stereo_pred_all = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (igf_stereo_pred_all != 0)
  {
    for (grp = 0; grp < num_window_groups; grp++)
    {
      memset(&cplx_pred_used[grp][igf_start_sfb], 1,
             (igf_stop_sfb - igf_start_sfb) * (sizeof(cplx_pred_used[grp][0])));
    }

    memset(&cplx_pred_used[grp][igf_stop_sfb], 0,
           (SFB_NUM_MAX - igf_stop_sfb) * (sizeof(cplx_pred_used[grp][0])));
  }
  else
  {
    for (grp = 0; grp < num_window_groups; grp++)
    {
      for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += SFB_PER_PRED_BAND)
      {
        cplx_pred_used[grp][sfb] = (UWORD8)ia_core_coder_read_bits_buf(it_bit_buff, 1);

        if (sfb + 1 < igf_stop_sfb)
          cplx_pred_used[grp][sfb + 1] = cplx_pred_used[grp][sfb];
      }

      memset(&cplx_pred_used[grp][igf_stop_sfb], 0,
             (SFB_NUM_MAX - igf_stop_sfb) * (sizeof(cplx_pred_used[grp][0])));
    }
  }

  *igf_pred_dir = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  if (0 == usac_data->usac_independency_flg)
    delta_code_time = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  else
    delta_code_time = 0;

  for (grp = 0; grp < num_window_groups; grp++)
  {
    for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb += SFB_PER_PRED_BAND)
    {
      if (delta_code_time != 1)
      {
        if (sfb > igf_start_sfb)
        {
          last_alpha_q_re = alpha_q_re[grp][sfb - 1];
        }
        else
        {
          last_alpha_q_re = 0;
        }
      }
      else
      {
        last_alpha_q_re = alpha_q_re_prev[sfb];
      }

      if (cplx_pred_used[grp][sfb] != 1)
      {
        alpha_q_re[grp][sfb] = 0;
        alpha_q_im[grp][sfb] = 0;
      }
      else
      {
        dpcm_alpha = -ia_core_coder_huff_codeword(ptr_huff_code_word, it_bit_buff) + 60;
        alpha_q_re[grp][sfb] = dpcm_alpha + last_alpha_q_re;
        alpha_q_im[grp][sfb] = 0;
      }

      if ((sfb + 1) < igf_stop_sfb)
      {
        alpha_q_re[grp][sfb + 1] = alpha_q_re[grp][sfb];
        alpha_q_im[grp][sfb + 1] = alpha_q_im[grp][sfb];
      }

      alpha_q_re_prev[sfb] = alpha_q_re[grp][sfb];
      alpha_q_im_prev[sfb] = alpha_q_im[grp][sfb];
    }

    memset(&alpha_q_re[grp][igf_stop_sfb], 0,
           (SFB_NUM_MAX - igf_stop_sfb) * sizeof(alpha_q_re[grp][0]));
    memset(&alpha_q_im[grp][igf_stop_sfb], 0,
           (SFB_NUM_MAX - igf_stop_sfb) * sizeof(alpha_q_im[grp][0]));
    memset(&alpha_q_re_prev[igf_stop_sfb], 0,
           (SFB_NUM_MAX - igf_stop_sfb) * sizeof(alpha_q_re_prev[0]));
    memset(&alpha_q_im_prev[igf_stop_sfb], 0,
           (SFB_NUM_MAX - igf_stop_sfb) * sizeof(alpha_q_im_prev[0]));
  }

  return;
}

/**
 *  ia_core_coder_get_igf_levels
 *
 *  \brief Calculate igf levels
 *
 *  \param [in]    it_bit_buff      bit buffer stream
 *  \param [in]    usac_data      USAC data structure
 *  \param [in]    igf_config      igf config structure
 *  \param [in]    num_window_groups  number of window groups
 *  \param [in]    chn          channel under process
 *  \param [in]    igf_frame_id    igf frame id
 *  \param [in]    igf_win_type    igf window type
 *
 *  \return WORD32
 *
 */
WORD32 ia_core_coder_get_igf_levels(ia_bit_buf_struct *it_bit_buff,
                                    ia_usac_data_struct *usac_data,
                                    ia_usac_igf_config_struct *igf_config,
                                    WORD32 num_window_groups, WORD32 chn, WORD32 igf_frame_id,
                                    WORD8 igf_win_type)
{
  WORD32 win, sfb, igf_all_zero = 0, bit_offset = 0;
  WORD32 factor = 2;
  WORD32 sf_idx = 0;
  WORD32 igf_start_sfb;
  WORD32 igf_stop_sfb;
  WORD32 flag = 0;
  WORD32 flag_swb_offset = 0;
  FLOAT32 igf_emphasis = 0;
  WORD16 *swb_offset = (WORD16 *)usac_data->ptr_fft_scratch;

  ia_bit_buf_struct it_bit_buff_temp = {0};
  ia_state_arith stat = {0, 0, 0};
  ia_usac_igf_dec_data_struct *igf_dec_data = &usac_data->igf_dec_data[chn];
  ia_core_coder_igf_level_scr_t *pscr;
  ia_sfb_info_struct *pstr_usac_winmap = usac_data->pstr_usac_winmap[igf_win_type];

  if (3 == igf_frame_id)
  {
    sf_idx = 1;
  }
  igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;
  igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;

  igf_all_zero = ia_core_coder_read_bits_buf(it_bit_buff, 1);

  ia_core_coder_igf_get_swb_offset(igf_win_type, swb_offset, pstr_usac_winmap);
  if (swb_offset[igf_start_sfb] <= igf_config->igf_grid[igf_win_type].igf_min)
  {
    flag_swb_offset = 1;
  }

  pscr = (ia_core_coder_igf_level_scr_t *)usac_data->ptr_fft_scratch;

  if (igf_config->use_high_res || igf_win_type > 1)
  {
    factor = 1;
  }

  if (igf_all_zero || usac_data->usac_independency_flg ||
      (igf_win_type != igf_dec_data->igf_memory.prev_win_type))
  {
    for (win = 0; win < MAX_SHORT_WINDOWS; win++)
    {
      ia_core_coder_memset(igf_dec_data->igf_bitstream[sf_idx].igf_levels_curr_float[win],
                           NSFB_LONG);
    }

    memset(igf_dec_data->igf_envelope.igf_levels_prev_fix, 0,
           NSFB_LONG * sizeof(igf_dec_data->igf_envelope.igf_levels_prev_fix[0]));

    igf_dec_data->igf_envelope.igf_arith_time_idx = 0;
    igf_dec_data->igf_envelope.igf_prev_win_ctx = 0;
  }

  if (igf_all_zero)
  {
    ia_core_coder_igf_reset_data(igf_dec_data, igf_frame_id);
  }
  else
  {
    it_bit_buff_temp = *it_bit_buff;
    bit_offset = ia_core_coder_arith_first_symbol(&it_bit_buff_temp, &stat);

    if (igf_win_type > 1)
    {
      igf_emphasis = 40;
    }
    else
    {
      igf_emphasis = 0;
    }

    for (win = 0; win < num_window_groups; win++)
    {
      ia_core_coder_igf_arith_decode(&it_bit_buff_temp, &bit_offset, &stat, igf_win_type,
                                     igf_config, igf_dec_data,
                                     pscr->igf_curr_q[win] + igf_start_sfb);

      igf_dec_data->igf_envelope.igf_prev_win_ctx =
          igf_dec_data->igf_envelope.igf_levels_prev_fix[igf_start_sfb];

      for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb++)
      {
        igf_dec_data->igf_envelope.igf_levels_prev_fix[sfb] = pscr->igf_curr_q[win][sfb];
        pscr->igf_curr_q[win][sfb] *= factor;

        igf_dec_data->igf_bitstream[sf_idx].igf_levels_curr_float[win][sfb] = (FLOAT32)pow(
            2.0, ia_mul_flt(0.25, ia_sub_flt((FLOAT32)pscr->igf_curr_q[win][sfb], igf_emphasis)));
      }
      igf_dec_data->igf_envelope.igf_arith_time_idx++;
    }
    ia_core_coder_skip_bits_buf(it_bit_buff, bit_offset - 14);
  }

  igf_dec_data->igf_bitstream[sf_idx].igf_all_zero = igf_all_zero;
  igf_dec_data->igf_memory.prev_win_type = igf_win_type;

  /*Checking all values in igf_curr[ ][ ][ ] == 0*/
  for (win = 0; win < num_window_groups; win++)
  {
    for (sfb = igf_start_sfb; sfb < igf_stop_sfb; sfb++)
    {
      if (pscr->igf_curr_q[win][sfb] != 0)
      {
        flag = 1;
        break;
      }
    }
    if (flag == 1)
    {
      break;
    }
  }
  /*igf_AllZero shall be 1 if one of the following conditions is true: m_igfStartSfb == *
  m_igfStopSfb
  all values in igf_curr[ ][ ][ ] == 0
  swb_offset[m_igfStartSfb] ≤ igfMin (flag_swb_offset)*/

  if (((igf_start_sfb == igf_stop_sfb) || (!flag) || (flag_swb_offset)) && (igf_all_zero != 1))
  {
    return -1;
  }

  return igf_all_zero;
}

/**
 *  ia_core_coder_get_igf_data
 *
 *  \brief Update igf decoder data structure using bit stream
 *
 *  \param [in]    it_bit_buff    bit stream buffer
 *  \param [in]    usac_data    USAC data structure
 *  \param [in,out]  igf_config    igf configuration structure
 *  \param [in]    chn        channel under process
 *  \param [in]    igf_frame_id  igf frame id type
 *  \param [in]    igf_win_type  igf frame type
 *
 *
 *
 */
VOID ia_core_coder_get_igf_data(ia_bit_buf_struct *it_bit_buff, ia_usac_data_struct *usac_data,
                                ia_usac_igf_config_struct *igf_config, WORD32 chn,
                                WORD32 igf_frame_id, WORD32 igf_win_type)
{
  WORD32 i, use_prev_tile, use_prev_whitening_level;
  WORD32 is_rem_tiles_diff;
  WORD32 sf_idx = 0;
  WORD32 num_tiles;

  ia_usac_igf_dec_data_struct *igf_dec_data = &usac_data->igf_dec_data[chn];

  if (3 == igf_frame_id)
  {
    sf_idx = 1;
  }
  num_tiles = igf_config->igf_grid[igf_win_type].igf_num_tiles;

  if (usac_data->usac_independency_flg)
  {
    use_prev_tile = 0;
  }
  else
  {
    use_prev_tile = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  }

  if (0 == use_prev_tile)
  {
    for (i = 0; i < num_tiles; i++)
    {
      igf_dec_data->igf_bitstream[sf_idx].igf_tile_num[i] =
          (WORD8)ia_core_coder_read_bits_buf(it_bit_buff, 2);
    }
  }
  else
  {
    memcpy(igf_dec_data->igf_bitstream[sf_idx].igf_tile_num,
           igf_dec_data->igf_memory.prev_tile_num,
           num_tiles * sizeof(igf_dec_data->igf_bitstream[sf_idx].igf_tile_num[0]));
  }

  memcpy(igf_dec_data->igf_memory.prev_tile_num, igf_dec_data->igf_bitstream[sf_idx].igf_tile_num,
         num_tiles * sizeof(igf_dec_data->igf_memory.prev_tile_num[0]));
  memset(igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level, 0,
         num_tiles * sizeof(igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level[0]));

  memset(&igf_dec_data->igf_memory.prev_tile_num[num_tiles], 3,
         (4 - num_tiles) * sizeof(igf_dec_data->igf_memory.prev_tile_num[0]));
  memset(&igf_dec_data->igf_memory.prev_whitening_level[num_tiles], 0,
         (4 - num_tiles) * sizeof(igf_dec_data->igf_memory.prev_whitening_level[0]));
  memset(&igf_dec_data->igf_bitstream[sf_idx].igf_tile_num[num_tiles], 3,
         (4 - num_tiles) * sizeof(igf_dec_data->igf_bitstream[sf_idx].igf_tile_num[0]));
  memset(&igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level[num_tiles], 0,
         (4 - num_tiles) * sizeof(igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level[0]));

  if (igf_config->use_whitening && (1 != igf_win_type))
  {
    if (!usac_data->usac_independency_flg)
    {
      use_prev_whitening_level = ia_core_coder_read_bits_buf(it_bit_buff, 1);
    }
    else
    {
      use_prev_whitening_level = 0;
    }
    if (use_prev_whitening_level == 0)
    {
      i = 0;
      ia_core_coder_igf_decode_whitening_level(it_bit_buff, &igf_dec_data->igf_bitstream[sf_idx],
                                               i);
      is_rem_tiles_diff = ia_core_coder_read_bits_buf(it_bit_buff, 1);

      if (is_rem_tiles_diff == 1)
      {
        for (i = 1; i < num_tiles; i++)
        {
          ia_core_coder_igf_decode_whitening_level(it_bit_buff,
                                                   &igf_dec_data->igf_bitstream[sf_idx], i);
        }
      }
      else
      {
        for (i = 1; i < num_tiles; i++)
        {
          igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level[i] =
              igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level[0];
        }
      }
    }
    else
    {
      memcpy(igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level,
             igf_dec_data->igf_memory.prev_whitening_level,
             num_tiles * sizeof(igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level[0]));
    }
  }

  memcpy(igf_dec_data->igf_memory.prev_whitening_level,
         igf_dec_data->igf_bitstream[sf_idx].igf_whitening_level,
         MAX_IGF_TILES * sizeof(igf_dec_data->igf_memory.prev_whitening_level[0]));

  if (igf_config->use_tnf && (1 != igf_win_type))
  {
    igf_dec_data->igf_bitstream[sf_idx].igf_is_tnf = ia_core_coder_read_bits_buf(it_bit_buff, 1);
  }
}

/**
 *  ia_core_coder_igf_init
 *
 *  \brief Initialize igf config structure
 *
 *  \param [in,out]  usac_data        usac data config
 *  \param [in]    ptr_usac_ele_config    usac element config
 *  \param [in]    ele_type        element type
 *  \param [in]    id            element index in number of
 * elements
 *  \param [in]    sample_rate        sample rate
 *  \param [in]    chan          channel under process
 *
 *
 *
 */
VOID ia_core_coder_igf_init(ia_usac_data_struct *usac_data,
                            ia_usac_dec_element_config_struct *ptr_usac_ele_config,
                            WORD32 ele_type, WORD32 id, UINT32 sample_rate, WORD32 chan)
{
  WORD32 sfb, grid;
  WORD32 igf_sfb_start_long = 0;
  WORD32 igf_sfb_stop_long = 0;
  WORD32 igf_sfb_start_short = 0;
  WORD32 igf_sfb_stop_short = 0;

  ia_usac_igf_config_struct *igf_config = &usac_data->igf_config[id];
  ia_usac_igf_init_config_struct *usac_igf_config =
      &ptr_usac_ele_config->str_usac_ele_igf_init_config;
  ia_sfb_info_struct *igf_sfb_info_long = usac_data->pstr_usac_winmap[ONLY_LONG_SEQUENCE];
  ia_sfb_info_struct *igf_sfb_info_short = usac_data->pstr_usac_winmap[EIGHT_SHORT_SEQUENCE];

  if (ele_type == ID_USAC_CPE || ele_type == ID_USAC_SCE)
    igf_config->igf_active = (WORD8)ptr_usac_ele_config->enhanced_noise_filling;

  if (igf_config->igf_active)
  {
    igf_config->igf_after_tns_synth = usac_igf_config->igf_after_tns_synth;
    igf_config->use_inf = usac_igf_config->igf_use_enf;
    igf_config->use_high_res = usac_igf_config->igf_use_high_resolution;
    igf_config->use_whitening = usac_igf_config->igf_use_whitening;
    igf_config->use_tnf = usac_igf_config->igf_use_enf;

    if (1 == usac_igf_config->igf_independent_tilling)
    {
      igf_config->igf_independent_tiling = 1;
    }
    else
    {
      igf_config->igf_independent_tiling = 0;
    }

    igf_sfb_start_long =
        ia_min_int(11 + usac_igf_config->igf_start_index, igf_sfb_info_long->sfb_per_bk - 5);

    if (usac_igf_config->igf_stop_index == 15)
    {
      igf_sfb_stop_long = igf_sfb_info_long->sfb_per_bk;
    }
    else
    {
      igf_sfb_stop_long =
          ia_min_int(igf_sfb_info_long->sfb_per_bk,
                     ia_max_int(igf_sfb_start_long +
                                    (((igf_sfb_info_long->sfb_per_bk - (igf_sfb_start_long + 1)) *
                                      (usac_igf_config->igf_stop_index + 2)) >>
                                     4),
                                igf_sfb_start_long + 1));
    }

    igf_sfb_start_short = -1;
    igf_sfb_stop_short = -1;
    for (sfb = 0; sfb < igf_sfb_info_short->sfb_per_sbk; sfb++)
    {
      if (igf_sfb_info_short->ptr_sfb_tbl[sfb] >=
          (igf_sfb_info_long->ptr_sfb_tbl[igf_sfb_start_long - 1] >> 3))
      {
        if (igf_sfb_start_short < 0)
        {
          igf_sfb_start_short = sfb + 1;
        }
      }

      if (igf_sfb_info_short->ptr_sfb_tbl[sfb] >=
          (igf_sfb_info_long->ptr_sfb_tbl[igf_sfb_stop_long - 1] >> 3))
      {
        if (igf_sfb_stop_short < 0)
        {
          igf_sfb_stop_short = sfb + 1;
        }
      }
    }

    igf_config->igf_grid[0].igf_sfb_start = (WORD8)igf_sfb_start_long;
    igf_config->igf_grid[0].igf_sfb_stop = (WORD8)igf_sfb_stop_long;
    igf_config->igf_grid[1].igf_sfb_start = (WORD8)igf_sfb_start_short;
    igf_config->igf_grid[1].igf_sfb_stop = (WORD8)igf_sfb_stop_short;

    for (grid = 0; grid < 4; grid++)
    {
      igf_config->igf_grid[grid].igf_min =
          (WORD8)ia_core_coder_igf_get_igf_min(grid, usac_data->ccfl, sample_rate);
    }

    igf_config->igf_grid[0].igf_num_tiles = (WORD8)ia_core_coder_igf_get_num_tiles(
        &igf_config->igf_grid[0], igf_config->use_high_res,
        (WORD16 *)(igf_sfb_info_long->ptr_sfb_tbl - 1), NULL);
    igf_config->igf_grid[1].igf_num_tiles = (WORD8)ia_core_coder_igf_get_num_tiles(
        &igf_config->igf_grid[1], igf_config->use_high_res,
        (WORD16 *)(igf_sfb_info_short->ptr_sfb_tbl - 1), NULL);

    igf_config->igf_grid[2].igf_sfb_start = igf_config->igf_grid[1].igf_sfb_start;
    igf_config->igf_grid[2].igf_sfb_stop = igf_config->igf_grid[1].igf_sfb_stop;
    igf_config->igf_grid[2].igf_num_tiles = igf_config->igf_grid[1].igf_num_tiles;

    igf_config->igf_grid[3].igf_sfb_start = igf_config->igf_grid[1].igf_sfb_start;
    igf_config->igf_grid[3].igf_sfb_stop = igf_config->igf_grid[1].igf_sfb_stop;
    igf_config->igf_grid[3].igf_num_tiles = igf_config->igf_grid[1].igf_num_tiles;

    ia_core_coder_igf_reset(&usac_data->igf_dec_data[chan]);
    if (ele_type == ID_USAC_CPE)
      ia_core_coder_igf_reset(&usac_data->igf_dec_data[chan + 1]);
  }
}

/**
 *  ia_core_coder_igf_tnf
 *
 *  \brief Apply igf tnf
 *
 *  \param [in]    usac_data    USAC data config
 *  \param [in]    igf_config    igf config structure
 *  \param [in]    chn        channel under process
 *  \param [in]    igf_frame_id  frame index in a igf frame
 *  \param [in]    igf_win_type  igf window type
 *  \param [in]    win        window type
 *  \param [in]    coef      IGF coeff
 *  \param [in]    scratch      Scratch buffer for internal processing
 *
 *
 *
 */
VOID ia_core_coder_igf_tnf(ia_usac_data_struct *usac_data, ia_usac_igf_config_struct *igf_config,
                           WORD32 chn, WORD32 igf_frame_id, WORD32 igf_win_type, WORD32 win,
                           FLOAT32 *coef, FLOAT32 *scratch)
{
  WORD32 swb;
  WORD32 sf_idx = 0;
  WORD32 igf_start_sfb;
  WORD32 igf_stop_sfb;

  WORD16 *swb_offset = (WORD16 *)scratch;
  ia_sfb_info_struct *pstr_usac_winmap = usac_data->pstr_usac_winmap[win];
  ia_usac_igf_dec_data_struct *igf_dec_data = &usac_data->igf_dec_data[chn];
  igf_dec_data->igf_scratch = (FLOAT32 *)swb_offset + 64;

  if (3 == igf_frame_id)
  {
    sf_idx = 1;
  }
  igf_start_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_start;
  igf_stop_sfb = igf_config->igf_grid[igf_win_type].igf_sfb_stop;

  ia_core_coder_igf_get_swb_offset(igf_win_type, swb_offset, pstr_usac_winmap);

  if (igf_config->use_tnf && (1 != igf_win_type))
  {
    if (igf_dec_data->igf_bitstream[sf_idx].igf_is_tnf == 1)
    {
      ia_core_coder_tnf_apply(igf_dec_data, swb_offset[igf_start_sfb], swb_offset[igf_stop_sfb]);

      for (swb = swb_offset[igf_start_sfb]; swb < swb_offset[igf_stop_sfb]; swb++)
      {
        if (0 == igf_dec_data->igf_tnf_mask[swb])
        {
          coef[swb] = igf_dec_data->igf_tnf_spec_float[swb];
        }
      }
    }
  }
}

/**
 *  ia_core_coder_igf_stereofilling
 *
 *  \brief Performs igf stereo filling
 *
 *  \param [in,out]  coef            IGF coefficient
 *  \param [in]    dmx_prev          Previous downmix coeff
 *  \param [in]    info            sfb info structure
 *  \param [in]    noise_filling        noise filling config
 *  \param [in]    band_quantized_to_zero    flag to indicate zero quantized band
 *  \param [in]    stereo_filling        flag to enable stereo filling
 *  \param [in]    win_tot            window
 *  \param [in]    sfb              scale factor band
 *  \param [in]    noise_filling_start_offset  noise filling start offset
 *  \param [in]    grp              group
 *  \param [in]    max_noise_sfb        sfb noise
 *
 *
 *
 */
VOID ia_core_coder_igf_stereofilling(FLOAT32 *coef, FLOAT32 *dmx_prev, ia_sfb_info_struct *info,
                                     WORD32 noise_filling, WORD32 band_quantized_to_zero,
                                     WORD32 stereo_filling, WORD32 win_tot, WORD32 sfb,
                                     WORD32 noise_filling_start_offset, WORD32 grp,
                                     WORD32 max_noise_sfb)
{
  WORD32 grp_idx, sfb_idx, win, start_sfb, offset;

  if (band_quantized_to_zero && noise_filling && dmx_prev && stereo_filling)
  {
    FLOAT32 energy_dmx = 0.0f, sfb_width = 0.0f, energy_coef = 0.0f;

    for (grp_idx = 0; grp_idx < info->group_len[grp]; grp_idx++)
    {
      win = win_tot + grp_idx;
      offset = win * info->bins_per_sbk;
      if (sfb > 0)
      {
        start_sfb = info->ptr_sfb_tbl[sfb - 1];
      }
      else
      {
        start_sfb = 0;
      }

      if (start_sfb >= noise_filling_start_offset && sfb < max_noise_sfb)
      {
        for (sfb_idx = start_sfb; sfb_idx < info->ptr_sfb_tbl[sfb]; sfb_idx++)
        {
          energy_dmx = ia_add_flt(
              energy_dmx, ia_mul_flt(dmx_prev[offset + sfb_idx], dmx_prev[offset + sfb_idx]));
          coef[offset + sfb_idx] = ia_mul_flt(coef[offset + sfb_idx], 4);
          energy_coef =
              ia_add_flt(energy_coef, ia_mul_flt(coef[offset + sfb_idx], coef[offset + sfb_idx]));
        }
        sfb_width = ia_add_flt(sfb_width, (FLOAT32)(info->ptr_sfb_tbl[sfb] - start_sfb));
      }
    }
    if (ia_lt_flt(energy_coef, sfb_width))
    {
      energy_dmx =
          ((FLOAT32)ia_sqrt_flt((FLOAT64)(ia_sub_flt(sfb_width, energy_coef)) / energy_dmx));
      energy_dmx = ia_min_flt(energy_dmx, 10);
      energy_coef = 0;

      for (grp_idx = 0; grp_idx < info->group_len[grp]; grp_idx++)
      {
        win = win_tot + grp_idx;
        offset = win * info->bins_per_sbk;
        if (sfb > 0)
        {
          start_sfb = info->ptr_sfb_tbl[sfb - 1];
        }
        else
        {
          start_sfb = 0;
        }

        if (start_sfb >= noise_filling_start_offset && sfb < max_noise_sfb)
        {
          for (sfb_idx = start_sfb; sfb_idx < info->ptr_sfb_tbl[sfb]; sfb_idx++)
          {
            coef[offset + sfb_idx] = ia_add_flt(
                coef[offset + sfb_idx], ia_mul_flt(dmx_prev[offset + sfb_idx], energy_dmx));
            energy_coef = ia_add_flt(energy_coef,
                                     ia_mul_flt(coef[offset + sfb_idx], coef[offset + sfb_idx]));
          }
        }
      }

      if (ia_lt_flt(0, energy_coef) && (!ia_eq_flt(energy_coef, sfb_width)))
      {
        energy_coef = (FLOAT32)ia_sqrt_flt(sfb_width / ia_add_flt(energy_coef, 1e-8f));
        energy_coef = ia_min_flt(10.0f, energy_coef);

        for (grp_idx = 0; grp_idx < info->group_len[grp]; grp_idx++)
        {
          win = win_tot + grp_idx;
          offset = win * info->bins_per_sbk;
          if (sfb > 0)
          {
            start_sfb = info->ptr_sfb_tbl[sfb - 1];
          }
          else
          {
            start_sfb = 0;
          }

          if (start_sfb >= noise_filling_start_offset && sfb < max_noise_sfb)
          {
            for (sfb_idx = start_sfb; sfb_idx < info->ptr_sfb_tbl[sfb]; sfb_idx++)
            {
              coef[offset + sfb_idx] = (ia_mul_flt(coef[offset + sfb_idx], energy_coef));
            }
          }
        }
      }
    }
  }
}

/**
 *  ia_core_coder_igf_mono
 *
 *  \brief igf mono processing including igf whitening and applying mono igf
 *
 *  \param [in,out]    usac_data      USAC data structure
 *  \param [in,out]    igf_config      IGF config structure
 *  \param [in]      chn          channel under process
 *  \param [in]      igf_win_type    IGF window type
 *  \param [in]      igf_frame_id    index of IGF frame
 *  \param [in]      num_groups      number of groups
 *  \param [in]      group_len      group length
 *  \param [in]      bins_per_sbk    bins per subblock
 *  \param [in]      win          window type
 *  \param [in]      coef        IGF coeff
 *  \param [in]      scratch        Scratch buffer for internal
 * processing
 *
 *
 *
 */
VOID ia_core_coder_igf_mono(ia_usac_data_struct *usac_data, ia_usac_igf_config_struct *igf_config,
                            WORD32 chn, WORD32 igf_win_type, WORD32 igf_frame_id,
                            WORD32 num_groups, WORD16 *group_len, WORD32 bins_per_sbk, WORD32 win,
                            FLOAT32 *coef, FLOAT32 *scratch)
{
  WORD32 grp, grp_idx;
  WORD32 start_win_offset;
  WORD32 win_offset = 0;
  WORD32 sf_idx = 0;
  UWORD32 igf_nf_seed_value[2] = {0};

  if (3 == igf_frame_id)
  {
    sf_idx = 1;
  }

  WORD16 *swb_offset = (WORD16 *)scratch;
  UWORD32 *p_seed_value = &(usac_data->seed_value[chn]);

  FLOAT32 *igf_scratch = (FLOAT32 *)((WORD16 *)swb_offset + 64);
  FLOAT32 *igf_input_spec;

  ia_sfb_info_struct *pstr_usac_winmap = usac_data->pstr_usac_winmap[win];
  ia_usac_igf_dec_data_struct *igf_dec_data = &usac_data->igf_dec_data[chn];
  ia_usac_igf_bitstream_struct *igf_bitstream = &igf_dec_data->igf_bitstream[sf_idx];
  ia_usac_igf_envelope_struct *igf_envelope = &igf_dec_data->igf_envelope;

  igf_input_spec = igf_dec_data->igf_input_spec;

  igf_nf_seed_value[0] = *p_seed_value;
  igf_nf_seed_value[1] = *p_seed_value;

  if (0 == igf_bitstream->igf_all_zero)
  {
    ia_core_coder_igf_get_swb_offset(igf_win_type, swb_offset, pstr_usac_winmap);

    if (igf_config->use_whitening && (1 != igf_win_type))
    {
      ia_core_coder_igf_apply_whitening(igf_config, igf_bitstream, igf_input_spec, swb_offset,
                                        igf_win_type, igf_scratch);
    }
    for (grp = 0; grp < num_groups; grp++)
    {
      start_win_offset = win_offset;
      ia_core_coder_igf_reset_level(igf_envelope);

      for (grp_idx = 0; grp_idx < group_len[grp]; grp_idx++)
      {
        ia_core_coder_igf_calc_mono(igf_config, igf_dec_data, igf_bitstream,
                                    &coef[win_offset * bins_per_sbk],
                                    &igf_input_spec[win_offset * bins_per_sbk], swb_offset,
                                    &igf_nf_seed_value[0], igf_win_type);
        win_offset++;
      }
      ia_core_coder_igf_norm(igf_config, igf_envelope, igf_win_type, group_len[grp]);
      win_offset = start_win_offset;

      for (grp_idx = 0; grp_idx < group_len[grp]; grp_idx++)
      {
        ia_core_coder_igf_apply_mono(igf_dec_data, igf_config, igf_envelope, igf_bitstream,
                                     &coef[win_offset * bins_per_sbk],
                                     &igf_input_spec[win_offset * bins_per_sbk], swb_offset,
                                     &igf_nf_seed_value[1], igf_win_type, grp);
        win_offset++;
      }
    }
    *p_seed_value = igf_nf_seed_value[0];
  }
}
/** @} */ /* End of CoreDecProc */