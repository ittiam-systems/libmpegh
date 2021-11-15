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

#ifndef IMPEGHD_TBE_DEC_H
#define IMPEGHD_TBE_DEC_H

#define LEN_TBE_FRAME (LEN_FRAME)
#define NUM_TBE_FRAME (4)
#define NUM_SUBFR (16)
#define LEN_SUBFR_TBE ((LEN_TBE_FRAME / NUM_TBE_FRAME))
#define PI_INV (1.0 / PI)
#define NUM_SUBGAINS (4)
#define AP_NUM (3)
#define LPC_ORDER_TBE (10)
#define LPC_WHTN_ORDER (4)
#define NL_BUFF_OFFSET (12)
#define NL_BUFF_OFFSET_MAX (32)
#define RESP_ORDER (15)
#define TBE_LAHEAD (16)
#define TBE_OVERLAP_LEN (16)
#define TBE_DEC_DELAY ((TBE_LAHEAD + (NL_BUFF_OFFSET / 2)))
#define TBE_PIT_MAX (434)
#define VAL_A (0.57f)
#define VAL_B (0.098f)
#define SCRATCH_IDX_NL_LEVEL_2 (524)
#define ENV_SHAPE_CONST_1 (1.09875f)
#define ENV_SHAPE_CONST_2 (0.49875f)
#define ENV_SHAPE_CONST_3 (0.6f)
#define ENV_SHAPE_CONST_4 (0.999f)
#define RAND_SIGN_CONST_1 (31821L)
#define RAND_SIGN_CONST_2 (13849L)
#define RAND_SIGN_CONST_3 (0.0078f)
#define RAND_VEC_GAIN_1 (563.154f)
#define RAND_VEC_GAIN_2 (225.261f)

/* TD resampler */
#define L_FRAME_MAX (2048)
#define L_FILT_MAX (30)
#define UPSAMP_FILT_MEM_SIZE (30)
#define DOWNSAMP_FILT_MEM_SIZE (60)
#define UPSAMP_FILT_LEN (15)
#define DOWNSAMP_FILT_LEN (30)

typedef struct
{
  WORD32 harm_ext_mode;
  WORD32 frame_gain_idx;
  WORD32 sub_gain_idx;
  WORD32 lsf_idx[2];
  WORD32 idx_mix_config;
  WORD32 idx_shb_fr_gain;
  WORD32 idx_res_sub_gains;
  WORD32 idx_shb_exc_resp[2];
  WORD32 hr_config;
  WORD32 nl_config;
} ia_usac_tbe_bitstream_struct, *ia_usac_tbe_bitstream_handle;

typedef struct
{
  FLOAT32 gen_synth_state_lsyn_filt_local[(2 * AP_NUM + 1)];
  FLOAT32 mem_csfilt[2];
  FLOAT32 mem_gen_exc_filt_down[(2 * AP_NUM + 1)];
  FLOAT32 mem_resp_exc_whtnd[RESP_ORDER];
  FLOAT32 mem_whtn_filt[LPC_WHTN_ORDER];
  FLOAT32 old_tbe_exc[2 * TBE_PIT_MAX];
  FLOAT32 old_tbe_exc_extended[NL_BUFF_OFFSET];
  FLOAT32 state_lpc_syn[LPC_ORDER_TBE];
  FLOAT32 state_syn_exc[TBE_LAHEAD];
  FLOAT32 syn_overlap[TBE_LAHEAD];
  FLOAT32 wn_ana_mem[LPC_ORDER_TBE];
  FLOAT32 wn_syn_mem[LPC_ORDER_TBE];

} ia_usac_tbe_state_struct, *ia_usac_tbe_state_handle;

typedef struct
{
  ia_usac_tbe_bitstream_struct bit_stream[2];
  ia_usac_tbe_state_struct tbe_state;
  WORD16 tbe_seed[2];
  FLOAT32 lsp_prev_interp[LPC_ORDER_TBE];
  FLOAT32 synth_prev[2 * LEN_FRAME];
  FLOAT32 synth_curr[2 * LEN_FRAME];
  FLOAT32 exc[(TBE_PIT_MAX + (LEN_FRAME + 1) + LEN_SUBFR) * 2];

} ia_usac_tbe_data_struct, *ia_usac_tbe_data_handle;

VOID ia_core_coder_tbe_apply(ia_usac_tbe_data_handle tbe_dec_data, WORD32 frame_class,
                             FLOAT32 *ptr_tbe_exc_extended, FLOAT32 *ptr_voice_factors,
                             FLOAT32 *ptr_synth, WORD32 first_frame, FLOAT32 *ptr_scratch);

VOID ia_core_coder_tbe_gen_transition(ia_usac_tbe_data_handle tbe_dec_data, WORD32 length,
                                      FLOAT32 *ptr_output, FLOAT32 *ptr_scratch);

VOID ia_core_coder_tbe_init(ia_usac_tbe_data_handle tbe_dec_data);

VOID ia_core_coder_tbe_data(ia_usac_tbe_data_handle tbe_dec_data,
                            ia_bit_buf_struct *ptr_it_bit_buff, WORD32 tbe_frame_class);

VOID ia_core_coder_mix_past_exc(ia_usac_tbe_data_handle tbe_dec_data, FLOAT32 *ptr_error,
                                WORD32 subfr_idx, WORD32 pitch_lag, WORD32 pitch_lag_frac);

VOID ia_core_coder_tbe_prep_exc(ia_usac_tbe_data_handle tbe_dec_data, WORD32 i_subfr,
                                FLOAT32 gain_pit, FLOAT32 gain_code, FLOAT32 *ptr_code,
                                FLOAT32 voice_fac, FLOAT32 *ptr_voice_factors);

VOID ia_core_coder_td_resampler(FLOAT32 *ptr_input, WORD16 num_samples_in, WORD32 fs_in,
                                FLOAT32 *ptr_output, WORD32 fs_out, FLOAT32 *ptr_filter_mem,
                                WORD32 interp_only, FLOAT32 *ptr_scratch);

VOID ia_core_coder_memset(FLOAT32 *x, WORD32 n);
VOID ia_core_coder_mem_cpy(const FLOAT32 *x, FLOAT32 *y, WORD32 n);
VOID ia_core_coder_reset_tbe_state(ia_usac_tbe_data_handle tbe_dec_data);

#endif /* IMPEGHD_TBE_DEC_H */
