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

#ifndef IA_CORE_CODER_MAIN_H
#define IA_CORE_CODER_MAIN_H

typedef struct
{
  WORD32 data_present;
  WORD32 pitch_lag_idx;
  WORD32 gain_idx;

  WORD32 frame_len;
  WORD32 transition_len;
  WORD32 codec_delay;

  WORD32 pitch_min;
  WORD32 pitch_fr2;
  WORD32 pitch_fr1;
  WORD32 pitch_max;
  WORD32 pitch_res;

  WORD32 prev_pitch;
  WORD32 prev_pitch_fr;
  WORD32 prev_gain_idx;
  FLOAT32 prev_gain;
  FLOAT32 prev_in_buf[LTPF_NUM_FILT_COEF2 - 1];
  FLOAT32 prev_out_buf[LTPF_PITCH_MAX_96K + (LTPF_NUM_FILT_COEF1 / 2)];
  FLOAT32 *ptr_ltpf_scratch;
} ia_ltpf_data_str;

typedef struct ia_usac_lpd_decoder
{
  WORD32 mode_prev;
  WORD32 len_subfrm;
  WORD32 pitch_prev[NUM_SUBFR_SUPERFRAME_BY2 - 1];
  FLOAT32 synth_prev[MAX_PITCH + SYNTH_DELAY_LMAX];
  FLOAT32 xcitation_prev[MAX_PITCH + INTER_LP_FIL_ORDER + 1];
  FLOAT32 gain_prev[NUM_SUBFR_SUPERFRAME_BY2 - 1];
  FLOAT32 lp_flt_coeff_a_prev[2 * (ORDER + 1)];
  FLOAT32 exc_prev[1 + (2 * FAC_LENGTH)];
  FLOAT32 bpf_prev[2 * FILTER_DELAY + 1 + 2 * LEN_SUBFR];
  FLOAT32 fac_gain;
  FLOAT32 fac_fd_data[FAC_LENGTH / 4];
  FLOAT32 lspold[ORDER];
  FLOAT32 lsf_prev[ORDER];

  FLOAT32 gain_threshold;
  FLOAT32 *fd_synth;
  FLOAT32 fd_synth_buf[3 * LEN_FRAME + 1 + ORDER];

  WORD32 fscale;
  WORD32 bpf_active_prev;

  WORD32 tns_data_present;
  WORD32 max_sfb;
  WORD32 window_shape;
  WORD32 window_shape_prev;
  WORD32 fdp_data_present;
  WORD32 fdp_spacing_index;
  WORD32 fdp_int[172];
  WORD32 fdp_quant_spec_prev[2][172];
  FLOAT32 synth_prev_fb[2 * (MAX_PITCH + SYNTH_DELAY_LMAX)];
  FLOAT32 *overlap_buf;
  FLOAT32 overlap_buf_fb[LEN_SUPERFRAME];
  FLOAT32 exc_prev_fb[(4 * FAC_LENGTH)];
  FLOAT32 td_resamp_mem[2 * L_FILT_MAX];
  ia_tns_frame_info_struct str_tns_info_td;
  ia_ltpf_data_str ltpf_data;
  ia_usac_tbe_data_struct tbe_dec_data;
  WORD32 td_resamp_delay;

} ia_usac_lpd_decoder, *ia_usac_lpd_decoder_handle;

typedef struct ia_usac_td_config_struct
{
  WORD32 full_band_lpd;
  WORD32 num_frame;
  WORD32 len_frame;
  WORD32 len_subfrm; // sbfram info remain same.. may not be required
  WORD32 num_subfrm;
  WORD32 lpd_stereo_idx;
  WORD32 igf_active;
  WORD32 fscale;
  WORD32 fscale_full_band;
  WORD32 fac_fb;
  WORD32 len_frame_fb;
} ia_usac_td_config_struct, *ia_usac_td_config_handle;

typedef struct ia_usac_data_main_struct
{
  FLOAT32 time_sample_hoa_vect[MAX_NUM_CHANNELS][1024];
  FLOAT32 time_sample_vector[MAX_NUM_CHANNELS][1024];
  FLOAT32 *time_sample_stft[MAX_NUM_CHANNELS];
  FLOAT32 overlap_data_ptr[MAX_NUM_CHANNELS][1024 + 256];
  WORD32 window_shape[MAX_NUM_CHANNELS];
  WORD32 window_shape_prev[MAX_NUM_CHANNELS];
  WORD32 window_sequence[MAX_NUM_CHANNELS];
  WORD32 window_sequence_last[MAX_NUM_CHANNELS];

  WORD32 output_samples;
  WORD32 sbr_ratio_idx;
  WORD32 usac_independency_flg;
  WORD32 is_base_line_profile_3b;

  WORD32 sampling_rate_idx;
  FLOAT32 lpc_prev[MAX_NUM_CHANNELS][ORDER + 1];
  FLOAT32 acelp_in[MAX_NUM_CHANNELS][1 + (2 * FAC_LENGTH)];
  WORD32 td_frame_prev[MAX_NUM_CHANNELS];
  WORD32 alpha_q_re[MAX_NUM_CHANNELS][MAX_SHORT_WINDOWS][SFB_NUM_MAX];
  WORD32 alpha_q_im[MAX_NUM_CHANNELS][MAX_SHORT_WINDOWS][SFB_NUM_MAX];
  UWORD8 cplx_pred_used[MAX_NUM_CHANNELS][MAX_SHORT_WINDOWS][SFB_NUM_MAX];

  WORD32 alpha_q_re_prev[MAX_NUM_CHANNELS][SFB_NUM_MAX];
  WORD32 alpha_q_im_prev[MAX_NUM_CHANNELS][SFB_NUM_MAX];
  FLOAT32 dmx_re_prev[MAX_NUM_CHANNELS][BLOCK_LEN_LONG]; // changed to float
  UWORD8 *ms_used[MAX_NUM_CHANNELS];

  FLOAT32 *coef[MAX_NUM_CHANNELS];
  FLOAT32 *coef_save_float[MAX_NUM_CHANNELS];

  WORD16 *factors[MAX_NUM_CHANNELS];
  UWORD8 *group_dis[MAX_NUM_CHANNELS];
  ia_tns_frame_info_struct *pstr_tns[MAX_NUM_CHANNELS];

  ia_usac_lpd_decoder_handle str_tddec[MAX_NUM_CHANNELS];

  WORD32 arith_prev_n[MAX_NUM_CHANNELS];
  WORD8 c_prev[MAX_NUM_CHANNELS][1024 / 2 + 4];
  WORD8 c[MAX_NUM_CHANNELS][1024 / 2 + 4];

  WORD32 noise_filling_config[MAX_NUM_ELEMENTS];
  UWORD32 seed_value[MAX_NUM_CHANNELS];
  WORD32 present_chan;

  WORD32 fac_data_present[MAX_NUM_CHANNELS];
  WORD32 fac_data[MAX_NUM_CHANNELS][FAC_LENGTH + 1];

  ia_sfb_info_struct *pstr_sfb_info[MAX_NUM_CHANNELS];
  ia_sfb_info_struct str_only_long_info;
  ia_sfb_info_struct str_eight_short_info;
  ia_sfb_info_struct *pstr_usac_winmap[NUM_WIN_SEQ];
  WORD16 sfb_width_short[(1 << LEN_MAX_SFBS)];

  WORD32 ccfl;
  WORD32 len_subfrm;
  WORD32 num_subfrm;

  WORD32 *scratch_int_buf;
  FLOAT32 pitch_gain[32];
  FLOAT32 lp_flt_coff[288];
  FLOAT32 exc_buf[1456];
  FLOAT32 synth_buf[1888];
  WORD32 pitch[32];
  UWORD16 *huffman_code_book_scl;
  UWORD32 *huffman_code_book_scl_index;

  WORD32 (*tns_max_bands_tbl_usac)[16][2];

  WORD16 sfb_width_long[(1 << LEN_MAX_SFBL)];

  FLOAT32 *x_ac_dec_float;
  FLOAT32 *scratch_buffer_float;
  FLOAT32 *ptr_fft_scratch;

  FLOAT32 mct_prev_out_spec[MAX_NUM_CHANNELS][1024];
  WORD8 usac_first_frame_flag[MAX_NUM_CHANNELS];
  WORD16 prev_aliasing_symmetry[MAX_NUM_CHANNELS];
  WORD16 curr_aliasing_symmetry[MAX_NUM_CHANNELS];

  FLOAT32 scale_factors[MAX_NUM_CHANNELS][SFB_NUM_MAX];
  ia_usac_igf_config_struct igf_config[MAX_NUM_ELEMENTS];
  ia_usac_igf_dec_data_struct igf_dec_data[MAX_NUM_CHANNELS];

  FLAG igf_hasmask[MAX_NUM_ELEMENTS][MAX_NUM_CHANNELS];
  WORD32 igf_pred_dir[MAX_NUM_ELEMENTS];
  WORD32 igf_sfb_start[MAX_NUM_CHANNELS];
  WORD32 igf_sfb_stop[MAX_NUM_CHANNELS];

  ia_usac_td_config_struct td_config[MAX_NUM_ELEMENTS];
  ia_usac_slpd_dec_data_struct slpd_dec_data[MAX_NUM_CHANNELS >> 1];
  WORD16 fdp_quant_spec_prev[MAX_NUM_CHANNELS][2 * 172];

  FLOAT32 arr_coef[MAX_NUM_CHANNELS][(LN2 + LN2 / 8)];
  FLOAT32 arr_coef_save_float[MAX_NUM_CHANNELS][(LN2 + LN2 / 8)];
  WORD16 arr_factors[MAX_NUM_CHANNELS][MAXBANDS];
  UWORD8 arr_group_dis[MAX_NUM_CHANNELS][NSHORT];
  UWORD8 arr_ms_used[MAX_NUM_CHANNELS][MAXBANDS];
  ia_usac_lpd_decoder arr_str_tddec[MAX_NUM_CHANNELS];
  ia_tns_frame_info_struct arr_str_tns[MAX_NUM_CHANNELS];

  WORD8 slpd_index[MAX_NUM_CHANNELS >> 1];
} ia_usac_data_struct;

typedef struct
{
  FLOAT32 x[LEN_SUPERFRAME];
  FLOAT32 alfd_gains[LEN_SUPERFRAME / (4 * 8)];
  FLOAT32 i_ap[ORDER + 1];
  FLOAT32 buf[ORDER + LEN_SUPERFRAME];
  FLOAT32 gain1[LEN_SUPERFRAME];
  FLOAT32 gain2[LEN_SUPERFRAME];
  FLOAT32 xn_buf[LEN_SUPERFRAME + (2 * FAC_LENGTH)];
  FLOAT32 xn1[2 * FAC_LENGTH];
  FLOAT32 facwindow[2 * FAC_LENGTH];
  FLOAT32 xn_buf_fb[2 * (LEN_SUPERFRAME + (2 * FAC_LENGTH))];
  FLOAT32 xn1_fb[2 * 2 * FAC_LENGTH];
  WORD32 igf_inf_mask[LEN_SUPERFRAME];
  WORD16 swb_offset[64 * 2];
} ia_core_coder_tcx_mdct_scratch_t;

typedef struct
{
  FLOAT32 lsp_curr[ORDER];
  FLOAT32 lsf_curr[ORDER];
  FLOAT32 lsf[(2 * NUM_FRAMES + 1) * ORDER];
  FLOAT32 synth_buf_fb[2 * (NUM_SUBFR_SUPERFRAME_BY2 * LEN_SUBFR + LEN_SUPERFRAME)];
  FLOAT32 tbe_synth[2 * LEN_SUPERFRAME];
  FLOAT32 bwe_exc_extended[2 * LEN_FRAME + NL_BUFF_OFFSET_MAX];
  FLOAT32 tmp_tbe[2 * LEN_SUPERFRAME];
  FLOAT32 tbe_synth_align[2 * LEN_SUPERFRAME];
  FLOAT32 voice_factors[MAX_NUM_SUBFR];
  FLOAT32 pitch_buffer[MAX_NUM_SUBFR];
  FLOAT32 synth_rs[8 * LEN_FRAME];
} ia_core_coder_lpd_dec_scr_t;

typedef struct
{
  FLOAT32 synth_buf[2 * (MAX_PITCH + SYNTH_DELAY_LMAX) + LEN_SUPERFRAME];
  FLOAT32 synth_resamp[+2 * LEN_FRAME + L_FILT_MAX];
} ia_core_coder_synth_end_scr_t;

typedef struct
{
  FLOAT32 synth[LEN_SUPERFRAME];
} ia_core_coder_lpd_dec_upd_scr_t;

typedef struct
{
  FLOAT32 synth_stereo[MAX_NUM_CHANNELS][LEN_SUPERFRAME];
  FLOAT32 slpd_scratch[2 * LEN_SUPERFRAME];
  FLOAT32 buffer_lb[(1 + 2 * LEN_FRAME + LEN_SUPERFRAME)];
} ia_core_coder_data_scr_t;

typedef struct
{
  FLOAT32 data_r[LEN_SUPERFRAME * 2];
  FLOAT32 data_i[LEN_SUPERFRAME * 2];
} ia_core_coder_lpc_scratch_t;

IA_ERRORCODE ia_core_coder_tns_apply(ia_usac_data_struct *usac_data, FLOAT32 *spec, WORD32 nbands,
                                     ia_sfb_info_struct *pstr_sfb_info,
                                     ia_tns_frame_info_struct *pstr_tns,
                                     ia_usac_igf_config_struct *igf_config);

IA_ERRORCODE ia_core_coder_tw_buff_update(ia_usac_data_struct *usac_data, WORD32 i,
                                          ia_usac_lpd_decoder_handle st);
VOID ia_core_coder_td_frm_dec(ia_usac_data_struct *usac_data, WORD32 k, WORD32 mod0);

IA_ERRORCODE ia_core_coder_fd_frm_dec(ia_usac_data_struct *usac_data, WORD32 i_ch, WORD32 id);

IA_ERRORCODE ia_core_coder_acelp_mdct(FLOAT32 *ptr_in, FLOAT32 *ptr_out, WORD32 length,
                                      FLOAT32 *ptr_scratch, FLOAT32 *ptr_fft_scratch);

IA_ERRORCODE ia_core_coder_acelp_mdct_main(ia_usac_data_struct *usac_data, FLOAT32 *in,
                                           FLOAT32 *out, WORD32 l, WORD32 m, FLOAT32 *scratch);

IA_ERRORCODE ia_core_coder_fwd_alias_cancel_tool(
    ia_usac_data_struct *usac_data, WORD32 *fac_prm, ia_usac_td_config_handle td_config,
    WORD32 fac_length, FLOAT32 *lp_filt_coeff, FLOAT32 *fac_time_sig, FLOAT32 *fac_time_sig_fb,
    WORD32 fscale, FLOAT32 *tmp_scratch, FLOAT32 *zir, WORD32 stereo_lpd, WORD32 len_subfrm);

IA_ERRORCODE ia_core_coder_reset_acelp_data(ia_usac_data_struct *usac_data,
                                            ia_usac_lpd_decoder_handle st,
                                            ia_usac_td_config_handle td_config,
                                            FLOAT32 *ptr_ola_buff, WORD32 last_was_short,
                                            WORD32 chn);
#endif
