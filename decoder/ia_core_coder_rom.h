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

#ifndef IA_CORE_CODER_ROM_H
#define IA_CORE_CODER_ROM_H
#define ARITH_HASH_TABLE_SIZE 742
typedef struct
{
  ia_sampling_rate_info_struct str_sample_rate_info[13];

  UWORD16 huffman_code_book_scl[122];
  UWORD32 huffman_code_book_scl_index[33];
} ia_mpegh_dec_huffman_tables_struct;

extern const ia_mpegh_dec_huffman_tables_struct ia_core_coder_mpeghd_huffmann_tables;
typedef struct
{
  WORD32 tns_max_bands_tbl_usac[16][2];

} ia_mpegh_dec_block_tables_struct;

extern const ia_mpegh_dec_block_tables_struct ia_core_coder_mpeghd_block_tables;
typedef struct
{
  ia_mpegh_dec_block_tables_struct *pstr_block_tables;
  ia_mpegh_dec_huffman_tables_struct *pstr_huffmann_tables;
} ia_mpegh_dec_tables_struct;

extern const WORD32 ia_core_coder_harmonic_spacing[256];

extern const UWORD16 ia_core_coder_fdp_exp[182];
extern const UWORD16 ia_core_coder_fdp_scf[63];
extern const UWORD16 ia_core_coder_fdp_s1[129];
extern const WORD16 ia_core_coder_fdp_s2[129];

extern const WORD32 ia_core_coder_esc_nb_offset[8];

extern const FLOAT32 ia_core_coder_pow_table[1024];
extern const FLOAT32 ia_core_coder_pow_2_table[64];

extern const FLOAT32 ia_core_coder_pow_2_inv_table[64];

extern const FLOAT32 ia_core_coder_pow_14_3[8];
extern const FLOAT32 ia_pow_10_20_slpd_apply[2][101];

extern const UWORD16 ia_core_coder_ari_cf_r[3][4];

extern const UWORD16 ia_core_coder_ari_lookup_m[ARITH_HASH_TABLE_SIZE];

extern const UWORD32 ia_core_coder_ari_hash_m[ARITH_HASH_TABLE_SIZE];

extern const UWORD8 ia_core_coder_ari_hash_m_1[ARITH_HASH_TABLE_SIZE];
extern const UWORD16 ia_core_coder_ari_cf_m[64][17];

extern const FLOAT32 ia_core_coder_mdst_fcoeff_long_sin[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_long_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_long_sin_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_long_kbd_sin[];
extern const FLOAT32 *const ia_core_coder_mdst_fcoeff_longshort_curr[2][2];

extern const FLOAT32 ia_core_coder_mdst_fcoeff_start_sin[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_start_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_start_sin_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_start_kbd_sin[];

extern const FLOAT32 *const ia_core_coder_mdst_fcoeff_start_curr[2][2];

extern const FLOAT32 ia_core_coder_mdst_fcoeff_stop_sin[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stop_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stop_sin_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stop_kbd_sin[];

extern const FLOAT32 *const ia_core_coder_mdst_fcoeff_stop_cur[2][2];

extern const FLOAT32 ia_core_coder_mdst_fcoeff_stopstart_sin[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stopstart_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stopstart_sin_kbd[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stopstart_kbd_sin[];

extern const FLOAT32 *const ia_core_coder_mdst_fcoeff_stopstart_cur[2][2];

extern const FLOAT32 ia_core_coder_mdst_fcoeff_l_s_start_left_sin[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_l_s_start_left_kbd[];

extern const FLOAT32 ia_core_coder_mdst_fcoeff_stop_stopstart_left_sin[];
extern const FLOAT32 ia_core_coder_mdst_fcoeff_stop_stopstart_left_kbd[];

extern const FLOAT32 *const ia_core_coder_mdst_fcoeff_l_s_start_left_prev[2];

extern const FLOAT32 *const ia_core_coder_mdst_fcoeff_stop_stopstart_left_prev[2];

extern const UWORD16 ia_core_coder_cf_se_01[27];
extern const UWORD16 ia_core_coder_cf_se_10[27];
extern const UWORD16 ia_core_coder_cf_se_02[7][27];
extern const UWORD16 ia_core_coder_cf_se_20[7][27];
extern const UWORD16 ia_core_coder_cf_se_11[7][7][27];
extern const WORD16 ia_core_coder_cf_offset_se_02[7];
extern const WORD16 ia_core_coder_cf_offset_se_20[7];
extern const WORD16 ia_core_coder_cf_offset_se_11[7][7];
extern const UWORD16 ia_core_coder_cf_for_bit[2];

extern const WORD32 ia_sampling_rate_tbl[];
extern const FLOAT32 ia_ltpf_filter_coef1[2][8];
extern const FLOAT32 ia_ltpf_filter_coef2[4][7];
extern const FLOAT32 ia_core_coder_tnf_acf_win[8];

extern const UWORD32 ia_core_coder_pow10_gain_div28[128];
extern const WORD16 ia_core_coder_max_band[2][4];
extern const WORD16 ia_core_coder_ild_qtable[];
extern const FLOAT32 ia_core_coder_pow_igf_whitening[64];
extern const FLOAT32 ia_core_coder_pow_neg_igf_whitening[64];
extern const FLOAT32 ia_core_coder_res_pred_gain_qtable[];
extern const WORD32 ia_pcm_bits_per_sample_table[];
extern const WORD32 ia_pcm_frame_size_tbl[];

extern const ia_usac_samp_rate_info ia_core_coder_samp_rate_info[];
extern const WORD32 ia_core_coder_sampling_boundaries[(1 << LEN_SAMP_IDX)];

extern const WORD16 ia_core_coder_band_lim_erb2[];
extern const WORD16 ia_core_coder_band_lim_erb4[];

extern const FLOAT32 ia_core_coder_slpd_sintable_336[336 + 1];
extern const FLOAT32 ia_core_coder_slpd_sintable_168[168 + 1];

extern const FLOAT32 ia_core_coder_sin_window160[160];
extern const FLOAT32 ia_core_coder_sin_window80[80];

extern const FLOAT32 ia_core_coder_pre_post_twid_cos_48[48];
extern const FLOAT32 ia_core_coder_pre_post_twid_sin_48[48];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_64[64];
extern const FLOAT32 ia_core_coder_pre_post_twid_sin_64[64];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_384[384];
extern const FLOAT32 ia_core_coder_pre_post_twid_sin_384[384];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_512[512];
extern const FLOAT32 ia_core_coder_pre_post_twid_sin_512[512];
extern const FLOAT32 ia_core_coder_pre_twid_type2_dct_dst_cos_1024[1024];
extern const FLOAT32 ia_core_coder_pre_twid_type2_dct_dst_sin_1024[1024];

extern const FLOAT32 ia_core_coder_kbd_win16[16];
extern const FLOAT32 ia_core_coder_kbd_win4[4];
extern const FLOAT32 ia_core_coder_kbd_win1024[1024];

extern const FLOAT32 ia_core_coder_kbd_win960[960];
extern const FLOAT32 ia_core_coder_kbd_win768[768];
extern const FLOAT32 ia_core_coder_kbd_win192[192];
extern const FLOAT32 ia_core_coder_kbd_win96[96];
extern const FLOAT32 ia_core_coder_kbd_win48[48];

extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_24[4][24];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_32[4][32];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_48[4][48];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_64[4][64];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_96[4][96];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_128[4][128];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_192[4][192];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_256[4][256];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_384[4][384];
extern const FLOAT32 ia_core_coder_pre_post_twid_cos_sin_512[4][512];

extern const FLOAT32 ia_core_coder_sine_window64[64];
extern const FLOAT32 ia_core_coder_sine_window384[384];
extern const FLOAT32 ia_core_coder_sine_window512[512];
extern const FLOAT32 ia_core_coder_kbd_window64[64];
extern const FLOAT32 ia_core_coder_kbd_window96[96];
extern const FLOAT32 ia_core_coder_kbd_window128[128];
extern const FLOAT32 ia_core_coder_kbd_window192[192];
extern const FLOAT32 ia_core_coder_kbd_window256[256];
extern const FLOAT32 ia_core_coder_kbd_window384[384];
extern const FLOAT32 ia_core_coder_kbd_window512[512];
extern const FLOAT32 ia_core_coder_sine_window96[96];
extern const FLOAT32 ia_core_coder_sine_window128[128];
extern const FLOAT32 ia_core_coder_sine_window192[192];
extern const FLOAT32 ia_core_coder_sine_window256[256];

extern const FLOAT32 ia_core_coder_dico_lsf_abs_8b_flt[];
extern const WORD32 ia_core_coder_sample_freq_idx_table[17];

extern const UWORD8 ia_core_coder_num_bits_acelp_coding[8][4];
extern const FLOAT32 ia_core_coder_gamma_table[17];

extern const WORD16 ia_core_coder_book_scl_code_book[];
extern const WORD32 ia_core_coder_book_scl_index[];

extern const ia_huff_code_word_struct ia_core_coder_book_scl_huff_code_word[];

extern const FLOAT32 ia_core_coder_weight_table_avq[];

extern const FLOAT32 ia_core_coder_pow_10_i_by_128[128];

#endif /*IMPEGHD_ROM_H*/
