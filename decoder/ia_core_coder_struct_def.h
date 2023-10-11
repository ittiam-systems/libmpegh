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

#ifndef IA_CORE_CODER_STRUCT_DEF_H
#define IA_CORE_CODER_STRUCT_DEF_H

#include <setjmp.h>

#include "impeghd_cicp_defines.h"
#include "impeghd_cicp_struct_def.h"
#include "impeghd_format_conv_defines.h"
#include "impeghd_format_conv_data.h"
#include "ia_core_coder_cnst.h"
#define MAX_OUTPUT_CHANNELS (8)
#define MAX_NUM_OTT (1)

#define MAX_ARBITRARY_TREE_LEVELS (2)
#define MAX_PARAMETER_SETS (8)
#define MAX_ARBITRARY_TREE_INDEX ((1 << (MAX_ARBITRARY_TREE_LEVELS + 1)) - 1)
#define MAX_NUM_QMF_BANDS (64)
#define MAX_NUM_OTT_AT (MAX_OUTPUT_CHANNELS * ((1 << MAX_ARBITRARY_TREE_LEVELS) - 1))
#define MAX_PARAMETER_BANDS (28)
#define MAX_DECOR_CONFIG_IDX (2)

#define MAX_HYBRID_BANDS (MAX_NUM_QMF_BANDS - 3 + 10)

#define MAX_TIME_SLOTS (72)

#define MAX_M2_OUTPUT (8)
#define QMF_BANDS_TO_HYBRID (3)
#define PROTO_LEN (13)
#define BUFFER_LEN_LF (PROTO_LEN - 1 + MAX_TIME_SLOTS)
#define BUFFER_LEN_HF ((PROTO_LEN - 1) / 2)
#define MAX_TIME_SLOTS (72)
#define MAX_NO_TIME_SLOTS_DELAY (14)

typedef struct
{
  UWORD32 ui_pcm_wdsz;
  UWORD32 ui_samp_freq;
  UWORD32 ui_n_channels;
  WORD32 i_channel_mask;
  UWORD32 ui_channel_mode;
  WORD32 ui_effect_type;
  WORD32 ui_target_loudness;
  WORD32 ui_target_loudness_set;
  WORD32 drc_apply;
  WORD32 ui_loud_norm_flag;
  WORD32 ui_cicp_layout_idx;
  UWORD32 header_dec_done;
  UWORD32 ui_max_channels;

  WORD32 ui_mhas_flag;
  WORD8 i_preset_id;
  WORD32 discard_au_preroll;
  WORD32 ui_raw_flag;
  WORD32 ui_binaural_flag;
  WORD32 stream_samp_freq;
  UWORD32 out_samp_freq;
  WORD32 resample_output;
  WORD32 output_framelength;
  WORD32 fac_up;
  WORD32 fac_down;
  WORD32 extrn_rend_flag;
  WORD32 oam_data_present;
  WORD32 oam_md_payload_length;
  UWORD8 *ptr_oam_md_bit_buf;
  WORD32 hoa_data_present;
  WORD32 hoa_md_payload_length;
  UWORD8 *ptr_hoa_md_bit_buf;
  WORD32 ch_data_present;
  WORD32 ch_md_payload_length;
  UWORD8 *ptr_ch_md_bit_buf;
  WORD32 pcm_data_length;
  WORD32 obj_offset;
  WORD32 hoa_offset;
  WORD8 *ptr_ext_ren_pcm_buf;
} ia_mpegh_dec_config_struct;

typedef struct ia_format_conv_state_struct
{
  WORD32 format_in_chan[FC_MAX_CHANNELS * MAX_NUM_SIGNALGROUPS];
  FLOAT32 **stft_in_buf;
  FLOAT32 **stft_out_buf;
  ia_format_conv_param *fc_params;
} ia_format_conv_state_struct;

typedef struct ia_ds_state_struct
{
  FLOAT32 **stft_in_buf;
  FLOAT32 **stft_out_buf;
  ia_ds_param *ds_params;
} ia_ds_state_struct;

typedef struct ia_mpegh_dec_state_struct
{
  ia_mpegh_dec_config_struct *p_config;
  UWORD32 ui_in_bytes;
  UWORD32 ui_out_bytes;
  UWORD32 ui_exec_done;
  UWORD32 sampling_rate;
  UWORD32 extension_samp_rate;
  UWORD32 bit_rate;
  UWORD32 ui_init_done;
  UWORD32 header_dec_done;
  WORD32 frame_counter;
  UWORD32 ch_config;
  struct ia_bit_buf_struct str_bit_buf, *pstr_bit_buf;
  VOID *mpeghd_scratch_mem_v;
  VOID *mpeghd_persistent_mem_v;

  WORD32 i_bytes_consumed;
  WORD32 num_of_output_ch;
  VOID *ia_audio_specific_config;

  UWORD16 *huffman_code_book_scl;
  UWORD32 *huffman_code_book_scl_index;

  ia_mpegh_dec_tables_struct *pstr_mpeghd_tables;

  VOID *pstr_dec_data;

  WORD32 decode_create_done;
  WORD32 delay_in_samples;
  WORD32 is_base_line_profile_3b;
  UWORD8 preroll_config_prev[288]; // max of escapedValue(4, 4, 8) i.e. 2^4 -1  + 2^4 -1 + 2^8 -1;
  WORD32 preroll_config_present;
  ia_format_conv_state_struct state_format_conv;
  ia_ds_state_struct state_domain_switcher;
  jmp_buf *xmpeghd_jmp_buf;
  WORD8 prev_cfg_data[MAX_CFG_DATA_LENGTH];
  WORD32 prev_cfg_len;
  WORD32 flush;
} ia_mpegh_dec_state_struct;

typedef struct ia_mpegh_dec_api_struct
{
  ia_mpegh_dec_state_struct *p_state_mpeghd;
  ia_mpegh_dec_config_struct mpeghd_config;
  ia_mem_info_struct *p_mem_info_mpeghd;
  pVOID *pp_mem_mpeghd;
  ia_mpegh_dec_tables_struct mpeghd_tables;
} ia_mpegh_dec_api_struct;

IA_ERRORCODE ia_core_coder_decoder_flush_api(ia_mpegh_dec_api_struct *p_obj_mpegh_dec);

IA_ERRORCODE ia_core_coder_dec_init(ia_mpegh_dec_api_struct *p_obj_mpegh_dec);

IA_ERRORCODE ia_core_coder_dec_execute(ia_mpegh_dec_api_struct *p_obj_mpegh_dec);

#endif /* IA_CORE_CODER_STRUCT_DEF_H */
