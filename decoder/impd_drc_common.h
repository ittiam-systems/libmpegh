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

#ifndef IMPD_DRC_COMMON_H
#define IMPD_DRC_COMMON_H

#include "ia_core_coder_constants.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUDIO_CODEC_FRAME_SIZE_MAX 1024
#define AUDIO_CODEC_SUBBAND_COUNT_MAX 256
#define AUDIO_CODEC_SUBBAND_COUNT_QMF64 64
#define AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR_QMF64 64
#define AUDIO_CODEC_SUBBAND_COUNT_QMF71 71
#define AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR_QMF71 64
#define AUDIO_CODEC_SUBBAND_COUNT_STFT256 256
#define AUDIO_CODEC_SUBBAND_DOWNSAMPLING_FACTOR_STFT256 256
#define BAND_COUNT_MAX (4)

#define DELAY_MODE_REGULAR 0
#define DELAY_MODE_LOW 1
#define DELAY_SAMPLES 128
#define DOWNMIX_COEFF_COUNT_MAX (32 * 32)
#define DOWNMIX_ID_COUNT_MAX (8)
#define DOWNMIX_INSTRUCTION_COUNT_MAX 16
#define DRC_CODEC_FRAME_SIZE_MAX (AUDIO_CODEC_FRAME_SIZE_MAX / 8)
#define DRC_COEFF_COUNT_MAX 4
#define DRC_INSTRUCTIONS_COUNT_MAX (32)
#define EXT_COUNT_MAX (2)
#define FILT_SLOPE (-24.0f)

#define ID_FOR_BASE_LAYOUT 0x0
#define ID_FOR_ANY_DOWNMIX 0x7F
#define ID_FOR_NO_DRC 0x0
#define ID_FOR_ANY_DRC 0x3F
#define INV_LOG10_2 3.32192809f

#define LOCATION_SELECTED 0x1
#define LOUDNESS_INFO_COUNT_MAX (DOWNMIX_INSTRUCTION_COUNT_MAX + 20)

#define MAX_BS_BUF_SIZE 768
#define MAX_CHANNEL_COUNT (16)
#define MAX_GAIN_ELE_COUNT 15
#define MAX_LOUDNESS_INFO_COUNT (16)
#define MAX_NUM_COMPRESSION_EQ (16)
#define MAX_NUM_DOWNMIX_ID_REQUESTS 15
#define MAX_NUM_DRC_FEATURE_REQUESTS 7
#define MAX_NUM_DRC_EFFECT_TYPE_REQUESTS 15
#define MAX_NUM_GROUP_ID_REQUESTS 16
#define MAX_NUM_GROUP_PRESET_ID_REQUESTS 15
#define MAX_NUM_MEMBERS_GROUP_PRESET 15
#define MAX_PARAM_DRC_LOOK_AHEAD_VAL 127
#define MAX_SAMP_FREQ_IN_KHZ 96
#define MAX_ATTACK_VAL (MAX_PARAM_DRC_LOOK_AHEAD_VAL * MAX_SAMP_FREQ_IN_KHZ)
#define MAX_SEC_LEN 110
#define MAX_NUMBUF_SEC_VAL (MAX_ATTACK_VAL / MAX_SEC_LEN)
#define MAXPACKETLOSSTIME 2.5f
#define MAX_SIGNAL_DELAY (1024 + 512)
#define MAX_SIGNATURE_DATA_LENGTH_PLUS_ONE 256
#define MAX_SUBBAND_CHANNEL_COUNT (128)
#define MEASUREMENT_COUNT_MAX 16
#define MIN_DRC_SAMP_FREQ 1000

#define N_DELTA_TIME_CODE_TABLE_ENTRIES_MAX (512 + 14)
#define NUM_ELE_IN_CPLX_NUM 2
#define NUM_GAIN_DEC_INSTANCES 2
#define NODE_COUNT_MAX DRC_CODEC_FRAME_SIZE_MAX
#define MAX_NODE 32

#define SELECTION_CANDIDATE_COUNT_MAX 32
#define SEL_DRC_COUNT (3)
#define SEQUENCE_COUNT_MAX (24)
#define SLOPE_FACTOR_DB_TO_LINEAR 0.1151f
#define SPEAKER_POS_COUNT_MAX (8)
#define SUB_DRC_COUNT 4
#define SUBBAND_DOMAIN_MODE_OFF 0
#define SUBBAND_DOMAIN_MODE_QMF64 1
#define SUBBAND_DOMAIN_MODE_QMF71 2
#define SUBBAND_DOMAIN_MODE_STFT256 3

#define UNDEFINED_LOUDNESS_VALUE 1000.0f
#define UNIDRCGAINEXT_TERM 0x0
#define UNIDRCLOUDEXT_TERM 0x0
#define UNIDRCCONFEXT_TERM 0x0
#define UNIDRCINTERFACEEXT_TERM 0x0

#define CHANNEL_GROUP_COUNT_MAX SEQUENCE_COUNT_MAX
#define GAIN_SET_COUNT_MAX SEQUENCE_COUNT_MAX

#ifndef bool
#define bool WORD32
#endif

typedef struct ia_drc_sel_proc_output_struct
{
  WORD32 group_id[LOUDNESS_INFO_COUNT_MAX];
  WORD32 sel_drc_set_ids[SUB_DRC_COUNT];
  WORD32 sel_downmix_ids[SUB_DRC_COUNT];

  FLOAT32 downmix_matrix[MAX_SUBBAND_CHANNEL_COUNT][MAX_SUBBAND_CHANNEL_COUNT];
  FLOAT32 group_id_loudness[LOUDNESS_INFO_COUNT_MAX];

  WORD32 active_downmix_id;
  WORD32 base_channel_count;
  WORD32 downmix_matrix_present;
  WORD32 drc_characteristic_target;
  WORD32 group_id_loudness_count;
  WORD32 num_sel_drc_sets;
  WORD32 target_channel_count;
  WORD32 target_layout;

  FLOAT32 boost;
  FLOAT32 compress;
  FLOAT32 loud_norm_gain_db;
  FLOAT32 output_loudness;
  FLOAT32 output_peak_level_db;
} ia_drc_sel_proc_output_struct;

#ifdef __cplusplus
}
#endif
#endif
