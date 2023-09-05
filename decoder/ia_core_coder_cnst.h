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

#ifndef IA_CORE_CODER_CNST_H
#define IA_CORE_CODER_CNST_H
#define BITS_FOR_SD_DATA 27
#define BITS_FOR_PSDI_DATA 45

/* Below values will be used for writing */
/* 24-bit PCM data                       */
#define MIN_FLT_VAL_24 (-8388608.0f)
#define MAX_FLT_VAL_24 (8388607.0f)
#define MUL_FAC_PCM_24 (256)
/* Below values will be used for writing */
/* 16-bit PCM data                       */
#define MIN_FLT_VAL_16 (-32768.0f)
#define MAX_FLT_VAL_16 (32767.0f)
/* Below values will be used for writing */
/* 32-bit PCM data                       */
#define MIN_FLT_VAL_32 (-2147483647.0f)
#define MAX_FLT_VAL_32 (2147483647.0f)
#define MUL_FAC_PCM_32 (65536)

#define ESC_VAL_BITS_2 3      // 0000 0011
#define ESC_VAL_BITS_4 15     // 0000 1111
#define ESC_VAL_BITS_8 255    // 1111 1111
#define ESC_VAL_BITS_16 65535 // 1111 1111 1111 1111

#define QUANT_SPEC_MAX 31775
#define QUANT_SPEC_MIN -31775
#define UNCODED_LINES 172
#define CICP2GEOMETRY_MAX_SPKIDX (21)
#define MAX_ELEMENTS 68

#define LEN_SUPERFRAME 1024
#define LEN_FRAME 256
#define NUM_FRAMES 4
#define MAX_NUM_SUBFR 4

#define ORDER 16
#define ORDER_BY_2 (ORDER / 2)

#define FAC_LENGTH 128
#define BPF_SUBFRAME 1

#define SAMPLE_BOUNDARY 11
#define NUM_SUBFR_SUPERFRAME 16
#define NUM_SUBFR_SUPERFRAME_BY2 (NUM_SUBFR_SUPERFRAME / 2)
#define SYNTH_DELAY_LMAX ((NUM_SUBFR_SUPERFRAME_BY2 - 1) * LEN_SUBFR)

#define FSCALE_DENOM 12800
#define FSCALE_DENOM_BY_2 6400
#define FAC_FSCALE_MAX 24000
#define FAC_FSCALE_MIN 6000
#define FSCALE_MAX 32000
#define MAX_SAMPLING_RATE (48000)
#define FILTER_DELAY 12
#define DEFAULT_ATTACK_TIME_MS (5.0f)
#define DEFAULT_ATTACK_TIME_MS_INT (5)
#define DEFAULT_RELEASE_TIME_MS (50.0f)
#define DEFAULT_RELEASE_TIME_MS_INT (50)
#define PEAK_LIM_SIZE ((DEFAULT_ATTACK_TIME_MS_INT * MAX_SAMPLING_RATE / 1000) + 1)
#define LIM_DEFAULT_THRESHOLD (0.89125094f)
#define TMIN 34
#define TFR2 (162 - TMIN)
#define TFR1 160
#define TMAX (27 + 6 * TMIN)

#define UP_SAMP 4
#define INTER_LP_FIL_ORDER 16
#define INTER_LP_FIL_LEN (UP_SAMP * INTER_LP_FIL_ORDER + 1)

#define MAX_PITCH                                                                                \
  (TMAX + (6 * ((((FAC_FSCALE_MAX * TMIN) + (FSCALE_DENOM / 2)) / FSCALE_DENOM) - TMIN)))

#ifndef PI
#define PI 3.14159265358979323846264338327950288
#endif
#define PIx2 2 * PI
#define PI_BY_6400 (PI / 6400.0)
#define ONEBYPIx2 0.5f * (PI_INV)
#define LPC_ORDER_TBE_MINUS1 LPC_ORDER_TBE - 1
#define TBE_DELTA 0.0234952f
#define TBE_DELTA_DEV 0.0235186952f

#define PREEMPH_FILT_FAC 0.68f

#define IGAMMA1 1975684956

#define TILT_CODE 0.3f

#define BLOCK_LEN_LONG 1024
#define BLOCK_LEN_SHORT 128
#define BLOCK_LEN_LONG_S 960
#define BLOCK_LEN_SHORT_S 120

#define WIN_LEN_1024 1024
#define WIN_LEN_768 768
#define WIN_LEN_512 512
#define WIN_LEN_192 192
#define WIN_LEN_128 128
#define WIN_LEN_960 960
#define WIN_LEN_480 480
#define WIN_LEN_256 256
#define WIN_LEN_120 120
#define WIN_LEN_96 96
#define WIN_LEN_64 64
#define WIN_LEN_48 48
#define WIN_LEN_16 16
#define WIN_LEN_4 4

#define TW_IPLEN2S 12
#define TW_OS_FACTOR_WIN 16

#define WIN_SEL_0 0
#define WIN_SEL_1 1
#define WIN_SEL_2 2

#define NUM_TW_NODES 16

#define CORE_MODE_FD 0

#define ONLY_LONG_SEQUENCE 0
#define LONG_START_SEQUENCE 1
#define EIGHT_SHORT_SEQUENCE 2
#define LONG_STOP_SEQUENCE 3
#define STOP_START_SEQUENCE 4
#define NUM_WIN_SEQ 5

#define NSFB_SHORT 16
#define NSFB_LONG 53
#define MAX_SHORT_IN_LONG_BLOCK 8
#define MAX_SHORT_WINDOWS 8

#define CORE_LEFT 0
#define CORE_RIGHT 1

#define MAX_NUM_ELEMENTS 50
#define MAX_NUM_CHANNELS 24
#define MAX_NUM_CHANNELS_LVL1 5
#define MAX_NUM_CHANNELS_LVL2 9
#define MAX_NUM_CHANNELS_LVL3 16
#ifdef LC_LEVEL_4
#define MAX_NUM_CHANNELS_LVL4 (28)
#endif
#define MAX_NUM_CHANNELS_USAC_LVL2 24

#define SFB_NUM_MAX ((NSFB_SHORT + 1) * MAX_SHORT_IN_LONG_BLOCK)

#define MAX_31 (WORD32)0x3FFFFFFF
#define MIN_31 (WORD32)0xC0000000
#define LEN_SUBFR 64

#define SFB_PER_PRED_BAND 2

#define MAX_CC_CHANNEL_NUM (2)
#define MEAN_BUF_LEN 3
#define MAX_BS_ELEMENT (8 + MAX_CC_CHANNEL_NUM)
/* ltpf */
#define LTPF_MAX_DELAY (3)
#define LTPF_NUM_LPC_COEFFS (24)
#define LTPF_ATT_FAC (0.95f)
#define LTPF_NUM_FILT_COEF1 (8)
#define LTPF_NUM_FILT_COEF1_BY2 (4)
#define LTPF_NUM_FILT_COEF2 (7)
#define LTPF_PITCH_MIN_96K (256)
#define LTPF_PITCH_MAX_96K (54 + 6 * LTPF_PITCH_MIN_96K)
#define LTPF_MAX_BS_SAMPLES (1024) /*Max Block Size in samples*/
#define LTPF_MAX_TRANS_LEN (LTPF_MAX_BS_SAMPLES >> 3)
#define LTPF_SCRATCH_SIZE                                                                        \
  (((LTPF_NUM_FILT_COEF2 - 1 + LTPF_MAX_BS_SAMPLES) +                                            \
    (LTPF_PITCH_MAX_96K + LTPF_NUM_FILT_COEF1 / 2 + LTPF_MAX_BS_SAMPLES) +                       \
    (LTPF_MAX_TRANS_LEN) + (LTPF_MAX_TRANS_LEN + LTPF_NUM_LPC_COEFFS)) *                         \
   sizeof(WORD32))

#define FD_NOISE_SHAPING_RES (64)
#define BPF_DELAY (1) /* Bass postfilter delay (subframe) */
#define L_EXTRA (96)
#define LPD_DELAY (NUM_SUBFR_SUPERFRAME_BY2)
#define MAX_NUM_DM_ID 8

#endif /* IA_CORE_CODER_CNST_H */
