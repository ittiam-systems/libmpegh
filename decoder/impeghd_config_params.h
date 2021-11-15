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

#ifndef IMPEGHD_CONFIG_H
#define IMPEGHD_CONFIG_H

/* Input Config Params default values */
#define IMPEGHD_CONFIG_PARAM_PCM_WD_SZ_DFLT_VAL (16)
#define IMPEGHD_CONFIG_PARAM_MAX_CHANS_DFLT_VAL (24)
#define IMPEGHD_CONFIG_PARAM_MHAS_FLAG_DFLT_VAL (1)
#define IMPEGHD_CONFIG_PARAM_EFFECT_TYPE_DFLT_VAL (-1)
#define IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL (0)
#define IMPEGHD_CONFIG_PARAM_TGT_LOUDNESS_DFLT_VAL (-24)
#define IMPEGHD_CONFIG_PARAM_LOUD_NORM_FLG_DFLT_VAL (0)
#define IMPEGHD_CONFIG_PARAM_PRESET_ID_DFLT_VAL (-1)
#define IMPEGHD_CONFIG_PARAM_PRESET_ID_MAX_VAL (32) // MAE_MAX_NUM_GROUPS
#define IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL (0)
#define IMPEGHD_CONFIG_PARAM_BR_FLAG_DFLT_VAL (0)
#define IMPEGHD_CONFIG_PARAM_MAE_FLAG_DFLT_VAL (0)
#define IMPEGHD_CONFIG_PARAM_OUT_FS_DFLT_VAL (48000)
#endif /* IMPEGHD_CONFIG_H */
