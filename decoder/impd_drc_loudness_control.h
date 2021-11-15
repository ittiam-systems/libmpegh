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

#ifndef IMPD_DRC_LOUDNESS_CONTROL_H
#define IMPD_DRC_LOUDNESS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

VOID impd_drc_signal_peak_level_info(
    WORD32 *peak_info_count, WORD32 eq_set_id[], FLOAT32 signal_peak_level[],
    WORD32 explicit_peak_information_present[], ia_drc_config *pstr_drc_config,
    ia_drc_loudness_info_set_struct *pstr_loudness_info,
    ia_drc_instructions_struct *str_drc_instruction_str, WORD32 requested_dwnmix_id,
    WORD32 album_mode, WORD32 num_compression_eq_count, WORD32 *num_compression_eq_id);

IA_ERRORCODE
impd_drc_loudness_pk_to_avg_info(FLOAT32 *loudness_peak_2_avg_value,
                                 WORD32 *loudness_peak_2_avg_value_present,
                                 ia_drc_loudness_info_set_struct *pstr_loudness_info,
                                 ia_drc_instructions_struct *str_drc_instruction_str,
                                 WORD32 requested_dwnmix_id, WORD32 dyn_range_measurement_type,
                                 WORD32 album_mode);

IA_ERRORCODE
impd_drc_find_high_pass_loudness_adjust(WORD32 *loudness_hp_adjust_flag,
                                        FLOAT32 *loudness_hp_adjust,
                                        ia_drc_loudness_info_set_struct *pstr_loudness_info,
                                        WORD32 requested_dwnmix_id, WORD32 drc_set_id_requested,
                                        WORD32 album_mode, FLOAT32 device_cutoff_freq);

IA_ERRORCODE impd_drc_init_loudness_control(
    WORD32 *loudness_info_count, WORD32 eq_set_id[], FLOAT32 loud_norm_gain_db[],
    FLOAT32 loudness[], ia_drc_sel_proc_params_struct *pstr_drc_sel_proc_params_struct,
    ia_drc_loudness_info_set_struct *pstr_loudness_info, WORD32 requested_dwnmix_id,
    ia_drc_instructions_struct *str_drc_instruction_str, WORD32 num_compression_eq_count,
    WORD32 *num_compression_eq_id);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
