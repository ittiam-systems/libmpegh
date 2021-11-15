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

#ifndef IMPD_DRC_INIT_H
#define IMPD_DRC_INIT_H

#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_common.h"
#include "impd_drc_struct.h"
#include "impd_drc_interface.h"
#include "impd_drc_gain_dec.h"
#include "impeghd_error_codes.h"
#include "impd_drc_filter_bank.h"
#include "impd_drc_multi_band.h"
#include "impd_drc_process_audio.h"
#include "impd_drc_peak_limiter.h"
#include "impd_drc_selection_process.h"

VOID impd_drc_update_effect_type_request(WORD8 *, WORD8 *, UWORD64);

IA_ERRORCODE impd_drc_init_bitstream_dec(ia_drc_bits_dec_struct *, WORD32, WORD32, WORD32, WORD32,
                                         WORD32 *);
IA_ERRORCODE impd_drc_init_bitstream_config(ia_drc_config *pstr_drc_config);

IA_ERRORCODE impd_drc_init_selected_drc_set(ia_drc_params_struct *ia_drc_params_struct,
                                            ia_filter_banks_struct *ia_filter_banks_struct,
                                            ia_drc_overlap_params_struct *pstr_overlap_params,
                                            WORD32 audio_num_chan, WORD32 drc_set_id_selected,
                                            WORD32 downmix_id_selected,
                                            ia_drc_config *drc_config);

IA_ERRORCODE impd_drc_init_decode(ia_drc_gain_dec_struct *p_drc_gain_dec_structs,
                                  WORD32 frame_size, WORD32 sample_rate,
                                  WORD32 gain_delay_samples, WORD32 delay_mode,
                                  WORD32 sub_band_domain_mode);

IA_ERRORCODE impd_drc_init_decode_post_config(ia_drc_gain_dec_struct *p_drc_gain_dec_structs,
                                              WORD32 audio_num_chan, WORD32 *drc_set_id_processed,
                                              WORD32 *downmix_id_processed,
                                              WORD32 num_sets_processed, WORD32 channel_offset,
                                              WORD32 num_ch_process,
                                              ia_drc_config *pstr_drc_config, pVOID *mem_ptr);
#endif /* IMPEGHD_UNI_DRC_STRUCT_H */
