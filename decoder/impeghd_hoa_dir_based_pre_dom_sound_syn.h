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

#ifndef __IA_MPEGH_HOA_DIR_BASED_PRE_DOM_SOUND_SYN_H__
#define __IA_MPEGH_HOA_DIR_BASED_PRE_DOM_SOUND_SYN_H__

IA_ERRORCODE impeghd_hoa_compute_hoa_representation_of_spat_pred(
    ia_spatial_dec_str *dec_handle, const pUWORD32 prediction_type_vec,
    const pUWORD32 prediction_indices_mat, const pFLOAT32 prediction_factors_mat,
    const pFLOAT32 ptr_fade_win, pFLOAT32 ptr_pred_grid_dir_sigs,
    WORD32 *spat_prediction_in_curr_frame);

IA_ERRORCODE impeghd_hoa_compute_hoa_repr_of_dir_sigs(ia_spatial_dec_str *dec_handle,
                                                      pUWORD32 sig_indices_set);

IA_ERRORCODE
impeghd_hoa_compute_smoothed_hoa_representation_of_spat_pred_sigs(ia_spatial_dec_str *dec_handle);

IA_ERRORCODE
impeghd_hoa_dir_based_pre_dom_sound_syn_process(ia_spatial_dec_str *dec_handle,
                                                pUWORD32 sig_indices_set,
                                                pFLOAT32 out_dir_based_pre_dom_sounds);
#endif //__IA_MPEGH_HOA_DIR_BASED_PRE_DOM_SOUND_SYN__
