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

#ifndef __IMPEGHD_EXT_REND_INTRFC_H__
#define __IMPEGHD_EXT_REND_INTRFC_H__

IA_ERRORCODE impeghd_write_oam_meta_data_for_ext_ren(
    ia_write_bit_buf_struct *pstr_bit_buf, ia_usac_config_struct *pstr_usac_cfg,
    ia_obj_ren_dec_state_struct *pstr_obj_ren_dec_state, ia_signals_3d *pstr_signals_3d,
    ia_enh_obj_md_frame_str *pstr_enh_oam_frm, ia_mae_audio_scene_info *pstr_mae_asi);

IA_ERRORCODE impeghd_write_ch_meta_data_for_ext_ren(ia_write_bit_buf_struct *pstr_bit_buf,
                                                    ia_usac_config_struct *pstr_usac_cfg,
                                                    ia_signals_3d *pstr_signals_3d,
                                                    WORD32 ui_cicp_idx);

IA_ERRORCODE impeghd_write_hoa_meta_data_for_ext_ren(ia_write_bit_buf_struct *pstr_bit_buf,
                                                     ia_usac_config_struct *pstr_usac_cfg,
                                                     ia_signals_3d *pstr_signals_3d,
                                                     ia_mae_audio_scene_info *pstr_mae_asi);

#endif /* __IMPEGHD_EXT_REND_INTRFC_H__ */