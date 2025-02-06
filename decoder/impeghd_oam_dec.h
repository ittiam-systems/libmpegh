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

#ifndef IMPEGHD_OAM_DEC_H
#define IMPEGHD_OAM_DEC_H

IA_ERRORCODE impeghd_obj_md_low_delay_dec(ia_oam_dec_state_struct *ptr_oam_dec_state,
                                          ia_oam_dec_config_struct *p_obj_md_cfg,
                                          ia_bit_buf_struct *ptr_bit_buf);

IA_ERRORCODE impeghd_obj_md_cfg(ia_oam_dec_config_struct *p_obj_md_cfg,
                                ia_bit_buf_struct *ptr_bit_buf, WORD32 cc_frame_len,
                                WORD32 num_objects);

IA_ERRORCODE impeghd_obj_md_dec(ia_oam_dec_state_struct *ptr_oam_dec_state,
                                ia_oam_dec_config_struct *p_obj_md_cfg,
                                ia_bit_buf_struct *ptr_bit_buf);

VOID impeghd_descale_ld_obj_md(ia_oam_dec_state_struct *ptr_oam_dec_state,
                               ia_oam_dec_config_struct *p_obj_md_cfg);

IA_ERRORCODE impeghd_enh_obj_md_config(ia_enh_oam_config_struct *p_enh_obj_md_cfg,
                                       ia_bit_buf_struct *ptr_bit_buf, WORD32 num_objects);

IA_ERRORCODE impeghd_enh_obj_md_frame(ia_enh_obj_md_frame_str *p_enh_oj_md_frame,
                                      ia_bit_buf_struct *ptr_bit_buf, WORD32 num_objects,
                                      WORD32 independency_flag);

#endif /* IMPEGHD_OAM_DEC_H */
