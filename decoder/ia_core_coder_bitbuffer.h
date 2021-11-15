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

#ifndef IA_CORE_CODER_BITBUFFER_H
#define IA_CORE_CODER_BITBUFFER_H

#include <setjmp.h>

typedef struct ia_bit_buf_struct
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;

  UWORD8 *ptr_read_next;

  WORD32 bit_pos;
  WORD32 cnt_bits;

  WORD32 size;
  WORD32 bit_count;
  WORD32 valid_bits;
  UWORD8 byte;
  UWORD8 *byte_ptr;
  UWORD8 *ptr_start;
  WORD32 write_bit_count;
  WORD32 max_size;
  jmp_buf *xmpeghd_jmp_buf;

} ia_bit_buf_struct;

typedef struct ia_bit_buf_struct *ia_handle_bit_buf_struct;

ia_bit_buf_struct *ia_core_coder_create_bit_buf(ia_bit_buf_struct *it_bit_buff,
                                                UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size);

VOID ia_core_coder_create_init_bit_buf(ia_bit_buf_struct *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                       WORD32 bit_buf_size);

WORD32 ia_core_coder_read_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);
UWORD32 ia_core_coder_read_bits_buf_32(ia_bit_buf_struct *it_bit_buff);
WORD32 ia_core_coder_skip_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);

WORD32 ia_core_coder_show_bits_buf(ia_bit_buf_struct *it_bit_buff, WORD no_of_bits);

VOID ia_core_coder_mpeghd_read_byte_corr1(UWORD8 **ptr_read_next, WORD32 *ptr_bit_pos,
                                          WORD32 *readword, UWORD8 *p_bit_buf_end);

#define get_no_bits_available(it_bit_buff) ((it_bit_buff)->cnt_bits)
#define ia_core_coder_no_bits_read(it_bit_buff) ((it_bit_buff)->size - (it_bit_buff)->cnt_bits)

typedef struct
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_read_next;
  UWORD8 *ptr_write_next;

  WORD32 read_position;
  WORD32 write_position;
  WORD32 cnt_bits;
  WORD32 size;

} ia_write_bit_buf_struct;

WORD32 impeghd_create_write_bit_buffer(ia_write_bit_buf_struct *it_bit_buf,
                                       UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size,
                                       WORD32 init);

UWORD8 impeghd_write_bits_buf(ia_write_bit_buf_struct *it_bit_buf, UWORD32 write_val,
                              UWORD8 num_bits);

WORD32 impeghd_write_escape_value(ia_write_bit_buf_struct *it_bit_buff, UWORD32 value,
                                  UWORD32 no_bits1, UWORD32 no_bits2, UWORD32 no_bits3);

#endif /* IA_CORE_CODER_BITBUFFER_H */
