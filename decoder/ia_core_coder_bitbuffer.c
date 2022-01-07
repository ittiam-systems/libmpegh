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

#include <string.h>

#include <impeghd_type_def.h>
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_constants.h"
#include <ia_core_coder_basic_op.h>
#include "ia_core_coder_intrinsics.h"
#include "impeghd_error_codes.h"
#include "impeghd_intrinsics_flt.h"

/**
 * @defgroup CoreDecBSParse Core Decoder Bitstream Reading Utility
 * @ingroup  CoreDecBSParse
 * @brief Core Decoder Bitstream Reading Utility
 *
 * @{
 */

/**
 *  ia_core_coder_skip_bits_buf
 *
 *  \brief Helper function to skip bits in bit buffer.
 *
 *  \param [in,out] pstr_bit_buff Pointer to bit buffer structure.
 *  \param [in]  num_of_bits  No. of bits to skip.
 *
 *  \return WORD32     No. of bits to skip.
 *
 */
WORD32 ia_core_coder_skip_bits_buf(ia_bit_buf_struct *pstr_bit_buff, WORD num_of_bits)
{
  UWORD8 *ptr_read_next = pstr_bit_buff->ptr_read_next;
  WORD bit_position = pstr_bit_buff->bit_pos;

  if (pstr_bit_buff->cnt_bits < num_of_bits || pstr_bit_buff->cnt_bits < 0)
    longjmp(*(pstr_bit_buff->xmpeghd_jmp_buf),
            IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES);
  bit_position -= (num_of_bits % 8);
  ptr_read_next += (num_of_bits >> 3);
  pstr_bit_buff->cnt_bits -= num_of_bits;
  if (bit_position < 0)
  {
    ptr_read_next++;
    bit_position += 8;
  }
  if (bit_position > 7)
  {
    longjmp(*(pstr_bit_buff->xmpeghd_jmp_buf),
            IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES);
  }
  pstr_bit_buff->bit_pos = (WORD16)bit_position;
  pstr_bit_buff->ptr_read_next = ptr_read_next;

  return num_of_bits;
}

/**
 *  ia_core_coder_show_bits_buf
 *
 *  \brief Helper function to read bits, but not update bit buffer structure.
 *
 *  \param [in,out] pstr_bit_buff Pointer to bit buffer structure.
 *  \param [in]  num_of_bits  No. of bits whose value is needed.
 *
 *  \return WORD32
 *
 */
WORD32 ia_core_coder_show_bits_buf(ia_bit_buf_struct *pstr_bit_buff, WORD num_of_bits)
{
  UWORD32 return_val;
  UWORD8 *ptr_read_next = pstr_bit_buff->ptr_read_next;
  WORD bit_position = pstr_bit_buff->bit_pos;

  if (num_of_bits == 0)
  {
    return 0;
  }

  if (pstr_bit_buff->cnt_bits < num_of_bits || pstr_bit_buff->cnt_bits < 0)
  {
    longjmp(*(pstr_bit_buff->xmpeghd_jmp_buf),
            IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES);
  }
  bit_position -= num_of_bits;
  return_val = (UWORD32)*ptr_read_next;

  while (bit_position < -1)
  {
    ptr_read_next++;
    return_val <<= 8;

    return_val |= (UWORD32)*ptr_read_next;
    bit_position += 8;
  }

  if (bit_position == -1)
  {
    ptr_read_next++;
    return_val <<= 8;
    bit_position += 8;
  }

  return_val = return_val << ((31 - num_of_bits) - bit_position) >> (32 - num_of_bits);

  return return_val;
}

/**
 *  ia_core_coder_read_bits_buf
 *
 *  \brief Helper function to read bits.
 *
 *  \param [in,out] pstr_bit_buff Pointer to bit buffer structure.
 *  \param [in]  num_of_bits  Pointer to no of bits to be read.
 *
 *  \return WORD32 Value read from bit stream.
 *
 */
WORD32 ia_core_coder_read_bits_buf(ia_bit_buf_struct *pstr_bit_buff, WORD num_of_bits)
{
  UWORD8 *ptr_read_next = pstr_bit_buff->ptr_read_next;
  WORD bit_position = pstr_bit_buff->bit_pos;
  UWORD32 return_val;

  if (num_of_bits == 0)
  {
    return 0;
  }

  if (pstr_bit_buff->cnt_bits < num_of_bits || pstr_bit_buff->cnt_bits < 0)
  {
    longjmp(*(pstr_bit_buff->xmpeghd_jmp_buf),
            IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES);
  }

  pstr_bit_buff->cnt_bits -= num_of_bits;
  return_val = (UWORD32)*ptr_read_next;

  bit_position -= num_of_bits;
  while (bit_position < -1)
  {

    ptr_read_next++;
    bit_position += 8;
    return_val <<= 8;

    return_val |= (UWORD32)*ptr_read_next;
  }

  if (bit_position == -1)
  {
    ptr_read_next++;
    return_val <<= 8;
    bit_position += 8;
  }
  pstr_bit_buff->bit_pos = (WORD16)bit_position;
  pstr_bit_buff->ptr_read_next = ptr_read_next;
  return_val = return_val << ((31 - num_of_bits) - bit_position) >> (32 - num_of_bits);

  return return_val;
}
/**
*  ia_core_coder_read_bits_buf_32
*
*  \brief Helper function to read bits.
*
*  \param [in,out] pstr_bit_buff Pointer to bit buffer structure.
*
*  \return UWORD32 Value read from bit stream.
*
*/
UWORD32 ia_core_coder_read_bits_buf_32(ia_bit_buf_struct *pstr_bit_buff)
{
  WORD32 num_of_bits = 32;
  UWORD32 return_val;
  UWORD8 *ptr_read_next = pstr_bit_buff->ptr_read_next;
  WORD bit_position = pstr_bit_buff->bit_pos;

  if (pstr_bit_buff->cnt_bits < num_of_bits || pstr_bit_buff->cnt_bits < 0)
  {
    longjmp(*(pstr_bit_buff->xmpeghd_jmp_buf),
            IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_INPUT_BYTES);
  }

  pstr_bit_buff->cnt_bits -= num_of_bits;
  return_val = (UWORD32)*ptr_read_next;

  bit_position -= num_of_bits;
  while (bit_position < -1)
  {
    bit_position += 8;
    ptr_read_next++;

    return_val <<= 8;

    return_val |= (UWORD32)*ptr_read_next;
  }

  if (bit_position == -1)
  {
    bit_position += 8;
    ptr_read_next++;
  }

  pstr_bit_buff->ptr_read_next = ptr_read_next;
  pstr_bit_buff->bit_pos = (WORD16)bit_position;

  return return_val;
}

/**
 *  ia_core_coder_mpeghd_read_byte_corr1
 *
 *  \brief Function to read bit stream data.
 *
 *  \param [in,out] ptr_read_next Pointer to the next location to be read in bit buffer.
 *  \param [in,out] ptr_bit_pos   Pointer to bit position.
 *  \param [out] ptr_read_word      Pointer to the word read from bit stream.
 *  \param [in]  ptr_bit_buf_end Pointer to the end of bit buffer
 *
 */
VOID ia_core_coder_mpeghd_read_byte_corr1(UWORD8 **ptr_read_next, WORD32 *ptr_bit_pos,
                                          WORD32 *ptr_read_word, UWORD8 *ptr_bit_buf_end)
{
  UWORD8 *ptr_v = *ptr_read_next;
  WORD32 bits_used = *ptr_bit_pos;
  WORD32 bits_temp_count = 0;

  while (bits_used >= 8)
  {
    bits_used -= 8;
    if ((ptr_bit_buf_end < ptr_v) && (ptr_bit_buf_end != 0))
      bits_temp_count += 8;
    else
    {
      *ptr_read_word = (*ptr_read_word << 8) | *ptr_v;
      ptr_v++;
    }
  }
  *ptr_bit_pos = bits_used + bits_temp_count;
  *ptr_read_next = ptr_v;
  return;
}

/**
 *  impeghd_create_write_bit_buffer
 *
 *  \brief Function to create write bitbuffer
 *
 *  \param [in,out] pstr_bit_buff    Pointer to bit buffer structure.
 *  \param [in]     ptr_bit_buf_base Pointer to bit buffer base
 *  \param [in]     bit_buffer_size  bitbuffer size
 *  \param [in]     init             init flag
 *
 *  \return ia_bit_buf_struct
 *
 */
WORD32 impeghd_create_write_bit_buffer(ia_write_bit_buf_struct *pstr_bit_buff,
                                       UWORD8 *ptr_bit_buf_base, UWORD32 bit_buffer_size,
                                       WORD32 init)
{
  pstr_bit_buff->ptr_bit_buf_base = ptr_bit_buf_base;
  pstr_bit_buff->ptr_bit_buf_end = ptr_bit_buf_base + bit_buffer_size - 1;
  pstr_bit_buff->ptr_read_next = ptr_bit_buf_base;
  pstr_bit_buff->ptr_write_next = ptr_bit_buf_base;

  if (init)
  {
    pstr_bit_buff->write_position = 7;
    pstr_bit_buff->read_position = 7;
    pstr_bit_buff->cnt_bits = 0;
    pstr_bit_buff->size = bit_buffer_size * 8;
  }

  return (0);
}

/**
 *  impeghd_write_bits_buf
 *
 *  \brief Function to write to bitbuffer
 *
 *  \param [in,out] pstr_bit_buff Pointer to bit buffer structure.
 *  \param [in]     write_val     Value to write
 *  \param [in]     num_of_bits   Number of bits to write
 *
 *  \return UWORD8
 *
 */
UWORD8 impeghd_write_bits_buf(ia_write_bit_buf_struct *pstr_bit_buff, UWORD32 write_val,
                              UWORD8 num_of_bits)
{
  UWORD8 *ptr_bit_buf_base;
  UWORD8 *ptr_bit_buf_end;
  UWORD8 *ptr_write_next;
  WORD32 position_to_write;
  UWORD8 written_bits = num_of_bits;
  WORD32 bits_to_write;

  if (pstr_bit_buff)
  {
    pstr_bit_buff->cnt_bits += num_of_bits;

    position_to_write = pstr_bit_buff->write_position;
    ptr_write_next = pstr_bit_buff->ptr_write_next;
    ptr_bit_buf_end = pstr_bit_buff->ptr_bit_buf_end;
    ptr_bit_buf_base = pstr_bit_buff->ptr_bit_buf_base;

    while (num_of_bits)
    {

      bits_to_write = ia_min_int(position_to_write + 1, num_of_bits);
      *ptr_write_next &= ~(((1 << bits_to_write) - 1) << (position_to_write + 1 - bits_to_write));
      *ptr_write_next |=
          (UWORD8)(write_val << (32 - num_of_bits) >>
                   (32 - bits_to_write) << (position_to_write + 1 - bits_to_write));

      position_to_write -= bits_to_write;
      num_of_bits -= bits_to_write;
      if (position_to_write < 0)
      {
        ptr_write_next++;
        position_to_write += 8;
        if (ptr_write_next > ptr_bit_buf_end)
        {
          ptr_write_next = ptr_bit_buf_base;
        }
      }
    }
    pstr_bit_buff->ptr_write_next = ptr_write_next;
    pstr_bit_buff->write_position = position_to_write;
  }

  return (written_bits);
}

/**
 *  impeghd_write_escape_value
 *
 *  \brief Function to write escape value
 *
 *  \param [in,out] pstr_bit_buff Pointer to bit buffer structure.
 *  \param [in]     value         Value to write
 *  \param [in]     no_bits1      No of bits to write
 *  \param [in]     no_bits2      No of bits to write
 *  \param [in]     no_bits3      No of bits to write
 *
 *  \return WORD32
 *
 */
WORD32 impeghd_write_escape_value(ia_write_bit_buf_struct *pstr_bit_buff, UWORD32 value,
                                  UWORD32 no_bits1, UWORD32 no_bits2, UWORD32 no_bits3)
{

  WORD32 bit_count = 0;
  UWORD32 esc_value = 0;
  UWORD32 max_value;
  max_value = (1 << no_bits1) - 1;
  esc_value = ia_min_int(value, max_value);
  bit_count += impeghd_write_bits_buf(pstr_bit_buff, esc_value, no_bits1);
  if (esc_value == max_value)
  {
    value = value - esc_value;
    max_value = (1 << no_bits2) - 1;
    esc_value = ia_min_int(value, max_value);
    bit_count += impeghd_write_bits_buf(pstr_bit_buff, esc_value, no_bits2);

    if (esc_value == max_value)
    {
      value = value - esc_value;
      max_value = (1 << no_bits3) - 1;
      esc_value = ia_min_int(value, max_value);
      bit_count += impeghd_write_bits_buf(pstr_bit_buff, esc_value, no_bits3);
    }
  }

  return bit_count;
}
/** @} */ /* End of CoreDecBSParse */