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
#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include "ia_core_coder_defines.h"

#include "impd_drc_common.h"
#include "impd_drc_extr_delta_coded_info.h"
#include "impd_drc_struct.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_interface.h"
#include <ia_core_coder_info.h>
#include <ia_core_coder_rom.h>
#include "ia_core_coder_dec.h"

#include <ia_core_coder_basic_op.h>
#include "ia_core_coder_channel.h"
#include "ia_core_coder_channelinfo.h"
#include "ia_core_coder_env_extr.h"
#include "ia_core_coder_function_selector.h"
#include "impeghd_memory_standards.h"
#include "impeghd_error_codes.h"
#include "impeghd_hoa_common_values.h"
#include "impeghd_hoa_matrix_struct.h"
#include "impeghd_hoa_matrix.h"
#include "ia_core_coder_config.h"
#include "ia_core_coder_struct_def.h"
#include "ia_core_coder_headerdecode.h"

/**
 * @defgroup CoreDecInit Core Decoder initialization
 * @ingroup  CoreDecInit
 * @brief Core Decoder initialization
 *
 * @{
 */

/**
 *  ia_core_coder_create_bit_buf
 *
 *  \brief Initialize bit buffer structure entries.
 *
 *  \param [i/o] it_bit_buff      Pointer to bit buffer structure.
 *  \param [in]  ptr_bit_buf_base Pointer to bit buffer base.
 *  \param [in]  bit_buf_size     Pointer to bit buffer size.
 *
 *  \return ia_bit_buf_struct * Pointer to bit buffer structure

 *
 */
ia_bit_buf_struct *ia_core_coder_create_bit_buf(ia_bit_buf_struct *it_bit_buff,
                                                UWORD8 *ptr_bit_buf_base, WORD32 bit_buf_size)
{
  it_bit_buff->size = bit_buf_size << 3;
  it_bit_buff->max_size = it_bit_buff->size;

  it_bit_buff->cnt_bits = 0;

  it_bit_buff->bit_pos = 7;

  it_bit_buff->ptr_bit_buf_base = ptr_bit_buf_base;
  it_bit_buff->ptr_read_next = ptr_bit_buf_base;

  it_bit_buff->ptr_bit_buf_end = ptr_bit_buf_base + bit_buf_size - 1;

  return it_bit_buff;
}

/**
 *  ia_core_coder_create_init_bit_buf
 *
 *  \brief Create bit buffer reading structure.
 *
 *  \param [i/o] it_bit_buff      Pointer to bit buffer handle.
 *  \param [in]  ptr_bit_buf_base Pointer to bit buffer base.
 *  \param [in]  bit_buf_size     Bit buffer size.
 *
 *  \return VOID
 *
 */
VOID ia_core_coder_create_init_bit_buf(ia_bit_buf_struct *it_bit_buff, UWORD8 *ptr_bit_buf_base,
                                       WORD32 bit_buf_size)
{
  ia_core_coder_create_bit_buf(it_bit_buff, ptr_bit_buf_base, bit_buf_size);
  it_bit_buff->cnt_bits = (bit_buf_size << 3);
}
/** @} */ /* End of CoreDecInit */