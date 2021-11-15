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

#include "ia_core_coder_cnst.h"
#include <ia_core_coder_constants.h>
#include <impeghd_type_def.h>

#include "ia_core_coder_acelp_info.h"
#include "ia_core_coder_bitbuffer.h"
#include "ia_core_coder_defines.h"
#include "ia_core_coder_interface.h"
#include "ia_core_coder_info.h"
#include "ia_core_coder_rom.h"
#include "ia_core_coder_tns_usac.h"
#include "ia_core_coder_windows.h"
#include "impeghd_error_codes.h"

/**
 * @defgroup CoreDecProc Core Decoder processing
 * @ingroup  CoreDecProc
 * @brief Core Decoder processing
 *
 * @{
 */

/**
 *  ia_core_coder_calc_window
 *
 *  \brief Calculate window based on selection & length
 *
 *  \param [out]   window        Selected window type
 *  \param [in]    window_length Length of window needed
 *  \param [in]    window_shape  Shape of window needed
 *
 *  \return IA_ERRORCODE
 *
 */
IA_ERRORCODE ia_core_coder_calc_window(FLOAT32 **window, WORD32 window_length,
                                       WORD32 window_shape)
{
  switch (window_shape)
  {
  case WIN_SEL_1:
    switch (window_length)
    {
    case WIN_LEN_1024:
      *window = (FLOAT32 *)ia_core_coder_kbd_win1024;
      break;
    case WIN_LEN_128:
      *window = (FLOAT32 *)ia_core_coder_kbd_window128;
      break;
    case WIN_LEN_64:
      *window = (FLOAT32 *)ia_core_coder_kbd_window64;
      break;
    case WIN_LEN_16:
      *window = (FLOAT32 *)ia_core_coder_kbd_win16;
      break;
    case WIN_LEN_4:
      *window = (FLOAT32 *)ia_core_coder_kbd_win4;
      break;
    default:
      return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_WINDOW_LEN;
      break;
    }
    break;
  case WIN_SEL_0:
    switch (window_length)
    {
    case WIN_LEN_1024:
      *window = (FLOAT32 *)ia_core_coder_sine_window1024;
      break;
    case WIN_LEN_256:
      *window = (FLOAT32 *)ia_core_coder_sine_window256;
      break;
    case WIN_LEN_128:
      *window = (FLOAT32 *)ia_core_coder_sine_window128;
      break;
    default:
      return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_WINDOW_LEN;
      break;
    }
    break;

  default:
    return IA_MPEGH_DEC_EXE_FATAL_UNSUPPORTED_WINDOW_SHAPE;
    break;
  }
  return IA_MPEGH_DEC_NO_ERROR;
}
/** @} */ /* End of CoreDecProc */