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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <impeghd_type_def.h>
#include "impeghd_memory_standards.h"
#include "impeghd_mp4_file.h"
#include "impeghd_mp4_utils.h"
#include "impeghd_mp4_odf.h"
#include "impeghd_mp4_defs.h"
#include "impeghd_mp4_atoms.h"
#include "impeghd_mp4_object_type.h"
#include "impeghd_mp4_proto.h"
#include "impeghd_mp4_parser.h"
#include "impeghd_mp4_file_wrapper.h"


/**
*  impeghd_mp4_find_stsz
*
*  \brief function to read into buffer
*
*  \param [in]  itf       Pointer to file context
*  \param [out] offset    offset
*  \param [out] stsz_size stsz size
*
*  \return WORD32
*
*/

IA_ERRORCODE impeghd_mp4_find_stsz(it_avi_file_ctxt *itf, WORD32 *offset, WORD32 *stsz_size)
{
  WORD8 buf_size[4];
  WORD8 buf_test;
  WORD32 bytes_test;
  UWORD32 *data_size = (UWORD32 *)&buf_size[0];
  *stsz_size = 0;
  while (!feof(itf->fp))
  {
    bytes_test = impeghd_mp4_fread(&buf_test, 1, 1, itf);
    if (buf_test == 's')
    {
      bytes_test = impeghd_mp4_fread(&buf_test, 1, 1, itf);
      if (buf_test == 't')
      {
        bytes_test = impeghd_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 's')
          bytes_test = impeghd_mp4_fread(&buf_test, 1, 1, itf);
        if (buf_test == 'z')
        {
          impeghd_mp4_fseek(itf, -8, SEEK_CUR);
          bytes_test = impeghd_mp4_fread(buf_size, 1, 4, itf);
          if (bytes_test < 4)
          {
            *stsz_size = -1;
            return 1;
          }

          *stsz_size = impeghd_mp4_rev32(*data_size);
          bytes_test = impeghd_mp4_fread(buf_size, 1, 4, itf);
          *offset = ftell(itf->fp);
          *stsz_size = *stsz_size - 8;
          return IT_OK;
        }
      }
      if (buf_test == 'm')
        impeghd_mp4_fseek(itf, -1, SEEK_CUR);
    }
  }
  return IT_OK;
}


/**
*  impeghd_mp4_parse_mae_boxes
*
*  \brief function to parse mae boxes
*
*  \param [out] g_pf_inp_str   Input file pointer
*  \param [out] ptr_dec_api    Pointer to api struct
*
*  \return IA_ERRORCODE
*
*/
IA_ERRORCODE impeghd_mp4_parse_mae_boxes(ia_file_wrapper* g_pf_inp_str, pVOID ptr_dec_api)
{

  IA_ERRORCODE error = IT_OK;
  ia_mpeghd_api_struct *pstr_dec_api = (ia_mpeghd_api_struct *)ptr_dec_api;
  WORD32 offset = 0;
  UWORD32 number_entries;
  WORD32 *length_store;
  UWORD8 charbuf[10];
  UWORD32 *data_size;
  WORD32 fread_size = 0;
  WORD32 stsz_size;
  mp4_info *m_mp4;
  it_avi_file_ctxt *itf;

  pstr_dec_api->input_config.ptr_maeg_buf = calloc(MAE_BUFF_SIZE, 1);
  pstr_dec_api->input_config.ptr_maei_buf = calloc(MAE_BUFF_SIZE, 1);
  pstr_dec_api->input_config.ptr_maes_buf = calloc(MAE_BUFF_SIZE, 1);
  pstr_dec_api->input_config.ptr_maep_buf = calloc(MAE_BUFF_SIZE, 1);

  m_mp4 = (mp4_info *)impeghd_mp4_malloc_wrapper(sizeof(mp4_info));
  m_mp4->imp_trak_info[0] = NULL;
  m_mp4->imp_trak_info[1] = NULL;
  m_mp4->fp = (pVOID)g_pf_inp_str->file_cntxt;
  m_mp4->st_maei_info.init = 1;

  m_mp4->st_maei_info.ptr_maeg_buf = pstr_dec_api->input_config.ptr_maeg_buf;
  m_mp4->st_maei_info.ptr_maei_buf = pstr_dec_api->input_config.ptr_maei_buf;
  m_mp4->st_maei_info.ptr_maes_buf = pstr_dec_api->input_config.ptr_maes_buf;
  m_mp4->st_maei_info.ptr_maep_buf = pstr_dec_api->input_config.ptr_maep_buf;

  impeghd_mp4_init_wrap(m_mp4);

  pstr_dec_api->input_config.maeg_flag = m_mp4->st_maei_info.maeg_flag;
  pstr_dec_api->input_config.maei_flag = m_mp4->st_maei_info.maei_flag;
  pstr_dec_api->input_config.maes_flag = m_mp4->st_maei_info.maes_flag;
  pstr_dec_api->input_config.maep_flag = m_mp4->st_maei_info.maep_flag;

  pstr_dec_api->input_config.maeg_len = m_mp4->st_maei_info.maeg_len;
  pstr_dec_api->input_config.maei_len = m_mp4->st_maei_info.maei_len;
  pstr_dec_api->input_config.maes_len = m_mp4->st_maei_info.maes_len;
  pstr_dec_api->input_config.maep_len = m_mp4->st_maei_info.maep_len;


  number_entries = m_mp4->imp_trak_info[1]->stsz_count;
  length_store = malloc(number_entries * sizeof(WORD32));
  data_size = (UWORD32 *)&charbuf[0];
  itf = (it_avi_file_ctxt *)g_pf_inp_str->file_cntxt;
  impeghd_mp4_fseek(itf, 0, SEEK_SET);

  error = impeghd_mp4_find_stsz(itf, &offset, &stsz_size);
  if (error)
  {
    return error;
  }
  /* Seeking to a position after version, sample_size and sample_count */
  fseek(itf->fp, 12, SEEK_CUR);
  for (UWORD32 j = 0; j < number_entries; j++)
  {
    fread_size = (WORD32)fread(&charbuf, 4, 1, itf->fp);
    (*data_size) = (*data_size) * fread_size;
    length_store[j] = impeghd_mp4_rev32(*data_size);
  }
    impeghd_mp4_fseek(itf, 0, SEEK_SET);
    return IT_OK;
}