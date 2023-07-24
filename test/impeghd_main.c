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
#include "impeghd_api.h"
#include "impeghd_error_codes.h"
#include "impeghd_error_handler.h"
#include "impeghd_mp4_parser.h"
#include "impeghd_mp4_file_wrapper.h"
#include "impeghd_config_params.h"

/**
 * @defgroup SampleApp Sample test bench
 * @ingroup  SampleApp
 * @brief Sample test bench
 *
 * @{
 */

VOID impeghd_error_handler_init();
VOID ia_testbench_error_handler_init();

extern ia_error_info_struct impeghd_ia_testbench_error_info;
extern ia_error_info_struct impeghd_error_info;

/*****************************************************************************/
/* Process select hash defines                                               */
/*****************************************************************************/
#define WAV_HEADER
#define DISPLAY_MESSAGE
//#define PRINT_SPEAKER_INFO

/*****************************************************************************/
/* Constant hash defines                                                     */
/*****************************************************************************/
#define IA_MAX_CMD_LINE_LENGTH 300
#define IA_MAX_ARGS 20
#define IA_SCREEN_WIDTH 80
#define PARAMFILE "paramfilesimple.txt"

/*****************************************************************************/
/* Error codes for the testbench                                             */
/*****************************************************************************/
#define IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED 0xFFFFA000
#define IA_TESTBENCH_FATAL_WAV_HDR_WRITE_FAIL 0xFFFFA001

#ifdef ARM_PROFILE_HW
#include <sys/time.h>
//#define CLK_FREQ_BOARD_MHZ 1500 // IMX8 for profiling
#define CLK_FREQ_BOARD_MHZ 1734 // Tegra for profiling
//#define CLK_FREQ_BOARD_MHZ 716 // a9 omap4430 board
//#define CLK_FREQ_BOARD_MHZ 1555 //Nexus6P
//#define CLK_FREQ_BOARD_MHZ 2035 //Tegra Board
//#define CLK_FREQ_BOARD_MHZ 550 //a8  board
//#define CLK_FREQ_BOARD_MHZ 297 //dm365 board
//#define CLK_FREQ_BOARD_MHZ 1209.6 //a5 board

long long itGetMs(void)
{
  struct timeval t;
  long long currTime;

  if (gettimeofday(&t, NULL) == -1)
  {
    printf("Error in gettimeofday. It has returned -1. \n");
  }
  currTime = ((t.tv_sec * 1000 * 1000) + (t.tv_usec));
  return currTime;
}
#define ARM_PROFILE_HW_VARDEC                                                                    \
  WORD32 frame_count_b = 0;                                                                      \
  long long cycles_b = 0;                                                                        \
  long long start1_b, stop1_b;                                                                   \
  double Curr_b, Ave_b = 0, Sum_b = 0;                                                           \
  double Peak_b = 0;                                                                             \
  WORD32 Peak_frame_b = 0;

#define ARM_PROFILE_HW_START start1_b = itGetMs();

#define ARM_PROFILE_HW_STOP                                                                      \
  stop1_b = itGetMs();                                                                           \
  cycles_b = (stop1_b - start1_b);

#define ARM_PROFILE_HW_CALC_CYCLES(i_out_bytes, i_pcm_wd_sz, i_samp_freq, i_num_chan)            \
  if (i_out_bytes != 0)                                                                          \
  {                                                                                              \
    if (frame_count_b)                                                                           \
    {                                                                                            \
      double i_out_samples_per_ch = (i_out_bytes) / ((i_pcm_wd_sz / 8) * i_num_chan);            \
      Curr_b = (((double)cycles_b / 1000000) * CLK_FREQ_BOARD_MHZ) /                             \
               (i_out_samples_per_ch / i_samp_freq);                                             \
      frame_count_b++;                                                                           \
      Sum_b += Curr_b;                                                                           \
      Ave_b = Sum_b / frame_count_b;                                                             \
      if (Peak_b < Curr_b)                                                                       \
      {                                                                                          \
        Peak_b = Curr_b;                                                                         \
        Peak_frame_b = frame_count_b;                                                            \
      }                                                                                          \
    }                                                                                            \
    else                                                                                         \
    {                                                                                            \
      frame_count_b++;                                                                           \
    }                                                                                            \
    cycles_b = 0;                                                                                \
  }

#define ARM_PROFILE_PRINT_ONCE(i_out_bytes, i_pcm_wd_sz, i_samp_freq, i_num_chan)                \
  double i_out_samples_per_ch = (i_out_bytes) / ((i_pcm_wd_sz / 8) * i_num_chan);                \
  printf("i_out_samples_per_ch = %lf, i_out_bytes %d\n", i_out_samples_per_ch, i_out_bytes);

#define ARM_PROFILE_HW_PRINT_PROF_DATA                                                           \
  fprintf(stdout, "\n Peak MCPS = %f\n", Peak_b);                                                \
  fprintf(stdout, " Avg MCPS = %f\n", Ave_b);                                                    \
  fprintf(stdout, " Peak frame = %d\n", Peak_frame_b);
#else
#define ARM_PROFILE_HW_VARDEC
#define ARM_PROFILE_HW_START
#define ARM_PROFILE_HW_STOP
#define ARM_PROFILE_HW_CALC_CYCLES(i_out_bytes, i_pcm_wd_sz, i_samp_freq, i_num_chan)
#define ARM_PROFILE_HW_PRINT_PROF_DATA
#define ARM_PROFILE_PRINT_ONCE(i_out_bytes, i_pcm_wd_sz, i_samp_freq, i_num_chan)
#endif
/*****************************************************************************/
/* Global variables                                                          */
/*****************************************************************************/
FILE *g_pf_out;
FILE *g_pf_brir;
WORD32 g_binaural_flag;
ia_file_wrapper *g_pf_inp; /* file pointer to bitstream file (mp4) */
FILE *g_pf_ei_bs = NULL;
FILE *g_pf_lsi_bs = NULL;
FILE *g_pf_sd_bs = NULL;
FILE *g_pf_ext_ren_oam_md = NULL;
FILE *g_pf_ext_ren_ch_md = NULL;
FILE *g_pf_ext_ren_hoa_md = NULL;
FILE *g_pf_ext_ren_pcm = NULL;
WORD8 out_filename[IA_MAX_CMD_LINE_LENGTH] = "";
WORD32 raw_testing = 0;

#ifdef WAV_HEADER
/**
*  impeghd_write16_bits_lh
*
*  \brief Write 16 bits low high (always little endian)
*
*  \param [in] fp Pointer to FILE object
*  \param [in] i  Data to be written
*
*
*
*/
static VOID impeghd_write16_bits_lh(FILE *fp, WORD32 i)
{
  putc(i & 0xff, fp);
  putc((i >> 8) & 0xff, fp);
}

/**
*  impeghd_write32_bits_lh
*
*  \brief Write 32 bits low high (always little endian)
*
*  \param [in] fp Pointer to FILE object
*  \param [in] i  Data to be written
*
*
*
*/
static VOID impeghd_write32_bits_lh(FILE *fp, WORD32 i)
{
  impeghd_write16_bits_lh(fp, (WORD32)(i & 0xffffL));
  impeghd_write16_bits_lh(fp, (WORD32)((i >> 16) & 0xffffL));
}
#endif

/**
*  write_wav_header
*
*  \brief Write wav header to a wav file
*
*  \param [in] fp       Pointer to FILE object
*  \param [in] pcmbytes Total length in bytes
*  \param [in] freq     Sampling frequency
*  \param [in] channels Number of channels
*  \param [in] bits     Number of bits in a PCM word
*
*  \return IA_ERRORCODE       Processing error if any
*
*/
IA_ERRORCODE write_wav_header(FILE *fp, WORD32 pcmbytes, WORD32 freq, WORD32 channels,
                              WORD32 bits)
{
#ifdef WAV_HEADER
#ifndef ARM_PROFILE_BOARD
  WORD32 bytes = (bits + 7) / 8;
  fwrite("RIFF", 1, 4, fp);                           /* label */
  impeghd_write32_bits_lh(fp, pcmbytes + 44 - 8);     /* length in bytes without header */
  fwrite("WAVEfmt ", 2, 4, fp);                       /* 2 labels */
  impeghd_write32_bits_lh(fp, 2 + 2 + 4 + 4 + 2 + 2); /* length of PCM format decl area */
  impeghd_write16_bits_lh(fp, 1);                     /* is pcm? */
  impeghd_write16_bits_lh(fp, channels);
  impeghd_write32_bits_lh(fp, freq);
  impeghd_write32_bits_lh(fp, freq * channels * bytes); /* bps */
  impeghd_write16_bits_lh(fp, channels * bytes);
  impeghd_write16_bits_lh(fp, bits);
  fwrite("data", 1, 4, fp);
  impeghd_write32_bits_lh(fp, pcmbytes);
#endif
#endif
  return (ferror(fp) ? IA_TESTBENCH_FATAL_WAV_HDR_WRITE_FAIL : IA_MPEGH_DEC_NO_ERROR);
}

#ifdef DISPLAY_MESSAGE
/**
*  ia_display_id_message
*
*  \brief Display the ID message of the process
*
*  \param [in] lib_name    Library name
*  \param [in] lib_version Library version
*
*
*
*/
VOID ia_display_id_message(WORD8 lib_name[], WORD8 lib_version[])
{
  WORD8 str[4][IA_SCREEN_WIDTH] = {"ITTIAM SYSTEMS PVT LTD, BANGALORE\n",
                                   "http:\\\\www.ittiam.com\n", "", ""};
  WORD8 spaces[IA_SCREEN_WIDTH / 2 + 1];
  WORD32 i, spclen;

  strcpy((pCHAR8)str[2], (pCHAR8)lib_name);
  strcat((pCHAR8)str[2], (pCHAR8)lib_version);
  strcat((pCHAR8)str[2], "\n");
  strcat((pCHAR8)str[4 - 1], "\n");

  for (i = 0; i < IA_SCREEN_WIDTH / 2 + 1; i++)
  {
    spaces[i] = ' ';
  }

  for (i = 0; i < 4; i++)
  {
    spclen = IA_SCREEN_WIDTH / 2 - (WORD32)(strlen((pCHAR8)str[i]) / 2);
    spaces[spclen] = '\0';
    printf("%s", (pCHAR8)spaces);
    spaces[spclen] = ' ';
    printf("%s", (pCHAR8)str[i]);
  }
}
#endif /* DISPLAY_MESSAGE */

/**
*  impeghd_parse_config_param
*
*  \brief Parse config parameters
*
*  \param [in]     argc        Argument count
*  \param [in]     argv        Pointer to the argument strings
*  \param [in,out] ptr_dec_api Pointer to the decoder API structure
*
*  \return IA_ERRORCODE        Processing error if any
*
*/
IA_ERRORCODE impeghd_parse_config_param(WORD32 argc, pWORD8 argv[], pVOID ptr_dec_api)
{
  LOOPIDX i;

  ia_mpeghd_api_struct *pstr_dec_api = (ia_mpeghd_api_struct *)ptr_dec_api;

  for (i = 0; i < argc; i++)
  {
    /* PCM WORD Size (For single input file) */
    if (!strncmp((pCHAR8)argv[i], "-pcmsz:", 7))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 7);
      pstr_dec_api->input_config.ui_pcm_wd_sz = atoi(pb_arg_val);
    }

    /*For MPEG-D DRC effect type*/
    if (!strncmp((pCHAR8)argv[i], "-effect:", 8))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 8);
      pstr_dec_api->input_config.ui_effect = atoi(pb_arg_val);
    }
    /*For MPEG-D DRC target loudness*/
    if (!strncmp((pCHAR8)argv[i], "-target_loudness:", 17))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 17);
      pstr_dec_api->input_config.ui_target_loudness[0] = 1;
      pstr_dec_api->input_config.ui_target_loudness[1] = atoi(pb_arg_val);
      if ((pstr_dec_api->input_config.ui_target_loudness[1] > 0) ||
          (pstr_dec_api->input_config.ui_target_loudness[1] < -63))
      {
        pstr_dec_api->input_config.ui_target_loudness[1] = 0;
      }
      pstr_dec_api->input_config.ui_target_loudness[1] =
          -(pstr_dec_api->input_config.ui_target_loudness[1] << 2);
    }
    if (!strncmp((pCHAR8)argv[i], "-cicp:", 6))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 6);
      pstr_dec_api->input_config.ui_cicp_layout_idx = atoi(pb_arg_val);
    }
    if (!strncmp((pCHAR8)argv[i], "-preset:", 8))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 8);
      pstr_dec_api->input_config.i_preset_id = (WORD8)atoi(pb_arg_val);
    }
    if (!strncmp((const char *)argv[i], "-iei:", 5))
    {
      WORD32 ei_sz = 0;
      fseek(g_pf_ei_bs, 0L, SEEK_END);
      ei_sz = ftell(g_pf_ei_bs);
      rewind(g_pf_ei_bs);
      pstr_dec_api->input_config.ptr_ei_buf = calloc(1, ei_sz + 1);
      if (pstr_dec_api->input_config.ptr_ei_buf)
      {
        pstr_dec_api->input_config.ei_info_size =
            (WORD32)fread(pstr_dec_api->input_config.ptr_ei_buf, 1, ei_sz, g_pf_ei_bs);
      }
      fclose(g_pf_ei_bs);
      g_pf_ei_bs = NULL;
      pstr_dec_api->input_config.ei_info_flag = 1;
    }
    if (!strncmp((const char *)argv[i], "-ilsi:", 6))
    {
      WORD32 lsi_sz = 0;
      fseek(g_pf_lsi_bs, 0L, SEEK_END);
      lsi_sz = ftell(g_pf_lsi_bs);
      rewind(g_pf_lsi_bs);
      pstr_dec_api->input_config.ptr_ls_buf = calloc(1, lsi_sz + 1);
      if (pstr_dec_api->input_config.ptr_ls_buf)
      {
        pstr_dec_api->input_config.lsi_info_size =
            (WORD32)fread(pstr_dec_api->input_config.ptr_ls_buf, 1, lsi_sz, g_pf_lsi_bs);
      }
      fclose(g_pf_lsi_bs);
      g_pf_lsi_bs = NULL;
      pstr_dec_api->input_config.lsi_info_flag = 1;
    }
    if (!strncmp((const char *)argv[i], "-isdi:", 6))
    {
      WORD32 sdi_sz = 0;
      fseek(g_pf_sd_bs, 0L, SEEK_END);
      sdi_sz = ftell(g_pf_sd_bs);
      rewind(g_pf_sd_bs);
      pstr_dec_api->input_config.ptr_sd_buf = calloc(1, sdi_sz + 1);
      if (pstr_dec_api->input_config.ptr_sd_buf)
      {
        pstr_dec_api->input_config.sd_info_size =
            (WORD32)fread(pstr_dec_api->input_config.ptr_sd_buf, 1, sdi_sz, g_pf_sd_bs);
      }
      fclose(g_pf_sd_bs);
      g_pf_sd_bs = NULL;
      pstr_dec_api->input_config.sd_info_flag = 1;
    }
    if (!strncmp((const char *)argv[i], "-ext_ren:", 9))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 9);
      pstr_dec_api->input_config.extrn_rend_flag = (WORD8)atoi(pb_arg_val);
      if (pstr_dec_api->input_config.extrn_rend_flag)
      {
        pstr_dec_api->input_config.ptr_ext_ren_ch_data_buf = calloc(768, 1);
        pstr_dec_api->input_config.ptr_ext_ren_oam_data_buf = calloc(768, 1);
        pstr_dec_api->input_config.ptr_ext_ren_hoa_data_buf = calloc(768, 1);
        pstr_dec_api->input_config.ptr_ext_ren_pcm_buf = calloc(1024 * 32, 4);
      }
    }
    if (!strncmp((const char *)argv[i], "-ibrir:", 7))
    {
      WORD32 brir_sz = 0;
      fseek(g_pf_brir, 0L, SEEK_END);
      brir_sz = ftell(g_pf_brir);
      rewind(g_pf_brir);
      pstr_dec_api->input_config.ptr_brir_buf = calloc(1, brir_sz + 1);
      if (pstr_dec_api->input_config.ptr_brir_buf)
      {
        pstr_dec_api->input_config.binaural_data_len =
            (WORD32)fread(pstr_dec_api->input_config.ptr_brir_buf, 1, brir_sz, g_pf_brir);
      }
      fclose(g_pf_brir);
      g_pf_brir = NULL;
      pstr_dec_api->input_config.binaural_flag = 1;
    }
    if (!strncmp((const char *)argv[i], "-out_fs:", 8))
    {
      pCHAR8 pb_arg_val = (pCHAR8)(argv[i] + 8);
      pstr_dec_api->input_config.out_samp_freq = atoi(pb_arg_val);
      pstr_dec_api->input_config.enable_resamp = 1;
    }
  }

  return IA_MPEGH_DEC_NO_ERROR;
}

/**
*  malloc_global
*
*  \brief Function to allocate memory in testbench
*
*  \param [in] size      Bytes to be allocated
*  \param [in] alignment Bytes to align
*
*  \return pVOID         Address to the memory allocated or NULL pointer if fails
*
*/
pVOID malloc_global(UWORD32 size, UWORD32 alignment) { return malloc(size + alignment); }

/**
*  free_global
*
*  \brief Function to free memory in testbench
*
*  \param [in] ptr Pointer to the memory
*
*
*
*/
VOID free_global(pVOID ptr)
{
  free(ptr);
  ptr = NULL;
}

/**
*  impeghd_main_process
*
*  \brief Stacked decoder processing
*
*  \param [in] argc     Argument count
*  \param [in] argv     Pointer to the argument strings
*
*  \return IA_ERRORCODE Processing error if any
*
*/
IA_ERRORCODE impeghd_main_process(WORD32 argc, pWORD8 argv[])
{
  LOOPIDX i;
  WORD frame_counter = 0;

  /* Error code */
  IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
  IA_ERRORCODE err_code_reinit = IA_MPEGH_DEC_NO_ERROR;

  /* API obj */
  pVOID pv_ia_process_api_obj;

  pWORD8 pb_inp_buf = 0, pb_out_buf = 0;

  UWORD32 ui_inp_size = 0;
  WORD32 i_total_bytes = 0;
  WORD32 offset_dash = 0, size_dash = 0, loc = 0;
  ARM_PROFILE_HW_VARDEC;

  /* The error init function */
  VOID (*p_error_init)();

  /* The process error info structure */
  ia_error_info_struct *p_proc_err_info;

  ia_mpeghd_api_struct str_dec_api = {{0}, {0}};
  ia_input_config *pstr_in_cfg = &str_dec_api.input_config;
  ia_output_config *pstr_out_cfg = &str_dec_api.output_config;

  WORD32 i_buff_size, i_bytes_read;

  /* Process struct initing */
  p_error_init = impeghd_error_handler_init;
  p_proc_err_info = &impeghd_error_info;
  /* Process struct initing end */

  /* ******************************************************************/
  /* Initialize the error handler                                     */
  /* ******************************************************************/
  (*p_error_init)();

  str_dec_api.input_config.ui_mhas_flag = IMPEGHD_CONFIG_PARAM_MHAS_FLAG_DFLT_VAL;
  str_dec_api.input_config.ui_pcm_wd_sz = IMPEGHD_CONFIG_PARAM_PCM_WD_SZ_DFLT_VAL;
  str_dec_api.input_config.ui_cicp_layout_idx = IMPEGHD_CONFIG_PARAM_CICP_IDX_DFLT_VAL;
  str_dec_api.input_config.i_preset_id = IMPEGHD_CONFIG_PARAM_PRESET_ID_DFLT_VAL;
  str_dec_api.input_config.ei_info_flag = IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL;
  str_dec_api.input_config.lsi_info_flag = IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL;
  str_dec_api.input_config.sd_info_flag = IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL;
  str_dec_api.input_config.ei_info_size = IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL;
  str_dec_api.input_config.lsi_info_size = IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL;
  str_dec_api.input_config.sd_info_size = IMPEGHD_CONFIG_PARAM_EI_FLAG_DFLT_VAL;
  str_dec_api.input_config.binaural_flag = IMPEGHD_CONFIG_PARAM_BR_FLAG_DFLT_VAL;
  str_dec_api.input_config.binaural_data_len = IMPEGHD_CONFIG_PARAM_BR_FLAG_DFLT_VAL;
  str_dec_api.input_config.maeg_flag = IMPEGHD_CONFIG_PARAM_MAE_FLAG_DFLT_VAL;
  str_dec_api.input_config.maei_flag = IMPEGHD_CONFIG_PARAM_MAE_FLAG_DFLT_VAL;
  str_dec_api.input_config.maeg_len = IMPEGHD_CONFIG_PARAM_MAE_FLAG_DFLT_VAL;
  str_dec_api.input_config.maei_len = IMPEGHD_CONFIG_PARAM_MAE_FLAG_DFLT_VAL;
  /* ******************************************************************/
  /* Parse input configuration parameters                             */
  /* ******************************************************************/
  impeghd_parse_config_param(argc, argv, &str_dec_api);
  pstr_out_cfg->malloc_mpegh = &malloc_global;
  pstr_out_cfg->free_mpegh = &free_global;

  if (g_pf_inp->is_mp4_file)
  {
    impeghd_mp4_parse_mae_boxes(g_pf_inp, &str_dec_api);
  }

  /********************************************************************/
  /* Create Decoder Instance                                          */
  /* ******************************************************************/
  /* First part                                        */
  /* Get Library Name, Library Version                 */
  /* Initialize API structure + Default config set     */
  /* Set config params from user                       */
  /* Initialize memory tables                          */
  /* Get memory information and allocate memory        */
  err_code = ia_mpegh_dec_create((pVOID)pstr_in_cfg, (pVOID)pstr_out_cfg);
  _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

#ifdef DISPLAY_MESSAGE
  ia_display_id_message(pstr_out_cfg->p_lib_name, pstr_out_cfg->p_version_num);
#endif

  pv_ia_process_api_obj = pstr_out_cfg->pv_ia_process_api_obj;

  pb_inp_buf = (pWORD8)pstr_out_cfg->mem_info_table[IA_MEMTYPE_INPUT].mem_ptr;
  pb_out_buf = (pWORD8)pstr_out_cfg->mem_info_table[IA_MEMTYPE_OUTPUT].mem_ptr;

  /* ******************************************************************/
  /* Initialize MPEG-H 3D Audio Low Complexity Profile Decoder */
  /* ******************************************************************/
  /* Start of  second part */
  i_buff_size = pstr_out_cfg->ui_inp_buf_size;
  ui_inp_size = pstr_out_cfg->ui_inp_buf_size;
  pstr_out_cfg->i_bytes_consumed = pstr_out_cfg->ui_inp_buf_size;
  i_bytes_read = 0;
  {
    for (i = 0; i < (i_buff_size - pstr_out_cfg->i_bytes_consumed); i++)
    {
      pb_inp_buf[i] = pb_inp_buf[i + pstr_out_cfg->i_bytes_consumed];
    }

    impeghd_mp4_fw_read(
        g_pf_inp, (pUWORD8)(pb_inp_buf + i_buff_size - pstr_out_cfg->i_bytes_consumed),
        (ui_inp_size - (i_buff_size - pstr_out_cfg->i_bytes_consumed)), (pUWORD32)&i_bytes_read);

    if (!raw_testing && i_bytes_read == 0)
    {
      impeghd_mp4_fw_read(g_pf_inp,
                          (pUWORD8)(pb_inp_buf + i_buff_size - pstr_out_cfg->i_bytes_consumed),
                          (ui_inp_size - (i_buff_size - pstr_out_cfg->i_bytes_consumed)),
                          (pUWORD32)&i_bytes_read);
    }

    i_buff_size = i_buff_size - (pstr_out_cfg->i_bytes_consumed - i_bytes_read);

    if (i_buff_size <= 0)
    {
      err_code = IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_INPUT_BYTES;
      _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);
      goto exit_path;
    }
    pstr_in_cfg->num_inp_bytes = i_buff_size;
    if (raw_testing)
    {
      pstr_in_cfg->ui_raw_flag = 1;
    }
    pstr_out_cfg->i_bytes_consumed = 0;
    do
    {
      if (((WORD32)ui_inp_size - (WORD32)(i_buff_size - pstr_out_cfg->i_bytes_consumed)) > 0)
      {
        for (i = 0; i < (i_buff_size - pstr_out_cfg->i_bytes_consumed); i++)
        {
          pb_inp_buf[i] = pb_inp_buf[i + pstr_out_cfg->i_bytes_consumed];
        }
        impeghd_mp4_fw_read(
            g_pf_inp, (pUWORD8)(pb_inp_buf + i_buff_size - pstr_out_cfg->i_bytes_consumed),
            ((WORD32)ui_inp_size - (WORD32)(i_buff_size - pstr_out_cfg->i_bytes_consumed)),
            (pUWORD32)&i_bytes_read);
        i_buff_size = i_buff_size - (pstr_out_cfg->i_bytes_consumed - i_bytes_read);
        if ((i_buff_size <= 0) ||
            ((err_code_reinit == IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_INPUT_BYTES) &&
             i_bytes_read == 0))
        {
          _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code_reinit);
          goto exit_path;
        }
      }
      err_code =
          ia_mpegh_dec_init(pv_ia_process_api_obj, (pVOID)pstr_in_cfg, (pVOID)pstr_out_cfg);
      if (err_code == IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_EI_BYTES ||
          err_code == IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_LSI_BYTES ||
          err_code == IA_MPEGH_DEC_INIT_NONFATAL_INSUFFICIENT_BRIR_BYTES)
      {
        _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);
        goto exit_path;
      }
      err_code_reinit = err_code;
    } while ((!pstr_out_cfg->ui_init_done));

    if (pstr_in_cfg->extrn_rend_flag)
    {
      WORD8 ext_ren_filename[IA_MAX_CMD_LINE_LENGTH] = "";

      if (pstr_out_cfg->oam_data_present)
      {
        strcat((char *)ext_ren_filename, (const char *)out_filename);
        strcat((char *)ext_ren_filename, (const char *)"_ext_ren_oam_md.bs");

        g_pf_ext_ren_oam_md = fopen((const char *)ext_ren_filename, "wb");
        if (g_pf_ext_ren_oam_md == NULL)
        {
          return -1;
        }
        ext_ren_filename[0] = '\0';
      }
      if (pstr_out_cfg->hoa_data_present)
      {
        strcat((char *)ext_ren_filename, (const char *)out_filename);
        strcat((char *)ext_ren_filename, (const char *)"_ext_ren_hoa_md.bs");
        g_pf_ext_ren_hoa_md = fopen((const char *)ext_ren_filename, "wb");
        if (g_pf_ext_ren_hoa_md == NULL)
        {
          return -1;
        }
        ext_ren_filename[0] = '\0';
      }
      if (pstr_out_cfg->ch_data_present)
      {
        strcat((char *)ext_ren_filename, (const char *)out_filename);
        strcat((char *)ext_ren_filename, (const char *)"_ext_ren_ch_md.bs");
        g_pf_ext_ren_ch_md = fopen((const char *)ext_ren_filename, "wb");
        if (g_pf_ext_ren_ch_md == NULL)
        {
          return -1;
        }
        ext_ren_filename[0] = '\0';
      }
      strcat((char *)ext_ren_filename, (const char *)out_filename);
      strcat((char *)ext_ren_filename, (const char *)"_ext_ren_pcm.raw");
      g_pf_ext_ren_pcm = fopen((const char *)ext_ren_filename, "wb");
      if (g_pf_ext_ren_pcm == NULL)
      {
        return -1;
      }
      ext_ren_filename[0] = '\0';
    }

    _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

    err_code_reinit = err_code;
  }
  fprintf(stderr, "Decoder Initialization complete\r");
/* End of second part */

#ifdef PRINT_SPEAKER_INFO
  /* Print speaker configuration */
  printf("\nNumber of speakers : %d \n", pstr_out_cfg->num_speakers);
  printf("CICP Index : %d \n", pstr_out_cfg->cicp_index);
  printf("Speaker layout : %d \n", pstr_out_cfg->spk_layout);
  printf("Binaural rendering : %d \n", pstr_out_cfg->is_binaural_rendering);
  if (1 != pstr_out_cfg->is_binaural_rendering)
  {
    for (WORD32 spk = 0; spk < pstr_out_cfg->num_speakers; spk++)
    {
      printf("Azimuth : %d Elevation : %d LFE Channel : %d\n", pstr_out_cfg->azimuth[spk],
             pstr_out_cfg->elevation[spk], pstr_out_cfg->is_lfe[spk]);
    }
  }
#endif

  // This condition is added so as to avoid re-writing wave header in
  // middle of wave file in case of errors and when we are not opening
  // new file in case of errors.
  err_code = write_wav_header(g_pf_out, 0, pstr_out_cfg->i_samp_freq, pstr_out_cfg->i_num_chan,
                              pstr_out_cfg->i_pcm_wd_sz);

  _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

  /* ******************************************************************/
  /* START EXECUTION OF MPEG-H 3D Audio Low Complexity Profile DECODER */
  /* ******************************************************************/
  fprintf(stderr, "\n");
  do
  {
    if (((WORD32)ui_inp_size - (WORD32)(i_buff_size - pstr_out_cfg->i_bytes_consumed)) > 0)
    {
      for (i = 0; i < (i_buff_size - pstr_out_cfg->i_bytes_consumed); i++)
      {
        pb_inp_buf[i] = pb_inp_buf[i + pstr_out_cfg->i_bytes_consumed];
      }
      g_pf_inp->is_execution = 1;
      err_code = impeghd_mp4_fw_read(
          g_pf_inp, (pUWORD8)(pb_inp_buf + i_buff_size - pstr_out_cfg->i_bytes_consumed),
          ((WORD32)ui_inp_size - (WORD32)(i_buff_size - pstr_out_cfg->i_bytes_consumed)),
          (pUWORD32)&i_bytes_read);
      i_buff_size = i_buff_size - (pstr_out_cfg->i_bytes_consumed - i_bytes_read);
    }
    pstr_in_cfg->num_inp_bytes = i_buff_size;

    ARM_PROFILE_HW_START;
    if (pstr_in_cfg->num_inp_bytes <= 0)
      break;
    /* Execute process */
    err_code =
        ia_mpegh_dec_execute(pv_ia_process_api_obj, (pVOID)pstr_in_cfg, (pVOID)pstr_out_cfg);
    _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);
    if (err_code == IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_SD_BYTES ||
        err_code == IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_HOA_BYTES ||
        err_code == IA_MPEGH_DEC_EXE_NONFATAL_INSUFFICIENT_METADATA_BYTES)
    {
      goto exit_path;
    }

    ARM_PROFILE_HW_STOP;
    if (pstr_in_cfg->extrn_rend_flag)
    {
      if (pstr_out_cfg->oam_data_present)
      {
        if (g_pf_ext_ren_oam_md != NULL)
        {
          (VOID) fwrite(pstr_in_cfg->ptr_ext_ren_oam_data_buf, 1,
                        pstr_out_cfg->oam_md_payload_length, g_pf_ext_ren_oam_md);
        }
      }
      if (pstr_out_cfg->hoa_data_present)
      {
        if (g_pf_ext_ren_hoa_md != NULL)
        {
          (VOID) fwrite(pstr_in_cfg->ptr_ext_ren_hoa_data_buf, 1,
                        pstr_out_cfg->hoa_md_payload_length, g_pf_ext_ren_hoa_md);
        }
      }
      if (pstr_out_cfg->ch_data_present)
      {
        if (g_pf_ext_ren_ch_md != NULL)
        {
          (VOID) fwrite(pstr_in_cfg->ptr_ext_ren_ch_data_buf, 1,
                        pstr_out_cfg->ch_md_payload_length, g_pf_ext_ren_ch_md);
        }
      }
      if (g_pf_ext_ren_pcm != NULL)
      {
        fwrite(pstr_in_cfg->ptr_ext_ren_pcm_buf, 1, pstr_out_cfg->pcm_payload_length,
               g_pf_ext_ren_pcm);
      }
    }
    if (err_code_reinit != IA_MPEGH_DEC_NO_ERROR)
      memset(pb_out_buf, 0, pstr_out_cfg->num_out_bytes);
    i_total_bytes += pstr_out_cfg->num_out_bytes;
    if (pstr_out_cfg->num_out_bytes > 0)
    {
#ifndef ARM_PROFILE_HW
      fwrite(pb_out_buf, sizeof(WORD8), pstr_out_cfg->num_out_bytes, g_pf_out);
      fflush(g_pf_out);
#endif
    }

    frame_counter++;
    fprintf(stderr, "Frames Processed : [%5d] \r", frame_counter);

    ARM_PROFILE_HW_CALC_CYCLES(pstr_out_cfg->num_out_bytes, pstr_in_cfg->ui_pcm_wd_sz,
                               pstr_out_cfg->i_samp_freq, pstr_out_cfg->i_num_chan);

    /* Do till the process execution is done */
  } while (i_buff_size > 0);

  if (pstr_in_cfg->extrn_rend_flag)
  {
    if (pstr_out_cfg->oam_data_present)
    {
      if (g_pf_ext_ren_oam_md != NULL)
      {
        (VOID) fclose(g_pf_ext_ren_oam_md);
      }
    }
    if (pstr_out_cfg->hoa_data_present)
    {
      if (g_pf_ext_ren_hoa_md != NULL)
      {
        (VOID) fclose(g_pf_ext_ren_hoa_md);
      }
    }
    if (pstr_out_cfg->ch_data_present)
    {
      if (g_pf_ext_ren_ch_md != NULL)
      {
        (VOID) fclose(g_pf_ext_ren_ch_md);
      }
    }
    if (g_pf_ext_ren_pcm)
    {
      (VOID) fclose(g_pf_ext_ren_pcm);
    }
  }

  ARM_PROFILE_HW_PRINT_PROF_DATA
  ARM_PROFILE_PRINT_ONCE(pstr_out_cfg->num_out_bytes, pstr_in_cfg->ui_pcm_wd_sz,
                         pstr_out_cfg->i_samp_freq, pstr_out_cfg->i_num_chan);

  fprintf(stderr, "Frames Processed : [%5d] \n", frame_counter);
  if (!fseek(g_pf_out, 0, SEEK_SET))
  {
    err_code = write_wav_header(g_pf_out, i_total_bytes, pstr_out_cfg->i_samp_freq,
                                pstr_out_cfg->i_num_chan, pstr_out_cfg->i_pcm_wd_sz);
    _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);
  }
exit_path:
  err_code = ia_mpegh_dec_delete((pVOID)pstr_out_cfg);
  _IA_HANDLE_ERROR(p_proc_err_info, (pWORD8) "", err_code);

  if (str_dec_api.input_config.ei_info_flag && str_dec_api.input_config.ptr_ei_buf)
  {
    free(str_dec_api.input_config.ptr_ei_buf);
  }
  if (str_dec_api.input_config.lsi_info_flag && str_dec_api.input_config.ptr_ls_buf)
  {
    free(str_dec_api.input_config.ptr_ls_buf);
  }
  if (str_dec_api.input_config.sd_info_flag && str_dec_api.input_config.ptr_sd_buf)
  {
    free(str_dec_api.input_config.ptr_sd_buf);
  }
  if (str_dec_api.input_config.extrn_rend_flag && str_dec_api.output_config.ch_data_present)
  {
    free(str_dec_api.input_config.ptr_ext_ren_ch_data_buf);
  }
  if (str_dec_api.input_config.extrn_rend_flag && str_dec_api.output_config.oam_data_present)
  {
    free(str_dec_api.input_config.ptr_ext_ren_oam_data_buf);
  }
  if (str_dec_api.input_config.extrn_rend_flag && str_dec_api.output_config.hoa_data_present)
  {
    free(str_dec_api.input_config.ptr_ext_ren_hoa_data_buf);
  }
  if (str_dec_api.input_config.binaural_flag && str_dec_api.input_config.ptr_brir_buf)
  {
    free(str_dec_api.input_config.ptr_brir_buf);
  }
  if (str_dec_api.input_config.ptr_maeg_buf)
  {
    free(str_dec_api.input_config.ptr_maeg_buf);
  }
  if (str_dec_api.input_config.ptr_maei_buf)
  {
    free(str_dec_api.input_config.ptr_maei_buf);
  }
  if (str_dec_api.input_config.ptr_maes_buf)
  {
    free(str_dec_api.input_config.ptr_maes_buf);
  }
  if (str_dec_api.input_config.ptr_maep_buf)
  {
    free(str_dec_api.input_config.ptr_maep_buf);
  }

  return err_code;
}

/**
*  print_usage
*
*  \brief Prints the usage of the executable
*
*
*
*/
VOID print_usage()
{
  ia_output_config str_output_config = {0};

  /* Get library id and version number */
  ia_mpegh_dec_get_lib_id_strings(&str_output_config);
#ifdef DISPLAY_MESSAGE
  ia_display_id_message(str_output_config.p_lib_name, str_output_config.p_version_num);
#endif
  printf("\n Usage \n");
  printf("\n <exceutable> -ifile:<input_file> -ofile:<out_file> [options]\n");
  printf("\n[options] can be,");
  printf("\n[-pcmsz:<pcmwordsize>]");
  printf("\n[-target_loudness:<target_loudness>]");
  printf("\n[-effect:<drc_effect_type>]");
  printf("\n[-cicp:<target_layout>]");
  printf("\n[-ilsi:<lsi_file>]");
  printf("\n[-isdi:<sd_file>]");
  printf("\n[-iei:<ei_file>]");
  printf("\n[-ibrir:<brir_file>]");
  printf("\n[-out_fs:<output_samp_freq>]");
  printf("\n[-ext_ren:<extrn_rend_flag>]");
  printf("\n\nwhere, \n  <inputfile>        is the input MPEGH file name.");
  printf("\n  <outputfile>       is the output file name.");
  printf("\n  <pcmwordsize>      is the bits per sample info.");
  printf("\n  <target_loudness>  is target loudness in dB.");
  printf("\n  <drc_effect_type>  is drc effect type.");
  printf("\n  <target_layout>    is target speaker layout.");
  printf("\n \tDescription in format Front/Surr.LFE"
         "\n \t1: 1/0.0    - C"
         "\n \t2: 2/0.0    - L, R"
         "\n \t3: 3/0.0    - C, L, R"
         "\n \t4: 3/1.0    - C, L, R, Cs"
         "\n \t5: 3/2.0    - C, L, R, Ls, Rs"
         "\n \t6: 3/2.1    - C, L, R, Ls, Rs, LFE"
         "\n \t7: 5/2.1    - C, Lc, Rc, L, R, Ls, Rs, LFE"
         "\n \t8: NA"
         "\n \t9: 2/1.0    - L, R, Cs"
         "\n \t10: 2/2.0   - L, R, Ls, Rs"
         "\n \t11: 3/3.1   - C, L, R, Ls, Rs, Cs, LFE"
         "\n \t12: 3/4.1   - C, L, R, Ls, Rs, Lsr, Rsr, LFE"
         "\n \t13: 11/11.2 - C, Lc, Rc, L, R, Lss, Rss, Lsr, Rsr, Cs, LFE, LFE2, Cv, Lv, Rv, "
         "\n \t              Lvss, Rvss, Ts, Lvr, Rvr, Cvr, Cb, Lb, Rb"
         "\n \t14: 5/2.1   - C, L, R, Ls, Rs, LFE, Lv, Rv"
         "\n \t15: 5/5.2   - C, L, R, Lss, Rss, Ls, Rs, Lv, Rv, Cvr, LFE, LFE2"
         "\n \t16: 5/4.1   - C, L, R, Ls, Rs, LFE, Lv, Rv, Lvs, Rvs"
         "\n \t17: 6/5.1   - C, L, R, Ls, Rs, LFE, Lv, Rv, Cv, Lvs, Rvs, Ts"
         "\n \t18: 6/7.1   - C, L, R, Ls, Rs, Lbs, Rbs, LFE, Lv, Rv, Cv, Lvs, Rvs, Ts"
         "\n \t19: 5/6.1   - C, L, R, Lss, Rss, Lsr, Rsr, LFE, Lv, Rv, Lvr, Rvr"
         "\n \t20: 7/6.1   - C, Leos, Reos, L, R, Lss, Rss, Lsr, Rsr, LFE, Lv, Rv, Lvs, Rvs"
         "\n \tNote: CICP 13 is applicable for baseline profile streams with only object audio.");
  printf("\n  <lsi_file>         bitstream file with local setup information.");
  printf("\n  <sd_file>          bitstream file with scene displacement information.");
  printf("\n  <ei_file>          bitstream file with element interaction information.");
  printf("\n  <brir_file>        bitstream file with binaural renderer impulse response data.");
  printf("\n  Note: The <lsi_file>, <sd_file>, <ei_file> and <brir_file> are bitstream files"
         "\n        expected from the user."
         "\n        The bitstream syntax is specified in the MPEG-H specification document.");
  printf("\n  <output_samp_freq> desired sampling frequency of the output stream - ");
  printf("\n                     invokes resampler if needed.");
  printf("\n  <extrn_rend_flag>  flag to enable external rendering interfaces creation.");
  printf("\n                     Enabling this flag creates external rendering interface");
  printf("\n                     bitstreams in the same location as decoder executable.");
}

/**
*  main
*
*  \brief Main function
*
*  \param [in] argc     Argument count
*  \param [in] argv     Pointer to the argument strings
*
*  \return IA_ERRORCODE Processing error if any
*
*/
IA_ERRORCODE main(WORD32 argc, char *argv[])
{
  FILE *param_file_id = NULL;

  WORD8 curr_cmd[IA_MAX_CMD_LINE_LENGTH];
  WORD32 fargc, curpos;
  WORD32 processcmd = 0;

  WORD8 fargv[IA_MAX_ARGS][IA_MAX_CMD_LINE_LENGTH];

  pWORD8 pargv[IA_MAX_ARGS];

  WORD8 pb_input_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
  WORD8 pb_output_file_path[IA_MAX_CMD_LINE_LENGTH] = "";
  ia_testbench_error_handler_init();
  g_pf_inp = NULL;
  g_pf_out = NULL;
  if (argc < 3)
  {
    if ((argc == 2) && (!strncmp((const char *)argv[1], "-paramfile:", 11)))
    {
      pWORD8 paramfile = (pWORD8)argv[1] + 11;

      param_file_id = fopen((const char *)paramfile, "r");
      if (param_file_id == NULL)
      {
        print_usage();
        return IA_MPEGH_DEC_NO_ERROR;
      }
    }
    else
    {
      param_file_id = fopen(PARAMFILE, "r");
      if (param_file_id == NULL)
      {
        print_usage();
        return IA_MPEGH_DEC_NO_ERROR;
      }
    }

    /* Process one line at a time */
    while (fgets((char *)curr_cmd, IA_MAX_CMD_LINE_LENGTH, param_file_id))
    {
      curpos = 0;
      fargc = 0;
      /* if it is not a param_file command and if */
      /* CLP processing is not enabled */
      if (curr_cmd[0] != '@' && !processcmd)
      { /* skip it */
        continue;
      }

      while (sscanf((char *)curr_cmd + curpos, "%s", fargv[fargc]) != EOF)
      {
        if (fargv[0][0] == '/' && fargv[0][1] == '/')
          break;
        if (strcmp((const char *)fargv[0], "@echo") == 0)
          break;
        if (strcmp((const char *)fargv[fargc], "@New_line") == 0)
        {
          if (fgets((char *)curr_cmd + curpos, IA_MAX_CMD_LINE_LENGTH, param_file_id) == NULL)
          {
            impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                  (pWORD8) "Paramfile new line ",
                                  IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
            exit(1);
          }
          continue;
        }
        curpos += (WORD32)strlen((const char *)fargv[fargc]);
        while (*(curr_cmd + curpos) == ' ' || *(curr_cmd + curpos) == '\t')
          curpos++;
        fargc++;
      }

      if (fargc < 1) /* for blank lines etc. */
        continue;

      if (strcmp((const char *)fargv[0], "@Output_path") == 0)
      {
        if (fargc > 1)
          strcpy((char *)pb_output_file_path, (const char *)fargv[1]);
        else
          strcpy((char *)pb_output_file_path, "");
        continue;
      }

      if (strcmp((const char *)fargv[0], "@Input_path") == 0)
      {
        if (fargc > 1)
          strcpy((char *)pb_input_file_path, (const char *)fargv[1]);
        else
          strcpy((char *)pb_input_file_path, "");
        continue;
      }

      if (strcmp((const char *)fargv[0], "@Start") == 0)
      {
        processcmd = 1;
        continue;
      }

      if (strcmp((const char *)fargv[0], "@Stop") == 0)
      {
        processcmd = 0;
        continue;
      }

      /* otherwise if this a normal command and its enabled for execution */
      if (processcmd)
      {
        WORD32 i;
        IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;
        WORD32 file_count = 0;

        for (i = 0; i < fargc; i++)
        {
          printf("%s ", fargv[i]);
          pargv[i] = fargv[i];

          if (!strncmp((const char *)fargv[i], "-ifile:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;
            WORD8 pb_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
            raw_testing = 0;
            strcat((char *)pb_input_file_name, (const char *)pb_input_file_path);
            strcat((char *)pb_input_file_name, (const char *)pb_arg_val);
            g_pf_inp = impeghd_mp4_fw_open((pWORD8)pb_input_file_name);
            if (g_pf_inp == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input File",
                                    err_code);
              exit(1);
            }
            file_count++;
            if (g_pf_inp->is_mp4_file)
            {
              if (g_pf_inp->is_mp4_mhm1)
              {
                raw_testing = 0;
              }
              else
              {
                raw_testing = 1;
              }
            }
          }

          if (!strncmp((const char *)fargv[i], "-ofile:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;
            WORD8 pb_output_file_name[IA_MAX_CMD_LINE_LENGTH] = "";

            strcat((char *)pb_output_file_name, (const char *)pb_output_file_path);
            strcat((char *)pb_output_file_name, (const char *)pb_arg_val);

            size_t len = strlen((const char *)pb_output_file_name);
            memcpy(out_filename, pb_output_file_name, len - 4);

            g_pf_out = fopen((const char *)pb_output_file_name, "wb");
            if (g_pf_out == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Output File",
                                    err_code);
              exit(1);
            }
            file_count++;
          }
          if (!strncmp((const char *)fargv[i], "-ibrir:", 7))
          {
            pWORD8 pb_arg_val = fargv[i] + 7;
            WORD8 pb_brir_file_name[IA_MAX_CMD_LINE_LENGTH] = "";

            strcat((char *)pb_brir_file_name, (const char *)pb_input_file_path);
            strcat((char *)pb_brir_file_name, (const char *)pb_arg_val);

            g_pf_brir = fopen((const char *)pb_brir_file_name, "rb");

            if (g_pf_brir == NULL)
            {
              err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
              impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "BRIR File",
                                    err_code);
              exit(1);
            }
          }
          if (!strncmp((const char *)fargv[i], "-iei:", 5))
          {
            pWORD8 pb_arg_val = fargv[i] + 5;
            WORD8 pb_ei_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
            strcat((char *)pb_ei_input_file_name, (const char *)pb_input_file_path);
            strcat((char *)pb_ei_input_file_name, (const char *)pb_arg_val);
            g_pf_ei_bs = fopen((char *)pb_ei_input_file_name, "rb");
            if (g_pf_ei_bs == NULL)
            {
              impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                    (pWORD8) "Element Interaction bit stream File",
                                    IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
              exit(1);
            }
          }
          if (!strncmp((const char *)fargv[i], "-ilsi:", 6))
          {
            pWORD8 pb_arg_val = fargv[i] + 6;
            WORD8 pb_lsi_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
            strcat((char *)pb_lsi_input_file_name, (const char *)pb_input_file_path);
            strcat((char *)pb_lsi_input_file_name, (const char *)pb_arg_val);
            g_pf_lsi_bs = fopen((char *)pb_lsi_input_file_name, "rb");
            if (g_pf_lsi_bs == NULL)
            {
              impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                    (pWORD8) "Local Setup Information bit stream file",
                                    IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
              exit(1);
            }
          }
          if (!strncmp((const char *)fargv[i], "-isdi:", 6))
          {
            pWORD8 pb_arg_val = fargv[i] + 6;
            WORD8 pb_sdi_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
            strcat((char *)pb_sdi_input_file_name, (const char *)pb_input_file_path);
            strcat((char *)pb_sdi_input_file_name, (const char *)pb_arg_val);
            g_pf_sd_bs = fopen((char *)pb_sdi_input_file_name, "rb");
            if (g_pf_sd_bs == NULL)
            {
              impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                    (pWORD8) "Scene Displacement Information bit stream file",
                                    IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
              exit(1);
            }
          }
        }
        if ((g_pf_inp == NULL) || (g_pf_out == NULL))
        {
          print_usage();
          return IA_MPEGH_DEC_NO_ERROR;
        }

        printf("\n");

        if (file_count < 2)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input or Output File",
                                err_code);
        }
        impeghd_main_process(fargc, pargv);
      }

      if (g_pf_out)
      {
        fclose(g_pf_out);
        g_pf_out = NULL;
      }

      if (g_pf_brir)
      {
        fclose(g_pf_brir);
        g_pf_brir = NULL;
      }

      if (g_pf_ei_bs)
      {
        fclose(g_pf_ei_bs);
        g_pf_ei_bs = NULL;
      }

      if (g_pf_lsi_bs)
      {
        fclose(g_pf_lsi_bs);
        g_pf_lsi_bs = NULL;
      }

      if (g_pf_sd_bs)
      {
        fclose(g_pf_sd_bs);
        g_pf_sd_bs = NULL;
      }

      if (g_pf_inp)
      {
        impeghd_mp4_fw_close(g_pf_inp);
        g_pf_inp = NULL;
      }
    }
  }
  else
  {
    WORD32 i;
    IA_ERRORCODE err_code = IA_MPEGH_DEC_NO_ERROR;

    for (i = 1; i < argc; i++)
    {
      if (!strncmp((const char *)argv[i], "-ifile:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;
        printf("%s ", argv[i]);
        g_pf_inp = impeghd_mp4_fw_open((pWORD8)pb_arg_val);

        if (g_pf_inp == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input File",
                                err_code);
          exit(1);
        }
        raw_testing = 0;
        g_binaural_flag = 0;
        if (g_pf_inp->is_mp4_file)
        {
          if (g_pf_inp->is_mp4_mhm1)
          {
            raw_testing = 0;
          }
          else
          {
            raw_testing = 1;
          }
        }
      }
      if (!strncmp((const char *)argv[i], "-ibrir:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;
        WORD8 pb_brir_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
        printf("%s ", argv[i]);
        strcat((char *)pb_brir_file_name, (const char *)pb_input_file_path);
        strcat((char *)pb_brir_file_name, (const char *)pb_arg_val);

        g_pf_brir = fopen((const char *)pb_brir_file_name, "rb");
        if (g_pf_brir == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "BRIR File", err_code);
          exit(1);
        }
      }
      if (!strncmp((const char *)argv[i], "-iei:", 5))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 5;
        WORD8 pb_ei_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
        printf("%s ", argv[i]);
        strcat((char *)pb_ei_input_file_name, (const char *)pb_input_file_path);
        strcat((char *)pb_ei_input_file_name, (const char *)pb_arg_val);
        g_pf_ei_bs = fopen((char *)pb_ei_input_file_name, "rb");
        if (g_pf_ei_bs == NULL)
        {
          impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                (pWORD8) "Element Interaction bit stream File",
                                IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
          exit(1);
        }
      }
      if (!strncmp((const char *)argv[i], "-ilsi:", 6))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 6;
        WORD8 pb_lsi_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
        printf("%s ", argv[i]);
        strcat((char *)pb_lsi_input_file_name, (const char *)pb_input_file_path);
        strcat((char *)pb_lsi_input_file_name, (const char *)pb_arg_val);
        g_pf_lsi_bs = fopen((char *)pb_lsi_input_file_name, "rb");
        if (g_pf_lsi_bs == NULL)
        {
          impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                (pWORD8) "Local Setup Information bit stream file",
                                IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
          exit(1);
        }
      }
      if (!strncmp((const char *)argv[i], "-isdi:", 6))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 6;
        WORD8 pb_sdi_input_file_name[IA_MAX_CMD_LINE_LENGTH] = "";
        printf("%s ", argv[i]);
        strcat((char *)pb_sdi_input_file_name, (const char *)pb_input_file_path);
        strcat((char *)pb_sdi_input_file_name, (const char *)pb_arg_val);
        g_pf_sd_bs = fopen((char *)pb_sdi_input_file_name, "rb");
        if (g_pf_sd_bs == NULL)
        {
          impeghd_error_handler(&impeghd_ia_testbench_error_info,
                                (pWORD8) "Scene Displacement Information bit stream file",
                                IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED);
          exit(1);
        }
      }
      if (!strncmp((const char *)argv[i], "-ofile:", 7))
      {
        pWORD8 pb_arg_val = (pWORD8)argv[i] + 7;
        size_t len = strlen((const char *)pb_arg_val);
        memcpy(out_filename, pb_arg_val, len - 4);
        printf("%s ", argv[i]);
        g_pf_out = fopen((const char *)pb_arg_val, "wb");
        if (g_pf_out == NULL)
        {
          err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
          impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Output File",
                                err_code);
          exit(1);
        }
      }
      if ((!strncmp((const char *)argv[i], "-help", 5)) ||
          (!strncmp((const char *)argv[i], "-h", 2)))
      {
        print_usage();
        exit(1);
      }
    }

    if ((g_pf_inp == NULL) || (g_pf_out == NULL))
    {
      print_usage();
      err_code = IA_TESTBENCH_MFMAN_FATAL_FILE_OPEN_FAILED;
      impeghd_error_handler(&impeghd_ia_testbench_error_info, (pWORD8) "Input or Output File",
                            err_code);
      exit(1);
    }

    printf("\n");

    impeghd_main_process(argc - 1, (pWORD8 *)(&argv[1]));
    if (g_pf_out)
    {
      fclose(g_pf_out);
      g_pf_out = NULL;
    }

    if (g_pf_brir)
    {
      fclose(g_pf_brir);
      g_pf_brir = NULL;
    }

    if (g_pf_ei_bs)
    {
      fclose(g_pf_ei_bs);
      g_pf_ei_bs = NULL;
    }

    if (g_pf_lsi_bs)
    {
      fclose(g_pf_lsi_bs);
      g_pf_lsi_bs = NULL;
    }

    if (g_pf_sd_bs)
    {
      fclose(g_pf_sd_bs);
      g_pf_sd_bs = NULL;
    }

    if (g_pf_inp)
    {
      impeghd_mp4_fw_close(g_pf_inp);
      g_pf_inp = NULL;
    }
  }
  if (param_file_id != NULL)
  {
    fclose(param_file_id);
  }
  return IA_MPEGH_DEC_NO_ERROR;
} /* end impeghd_main */
/** @} */ /* End of SampleApp */