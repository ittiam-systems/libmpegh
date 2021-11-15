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

#ifndef IMPEGHD_MP4_INIT_H
#define IMPEGHD_MP4_INIT_H

WORD32 impeghd_mp4_read_atom(mp4_info *m_info, VOID **fp, ia_mp4_node **n, ia_mp4_trak_init **p,
                             ia_mp4_mem_node **m, UWORD32 prev_state);
WORD32 impeghd_mp4_read_header_info(pUWORD8 buf, trak_info *m);
WORD32 impeghd_mp4_get_es(pVOID fp, ia_mp4_es_desc **n, ia_mp4_mem_node **m, trak_info *p);
WORD32 impeghd_mp4_read_samples(pVOID fp, ia_mp4_sample_entry **n, ia_mp4_mem_node **m,
                                trak_info *p, maei_info *ptr_mae_info);
WORD32 impeghd_mp4_get_decoder_config_descr(pVOID fp, ia_mp4_es_desc **n, ia_mp4_mem_node **m,
                                            trak_info *p);
WORD32 impeghd_mp4_get_sl_config_descr(pVOID fp, ia_mp4_es_desc **n);
WORD32 impeghd_mp4_get_iod(pVOID *fp, ia_mp4_initial_obj_desc *od, ia_mp4_mem_node **m);

#define IT_NULL 0
#define IT_NONE 1852796517
#define IT_MOOV 1836019574
#define IT_MVHD 1836476516
#define IT_IODS 1768907891
#define IT_TRAK 1953653099
#define IT_TKHD 1953196132
#define IT_TREF 1953654118
#define IT_MDIA 1835297121
#define IT_MDHD 1835296868
#define IT_HDLR 1751411826
#define IT_MINF 1835626086
#define IT_DINF 1684631142
#define IT_DREF 1685218662
#define IT_STBL 1937007212
#define IT_STTS 1937011827
#define IT_STSD 1937011556
#define IT_STSZ 1937011578
#define IT_STSC 1937011555
#define IT_STCO 1937007471
#define IT_CTTS 1668576371
#define IT_STSS 1937011571
#define IT_NMHD 1852663908
#define IT_SMHD 1936549988
#define IT_HMHD 1752000612
#define IT_VMHD 1986881636
#define IT_FREE 1718773093
#define IT_SKIP 1936419184
#define IT_UDTA 1969517665
#define IT_NAME 1851878757
#define IT_HINF 1751740006
#define IT_HNTI 1752069225
#define IT_MP4V 1836070006
#define IT_MP4A 1836069985
#define IT_MP4S 1836070003
#define IT_DPND 1685089892
#define IT_IPIR 1768974706
#define IT_MPOD 1836085092
#define IT_SYNC 1937337955
#define IT_EDTS 1701082227
#define IT_ELTS 1701606515
#define IT_ALL 1634495520
#define IT_STSH 1937011560
#define IT_STDP 1937007728
#define IT_MJPA 1835692129
#define IT_ODSM 1868854125
#define IT_SDSM 1935962989
#define IT_VIDE 1986618469
#define IT_SOUN 1936684398
#define IT_HINT 1751740020
#define IT_TRPY 1953656953
#define IT_NUMP 1853189488
#define IT_TPYL 1953528172
#define IT_MAXR 1835104370
#define IT_DMED 1684890980
#define IT_DIMM 1684630893
#define IT_DREP 1685218672
#define IT_TMIN 1953327470
#define IT_TMAX 1953325432
#define IT_PMAX 1886216568
#define IT_DMAX 1684889976
#define IT_PAYT 1885436276
#define IT_RTP 1920233504
#define IT_MHM1 1835560241
#define IT_TIMS 1953066355
#define IT_TSRO 1953722991
#define IT_SNRO 1936618095
#define IT_MHAC 1835557187
#define IT_BTRT 1651798644
#define IT_ESDS 1702061171
#define IT_MAEI 1835099465
#define IT_MAEG 1835099463
#define IT_MAES 1835099475
#define IT_MAEP 1835099472
#define IT_MAEL 1835099468
#endif /* IMPEGHD_MP4_INIT_H */
