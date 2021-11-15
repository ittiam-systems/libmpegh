################################################################################
#     Copyright (c) [2020]-[2021] Ittiam Systems Pvt. Ltd.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#  modification, are permitted (subject to the limitations in the
#   disclaimer below) provided that the following conditions are met:
#   *   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#   *   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#   *   Neither the names of Dolby Laboratories, Inc. (or its affiliates),
#   Ittiam Systems Pvt. Ltd. nor the names of its contributors may be used
#   to endorse or promote products derived from this software without
#   specific prior written permission.
#
#   NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED
#   BY THIS LICENSE. YOUR USE OF THE SOFTWARE MAY REQUIRE ADDITIONAL PATENT
#   LICENSE(S) BY THIRD PARTIES, INCLUDING, WITHOUT LIMITATION, DOLBY
#   LABORATORIES, INC. OR ANY OF ITS AFFILIATES. THIS SOFTWARE IS PROVIDED
#   BY ITTIAM SYSTEMS LTD. AND ITS CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
#   IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#   IN NO EVENT SHALL ITTIAM SYSTEMS LTD OR ITS CONTRIBUTORS BE LIABLE FOR
#   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
#   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#   STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
#   IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.
################################################################################

import subprocess,sys,os

#Binary Compare
def binary_compare(output, reference):
    with open(output, 'rb') as file1, open(reference, 'rb') as file2:
        data1 = file1.read()
        data2 = file2.read()
      
    if data1 != data2:
        print("File do not match with reference.")
    else:
        print("File match with reference.")
        
ifile = "-ifile:"
ofile = "-ofile:"   
input_dir = "inp/"
output_dir = "out/"  
ref_dir = "ref/"  
        
#Input files running
for in_file in os.listdir(r"inp/"):
	outname = os.path.splitext(in_file)[0]
	subprocess.call([sys.argv[1], ifile + input_dir + in_file, ofile + output_dir + outname + ".wav"])
	binary_compare(output_dir + outname + ".wav",ref_dir + outname + ".wav")