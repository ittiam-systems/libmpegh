cmake_minimum_required (VERSION 3.8)

project (MPEG-H_Decoder VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_SUPPRESS_REGENERATION true)

# Add Library Target
add_library (
        ia_mpeghd_lib 
        STATIC	
	decoder/impd_drc_dynamic_payload.c
	decoder/impd_drc_extr_delta_coded_info.c					
	decoder/impd_drc_filter_bank.c
	decoder/impd_drc_gain_dec.c
	decoder/impd_drc_host_params.c
	decoder/impd_drc_init.c
	decoder/impd_drc_loudness_control.c
	decoder/impd_drc_multiband.c
	decoder/impd_drc_process.c
	decoder/impd_drc_rom.c
	decoder/impd_drc_selection_process.c
	decoder/impd_drc_selection_process_drcset_selection.c
	decoder/impd_drc_selection_process_init.c
	decoder/impd_drc_static_payload.c
	decoder/impeghd_3d_vec_basic_ops.c
	decoder/impeghd_api.c
	decoder/impeghd_binaural.c
	decoder/impeghd_binaural_filter_design.c
	decoder/impeghd_binaural_renderer.c
	decoder/impeghd_cicp_2_geometry.c
	decoder/impeghd_cicp_2_geometry_rom.c
	decoder/impeghd_dmx_mtx.c
	decoder/impeghd_ele_interaction_intrfc.c
	decoder/impeghd_ext_rend_intrfc.c
	decoder/impeghd_ext_rend_intrfc_util.c
	decoder/impeghd_fft.c
	decoder/impeghd_fft_ifft_rom.c
	decoder/impeghd_format_conv_init.c
	decoder/impeghd_format_conv_rom.c
	decoder/impeghd_hoa_amb_syn.c
	decoder/impeghd_hoa_ch_reassignment.c
	decoder/impeghd_hoa_common_functions.c
	decoder/impeghd_hoa_config.c
	decoder/impeghd_hoa_dec_init.c
	decoder/impeghd_hoa_decoder.c
	decoder/impeghd_hoa_dir_based_pre_dom_sound_syn.c
	decoder/impeghd_hoa_frame.c
	decoder/impeghd_hoa_inverse_dyn_correction.c
	decoder/impeghd_hoa_matrix.c
	decoder/impeghd_hoa_nfc_filtering.c
	decoder/impeghd_hoa_pre_dom_sound_syn.c
	decoder/impeghd_hoa_renderer.c
	decoder/impeghd_hoa_render_mtrx.c
	decoder/impeghd_hoa_robust_pan.c
	decoder/impeghd_hoa_rom.c
	decoder/impeghd_hoa_simple_mtrx.c
	decoder/impeghd_hoa_space_positions.c
	decoder/impeghd_hoa_spatial_decoder.c
	decoder/impeghd_hoa_vector_based_predom_sound_syn.c
	decoder/impeghd_ifft.c
	decoder/impeghd_inv_mtx_cicp_rom.c
	decoder/impeghd_mae_mp4_intrfc.c
	decoder/impeghd_metadata_preprocessor.c
	decoder/impeghd_metadata_preprocessor_rom.c
	decoder/impeghd_mhas_parse.c
	decoder/impeghd_multichannel.c
	decoder/impeghd_multichannel_rom.c
	decoder/impeghd_oam_dec.c
	decoder/impeghd_oam_dec_ld.c
	decoder/impeghd_obj_ren_dec.c
	decoder/impeghd_obj_ren_vbap.c
	decoder/impeghd_peak_limiter.c
	decoder/impeghd_resampler.c
	decoder/impeghd_resampler_rom.c
	decoder/impeghd_scene_displacement.c
	decoder/impeghd_tbe_dec.c
	decoder/impeghd_tbe_rom.c
	decoder/ia_core_coder_acelp_bitparse.c
	decoder/ia_core_coder_acelp_decode.c
	decoder/ia_core_coder_acelp_mdct.c
	decoder/ia_core_coder_acelp_tools.c
	decoder/ia_core_coder_arith_dec.c
	decoder/ia_core_coder_avq_dec.c
	decoder/ia_core_coder_avq_rom.c
	decoder/ia_core_coder_basic_ops.c
	decoder/ia_core_coder_bitbuffer.c
	decoder/ia_core_coder_common_initfuncs.c
	decoder/ia_core_coder_create.c
	decoder/ia_core_coder_decode_main.c
	decoder/ia_core_coder_env_extr.c
	decoder/ia_core_coder_ext_ch_ele.c
	decoder/ia_core_coder_function_selector.c
	decoder/ia_core_coder_fwd_alias_cnx.c
	decoder/ia_core_coder_headerdecode.c
	decoder/ia_core_coder_hufftables_rom.c
	decoder/ia_core_coder_huff_tools.c
	decoder/ia_core_coder_igf_dec.c
	decoder/ia_core_coder_imdct.c
	decoder/ia_core_coder_init_config.c
	decoder/ia_core_coder_lpc.c
	decoder/ia_core_coder_lpc_dec.c
	decoder/ia_core_coder_ltpf.c
	decoder/ia_core_coder_process.c
	decoder/ia_core_coder_rom.c
	decoder/ia_core_coder_spectrum_dec.c
	decoder/ia_core_coder_stereo_lpd.c
	decoder/ia_core_coder_tcx_fwd_alcnx.c
	decoder/ia_core_coder_tcx_fwd_mdct.c
	decoder/ia_core_coder_tns.c
	decoder/ia_core_coder_windowing.c
        )

add_executable (ia_mpeghd_testbench
	test/impeghd_error.c  
	test/impeghd_main.c
	test/mp4/impeghd_mp4_file.c  
	test/mp4/impeghd_mp4_file_wrapper.c  
	test/mp4/impeghd_mp4_init.c  
	test/mp4/impeghd_mp4_mae_parser.c
	test/mp4/impeghd_mp4_parser.c  
	test/mp4/impeghd_mp4_utils.c  
	test/mp4/impeghd_mp4_video.c
	)

if(MSVC)
	target_compile_options(
		ia_mpeghd_lib	
		PRIVATE
		-D_CRT_SECURE_NO_WARNINGS
		-D_X86_
		)
	target_compile_options(
		ia_mpeghd_testbench
		PRIVATE
		-D_CRT_SECURE_NO_WARNINGS
		-D_X86_
		)
else()
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
		target_compile_options(
			ia_mpeghd_lib	
			PRIVATE
			-D_X86_
			-c -O3 -Wall -Wsequence-point
			-Wunused-function 
			)
		target_compile_options(ia_mpeghd_testbench 
			PRIVATE
			-c -O3 -Wall -Wsequence-point 
			-Wunused-function 
			)
		set(PLAT_PATH ${CMAKE_CURRENT_LIST_DIR}/decoder/x86_64) 
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
		target_compile_options(
			ia_mpeghd_lib
			PRIVATE
			-D_X86_
			-c -m32 -O3 -Wall -Wsequence-point
			-Wunused-function 
			)

		set_target_properties(ia_mpeghd_testbench PROPERTIES LINK_FLAGS "-m32")

		target_compile_options(ia_mpeghd_testbench 
			PRIVATE 
			-c -O3 -m32 -Wall -Wsequence-point 
			-Wunused-function
			)
		set(PLAT_PATH ${CMAKE_CURRENT_LIST_DIR}/decoder/x86) 
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
		target_compile_options(
			ia_mpeghd_lib
			PRIVATE
			-D_ARM_
			-c -O3 -mcpu=cortex-a8 -mfloat-abi=hard -Wall 
			-Wsequence-point -Wunused-function --std=gnu99
			)
		target_compile_options(ia_mpeghd_testbench 
			PRIVATE
			-c -O3 -mcpu=cortex-a8  -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing
			-mfloat-abi=hard 
			)
	elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
		target_compile_options(
			ia_mpeghd_lib
			PRIVATE
			-D_ARM_
			-c -O3 -march=armv8-a -Wall -Wsequence-point 
			-Wunused-function --std=gnu99
			)
		target_compile_options(ia_mpeghd_testbench 
			PRIVATE
		        -c -O3 -march=armv8-a -Wall -Wsequence-point 
			-Wunused-function -Wno-strict-aliasing -flto
			)
		set(PLAT_PATH ${CMAKE_CURRENT_LIST_DIR}/decoder/armv8) 
	endif()
endif()

target_include_directories (
        ia_mpeghd_lib
        PUBLIC 
	${PLAT_PATH}
	${CMAKE_CURRENT_LIST_DIR}/decoder 
	${CMAKE_CURRENT_LIST_DIR}/test
        )


target_include_directories (
        ia_mpeghd_testbench 
        PUBLIC 
	${PLAT_PATH}
	${CMAKE_CURRENT_LIST_DIR}/decoder 
	${CMAKE_CURRENT_LIST_DIR}/test
	${CMAKE_CURRENT_LIST_DIR}/test/mp4
        )

if(MSVC)
	target_link_libraries (
		ia_mpeghd_testbench 
		LINK_PUBLIC
		ia_mpeghd_lib
		)
else()
	target_link_libraries (
		ia_mpeghd_testbench 
		LINK_PUBLIC
		ia_mpeghd_lib
		m
		)
endif()

