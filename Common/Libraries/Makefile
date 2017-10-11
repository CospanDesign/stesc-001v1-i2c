CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

###########################################

vpath %.c CMSIS/CM3/CoreSupport STM32_EVAL/Common STM32_EVAL STM32_EVAL/STM32303C CMSIS/CMSIS_2_x/DSP_Lib/Source/CommonTables CMSIS/CMSIS_2_x/DSP_Lib/Source/ComplexMathFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/BasicMathFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/SupportFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/StatisticsFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/ControllerFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/FastMathFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/TransformFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/MatrixFunctions CMSIS/CMSIS_2_x/DSP_Lib/Source/FilteringFunctions STM32F30x_StdPeriph_Driver/src

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -march=armv7e-m
CFLAGS += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -ICMSIS/CM3/CoreSupport
CFLAGS += -ICMSIS/CMSIS_2_x/Include
CFLAGS += -ICMSIS/CMSIS_2_x/Device/ST/STM32F30x/Include
CFLAGS += -ISTM32F30x_StdPeriph_Driver/inc
CFLAGS += -ISTM32_EVAL/Common
CFLAGS += -ISTM32_EVAL/STM32303C_EVAL
CFLAGS += -I../../Web/Project
CFLAGS += -I../../Web/SystemDriveParams
CFLAGS += -DARM_MATH_CM4 -DUSE_STDPERIPH_DRIVER -DSTM32F30X -DUSE_STM32303C_EVAL -DCCRAM

SRCS =	arm_const_structs.c \
				arm_common_tables.c \
				arm_cmplx_mag_squared_f32.c \
				arm_cmplx_conj_q15.c \
				arm_cmplx_mag_squared_q15.c \
				arm_cmplx_mag_f32.c \
				arm_cmplx_mult_real_q15.c \
				arm_cmplx_mult_cmplx_q31.c \
				arm_cmplx_mult_cmplx_f32.c \
				arm_cmplx_dot_prod_q15.c \
				arm_cmplx_mult_real_q31.c \
				arm_cmplx_mult_cmplx_q15.c \
				arm_cmplx_mag_q15.c \
				arm_cmplx_mag_q31.c \
				arm_cmplx_dot_prod_q31.c \
				arm_cmplx_conj_q31.c \
				arm_cmplx_mag_squared_q31.c \
				arm_cmplx_dot_prod_f32.c \
				arm_cmplx_mult_real_f32.c \
				arm_cmplx_conj_f32.c \
				arm_mult_f32.c \
				arm_sub_q31.c \
				arm_abs_q15.c \
				arm_sub_q7.c \
				arm_mult_q7.c \
				arm_sub_f32.c \
				arm_abs_f32.c \
				arm_offset_f32.c \
				arm_negate_f32.c \
				arm_shift_q31.c \
				arm_shift_q15.c \
				arm_mult_q15.c \
				arm_offset_q7.c \
				arm_add_q15.c \
				arm_add_q7.c \
				arm_abs_q31.c \
				arm_offset_q31.c \
				arm_dot_prod_f32.c \
				arm_negate_q31.c \
				arm_abs_q7.c \
				arm_add_f32.c \
				arm_sub_q15.c \
				arm_dot_prod_q31.c \
				arm_scale_q7.c \
				arm_negate_q15.c \
				arm_scale_f32.c \
				arm_offset_q15.c \
				arm_dot_prod_q15.c \
				arm_shift_q7.c \
				arm_add_q31.c \
				arm_scale_q15.c \
				arm_dot_prod_q7.c \
				arm_negate_q7.c \
				arm_mult_q31.c \
				arm_scale_q31.c \
				arm_q7_to_q31.c \
				arm_copy_q7.c \
				arm_q7_to_q15.c \
				arm_float_to_q15.c \
				arm_q15_to_float.c \
				arm_fill_q7.c \
				arm_q15_to_q7.c \
				arm_float_to_q31.c \
				arm_q31_to_q7.c \
				arm_q31_to_q15.c \
				arm_float_to_q7.c \
				arm_fill_q31.c \
				arm_q7_to_float.c \
				arm_q15_to_q31.c \
				arm_q31_to_float.c \
				arm_fill_q15.c \
				arm_fill_f32.c \
				arm_copy_q15.c \
				arm_copy_q31.c \
				arm_copy_f32.c \
				arm_min_q7.c \
				arm_min_q15.c \
				arm_std_q31.c \
				arm_min_q31.c \
				arm_power_q7.c \
				arm_max_q7.c \
				arm_std_q15.c \
				arm_rms_f32.c \
				arm_max_q15.c \
				arm_max_q31.c \
				arm_std_f32.c \
				arm_mean_q15.c \
				arm_power_q15.c \
				arm_var_q15.c \
				arm_var_f32.c \
				arm_min_f32.c \
				arm_rms_q15.c \
				arm_rms_q31.c \
				arm_max_f32.c \
				arm_mean_f32.c \
				arm_var_q31.c \
				arm_power_q31.c \
				arm_power_f32.c \
				arm_mean_q7.c \
				arm_mean_q31.c \
				arm_sin_cos_q31.c \
				arm_pid_reset_q31.c \
				arm_pid_init_q31.c \
				arm_pid_init_q15.c \
				arm_pid_reset_q15.c \
				arm_pid_init_f32.c \
				arm_sin_cos_f32.c \
				arm_pid_reset_f32.c \
				arm_sin_f32.c \
				arm_sqrt_q31.c \
				arm_sin_q15.c \
				arm_sqrt_q15.c \
				arm_cos_q31.c \
				arm_cos_q15.c \
				arm_cos_f32.c \
				arm_sin_q31.c \
				arm_rfft_q15.c \
				arm_dct4_q31.c \
				arm_rfft_q31.c \
				arm_cfft_radix4_init_q31.c \
				arm_cfft_q15.c \
				arm_bitreversal.c \
				arm_dct4_init_q15.c \
				arm_rfft_init_q31.c \
				arm_cfft_f32.c \
				arm_rfft_fast_f32.c \
				arm_cfft_radix4_init_f32.c \
				arm_cfft_radix8_f32.c \
				arm_cfft_radix4_f32.c \
				arm_cfft_radix2_q31.c \
				arm_cfft_radix2_init_q15.c \
				arm_rfft_init_f32.c \
				arm_rfft_fast_init_f32.c \
				arm_cfft_radix4_init_q15.c \
				arm_rfft_init_q15.c \
				arm_cfft_radix2_init_q31.c \
				arm_rfft_f32.c \
				arm_cfft_radix4_q15.c \
				arm_cfft_q31.c \
				arm_dct4_init_f32.c \
				arm_dct4_init_q31.c \
				arm_dct4_f32.c \
				arm_cfft_radix2_f32.c \
				arm_dct4_q15.c \
				arm_cfft_radix2_init_f32.c \
				arm_cfft_radix4_q31.c \
				arm_cfft_radix2_q15.c \
				arm_mat_scale_q15.c \
				arm_mat_mult_fast_q31.c \
				arm_mat_trans_f32.c \
				arm_mat_add_q31.c \
				arm_mat_mult_f32.c \
				arm_mat_mult_q31.c \
				arm_mat_sub_q31.c \
				arm_mat_init_f32.c \
				arm_mat_trans_q31.c \
				arm_mat_sub_f32.c \
				arm_mat_scale_f32.c \
				arm_mat_cmplx_mult_q15.c \
				arm_mat_inverse_f32.c \
				arm_mat_scale_q31.c \
				arm_mat_mult_fast_q15.c \
				arm_mat_init_q31.c \
				arm_mat_trans_q15.c \
				arm_mat_init_q15.c \
				arm_mat_sub_q15.c \
				arm_mat_cmplx_mult_f32.c \
				arm_mat_mult_q15.c \
				arm_mat_add_f32.c \
				arm_mat_add_q15.c \
				arm_mat_inverse_f64.c \
				arm_mat_cmplx_mult_q31.c \
				arm_fir_sparse_q7.c \
				arm_fir_decimate_fast_q15.c \
				arm_iir_lattice_f32.c \
				arm_correlate_q31.c \
				arm_conv_partial_opt_q15.c \
				arm_fir_sparse_init_q31.c \
				arm_lms_norm_q15.c \
				arm_fir_init_f32.c \
				arm_fir_f32.c \
				arm_fir_interpolate_q15.c \
				arm_fir_sparse_q31.c \
				arm_fir_decimate_init_f32.c \
				arm_conv_partial_fast_opt_q15.c \
				arm_biquad_cascade_df2T_init_f64.c \
				arm_fir_decimate_q31.c \
				arm_conv_partial_fast_q15.c \
				arm_fir_sparse_init_q7.c \
				arm_fir_sparse_init_f32.c \
				arm_fir_lattice_init_f32.c \
				arm_lms_q31.c \
				arm_conv_partial_q7.c \
				arm_fir_lattice_q31.c \
				arm_conv_partial_fast_q31.c \
				arm_fir_q7.c \
				arm_lms_norm_q31.c \
				arm_correlate_fast_q15.c \
				arm_fir_lattice_q15.c \
				arm_biquad_cascade_stereo_df2T_f32.c \
				arm_fir_q31.c \
				arm_fir_init_q31.c \
				arm_conv_q15.c \
				arm_biquad_cascade_df1_q15.c \
				arm_correlate_opt_q15.c \
				arm_iir_lattice_init_q31.c \
				arm_conv_fast_q15.c \
				arm_fir_interpolate_f32.c \
				arm_lms_norm_init_q15.c \
				arm_conv_q31.c \
				arm_correlate_q15.c \
				arm_iir_lattice_q15.c \
				arm_biquad_cascade_stereo_df2T_init_f32.c \
				arm_fir_decimate_f32.c \
				arm_fir_lattice_init_q15.c \
				arm_biquad_cascade_df1_init_q31.c \
				arm_biquad_cascade_df1_init_q15.c \
				arm_fir_fast_q31.c \
				arm_fir_decimate_q15.c \
				arm_iir_lattice_init_f32.c \
				arm_fir_sparse_init_q15.c \
				arm_lms_init_q15.c \
				arm_fir_lattice_init_q31.c \
				arm_biquad_cascade_df1_init_f32.c \
				arm_correlate_opt_q7.c \
				arm_conv_partial_opt_q7.c \
				arm_fir_interpolate_init_f32.c \
				arm_correlate_fast_q31.c \
				arm_fir_init_q15.c \
				arm_biquad_cascade_df1_32x64_init_q31.c \
				arm_fir_q15.c \
				arm_conv_opt_q7.c \
				arm_lms_norm_init_f32.c \
				arm_biquad_cascade_df2T_f64.c \
				arm_biquad_cascade_df1_f32.c \
				arm_correlate_f32.c \
				arm_fir_interpolate_init_q31.c \
				arm_fir_sparse_q15.c \
				arm_fir_init_q7.c \
				arm_biquad_cascade_df2T_init_f32.c \
				arm_correlate_q7.c \
				arm_conv_fast_q31.c \
				arm_lms_q15.c \
				arm_conv_opt_q15.c \
				arm_correlate_fast_opt_q15.c \
				arm_fir_decimate_fast_q31.c \
				arm_fir_sparse_f32.c \
				arm_fir_interpolate_q31.c \
				arm_conv_f32.c \
				arm_biquad_cascade_df1_32x64_q31.c \
				arm_lms_norm_init_q31.c \
				arm_lms_init_q31.c \
				arm_iir_lattice_init_q15.c \
				arm_lms_norm_f32.c \
				arm_conv_partial_f32.c \
				arm_fir_fast_q15.c \
				arm_lms_init_f32.c \
				arm_fir_decimate_init_q31.c \
				arm_biquad_cascade_df1_q31.c \
				arm_biquad_cascade_df2T_f32.c \
				arm_conv_partial_q15.c \
				arm_fir_lattice_f32.c \
				arm_biquad_cascade_df1_fast_q15.c \
				arm_biquad_cascade_df1_fast_q31.c \
				arm_conv_partial_q31.c \
				arm_fir_decimate_init_q15.c \
				arm_conv_q7.c \
				arm_lms_f32.c \
				arm_fir_interpolate_init_q15.c \
				arm_iir_lattice_q31.c \
				arm_conv_fast_opt_q15.c \
				core_cm3.c

OBJS = $(addprefix objs/,$(SRCS:.c=.o))
DEPS = $(addprefix deps/,$(SRCS:.c=.d))

.PHONY: all clean

all: libstm32f3.a

-include $(DEPS)

dirs:
	mkdir -p deps objs
	touch dirs

objs/%.o : %.c dirs
	$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF deps/$(*F).d

libstm32f3.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f dirs $(OBJS) $(DEPS) libstm32f3.a


