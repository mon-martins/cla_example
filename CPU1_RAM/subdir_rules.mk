################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1806428945: ../cla_example.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/ti/sysconfig_1.24.0/sysconfig_cli.bat" --script "C:/Users/ramon/Documents/git/codigos/cla_example/cla_example.syscfg" -o "syscfg" -s "D:/ti/c2000/C2000Ware_5_05_00_00/.metadata/sdk.json" -b "/boards/LAUNCHXL_F28379D" --compiler ccs
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/board.c: build-1806428945 ../cla_example.syscfg
syscfg/board.h: build-1806428945
syscfg/board.cmd.genlibs: build-1806428945
syscfg/board.opt: build-1806428945
syscfg/board.json: build-1806428945
syscfg/pinmux.csv: build-1806428945
syscfg/device_cmd.cmd: build-1806428945
syscfg/device_cmd.c: build-1806428945
syscfg/device_cmd.h: build-1806428945
syscfg/device_cmd.opt: build-1806428945
syscfg/device_cmd.cmd.genlibs: build-1806428945
syscfg/c2000ware_libraries.cmd.genlibs: build-1806428945
syscfg/c2000ware_libraries.opt: build-1806428945
syscfg/c2000ware_libraries.c: build-1806428945
syscfg/c2000ware_libraries.h: build-1806428945
syscfg/clocktree.h: build-1806428945
syscfg: build-1806428945

syscfg/%.obj: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="C:/Users/ramon/Documents/git/codigos/cla_example" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/device" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/device/include" --include_path="D:/ti/c2000/C2000Ware_5_05_00_00/driverlib/f2837xd/driverlib" --include_path="D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/src" --advice:performance=all --define=CPU1 --define=DEBUG --define=_LAUNCHXL_F28379D --define=_DUAL_CORE_ --define=F2837XD_DEVICE --c99 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --cla_background_task=off --cla_signed_compare_workaround=on --preproc_with_compile --preproc_dependency="syscfg/$(basename $(<F)).d_raw" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/CPU1_RAM/syscfg" --obj_directory="syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="C:/Users/ramon/Documents/git/codigos/cla_example" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/device" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/device/include" --include_path="D:/ti/c2000/C2000Ware_5_05_00_00/driverlib/f2837xd/driverlib" --include_path="D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/src" --advice:performance=all --define=CPU1 --define=DEBUG --define=_LAUNCHXL_F28379D --define=_DUAL_CORE_ --define=F2837XD_DEVICE --c99 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --cla_background_task=off --cla_signed_compare_workaround=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.cla $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 -Ooff --include_path="C:/Users/ramon/Documents/git/codigos/cla_example" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/device" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/device/include" --include_path="D:/ti/c2000/C2000Ware_5_05_00_00/driverlib/f2837xd/driverlib" --include_path="D:/ti/ccs1281/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/src" --advice:performance=all --define=CPU1 --define=DEBUG --define=_LAUNCHXL_F28379D --define=_DUAL_CORE_ --define=F2837XD_DEVICE --c99 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=eabi --cla_background_task=off --cla_signed_compare_workaround=on --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" --include_path="C:/Users/ramon/Documents/git/codigos/cla_example/CPU1_RAM/syscfg" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


