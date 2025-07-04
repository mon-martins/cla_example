################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../2837xD_RAM_CLA_lnk_cpu1.cmd 

SYSCFG_SRCS += \
../lab_f2837xd_launchpad.syscfg 

CLA_SRCS += \
../lab_cla_tasks.cla 

LIB_SRCS += \
C:/ti/c2000/C2000Ware_5_04_00_00/driverlib/f2837xd/driverlib/ccs/Debug/driverlib.lib 

C_SRCS += \
./syscfg/board.c \
./syscfg/c2000ware_libraries.c \
../lab_main.c 

GEN_FILES += \
./syscfg/board.c \
./syscfg/board.opt \
./syscfg/c2000ware_libraries.opt \
./syscfg/c2000ware_libraries.c 

CLA_DEPS += \
./lab_cla_tasks.d 

GEN_MISC_DIRS += \
./syscfg 

C_DEPS += \
./syscfg/board.d \
./syscfg/c2000ware_libraries.d \
./lab_main.d 

GEN_OPTS += \
./syscfg/board.opt \
./syscfg/c2000ware_libraries.opt 

OBJS += \
./lab_cla_tasks.obj \
./syscfg/board.obj \
./syscfg/c2000ware_libraries.obj \
./lab_main.obj 

GEN_MISC_FILES += \
./syscfg/board.h \
./syscfg/board.cmd.genlibs \
./syscfg/board.json \
./syscfg/pinmux.csv \
./syscfg/c2000ware_libraries.cmd.genlibs \
./syscfg/c2000ware_libraries.h \
./syscfg/clocktree.h 

GEN_MISC_DIRS__QUOTED += \
"syscfg" 

OBJS__QUOTED += \
"lab_cla_tasks.obj" \
"syscfg\board.obj" \
"syscfg\c2000ware_libraries.obj" \
"lab_main.obj" 

GEN_MISC_FILES__QUOTED += \
"syscfg\board.h" \
"syscfg\board.cmd.genlibs" \
"syscfg\board.json" \
"syscfg\pinmux.csv" \
"syscfg\c2000ware_libraries.cmd.genlibs" \
"syscfg\c2000ware_libraries.h" \
"syscfg\clocktree.h" 

C_DEPS__QUOTED += \
"syscfg\board.d" \
"syscfg\c2000ware_libraries.d" \
"lab_main.d" 

GEN_FILES__QUOTED += \
"syscfg\board.c" \
"syscfg\board.opt" \
"syscfg\c2000ware_libraries.opt" \
"syscfg\c2000ware_libraries.c" 

CLA_DEPS__QUOTED += \
"lab_cla_tasks.d" 

SYSCFG_SRCS__QUOTED += \
"../lab_f2837xd_launchpad.syscfg" 

C_SRCS__QUOTED += \
"./syscfg/board.c" \
"./syscfg/c2000ware_libraries.c" \
"../lab_main.c" 


