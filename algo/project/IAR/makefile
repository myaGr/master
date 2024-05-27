###############################################################################
## @file        makefile
 # @brief       Generic Makefile Template for C/C++ Projects
 #              The Linux build makefile
 # @version     V1.0-2022/01/30
 # @Author:     caizhijie
 # @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
###############################################################################

###############################################################################
#   Compile IAR Mode
###############################################################################
CC_PATH :=
CROSS :=
CC_SUFIX := .exe
CC := "iccarm$(CC_SUFIX)"
CXX := 
ASM := "$(CC_PATH)iasmarm$(CC_SUFIX)"
LINK := "$(CC_PATH)ilinkarm$(CC_SUFIX)"
AR := "$(CC_PATH)iarchive$(CC_SUFIX)"
ELFTOOL := 

###############################################################################
#   SYSTEM Command
###############################################################################
RM := rm
CP := cp
MK_MKDIR=$(if ${1}, mkdir -p $(1))

# makefile path
mkfile_path = $(abspath $(lastword $(MAKEFILE_LIST)))
# VERSION_FLAG := -DAG_VERSION=\"$(LIB_VERSION)\" -DGIT_VERSION=\"$(GIT_VERSION)\"

###############################################################################
#   version information
###############################################################################
LOC_CORE ?= loc_core

TARGET_CORE ?= lib$(LOC_CORE).a

#SRC_EXTS := .c .C .cc .cpp .CPP .c++ .cxx .cp
C_EXTS := .c
CXX_EXTS := .cpp

###############################################################################
#   compile options
###############################################################################
QX_SDK_ENABLE := 1
SINGLE_MODE:= 1
VDR_ENABLE := 0
FAST_MEMORY_ENABLE := 1
LARGE_LOG_CACHE := 1
###############################################################################
#   Include directory and source files of vlt core
###############################################################################
# Project based directory
PROJ_DIR_LOC := ../../
PROJ_DIR_LOC_CORE := ../../core/

# Header files directory
HEADER_DIR_LOC_CORE := -I$(PROJ_DIR_LOC_CORE)api \
		-I$(PROJ_DIR_LOC_CORE)api/qxsi \
		-I$(PROJ_DIR_LOC_CORE)api/dysk \
		-I$(PROJ_DIR_LOC_CORE)cmn \
		-I$(PROJ_DIR_LOC_CORE)dcp \
		-I$(PROJ_DIR_LOC_CORE)hpp \
		-I$(PROJ_DIR_LOC_CORE)hpp/ver1.0 \
		-I$(PROJ_DIR_LOC_CORE)hpp/ver1.0/include \
		-I$(PROJ_DIR_LOC_CORE)midware \
		-I$(PROJ_DIR_LOC_CORE)midware/quicklz \
		-I$(PROJ_DIR_LOC_CORE)os \
		-I$(PROJ_DIR_LOC_CORE)ppp \
		-I$(PROJ_DIR_LOC_CORE)sd \
		-I$(PROJ_DIR_LOC_CORE)sm \
		-I$(PROJ_DIR_LOC_CORE)vdr \
		-I$(PROJ_DIR_LOC_CORE)ort \
		-I$(PROJ_DIR_LOC_CORE)ekf
#
#  Vlt Core source File
#
# The source file types (headers excluded).
SRC_DIR_LOC_CORE := $(PROJ_DIR_LOC_CORE)api \
		$(PROJ_DIR_LOC_CORE)cmn \
		$(PROJ_DIR_LOC_CORE)dcp \
		$(PROJ_DIR_LOC_CORE)hpp \
		$(PROJ_DIR_LOC_CORE)hpp/ver1.0 \
		$(PROJ_DIR_LOC_CORE)midware \
		$(PROJ_DIR_LOC_CORE)midware/quicklz \
		$(PROJ_DIR_LOC_CORE)ppp \
		$(PROJ_DIR_LOC_CORE)sd \
		$(PROJ_DIR_LOC_CORE)sm \
 		$(PROJ_DIR_LOC_CORE)ort

C_SOURCES_LOC_CORE := $(foreach d,$(SRC_DIR_LOC_CORE),$(wildcard $(addprefix $(d)/*,$(C_EXTS))))
C_OBJECTS_LOC_CORE = $(C_SOURCES_LOC_CORE:%.c=%.o)

ifeq ($(SINGLE_MODE),1)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)os/*.c)
endif

ifeq ($(VDR_ENABLE),1)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)vdr/*.c)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ekf/*.c)
else
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)vdr/stub/vdr_task_stub.c)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ekf/stub/ekf_task_stub.c)
endif

ifeq ($(CHIP_TYPE),1)
FLAGS += -DST_9100
endif

ifeq ($(QX_SDK_ENABLE),1)
FLAGS += -DFEATURE_USE_QXWZ_SSR
endif

ifeq ($(FAST_MEMORY_ENABLE),1)
FLAGS += -DFEATURE_USE_FAST_MEMORY
endif

ifeq ($(UNICORE),1)
FLAGS += -DUNICORE
endif

ifeq ($(LARGE_LOG_CACHE),1)
FLAGS += -DENABLE_LARGE_LOG_CACHE
endif

FLAGS += -DGIT_COMMIT_ID=\"$(COMMIT_ID)\" -DGIT_TAG=\"$(GIT_TAG)\"
FLAGS += $(HEADER_DIR_LOC_CORE)

CFLAGS += -D __STA8100__\
	-D__ICACHE_PRESENT=1 \
	-D__DCACHE_PRESENT=1 \
	-Dtviibh8m=1 \
	-DCYT4BF8CDE=1 \
	-DCY_USE_PSVP=0 \
	-D__TARGET_FPU_VFP \
	-DARM_MATH_CM7 \
	-D__FPU_USED=1 \
	-D__FPU_PRESENT=1 \
	-D_DLIB_FILE_DESCRIPTOR \
	--no_cse \
	--no_unroll \
	--no_inline \
	--no_code_motion \
	--no_tbaa \
	--no_clustering \
	--no_scheduling \
	--debug \
	--endian=little \
	--cpu=Cortex-M7 \
	-e --fpu=VFPv5_d16 \
	-Ohs \
	--use_c++_inline \
	-D__ICCARM__=1 \
	-DSYS_BASIC_TYPES

CFLAGS += $(FLAGS)

loc_core: $(TARGET_CORE)
	echo 'target loc_core.a is done'

$(TARGET_CORE): $(C_OBJECTS_LOC_CORE)
	$(AR) $(C_OBJECTS_LOC_CORE) -o $@

$(C_OBJECTS_LOC_CORE):%.o:%.c
	$(CC) $(CFLAGS) -c $< -o $@

.PHONY: clean all 
clean:
	$(RM) -rf $(TARGET_CORE)
	$(RM) -rf $(C_OBJECTS_LOC)
	$(RM) -rf $(C_OBJECTS_LOC_CORE) $(TARGET_CORE)

all:
	make clean
	make loc_core