###############################################################################
## @file        makefile
 # @brief       Generic Makefile Template for C/C++ Projects
 #              The Linux build makefile
 # @version     V1.0-2022/01/30
 # @Author:     caizhijie
 # @copyright   Copyright (c) 2022-2022  Guangzhou Asensing Techology Co.,Ltd.
###############################################################################

###############################################################################
#   Compile GCC Mode
###############################################################################
CC_PATH :=
CROSS := arm-none-eabi-gcc
CC_SUFIX :=
ASM :=
LINK :=
OPTI_LEVEL :=0
ifeq ($(CC_PATH),)
CC := $(subst gcc,,$(CROSS))gcc
CXX := $(subst gcc,,$(CROSS))g++
AR := $(subst gcc,,$(CROSS))ar$(CC_SUFIX)
STRIP := $(subst gcc,,$(CROSS))strip$(CC_SUFIX)
ELFTOOL :=
else
CC := "$(CC_PATH)/$(subst gcc,,$(CROSS))gcc"
CXX := "$(CC_PATH)/$(subst gcc,,$(CROSS))g++"
AR := "$(CC_PATH)/$(subst gcc,,$(CROSS))ar$(CC_SUFIX)"
STRIP := "$(CC_PATH)/$(subst gcc,,$(CROSS))strip$(CC_SUFIX)"
ELFTOOL :=
endif

###############################################################################
#   SYSTEM Command
###############################################################################
RM := rm
CP := cp
MK_MKDIR=$(if ${1}, mkdir -p $(1))

# makefile path
mkfile_path = $(abspath $(lastword $(MAKEFILE_LIST)))
GIT_TAG = $(shell git for-each-ref  --sort=-taggerdate --format='%(tag)' --count=1)
GIT_BRANCH = $(shell git rev-parse --abbrev-ref HEAD)
GIT_COMMIT_ID = $(shell git rev-parse --short HEAD)

# VERSION_FLAG := -DAG_VERSION=\"$(LIB_VERSION)\" -DGIT_VERSION=\"$(GIT_VERSION)\"

###############################################################################
#   compile options
###############################################################################
QX_SDK_ENABLE := 0
MULTI_FREQ := 1
GPERF_ENABLE := 0
TASK_STACK := 1
PPP_RTK_ENABLE := 0
VDR_ENABLE := 0
ORT_ENABLE := 0
DEBUG_MODE := 0
TC_ENABLE := 0
DELOS_PRINTF :=1
VDR_STATIC_ENABLE :=0

###############################################################################
#   version information
###############################################################################
LOC_CORE ?= loc_core

ADAPTER_LIB ?= ssr2los_adapter
SSR_LIB ?= qxinertial
GPERF_LIB ?= profiler
ifeq ($(PPP_RTK_ENABLE), 1)
LIB_NAME_RTK =_ppp
else
LIB_NAME_RTK =_rtk
endif
ifeq ($(ORT_ENABLE), 1)
LIB_NAME_ORT =_ort
else
LIB_NAME_ORT =
endif
ifeq ($(VDR_ENABLE),1)
LIB_NAME_VDR =_vdr
else
LIB_NAME_VDR =
endif
TARGET_CORE ?= lib$(LOC_CORE)$(LIB_NAME_RTK)$(LIB_NAME_ORT)$(LIB_NAME_VDR).a
#SRC_EXTS := .c .C .cc .cpp .CPP .c++ .cxx .cp
C_EXTS := .c
CXX_EXTS := .cpp

###############################################################################
#   Include directory and source files of vlt core
###############################################################################
# Project based directory
PROJ_DIR_LOC := ../../
PROJ_DIR_LOC_CORE := ../../core/
PROJ_DIR_LOC_APP := ../../app/loc_app/
PROJ_DIR_LOC_PB := ../../app/playback
PROJ_LIB_LOC_CORE := ../../3rdparty/qianxun/lib/$(subst gcc,,$(CROSS))gcc
PROJ_LIB_LOC_GPERF := ../../3rdparty/gperftools/linux-lib
PROJ_DIR_LOC_3RD := ../../3rdparty/

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

HEADER_DIR_LOC_3RD := -I$(PROJ_DIR_LOC_3RD) \
		-I$(PROJ_DIR_LOC_3RD)serial \
		-I$(PROJ_DIR_LOC_3RD)serial/linux \
		-I$(PROJ_DIR_LOC_3RD)qianxun/include \
		-I$(PROJ_DIR_LOC_3RD)delos/

HEADER_DIR_LOC_APP := -I$(PROJ_DIR_LOC_APP) \
		-I$(PROJ_DIR_LOC_APP)driver \
		-I../../app/loc_parse

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
		$(PROJ_DIR_LOC_CORE)sd \
		$(PROJ_DIR_LOC_CORE)sm

C_SOURCES_LOC_CORE = $(foreach d,$(SRC_DIR_LOC_CORE),$(wildcard $(addprefix $(d)/*,$(C_EXTS))))

ifeq ($(PPP_RTK_ENABLE),1)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ppp/*.c)
else
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ppp/stub/ppp_task_stub.c)
endif

ifeq ($(VDR_ENABLE),1)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)vdr/*.c)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ekf/*.c)
else
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)vdr/stub/vdr_task_stub.c)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ekf/stub/ekf_task_stub.c)
endif

ifeq ($(ORT_ENABLE),1)
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ort/*.c)
else
C_SOURCES_LOC_CORE += $(wildcard $(PROJ_DIR_LOC_CORE)ort/stub/ort_task_stub.c)
endif

C_OBJECTS_LOC_CORE = $(C_SOURCES_LOC_CORE:%.c=%.o)

FLAGS += -mcpu=cortex-a7 -mfloat-abi=hard -mfpu=vfpv4-d16 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin -Wextra -Wno-unused -Wno-unused-parameter -Wno-missing-field-initializers -DEMBEDDED
FLAGS += -Wall -Os

ifeq ($(DEBUG_MODE),1)
FLAGS += -g3
else
FLAGS += -g
endif

FLAGS += -DGIT_BRANCH=\"$(GIT_BRANCH)\" -DGIT_COMMIT_ID=\"$(GIT_COMMIT_ID)\" -DGIT_TAG=\"$(GIT_TAG)\"
FLAGS += $(HEADER_DIR_LOC_CORE) $(HEADER_DIR_LOC_APP) $(HEADER_DIR_LOC_3RD) $(HEADER_DIR_LOC_PB)

ifeq ($(MULTI_FREQ),1)
FLAGS += -DENABLE_MULTI_FREQ
endif

ifeq ($(CHIP_TYPE),1)
FLAGS += -DST_9100
endif

ifeq ($(DELOS_PRINTF),1)
FLAGS += -DENABLE_DELOS_PRINTF
endif

ifeq ($(TASK_STACK),1)
FLAGS += -DENABLE_TASK_STACK
endif

ifeq ($(TC_ENABLE),1)
FLAGS += -DTC_PRDR
endif

ifeq ($(VDR_STATIC_ENABLE),1)
FLAGS += -DFORCE_INIT
endif

ifeq ($(QX_SDK_ENABLE),1)
LIBS  += -l$(SSR_LIB) -l$(ADAPTER_LIB)
FLAGS += -DFEATURE_USE_QXWZ_SSR
CORE_LIBFLAGS = -L $(PROJ_LIB_LOC_CORE) -l$(ADAPTER_LIB)
APP_LIBFLAGS = -L $(PROJ_LIB_LOC_CORE) $(LIBS)
endif

CFLAGS += -std=gnu99 $(FLAGS)
CXXFLAGS += -std=c++11 $(FLAGS)
CCFLAGS += -std=gnu99 $(FLAGS)

loc_core: $(TARGET_CORE)
	echo 'target $(TARGET_CORE) is done'

$(TARGET_CORE): $(C_OBJECTS_LOC_CORE)
	$(AR) -r $@ $^ $(CORE_LIBFLAGS)

$(C_OBJECTS_LOC_CORE):%.o:%.c
	$(CC) $(CCFLAGS) -c $^ -o $@

.PHONY: clean all 
clean:
	$(RM) -rf $(TARGET_CORE)
	$(RM) -rf $(C_OBJECTS_LOC) $(CXX_OBJECTS_LOC)
	$(RM) -rf $(C_OBJECTS_LOC_CORE) $(TARGET_CORE)

all:
	make clean
	make loc_core
