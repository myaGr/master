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
CC_PATH ?=
CROSS ?=
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
CC := $(CC_PATH)/$(subst gcc,,$(CROSS))gcc
CXX := $(CC_PATH)/$(subst gcc,,$(CROSS))g++
AR := $(CC_PATH)/$(subst gcc,,$(CROSS))ar$(CC_SUFIX)
STRIP := $(CC_PATH)/$(subst gcc,,$(CROSS))strip$(CC_SUFIX)
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
#   version information
###############################################################################
TARGET_SUFFIX ?=
LOC_CORE ?= loc_core$(TARGET_SUFFIX)
LOC_APP  ?= loc_app
LOC_PB  ?= loc_pb

ADAPTER_LIB ?= ssr2los_adapter
SSR_LIB ?= qxinertial
GPERF_LIB ?= profiler
TARGET_CORE ?= lib$(LOC_CORE).so
TARGET_APP  ?= $(LOC_APP).bin
TARGET_PB  ?= $(LOC_PB).bin

#SRC_EXTS := .c .C .cc .cpp .CPP .c++ .cxx .cp
C_EXTS := .c
CXX_EXTS := .cpp

###############################################################################
#   compile options
###############################################################################
QX_SDK_ENABLE := 0
MULTI_FREQ := 0
GPERF_ENABLE := 0
DEBUG_ENABLE := 0
TC_ENABLE := 0
VDR_STATIC_ENABLE := 0
LARGE_LOG_CACHE := 1
#RESET_FILTER := 0
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
		-I$(PROJ_DIR_LOC_3RD)qianxun/include

HEADER_DIR_LOC_APP := -I$(PROJ_DIR_LOC_APP) \
		-I$(PROJ_DIR_LOC_APP)driver \
		-I../../app/loc_parse \
		-I../../utils

HEADER_DIR_LOC_PB := -I$(PROJ_DIR_LOC_PB)
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
		$(PROJ_DIR_LOC_CORE)os/linux \
		$(PROJ_DIR_LOC_CORE)ppp \
		$(PROJ_DIR_LOC_CORE)sd \
		$(PROJ_DIR_LOC_CORE)sm \
		$(PROJ_DIR_LOC_CORE)vdr \
 		$(PROJ_DIR_LOC_CORE)ort \
		$(PROJ_DIR_LOC_CORE)ekf

CXX_SOURCES_APP = $(wildcard $(PROJ_DIR_LOC)app/loc_app/*.cpp) \
		$(wildcard $(PROJ_DIR_LOC)app/loc_app/driver/ntrip_client.cpp) \
		$(wildcard $(PROJ_DIR_LOC)app/loc_app/driver/uart_driver.cpp) \
		$(wildcard $(PROJ_DIR_LOC)app/loc_app/driver/qx_ssr_client.cpp) \
		$(wildcard $(PROJ_DIR_LOC)utils/*.cpp) \
		$(wildcard $(PROJ_DIR_LOC)3rdparty/*.cpp) \
		$(wildcard $(PROJ_DIR_LOC)3rdparty/serial/*.cpp) \
		$(wildcard $(PROJ_DIR_LOC)3rdparty/serial/linux/*.cpp)

C_SOURCES_APP:= $(wildcard $(PROJ_DIR_LOC)app/loc_app/*.c) \
		$(wildcard $(PROJ_DIR_LOC)utils/*.c) \
		$(wildcard $(PROJ_DIR_LOC)3rdparty/*.c)

CXX_SOURCES_PB := $(wildcard $(PROJ_DIR_LOC)app/playback/*.cpp) \
		$(wildcard $(PROJ_DIR_LOC)3rdparty/*.cpp) \
		$(wildcard $(PROJ_DIR_LOC)utils/*.cpp) \
		$(PROJ_DIR_LOC)app/loc_parse/ipc_parse.cpp \
		$(PROJ_DIR_LOC)app/loc_parse/ipc_extend_log.cpp

C_SOURCES_PB := $(wildcard $(PROJ_DIR_LOC)app/playback/*.c) \
		$(wildcard $(PROJ_DIR_LOC)3rdparty/*.c) \
		$(wildcard $(PROJ_DIR_LOC)utils/*.c)

C_SOURCES_LOC_CORE := $(foreach d,$(SRC_DIR_LOC_CORE),$(wildcard $(addprefix $(d)/*,$(C_EXTS))))
C_OBJECTS_LOC_CORE = $(C_SOURCES_LOC_CORE:%.c=%.o)

C_OBJECTS_APP = $(C_SOURCES_APP:%.c=%.o)
CXX_OBJECTS_APP = $(CXX_SOURCES_APP:%.cpp=%.o)

C_OBJECTS_PB = $(C_SOURCES_PB:%.c=%.o)
CXX_OBJECTS_PB = $(CXX_SOURCES_PB:%.cpp=%.o)

FLAGS += -Wall -O$(OPTI_LEVEL) -Wall -fmessage-length=0 -ffunction-sections -fdata-sections -mcpu=cortex-a53
ifeq ($(DEBUG_ENABLE),1)
FLAGS += -g3
endif
FLAGS += -DGNUC_MACRO -DOPT_TIME
FLAGS += -DGIT_BRANCH=\"$(GIT_BRANCH)\" -DGIT_COMMIT_ID=\"$(GIT_COMMIT_ID)\" -DGIT_TAG=\"$(GIT_TAG)\"
FLAGS += $(HEADER_DIR_LOC_CORE) $(HEADER_DIR_LOC_APP) $(HEADER_DIR_LOC_3RD) $(HEADER_DIR_LOC_PB)

ifeq ($(CHIP_TYPE),1)
FLAGS += -DST_9100
endif

ifeq ($(MULTI_FREQ),1)
FLAGS += -DENABLE_MULTI_FREQ
endif

#ifeq ($(RESET_FILTER),1)
#FLAGS += -DRESET_FILTER
#endif

ifeq ($(TC_ENABLE),1)
FLAGS += -DTC_PRDR
endif

ifeq ($(VDR_STATIC_ENABLE),1)
FLAGS += -DFORCE_INIT
endif

ifeq ($(LARGE_LOG_CACHE),1)
FLAGS += -DENABLE_LARGE_LOG_CACHE
endif

ifeq ($(QX_SDK_ENABLE),1)
LIBS  += -l$(SSR_LIB) -l$(ADAPTER_LIB)
FLAGS += -DFEATURE_USE_QXWZ_SSR
CORE_LIBFLAGS = -L $(PROJ_LIB_LOC_CORE) -l$(ADAPTER_LIB)
APP_LIBFLAGS = -L $(PROJ_LIB_LOC_CORE) $(LIBS)
endif
ifeq ($(GPERF_ENABLE),1)
FLAGS += -DFEATURE_PROFILER
CORE_LIBFLAGS += -L $(PROJ_LIB_LOC_GPERF) -l$(GPERF_LIB)
endif

CFLAGS += -std=gnu99 $(FLAGS)
CXXFLAGS += -std=c++11 -pthread $(FLAGS) 
CCFLAGS += -std=gnu99 -pthread $(FLAGS)

loc_core: $(TARGET_CORE)
	echo 'target loc_core.so is done'
loc_pb: $(TARGET_PB)
	echo 'target loc_pb.bin is done'
loc_app: $(TARGET_APP)
	echo 'target loc_app.bin is done'

$(TARGET_APP): $(C_OBJECTS_LOC_CORE) $(CXX_OBJECTS_APP) $(C_OBJECTS_APP)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(APP_LIBFLAGS)

$(TARGET_PB): $(C_OBJECTS_LOC_CORE) $(CXX_OBJECTS_PB) $(C_OBJECTS_PB)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(CORE_LIBFLAGS)

$(TARGET_CORE): $(C_OBJECTS_LOC_CORE)
	$(CC) -fPIC -shared $(CXXFLAGS) -o $@ $^ $(CORE_LIBFLAGS)

$(C_OBJECTS_LOC_CORE):%.o:%.c
	$(CC) -fPIC -shared $(CCFLAGS) -c $^ -o $@

$(CXX_OBJECTS_APP):%.o:%.cpp
	$(CXX) -fPIC -shared $(CXXFLAGS) -c $^ -o $@

$(C_OBJECTS_APP):%.o:%.c
	$(CC) -fPIC -shared $(CCFLAGS) -c $^ -o $@

$(CXX_OBJECTS_PB):%.o:%.cpp
	$(CXX) -fPIC -shared $(CXXFLAGS) -c $^ -o $@

$(C_OBJECTS_PB):%.o:%.c
	$(CC) -fPIC -shared $(CCFLAGS) -c $^ -o $@

.PHONY: clean all 
clean:
	$(RM) -rf $(C_OBJECTS_LOC_CORE)
	$(RM) -rf $(C_OBJECTS_PB) $(CXX_OBJECTS_PB)
	$(RM) -rf $(C_OBJECTS_APP) $(CXX_OBJECTS_APP)
	$(RM) -rf $(TARGET_CORE) $(TARGET_PB) $(TARGET_APP)

all:
	make clean
	make loc_core loc_app loc_pb
