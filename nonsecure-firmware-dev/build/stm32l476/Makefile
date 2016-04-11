#*******************************************************************************
#   Ledger Blue - Non secure firmware
#   (c) 2016 Ledger
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#*******************************************************************************

################
# Default rule #
################

all: default

# consider every intermediate target as final to avoid deleting intermediate files
.SECONDARY:

# disable builtin rules that overload the build process (and the debug log !!)
.SUFFIXES:
MAKEFLAGS += -r

SHELL =       /bin/bash
#.ONESHELL:

DATE        := $(shell date "+%d %B %G")

#VERSION     := $(shell date "+%Y%m%d")
VERSION     := 1

$(info Building version: $(VERSION))


############
# Platform #
############
ifeq ($(PROG),)
PROG     := stm32l476_seproxyhal
endif

CONFIG_PRODUCTIONS := bin/$(PROG)


SOURCE_PATH   := src/ ../../bagl/src
SOURCE_FILES  := $(foreach path, $(SOURCE_PATH),$(shell find $(path) | grep "\.c$$") )
INCLUDES_PATH := ../../bagl/include include/ include/stm32l4_hal $(dir $(foreach path, $(SOURCE_PATH),$(shell find $(path) | grep "\.h$$") ))

DEFINES += STM32L476xx USE_STM32L4XX_NUCLEO USE_HAL_DRIVER gcc __IO=volatile

DEFINES += MSIRANGE=5 #6 #ok tested ble make discoverable after disconnect
#DEFINES += MSIRANGE=6 #6 #ok tested ble make discoverable after disconnect
#DEFINES += FREQUENCY_HZ='(0x10000<<MSIRANGE)'
DEFINES += FREQUENCY_HZ=64000000
#DEFINES += assert_param\(x\)= 

# NOTE: 
DEFINES += HAVE_PT035HV 
DEFINES += HAVE_BAGL BAGL_WIDTH=320 BAGL_HEIGHT=480 #BAGL_COMPONENT_MAXCOUNT=32

# for bluenrg interface weirdness debug
DEFINES += HAVE_SPI_DEBUG
# have bluenrg command timeout 
DEFINES += BLE_CMD_TIMEOUT
# use the SPBTLE-RF with the MS stack
DEFINES += BLUENRG_MS

DEFINES += HAVE_SE
DEFINES += HAVE_BLE

DEFINES += BACKLIGHT_AUTODIM_MS=\(1000*20\) 

# blue final
DEFINES += WATCHDOG_POWEROFF_MS=\(10*1000*60\) BACKLIGHT_AUTOOFF_MS=\(1000*30\) HAVE_PT035HV_ROTATION_180

#DEFINES += PRINTF\(...\)='screen_printf(__VA_ARGS__)'
DEFINES += PRINTF\(...\)

DEFINES += VERSION=\"$(VERSION)\"

##############
# Compiler #
##############
CCPATH   := $(BOLOS_ENV)/gcc-arm-none-eabi-4_7-2013q2/bin/
TARGET   := arm-none-eabi-
CC       := $(CCPATH)/$(TARGET)gcc
CFLAGS   := -ggdb2 -Werror=int-to-pointer-cast -Wall -Wextra #-save-temps
CFLAGS   += -O3 -Os
#CFLAGS   += -O0
CFLAGS   += -mcpu=cortex-m4 -fno-common -mtune=cortex-m4 -mthumb -nostartfiles -mlittle-endian -finline-limit-0 -fdata-sections -ffunction-sections -funsigned-char -funsigned-bitfields -fshort-enums --std=gnu99
# NOT SUPPORTED BY STM3L152 CFLAGS   += -fpack-struct
#-pg --coverage
LD       := $(CCPATH)/$(TARGET)gcc
LDFLAGS  :=  -ggdb2 -Wall 
LDFLAGS  += -O3 -Os
#LDFLAGS  += -O0 
LDFLAGS   += -mcpu=cortex-m4 -mthumb -fno-common -ffunction-sections -fdata-sections -fwhole-program -nostartfiles -Wl,--gc-sections 
LDFLAGS  += -Tscript.ld -Wl,-Map,debug/$(PROG).map
#LDFLAGS  += -nodefaultlibs
#LDFLAGS  += -nostdlib -nostdinc
LDLIBS   += -lm
# -mno-unaligned-access 
#-pg --coverage

#LDFLAGS += --specs=rdimon.specs 
#LDLIBS  += -lc -lrdimon 

### computed variables
VPATH := $(dir $(SOURCE_FILES))
OBJECT_FILES := $(sort $(addprefix obj/, $(addsuffix .o, $(basename $(notdir $(SOURCE_FILES))))))
DEPEND_FILES := $(sort $(addprefix dep/, $(addsuffix .d, $(basename $(notdir $(SOURCE_FILES))))))

-include $(DEPEND_FILES)

clean:
	rm -fr obj bin debug dep script.ld

prepare:
	@mkdir -p bin obj debug dep

.SECONDEXPANSION:

# default is not to display make commands
log = $(if $(strip $(VERBOSE)),$1,@$1)

default: prepare bin/$(PROG)

bin/$(PROG): $(OBJECT_FILES) script.ld.template Makefile
	@echo "[LINK] 	$@"
	$(call log,python preprocess.py $(addprefix -D,$(DEFINES)) -d -s script.ld.template > script.ld)
	$(call log,$(call link_cmdline,$(OBJECT_FILES) $(LDLIBS),$@))
	$(CCPATH)/$(TARGET)objcopy -O ihex -S bin/$(PROG) bin/$(PROG).hex
	$(CCPATH)/$(TARGET)objdump -d -S bin/$(PROG) > debug/$(PROG).asm
	$(call log,[ ! -f bin/stm32l476_seproxyhal.hex ] || srec_cat -multiple -Output -Intel bin/stm32l476_seproxyhal.hex -Intel -crop 0x08000000 0x20000000 -random-fill -minimum-addr bin/stm32l476_seproxyhal.hex -Intel 0x0803FFB0  > bin/$(PROG)-strip.hex)
	$(call log,[ ! -f bin/stm32l476_seproxyhal.hex ] || python dumpLoadMCU.py --targetId=0x04000001 --fileName=`pwd`/bin/$(PROG)-strip.hex --bootAddr=`cat debug/$(PROG).map | grep "          Reset_Handler$$" | grep 0x | tr -s ' '| cut -f2 -d' '` > bin/$(PROG).patch_apdu)
	$(call log,)

dep/%.d: %.c Makefile
	@echo "[DEP]    $@"
	@mkdir -p dep
	$(call log,$(call dep_cmdline,$(INCLUDES_PATH), $(DEFINES),$<,$@))

obj/%.o: %.c dep/%.d
	@echo "[CC]	$@"
	$(call log,$(call cc_cmdline,$(INCLUDES_PATH), $(DEFINES),$<,$@))


### BEGIN GCC COMPILER RULES

# link_cmdline(objects,dest)		Macro that is used to format arguments for the linker
link_cmdline = $(LD) $(LDFLAGS) -o $(2) $(1)

# dep_cmdline(include,defines,src($<),dest($@))	Macro that is used to format arguments for the dependency creator
dep_cmdline = $(CC) -M $(CFLAGS) $(addprefix -D,$(2)) $(addprefix -I,$(1)) $(3) | sed 's/\($*\)\.o[ :]*/obj\/\1.o: /g' | sed -e 's/[:\t ][^ ]\+\.c//g' > dep/$(basename $(notdir $(4))).d 2>/dev/null

# cc_cmdline(include,defines,src,dest)	Macro that is used to format arguments for the compiler
cc_cmdline = $(CC) -c $(CFLAGS) $(addprefix -D,$(2)) $(addprefix -I,$(1)) -o $(4) $(3)

### END GCC COMPILER RULES
