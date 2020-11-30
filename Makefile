OBJECTS=main.o hal.o ringbuf.o \
driverlib/MSP430F5xx_6xx/dma.o \
driverlib/MSP430F5xx_6xx/gpio.o \
driverlib/MSP430F5xx_6xx/pmm.o \
driverlib/MSP430F5xx_6xx/sfr.o \
driverlib/MSP430F5xx_6xx/tlv.o \
driverlib/MSP430F5xx_6xx/ucs.o \
driverlib/MSP430F5xx_6xx/usci_a_uart.o \
driverlib/MSP430F5xx_6xx/crc.o \
USB_API/USB_CDC_API/UsbCdc.o \
USB_API/USB_WPAN_API/UsbWpan.o \
USB_API/USB_Common/usb.o \
USB_API/USB_Common/usbdma.o \
USB_app/usbConstructs.o \
USB_app/usbEventHandling.o \
USB_config/descriptors.o \
USB_config/UsbIsr.o

PROJ=usb_uart_bridge
MAP=$(PROJ).map
OUTFILE=$(PROJ).out
OUTHEX=$(PROJ).hex

LP=$(PROJ)_lp
MAP_LP=$(LP).map
OUTFILE_LP=$(LP).out
OUTHEX_LP=$(LP).hex

MAKEFILE=Makefile
OUTDIR=BUILD
OUTDIR_LP=BUILD_LP

ifeq ($(OS),Windows_NT)
	ifeq ($(shell uname -o),Cygwin)
		RM= rm -rf
		MKDIR = mkdir -p
	else
		RM= del /q
		MKDIR = mkdir
	endif
else
	ECHO = echo -e
	SHELL = bash
	ifeq ($(shell uname -s), Darwin)
		SUDO = 
	else
		SUDO = sudo
	endif

	RM= rm -rf
	MKDIR = mkdir -p

	# this environment variable is rarely set by default on Linux machines
	ifeq (,$(PYTHON2))
		PYTHON2 = $(shell which python2)
	endif

	ifeq (,$(MSP430_TOOLCHAIN_PATH))
		ifneq (,$(shell which msp430-elf-gcc))
			# get MSP430_TOOLCHAIN_PATH from msp430-elf-gcc itself
			MSP430_TOOLCHAIN_PATH := \
				$(shell dirname \
					$(shell dirname \
						$(shell dirname \
							$(shell realpath \
								$(shell msp430-elf-gcc --print-file-name libc.a)\
							)\
						)\
					)\
				)
		endif
	endif
endif

ifeq (,$(MSP430_TOOLCHAIN_PATH))
$(error Please install the MSP430 toolchain and set the MSP430_TOOLCHAIN_PATH environment variable)
endif

GCC_DIR = $(MSP430_TOOLCHAIN_PATH)/bin
SUPPORT_FILE_DIRECTORY = $(MSP430_TOOLCHAIN_PATH)/include

INCLUDES = -I $(SUPPORT_FILE_DIRECTORY) \
		   -I ./driverlib/MSP430F5xx_6xx \
		   -I ./USB_config \
		   -I .

DEVICE  = MSP430F5503
DEVICE_LP = MSP430F5529
CC      = $(GCC_DIR)/msp430-elf-gcc
GDB     = $(GCC_DIR)/msp430-elf-gdb
OBJCOPY = $(GCC_DIR)/msp430-elf-objcopy
OBJSIZE = $(GCC_DIR)/msp430-elf-size

CFLAGS_COMMON = $(INCLUDES) -Os -Wall -g0 -mlarge -mcode-region=none -mhwmult=f5series -gdwarf-3 -gstrict-dwarf -std=c99 
LFLAGS_COMMON = -L $(SUPPORT_FILE_DIRECTORY) -Wl,--gc-sections -T"./USB_API/msp430USB.cmd"
CFLAGS = $(CFLAGS_COMMON) -mmcu=$(DEVICE)
LFLAGS = $(LFLAGS_COMMON) -Wl,-Map,$(MAP) -T"$(SUPPORT_FILE_DIRECTORY)/msp430f5503.ld" 
CFLAGS_LP = $(CFLAGS_COMMON) -mmcu=$(DEVICE_LP) -DBRIDGE_UART0 -DLAUNCHPAD
LFLAGS_LP = $(LFLAGS_COMMON) -Wl,-Map,$(MAP_LP) -T"$(SUPPORT_FILE_DIRECTORY)/msp430f5529.ld"

OBJS= $(patsubst %.o,$(OUTDIR)/%.o,$(OBJECTS))
OBJS_LP= $(patsubst %.o,$(OUTDIR_LP)/%.o,$(OBJECTS))

all: target launchpad

target: $(OUTHEX)

launchpad: $(OUTHEX_LP)

clean: 
	$(RM) $(OUTDIR)
	$(RM) $(OUTDIR_LP)
	$(RM) $(MAP)
	$(RM) $(MAP_LP)
	$(RM) $(OUTFILE)
	$(RM) $(OUTFILE_LP)
	$(RM) $(OUTHEX)
	$(RM) $(OUTHEX_LP)
	@$(ECHO) "\x1B[1;34mFinished clean\x1B[0m"

require_python2:
ifeq (,$(PYTHON2))
	$(error Please set the PYTHON2 environment variable to the path to the python2 executable)
endif
	
debug: $(OUTHEX)
	@$(ECHO) "\x1B[1;34mStarting GDB\x1B[0m"
	@$(ECHO) $(GDB)
	$(GDB) $(OUTFILE)

debug_launchpad: $(OUTHEX_LP)
	@$(ECHO) "\x1B[1;34mStarting GDB\x1B[0m"
	@$(ECHO) $(GDB)
	$(GDB) $(OUTFILE_LP)

program: $(OUTHEX) require_python2
	$(SUDO) $(PYTHON2) -m msp430.bsl5.hid_1 -e -P $(OUTHEX)

program_launchpad: $(OUTHEX_LP) require_python2
	$(SUDO) $(PYTHON2) -m msp430.bsl5.hid_1 -e -P $(OUTHEX_LP)

$(OUTDIR)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $? -o $@

$(OUTDIR_LP)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS_LP) -c $? -o $@

%.hex: %.out
	$(OBJCOPY) -O ihex $< $@
	@$(ECHO) "\x1B[1;34m$@ completed\x1B[0m"
	$(OBJSIZE) $<

$(OUTFILE): ${OBJS}
	@$(ECHO) "\x1B[1;34mLinking $@\x1B[0m"
	$(CC) $(CFLAGS) $(LFLAGS) $? -o $(OUTFILE)

$(OUTFILE_LP): ${OBJS_LP}
	@$(ECHO) "\x1B[1;34mLinking $@\x1B[0m"
	$(CC) $(CFLAGS_LP) $(LFLAGS_LP) $? -o $(OUTFILE_LP)

.PHONY: all target launchpad clean debug debug_launchpad program program_launchpad require_python2
