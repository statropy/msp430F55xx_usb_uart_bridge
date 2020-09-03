OBJECTS=main.o hal.o ringbuf.o \
driverlib/MSP430F5xx_6xx/dma.o \
driverlib/MSP430F5xx_6xx/gpio.o \
driverlib/MSP430F5xx_6xx/pmm.o \
driverlib/MSP430F5xx_6xx/sfr.o \
driverlib/MSP430F5xx_6xx/tlv.o \
driverlib/MSP430F5xx_6xx/ucs.o \
driverlib/MSP430F5xx_6xx/usci_a_uart.o \
USB_API/USB_CDC_API/UsbCdc.o \
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
	RM= rm -rf
	MKDIR = mkdir -p
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

CFLAGS = $(INCLUDES) -mmcu=$(DEVICE) -Os -Wall -g0 -mlarge -mcode-region=either -mdata-region=lower -mhwmult=f5series -gdwarf-3 -gstrict-dwarf
LFLAGS = -L $(SUPPORT_FILE_DIRECTORY) -Wl,-Map,$(MAP),--gc-sections -T"$(SUPPORT_FILE_DIRECTORY)/msp430f5503.ld" -T"./USB_API/msp430USB.cmd"
CFLAGS_LP = $(INCLUDES) -mmcu=$(DEVICE_LP) -Os -Wall -g0 -mlarge -mcode-region=either -mdata-region=lower -mhwmult=f5series -gdwarf-3 -gstrict-dwarf -DBRIDGE_UART0 -DLAUNCHPAD
LFLAGS_LP = -L $(SUPPORT_FILE_DIRECTORY) -Wl,-Map,$(MAP_LP),--gc-sections -T"$(SUPPORT_FILE_DIRECTORY)/msp430f5529.ld" -T"./USB_API/msp430USB.cmd"

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
	@echo "\e[1;34mFinished clean\e[0m"
	
debug: $(OUTHEX)
	@echo "\e[1;34mStarting GDB\e[0m"
	@echo $(GDB)
	$(GDB) $(OUTFILE)

debug_launchpad: $(OUTHEX_LP)
	@echo "\e[1;34mStarting GDB\e[0m"
	@echo $(GDB)
	$(GDB) $(OUTFILE_LP)

program: $(OUTHEX)
	$(PYTHON2) -m msp430.bsl5.hid_1 -e -P $(OUTHEX)

program_launchpad: $(OUTHEX_LP)
	$(PYTHON2) -m msp430.bsl5.hid_1 -e -P $(OUTHEX_LP)

$(OUTDIR)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $? -o $@

$(OUTDIR_LP)/%.o : %.c
	@mkdir -p $(@D)
	$(CC) $(CFLAGS_LP) -c $? -o $@

%.hex: %.out
	$(OBJCOPY) -O ihex $< $@
	@echo "\e[1;34m$@ completed\e[0m"

$(OUTFILE): ${OBJS}
	@echo "\e[1;34mLinking $@\e[0m"
	$(CC) $(CFLAGS) $(LFLAGS) $? -o $(OUTFILE)

$(OUTFILE_LP): ${OBJS_LP}
	@echo "\e[1;34mLinking $@\e[0m"
	$(CC) $(CFLAGS_LP) $(LFLAGS_LP) $? -o $(OUTFILE_LP)

.PHONY: all target launchpad clean debug debug_launchpad program program_launchpad
