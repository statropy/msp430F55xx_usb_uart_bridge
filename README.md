![CI](https://github.com/jadonk/msp430F55xx_usb_uart_bridge/workflows/CI/badge.svg)

# msp430F55xx_usb_uart_bridge
Bridge between UART and USB CDC serial port for MSP430F55xx

## Build configurations using [msp430-gcc](https://www.ti.com/tool/download/MSP430-GCC-OPENSOURCE)

Install [python-msp430-tools](https://github.com/statropy/ti_msp430_python_tools) to load program using BSL.

### BSL Mode

Place board into BSL mode before programming

* Unplug board from external power and USB
* Hold down BSL button
* Plug in USB cable connected to host computer
* Wait approximately 1 second and release BSL button

### Beagle Connect: MSP430F5503 target using UART1

The User button (under the battery JST connector) is the BSL button

#### Building

`MSP430_TOOLCHAIN_PATH=<msp430-gcc dir> make`

#### Programming

`MSP430_TOOLCHAIN_PATH=<msp430-gcc dir> PYTHON2=<python2 interpreter> make program`

### Launchpad MSP-EXP430F5529LP: MSP430F5529 target using UART0

#### Building

`MSP430_TOOLCHAIN_PATH=<msp430-gcc dir> make launchpad`

#### Programming

`MSP430_TOOLCHAIN_PATH=<msp430-gcc dir> PYTHON2=<python2 interpreter> make program_launchpad`

## Build configurations for Code Composer Studio 10.0.0.00010

* FF5529LP: MSP-EXP430F5529LP launchpad, using UART1 (Tx/Rx pins to debug MCU)
* FF529LP_UART0: MSP-EXP430F5529LP launchpad, using UART0 (P3.3 and P3.4)
* Debug: MSP430F5503 target using UART1
* Release: MSP430F5503 target using UART1
