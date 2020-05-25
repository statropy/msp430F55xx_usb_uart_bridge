
##echo_test##
- USB CDC used for `port`
- UART on board wired for loopback
- Checks that complete message is echoed back

##relay_test##
- USB CDC for port1
- UART with USB/Serial converter for port2
- Sends to port1, reads from port2, checks for complete message
- Sends to port2, reads from port1, checks for complete message
