
# input file
Release\ISP_UART.hex -intel

-crop 0x100000 0x101000

# produce the output file
-Output
Release\LDROM_Bootloader.hex -intel

