alias km='cat /proc/kmsg '
alias iod3='ioctl /dev/dpram0 0x6fd3 '
alias iod0='ioctl /dev/dpram0 0x6fd0 '
alias ioc0='ioctl /dev/dpram0 0xf0c0 '
alias ioc1='ioctl /dev/dpram0 0xf0c1 '
alias dp='cat /proc/driver/dpram '
alias tt='cat /proc/tty/driver/ttySAC '
alias ra='ipctool 07 00 00 00 01 01 03 '
alias vg='ipctool -d 07 00 02 ff 0a 01 02 '
alias ro='radiooptions 5 '
alias pa='ipctool 09 00 35 00 05 01 03 82 00 '
alias ip='ipctool '
alias ipd='ipctool -d '
alias l='ls -l'
alias ci='cat /proc/interrupts'
alias dm='dmesg -c '
export LD_LIBRARY_PATH=/lib:/system/lib:/sd/bin
export TERMINFO=/system/lib/terminfo
export TERM=xterm


