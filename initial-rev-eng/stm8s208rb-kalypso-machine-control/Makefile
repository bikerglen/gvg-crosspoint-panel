download: blink.ihx
	stm8flash -c stlinkv21 -p stm8s208rb -w blink.ihx

blink.ihx: blink.c
	sdcc -mstm8 --std-c99 blink.c

