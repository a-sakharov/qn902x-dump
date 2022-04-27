.PHONY: all clean disasm

all: loader.bin

launcher.o: launcher.S
	arm-none-eabi-gcc -c -O0 -mcpu=cortex-m0 -mthumb -Wall launcher.S -o launcher.o

main.o: main.c
	arm-none-eabi-gcc -mlittle-endian -mthumb -mcpu=cortex-m0 -nostdlib -O0 -Wall -Icmsis -c main.c -o main.o

loader.elf: main.o launcher.o link.ld
	arm-none-eabi-gcc launcher.o main.o -mcpu=cortex-m0 -mthumb -Wall --specs=nosys.specs -nostdlib -lgcc -T./link.ld -o loader.elf

loader.bin: loader.elf
	arm-none-eabi-objcopy -O binary loader.elf loader.bin

disasm: loader.bin
	arm-none-eabi-objdump loader.bin -D -b binary -m arm -Mforce-thumb
	arm-none-eabi-objdump -d loader.elf -Mforce-thumb -l

clean:
	rm -f launcher.o main.o loader.elf loader.bin
