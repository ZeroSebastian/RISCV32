file=$1
riscv64-unknown-elf-gcc -g -march=rv32i -mabi=ilp32 -ffreestanding \
	-O0 -Wl,--gc-sections -nostartfiles -nostdlib -nodefaultlibs \
	-Wl,-T,default.ld crt0.s $file.c -o $file.out
	
	# Linken
riscv64-unknown-elf-ld -melf32lriscv -T default.ld \
$file.out -o $file

# Speicherabbild .bin
riscv64-unknown-elf-objcopy $file -O binary $file.bin
# hex file with big endianness instead of little
riscv64-unknown-elf-objcopy --reverse-bytes=4 -I binary $file.bin -O ihex $file.hex

xxd $file.bin
cat $file.hex
riscv64-unknown-elf-objdump -D $file.out
rm $file.out
