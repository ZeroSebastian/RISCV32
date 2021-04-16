file=$1
riscv64-unknown-elf-gcc -march=rv32i -mabi=ilp32 -o $file.o -c $file.s
# Linken
riscv64-unknown-elf-ld -melf32lriscv -T minimal.ld \
$file.o -o $file
# speicherabbild
riscv64-unknown-elf-objcopy -O binary $file $file.bin
# hex file
riscv64-unknown-elf-objcopy -I binary $file.bin -O ihex $file.hex

xxd $file.bin
riscv64-unknown-elf-objdump --disassemble $file
riscv64-unknown-elf-objdump -d $file > $file.disassembly.txt
rm $file.o
rm $file
