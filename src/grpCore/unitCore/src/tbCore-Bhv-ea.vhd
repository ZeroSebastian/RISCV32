-------------------------------------------------------------------------------
-- Title      : Testbench for RISC-V 32-Bit FSMD Core
-- Project    : RISC-V 32-Bit Core
-------------------------------------------------------------------------------
-- File       : tbCore-ea.vhd
-- Author	  : Binder Alexander
-- Date		  : 11.11.2019
-- Revisions  : V1, 11.11.2019 -ba
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use std.textio.all;
use std.standard.all;

library work;
use work.RISCV.all;
use work.Global.all;

entity tbCore is
end entity tbCore;

architecture bhv of tbCore is

    constant MemSize : natural := 16384;
    type aMemory is array (0 to MemSize-1) of std_logic_vector(cByte-1 downto 0);
    shared variable Memory : aMemory := (others  => (others => '0'));

    signal clk              : std_ulogic := '0';
    signal reset            : std_logic := '0';

    signal instAddress      : std_logic_vector(cBitWidth-1 downto 0);
    signal instRead         : std_logic;
    signal instReadData     : std_logic_vector(cBitWidth-1 downto 0) := (others => '0');

    signal dataAddress      : std_logic_vector(cBitWidth-1 downto 0);
    signal dataByteEnable   : std_logic_vector(cBitWidth/cByte-1 downto 0);
    signal dataWrite        : std_logic;
    signal dataWriteData    : std_logic_vector(cBitWidth-1 downto 0);
    signal dataRead         : std_logic := '1';
    signal dataReadData     : std_logic_vector(cBitWidth-1 downto 0) := (others => '0');

    -- variables for automatic tests
    signal test_finished    : bit := '0';
    signal test_finished_ack: bit := '0';
    
    -- filenames
    type aTestFiles is array (natural range <>) of line;
    -- pc values
    type aPCValues is array (natural range <>) of aRegValue;
    

begin

-- Clock Gen
clk <= not(clk) after 10 ns;

UUT: entity work.Core(rtl)
    port map(
        csi_clk             => clk,
        rsi_reset_n         => reset,

        avm_i_address       => instAddress,
        avm_i_read          => instRead,
        avm_i_readdata      => instReadData,
        avm_i_waitrequest   => '0',

        avm_d_address       => dataAddress,
        avm_d_byteenable    => dataByteEnable,
        avm_d_write         => dataWrite,
        avm_d_writedata     => dataWriteData,
        avm_d_read          => dataRead,
        avm_d_readdata      => dataReadData,
        avm_d_waitrequest   => '0'
    );

ReadROM: process is
    type char_file_t is file of character;
    file char_file          : char_file_t;
    variable char_v         : character;
    variable i              : natural := 0;

    type char_arr is array (63 downto 0) of character;
    file test_file          : text;
    variable line_num       : line;
    variable line_content   : string(1 to 64);

    -- select test mode
    -- Single File runs the test with a single specified file
    -- FullISA runs the riscv isa tests
    type aTestMode is (SingleFile, FullISA);
    constant testMode : aTestMode := FullISA;
    
    constant cNoTests : integer := 35;
    
    variable vTestFiles : aTestFiles(0 to cNoTests - 1) := (
    new string'("rv32ui-p-add.bin"),
    new string'("rv32ui-p-addi.bin"),
    new string'("rv32ui-p-and.bin"),
    new string'("rv32ui-p-andi.bin"),
    new string'("rv32ui-p-auipc.bin"),
    new string'("rv32ui-p-beq.bin"),
    new string'("rv32ui-p-bgeu.bin"),
    new string'("rv32ui-p-blt.bin"),
    new string'("rv32ui-p-bltu.bin"),
    new string'("rv32ui-p-fence_i.bin"),
    new string'("rv32ui-p-jal.bin"),
    new string'("rv32ui-p-jalr.bin"),
    new string'("rv32ui-p-lb.bin"),
    new string'("rv32ui-p-lbu.bin"),
    new string'("rv32ui-p-lh.bin"),
    new string'("rv32ui-p-lhu.bin"),
    new string'("rv32ui-p-lui.bin"),
    new string'("rv32ui-p-lw.bin"),
    new string'("rv32ui-p-or.bin"),
    new string'("rv32ui-p-ori.bin"),    
    new string'("rv32ui-p-sb.bin"),
    new string'("rv32ui-p-sh.bin"),
    new string'("rv32ui-p-simple.bin"),
    new string'("rv32ui-p-sll.bin"),
    new string'("rv32ui-p-slti.bin"),
    new string'("rv32ui-p-sltiu.bin"),
    new string'("rv32ui-p-sltu.bin"),
    new string'("rv32ui-p-sra.bin"),
    new string'("rv32ui-p-srai.bin"),
    new string'("rv32ui-p-srl.bin"),    
    new string'("rv32ui-p-srli.bin"),
    new string'("rv32ui-p-sub.bin"),
    new string'("rv32ui-p-sw.bin"),
    new string'("rv32ui-p-xor.bin"),
    new string'("rv32ui-p-xori.bin")
    --new string'("rv32ui-p-srl.bin")
    );
    
    constant cPCValues : aPCValues (0 to cNoTests - 1) := (
        x"000005F4",
        x"0000039C",
        x"000005CC",
        x"000002D4",
        x"0000015C",
        x"000003D4",
        x"00000468",
        x"000003D4",
        x"00000408",
        x"000001F8",
        x"00000168",
        x"000001E8",
        x"00000360",
        x"00000360",
        x"00000380",
        x"00000394",
        x"00000174",
        x"000003A0",
        x"000005D8",
        x"000002F0",
        x"00000508",
        x"0000058C",
        x"00000100",
        x"00000664",
        x"00000388",
        x"00000388",
        x"000005Dc",
        x"000006b0",
        x"000003CC",
        x"00000698",
        x"000003B4",
        x"000005D4",
        x"00000598",
        x"000005D4",
        x"000002F8"
    );
    
begin

    if testMode = SingleFile then
        file_open(char_file, "../../../../../test/rv32mi-p-csr.bin");
        while not endfile(char_file) and (i < MemSize) loop
            read(char_file, char_v);
            Memory(i) := std_logic_vector(to_unsigned(character'pos(char_v), cByte));
            i := i+1;
        end loop;

        report integer'image(i) & " Bytes written into ROM!";
        file_close(char_file);

    elsif testMode = FullISA then
        for j in 0 to vTestFiles'length - 1 loop
            -- open next testfile
            file_open(char_file, "../../../../../test/" & vTestFiles(j).all);
            
            -- reset memory
            Memory := (others=>(others=>'0'));

            -- read binary file
            i := 0;
            while not endfile(char_file) and (i < MemSize) loop
                 read(char_file, char_v);
                 Memory(i) := std_logic_vector(to_unsigned(character'pos(char_v), cByte));
                 i := i+1;
            end loop;

            file_close(char_file);

            -- check for finished test
            wait until test_finished = '1';

            if(instAddress /= to_stdlogicVector(cPCValues(j))) then 
                report "Test produced an error: " & vTestFiles(j).all
                severity failure;
            end if;
            
            if Memory(to_integer(unsigned(instAddress)) + 4) /= x"00" then
                report "Test produced an error: " & "knalle"
                severity failure;
            end if;

            test_finished_ack <= '1';
            wait until test_finished = '0';
            test_finished_ack <= '0';
        end loop;
        
        
        

        file_close(test_file);
        report "All Tests finished without errors!";
        wait;

    end if;

    wait;
end process ReadROM;

CheckResult: process(clk) is
    variable last_address           : integer := 0;
    variable address_cycle_count    : integer := 0;
begin
    if rising_edge(clk) then
        if to_integer(unsigned(instAddress)) = last_address then
            address_cycle_count := address_cycle_count + 1;
        else
            last_address := to_integer(unsigned(instAddress));
            address_cycle_count := 0;
        end if;

        -- when address hasn't changed in 32 cycles assume program finished
        if address_cycle_count = 32 then
            test_finished <= '1';
        end if;

        if test_finished_ack = '1' then
            reset <= '0';
            address_cycle_count := 0;
            test_finished <= '0';
        else
            reset <= 'Z';
        end if;
    end if;
end process CheckResult;

Stimuli: process is
begin
    reset <= 'H' after 100 ns;
    wait;

end process Stimuli;

InstructionMemory: process(clk) is
begin
    if rising_edge(clk) then
        if (instRead = '1') and (to_integer(unsigned(instAddress))+cByteWidth-1) < MemSize then
            readInstructionMemory : for i in 0 to cByteWidth-1 loop
                instReadData(((4-i)*cByte)-1 downto (3-i)*cByte) <= Memory(to_integer(unsigned(instAddress))+i);
            end loop;
        end if;
    end if;
end process InstructionMemory;

DataMemory: process(clk) is
begin

    if rising_edge(clk) then
        if (dataRead = '1') and (to_integer(unsigned(dataAddress))+cByteWidth-1) < MemSize then
            readDataMemory : for i in 0 to cByteWidth-1 loop
                dataReadData(((4-i)*cByte)-1 downto (3-i)*cByte) <= Memory(to_integer(unsigned(dataAddress))+i);
            end loop;
        end if;

        if (dataWrite = '1') and (to_integer(unsigned(dataAddress))+cByteWidth-1) < MemSize then
            if dataByteEnable = "1111" then
                writeWord : for i in 0 to 3 loop
                        Memory(to_integer(unsigned(dataAddress))+i) := dataWriteData(((4-i)*cByte)-1 downto (3-i)*cByte);
                end loop;
            elsif dataByteEnable = "0011" then
                writeHalfword: for i in 0 to 1 loop
                    Memory(to_integer(unsigned(dataAddress))+i) := dataWriteData(((2-i)*cByte)-1 downto (1-i)*cByte);
                end loop;
            elsif dataByteEnable = "0001" then
                Memory(to_integer(unsigned(dataAddress))) := dataWriteData(cByte-1 downto 0);
            end if;
        end if;
    end if;
end process DataMemory;

end architecture bhv;
