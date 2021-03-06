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

    constant MemSize       : natural := 16384;
    type aMemory is array (0 to MemSize - 1) of std_logic_vector(cByte - 1 downto 0);
    shared variable Memory : aMemory := (others => (others => '0'));

    signal clk   : std_ulogic := '0';
    signal reset : std_logic  := '0';

    signal instAddress  : std_logic_vector(cBitWidth - 1 downto 0);
    signal instRead     : std_logic;
    signal instReadData : std_logic_vector(cBitWidth - 1 downto 0) := (others => '0');

    signal dataAddress    : std_logic_vector(cBitWidth - 1 downto 0);
    signal dataByteEnable : std_logic_vector(cBitWidth / cByte - 1 downto 0);
    signal dataWrite      : std_logic;
    signal dataWriteData  : std_logic_vector(cBitWidth - 1 downto 0);
    signal dataRead       : std_logic                                := '1';
    signal dataReadData   : std_logic_vector(cBitWidth - 1 downto 0) := (others => '0');

    -- variables for automatic tests
    signal test_finished     : bit := '0';
    signal test_finished_ack : bit := '0';

    type aTestCase is record
        filename     : line;
        pcSuccessval : aRegValue;
    end record;

    type aTestSuite is array (natural range <>) of aTestCase;

begin

    -- Clock Gen
    clk <= not (clk) after 10 ns;

    UUT : entity work.Core(rtl)
        port map(
            csi_clk           => clk,
            rsi_reset_n       => reset,
            avm_i_address     => instAddress,
            avm_i_read        => instRead,
            avm_i_readdata    => instReadData,
            avm_i_waitrequest => '0',
            avm_d_address     => dataAddress,
            avm_d_byteenable  => dataByteEnable,
            avm_d_write       => dataWrite,
            avm_d_writedata   => dataWriteData,
            avm_d_read        => dataRead,
            avm_d_readdata    => dataReadData,
            avm_d_waitrequest => '0'
        );

    ReadROM : process is
        type char_file_t is file of character;
        file char_file  : char_file_t;
        variable char_v : character;
        variable i      : natural := 0;

        type char_arr is array (63 downto 0) of character;
        file test_file        : text;
        variable line_num     : line;
        variable line_content : string(1 to 64);

        -- select test mode
        -- Single File runs the test with a single specified file
        -- FullISA runs the riscv isa tests
        type aTestMode is (SingleFile, FullISA);
        constant testMode : aTestMode := FullISA;

        constant cNoTests : integer := 48;

        variable vTestSuite : aTestSuite(0 to cNoTests - 1) := (
            (new string'("riscv-test/rv32um-p-remu.bin"), x"00000280"),
            (new string'("riscv-test/rv32um-p-rem.bin"), x"00000280"),
            (new string'("riscv-test/rv32um-p-div.bin"), x"00000280"),
            (new string'("riscv-test/rv32um-p-divu.bin"), x"00000284"),
            (new string'("riscv-test/rv32um-p-mulhsu.bin"), x"0000066C"),
            (new string'("riscv-test/rv32um-p-mulh.bin"), x"0000066C"),
            (new string'("riscv-test/rv32ui-p-add.bin"), x"00000684"),
            (new string'("riscv-test/rv32ui-p-addi.bin"), x"0000042C"),
            (new string'("riscv-test/rv32ui-p-and.bin"), x"0000065C"),
            (new string'("riscv-test/rv32ui-p-andi.bin"), x"00000364"),
            (new string'("riscv-test/rv32ui-p-auipc.bin"), x"000001EC"),
            (new string'("riscv-test/rv32ui-p-beq.bin"), x"00000464"),
            (new string'("riscv-test/rv32ui-p-bge.bin"), x"000004C4"),
            (new string'("riscv-test/rv32ui-p-bgeu.bin"), x"000004F8"),
            (new string'("riscv-test/rv32ui-p-blt.bin"), x"00000464"),
            (new string'("riscv-test/rv32ui-p-bltu.bin"), x"00000498"),
            (new string'("riscv-test/rv32ui-p-bne.bin"), x"00000468"),
            (new string'("riscv-test/rv32ui-p-jal.bin"), x"000001f8"),
            (new string'("riscv-test/rv32ui-p-jalr.bin"), x"00000298"),
            (new string'("riscv-test/rv32ui-p-lb.bin"), x"000003f0"),
            (new string'("riscv-test/rv32ui-p-lbu.bin"), x"000003f0"),
            (new string'("riscv-test/rv32ui-p-lh.bin"), x"00000410"),
            (new string'("riscv-test/rv32ui-p-lhu.bin"), x"00000424"),
            (new string'("riscv-test/rv32ui-p-lui.bin"), x"00000204"),
            (new string'("riscv-test/rv32ui-p-lw.bin"), x"00000430"),
            (new string'("riscv-test/rv32ui-p-or.bin"), x"00000668"),
            (new string'("riscv-test/rv32ui-p-ori.bin"), x"00000380"),
            (new string'("riscv-test/rv32ui-p-sb.bin"), x"00000598"),
            (new string'("riscv-test/rv32ui-p-sh.bin"), x"0000061C"),
            (new string'("riscv-test/rv32ui-p-simple.bin"), x"00000188"),
            (new string'("riscv-test/rv32ui-p-sll.bin"), x"000006F4"),
            (new string'("riscv-test/rv32ui-p-slli.bin"), x"00000428"),
            (new string'("riscv-test/rv32ui-p-slt.bin"), x"0000066C"),
            (new string'("riscv-test/rv32ui-p-slti.bin"), x"00000418"),
            (new string'("riscv-test/rv32ui-p-sltiu.bin"), x"00000418"),
            (new string'("riscv-test/rv32ui-p-sltu.bin"), x"0000066C"),
            (new string'("riscv-test/rv32ui-p-sra.bin"), x"00000740"),
            (new string'("riscv-test/rv32ui-p-srai.bin"), x"0000045C"),
            (new string'("riscv-test/rv32ui-p-srl.bin"), x"00000728"),
            (new string'("riscv-test/rv32ui-p-srli.bin"), x"00000444"),
            (new string'("riscv-test/rv32ui-p-sub.bin"), x"00000664"),
            (new string'("riscv-test/rv32ui-p-sw.bin"), x"00000628"),
            (new string'("riscv-test/rv32ui-p-xor.bin"), x"00000664"),
            (new string'("riscv-test/rv32ui-p-xori.bin"), x"00000388"),
            (new string'("riscv-test/rv32mi-p-csr.bin"), x"00000354"),
            (new string'("riscv-test/rv32um-p-mul.bin"), x"0000066C"),
            (new string'("riscv-test/rv32um-p-mulhu.bin"), x"0000066C"),
            (new string'("riscv-test/rv32ui-p-fence_i.bin"), x"00000290")
        );

    begin
        if testMode = SingleFile then
            file_open(char_file, "../../../../../test/rv32mi-p-csr.bin");
            while not endfile(char_file) and (i < MemSize) loop
                read(char_file, char_v);
                Memory(i) := std_logic_vector(to_unsigned(character'pos(char_v), cByte));
                i         := i + 1;
            end loop;

            report integer'image(i) & " Bytes written into ROM!";
            file_close(char_file);

        elsif testMode = FullISA then
            for j in 0 to vTestSuite'length - 1 loop
                -- open next testfile
                file_open(char_file, "../../../../../test/" & vTestSuite(j).filename.all);

                -- reset memory
                Memory := (others => (others => '0'));

                -- read binary file
                i := 0;
                while not endfile(char_file) and (i < MemSize) loop
                    read(char_file, char_v);
                    Memory(i) := std_logic_vector(to_unsigned(character'pos(char_v), cByte));
                    i         := i + 1;
                end loop;

                file_close(char_file);

                -- check for finished test
                wait until test_finished = '1';

                if (instAddress /= to_stdlogicVector(vTestSuite(j).pcSuccessval)) then
                    report "Test produced an error: " & vTestSuite(j).filename.all
                    severity failure;
                else
                    report vTestSuite(j).filename.all & " executed successfully!"
                    severity note;
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

    CheckResult : process(clk) is
        variable last_address        : integer := 0;
        variable address_cycle_count : integer := 0;
    begin
        if rising_edge(clk) then
            if to_integer(unsigned(instAddress)) = last_address then
                address_cycle_count := address_cycle_count + 1;
            else
                last_address        := to_integer(unsigned(instAddress));
                address_cycle_count := 0;
            end if;

            -- when address hasn't changed in 32 cycles assume program finished
            if address_cycle_count = 45 then
                test_finished <= '1';
            end if;

            if test_finished_ack = '1' then
                reset               <= '0';
                address_cycle_count := 0;
                test_finished       <= '0';
            else
                reset <= 'Z';
            end if;
        end if;
    end process CheckResult;

    Stimuli : process is
    begin
        reset <= 'H' after 100 ns;
        wait;

    end process Stimuli;

    InstructionMemory : process(clk) is
    begin
        if rising_edge(clk) then
            if (instRead = '1') and (to_integer(unsigned(instAddress)) + cByteWidth - 1) < MemSize then
                readInstructionMemory : for i in 0 to cByteWidth - 1 loop
                    instReadData(((1 + i) * cByte) - 1 downto (i) * cByte) <= Memory(to_integer(unsigned(instAddress)) + i);
                end loop;
            end if;
        end if;
    end process InstructionMemory;

    DataMemory : process(clk) is
    begin
        if rising_edge(clk) then
            if (dataRead = '1') and (to_integer(unsigned(dataAddress)) + cByteWidth - 1) < MemSize then
                readDataMemory : for i in 0 to cByteWidth - 1 loop
                    dataReadData(((1 + i) * cByte) - 1 downto (i) * cByte) <= Memory(to_integer(unsigned(dataAddress)) + i);
                end loop;
            end if;

            if (dataWrite = '1') and (to_integer(unsigned(dataAddress)) + cByteWidth - 1) < MemSize then
                -- write every byte that byteenable is set for
                for i in 0 to 3 loop
                    if dataByteEnable(i) = '1' then
                        Memory(to_integer(unsigned(dataAddress)) + i) := dataWriteData(((1 + i) * cByte - 1) downto i * cByte);
                    end if;
                end loop;

            end if;
        end if;
    end process DataMemory;

end architecture bhv;
