-------------------------------------------------------------------------------
-- Title      : RISC-V 32-Bit FSMD Core
-- Project    : RISC-V 32-Bit Core
-------------------------------------------------------------------------------
-- File       : Core-Rtl-a.vhd
-- Author     : Binder Alexander
-- Date		  : 11.11.2019
-- Revisions  : V1, 11.11.2019 -ba
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------

architecture rtl of Core is

    -- common registers
    signal R, NxR : aRegSet;

    -- Register File
    signal RegFile, NxRegFile : aRegFile;
    constant cInitValRegFile  : aRegFile := (others => (others => '0'));

    -- bussignals for remapping
    signal i_readdata_remapped  : std_ulogic_vector(cBitWidth - 1 downto 0);
    signal d_readdata_remapped  : std_ulogic_vector(cBitWidth - 1 downto 0);
    signal d_writedata_remapped : std_ulogic_vector(cBitWidth - 1 downto 0);

begin

    -- remap bus signals to internal representation (memory is little endian, cpu uses big endian)
    -- instrucion bus
    i_readdata_remapped <= swapEndianess(to_stdULogicVector(avm_i_readdata));
    -- data bus
    d_readdata_remapped <= swapEndianess(to_stdULogicVector(avm_d_readdata));
    avm_d_writedata     <= to_StdLogicVector(swapEndianess(d_writedata_remapped));

    Registers : process(csi_clk, rsi_reset_n)
    begin
        if (rsi_reset_n = not ('1')) then
            R       <= cInitValRegSet;
            RegFile <= cInitValRegFile;
        elsif ((csi_clk'event) and (csi_clk = '1')) then
            R       <= NxR;
            RegFile <= NxRegFile;
        end if;
    end process;

    Comb : process(R, RegFile, i_readdata_remapped, d_readdata_remapped)
        variable vRegReadData1      : aRegValue      := (others => '0');
        variable vRegReadData2      : aRegValue      := (others => '0');
        variable vAluSrc1           : aCtrl2Signal   := (others => '0');
        variable vAluSrc2           : aCtrlSignal    := '0';
        variable vImm               : aImm           := (others => '0');
        variable vPCPlus4           : aPCValue       := (others => '0');
        variable vNextPC            : aPCValue       := (others => '0');
        variable vJumpAdr           : aPCValue       := (others => '0');
        variable vDataMemReadData   : aWord          := (others => '0');
        variable vDataMemWriteData  : aWord          := (others => '0');
        variable vDataMemByteEnable : aMemByteselect := (others => '0');
        variable vCsrAddrMapped     : integer        := 255;
        variable vCsrReadData       : aRegValue      := (others => '0');
        variable vALUValues         : aALUValues;

    begin
        -- default signal values
        NxR       <= R;
        NxRegFile <= RegFile;

        -- default variable values
        vRegReadData1      := (others => '0'); -- register file read data 1
        vRegReadData2      := (others => '0'); -- register file read data 2
        vAluSrc1           := (others => '0'); -- alu input 1 mux
        vAluSrc2           := '0';      -- alu input 2 mux
        vImm               := (others => '0'); -- extended Immediate
        vPCPlus4           := (others => '0'); -- current program counter plus 4
        vNextPC            := (others => '0'); -- next program counter value
        vJumpAdr           := (others => '0'); -- calculated jump address from instruction
        vDataMemReadData   := (others => '0'); -- data memory read data
        vDataMemWriteData  := (others => '0'); -- data memory write data
        vDataMemByteEnable := (others => '0'); -- data memory byte enable
        vCsrAddrMapped     := 255;      -- csr address mapped into linear space
        vCsrReadData       := (others => '0'); -- csr register read data
        vALUValues         := cALUValuesZero; -- ALU values

        -------------------------------------------------------------------------------
        -- Control Unit
        -------------------------------------------------------------------------------
        NxR.incPC        <= cNoIncPC;
        NxR.aluCalc      <= '0';
        NxR.memRead      <= '0';
        NxR.memWrite     <= '0';
        NxR.memToReg     <= cMemToRegALU;
        NxR.jumpToAdr    <= cNoJump;
        NxR.csrRead      <= '0';
        NxR.csrWriteMode <= cModeNoWrite;

        if R.ctrlState = Fetch then
            NxR.ctrlState <= ReadReg;

        elsif R.ctrlState = ReadReg then

            NxR.incPC     <= cIncPC;
            NxR.aluCalc   <= '1';
            vAluSrc1      := cALUSrc1RegFile;

            case R.curInst(aOPCodeRange) is

                when cOpRType | cOpIArith =>
                    NxR.ctrlState <= CalculateALUOp;
                    -- ALU OpCode
                    case R.curInst(aFunct3Range) is
                        when cFunct3addsub => -- add/sub
                            if R.curInst(aOPCodeRange) = cOpRType and R.curInst(cFunct7OtherInstrPos) = '1' then
                                NxR.aluOp <= ALUOpSub;
                            else
                                NxR.aluOp <= ALUOpAdd;
                            end if;
                        when cFunct3sll  => NxR.aluOp <= ALUOpSLL; -- shift left logical
                        when cFunct3slt  => NxR.aluOp <= ALUOpSLT; -- signed less than
                        when cFunct3sltu => NxR.aluOp <= ALUOpSLTU; -- unsigned less than
                        when cFunct3xor  => NxR.aluOp <= ALUOpXor; -- xor
                        when cFunct3sr => -- shift rigth logical/arithmetical                            
                            if R.curInst(cFunct7OtherInstrPos) = '0' then
                                NxR.aluOp <= ALUOpSRL;
                            else
                                NxR.aluOp <= ALUOpSRA;
                            end if;
                        when cFunct3or   => NxR.aluOp <= ALUOpOr; -- or
                        when cFunct3and  => NxR.aluOp <= ALUOpAnd; -- and
                        when others      => null;
                    end case;

                    -- Immediate or Register Instruction
                    if R.curInst(aOPCodeRange) = cOpRType then
                        vAluSrc2 := cALUSrc2RegFile;
                    else
                        vAluSrc2 := cALUSrc2ImmGen;
                    end if;

                when cOpILoad =>
                    NxR.aluOp     <= ALUOpAdd;
                    NxR.incPC     <= cNoIncPC;
                    vAluSrc2      := cALUSrc2ImmGen;
                    NxR.ctrlState <= CalculateLoad;

                when cOpSType =>
                    NxR.aluOp     <= ALUOpAdd;
                    vAluSrc2      := cALUSrc2ImmGen;
                    NxR.ctrlState <= CalculateStore;

                when cOpJType | cOpIJumpReg =>
                    -- alu config only needed for Jump Reg
                    NxR.aluOp     <= ALUOpAdd;
                    vAluSrc2      := cALUSrc2ImmGen;
                    NxR.jumpToAdr <= cJump;
                    NxR.ctrlState <= CalculateJump;

                when cOpBType =>
                    NxR.aluOp <= ALUOpSub;
                    vAluSrc2  := cALUSrc2RegFile;
                    NxR.incPC <= cNoIncPC;
                    NxR.ctrlState <= CalculateBranch;

                when cOpLUI | cOpAUIPC =>
                    NxR.aluOp     <= ALUOpAdd;
                    NxR.ctrlState <= CalculateUpperimmediate;
                    if R.curInst(aOPCodeRange) = cOpLUI then
                        vAluSrc1 := cALUSrc1Zero;
                    else
                        vAluSrc1 := cALUSrc1PC;
                    end if;
                    vAluSrc2      := cALUSrc2ImmGen;

                when cOpFence =>        -- implemented as NOP
                    NxR.aluCalc   <= '0';
                    NxR.ctrlState <= Wait0;

                when cOpSys =>
                    case R.curInst(aFunct3Range) is
                        when cSysEnv =>
                            NxR.ctrlState <= Trap;
                        when others =>
                            NxR.incPC     <= cNoIncPC;
                            NxR.ctrlState <= CalculateSys;
                    end case;

                when others =>
                    null;               -- not implemented

            end case;

        -- U-Type Load Immediate
        elsif R.ctrlState = CalculateUpperimmediate then
            NxR.regWriteEn <= '1';
            NxR.ctrlState  <= WriteReg;

        -- J-Type Jump Instruction
        elsif R.ctrlState = CalculateJump then
            NxR.regWriteEn <= '1';
            NxR.ctrlState  <= WriteReg;

        -- B-Type Conditional Branch
        elsif R.ctrlState = CalculateBranch then
            NxR.ctrlState <= CheckJump;

        -- I-Type Load Instruction
        elsif R.ctrlState = CalculateLoad then
            NxR.memRead   <= '1';
            NxR.incPC     <= cIncPC;
            NxR.ctrlState <= DataAccess0;

        -- S-Type Store Instruction
        elsif R.ctrlState = CalculateStore then
            NxR.memWrite  <= '1';
            NxR.ctrlState <= DataAccess0;

        -- R-Type or I-Type Register Instruction
        elsif R.ctrlState = CalculateALUOp then
            NxR.regWriteEn <= '1';
            NxR.ctrlState  <= WriteReg;

        -- CSR Instruction
        elsif R.ctrlState = CalculateSys then
            case R.curInst(aFunct3Range) is
                when cSysRW | cSysRWI =>
                    if R.curInst(11 downto 7) = "00000" then
                        NxR.csrRead <= '0';
                    else
                        NxR.csrRead <= '1';
                    end if;
                    NxR.csrWriteMode <= cModeWrite;
                when cSysRS | cSysRSI =>
                    NxR.csrRead <= '1';
                    if R.curInst(19 downto 15) = "00000" then
                        NxR.csrWriteMode <= cModeNoWrite;
                    else
                        NxR.csrWriteMode <= cModeSet;
                    end if;
                when cSysRC | cSysRCI =>
                    NxR.csrRead <= '1';
                    if R.curInst(19 downto 15) = "00000" then
                        NxR.csrWriteMode <= cModeNoWrite;
                    else
                        NxR.csrWriteMode <= cModeClear;
                    end if;
                when others => null;
            end case;
            NxR.incPC     <= cIncPC;
            NxR.ctrlState <= DataAccess0;        

        elsif R.ctrlState = DataAccess0 then
            case R.curInst(aOPCodeRange) is
                when cOpILoad =>
                    NxR.memToReg  <= cMemToRegMem;
                    NxR.ctrlState <= DataAccess1;
                when cOpSType =>
                    NxR.ctrlState <= Wait1;
                when cOpSys =>
                    NxR.regWriteEn <= R.csrRead;
                    NxR.ctrlState  <= WriteReg;
                when others =>
                    null;
            end case;

        elsif R.ctrlState = DataAccess1 then
            NxR.regWriteEn <= '1';
            NxR.ctrlState  <= WriteReg;

        elsif R.ctrlState = CheckJump then
            NxR.incPC     <= cIncPC;
            case R.curInst(aFunct3Range) is
                when cCondEq =>
                    if R.statusReg(cStatusZeroBit) = '1' then
                        NxR.jumpToAdr <= cJump;
                    end if;
                when cCondNe =>
                    if R.statusReg(cStatusZeroBit) = '0' then
                        NxR.jumpToAdr <= cJump;
                    end if;
                when cCondLt =>
                    if R.statusReg(cStatusNegBit) = '1' then
                        NxR.jumpToAdr <= cJump;
                    end if;
                when cCondGe =>
                    if R.statusReg(cStatusNegBit) = '0' then
                        NxR.jumpToAdr <= cJump;
                    end if;
                when cCondLtu =>
                    if R.statusReg(cStatusCarryBit) = '1' then
                        NxR.jumpToAdr <= cJump;
                    end if;
                when cCondGeu =>
                    if R.statusReg(cStatusCarryBit) = '0' then
                        NxR.jumpToAdr <= cJump;
                    end if;
                when others =>
                    null;
            end case;
            NxR.ctrlState <= Wait0;

        elsif R.ctrlState = WriteReg then
            NxR.regWriteEn <= '0';
            NxR.ctrlState  <= Fetch;

        elsif R.ctrlState = Wait0 then
            NxR.ctrlState <= Wait1;

        elsif R.ctrlState = Wait1 then
            NxR.ctrlState <= Fetch;

        elsif R.ctrlState = Trap then
            if R.curInst(31 downto 20) = cMTrapRet then
                NxR.ctrlState <= Wait1;
            else
                NxR.ctrlState <= Trap;
            end if;
        else
            null;
        end if;

        -------------------------------------------------------------------------------
        -- Program Counter
        -------------------------------------------------------------------------------
        vPCPlus4 := std_ulogic_vector(to_unsigned(
            to_integer(unsigned(R.curPC)) + cPCIncrement, cPCWidth));

        -------------------------------------------------------------------------------
        -- Instruction Memory
        -------------------------------------------------------------------------------
        if R.ctrlState = WriteReg or R.ctrlState = Wait1 then
            avm_i_read <= '1';
        else
            avm_i_read <= '0';
        end if;

        avm_i_address <= std_logic_vector(R.curPC);
        NxR.curInst   <= std_ulogic_vector(i_readdata_remapped);

        -------------------------------------------------------------------------------
        -- Register File
        -------------------------------------------------------------------------------
        -- read registers
        vRegReadData1 := RegFile(to_integer(unsigned(R.curInst(aRs1AddrRange))));
        vRegReadData2 := RegFile(to_integer(unsigned(R.curInst(aRs2AddrRange))));

        -- write register
        if R.regWriteEn = '1' and R.curInst(aRdAddrRange) /= "00000" then
            NxRegFile(to_integer(unsigned(R.curInst(aRdAddrRange)))) <= R.regWriteData;
        end if;

        -------------------------------------------------------------------------------
        -- Immediate Extension
        -------------------------------------------------------------------------------
        vImm := (others => '0');
        case R.curInst(aOPCodeRange) is
            when cOpRType =>
                vImm := (others => '0'); -- doesnt matter what immediate extension does when not needed
            when cOpIJumpReg | cOpILoad | cOpIArith =>
                vImm(aITypeRangeInImmRange)                           := R.curInst(aITypeImmsrcRange);
                vImm(vImm'high downto aITypeRangeInImmRange'high + 1) := (others => R.curInst(aJTypeImmsrc1Range'high));
            when cOpSType =>
                vImm(aSTypeRangeInImmRange)                           := R.curInst(aSTypeImmsrc1Range) & R.curInst(aSTypeImmsrc2Range);
                vImm(vImm'high downto aSTypeRangeInImmRange'high + 1) := (others => R.curInst(aBTypeImmsrc1Range'high));
            when cOpBType =>
                vImm(aBTypeRangeInImmRange)                           := R.curInst(aBTypeImmsrc1Range) & R.curInst(aBTypeImmsrc2Range) & R.curInst(aBTypeImmsrc3Range) & R.curInst(aBTypeImmsrc4Range);
                vImm(vImm'high downto aBTypeRangeInImmRange'high + 1) := (others => R.curInst(aBTypeImmsrc1Range'high));
            when cOpLUI | cOpAUIPC =>
                vImm(aUTypeRangeInImmRange) := R.curInst(aUTypeImmsrcRange);
            when cOpJType =>
                vImm(aJTypeRangeInImmRange)                           := R.curInst(aJTypeImmsrc1Range) & R.curInst(aJTypeImmsrc2Range) & R.curInst(aJTypeImmsrc3Range) & R.curInst(aJTypeImmsrc4Range);
                vImm(vImm'high downto aJTypeRangeInImmRange'high + 1) := (others => R.curInst(aJTypeImmsrc1Range'high));
            when others =>
                vImm := (others => '-');
        end case;

        -------------------------------------------------------------------------------
        -- ALU
        -------------------------------------------------------------------------------
        -- add and substract from alu needed for other operations
        if (R.aluOp = ALUOpAdd) then
            vALUValues.addsubRes := std_ulogic_vector(unsigned('0' & R.aluData1) + unsigned('0' & R.aluData2));
            if ((R.aluData1(R.aluData1'high) = R.aluData2(R.aluData2'high)) and (R.aluData1(R.aluData1'high) /= vALUValues.addsubRes(vALUValues.addsubRes'high - 1))) then
                vALUValues.addsubCarry := '1';
            end if;
        else
            vALUValues.addsubRes := std_ulogic_vector(unsigned('0' & R.aluData1) - unsigned('0' & R.aluData2));
            if ((R.aluData1(R.aluData1'high) /= R.aluData2(R.aluData2'high)) and (R.aluData1(R.aluData1'high) /= vALUValues.addsubRes(vALUValues.addsubRes'high - 1))) then
                vALUValues.addsubCarry := '1';
            end if;
        end if;

        -- calculate shift amount
        vALUValues.shiftAmount := to_integer(unsigned(R.aluData2(aALUShiftRange)));

        case R.aluOp is
            when ALUOpAdd | ALUOpSub =>
                vALUValues.aluRawRes := vALUValues.addsubRes;
            when ALUOpSLT =>
                vALUValues.aluRawRes    := (others => '0');
                vALUValues.aluRawRes(0) := (vALUValues.addsubRes(vALUValues.addsubRes'high - 1) or vALUValues.addsubCarry) and not (not R.aluData1(R.aluData1'high) and R.aluData2(R.aluData2'high));

            when ALUOpSLTU =>
                vALUValues.aluRawRes := (0 => vALUValues.addsubRes(vALUValues.addsubRes'high), others => '0');
            when ALUOpAnd =>
                vALUValues.aluRawRes := '0' & (R.aluData1 AND R.aluData2);
            when ALUOpOr =>
                vALUValues.aluRawRes := '0' & (R.aluData1 OR R.aluData2);
            when ALUOpXor =>
                vALUValues.aluRawRes := '0' & (R.aluData1 XOR R.aluData2);
            when ALUOpSLL =>
                vALUValues.aluRawRes := std_ulogic_vector(
                    shift_left(unsigned('0' & R.aluData1), vALUValues.shiftAmount));
            when ALUOpSRL | ALUOpSRA =>
                vALUValues.srValue   := '0';
                if (R.aluOp = ALUOpSRA) then
                    vALUValues.srValue := R.aluData1(R.aluData1'high);
                end if;
                vALUValues.aluRawRes := std_ulogic_vector(
                    shift_right(signed(vALUValues.srValue & R.aluData1), vALUValues.shiftAmount));
            when ALUOpNOP =>
                vALUValues.aluRawRes := (others => '-');
        end case;

        -- Remove Carry Bit and Store New Value
        if (R.aluCalc = '1') then
            vALUValues.aluRes := vALUValues.aluRawRes(vALUValues.aluRes'range);
            NxR.aluRes        <= vALUValues.aluRes;
        else
            vALUValues.aluRes := R.aluRes;
        end if;

        -- Set Status Registers
        NxR.statusReg(cStatusZeroBit)  <= nor_reduce(vALUValues.aluRawRes(vALUValues.aluRawRes'high - 1 downto 0));
        NxR.statusReg(cStatusNegBit)   <= vALUValues.aluRawRes(vALUValues.aluRawRes'high - 1);
        NxR.statusReg(cStatusCarryBit) <= vALUValues.aluRawRes(vALUValues.aluRawRes'high);

        -------------------------------------------------------------------------------
        -- Jump Adress Calculation
        -------------------------------------------------------------------------------
        if R.curInst(aOPCodeRange) = cOpIJumpReg then -- JAL or JALR
            vJumpAdr := vALUValues.aluRes;
        else
            vJumpAdr := std_ulogic_vector(resize(unsigned(vImm) + unsigned(R.curPC), cImmLen));
        end if;

        -------------------------------------------------------------------------------
        -- CSR Unit
        -------------------------------------------------------------------------------
        vCsrAddrMapped := mapCsrAddr(R.curInst(31 downto 20));

        if R.csrRead = '1' then
            if mapCsrAddrValid(vCsrAddrMapped) then
                vCsrReadData := R.csrReg(vCsrAddrMapped);
            end if;
        end if;

        if R.csrWriteMode /= cModeNoWrite then
            if mapCsrAddrValid(vCsrAddrMapped) then
                case R.csrWriteMode is
                    when cModeWrite =>
                        NxR.csrReg(vCsrAddrMapped) <= R.csrWriteData;
                    when cModeSet =>
                        NxR.csrReg(vCsrAddrMapped) <= R.csrWriteData or vCsrReadData;
                    when cModeClear =>
                        NxR.csrReg(vCsrAddrMapped) <= (not R.csrWriteData) and vCsrReadData;
                    when others =>
                        null;
                end case;
            end if;
        end if;

        -------------------------------------------------------------------------------
        -- Data Memory
        -------------------------------------------------------------------------------
        case R.curInst(aFunct3Range) is
            when cMemByte =>
                vDataMemReadData                                  := std_ulogic_vector(
                    resize(signed(d_readdata_remapped(cByte - 1 downto 0)), cBitWidth));
                vDataMemWriteData(4 * cByte - 1 downto 3 * cByte) := vRegReadData2(cByte - 1 downto 0);
                vDataMemByteEnable                                := cEnableByte;

            when cMemHalfWord =>
                vDataMemReadData                                  := std_ulogic_vector(
                    resize(signed(d_readdata_remapped(2 * cByte - 1 downto 0)), cBitWidth));
                vDataMemWriteData(4 * cByte - 1 downto 2 * cByte) := vRegReadData2(2 * cByte - 1 downto 0);
                vDataMemByteEnable                                := cEnableHalfWord;

            when cMemWord =>
                vDataMemReadData   := std_ulogic_vector(
                    resize(signed(d_readdata_remapped(4 * cByte - 1 downto 0)), cBitWidth));
                vDataMemWriteData  := vRegReadData2;
                vDataMemByteEnable := cEnableWord;

            when cMemUnsignedByte =>
                vDataMemReadData                                  := std_ulogic_vector(resize(unsigned(
                    d_readdata_remapped(cByte - 1 downto 0)), cBitWidth));
                vDataMemWriteData(4 * cByte - 1 downto 3 * cByte) := vRegReadData2(cByte - 1 downto 0);
                vDataMemByteEnable                                := cEnableByte;

            when cMemUnsignedHalfWord =>
                vDataMemReadData                                  := std_ulogic_vector(resize(unsigned(
                    d_readdata_remapped(2 * cByte - 1 downto 0)), cBitWidth));
                vDataMemWriteData(4 * cByte - 1 downto 2 * cByte) := vRegReadData2(2 * cByte - 1 downto 0);
                vDataMemByteEnable                                := cEnableHalfWord;

            when others =>
                vDataMemReadData   := (others => '0');
                vDataMemWriteData  := (others => '0');
                vDataMemByteEnable := (others => '0');
        end case;

        avm_d_address        <= to_StdLogicVector(vALUValues.aluRes);
        avm_d_byteenable     <= to_StdLogicVector(vDataMemByteEnable);
        avm_d_write          <= std_logic(R.memWrite);
        d_writedata_remapped <= (vDataMemWriteData);
        avm_d_read           <= std_logic(R.memRead);

        -------------------------------------------------------------------------------
        -- Multiplexer
        -------------------------------------------------------------------------------

        -- MUX ALUSrc
        if vAluSrc2 = cALUSrc2RegFile then
            NxR.aluData2 <= vRegReadData2;
        else
            NxR.aluData2 <= vImm;
        end if;

        -- MUX RegWriteData
        if R.jumpToAdr = cJump then
            NxR.regWriteData <= vPCPlus4;
        elsif R.memToReg = cMemToRegMem then
            NxR.regWriteData <= vDataMemReadData;
        elsif R.csrRead = '1' then
            NxR.regWriteData <= vCsrReadData;
        else
            NxR.regWriteData <= vALUValues.aluRes;
        end if;

        -- Mux PCInc
        if R.incPC = cNoIncPC then
            vNextPC := R.curPC;
        else
            vNextPC := vPCPlus4;
        end if;
        -- Mux PCJump
        if R.jumpToAdr = cNoJump then
            NxR.curPC <= vNextPC;
        else
            NxR.curPC <= vJumpAdr;
        end if;

        -- Mux ALU1Src
        case vAluSrc1 is
            when cALUSrc1RegFile => NxR.aluData1 <= vRegReadData1;
            when cALUSrc1Zero    => NxR.aluData1 <= (others => '0');
            when cALUSrc1PC      => NxR.aluData1 <= R.curPC;
            when others          => null;
        end case;

        -- Mux CsrWriteData
        if R.curInst(14) = cCsrDataReg then
            NxR.csrWriteData <= vRegReadData1;
        else
            NxR.csrWriteData <= std_ulogic_vector(resize(unsigned(R.curInst(19 downto 15)), cBitWidth));
        end if;

    end process;

end architecture rtl;
