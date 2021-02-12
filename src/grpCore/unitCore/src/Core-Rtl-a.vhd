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
        variable vAluSrc2           : aCtrl2Signal   := (others => '0');
        variable vImm               : aImm           := (others => '0');
        variable vPCPlus4           : aPCValue       := (others => '0');
        variable vJumpAdr           : aPCValue       := (others => '0');
        variable vDataMemReadData   : aWord          := (others => '0');
        variable vDataMemWriteData  : aWord          := (others => '0');
        variable vDataMemByteEnable : aMemByteselect := (others => '0');
        variable vCsrAddrMapped     : integer        := 255;
        variable vCsrReadData       : aRegValue      := (others => '0');
        variable vALUValues         : aALUValues;
        variable vALUOp             : aALUOp;
        variable vRegWritedataSrc   : aCtrl2Signal;
        variable vRegFileWriteEN    : std_ulogic;
        variable vRegWriteData      : aRegValue;
        variable vPCInc             : std_ulogic;

    begin
        -- default signal values
        NxR       <= R;
        NxRegFile <= RegFile;

        -- default variable values
        vRegReadData1      := (others => '0'); -- register file read data 1
        vRegReadData2      := (others => '0'); -- register file read data 2
        vAluSrc1           := cALUSrc1RegFile; -- alu input 1 mux
        vAluSrc2           := (others => '0'); -- alu input 2 mux
        vImm               := (others => '0'); -- extended Immediate
        vPCPlus4           := (others => '0'); -- current program counter plus 4
        vJumpAdr           := (others => '0'); -- calculated jump address from instruction
        vDataMemReadData   := (others => '0'); -- data memory read data
        vDataMemWriteData  := (others => '0'); -- data memory write data
        vDataMemByteEnable := (others => '0'); -- data memory byte enable
        vCsrAddrMapped     := 255;      -- csr address mapped into linear space
        vCsrReadData       := (others => '0'); -- csr register read data
        vALUValues         := cALUValuesDefault; -- ALU values
        vALUOp             := ALUOpAdd;
        vRegWritedataSrc   := cRegWritedataALUSrc;
        vRegFileWriteEN    := '0';
        vRegWriteData      := (others => '0');
        vPCInc             := '0';

        -------------------------------------------------------------------------------
        -- Control Unit
        -------------------------------------------------------------------------------
        vALUValues.aluCalc := '0';
        NxR.memRead        <= '0';
        NxR.memWrite       <= '0';
        NxR.csrRead        <= '0';
        NxR.csrWriteMode   <= cModeNoWrite;

        if R.ctrlState = Fetch then
            NxR.ctrlState <= ReadReg;

        elsif R.ctrlState = ReadReg then
            case R.curInst(aOPCodeRange) is
                when cOpRType | cOpIArith =>
                    NxR.ctrlState <= CalculateALUOp;

                when cOpILoad =>
                    NxR.ctrlState <= CalculateLoad;

                when cOpSType =>
                    NxR.ctrlState <= CalculateStore;

                when cOpJType | cOpIJumpReg =>
                    NxR.ctrlState <= CalculateJump;

                when cOpBType =>
                    NxR.ctrlState <= CalculateBranch;

                when cOpLUI | cOpAUIPC =>
                    NxR.ctrlState <= CalculateUpperimmediate;

                when cOpFence =>        -- implemented as NOP
                    NxR.ctrlState <= Wait0;

                when cOpSys =>
                    case R.curInst(aFunct3Range) is
                        when cSysEnv =>
                            NxR.ctrlState <= EnterTrap;
                        when others =>
                            NxR.ctrlState <= CalculateSys;
                    end case;

                when others =>
                    null;               -- not implemented

            end case;

        -- U-Type Load Immediate
        elsif R.ctrlState = CalculateUpperimmediate then
            NxR.ctrlState      <= WriteReg;
            vRegFileWriteEN    := '1';
            vALUOp             := ALUOpAdd;
            vPCInc             := '1';
            if R.curInst(aOPCodeRange) = cOpLUI then
                vAluSrc1 := cALUSrc1Zero;
            else
                vAluSrc1 := cALUSrc1PC;
            end if;
            vAluSrc2           := cALUSrc2ImmGen;
            vALUValues.aluCalc := '1';

        -- J-Type Jump Instruction
        elsif R.ctrlState = CalculateJump then
            NxR.ctrlState      <= WriteReg;
            vRegFileWriteEN    := '1';
            vPCInc             := '1';
            -- alu config only needed for Jump Reg
            vALUOp             := ALUOpAdd;
            vAluSrc1           := cALUSrc1RegFile;
            vAluSrc2           := cALUSrc2ImmGen;
            vALUValues.aluCalc := '1';
            vRegWritedataSrc   := cRegWritedataPCPlus4Src;

        -- B-Type Conditional Branch
        elsif R.ctrlState = CalculateBranch then
            NxR.ctrlState      <= CheckJump;
            vALUOp             := ALUOpSub;
            vAluSrc1           := cALUSrc1RegFile;
            vAluSrc2           := cALUSrc2RegFile;
            vALUValues.aluCalc := '1';

        -- I-Type Load Instruction
        elsif R.ctrlState = CalculateLoad then
            NxR.memRead        <= '1';
            NxR.ctrlState      <= DataAccess0;
            vALUOp             := ALUOpAdd;
            vAluSrc1           := cALUSrc1RegFile;
            vAluSrc2           := cALUSrc2ImmGen;
            vALUValues.aluCalc := '1';

        -- S-Type Store Instruction
        elsif R.ctrlState = CalculateStore then
            NxR.memWrite       <= '1';
            NxR.ctrlState      <= DataAccess0;
            vALUOp             := ALUOpAdd;
            vAluSrc1           := cALUSrc1RegFile;
            vAluSrc2           := cALUSrc2ImmGen;
            vALUValues.aluCalc := '1';

        -- R-Type or I-Type Register Instruction
        elsif R.ctrlState = CalculateALUOp then
            NxR.ctrlState      <= WriteReg;
            vPCInc             := '1';
            vRegFileWriteEN    := '1';
            vALUValues.aluCalc := '1';

            vAluSrc1 := cALUSrc1RegFile;
            -- Immediate or Register Instruction
            if R.curInst(aOPCodeRange) = cOpRType then
                vAluSrc2 := cALUSrc2RegFile;
            else
                vAluSrc2 := cALUSrc2ImmGen;
            end if;

            -- ALU OpCode
            case R.curInst(aFunct3Range) is
                when cFunct3addsub =>   -- add/sub
                    if R.curInst(aOPCodeRange) = cOpRType and R.curInst(cFunct7OtherInstrPos) = '1' then
                        vALUOp := ALUOpSub;
                    else
                        vALUOp := ALUOpAdd;
                    end if;
                when cFunct3sll  => vALUOp := ALUOpSLL; -- shift left logical
                when cFunct3slt  => vALUOp := ALUOpSLT; -- signed less than
                when cFunct3sltu => vALUOp := ALUOpSLTU; -- unsigned less than
                when cFunct3xor  => vALUOp := ALUOpXor; -- xor
                when cFunct3sr =>       -- shift rigth logical/arithmetical                            
                    if R.curInst(cFunct7OtherInstrPos) = '0' then
                        vALUOp := ALUOpSRL;
                    else
                        vALUOp := ALUOpSRA;
                    end if;
                when cFunct3or   => vALUOp := ALUOpOr; -- or
                when cFunct3and  => vALUOp := ALUOpAnd; -- and
                when others      => null;
            end case;

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
            NxR.ctrlState <= DataAccess0;

        elsif R.ctrlState = DataAccess0 then
            vPCInc := '1';
            case R.curInst(aOPCodeRange) is
                when cOpILoad =>
                    NxR.ctrlState <= DataAccess1;
                when cOpSType =>
                    NxR.ctrlState <= Wait1;
                when cOpSys =>
                    vRegWritedataSrc := cRegWritedataCSRSrc;
                    vRegFileWriteEN  := '1';
                    NxR.ctrlState    <= WriteReg;
                when others =>
                    null;
            end case;

        elsif R.ctrlState = DataAccess1 then
            NxR.ctrlState    <= WriteReg;
            vRegFileWriteEN  := '1';
            vRegWritedataSrc := cRegWritedataMemRdSrc;

        elsif R.ctrlState = CheckJump then

            NxR.ctrlState <= Wait0;

            case R.curInst(aFunct3Range) is
                when cCondEq =>
                    if R.statusReg(cStatusZeroBit) = '1' then
                        NxR.ctrlState <= PerformJump;
                    end if;
                when cCondNe =>
                    if R.statusReg(cStatusZeroBit) = '0' then
                        NxR.ctrlState <= PerformJump;
                    end if;
                when cCondLt =>
                    if R.statusReg(cStatusNegBit) = '1' then
                        NxR.ctrlState <= PerformJump;
                    end if;
                when cCondGe =>
                    if R.statusReg(cStatusNegBit) = '0' then
                        NxR.ctrlState <= PerformJump;
                    end if;
                when cCondLtu =>
                    if R.statusReg(cStatusCarryBit) = '1' then
                        NxR.ctrlState <= PerformJump;
                    end if;
                when cCondGeu =>
                    if R.statusReg(cStatusCarryBit) = '0' then
                        NxR.ctrlState <= PerformJump;
                    end if;
                when others =>
                    null;
            end case;

        elsif R.ctrlState = PerformJump then
            vRegWritedataSrc := cRegWritedataPCPlus4Src;
            NxR.ctrlState    <= Wait1;
            vPCInc           := '1';

        elsif R.ctrlState = WriteReg then
            NxR.ctrlState <= Fetch;

        elsif R.ctrlState = Wait0 then
            vPCInc        := '1';
            NxR.ctrlState <= Wait1;

        elsif R.ctrlState = Wait1 then
            NxR.ctrlState <= Fetch;

        elsif R.ctrlState = EnterTrap then
            vPCInc := '1';
            if R.curInst(31 downto 20) = cMTrapRet then
                NxR.ctrlState <= Wait1;
            else
                NxR.ctrlState <= Trap;
            end if;

        elsif R.ctrlState = Trap then
            NxR.ctrlState <= Trap;

        else
            null;
        end if;

        -------------------------------------------------------------------------------
        -- Register File - Read Stage
        -------------------------------------------------------------------------------
        -- read registers from regfile
        vRegReadData1 := RegFile(to_integer(unsigned(R.curInst(aRs1AddrRange))));
        vRegReadData2 := RegFile(to_integer(unsigned(R.curInst(aRs2AddrRange))));

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

        -- MUX ALUSrc2
        case vAluSrc2 is
            when cALUSrc2RegFile => vALUValues.aluData2 := vRegReadData2;
            when cALUSrc2ImmGen  => vALUValues.aluData2 := vImm;
            when cALUSrc2Const4  => vALUValues.aluData2 := std_ulogic_vector(to_unsigned(4, vALUValues.aluData2'length));
            when others          => null;
        end case;

        -- Mux ALUSrc1
        case vAluSrc1 is
            when cALUSrc1RegFile => vALUValues.aluData1 := vRegReadData1;
            when cALUSrc1Zero    => vALUValues.aluData1 := (others => '0');
            when cALUSrc1PC      => vALUValues.aluData1 := R.curPC;
            when others          => null;
        end case;

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

        if (R.ctrlState = Fetch) then
            NxR.curInst <= std_ulogic_vector(i_readdata_remapped);
        end if;

        -------------------------------------------------------------------------------
        -- ALU
        -------------------------------------------------------------------------------
        -- add and substract from alu needed for other operations
        if (vALUOp = ALUOpAdd) then
            vALUValues.addsubRes := std_ulogic_vector(unsigned('0' & vALUValues.aluData1) + unsigned('0' & vALUValues.aluData2));
            if ((vALUValues.aluData1(vALUValues.aluData1'high) = vALUValues.aluData2(vALUValues.aluData2'high)) and (vALUValues.aluData1(vALUValues.aluData1'high) /= vALUValues.addsubRes(vALUValues.addsubRes'high - 1))) then
                vALUValues.addsubCarry := '1';
            end if;
        else
            vALUValues.addsubRes := std_ulogic_vector(unsigned('0' & vALUValues.aluData1) - unsigned('0' & vALUValues.aluData2));
            if ((vALUValues.aluData1(vALUValues.aluData1'high) /= vALUValues.aluData2(vALUValues.aluData2'high)) and (vALUValues.aluData1(vALUValues.aluData1'high) /= vALUValues.addsubRes(vALUValues.addsubRes'high - 1))) then
                vALUValues.addsubCarry := '1';
            end if;
        end if;

        -- calculate shift amount
        vALUValues.shiftAmount := to_integer(unsigned(vALUValues.aluData2(aALUShiftRange)));

        case vALUOp is
            when ALUOpAdd | ALUOpSub =>
                vALUValues.aluRawRes := vALUValues.addsubRes;
            when ALUOpSLT =>
                vALUValues.aluRawRes    := (others => '0');
                vALUValues.aluRawRes(0) := (vALUValues.addsubRes(vALUValues.addsubRes'high - 1) or vALUValues.addsubCarry) and not (not vALUValues.aluData1(vALUValues.aluData1'high) and vALUValues.aluData2(vALUValues.aluData2'high));

            when ALUOpSLTU =>
                vALUValues.aluRawRes := (0 => vALUValues.addsubRes(vALUValues.addsubRes'high), others => '0');
            when ALUOpAnd =>
                vALUValues.aluRawRes := '0' & (vALUValues.aluData1 AND vALUValues.aluData2);
            when ALUOpOr =>
                vALUValues.aluRawRes := '0' & (vALUValues.aluData1 OR vALUValues.aluData2);
            when ALUOpXor =>
                vALUValues.aluRawRes := '0' & (vALUValues.aluData1 XOR vALUValues.aluData2);
            when ALUOpSLL =>
                vALUValues.aluRawRes := std_ulogic_vector(
                    shift_left(unsigned('0' & vALUValues.aluData1), vALUValues.shiftAmount));
            when ALUOpSRL | ALUOpSRA =>
                vALUValues.srValue   := '0';
                if (vALUOp = ALUOpSRA) then
                    vALUValues.srValue := vALUValues.aluData1(vALUValues.aluData1'high);
                end if;
                vALUValues.aluRawRes := std_ulogic_vector(
                    shift_right(signed(vALUValues.srValue & vALUValues.aluData1), vALUValues.shiftAmount));
            when ALUOpNOP =>
                vALUValues.aluRawRes := (others => '-');
        end case;

        -- Remove Carry Bit and Store New Value
        if (vALUValues.aluCalc = '1') then
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
        -- Register File - Write Stage
        -------------------------------------------------------------------------------
        -- MUX RegWriteData
        case vRegWritedataSrc is
            when cRegWritedataPCPlus4Src => vRegWriteData := vPCPlus4;
            when cRegWritedataMemRdSrc   => vRegWriteData := vDataMemReadData;
            when cRegWritedataCSRSrc     => vRegWriteData := vCsrReadData;
            when cRegWritedataALUSrc     => vRegWriteData := vALUValues.aluRes;
            when others                  => null;
        end case;

        if vRegFileWriteEN = '1' and R.curInst(aRdAddrRange) /= "00000" then
            NxRegFile(to_integer(unsigned(R.curInst(aRdAddrRange)))) <= vRegWriteData;
        end if;

        -------------------------------------------------------------------------------
        -- Multiplexer
        -------------------------------------------------------------------------------   
        -- Select next PC
        if vRegWritedataSrc = cRegWritedataPCPlus4Src then
            NxR.curPC <= vJumpAdr;
        elsif vPCInc = '1' then
            NxR.curPC <= vPCPlus4;
        else
            NxR.curPC <= R.curPC;
        end if;

        -- Mux CsrWriteData
        if R.curInst(14) = cCsrDataReg then
            NxR.csrWriteData <= vRegReadData1;
        else
            NxR.csrWriteData <= std_ulogic_vector(resize(unsigned(R.curInst(19 downto 15)), cBitWidth));
        end if;

    end process;

end architecture rtl;
