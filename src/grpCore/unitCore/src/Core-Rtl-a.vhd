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
        variable vReadInstr         : std_ulogic;
        variable vInstrAddrSrc      : aCtrlSignal;
        variable vReadMem           : std_ulogic;
        variable vWriteMem          : std_ulogic;
        variable vCSRRead           : std_ulogic;
        variable vCSRWriteMode      : aCtrl2Signal;
        variable vCSRWriteData      : aRegValue;

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
        vReadInstr         := '0';
        vInstrAddrSrc      := cInstrAddrPCSrc;
        vReadMem           := '0';
        vWriteMem          := '0';
        vCSRRead           := '0';
        vCSRWriteMode      := cModeNoWrite;
        vCSRWriteData      := (others => '0');

        -------------------------------------------------------------------------------
        -- Control Unit
        -------------------------------------------------------------------------------
        case R.ctrlState is
            when InitState =>
                NxR.ctrlState <= Fetch;
                vReadInstr    := '1';

            when Fetch =>
                NxR.ctrlState <= ReadReg;

            when ReadReg =>
                -- increment pc with alu
                vPCInc             := '1';
                vAluSrc1           := cALUSrc1PC;
                vAluSrc2           := cALUSrc2Const4;
                vALUOp             := ALUOpAdd;
                vALUValues.aluCalc := '1';

                -- decode instruction
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

                    when cOpFence =>    -- implemented as NOP
                        NxR.ctrlState <= Wait0;

                    when cOpSys =>
                        case R.curInst(aFunct3Range) is
                            when cSysEnv =>
                                NxR.ctrlState <= Trap;
                            when others =>
                                NxR.ctrlState <= CalculateSys;
                        end case;

                    when others =>
                        null;           -- not implemented
                end case;

            -- U-Type Load Immediate
            when CalculateUpperimmediate =>
                NxR.ctrlState      <= Fetch;
                vReadInstr         := '1';
                vRegFileWriteEN    := '1';
                vALUOp             := ALUOpAdd;
                if R.curInst(aOPCodeRange) = cOpLUI then
                    vAluSrc1 := cALUSrc1Zero;
                else
                    vAluSrc1 := cALUSrc1PrevPC;
                end if;
                vAluSrc2           := cALUSrc2ImmGen;
                vALUValues.aluCalc := '1';

            -- J-Type Jump Instruction
            when CalculateJump =>
                NxR.ctrlState   <= Fetch;
                vRegFileWriteEN := '1';
                vReadInstr      := '1';
                vInstrAddrSrc   := cInstrAddrALUSrc;

                -- alu config only needed for Jump Reg
                vALUOp             := ALUOpAdd;
                if (R.curInst(aOPCodeRange) = cOpIJumpReg) then
                    vAluSrc1 := cALUSrc1RegFile;
                else
                    vAluSrc1 := cALUSrc1PrevPC;
                end if;
                vAluSrc2           := cALUSrc2ImmGen;
                vALUValues.aluCalc := '1';
                vRegWritedataSrc   := cRegWritedataPCSrc;

            -- B-Type Conditional Branch
            when CalculateBranch =>
                NxR.ctrlState <= Wait0;

                -- configure ALU
                case R.curInst(aFunct3Range) is
                    when cCondEq | cCondNe   => vALUOp := ALUOpSub;
                    when cCondLt | cCondGe   => vALUOp := ALUOpSLT;
                    when cCondLtu | cCondGeu => vALUOp := ALUOpSLTU;
                    when others              => null;
                end case;
                vAluSrc1           := cALUSrc1RegFile;
                vAluSrc2           := cALUSrc2RegFile;
                vALUValues.aluCalc := '1';

            -- I-Type Load Instruction
            when CalculateLoad =>
                NxR.ctrlState      <= WaitLoad;
                vALUOp             := ALUOpAdd;
                vAluSrc1           := cALUSrc1RegFile;
                vAluSrc2           := cALUSrc2ImmGen;
                vALUValues.aluCalc := '1';
                vReadMem           := '1';

            -- Read data from data mem
            when WaitLoad =>
                NxR.ctrlState    <= Fetch;
                vReadInstr       := '1';
                vRegFileWriteEN  := '1';
                vRegWritedataSrc := cRegWritedataMemRdSrc;

            -- S-Type Store Instruction
            when CalculateStore =>
                NxR.ctrlState      <= Wait0;
                vWriteMem          := '1';
                vALUOp             := ALUOpAdd;
                vAluSrc1           := cALUSrc1RegFile;
                vAluSrc2           := cALUSrc2ImmGen;
                vALUValues.aluCalc := '1';

            -- R-Type or I-Type Register Instruction
            when CalculateALUOp =>
                NxR.ctrlState      <= Fetch;
                vRegFileWriteEN    := '1';
                vALUValues.aluCalc := '1';
                vReadInstr         := '1';

                vAluSrc1 := cALUSrc1RegFile;
                -- Immediate or Register Instruction
                if R.curInst(aOPCodeRange) = cOpRType then
                    vAluSrc2 := cALUSrc2RegFile;
                else
                    vAluSrc2 := cALUSrc2ImmGen;
                end if;

                -- ALU OpCode
                case R.curInst(aFunct3Range) is
                    when cFunct3addsub => -- add/sub
                        if R.curInst(aOPCodeRange) = cOpRType and R.curInst(cFunct7OtherInstrPos) = '1' then
                            vALUOp := ALUOpSub;
                        else
                            vALUOp := ALUOpAdd;
                        end if;
                    when cFunct3sll  => vALUOp := ALUOpSLL; -- shift left logical
                    when cFunct3slt  => vALUOp := ALUOpSLT; -- signed less than
                    when cFunct3sltu => vALUOp := ALUOpSLTU; -- unsigned less than
                    when cFunct3xor  => vALUOp := ALUOpXor; -- xor
                    when cFunct3sr =>   -- shift rigth logical/arithmetical                            
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
            when CalculateSys =>
                case R.curInst(aFunct3Range) is
                    when cSysRW | cSysRWI =>
                        if R.curInst(11 downto 7) /= "00000" then
                            vCSRRead := '1';
                        end if;
                    when cSysRS | cSysRSI =>
                        vCSRRead := '1';
                        if R.curInst(19 downto 15) /= "00000" then
                            vCSRWriteMode := cModeSet;
                        end if;
                    when cSysRC | cSysRCI =>
                        vCSRRead := '1';
                        if R.curInst(19 downto 15) /= "00000" then
                            vCSRWriteMode := cModeClear;
                        end if;
                    when others => null;
                end case;
                vRegWritedataSrc := cRegWritedataCSRSrc;
                vRegFileWriteEN  := '1';
                vReadInstr       := '1';
                NxR.ctrlState    <= Fetch;

            when PerformBranch =>
                vRegWritedataSrc := cRegWritedataPCSrc;
                vReadInstr       := '1';
                vInstrAddrSrc    := cInstrAddrALUSrc;
                NxR.ctrlState    <= Fetch;

                -- alu config for new pc
                vALUOp             := ALUOpAdd;
                vAluSrc1           := cALUSrc1PrevPC;
                vAluSrc2           := cALUSrc2ImmGen;
                vALUValues.aluCalc := '1';

            when Wait0 =>
                NxR.ctrlState <= Fetch;
                vReadInstr    := '1';

            when Trap =>
                if R.curInst(31 downto 20) = cMTrapRet then
                    NxR.ctrlState <= Fetch;
                    vReadInstr    := '1';
                else
                    NxR.ctrlState <= Trap;
                end if;

            when others => null;
        end case;

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
            when cALUSrc1PrevPC  => vALUValues.aluData1 := std_ulogic_vector(unsigned(R.curPC) - 4);
            when cALUSrc1PC      => vALUValues.aluData1 := R.curPC;
            when others          => null;
        end case;

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

        -- Set Status Registers
        vALUValues.zero     := nor_reduce(vALUValues.aluRawRes(vALUValues.aluRawRes'high - 1 downto 0));
        vALUValues.negative := vALUValues.aluRawRes(vALUValues.aluRawRes'high - 1);
        vALUValues.carry    := vALUValues.aluRawRes(vALUValues.aluRawRes'high);

        -- Remove Carry Bit and Store New Value
        if (vALUValues.aluCalc = '1') then
            vALUValues.aluRes := vALUValues.aluRawRes(vALUValues.aluRes'range);
            NxR.aluRes        <= vALUValues.aluRes;
        else
            vALUValues.aluRes := R.aluRes;
        end if;

        -------------------------------------------------------------------------------
        -- BranchCheck Unit
        -------------------------------------------------------------------------------
        if (R.ctrlState = CalculateBranch) then
            -- check if condition is met
            case R.curInst(aFunct3Range) is
                when cCondEq | cCondGe | cCondGeu =>
                    if vALUValues.zero = '1' then
                        NxR.ctrlState <= PerformBranch;
                    end if;
                when cCondNe | cCondLt | cCondLtu =>
                    if vALUValues.zero = '0' then
                        NxR.ctrlState <= PerformBranch;
                    end if;
                when others =>
                    null;
            end case;
        end if;

        -------------------------------------------------------------------------------
        -- Instruction Memory
        -------------------------------------------------------------------------------
        avm_i_read <= std_logic(vReadInstr);

        case vInstrAddrSrc is
            when cInstrAddrPCSrc  => avm_i_address <= std_logic_vector(R.curPC);
            when cInstrAddrALUSrc => avm_i_address <= std_logic_vector(vALUValues.aluRes);
            when others           => null;
        end case;

        if (R.ctrlState = Fetch) then
            NxR.curInst <= std_ulogic_vector(i_readdata_remapped);
        end if;

        -------------------------------------------------------------------------------
        -- CSR Unit
        -------------------------------------------------------------------------------
        -- Mux CsrWriteData
        if R.curInst(14) = cCsrDataReg then
            --NxR.csrWriteData <= vRegReadData1;
            vCSRWriteData := vRegReadData1;
        else
            --NxR.csrWriteData <= std_ulogic_vector(resize(unsigned(R.curInst(19 downto 15)), cBitWidth));
            vCSRWriteData := std_ulogic_vector(resize(unsigned(R.curInst(19 downto 15)), cBitWidth));
        end if;

        vCsrAddrMapped := mapCsrAddr(R.curInst(31 downto 20));

        if vCSRRead = '1' then
            if mapCsrAddrValid(vCsrAddrMapped) then
                vCsrReadData := R.csrReg(vCsrAddrMapped);
            end if;
        end if;

        if vCSRWriteMode /= cModeNoWrite then
            if mapCsrAddrValid(vCsrAddrMapped) then
                case vCSRWriteMode is
                    when cModeWrite =>
                        NxR.csrReg(vCsrAddrMapped) <= vCSRWriteData;
                    when cModeSet =>
                        NxR.csrReg(vCsrAddrMapped) <= vCSRWriteData or vCsrReadData;
                    when cModeClear =>
                        NxR.csrReg(vCsrAddrMapped) <= (not vCSRWriteData) and vCsrReadData;
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
        avm_d_write          <= std_logic(vWriteMem);
        d_writedata_remapped <= (vDataMemWriteData);
        avm_d_read           <= std_logic(vReadMem);

        -------------------------------------------------------------------------------
        -- Register File - Write Stage
        -------------------------------------------------------------------------------
        -- MUX RegWriteData
        case vRegWritedataSrc is
            when cRegWritedataPCSrc    => vRegWriteData := R.curPC;
            when cRegWritedataMemRdSrc => vRegWriteData := vDataMemReadData;
            when cRegWritedataCSRSrc   => vRegWriteData := vCsrReadData;
            when cRegWritedataALUSrc   => vRegWriteData := vALUValues.aluRes;
            when others                => null;
        end case;

        if vRegFileWriteEN = '1' and R.curInst(aRdAddrRange) /= "00000" then
            NxRegFile(to_integer(unsigned(R.curInst(aRdAddrRange)))) <= vRegWriteData;
        end if;

        -------------------------------------------------------------------------------
        -- Multiplexer
        -------------------------------------------------------------------------------   
        -- Select next PC
        if vRegWritedataSrc = cRegWritedataPCSrc or vPCInc = '1' then
            NxR.curPC <= vALUValues.aluRes;
        else
            NxR.curPC <= R.curPC;
        end if;

    end process;

end architecture rtl;
