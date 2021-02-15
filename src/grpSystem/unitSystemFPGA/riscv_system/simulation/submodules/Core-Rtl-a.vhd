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
    constant cInitValRegFile : aRegFile := (others => (others => '0'));
    signal RegFile           : aRegFile := cInitValRegFile;
    attribute ramstyle       : string;
    attribute ramstyle of RegFile : signal is "MLAB";

    -- RAM data
    signal RAM, NxRAM         : aRAM;
    signal RAMCtrl, NxRAMCtrl : aRAMCtrl := cRAMCtrlDefault;

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

    RAMData : process(csi_clk) is
    begin
        if (rising_edge(csi_clk)) then
            -- write
            RegFile(NxRAMCtrl.regfileWrAddr) <= NxRAMCtrl.regfileWrData;
            
            -- read
             RAMCtrl.rs1Data <= RegFile(NxRAMCtrl.regfileRs1Addr);
             RAMCtrl.rs2Data <= RegFile(NxRAMCtrl.regfileRs2Addr);
        end if;
    end process;

    Registers : process(csi_clk, rsi_reset_n)
    begin
        if (rsi_reset_n = not ('1')) then
            R <= cInitValRegSet;
        elsif (rising_edge(csi_clk)) then
            R <= NxR;
        end if;
    end process;

    Comb : process(R, RAMCtrl, i_readdata_remapped, d_readdata_remapped)
        variable vRegfile  : aRegfileValues;
        variable vImm      : aImm;
        variable vALU      : aALUValues;
        variable vCSR      : aCSRValues;
        variable vDataMem  : aDataMemValues;
        variable vPCEN     : std_ulogic;
        variable vInstrMem : aInstrMemValues;

    begin
        -- default signal values
        NxR       <= R;
        NxRAMCtrl <= RAMCtrl;

        -- default variable values
        vRegfile  := cRegfileValuesDefault;
        vImm      := (others => '0');
        vALU      := cALUValuesDefault;
        vCSR      := cCSRValuesDefault;
        vDataMem  := cDataMemDefault;
        vPCEN     := '0';
        vInstrMem := cInstrMemDefault;

        -------------------------------------------------------------------------------
        -- Control Unit
        -------------------------------------------------------------------------------
        case R.ctrlState is
            when InitState =>
                vInstrMem.read := '1';
                NxR.ctrlState  <= Fetch;

            when Fetch =>
                -- increment pc with alu
                vPCEN         := '1';
                vALU.src1     := cALUSrc1PC;
                vALU.src2     := cALUSrc2Const4;
                vALU.op       := ALUOpAdd;
                vALU.aluCalc  := '1';
                NxR.ctrlState <= ReadReg;

            when ReadReg =>
                -- decode instruction
                case R.curInst(aOPCodeRange) is
                    when cOpLUI | cOpAUIPC =>
                        NxR.ctrlState <= CalculateUpperimmediate;
                    when cOpJType | cOpIJumpReg =>
                        NxR.ctrlState <= CalculateJump;
                    when cOpBType =>
                        NxR.ctrlState <= CalculateBranch;
                    when cOpILoad =>
                        NxR.ctrlState <= CalculateLoad;
                    when cOpSType =>
                        NxR.ctrlState <= CalculateStore;
                    when cOpRType | cOpIArith =>
                        NxR.ctrlState <= CalculateALUOp;
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
                NxR.ctrlState        <= Fetch;
                vInstrMem.read       := '1';
                vRegfile.writeEnable := '1';
                vALU.op              := ALUOpAdd;
                if R.curInst(aOPCodeRange) = cOpLUI then
                    vALU.src1 := cALUSrc1Zero;
                else
                    vALU.src1 := cALUSrc1PrevPC;
                end if;
                vALU.src2            := cALUSrc2ImmGen;
                vALU.aluCalc         := '1';

            -- J-Type Jump Instruction
            when CalculateJump =>
                NxR.ctrlState        <= Fetch;
                vRegfile.writeEnable := '1';
                vInstrMem.read       := '1';
                vInstrMem.src        := cInstrAddrALUSrc;

                -- alu config only needed for Jump Reg
                vALU.op               := ALUOpAdd;
                if (R.curInst(aOPCodeRange) = cOpIJumpReg) then
                    vALU.src1 := cALUSrc1RegFile;
                else
                    vALU.src1 := cALUSrc1PrevPC;
                end if;
                vALU.src2             := cALUSrc2ImmGen;
                vALU.aluCalc          := '1';
                vPCEN                 := '1';
                vRegfile.writeDataSrc := cRegWritedataPCSrc;

            -- B-Type Conditional Branch
            when CalculateBranch =>
                NxR.ctrlState <= Wait0;

                -- configure ALU
                case R.curInst(aFunct3Range) is
                    when cCondEq | cCondNe   => vALU.op := ALUOpSub;
                    when cCondLt | cCondGe   => vALU.op := ALUOpSLT;
                    when cCondLtu | cCondGeu => vALU.op := ALUOpSLTU;
                    when others              => null;
                end case;
                vALU.src1    := cALUSrc1RegFile;
                vALU.src2    := cALUSrc2RegFile;
                vALU.aluCalc := '1';

            when PerformBranch =>
                NxR.ctrlState         <= Fetch;
                vPCEN                 := '1';
                vRegfile.writeDataSrc := cRegWritedataPCSrc;
                vInstrMem.read        := '1';
                vInstrMem.src         := cInstrAddrALUSrc;
                -- alu config for new pc
                vALU.op               := ALUOpAdd;
                vALU.src1             := cALUSrc1PrevPC;
                vALU.src2             := cALUSrc2ImmGen;
                vALU.aluCalc          := '1';

            -- I-Type Load Instruction
            when CalculateLoad =>
                NxR.ctrlState <= WaitLoad;
                vALU.op       := ALUOpAdd;
                vALU.src1     := cALUSrc1RegFile;
                vALU.src2     := cALUSrc2ImmGen;
                vALU.aluCalc  := '1';
                vDataMem.read := '1';

            -- Read data from data mem
            when WaitLoad =>
                NxR.ctrlState         <= Fetch;
                vInstrMem.read        := '1';
                vRegfile.writeEnable  := '1';
                vRegfile.writeDataSrc := cRegWritedataMemRdSrc;

            -- S-Type Store Instruction
            when CalculateStore =>
                NxR.ctrlState  <= Wait0;
                vDataMem.write := '1';
                vALU.op        := ALUOpAdd;
                vALU.src1      := cALUSrc1RegFile;
                vALU.src2      := cALUSrc2ImmGen;
                vALU.aluCalc   := '1';

            -- R-Type or I-Type Register Instruction
            when CalculateALUOp =>
                NxR.ctrlState        <= Fetch;
                vRegfile.writeEnable := '1';
                vALU.aluCalc         := '1';
                vInstrMem.read       := '1';

                vALU.src1 := cALUSrc1RegFile;
                -- Immediate or Register Instruction
                if R.curInst(aOPCodeRange) = cOpRType then
                    vALU.src2 := cALUSrc2RegFile;
                else
                    vALU.src2 := cALUSrc2ImmGen;
                end if;

                -- ALU OpCode
                case R.curInst(aFunct3Range) is
                    when cFunct3addsub => -- add/sub
                        if R.curInst(aOPCodeRange) = cOpRType and R.curInst(cFunct7OtherInstrPos) = '1' then
                            vALU.op := ALUOpSub;
                        else
                            vALU.op := ALUOpAdd;
                        end if;
                    when cFunct3sll  => vALU.op := ALUOpSLL; -- shift left logical
                    when cFunct3slt  => vALU.op := ALUOpSLT; -- signed less than
                    when cFunct3sltu => vALU.op := ALUOpSLTU; -- unsigned less than
                    when cFunct3xor  => vALU.op := ALUOpXor; -- xor
                    when cFunct3sr =>   -- shift rigth logical/arithmetical                            
                        if R.curInst(cFunct7OtherInstrPos) = '0' then
                            vALU.op := ALUOpSRL;
                        else
                            vALU.op := ALUOpSRA;
                        end if;
                    when cFunct3or   => vALU.op := ALUOpOr; -- or
                    when cFunct3and  => vALU.op := ALUOpAnd; -- and
                    when others      => null;
                end case;

            -- CSR Instruction
            when CalculateSys =>
                case R.curInst(aFunct3Range) is
                    when cSysRW | cSysRWI =>
                        if R.curInst(11 downto 7) /= "00000" then
                            vCSR.read := '1';
                        end if;
                    when cSysRS | cSysRSI =>
                        vCSR.read := '1';
                        if R.curInst(19 downto 15) /= "00000" then
                            vCSR.writeMode := cModeSet;
                        end if;
                    when cSysRC | cSysRCI =>
                        vCSR.read := '1';
                        if R.curInst(19 downto 15) /= "00000" then
                            vCSR.writeMode := cModeClear;
                        end if;
                    when others => null;
                end case;
                vRegfile.writeDataSrc := cRegWritedataCSRSrc;
                vRegfile.writeEnable  := '1';
                vInstrMem.read        := '1';
                NxR.ctrlState         <= Fetch;

            when Wait0 =>
                NxR.ctrlState  <= Fetch;
                vInstrMem.read := '1';

            when Trap =>
                if R.curInst(31 downto 20) = cMTrapRet then
                    NxR.ctrlState  <= Fetch;
                    vInstrMem.read := '1';
                else
                    NxR.ctrlState <= Trap;
                end if;

            when others => null;
        end case;

        -------------------------------------------------------------------------------
        -- Register File - Read Stage
        -------------------------------------------------------------------------------
        -- read registers from regfile
        --vRegfile.readData1 := RegFile(to_integer(unsigned(R.curInst(aRs1AddrRange))));
        --vRegfile.readData2 := RegFile(to_integer(unsigned(R.curInst(aRs2AddrRange))));

        if (R.ctrlState = ReadReg) then
            NxRAMCtrl.regfileRs1Addr <= to_integer(unsigned(R.curInst(aRs1AddrRange)));
            NxRAMCtrl.regfileRs2Addr <= to_integer(unsigned(R.curInst(aRs2AddrRange)));
        end if;

        vRegfile.readData1 := RAMCtrl.rs1Data;
        vRegfile.readData2 := RAMCtrl.rs2Data;

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
            when others => null;
        end case;

        -- MUX ALUSrc2
        case vALU.src2 is
            when cALUSrc2RegFile => vALU.aluData2 := vRegfile.readData2;
            when cALUSrc2ImmGen  => vALU.aluData2 := vImm;
            when cALUSrc2Const4  => vALU.aluData2 := std_ulogic_vector(to_unsigned(4, vALU.aluData2'length));
            when others          => null;
        end case;

        -- Mux ALUSrc1
        case vALU.src1 is
            when cALUSrc1RegFile => vALU.aluData1 := vRegfile.readData1;
            when cALUSrc1Zero    => vALU.aluData1 := (others => '0');
            when cALUSrc1PrevPC  => vALU.aluData1 := std_ulogic_vector(unsigned(R.curPC) - 4);
            when cALUSrc1PC      => vALU.aluData1 := R.curPC;
            when others          => null;
        end case;

        -------------------------------------------------------------------------------
        -- ALU
        -------------------------------------------------------------------------------
        -- add and substract from alu needed for other operations
        if (vALU.op = ALUOpAdd) then
            vALU.addsubRes := std_ulogic_vector(unsigned('0' & vALU.aluData1) + unsigned('0' & vALU.aluData2));
            if ((vALU.aluData1(vALU.aluData1'high) = vALU.aluData2(vALU.aluData2'high)) and (vALU.aluData1(vALU.aluData1'high) /= vALU.addsubRes(vALU.addsubRes'high - 1))) then
                vALU.addsubCarry := '1';
            end if;
        else
            vALU.addsubRes := std_ulogic_vector(unsigned('0' & vALU.aluData1) - unsigned('0' & vALU.aluData2));
            if ((vALU.aluData1(vALU.aluData1'high) /= vALU.aluData2(vALU.aluData2'high)) and (vALU.aluData1(vALU.aluData1'high) /= vALU.addsubRes(vALU.addsubRes'high - 1))) then
                vALU.addsubCarry := '1';
            end if;
        end if;

        -- calculate shift amount
        vALU.shiftAmount := to_integer(unsigned(vALU.aluData2(aALUShiftRange)));

        case vALU.op is
            when ALUOpAdd | ALUOpSub =>
                vALU.aluRawRes := vALU.addsubRes;
            when ALUOpSLT =>
                vALU.aluRawRes    := (others => '0');
                vALU.aluRawRes(0) := (vALU.addsubRes(vALU.addsubRes'high - 1) or vALU.addsubCarry) and not (not vALU.aluData1(vALU.aluData1'high) and vALU.aluData2(vALU.aluData2'high));
            when ALUOpSLTU =>
                vALU.aluRawRes := (0 => vALU.addsubRes(vALU.addsubRes'high), others => '0');
            when ALUOpAnd =>
                vALU.aluRawRes := '0' & (vALU.aluData1 AND vALU.aluData2);
            when ALUOpOr =>
                vALU.aluRawRes := '0' & (vALU.aluData1 OR vALU.aluData2);
            when ALUOpXor =>
                vALU.aluRawRes := '0' & (vALU.aluData1 XOR vALU.aluData2);
            when ALUOpSLL =>
                vALU.aluRawRes := std_ulogic_vector(
                    shift_left(unsigned('0' & vALU.aluData1), vALU.shiftAmount));
            when ALUOpSRL | ALUOpSRA =>
                vALU.srValue   := '0';
                if (vALU.op = ALUOpSRA) then
                    vALU.srValue := vALU.aluData1(vALU.aluData1'high);
                end if;
                vALU.aluRawRes := std_ulogic_vector(
                    shift_right(signed(vALU.srValue & vALU.aluData1), vALU.shiftAmount));
            when ALUOpNOP =>
                vALU.aluRawRes := (others => '-');
        end case;

        -- Set Status Registers
        vALU.zero     := nor_reduce(vALU.aluRawRes(vALU.aluRawRes'high - 1 downto 0));
        vALU.negative := vALU.aluRawRes(vALU.aluRawRes'high - 1);
        vALU.carry    := vALU.aluRawRes(vALU.aluRawRes'high);

        -- Remove Carry Bit and Store New Value
        if (vALU.aluCalc = '1') then
            vALU.aluRes := vALU.aluRawRes(vALU.aluRes'range);
            NxR.aluRes  <= vALU.aluRes;
        else
            vALU.aluRes := R.aluRes;
        end if;

        -------------------------------------------------------------------------------
        -- BranchCheck Unit
        -------------------------------------------------------------------------------
        if (R.ctrlState = CalculateBranch) then
            -- check if condition is met
            case R.curInst(aFunct3Range) is
                when cCondEq | cCondGe | cCondGeu =>
                    if vALU.zero = '1' then
                        NxR.ctrlState <= PerformBranch;
                    end if;
                when cCondNe | cCondLt | cCondLtu =>
                    if vALU.zero = '0' then
                        NxR.ctrlState <= PerformBranch;
                    end if;
                when others =>
                    null;
            end case;
        end if;

        -------------------------------------------------------------------------------
        -- Instruction Memory
        -------------------------------------------------------------------------------
        avm_i_read <= std_logic(vInstrMem.read);

        case vInstrMem.src is
            when cInstrAddrPCSrc  => avm_i_address <= std_logic_vector(R.curPC);
            when cInstrAddrALUSrc => avm_i_address <= std_logic_vector(vALU.aluRes);
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
            vCSR.writeData := vRegfile.readData1;
        else
            vCSR.writeData := std_ulogic_vector(resize(unsigned(R.curInst(19 downto 15)), cBitWidth));
        end if;

        vCSR.addrMapped := mapCsrAddr(R.curInst(31 downto 20));

        if vCSR.read = '1' then
            if mapCsrAddrValid(vCSR.addrMapped) then
                vCSR.readData := R.csrReg(vCSR.addrMapped);
            end if;
        end if;

        if vCSR.writeMode /= cModeNoWrite then
            if mapCsrAddrValid(vCSR.addrMapped) then
                case vCSR.writeMode is
                    when cModeWrite =>
                        NxR.csrReg(vCSR.addrMapped) <= vCSR.writeData;
                    when cModeSet =>
                        NxR.csrReg(vCSR.addrMapped) <= vCSR.writeData or vCSR.readData;
                    when cModeClear =>
                        NxR.csrReg(vCSR.addrMapped) <= (not vCSR.writeData) and vCSR.readData;
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
                vDataMem.readData                                  := std_ulogic_vector(
                    resize(signed(d_readdata_remapped(cByte - 1 downto 0)), cBitWidth));
                vDataMem.writeData(4 * cByte - 1 downto 3 * cByte) := vRegfile.readData2(cByte - 1 downto 0);
                vDataMem.byteenable                                := cEnableByte;

            when cMemHalfWord =>
                vDataMem.readData                                  := std_ulogic_vector(
                    resize(signed(d_readdata_remapped(2 * cByte - 1 downto 0)), cBitWidth));
                vDataMem.writeData(4 * cByte - 1 downto 2 * cByte) := vRegfile.readData2(2 * cByte - 1 downto 0);
                vDataMem.byteenable                                := cEnableHalfWord;

            when cMemWord =>
                vDataMem.readData   := std_ulogic_vector(
                    resize(signed(d_readdata_remapped(4 * cByte - 1 downto 0)), cBitWidth));
                vDataMem.writeData  := vRegfile.readData2;
                vDataMem.byteenable := cEnableWord;

            when cMemUnsignedByte =>
                vDataMem.readData                                  := std_ulogic_vector(resize(unsigned(
                    d_readdata_remapped(cByte - 1 downto 0)), cBitWidth));
                vDataMem.writeData(4 * cByte - 1 downto 3 * cByte) := vRegfile.readData2(cByte - 1 downto 0);
                vDataMem.byteenable                                := cEnableByte;

            when cMemUnsignedHalfWord =>
                vDataMem.readData                                  := std_ulogic_vector(resize(unsigned(
                    d_readdata_remapped(2 * cByte - 1 downto 0)), cBitWidth));
                vDataMem.writeData(4 * cByte - 1 downto 2 * cByte) := vRegfile.readData2(2 * cByte - 1 downto 0);
                vDataMem.byteenable                                := cEnableHalfWord;

            when others =>
                vDataMem.readData   := (others => '0');
                vDataMem.writeData  := (others => '0');
                vDataMem.byteenable := (others => '0');
        end case;

        avm_d_address        <= to_StdLogicVector(vALU.aluRes);
        avm_d_byteenable     <= to_StdLogicVector(vDataMem.byteenable);
        avm_d_write          <= std_logic(vDataMem.write);
        d_writedata_remapped <= vDataMem.writeData;
        avm_d_read           <= std_logic(vDataMem.read);

        -------------------------------------------------------------------------------
        -- Register File - Write Stage
        -------------------------------------------------------------------------------
        -- MUX RegWriteData
        case vRegfile.writeDataSrc is
            when cRegWritedataPCSrc    => vRegfile.writeData := R.curPC;
            when cRegWritedataMemRdSrc => vRegfile.writeData := vDataMem.readData;
            when cRegWritedataCSRSrc   => vRegfile.writeData := vCSR.readData;
            when cRegWritedataALUSrc   => vRegfile.writeData := vALU.aluRes;
            when others                => null;
        end case;

        if vRegfile.writeEnable = '1' and R.curInst(aRdAddrRange) /= "00000" then
            NxRAMCtrl.regfileWrAddr <= to_integer(unsigned(R.curInst(aRdAddrRange)));
            NxRAMCtrl.regfileWrData <= vRegfile.writeData;
        else
            NxRAMCtrl.regfileWrAddr <= RAMCtrl.regfileWrAddr;
            NxRAMCtrl.regfileWrData <= RAMCtrl.regfileWrData;
        end if;

        -------------------------------------------------------------------------------
        -- PC
        -------------------------------------------------------------------------------   
        -- write next pc if enabled
        if vPCEN = '1' then
            NxR.curPC <= vALU.aluRes;
        end if;

    end process;

end architecture rtl;
