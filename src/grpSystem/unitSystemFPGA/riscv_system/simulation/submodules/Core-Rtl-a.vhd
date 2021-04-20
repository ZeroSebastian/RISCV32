-------------------------------------------------------------------------------
-- Title      : RISC-V 32-Bit FSMD Core
-- Project    : RISC-V 32-Bit Core
-------------------------------------------------------------------------------
-- File       : Core-Rtl-a.vhd
-- Author     : Binder Alexander, Jahn Sebastian
-- Date       : 11.11.2019
-- Revisions  : V1, 11.11.2019 -ba
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------

architecture rtl of Core is

    -- common registers
    signal R, NxR : aRegSet;

    -- RAM data
    constant cInitRAM         : aRAM     := (others => (others => '0'));
    signal RAM                : aRAM     := cInitRAM;
    signal RAMCtrl, NxRAMCtrl : aRAMCtrl := cRAMCtrlDefault;
    attribute ramstyle        : string;
    attribute ramstyle of RAM : signal is "MLAB";

    -- bussignals for remapping
    signal i_readdata  : std_ulogic_vector(cBitWidth - 1 downto 0);
    signal d_readdata  : std_ulogic_vector(cBitWidth - 1 downto 0);
    signal d_writedata : std_ulogic_vector(cBitWidth - 1 downto 0);

begin

    -- remap bus signals to internal representation (memory is little endian, cpu uses big endian)
    -- instrucion bus
    i_readdata      <= std_ulogic_vector(avm_i_readdata);
    -- data bus
    d_readdata      <= std_ulogic_vector(avm_d_readdata);
    avm_d_writedata <= std_logic_vector(d_writedata);

    RAMData : process(csi_clk) is
    begin
        if (rising_edge(csi_clk)) then
            -- write
            RAM(NxRAMCtrl.regfileWrAddr) <= NxRAMCtrl.regfileWrData;
            -- read
            RAMCtrl.rs1Data              <= RAM(NxRAMCtrl.regfileRs1Addr);
            RAMCtrl.rs2Data              <= RAM(NxRAMCtrl.regfileRs2Addr);
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

    Comb : process(R, RAMCtrl, i_readdata, d_readdata, avm_d_waitrequest)
        variable vRegfile  : aRegfileValues;
        variable vImm      : aImm;
        variable vALU      : aALUValues;
        variable vCSR      : aCSRValues;
        variable vDataMem  : aDataMemValues;
        variable vPCEN     : std_ulogic;
        variable vInstrMem : aInstrMemValues;
        variable vDivider  : aDividerValues;

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
        vDivider  := cDividerDefault;

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
                vALU.calc     := '1';
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
                        NxR.ctrlState  <= Fetch;
                        vInstrMem.read := '1';
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
                vALU.calc            := '1';

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
                vALU.calc             := '1';
                vPCEN                 := '1';
                vRegfile.writeDataSrc := cRegWritedataPCSrc;

            -- B-Type Conditional Branch
            when CalculateBranch =>
                -- configure ALU
                case R.curInst(aFunct3Range) is
                    when cCondEq | cCondNe   => vALU.op := ALUOpSub;
                    when cCondLt | cCondGe   => vALU.op := ALUOpSlt;
                    when cCondLtu | cCondGeu => vALU.op := ALUOpSltu;
                    when others              => null;
                end case;
                vALU.src1 := cALUSrc1RegFile;
                vALU.src2 := cALUSrc2RegFile;
                vALU.calc := '1';

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
                vALU.calc             := '1';

            -- I-Type Load Instruction
            when CalculateLoad =>
                NxR.dataBusRead <= '1';
                NxR.ctrlState   <= LoadIdleState;
                vALU.op         := ALUOpAdd;
                vALU.src1       := cALUSrc1RegFile;
                vALU.src2       := cALUSrc2ImmGen;
                vALU.calc       := '1';

            -- idle state -> registered address and read must at least be two cycles on bus
            when LoadIdleState =>
                NxR.ctrlState <= WaitLoad;

            -- Read data from data mem
            when WaitLoad =>
                -- only transit when slave is ready
                if (avm_d_waitrequest = '0') then
                    NxR.ctrlState         <= Fetch;
                    NxR.dataBusRead       <= '0';
                    vInstrMem.read        := '1';
                    vRegfile.writeEnable  := '1';
                    vRegfile.writeDataSrc := cRegWritedataMemRdSrc;
                end if;

            -- S-Type Store Instruction
            when CalculateStore =>
                NxR.ctrlState    <= StoreIdleState;
                NxR.dataBusWrite <= '1';
                vALU.op          := ALUOpAdd;
                vALU.src1        := cALUSrc1RegFile;
                vALU.src2        := cALUSrc2ImmGen;
                vALU.calc        := '1';

            when StoreIdleState =>
                NxR.ctrlState <= WaitStore;

            -- R-Type or I-Type Register Instruction
            when CalculateALUOp =>
                NxR.ctrlState        <= Fetch;
                vRegfile.writeEnable := '1';
                vALU.calc            := '1';
                vInstrMem.read       := '1';

                vALU.src1 := cALUSrc1RegFile;
                -- Immediate or Register Instruction
                if R.curInst(aOPCodeRange) = cOpRType then
                    vALU.src2 := cALUSrc2RegFile;
                else
                    vALU.src2 := cALUSrc2ImmGen;
                end if;

                -- mul div operation
                if gImplementMExtension = 1 and R.curInst(aOPCodeRange) = cOpRType and R.curInst(aFunct7Range) = cMulDivOp then
                    case R.curInst(aFunct3Range) is
                        when cMul    => vALU.op := ALUOpMul;
                        when cMulhu  => vALU.op := ALUOpMulhu;
                        when cMulh   => vALU.op := ALUOpMulh;
                        when cMulhsu => vALU.op := ALUOpMulhsu;
                        when cDivu | cDiv | cRem | cRemu => -- prepare division
                            if (R.curInst(aFunct3Range) = cDiv or R.curInst(aFunct3Range) = cRem) then
                                vDivider.calcSigned := '1';
                                vDivider.divisor    := signed(RAMCtrl.rs2Data(RAMCtrl.rs2Data'high) & RAMCtrl.rs2Data);
                                vDivider.dividend   := signed(RAMCtrl.rs1Data(RAMCtrl.rs1Data'high) & RAMCtrl.rs1Data);

                            elsif (R.curInst(aFunct3Range) = cDivu or R.curInst(aFunct3Range) = cRemu) then
                                vDivider.calcSigned := '0';
                                vDivider.divisor    := signed('0' & RAMCtrl.rs2Data);
                                vDivider.dividend   := signed('0' & RAMCtrl.rs1Data);
                            end if;

                            vDivider.start := '1';
                            NxR.ctrlState  <= WaitDivide;

                            -- abort loading and writing
                            vRegfile.writeEnable := '0';
                            vALU.calc            := '0';
                            vInstrMem.read       := '0';

                        when others => null;
                    end case;
                -- normal alu operation
                else
                    -- ALU OpCode
                    case R.curInst(aFunct3Range) is
                        when cFunct3addsub => -- add/sub
                            if R.curInst(aOPCodeRange) = cOpRType and R.curInst(cFunct7OtherInstrPos) = '1' then
                                vALU.op := ALUOpSub;
                            else
                                vALU.op := ALUOpAdd;
                            end if;
                        when cFunct3sll  => vALU.op := ALUOpSll; -- shift left logical
                        when cFunct3slt  => vALU.op := ALUOpSlt; -- signed less than
                        when cFunct3sltu => vALU.op := ALUOpSltu; -- unsigned less than
                        when cFunct3xor  => vALU.op := ALUOpXor; -- xor
                        when cFunct3sr => -- shift rigth logical/arithmetical                            
                            if R.curInst(cFunct7OtherInstrPos) = '0' then
                                vALU.op := ALUOpSrl;
                            else
                                vALU.op := ALUOpSra;
                            end if;
                        when cFunct3or   => vALU.op := ALUOpOr; -- or
                        when cFunct3and  => vALU.op := ALUOpAnd; -- and
                        when others      => null;
                    end case;
                end if;

            when WaitDivide =>
                -- wait for busy flag to be cleared
                vDivider.start := '0';
                if (gImplementMExtension = 1 and R.divisionDone = '1') then
                    NxR.ctrlState        <= Fetch;
                    vRegfile.writeEnable := '1';
                    vALU.calc            := '1';
                    vInstrMem.read       := '1';

                    if (R.curInst(aFunct3Range) = cDiv or R.curInst(aFunct3Range) = cDivu) then
                        vALU.op := ALUOpDiv;
                    else
                        vALU.op := ALUOpRem;
                    end if;

                end if;
            -- CSR Instruction
            when CalculateSys =>
                case R.curInst(aFunct3Range) is
                    when cSysRW | cSysRWI =>
                        if R.curInst(aCSRRWCheckRange) /= "00000" then
                            vRegfile.writeEnable := '1';
                        end if;
                        vCSR.writeMode := cModeWrite;
                    when cSysRS | cSysRSI =>
                        vRegfile.writeEnable := '1';
                        if R.curInst(aCSRRSRCCheckRange) /= "00000" then
                            vCSR.writeMode := cModeSet;
                        end if;
                    when cSysRC | cSysRCI =>
                        vRegfile.writeEnable := '1';
                        if R.curInst(aCSRRSRCCheckRange) /= "00000" then
                            vCSR.writeMode := cModeClear;
                        end if;
                    when others => null;
                end case;
                vRegfile.writeDataSrc := cRegWritedataCSRSrc;
                vInstrMem.read        := '1';
                NxR.ctrlState         <= Fetch;

            when WaitStore =>
                -- only transit when slave is ready
                if (avm_d_waitrequest = '0') then
                    -- only transit when slave is ready
                    NxR.ctrlState    <= Fetch;
                    NxR.dataBusWrite <= '0';
                    vInstrMem.read   := '1';
                end if;

            when Trap =>
                if R.curInst(aTrapMetCheckRange) = cMTrapRet then
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

        -------------------------------------------------------------------------------
        -- Divider
        -------------------------------------------------------------------------------  

        if (gImplementMExtension = 1) then
            vDivider.Q       := R.divisionQ;
            vDivider.s       := R.divisionS;
            NxR.divisionDone <= '0';
            case R.divisionState is
                when prepare =>
                    if (vDivider.start = '1') then
                        NxR.divisionQ                <= (others => '0');
                        vDivider.Q                   := (others => '0');
                        if (vDivider.calcSigned = '1') then
                            NxR.signDividend <= vDivider.dividend(vDivider.dividend'high);
                            vDivider.s       := (others => vDivider.dividend(vDivider.dividend'high));
                        else
                            vDivider.s       := (others => '0');
                            NxR.signDividend <= '0';
                        end if;
                        vDivider.s(vDivider.dividend'range) := signed(vDivider.dividend);
                        NxR.divisionS                <= vDivider.s;
                        NxR.divisionCount            <= cBitWidth;
                        NxR.divisionIntermZeroRem    <= '0';
                        NxR.divisionState            <= calc;
                        NxR.divisor                  <= vDivider.divisor;
                        -- check if divisor is 0 -> set fields
                        if (vDivider.divisor = 0) then
                            NxR.divisionState                                                           <= prepare;
                            NxR.divisionDone                                                            <= '1';
                            NxR.divisionQ                                                               <= (others => '1');
                            NxR.divisionS                                                               <= (others => '0');
                            NxR.divisionS(NxR.divisionS'high - 1 downto NxR.divisionS'high - cBitWidth) <= vDivider.dividend(vDivider.dividend'high - 1 downto 0);
                        -- calculate first step otherwise
                        else
                            -- ALU config
                            vALU.src1         := cALUSrc1DivS;
                            vALU.src2         := cALUSrc2DivDivisor;
                            vALU.stepDiv      := '1';
                            -- check if 1 or -1 should be stored
                            if (vDivider.s(vDivider.s'high) /= vDivider.divisor(vDivider.divisor'high)) then
                                vDivider.Q(cBitWidth - 1) := '0';
                                vALU.op                   := ALUOpAdd;
                            else
                                vDivider.Q(cBitWidth - 1) := '1';
                                vALU.op                   := ALUOpSub;
                            end if;
                            -- shift left and let alu calculate
                            vDivider.s        := shift_left(vDivider.s, 1);
                            NxR.divisionCount <= cBitWidth - 1;
                            NxR.divisionQ     <= vDivider.Q;
                            NxR.divisionS     <= vDivider.s;
                        end if;
                    end if;
                when calc =>
                    vDivider.divisor := R.divisor;
                    -- calculating step
                    if (R.divisionCount > 0) then
                        -- ALU config
                        vALU.src1         := cALUSrc1DivS;
                        vALU.src2         := cALUSrc2DivDivisor;
                        vALU.stepDiv      := '1';
                        -- intermediate zero remainder?
                        if R.aluZero = '1' and R.divisionCount /= 1 then
                            NxR.divisionIntermZeroRem <= '1';
                        end if;
                        -- check if 1 or -1 should be stored
                        if (vDivider.s(vDivider.s'high) /= R.divisor(R.divisor'high)) then
                            vDivider.Q(R.divisionCount - 1) := '0';
                            vALU.op                         := ALUOpAdd;
                        else
                            vDivider.Q(R.divisionCount - 1) := '1';
                            vALU.op                         := ALUOpSub;
                        end if;
                        -- shift left and let alu calculate
                        vDivider.s        := shift_left(vDivider.s, 1);
                        NxR.divisionCount <= R.divisionCount - 1;
                    -- final result, convert and check if correction step is needed
                    elsif (R.divisionCount = 0) then
                        NxR.divisionState <= prepare;
                        NxR.divisionDone  <= '1';
                        -- convert from 1s complement to 2s complement
                        vDivider.Q        := shift_left(vDivider.Q, 1) + 1;
                        -- correct result if signs do not match
                        if vDivider.s(vDivider.s'high) /= R.signDividend or R.divisionIntermZeroRem = '1' then
                            vALU.src1    := cALUSrc1DivS;
                            vALU.src2    := cALUSrc2DivDivisor;
                            vALU.stepDiv := '1';
                            -- correct vDivider.Q and use ALU to add/subtract divisor from remainder
                            if (R.divisor(R.divisor'high) = vDivider.s(vDivider.s'high)) then
                                vDivider.Q := vDivider.Q + 1;
                                vALU.op    := ALUOpSub;
                            else
                                vDivider.Q := vDivider.Q - 1;
                                vALU.op    := ALUOpAdd;
                            end if;
                        end if;
                    end if;

                    NxR.divisionQ <= vDivider.Q;
                    NxR.divisionS <= vDivider.s;

                when others => null;
            end case;
        end if;

        -------------------------------------------------------------------------------
        -- ALU
        -------------------------------------------------------------------------------
        -- Mux ALUSrc1
        case vALU.src1 is
            when cALUSrc1RegFile => vALU.data1 := '0' & vRegfile.readData1;
            when cALUSrc1Zero    => vALU.data1 := (others => '0');
            when cALUSrc1PrevPC  => vALU.data1 := '0' & std_ulogic_vector(unsigned(R.curPC) - 4);
            when cALUSrc1PC      => vALU.data1 := '0' & R.curPC;
            when cALUSrc1DivS    => vALU.data1 := std_ulogic_vector(vDivider.s(vDivider.s'high downto vDivider.s'high - cBitWidth));
            when others          => null;
        end case;

        -- MUX ALUSrc2
        case vALU.src2 is
            when cALUSrc2RegFile    => vALU.data2 := '0' & vRegfile.readData2;
            when cALUSrc2ImmGen     => vALU.data2 := '0' & vImm;
            when cALUSrc2Const4     => vALU.data2 := std_ulogic_vector(to_unsigned(4, vALU.data2'length));
            when cALUSrc2DivDivisor => vALU.data2 := std_ulogic_vector(vDivider.divisor);
            when others             => null;
        end case;

        -- add and substract from alu needed for other operations
        if (vALU.op = ALUOpAdd) then
            vALU.addsubRes := std_ulogic_vector(unsigned(vALU.data1) + unsigned(vALU.data2));
            if ((vALU.data1(vALU.data1'high - 1) = vALU.data2(vALU.data2'high - 1)) and (vALU.data1(vALU.data1'high - 1) /= vALU.addsubRes(vALU.addsubRes'high - 1))) then
                vALU.addsubCarry := '1';
            end if;
        else
            vALU.addsubRes := std_ulogic_vector(unsigned(vALU.data1) - unsigned(vALU.data2));
            if ((vALU.data1(vALU.data1'high - 1) /= vALU.data2(vALU.data2'high - 1)) and (vALU.data1(vALU.data1'high - 1) /= vALU.addsubRes(vALU.addsubRes'high - 1))) then
                vALU.addsubCarry := '1';
            end if;
        end if;

        -- calculate shift amount
        vALU.shiftAmount := to_integer(unsigned(vALU.data2(aALUShiftRange)));

        if gImplementMExtension = 1 then
            -- calculate multiplication
            -- correct sign for multiplicands
            case vALU.op is
                when ALUOpMul | ALUOpMulhu =>
                    vALU.mul1 := vALU.data1;
                    vALU.mul2 := vALU.data2;
                when ALUOpMulh =>
                    vALU.mul1 := vALU.data1(vALU.data1'high - 1) & vALU.data1(vALU.data1'high - 1 downto 0);
                    vALU.mul2 := vALU.data2(vALU.data2'high - 1) & vALU.data2(vALU.data2'high - 1 downto 0);
                when ALUOpMulhsu =>
                    vALU.mul1 := vALU.data1(vALU.data1'high - 1) & vALU.data1(vALU.data1'high - 1 downto 0);
                    vALU.mul2 := vALU.data2;
                when others => null;
            end case;

            vALU.mulRes := std_ulogic_vector(signed(vALU.mul1) * signed(vALU.mul2));
        end if;

        case vALU.op is
            when ALUOpAdd | ALUOpSub =>
                vALU.rawRes := vALU.addsubRes;
            when ALUOpSlt =>
                vALU.rawRes    := (others => '0');
                vALU.rawRes(0) := (vALU.addsubRes(vALU.addsubRes'high - 1) or vALU.addsubCarry) and not (not vALU.data1(vALU.data1'high - 1) and vALU.data2(vALU.data2'high - 1));
            when ALUOpSltu =>
                vALU.rawRes := (0 => vALU.addsubRes(vALU.addsubRes'high), others => '0');
            when ALUOpAnd =>
                vALU.rawRes := (vALU.data1 AND vALU.data2);
            when ALUOpOr =>
                vALU.rawRes := (vALU.data1 OR vALU.data2);
            when ALUOpXor =>
                vALU.rawRes := (vALU.data1 XOR vALU.data2);
            when ALUOpSll =>
                vALU.rawRes := std_ulogic_vector(
                    shift_left(unsigned(vALU.data1), vALU.shiftAmount));
            when ALUOpSrl | ALUOpSra =>
                vALU.srValue := '0';
                if (vALU.op = ALUOpSra) then
                    vALU.srValue := vALU.data1(vALU.data1'high - 1);
                end if;
                vALU.rawRes  := std_ulogic_vector(
                    shift_right(signed(vALU.srValue & vALU.data1(vALU.data1'high - 1 downto 0)), vALU.shiftAmount));
            when ALUOpMul =>
                vALU.rawRes := vALU.mulRes(vALU.rawRes'range);
            when ALUOpMulhu | ALUOpMulhsu | ALUOpMulh =>
                vALU.rawRes := '0' & vALU.mulRes(vALU.mulRes'high - 2 downto vALU.mulRes'high - 2 - (vALU.res'length - 1));
            when ALUOpDiv | ALUOpDivu =>
                vALU.rawRes := std_ulogic_vector(vDivider.Q(vDivider.Q'high) & vDivider.Q);
            when ALUOpRem | ALUOpRemu =>
                vALU.rawRes := std_ulogic_vector(vDivider.s(vDivider.s'high downto vDivider.s'high - cBitWidth));
            when ALUOpNOP =>
                vALU.rawRes := (others => '-');
            when others => null;
        end case;

        -- Set Status Registers
        vALU.zero     := nor_reduce(vALU.rawRes(vALU.rawRes'high - 1 downto 0));
        vALU.negative := vALU.rawRes(vALU.rawRes'high - 1);
        vALU.carry    := vALU.rawRes(vALU.rawRes'high);

        -- Remove Carry Bit and Store New Value
        if (vALU.calc = '1') then
            vALU.res   := vALU.rawRes(vALU.res'range);
            NxR.aluRes <= vALU.res;
        else
            vALU.res := R.aluRes;
        end if;

        -- forward alu res to division unit if told so
        if (gImplementMExtension = 1 and vALU.stepDiv = '1') then
            NxR.divisionS(NxR.divisionS'high downto NxR.divisionS'high - cBitWidth) <= signed(vALU.rawRes);
            NxR.aluZero                                                             <= vALU.zero;
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
                    else
                        NxR.ctrlState  <= Fetch;
                        vInstrMem.read := '1';
                    end if;
                when cCondNe | cCondLt | cCondLtu =>
                    if vALU.zero = '0' then
                        NxR.ctrlState <= PerformBranch;
                    else
                        NxR.ctrlState  <= Fetch;
                        vInstrMem.read := '1';
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
            when cInstrAddrALUSrc => avm_i_address <= std_logic_vector(vALU.res);
            when others           => null;
        end case;

        if (R.ctrlState = Fetch) then
            NxR.curInst <= i_readdata;
        end if;

        -------------------------------------------------------------------------------
        -- CSR Unit
        -------------------------------------------------------------------------------
        -- Mux CsrWriteData
        if R.curInst(cCSRDataSrcPos) = cCsrDataReg then
            vCSR.writeData := vRegfile.readData1;
        else
            vCSR.writeData := std_ulogic_vector(resize(unsigned(R.curInst(aCSRRSRCCheckRange)), cBitWidth));
        end if;

        vCSR.addrMapped := mapCsrAddr(R.curInst(aCSRRegAddrRange));

        if mapCsrAddrValid(vCSR.addrMapped) then
            vCSR.readData := R.csrReg(vCSR.addrMapped);
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
        vDataMem.address := vALU.res;
        case R.curInst(aFunct3Range) is
            when cMemByte | cMemUnsignedByte =>
                -- check if address is byte aligned
                case vDataMem.address(aMemLowerBytes'range) is
                    when cAddressByte0 =>
                        vDataMem.readData(aByte0AcceseRange)  := d_readdata(aByte0AcceseRange);
                        vDataMem.byteenable                   := cEnableByte0;
                        vDataMem.writeData(aByte0AcceseRange) := vRegfile.readData2(aByte0AcceseRange);
                    when cAddressByte1 =>
                        vDataMem.readData(aByte0AcceseRange)  := d_readdata(aByte1AcceseRange);
                        vDataMem.byteenable                   := cEnableByte1;
                        vDataMem.writeData(aByte1AcceseRange) := vRegfile.readData2(aByte0AcceseRange);
                    when cAddressByte2 =>
                        vDataMem.readData(aByte0AcceseRange)  := d_readdata(aByte2AcceseRange);
                        vDataMem.byteenable                   := cEnableByte2;
                        vDataMem.writeData(aByte2AcceseRange) := vRegfile.readData2(aByte0AcceseRange);
                    when cAddressByte3 =>
                        vDataMem.readData(aByte0AcceseRange)  := d_readdata(aByte3AcceseRange);
                        vDataMem.byteenable                   := cEnableByte3;
                        vDataMem.writeData(aByte3AcceseRange) := vRegfile.readData2(aByte0AcceseRange);
                    when others => null;
                end case;
                vDataMem.address(aMemLowerBytes'range) := "00";
                -- signed or unsigned read?
                if (R.curInst(aFunct3Range) = cMemByte) then
                    vDataMem.readData(vDataMem.readData'high downto aByte0AcceseRange'high + 1) := (others => vDataMem.readData(aByte0AcceseRange'high));
                else
                    vDataMem.readData(vDataMem.readData'high downto aByte0AcceseRange'high + 1) := (others => '0');
                end if;

            when cMemHalfWord | cMemUnsignedHalfWord =>
                -- check how the halfword is aligned
                case vDataMem.address(aMemLowerBytes'range) is
                    when cAddressByte0 =>
                        vDataMem.readData(aHalfword0AcceseRange)  := d_readdata(aHalfword0AcceseRange);
                        vDataMem.writeData(aHalfword0AcceseRange) := vRegfile.readData2(aHalfword0AcceseRange);
                        vDataMem.byteenable                       := cEnableHalfWord0;
                    when cAddressByte2 =>
                        vDataMem.readData(aHalfword0AcceseRange)  := d_readdata(aHalfword1AcceseRange);
                        vDataMem.writeData(aHalfword1AcceseRange) := vRegfile.readData2(aHalfword0AcceseRange);
                        vDataMem.byteenable                       := cEnableHalfWord1;

                    when others => null;
                end case;
                vDataMem.address(aMemLowerBytes'range) := "00";

                -- signed or unsigned read?
                if (R.curInst(aFunct3Range) = cMemHalfWord) then
                    vDataMem.readData(vDataMem.readData'high downto aHalfword0AcceseRange'high + 1) := (others => vDataMem.readData(aHalfword0AcceseRange'high));
                else
                    vDataMem.readData(vDataMem.readData'high downto aHalfword0AcceseRange'high + 1) := (others => '0');
                end if;

            when cMemWord =>
                vDataMem.readData   := std_ulogic_vector(d_readdata);
                vDataMem.writeData  := vRegfile.readData2;
                vDataMem.byteenable := cEnableWord;
            when others =>
                vDataMem.readData   := (others => '0');
                vDataMem.writeData  := (others => '0');
                vDataMem.byteenable := (others => '0');
        end case;

        -- new reg value when waitrequest is not '1'
        -- otherwise the values must remain constant
        if (R.ctrlState = CalculateStore or R.ctrlState = CalculateLoad) then
            NxR.dataBusWriteData  <= vDataMem.writeData;
            NxR.dataBusByteenable <= vDataMem.byteenable;
            NxR.dataBusAddress    <= vDataMem.address;
        end if;

        avm_d_address    <= std_logic_vector(R.dataBusAddress);
        avm_d_byteenable <= std_logic_vector(R.dataBusByteenable);
        avm_d_write      <= std_logic(R.dataBusWrite);
        d_writedata      <= R.dataBusWriteData;
        avm_d_read       <= std_logic(R.dataBusRead);

        -------------------------------------------------------------------------------
        -- Register File - Write Stage
        -------------------------------------------------------------------------------
        -- MUX RegWriteData
        case vRegfile.writeDataSrc is
            when cRegWritedataPCSrc    => vRegfile.writeData := R.curPC;
            when cRegWritedataMemRdSrc => vRegfile.writeData := vDataMem.readData;
            when cRegWritedataCSRSrc   => vRegfile.writeData := vCSR.readData;
            when cRegWritedataALUSrc   => vRegfile.writeData := vALU.res;
            when others                => null;
        end case;

        -- only configure to write when reg address is not 0
        if vRegfile.writeEnable = '1' and R.curInst(aRdAddrRange) /= (aRdAddrRange => '0') then
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
            NxR.curPC <= vALU.res;
        end if;

    end process;

end architecture rtl;
