-------------------------------------------------------------------------------
-- Title      : RISC-V Project Package
-- Project    : RISC-V 32-Bit Core
-------------------------------------------------------------------------------
-- File       : RegFile-Rtl-a.vhd
-- Author	    : Binder Alexander
-- Date		    : 06.09.2019
-- Revisions  : V1, 06.09.2019 -ba
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library work;
use work.Global.all;

package RISCV is

    -------------------------------------------------------------------------------
    -- Common
    -------------------------------------------------------------------------------
    constant cBitWidth     : natural := 32;
    constant cByteWidth    : natural := cBitWidth / cByte;
    constant cInstWidth    : natural := 32;
    constant cCsrAddrWidth : natural := 12;
    constant cCsrAddrCnt   : natural := 36;

    subtype aInst is std_ulogic_vector(cInstWidth - 1 downto 0);
    subtype aWord is std_ulogic_vector(cBitWidth - 1 downto 0);
    subtype aCsrAddr is std_ulogic_vector(cCsrAddrWidth - 1 downto 0);
    subtype aCtrlSignal is std_ulogic;
    subtype aCtrl2Signal is std_ulogic_vector(1 downto 0);
    subtype aCtrl3Signal is std_ulogic_vector(2 downto 0);

    constant cStatusZeroBit  : natural := 0;
    constant cStatusNegBit   : natural := 1;
    constant cStatusCarryBit : natural := 2;

    -------------------------------------------------------------------------------
    -- Ranges 
    -------------------------------------------------------------------------------
    subtype aOPCodeRange is natural range 6 downto 0;
    subtype aFunct3Range is natural range 14 downto 12;
    subtype aFunct7Range is natural range 31 downto 25;
    subtype aRs1AddrRange is natural range 19 downto 15;
    subtype aRs2AddrRange is natural range 24 downto 20;
    subtype aRdAddrRange is natural range 11 downto 7;

    subtype aITypeImmsrcRange is natural range 31 downto 20; -- range of i type immediate inside instruction
    subtype aITypeRangeInImmRange is natural range 11 downto 0; -- range of i type immediate inside extended immediate

    subtype aSTypeImmsrc1Range is natural range 31 downto 25; -- range of s type immediate 11 downto 5
    subtype aSTypeImmsrc2Range is natural range 11 downto 7; -- range of s type immediate 4 downto 0
    subtype aSTypeRangeInImmRange is natural range 11 downto 0; -- range of s type immediate inside extended immediate

    subtype aBTypeImmsrc1Range is natural range 31 downto 31; -- b type immediate src 1
    subtype aBTypeImmsrc2Range is natural range 7 downto 7; -- b type immediate src 2
    subtype aBTypeImmsrc3Range is natural range 30 downto 25; -- b type immediate src 3
    subtype aBTypeImmsrc4Range is natural range 11 downto 8; -- b type immediate src 4
    subtype aBTypeRangeInImmRange is natural range 12 downto 1; -- range of b type immediate inside extended immediate

    subtype aUTypeImmsrcRange is natural range 31 downto 12; -- u type immediadte src
    subtype aUTypeRangeInImmRange is natural range 31 downto 12; -- range of u type immediate inside extended immediate

    subtype aJTypeImmsrc1Range is natural range 31 downto 31; -- j type immediate src 1
    subtype aJTypeImmsrc2Range is natural range 19 downto 12; -- j type immediate src 2
    subtype aJTypeImmsrc3Range is natural range 20 downto 20; -- j type immediate src 3
    subtype aJTypeImmsrc4Range is natural range 30 downto 21; -- j type immediate src 4
    subtype aJTypeRangeInImmRange is natural range 20 downto 1; -- range of j type immediate inside extended immediate

    subtype aALUShiftRange is natural range 5 downto 0; -- alu shift range
    subtype aFunct7OtherInstructionRange is natural range 30 downto 30; -- bit that is set when the other instruction should be executed

    -------------------------------------------------------------------------------
    -- OpCodes
    -------------------------------------------------------------------------------
    subtype aOpCode is std_ulogic_vector(aOPCodeRange);
    subtype aFunct3 is std_ulogic_vector(aFunct3Range);
    subtype aFunct7 is std_ulogic_vector(aFunct7Range);
    subtype aFunct12 is std_ulogic_vector(11 downto 0);

    constant cOpRType    : aOpCode := "0110011";
    constant cOpIArith   : aOpCode := "0010011";
    constant cOpILoad    : aOpCode := "0000011";
    constant cOpIJumpReg : aOpCode := "1100111";
    constant cOpSType    : aOpCode := "0100011";
    constant cOpBType    : aOpCode := "1100011";
    constant cOpJType    : aOpCode := "1101111";
    constant cOpLUI      : aOpCode := "0110111";
    constant cOpAUIPC    : aOpCode := "0010111";
    constant cOpFence    : aOpCode := "0001111";
    constant cOpSys      : aOpCode := "1110011";

    -- Branch Funct3
    constant cCondEq  : aFunct3 := "000";
    constant cCondNe  : aFunct3 := "001";
    constant cCondLt  : aFunct3 := "100";
    constant cCondGe  : aFunct3 := "101";
    constant cCondLtu : aFunct3 := "110";
    constant cCondGeu : aFunct3 := "111";

    -- System Funct3
    constant cSysEnv : aFunct3 := "000";
    constant cSysRW  : aFunct3 := "001";
    constant cSysRS  : aFunct3 := "010";
    constant cSysRC  : aFunct3 := "011";
    constant cSysRWI : aFunct3 := "101";
    constant cSysRSI : aFunct3 := "110";
    constant cSysRCI : aFunct3 := "111";

    -- Mul/Div Funct3
    constant cMulDivOp : aFunct7 := "0000001";
    constant cMul      : aFunct3 := "000";
    constant cMulh     : aFunct3 := "001";
    constant cMulhsu   : aFunct3 := "010";
    constant cMulhu    : aFunct3 := "011";
    constant cDiv      : aFunct3 := "100";
    constant cDivu     : aFunct3 := "101";
    constant cRem      : aFunct3 := "110";
    constant cRemu     : aFunct3 := "111";

    -- Trap Return Funct12
    constant cMTrapRet : aFunct12 := "001100000010";

    -------------------------------------------------------------------------------
    -- Control Unit
    -------------------------------------------------------------------------------
    type aControlUnitState is (
        InitState,
        Fetch,
        ReadReg,
        CalculateUpperimmediate,
        CalculateJump,
        CalculateLoad,
        WaitLoad,
        CalculateStore,
        Wait0,
        CalculateBranch,
        PerformBranch,
        CalculateALUOp,
        CalculateSys,
        CalculateMulDiv,
        Trap);

    constant cMemToRegALU : aCtrlSignal := '0';
    constant cMemToRegMem : aCtrlSignal := '1';

    constant cNoJump : aCtrlSignal := '0';
    constant cJump   : aCtrlSignal := '1';

    constant cALUSrc1RegFile : aCtrl2Signal := "00";
    constant cALUSrc1Zero    : aCtrl2Signal := "01";
    constant cALUSrc1PrevPC  : aCtrl2Signal := "10";
    constant cALUSrc1PC      : aCtrl2Signal := "11";

    constant cALUSrc2RegFile : aCtrl2Signal := "00";
    constant cALUSrc2ImmGen  : aCtrl2Signal := "01";
    constant cALUSrc2Const4  : aCtrl2Signal := "10";

    constant cRegWritedataALUSrc   : aCtrl2Signal := "00";
    constant cRegWritedataMemRdSrc : aCtrl2Signal := "01";
    constant cRegWritedataPCSrc    : aCtrl2Signal := "10";
    constant cRegWritedataCSRSrc   : aCtrl2Signal := "11";

    constant cCsrDataReg  : aCtrlSignal := '0';
    constant cCsrDataImm  : aCtrlSignal := '1';
    -------------------------------------------------------------------------------
    -- Program Counter
    -------------------------------------------------------------------------------
    constant cPCWidth     : natural     := cBitWidth;
    constant cPCIncrement : natural     := cInstWidth / cByte;

    subtype aPCValue is std_ulogic_vector(cBitWidth - 1 downto 0);

    -------------------------------------------------------------------------------
    -- Register File
    -------------------------------------------------------------------------------
    constant cRegWidth    : natural := cBitWidth;
    constant cRegCount    : natural := 32;
    constant cRegAdrWidth : natural := LogDualis(cRegCount);

    subtype aRegValue is std_ulogic_vector(cRegWidth - 1 downto 0);
    subtype aRegAdr is std_ulogic_vector(cRegAdrWidth - 1 downto 0);
    type aRegFile is array (0 to cRegCount - 1) of aRegValue;

    type aRegfileValues is record
        readData1    : aRegValue;
        readData2    : aRegValue;
        writeData    : aRegValue;
        writeEnable  : std_ulogic;
        writeDataSrc : aCtrl2Signal;
    end record;

    constant cRegfileValuesDefault : aRegfileValues := (
        readData1    => (others => '0'),
        readData2    => (others => '0'),
        writeData    => (others => '0'),
        writeEnable  => '0',
        writeDataSrc => cRegWritedataALUSrc
    );

    -------------------------------------------------------------------------------
    -- Immediate Extension
    -------------------------------------------------------------------------------
    constant cImmLen : natural := cBitWidth;

    subtype aImm is std_ulogic_vector(cImmLen - 1 downto 0);

    -------------------------------------------------------------------------------
    -- ALU
    -------------------------------------------------------------------------------
    constant cALUWidth : natural := cBitWidth;

    type aALUOp is (
        ALUOpAdd, ALUOpSub,
        ALUOpSLT, ALUOpSLTU,            -- less than (unsigned)
        ALUOpAnd, ALUOpOr, ALUOpXor,
        ALUOpSLL, ALUOpSRL, ALUOpSRA,   -- shift left/right logical/arithmetic
        ALUOpMul, ALUOpMulh, ALUOpMulhu, ALUOpMulhsu, -- multiplication
        ALUOpNOP
    );
    subtype aRawALUValue is std_ulogic_vector(cALUWidth downto 0); -- incl. overflow bit
    subtype aALUValue is std_ulogic_vector(cALUWidth - 1 downto 0);
    subtype aALUMuLValue is std_ulogic_vector((cALUWidth * 2 + 2) - 1 downto 0);
    subtype aALUMultiplicandValue is std_ulogic_vector(cALUWidth downto 0);

    type aALUValues is record
        src1        : aCtrl2Signal;
        src2        : aCtrl2Signal;
        op          : aALUOp;
        calc        : std_ulogic;
        data1       : aALUValue;
        data2       : aALUValue;
        addsubRes   : aRawALUValue;
        addsubCarry : std_ulogic;
        mul1        : aALUMultiplicandValue;
        mul2        : aALUMultiplicandValue;
        mulRes      : aALUMuLValue;
        srValue     : std_ulogic;
        shiftAmount : natural;
        rawRes      : aRawALUValue;
        res         : aALUValue;
        zero        : std_ulogic;
        negative    : std_ulogic;
        carry       : std_ulogic;
    end record;

    type aALUFlags is record
        zero     : std_ulogic;
        negative : std_ulogic;
        carry    : std_ulogic;
    end record;

    constant cALUValuesDefault : aALUValues := (
        src1        => cALUSrc1RegFile,
        src2        => cALUSrc2RegFile,
        op          => ALUOpAdd,
        calc        => '0',
        data1       => (others => '0'),
        data2       => (others => '0'),
        addsubRes   => (others => '0'),
        addsubCarry => '0',
        mul1        => (others => '0'),
        mul2        => (others => '0'),
        mulRes      => (others => '0'),
        srValue     => '0',
        shiftAmount => 0,
        rawRes      => (others => '0'),
        res         => (others => '0'),
        zero        => '0',
        negative    => '0',
        carry       => '0');

    constant cALUFLagsDefault : aALUFlags := (
        zero     => '0',
        negative => '0',
        carry    => '0');

    constant cFunct3addsub : aFunct3 := "000";
    constant cFunct3sll    : aFunct3 := "001";
    constant cFunct3slt    : aFunct3 := "010";
    constant cFunct3sltu   : aFunct3 := "011";
    constant cFunct3xor    : aFunct3 := "100";
    constant cFunct3sr     : aFunct3 := "101";
    constant cFunct3or     : aFunct3 := "110";
    constant cFunct3and    : aFunct3 := "111";

    constant cFunct7OtherInstrPos : natural := 30;

    -------------------------------------------------------------------------------
    -- Memory
    -------------------------------------------------------------------------------    
    subtype aMemByteselect is std_ulogic_vector(3 downto 0);
    -- Memory Funct3
    constant cMemByte             : aFunct3 := "000";
    constant cMemHalfWord         : aFunct3 := "001";
    constant cMemWord             : aFunct3 := "010";
    constant cMemUnsignedByte     : aFunct3 := "100";
    constant cMemUnsignedHalfWord : aFunct3 := "101";

    constant cEnableByte     : aMemByteselect := "0001";
    constant cEnableHalfWord : aMemByteselect := "0011";
    constant cEnableWord     : aMemByteselect := "1111";

    type aDataMemValues is record
        readData   : aRegValue;
        writeData  : aRegValue;
        byteenable : aMemByteselect;
        read       : std_ulogic;
        write      : std_ulogic;
    end record;

    constant cDataMemDefault : aDataMemValues := (
        readData   => (others => '0'),
        writeData  => (others => '0'),
        byteenable => cEnableWord,
        read       => '0',
        write      => '0'
    );

    constant cInstrAddrPCSrc  : aCtrlSignal := '0';
    constant cInstrAddrALUSrc : aCtrlSignal := '1';

    type aInstrMemValues is record
        read : std_ulogic;
        src  : aCtrlSignal;
    end record;

    constant cInstrMemDefault : aInstrMemValues := (
        read => '0',
        src  => cInstrAddrPCSrc
    );

    -------------------------------------------------------------------------------
    -- CSR
    -------------------------------------------------------------------------------
    constant cModeNoWrite : aCtrl2Signal := "00";
    constant cModeClear   : aCtrl2Signal := "01";
    constant cModeSet     : aCtrl2Signal := "10";
    constant cModeWrite   : aCtrl2Signal := "11";

    type aCSRValues is record
        addrMapped : integer;
        readData   : aRegValue;
        writeData  : aRegValue;
        writeMode  : aCtrl2Signal;
    end record;

    constant cCSRValuesDefault : aCSRValues := (
        addrMapped => 255,
        readData   => (others => '0'),
        writeData  => (others => '0'),
        writeMode  => cModeNoWrite
    );

    -- machine information registers
    constant cCsrMVendorId  : aCsrAddr := x"F11";
    constant cCsrMArchId    : aCsrAddr := x"F12";
    constant cCsrMImpId     : aCsrAddr := x"F13";
    constant cCsrMHartId    : aCsrAddr := x"F14";
    -- machine trap setup
    constant cCsrMStatus    : aCsrAddr := x"300";
    constant cCsrMIsa       : aCsrAddr := x"301";
    constant cCsrMEdeleg    : aCsrAddr := x"302";
    constant cCsrMIdeleg    : aCsrAddr := x"303";
    constant cCsrMIe        : aCsrAddr := x"304";
    constant cCsrMTvec      : aCsrAddr := x"305";
    constant cCsrMCounteren : aCsrAddr := x"306";
    -- machine trap handling
    constant cCsrMScratch   : aCsrAddr := x"340";
    constant cCsrMEpc       : aCsrAddr := x"341";
    constant cCsrMCause     : aCsrAddr := x"342";
    constant cCsrMTval      : aCsrAddr := x"343";
    constant cCsrMIp        : aCsrAddr := x"344";
    -- machine memory protection
    constant cCsrPmpcfg0    : aCsrAddr := x"3A0";
    constant cCsrPmpcfg1    : aCsrAddr := x"3A1";
    constant cCsrPmpcfg2    : aCsrAddr := x"3A2";
    constant cCsrPmpcfg3    : aCsrAddr := x"3A3";
    constant cCsrPmpaddr0   : aCsrAddr := x"3B0";
    constant cCsrPmpaddr1   : aCsrAddr := x"3B1";
    constant cCsrPmpaddr2   : aCsrAddr := x"3B2";
    constant cCsrPmpaddr3   : aCsrAddr := x"3B3";
    constant cCsrPmpaddr4   : aCsrAddr := x"3B4";
    constant cCsrPmpaddr5   : aCsrAddr := x"3B5";
    constant cCsrPmpaddr6   : aCsrAddr := x"3B6";
    constant cCsrPmpaddr7   : aCsrAddr := x"3B7";
    constant cCsrPmpaddr8   : aCsrAddr := x"3B8";
    constant cCsrPmpaddr9   : aCsrAddr := x"3B9";
    constant cCsrPmpaddr10  : aCsrAddr := x"3BA";
    constant cCsrPmpaddr11  : aCsrAddr := x"3BB";
    constant cCsrPmpaddr12  : aCsrAddr := x"3BC";
    constant cCsrPmpaddr13  : aCsrAddr := x"3BD";
    constant cCsrPmpaddr14  : aCsrAddr := x"3BE";
    constant cCsrPmpaddr15  : aCsrAddr := x"3BF";

    type aCsrSet is array (cCsrAddrCnt - 1 downto 0) of aRegValue;

    -------------------------------------------------------------------------------
    -- RegSet
    -------------------------------------------------------------------------------

    type aRegSet is record
        -- common signals
        curInst   : aInst;
        -- control signals
        ctrlState : aControlUnitState;
        -- signals for program counter
        curPC     : aPCValue;
        -- signals for ALU
        aluRes    : aALUValue;
        -- signals for CSR
        csrReg    : aCsrSet;
    end record aRegSet;

    constant cInitValRegSet : aRegSet := (
        curInst   => (others => '0'),
        ctrlState => InitState,
        curPC     => (others => '0'),
        aluRes    => (others => '0'),
        csrReg    => (others => (others => '0'))
    );

    -------------------------------------------------------------------------------
    -- RAM definition
    -------------------------------------------------------------------------------
    -- store working registers and csr regs, workaround needed for block ram generation
    type aRAM is array (0 to cRegCount + cCsrAddrCnt - 1) of aRegValue;

    type aRAMCtrl is record
        -- interfacing regfile
        regfileRs1Addr  : integer;
        regfileRs2Addr  : integer;
        rs1Data         : aRegValue;
        rs2Data         : aRegValue;
        regfileWrAddr   : integer;
        regfileWrData   : aRegValue;
        -- interfacing csr
        csrAddrRemapped : integer;
        csrReadData     : aRegValue;
        csrWrData       : aRegValue;

    end record;

    constant cRAMCtrlDefault : aRAMCtrl := (
        regfileRs1Addr  => 0,
        regfileRs2Addr  => 0,
        rs1Data         => (others => '0'),
        rs2Data         => (others => '0'),
        -- init ram causes all zeros in r0
        regfileWrAddr   => 0,
        regfileWrData   => (others => '0'),
        csrAddrRemapped => 255,
        csrReadData     => (others => '0'),
        csrWrData       => (others => '0')
    );

    ------------------------------------------------------------------------------
    -- Function Definitions
    ------------------------------------------------------------------------------
    -- function mapCsrAddr maps CsrAddr to linear space
    function mapCsrAddr(addr : aCsrAddr) return integer;
    -- functio mapCsrAddrValid checks if the mapped address is in allowed region
    function mapCsrAddrValid(mapAddr : integer) return boolean;
    -- function swapEndianess swaps the endianess of the input
    function swapEndianess(vec : aWord) return aWord;

end RISCV;

package body RISCV is

    function swapEndianess(vec : aWord) return aWord is
        variable swappedVec : aWord;
    begin
        swappedVec(7 downto 0)              := vec(cBitWidth - 1 downto 24);
        swappedVec(15 downto 8)             := vec(23 downto 16);
        swappedVec(23 downto 16)            := vec(15 downto 8);
        swappedVec(cBitWidth - 1 downto 24) := vec(7 downto 0);
        return swappedVec;
    end function;

    function mapCsrAddr(addr : aCsrAddr) return integer is
    begin
        case addr is
            -- machine information registers
            when cCsrMVendorId  => return 0;
            when cCsrMArchId    => return 1;
            when cCsrMImpId     => return 2;
            when cCsrMHartId    => return 3;
            -- machine trap setup
            when cCsrMStatus    => return 4;
            when cCsrMIsa       => return 5;
            when cCsrMEdeleg    => return 6;
            when cCsrMIdeleg    => return 7;
            when cCsrMIe        => return 8;
            when cCsrMTvec      => return 9;
            when cCsrMCounteren => return 10;
            -- machine trap handling
            when cCsrMScratch   => return 11;
            when cCsrMEpc       => return 12;
            when cCsrMCause     => return 13;
            when cCsrMTval      => return 14;
            when cCsrMIp        => return 15;
            -- machine memory protection
            when cCsrPmpcfg0    => return 16;
            when cCsrPmpcfg1    => return 17;
            when cCsrPmpcfg2    => return 18;
            when cCsrPmpcfg3    => return 19;
            when cCsrPmpaddr0   => return 20;
            when cCsrPmpaddr1   => return 21;
            when cCsrPmpaddr2   => return 22;
            when cCsrPmpaddr3   => return 23;
            when cCsrPmpaddr4   => return 24;
            when cCsrPmpaddr5   => return 25;
            when cCsrPmpaddr6   => return 26;
            when cCsrPmpaddr7   => return 27;
            when cCsrPmpaddr8   => return 28;
            when cCsrPmpaddr9   => return 29;
            when cCsrPmpaddr10  => return 30;
            when cCsrPmpaddr11  => return 31;
            when cCsrPmpaddr12  => return 32;
            when cCsrPmpaddr13  => return 33;
            when cCsrPmpaddr14  => return 34;
            when cCsrPmpaddr15  => return 35;

            when others => return 255;  --invalid address
        end case;
    end function;

    function mapCsrAddrValid(mapAddr : integer) return boolean is
    begin
        if mapAddr /= 255 then
            return true;
        else
            return false;
        end if;
    end function;

end RISCV;
