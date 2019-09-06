-------------------------------------------------------------------------------
-- Title      : Testbench for RISC-V 32-Bit Register File
-- Project    : RISC-V 32-Bit Core
-------------------------------------------------------------------------------
-- File       : tbRegFile-ea.vhd
-- Author	  : Binder Alexander
-- Date		  : 06.09.2019
-- Revisions  : V1, 06.09.2019 -ba
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

entity tbRegFile is
end entity tbRegFile;

architecture bhv of tbRegFile is
    
begin

UUT: entity work.RegFile(rtl)
    port map()
    
end architecture bhv;