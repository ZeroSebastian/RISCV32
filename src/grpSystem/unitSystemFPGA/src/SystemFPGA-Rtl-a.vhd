-------------------------------------------------------------------------------
-- Title      : RISC-V 32-Bit System
-- Project    : RISC-V 32-Bit Core
-------------------------------------------------------------------------------
-- File       : SystemFPGA-Rtl-a.vhd
-- Author	  : Binder Alexander
-- Date		  : 20.04.2020
-- Revisions  : V1, 20.04.2020 -ba
-------------------------------------------------------------------------------
-- Description:
-------------------------------------------------------------------------------

architecture rtl of SystemFPGA is
        
	component riscv_system is
		port (
			clk_clk         : in  std_logic;
			reset_reset_n   : in  std_logic;
			switches_export : in  std_logic_vector(7 downto 0);
			leds_export     : out std_logic_vector(7 downto 0)
		);
	end component riscv_system;

begin

	scale_clk : process(CLOCK_50) is
	begin
		if rising_edge(CLOCK_50) then
			LEDR(9) <= SW(9);
			LEDR(8) <= SW(8);
		end if;
	end process;

	u0 : component riscv_system
		port map (
			clk_clk         => CLOCK_50, --slow_clk,
			reset_reset_n   => SW(9),
			switches_export => SW(7 downto 0),
			leds_export     => LEDR(7 downto 0)
		);

	
end architecture rtl;
