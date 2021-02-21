
module riscv_system (
	clk_clk,
	leds_export,
	reset_reset_n,
	switches_export,
	hex3_hex0_export,
	hex5_hex4_export,
	pushbuttons_export);	

	input		clk_clk;
	output	[7:0]	leds_export;
	input		reset_reset_n;
	input	[7:0]	switches_export;
	output	[31:0]	hex3_hex0_export;
	output	[15:0]	hex5_hex4_export;
	input	[7:0]	pushbuttons_export;
endmodule
