
module riscv_system (
	clk_clk,
	const_high_export,
	leds_export,
	reset_reset_n,
	switches_export);	

	input		clk_clk;
	input	[31:0]	const_high_export;
	output	[7:0]	leds_export;
	input		reset_reset_n;
	input	[7:0]	switches_export;
endmodule
