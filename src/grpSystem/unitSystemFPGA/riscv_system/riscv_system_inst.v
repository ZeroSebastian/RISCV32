	riscv_system u0 (
		.clk_clk            (<connected-to-clk_clk>),            //         clk.clk
		.leds_export        (<connected-to-leds_export>),        //        leds.export
		.reset_reset_n      (<connected-to-reset_reset_n>),      //       reset.reset_n
		.switches_export    (<connected-to-switches_export>),    //    switches.export
		.hex3_hex0_export   (<connected-to-hex3_hex0_export>),   //   hex3_hex0.export
		.hex5_hex4_export   (<connected-to-hex5_hex4_export>),   //   hex5_hex4.export
		.pushbuttons_export (<connected-to-pushbuttons_export>)  // pushbuttons.export
	);

