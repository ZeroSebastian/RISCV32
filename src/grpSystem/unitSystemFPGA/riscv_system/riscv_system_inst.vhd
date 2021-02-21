	component riscv_system is
		port (
			clk_clk            : in  std_logic                     := 'X';             -- clk
			leds_export        : out std_logic_vector(7 downto 0);                     -- export
			reset_reset_n      : in  std_logic                     := 'X';             -- reset_n
			switches_export    : in  std_logic_vector(7 downto 0)  := (others => 'X'); -- export
			hex3_hex0_export   : out std_logic_vector(31 downto 0);                    -- export
			hex5_hex4_export   : out std_logic_vector(15 downto 0);                    -- export
			pushbuttons_export : in  std_logic_vector(7 downto 0)  := (others => 'X')  -- export
		);
	end component riscv_system;

	u0 : component riscv_system
		port map (
			clk_clk            => CONNECTED_TO_clk_clk,            --         clk.clk
			leds_export        => CONNECTED_TO_leds_export,        --        leds.export
			reset_reset_n      => CONNECTED_TO_reset_reset_n,      --       reset.reset_n
			switches_export    => CONNECTED_TO_switches_export,    --    switches.export
			hex3_hex0_export   => CONNECTED_TO_hex3_hex0_export,   --   hex3_hex0.export
			hex5_hex4_export   => CONNECTED_TO_hex5_hex4_export,   --   hex5_hex4.export
			pushbuttons_export => CONNECTED_TO_pushbuttons_export  -- pushbuttons.export
		);

