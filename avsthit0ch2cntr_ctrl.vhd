-- File name: avsthit0ch2cntr_ctrl.vhd 
-- Author: Yifeng Wang (yifenwan@phys.ethz.ch)
-- =======================================
-- Revision: 1.0 (file created)
--		Date: Feb 28, 2024
-- =========
-- Description:	[Avalon-ST hit type 0 channel rate to Counter Controller] 
-- 		It connects the Avalon-ST the channel index portion of "data" signal role.
--		Use a binary to one-hot decoder to translate channel index (5 bit) to 
--		the counter to accumulate the number of errors synchronized to the input clock
--		1) The sclr will reset the value but not the overflow flag.
--		2) The rst will reset both the value and the overflow flag.

-- System Block Diagram:
--
--				   +-------------------------------------------------------------------------------------------------------------------+
--				   |   AVST_CHCNT_SYSTEM                                                                                               |
--				   |   +-----------------------------------------------------------+       +--------------------------------------+    |
--			  AVST |   |    FABRIC                                                 |       | COUNTER_AVMM                         |    |
--			 ------+-->|      ch_id                                   ch_sel       |       |    +--------+                        |    |
--				   |   |      +---+             xxxxxx     32bit      +---+        |       |    |        +-+                      |    |
--				   |   |      |   |     5bit xxxx    xxx--------------+   +--------+-------+----+ counter| |                      |    |
--				   |   |      |   +---------x          x              |   |        |       | ---|        | +-------+              |    |
--				   |   |      +-^-+        xxxx comb  xx              +-^-+        |       |    +        | |       |              |    |
--				   |   |       en           xx  logic xxxx            en_reg       |       |    |        | |       |              |    |
--				   |   |      +---+    1bit x            x  1bit      +---+        |       |    |        | |       |              |    |
--			  clk  |   |      |   +---------x     xx     x------------+   +--------+-------+    ++-------+ +       |              |    |
--			 ------+   |      |   |         xxxxxxxx    xx            |   |        |       |     +---------+    +--+--------------+    |
--			  rst  |   |      +-^-+                 xxx x             +-^-+        |       |            x32     |     (Agent)     |    |
--			 ------+   |                                                           |       |                    | AVMM interface  |    |
--				   |   +-----------------------------------------------------------+       +--------------------+---------^-------+    |
--				   |                                                                            |                         |            |
--				   +-------+----------------------------------------------------------------------------------------------+------------+
--						                                                                        | sclr                    |             
--						                                                                        |                         v             
--		
--		
--		
-- 		
-- ================ synthsizer configuration =================== 		
-- altera vhdl_input_version vhdl_2008
-- ============================================================= 


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use IEEE.math_real.log2;
use IEEE.math_real.ceil;
use ieee.std_logic_arith.conv_std_logic_vector;


entity avsthit0ch2cntr_ctrl is 
generic (
	ST_DATA_W				: natural := 45;
	ST_CH_W					: natural := 4;
	ST_ERR_W				: natural := 3;
	CHANNEL_BITFIELD_MSB	: natural := 40;
	CHANNEL_BITFIELD_LSB	: natural := 36;
	CHANNEL_ID_W			: natural := 5;
	CHANNEL_SEL_W			: natural := 32;
	RST_TIMER				: boolean := true;
	RST_INTERVAL			: integer := 1000; -- unit [ms]
	CLK_FREQUENCY			: natural := 125000000; -- unit [Hz]
	BYPASS_EXTERNAL_SCLR	: boolean := false; -- use internal timer for sclr the counters 
	DEBUG					: natural := 1

);
port (
	-- avalon streaming hit-type 0 interface
	asi_hit_type0_channel			: in  std_logic_vector(ST_CH_W-1 downto 0); -- for asic 0-15
	asi_hit_type0_startofpacket		: in  std_logic;
	asi_hit_type0_endofpacket		: in  std_logic;
	asi_hit_type0_error				: in  std_logic_vector(ST_ERR_W-1 downto 0); 
	asi_hit_type0_data				: in  std_logic_vector(ST_DATA_W-1 downto 0); -- valid is a seperate signal
	asi_hit_type0_valid 			: in  std_logic;
	
	-- avalon slave read-write interface
	-- CSR: write non-zero value to it to enable auto-sclr of counters
	--		write to set the auto-sclr interval
	--		read back value is what is written or default interval 
	avs_ctrl_writedata				: in  std_logic_vector(31 downto 0);
	avs_ctrl_write					: in  std_logic;
	avs_ctrl_readdata				: out std_logic_vector(31 downto 0);
	avs_ctrl_read					: in  std_logic;
	
	-- counter control port 
	o_counter_sclr					: out std_logic;
	o_counter_en					: out std_logic_vector(CHANNEL_SEL_W-1 downto 0);
	
	-- reset and clock interface
	i_sclr					: in  std_logic;
	i_rst					: in  std_logic;
	i_clk					: in  std_logic

);
end entity avsthit0ch2cntr_ctrl;



architecture rtl of avsthit0ch2cntr_ctrl is 

	signal timer_rst_pulse			: std_logic;
	signal sclr_or_logic			: std_logic;
	
	signal ch_id					: std_logic_vector(CHANNEL_ID_W-1 downto 0);
	signal en_d1,en_d2				: std_logic; --en_d2 not needed
	signal ch_sel					: std_logic_vector(CHANNEL_SEL_W-1 downto 0);
	
	signal csr_reset_interval		: std_logic_vector(19 downto 0)	:= std_logic_vector(to_unsigned(RST_INTERVAL, 20)); -- maximum interval is 2^20 =1048576s=291hrs
	signal csr_change_alert			: std_logic := '0'; -- this will reset the timer 
	
	signal avs_ctrl_write_d1		: std_logic;
	
	signal ch_sel_comb				: std_logic_vector(CHANNEL_SEL_W-1 downto 0);
	-- calc reset interval in tick
	signal calc_interval_mult_cnt		: unsigned(7 downto 0);
	signal timer_rst_tick_cnt			: std_logic_vector(39 downto 0);
	signal calc_interval_result_valid	: std_logic;
	-- multiplier
	constant lpm_mult_pipe_latency		: natural := 10;
	component alt_lpm_mult
	PORT
	(
		clock		: IN STD_LOGIC ;
		dataa		: IN STD_LOGIC_VECTOR (19 DOWNTO 0);
		datab		: IN STD_LOGIC_VECTOR (19 DOWNTO 0);
		result		: OUT STD_LOGIC_VECTOR (39 DOWNTO 0)
	);
	end component;
	
	
begin
	-- control port
	o_counter_en		<=	ch_sel;
	
	proc_avmm_interface : process (i_clk,i_rst)
	begin
		if (i_rst = '1') then
			csr_reset_interval		<= std_logic_vector(to_unsigned(RST_INTERVAL, csr_reset_interval'length)); -- be careful here, after reset it go back to default
			avs_ctrl_readdata		<= (others => '0'); -- not necessary
		elsif (rising_edge(i_clk)) then
			avs_ctrl_readdata(19 downto 0)		<= csr_reset_interval;
			avs_ctrl_readdata(31 downto 20)		<= (others=> '0');
			
			if (avs_ctrl_write = '1') then -- in case of conflict first handle write
				csr_reset_interval 					<= avs_ctrl_writedata(19 downto 0);
			elsif (avs_ctrl_read = '1') then
			
			else
				csr_reset_interval					<= csr_reset_interval;
			end if;
		end if;
	end process proc_avmm_interface;
	
	proc_csr_change_alert : process (i_clk,i_rst)
	begin
		if (i_rst = '1') then
			csr_change_alert		<= '0';
		elsif (rising_edge(i_clk)) then
			avs_ctrl_write_d1	<= avs_ctrl_write;
			if (avs_ctrl_write_d1 = '1' and avs_ctrl_write = '0') then
				csr_change_alert		<= '1';
			else 
				csr_change_alert		<= '0';
			end if;
		end if;
	end process proc_csr_change_alert;
	

	gen_sclr_logic_comb	: if BYPASS_EXTERNAL_SCLR = false generate 
		sclr_or_logic			<= i_sclr or timer_rst_pulse;
		o_counter_sclr			<= sclr_or_logic;
	else generate
		o_counter_sclr			<= sclr_or_logic;
	end generate gen_sclr_logic_comb;
	
	proc_binary2onehot_comb	: process (all)
		variable ch_id_integer 		: integer;
	begin
		ch_sel_comb		<= (others=>'0');
		ch_id_integer	:= to_integer(unsigned(ch_id));
		ch_sel_comb(ch_id_integer)		<= '1';
		
	end process proc_binary2onehot_comb;
	
	
	
	proc_reg_in	: process (i_clk,i_rst)
	begin
		if (i_rst = '1') then
			ch_id	<= (others=>'0');
		elsif (rising_edge(i_clk)) then
			if (asi_hit_type0_valid = '1') then -- redundent check, can be removed
				ch_id		<= asi_hit_type0_data(CHANNEL_BITFIELD_MSB downto CHANNEL_BITFIELD_LSB);
			else
				ch_id		<= ch_id;
			end if;
		end if;
	end process proc_reg_in;
		
	proc_reg_out : process (i_clk,i_rst)
	begin 
		if (i_rst = '1') then
			ch_sel	<= (others=>'0');
		elsif (rising_edge(i_clk)) then
			if (en_d1 = '1') then -- latch only when there was a hit
				ch_sel	<= ch_sel_comb;
			else 
				ch_sel	<= (others => '0');
			end if;
		end if;
	end process proc_reg_out;
	
	proc_en_delay : process (i_clk,i_rst)
	begin	
		if (i_rst = '1') then
			en_d1		<= '0';
		elsif (rising_edge(i_clk)) then
			en_d1		<= asi_hit_type0_valid;
			en_d2		<= en_d1;
		end if;
	end process proc_en_delay;
	
	
	proc_timer : process (i_clk,i_rst)
		variable timer_cnt				: std_logic_vector(39 downto 0) := (others=>'0'); -- 32 bit counter is good for 34.35s at 125MHz 
																						-- 48 bit counter will overflow for 22 days. 
																						-- TODO: be mindful of the speed panelty 
		variable timer_tick				: std_logic;
		variable timer_tick_d1			: std_logic;
		
	begin
		if (i_rst = '1') then
			timer_cnt		:= (others => '0');
			timer_tick		:= '0';
		elsif (rising_edge(i_clk)) then
			timer_tick_d1		:= timer_tick;
			if (to_integer(unsigned(csr_reset_interval)) = 0) then
				timer_cnt			:= (others => '0');
				timer_tick			:= '0';
				timer_rst_pulse		<= '0';
			else
				-- when non-zero reset interval is set
				if (to_integer(unsigned(timer_cnt))	< to_integer(unsigned(timer_rst_tick_cnt)) and csr_change_alert = '0' and i_sclr = '0' and calc_interval_result_valid = '1') then 
				-- reset timer count 	1) once it reached the count given the set interval
				-- 						2) csr is modified by outside 
				--						3) a sclr is issued from outside 
					timer_cnt		:= conv_std_logic_vector(to_integer(unsigned(timer_cnt))+1, timer_cnt'length);
				else
					timer_cnt		:= (others => '0');
					timer_tick		:= not timer_tick; -- the tick changes every mark of interval or csr has been written
				end if;
				-- a reset request pulse of 1 cycle will be sent to the counters
				timer_rst_pulse		<= timer_tick_d1 xor timer_tick;
			end if;
		end if;
	end process proc_timer;
	
	proc_calc_interval_rst_tick_cnt : process (i_clk, i_rst)
	begin
		-- 20 bit x 20 bit = 40 bit (counter is 48 bit, TODO: resize one of them)
		if (i_rst = '1') then
			calc_interval_mult_cnt				<= (others => '0');
		elsif (rising_edge(i_clk)) then
			-- wait for multiplier pipeline to finish
			if (csr_change_alert = '1') then 
				calc_interval_mult_cnt				<= to_unsigned(lpm_mult_pipe_latency,calc_interval_mult_cnt'length);
			elsif (to_integer(calc_interval_mult_cnt) /= 0) then -- count down from 10 to 0
				calc_interval_mult_cnt				<= calc_interval_mult_cnt - 1;
			end if;
			-- latch the result from multiplier 
			if (calc_interval_mult_cnt = 0) then 
				calc_interval_result_valid			<= '1';
			else
				calc_interval_result_valid			<= '0';
			end if;
		end if;
	end process;
	
	
	mult_interval2tick : alt_lpm_mult PORT MAP (
		clock	 => i_clk,
		dataa	 => csr_reset_interval,
		datab	 => std_logic_vector(to_unsigned(CLK_FREQUENCY/1000,20)),
		result	 => timer_rst_tick_cnt
	);


end architecture rtl;
	


















