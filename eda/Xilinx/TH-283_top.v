`define GPIO_STATUS_NO		4
`define GPIO_1_NO		4

module TH283_top (
	input  io_clock,
	output io_sysReset_out,
	input  io_jtag_tms,
	input  io_jtag_tdi,
	output io_jtag_tdo,
	input  io_jtag_tck,
	output io_uartStd_txd,
	input  io_uartStd_rxd,
	output io_uartStd_rts,
	input  io_uartStd_cts,
	inout  [`GPIO_STATUS_NO - 1:0] io_gpioStatus,
	output io_spi0_sclk,
	output [0:0] io_spi0_ss,
	output io_spi0_mosi,
	input  io_spi0_miso,
	output io_spi0_rst,
	output io_spi0_wp,
	output io_spi0_hold,
	inout  io_i2c0_scl,
	inout  io_i2c0_sda/*,
	input  io_freqCounter0_clock
*/
);

assign reset = 1'b0;
assign io_spi0_hold = 1'b1;
assign io_spi0_wp = 1'b1;
assign io_spi0_rst = 1'b1;

wire [`GPIO_STATUS_NO - 1:0] gpioStatus_rd;
wire [`GPIO_STATUS_NO - 1:0] gpioStatus_wr;
wire [`GPIO_STATUS_NO - 1:0] gpioStatus_wrEn;

IOBUF#(
	.DRIVE(12),
	.IBUF_LOW_PWR("TRUE"),
	.IOSTANDARD("DEFAULT"),
	.SLEW("SLOW")
) IOBUF_gpioStatus[`GPIO_STATUS_NO - 1:0] (
	.O(gpioStatus_rd),
	.IO(io_gpioStatus),
	.I(gpioStatus_wr),
	// high = input, low = output
	.T(!gpioStatus_wrEn)
);

wire [`GPIO_1_NO - 1:0] gpio1_rd;
wire [`GPIO_1_NO - 1:0] gpio1_wr;
wire [`GPIO_1_NO - 1:0] gpio1_wrEn;

wire [`GPIO_STATUS_NO - 1:0] gpio1;

assign gpio1[0] = gpio1[1];
assign gpio1[2] = gpio1[3];

generate
	genvar i;
	for (i = 0; i < 4; i = i + 1) begin
		assign gpio1[i] = gpio1_wrEn[i] ? gpio1_wr[i] : 1'bZ;
		assign gpio1_rd[i] = gpio1[i];
	end
endgenerate


wire i2c0_scl_read;
wire i2c0_sda_read;
wire i2c0_scl_write;
wire i2c0_sda_write;

PULLUP PU_i2c0_scl (
	.O(io_i2c0_scl)
);
PULLUP PU_i2c0_sda (
	.O(io_i2c0_sda)
);

IBUF IBUF_i2c0_scl_read (
	.O(i2c0_scl_read),
	.I(io_i2c0_scl)
);
IBUF IBUF_i2c0_sda_read (
	.O(i2c0_sda_read),
	.I(io_i2c0_sda)
);

OBUFT OBUFT_i2c0_scl_write (
	.O(io_i2c0_scl),
	.I(1'b0),
	.T(!i2c0_scl_write)
);
OBUFT OBUFT_i2c0_sda_write (
	.O(io_i2c0_sda),
	.I(1'b0),
	.T(!i2c0_sda_write)
);



wire io_freqCounter0_clock;
wire foo1, foo2, foo3, foo4, foo5;
wire locked;
wire pwrdwn = 1'b0;
wire feedback;

PLLE2_BASE #(
	.BANDWIDTH("OPTIMIZED"),  // OPTIMIZED, HIGH, LOW
	.CLKFBOUT_MULT(8),        // Multiply value for all CLKOUT, (2-64)
	.CLKFBOUT_PHASE(0.0),     // Phase offset in degrees of CLKFB, (-360.000-360.000).
	.CLKIN1_PERIOD(10.0),      // Input clock period in ns to ps resolution (i.e. 33.333 is 30 MHz).
	// CLKOUT0_DIVIDE - CLKOUT5_DIVIDE: Divide amount for each CLKOUT (1-128)
	.CLKOUT0_DIVIDE(2),
	.CLKOUT1_DIVIDE(1),
	.CLKOUT2_DIVIDE(1),
	.CLKOUT3_DIVIDE(1),
	.CLKOUT4_DIVIDE(1),
	.CLKOUT5_DIVIDE(1),
	// CLKOUT0_DUTY_CYCLE - CLKOUT5_DUTY_CYCLE: Duty cycle for each CLKOUT (0.001-0.999).
	.CLKOUT0_DUTY_CYCLE(0.5),
	.CLKOUT1_DUTY_CYCLE(0.5),
	.CLKOUT2_DUTY_CYCLE(0.5),
	.CLKOUT3_DUTY_CYCLE(0.5),
	.CLKOUT4_DUTY_CYCLE(0.5),
	.CLKOUT5_DUTY_CYCLE(0.5),
	// CLKOUT0_PHASE - CLKOUT5_PHASE: Phase offset for each CLKOUT (-360.000-360.000).
	.CLKOUT0_PHASE(0.0),
	.CLKOUT1_PHASE(0.0),
	.CLKOUT2_PHASE(0.0),
	.CLKOUT3_PHASE(0.0),
	.CLKOUT4_PHASE(0.0),
	.CLKOUT5_PHASE(0.0),
	.DIVCLK_DIVIDE(1),        // Master division value, (1-56)
	.REF_JITTER1(0.0),        // Reference input jitter in UI, (0.000-0.999).
	.STARTUP_WAIT("FALSE")    // Delay DONE until PLL Locks, ("TRUE"/"FALSE")
) PLLE2_BASE_inst (
	// Clock Outputs: 1-bit (each) output: User configurable clock outputs
	.CLKOUT0(io_freqCounter0_clock),   // 1-bit output: CLKOUT0
	.CLKOUT1(foo1),   // 1-bit output: CLKOUT1
	.CLKOUT2(foo2),   // 1-bit output: CLKOUT2
	.CLKOUT3(foo3),   // 1-bit output: CLKOUT3
	.CLKOUT4(foo4),   // 1-bit output: CLKOUT4
	.CLKOUT5(foo5),   // 1-bit output: CLKOUT5
	// Feedback Clocks: 1-bit (each) output: Clock feedback ports
	.CLKFBOUT(feedback), // 1-bit output: Feedback clock
	.LOCKED(locked),     // 1-bit output: LOCK
	.CLKIN1(io_clock),     // 1-bit input: Input clock
	// Control Ports: 1-bit (each) input: PLL control ports
	.PWRDWN(pwrdwn),     // 1-bit input: Power-down
	.RST(io_sysReset_out),           // 1-bit input: Reset
	// Feedback Clocks: 1-bit (each) input: Clock feedback ports
	.CLKFBIN(feedback)    // 1-bit input: Feedback clock
);








HydrogenTest SOC (
	.io_sys_clock(io_clock),
	.io_sys_reset(reset),
	.io_sys_sysReset_out(io_sysReset_out),
	.io_sys_jtag_tms(io_jtag_tms),
	.io_sys_jtag_tdi(io_jtag_tdi),
	.io_sys_jtag_tdo(io_jtag_tdo),
	.io_sys_jtag_tck(io_jtag_tck),
	.io_per_uartStd_txd(io_uartStd_txd),
	.io_per_uartStd_rxd(io_uartStd_rxd),
	.io_per_uartStd_rts(io_uartStd_rts),
	.io_per_uartStd_cts(io_uartStd_cts),
	.io_per_gpioStatus_pins_read(gpioStatus_rd),
	.io_per_gpioStatus_pins_write(gpioStatus_wr),
	.io_per_gpioStatus_pins_writeEnable(gpioStatus_wrEn),
	.io_per_gpio1_pins_read(gpio1_rd),
	.io_per_gpio1_pins_write(gpio1_wr),
	.io_per_gpio1_pins_writeEnable(gpio1_wrEn),
	.io_per_spi0_ss(io_spi0_ss),
	.io_per_spi0_sclk(io_spi0_sclk),
	.io_per_spi0_mosi(io_spi0_mosi),
	.io_per_spi0_miso(io_spi0_miso),
	.io_per_i2c0_scl_write(i2c0_scl_write),
	.io_per_i2c0_scl_read(i2c0_scl_read),
	.io_per_i2c0_sda_write(i2c0_sda_write),
	.io_per_i2c0_sda_read(i2c0_sda_read),
	.io_per_freqCounter0_clock(io_freqCounter0_clock)
);

endmodule
