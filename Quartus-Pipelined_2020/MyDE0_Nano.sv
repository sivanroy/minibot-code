
//=======================================================
//  MyARM
//=======================================================

module MyDE0_Nano(

//////////// CLOCK //////////
input logic 		          		CLOCK_50,

//////////// LED //////////
output logic		     [7:0]		LED,

//////////// KEY //////////
input logic 		     [1:0]		KEY,

//////////// SW //////////
input logic 		     [3:0]		SW,

//////////// SDRAM //////////
output logic		    [12:0]		DRAM_ADDR,
output logic		     [1:0]		DRAM_BA,
output logic		          		DRAM_CAS_N,
output logic		          		DRAM_CKE,
output logic		          		DRAM_CLK,
output logic		          		DRAM_CS_N,
inout logic 		    [15:0]		DRAM_DQ,
output logic		     [1:0]		DRAM_DQM,
output logic		          		DRAM_RAS_N,
output logic		          		DRAM_WE_N,

//////////// EPCS //////////
output logic		          		EPCS_ASDO,
input logic 		          		EPCS_DATA0,
output logic		          		EPCS_DCLK,
output logic		          		EPCS_NCSO,

//////////// Accelerometer and EEPROM //////////
output logic		          		G_SENSOR_CS_N,
input logic 		          		G_SENSOR_INT,
output logic		          		I2C_SCLK,
inout logic 		          		I2C_SDAT,

//////////// ADC //////////
output logic		          		ADC_CS_N,
output logic		          		ADC_SADDR,
output logic		          		ADC_SCLK,
input logic 		          		ADC_SDAT,

//////////// 2x13 GPIO Header //////////
inout logic 		    [12:0]		GPIO_2,
input logic 		     [2:0]		GPIO_2_IN,

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_0_PI,
input logic 		     [1:0]		GPIO_0_PI_IN,

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout logic 		    [33:0]		GPIO_1,
input logic 		     [1:0]		GPIO_1_IN
);			 

//=======================================================
//  Clock | Reset
//=======================================================

	logic clk, reset;
	
	assign clk   = CLOCK_50;
	assign reset = GPIO_0_PI[1];

//=======================================================
//  SPI
//=======================================================

	logic 		   spi_clk, spi_cs, spi_mosi, spi_miso;
	logic [31:0]   DataToRPi;
	logic [7:0]    DataAddr;

	spi_slave spi_slave_instance(
		.SPI_CLK    (spi_clk),		// input
		.SPI_CS     (spi_cs),		// input
		.SPI_MOSI   (spi_mosi),		// input
		.SPI_MISO   (spi_miso),		// output
		.DataAddr   (DataAddr),		// output : address of what we want to read form DE0 to RPi
		.DataToRPi  (DataToRPi),	// input  : data to send to the RPi
		.Clk        (clk)				// input
	);
	/*
	assign spi_clk  		= GPIO_0_PI[11];	// SCLK = pin 16 = GPIO_11
	assign spi_cs   		= GPIO_0_PI[9];	// CE0  = pin 14 = GPIO_9
	assign spi_mosi     	= GPIO_0_PI[15];	// MOSI = pin 20 = GPIO_15
	
	assign GPIO_0_PI[13] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 18 = GPIO_13
	*/

	// Connect to JP2 on the DE0-Nano : GPI0-1
	assign spi_clk  		= GPIO_1[22];	// SCLK = pin 27 = GPIO_22
	assign spi_cs   		= GPIO_1[20];	// CE0  = pin 25 = GPIO_20
	assign spi_mosi     	= GPIO_1[23];	// MOSI = pin 28 = GPIO_23

	assign GPIO_1[21] = spi_cs ? 1'bz : spi_miso;  // MISO = pin 26 = GPIO_21
	


//=======================================================
//  Memory
//=======================================================	

	logic [31:0] count;

	always_comb begin
		case(DataAddr)
			2'b00: count = countLeftEnc;
			2'b01: count = countRightEnc;
			2'b10: count = countLeftOdo;
			2'b11: count = countRightOdo;
		default: count = 32'b0;
		endcase
	end

	assign DataToRPi = count;

//=======================================================
//  Encoder
//=======================================================

	/*
	---------- PINs ----------
	ENC 1A : pin 4  - GPIO_1[1]
	ENC 1B : pin 6  - GPIO_1[3]
	ENC 2A : pin 5  - GPIO_1[2]
	ENC 2B : pin 8  - GPIO_1[5]

	ODO 1A : pin 13 - GPIO_1[8]
	ODO 1B : pin 14 - GPIO_1[9]
	ODO 2A : pin 15 - GPIO_1[10]
	ODO 2B : pin 16 - GPIO_1[11]

	ECHO   : pin 21  - GPIO_1[16]
	TRIGGER: pin 22  - GPIO_1[17]
	*/

    // Encoder - Odometer
    logic [31:0] countLeftEnc, countRightEnc, countLeftOdo, countRightOdo;

	logic leftEncA, leftEncB, rightEncA, rightEncB;
	assign leftEncA  = GPIO_1[1];
	assign leftEncB  = GPIO_1[3];
	assign rightEncA = GPIO_1[2];
	assign rightEncB = GPIO_1[5];

	logic leftOdoA, leftOdoB, rightOdoA, rightOdoB;
	assign leftOdoA  = GPIO_1[8];
	assign leftOdoB  = GPIO_1[9];
	assign rightOdoA = GPIO_1[10];
	assign rightOdoB = GPIO_1[11];

	Encoder leftEnc(clk, leftEncA, leftEncB, countLeftEnc);
	Encoder rightEnc(clk, rightEncA, rightEncB, countRightEnc);

	Encoder leftOdo(clk, leftOdoA, leftOdoB, countLeftOdo);
	Encoder rightOdo(clk, rightOdoA, rightOdoB, countRightOdo);

	// Sonar


endmodule

