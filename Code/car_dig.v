`timescale 1 ns / 100 ps;
module car_dig(clk,rst_n,gt_sin,gt_cos,sinSAR,cosSAR,smpl,SCLK,SS_n,MISO,MOSI,eep_addr,
           eep_cs_n,eep_r_w_n,eep_rd_data,dst,chrg_pmp_en,RDY);

input clk,rst_n;                // clock 500MHz, rst_n active low reset (deassert on negedge clock)
input gt_sin,gt_cos;		// Comparator signals from A2D_analog output for current and voltage channels
input [11:0] eep_rd_data;       // data from EEPROM

output smpl;			// Sample signal to A2D Analog 
output eep_cs_n, eep_r_w_n;	// EEPROM bus control signals
output chrg_pmp_en;		// charge pump enable signal (assert for 3ms when writing)
output RDY;
output [1:0] eep_addr;		// EEPROM address
output [11:0] sinSAR,cosSAR;	// inputs to dual DAC of A2D_analog
output [11:0] dst;		// used as write data to EEPROM

////////////////////
// SPI interface //
//////////////////
input SCLK,SS_n,MOSI;
output MISO;

////////////////////////////////////////////////
// Define any interconnects wider than 1-bit //
//////////////////////////////////////////////
wire [15:0] cmd_rcvd;
wire [11:0] sinSAR,cosSAR;

//////////////////////////////////
// Instantiate "modbus" module // 
////////////////////////////////
SPI_tx iSPI(.clk(clk), .rst_n(rst_n), .wrt(wrt_SPI), .tx_data({4'b0000,dst}), .cmd_rcvd(cmd_rcvd),
         .cmd_rdy(cmd_rdy), .rsp_rdy(RDY), .SCLK(SCLK), .SS_n(SS_n), .MISO(MISO), .MOSI(MOSI));

//////////////////////////////
// Instantiate A2D Digital //
////////////////////////////
AtoD_digital iA2D_dig(.clk(clk), .rst_n(rst_n), .strt_cnv(strt_cnv), .gt_cos(gt_cos), .gt_sin(gt_sin),
                 .cnv_cmplt(cnv_cmplt), .smpl(smpl), .sinSAR(sinSAR), .cosSAR(cosSAR));


///////////////////////////////
// Instantiate Digital Core //
/////////////////////////////
dig_core iDIG(.clk(clk), .rst_n(rst_n), .cmd_rdy(cmd_rdy), .cmd_rcvd(cmd_rcvd), .wrt_SPI(wrt_SPI),
              .dst(dst), .eep_rd_data(eep_rd_data), .eep_cs_n(eep_cs_n),
              .eep_r_w_n(eep_r_w_n), .eep_addr(eep_addr), .chrg_pmp_en(chrg_pmp_en), .strt_cnv(strt_cnv),
              .cnv_cmplt(cnv_cmplt),
              ///// A2D_digital returns sinSAR & cosSAR as unsigned, so convert to signed here //////
              .sinSAR({~sinSAR[11],sinSAR[10:0]}), .cosSAR({~cosSAR[11],cosSAR[10:0]}));	

endmodule
