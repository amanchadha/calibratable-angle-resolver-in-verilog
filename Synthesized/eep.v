`timescale 1 ns / 100 ps;
module eep(clk,por_n,eep_addr,wrt_data,rd_data,eep_cs_n,eep_r_w_n,chrg_pmp_en);

input clk,por_n;
input [1:0] eep_addr;	        // only 4 entry memory
input eep_cs_n,eep_r_w_n;	   // bus control signal
input chrg_pmp_en;          // must be held for at least 3ms

input [11:0] wrt_data;		// data to be written 
output [11:0] rd_data;		// data being read 

reg [11:0] rd_data;
reg [11:0] eep_mem[0:3];	// 4-entry, 12-bit words
reg [11:0] eep_wrt_data;
reg [1:0] eep_wrt_addr;
reg eep_wrt;
reg good_at_start;

/////////////////////////////////////////////////
// EEPROM read is through an active low latch //
///////////////////////////////////////////////
always @(clk)
  if (~clk && ~eep_cs_n && eep_r_w_n)
    rd_data = eep_mem[eep_addr];

///////////////////////////////////////////////////////////////
// eep_wrt signal will go high 1 cycle after bus write cycle//
/////////////////////////////////////////////////////////////
always @(posedge clk or negedge por_n)
  if (~por_n)
    eep_wrt <= 1'b0;
  else
    eep_wrt <= (~eep_cs_n & ~eep_r_w_n);

///////////////////////////////////////////
// flop write data on a bus write cycle //
/////////////////////////////////////////
always @(posedge clk)
  if (~eep_cs_n && ~eep_r_w_n) 
    begin
      eep_wrt_data <= wrt_data;
      eep_wrt_addr <= eep_addr;
    end

//////////////////////////////////////////////////
// Implement write looking for 3ms chrg_pmp_en //
////////////////////////////////////////////////
always @(posedge eep_wrt) begin
  @(posedge clk)
  if (~chrg_pmp_en) $display("ERROR: write to EEPROM while chrg_pmp_en low");
    
  good_at_start = chrg_pmp_en;
  
  repeat(1500000) @(posedge clk)	// wait for what should be 3ms of clocks(3ms clocks = 150000)
  if (~chrg_pmp_en) $display("ERROR: Hey, you didn't hold chrg_pmp_en long enough");
    
  if (chrg_pmp_en && good_at_start)begin
    $display("SUCCESS: Hey, I have written data to my EEPROM");
    eep_mem[eep_wrt_addr] <= eep_wrt_data;
  end
  else
    eep_mem[eep_wrt_addr] <= 12'hxxx;
end

/////////////////////////////////////////////////////////////////
// EEPROM is non-volatile, so load initial contents from file //
///////////////////////////////////////////////////////////////
initial begin
  $readmemh("eep_init.txt",eep_mem);
end

endmodule



