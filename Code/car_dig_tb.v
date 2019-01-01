`timescale 1 ns / 100 ps;
module car_dig_tb();

////////////////////////////////////////////////
// Define any interconnects wider than 1-bit //
//////////////////////////////////////////////
wire [1:0] eep_addr;
wire [11:0] eep_rd_data;
wire [11:0] dst,cosSAR,sinSAR;
wire [11:0] ana_cos,ana_sin;
wire [15:0] resp; 		// response from DUT
reg  [1:0] state,nxt_state;

parameter clock=2;

/////////////////////////////////////////////
// Define any registers used in testbench //
///////////////////////////////////////////
reg [15:0] command;		   	    // connected to command of master SPI
reg wrt_cmd;		         	    // connected to wrt of master SPI
reg clk,rst_n,por_n;
//reg [4:0] index, compare;
integer index, compare;
reg [15:0] cmd_mem[0:20];	 // 8-entry, 16-bit commands
reg [15:0] rcv_mem[0:20];	 // 8-entry, 16-bit commands


parameter IDLE = 2'b00;
parameter SEND = 2'b01;
parameter  RCV = 2'b10;

//////////////////////
// Instantiate DUT //
////////////////////
car_dig DUT(.clk(clk), .rst_n(rst_n), .gt_sin(gt_sin), .gt_cos(gt_cos), .sinSAR(sinSAR), .cosSAR(cosSAR), .smpl(smpl),
            .SCLK(SCLK), .SS_n(SS_n), .MISO(MISO), .MOSI(MOSI), .eep_addr(eep_addr), .eep_cs_n(eep_cs_n),
            .eep_r_w_n(eep_r_w_n), .eep_rd_data(eep_rd_data), .dst(dst), .chrg_pmp_en(chrg_pmp_en), .RDY(RDY));
        

///////////////////////////////
// Instantiate EEPROM Model //
/////////////////////////////
eep iEEP(.clk(clk), .por_n(rst_n), .eep_addr(eep_addr), .wrt_data(dst),.rd_data(eep_rd_data), .eep_cs_n(eep_cs_n),
         .eep_r_w_n(eep_r_w_n), .chrg_pmp_en(chrg_pmp_en));


  ///////////////////////////////////
 // Instantiate A2D analog model //
/////////////////////////////////
AtoD_analog iAna(.cosSAR(cosSAR), .sinSAR(sinSAR), .ana_cos(ana_cos), .ana_sin(ana_sin), .smpl(smpl),
                .gt_cos(gt_cos), .gt_sin(gt_sin));

//////////////////////////////////////////////////////////////
// Instantiate sequencer to read and apply analog_vals.txt //
////////////////////////////////////////////////////////////
AtoD_sequencer iSEQ(.smpl(smpl), .ana_cos(ana_cos), .ana_sin(ana_sin));

 
/////////////////////////////
// Instantiate Master SPI //
///////////////////////////
SPI_mstr iMaster(.clk(clk), .rst_n(rst_n), .wrt_cmd(wrt_cmd), .command(command), .done(done),
                .resp(resp), .SCLK(SCLK), .SS_n(SS_n), .MISO(MISO), .MOSI(MOSI));


///state register for my testbench///
    always @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= IDLE;
    else
      state <= nxt_state;
      
always
  ///////////////////
  // 500MHz clock // 
  /////////////////
  #(clock/2) clk = ~clk;
  
  ///////////////////////////////////////////////
 // Command loading to SPI_master from a file //
///////////////////////////////////////////////
initial begin
  $readmemh("command.txt",cmd_mem);
  $readmemh("results.txt",rcv_mem);
end

/////////////////////////////////////////////////////////////////
// The following section actually implements the real testing //
///////////////////////////////////////////////////////////////

initial
begin
      clk = 1'b0;
      rst_n = 1'b0;
      wrt_cmd = 1'b0;
      por_n = 1'b0;
      index = 0;
      compare = 0;
      @(posedge clk);
      @(negedge clk);
      #(clock*5) rst_n = 1'b1;         
end
  
  always @(state,done,RDY)
    begin
      ///default output values///
      
    case(state)
      
      IDLE: begin
             nxt_state = SEND;
            end
            
      SEND: begin
             wrt_cmd = 1'b1;
             nxt_state = RCV;
             por_n = 1'b1;
             command = cmd_mem[index];
             index = index +1;
            end
            
      RCV: begin
              if(done)begin
                wrt_cmd = 1'b0;
                nxt_state = RCV;
              end
              else if(RDY)begin
                 nxt_state = SEND;
                 if(resp==rcv_mem[compare])begin
                   $display("Values are a match! :) %h %h %h",resp,rcv_mem[compare],compare);
                   compare = compare + 1;
                 end
               else begin
               $display("Values do not match :( %h %h %h",resp,rcv_mem[compare],compare);
               compare = compare + 1;
                end
              end
            end
           default: nxt_state = IDLE;
        endcase
      end
 
  
endmodule
