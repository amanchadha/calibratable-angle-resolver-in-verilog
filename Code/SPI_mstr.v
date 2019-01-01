`timescale 1ns/100ps;
module SPI_mstr(clk,rst_n,SS_n,SCLK,command,MISO,wrt_cmd,done,resp,MOSI);

  input clk,rst_n;
  input wrt_cmd;                          //Input to Master to initiate SPI transaction
//  input RDY;                             // Input fro slave saying response to previous command is ready
  input MISO;                           // Input bit stream of data written from slave to master
  input [15:0] command;                // 16 bit command provided from user
  output SS_n,SCLK,done,MOSI;         // MOSI - Serial data of Master which is resp[0]
  output [15:0] resp;		      	       // parallel data of MISO
  
  reg [15:0] cmd_buf;            // Command buffer to capture 16 bit command to be sent to MOSI
  reg [1:0] state,nxt_state;
  reg [9:0] pause_cntr;
  reg [4:0] bit_cntr;
  reg [15:0] resp;
  reg done;
  reg SS_n;
  reg rst_cnt,en_cnt,shft,ld;

  localparam IDLE = 2'b00;
  localparam BITS = 2'b01;
  localparam TRAIL = 2'b10;

  ///////////////////////////////
  // Implement state register //
  /////////////////////////////
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      state <= IDLE;
    else
      state <= nxt_state;

  ////////////////////////////
  // Implement bit counter //
  //////////////////////////
  always @(posedge clk)
    if (rst_cnt)
      bit_cntr <= 5'b00000;
    else if (en_cnt)
      bit_cntr <= bit_cntr + 1;

  //////////////////////////////
  // Implement pause counter //
  ////////////////////////////
  always @(posedge clk)
    if (rst_cnt)
      pause_cntr <= 10'h1EE;
    else
      pause_cntr <= pause_cntr + 1;

  assign SCLK = pause_cntr[9];

  //////////////////////////////////////
  // resp is shifted on fall of SCLK //
  ////////////////////////////////////
  always @(posedge clk)
    if (shft)
      resp <= {resp[14:0],MISO};
  
   //////////////////////////////////////
  // command is given serially to MOSI//
  /////////////////////////////////////
  always @(posedge clk)
    if(ld)
      cmd_buf <= command;
    else if(shft)
      cmd_buf <= {cmd_buf[14:0],1'b0};
      
            
  assign MOSI = (SS_n)? 1'bx : cmd_buf[15];
      
  ////////////////////////////////////////
  // Implement SM that controls output //
  //////////////////////////////////////
  always @(state, wrt_cmd, pause_cntr)
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
      rst_cnt = 1'b0; 
      SS_n = 1'b1;
      en_cnt = 1'b0;
      shft = 1'b0;
      done = 1'b0;
      ld = 1'b0;

      case (state)
        
        IDLE : begin
               rst_cnt = 1'b1;
               ld = 1'b1;
               if(wrt_cmd) 
               begin
                  SS_n = 1'b0;
                  nxt_state = BITS;
               end
               else nxt_state = IDLE;
               end
               
        BITS : begin
          ////////////////////////////////////
          // For the 16 bits of the packet //
          //////////////////////////////////
          SS_n = 1'b0;
          en_cnt = &pause_cntr;
          shft = en_cnt;
          if (bit_cntr==5'h10) 
            begin
              rst_cnt = 1'b1;
              nxt_state = TRAIL;
            end
          else
            nxt_state = BITS;         
        end
        
        default : begin 	// this is TRAIL state
          //////////////////////////////////////////
          // This state keeps SS_n low till 16   //
          // clocks after the last fall of SCLK //
          ///////////////////////////////////////
          SS_n = 1'b0;
          en_cnt = 1'b1;
          if (bit_cntr==5'h10)
            begin
              done = 1'b1;
              nxt_state = IDLE;
            end
        end
      endcase
    end

endmodule 
