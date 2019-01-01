`timescale 1ns/100ps;
module SPI_tx(clk,rst_n,tx_data,wrt,SCLK,SS_n,MOSI,MISO,cmd_rcvd,cmd_rdy,rsp_rdy);

  input clk,rst_n;
  input SS_n,SCLK,MOSI;       // Input from SPI master selecting slave
  input wrt;             // Input asssert signal from Digital core to write 16 bit digital data to SPI reg
  input [15:0] tx_data;     //  12 bit digital data from Digital core

  output cmd_rdy;           // cmd_rdy signal to digital core saying command has been sent from Master(all 16 bits packet)  
  output rsp_rdy;          // output to master saying last command sent has been processed 
  output MISO;            // Data to master based on the command sent
  output [15:0]cmd_rcvd; // 16 bit command packet from Master

  reg [15:0] shft_reg,buffer;
  reg set_cmd,set_rdy,clr_cmd,clr_rsp;
  reg [1:0] state,nxt_state;
  reg shft,ld;
  reg SCLK_ff1,SCLK_ff2,SCLK_ff3,SS_n_ff1,SS_n_ff2;
  reg MOSI_ff1,MOSI_ff2,MOSI_ff3;
  reg cmd_rdy, rsp_rdy;
  reg [15:0] cmd_rcvd ;
  reg wrt_ff1;
  
  wire negSCLK;
  
  
  localparam IDLE = 2'b00;
  localparam TX   = 2'b01;
  localparam RD   = 2'b10;

      ///////////////////////////////////////////
     // write is double buffered...meaning    //
    // our core can write to SPI output      //
   // while read of previous in progress    //
  ///////////////////////////////////////////
  always@(posedge clk)
    if (wrt) begin
      $display("Command processed:write to SPI register from digital core");
      buffer <= tx_data;
    end

    ////////////////////////////////////
   // create parallel shift register //
  ////////////////////////////////////
  always@(posedge clk)
    if (ld)
      shft_reg <= buffer;
    else if (shft)
      shft_reg <= {shft_reg[14:0],1'b0};
   
  //////////////////////////////////
 //Triple Flop Master output MOSI// 
//////////////////////////////////      
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      begin
        MOSI_ff1 <= 1'b0;
        MOSI_ff2 <= 1'b0;
        MOSI_ff3 <= 1'b0;
      end
    else
      begin
        MOSI_ff1 <= MOSI; 
        MOSI_ff2 <= MOSI_ff1;
        MOSI_ff3 <= MOSI_ff2;
      end

//Double flopping wrt signal from digital core
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      begin
        wrt_ff1 <= 1'b0;
      end
    else
      begin
         wrt_ff1 <= wrt;
      end
  
      
    ////////////////////////////////////////  
   // rsp_rdy flipflop with              //
  //  asynchronous reset and            //
 //   syncronously set with wrt signal //    
////////////////////////////////////////
  always @(posedge clk,negedge rst_n)
    if(!rst_n)
      rsp_rdy <= 1'b0;
   else if(set_rdy)
      rsp_rdy <= 1'b1;
   else if(clr_rsp)
      rsp_rdy <= 1'b0;
 
    ////////////////////////////////////////////////  
   // cmd_rdy flipflop with                      //
  //  asynchronous reset and                    //
 //   syncronously set after a SPI transaction //    
////////////////////////////////////////////////

  always @(posedge clk,negedge rst_n)
  begin
    if(!rst_n)
      cmd_rdy <= 1'b0;
    else if(set_cmd)
      cmd_rdy <= 1'b1;
    else if(clr_cmd)
     cmd_rdy <= 1'b0;
 end    

   //////////////////////////////////////
  //cmd_rcvd shift register to        //
 //display parallel result of MOSI   //
//////////////////////////////////////
always @(posedge clk)
  if(shft)
  cmd_rcvd <= {cmd_rcvd[14:0],MOSI_ff3};
    
      
  ////////////////////////////////////////////////////////////
  // double flop SCLK and SS_n for meta-stability purposes //
  ////////////////////////////////////////////////////////////
  always @(posedge clk, negedge rst_n)
    if (!rst_n)
      begin
        SCLK_ff1 <= 1'b0;
        SCLK_ff2 <= 1'b0;
        SCLK_ff3 <= 1'b0;
        SS_n_ff1 <= 1'b1;
        SS_n_ff2 <= 1'b1;
      end
    else
      begin
        SCLK_ff1 <= SCLK; 
        SCLK_ff2 <= SCLK_ff1;
        SCLK_ff3 <= SCLK_ff2;
        SS_n_ff1 <= SS_n;
        SS_n_ff2 <= SS_n_ff1; 
      end

  ///////////////////////////////
  // Implement state register //
  /////////////////////////////
  always @(posedge clk or negedge rst_n)
    if (!rst_n)
      state <= IDLE;
    else
      state <= nxt_state;


  ///////////////////////////////////////
  // Implement state tranisiton logic //
  /////////////////////////////////////
  always @(state,SS_n_ff2,wrt_ff1,negSCLK)
    begin
      //////////////////////
      // Default outputs //
      ////////////////////
      shft = 0;
      ld = 0;
      set_cmd = 1'b0;
      set_rdy = 1'b0;
      clr_rsp = 1'b1;
      clr_cmd = 1'b1;  
 
   
      /////////////////////////////
      // State transition logic //
      ///////////////////////////
      case (state)
        
        IDLE : begin
               ld = 1;
               if (!SS_n_ff2) begin
                    nxt_state = TX;
                    //clr_cmd = 1'b1;
                    //clr_rsp = 1'b1;
               end
               else nxt_state = IDLE;
               end
        
        TX : begin
              shft = negSCLK;
              if(SS_n_ff2) begin
                set_cmd = 1'b1;
                nxt_state = RD;
              end                   
              else nxt_state = TX;
            end 
          
        default: begin
              if(wrt_ff1)begin
                  nxt_state = IDLE;
                  set_rdy = 1'b1;
                end               
              else 
                nxt_state = RD;
              end
      
             
    endcase
  end
  
  /////////////////////////////////////////////////////
  // If SCLK_ff3 is still high, but SCLK_ff2 is low //
  // then a negative edge of SCLK has occurred.    //
  //////////////////////////////////////////////////
  assign negSCLK = ~SCLK_ff2 && SCLK_ff3;  
  
  ///// MISO is shift_reg[15] with a tri-state ///////////
  assign MISO = (SS_n_ff2) ? 1'bz : shft_reg[15];
  
  

endmodule
