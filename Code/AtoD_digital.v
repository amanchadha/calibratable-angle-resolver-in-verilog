
`timescale 1ns/100ps;
module AtoD_digital(clk,rst_n,gt_sin,gt_cos,strt_cnv,sinSAR,cosSAR,smpl,cnv_cmplt);
  
  input clk,rst_n;
  input gt_sin,gt_cos,strt_cnv;                     //gt_cos, gt_sin compared signals from A2D ANalog block
  output [11:0] sinSAR,cosSAR;                     // Converted output
  output smpl,cnv_cmplt;
  
  reg [11:0] sinSAR, cosSAR,mask;
  reg [1:0] state,nxt_state;   
  reg [6:0] count;                                 //7 bit counter for counting 128 clock cycles
  reg shift,smpl,ld;                              //Comparision starts after 128 clocks, sample is asserted for 128 clocks after strt_cnv
  reg gt_sinFF1,gt_sinFF2,gt_cosFF1,gt_cosFF2;   //Double flopping gt_sin and gt_cos signals
  reg cnv_cmplt, set_cc, clr_cc,clr_cnt;
  reg strt_cnv_ff1,strt_cnv_ff2;
  
  wire [11:0] nxt_cosSAR,nxt_sinSAR,nxt_mask;
  
  
  parameter IDLE    = 2'b00;
  parameter START   = 2'b01;
  parameter SAMPLE  = 2'b10;
  parameter CONV    = 2'b11;

  ///////////////////
 //State Register //
///////////////////
  always @(posedge clk, negedge rst_n)
  if (!rst_n) 
    state <= IDLE;
  else state <= nxt_state;
 
  //////////////////////////////////////////////
 //Double flopping gt_sin and gt_cos signals //
//////////////////////////////////////////////
 
  always @(posedge clk, negedge rst_n)
  if (!rst_n) begin
    gt_sinFF1 <=1'b0;
    gt_sinFF2 <=1'b0;
    gt_cosFF1 <=1'b0;
    gt_cosFF2 <=1'b0;
  end
  else begin
    gt_sinFF1 <=gt_sin;
    gt_sinFF2 <=gt_sinFF1;
    gt_cosFF1 <=gt_cos;
    gt_cosFF2 <=gt_cosFF1;
  end
  
 //Double flopping start_cnv signal coming from digital core
  always @(posedge clk, negedge rst_n)
  if (!rst_n) begin
    strt_cnv_ff1 <=1'b0;
    strt_cnv_ff2 <=1'b0;
  end
  else begin
    strt_cnv_ff1 <=strt_cnv;
    strt_cnv_ff2 <=strt_cnv_ff1;
  end
 
 
   //////////////////////////////////////
  //Counter to count 128 clock cycles //
 //////////////////////////////////////
  always@(posedge clk)
  if(strt_cnv)
    count <= 7'h00;
  else if(clr_cnt)
    count <= 7'h00;
  else
    count <= count + 1;
    
   /////////////////////////////////////////// 
  //sinSAR, cosSAR, mask registers loading //
 ///////////////////////////////////////////
 always@(posedge clk)
 if(ld) begin
    sinSAR <= 12'h800;
    cosSAR <= 12'h800;
    mask   <= 12'h800;
 end
 else if(shift) begin
    cosSAR <=nxt_cosSAR;
    sinSAR <=nxt_sinSAR;
    mask <= nxt_mask;
 end

  ///////////////////////////////////
 // A2D Conversion complete logic //
///////////////////////////////////
 always@(posedge clk,negedge rst_n)
    if(!rst_n)
        cnv_cmplt <= 1'b0;
    else if(set_cc)
        cnv_cmplt <= 1'b1;
    else if(clr_cc)
        cnv_cmplt <= 1'b0;
    
       
  /////////////////////////        
 //FINITE STATE MACHINE //
/////////////////////////
  always@(state,strt_cnv_ff2,mask,count)
  begin
     ///////////////////
    //Default outputs//
   ///////////////////
    smpl = 1'b0;
    ld = 1'b0;
    set_cc = 1'b0;
    clr_cc = 1'b0;
    clr_cnt= 1'b0;
    shift = 1'b0;
        
    case(state)
      
      IDLE: if(!rst_n)
              nxt_state = IDLE;
            else nxt_state = START;
              
      START: begin
               if(strt_cnv_ff2) begin
                 nxt_state = SAMPLE;                 // When strt_cnv signal is high, counter is cleared and conv_cmplt signal is deasserted
                 clr_cnt = 1;         
                 clr_cc = 1'b1;          
               end
               else 
               nxt_state = START; 
             end
                      
            
      SAMPLE: begin 
              smpl = 1;                        //Asserting sample for first 128 clock cycles after strt_conv is asserted
               if(&count) begin
                nxt_state = CONV;
                ld=1'b1;                           //sinSAR, cosSAR, mask registers loaded with default values after 1st 128 clock cycles
               end
               else 
                nxt_state = SAMPLE;
              end
              
      CONV: begin
            clr_cc = 1'b1;
            shift = &count;                // Shift is asserted every 128 clock cycles to put nxtSAR logic
            if(&count && mask[0]) begin     // Checking 12 iterations is done for SAR algorithm
              nxt_state = START;
              set_cc = 1;
              end
            else 
              nxt_state = CONV;
            end
      
                   
      default: nxt_state = IDLE;
      
    endcase
  end
    
   //////////////////////////////////////////////////
  //Shifting mask by 1 bit every 128 clock cycles // 
 ////////////////////////////////////////////////// 
 
  assign nxt_mask = (!shift)? mask : (mask>>1);
   
   /////////////////////////////////////////////////////////////////////
  //nxt_SAR logic : Assigning values to next states of SAR registers //
 /////////////////////////////////////////////////////////////////////
    
  assign nxt_sinSAR = (!shift)? sinSAR : (gt_sinFF2)? ((sinSAR | (mask>>1)) & ~(mask)) : (sinSAR|(mask>>1));
  
  assign nxt_cosSAR = (!shift)? cosSAR : (gt_cosFF2)? ((cosSAR | (mask>>1)) & ~(mask)) : (cosSAR|(mask>>1));
       
endmodule

