`timescale 1ns/100ps;
module dig_core(clk,rst_n,cmd_rdy,cmd_rcvd,cnv_cmplt,sinSAR,cosSAR,eep_rd_data,wrt_SPI,eep_addr,eep_cs_n,eep_r_w_n,chrg_pmp_en,strt_cnv,dst);
  
  input clk,rst_n;
  input cmd_rdy;                                     //Input from SPI to say whether command is given or not
  input [15:0] cmd_rcvd;                            // 16 bit command from SPI which is to be encoded
  input cnv_cmplt;                                 // Input from A2D digital, indication that A2D conversion is over, start correction math
  input [11:0] sinSAR,cosSAR;                     // A2D converted values from A2D digital block
  input [11:0] eep_rd_data;                      // EEPROM read data 
  
  output wrt_SPI;                              // Write Assert signal to SPI Module to write data to SPI register
  output [1:0] eep_addr;                      // EEPROM address to read and write data on address specified
  output eep_cs_n,eep_r_w_n,chrg_pmp_en;     // EEPROM chip select signal(active low),R/W signal(active low) and charge pump enable(active for 3ms)
  output strt_cnv;                          // Output to A2D block to start A2D conversion  
  output [11:0] dst;                       // 12 bit output from digital core
  
  reg [20:0] count;                                               // 21 bit counter to wait for 3ms
  wire tm_eq_3ms;                                                // signal to indicate counter hs counted 3ms
  reg clr_cnt,cnt_en;                                           // Synchronous clear signal for wait Counter    
  reg [24:0] Prod;                                             // 25 bit product register
  reg signed[11:0] sinCorr,cosCorr;                           // Corrected SIN and COS value registers 
  reg [11:0] AngleA, temp;                                   // Angle accumulator and temp registers
  reg eep_cs_n,eep_r_w_n,chrg_pmp_en;                       // EEPROM signals for read and write    
  reg [1:0] eep_addr;                                      // 2 bit EEPROM address to be read or written
  reg ld_anglea,clr_anglea ;                              // Load and clear signal for AngleA register   
  reg strt_cnv;                                          // Signals of A2D conversion 
  reg clr_p,init,cmplmnt,addone;                        // Prod register signals, complement and adddone signal 
  reg ld_cos,ld_sin,ld_temp;                           // Load signal for sinCorr, cosCorr, temp registers 
  reg wrt_SPI;                                        // write assert for SPI register  
  reg [3:0] src0_sel, src1_sel;                      // src1 and src0 select mux lines
  reg [3:0] state, nxt_state;                       // 4 bit state register
  reg inCMD,set_inCMD;                             // inCMD flop, asserted when there is an incoming command
  reg UNLOCK,set_UNLOCK,clr_UNLOCK;               //UNLOCK flop, asserted when its an UNLOCK EEPROM command
  reg cmd_rdy_ff1,cmd_rdy_ff2; 
  reg [15:0] cmd_strd;
  
  wire [1:0]  Prod_sel;                          // Last two bits of Prod register for BOOTH algorithm
  wire [11:0] src0_pos, src0, src1, dst_sat;    // Intermediate buses for ALU, saturation logic
  wire [11:0] sinCorr_shft,cosCorr_shft;       // ASR values of sinCorr and cosCorr registers (Barrel shifter values )
  wire [11:0] dst, arctantable,cmd_data;      // dst bus,arctan values look up table, cmd_data in command
  wire [3:0]  rx_cmd;  
  wire [1:0] cmd_addr;
  
   ////////////////////////
  // Constant Parameters//
 ////////////////////////
   
  parameter constant_zero = 12'h000;       // Constant zero 
  parameter constant_pos  = 12'h7FF;      // Constant 7FF loaded to Angle_accum register based on position of angle
  parameter constant_neg  = 12'h800;     // Constant 800 loaded to Angle_accum register based on position of angle
  parameter constant_SPI  = 12'hA5A;    // Constant A5A for posAck
  
  //////////////////////////////
 //Parameter values of EEPROM// 
//////////////////////////////
localparam Sinoff   = 2'b00;  
localparam Sinscale = 2'b01; 
localparam Cosoff   = 2'b10; 
localparam Cosscale = 2'b11; 

  /////////////////////////////
 //Parameterising FSM States//
/////////////////////////////
  parameter IDLE         = 4'b0000;
  parameter WAIT         = 4'b0001;
  parameter WAIT_CC      = 4'b0010;
  parameter OFFSIN       = 4'b0011;
  parameter SCALE_SIN    = 4'b0100;
  parameter OFFCOS       = 4'b0101;  
  parameter SCALE_COS    = 4'b0110;
  parameter CMD_INTR     = 4'b0111;
  parameter INITAA       = 4'b1000;
  parameter INVSIN       = 4'b1001;  
  parameter INVCOS       = 4'b1010;
  parameter CORDIC1      = 4'b1011;
  parameter CORDIC2      = 4'b1100;
  parameter CORDIC3      = 4'b1101;
  parameter CORDIC4      = 4'b1110;
  
   /////////////////////////////////////////////
  //Parameterising different Command encoding//
 ///////////////////////////////////////////// 
  parameter RD_CORR_SIN  = 4'b0100;
  parameter RD_CORR_COS  = 4'b0101;
  parameter ENTER_CM     = 4'b0110;
  parameter UNLOCK_EEP   = 4'b0111;
  parameter READ_EEP     = 4'b10xx;
  parameter WRITE_EEP    = 4'b11xx;
     
    
    /////////////////////////////////////////////////
   //Parameterising src0 and src1 mux select lines//
  /////////////////////////////////////////////////
  localparam src0_eepscale      = 4'b0000;
  localparam src0_sinSAR        = 4'b0001;
  localparam src0_cosSAR        = 4'b0010; 
  localparam src0_sinCorr       = 4'b0011;
  localparam src0_cosCorr       = 4'b0100;
  localparam src0_sinCorrshft   = 4'b0101;
  localparam src0_cosCorrshft   = 4'b0110;
  localparam src0_arctantable   = 4'b0111;
  localparam src0_const_7ff     = 4'b1000;
  localparam src0_const_800     = 4'b1001;
  localparam src0_const_A5A     = 4'b1010;
  localparam src0_const_000     = 4'b1011;
     
  localparam src1_eepoffset     = 4'b0000;
  localparam src1_Prodadd       = 4'b0001; 
  localparam src1_Multres       = 4'b0010;  
  localparam src1_sinCorr       = 4'b0011;
  localparam src1_cosCorr       = 4'b0100;
  localparam src1_anglea        = 4'b0101; 
  localparam src1_temp          = 4'b0110;
  localparam src1_cmddata       = 4'b0111;
  localparam src1_const_A5A     = 4'b1000;
  localparam src1_const_000     = 4'b1001;
   
 
  ////////////////////////////
 //Loading State register  //
////////////////////////////    
  always@(posedge clk, negedge rst_n)
  if (!rst_n)
    state <= IDLE;
  else 
    state <= nxt_state;
      
  ///////////////////////////////////////////
 // 21 bit Counter to count 3ms wait time //
/////////////////////////////////////////// 
  always@(posedge clk, negedge rst_n)
    if(!rst_n) 
      count <= 21'h000000;
    else if(clr_cnt)
      count <= 21'h000000;
    else if(cnt_en)
      count <= count + 1; 


  ////////////////////////////
 //Loading sinCorr register//
////////////////////////////
  always@(posedge clk, negedge rst_n)  
  if(!rst_n) 
    sinCorr <= 12'h000;   
  else if(ld_sin) 
    sinCorr <= dst;
    
  ////////////////////////////
 //Loading cosCorr register//
////////////////////////////
  always@(posedge clk, negedge rst_n)  
  if(!rst_n)
    cosCorr <= 12'h000;
  else if(ld_cos) 
    cosCorr <= dst;
    
  ////////////////////////////
 //Loading Product register//
////////////////////////////     
  always@(posedge clk, negedge rst_n)
    if(!rst_n)
      Prod <= 25'h0000000;
    else if(clr_p)
      Prod <= 25'h0000000;
    else if(init) begin
      Prod[24:13] <= 12'h000;
      Prod[12:1] <= dst;
      Prod[0] <= 1'b0;    
    end
    else begin
     case(Prod_sel)
      2'b00: Prod <= {Prod[24],Prod[24:1]};               // Prod_sel = 00, Prod = Prod >>>1
      2'b01: Prod <= {dst[11],dst[11:0],Prod[12:1]};      // Prod_sel = 01, Prod = (Prod[24:13] + src0[11:0]) >>> 1
      2'b10: Prod <= {dst[11],dst[11:0],Prod[12:1]};      // Prod_sel = 10, Prod = (Prod[24:13] + src0[11:0]) >>> 1
      2'b11: Prod <= {Prod[24],Prod[24:1]};               // Prod_sel = 11, Prod = Prod >>>1
     endcase
  end
  
  //////////////////////////////////////
 //Loading Angle_accumulator register//
//////////////////////////////////////
  always@(posedge clk, negedge rst_n)  
  if(!rst_n)
     AngleA <= 12'h000;
  else if(ld_anglea)
     AngleA <= dst;
  else if(clr_anglea)
     AngleA <= 12'h000;
  
  /////////////////////////
 //Loading temp register//
/////////////////////////
  always@(posedge clk, negedge rst_n)  
  if(!rst_n)
    temp <= 12'h000;
  else if(ld_temp) 
    temp <= dst; 

  //////////////////////
 //Setting inCMD flop//
//////////////////////
  always@(posedge clk, negedge rst_n)  
  if(!rst_n)
    inCMD <= 1'b0;
  else if(set_inCMD) 
    inCMD <= 1'b1;                            //To go out of command mode you to have reset the machine
  

  //////////////////////
 //Setting UNLOCK flop//
//////////////////////
  always@(posedge clk, negedge rst_n)  
  if(!rst_n)
    UNLOCK <= 1'b0;
  else if(set_UNLOCK) 
    UNLOCK <= 1'b1;
  else if(clr_UNLOCK)
    UNLOCK <= 1'b0;   
    
//Double flopping cmd_rdy signals//
always @(posedge clk, negedge rst_n)
    if (!rst_n)
      begin
        cmd_rdy_ff1 <= 1'b0;
        cmd_rdy_ff2 <= 1'b0;
      end
    else
      begin
        cmd_rdy_ff1 <= cmd_rdy;
        cmd_rdy_ff2 <= cmd_rdy_ff1; 
      end
      
//Store the command from SPI//
//always@(cmd_rdy_ff1)
//cmd_strd = cmd_rcvd;
always@(posedge clk, negedge rst_n)  
  if(!rst_n)
    cmd_strd <= 16'h0000;
  else if(cmd_rdy_ff1) 
    cmd_strd <= cmd_rcvd;
 
    
  /////////////////////////////////////       
 //Finite State Machine for Datapath// 
/////////////////////////////////////
  always@(*)
  begin
     ///////////////////    
    //Default outputs//
   ///////////////////
      clr_cnt = 1'b0;
      clr_p = 1'b0; 
      strt_cnv = 1'b0;
      ld_sin = 1'b0;
      ld_cos = 1'b0;
      init = 1'b1;      
      cmplmnt = 1'b0;
      addone = 1'b0;
      ld_anglea = 1'b0;
      clr_anglea = 1'b0;
      ld_temp = 1'b0;     
      cnt_en =1'b0;  
      set_inCMD = 1'b0;
      set_UNLOCK = 1'b0;
      clr_UNLOCK = 1'b0; 
      wrt_SPI = 1'b0;
      eep_cs_n = 1'b1;
      eep_r_w_n= 1'b1;
      chrg_pmp_en = 1'b0;
      src0_sel = src0_const_000;
      src1_sel = src1_const_000;
      eep_addr = Sinoff;
      
    case(state)
        IDLE:        begin
                       nxt_state = WAIT;
                     end
              
        WAIT:        begin
                        cnt_en = 1'b1;
                        if(cmd_rdy_ff2 && (rx_cmd==4'b0110)) begin                 //Checking for ENTER_CM mode
                            set_inCMD = 1'b1;
                            src0_sel  = src0_const_000;
                            src1_sel  = src1_const_A5A;
                            wrt_SPI   = 1'b1;
                            strt_cnv  = 1'b1;      
                            clr_cnt = 1'b1;                              
                            nxt_state = WAIT_CC;                            
                        end
                        else if(tm_eq_3ms)begin
                             strt_cnv = 1'b1;
                             nxt_state = WAIT_CC;
                             clr_cnt = 1'b1;
                        end
                        else nxt_state = WAIT; 
                      end
              
                
        WAIT_CC:   begin         
                        if(cnv_cmplt)
                          nxt_state = OFFSIN;                             
                        else 
                          nxt_state = WAIT_CC;
                     end
        
        OFFSIN:      begin
                        nxt_state = SCALE_SIN;
                        src0_sel  = src0_sinSAR; 
                        src1_sel  = src1_eepoffset;
                        eep_addr  = Sinoff;
                        eep_cs_n  = 1'b0;
                        eep_r_w_n = 1'b1;
                        ld_sin =1'b1;
                     end
         
                                        
        SCALE_SIN:   begin  
                        init = 1'b0;
                        eep_addr  = Sinscale;
                        eep_cs_n  = 1'b0;
                        eep_r_w_n = 1'b1;
                         if(~(&count[3:2])) begin
                           cnt_en = 1'b1;   
                             if(Prod_sel==2'b01) begin
                                src0_sel = src0_eepscale;
                                src1_sel = src1_Prodadd;
                                nxt_state = SCALE_SIN;
                            end
                            else if(Prod_sel==2'b10) begin
                                src0_sel = src0_eepscale;
                                src1_sel = src1_Prodadd;
                                cmplmnt = 1'b1;
                                addone = 1'b1;
                                nxt_state = SCALE_SIN;
                            end
                            else nxt_state = SCALE_SIN;
                           end
                          else begin
                                nxt_state = OFFCOS;
                                src0_sel = src0_const_000;
                                src1_sel = src1_Multres;
                                clr_cnt = 1'b1;
                                clr_p = 1'b1;     
                                ld_sin = 1'b1;               
                          end
                     end
                
        OFFCOS:      begin
                        nxt_state = SCALE_COS;
                        strt_cnv = 1'b1;
                        src0_sel = src0_cosSAR; 
                        src1_sel = src1_eepoffset;
                        eep_addr  = Cosoff;
                        eep_cs_n  = 1'b0;
                        eep_r_w_n = 1'b1;
                        ld_cos =1'b1;
                     end
        
                    
        SCALE_COS:   begin
                        init = 1'b0;
                        eep_addr  = Cosscale;
                        eep_cs_n  = 1'b0;
                        eep_r_w_n = 1'b1;
                        if(~(&count[3:2])) begin
                          cnt_en = 1'b1;   
                            if(Prod_sel==2'b01) begin
                                src0_sel = src0_eepscale;
                                src1_sel = src1_Prodadd;
                                nxt_state = SCALE_COS;
                            end
                            else if (Prod_sel==2'b10) begin
                                src0_sel = src0_eepscale;
                                src1_sel = src1_Prodadd;
                                cmplmnt = 1'b1;
                                addone = 1'b1;
                                nxt_state = SCALE_COS;
                            end
                            else nxt_state = SCALE_COS;
                            
                          end
                          else begin
                                src0_sel = src0_const_000;
                                src1_sel = src1_Multres;
                                clr_cnt = 1'b1;
                                clr_p = 1'b1;
                                ld_cos = 1'b1;
                                if(inCMD ==1'b1)
                                     nxt_state = CMD_INTR;
                                else  nxt_state = INITAA; 
                          end 
                     end              
             
                    
        CMD_INTR:    begin
                        
                        nxt_state = WAIT_CC;
                        
                        casex(rx_cmd)
                          
                          RD_CORR_SIN: begin
                                          src0_sel = src0_const_000;
                                          src1_sel = src1_sinCorr;
                                          wrt_SPI  = 1'b1;
                                        //  nxt_state = WAIT_CC;
                                       end
                                        
                          RD_CORR_COS: begin
                                          src0_sel = src0_const_000;
                                          src1_sel = src1_cosCorr;
                                          wrt_SPI  = 1'b1;
                                        //  nxt_state = WAIT_CC;
                                       end
                                       
                          ENTER_CM:   begin
                                          src0_sel = src0_const_000;
                                          src1_sel = src1_const_A5A;
                                          wrt_SPI  = 1'b1;
                                         // nxt_state = WAIT_CC;
                                      end
                                       
                          UNLOCK_EEP:  begin
                                          src0_sel = src0_const_000;
                                          src1_sel = src1_const_A5A;
                                          set_UNLOCK  = 1'b1;
                                          wrt_SPI  = 1'b1;
                                         // nxt_state = WAIT_CC;
                                       end
                                       
                          READ_EEP:   begin
                                          src0_sel = src1_eepoffset;              //eepoffset here means data from eep_rd_data bus
                                          src1_sel = src1_const_000;
                                          eep_addr = cmd_addr;
                                          eep_cs_n = 1'b0;
                                          eep_r_w_n = 1'b1;
                                          wrt_SPI  = 1'b1;
                                        //  nxt_state = WAIT_CC; 
                                      end
                                      
                          WRITE_EEP:  begin
                                        if(UNLOCK)begin
                                            cnt_en = 1'b1;
                                          if(~tm_eq_3ms)begin 
                                            eep_cs_n = 1'b0;
                                            eep_r_w_n= 1'b0;
                                            chrg_pmp_en = 1'b1;
                                            eep_addr = cmd_addr;
                                            src0_sel = src0_const_000;
                                            src1_sel = src1_cmddata;
                                            nxt_state = CMD_INTR;
                                          end
                                          else begin
                                            clr_cnt = 1'b1;
                                            clr_UNLOCK = 1'b1;
                                            src0_sel = src0_const_000;
                                            src1_sel = src1_const_A5A;
                                            wrt_SPI  = 1'b1;
                                           // nxt_state = WAIT_CC;
                                          end
                                        end                                                                                 
                                        else begin
                                            src0_sel = src0_const_A5A;
                                            src1_sel = src1_const_000;
                                            cmplmnt = 1'b1;
                                            wrt_SPI  = 1'b1;
                                           // nxt_state = WAIT_CC;
                                        end
                                      end                          
                       endcase
                      end
                      
                          
       INITAA:      begin
                       if(cosCorr[11])
                          if(sinCorr[11])begin
                               src0_sel = src0_const_800;
                               src1_sel = src1_const_000;
                               ld_anglea = 1'b1;
                               nxt_state = INVSIN; 
                          end
                          else begin
                               src0_sel = src0_const_7ff;
                               src1_sel = src1_const_000;
                               ld_anglea = 1'b1;
                               nxt_state = INVSIN; 
                          end
                        else begin
                              clr_anglea = 1'b1;
                              nxt_state = CORDIC1;
                        end
                     end
        
        INVSIN:      begin
                        nxt_state = INVCOS;
                        src0_sel = src0_sinCorr;
                        src1_sel = src1_const_000;
                        cmplmnt = 1'b1;
                        addone = 1'b1;   
                        ld_sin = 1'b1;                 
                     end
        
        INVCOS:      begin
                         nxt_state = CORDIC1;
                         src0_sel = src0_cosCorr;
                         src1_sel = src1_const_000;
                         cmplmnt = 1'b1;
                         addone = 1'b1;     
                         ld_cos = 1'b1;               
                     end
               
         CORDIC1:    begin                                              
                        if(~(&count[3:2]))begin
                           nxt_state = CORDIC2;
                           ld_anglea = 1'b1;
                           if(~sinCorr[11])begin
                               src0_sel = src0_arctantable;
                               src1_sel = src1_anglea;
                           end
                           else begin
                               src0_sel = src0_arctantable;
                               src1_sel = src1_anglea;
                               cmplmnt = 1'b1;
                               addone = 1'b1; 
                          end 
                        end
                        else begin
                              nxt_state = WAIT_CC;
                              clr_cnt = 1'b1;
                              src0_sel = src0_const_000;
                              src1_sel = src1_anglea;
                              wrt_SPI = 1'b1;                              
                        end
                     end
                  
                     
        CORDIC2:     begin
                        nxt_state= CORDIC3;
                        ld_temp = 1'b1;
                        if (~sinCorr[11])begin
                              src0_sel = src0_sinCorrshft;
                              src1_sel = src1_cosCorr;
                        end 
                        else begin
                              src0_sel = src0_sinCorrshft;
                              src1_sel = src1_cosCorr;
                              cmplmnt = 1'b1;
                              addone = 1'b1;
                        end 
                     end
                
                
        CORDIC3:     begin
                        nxt_state= CORDIC4;
                        ld_sin = 1'b1;
                        if (~sinCorr[11])begin
                              src0_sel = src0_cosCorrshft;
                              src1_sel = src1_sinCorr;
                              cmplmnt = 1'b1;
                              addone = 1'b1;
                      
                        end 
                        else begin
                              src0_sel = src0_cosCorrshft;
                              src1_sel = src1_sinCorr;
                        end 
                     end
                
        CORDIC4:     begin
                        nxt_state= CORDIC1;
                        src0_sel = src0_const_000;
                        src1_sel = src1_temp;
                        ld_cos = 1'b1;
                        cnt_en = 1'b1;
                     end 
                  
        default:     nxt_state = WAIT;
      endcase
    end 
    
 
    
  ///////////////////////////////   
 //@500MHz => 21'h16E360 = 3ms// 
///////////////////////////////
assign tm_eq_3ms = (count==21'h16E360)? 1'b1 : 1'b0;    
          
           
  ////////////////////////////////// 
 //Last two bits of Prod register//
//////////////////////////////////
assign Prod_sel = Prod[1:0];           
           
           
  //////////////////////////////
 //src0_sel Multiplexer logic//
////////////////////////////// 
 assign src0_pos =  (src0_sel==src0_eepscale)    ? eep_rd_data:
                    (src0_sel==src0_sinSAR)      ? sinSAR:
                    (src0_sel==src0_cosSAR)      ? cosSAR:
                    (src0_sel==src0_sinCorr)     ? sinCorr: 
                    (src0_sel==src0_cosCorr)     ? cosCorr: 
                    (src0_sel==src0_sinCorrshft) ? sinCorr_shft:
                    (src0_sel==src0_cosCorrshft) ? cosCorr_shft:
                    (src0_sel==src0_arctantable) ? arctantable:
                    (src0_sel==src0_const_7ff)   ? constant_pos:
                    (src0_sel==src0_const_800)   ? constant_neg:
                    (src0_sel==src0_const_A5A)   ? constant_SPI:constant_zero;
                     
  //////////////////////////////    
 //src1_sel Multiplexer logic//
//////////////////////////////
 assign src1 = (src1_sel==src1_eepoffset)      ? eep_rd_data:
               (src1_sel==src1_Prodadd)        ? Prod[24:13]:
               (src1_sel==src1_Multres)        ? Prod[23:12]: 
               (src1_sel==src1_sinCorr)        ? sinCorr: 
               (src1_sel==src1_cosCorr)        ? cosCorr: 
               (src1_sel==src1_anglea)         ? AngleA:
               (src1_sel==src1_temp)           ? temp:
               (src1_sel==src1_cmddata)        ? cmd_data:
               (src1_sel==src1_const_A5A)      ? constant_SPI:constant_zero;
                    
              
  ///////////////////////////////            
 //arctan Look up table values// 
///////////////////////////////                                                
assign arctantable = (count==4'd0) ? 12'h200: 
                     (count==4'd1) ? 12'h12E:
                     (count==4'd2) ? 12'h0A0:
                     (count==4'd3) ? 12'h051:
                     (count==4'd4) ? 12'h029: 
                     (count==4'd5) ? 12'h014:
                     (count==4'd6) ? 12'h00A: 
                     (count==4'd7) ? 12'h005:
                     (count==4'd8) ? 12'h003:
                     (count==4'd9) ? 12'h001:
                     (count==4'd10)? 12'h001:
                     (count==4'd11)? 12'h000: 12'hxxx;
                                 
              
  ///////////////////////////////         
 //Complement term generation //
///////////////////////////////   
assign src0 = (cmplmnt) ? ~(src0_pos) : src0_pos;
 
  ////////////////////////////////////////  
 // ALU operation and output on dst bus//
//////////////////////////////////////// 
assign dst_sat = src1 + src0 + addone;


  /////////////////////
 //Saturation Logic //  
/////////////////////
assign dst = (src0[11] && src1[11] && ~dst_sat[11])  ? constant_neg:
             (~src0[11] && ~src1[11] && dst_sat[11]) ? constant_pos:
             dst_sat; 
             
  /////////////////////////////////////////////////////            
 //Arithmetic Right shifted value from Barrel Shifter/ 
/////////////////////////////////////////////////////           
assign sinCorr_shft = sinCorr >>> count;
assign cosCorr_shft = cosCorr >>> count;

  //////////////////////////////////////////////////
 //Command Interpretation of SPI Master s command// 
//////////////////////////////////////////////////
assign rx_cmd     = cmd_strd[15:12];
assign cmd_data   = cmd_strd[11:0];
assign cmd_addr   = cmd_strd[13:12];
             
endmodule
