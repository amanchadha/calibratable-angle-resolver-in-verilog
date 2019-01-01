`timescale 1ns/100ps;
module AtoD_analog(cosSAR,sinSAR,ana_cos,ana_sin,smpl,gt_cos,gt_sin);

input [11:0] cosSAR,sinSAR;	
// from A2D digital, acts as DAC input
input [11:0] ana_cos,ana_sin;	
// 12-bit representation of analog input
input smpl;			
// sample input from A2D digital

output gt_cos,gt_sin;		// represents comparator output of both sin and cos channels.

reg [11:0] ana_cos_smpld,ana_sin_smpld;	// sampled versions of "analog" inputs

always @(negedge smpl)
  begin
    ana_cos_smpld = ana_cos;
    ana_sin_smpld = ana_sin;
  end

assign gt_cos = (cosSAR>ana_cos_smpld) ? 1'b1 : 1'b0;
assign gt_sin = (sinSAR>ana_sin_smpld) ? 1'b1 : 1'b0;


endmodule








