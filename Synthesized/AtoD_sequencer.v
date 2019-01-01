
`timescale 1 ns / 100 ps;
module AtoD_sequencer(smpl, ana_sin, ana_cos);

input smpl;		// when smpl falls we wait 10 time units an move on to next analog value from file 

output [11:0] ana_sin, ana_cos;

reg [23:0] analog_vals[0:16383];	// stores {ana_cos,ana_sin} values
reg [11:0] ana_sin, ana_cos;
reg [13:0] indx;

initial
  $readmemh("AtoD_vals.txt",analog_vals);

initial begin
  indx = 0;
  #1
  forever begin
    ana_sin = analog_vals[indx][23:12];	// high 12-bits is ana_sin
    ana_cos = analog_vals[indx][11:0];	// low 12-bits is ana_cos
    @(negedge smpl);
    #10;
    indx = indx + 1;
  end
end

endmodule

