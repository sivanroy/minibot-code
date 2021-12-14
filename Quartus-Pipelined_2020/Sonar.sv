
module Sonar  (input  logic clk, echo,
					output logic trigger,
					output logic [31:0] count);
					
	logic [31:0] dt, cnt, inc;
	logic trigg, triggerON, echoON;
	
	
	always_ff @(posedge clk) begin
		if (dt == 32'd500) begin
			triggerON <= 1'b0;
		end
		else if (dt == 32'd3_001_000) begin  // 60_010µ/20n = 3_000_500
			dt <= 32'd0;
			triggerON <= 1'b1;
			cnt <= inc;
			inc <= 32'd0;
		end
		else begin
			dt <= dt + 32'd1;
		end

		if (echo) inc <= inc + 32'd1;
		
		if (triggerON) trigg <= 1'b1;
		else trigg <= 1'b0;
	end
	
	
	assign trigger = trigg;
	assign count = cnt;
	
endmodule


