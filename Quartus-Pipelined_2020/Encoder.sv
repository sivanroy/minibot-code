
module Encoder(input  logic clk, A, B,
					output logic [31:0] count);
	
	
	logic [31:0] dt, cnt, cntPosA, cntNegA, cntPosB, cntNegB;
	logic reset;


	always_ff @(posedge clk) begin
		if (dt == 32'd50_000) begin // we compute the count incrementation every 50k clock -> 1 ms
			cnt <= cntPosA + cntPosB + cntNegA + cntNegB;
			reset <= 1'b1;
			dt <= 32'd0;
		end
		else begin
			reset <= 1'b0;
			dt <= dt + 32'b1;
		end
	end


	always_ff @(posedge A, posedge reset) begin
		if (reset) cntPosA <= 32'd1048; 
		else cntPosA <= B ? cntPosA - 32'b1 : cntPosA + 32'b1;
	end

	always_ff @(negedge A, posedge reset) begin
		if (reset) cntNegA <= 32'd1048; 
		else cntNegA <= B ? cntNegA + 32'b1 : cntNegA - 32'b1;
	end

	always_ff @(posedge B, posedge reset) begin
		if (reset) cntPosB <= 32'd1048; 
		else cntPosB <= A ? cntPosB + 32'b1 : cntPosB - 32'b1;
	end

	always_ff @(negedge B, posedge reset) begin
		if (reset) cntNegB <= 32'd1048; 
		else cntNegB <= A ? cntNegB - 32'b1 : cntNegB + 32'b1;
	end

	assign count = cnt;

endmodule