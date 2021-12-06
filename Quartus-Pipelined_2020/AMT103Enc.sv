
module AMT103Enc(input  logic clk, A, B,
		 output logic [31:0] Counts);
		 
logic [31:0] cnt, dt;
logic reset;


typedef enum logic [1:0] {S0, S1, S2, S3} statetype; // S0 = 00, S1 = 01, S2 = 10, S3 = 11
statetype prevstate, state, nextstate;

always_ff @(posedge clk) begin
	if (reset) begin
		state <= S0;
		prevstate <= S0;
	end
	else begin
		state <= nextstate;
		prevstate <= state;
	end
end

// next state logic

always_comb begin
	if (reset) begin
			cnt = 32'b0;
		   end
	else begin
		case(state)
			S0:     if (A)  nextstate = S2; // 00 -> 10 : CW
				else if (B)  nextstate = S1; // 00 -> 01 : CCW
				else         nextstate = S0; // NoChange

			S1:     if (A)  nextstate = S3; // 01 -> 11 : CCW
				else if (~B) nextstate = S0; // 01 -> 00 : CW
				else         nextstate = S1; // NoChange

			S2:     if (~A) nextstate = S0; // 10 -> 00 : CCW
				else if (B)  nextstate = S3; // 10 -> 11 : CW
				else         nextstate = S2; // NoChange
	
			S3:     if (~A) nextstate = S1; // 11 -> 01 : CW
				else if (~B) nextstate = S2; // 11 -> 10 : CCW
				else         nextstate = S3; // NoChange

			default:	  nextstate = S0;
		endcase
	end
end
// output logic
always_comb begin
	case({prevstate, state})

		// CW output logic
		{S0, S2}: begin
				cnt = cnt + 1;
			  end
		{S1, S0}: begin
				cnt = cnt + 1;
			  end
		{S2, S3}: begin
				cnt = cnt + 1;
			  end
		{S3, S1}: begin
				cnt = cnt + 1;
			  end

		// CCW output logic
		{S0, S1}: begin
				cnt = cnt - 1;
			  end
		{S1, S3}: begin
				cnt = cnt - 1;
			  end
		{S2, S0}: begin
				cnt = cnt - 1;
			  end
		{S3, S2}: begin
				cnt = cnt - 1;
			  end

		// NoChange output logic
		default:  begin
				cnt = cnt;
			  end
	endcase
end
endmodule

