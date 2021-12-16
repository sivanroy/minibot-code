
module SonarFSM(input logic clk, echo,
						input logic [31:0] DataFromRPi,
						output logic trigger,
						output logic [31:0] count);
						
	typedef enum logic [1:0] {STATE_INIT, STATE_TRIGG, STATE_ECHO, STATE_END} statetype;
	statetype state, nextstate;
	
	logic [31:0] timer, cnt;
	logic lastEcho, reset;
	
	assign reset = DataFromRPi[0];
	
	
	always_ff @(posedge clk) begin
		if (reset) state <= STATE_INIT;
		else state <= nextstate;		
	end
	
// Next State Logic

	always_comb begin
		case(state)
			STATE_INIT: begin
					cnt = 32'd0;
					timer = 32'd0;
					trigger = 1'b0;
					lastEcho = 1'b0;
					nextstate = STATE_TRIGG;
			
			end
			STATE_TRIGG: begin
					if (timer == 32'd500) begin
						trigger = 1'b0;
						nextstate = STATE_ECHO;
					end
					else begin
						trigger = 1'b1;
						nextstate = STATE_TRIGG;
					end
					timer = timer + 32'd1;
			end
			STATE_ECHO: begin
					case({lastEcho, echo})
						0: nextstate = STATE_ECHO;
						
						1: begin
								lastEcho = echo;
								cnt = cnt + 32'd1;
								nextstate = STATE_ECHO;
							end
						
						2: nextstate = STATE_END;
						
						3: begin
								cnt = cnt + 32'd1;
								nextstate = STATE_ECHO;
							end
						
					endcase
					timer = timer + 32'd1;
			
			end
			STATE_END: begin
					if (timer == 32'd3_000_500) begin
						count = cnt;
						nextstate = STATE_INIT;
					end
					else begin
						timer = timer + 32'd1;
						nextstate = STATE_END;
					end
			end
		endcase
	end
endmodule
		
		
	