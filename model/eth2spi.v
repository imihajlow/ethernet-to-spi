`timescale 1ns/1ns
module eth2spi(eth_line, sck, mosi, cs);
    input eth_line;
    output sck, mosi, cs;

    assign mosi = eth_line;
    wire #11 n_eth_line = ~eth_line; // 74hc86
    wire #11 eth_line_delayed = ~n_eth_line; // 74hc86 XOR
    wire #11 edge_in = eth_line ^ eth_line_delayed; // 74hc86


    // mosfet + RC + 74hc14 contraption
    reg n_idle;
    integer last_edge_time;
    initial begin
        last_edge_time = 0;
        n_idle = 1'b0;
    end
    always @(posedge edge_in) begin
        last_edge_time <= $time;
        #10 n_idle <= 1'b1;
    end
    always @(posedge edge_in) begin
        #130
        if ($time - last_edge_time > 120) begin
            n_idle = 1'b0;
        end
    end

    wire #30 edge_ena = n_idle | n_eth_line; // 74hc32 + additional RC delay

    wire #10 edge_ = edge_in & edge_ena; // 74hc08

    // mosfet + RC + 74hc14 + 74hc08
    reg inhibit;
    initial begin
        inhibit = 1'b0;
    end
    always @(posedge edge_) begin
        if (~inhibit) begin
            inhibit = 1'b1;
            #75 inhibit = 1'b0;
        end
    end

    assign sck = inhibit;
    assign cs = n_idle;

endmodule
