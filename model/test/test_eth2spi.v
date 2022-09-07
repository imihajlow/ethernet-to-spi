`timescale 1ns/1ns
module test_eth2spi();

task assert;
    input v;
    if (v !== 1'b1)
        $fatal;
endtask

reg clk;
initial begin
    clk = 1'b0;
end
always begin
    #50 clk = ~clk;
end

reg tx_ena;
reg tx_idle;
initial begin
    tx_ena = 1'b0;
    tx_idle = 1'b1;
end

reg [63:0] src = 64'h2f9a77c388e50055;
reg [5:0] i;
initial begin
    i = 0;
end

wire data = src[i];

wire manchester = tx_ena ? (data ^ clk) : tx_idle;

always @(posedge clk) begin
    if (tx_ena) begin
        i <= i + 1;
    end
end

wire sck, mosi, cs;
eth2spi eth2spi_inst(
    .eth_line(manchester),
    .sck(sck),
    .mosi(mosi),
    .cs(cs));

reg [63:0] dst;
always @(posedge sck) begin
    dst <= {mosi, dst[63:1]};
end

initial begin
    #1000

    // transmit with idle 1
    @(posedge clk);
    #1
    tx_ena = 1'b1;
    #6400
    tx_ena = 1'b0;
    #1000
    assert(dst === src);

    // transmit with idle 0
    tx_idle = 1'b0;
    dst = 64'd0;
    #1000
    @(posedge clk);
    #1
    tx_ena = 1'b1;
    #6400
    tx_ena = 1'b0;
    #1000
    assert(dst === src);
    $finish;
end

initial begin
    $dumpfile("test_eth2spi.vcd");
    $dumpvars;
end


endmodule
