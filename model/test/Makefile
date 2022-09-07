.PHONY: all clean


all: test_eth2spi

clean:
	rm -f ./test_eth2spi
	rm -f *.vcd

test_eth2spi: test_eth2spi.v ../eth2spi.v
	iverilog -o $@ $^
	./verilog-verify.sh ./test_eth2spi

