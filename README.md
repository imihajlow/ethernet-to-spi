# An experiment project to receive and decode a 10BASE-T Ethernet signal

The idea is to convert the Manchester-encoded 10BASE-T signal to SPI and use an STM32 MCU to parse it.

A 75C1168 chip is used to convert differential signals to 5V logic levels.

Then, edges are detected using a 74HC86 XOR.

![edge detect](/images/edgedetect.jpeg)

*Blue is the input signal, yellow - detected edges.*

Then, an `~idle` signal is formed using a delay circuit. It goes from 0 to 1 with the first edge and goes back to 0 after approximately 2 µS after the last transition. This signal is used by the MCU to detect the end of an Ethernet frame.

![frame detect](/images/framedetect.jpeg)

When the line is idle, the output of an 75C1168 is undefined. In my case it was always high. Since the Ethernet frame always starts with a pattern of 101010..., this leads to the first transition always being from 1 to 0, which generates the first edge. This edge must be ignored for SPI clock generation. However, the case with a low idle state needs to be handled as well. In that case the line remains low on the frame begin and the first edge should not be ignored.

The filtering of the first edge is done using the already generated `~idle` signal.


![first edge filtering](/images/firstfilter.jpeg)

*First edge is filtered*

The resulting filtered edge signal is fed onto a non-retriggerable monostable circuit which generates a pulse of approximately 75 nS, which is finally used as an SPI clock.

![SPI signal](/images/spi1.jpeg)

![SPI signal](/images/spi2.jpeg)

*Blue - generated SPI SCK signal, yellow - SPI MOSI which is just the input 10BASE-T signal.*
