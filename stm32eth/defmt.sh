#!/bin/bash

PORT=$1

echo device = $1
exec 3<$1
stty -f $1 speed 115200

defmt-print -e target/thumbv7m-none-eabi/release/stm32eth --show-skipped-frames --verbose <&3
