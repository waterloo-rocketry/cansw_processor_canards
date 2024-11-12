#!/bin/sh

${CLANG_FORMAT:-clang-format} -i --style=file:STM32Cube/src/lib/rocketlib/.clang-format STM32Cube/src/application/*/*.c STM32Cube/src/application/*/*.h STM32Cube/src/drivers/*/*.c STM32Cube/src/drivers/*/*.h
