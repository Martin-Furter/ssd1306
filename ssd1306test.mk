##
## This file is part of the libopencm3 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##
## Copied from:
## https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/other/timer_interrupt/Makefile

# verbose
# V=1

OPENCM3_LINK_CPLUSPLUS = 1
OPENCM3_DIR = $(HOME)/src/libopencm3/libopencm3
BINARY = ssd1306test

LIBNAME     = opencm3_stm32f1
DEFS        += -DSTM32F1
LDSCRIPT    = $(OPENCM3_DIR)/lib/stm32/f1/stm32f103x8.ld

FP_FLAGS    ?= -msoft-float
ARCH_FLAGS  = -mthumb -mcpu=cortex-m3 $(FP_FLAGS) -mfix-cortex-m3-ldrd

# OpenOCD specific variables

OOCD        ?= openocd
OOCD_INTERFACE  ?= flossjtag
OOCD_TARGET ?= stm32f1x

# this assumes that the examples have been put into the opencm3 directory
include $(OPENCM3_DIR)/examples/examples/rules.mk

