# litex/soc/cores/cpu/rocket2/core.py
# Rocket Chip core support for the LiteX SoC.
#
# Author: Stefan Adams <stefan.adams@vipcomag.de>
#
# derived from:
#
# Author: Gabriel L. Somlo <somlo@cmu.edu>
# Copyright (c) 2019, Carnegie Mellon University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above
#       copyright notice, this list of conditions and the following
#       disclaimer in the documentation and/or other materials provided
#       with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os

from migen import *

from litex.soc.interconnect import axi
from litex.soc.interconnect import wishbone
from litex.soc.cores.cpu import CPU
from litedram.frontend.axi import *
from litedram.frontend.wishbone import *


CPU_VARIANTS = {
    32 : {
        "standard"       : "freechips.rocketchip.system.LitexConfig32",
        "linux"          : "freechips.rocketchip.system.LitexLinuxConfig32",
        "linux+dualcore" : "freechips.rocketchip.system.LitexLinuxConfigDualCore32",
        "full"           : "freechips.rocketchip.system.LitexFullConfig32",
    },
    64 : {
        "standard"       : "freechips.rocketchip.system.LitexConfig64",
        "linux"          : "freechips.rocketchip.system.LitexLinuxConfig64",
        "linux+dualcore" : "freechips.rocketchip.system.LitexLinuxConfigDualCore64",
        "full"           : "freechips.rocketchip.system.LitexFullConfig64",
    }
}

GCC_FLAGS = {
    32 : {
        "standard"       : "-march=rv32imac   -mabi=ilp32",
        "linux"          : "-march=rv32imac   -mabi=ilp32",
        "linux+dualcore" : "-march=rv32imac   -mabi=ilp32",
        "full"           : "-march=rv32imafdc -mabi=ilp32",
    },
    64 : {
        "standard"       : "-march=rv64imac   -mabi=lp64",
        "linux"          : "-march=rv64imac   -mabi=lp64",
        "linux+dualcore" : "-march=rv64imac   -mabi=lp64",
        "full"           : "-march=rv64imafdc -mabi=lp64",
    }
}

class Rocket(CPU):
    io_regions = {0x10000000:0x70000000} # origin, length

    @property
    def mem_map(self):
        # Rocket reserves the first 256Mbytes for internal use, so we must change default mem_map.
        return {
            "rom"      : 0x10000000,
            "sram"     : 0x11000000,
            "csr"      : 0x12000000,
            "ethmac"   : 0x30000000,
            "main_ram" : 0x80000000,
        }

    @property
    def gcc_flags(self):
        flags =  "-mno-save-restore "
        flags += GCC_FLAGS[self.data_width][self.variant]
        flags += " -D__rocket__"
        flags += " -D__rocket" + str(self.data_width) + "__"
        return flags

    def __init__(self, platform, variant):

        assert variant in CPU_VARIANTS[self.data_width], "Unsupported variant %s" % variant

        self.platform  = platform
        self.variant   = variant
        self.reset     = Signal()
        self.interrupt = Signal(8)

        # memory bus for litex io+bios
        self.mmio_axi  = mmio_axi = axi.AXIInterface(data_width=32, address_width=32, id_width=4)
        self.mmio_wb   = mmio_wb = wishbone.Interface(data_width=32, adr_width=30)

        self.buses     = [mmio_wb]

        # # #

        self.cpu_params = dict(
            # clock, reset
            i_clock=ClockSignal(),
            i_reset=ResetSignal() | self.reset,

            # debug (ignored)
            #o_debug_clockeddmi_dmi_req_ready      = ,
            i_debug_clockeddmi_dmi_req_valid       = 0,
            i_debug_clockeddmi_dmi_req_bits_addr   = 0,
            i_debug_clockeddmi_dmi_req_bits_data   = 0,
            i_debug_clockeddmi_dmi_req_bits_op     = 0,
            i_debug_clockeddmi_dmi_resp_ready      = 0,
            #o_debug_clockeddmi_dmi_resp_valid     = ,
            #o_debug_clockeddmi_dmi_resp_bits_data = ,
            #o_debug_clockeddmi_dmi_resp_bits_resp = ,
            i_debug_clockeddmi_dmiClock            = 0,
            i_debug_clockeddmi_dmiReset            = 0,
            #o_debug_ndreset                       = ,
            #o_debug_dmactive                      = ,

            # irq
            i_interrupts=self.interrupt,

            # axi mmio (not cached)
            i_mmio_axi4_0_aw_ready      = mmio_axi.aw.ready,
            o_mmio_axi4_0_aw_valid      = mmio_axi.aw.valid,
            o_mmio_axi4_0_aw_bits_id    = mmio_axi.aw.id,
            o_mmio_axi4_0_aw_bits_addr  = mmio_axi.aw.addr,
            o_mmio_axi4_0_aw_bits_len   = mmio_axi.aw.len,
            o_mmio_axi4_0_aw_bits_size  = mmio_axi.aw.size,
            o_mmio_axi4_0_aw_bits_burst = mmio_axi.aw.burst,
            o_mmio_axi4_0_aw_bits_lock  = mmio_axi.aw.lock,
            o_mmio_axi4_0_aw_bits_cache = mmio_axi.aw.cache,
            o_mmio_axi4_0_aw_bits_prot  = mmio_axi.aw.prot,
            o_mmio_axi4_0_aw_bits_qos   = mmio_axi.aw.qos,

            i_mmio_axi4_0_w_ready       = mmio_axi.w.ready,
            o_mmio_axi4_0_w_valid       = mmio_axi.w.valid,
            o_mmio_axi4_0_w_bits_data   = mmio_axi.w.data,
            o_mmio_axi4_0_w_bits_strb   = mmio_axi.w.strb,
            o_mmio_axi4_0_w_bits_last   = mmio_axi.w.last,

            o_mmio_axi4_0_b_ready       = mmio_axi.b.ready,
            i_mmio_axi4_0_b_valid       = mmio_axi.b.valid,
            i_mmio_axi4_0_b_bits_id     = mmio_axi.b.id,
            i_mmio_axi4_0_b_bits_resp   = mmio_axi.b.resp,

            i_mmio_axi4_0_ar_ready      = mmio_axi.ar.ready,
            o_mmio_axi4_0_ar_valid      = mmio_axi.ar.valid,
            o_mmio_axi4_0_ar_bits_id    = mmio_axi.ar.id,
            o_mmio_axi4_0_ar_bits_addr  = mmio_axi.ar.addr,
            o_mmio_axi4_0_ar_bits_len   = mmio_axi.ar.len,
            o_mmio_axi4_0_ar_bits_size  = mmio_axi.ar.size,
            o_mmio_axi4_0_ar_bits_burst = mmio_axi.ar.burst,
            o_mmio_axi4_0_ar_bits_lock  = mmio_axi.ar.lock,
            o_mmio_axi4_0_ar_bits_cache = mmio_axi.ar.cache,
            o_mmio_axi4_0_ar_bits_prot  = mmio_axi.ar.prot,
            o_mmio_axi4_0_ar_bits_qos   = mmio_axi.ar.qos,

            o_mmio_axi4_0_r_ready       = mmio_axi.r.ready,
            i_mmio_axi4_0_r_valid       = mmio_axi.r.valid,
            i_mmio_axi4_0_r_bits_id     = mmio_axi.r.id,
            i_mmio_axi4_0_r_bits_data   = mmio_axi.r.data,
            i_mmio_axi4_0_r_bits_resp   = mmio_axi.r.resp,
            i_mmio_axi4_0_r_bits_last   = mmio_axi.r.last,
        )

        # adapt axi interface to wishbone
        mmio_a2w = ResetInserter()(axi.AXI2Wishbone(mmio_axi, mmio_wb, base_address=0))

        # NOTE: AXI2Wishbone FSMs must be reset with the CPU!
        self.comb += mmio_a2w.reset.eq(ResetSignal() | self.reset)

        self.submodules += mmio_a2w

        # remember this to add verilog sources later
        self.platform = platform
        self.variant  = variant

    def set_reset_address(self, reset_address):
        assert not hasattr(self, "reset_address")
        self.reset_address = reset_address
        assert reset_address == 0x10000000, "cpu_reset_addr is hardcoded during rocket-chip elaboration!"

    def add_sources(self):
        basename = CPU_VARIANTS[self.data_width][self.variant] + "Mem" + str(self.mem_width)
        self.platform.add_sources(
            _get_gdir(),
            basename + ".v",
            basename + ".behav_srams.v",
            "plusarg_reader.v",
            "AsyncResetReg.v",
            "EICG_wrapper.v",
        )

    def do_finalize(self):
        assert hasattr(self, "reset_address"), "reset_address should be defined!"
        assert hasattr(self, "axi2native"), "sdram should be connected!"
        self.specials += Instance("LitexRocketSystem", **self.cpu_params)

    def connect_sdram(self, soc, size, use_axi=True):
        mem_width = soc.sdram.crossbar.controller.data_width
        if mem_width < 64: mem_width = 64
        self.mem_width = mem_width
        self.sdram_size = size
        self.clk_freq = soc.clk_freq

        # add sources
        self.add_sources()

        # add sdram
        mem_axi = axi.AXIInterface(data_width=mem_width, address_width=32, id_width=4)
        mem_params = dict(
            # axi memory (L2-cached)
            i_mem_axi4_0_aw_ready      = mem_axi.aw.ready,
            o_mem_axi4_0_aw_valid      = mem_axi.aw.valid,
            o_mem_axi4_0_aw_bits_id    = mem_axi.aw.id,
            o_mem_axi4_0_aw_bits_addr  = mem_axi.aw.addr,
            o_mem_axi4_0_aw_bits_len   = mem_axi.aw.len,
            o_mem_axi4_0_aw_bits_size  = mem_axi.aw.size,
            o_mem_axi4_0_aw_bits_burst = mem_axi.aw.burst,
            o_mem_axi4_0_aw_bits_lock  = mem_axi.aw.lock,
            o_mem_axi4_0_aw_bits_cache = mem_axi.aw.cache,
            o_mem_axi4_0_aw_bits_prot  = mem_axi.aw.prot,
            o_mem_axi4_0_aw_bits_qos   = mem_axi.aw.qos,

            i_mem_axi4_0_w_ready       = mem_axi.w.ready,
            o_mem_axi4_0_w_valid       = mem_axi.w.valid,
            o_mem_axi4_0_w_bits_data   = mem_axi.w.data,
            o_mem_axi4_0_w_bits_strb   = mem_axi.w.strb,
            o_mem_axi4_0_w_bits_last   = mem_axi.w.last,

            o_mem_axi4_0_b_ready       = mem_axi.b.ready,
            i_mem_axi4_0_b_valid       = mem_axi.b.valid,
            i_mem_axi4_0_b_bits_id     = mem_axi.b.id,
            i_mem_axi4_0_b_bits_resp   = mem_axi.b.resp,

            i_mem_axi4_0_ar_ready      = mem_axi.ar.ready,
            o_mem_axi4_0_ar_valid      = mem_axi.ar.valid,
            o_mem_axi4_0_ar_bits_id    = mem_axi.ar.id,
            o_mem_axi4_0_ar_bits_addr  = mem_axi.ar.addr,
            o_mem_axi4_0_ar_bits_len   = mem_axi.ar.len,
            o_mem_axi4_0_ar_bits_size  = mem_axi.ar.size,
            o_mem_axi4_0_ar_bits_burst = mem_axi.ar.burst,
            o_mem_axi4_0_ar_bits_lock  = mem_axi.ar.lock,
            o_mem_axi4_0_ar_bits_cache = mem_axi.ar.cache,
            o_mem_axi4_0_ar_bits_prot  = mem_axi.ar.prot,
            o_mem_axi4_0_ar_bits_qos   = mem_axi.ar.qos,

            o_mem_axi4_0_r_ready       = mem_axi.r.ready,
            i_mem_axi4_0_r_valid       = mem_axi.r.valid,
            i_mem_axi4_0_r_bits_id     = mem_axi.r.id,
            i_mem_axi4_0_r_bits_data   = mem_axi.r.data,
            i_mem_axi4_0_r_bits_resp   = mem_axi.r.resp,
            i_mem_axi4_0_r_bits_last   = mem_axi.r.last
        )
        self.cpu_params.update(mem_params)
        base = soc.mem_map["main_ram"]
        port = soc.sdram.crossbar.get_port(data_width=mem_width)
        self.submodules.axi2native = LiteDRAMAXI2Native(mem_axi, port, base_address=base)
        soc.add_memory_region("main_ram", base, size)
        if hasattr(soc, "with_busmasters") and soc.with_busmasters:
            # add dma channel
            dma_wb = wishbone.Interface()
            if use_axi:
                dma_axi = axi.AXIInterface(data_width=32, address_width=32, id_width=4)
                dma_params = dict(
                    # dma slave
                    o_l2_frontend_bus_axi4_0_aw_ready      = dma_axi.aw.ready,
                    i_l2_frontend_bus_axi4_0_aw_valid      = dma_axi.aw.valid,
                    i_l2_frontend_bus_axi4_0_aw_bits_id    = dma_axi.aw.id,
                    i_l2_frontend_bus_axi4_0_aw_bits_addr  = dma_axi.aw.addr,
                    i_l2_frontend_bus_axi4_0_aw_bits_len   = dma_axi.aw.len,
                    i_l2_frontend_bus_axi4_0_aw_bits_size  = dma_axi.aw.size,
                    i_l2_frontend_bus_axi4_0_aw_bits_burst = dma_axi.aw.burst,
                    i_l2_frontend_bus_axi4_0_aw_bits_lock  = dma_axi.aw.lock,
                    i_l2_frontend_bus_axi4_0_aw_bits_cache = dma_axi.aw.cache,
                    i_l2_frontend_bus_axi4_0_aw_bits_prot  = dma_axi.aw.prot,
                    i_l2_frontend_bus_axi4_0_aw_bits_qos   = dma_axi.aw.qos,

                    o_l2_frontend_bus_axi4_0_w_ready       = dma_axi.w.ready,
                    i_l2_frontend_bus_axi4_0_w_valid       = dma_axi.w.valid,
                    i_l2_frontend_bus_axi4_0_w_bits_data   = dma_axi.w.data,
                    i_l2_frontend_bus_axi4_0_w_bits_strb   = dma_axi.w.strb,
                    i_l2_frontend_bus_axi4_0_w_bits_last   = dma_axi.w.last,

                    i_l2_frontend_bus_axi4_0_b_ready       = dma_axi.b.ready,
                    o_l2_frontend_bus_axi4_0_b_valid       = dma_axi.b.valid,
                    o_l2_frontend_bus_axi4_0_b_bits_id     = dma_axi.b.id,
                    o_l2_frontend_bus_axi4_0_b_bits_resp   = dma_axi.b.resp,

                    o_l2_frontend_bus_axi4_0_ar_ready      = dma_axi.ar.ready,
                    i_l2_frontend_bus_axi4_0_ar_valid      = dma_axi.ar.valid,
                    i_l2_frontend_bus_axi4_0_ar_bits_id    = dma_axi.ar.id,
                    i_l2_frontend_bus_axi4_0_ar_bits_addr  = dma_axi.ar.addr,
                    i_l2_frontend_bus_axi4_0_ar_bits_len   = dma_axi.ar.len,
                    i_l2_frontend_bus_axi4_0_ar_bits_size  = dma_axi.ar.size,
                    i_l2_frontend_bus_axi4_0_ar_bits_burst = dma_axi.ar.burst,
                    i_l2_frontend_bus_axi4_0_ar_bits_lock  = dma_axi.ar.lock,
                    i_l2_frontend_bus_axi4_0_ar_bits_cache = dma_axi.ar.cache,
                    i_l2_frontend_bus_axi4_0_ar_bits_prot  = dma_axi.ar.prot,
                    i_l2_frontend_bus_axi4_0_ar_bits_qos   = dma_axi.ar.qos,

                    i_l2_frontend_bus_axi4_0_r_ready       = dma_axi.r.ready,
                    o_l2_frontend_bus_axi4_0_r_valid       = dma_axi.r.valid,
                    o_l2_frontend_bus_axi4_0_r_bits_id     = dma_axi.r.id,
                    o_l2_frontend_bus_axi4_0_r_bits_data   = dma_axi.r.data,
                    o_l2_frontend_bus_axi4_0_r_bits_resp   = dma_axi.r.resp,
                    o_l2_frontend_bus_axi4_0_r_bits_last   = dma_axi.r.last
                )
                self.cpu_params.update(dma_params)
                self.submodules.wb2axi = ResetInserter()(_Wishbone2AXI(dma_wb, dma_axi))
                self.comb += self.wb2axi.reset.eq(ResetSignal() | self.reset)
                # make rocket-chip peripherals accessible from LiteX
                soc.add_wb_slave(0x00000000, dma_wb, 0x10000000)
            else:
                # use another dram port. no access to rocket-chip peripherals.
                port = soc.sdram.crossbar.get_port()
                port_wb = wishbone.Interface(data_width=port.data_width)
                self.submodules.wbmem = LiteDRAMWishbone2Native(port_wb, port, base_address=base)
                self.submodules.wbcvt = wishbone.Converter(dma_wb, port_wb)
            soc.add_wb_slave(base, dma_wb, size)

    def build_dts(self, bootargs="", devices="//insert your devices here\n"):
        if len(bootargs):
            bootargs = " " + bootargs
        dtsname = CPU_VARIANTS[self.data_width][self.variant] + "Mem" + str(self.mem_width) + ".dts"
        with open(os.path.join(_get_gdir(), dtsname), "r") as f:
            dtslines = f.readlines()
        dram_search = "memory@" + hex(self.mem_map["main_ram"])[2:]
        inmemory1 = False
        inmemory2 = False
        dts = ""
        for i in dtslines:
            if i.find("timebase-frequency") > -1:
                pass
            elif i.find("cpus {") > -1:
                # insert before cpus section
                tabs = i.split("L", 1)[0]
                dts += tabs + "chosen {\n"
                dts += tabs + '\tbootargs = "earlycon=sbi console=hvc0 swiotlb=noforce' + bootargs + '";\n'
                dts += tabs + "};\n"
                dts += i
            elif i.find("cpu@0") > -1:
                # insert before cpu section
                dts += i.split("L", 1)[0]
                dts += "timebase-frequency = <" + str(self.clk_freq//100) + ">;\n"
                dts += i
            elif i.find("riscv,isa =") > -1:
                # cheat riscv-pk
                dts += i.split("=", 1)[0] + '= "rv' + str(self.data_width) + 'imafdc";\n'
            elif i.find("mmio-port-axi4@") > -1:
                # add our devices here
                inmemory1 = True
                dts += devices
            elif inmemory1 and i.find("};") > -1:
                inmemory1 = False
            elif i.find(dram_search) > -1:
                inmemory2 = True
                dts += i
            elif inmemory2 and i.find("reg =") > -1:
                # fix memory size
                inmemory2 = False
                dts += i.split("=", 1)[0] + "= <" + hex(self.mem_map["main_ram"]) + " " + hex(self.sdram_size) + ">;\n"
            elif not inmemory1:
                dts += i
        return dts


class Rocket64(Rocket):
    name                 = "rocket64"
    data_width           = 64
    endianness           = "little"
    gcc_triple           = ("riscv64-unknown-linux-gnu", "riscv64-unknown-elf")
    linker_output_format = "elf64-littleriscv"

    def __init__(self, platform, variant="standard"):
        Rocket.__init__(self, platform, variant)


class Rocket32(Rocket):
    name                 = "rocket32"
    data_width           = 32
    endianness           = "little"
    gcc_triple           = ("riscv64-unknown-linux-gnu", "riscv32-unknown-elf", "riscv-none-embed")
    linker_output_format = "elf32-littleriscv"

    def __init__(self, platform, variant="standard"):
        Rocket.__init__(self, platform, variant)


def _get_gdir():
    return os.path.join(os.path.abspath(os.path.dirname(__file__)), "verilog", "generated-src")


class _Wishbone2AXI(Module):

    def __init__(self, slave_wb, master_axi):

        # assume a 32bit litex system here
        assert slave_wb.adr_width == 30 and slave_wb.data_width == 32

        # # #

        self.comb += [
            # write
            master_axi.aw.size.eq(0b010),
            master_axi.aw.addr.eq(Cat(0, 0, slave_wb.adr)),
            master_axi.w.last.eq(1),
            master_axi.w.strb.eq(slave_wb.sel),
            master_axi.w.data.eq(slave_wb.dat_w),
            # read
            master_axi.ar.size.eq(0b010),
            master_axi.ar.addr.eq(Cat(0, 0, slave_wb.adr)),
            slave_wb.dat_r.eq(master_axi.r.data),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(slave_wb.cyc & slave_wb.stb,
                If(slave_wb.we,
                    NextValue(master_axi.aw.valid, 1),
                    NextValue(master_axi.w.valid, 1),
                    NextState("WRITE")
                ).Else(
                    NextValue(master_axi.ar.valid, 1),
                    NextState("READ")
                )
            )
        )
        fsm.act("WRITE",
            If(master_axi.aw.ready,
                NextValue(master_axi.aw.valid, 0)
            ),
            If(master_axi.w.ready,
                NextValue(master_axi.w.valid, 0)
            ),
            If(master_axi.b.valid,
                master_axi.b.ready.eq(1),
                slave_wb.ack.eq(1),
                slave_wb.err.eq(master_axi.b.resp != 0b00),
                NextState("IDLE")
            )
        )
        fsm.act("READ",
            If(master_axi.ar.ready,
                NextValue(master_axi.ar.valid, 0)
            ),
            If(master_axi.r.valid,
                master_axi.r.ready.eq(1),
                slave_wb.ack.eq(1),
                slave_wb.err.eq(master_axi.r.resp != 0b00),
                NextState("IDLE")
            )
        )

