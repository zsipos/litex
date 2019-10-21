# litex/soc/cores/cpu/rocket2/core.py
# Rocket Chip core support for the LiteX SoC.
#
# Copyright (c) 2019, Stefan Adams <stefan.adams@vipcomag.de>
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

CPU_VARIANTS = {
    32 : {
        "linux"          : "freechips.rocketchip.system.LitexLinuxConfig32",
        "linux+dualcore" : "freechips.rocketchip.system.LitexLinuxConfigDualCore32"
    },
    64 : {
        "linux"          : "freechips.rocketchip.system.LitexLinuxConfig64",
        "linux+dualcore" : "freechips.rocketchip.system.LitexLinuxConfigDualCore64",
        "full"           : "freechips.rocketchip.system.LitexFullConfig64",
    }
}

GCC_FLAGS = {
    32 : {
        "linux"          : "-march=rv32imac   -mabi=ilp32 ",
        "linux+dualcore" : "-march=rv32imac   -mabi=ilp32 ",
    },
    64 : {
        "linux"          : "-march=rv64imac   -mabi=lp64 ",
        "linux+dualcore" : "-march=rv64imac   -mabi=lp64 ",
        "full"           : "-march=rv64imafdc -mabi=lp64 ",
    }
}

class Rocket(CPU):
    io_regions = {0x20000000:0x60000000} # origin, length

    @property
    def mem_map(self):
        # Rocket reserves the first 256Mbytes for internal use, so we must change default mem_map.
        return {
            "rom"      : 0x10000000,
            "sram"     : 0x11000000,
            "csr"      : 0x20000000,
            "ethmac"   : 0x30000000,
            "main_ram" : 0x80000000,
        }

    @property
    def gcc_flags(self):
        flags =  "-mno-save-restore "
        flags += GCC_FLAGS[self.data_width][self.variant]
        flags += "-D__rocket__ "
        flags += "-D__rocket" + str(self.data_width) + "__ "
        return flags

    def __init__(self, platform, variant):

        assert variant in CPU_VARIANTS[self.data_width], "Unsupported variant %s" % variant

        self.platform  = platform
        self.variant   = variant
        self.reset     = Signal()
        self.interrupt = Signal(8)

        # memory bus for bios
        self.mem_axi   = mem_axi  = axi.AXIInterface(data_width=32, address_width=32, id_width=4)
        self.mmio_axi  = mmio_axi = axi.AXIInterface(data_width=32, address_width=32, id_width=4)

        self.mem_wb    = mem_wb  = wishbone.Interface(data_width=32, adr_width=30)
        self.mmio_wb   = mmio_wb = wishbone.Interface(data_width=32, adr_width=30)

        self.buses     = [mem_wb, mmio_wb]

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

            # axi memory (L1-cached)
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
            i_mem_axi4_0_r_bits_last   = mem_axi.r.last,

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

        # adapt axi interfaces to wishbone
        mem_a2w  = ResetInserter()(axi.AXI2Wishbone(mem_axi,  mem_wb , base_address=0))
        mmio_a2w = ResetInserter()(axi.AXI2Wishbone(mmio_axi, mmio_wb, base_address=0))

        # NOTE: AXI2Wishbone FSMs must be reset with the CPU!
        self.comb += [
            mem_a2w.reset.eq (ResetSignal() | self.reset),
            mmio_a2w.reset.eq(ResetSignal() | self.reset),
        ]

        self.submodules += mem_a2w, mmio_a2w

        # add verilog sources
        self.platform = platform
        self.variant  = variant

    def set_reset_address(self, reset_address):
        assert not hasattr(self, "reset_address")
        self.reset_address = reset_address
        assert reset_address == 0x10000000, "cpu_reset_addr hardcoded in during elaboration!"

    def add_sources(self, mem_width):
        basename = CPU_VARIANTS[self.data_width][self.variant] + "Mem" + str(mem_width)
        self.platform.add_sources(
            _get_gdir(),
            basename + ".v",
            basename + ".behav_srams.v",
        )
        self.platform.add_sources(
            os.path.join(_get_vdir(), "vsrc"),
            "plusarg_reader.v",
            "AsyncResetReg.v",
            "EICG_wrapper.v",
        )

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        assert hasattr(self, "axi2native")
        self.specials += Instance("LitexRocketSystem", **self.cpu_params)

    def connect_sdram(self, soc):
        mem_width = soc.sdram.crossbar.controller.data_width
        if mem_width < 64: mem_width = 64
        self.mem_width = mem_width

        # add sources
        self.add_sources(mem_width)

        # add sdram channel
        mem2_axi = axi.AXIInterface(data_width=mem_width, address_width=32, id_width=4)

        mem2_params = dict(
            # axi memory2 (L1-cached)
            i_mem2_axi4_0_aw_ready      = mem2_axi.aw.ready,
            o_mem2_axi4_0_aw_valid      = mem2_axi.aw.valid,
            o_mem2_axi4_0_aw_bits_id    = mem2_axi.aw.id,
            o_mem2_axi4_0_aw_bits_addr  = mem2_axi.aw.addr,
            o_mem2_axi4_0_aw_bits_len   = mem2_axi.aw.len,
            o_mem2_axi4_0_aw_bits_size  = mem2_axi.aw.size,
            o_mem2_axi4_0_aw_bits_burst = mem2_axi.aw.burst,
            o_mem2_axi4_0_aw_bits_lock  = mem2_axi.aw.lock,
            o_mem2_axi4_0_aw_bits_cache = mem2_axi.aw.cache,
            o_mem2_axi4_0_aw_bits_prot  = mem2_axi.aw.prot,
            o_mem2_axi4_0_aw_bits_qos   = mem2_axi.aw.qos,

            i_mem2_axi4_0_w_ready       = mem2_axi.w.ready,
            o_mem2_axi4_0_w_valid       = mem2_axi.w.valid,
            o_mem2_axi4_0_w_bits_data   = mem2_axi.w.data,
            o_mem2_axi4_0_w_bits_strb   = mem2_axi.w.strb,
            o_mem2_axi4_0_w_bits_last   = mem2_axi.w.last,

            o_mem2_axi4_0_b_ready       = mem2_axi.b.ready,
            i_mem2_axi4_0_b_valid       = mem2_axi.b.valid,
            i_mem2_axi4_0_b_bits_id     = mem2_axi.b.id,
            i_mem2_axi4_0_b_bits_resp   = mem2_axi.b.resp,

            i_mem2_axi4_0_ar_ready      = mem2_axi.ar.ready,
            o_mem2_axi4_0_ar_valid      = mem2_axi.ar.valid,
            o_mem2_axi4_0_ar_bits_id    = mem2_axi.ar.id,
            o_mem2_axi4_0_ar_bits_addr  = mem2_axi.ar.addr,
            o_mem2_axi4_0_ar_bits_len   = mem2_axi.ar.len,
            o_mem2_axi4_0_ar_bits_size  = mem2_axi.ar.size,
            o_mem2_axi4_0_ar_bits_burst = mem2_axi.ar.burst,
            o_mem2_axi4_0_ar_bits_lock  = mem2_axi.ar.lock,
            o_mem2_axi4_0_ar_bits_cache = mem2_axi.ar.cache,
            o_mem2_axi4_0_ar_bits_prot  = mem2_axi.ar.prot,
            o_mem2_axi4_0_ar_bits_qos   = mem2_axi.ar.qos,

            o_mem2_axi4_0_r_ready       = mem2_axi.r.ready,
            i_mem2_axi4_0_r_valid       = mem2_axi.r.valid,
            i_mem2_axi4_0_r_bits_id     = mem2_axi.r.id,
            i_mem2_axi4_0_r_bits_data   = mem2_axi.r.data,
            i_mem2_axi4_0_r_bits_resp   = mem2_axi.r.resp,
            i_mem2_axi4_0_r_bits_last   = mem2_axi.r.last,
        )
        self.cpu_params.update(mem2_params)
        port = soc.sdram.crossbar.get_port(data_width=mem_width)
        axi2native = LiteDRAMAXI2Native(mem2_axi, port)
        self.submodules.axi2native = axi2native

    def build_dts(self,
                  bootargs="",
                  sdram_size=0x80000000,
                  timebase_frequency=600000,
                  devices="//insert your devices here\n"):
        if len(bootargs):
            bootargs = " " + bootargs
        dtsname = CPU_VARIANTS[self.data_width][self.variant] + "Mem" + str(self.mem_width) + ".dts"
        with open(os.path.join(_get_gdir(), dtsname), "r") as f:
            dtslines = f.readlines()
        dram_search = "memory@" + hex(self.mem_map["main_ram"])[2:]
        bios_search = "memory@" + hex(self.mem_map["rom"])[2:]
        # find dram label
        for i in dtslines:
            if i.find(dram_search) > -1:
                dramlabel = i.split(":", 1)[0].lstrip()
                break
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
                dts += tabs + "};\n";
                dts += i
            elif i.find("cpu@0") > -1:
                # insert before cpu section
                dts += i.split("L", 1)[0]
                dts += "timebase-frequency = <" + str(timebase_frequency) + ">;\n"
                dts += i
            elif i.find("next-level-cache =") > -1:
                # bios rom is of no interest for linux
                dts += i.split("=", 1)[0] + "= <&" + dramlabel + ">;\n"
            elif i.find("riscv,isa =") > -1:
                # cheat riscv-pk
                dts += i.split("=", 1)[0] + '= "rv' + str(self.data_width) + 'imafdc";\n'
            elif i.find("mmio-port-axi4@") > -1:
                # add our devices here
                inmemory1 = True
                dts += devices
            elif i.find(bios_search) > -1:
                inmemory1 = True
            elif inmemory1 and i.find("};") > -1:
                inmemory1 = False
            elif i.find(dram_search) > -1:
                inmemory2 = True
                dts += i
            elif inmemory2 and i.find("reg =") > -1:
                # fix memory size
                inmemory2 = False
                dts += i.split("=", 1)[0] + "= <" + hex(self.mem_map["main_ram"]) + " " + hex(sdram_size) + ">;\n"
            elif not inmemory1:
                dts += i
        return dts


class Rocket64(Rocket):
    name                 = "rocket64"
    data_width           = 64
    endianness           = "little"
    gcc_triple           = ("riscv64-unknown-linux-gnu")
    linker_output_format = "elf64-littleriscv"

    def __init__(self, platform, variant="linux"):
        Rocket.__init__(self, platform, variant)


class Rocket32(Rocket):
    name                 = "rocket32"
    data_width           = 32
    endianness           = "little"
    gcc_triple           = ("riscv64-unknown-linux-gnu")
    linker_output_format = "elf32-littleriscv"

    def __init__(self, platform, variant="linux"):
        Rocket.__init__(self, platform, variant)


def _get_vdir():
    return os.path.join(os.path.abspath(os.path.dirname(__file__)), "verilog")

def _get_gdir():
    return os.path.join(_get_vdir(), "rocket-chip", "vsim", "generated-src")
