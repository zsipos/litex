// See LICENSE.SiFive for license details.
// See LICENSE.Berkeley for license details.

package freechips.rocketchip.system

import Chisel._
import freechips.rocketchip.config.Config
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket.{DCacheParams, ICacheParams, MulDivParams, RocketCoreParams}
import freechips.rocketchip.tile.RocketTileParams

class WithLitexMemPorts extends Config((site, here, up) => {
  case ExtMem => Some(MemoryPortParams(MasterPortParams(
    base = x"1000_0000", //litex bios base
    size = x"2000_0000", //up to io range
    beatBytes = site(MemoryBusKey).beatBytes,
    idBits = 4), 1))
  case ExtMem2 => Some(MemoryPortParams(MasterPortParams(
    base = x"4000_0000", //litex dram base
    size = x"4000_0000", //up to litex shadow base
    beatBytes = site(MemoryBusKey).beatBytes,
    idBits = 4), 1))
})

class WithLitexMMIOPort extends Config((site, here, up) => {
  case ExtBus => Some(MasterPortParams(
    base = x"8000_0000", //litex shadow base
    size = x"8000_0000", //up to the end
    beatBytes = site(MemoryBusKey).beatBytes,
    idBits = 4))
})

class WithNMediumCores(n: Int) extends Config((site, here, up) => {
  case RocketTilesKey => {
    val med = RocketTileParams(
      core   = RocketCoreParams(fpu = None, mulDiv = Some(MulDivParams(
        mulUnroll = 8,
        mulEarlyOut = true,
        divEarlyOut = true))),
      dcache = Some(DCacheParams(
        nWays = 8,
        //nTLBEntries = 64,
        rowBits = site(SystemBusKey).beatBits,
        nMSHRs = 0,
        blockBytes = site(CacheBlockBytes))),
      icache = Some(ICacheParams(
        nWays = 8,
        //nTLBEntries = 64,
        rowBits = site(SystemBusKey).beatBits,
        blockBytes = site(CacheBlockBytes))))
    List.tabulate(n)(i => med.copy(hartId = i))
  }
})

class BaseLitexConfig extends Config(
  new WithLitexMemPorts() ++
  new WithLitexMMIOPort() ++
  new WithNoSlavePort ++
  new WithNExtTopInterrupts(8) ++
  new WithoutTLMonitors ++
  new BaseConfig
)

class LitexConfig extends Config(
  new WithNSmallCores(1) ++
  new BaseLitexConfig
)

class LitexLinuxConfig extends Config(
  new WithNMediumCores(1) ++
  new BaseLitexConfig
)

class LitexFullConfig extends Config(
  new WithNBigCores(1) ++
  new BaseLitexConfig
)
