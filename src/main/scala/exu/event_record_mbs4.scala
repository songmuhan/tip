package boom.exu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket._

import boom.common._
import boom.util._


class EventRecoderMBSRAM4IO(readWidth: Int)(implicit p: Parameters) extends BoomBundle
{
  val event_pc    = Input(UInt(32.W))
  val event_val   = Input(Bool())
  val read_addr   = Input(Vec(readWidth, Valid(UInt(10.W))))
  val read_data   = Output(Vec(readWidth, UInt(64.W)))
  val reset_recorder = Input(Bool())
  val thrInit     = Input(UInt(8.W))
  val thrStep     = Input(UInt(8.W))
  val thrMaxv     = Input(UInt(8.W))
}


//conftable and buffer has same bank number, and all are divided by pc
//sram has multi ways
class EventRecoder_MultiWaySRAM4(readWidth: Int, numMod: Int, numConf: Int, numBuffer: Int, ways: Int)(implicit p: Parameters) extends BoomModule
{

	val io = IO(new EventRecoderMBSRAM4IO(readWidth))
  val modIdxSz = log2Ceil(numMod)

  //event confidence
  val numConfEntries    = numConf
  val numWay            = ways
  val confCounterWidth  = 8
  val confTableRows     = numConfEntries/numWay
  val confIdxSz         = log2Ceil(numConfEntries)

  val bufferEntries     = numBuffer
  val pcAddrSz          = 28
  val dataWidth         = pcAddrSz + 4
  val bufferRows        = bufferEntries/numWay
  val bufferIdxSz       = log2Ceil(bufferEntries)
  val bufRowIdxSz       = log2Ceil(bufferRows)

  val init_confthresh = 16
  val threshold_init  = io.thrInit
  val threshold_step  = io.thrStep
  val threshold_maxv  = io.thrMaxv

  def tableidx(eventpc: UInt) = { eventpc(confIdxSz, 1) }
  def GetRowIdx(conf_idx: UInt): UInt = {
    if (numWay == 1) return conf_idx
    else return conf_idx >> log2Ceil(numWay).U
  }
  def GetWayIdx(conf_idx: UInt): UInt = {
    if(numWay == 1) { return 0.U }
    else  { return conf_idx(log2Ceil(numWay)-1, 0).asUInt }
  }

  val s0_eventpc  = io.event_pc
  val s0_eventval = io.event_val
  val s0_confidx  = tableidx(s0_eventpc)

  val s1_eventpc  = RegNext(s0_eventpc)
  val s1_eventval = RegNext(s0_eventval)
  val s1_confidx  = tableidx(s1_eventpc)
  val s1_bankidx  = GetWayIdx(s1_confidx)
  val s1_rowidx   = GetRowIdx(s1_confidx)

  val doing_conf_reset = RegInit(true.B)
  val doing_buf_reset  = RegInit(true.B)

  val conf_table  = SyncReadMem(confTableRows, Vec(numWay, UInt(confCounterWidth.W)))
  val needRecord  = WireInit(false.B)

  val cur_confThr = RegInit(VecInit(Seq.fill(numWay){init_confthresh.U(confCounterWidth.W)}))
  val s1_confcnts = conf_table.read(GetRowIdx(s0_confidx), s0_eventval)
  val s1_conf     = s1_confcnts(s1_bankidx)

  needRecord := false.B
  when (!doing_conf_reset && s1_eventval) {
    needRecord := (s1_conf === cur_confThr(s1_bankidx))
    val newval = Mux(needRecord, 0.U, s1_conf + 1.U)
    conf_table.write(s1_rowidx, VecInit(Seq.fill(numWay)(newval)), Seq.tabulate(numWay)(s1_bankidx === _.U))
  }


  //---------------------------------------------------------------------//
  val event_buffer  = SyncReadMem(bufferRows, Vec(numWay, UInt(dataWidth.W)))
  val bufidxs       = RegInit(VecInit(Seq.fill(numWay){0.U(bufRowIdxSz.W)})) 

  when (!doing_buf_reset && s1_eventval && needRecord) {
    val newval = Cat(cur_confThr(s1_bankidx)(7,4), s1_eventpc(pcAddrSz-1, 0))
    event_buffer.write(bufidxs(s1_bankidx), VecInit(Seq.fill(numWay)(newval)), Seq.tabulate(numWay)(s1_bankidx === _.U))
    when (bufidxs(s1_bankidx) === (bufferRows-1).U) {
      bufidxs(s1_bankidx) := 0.U
      when (cur_confThr(s1_bankidx) < threshold_maxv) {
        cur_confThr(s1_bankidx) := cur_confThr(s1_bankidx) + threshold_step
      }
    }
    .otherwise {
      bufidxs(s1_bankidx) := bufidxs(s1_bankidx) + 1.U
    }
  }


  //---------------------------------------------------------------------//
  val reg_read_data = Reg(Vec(readWidth, UInt(dataWidth.W)))
  for (i <- 0 until readWidth) {
    val ridx = io.read_addr(i).bits(bufferIdxSz-1, 0)
    val rbankidx = GetWayIdx(ridx)
    val rdata = event_buffer.read(GetRowIdx(ridx), io.read_addr(i).valid)

    when (io.read_addr(i).valid) {
      reg_read_data(i) := rdata(rbankidx)
    }
    .otherwise {
      reg_read_data(i) := 0.U
    }
  }

  for(i <- 0 until readWidth){
		io.read_data(i) := Cat(0.U((64-dataWidth).W), RegNext(reg_read_data(i)))
	}

  when (io.reset_recorder) {
    for (w <- 0 until numWay) {
      cur_confThr(w) := threshold_init
      bufidxs(w) := 0.U
    }
    doing_conf_reset  := true.B 
    doing_buf_reset   := true.B 
  }

  val reset_conf_idx = RegInit(0.U(log2Ceil(confTableRows).W))
  reset_conf_idx := reset_conf_idx + doing_conf_reset
  when (reset_conf_idx === (confTableRows-1).U) { doing_conf_reset := false.B }
  when (doing_conf_reset) { 
    conf_table.write(reset_conf_idx, VecInit(Seq.fill(numWay)(0.U)), (~(0.U(numWay.W))).asBools)
  }

  val reset_buf_idx = RegInit(0.U(bufRowIdxSz.W))
  reset_buf_idx := reset_buf_idx + doing_buf_reset
  when (reset_buf_idx === (bufferRows-1).U) { doing_buf_reset := false.B }
  when (doing_buf_reset) { 
    event_buffer.write(reset_buf_idx, VecInit(Seq.fill(numWay)(0.U)), (~(0.U(numWay.W))).asBools)
  }


  override def toString: String = BoomCoreStringPrefix(
    "== Event_Recorder (SRAM) ==",
    "numWay             : " + numWay,
    "numConfEntries     : " + numConfEntries,
    "confCounterWidth   : " + confCounterWidth,
    "confIdxSz          : " + confIdxSz,
    "bufferEntries      : " + bufferEntries,
    "bufferIdxSz        : " + bufferIdxSz,
    "pcAddrSz           : " + pcAddrSz)
}


class EventRecoders_SRAM1(readWidth: Int)(implicit p: Parameters) extends BoomModule
{
  val io = IO(new EventRecoderMBSRAM4IO(readWidth))
  val numMod    = 8
  val modIdxSz  = log2Ceil(numMod)
  val numConf   = 128
  val numBuffer = 32
  val numWays   = 4
  val confIdxSz = log2Ceil(numConf)
  val recorders = for (w <- 0 until numMod) yield { val er = Module(new EventRecoder_MultiWaySRAM4(readWidth, numMod, numConf, numBuffer, numWays)); er }

  for (w <- 0 until numMod) {
    recorders(w).io.reset_recorder := io.reset_recorder
    recorders(w).io.thrInit := io.thrInit
    recorders(w).io.thrStep := io.thrStep
    recorders(w).io.thrMaxv := io.thrMaxv
  }

  def getModidx(eventpc: UInt) = { eventpc(confIdxSz+modIdxSz, confIdxSz+1) }
  for (w <- 0 until numMod) {
    val midx = getModidx(io.event_pc)
    recorders(w).io.event_pc := io.event_pc
    recorders(w).io.event_val := io.event_val && (midx === w.U)
  }

  
  for (n <- 0 until readWidth) {
    val midx = io.read_addr(n).bits(modIdxSz-1, 0)
    val ridx = Cat(0.U(modIdxSz.W), io.read_addr(n).bits(9, modIdxSz))
    io.read_data(n) := 0.U

    for (w <- 0 until numMod) {
      recorders(w).io.read_addr(n).valid := io.read_addr(n).valid && midx === w.U
      recorders(w).io.read_addr(n).bits := ridx

      when (RegNext(RegNext(midx)) === w.U) {
        io.read_data(n) := recorders(w).io.read_data(n)
      }
    }
  }
}
