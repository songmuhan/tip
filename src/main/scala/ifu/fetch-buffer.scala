//******************************************************************************
// Copyright (c) 2018 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Fetch Buffer
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//
// Takes a FetchBundle and converts into a vector of MicroOps.

package boom.ifu

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.{Parameters}
import freechips.rocketchip.rocket.{MStatus, BP, BreakpointUnit}

import boom.common._
import boom.util.{BoolToChar, MaskUpper}

/**
 * Bundle that is made up of converted MicroOps from the Fetch Bundle
 * input to the Fetch Buffer. This is handed to the Decode stage.
 */
class FetchBufferResp(implicit p: Parameters) extends BoomBundle
{
  val uops = Vec(coreWidth, Valid(new MicroOp()))
}

/**
 * Buffer to hold fetched packets and convert them into a vector of MicroOps
 * to give the Decode stage
 *
 * @param num_entries effectively the number of full-sized fetch packets we can hold.
 */
class FetchBuffer(implicit p: Parameters) extends BoomModule
  with HasBoomCoreParameters
  with HasBoomFrontendParameters
{
  val numEntries = numFetchBufferEntries
  val io = IO(new BoomBundle {
    val enq = Flipped(Decoupled(new FetchBundle()))
    val deq = new DecoupledIO(new FetchBufferResp())

    // Was the pipeline redirected? Clear/reset the fetchbuffer.
    val clear = Input(Bool())
  })

  require (numEntries > fetchWidth)
  require (numEntries % coreWidth == 0)
  val numRows = numEntries / coreWidth

  val ram = Reg(Vec(numEntries, new MicroOp))
  ram.suggestName("fb_uop_ram")
  val deq_vec = Wire(Vec(numRows, Vec(coreWidth, new MicroOp)))

  val head = RegInit(1.U(numRows.W))
  val tail = RegInit(1.U(numEntries.W))

  val maybe_full = RegInit(false.B)

  //-------------------------------------------------------------
  // **** Enqueue Uops ****
  //-------------------------------------------------------------
  // Step 1: Convert FetchPacket into a vector of MicroOps.
  // Step 2: Generate one-hot write indices.
  // Step 3: Write MicroOps into the RAM.

  def rotateLeft(in: UInt, k: Int) = {
    val n = in.getWidth
    Cat(in(n-k-1,0), in(n-1, n-k))
  }

  val might_hit_head = (1 until fetchWidth).map(k => VecInit(rotateLeft(tail, k).asBools.zipWithIndex.filter
    {case (e,i) => i % coreWidth == 0}.map {case (e,i) => e}).asUInt).map(tail => head & tail).reduce(_|_).orR
  val at_head = (VecInit(tail.asBools.zipWithIndex.filter {case (e,i) => i % coreWidth == 0}
    .map {case (e,i) => e}).asUInt & head).orR
  val do_enq = !(at_head && maybe_full || might_hit_head)

  io.enq.ready := do_enq

  // Input microops.
  val in_mask = Wire(Vec(fetchWidth, Bool()))
  val in_uops = Wire(Vec(fetchWidth, new MicroOp()))

  // Step 1: Convert FetchPacket into a vector of MicroOps.
  for (b <- 0 until nBanks) {
    for (w <- 0 until bankWidth) {
      val i = (b * bankWidth) + w

      val pc = (bankAlign(io.enq.bits.pc) + (i << 1).U)

      in_uops(i)                := DontCare
      in_mask(i)                := io.enq.valid && io.enq.bits.mask(i)
      in_uops(i).edge_inst      := false.B
      in_uops(i).debug_pc       := pc
      in_uops(i).pc_lob         := pc

      in_uops(i).is_sfb         := io.enq.bits.sfbs(i) || io.enq.bits.shadowed_mask(i)

      if (w == 0) {
        when (io.enq.bits.edge_inst(b)) {
          in_uops(i).debug_pc  := bankAlign(io.enq.bits.pc) + (b * bankBytes).U - 2.U
          in_uops(i).pc_lob    := bankAlign(io.enq.bits.pc) + (b * bankBytes).U
          in_uops(i).edge_inst := true.B
        }
      }
      in_uops(i).ftq_idx        := io.enq.bits.ftq_idx
      in_uops(i).inst           := io.enq.bits.exp_insts(i)
      in_uops(i).debug_inst     := io.enq.bits.insts(i)
      in_uops(i).is_rvc         := io.enq.bits.insts(i)(1,0) =/= 3.U
      in_uops(i).taken          := io.enq.bits.cfi_idx.bits === i.U && io.enq.bits.cfi_idx.valid

      in_uops(i).xcpt_pf_if     := io.enq.bits.xcpt_pf_if
      in_uops(i).xcpt_ae_if     := io.enq.bits.xcpt_ae_if
      in_uops(i).bp_debug_if    := io.enq.bits.bp_debug_if_oh(i)
      in_uops(i).bp_xcpt_if     := io.enq.bits.bp_xcpt_if_oh(i)
      
      in_uops(i).icache_req_cyl := io.enq.bits.icache_req_cyl
      in_uops(i).icache_resp_cyl:= io.enq.bits.icache_resp_cyl
      in_uops(i).debug_fsrc     := io.enq.bits.fsrc
      in_uops(i).tea_psv.itlb_pmiss     := false.B
      in_uops(i).tea_psv.itlb_smiss     := false.B
      in_uops(i).tea_psv.icache_miss    := false.B
      in_uops(i).tea_psv.memory_order_xcpt := false.B
    }
  }

   // Fill in miss information to the first instruction from the fetch bundle
  when(in_mask.reduce(_||_)) {
    val firstValid = PriorityEncoder(in_mask)
    in_uops(firstValid).tea_psv.icache_miss := io.enq.bits.icache_miss
    in_uops(firstValid).tea_psv.itlb_pmiss := io.enq.bits.itlb_pmiss
    in_uops(firstValid).tea_psv.itlb_smiss := io.enq.bits.itlb_smiss
    in_uops(firstValid).tea_psv.memory_order_xcpt := io.enq.bits.memory_order_xcpt
  }

  // It can happen that a previous fetch bundle carries a
  // iTLB or iCache miss event but no valid instruction
  // If the fist instruction of the next fetch bundle is an
  // edge instruction it might inherit the miss information
  val edge_icache_miss = RegInit(false.B)
  val edge_itlb_pmiss  = RegInit(false.B)
  val edge_itlb_smiss  = RegInit(false.B)
  val edge_memory_order_xcpt = RegInit(false.B)
  val edge_pc = Reg(UInt(coreMaxAddrBits.W))

  val edge_event_miss = (edge_icache_miss || edge_itlb_pmiss || edge_itlb_smiss || edge_memory_order_xcpt) && in_mask(0) && in_uops(0).edge_inst && (in_uops(0).debug_pc === edge_pc)

  when(edge_event_miss) {
    in_uops(0).tea_psv.itlb_pmiss  := io.enq.bits.itlb_pmiss || edge_itlb_pmiss
    in_uops(0).tea_psv.itlb_smiss  := io.enq.bits.itlb_smiss || edge_itlb_smiss
    in_uops(0).tea_psv.icache_miss := io.enq.bits.icache_miss || edge_icache_miss
    in_uops(0).tea_psv.memory_order_xcpt := io.enq.bits.memory_order_xcpt || edge_memory_order_xcpt
  }

  when (io.enq.valid && do_enq) {
    edge_itlb_pmiss  := false.B
    edge_itlb_smiss  := false.B
    edge_icache_miss := false.B
    edge_memory_order_xcpt := false.B
    when(!io.enq.bits.mask.orR() && (io.enq.bits.icache_miss || io.enq.bits.itlb_pmiss || io.enq.bits.itlb_smiss || io.enq.bits.memory_order_xcpt)) {
      edge_itlb_pmiss  := io.enq.bits.itlb_pmiss
      edge_itlb_smiss  := io.enq.bits.itlb_smiss
      edge_icache_miss := io.enq.bits.icache_miss
      edge_memory_order_xcpt   := io.enq.bits.memory_order_xcpt
      edge_pc := io.enq.bits.pc
    }
  }
  
  if (DEBUG_PRINTF) {
    def instrFromUOp(uop: MicroOp) = if (uop.is_rvc == true.B) uop.debug_inst(15, 0) else uop.debug_inst
    def pcFromUOp(uop: MicroOp): UInt = uop.debug_pc(vaddrBits-1,0)
    val debug_tsc_reg = RegInit(0.U(xLen.W))
    debug_tsc_reg := debug_tsc_reg + 1.U
    when(io.enq.valid && do_enq) {
      for (b <- 0 until nBanks) {
        for (w <- 0 until bankWidth) {
          val i = (b * bankWidth) + w
          when(io.enq.bits.mask(i)) {
            printf("%d | [FETCHB] | enqueue     | 0x%x DASM(0x%x)\n", debug_tsc_reg, pcFromUOp(in_uops(i)), instrFromUOp(in_uops(i)));
            printf("%d | [FETCHB] | i$ req:%d resp:%d | 0x%x DASM(0x%x)\n", debug_tsc_reg,in_uops(i).icache_req_cyl, in_uops(i).icache_resp_cyl, pcFromUOp(in_uops(i)), instrFromUOp(in_uops(i)));
            in_uops(i).fetch_buf_enq_cycle := debug_tsc_reg
          }
        }
      }
    }
  }


  // Step 2. Generate one-hot write indices.
  val enq_idxs = Wire(Vec(fetchWidth, UInt(numEntries.W)))

  def inc(ptr: UInt) = {
    val n = ptr.getWidth
    Cat(ptr(n-2,0), ptr(n-1))
  }

  var enq_idx = tail
  for (i <- 0 until fetchWidth) {
    enq_idxs(i) := enq_idx
    enq_idx = Mux(in_mask(i), inc(enq_idx), enq_idx)
  }

  // Step 3: Write MicroOps into the RAM.
  for (i <- 0 until fetchWidth) {
    for (j <- 0 until numEntries) {
      when (do_enq && in_mask(i) && enq_idxs(i)(j)) {
        ram(j) := in_uops(i)
      }
    }
  }

  //-------------------------------------------------------------
  // **** Dequeue Uops ****
  //-------------------------------------------------------------

  val tail_collisions = VecInit((0 until numEntries).map(i =>
                          head(i/coreWidth) && (!maybe_full || (i % coreWidth != 0).B))).asUInt & tail
  val slot_will_hit_tail = (0 until numRows).map(i => tail_collisions((i+1)*coreWidth-1, i*coreWidth)).reduce(_|_)
  val will_hit_tail = slot_will_hit_tail.orR

  val do_deq = io.deq.ready && !will_hit_tail

  val deq_valids = (~MaskUpper(slot_will_hit_tail)).asBools

  // Generate vec for dequeue read port.
  for (i <- 0 until numEntries) {
    deq_vec(i/coreWidth)(i%coreWidth) := ram(i)
  }

  io.deq.bits.uops zip deq_valids           map {case (d,v) => d.valid := v}
  io.deq.bits.uops zip Mux1H(head, deq_vec) map {case (d,q) => d.bits  := q}
  io.deq.valid := deq_valids.reduce(_||_)

  if (DEBUG_PRINTF) {
    def instrFromUOp(uop: MicroOp) = if (uop.is_rvc == true.B) uop.debug_inst(15, 0) else uop.debug_inst
    def pcFromUOp(uop: MicroOp): UInt = uop.debug_pc(vaddrBits-1,0)
    val debug_tsc_reg = RegInit(0.U(xLen.W))
    debug_tsc_reg := debug_tsc_reg + 1.U
    for (i <- 0 until coreWidth) {
        val uop = io.deq.bits.uops(i).bits
        when(deq_valids(i)) {
          printf("%d | [FETCHB] | dequeue | $:%x BP:%x | 0x%x DASM(0x%x)\n", debug_tsc_reg, uop.tea_psv.icache_miss.asUInt, uop.tea_psv.branch_miss.asUInt, pcFromUOp(uop), instrFromUOp(uop));
      }
    }
  }

  //-------------------------------------------------------------
  // **** Update State ****
  //-------------------------------------------------------------

  when (do_enq) {
    tail := enq_idx
    when (in_mask.reduce(_||_)) {
      maybe_full := true.B
    }
  }

  when (do_deq) {
    head := inc(head)
    maybe_full := false.B
  }

  when (io.clear) {
    head := 1.U
    tail := 1.U
    maybe_full := false.B
  }

  // TODO Is this necessary?
  when (reset.asBool) {
    io.deq.bits.uops map { u => u.valid := false.B }
  }

}
