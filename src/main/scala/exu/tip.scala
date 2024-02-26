package boom.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tile.{TraceBundle}
import freechips.rocketchip.util.{CoreMonitorBundle}
import boom.common._
import boom.ifu.{HasBoomFrontendParameters}
import boom.exu.FUConstants._

class TipFlushes(implicit p: Parameters) extends BoomBundle {
    val exception = Bool()
    val flush = Bool()
    val mispredicted = Bool()
}

class OIR(implicit p: Parameters) extends BoomBundle {
    val inst = UInt(32.W)
    val flushes = new TipFlushes
}

class TipException(implicit p: Parameters) extends BoomBundle {
    val valid = Bool()
    val badvaddr = UInt(xLen.W)
    // val cause = UInt(xLen.W)
    val inst = UInt(32.W)
}

class TipReg(implicit p: Parameters) extends BoomBundle {
    val insts = Vec(retireWidth, UInt(32.W))
    val valids = Vec(retireWidth, Bool())
    val oldestId = UInt(log2Ceil(retireWidth).W)
  
    val stalled = Bool()
    val frontend = Bool()
    val flushes = new TipFlushes
    val sample_valid = Bool()
}

class TipIo(implicit p: Parameters) extends BoomBundle {
    val arch_valids = Input(Vec(retireWidth, Bool()))
    val instr_valids = Input(Vec(retireWidth, Bool()))
    val misspeculated = Input(Vec(retireWidth, Bool()))
    val exception = Input(new TipException)
    val uops = Input(Vec(retireWidth, new MicroOp()))
    val cpu_cycle = Input(UInt(64.W))

    val out = Output(new TipReg)
}

class Tip(implicit p: Parameters) extends BoomModule {
    val io = IO(new TipIo)

    /* out: public registers interact with outside, valid iff out.sample_valid */
    val out = RegInit(WireDefault(0.U.asTypeOf(new TipReg)))
    io.out := out

    /* offending instruction register*/
    val ori = RegInit(WireDefault(0.U.asTypeOf(new OIR)))
    /* are we still in drain state, waiting for the first dispatch instruction */
    val waiting_for_dispatching = RegInit(false.B)
    /* instr addrs of rob head row */
    // val addrs = VecInit(io.uops.map(_.debug_pc(vaddrBits - 1, 0)))
    /* is these instr flush the pipeline? */
    // val flush_on_commits = VecInit(io.uops.map(_.flush_on_commit))
    /* at least one instr is committing*/
    val committing = io.arch_valids.reduce(_ || _)
    /* at least one instr is valid */
    val rob_not_empty = io.instr_valids.reduce(_ || _)
    
    
    /* computing: rob is not empty and at least one instr is *committing* */
    val is_computing = rob_not_empty && committing
    /* stalled: rob is not empty and no instr is *committing* */
    val is_stalled = rob_not_empty && !committing 
    /* flush: rob is empty and ori denotes this flush is caused by mispredicted/flush_on_commit/exception */
    val ori_flush = ori.flushes.flush || ori.flushes.mispredicted || ori.flushes.exception
    val is_flushes = !rob_not_empty && ori_flush
    /* drain: rob is empty and no flags in ori is true */
    val is_drain = !rob_not_empty && !ori_flush

     
    /* ori update logic:
     *  1. If at least one instruction is committing, checking the youngest committing instr.
     *     1) if mispredicted or flush_on_commit, set corresponding ori flags and instr addr.
     *     2) if normally committing, clear all ori flags, and set instr addr.
     * 2.  If no instruction is committing, checking whether instr at rob head cause an exception.
     *     1) if exception is valid, set corresponding ori flags and instr addr.
     *     2) else, do nothing.
     */

    /* Q: always update ori or only in mispredicted/flush/exception state?
     * A: always update ori. If only update in mispredicted/flush/exception state, consider the following situation:
     *    1. mispredicted instr commit. 
     *    2. ori update.
     *    3. normal instr commit. <- ori is not updated here, ori saves the mispredicted instr.
     *    4. icache miss.
     *    5. tip invoke sampling. <- tip checks the ori flags, identify this state as mispredicted(flush)
     *                               in fact, this state is drain.
     */ 

     
    /* youngest index is the last true in io.arch_valids */
    val reversedValidIndex = PriorityEncoder(io.arch_valids.reverse)
    val youngest_index = (io.arch_valids.length - 1).U - reversedValidIndex

    val youngest_mispredicted = io.misspeculated(youngest_index)
    
    val youngest_uop = io.uops(youngest_index)
    val youngest_flush = youngest_uop.flush_on_commit

    
    when(io.exception.valid) {
        ori.flushes.flush := false.B
        ori.flushes.exception := true.B
        ori.flushes.mispredicted := false.B
        ori.inst := io.exception.inst
    }.elsewhen(committing) {
        ori.flushes.flush := youngest_flush
        ori.flushes.exception := false.B
        ori.flushes.mispredicted := youngest_mispredicted
        ori.inst := youngest_uop.inst
    }

    /* now we update tip output register by different rob state */
    when(waiting_for_dispatching) {
    /* now we are waiting for the first dispatching instruction 
     *   1. flag-related register should be fronzen                  
     *   2. address-related register can be update normally          
     */
        assert(out.frontend) 
        when(rob_not_empty) {
            waiting_for_dispatching := false.B
            out.sample_valid := true.B
            out.insts := io.uops.map(_.inst) 
            out.valids := io.instr_valids
        }

    }.otherwise {
        when(is_computing) {

            out.insts := io.uops.map(_.inst) 
            out.valids := io.arch_valids

            out.flushes := 0.U.asTypeOf(new TipFlushes)
            out.stalled := false.B
            out.frontend := false.B

            out.sample_valid := true.B

        }.elsewhen(is_stalled) {
            /* fixme: bank pointer */

            val rob_valid = io.instr_valids.asUInt.orR
            val rob_valid_index = Mux(rob_valid, PriorityEncoder(io.instr_valids), 0.U)

            out.insts := io.uops.map(_.inst) 
            out.valids := io.instr_valids
            out.oldestId := rob_valid_index

            out.stalled := true.B
            out.flushes := 0.U.asTypeOf(new TipFlushes)
            out.frontend := false.B

            out.sample_valid := true.B

        }.elsewhen(is_flushes) {

            out.insts := VecInit(ori.inst +: Seq.fill(retireWidth - 1)(0.U(32.W)))
            out.valids := VecInit(true.B +: Seq.fill(retireWidth - 1)(false.B))
            out.oldestId := 0.U

            out.flushes := ori.flushes
            out.stalled := false.B
            out.frontend := false.B

            out.sample_valid := true.B

        }.otherwise { // drained
            out.frontend := true.B
            out.flushes := 0.U.asTypeOf(new TipFlushes)
            out.stalled := false.B

            out.sample_valid := false.B

            waiting_for_dispatching := true.B

            out.insts := VecInit(Seq.fill(retireWidth)(0.U(32.W)))
            out.valids := VecInit(Seq.fill(retireWidth)(false.B))
        }
    }
}
