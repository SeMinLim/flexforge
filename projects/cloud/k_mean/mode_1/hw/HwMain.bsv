import FIFO::*;
import FIFOF::*;
import Clocks::*;
import Vector::*;

import Serializer::*;

import BRAM::*;
import BRAMFIFO::*;

import PcieCtrl::*;

import DRAMController::*;
import DRAMArbiterRemote::*;

import FloatingPoint::*;
import Float32::*;

import Kmean::*;


typedef 131072 TotalByte; // 32*1024*4
typedef 2048 TotalWords; // 32*64
typedef 32 NumData;
typedef 1024 Dimension;


interface HwMainIfc;
endinterface
module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram) (HwMainIfc);

	Clock curClk <- exposeCurrentClock;
	Reset curRst <- exposeCurrentReset;

	Clock pcieclk = pcie.user_clk;
	Reset pcierst = pcie.user_rst;

	// Cycle Counter
	FIFOF#(Bit#(32)) cycleQ <- mkFIFOF;
	Reg#(Bit#(32)) cycleCount <- mkReg(0);
	Reg#(Bit#(32)) cycleStart <- mkReg(0);
	Reg#(Bit#(32)) cycleEnd <- mkReg(0);
	rule incCycleCount;
		cycleCount <= cycleCount + 1;
	endrule

	// Serializer
	DeSerializerIfc#(32, 16) deserializer32b <- mkDeSerializer;

	// DRAM Arbiter
	DRAMArbiterRemoteIfc#(2) dramArbiter <- mkDRAMArbiterRemote(dram);
	
	// K-mean App
	KmeanIfc kmean <- mkKmean;
	//--------------------------------------------------------------------------------------
	// Pcie Read and Write
	//--------------------------------------------------------------------------------------
	SyncFIFOIfc#(Tuple2#(IOReadReq, Bit#(32))) pcieRespQ <- mkSyncFIFOFromCC(2048, pcieclk);
	SyncFIFOIfc#(IOReadReq) pcieReadReqQ <- mkSyncFIFOToCC(2048, pcieclk, pcierst);
	SyncFIFOIfc#(IOWrite) pcieWriteQ <- mkSyncFIFOToCC(2048, pcieclk, pcierst);
	rule getReadReq;
		let r <- pcie.dataReq;
		pcieReadReqQ.enq(r);
	endrule
	rule returnReadResp;
		let r_ = pcieRespQ.first;
		pcieRespQ.deq;

		pcie.dataSend(tpl_1(r_), tpl_2(r_));
	endrule
	rule getWriteReq;
		let w <- pcie.dataReceive;
		pcieWriteQ.enq(w);
	endrule
	//--------------------------------------------------------------------------------------------
	// Get Commands from Host via PCIe
	//--------------------------------------------------------------------------------------------
	FIFOF#(Bit#(32)) statusCheckerQ <- mkFIFOF;
	// Initial Setting
	Reg#(Bool) stage1 <- mkReg(False);
	// DRAM Reader
	Reg#(Bool) stage2 <- mkReg(False);
	// Data Receiver
	Reg#(Bool) stage3 <- mkReg(False);
	rule getCmd;
		pcieWriteQ.deq;
		let w = pcieWriteQ.first;

		let d = w.data;
		let a = w.addr;
		let off = (a >> 2);

		if ( off == 0 ) begin
			stage1 <= True;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Initial setting start!\033[0m\n", cycleCount);
		end else if ( off == 1 ) begin
			deserializer32b.put(d);
		end else if ( off == 2 ) begin
			stage2 <= True;
			stage3 <= True;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: System start!\033[0m\n", cycleCount);
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 1 (Initial Setting)
	//
	// This stage writes the data take from the host through PCIe to DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(32)) stage1Cnt <- mkReg(0); // Good!!!
	rule dramWriterInit( stage1 );
		if ( stage1Cnt != 0 ) begin
			let payload <- deserializer32b.get;
			dramArbiter.access[0].users[0].write(payload);
			if ( stage1Cnt == fromInteger(valueOf(TotalWords)) ) begin
				stage1Cnt <= 0;
				stage1 <= False;

				statusCheckerQ.enq(1); // Check dram writer initial set done
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Initial setting \033[1;32mdone!\033[0m\n", cycleCount);
			end else begin
				stage1Cnt <= stage1Cnt + 1;
			end
		end else begin
			dramArbiter.access[0].users[0].cmd(0, fromInteger(valueOf(TotalWords)), True);
			stage1Cnt <= stage1Cnt + 1;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 2 (DRAM Reader)
	//
	// This stage read the data from DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(32)) stage2Cnt <- mkReg(0); // Good!!!
	rule dramReader( stage2 );
		if ( stage2Cnt != 0 ) begin
			let payload <- dramArbiter.access[0].users[0].read;
			kmean.dataIn(payload);
				
			if ( stage2Cnt == fromInteger(valueOf(TotalWords)) ) begin
				stage2Cnt <= 0;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 1024-dimension data \033[1;32mdone!\033[0m\n", cycleCount);
				stage2 <= False;
			end else begin
				stage2Cnt <= stage2Cnt + 1;
			end
		end else begin
			dramArbiter.access[0].users[0].cmd(0, fromInteger(valueOf(TotalWords)), False);
			stage2Cnt <= stage2Cnt + 1;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024-dimension data \033[1;32mstart!\033[0m\n", cycleCount);
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 3 (Data Receiver)
	//
	// This stage receives status data from K-mean App
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(32)) stage3Cnt_1 <- mkReg(0);
	Reg#(Bit#(32)) stage3Cnt_2 <- mkReg(0);
	rule dataReceiver( stage3 );
		let d <- kmean.dataOut;
		if ( stage3Cnt_1 + 1 == fromInteger(valueOf(PeWays)) ) begin
			if ( stage3Cnt_2 + 1 == fromInteger(valueOf(NumData)) ) begin
				statusCheckerQ.enq(1);
				cycleQ.enq(cycleCount);
				stage3Cnt_2 <= 0;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: System \033[1;32m finish!\033[0m\n", cycleCount);
			end else begin
				stage3Cnt_2 <= stage3Cnt_2 + 1;
			end
			stage3Cnt_1 <= 0;
		end else begin
			stage3Cnt_1 <= stage3Cnt_1 + 1;
		end
	endrule
	//-------------------------------------------------------------------------------------------------
	// Stage 4 (Status Check)
	//
	// This stage checks the status of the system and relays it to the host
	//-------------------------------------------------------------------------------------------------
	rule getStatus;
		pcieReadReqQ.deq;
		let r = pcieReadReqQ.first;
		Bit#(4) a = truncate(r.addr>>2);
		if ( a == 0 ) begin
			if ( statusCheckerQ.notEmpty ) begin
				pcieRespQ.enq(tuple2(r, statusCheckerQ.first));
				statusCheckerQ.deq;
			end else begin
				pcieRespQ.enq(tuple2(r, 32'hffffffff));
			end
		end else if ( a == 1 ) begin
			if ( cycleQ.notEmpty ) begin
				pcieRespQ.enq(tuple2(r, cycleQ.first));
				cycleQ.deq;
			end else begin
				pcieRespQ.enq(tuple2(r, 32'hffffffff));
			end
		end
	endrule
endmodule
