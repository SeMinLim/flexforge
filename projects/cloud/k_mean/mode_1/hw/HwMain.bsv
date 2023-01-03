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

import Kmean::*;

Integer numData = 1024;
Integer totalByte = 3*1024*4;
Integer totalWords = 3*64;
Integer bramFifoSize = 1024;

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

	// BRAM
	BRAM_Configure cfg = defaultValue;
	cfg.memorySize = 1024*128;
	BRAM2Port#(Bit#(32), Vector#(4, Bit#(32))) bram <- mkBRAM2Server(cfg);
	
	// Serializer
	DeSerializerIfc#(32, 16) deserializer32b <- mkDeSerializer;
	DeSerializerIfc#(128, 4) deserializer128b <- mkDeSerializer;
	SerializerIfc#(512, 4) serializer128b <- mkSerializer;

	// DRAM Arbiter
	DRAMArbiterRemoteIfc#(2) dramArbiter <- mkDRAMArbiterRemote(dram);
	
	// K-mean App
	KmeanIfc kmean <- mkKmean;
	//--------------------------------------------------------------------------------------
	// Pcie Read and Write
	//--------------------------------------------------------------------------------------
	SyncFIFOIfc#(Tuple2#(IOReadReq, Bit#(32))) pcieRespQ <- mkSyncFIFOFromCC(1024, pcieclk);
	SyncFIFOIfc#(IOReadReq) pcieReadReqQ <- mkSyncFIFOToCC(1024, pcieclk, pcierst);
	SyncFIFOIfc#(IOWrite) pcieWriteQ <- mkSyncFIFOToCC(1024, pcieclk, pcierst);
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
	// Data Organizer
	Reg#(Bool) stage3 <- mkReg(False);
	// Data Relayer
	Reg#(Bool) stage4 <- mkReg(False);
	Reg#(Bool) bramWriteDone <- mkReg(False);
	// Data Receiver
	Reg#(Bool) stage5 <- mkReg(False);
	// DRAM Writer
	Reg#(Bool) stage6 <- mkReg(False);
	rule getCmd;
		pcieWriteQ.deq;
		let w = pcieWriteQ.first;

		let d = w.data;
		let a = w.addr;
		let off = (a >> 2);

		if ( off == 0 ) begin
			stage1 <= True; // Main Initializing
			kmean.initSet; // K-mean Initializing
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Initial setting start!\n", cycleCount);
		end else if ( off == 1 ) begin
			deserializer32b.put(d);
		end else if ( off == 2 ) begin
			stage2 <= True;
			stage3 <= True;
			stage4 <= True;
			stage5 <= True;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: System start!\n", cycleCount);
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Usage of the entire memory
	//  For Mode 1
	//  0 ~ 12,287
	//--------------------------------------------------------------------------------------------
	// Stage 1 (Initial Setting)
	//
	// This stage writes the data take from the host through PCIe to DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(32)) dramWrInitCnt <- mkReg(0); // Perfect!!!
	rule dramWriterInit( stage1 );
		if ( dramWrInitCnt != 0 ) begin
			let payload <- deserializer32b.get;
			dramArbiter.access[0].users[0].write(payload);
			if ( dramWrInitCnt == fromInteger(totalWords) ) begin
				dramWrInitCnt <= 0;
				stage1 <= False;

				//statusCheckerQ.enq(1); // Check dram writer initial set done
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Initial setting \033[1;32mdone!\n", cycleCount);
			end else begin
				dramWrInitCnt <= dramWrInitCnt + 1;
			end
		end else begin
			dramArbiter.access[0].users[0].cmd(0, fromInteger(totalWords), True);
			dramWrInitCnt <= dramWrInitCnt + 1;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 2 (DRAM Reader)
	//
	// This stage read the data from DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(16)) dramRdCnt <- mkReg(0);
	rule dramReader( stage2 );
		if ( dramRdCnt != 0 ) begin
			let payload <- dramArbiter.access[0].users[0].read;
			serializer128b.put(payload);
				
			if ( dramRdCnt == 192 ) begin
				dramRdCnt <= 0;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 x, y, and index data \033[1;32mdone!\n", cycleCount);
			end else begin
				dramRdCnt <= dramRdCnt + 1;
			end
		end else begin
			dramArbiter.access[0].users[0].cmd(0, 192, False); // 3*1024*4 = 192*64
			dramRdCnt <= dramRdCnt + 1;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 x, y, and index data \033[1;32mstart!\n", cycleCount);
		end
	endrule
	//-------------------------------------------------------------------------------------------------
	// Stage 3 (Data Serializer & Data Deserializer)
	//
	// This state splits the 512-bit payload to 32-bit data or merges the data to a payload
	//-------------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) originDataQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) dataInBuffer <- mkReg(0);
	Reg#(Bit#(4)) dataInCnt <- mkReg(0);
	rule dataOrganizerDataIn( stage3 );
		Vector#(3, Bit#(32)) v = replicate(0);
		if ( dataInCnt == 0 ) begin
			let d <- serializer128b.get;
			v[0] = d[31:0]; // x
			v[1] = d[63:32]; // y
			v[2] = d[95:64]; // idx
			dataInBuffer <= zeroExtend(d[127:96]);
			dataInCnt <= dataInCnt + 1;
		end else if ( dataInCnt == 1 ) begin
			let d <- serializer128b.get;
			Bit#(32) b = truncate(dataInBuffer);
			v[0] = b;
			v[1] = d[31:0];
			v[2] = d[63:32];
			dataInBuffer <= zeroExtend(d[127:64]);
			dataInCnt <= dataInCnt + 1;
		end else if ( dataInCnt == 2 ) begin
			let d <- serializer128b.get;
			Bit#(64) b = truncate(dataInBuffer);
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = d[31:0];
			dataInBuffer <= d[127:32];
			dataInCnt <= dataInCnt + 1;
		end else if ( dataInCnt == 3 ) begin
			let b = dataInBuffer;
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = b[95:64];
			dataInBuffer <= 0;
			dataInCnt <= 0;
		end
		originDataQ.enq(v);
	endrule

	FIFO#(Vector#(3, Bit#(32))) updatedDataQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) dataOutBuffer <- mkReg(0);
	Reg#(Bit#(4)) dataOutCnt <- mkReg(0);
	rule dataOrganizerDataOut( stage3 );
		updatedDataQ.deq;
		let v = updatedDataQ.first;
		Bit#(96) d = (zeroExtend(v[2])) | (zeroExtend(v[1])) | (zeroExtend(v[0]));

		if ( dataOutCnt == 0 ) begin
			dataOutBuffer <= d;
			dataOutCnt <= dataOutCnt + 1;
		end else if ( dataOutCnt == 1 ) begin
			Bit#(128) b = zeroExtend(dataOutBuffer);
			Bit#(128) d_0 = zeroExtend(d[31:0]);
			Bit#(128) p = (d_0 << 96) | (b);
			deserializer128b.put(p);
			dataOutBuffer <= (d >> 32);
			dataOutCnt <= dataOutCnt + 1;
		end else if ( dataOutCnt == 2 ) begin
			Bit#(128) b = zeroExtend(dataOutBuffer);
			Bit#(128) d_01 = zeroExtend(d[63:0]);
			Bit#(128) p = (d_01 << 64) | (b);
			deserializer128b.put(p);
			dataOutBuffer <= (d >> 64);
			dataOutCnt <= dataOutCnt + 1;
		end else if ( dataOutCnt == 3 ) begin
			Bit#(128) b = zeroExtend(dataOutBuffer);
			Bit#(128) d_012 = zeroExtend(d);
			Bit#(128) p = (d_012 << 32) | (b);
			deserializer128b.put(p);
			dataOutBuffer <= 0;
			dataOutCnt <= 0;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 4 (K-mean, Data Relayer)
	//
	// This stage relays the data to K-mean app ***Important***
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(16)) relayDataCnt <- mkReg(0);
	rule dataRelayer( stage4 );
		originDataQ.deq;
		let v = originDataQ.first;

		kmean.dataIn(v);

		if ( relayDataCnt + 1 == fromInteger(bramFifoSize) ) begin
			relayDataCnt <= 0;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Relay 1024 x, y, and index data \033[1;32mdone!\n", cycleCount);
		end else begin
			relayDataCnt <= relayDataCnt + 1;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 5 (Updated Data Receiver)
	//
	// This stage receives updated data from K-mean app, goes to stage 3 and then goes to 6
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(16)) recvDataCnt <- mkReg(0);
	rule dataReceiver( stage5 );
		let d <- kmean.dataOut;
		updatedDataQ.enq(d);
		if ( recvDataCnt != 0 ) begin
			if ( recvDataCnt + 1 == fromInteger(bramFifoSize) ) begin
				recvDataCnt <= 0;

				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Receive 1024 updated data \033[1;32m done!\n", cycleCount);
			end else begin
				recvDataCnt <= recvDataCnt + 1;
			end
		end else begin
			recvDataCnt <= recvDataCnt + 1;
		end
	endrule
	rule flagReceiver( stage5 );
		let f <- kmean.flagOut;
		flagQ.enq(f);
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 6 (DRAM Writer)
	//
	// This stage writes the updated data to DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(16)) dramWrRsltCnt <- mkReg(0);
	rule dramWriterRslt( stage6 );
		if ( dramWrRsltCnt != 0 ) begin
			let payload <- deserializer128b.get;
			dramArbiter.access[0].users[0].write(payload);
			if ( dramWrRsltCnt == 192 ) begin
				flagQ.deq;
				let f = flagQ.first;
				if ( f == 1 ) begin
					stage3 <= False;
					stage4 <= False;
					stage5 <= False;
					statusCheckerQ.enq(1);
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: System \033[1;32mfinish!\n", cycleCount);
				end else begin
					stage2 <= True;
				end
				stage6 <= False;
				dramWrRsltCnt <= 0;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Writing 1024 updated data \033[1;32mdone!\n", cycleCount);
			end else begin
				dramWrRsltCnt <= dramWrRsltCnt + 1;
			end
		end else begin
			dramArbiter.access[0].users[0].cmd(0, 192, True);
			dramWrRsltCnt <= dramWrRsltCnt + 1;
		end
	endrule
