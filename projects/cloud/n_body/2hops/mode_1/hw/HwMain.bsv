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

import Nbody::*;


Integer byteTotal = 16*1024*1024*4;
Integer wordsTotal64Byte = 7*1024*1024;
Integer wordsPm64Byte = 4*1024*1024;
Integer wordsV64Byte = 3*1024*1024;

Integer particles = 16777216;
Integer unitParticles = 1024;
Integer dramProcUnit = 16384;

Integer bramFifoSize = 1024;
Integer totalReplicate = 1024;


interface HwMainIfc;
endinterface
module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram) (HwMainIfc);

	Clock curClk <- exposeCurrentClock;
	Reset curRst <- exposeCurrentReset;

	Clock pcieclk = pcie.user_clk;
	Reset pcierst = pcie.user_rst;	

	FIFOF#(Bit#(32)) cycleQ <- mkFIFOF;
	Reg#(Bit#(32)) cycleCount <- mkReg(0);
	Reg#(Bit#(32)) cycleStart <- mkReg(0);
	Reg#(Bit#(32)) cycleEnd <- mkReg(0);
	rule incCycleCount;
		cycleCount <= cycleCount + 1;
	endrule

	DeSerializerIfc#(32, 16) deserializer32b <- mkDeSerializer;
	DeSerializerIfc#(128, 4) deserializer128bPm <- mkDeSerializer;
	DeSerializerIfc#(128, 4) deserializer128bV <- mkDeSerializer;
	SerializerIfc#(512, 4) serializer128bP_i <- mkSerializer;
	SerializerIfc#(512, 4) serializer128bPm_j <- mkSerializer;
	SerializerIfc#(512, 4) serializer128bV <- mkSerializer;

	DRAMArbiterRemoteIfc#(2) dramArbiter <- mkDRAMArbiterRemote(dram);
	NbodyIfc nbody <- mkNbody;
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

	Reg#(Bool) stage1 <- mkReg(False);

	Reg#(Bool) stage2 <- mkReg(False);
	Reg#(Bool) dramReaderPm <- mkReg(True);
	Reg#(Bool) dramReaderV <- mkReg(False);
	Reg#(Bool) dramProcUnitFirst <- mkReg(True);	
	Reg#(Bit#(16)) startCnt <- mkReg(0);

	Reg#(Bool) stage3 <- mkReg(False);

	Reg#(Bool) stage4 <- mkReg(False);
	Reg#(Bool) dataRelayer1stPhase_i <- mkReg(True);
	Reg#(Bool) dataRelayer1stPhase_j <- mkReg(True);
	Reg#(Bool) dataRelayer2ndPhase <- mkReg(False);

	Reg#(Bool) stage5 <- mkReg(False);
	Reg#(Bool) dataReceiverP <- mkReg(True);
	Reg#(Bool) dataReceiverV <- mkReg(False);

	Reg#(Bool) stage6 <- mkReg(False);
	Reg#(Bool) dramWriterPm <- mkReg(True);
	Reg#(Bool) dramWriterV <- mkReg(False);
	Reg#(Bit#(16)) doneCnt <- mkReg(0);
	rule getCmd;
		pcieWriteQ.deq;
		let w = pcieWriteQ.first;

		let d = w.data;
		let a = w.addr;
		let off = (a >> 2);
	
		if ( off == 0 ) begin
			stage1 <= True;
		end else if ( off == 1 ) begin
			deserializer32b.put(d);
		end else if ( off == 2 ) begin 
			stage2 <= True;	
			cycleStart <= cycleCount;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Usage of the entire memory
	//  For Mode 1
	//   OriginPmAdd => 0 ~ 268,435,455                  OriginVAdd => 268,435,456 ~ 469,762,047
	//   UpdatedPmAdd => 469,762,048 ~ 738,197,503       UpdatedVAdd => 738,197,504 ~ 939,524,095
	//--------------------------------------------------------------------------------------------
	// Stage 1 (Initial Setting)
	//
	// This stage writes the data take from the host through PCIe to DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(32)) dramWrInitCnt <- mkReg(0); // Perfect!
	rule dramWriterInit( stage1 );
		if ( dramWrInitCnt != 0 ) begin
			let payload <- deserializer32b.get;
			dramArbiter.access[0].users[0].write(payload);
			if ( dramWrInitCnt == fromInteger(wordsTotal64Byte) ) begin
				dramWrInitCnt <= 0;
				stage1 <= False;

				statusCheckerQ.enq(1); // Check dram writer initial set done
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Initial setting done!\n", cycleCount);
			end else begin
				dramWrInitCnt <= dramWrInitCnt + 1;
			end
		end else begin
			dramArbiter.access[0].users[0].cmd(0, fromInteger(wordsTotal64Byte), True);
			dramWrInitCnt <= dramWrInitCnt + 1;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 2 (DRAM Reader)
	// 
	// This stage read the data from DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(64)) dramRdAddPm <- mkReg(0);
	Reg#(Bit#(64)) dramRdAddV <- mkReg(268435456);

	Reg#(Bit#(16)) dramRdPmCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) dramRdPmCnt_2 <- mkReg(0);
	
	Reg#(Bit#(16)) dramRdVCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) dramRdVCnt_2 <- mkReg(0);

	rule dramReader( stage2 );
		if ( dramReaderPm ) begin
			if ( dramRdPmCnt_1 != 0 ) begin
				let payload <- dramArbiter.access[0].users[0].read;
				serializer128bP_i.put(payload);
				serializer128bPm_j.put(payload);

				if ( dramRdPmCnt_1 == 256 ) begin
					if ( dramRdPmCnt_2 + 1 == fromInteger(dramProcUnit) ) begin
						dramRdAddPm <= 0;
						dramRdPmCnt_2 <= 0;

						dramReaderPm <= False;
						dramReaderV <= True;
					end else begin
						dramRdAddPm <= dramRdAddPm + (1024*4*4);
						dramRdPmCnt_2 <= dramRdPmCnt_2 + 1;
						stage2 <= False;
					end
					dramRdPmCnt_1 <= 0;
				end else begin
					dramRdPmCnt_1 <= dramRdPmCnt_1 + 1;
				end
			end else begin
				dramArbiter.access[0].users[0].cmd(dramRdAddPm, 256, False); // Read 1024 particles
				dramRdPmCnt_1 <= dramRdPmCnt_1 + 1;
				stage3 <= True;
			end
		end else if ( dramReaderV ) begin
			if ( dramRdVCnt_1 != 0 ) begin
				let payload <- dramArbiter.access[0].users[1].read;
				serializer128bV.put(payload);
				if ( dramRdVCnt_1 == 192 ) begin
					if ( dramRdVCnt_2 + 1 == fromInteger(dramProcUnit) ) begin
						dramRdAddV <= 268435456;
						dramRdVCnt_2 <= 0;
					end else begin
						dramRdAddV <= dramRdAddV + (1024*3*4);
						dramRdVCnt_2 <= dramRdVCnt_2 + 1;
					end
					dramRdVCnt_1 <= 0;

					dramReaderPm <= True;
					dramReaderV <= False;

					stage2 <= False;
					stage6 <= True; // Shut DRAM Reader down & Open DRAM Writer up
				end else begin
					dramRdVCnt_1 <= dramRdVCnt_1 + 1;
				end
			end else begin
				dramArbiter.access[0].users[1].cmd(dramRdAddV, 192, False); // Read 1024 particles
				dramRdVCnt_1 <= dramRdVCnt_1 + 1;
			end
		end
	endrule
	//-------------------------------------------------------------------------------------------------
	// Stage 3 (Data Serializer & Data Deserializer) 
	//
	// This state splits the 512-bit payload to 32-bit data or merges the data to a payload
	//-------------------------------------------------------------------------------------------------
	// To N-body
	//-------------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) originDataP_iQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Bit#(32)) originDataM_iQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Vector#(4, Bit#(32))) originDataPm_jQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(16)) dramProcUnitIdxPmj <- mkReg(0);
	Reg#(Bit#(16)) idxCntPmj <- mkReg(0);
	rule dataOrganizerPmIn_j( stage3 );
		Vector#(4, Bit#(32)) pm = replicate(0);
		Vector#(3, Bit#(32)) p = replicate(0);
		Bit#(16) idx = dramProcUnitIdxPmj;

		let d <- serializer128bPm_j.get;

		pm[0] = d[31:0]; // Position X 
		pm[1] = d[63:32]; // Position Y
		pm[2] = d[95:64]; // Position Z
		pm[3] = d[127:96]; // Mass

		for ( Integer i = 0; i < 3; i = i + 1 ) p[i] = pm[i]

		if ( idx != 0 ) begin
			if ( idx + 1 == fromInteger(dramProcUnit) ) begin
				if ( idxCnt + 1 == fromInteger(bramFifoSize) ) begin
					dramProcUnitIdx <= 0;
					idxCnt <= 0;
				end else begin
					idxCnt <= idxCnt + 1;
				end
			end else begin
				if ( idxCnt + 1 == fromInteger(bramFifoSize) ) begin
					dramProcUnitIdx <= 0;
					idxCnt <= 0;
				end else begin
					idxCnt <= idxCnt + 1;
				end
			end

		end else begin
			if ( idxCnt + 1 == fromInteger(bramFifoSize) ) begin
				idxCnt <= 0;
				dramProcUnitIdx <= dramProcUnitIdx + 1;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Read and organize pos_i done!\n", cycleCount);
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Read and organize pos_j done(%d/16384)!\n", cycleCount, (idx+1));
			end else begin
				idxCnt <= idxCnt + 1;
			end
			originDataP_iQ.enq(p); // Operand i
			originDataM_iQ.enq(pm[3]);
			originDataPm_jQ.enq(pm); // Operand j
			stage4 <= True;
		end
	endrule

	FIFO#(Vector#(3, Bit#(32))) originDataVQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) toNbodyVBuffer <- mkReg(0);
	Reg#(Bit#(4)) toNbodyVCnt <- mkReg(0);
	rule dataOrganizerVIn( stage3 );
		Vector#(3, Bit#(32)) v = replicate(0);
		if ( toNbodyVCnt == 0 ) begin
			let d <- serializer128bV.get;
			v[0] = d[31:0]; // Velocity X
			v[1] = d[63:32]; // Velocity Y
			v[2] = d[95:64]; //  Velocity Z
			toNbodyVBuffer <= zeroExtend(d[127:96]);	
			toNbodyVCnt <= toNbodyVCnt + 1;			
		end else if ( toNbodyVCnt == 1 ) begin
			let d <- serializer128bV.get;
			Bit#(32) b = truncate(toNbodyVBuffer);
			v[0] = b;
			v[1] = d[31:0];
			v[2] = d[63:32];
			toNbodyVBuffer <= zeroExtend(d[127:64]);
			toNbodyVCnt <= toNbodyVCnt + 1;
		end else if ( toNbodyVCnt == 2 ) begin
			let d <- serializer128bV.get;
			Bit#(64) b = truncate(toNbodyVBuffer);
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = d[31:0];
			toNbodyVBuffer <= d[127:32];
			toNbodyVCnt <= toNbodyVCnt + 1;
		end else if ( toNbodyVCnt == 3 ) begin
			let b = toNbodyVBuffer;
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = b[95:64];
			toNbodyVBuffer <= 0;
			toNbodyVCnt <= 0;
		end
		originDataVQ.enq(v);
	endrule
	//-------------------------------------------------------------------------------------------------
	// From N-body
	//-------------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) updatedDataPQ <- mkSizedBRAMFIFO(1024);
	rule dataOrganizerPmOut( stage3 );
		updatedDataPQ.deq;
		originDataM_iQ.deq;
		Vector#(3, Bit#(32)) v = updatedDataPQ.first;
		Bit#(32) v_3 = originDataM_iQ.first;

		Bit#(128) d = (zeroExtend(v_3)) | (zeroExtend(v[2])) | (zeroExtend(v[1])) | (zeroExtend(v[0]));
			
		deserializer128bPm.put(d);
	endrule

	FIFO#(Vector#(3, Bit#(32))) updatedDataVQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) fromNbodyVBuffer <- mkReg(0);
	Reg#(Bit#(4)) fromNbodyVCnt <- mkReg(0);
	rule dataOrganizerVOut( stage3 );
		updatedDataVQ.deq;
		let v = updatedDataVQ.first;
		Bit#(96) d = (zeroExtend(v[2])) | (zeroExtend(v[1])) | (zeroExtend(v[0]));

		if ( fromNbodyVCnt == 0 ) begin
			fromNbodyVBuffer <= d;
			fromNbodyVCnt <= fromNbodyVCnt + 1;
		end else if ( fromNbodyVCnt == 1 ) begin
			Bit#(128) b = zeroExtend(fromNbodyVBuffer);
			Bit#(128) d_0 = zeroExtend(d[31:0]);
			Bit#(128) p = (d_0 << 96) | (b);
			deserializer128bV.put(p);
			fromNbodyVBuffer <= (d >> 32);
			fromNbodyVCnt <= fromNbodyVCnt + 1;
		end else if ( fromNbodyVCnt == 2 ) begin
			Bit#(128) b = zeroExtend(fromNbodyVBuffer);
			Bit#(128) d_01 = zeroExtend(d[63:0]);
			Bit#(128) p = (d_01 << 64) | (b);
			deserializer128bV.put(p);
			fromNbodyVBuffer <= (d >> 64);
			fromNbodyVCnt <= fromNbodyVCnt + 1;
		end else if ( fromNbodyVCnt == 3 ) begin
			Bit#(128) b = zeroExtend(fromNbodyVBuffer);
			Bit#(128) d_012 = zeroExtend(d);
			Bit#(128) p = (d_012 << 32) | (b);
			deserializer128bV.put(p);
			fromNbodyVBuffer <= 0;
			fromNbodyVCnt <= 0;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 4 (N-body, Data Relayer)
	//
	// This stage relays the data to N-body app
	//--------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) originDataPQ <- mkSizedBRAMFIFO(1024);
	Reg#(Vector#(3, Bit#(32))) relayDataPBuffer_i <- mkReg(replicate(0));
	Reg#(Bit#(16)) relayDataPCnt_i_1 <- mkReg(0);
	Reg#(Bit#(16)) relayDataPCnt_i_2 <- mkReg(0);
	rule dataRelayerPI( stage4 && dataRelayer1stPhase_i );
		if ( relayDataPCnt_i_1 != 0 ) begin
			let p_i = relayDataPBuffer_i;
			nbody.dataPIn_i(p_i);

			if ( relayDataPCnt_i_1 + 1 == fromInteger(totalReplicate) ) begin
				if ( relayDataPCnt_i_2 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataPCnt_i_2 <= 0;
					dataRelayer1stPhase_i <= False;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay 1024 operand i done!\n", cycleCount);
				end else begin
					relayDataPCnt_i_2 <= relayDataPCnt_i_2 + 1;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay %dth operand i done!\n", cycleCount, (relayDataPCnt_i_2+1));
				end
				relayDataPCnt_i_1 <= 0;
			end else begin
				relayDataPCnt_i_1 <= relayDataPCnt_i_1 + 1;
			end
		end else begin
			originDataP_iQ.deq;
			let p_i = originDataP_iQ.first;
			
			nbody.dataPIn_i(p_i);
			
			relayDataPBuffer_i <= p_i;
			relayDataPCnt_i_1 <= relayDataPCnt_i_1 + 1;

			originDataPQ.enq(p_i);
		end
	endrule
	Reg#(Bit#(16)) relayDataPmCnt_j_1 <- mkReg(0);
	Reg#(Bit#(16)) relayDataPmCnt_j_2 <- mkReg(0);
	Reg#(Bit#(16)) relayDataPmCnt_j_3 <- mkReg(0);
	rule dataRelayerPmJ( stage4 );
		originDataPm_jQ.deq;
		let p_j = originDataPm_jQ.first;
		nbody.dataPmIn_j(p_j);

		if ( relayDataPmCnt_j_3 + 1 == fromInteger(dramProcUnit) ) begin
			if ( relayDataPmCnt_j_2 + 1 == fromInteger(totalReplicate) ) begin
				if ( relayDataPmCnt_j_1 != 0 ) begin
					if ( relayDataPmCnt_j_1 + 1 == fromInteger(bramFifoSize) ) begin
						relayDataPmCnt_j_1 <= 0;
						relayDataPmCnt_j_2 <= 0;
						relayDataPmCnt_j_3 <= 0;
						$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay %dth 1024 operand j done!\n", cycleCount, relayDataPmCnt_j_3+1);
					end else begin
						relayDataPmCnt_j_1 <= relayDataPmCnt_j_1 + 1;
					end
				end else begin
					relayDataPmCnt_j_1 <= relayDataPmCnt_j_1 + 1;
					stage2 <= True;
				end
			end else begin
				if ( relayDataPmCnt_j_1 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataPmCnt_j_1 <= 0;
					relayDataPmCnt_j_2 <= relayDataPmCnt_j_2 + 1;
				end else begin
					relayDataPmCnt_j_1 <= relayDataPmCnt_j_1 + 1;
				end
				originDataPm_jQ.enq(p_j); // Replicate operand j
			end
		end else begin
			if ( relayDataPmCnt_j_2 + 1 == fromInteger(totalReplicate) ) begin
				if ( relayDataPmCnt_j_1 != 0 ) begin
					if ( relayDataPmCnt_j_1 + 1 == fromInteger(bramFifoSize) ) begin
						relayDataPmCnt_j_1 <= 0;
						relayDataPmCnt_j_2 <= 0;
						relayDataPmCnt_j_3 <= relayDataPmCnt_j_3 + 1;
						$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay %dth 1024 operand j done!\n", cycleCount, relayDataPmCnt_j_3+1);
					end else begin
						relayDataPmCnt_j_1 <= relayDataPmCnt_j_1 + 1;
					end
				end else begin
					relayDataPmCnt_j_1 <= relayDataPmCnt_j_1 + 1;
					stage2 <= True;
				end
			end else begin
				if ( relayDataPmCnt_j_1 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataPmCnt_j_1 <= 0;
					relayDataPmCnt_j_2 <= relayDataPmCnt_j_2 + 1;
				end else begin
					relayDataPmCnt_j_1 <= relayDataPmCnt_j_1 + 1;
				end
				originDataPm_jQ.enq(p_j); // Replicate operand j
			end
		end
	endrule

	Reg#(Bit#(16)) relayDataPVCnt <- mkReg(0);
	rule dataRelayerPV( stage4 );
		originDataPQ.deq;
		originDataVQ.deq;
		let p = originDataPQ.first;
		let v = originDataVQ.first;

		nbody.dataPIn(p);
		nbody.dataVIn(v);
			
		if ( relayDataPVCnt + 1 == fromInteger(bramFifoSize) ) begin
			relayDataPVCnt <= 0;
			dataRelayer1stPhase_i <= True;
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay 1024 velocity data done!\n", cycleCount);
		end else begin
			relayDataPVCnt <= relayDataPVCnt + 1;
			stage5 <= True;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 5 (Updated Data Receiver)
	// 
	// This stage receives updated data from N-body app, goes to stage 3 and then goes to 6
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(16)) recvDataPCnt <- mkReg(0);
	Reg#(Bit#(16)) recvDataVCnt <- mkReg(0);
	rule dataReceiver( stage5 );
		if ( dataReceiverP ) begin
			let d <- nbody.dataOutP;
			updatedDataPQ.enq(d);
			if ( recvDataPCnt != 0 ) begin
				if ( recvDataPCnt + 1 == fromInteger(bramFifoSize) ) begin
					recvDataPCnt <= 0;
					
					dataReceiverP <= False;
					dataReceiverV <= True;
				end else begin
					recvDataPCnt <= recvDataPCnt + 1;
				end
			end else begin
				recvDataPCnt <= recvDataPCnt + 1;
			end
		end else if ( dataReceiverV ) begin
			let d <- nbody.dataOutV;
			updatedDataVQ.enq(d);
			if ( recvDataVCnt != 0 ) begin
				if ( recvDataVCnt + 1 == fromInteger(bramFifoSize) ) begin
					recvDataVCnt <= 0;
					
					dataReceiverP <= True;
					dataReceiverV <= False;
				end else begin
					recvDataVCnt <= recvDataVCnt + 1;
				end
			end else begin
				recvDataVCnt <= recvDataVCnt + 1;
			end
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 6 (DRAM Writer) 
	//
	// This stage writes the updated data to DRAM
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(64)) dramWrRsltAddPm <- mkReg(469762048);
	Reg#(Bit#(64)) dramWrRsltAddV <- mkReg(738197504);
	Reg#(Bit#(16)) dramWrRsltCntPm <- mkReg(0);
	Reg#(Bit#(16)) dramWrRsltCntV <- mkReg(0);	
	rule dramWriterRslt( stage6 );
		if ( dramWriterPm ) begin
			if ( dramWrRsltCntPm != 0 ) begin
				let payload <- deserializer128bPm.get;
				dramArbiter.access[0].users[0].write(payload);
				if ( dramWrRsltCntPm == 256 ) begin
					if ( (dramWrRsltAddPm + (1024*4*4)) == 738197504 ) begin
						dramWrRsltAddPm <= 469762048;
					end else begin
						dramWrRsltAddPm <= dramWrRsltAddPm + (1024*4*4);
					end
					dramWrRsltCntPm <= 0;
					
					dramWriterPm <= False;
					dramWriterV <= True;
					
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage6\033[0m: Writing 1024 updated position data done!\n", cycleCount);
				end else begin
					dramWrRsltCntPm <= dramWrRsltCntPm + 1;
				end
			end else begin
				dramArbiter.access[0].users[0].cmd(dramWrRsltAddPm, 256, True);
				dramWrRsltCntPm <= dramWrRsltCntPm + 1;
			end
		end else if ( dramWriterV ) begin
			if ( dramWrRsltCntV != 0 ) begin
				let payload <- deserializer128bV.get;
				dramArbiter.access[0].users[1].write(payload);
				if ( dramWrRsltCntV == 192 ) begin
					if ( (dramWrRsltAddV + (1024*3*4)) == 939524096 ) begin
						dramWrRsltAddV <= 738197504;
						
						// Initializing
						dramProcUnitFirst <= True;
						startCnt <= 0;
						doneCnt <= 0;
						stage3 <= False;
						stage4 <= False;
						stage5 <= False;
					end else begin
						dramWrRsltAddV <= dramWrRsltAddV + (1024*3*4);	
						doneCnt <= doneCnt + 1;
						stage2 <= True;			
					end
					dramWrRsltCntV <= 0;
					
					dramWriterPm <= True;
					dramWriterV <= False;
					
					stage6 <= False;
					dataRelayer1stPhase_i <= True;

					statusCheckerQ.enq(1); // Write 1024 updated particel values done
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage6\033[0m: Writing 1024 updated velocity data done!\n", cycleCount);
				end else begin
					dramWrRsltCntV <= dramWrRsltCntV + 1;
				end
			end else begin
				dramArbiter.access[0].users[1].cmd(dramWrRsltAddV, 192, True);
				dramWrRsltCntV <= dramWrRsltCntV + 1;
			end
		end
	endrule

	//-------------------------------------------------------------------------------------------------
	// FPGA1 (A Part of Checking Status)
	//-------------------------------------------------------------------------------------------------
	rule getStatus;
		pcieReadReqQ.deq;
		let r = pcieReadReqQ.first;
		Bit#(4) a = truncate(r.addr>>2);
		if ( a < 2 ) begin
			if ( statusCheckerQ.notEmpty ) begin
				pcieRespQ.enq(tuple2(r, statusCheckerQ.first));
				statusCheckerQ.deq;
			end else begin 
				pcieRespQ.enq(tuple2(r, 32'hffffffff));
			end
		end else begin
			if ( cycleQ.notEmpty ) begin
				pcieRespQ.enq(tuple2(r, cycleQ.first));
				cycleQ.deq;
			end else begin 
				pcieRespQ.enq(tuple2(r, 32'hffffffff));
			end
		end
	endrule
endmodule
