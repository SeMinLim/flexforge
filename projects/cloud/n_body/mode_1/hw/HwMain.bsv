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
	DeSerializerIfc#(128, 4) deserializer128bPos <- mkDeSerializer;
	DeSerializerIfc#(128, 4) deserializer128bVel <- mkDeSerializer;
	SerializerIfc#(512, 4) serializer128bPos <- mkSerializer;
	SerializerIfc#(512, 4) serializer128bVel <- mkSerializer;

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
	Reg#(Bool) dramReaderPos <- mkReg(True);
	Reg#(Bool) dramReaderVel <- mkReg(False);

	Reg#(Bool) stage3 <- mkReg(False);

	Reg#(Bool) stage4 <- mkReg(False);

	Reg#(Bool) stage5 <- mkReg(False);
	Reg#(Bool) dataReceiverPos <- mkReg(True);
	Reg#(Bool) dataReceiverVel <- mkReg(False);

	Reg#(Bool) stage6 <- mkReg(False);
	Reg#(Bool) dramWriterPos <- mkReg(True);
	Reg#(Bool) dramWriterVel <- mkReg(False);
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
	Reg#(Bit#(64)) dramRdAddPos <- mkReg(0);
	Reg#(Bit#(64)) dramRdAddVel <- mkReg(268435456);

	Reg#(Bit#(16)) dramRdPosCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) dramRdPosCnt_2 <- mkReg(0);
	
	Reg#(Bit#(16)) dramRdVelCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) dramRdVelCnt_2 <- mkReg(0);

	rule dramReader( stage2 );
		if ( dramReaderPos ) begin
			if ( dramRdPosCnt_1 != 0 ) begin
				let payload <- dramArbiter.access[0].users[0].read;
				serializer128bPos.put(payload);
				
				if ( dramRdPosCnt_1 == 256 ) begin
					if ( dramRdPosCnt_2 + 1 == fromInteger(dramProcUnit) ) begin
						dramRdAddPos <= 0;
						dramRdPosCnt_2 <= 0;

						dramReaderPos <= False;
						dramReaderVel <= True;
					end else begin
						dramRdAddPos <= dramRdAddPos + (1024*4*4);
						dramRdPosCnt_2 <= dramRdPosCnt_2 + 1;
						stage2 <= False;
					end
					dramRdPosCnt_1 <= 0;
				end else begin
					dramRdPosCnt_1 <= dramRdPosCnt_1 + 1;
				end
			end else begin
				dramArbiter.access[0].users[0].cmd(dramRdAddPos, 256, False); // Read 1024 position value of particles
				dramRdPosCnt_1 <= dramRdPosCnt_1 + 1;
				stage3 <= True;
			end
		end else if ( dramReaderVel ) begin
			if ( dramRdVelCnt_1 != 0 ) begin
				let payload <- dramArbiter.access[0].users[1].read;
				serializer128bVel.put(payload);
				if ( dramRdVelCnt_1 == 192 ) begin
					if ( dramRdVelCnt_2 + 1 == fromInteger(dramProcUnit) ) begin
						dramRdAddVel <= 268435456;
						dramRdVelCnt_2 <= 0;
					end else begin
						dramRdAddVel <= dramRdAddVel + (1024*3*4);
						dramRdVelCnt_2 <= dramRdVelCnt_2 + 1;
					end
					dramRdVelCnt_1 <= 0;

					dramReaderPos <= True;
					dramReaderVel <= False;

					stage2 <= False;
					stage6 <= True; // Shut DRAM Reader down & Open DRAM Writer up
				end else begin
					dramRdVelCnt_1 <= dramRdVelCnt_1 + 1;
				end
			end else begin
				dramArbiter.access[0].users[1].cmd(dramRdAddVel, 192, False); // Read 1024 velocity value of particles
				dramRdVelCnt_1 <= dramRdVelCnt_1 + 1;
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
	FIFO#(Vector#(3, Bit#(32))) originDataPos_iQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Vector#(4, Bit#(32))) originDataPos_jQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Bit#(32)) originDataMassQ <- mkSizedBRAMFIFO(1024);

	Reg#(Bit#(16)) dramProcUnitIdx <- mkReg(0);
	Reg#(Bit#(16)) idxCnt <- mkReg(0);
	rule dataOrganizerPosIn( stage3 );
		Vector#(4, Bit#(32)) pm = replicate(0);
		Vector#(3, Bit#(32)) p = replicate(0);
		Bit#(16) idx = dramProcUnitIdx;

		let d <- serializer128bPos.get;

		pm[0] = d[31:0]; // Position X 
		pm[1] = d[63:32]; // Position Y
		pm[2] = d[95:64]; // Position Z
		pm[3] = d[127:96]; // Mass

		for ( Integer i = 0; i < 3; i = i + 1 ) p[i] = pm[i];

		if ( idx != 0 ) begin
			if ( idx + 1 == fromInteger(dramProcUnit) ) begin
				if ( idxCnt + 1 == fromInteger(bramFifoSize) ) begin
					dramProcUnitIdx <= 0;
					idxCnt <= 0;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Read and organize pos_j done(%d/16384)!\n", cycleCount, (idx+1));
				end else begin
					idxCnt <= idxCnt + 1;
				end
			end else begin
				if ( idxCnt + 1 == fromInteger(bramFifoSize) ) begin
					dramProcUnitIdx <= dramProcUnitIdx + 1;
					idxCnt <= 0;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Read and organize pos_j done(%d/16384)!\n", cycleCount, (idx+1));
				end else begin
					idxCnt <= idxCnt + 1;
				end
			end
			originDataPos_jQ.enq(pm);
		end else begin
			if ( idxCnt + 1 == fromInteger(bramFifoSize) ) begin
				idxCnt <= 0;
				dramProcUnitIdx <= dramProcUnitIdx + 1;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Read and organize pos_i done!\n", cycleCount);
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage1\033[0m: Read and organize pos_j done(%d/16384)!\n", cycleCount, (idx+1));
			end else begin
				idxCnt <= idxCnt + 1;
			end
			originDataPos_iQ.enq(p); // Operand i
			originDataPos_jQ.enq(pm); // Operand j
			originDataMassQ.enq(pm[3]); // Mass

			stage4 <= True;
		end
	endrule

	FIFO#(Vector#(3, Bit#(32))) originDataVelQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) toNbodyVelBuffer <- mkReg(0);
	Reg#(Bit#(4)) toNbodyVelCnt <- mkReg(0);
	rule dataOrganizerVelIn( stage3 );
		Vector#(3, Bit#(32)) v = replicate(0);
		if ( toNbodyVelCnt == 0 ) begin
			let d <- serializer128bVel.get;
			v[0] = d[31:0]; // Velocity X
			v[1] = d[63:32]; // Velocity Y
			v[2] = d[95:64]; //  Velocity Z
			toNbodyVelBuffer <= zeroExtend(d[127:96]);	
			toNbodyVelCnt <= toNbodyVelCnt + 1;			
		end else if ( toNbodyVelCnt == 1 ) begin
			let d <- serializer128bVel.get;
			Bit#(32) b = truncate(toNbodyVelBuffer);
			v[0] = b;
			v[1] = d[31:0];
			v[2] = d[63:32];
			toNbodyVelBuffer <= zeroExtend(d[127:64]);
			toNbodyVelCnt <= toNbodyVelCnt + 1;
		end else if ( toNbodyVelCnt == 2 ) begin
			let d <- serializer128bVel.get;
			Bit#(64) b = truncate(toNbodyVelBuffer);
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = d[31:0];
			toNbodyVelBuffer <= d[127:32];
			toNbodyVelCnt <= toNbodyVelCnt + 1;
		end else if ( toNbodyVelCnt == 3 ) begin
			let b = toNbodyVelBuffer;
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = b[95:64];
			toNbodyVelBuffer <= 0;
			toNbodyVelCnt <= 0;
		end
		originDataVelQ.enq(v);
	endrule
	//-------------------------------------------------------------------------------------------------
	// From N-body
	//-------------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) updatedDataPosQ <- mkSizedBRAMFIFO(1024);
	rule dataOrganizerPosOut( stage3 );
		updatedDataPosQ.deq;
		originDataMassQ.deq;
		Vector#(3, Bit#(32)) v = updatedDataPosQ.first;
		Bit#(32) v_3 = originDataMassQ.first;

		Bit#(128) d = (zeroExtend(v_3)) | (zeroExtend(v[2])) | (zeroExtend(v[1])) | (zeroExtend(v[0]));
			
		deserializer128bPos.put(d);
	endrule

	FIFO#(Vector#(3, Bit#(32))) updatedDataVelQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) fromNbodyVelBuffer <- mkReg(0);
	Reg#(Bit#(4)) fromNbodyVelCnt <- mkReg(0);
	rule dataOrganizerVelOut( stage3 );
		updatedDataVelQ.deq;
		let v = updatedDataVelQ.first;
		Bit#(96) d = (zeroExtend(v[2])) | (zeroExtend(v[1])) | (zeroExtend(v[0]));

		if ( fromNbodyVelCnt == 0 ) begin
			fromNbodyVelBuffer <= d;
			fromNbodyVelCnt <= fromNbodyVelCnt + 1;
		end else if ( fromNbodyVelCnt == 1 ) begin
			Bit#(128) b = zeroExtend(fromNbodyVelBuffer);
			Bit#(128) d_0 = zeroExtend(d[31:0]);
			Bit#(128) p = (d_0 << 96) | (b);
			deserializer128bVel.put(p);
			fromNbodyVelBuffer <= (d >> 32);
			fromNbodyVelCnt <= fromNbodyVelCnt + 1;
		end else if ( fromNbodyVelCnt == 2 ) begin
			Bit#(128) b = zeroExtend(fromNbodyVelBuffer);
			Bit#(128) d_01 = zeroExtend(d[63:0]);
			Bit#(128) p = (d_01 << 64) | (b);
			deserializer128bVel.put(p);
			fromNbodyVelBuffer <= (d >> 64);
			fromNbodyVelCnt <= fromNbodyVelCnt + 1;
		end else if ( fromNbodyVelCnt == 3 ) begin
			Bit#(128) b = zeroExtend(fromNbodyVelBuffer);
			Bit#(128) d_012 = zeroExtend(d);
			Bit#(128) p = (d_012 << 32) | (b);
			deserializer128bVel.put(p);
			fromNbodyVelBuffer <= 0;
			fromNbodyVelCnt <= 0;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 4 (N-body, Data Relayer)
	//
	// This stage relays the data to N-body app
	//--------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) originDataPosQ <- mkSizedBRAMFIFO(1024);
	Reg#(Vector#(3, Bit#(32))) relayDataBufferPos_i <- mkReg(replicate(0));
	Reg#(Bit#(16)) relayDataCntPos_i_1 <- mkReg(0);
	Reg#(Bit#(16)) relayDataCntPos_i_2 <- mkReg(0);
	rule dataRelayerPos_i( stage4 );
		if ( relayDataCntPos_i_1 != 0 ) begin
			let p_i = relayDataBufferPos_i;
		
			nbody.dataPIn_i(p_i);

			if ( relayDataCntPos_i_1 + 1 == fromInteger(totalReplicate) ) begin
				if ( relayDataCntPos_i_2 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataCntPos_i_2 <= 0;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay 1024 operand i done!\n", cycleCount);
				end else begin
					relayDataCntPos_i_2 <= relayDataCntPos_i_2 + 1;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay %dth operand i done!\n", cycleCount, (relayDataCntPos_i_2+1));
				end
				relayDataCntPos_i_1 <= 0;
			end else begin
				relayDataCntPos_i_1 <= relayDataCntPos_i_1 + 1;
			end
		end else begin
			originDataPos_iQ.deq;
			let p_i = originDataPos_iQ.first;
			
			nbody.dataPIn_i(p_i);
			
			relayDataBufferPos_i <= p_i;
			relayDataCntPos_i_1 <= relayDataCntPos_i_1 + 1;

			originDataPosQ.enq(p_i); // Store position value for updating to BRAM
		end
	endrule
	Reg#(Bit#(16)) relayDataCntPos_j_1 <- mkReg(0);
	Reg#(Bit#(16)) relayDataCntPos_j_2 <- mkReg(0);
	Reg#(Bit#(16)) relayDataCntPos_j_3 <- mkReg(0);
	rule dataRelayerPos_j( stage4 );
		originDataPos_jQ.deq;
		let p_j = originDataPos_jQ.first;

		nbody.dataPmIn_j(p_j);

		if ( relayDataCntPos_j_3 + 1 == fromInteger(dramProcUnit) ) begin
			if ( relayDataCntPos_j_2 + 1 == fromInteger(totalReplicate) ) begin
				if ( relayDataCntPos_j_1 != 0 ) begin
					if ( relayDataCntPos_j_1 + 1 == fromInteger(bramFifoSize) ) begin
						relayDataCntPos_j_1 <= 0;
						relayDataCntPos_j_2 <= 0;
						relayDataCntPos_j_3 <= 0;
						$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay Pos_j done(%d)!\n", cycleCount, relayDataCntPos_j_3+1);
					end else begin
						relayDataCntPos_j_1 <= relayDataCntPos_j_1 + 1;
					end
				end else begin
					relayDataCntPos_j_1 <= relayDataCntPos_j_1 + 1;
					stage2 <= True;
				end
			end else begin
				if ( relayDataCntPos_j_1 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataCntPos_j_1 <= 0;
					relayDataCntPos_j_2 <= relayDataCntPos_j_2 + 1;
				end else begin
					relayDataCntPos_j_1 <= relayDataCntPos_j_1 + 1;
				end
				originDataPos_jQ.enq(p_j); // Replicate operand j
			end
		end else begin
			if ( relayDataCntPos_j_2 + 1 == fromInteger(totalReplicate) ) begin
				if ( relayDataCntPos_j_1 != 0 ) begin
					if ( relayDataCntPos_j_1 + 1 == fromInteger(bramFifoSize) ) begin
						relayDataCntPos_j_1 <= 0;
						relayDataCntPos_j_2 <= 0;
						relayDataCntPos_j_3 <= relayDataCntPos_j_3 + 1;
						$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage4\033[0m: Relay Pos_j done(%d)!\n", cycleCount, relayDataCntPos_j_3+1);
					end else begin
						relayDataCntPos_j_1 <= relayDataCntPos_j_1 + 1;
					end
				end else begin
					relayDataCntPos_j_1 <= relayDataCntPos_j_1 + 1;
					stage2 <= True;
				end
			end else begin
				if ( relayDataCntPos_j_1 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataCntPos_j_1 <= 0;
					relayDataCntPos_j_2 <= relayDataCntPos_j_2 + 1;
				end else begin
					relayDataCntPos_j_1 <= relayDataCntPos_j_1 + 1;
				end
				originDataPos_jQ.enq(p_j); // Replicate operand j
			end
		end
	endrule

	Reg#(Bit#(16)) relayDataPVCnt <- mkReg(0);
	rule dataRelayerPV( stage4 );
		originDataPosQ.deq;
		originDataVelQ.deq;
		let p = originDataPosQ.first;
		let v = originDataVelQ.first;

		nbody.dataPIn(p);
		nbody.dataVIn(v);
			
		if ( relayDataPVCnt + 1 == fromInteger(bramFifoSize) ) begin
			relayDataPVCnt <= 0;
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
		if ( dataReceiverPos ) begin
			let d <- nbody.dataOutP;
			updatedDataPosQ.enq(d);
			if ( recvDataPCnt != 0 ) begin
				if ( recvDataPCnt + 1 == fromInteger(bramFifoSize) ) begin
					recvDataPCnt <= 0;
					
					dataReceiverPos <= False;
					dataReceiverVel <= True;
				end else begin
					recvDataPCnt <= recvDataPCnt + 1;
				end
			end else begin
				recvDataPCnt <= recvDataPCnt + 1;
			end
		end else if ( dataReceiverVel ) begin
			let d <- nbody.dataOutV;
			updatedDataVelQ.enq(d);
			if ( recvDataVCnt != 0 ) begin
				if ( recvDataVCnt + 1 == fromInteger(bramFifoSize) ) begin
					recvDataVCnt <= 0;
					
					dataReceiverPos <= True;
					dataReceiverVel <= False;
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
	Reg#(Bit#(64)) dramWrRsltAddPos <- mkReg(469762048);
	Reg#(Bit#(64)) dramWrRsltAddVel <- mkReg(738197504);
	Reg#(Bit#(16)) dramWrRsltCntPos <- mkReg(0);
	Reg#(Bit#(16)) dramWrRsltCntVel <- mkReg(0);	
	rule dramWriterRslt( stage6 );
		if ( dramWriterPos ) begin
			if ( dramWrRsltCntPos != 0 ) begin
				let payload <- deserializer128bPos.get;
				dramArbiter.access[0].users[0].write(payload);
				if ( dramWrRsltCntPos == 256 ) begin
					if ( (dramWrRsltAddPos + (1024*4*4)) == 738197504 ) begin
						dramWrRsltAddPos <= 469762048;
					end else begin
						dramWrRsltAddPos <= dramWrRsltAddPos + (1024*4*4);
					end
					dramWrRsltCntPos <= 0;
					
					dramWriterPos <= False;
					dramWriterVel <= True;
					
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage6\033[0m: Writing 1024 updated position data done!\n", cycleCount);
				end else begin
					dramWrRsltCntPos <= dramWrRsltCntPos + 1;
				end
			end else begin
				dramArbiter.access[0].users[0].cmd(dramWrRsltAddPos, 256, True);
				dramWrRsltCntPos <= dramWrRsltCntPos + 1;
			end
		end else if ( dramWriterVel ) begin
			if ( dramWrRsltCntVel != 0 ) begin
				let payload <- deserializer128bVel.get;
				dramArbiter.access[0].users[1].write(payload);
				if ( dramWrRsltCntVel == 192 ) begin
					if ( (dramWrRsltAddVel + (1024*3*4)) == 939524096 ) begin
						dramWrRsltAddVel <= 738197504;
						
						// Initializing
						stage3 <= False;
						stage4 <= False;
						stage5 <= False;
					end else begin
						dramWrRsltAddVel <= dramWrRsltAddVel + (1024*3*4);	
						stage2 <= True;			
					end
					dramWrRsltCntVel <= 0;
					
					dramWriterPos <= True;
					dramWriterVel <= False;
					
					stage6 <= False;
		
					statusCheckerQ.enq(1); // Write 1024 updated particel values done
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[1;33mStage6\033[0m: Writing 1024 updated velocity data done!\n", cycleCount);
				end else begin
					dramWrRsltCntVel <= dramWrRsltCntVel + 1;
				end
			end else begin
				dramArbiter.access[0].users[1].cmd(dramWrRsltAddVel, 192, True);
				dramWrRsltCntVel <= dramWrRsltCntVel + 1;
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
