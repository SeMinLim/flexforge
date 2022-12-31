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

// 1024 particles
// 1 particle has 7 32-bit floating point values
// 3 position, 1 mass, and 3 velocity values
Integer particles = 1024;
Integer byteTotal = 7*1024*4;
Integer wordsTotal64Byte = 7*64;
Integer wordsPos64Byte = 4*64;
Integer wordsVel64Byte = 3*64;

Integer totalReplicate = 1024;
Integer bramFifoSize = 1024;


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

	BRAM_Configure cfg = defaultValue;
	cfg.memorySize = 1024*128;
	BRAM2Port#(Bit#(32), Vector#(4, Bit#(32))) bram <- mkBRAM2Server(cfg);

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
	// Initial Setting
	Reg#(Bool) stage1 <- mkReg(False);
	// DRAM Reader
	Reg#(Bool) stage2 <- mkReg(False);
	Reg#(Bool) dramReaderPos <- mkReg(True);
	Reg#(Bool) dramReaderVel <- mkReg(False);
	// Data Organizer
	Reg#(Bool) stage3 <- mkReg(False);
	// Data Relayer
	Reg#(Bool) stage4 <- mkReg(False);
	Reg#(Bool) bramWriteDone <- mkReg(False);
	// Data Receiver
	Reg#(Bool) stage5 <- mkReg(False);
	Reg#(Bool) dataReceiverPos <- mkReg(True);
	Reg#(Bool) dataReceiverVel <- mkReg(False);
	// DRAM Writer
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
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Initial setting start!\n", cycleCount);
		end else if ( off == 2 ) begin 
			stage2 <= True;	
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: System start!\n", cycleCount);
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Usage of the entire memory
	//  For Mode 1
	//   OriginPosAdd => 0 ~ 16,383                  OriginVelAdd => 16,384 ~ 28,671
	//   UpdatePosAdd => 28,672 ~ 45,055             UpdateVelAdd => 45,056 ~ 57,343
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
			if ( dramWrInitCnt == fromInteger(wordsTotal64Byte) ) begin
				dramWrInitCnt <= 0;
				stage1 <= False;

				//statusCheckerQ.enq(1); // Check dram writer initial set done
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Initial setting \033[1;32mdone!\n", cycleCount);
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
	Reg#(Bit#(64)) dramRdAddPos <- mkReg(0); // Perfect!!!
	Reg#(Bit#(64)) dramRdAddVel <- mkReg(16384);

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
					dramRdAddPos <= 0;
					dramRdPosCnt_2 <= 0;

					dramReaderPos <= False;
					dramReaderVel <= True;
					
					dramRdPosCnt_1 <= 0;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 position and mass data \033[1;32mdone!\n", cycleCount);
				end else begin
					dramRdPosCnt_1 <= dramRdPosCnt_1 + 1;
				end
			end else begin
				dramArbiter.access[0].users[0].cmd(dramRdAddPos, 256, False); // 1024*4*4 = 64*256
				dramRdPosCnt_1 <= dramRdPosCnt_1 + 1;
				stage3 <= True;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 position and mass data \033[1;32mstart!\n", cycleCount);
			end
		end else if ( dramReaderVel ) begin
			if ( dramRdVelCnt_1 != 0 ) begin
				let payload <- dramArbiter.access[0].users[1].read;
				serializer128bVel.put(payload);
				if ( dramRdVelCnt_1 == 192 ) begin
					dramRdAddVel <= 16384;
					dramRdVelCnt_1 <= 0;

					dramReaderPos <= True;
					dramReaderVel <= False;

					stage2 <= False;
					stage6 <= True; // Shut DRAM Reader down & Open DRAM Writer up
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 velocity data \033[1;32mdone!\n", cycleCount);
				end else begin
					dramRdVelCnt_1 <= dramRdVelCnt_1 + 1;
				end
			end else begin
				dramArbiter.access[0].users[1].cmd(dramRdAddVel, 192, False); // 1024*3*4 = 64*192
				dramRdVelCnt_1 <= dramRdVelCnt_1 + 1;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Read 1024 velocity data \033[1;32mstart!\n", cycleCount);
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
	rule dataOrganizerPosIn( stage3 );
		Vector#(4, Bit#(32)) pm = replicate(0);
		Vector#(3, Bit#(32)) p = replicate(0);

		let d <- serializer128bPos.get;

		pm[0] = d[31:0]; // Position X 
		pm[1] = d[63:32]; // Position Y
		pm[2] = d[95:64]; // Position Z
		pm[3] = d[127:96]; // Mass

		for ( Integer i = 0; i < 3; i = i + 1 ) p[i] = pm[i];

		originDataPos_iQ.enq(p); // Operand i
		originDataPos_jQ.enq(pm); // Operand j
		originDataMassQ.enq(pm[3]); // Mass

		stage4 <= True;
	endrule

	FIFO#(Vector#(3, Bit#(32))) originDataVelQ <- mkSizedBRAMFIFO(1024);
	Reg#(Bit#(96)) velInBuffer <- mkReg(0);
	Reg#(Bit#(4)) velInCnt <- mkReg(0);
	rule dataOrganizerVelIn( stage3 );
		Vector#(3, Bit#(32)) v = replicate(0);
		if ( velInCnt == 0 ) begin
			let d <- serializer128bVel.get;
			v[0] = d[31:0]; // Velocity X
			v[1] = d[63:32]; // Velocity Y
			v[2] = d[95:64]; //  Velocity Z
			velInBuffer <= zeroExtend(d[127:96]);	
			velInCnt <= velInCnt + 1;			
		end else if ( velInCnt == 1 ) begin
			let d <- serializer128bVel.get;
			Bit#(32) b = truncate(velInBuffer);
			v[0] = b;
			v[1] = d[31:0];
			v[2] = d[63:32];
			velInBuffer <= zeroExtend(d[127:64]);
			velInCnt <= velInCnt + 1;
		end else if ( velInCnt == 2 ) begin
			let d <- serializer128bVel.get;
			Bit#(64) b = truncate(velInBuffer);
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = d[31:0];
			velInBuffer <= d[127:32];
			velInCnt <= velInCnt + 1;
		end else if ( velInCnt == 3 ) begin
			let b = velInBuffer;
			v[0] = b[31:0];
			v[1] = b[63:32];
			v[2] = b[95:64];
			velInBuffer <= 0;
			velInCnt <= 0;
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
	Reg#(Bit#(96)) velOutBuffer <- mkReg(0);
	Reg#(Bit#(4)) velOutCnt <- mkReg(0);
	rule dataOrganizerVelOut( stage3 );
		updatedDataVelQ.deq;
		let v = updatedDataVelQ.first;
		Bit#(96) d = (zeroExtend(v[2])) | (zeroExtend(v[1])) | (zeroExtend(v[0]));

		if ( velOutCnt == 0 ) begin
			velOutBuffer <= d;
			velOutCnt <= velOutCnt + 1;
		end else if ( velOutCnt == 1 ) begin
			Bit#(128) b = zeroExtend(velOutBuffer);
			Bit#(128) d_0 = zeroExtend(d[31:0]);
			Bit#(128) p = (d_0 << 96) | (b);
			deserializer128bVel.put(p);
			velOutBuffer <= (d >> 32);
			velOutCnt <= velOutCnt + 1;
		end else if ( velOutCnt == 2 ) begin
			Bit#(128) b = zeroExtend(velOutBuffer);
			Bit#(128) d_01 = zeroExtend(d[63:0]);
			Bit#(128) p = (d_01 << 64) | (b);
			deserializer128bVel.put(p);
			velOutBuffer <= (d >> 64);
			velOutCnt <= velOutCnt + 1;
		end else if ( velOutCnt == 3 ) begin
			Bit#(128) b = zeroExtend(velOutBuffer);
			Bit#(128) d_012 = zeroExtend(d);
			Bit#(128) p = (d_012 << 32) | (b);
			deserializer128bVel.put(p);
			velOutBuffer <= 0;
			velOutCnt <= 0;
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Stage 4 (N-body, Data Relayer)
	//
	// This stage relays the data to N-body app ***Important***
	//--------------------------------------------------------------------------------------------
	FIFO#(Vector#(3, Bit#(32))) originDataPosQ <- mkSizedBRAMFIFO(1024);
	Reg#(Vector#(3, Bit#(32))) relayDataPos_iBuffer <- mkReg(replicate(0));
	Reg#(Bit#(16)) relayDataPos_iCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) relayDataPos_iCnt_2 <- mkReg(0);
	rule dataRelayerPos_i( stage4 );
		if ( relayDataPos_iCnt_1 != 0 ) begin
			let pos_i = relayDataPos_iBuffer;
		
			nbody.dataPIn_i(pos_i);

			if ( relayDataPos_iCnt_1 + 1 == 64 ) begin
				if ( relayDataPos_iCnt_2 + 1 == fromInteger(bramFifoSize) ) begin
					relayDataPos_iCnt_2 <= 0;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Relay 1024 pos_i \033[1;32mdone!\n", cycleCount);
				end else begin
					relayDataPos_iCnt_2 <= relayDataPos_iCnt_2 + 1;
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Relay %dth pos_i \033[1;32mdone!\n", cycleCount, (relayDataPos_iCnt_2+1));
				end
				relayDataPos_iCnt_1 <= 0;
			end else begin
				relayDataPos_iCnt_1 <= relayDataPos_iCnt_1 + 1;
			end
		end else begin
			originDataPos_iQ.deq;
			let pos_i = originDataPos_iQ.first;
			
			nbody.dataPIn_i(pos_i);
			
			relayDataPos_iBuffer <= pos_i;
			relayDataPos_iCnt_1 <= relayDataPos_iCnt_1 + 1;

			originDataPosQ.enq(pos_i); // Original position data should be stored to finally update position and velocity data
			stage5 <= True;
		end
	endrule

	Reg#(Bit#(32)) relayDataPos_jCnt_1 <- mkReg(0);
	Reg#(Bit#(32)) relayDataPos_jCnt_2 <- mkReg(0);
	rule dataRelayerPos_j_1( stage4 );
		if ( bramWriteDone ) begin
			if ( relayDataPos_jCnt_1 + 1 == fromInteger(bramFifoSize) ) begin
				if ( relayDataPos_jCnt_2 + 2 == fromInteger(totalReplicate) ) begin
					relayDataPos_jCnt_2 <= 0;
					bramWriteDone <= False;
				end else begin
					relayDataPos_jCnt_2 <= relayDataPos_jCnt_2 + 1;
				end
				relayDataPos_jCnt_1 <= 0;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Relay %dth 1024 pos_j \033[1;32mdone!\n", cycleCount, (relayDataPos_jCnt_2+2));
			end else begin
				relayDataPos_jCnt_1 <= relayDataPos_jCnt_1 + 1;
			end
			bram.portA.request.put(BRAMRequest{write:False, responseOnWrite:?, address:relayDataPos_jCnt_1*128, datain:?});
		end else begin
			originDataPos_jQ.deq;
			let pos_j = originDataPos_jQ.first;

			nbody.dataPIn_j(pos_j);
		
			if ( relayDataPos_jCnt_1 + 1 == fromInteger(bramFifoSize) ) begin
				relayDataPos_jCnt_1 <= 0;
				bramWriteDone <= True;
				$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Relay 1th 1024 operand j \033[1;32mdone!\n", cycleCount);
			end else begin
				relayDataPos_jCnt_1 <= relayDataPos_jCnt_1 + 1;
			end
			bram.portA.request.put(BRAMRequest{write:True, responseOnWrite:False, address:relayDataPos_jCnt_1*128, datain:pos_j});
		end
	endrule
	rule dataRelayerPos_j_2( stage4 );
		let v <- bram.portA.response.get();
		nbody.dataPIn_j(v);
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
			$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Relay 1024 velocity data \033[1;32mdone!\n", cycleCount);
		end else begin
			relayDataPVCnt <= relayDataPVCnt + 1;
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
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Receive 1024 updated position data \033[1;32m done!\n", cycleCount);
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
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Receive 1024 updated velocity data \033[1;32mdone!\n", cycleCount);
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
	Reg#(Bit#(64)) dramWrRsltAddPos <- mkReg(28672);
	Reg#(Bit#(64)) dramWrRsltAddVel <- mkReg(45056);
	Reg#(Bit#(16)) dramWrRsltCntPos <- mkReg(0);
	Reg#(Bit#(16)) dramWrRsltCntVel <- mkReg(0);	
	rule dramWriterRslt( stage6 );
		if ( dramWriterPos ) begin
			if ( dramWrRsltCntPos != 0 ) begin
				let payload <- deserializer128bPos.get;
				dramArbiter.access[0].users[0].write(payload);
				if ( dramWrRsltCntPos == 256 ) begin
					dramWrRsltAddPos <= 28672;
					dramWrRsltCntPos <= 0;
					
					dramWriterPos <= False;
					dramWriterVel <= True;
					
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Writing 1024 updated position data \033[1;32mdone!\n", cycleCount);
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
					dramWrRsltAddVel <= 45056;
						
					stage3 <= False;
					stage4 <= False;
					stage5 <= False;
					stage6 <= False;

					dramWrRsltCntVel <= 0;
					
					dramWriterPos <= True;
					dramWriterVel <= False;
					
					statusCheckerQ.enq(1); // Write 1024 updated particel values done
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: Writing 1024 updated velocity data \033[1;32mdone!\n", cycleCount);
					$write("\033[1;33mCycle %1d -> \033[1;33m[HwMain]: \033[0m: System \033[1;32mfinished!\n", cycleCount);
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
