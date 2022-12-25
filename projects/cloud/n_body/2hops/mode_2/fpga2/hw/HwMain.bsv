// This HwMain is for only FPGA2
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

import AuroraCommon::*;
import AuroraExtImportCommon::*;
import AuroraExtImport117::*;
import AuroraExtImport119::*;


Integer idxFPGA1 = 0;
Integer idxFPGA2 = 1;
Integer pubKeyFPGA1 = 1;
Integer pubKeyFPGA2 = 2;


Integer byteTotal = 16*1024*1024*4;
Integer wordsTotal64Byte = 7*1024*1024;
Integer wordsPm64Byte = 4*1024*1024;
Integer wordsV64Byte = 3*1024*1024;
Integer wordsPm64ByteHalf = 4*1024*512;
Integer wordsV64ByteHalf = 3*1024*512;


function Bit#(8) cycleDeciderExt(Bit#(8) routeCnt, Bit#(8) payloadByte);
	Bit#(8) auroraExtCnt = 0;
	if ( routeCnt == 0 ) begin
		Bit#(8) totalByte = 4+payloadByte;
		Bit#(16) totalBits = zeroExtend(totalByte) * 8;
		Bit#(16) decidedCycle = cycleDecider(totalBits);
		auroraExtCnt = truncate(decidedCycle);
	end else begin
		Bit#(8) totalByte = 4+2+payloadByte;
		Bit#(16) totalBits = zeroExtend(totalByte) * 8;
		Bit#(16) decidedCycle = cycleDecider(totalBits);
		auroraExtCnt = truncate(decidedCycle);
	end
	return auroraExtCnt;
endfunction


interface HwMainIfc;
endinterface
module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram, Vector#(2, AuroraExtIfc) auroraQuads) (HwMainIfc);

	Clock curClk <- exposeCurrentClock;
	Reset curRst <- exposeCurrentReset;

	Clock pcieclk = pcie.user_clk;
	Reset pcierst = pcie.user_rst;	

	Reg#(Bit#(32)) cycleCount <- mkReg(0);
	Reg#(Bit#(32)) stCycle <- mkReg(0);
	Reg#(Bit#(32)) edCycle <- mkReg(0);
	rule incCycleCount;
		cycleCount <= cycleCount + 1;
	endrule
	
	DeSerializerIfc#(32, 16) deserializer32b <- mkDeSerializer;
	DeSerializerIfc#(128, 4) deserializer128bPm <- mkDeSerializer;
	DeSerializerIfc#(128, 4) deserializer128bV <- mkDeSerializer;
	SerializerIfc#(512, 4) serializer128bPm <- mkSerializer;
	Serializerifc#(512, 4) serializer128bV <- mkSerializer;

	DRAMArbiterRemoteIfc#(2) dramArbiterRemote <- mkDRAMArbiterRemote(dram);
	//--------------------------------------------------------------------------------------------
	// Connection
	//  FPGA1(0) <-> (4)FPGA2
	//  FPGA1(1) <-> (5)FPGA2
	//  FPGA1(2) <-> (6)FPGA2
	//  FPGA1(3) <-> (7)FPGA2
	//--------------------------------------------------------------------------------------------
	// Usage of the entire memory
	//  Initial Set
	//   OriginPmAdd => 0 ~ 268,435,455                  OriginVAdd => 268,435,456 ~ 469,762,047
	//  For Mode 1
	//   OriginPmAdd => 0 ~ 268,435,455                  OriginVAdd => 268,435,456 ~ 469,762,047
	//   UpdatedPmAdd => 469,762,048 ~ 738,197,503       UpdatedVAdd => 738,197,504 ~ 939,524,095
	//  For Mode 2 ~ 5
	//   FPGA1
	//   OriginPmAdd => 0 ~ 134,217,727                  OriginVAdd => 268,435,456 ~ 369,098,751
	//   UpdatedPmAdd => 134,217,728 ~ 268,435,455       UpdatedVAdd => 369,098,752 ~ 467,762,047
	//   FPGA2
	//   OriginPmAdd => 0 ~ 134,217,727                  OriginVAdd => 268,435,456 ~ 369,098,751
	//   UpdatedPmAdd => 134,217,728 ~ 268,435,455       UpdatedVAdd => 369,098,752 ~ 467,762,047
	//--------------------------------------------------------------------------------------------
	// FPGA2 (Aurora Part)
	//--------------------------------------------------------------------------------------------
	FIFOF#(AuroraIfcType) recvPacketByAuroraFPGA2Q <- mkFIFOF;
	rule fpga2Receiver_Port4;
		Bit#(8) inPortFPGA2_1 = 4;
		Bit#(1) qidIn = inPortFPGA2_1[2];
		Bit#(2) pidIn = truncate(inPortFPGA2_1);

		let recvPacket <- auroraQuads[qidIn].user[pidIn].receive;
		recvPacketByAuroraFPGA2Q[i].enq(recvPacket);
	endrule
	FIFOF#(AuroraIfcType) sendPacketByAuroraFPGA2Q <- mkFIFOF;
	rule fpga2Sender_Port4;
		Bit#(8) outPortFPGA2_1 = 4;
		Bit#(1) qidOut = outPortFPGA2_1[2];
		Bit#(2) pidOut = truncate(outPortFPGA2_1);

		sendPacketByAuroraFPGA2Q.deq;
		let p = sendPacketByAuroraFPGA2Q.first;

		auroraQuads[qidOut].user[pidOut].send(AuroraSend{packet:p,num:8});
	endrule
	//--------------------------------------------------------------------------------------------
	// FPGA2 (Memory Part)
	//--------------------------------------------------------------------------------------------
	// Dram Writer
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(64)) memWrInitAddPm <- mkReg(0);
	Reg#(Bit#(64)) memWrInitAddV <- mkReg(268435456);

	Reg#(Bit#(32)) memWrCntIni <- mkReg(0);
	
	Reg#(Bool) fpga2MemWrInit <- mkReg(True);
	Reg#(Bool) fpag2MemWrRslt <- mkReg(False);
	
	Reg#(Bool) memWrPm <-mkReg(True);
	Reg#(Bool) memWrV <- mkReg(False);
	
	Reg#(Bool) fpga2MemReaderOn <- mkReg(False);
	rule fpga2MemWriterInit( fpga2MemWrInit );
		if ( memWrPm ) begin
			if ( memWrCntIni != 0 ) begin
				recvPacketByAuroraFPGA2Q.deq;
				let payload = recvPacketByAuroraFPGA2Q.first;
				dramArbiterRemote.access[0].users[0].write(payload);
				if ( memWrCntIni == fromInteger(wordsPm64ByteHalf) ) begin
					memWrCntIni <= 0;
					
					memWrPm <= False;
					memWrV <= True;
				end else begin
					memWrCntIni <= memWrCntIni + 1;
				end
			end else begin
				dramArbiterRemote.access[0].users[0].cmd(memWrInitAddPm, fromInteger(wordsPm64ByteHalf), True);
				memWrCntIni <= memWrCntIni + 1;
			end
		end else if ( memWrV ) begin
			if ( memWrCntIni != 0 ) begin
				recvPacketByAuroraFPGA2Q.deq;
				let payload = recvPacketByAuroraFPGA2Q.first;
				dramArbiterRemote.access[0].users[0].write(payload);
				if ( memWrCntIni == fromInteger(wordsV64ByteHalf) ) begin
					memWrCntIni <= 0;

					memWrPm <= True;
					memWrV <= False;
					
					fpga2MemWrInit <= False;
					fpga2MemWrRslt <= True;

					fpga2MemReaderOn <= True;
				end else begin
					memWrCntIni <= memWrCntIni + 1;
				end
			end else begin
				dramArbiterRemote.access[0].users[0].cmd(memWrInitAddV, fromInteger(wordsV64ByteHalf), True);
				memWrCntIni <= memWrCntIni + 1;
			end
		end
	endrule
	
	Reg#(Bit#(64)) memWrRsltAddPm <- mkReg(134217728);
	Reg#(Bit#(64)) memWrRsltAddV <- mkReg(369098752);
	Reg#(Bit#(8)) memWrRsltCntPm <- mkReg(0);
	Reg#(Bit#(8)) memWrRsltCntV <- mkReg(0);
	rule fpga2MemWriterRslt( fpga2MemWrRslt );
		if ( memWrPm ) begin
			if ( memWrRsltCntPm != 0 ) begin
				recvPacketByAuroraFPGA2Q.deq;
				let payload = recvPacketByAuroraFPGA2Q.first;
				dramArbiterRemote.access[0].users[0].write(payload);
				if ( memWrRsltCntPm == 32 ) begin
					if ( memWrRsltAddPm + (128*4*4) == 268435456 ) begin
						memWrRsltAddPm <= 134217728;
					end else begin
						memWrRsltAddPm <= memWrRsltAddPm + 1;
					end
					memWrRsltCntPm <= 0;
					memWrPm <= False;
					memWrV <= True;
				end else begin
					memWrRsltCntPm <= memWrRsltCntPm + 1;
				end
			end else begin
				dramArbiterRemote.access[0].users[0].cmd(memWrRsltAddPm, 32, True);
				memWrRsltCntPm <= memWrRsltCntPm + 1;
			end
		end else if ( memWrV ) begin
			if ( memWrRsltCntV != 0 ) begin
				recvPacketByAuroraFPGA2Q.deq;
				let payload = recvPacketByAuroraFPGA2Q.first;
				dramArbiterRemote.access[0].users[0].write(payload);
				if ( memWrRsltCntV == 24 ) begin
					if ( memWrRsltAddV + (128*3*4) == 467762048 ) begin
						memWrRsltAddV <= 369098752;
						fpga2MemWrInit <= True;
						fpga2MemWrRslt <= False;
					end else begin
						memWrRsltAddV <= memWrRsltAddV + 1;
					end
					memWrRsltCntV <= 0;
					memWrPm <= True;
					memWrV <= False;
				end else begin
					memWrRsltCntV <= memWrRsltCntV + 1;
				end
			end else begin
				dramArbiterRemote.access[0].users[0].cmd(memWrRsltAddV, 24, True);
				memWrRsltCntV <= memWrRsltCntV + 1;
			end
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Dram Reader
	//--------------------------------------------------------------------------------------------
	Reg#(Bit#(64)) memRdAddPm <- mkReg(0);
	Reg#(Bit#(64)) memRdAddV <- mkReg(268435456);

	Reg#(Bit#(32)) memRdPmCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) memRdPmCnt_2 <- mkReg(0);
	Reg#(Bit#(16)) memRdVCnt_1 <- mkReg(0);
	Reg#(Bit#(32)) memRdVCnt_2 <- mkReg(0);

	Reg#(Bool) memRdPm <- mkReg(True);
	Reg#(Bool) memRdV <- mkReg(False);
	rule fpga2MemReader( fpga2MemReaderOn );
		if ( memRdPm ) begin
			if ( memRdPmCnt_1 != 0) begin
				let payload <- dramArbiterRemote.access[0].users[0].read;
				sendPacketByAuroraFPGA2Q.enq(payload);
				if ( memRdPmCnt_1 == fromInteger(wordsPm64ByteHalf) ) begin
					if ( memRdPmCnt_2 == 127 ) begin
						memRdPmCnt_2 <= 0;

						memRdPm <= False;
						memRdV <= True;
					end else begin
						memRdPmCnt_2 <= memRdPmCnt_2 + 1;
					end
					memRdPmCnt_1 <= 0;
				end else begin
					memRdPmCnt_1 <= memRdPmCnt_1 + 1;
				end
			end else begin
				dramArbiterRemote.access[0].users[0].cmd(memRdAddPm, fromInteger(wordsPm64ByteHalf), False);
				memRdPmCnt_1 <= memRdPmCnt_1 + 1;
			end
		end else if ( memRdV ) begin
			if ( memRdVCnt_1 != 0 ) begin
				let payload <- dramArbiterRemote.access[0].users[0].read;
				sendPacketByAuroraFPGA2Q.enq(payload);
				if ( memRdVCnt_1 == 48 ) begin
					if ( memRdVCnt_2 == (fromInteger(wordsV64ByteHalf) - 1) ) begin
						memRdAddV <= 268435456;
						memRdCnt_2 <= 0;
					end else begin
						memRdAddV <= memRdAddV + (128*3*4);
						memRdCnt_2 <= memRdCnt_2 + 1;
					end
					memRdCnt_1 <= 0;

					memRdPm <= True;
					memRdV <= False;
				end else begin
					memRdVCnt_2 <= memRdVCnt_2 + 1;
					memRdVCnt_1 <= memRdVCnt_1 + 1;
				end
			end else begin
				dramArbiterRemote.access[0].users[0].cmd(memRdAddv, 24, False);
				memRdVCnt_1 <= memRdVCnt_1 + 1;
			end
		end
	endrule
	rule fpga2Decrypter( !openConnect );
		Integer privKeyFPGA2 = 2;

		recvPacketByAuroraFPGA2Q[0].deq;
		let recvPacket = recvPacketByAuroraFPGA2Q[0].first;

		Bit#(32) headerPart = recvPacket[31:0] ^ fromInteger(privKeyFPGA2);
		Bit#(8) numHops = headerPart[7:0];
		Bit#(24) packetHeader = headerPart[31:8];
		Bit#(8) routeCnt = zeroExtend(packetHeader[7:1]);
		Bit#(8) payloadByte = packetHeader[23:16];

		AuroraIfcType payload = recvPacket >> 32;
		if ( packetHeader[0] == 0 ) begin // Source Routing
			Bit#(32) aomNheader = payload[31:0] ^ fromInteger(privKeyFPGA2);
			Bit#(32) address = payload[63:32] ^ fromInteger(privKeyFPGA2); 

			if ( aomNheader[0] == 0 ) begin // Write
				if ( aomNheader[31:1] == 4*1024 ) begin
					validCheckConnectionFPGA2Q.enq(1);
				end else begin
					validCheckConnectionFPGA2Q.enq(0);
				end			
			end
		end else if ( packetHeader[0] == 1 ) begin // Data Sending
			Bit#(64) data = payload[63:0] ^ fromInteger(privKeyFPGA2);

			if ( data == 4294967296 ) begin
				validCheckConnectionFPGA2Q.enq(1);
			end else begin
				validCheckConnectionFPGA2Q.enq(0);
			end
		end
	endrule
	//--------------------------------------------------------------------------------------------
	// Send the packets to the host
	//--------------------------------------------------------------------------------------------
	FIFOF#(Bit#(32)) validCheckConnectionQ <- mkFIFOF;
	rule validCheckerFPGA1( openConnect );
		Bit#(8) validCheckPort = 1;
		Bit#(1) qidIn = validCheckPort[2];
		Bit#(2) pidIn = truncate(validCheckPort);

		let recvPacket <- auroraQuads[qidIn].user[pidIn].receive;

		if ( recvPacket == 1 ) begin
			validCheckConnectionQ.enq(1);
		end else if ( recvPacket == 0 ) begin
			validCheckConnectionQ.enq(0);
		end
	endrule
	rule validCheckerFPGA2( !openConnect );
		Bit#(8) validCheckPort = 5;
		Bit#(1) qidOut = validCheckPort[2];
		Bit#(2) pidOut = truncate(validCheckPort);

		validCheckConnectionFPGA2Q.deq;
		let sendPacket = validCheckConnectionFPGA2Q.first;

		auroraQuads[qidOut].user[pidOut].send(AuroraSend{packet:sendPacket,num:2});
	endrule

endmodule
