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

//import Kmean::*;


typedef 1 PubKeyFPGA1;
typedef 2 PubKeyFPGA2;
typedef 1 PrivKeyFPGA1;

typedef 0 IdxFPGA1;
typedef 1 IdxFPGA2;

typedef 131072 TotalByte;
typedef 2048 TotalWords;
typedef 65536 TotalByteFPGA1; // (32/2)*1024*4
typedef 1024 TotalWordsFPGA1; // (32/2)*64

typedef 32 NumData;
typedef 1024 Dimension;
typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;


interface HwMainIfc;
endinterface
module mkHwMain#(PcieUserIfc pcie, DRAMUserIfc dram, Vector#(2, AuroraExtIfc) auroraQuads) (HwMainIfc);

	Clock curClk <- exposeCurrentClock;
	Reset curRst <- exposeCurrentReset;

	Clock pcieclk = pcie.user_clk;
	Reset pcierst = pcie.user_rst;

	//--------------------------------------------------------------------------------------------
	// Aurora Part
	//
	// This part sends or receives packet via Aurora Protocol
	//--------------------------------------------------------------------------------------------
	// Send Packets from FPGA1 to FPGA2 via Aurora Protocol
	//--------------------------------------------------------------------------------------------
	FIFO#(Tuple3#(Bit#(128), Bit#(8), Bit#(1))) auroraSenderQ <- mkSizedBRAMFIFO(1024); // packet, port, sr/ds
	rule sendAuroraPackets;
		auroraSenderQ.deq;
		let a = auroraSenderQ.first;
		let payload = tpl_1(a);
		let outPort = tpl_2(a);
		let flag = tpl_3(a);

		Bit#(1) qidOut = outPort[2];
		Bit#(2) pidOut = truncate(outPort);

		AuroraIfcType packet = 0;
		if ( flag == 0 ) begin // Source Routing
			// Payload
			Bit#(32) address = 0;
			Bit#(32) aom = truncate(payload);
			Bit#(1) header = 0; // 0: Write, 1: Read
			Bit#(32) aomNheader = (aom << 1) | zeroExtend(header);
			// Header Part
			Bit#(8) payloadByte = 8;
			Bit#(8) startPoint = fromInteger(valueOf(IdxFPGA1));
			Bit#(8) routeCnt = 2;
			Bit#(1) sdFlag = 0;
			Bit#(8) numHops = 2;
			Bit#(32) headerPartSR = (zeroExtend(payloadByte) << 24) | (zeroExtend(startPoint) << 16) |
						(zeroExtend(routeCnt) << 9) | (zeroExtend(sdFlag) << 8) |
						(zeroExtend(numHops));
			// Encryption
			// Payload
			Bit#(32) encAddress = address ^ fromInteger(valueOf(PubKeyFPGA2));
			Bit#(32) encAomNheader = aomNheader ^ fromInteger(valueOf(PubKeyFPGA2));
			// Header Part
			Bit#(32) encHeaderPartSR = headerPartSR ^ fromInteger(valueOf(PubKeyFPGA2));
			// Final
			packet = (zeroExtend(encAddress) << 64) | (zeroExtend(encAomNheader) << 32) | (zeroExtend(encHeaderPartSR));
		end else begin // Data Sending
			// Header Part
			Bit#(8) payloadByte = 8;
			Bit#(8) startPoint = fromInteger(valueOf(IdxFPGA1));
			Bit#(8) routeCnt = 2;
			Bit#(1) sdFlag = 1;
			Bit#(8) numHops = 2;
			Bit#(32) headerPartDS = (zeroExtend(payloadByte) << 24) | (zeroExtend(startPoint) << 16) | 
						(zeroExtend(routeCnt) << 9) | (zeroExtend(sdFlag) << 8) | 
						(zeroExtend(numHops));
			// Encryption
			// Payload
			Bit#(128) encData = payload ^ fromInteger(valueOf(PubKeyFPGA2));
			// Header Part
			Bit#(32) encHeaderPartDS = headerPartDS ^ fromInteger(valueOf(PubKeyFPGA2));
			// Final
			packet = (zeroExtend(encData) << 32) | (zeroExtend(encHeaderPartDS));
		end
		auroraQuads[qidOut].user[pidOut].send(AuroraSend{packet:packet,num:3});
	endrule
	//--------------------------------------------------------------------------------------------
	// Receive Packets from FPGA2 to FPGA1 via Aurora Protocol
	//--------------------------------------------------------------------------------------------
	Vector#(4, FIFO#(AuroraIfcType)) auroraReceiverQs <- replicateM(mkFIFO);
	FIFOF#(AuroraIfcType) auroraReceiverQ <- mkSizedFIFOF(1024);
	FIFO#(Bit#(128)) dataQ <- mkSizedBRAMFIFO(1024);
	rule recvAuroraPackets;
		for ( Integer i = 0; i < 4; i = i + 1 ) begin
			let payload <- auroraQuads[0].user[i].receive;
 			auroraReceiverQs[i].enq(payload);
		end
	endrule
	rule recvAuroraPackets_2;
		for ( Integer i = 0; i < 4; i = i + 1 ) begin
			let payload = auroraReceiverQs[i].first;
 			auroraReceiverQ.enq(payload);
			auroraReceiverQs[i].deq;
		end
	endrule
	rule packetDecrypter(auroraReceiverQ.notEmpty);
		auroraReceiverQ.deq;
		let recvPacket = auroraReceiverQ.first;

		Bit#(32) headerPart = recvPacket[31:0] ^ fromInteger(valueOf(PrivKeyFPGA1));
		Bit#(8) numHops = headerPart[7:0];
		Bit#(24) packetHeader = headerPart[31:8];
		Bit#(8) payloadByte = packetHeader[23:16];

		if ( numHops != 0 ) begin
			Bit#(8) outPortFPGA1 = recvPacket[39:32] ^ fromInteger(valueOf(PrivKeyFPGA1));
			Bit#(1) qidOut = outPortFPGA1[2];
			Bit#(2) pidOut = truncate(outPortFPGA1);

			Bit#(8) newNumHops = numHops - 1;
			Bit#(32) newHeaderPart = (zeroExtend(packetHeader) << 8) | zeroExtend(newNumHops);
			Bit#(32) encNewHeaderPartTmp = newHeaderPart ^ fromInteger(valueOf(PubKeyFPGA2));
			AuroraIfcType encNewHeaderPart = zeroExtend(encNewHeaderPartTmp);

			AuroraIfcType remainingPacket = recvPacket >> 40;
			AuroraIfcType newPacket = (remainingPacket << 32) | encNewHeaderPart;

			auroraQuads[qidOut].user[pidOut].send(AuroraSend{packet:newPacket,num:3});
		end else begin
			AuroraIfcType payload = recvPacket >> 32;
			if ( packetHeader[0] == 0 ) begin // Source Routing
				Bit#(32) aomNheader = payload[31:0] ^ fromInteger(valueOf(PrivKeyFPGA1));
				Bit#(32) address = payload[63:32] ^ fromInteger(valueOf(PrivKeyFPGA1)); 
				
			end else if ( packetHeader[0] == 1 ) begin // Data Sending
				Bit#(128) data = payload[127:0] ^ fromInteger(valueOf(PrivKeyFPGA1));

				dataQ.enq(data);
			end
		end
	endrule
endmodule
