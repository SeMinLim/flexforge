import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

import CosineSimilarity::*;

typedef 64 NumData;
typedef 1024 Dimension;
typedef 5 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;


interface PeIfc;
	method Action putClusterHead(Bit#(32) ch);
	method Action putData(Bit#(32) d);
	method ActionValue#(Bit#(32)) getCosSim;
endinterface
module mkPe#(Bit#(PeWaysLog) peIdx) (PeIfc);
	CosineSimilarityIfc cossim <- mkCosineSimilarity;

	FIFO#(Bit#(32)) putChQ <- mkFIFO;
	FIFO#(Bit#(32)) putDataQ <- mkFIFO;
	FIFO#(Bit#(32)) getCosSimQ <- mkFIFO;

	rule relayValues;
		putChQ.deq;
		putDataQ.deq;		
		let a = putChQ.first;
		let b = putDataQ.first;

		cossim.aIn(a);	
		cossim.bIn(b);
	endrule
	
	rule getValue;
		let cs <- cossim.csOut;
		getCosSimQ.enq(cs);
	endrule

	method Action putClusterHead(Bit#(32) ch);
		putChQ.enq(ch);
	endmethod
	method Action putData(Bit#(32) d);
		putDataQ.enq(d);
	endmethod
	method ActionValue#(Bit#(32)) getCosSim;
		getCosSimQ.deq;
		return getCosSimQ.first;
	endmethod
endmodule

interface KmeanSubIfc;
	method Action clusterHeadIn(Bit#(32) ch);
	method Action dataIn(Bit#(32) d);
	method ActionValue#(Bit#(32)) cosSimOut;
endinterface
module mkKmeanSub(KmeanSubIfc);
	Vector#(PeWays, PeIfc) pes;
	Vector#(PeWays, FIFO#(Bit#(32))) clusterHeadInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Bit#(32))) dataInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Bit#(32))) cosSimOutQs <- replicateM(mkFIFO);
	FIFO#(Bit#(32)) cosSimOutQ <- mkFIFO;

	for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
		pes[i] <- mkPe(fromInteger(i));
		
		rule forwardClusterHead;
			clusterHeadInQs[i].deq;
			let ch = clusterHeadInQs[i].first;
			if ( i < valueOf(PeWays) - 1 ) begin
				clusterHeadInQs[i+1].enq(ch);
			end
			pes[i].putClusterHead(ch);
		endrule

		rule forwardData;
			dataInQs[i].deq;
			let d = dataInQs[i].first;
			if ( i < valueOf(PeWays) - 1 ) begin
				dataInQs[i+1].enq(d);
			end
			pes[i].putData(d);
		endrule

		rule getCosSim;
			let cs <- pes[i].getCosSim;
			cosSimOutQs[i].enq(cs);
		endrule

		rule forwardCosSim;
			cosSimOutQs[i].deq;
			let cs = cosSimOutQs[i].first;
			cosSimOutQ.enq(cs);
		endrule
	end
	method Action clusterHeadIn(Bit#(32) ch);
		clusterHeadInQs[0].enq(ch);
	endmethod
	method Action dataIn(Bit#(32) d);
		dataInQs[0].enq(d);
	endmethod
	method ActionValue#(Bit#(32)) cosSimOut;
		cosSimOutQ.deq;
		return cosSimOutQ.first;
	endmethod
endmodule

interface KmeanIfc;
	method Action clusterHeadIn(Bit#(32) ch);
	method Action dataIn(Bit#(32) d);
	method ActionValue#(Bit#(1)) dataOut;
endinterface
module mkKmean(KmeanIfc);
	KmeanSubIfc kmeanSub <- mkKmeanSub;

	// Cycle Counter
	FIFOF#(Bit#(32)) cycleQ <- mkFIFOF;
	Reg#(Bit#(32)) cycleCount <- mkReg(0);
	Reg#(Bit#(32)) cycleStart <- mkReg(0);
	Reg#(Bit#(32)) cycleEnd <- mkReg(0);
	rule incCycleCount;
		cycleCount <= cycleCount + 1;
	endrule

	FIFO#(Bit#(32)) chInQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Bit#(32)) dInQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Bit#(1)) dOutQ <- mkFIFO;

	// Relay cluster head	
	Reg#(Bit#(16)) relayChCnt <- mkReg(0);
	rule relay_ClusterHead;
		chInQ.deq;
		let ch = chInQ.first;
		kmeanSub.clusterHeadIn(ch);
		relayChCnt <= relayChCnt + 1;
		//$write("\033[1;33mCycle %1d -> \033[1;33m[Kmean]: \033[0m: Relay Cluster Head to PE \033[1;32m[%d]\n", cycleCount, relaychCnt);
	endrule

	// Relay data
	Reg#(Bit#(16)) relayDataCnt <- mkReg(0);
	rule relay_Data;
		dInQ.deq;
		let d = dInQ.first;
		kmeanSub.dataIn(d);
		relayDataCnt <= relayDataCnt + 1;
		//$write("\033[1;33mCycle %1d -> \033[1;33m[Kmean]: \033[0m: Relay Data to PE \033[1;32m[%d]\n", cycleCount, relayDataCnt);
	endrule

	// Receive cosine similarity value
	Reg#(Bit#(32)) relayCsCnt_1 <- mkReg(0);
	Reg#(Bit#(32)) relayCsCnt_2 <- mkReg(0);
	rule relay_CosSim;
		let cs <- kmeanSub.cosSimOut;
		if ( relayCsCnt_1 + 1 == fromInteger(valueOf(PeWays)) ) begin
			if ( relayCsCnt_2 + 1 == fromInteger(valueOf(NumData)) ) begin
				relayCsCnt_2 <= 0;
				dOutQ.enq(1);
			end else begin
				relayCsCnt_2 <= relayCsCnt_2 + 1;
			end
			relayCsCnt_1 <= 0;
			$write("\033[1;33mCycle %1d -> \033[1;33m[Kmean]: \033[0m: Get Minimum Value \033[1;32m[%d]\n", cycleCount, relayCsCnt_2);
		end else begin
			relayCsCnt_1 <= relayCsCnt_1 + 1;
		end
	endrule
	
	method Action clusterHeadIn(Bit#(32) ch);
		chInQ.enq(ch);
	endmethod
	method Action dataIn(Bit#(32) d);
		dInQ.enq(d);
	endmethod
	method ActionValue#(Bit#(1)) dataOut;
		dOutQ.deq;
		return dOutQ.first;
	endmethod
endmodule
