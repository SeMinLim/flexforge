import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

typedef 1024 NumData;
typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;
typedef TDiv#(NumData, PeWays) Cluster;

interface PeIfc;
	method Action putClusterHead(Vector#(2, Bit#(32)) ch);
	method Action putData(Vector#(3, Bit#(32)) d);
	method ActionValue#(Tuple2#(Vector#(3, Bit#(32)), Bit#(32))) getCosSim; // data, cossim
endinterface
module mkPe#(Bit#(PeWaysLog) peIdx) (PeIfc);
	CosinSimilarityIfc cossim <- mkCosineSimialrity;

	FIFO#(Vector#(2, Bit#(32))) putChQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) putDataQ <- mkFIFO;

	FIFO#(Vector#(3, Bit#(32))) getDataQ <- mkFIFO;
	FIFO#(Tuple2#(Vector#(3, Bit#(32)), Bit#(32))) getCosSimQ <- mkFIFO;

	rule relayValues;
		putChQ.deq;
		putDataQ.deq;
		let d = putDataQ.first;
		
		let a = putChQ.first;
		Vector#(2, Bit#(32)) b = replicate(0);
		b[0] = d[0];
		b[1] = d[1];

		cossim.aIn(a);	
		cossim.bIn(b);
		getDataQ.enq(d);
	endrule
	
	rule getValue;
		getDataQ.deq;
		let d = getDataQ.first;
		let cs <- cossim.csOut;

		getCosSimQ.enq(tuple2(d, cs));
	endrule

	method Action putClusterHead(Vector#(2, Bit#(32)) ch);
		putChQ.enq(ch);
	endmethod
	method Action putData(Vector#(3, Bit#(32)) d);
		putDataQ.enq(d);
	endmethod
	method ActionValue#(Tuple2#(Vector#(3, Bit#(32)), Bit#(32))) getCosSim;
		getCosSimQ.deq;
		return getCosSimQ.first;
	endmethod
endmodule

interface KmeanSubIfc;
	method Action clusterheadIn(Vector#(2, Bit#(32)) ch);
	method Action dataIn(Vector#(3, Bit#(32)) d);
	method ActionValue#(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog)) cosSimOut; // data, cossim, idx
endinterface
module mkKmeanSub(KmeanSubIfc);
	Vector#(PeWays, PeIfc) pes;
	Vector#(PeWays, FIFO#(Vector#(2, Bit#(32)))) clusterHeadInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) dataInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Tuple2#(Vector#(3, Bit#(32)), Bit#(32)))) cosSimOutQs <- replicateM(mkFIFO);
	
	FIFO#(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog))) cosSimOutQ <- mkFIFO;

	for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
		pes[i] <- mkPe(fromInteger(i));
		
		Reg#(Bit#(32)) chInIdx <- mkReg(0);
		rule forwardClusterHead;
			clusterHeadInQs[i].deq;
			let ch = clusterHeadInQs[i].first;
			if ( i < valueOf(PeWays) - 1 ) begin
				clusterHeadInQs[i+1].enq(ch);
			end

			chInIdx <= chInIdx + 1;
			Bit#(PeWaysLog) target_ch = truncate(chInIdx);
			if ( target_ch == fromInteger(i) ) begin
				pes[i].putClusterHead(ch);
			end
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
			Bit#(PeWaysLog) idx = fromInteger(i);
			cosSimOutQ.enq(tuple3(tpl_1(cs), tpl_2(cs), idx));
		endrule
	end
	method Action clusterHeadIn(Vector#(2, Bit#(32)) ch);
		clusterHeadInQs[0].enq(ch);
	endmethod
	method Action dataIn(Vector#(3, Bit#(32)) d);
		dataInQs[0].enq(d);
	endmethod
	method ActionValue#(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog))) cosSimOut;
		cosSimOutQ.deq;
		return cosSimOutQ.first;
	endmethod
endmodule

interface KmeanIfc;
	method Action initSet;
	method Action dIn(Vector#(3, Bit#(32)) d);
	method ActionValue#(Vector#(3, Bit#(32))) dOut;
	method ActionValue#(Bit#(1)) fOut;
endinterface
module mkKmean(KmeanIfc);
	KmeanSubIfc kmeanSub <- mkKmeanSub;
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) clusterQ <- replicateM(mkSizedBRAMFIFO(Cluster));

	FIFO#(Vector#(3, Bit#(32))) dInQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) dOutQ <- mkFIFO;
	FIFO#(Vector#(2, Bit#(32))) clusterHeadInQ <- mkFIFO;
	Reg#(Vector#(3, Bit#(32))) clusterHeadPrevBuffer <- mkReg(0);

	Reg#(Bit#(32)) initSetBuffer <- mkReg(0);
	Reg#(Bit#(8)) initSetCnt <- mkReg(0);
	rule clusterHead_InitSet( initialSet );
		Vector#(2, Bit#(32)) currH = replicate(0);

		if ( initSetCnt != 0 ) begin
			let prevH = initSetBuffer;
			currH[0] = prevH[0] + 32'b01000001001000000000000000000000;
			currH[1] = prevH[1] + 32'b01000001001000000000000000000000;
			clusterHeadInQ.enq(currH);
			initSetBuffer <= currH;

			if ( initSetCnt + 1 == fromInteger(valueOf(PeWays)) ) begin
				initSetCnt <= 0;
				initialSet <= False;
			end else begin
				initSetCnt <= initSetCnt + 1;
			end
		end else begin
			currH[0] = 0;
			currH[1] = 0;
			clusterHeadInQ.enq(currH);
			clusterHeadPrevBuffer <= currH;
			initSetBuffer <= currH;
			initSetCnt <= initSetCnt + 1;
		end
	endrule
	
	Reg#(Bit#(16)) relayChCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) relayChCnt_2 <- mkReg(0);
	rule relay_ClusterHead;
		clusterHeadInQ.deq;
		let ch = clusterHeadInQ.first;
		kmeanSub.clusterHeadIn(ch);

		if ( relayChCnt_1 + 1 == fromInteger(valueOf(NumData)) ) begin
			if ( relayChCnt_2 + 1 == fromInteger(valueOf(PeWays)) ) begin
				relayChCnt_1 <= 0;
				relayChCnt_2 <= 0;
			end else begin
				relayChCnt_2 <= relayChCnt_2 + 1;
			end
		end else begin
			clusterHeadInQ.enq(ch);
			if ( relayChCnt_2 + 1 == fromInteger(valueOf(PeWays)) ) begin
				relayChCnt_1 <= relayChCnt_1 + 1;
				relayChCnt_2 <= 0;
			end else begin
				relayChCnt_2 <= relayChCnt_2 + 1;
			end
		end
	endrule

	rule relay_Data;
		dInQ.deq;
		let d = dInQ.first;
		kmeanSub.dataIn(d);
	endrule

	rule relay_CosSim;
		let cs <- kmeanSub.cosSimOut;
		minDetector.vIn(cs);
	endrule

	rule recv_Min;
		let m <- minDetector.minOut;
		let data = tpl_1(m);
		let idx = tpl_2(m);

		for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
			if ( i == idx ) begin
				Vector#(3, Bit#(32)) d = replicate(0);
				Bit#(32) finalIdx = fromInteger(idx);
				d[0] = data[0];
				d[1] = data[1];
				d[2] = finalIdx;

				clusterQ[i].enq(d);
			end
		end
	endrule

	rule get_Mean_Phase_1;
		for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
			clusterQ[i].deq;
			d = clusterQ[i].first;
			meanCalculator[i].dataIn(d);
		end
	endrule

	rule get_Mean_Phase_2;
		for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
			let mean <- meanCaculator[i].meanOut;
			clusterHeadInQ.enq(mean);
			if ( i == 0 ) begin
				clusterHeadCurrBuffer <= mean;

			end
		end
	endrule

	rule clusterHead_Updater_1;
		let prevCh = clusterHeadPrevBuffer;
		let currCh = clusterHeadCurrBuffer;
		fpCmpr32.enq(prevCh, currCh);
	endrule

	rule clusterHead_Updater_2;
		fpCmpr32.deq;
		let flag = fpCmpr32.first;
		fOutQ.enq(flag);
	endrule
	method Action initSet;
		initialSet <= True;
	endmethod
	method Action dIn(Vector#(3, Bit#(32)) d);
		dInQ.enq(d);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) dOut;
		dOutQ.deq;
		return dOutQ.first;
	endmethod
	method ActionValue#(Bit#(1)) fOut;
		fOutQ.deq;
		return fOutQ.first;
	endmethod
endmodule
