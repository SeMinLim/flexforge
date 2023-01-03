import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;


interface KmeanPeIfc;
endinterface
module mkKmeanPe#(Bit#(PeWaysLog) peIdx) (KmeanPeIfc);
endmodule

interface KmeanSubIfc;
	method Action chIn(Vector#(2, Bit#(32)) ch);
	method Action dIn(Vector#(3, Bit#(32)) d);
	method ActionValue#(Vector#(3, Bit#(32))) csOut;
endinterface
module mkKmeanSub(KmeanSubIfc);

endmodule

interface KmeanIfc;
	method Action initSet;
	method Action dataIn(Vector#(3, Bit#(32)) d);
	method ActionValue#(Vector#(3, Bit#(32))) dataOut;
endinterface
module mkKmean(KmeanIfc);
	KmeanSubIfc kmeanSub <- mkKmeanSub;

	FIFO#(Vector#(3, Bit#(32))) dInQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) dOutQ <- mkFIFO;
	FIFO#(Vector#(2, Bit#(32))) clusterHeadQ <- mkFIFO;
	Reg#(Vector#(3, Bit#(32))) clusterHeadPrevBuffer <- mkReg(0);

	Reg#(Bit#(32)) initSetBuffer <- mkReg(0);
	Reg#(Bit#(8)) initSetCnt <- mkReg(0);
	rule clusterHead_InitSet( initialSet );
		Vector#(2, Bit#(32)) currH = replicate(0);

		if ( initSetCnt != 0 ) begin
			let prevH = initSetBuffer;
			currH[0] = prevH[0] + 32'b01000001001000000000000000000000;
			currH[1] = prevH[1] + 32'b01000001001000000000000000000000;
			clusterHeadQ.enq(currH);
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
			clusterHeadQ.enq(currH);
			clusterHeadPrevBuffer <= currH;
			initSetBuffer <= currH;
			initSetCnt <= initSetCnt + 1;
		end
	endrule
	
	rule relay_ClusterHead;
		clusterHeadQ.deq;
		let ch = clusterHeadQ.first;
		kmeanSub.chIn(ch);
	endrule

	rule relay_Data;
		dataInQ.deq;
		let d = dataInQ.first;
		kmeanSub.dIn(d);
	endrule

	rule recv_csResult;
		let cs <- kmeanSub.csResultOut;
		minDetector.

	rule clusterHead_Updater();
	endrule

	method Action initSet;
		initialSet <= True;
	endmethod
	method Action dataIn(Vector#(3, Bit#(32)) d);
		dInQ.enq(d);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) dataOut;
		dOutQ.deq;
		return dOutQ.first;
	endmethod
endmodule
