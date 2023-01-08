import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

import SizeCalculator::*;


typedef 1024 Dimension;
typedef 16 ByteWords;
typedef TDiv#(Dimension, ByteWords) Accumulate;


interface CosineSimilarityIfc;
	method Action aIn(Bit#(512) a);
	method ActionValue#(Bit#(32)) csOut;
endinterface
(* synthesize *)
module mkCosineSimilarity(CosineSimilarityIfc);
	SizeCalculatorIfc sizeCalculator <- mkSizeCalculator;

	Vector#(2, FIFO#(Bit#(512))) aInQs <- replicateM(mkSizedBRAMFIFO(128));
	FIFO#(Bit#(32)) csOutQ <- mkFIFO;
	
	// Get a's size
	FIFO#(Bit#(32)) aSizeQ <- mkSizedBRAMFIFO(32);
	rule aSize_1;
		aInQs[0].deq;
		let a = aInQs[0].first;
		
		aInQs[1].enq(a);
		sizeCalculator.enq(a);	
	endrule
	rule aSize_2;
		let aSize <- sizeCalculator.deq;
		csOutQ.enq(aSize);
	endrule
	
	// Get numerator value
	Vector#(16, FpPairIfc#(32)) numer_fpMult32 <- replicateM(mkFpMult32);
	Reg#(Bit#(16)) numerCnt <- mkReg(0);
	rule numerator_1;
		aInQs[1].deq;
		let a = aInQs[1].first;
		Bit#(32) b = 32'b00111111100000000000000000000000;

		for ( Integer i = 0; i < 16; i = i + 1 ) begin
			let av1 = a[((32*(i+1))-1):(32*i)];
			numer_fpMult32[i].enq(av1, b);
		end
	endrule
	rule numerator_2;
		Vector#(16, Bit#(32)) nv2 = replicate(0);
		for ( Integer i = 0; i < 16; i = i + 1 ) begin
			numer_fpMult32[i].deq;
			nv2[i] = numer_fpMult32[i].first;
		end
		
		if ( numerCnt + 1 == fromInteger(valueOf(Accumulate)) ) begin
			numerCnt <= 0;
		end else begin
			numerCnt <= numerCnt + 1;
		end
	endrule

	method Action aIn(Bit#(512) a);
		aInQs[0].enq(a);
	endmethod
	method ActionValue#(Bit#(32)) csOut;
		csOutQ.deq;
		return csOutQ.first;
	endmethod
endmodule
