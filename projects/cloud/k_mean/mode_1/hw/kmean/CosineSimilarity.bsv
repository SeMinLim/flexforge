import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;


interface CosineSimilarityIfc;
	method Action aIn(Vector#(2, Bit#(32)) a);
	method Action bIn(Vector#(2, Bit#(32)) b);
	method ActionValue#(Bit#(32)) csOut;
endinterface
(* synthesize *)
module mkCosineSimilarity;
	Vector#(2, FIFO#(Vector#(2, Bit#(32)))) aInQs <- mkFIFO;
	Vector#(2, FIFO#(Vector#(2, Bit#(32)))) bInQs <- mkFIFO;
	FIFO#(Bit#(32)) csOutQ <- mkFIFO;
	
	// Get a's size
	Vector#(2, FpPairIfc#(32)) aSize_fpMult32 <- replicateM(mkFpMult32);
	FpPairIfc#(32) aSize_fpAdd32 <- mkFpAdd32;
	FpFilterIfc#(32) aSize_fpSqrt32 <- mkFpSqrt32;
	FIFO#(Bit#(32)) aSizeQ <- mkFIFO;
	rule aSize_1;
		aInQs[0].deq;
		let a = aInQs[0].first;
		
		aSize_fpMult32[0].enq(a[0], a[0]);
		aSize_fpMult32[1].enq(a[1], a[1]);
		
		aInQs[1].enq(a);
	endrule
	rule aSize_2;
		aSize_fpMult32[0].deq;
		aSize_fpMult32[1].deq;
		let ax = aSize_fpMult32[0].first;
		let ay = aSize_fpMult32[1].first;

		aSize_fpAdd32.enq(ax, ay);
	endrule
	rule aSize_3;
		aSize_fpAdd32.deq;
		let aSizeTmp = aSize_fpAdd32.first;
		aSize_fpSqrt32.enq(aSizeTmp);
	endrule
	rule aSize_4;
		aSize_fpSqrt32.deq;
		let aSize = aSize_fpSqrt32.first;
		aSizeQ.enq(aSize);
	endrule
	
	// Get b's size
	Vector#(2, FpPairIfc#(32)) bSize_fpMult32 <- replicateM(mkFpMult32);
	FpPairIfc#(32) bSize_fpAdd32 <- mkFpAdd32;
	FpFilterIfc#(32) bSize_fpSqrt32 <- mkFpSqrt32;
	FIFO#(Bit#(32)) bSizeQ <- mkFIFO;
	rule bSize_1;
		bInQs[0].deq;
		let b = bInQs[0].first;

		bSize_fpMult32[0].enq(b[0], b[0]);
		bSize_fpMult32[1].enq(b[1], b[1]);

		bInQs[1].enq(b);
	endrule
	rule bSize_2;
		bSize_fpMult32[0].deq;
		bSize_fpMult32[1].deq;
		let bx = bSize_fpMult32[0].first;
		let by = bSize_fpMult32[1].first;

		bSize_fpAdd32.enq(bx, by);
	endrule
	rule bSize_3;
		bSize_fpAdd32.deq;
		let bSizeTmp = bSize_fpAdd32.first;
		bSize_fpSqrt32.enq(bSizeTmp);
	endrule
	rule bSize_4;
		bSize_fpSqrt32.deq;
		let bSize = bSize_fpSqrt32.first;
		bSizeQ.enq(bSize);
	endrule

	// Get denominator value
	FpPairIfc#(32) denom_fpMult32 <- mkFpMult32;
	FIFO#(Bit#(32)) denomQ <- mkFIFO;
	rule denominator_1;
		aSizeQ.deq;
		bSizeQ.deq;
		let aSize = aSizeQ.first;
		let bSize = bSizeQ.first;
		
		denom_fpMult32.enq(aSize, bSize);
	endrule
	rule denominator_2;
		denom_fpMult32.deq;
		let denom = denom_fpMult32.first;
		denomQ.enq(denom);
	endrule

	// Get numerator value
	Vector#(2, FpPairIfc#(32)) numer_fpMult32 <- replicateM(mkFpMult32);
	FpPairIfc#(32) numer_fpAdd32 <- mkFpAdd32;
	FIFO#(Bit#(32)) numerQ <- mkFIFO;
	rule numerator_1;
		aInQs[1].deq;
		bInQs[1].deq;
		let a = aInQs[1].first;
		let b = bInQs[1].first;

		numer_fpMult32[0].enq(a[0], b[0]);
		numer_fpMult32[1].enq(a[1], b[1]);
	endrule
	rule numerator_2;
		numer_fpMult32[0].deq;
		numer_fpMult32[1].deq;
		let x = numer_fpMult32[0].first;
		let y = numer_fpMult32[1].first;

		numer_fpAdd32.enq(x, y);
	endrule
	rule numerator_3;
		numer_fpAdd32.deq;
		let numer = numer_fpAdd32.first;
		numerQ.enq(numer);
	endrule
	
	// Get cosine similarity value
	FpPairIfc#(32) cs_fpDiv32 <- mkFpDiv32;
	rule cosine_similarity_1;
		denomQ.deq;
		numerQ.deq;
		let denom = denomQ.first;
		let numer = numerQ.first;

		cs_fpDiv32.enq(numer, denom);
	endrule
	rule cosine_similarity_2;
		cs_fpDiv32.deq;
		let cs = cs_fpDiv32.first;
		csOutQ.enq(cs);
	endrule

	method Action aIn(Vector#(2, Bit#(32)) a);
		aInQs[0].enq(a);
	endmethod
	method Action bIn(Vector#(2, Bit#(32)) b);
		bInQs[0].enq(b);
	endmethod
	method ActionValue#(Bit#(32)) csOut;
		csOutQ.deq;
		return csOutQ.first;
	endmethod
endmodule
