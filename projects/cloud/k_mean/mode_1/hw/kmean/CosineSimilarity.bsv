import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;


typedef 1024 Dimension;


interface CosineSimilarityIfc;
	method Action aIn(Bit#(32) a);
	method Action bIn(Bit#(32) b);
	method ActionValue#(Bit#(32)) csOut;
endinterface
(* synthesize *)
module mkCosineSimilarity(CosineSimilarityIfc);
	Vector#(2, FIFO#(Bit#(32))) aInQs <- replicateM(mkSizedBRAMFIFO(128));
	Vector#(2, FIFO#(Bit#(32))) bInQs <- replicateM(mkSizedBRAMFIFO(128));
	FIFO#(Bit#(32)) csOutQ <- mkFIFO;
	
	// Get a's size
	FpPairIfc#(32) aSize_fpMult32 <- mkFpMult32;
	FpPairIfc#(32) aSize_fpAdd32 <- mkFpAdd32;
	FpFilterIfc#(32) aSize_fpSqrt32 <- mkFpSqrt32;
	FIFO#(Bit#(32)) aSize_fpMult32Q <- mkSizedBRAMFIFO(64);
	FIFOF#(Bit#(32)) aSizeTmpQ <- mkSizedFIFOF(64);
	FIFO#(Bit#(32)) aSizeQ <- mkSizedBRAMFIFO(64);
	Reg#(Bit#(16)) aSizeCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) aSizeCnt_2 <- mkReg(0);
	rule aSize_1;
		aInQs[0].deq;
		let a = aInQs[0].first;
		
		aSize_fpMult32.enq(a, a);
		
		aInQs[1].enq(a);
	endrule
	rule aSize_2;
		aSize_fpMult32.deq;
		let v = aSize_fpMult32.first;

		aSize_fpMult32Q.enq(v);
	endrule
	rule aSize_3;
		if ( aSizeCnt_1 != 0 ) begin
			if ( aSizeTmpQ.notEmpty ) begin
				aSize_fpMult32Q.deq;
				aSizeTmpQ.deq;
				let curr_ax = aSize_fpMult32Q.first;
				let prev_ax = aSizeTmpQ.first;

				aSize_fpAdd32.enq(prev_ax, curr_ax);

				if ( aSizeCnt_1 + 1 == fromInteger(valueOf(Dimension)) ) begin
					aSizeCnt_1 <= 0;
				end else begin
					aSizeCnt_1 <= aSizeCnt_1 + 1;
				end
			end
		end else begin
			aSize_fpMult32Q.deq;
			let curr_ax = aSize_fpMult32Q.first;
		
			aSizeTmpQ.enq(curr_ax);
		
			aSizeCnt_1 <= aSizeCnt_1 + 1;
		end
	endrule
	rule aSize_4;
		aSize_fpAdd32.deq;
		let aSizeTmp = aSize_fpAdd32.first;

		if ( aSizeCnt_2 + 2 == fromInteger(valueOf(Dimension))) begin
			aSize_fpSqrt32.enq(aSizeTmp);
			aSizeCnt_2 <= 0;
		end else begin
			aSizeTmpQ.enq(aSizeTmp);
			aSizeCnt_2 <= aSizeCnt_2 + 1;
		end
	endrule
	rule aSize_5;
		aSize_fpSqrt32.deq;
		let aSize = aSize_fpSqrt32.first;
		aSizeQ.enq(aSize);
	endrule
	
	// Get b's size
	FpPairIfc#(32) bSize_fpMult32 <- mkFpMult32;
	FpPairIfc#(32) bSize_fpAdd32 <- mkFpAdd32;
	FpFilterIfc#(32) bSize_fpSqrt32 <- mkFpSqrt32;
	FIFO#(Bit#(32)) bSize_fpMult32Q <- mkSizedBRAMFIFO(64);
	FIFOF#(Bit#(32)) bSizeTmpQ <- mkSizedFIFOF(64);
	FIFO#(Bit#(32)) bSizeQ <- mkSizedBRAMFIFO(64);
	Reg#(Bit#(16)) bSizeCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) bSizeCnt_2 <- mkReg(0);
	rule bSize_1;
		bInQs[0].deq;
		let b = bInQs[0].first;
		
		bSize_fpMult32.enq(b, b);
		
		bInQs[1].enq(b);
	endrule
	rule bSize_2;
		bSize_fpMult32.deq;
		let v = bSize_fpMult32.first;

		bSize_fpMult32Q.enq(v);
	endrule
	rule bSize_3;
		if ( bSizeCnt_1 != 0 ) begin
			if ( bSizeTmpQ.notEmpty ) begin
				bSize_fpMult32Q.deq;
				bSizeTmpQ.deq;
				let curr_bx = bSize_fpMult32Q.first;
				let prev_bx = bSizeTmpQ.first;

				bSize_fpAdd32.enq(prev_bx, curr_bx);

				if ( bSizeCnt_1 + 1 == fromInteger(valueOf(Dimension)) ) begin
					bSizeCnt_1 <= 0;
				end else begin
					bSizeCnt_1 <= bSizeCnt_1 + 1;
				end
			end
		end else begin
			bSize_fpMult32Q.deq;
			let curr_bx = bSize_fpMult32Q.first;
		
			bSizeTmpQ.enq(curr_bx);
		
			bSizeCnt_1 <= bSizeCnt_1 + 1;
		end
	endrule
	rule bSize_4;
		bSize_fpAdd32.deq;
		let bSizeTmp = bSize_fpAdd32.first;

		if ( bSizeCnt_2 + 2 == fromInteger(valueOf(Dimension))) begin
			bSize_fpSqrt32.enq(bSizeTmp);
			bSizeCnt_2 <= 0;
		end else begin
			bSizeTmpQ.enq(bSizeTmp);
			bSizeCnt_2 <= bSizeCnt_2 + 1;
		end
	endrule
	rule bSize_5;
		bSize_fpSqrt32.deq;
		let bSize = bSize_fpSqrt32.first;
		bSizeQ.enq(bSize);
	endrule
	
	// Get denominator value
	FpPairIfc#(32) denom_fpMult32 <- mkFpMult32;
	FIFO#(Bit#(32)) denomQ <- mkSizedBRAMFIFO(64);
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
	FpPairIfc#(32) numer_fpMult32 <- mkFpMult32;
	FpPairIfc#(32) numer_fpAdd32 <- mkFpAdd32;
	FIFO#(Bit#(32)) numer_fpMult32Q <- mkSizedBRAMFIFO(64);
	FIFOF#(Bit#(32)) numerTmpQ <- mkSizedFIFOF(64);
	FIFO#(Bit#(32)) numerQ <- mkSizedBRAMFIFO(64);
	Reg#(Bit#(16)) numerCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) numerCnt_2 <- mkReg(0);
	rule numerator_1;
		aInQs[1].deq;
		bInQs[1].deq;
		let a = aInQs[1].first;
		let b = bInQs[1].first;

		numer_fpMult32.enq(a, b);
	endrule
	rule numerator_2;
		numer_fpMult32.deq;
		let v = numer_fpMult32.first;

		numer_fpMult32Q.enq(v);
	endrule
	rule numerator_3;
		if ( numerCnt_1 != 0 ) begin
			if ( numerTmpQ.notEmpty ) begin
				numer_fpMult32Q.deq;
				numerTmpQ.deq;
				let curr_x = numer_fpMult32Q.first;
				let prev_x = numerTmpQ.first;

				numer_fpAdd32.enq(prev_x, curr_x);

				if ( numerCnt_1 + 1 == fromInteger(valueOf(Dimension)) ) begin
					numerCnt_1 <= 0;
				end else begin
					numerCnt_1 <= numerCnt_1 + 1;
				end
			end
		end else begin
			numer_fpMult32Q.deq;
			let curr_x = numer_fpMult32Q.first;
		
			numerTmpQ.enq(curr_x);
		
			numerCnt_1 <= numerCnt_1 + 1;
		end
	endrule
	rule numerator_4;
		numer_fpAdd32.deq;
		let numerTmp = numer_fpAdd32.first;

		if ( numerCnt_2 + 2 == fromInteger(valueOf(Dimension))) begin
			numerQ.enq(numerTmp);
			numerCnt_2 <= 0;
		end else begin
			numerTmpQ.enq(numerTmp);
			numerCnt_2 <= numerCnt_2 + 1;
		end
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

	method Action aIn(Bit#(32) a);
		aInQs[0].enq(a);
	endmethod
	method Action bIn(Bit#(32) b);
		bInQs[0].enq(b);
	endmethod
	method ActionValue#(Bit#(32)) csOut;
		csOutQ.deq;
		return csOutQ.first;
	endmethod
endmodule
