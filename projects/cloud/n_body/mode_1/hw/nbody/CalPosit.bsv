import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;

Integer totalParticles = 1024;


interface CalPositPeIfc;
	method Action putA(Vector#(3, Bit#(32)) a);
	method Action putV(Vector#(3, Bit#(32)) v);
	method Action putP(Vector#(3, Bit#(32)) p);
	method ActionValue#(Vector#(3, Bit#(32))) resultGetP;
	method Bool resultExistP;
endinterface
module mkCalPositPe#(Bit#(PeWaysLog) peIdx)(CalPositPeIfc);
	FIFO#(Vector#(3, Bit#(32))) inputAQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) inputVQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) inputPQ <- mkFIFO;
	FIFOF#(Vector#(3, Bit#(32))) outputPQ <- mkFIFOF;

	Vector#(6, FpPairIfc#(32)) fpAdd32 <- replicateM(mkFpAdd32);
	Vector#(3, FpPairIfc#(32)) fpMult32 <- replicateM(mkFpMult32);

	rule calPos1;
		inputAQ.deq;
		let a = inputAQ.first;
		Bit#(32) scale = 32'b00111111000000000000000000000000;
		
		for ( Integer x = 0; x < 3; x = x + 1 ) fpMult32[x].enq(scale, a[x]);
	endrule
	rule calPos2;
		inputVQ.deq;
		let v = inputVQ.first;
		
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpMult32[x].deq;
			fpAdd32[x].enq(fpMult32[x].first, v[x]);
		end
	endrule
	rule calPos3;
		inputPQ.deq;
		let p = inputPQ.first;

		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpAdd32[x].deq;
			fpAdd32[x+3].enq(fpAdd32[x].first, p[x]);
		end
	endrule
	rule calPos4;
		Vector#(3, Bit#(32)) fr = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpAdd32[x+3].deq;
			fr[x] = fpAdd32[x+3].first;
		end
		
		outputPQ.enq(fr);
	endrule

	method Action putA(Vector#(3, Bit#(32)) a);
		inputAQ.enq(a);
	endmethod
	method Action putP(Vector#(3, Bit#(32)) p);
		inputPQ.enq(p);
	endmethod 
	method Action putV(Vector#(3, Bit#(32)) v);
		inputVQ.enq(v);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) resultGetP;
		outputPQ.deq;
		return outputPQ.first;
	endmethod
	method Bool resultExistP;
		return outputPQ.notEmpty;
	endmethod
endmodule


interface CalPositIfc;
	method Action aIn(Vector#(3, Bit#(32)) a);
	method Action vIn(Vector#(3, Bit#(32)) v);
	method Action pIn(Vector#(3, Bit#(32)) p);
	method ActionValue#(Vector#(3, Bit#(32))) pOut;
endinterface

(* synthesize *)
module mkCalPosit(CalPositIfc);
	Vector#(PeWays, CalPositPeIfc) pes;
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) aInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) vInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) pInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) pOutQs <- replicateM(mkFIFO);

	for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
		pes[i] <- mkCalPositPe(fromInteger(i));

		Reg#(Bit#(16)) aInIdx <- mkReg(0);
		rule forwardAccel;
			aInQs[i].deq;
			let d = aInQs[i].first;
			if ( i < (valueOf(PeWays) - 1) ) begin
				aInQs[i+1].enq(d);
			end

			aInIdx <= aInIdx + 1;
			Bit#(PeWaysLog) target_a = truncate(aInIdx);
			if ( target_a == fromInteger(i) ) begin
				pes[i].putA(d);
			end
		endrule
		Reg#(Bit#(16)) pInIdx <- mkReg(0);
		rule forwardPosit;
			pInQs[i].deq;
			let d = pInQs[i].first;
			if ( i < (valueOf(PeWays) - 1) ) begin
				pInQs[i+1].enq(d);
			end

			pInIdx <= pInIdx + 1;
			Bit#(PeWaysLog) target_p = truncate(pInIdx);
			if ( target_p == fromInteger(i) ) begin
				pes[i].putP(d);
			end
		endrule
		Reg#(Bit#(16)) vInIdx <- mkReg(0);
		rule forwardVeloc;
			vInQs[i].deq;
			let d = vInQs[i].first;
			if ( i < (valueOf(PeWays) - 1) ) begin
				vInQs[i+1].enq(d);
			end

			vInIdx <= vInIdx + 1;
			Bit#(PeWaysLog) target_v = truncate(vInIdx);
			if ( target_v == fromInteger(i) ) begin
				pes[i].putV(d);
			end
		endrule
		rule forwardResultP;
			if ( pes[i].resultExistP ) begin
				let d <- pes[i].resultGetP;
				pOutQs[i].enq(d);
			end else if ( i < (valueOf(PeWays) - 1) ) begin
				pOutQs[i+1].deq;
				pOutQs[i].enq(pOutQs[i+1].first);
			end
		endrule
	end
	method Action aIn(Vector#(3, Bit#(32)) a);
		aInQs[0].enq(a);
	endmethod
	method Action vIn(Vector#(3, Bit#(32)) v);
		vInQs[0].enq(v);
	endmethod
	method Action pIn(Vector#(3, Bit#(32)) p);
		pInQs[0].enq(p);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) pOut;
		pOutQs[0].deq;
		return pOutQs[0].first;
	endmethod
endmodule
