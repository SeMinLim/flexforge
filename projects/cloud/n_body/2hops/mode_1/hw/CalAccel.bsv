import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;

Integer totalParticles = 16*1024*1024;


interface CalAccelPeIfc;
	method Action putOperandI(Vector#(3, Bit#(32)) i);
	method Action putOperandJ(Vector#(4, Bit#(32)) j);
	method ActionValue#(Vector#(3, Bit#(32))) resultGet;
	method Bool resultExist;
endinterface
module mkCalAccelPe#(Bit#(PeWaysLog) peIdx) (CalAccelPeIfc);
	FIFO#(Vector#(3, Bit#(32))) inputIQ <- mkFIFO;
	FIFO#(Vector#(4, Bit#(32))) inputJQ <- mkFIFO;

	Vector#(3, FpPairIfc#(32)) fpSub32 <- replicateM(mkFpSub32);
	Vector#(3, FpPairIfc#(32)) fpAdd32 <- replicateM(mkFpAdd32);
	Vector#(9, FpPairIfc#(32)) fpMult32 <- replicateM(mkFpMult32);
	FpPairIfc#(32) fpDiv32 <- mkFpDiv32;
	FpFilterIfc#(32) fpSqrt32 <- mkFpSqrt32;

	FIFO#(Bit#(32)) massJQ <- mkFIFO;
	rule commonSub1;
		inputIQ.deq;
		inputJQ.deq;
		Vector#(3, Bit#(32)) i = inputIQ.first;
		Vector#(4, Bit#(32)) j = inputJQ.first;
		
		for ( Integer x = 0; x < 3; x = x + 1 ) fpSub32[x].enq(j[x], i[x]);
		
		massJQ.enq(j[3]);
	endrule
	FIFO#(Vector#(3, Bit#(32))) tmpResult1Q <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) subResultQ <- mkFIFO;
	rule commonSub2;
		Vector#(3, Bit#(32)) out1 = replicate(0);
		Vector#(3, Bit#(32)) out2 = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpSub32[x].deq;
			out1[x] = fpSub32[x].first;
			out2[x] = fpSub32[x].first;
		end
		
		tmpResult1Q.enq(out1);
		subResultQ.enq(out2);
	endrule
	
	rule denominatorMod1;
		tmpResult1Q.deq;
		Vector#(3, Bit#(32)) s = tmpResult1Q.first;

		for ( Integer x = 0; x < 3; x = x + 1 ) fpMult32[x].enq(s[x], s[x]);
	endrule
	FIFO#(Bit#(32)) m2Q <- mkFIFO;
	rule denominatorMod2;
		Vector#(3, Bit#(32)) m = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpMult32[x].deq;
			m[x] = fpMult32[x].first;
		end
		Bit#(32) m2 = m[0] + m[1] + m[2];
		m2Q.enq(m2);
	endrule
	rule denominatorMod3;
		m2Q.deq;
		fpSqrt32.enq(m2Q.first);
	endrule
	FIFO#(Bit#(32)) tmpResult2Q <- mkFIFO;
	rule denominatorMod4;
		fpSqrt32.deq;
		tmpResult2Q.enq(fpSqrt32.first);
	endrule

	FIFO#(Bit#(32)) p1Q <- mkFIFO;
	rule denominatorPow1;
		tmpResult2Q.deq;
		let p1 = tmpResult2Q.first;
		fpMult32[3].enq(p1, p1);
		p1Q.enq(p1);
	endrule
	rule denominatorPow2;
		p1Q.deq;
		fpMult32[3].deq;
		let p1 = p1Q.first;
		let p2 = fpMult32[0].first;

		fpMult32[4].enq(p1, p2);
	endrule
	FIFO#(Bit#(32)) tmpResult3Q <- mkFIFO;
	rule denominatorPow3;
		fpMult32[4].deq;
		tmpResult3Q.enq(fpMult32[0].first);
	endrule

	rule devider1;
		massJQ.deq;
		let massJ = massJQ.first;
		Bit#(32) g = 32'b00111100100000000000000000000000;
		fpMult32[5].enq(g, massJ);
	endrule
	rule devider2;
		tmpResult3Q.deq;
		fpMult32[5].deq;
		let p3 = tmpResult3Q.first;
		let gm = fpMult32[2].first;

		fpDiv32.enq(gm, p3);
	endrule
	FIFO#(Bit#(32)) tmpResult4Q <- mkFIFO;
	rule devider3;
		fpDiv32.deq;
		tmpResult4Q.enq(fpDiv32.first);
	endrule
	
	rule calAccelResult1;
		subResultQ.deq;
		tmpResult4Q.deq;
		let s = subResultQ.first;
		let d = tmpResult4Q.first;

		for ( Integer x = 0; x < 3; x = x + 1 ) fpMult32[x+6].enq(d, s[x]);
	endrule
	FIFO#(Vector#(3, Bit#(32))) frQ <- mkFIFO;
	rule calAccelResult2;
		Vector#(3, Bit#(32)) fr = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpMult32[x+6].deq;
			fr[x] = fpMult32[x+6].first;
		end

		frQ.enq(fr);
	endrule

	FIFOF#(Vector#(3, Bit#(32))) frBufferQ <- mkFIFOF;
	Reg#(Bit#(8)) accCnt_1 <- mkReg(0);
	rule calAccumulator1;
		frQ.deq;
		let currFr = frQ.first;
		if ( accCnt_1 != 0 ) begin
			if ( frBufferQ.notEmpty ) begin
				if ( accCnt_1 == 63 ) begin
					accCnt_1 <= 0;
				end else begin
					accCnt_1 <= accCnt_1 + 1;
				end
				
				frBufferQ.deq;
				let prevFr = frBufferQ.first;
				for ( Integer x = 0; x < 3; x = x + 1 ) fpAdd32[x].enq(prevFr[x], currFr[x]);
			end
		end else begin
			accCnt_1 <= accCnt_1 + 1;
			frBufferQ.enq(currFr);
		end
	endrule
	FIFOF#(Vector#(3, Bit#(32))) outputAQ <- mkFIFOF;
	Reg#(Bit#(8)) accCnt_2 <- mkReg(0);
	rule calAccumulator2;
		Vector#(3, Bit#(32)) acc = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpAdd32[x].deq;
			acc[x] = fpAdd32[x].first;
		end

		if ( accCnt_2 == 62 ) begin
			outputAQ.enq(acc);
			accCnt_2 <= 0;
		end else begin
			frBufferQ.enq(acc);
			accCnt_2 <= accCnt_2 + 1;
		end
	endrule

	method Action putOperandI(Vector#(3, Bit#(32)) i);
		inputIQ.enq(i);
	endmethod
	method Action putOperandJ(Vector#(4, Bit#(32)) j);
		inputJQ.enq(j);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) resultGet;
		outputAQ.deq;
		return outputAQ.first;
	endmethod
	method Bool resultExist;
		return outputAQ.notEmpty;
	endmethod
endmodule


interface CalAccelIfc;
	method Action iIn(Vector#(3, Bit#(32)) dataI);
	method Action jIn(Vector#(4, Bit#(32)) dataJ);
	method ActionValue#(Vector#(3, Bit#(32))) aOut;
endinterface
module mkCalAccel(CalAccelIfc);
	Vector#(3, FpPairIfc#(32)) fpAdd32 <- replicateM(mkFpAdd32);

	Vector#(PeWays, CalAccelPeIfc) pes;
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) iInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Vector#(4, Bit#(32)))) jInQs <- replicateM(mkFIFO);
	Vector#(PeWays, FIFO#(Vector#(3, Bit#(32)))) aOutQs <- replicateM(mkFIFO);

	for ( Integer x = 0; x < valueOf(PeWays); x = x + 1 ) begin
		pes[x] <- mkCalAccelPe(fromInteger(x));

		rule forwardOperandI;
			iInQs[x].deq;
			let d = iInQs[x].first;
			if ( x < (valueOf(PeWays) - 1) ) begin
				iInQs[x+1].enq(d);
			end
			pes[x].putOperandI(d);
		endrule
		Reg#(Bit#(24)) jInIdx <- mkReg(0);
		rule forwardOperandJ;
			jInQs[x].deq;
			let d = jInQs[x].first;
			if ( x < (valueOf(PeWays) - 1) ) begin
				jInQs[x+1].enq(d);
			end
			Bit#(PeWaysLog) target_j = truncate(jInIdx);
			if ( target_j == fromInteger(x) ) begin
				pes[x].putOperandJ(d);
			end
		endrule
		rule forwardResultA;
			if ( pes[x].resultExist ) begin
				let d <- pes[x].resultGet;
				aOutQs[x].enq(d);
			end else if ( x < (valueOf(PeWays)-1) ) begin
				aOutQs[x+1].deq;
				aOutQs[x].enq(aOutQs[x+1].first);
			end
		endrule
	end

	FIFOF#(Vector#(3, Bit#(32))) accBufferQ <- mkFIFOF;
	Reg#(Bit#(8)) accCnt_1 <- mkReg(0);
	rule accumulator1;
		aOutQs[0].deq;
		let currD = aOutQs[0].first;

		if ( accCnt_1 != 0 ) begin
			if ( accBufferQ.notEmpty ) begin
				if ( accCnt_1 == 15 ) begin
					accCnt_1 <= 0;
				end else begin
					accCnt_1 <= accCnt_1 + 1;
				end
			
				accBufferQ.deq;
				let prevD = accBufferQ.first;
				for ( Integer x = 0; x < 3; x = x + 1 ) fpAdd32[x].enq(prevD[x], currD[x]);
			end
		end else begin
			accBufferQ.enq(currD);
			accCnt_1 <= accCnt_1 + 1;
		end
	endrule
	FIFO#(Vector#(3, Bit#(32))) aOutQ <- mkFIFO;
	Reg#(Bit#(8)) accCnt_2 <- mkReg(0);
	rule accumulator2;
		Vector#(3, Bit#(32)) acc = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpAdd32[x].deq;
			acc[x] = fpAdd32[x].first;
		end

		if ( accCnt_2 == 14 ) begin
			aOutQ.enq(acc);
			accCnt_2 <= 0;
		end else begin
			accBufferQ.enq(acc);
			accCnt_2 <= accCnt_2 + 1;
		end
	
	endrule

	method Action iIn(Vector#(3, Bit#(32)) dataI);
		iInQs[0].enq(dataI);
	endmethod
	method Action jIn(Vector#(4, Bit#(32)) dataJ);
		jInQs[0].enq(dataJ);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) aOut;
		aOutQ.deq;
		return aOutQ.first;
	endmethod
endmodule
