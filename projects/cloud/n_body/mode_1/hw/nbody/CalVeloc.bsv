import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;


interface CalVelocIfc;
	method Action aIn(Vector#(3, Bit#(32)) a);
	method Action vIn(Vector#(3, Bit#(32)) v);
	method ActionValue#(Vector#(3, Bit#(32))) vOut;
endinterface
module mkCalVeloc(CalVelocIfc);
	Vector#(3, FpPairIfc#(32)) fpAdd32 <- replicateM(mkFpAdd32);

	FIFO#(Vector#(3, Bit#(32))) aInQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) vInQ <- mkFIFO;
	FIFOF#(Vector#(3, Bit#(32))) vOutQ <- mkFIFOF;

	rule calVel1;
		aInQ.deq;
		vInQ.deq;
		let a = aInQ.first;
		let v = vInQ.first;

		for ( Integer x = 0; x < 3; x = x + 1 ) fpAdd32[x].enq(v[x], a[x]);
	endrule
	rule calVel2;
		Vector#(3, Bit#(32)) fr = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpAdd32[x].deq;
			fr[x] = fpAdd32[x].first;
		end

		vOutQ.enq(fr);
	endrule

	method Action aIn(Vector#(3, Bit#(32)) a);
		aInQ.enq(a);
	endmethod
	method Action vIn(Vector#(3, Bit#(32)) v);
		vInQ.enq(v);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) vOut;
		vOutQ.deq;
		return vOutQ.first;
	endmethod
endmodule
