import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;


interface MinDetectorIfc;
	method Action vIn(Bit#(32) v);
	method ActionValue#(Bit#(32)) minOut;
endinterface
(* synthesize *)
module mkMinDetector(MinDetectorIfc);
	FIFO#(Bit#(32)) vInQ <- mkFIFO;
	FIFO#(Bit#(32)) minOutQ <- mkFIFO;

	Reg#(Bit#(8)) detectorCnt <- mkReg(0);
	rule getMinValue;
		vInQ.deq;
		let v = vInQ.first;

		if ( detectorCnt != 0 ) begin
			let m = minBuffer;

			Bool msign = m[31] == 1;
			Bool vsign = v[31] == 1;
			Bit#(8) me = truncate(m>>23);
			Bit#(8) ve = truncate(v>>23);
			Bit#(23) ms = truncate(m);
			Bit#(23) vs = truncate(v);
			Float fm = Float{sign: msign, exp: me, sfd: ms};
			Float fv = Float{sign: vsign, exp: ve, sfd: vs};

			if ( detectorCnt + 1 == fromInteger(valueOf(PeWays)) ) begin
				if ( fv < fm ) begin
					minOutQ.enq(v);
				end else begin
					minOutQ.enq(m);
				end
				detectorCnt <= 0;
			end else begin
				if ( fv < fm ) begin
					minBuffer <= v;
				end
				detectorCnt <= detectorCnt + 1;
			end
		end else begin
			minBuffer <= v;
			detectorCnt <= detectorCnt + 1;
		end
	endrule
	method Action vIn(Bit#(32) v);
		vInQ.enq(v);
	endmethod
	method ActionValue#(Bit#(32)) minOut;
		minOutQ.deq;
		return minOutQ.first;
	endmethod
endmodule
