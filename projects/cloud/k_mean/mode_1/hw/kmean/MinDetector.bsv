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
	method Action vIn(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog)) v);
	method ActionValue#(Tuple2#(Vector#(3, Bit#(32)), Bit#(PeWaysLog))) minOut;
endinterface
(* synthesize *)
module mkMinDetector(MinDetectorIfc);
	FIFO#(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog)) vInQ <- mkFIFO;
	FIFO#(Tuple2#(Vector#(3, Bit#(32)), Bit#(peWaysLog))) minOutQ <- mkFIFO;
	
	Reg#(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog))) minOutBuffer <- mkReg(0);
	Reg#(Bit#(8)) detectorCnt <- mkReg(0);
	rule getMinValue;
		vInQ.deq;
		let value = vInQ.first;
		let v = tpl_2(value);

		if ( detectorCnt != 0 ) begin
			let minim = minOutBuffer;
			let m = tpl_2(minim);	

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
					minOutQ.enq(tuple2(tpl_1(value), tpl_3(value)));
				end else begin
					minOutQ.enq(tuple2(tpl_1(minim), tpl_3(minim)));
				end
				detectorCnt <= 0;
			end else begin
				if ( fv < fm ) begin
					minOutBuffer <= value;
				end
				detectorCnt <= detectorCnt + 1;
			end
		end else begin
			minOutBuffer <= value;
			detectorCnt <= detectorCnt + 1;
		end
	endrule
	method Action vIn(Tuple3#(Vector#(3, Bit#(32)), Bit#(32), Bit#(PeWaysLog)) v);
		vInQ.enq(v);
	endmethod
	method ActionValue#(Tuple2#(Vector#(3, Bit#(32)), Bit#(PeWaysLog))) minOut;
		minOutQ.deq;
		return minOutQ.first;
	endmethod
endmodule
