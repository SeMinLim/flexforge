import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

import CosineSimilarity::*;

typedef 32 NumData;
typedef 1024 Dimension;
typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;


interface PeIfc;
        method Action putData(Bit#(512) d);
        method ActionValue#(Bit#(32)) getResult;
endinterface
(* synthesize *)
module mkPe#(Bit#(PeWaysLog) peIdx) (PeIfc);
        CosineSimilarityIfc cossim <- mkCosineSimilarity;

        FIFO#(Bit#(512)) putDataQ <- mkFIFO;
        FIFO#(Bit#(32)) getResultQ <- mkFIFO;

        rule relayValues;
                putDataQ.deq;
                let a = putDataQ.first;
                cossim.aIn(a);
        endrule
	
        rule getValue;
                let cs <- cossim.csOut;
		getResultQ.enq(cs);
        endrule

        method Action putData(Bit#(512) d);
                putDataQ.enq(d);
        endmethod
        method ActionValue#(Bit#(32)) getResult;
                getResultQ.deq;
                return getResultQ.first;
        endmethod
endmodule


interface KmeanIfc;
	method Action dataIn(Bit#(512) d);
	method ActionValue#(Bit#(32)) dataOut;
endinterface
(* synthesize *)
module mkKmean(KmeanIfc);
        Vector#(PeWays, PeIfc) pes;
        Vector#(PeWays, FIFO#(Bit#(512))) dataInQs <- replicateM(mkFIFO);
        Vector#(PeWays, FIFO#(Bit#(32))) dataOutQs <- replicateM(mkFIFO);
        FIFO#(Bit#(32)) dataOutQ <- mkFIFO;

        for ( Integer i = 0; i < valueOf(PeWays); i = i + 1 ) begin
                pes[i] <- mkPe(fromInteger(i));

                rule forwardData;
                        dataInQs[i].deq;
                        let d = dataInQs[i].first;
                        if ( i < valueOf(PeWays) - 1 ) begin
                                dataInQs[i+1].enq(d);
                        end
                        pes[i].putData(d);
                endrule

                rule forwardResult;
                        let r <- pes[i].getResult;
                        dataOutQ.enq(r);
                endrule
        end

	method Action dataIn(Bit#(512) d);
		dataInQs[0].enq(d);
	endmethod
	method ActionValue#(Bit#(32)) dataOut;
		dataOutQ.deq;
		return dataOutQ.first;
	endmethod
endmodule

