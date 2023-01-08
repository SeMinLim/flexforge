import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;


typedef 1024 Dimension;
typedef 16 ByteWords;
typedef TDiv#(Dimension, ByteWords) Accumulate;


interface SizeCalculatorIfc;
        method Action enq(Bit#(512) v);
        method ActionValue#(Bit#(32)) deq;
endinterface
(* synthesize *)
module mkSizeCalculator(SizeCalculatorIfc);
        FIFO#(Bit#(512)) vInQ <- mkSizedBRAMFIFO(64);
        FIFO#(Bit#(32)) sOutQ <- mkFIFO;

        // Get a's size
        Vector#(16, FpPairIfc#(32)) aSize_fpMult32 <- replicateM(mkFpMult32);
        Vector#(16, FpPairIfc#(32)) aSize_fpAdd32 <- replicateM(mkFpAdd32);
        Reg#(Bit#(16)) aSizeCnt <- mkReg(0);
        rule aSize_1;
                vInQ.deq;
                let a = vInQ.first;

                for ( Integer i = 0; i < 16; i = i + 1) begin
                        let av1 = a[((32*(i+1))-1):(32*i)];
                        aSize_fpMult32[i].enq(av1, av1);
                end
        endrule
        rule aSize_2;
                Vector#(16, Bit#(32)) av2 = replicate(0);
                for ( Integer i = 0; i < 16; i = i + 1 ) begin
                        aSize_fpMult32[i].deq;
                        av2[i] = aSize_fpMult32[i].first;
                end

                for ( Integer j = 0; j < 16; j = j + 2 ) begin
                        aSize_fpAdd32[j/2].enq(av2[j], av2[j+1]);
                end
        endrule
        rule aSize_3;
                Vector#(8, Bit#(32)) av3 = replicate(0);
                for ( Integer i = 0; i < 8; i = i + 1 ) begin
                        aSize_fpAdd32[i].deq;
                        av3[i] = aSize_fpAdd32[i].first;
                end
		
                for ( Integer j = 0; j < 8; j = j + 2 ) begin
                        aSize_fpAdd32[8+(j/2)].enq(av3[j], av3[j+1]);
                end
        endrule
        rule aSize_4;
                Vector#(4, Bit#(32)) av4 = replicate(0);
                for ( Integer i = 0; i < 4; i = i + 1 ) begin
                        aSize_fpAdd32[8+i].deq;
                        av4[i] = aSize_fpAdd32[8+i].first;
                end

                for ( Integer j = 0; j < 4; j = j + 2 ) begin
                        aSize_fpAdd32[12+(j/2)].enq(av4[j], av4[j+1]);
                end
        endrule
        rule aSize_5;
                Vector#(2, Bit#(32)) av5 = replicate(0);
                for ( Integer i = 0; i < 2; i = i + 1 ) begin
                        aSize_fpAdd32[12+i].deq;
                        av5[i] = aSize_fpAdd32[12+i].first;
                end

                aSize_fpAdd32[14].enq(av5[0], av5[1]);
        endrule
        rule aSize_6;
                aSize_fpAdd32[14].deq;
                let av6 = aSize_fpAdd32[14].first;
               	if ( aSizeCnt + 1 == fromInteger(valueOf(Accumulate)) ) begin
			sOutQ.enq(1);
			aSizeCnt <= 0;
		end else begin
			aSizeCnt <= aSizeCnt + 1;
		end
        endrule

	method Action enq(Bit#(512) v);
                vInQ.enq(v);
        endmethod
        method ActionValue#(Bit#(32)) deq;
                sOutQ.deq;
                return sOutQ.first;
        endmethod
endmodule

