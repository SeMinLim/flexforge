import FIFO::*;
import FIFOF::*;
import Vector::*;

import BRAM::*;
import BRAMFIFO::*;

import FloatingPoint::*;
import Float32::*;

import CalAccel::*;
import CalPosit::*;
import CalVeloc::*;

typedef 4 PeWaysLog;
typedef TExp#(PeWaysLog) PeWays;

Integer totalParticles = 1024;


interface NbodyIfc;
	method Action dataPIn_i(Vector#(3, Bit#(32)) originDataP_i);
	method Action dataPIn_j(Vector#(4, Bit#(32)) originDataP_j);
	method Action dataPIn(Vector#(3, Bit#(32)) originDataP);
	method Action dataVIn(Vector#(3, Bit#(32)) originDataV);
	method ActionValue#(Vector#(3, Bit#(32))) dataOutP;
	method ActionValue#(Vector#(3, Bit#(32))) dataOutV;
endinterface
module mkNbody(NbodyIfc);
	Vector#(3, FpPairIfc#(32)) fpAdd32 <- replicateM(mkFpAdd32);

	FIFO#(Vector#(3, Bit#(32))) dataP_iQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Vector#(4, Bit#(32))) dataP_jQ <- mkSizedBRAMFIFO(1024);

	FIFO#(Vector#(3, Bit#(32))) dataPQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Vector#(3, Bit#(32))) dataVQ <- mkSizedBRAMFIFO(1024);

	FIFO#(Vector#(3, Bit#(32))) resultOutPQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Vector#(3, Bit#(32))) resultOutVQ <- mkSizedBRAMFIFO(1024);

	CalAccelIfc calAccel <- mkCalAccel;
	CalPositIfc calPosit <- mkCalPosit;
	CalVelocIfc calVeloc <- mkCalVeloc;

	// Acceleration Input
	rule relayDataP_i;
		dataP_iQ.deq;
		calAccel.iIn(dataP_iQ.first);
	endrule
	rule relayDataP_j;
		dataP_jQ.deq;
		calAccel.jIn(dataP_jQ.first);
	endrule
	// Acceleration Output
 	FIFOF#(Vector#(3, Bit#(32))) accBufferQ <- mkFIFOF;
        Reg#(Bit#(8)) accCnt_1 <- mkReg(0);
        rule calAccelAccumulator1;
                if ( accCnt_1 != 0 ) begin
                        if ( accBufferQ.notEmpty ) begin
                                let currD <- calAccel.aOut;

                                if ( accCnt_1 == (fromInteger(valueOf(PeWays)) - 1) ) begin
                                        accCnt_1 <= 0;
                                end else begin
                                        accCnt_1 <= accCnt_1 + 1;
                                end

                                accBufferQ.deq;
                                let prevD = accBufferQ.first;
                                for ( Integer x = 0; x < 3; x = x + 1 ) fpAdd32[x].enq(prevD[x], currD[x]);
                        end
                end else begin
               		let currD <- calAccel.aOut;

                        accBufferQ.enq(currD);
                        accCnt_1 <= accCnt_1 + 1;
                end
        endrule
	FIFO#(Vector#(3, Bit#(32))) aOutQ <- mkSizedBRAMFIFO(1024);
        Reg#(Bit#(8)) accCnt_2 <- mkReg(0);
        Reg#(Bit#(16)) accCnt_3 <- mkReg(0);
        rule calAccelAccumulator2;
                Vector#(3, Bit#(32)) acc = replicate(0);
                for ( Integer x = 0; x < 3; x = x + 1 ) begin
                        fpAdd32[x].deq;
                        acc[x] = fpAdd32[x].first;
                end

                if ( accCnt_2 == (fromInteger(valueOf(PeWays)) - 2) ) begin
                      	calPosit.aIn(acc); 
			calVeloc.aIn(acc);

                        accCnt_2 <= 0;
                        accCnt_3 <= accCnt_3 + 1;
                        $write("acc final result out[%d]\n", accCnt_3);
                end else begin
                        accBufferQ.enq(acc);
                        accCnt_2 <= accCnt_2 + 1;
                end

        endrule
	// Position & Velocity Input
	rule relayDataV;
		dataVQ.deq;
		let v = dataVQ.first;
		calPosit.vIn(v);
		calVeloc.vIn(v);
	endrule
	rule relayDataP;
		dataPQ.deq;
		let p = dataPQ.first;
		calPosit.pIn(p);
	endrule

	// Position & Velocity Output
	rule recvResultP;
		let resP <- calPosit.pOut;
		resultOutPQ.enq(resP);
	endrule
	rule recvResultV;
		let resV <- calVeloc.vOut;
		resultOutVQ.enq(resV);
	endrule

	method Action dataPIn_i(Vector#(3, Bit#(32)) originDataP_i);
		dataP_iQ.enq(originDataP_i);
	endmethod
	method Action dataPIn_j(Vector#(4, Bit#(32)) originDataP_j);
		dataP_jQ.enq(originDataP_j);
	endmethod
	method Action dataPIn(Vector#(3, Bit#(32)) originDataP);
		dataPQ.enq(originDataP);
	endmethod
	method Action dataVIn(Vector#(3, Bit#(32)) originDataV);
		dataVQ.enq(originDataV);
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) dataOutP;
		resultOutPQ.deq;
		return resultOutPQ.first;
	endmethod
	method ActionValue#(Vector#(3, Bit#(32))) dataOutV;
		resultOutVQ.deq;
		return resultOutVQ.first;
	endmethod
endmodule

