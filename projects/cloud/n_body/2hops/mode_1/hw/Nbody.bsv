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

Integer totalParticles = 16*1024*1024;


interface NbodyIfc;
	method Action dataPIn_i(Vector#(3, Bit#(32)) originDataP_i);
	method Action dataPmIn_j(Vector#(4, Bit#(32)) originDataPm_j);
	method Action dataPIn(Vector#(3, Bit#(32)) originDataP);
	method Action dataVIn(Vector#(3, Bit#(32)) originDataV);
	method ActionValue#(Vector#(3, Bit#(32))) dataOutP;
	method ActionValue#(Vector#(3, Bit#(32))) dataOutV;
endinterface
module mkNbody(NbodyIfc);
	Vector#(3, FpPairIfc#(32)) fpAdd32 <- replicateM(mkFpAdd32);

	FIFO#(Vector#(3, Bit#(32))) accQ <- mkSizedBRAMFIFO(1024);

	FIFO#(Vector#(3, Bit#(32))) dataP_iQ <- mkFIFO;
	FIFO#(Vector#(4, Bit#(32))) dataPm_jQ <- mkFIFO;

	FIFO#(Vector#(3, Bit#(32))) dataPQ <- mkFIFO;
	FIFO#(Vector#(3, Bit#(32))) dataVQ <- mkFIFO;

	FIFO#(Vector#(3, Bit#(32))) resultOutPQ <- mkSizedBRAMFIFO(1024);
	FIFO#(Vector#(3, Bit#(32))) resultOutVQ <- mkSizedBRAMFIFO(1024);

	CalAccelIfc calAccel <- mkCalAccel;
	CalPositIfc calPosit <- mkCalPosit;
	CalVelocIfc calVeloc <- mkCalVeloc;

	// Acceleration Input
	rule relayDataPmI;
		dataP_iQ.deq;
		calAccel.iIn(dataP_iQ.first);
	endrule
	rule relayDataPmJ;
		dataPm_jQ.deq;
		calAccel.jIn(dataPm_jQ.first);
	endrule
	// Acceleration Output
	Reg#(Bit#(16)) accCnt_1 <- mkReg(0);
	Reg#(Bit#(16)) accCnt_2 <- mkReg(0);
	Reg#(Bool) firstPhase <- mkReg(True);
	rule accumulator1;
		if ( !firstPhase ) begin
			accQ.deq;
			let prevA = accQ.first;
			let currA <- calAccel.aOut;

			for ( Integer x = 0; x < 3; x = x + 1 ) fpAdd32[x].enq(prevA[x], currA[x]);

			if ( accCnt_1 == 1023 ) begin
				if ( accCnt_2 == 16383 ) begin
					accCnt_2 <= 0;
					firstPhase <= True;
				end else begin
					accCnt_2 <= accCnt_2 + 1;
				end
				accCnt_1 <= 0;
			end else begin
				accCnt_1 <= accCnt_1 + 1;
			end
		end else begin
			let a <- calAccel.aOut;
			accQ.enq(a);

			if ( accCnt_1 == 1023 ) begin
				accCnt_1 <= 0;
				accCnt_2 <= accCnt_2 + 1;
				firstPhase <= False;	
			end else begin
				accCnt_1 <= accCnt_1 + 1;
			end
		end
	endrule
	Reg#(Bit#(16)) accCnt_3 <- mkReg(0);
	Reg#(Bit#(16)) accCnt_4 <- mkReg(0);
	rule accumulator2;
		Vector#(3, Bit#(32)) a = replicate(0);
		for ( Integer x = 0; x < 3; x = x + 1 ) begin
			fpAdd32[x].deq;
			a[x] = fpAdd32[x].first;
		end

		if ( accCnt_3 == 16382 ) begin
			if ( accCnt_4 == 1022 ) begin
				accCnt_4 <= 0;
				accCnt_3 <= 0;
			end else begin
				accCnt_4 <= accCnt_4 + 1;
			end
			calPosit.aIn(a); 
			calVeloc.aIn(a);
		end else begin
			if ( accCnt_4 == 1022 ) begin
				accCnt_4 <= 0;
				accCnt_3 <= accCnt_3 + 1;
			end else begin
				accCnt_4 <= accCnt_4 + 1;
			end
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
		let res <- calPosit.pOut;
		resultOutPQ.enq(res);
	endrule
	rule recvResultV;
		let res <- calVeloc.vOut;
		resultOutVQ.enq(res);
	endrule

	method Action dataPIn_i(Vector#(3, Bit#(32)) originDataP_i);
		dataP_iQ.enq(originDataP_i);
	endmethod
	method Action dataPmIn_j(Vector#(4, Bit#(32)) originDataPm_j);
		dataPm_jQ.enq(originDataPm_j);
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

