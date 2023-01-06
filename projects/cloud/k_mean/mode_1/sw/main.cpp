#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <ctime>
#include <chrono>

#include "bdbmpcie.h"
#include "dmasplitter.h"

#define Dimension 1024
#define NumData 64

double timespec_diff_sec( timespec start, timespec end ) {
	double t = end.tv_sec - start.tv_sec;
	t += ((double)(end.tv_nsec - start.tv_nsec)/1000000000L);
	return t;
}

int main(int argc, char** argv) {
	srand(time(NULL));
	//------------------------------------------------------------------------------------
	// Initial
	//------------------------------------------------------------------------------------
	printf( "Software startec\n" ); fflush(stdout);
	BdbmPcie* pcie = BdbmPcie::getInstance();
	unsigned int d = pcie->readWord(0);
	printf( "Magic: %x\n", d );
	fflush( stdout );
	if ( d != 0xc001d00d ) {
		printf( "Magic number is incorrect (0xc001d00d)\n" );
		return -1;
	}
	printf( "\n" );
	fflush( stdout );	
	//------------------------------------------------------------------------------------
	// Generate the values of the position data
	//------------------------------------------------------------------------------------
	printf( "Started generating the 1024-dimensional position data\n" );
	fflush( stdout );
		
	float* fData = (float*)malloc(sizeof(float)*NumData*Dimension);
	for ( int i = 0; i < NumData*Dimension; i ++ ) {
		fData[i] = 0;
		if ( rand()%4 == 0 ) {
			fData[i] = ((float)(rand()%10000))/1000;
		}
	}	

	uint32_t* data = (uint32_t*)malloc(sizeof(uint32_t)*NumData*Dimension);
	for ( int i = 0; i < NumData*Dimension; i ++ ) {
		data[i] = *(uint32_t*)&fData[i];
	}
	//------------------------------------------------------------------------------------
	// Send the data through PCIe first & Check all data are stored well
	//------------------------------------------------------------------------------------
	int systemOn = 0;
	pcie->userWriteWord(systemOn*4, 0);
	printf( "K-mean Clustering System On!\n" );
	fflush( stdout );

	int stage_1 = 1;
	printf( "Started to send 1024-dimensional position data!\n" );
	fflush( stdout );
	for ( int i = 0; i < NumData; i ++ ) {
		for ( int j = 0; j < Dimension; j ++ ) {
			pcie->userWriteWord(stage_1*4, data[(i*Dimension)+j]);
		}
		printf( "Sent %d node done!\n", (i+1)*Dimension );
		sleep(1);
	}	
	printf( "Sending the data done!\n" );
	fflush( stdout );

	int statusCheck = 0;
	unsigned int status = 0;
	/*while ( 1 ) {
		status = pcie->userReadWord(statusCheck_1*4);
		if ( status == 1 ) {
			printf( "Storing the data to DRAM done!\n\n" );
			fflush( stdout );
			break;
		}
	}*/
        //------------------------------------------------------------------------------	
	// Send a command to HW to start running N-body
	//------------------------------------------------------------------------------
	timespec start;
	timespec now;
	int stage_2 = 2;
	printf( "The system mode\n" );
	printf( "1: Use only FPGA1\n" );
	printf( "2: Use both FPGA1 and FPGA2 with 1 Aurora lane (2hops)\n" );
	printf( "3: Use both FPGA1 and FPGA2 with 2 Aurora lanes (2hops)\n" );
	printf( "4: Use both FPGA1 and FPGA2 with 3 Aurora lanes (2hops)\n" );
	printf( "5: Use both FPGA1 and FPGA2 with 4 Aurora lanes (2hops)\n" );
	printf( "6: Use both FPGA1 and FPGA2 with 1 Aurora lane (4hops)\n" );
	printf( "Mode: 1\n\n" );
	fflush( stdout );
	pcie->userWriteWord(stage_2*4, 0);

	printf( "Started to K-mean clustering App\n\n" );
	fflush( stdout );	

	//clock_gettime(CLOCK_REALTIME, & start);
	//-------------------------------------------------------------------------------	
	// Status check over running N-body
	//-------------------------------------------------------------------------------
	sleep(60);
	statusCheck = 0;
	status = 0;
	for ( int i = 0; i < 10; i ++ ) {
		status = pcie->userReadWord(statusCheck*4);
		if ( status == 1 ) {
			printf( "Computing K-mean clustering app & writing the 1024 updated data to memory done!\n" );
			fflush( stdout );
			break;
		}
		sleep(60);
	}
	int cycleCheck = 1;
	printf( "Cycle: %d\n", pcie->userReadWord(cycleCheck*4) );
	
	//clock_gettime(CLOCK_REALTIME, & now);
	//printf( "\n" );
	//fflush( stdout );
	//-------------------------------------------------------------------------------	
	// Status check for finishing N-body App
	//-------------------------------------------------------------------------------
	//double diff = timespec_diff_sec(start, now);
	//printf( "Elapsed Time: %f\n", diff );
	//fflush( stdout );

	return 0;
}

