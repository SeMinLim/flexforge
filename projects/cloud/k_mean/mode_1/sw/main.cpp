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

#define PEs 16
#define Div 1024
#define NumData 1024
#define MAX_MIXING_COUNT 1000

double timespec_diff_sec( timespec start, timespec end ) {
	double t = end.tv_sec - start.tv_sec;
	t += ((double)(end.tv_nsec - start.tv_nsec)/1000000000L);
	return t;
}

int main(int argc, char** argv) {
	//srand(time(NULL)); // Do not need to refresh
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
	printf( "Started generating the 2-dimensional position data\n" );
	fflush( stdout );
	// Position X	
	int x = 0, y = 0;
	float tmp = 0.00;
	float* dataPosX = (float*)malloc(sizeof(float)*NumData);
	float posX = 0;
	for ( int i = 0; i < NumData; i ++ ) {
		dataPosX[i] = posX;
		posX = posX + 1.00;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumData;
		y = random() % NumData;
		if ( x != y ) {
			tmp = dataPosX[x];
			dataPosX[x] = dataPosX[y];
			dataPosX[y] = tmp;
		}
	}	
	uint32_t* dataPosXv = (uint32_t*)malloc(sizeof(uint32_t)*NumData);
	for ( int k = 0; k < NumData; k ++ ) {
		dataPosXv[k] = *(uint32_t*)&dataPosX[k];
	}
	// Position Y
	x = 0, y = 0;
	tmp = 0;
	float* dataPosY = (float*)malloc(sizeof(float)*NumData);
	float posY = 0.00;
	for ( int i = 0; i < NumData; i ++ ) {
		dataPosY[i] = posY;
		posY = posY + 1.00;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumData;
		y = random() % NumData;
		if ( x != y ) {
			tmp = dataPosY[x];
			dataPosY[x] = dataPosY[y];
			dataPosY[y] = tmp;
		}
	}
	uint32_t* dataPosYv = (uint32_t*)malloc(sizeof(uint32_t)*NumData);
	for ( int k = 0; k < NumData; k ++ ) {
		dataPosYv[k] = *(uint32_t*)&dataPosY[k];
	}
	// Cluster Head Index	
	float* dataIdx = (float*)malloc(sizeof(float)*NumData);
	float idx = 0.00;
	for ( int i = 0; i < NumData; i ++ ) {
		dataIdx[i] = idx;
	}
	uint32_t* dataIdxv = (uint32_t*)malloc(sizeof(uint32_t)*NumData);
	for ( int k = 0; k < NumData; k ++ ) {
		dataIdxv[k] = *(uint32_t*)&dataIdx[k];
	}
	//------------------------------------------------------------------------------------
	// Send the data through PCIe first & Check all data are stored well
	//------------------------------------------------------------------------------------
	int systemOn = 0;
	pcie->userWriteWord(systemOn*4, 0);
	printf( "K-mean Clustering System On!\n" );
	fflush( stdout );

	int stage_1 = 1;
	printf( "Started to send 2-dimensional position data and initial cluster head index!\n" );
	fflush( stdout );
	for ( int i = 0; i < NumData/PEs; i ++ ) {
		for ( int j = 0; j < PEs; j ++ ) {
			pcie->userWriteWord(stage_1*4, dataPosXv[(i*PEs) + j]); // Pos X
			pcie->userWriteWord(stage_1*4, dataPosYv[(i*PEs) + j]); // Pos Y
			pcie->userWriteWord(stage_1*4, dataIdxv[(i*PEs) + j]); // Cluster head idx
		}
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

