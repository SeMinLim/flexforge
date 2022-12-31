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

#define NumParticles 1024

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
	// Generate the values of the particles
	//------------------------------------------------------------------------------------
	printf( "Started generating the values of the particles\n" );
	fflush( stdout );
	// Location X
	int x = 0, y = 0;
	float tmp = 0;
	float* particleLocX = (float*)malloc(sizeof(float)*NumParticles);
	float locX = 0.00000;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleLocX[i] = locX;
		locX = locX + 0.00001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleLocX[x];
			particleLocX[x] = particleLocX[y];
			particleLocX[y] = tmp;
		}
	}
	uint32_t* particleLocXv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleLocXv[k] = *(uint32_t*)&particleLocX[k];
	}
	// Location Y
	x = 0, y = 0;
	tmp = 0;
	float* particleLocY = (float*)malloc(sizeof(float)*NumParticles);
	float locY = 0.00000;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleLocY[i] = locY;
		locY = locY + 0.00001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleLocY[x];
			particleLocY[x] = particleLocY[y];
			particleLocY[y] = tmp;
		}
	}
	uint32_t* particleLocYv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleLocYv[k] = *(uint32_t*)&particleLocY[k];
	}
	// Location Z
	x = 0, y = 0;
	tmp = 0;
	float* particleLocZ = (float*)malloc(sizeof(float)*NumParticles);
	float locZ = 0.00000;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleLocZ[i] = locZ;
		locY = locY + 0.00001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleLocZ[x];
			particleLocZ[x] = particleLocZ[y];
			particleLocZ[y] = tmp;
		}
	}
	uint32_t* particleLocZv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleLocZv[k] = *(uint32_t*)&particleLocZ[k];
	}
	// Mass
	x = 0, y = 0;
	tmp = 0;
	float* particleMass = (float*)malloc(sizeof(float)*NumParticles);
	float mass = 0.0000000;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleMass[i] = mass;
		mass = mass + 0.0000001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleMass[x];
			particleMass[x] = particleMass[y];
			particleMass[y] = tmp;
		}
	}
	uint32_t* particleMassv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleMassv[k] = *(uint32_t*)&particleMass[k];
	}
	// Velocity X
	x = 0, y = 0;
	tmp = 0;
	float* particleVelX = (float*)malloc(sizeof(float)*NumParticles);
	float velX = 0.8388607;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleVelX[i] = velX;
		velX = velX + 0.0000001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleVelX[x];
			particleVelX[x] = particleVelX[y];
			particleVelX[y] = tmp;
		}
	}
	uint32_t* particleVelXv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleVelXv[k] = *(uint32_t*)&particleVelX[k];
	}
	// Velocity Y
	x = 0, y = 0;
	tmp = 0;
	float* particleVelY = (float*)malloc(sizeof(float)*NumParticles);
	float velY = 0.8388607;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleVelY[i] = velY;
		velY = velY + 0.0000001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleVelY[x];
			particleVelY[x] = particleVelY[y];
			particleVelY[y] = tmp;
		}
	}
	uint32_t* particleVelYv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleVelYv[k] = *(uint32_t*)&particleVelY[k];
	}
	// Velocity Z
	x = 0, y = 0;
	tmp = 0;
	float* particleVelZ = (float*)malloc(sizeof(float)*NumParticles);
	float velZ = 0.8388607;
	for ( int i = 0; i < NumParticles; i ++ ) {
		particleVelZ[i] = velZ;
		velZ = velZ + 0.0000001;
	}
	for ( int j = 0; j < MAX_MIXING_COUNT; j ++ ) {
		x = random() % NumParticles;
		y = random() % NumParticles;
		if ( x != y ) {
			tmp = particleVelZ[x];
			particleVelZ[x] = particleVelZ[y];
			particleVelZ[y] = tmp;
		}
	}
	uint32_t* particleVelZv = (uint32_t*)malloc(sizeof(uint32_t)*NumParticles);
	for ( int k = 0; k < NumParticles; k ++ ) {
		particleVelZv[k] = *(uint32_t*)&particleVelZ[k];
	}
	printf( "Generating the values of the particles done!\n\n" );
	fflush( stdout );
	//------------------------------------------------------------------------------------
	// Send the values of the particles through PCIe first & Check all data are stored well
	//------------------------------------------------------------------------------------
	int systemOn = 0;
	pcie->userWriteWord(systemOn*4, 0);
	printf( "N-body Calculation System On!\n" );
	fflush( stdout );

	int stage_1 = 1;
	int div = 1024;
	printf( "Started to send the values of the particles\n" );
	fflush( stdout );
	for ( int i = 0; i < NumParticles/div; i ++ ) {
		for ( int j = 0; j < div; j ++ ) {
			pcie->userWriteWord(stage_1*4, particleLocXv[(i*div)+j]);
			pcie->userWriteWord(stage_1*4, particleLocYv[(i*div)+j]);
			pcie->userWriteWord(stage_1*4, particleLocZv[(i*div)+j]);
			pcie->userWriteWord(stage_1*4, particleMassv[(i*div)+j]);
		}
		//printf( "Sent %dth position and mass values\n", ((i*div) + div) );
		//fflush( stdout );
		//sleep(1);
	}

	for ( int i = 0; i < NumParticles/div; i ++ ) {
		for ( int j = 0; j < div; j ++ ) {
			pcie->userWriteWord(stage_1*4, particleVelXv[(i*div)+j]);
			pcie->userWriteWord(stage_1*4, particleVelYv[(i*div)+j]);
			pcie->userWriteWord(stage_1*4, particleVelZv[(i*div)+j]);
		}
		//printf( "Sent %dth velocity values\n", ((i*div) + div) );
		//fflush( stdout );
		//sleep(1);
	}
	printf( "Sending the values of the particles done!\n" );
	fflush( stdout );

	int statusCheck = 0;
	unsigned int status = 0;
	/*while ( 1 ) {
		status = pcie->userReadWord(statusCheck_1*4);
		if ( status == 1 ) {
			printf( "Storing the values of the particles to DRAM done!\n\n" );
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

	printf( "No need to send the data from FPGA1 to FPGA2\n" );
	printf( "Started to compute N-body App\n\n" );
	fflush( stdout );	

	//clock_gettime(CLOCK_REALTIME, & start);
	//-------------------------------------------------------------------------------	
	// Status check over running N-body
	//-------------------------------------------------------------------------------
	sleep(60);
	statusCheck = 0;
	status = 0;
	for ( int i = 0; i < 3; i ++ ) {
		status = pcie->userReadWord(statusCheck*4);
		if ( status == 1 ) {
			printf( "Computing N-body app & writing the 1024 updated data to memory done!\n" );
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
