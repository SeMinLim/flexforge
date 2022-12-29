LIBPATH =../../../../../../../bluespecpcie/
BLIBPATH=../../../../../../../bluelib/src
ROOT=../../../../../../
BOARD=vc707
BUILDTOOLS=$(LIBPATH)/buildtools/
TCLARGS=../../../../../../../../bluespecpcie/

CUSTOMBSV= -p +:$(LIBPATH)/dram/src/:$(ROOT)/src/hwlib/:$(ROOT)/auroraExt/xilinx:$(BLIBPATH)/ -check-assert
CUSTOMCPP_BSIM= $(ROOT)/auroraExt/xilinx/bdpi.cpp

include $(BUILDTOOLS)/Makefile.base


