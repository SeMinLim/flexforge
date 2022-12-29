LIBPATH =../../../../../../../bluespecpcie/
BLIBPATH=../../../../../../../bluelib/src
ROOT=../../../../../../
BOARD=vc707
BUILDTOOLS=$(LIBPATH)/buildtools/
TCLARGS=../../../../../../../../bluespecpcie/

CUSTOMBSV= -p +:$(LIBPATH)/dram/src/:$(ROOT)/src/hwlib/:$(BLIBPATH)/:$(BLIBPATH)/msfp/ -steps-max-intervals 3000000 +RTS -K80000k -RTS
CUSTOMCPP_BSIM= $(BLIBPATH)/bdpi.cpp

include $(BUILDTOOLS)/Makefile.base


