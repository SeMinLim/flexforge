LIBPATH =../../../../../../../bluespecpcie/
BLIBPATH=../../../../../../../bluelib/src
ROOT=../../../../../../
BOARD=vc707
BUILDTOOLS=$(LIBPATH)/buildtools/
TCLARGS=../../../../../../../../bluespecpcie/
KMEANPATH = kmean
AURORAPATH = $(ROOT)/auroraExt/xilinx

CUSTOMBSV= -p +:$(LIBPATH)/dram/src/:$(ROOT)/src/hwlib/:$(AURORAPATH)/:$(BLIBPATH)/:$(BLIBPATH)/msfp/:$(KMEANPATH)/ -check-assert -steps-max-intervals 3000000 +RTS -K1800000000G -RTS
CUSTOMCPP_BSIM=$(BLIBPATH)/bdpi.cpp

include $(BUILDTOOLS)/Makefile.base
