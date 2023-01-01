LIBPATH =../../../../../../bluespecpcie/
BLIBPATH=../../../../../../bluelib/src
ROOT=../../../../../
BOARD=vc707
BUILDTOOLS=$(LIBPATH)/buildtools/
TCLARGS=../../../../../../../bluespecpcie/
NBODYPATH = nbody

CUSTOMBSV= -p +:$(LIBPATH)/dram/src/:$(ROOT)/src/hwlib/:$(BLIBPATH)/:$(BLIBPATH)/msfp/:$(NBODYPATH) -steps-max-intervals 3000000 +RTS -K1800000000G -RTS
CUSTOMCPP_BSIM=$(BLIBPATH)/bdpi.cpp

include $(BUILDTOOLS)/Makefile.base


