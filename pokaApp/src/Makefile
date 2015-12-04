TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# For Kate Feng's implementation of devScaler.c, which uses Till
# Straumann's basicIoOps.h to bridge any mismatch between endianness
# of Joerger scaler hardware and endianness of processor.
#USR_CFLAGS += -DHAS_IOOPS_H

#=============================
# build an ioc application

PROD_IOC = poka

# Note that the order in which .dbd files and libraries are added to a list
# variable is important.  For .dbd files, things must be defined before they
# are referenced by a later .dbd file.  For libraries, things must be referenced
# before they are searched for; otherwise they won't be included.

# Custom stuff in development

##################################################################
# First, define poka_Common_DBD and poka_Common_LIBS from the modules
# defined in RELEASE
##################################################################
DBD += poka_Common.dbd
poka_Common_DBD += base.dbd

#ifdef CAPUTLOG
#	poka_Common_DBD += caPutLog.dbd
#	poka_Common_LIBS := caPutLog $(poka_Common_LIBS)
#endif

ifdef CAPUTRECORDER
	poka_Common_DBD += caputRecorder.dbd
	poka_Common_LIBS := caputRecorder $(poka_Common_LIBS)
endif

ifdef SNCSEQ
	ifeq (seq-2-1, $(findstring seq-2-1, $(SNCSEQ)))
		poka_Common_DBD += devSequencer.dbd
		poka_Common_LIBS := seqDev seq pv $(poka_Common_LIBS)
	else ifeq (seq-2.1, $(findstring seq-2.1, $(SNCSEQ)))
		poka_Common_DBD += devSequencer.dbd
		poka_Common_LIBS := seqDev seq pv $(poka_Common_LIBS)
	else
		# seq 2.2.
		poka_Common_LIBS := seq pv $(poka_Common_LIBS)
	endif
endif

ifdef ASYN
	poka_Common_DBD += asyn.dbd drvAsynSerialPort.dbd
	poka_Common_DBD += drvAsynIPPort.dbd drvVxi11.dbd devGpib.dbd
	poka_Common_LIBS := asyn $(poka_Common_LIBS)
endif

ifdef STD
	poka_Common_DBD += stdSupport.dbd
	poka_Common_LIBS := std $(poka_Common_LIBS)
endif

ifdef CALC
	poka_Common_DBD += calcSupport.dbd
	poka_Common_LIBS := calc $(poka_Common_LIBS)
endif

ifdef BUSY
	poka_Common_DBD += busySupport.dbd
	poka_Common_LIBS := busy $(poka_Common_LIBS)
endif

ifdef SSCAN
	poka_Common_DBD += sscanSupport.dbd
	poka_Common_LIBS := sscan $(poka_Common_LIBS)
	ifdef SNCSEQ
		poka_Common_DBD += sscanProgressSupport.dbd
		poka_Common_LIBS := scanProgress $(poka_Common_LIBS)
	endif
endif

ifdef AUTOSAVE
	poka_Common_DBD += asSupport.dbd
	poka_Common_LIBS := autosave $(poka_Common_LIBS)
endif

ifdef OPTICS
	poka_Common_DBD += opticsSupport.dbd
	poka_Common_LIBS := optics $(poka_Common_LIBS)
endif

ifdef MCA
	poka_Common_DBD += mcaSupport.dbd
	poka_Common_LIBS := mca $(poka_Common_LIBS)
endif

ifdef IP
	poka_Common_DBD += ipSupport.dbd
	poka_Common_LIBS := ip $(poka_Common_LIBS)
endif

ifdef MODBUS
	poka_Common_DBD += modbusSupport.dbd
	poka_Common_LIBS := modbus $(poka_Common_LIBS)
endif

ifdef MOTOR
	poka_Common_DBD += motorSupport.dbd devAcsMotor.dbd devAerotech.dbd
	poka_Common_DBD += devAttocube.dbd devFaulhaberMotor.dbd devImsMotor.dbd
	poka_Common_DBD += devKohzuMotor.dbd devMclennanMotor.dbd devMicos.dbd
	poka_Common_DBD += devMicroMo.dbd devNewFocus.dbd devNewport.dbd
  ifdef SNCSEQ
	poka_Common_DBD += devAerotechSeq.dbd devNewportSeq.dbd
  endif
	poka_Common_DBD += devOriel.dbd devPC6K.dbd devPIJena.dbd devPIMotor.dbd
	poka_Common_DBD += devSPiiPlus.dbd devSmartMotorMotor.dbd devSoftMotor.dbd
	poka_Common_DBD += devThorLabs.dbd motorSimSupport.dbd

	poka_Common_LIBS := motor $(poka_Common_LIBS)
	poka_Common_LIBS := Acs acsTech80 Aerotech Attocube Faulhaber Ims KohzuMotor $(poka_Common_LIBS)
	poka_Common_LIBS := Mclennan Micos MicroMo NewFocus Newport Oriel PI PIJena $(poka_Common_LIBS)
	poka_Common_LIBS := Parker SmartMotor ThorLabs softMotor motorSimSupport $(poka_Common_LIBS)
endif

ifdef STREAM
	poka_Common_DBD += stream.dbd streamSynApps.dbd
	poka_Common_LIBS := stream streamSynApps $(poka_Common_LIBS)
endif

ifdef DEVIOCSTATS
	poka_Common_DBD += devIocStats.dbd iocAdmin.dbd
	poka_Common_LIBS := devIocStats $(poka_Common_LIBS)
endif

ifdef ETHERIP
	poka_Common_DBD += ether_ip.dbd
	poka_Common_LIBS := ether_ip $(poka_Common_LIBS)
endif

ifdef ALIVE
	poka_Common_DBD += aliveSupport.dbd
	poka_Common_LIBS := alive $(poka_Common_LIBS)
endif

ifdef DELAYGEN
	poka_Common_DBD += delaygenSupport.dbd
	poka_Common_LIBS := delaygen $(poka_Common_LIBS)
endif

ifdef VAC
	poka_Common_DBD += vacSupport.dbd
	poka_Common_LIBS := vac $(poka_Common_LIBS)
endif

poka_Common_LIBS := $(poka_Common_LIBS) $(EPICS_BASE_IOC_LIBS)

##################################################################


ifeq (win32-x86,$(findstring win32-x86, $(T_A)))

	DBD  += iocpokaWin32.dbd
	iocpokaWin32_DBD += poka_Common.dbd

	ifdef MCA
		# If your host supports libpcap and libnet and you want to build
		# Canberra network support uncomment the following lines
		#iocpokaWin32_DBD += mcaCanberraSupport.dbd
		#poka_LIBS_WIN32 += mcaCanberra
		#poka_LIBS_WIN32 += wpcap Packet
	endif

	ifdef DXP
		# If you want to build parallel port or USB interfaces to Saturn and/or
		# xMAP DXP, then uncomment the following lines
		iocpokaWin32_DBD += dxpSupport.dbd
		poka_LIBS_WIN32 += dxp handel
		poka_LIBS_WIN32 += DLPORTIO PlxApi
		poka_SYS_LIBS_WIN32 += setupapi
	endif

	ifdef AREA_DETECTOR
		# areaDetector provides a Makefile for things it needs and supplies
		PROD_NAME = AD
		-include $(ADCORE)/ADApp/commonDriverMakefile
		iocpokaWin32_DBD += ADSupport.dbd NDPluginSupport.dbd
		poka_LIBS_WIN32 += $(AD_LIBS)
		# To add the simDetector uncomment the following lines
		iocpokaLinux_DBD += simDetectorSupport.dbd
		poka_LIBS_WIN32 += simDetector
	endif
endif

ifeq (windows-x64,$(findstring windows-x64, $(T_A)))
	DBD  += iocpokaWin64.dbd
	iocpokaWin64_DBD += poka_Common.dbd

	ifdef MCA
		# If your host supports libpcap and libnet and you want to build Canberra network
		# support uncomment the following line
		iocpokaWin64_DBD += mcaCanberraSupport.dbd
		poka_LIBS_WIN32 += mcaCanberra
		poka_LIBS_WIN32 += wpcap Packet
	endif

	ifdef AREA_DETECTOR
		# areaDetector provides a Makefile for things it needs and supplies
		PROD_NAME = AD
		-include $(ADCORE)/ADApp/commonDriverMakefile
		iocpokaWin64_DBD += ADSupport.dbd NDPluginSupport.dbd
		poka_LIBS_WIN32 += $(AD_LIBS)
		# To add the simDetector uncomment the following lines
		iocpokaLinux_DBD += simDetectorSupport.dbd
		poka_LIBS_WIN32 += simDetector
	endif
endif

ifeq ($(EPICS_HOST_ARCH), cygwin-x86)
	DBD  += iocpokaCygwin.dbd
	iocpokaCygwin_DBD += poka_Common.dbd

	ifdef DXP
		# If you want to build parallel port or USB interfaces to Saturn and/or
		# xMAP DXP,then uncomment the following lines
		#iocpokaCygwin_DBD += dxpSupport.dbd
		#poka_LIBS_cygwin32 += dxp handel
		#poka_SYS_LIBS_cygwin32 += DLPORTIO PlxApi setupapi
	endif

	poka_SYS_LIBS_cygwin32 += $(CYGWIN_RPC_LIB)

endif

ifeq (vxWorks,$(findstring vxWorks, $(T_A)))
	DBD  += iocpokaVX.dbd
	iocpokaVX_DBD += poka_Common.dbd

	ifdef ALLEN_BRADLEY
		poka_LIBS_vxWorks += allenBradley
	endif

	ifdef QUADEM
		iocpokaVX_DBD += drvAHxxx.dbd
		iocpokaVX_DBD += drvAPS_EM.dbd
		poka_LIBS_vxWorks += quadEM
	endif

	ifdef AREA_DETECTOR
		# areaDetector provides a Makefile for things it needs and supplies
		PROD_NAME = AD
		-include $(ADCORE)/ADApp/commonDriverMakefile
		iocpokaVX_DBD += ADSupport.dbd NDPluginSupport.dbd
		poka_LIBS_vxWorks += $(AD_LIBS)
		# To add the simDetector uncomment the following lines
		iocpokaVX_DBD += simDetectorSupport.dbd
		poka_LIBS_vxWorks += simDetector
	endif

	ifdef VME
		iocpokaVX_DBD += vmeVXSupport.dbd
		poka_LIBS_vxWorks += vme
	endif

	ifdef STD
		iocpokaVX_DBD += stdVXSupport.dbd
	endif

	ifdef IP
		iocpokaVX_DBD += ipVXSupport.dbd
	endif

	ifdef MCA
		iocpokaVX_DBD += SIS38XXSupport.dbd
		iocpokaVX_DBD += mcaCanberraSupport.dbd
		poka_LIBS_vxWorks += SIS38XX
		poka_LIBS_vxWorks += mcaCanberra
	endif

	ifdef IPUNIDIG
		iocpokaVX_DBD += ipUnidigSupport.dbd
		poka_LIBS_vxWorks += ipUnidig
	endif

	ifdef DAC128V
		iocpokaVX_DBD += dac128VSupport.dbd
		poka_LIBS_vxWorks += dac128V
	endif

	ifdef IP330
		iocpokaVX_DBD += ip330Support.dbd
		poka_LIBS_vxWorks += ip330
	endif

	ifdef CAMAC
		iocpokaVX_DBD += camacSupport.dbd
		iocpokaVX_DBD += devCamacSupport.dbd
		poka_LIBS_vxWorks += camac devCamac
	endif

	ifdef ALLEN_BRADLEY
		iocpokaVX_DBD += allenBradley.dbd
	endif

	ifdef LOVE
		iocpokaVX_DBD += devLove.dbd
		poka_LIBS_vxWorks += love
	endif

	ifdef IPAC
		iocpokaVX_DBD += tyGSOctal.dbd
		iocpokaVX_DBD += drvIpac.dbd
		poka_LIBS_vxWorks += TyGSOctal Ipac
	endif

	ifdef ASYN
		iocpokaVX_DBD += drvGsIP488.dbd
	endif

	ifdef SOFTGLUE
		iocpokaVX_DBD += softGlueSupport.dbd
		poka_LIBS_vxWorks += softGlue
	endif

	ifdef MOTOR
		iocpokaVX_DBD += devDeltaTau.dbd
		iocpokaVX_DBD += devOms.dbd
		poka_LIBS_vxWorks += oms DeltaTau
	endif

	ifdef DEVIOCSTATS
		iocpokaVX_DBD += devVxStats.dbd
	endif
endif

ifneq (,$(findstring linux, $(T_A)))
	DBD  += iocpokaLinux.dbd
	iocpokaLinux_DBD += poka_Common.dbd

ifeq ($(HAVE_HDF5), YES)
ifeq ($(HAVE_SZIP), YES)

	ifdef QUADEM
		iocpokaLinux_DBD += drvAHxxx.dbd
		poka_LIBS_Linux += quadEM
	endif

	ifdef AREA_DETECTOR
		# areaDetector provides a Makefile for things it needs and supplies
		PROD_NAME = AD
		-include $(ADCORE)/ADApp/commonDriverMakefile
		iocpokaLinux_DBD += ADSupport.dbd NDPluginSupport.dbd
		poka_LIBS_Linux += $(AD_LIBS)
		# To add the simDetector uncomment the following lines
		iocpokaLinux_DBD += simDetectorSupport.dbd
		poka_LIBS_Linux += simDetector
	endif

	ifdef DXP
		# If your host supports parallel port or USB interfaces to Saturn DXP, set
		# LINUX_USB_INSTALLED to YES in <synApps>/configure/CONFIG_SITE.
		ifeq ($(LINUX_USB_INSTALLED), YES)
			iocpokaLinux_DBD += dxpSupport.dbd
			poka_LIBS_Linux  += dxp handel
			poka_SYS_LIBS_Linux += usb
		endif
	endif

endif
endif

	ifdef MCA
		# If your host supports libpcap and libnet and you want to build Canberra network
		# support, set LINUX_NET_INSTALLED to YES in <synApps>/configure/CONFIG_SITE.
		ifeq ($(LINUX_NET_INSTALLED), YES)
			iocpokaLinux_DBD += mcaCanberraSupport.dbd
			poka_LIBS_Linux += mcaCanberra
			poka_SYS_LIBS_Linux += net pcap
		endif
	endif

	ifdef IPAC
		iocpokaLinux_DBD += drvIpac.dbd
		poka_LIBS_Linux += Ipac
	endif

	ifdef SOFTGLUE
		iocpokaLinux_DBD += softGlueSupport.dbd
		poka_LIBS_Linux += softGlue
	endif
endif

ifeq ($(OS_CLASS), Darwin)
	DBD  += iocpokaDarwin.dbd
	iocpokaDarwin_DBD += poka_Common.dbd
endif



# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
poka_SRCS_vxWorks  += iocpokaVX_registerRecordDeviceDriver.cpp
poka_SRCS_Linux    += iocpokaLinux_registerRecordDeviceDriver.cpp     pokaMain.cpp
ifeq (win32-x86,$(findstring win32-x86, $(T_A)))
	poka_SRCS_WIN32    += iocpokaWin32_registerRecordDeviceDriver.cpp     pokaMain.cpp
endif
ifeq (windows-x64,$(findstring windows-x64, $(T_A)))
	poka_SRCS_WIN32    += iocpokaWin64_registerRecordDeviceDriver.cpp     pokaMain.cpp
endif
poka_SRCS_cygwin32 += iocpokaCygwin_registerRecordDeviceDriver.cpp    pokaMain.cpp
poka_SRCS_solaris  += iocpoka_solaris_registerRecordDeviceDriver.cpp  pokaMain.cpp
poka_SRCS_Darwin   += iocpokaDarwin_registerRecordDeviceDriver.cpp    pokaMain.cpp



#The following adds support from base/src/vxWorks
poka_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

poka_LIBS_vxWorks  += $(poka_Common_LIBS)
poka_LIBS_Linux    += $(poka_Common_LIBS)
poka_LIBS_solaris  += $(poka_Common_LIBS)
poka_LIBS_cygwin32 += $(poka_Common_LIBS)
poka_LIBS_WIN32    += $(poka_Common_LIBS)
poka_LIBS_Darwin   += $(poka_Common_LIBS)


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

$(COMMON_DIR)/iocpokaWin32Include.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaWin64Include.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaCygwinInclude.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaVXInclude.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaLinuxInclude.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaDarwinInclude.dbd: $(COMMON_DIR)/poka_Common.dbd

$(COMMON_DIR)/iocpokaWin32.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaWin64.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaCygwin.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaVX.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaLinux.dbd: $(COMMON_DIR)/poka_Common.dbd
$(COMMON_DIR)/iocpokaDarwin.dbd: $(COMMON_DIR)/poka_Common.dbd

iocpokaWin32.dbd$(DEP): $(COMMON_DIR)/poka_Common.dbd
iocpokaWin64.dbd$(DEP): $(COMMON_DIR)/poka_Common.dbd
iocpokaCygwin.dbd$(DEP): $(COMMON_DIR)/poka_Common.dbd
iocpokaVX.dbd$(DEP): $(COMMON_DIR)/poka_Common.dbd
iocpokaLinux.dbd$(DEP): $(COMMON_DIR)/poka_Common.dbd
iocpokaDarwin.dbd$(DEP): $(COMMON_DIR)/poka_Common.dbd
