
# BEGIN DCMotorServer.cmd ------------------------------------------------------------

# http://www.aps.anl.gov/epics/modules/soft/asyn/R4-18/asynDriver.html

epicsEnvSet("INO","$(PREFIX)dcm:")


############ documentation for the IOC commands #################################
#epicsEnvSet("DCMotorServer","/home/prjemian/sketchbook/DCMotorServer/epics")
#cd ${DCMotorServer}

# Set up 1 local serial port(s)

# USB 0 connected to Arduino at 115200 baud
#drvAsynSerialPortConfigure("portName","ttyName",priority,noAutoConnect, noProcessEos)
#asynOctetSetInputEos(const char *portName, int addr, const char *eosin,const char *drvInfo)
#asynOctetSetOutputEos(const char *portName, int addr, const char *eosin,const char *drvInfo)

# Make port available from the iocsh command line
#asynOctetConnect(const char *entry, const char *port, int addr, int timeout_ms, int buffer_len, const char *drvInfo)

# define the serial port and connect it with asyn
#dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(INO)cr:,PORT=usb0,R=asyn_1,ADDR=0,OMAX=256,IMAX=256")

# define specific Arduino I/O to be used
#dbLoadRecords("DCMotorServer.db","P=$(INO)cr:,PORT=usb0")
############################################################################


drvAsynSerialPortConfigure("usb0", "/dev/ttyUSB0", 0, 0, 0)
asynSetOption(usb0, 0, baud, 115200)
asynOctetSetInputEos("usb0", 0, "\r\n")
asynOctetSetOutputEos("usb0", 0, "\n")

asynOctetConnect("usb0", "usb0", 0, 10)

# ask device to report device ID string
asynOctetWriteRead("usb0","?id\n")

# ask device to report device software version
asynOctetWriteRead("usb0","?v\n")

dbLoadRecords("$(ASYN)/db/asynRecord.db","P=$(INO),PORT=usb0,R=asyn_1,ADDR=0,OMAX=256,IMAX=256")
dbLoadRecords("$(TOP)/pokaApp/Db/DCMotorServer.db","P=$(INO),PORT=usb0")

# turn on diagnostics:
#   asynSetTraceIOMask "usb0" 0 2
#   asynSetTraceMask   "usb0" 0 9

# END DCMotorServer.cmd --------------------------------------------------------------
