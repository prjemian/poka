# soft scaler
dbLoadRecords("$(ASYN)/db/asynRecord.db","P=poka:,R=asynScaler,PORT=scaler1Port,ADDR=0,OMAX=0,IMAX=0")
# drvScalerSoftConfigure(char *portName, int maxChans, char *pvTemplate)
drvScalerSoftConfigure("scaler1Port", 8, "poka:scaler1:s%d")
dbLoadRecords("$(STD)/stdApp/Db/scaler.db","P=poka:,S=scaler1,OUT=@asyn(scaler1Port 0 0),DTYP=Asyn Scaler,FREQ=10000000")
dbLoadRecords("$(STD)/stdApp/Db/scalerSoftCtrl.db","P=poka:,Q=scaler1:,SCALER=poka:scaler1")
