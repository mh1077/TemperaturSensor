// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

void printAddress(DeviceAddress deviceAddress);
void printResolution(DeviceAddress deviceAddress);
void printData(DeviceAddress deviceAddress);

void swap(float* xp, float* yp);
void selectionSort(float arr[], int n);

//void saveConfigFile();
//bool loadConfigFile();
void saveConfigCallback();
void configModeCallback(WiFiManager *myWiFiManager);

void messureAndSort(void);
void messureSortAndSave(void);
void saveDataToFile(void);
void syncTime(void);

int DataSaving(void);
void mountAndReadData(void);

// Callbacks von FTP Server
void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace);
void _transferCallback(FtpTransferOperation ftpOperation, const char* name, unsigned int transferredSize);

void printAddress(DeviceAddress deviceAddress);
String printAddressAsString(DeviceAddress deviceAddress);