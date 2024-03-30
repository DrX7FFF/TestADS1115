#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <ADS1115_WE.h>
#include <AsyncTCP.h>
#include <mydebug.h>
#include <myfunction.h>
#include <SimpleKalmanFilter.h>

#define SDA_PIN 18
#define SCL_PIN 19
#define I2C_ADDRESS 0x48

#define MESURE_PIN 35
#define ADCRES 9
// #define ADCMAX 4095  // Résolution MAX
uint16_t ADCMAX = (1<<ADCRES)-1;  // Résolution MAX
#define VSS 3.3      // Tension MAX

#define PORTPLOT 47269

// TwoWire myWire = TwoWire(0);
// ADS1115_WE adc = ADS1115_WE(&Wire, I2C_ADDRESS);
ADS1115_WE adc = ADS1115_WE();

float voltage0;
float voltage255;
unsigned char targetInt = 0;
float voltageADC = 0;
int16_t voltageADCRawTemp = 0;
uint16_t voltageADCRaw = 0;
uint16_t voltageRaw = 0;
uint16_t voltageRawFiltered = 0;

float voltageESP = 0;
float voltageESPFiltered = 0;

// SimpleKalmanFilter filterVoltage(0.1, 0.08); //0.08 Amplitude du bruit
// SimpleKalmanFilter filterVoltage(40, 0.04); //40 Amplitude du bruit pour une résolution sur 12 bits 4096
SimpleKalmanFilter filterVoltage(10, 0.02); //10 Amplitude du bruit pour une résolution sur 9 bits 512

unsigned long memMillis;
unsigned long memMillis2;

void sendToTeleplot() {
	static char buffer[300];
	static WiFiUDP udpPlot;

	if (!WiFi.isConnected())
		return;

	float targetV = voltage0 + (voltage255-voltage0)*targetInt/255;
	sprintf(buffer, "targetInt:%u\ntargetV:%.3f\nvoltageADC:%.3f\nvoltageESP:%.3f\nvoltageESPFiltered:%.3f\nvoltageDeltaRaw:%d\nvoltageRaw:%u\nvoltageRawFiltered:%u\nvoltageADCRaw:%u\nvoltageADCRawTemp:%d",
			targetInt,
			targetV,
			voltageADC,
			voltageESP,
			voltageESPFiltered,
			voltageADCRaw - voltageRawFiltered,
			voltageRaw,
			voltageRawFiltered,
			voltageADCRaw,
			voltageADCRawTemp);
	udpPlot.beginPacket(WiFi.broadcastIP(), PORTPLOT);
	udpPlot.print(buffer);
	udpPlot.endPacket();
}

void onReceiveDebug(char *data, size_t len) {
	// DEBUGLOG("Receive %d car %c str %s at %lu\n", len, data[0], data, data);
	switch (data[0]) {
		case '4':
			filterVoltage.setMeasurementError(filterVoltage.getMeasurementError()/2);
			filterVoltage.reset();
			DEBUGLOG("MeasurementError : %.3f\n", filterVoltage.getMeasurementError());
			break;
		case '7':
			filterVoltage.setMeasurementError(filterVoltage.getMeasurementError()*2);
			filterVoltage.reset();
			DEBUGLOG("MeasurementError : %.3f\n", filterVoltage.getMeasurementError());
			break;
		case '5':
			filterVoltage.setProcessNoise(filterVoltage.getProcessNoise()/2);
			filterVoltage.reset();
			DEBUGLOG("ProcessNoise : %.3f\n", filterVoltage.getProcessNoise());
			break;
		case '8':
			filterVoltage.setProcessNoise(filterVoltage.getProcessNoise()*2);
			filterVoltage.reset();
			DEBUGLOG("ProcessNoise : %.3f\n", filterVoltage.getProcessNoise());
			break;
		case '0':
			filterVoltage.reset();
			break;
		case '?':
			DEBUGLOG("0			Reset filter\n");
			DEBUGLOG("4/7		MeasurementError : %.3f\n", filterVoltage.getMeasurementError());
			DEBUGLOG("5/8		ProcessNoise : %.3f\n", filterVoltage.getProcessNoise());
			DEBUGLOG("Calibration 0=%.3fV		255=%.3fV\n", voltage0, voltage255);
			break;
	}
}

void setup() {
	DEBUGINIT(onReceiveDebug);

	loadIP();
	myWifiBeginWPS();

	Wire.setPins(SDA_PIN, SCL_PIN);
	Wire.begin();

	/*
		byte error, address;
		int nDevices;

		DEBUGLOG("Scanning...\n");

		nDevices = 0;
		for (address = 1; address < 127; address++) {
			// The i2c_scanner uses the return value of
			// the Write.endTransmisstion to see if
			// a device did acknowledge to the address.
			Wire.beginTransmission(address);
			error = Wire.endTransmission();

			if (error == 0) {
				DEBUGLOG("I2C device found at address 0x%02X\n", address);
				nDevices++;
			} else if (error == 4) {
				DEBUGLOG("Unknown error at address 0x%02X\n", address);
			}
		}
		if (nDevices == 0)
			DEBUGLOG("No I2C devices found\n");
		else
			DEBUGLOG("done\n");
	*/
	if (!adc.init())
		DEBUGLOG("ADS1115 not connected!\n");

	adc.setPermanentAutoRangeMode(false);
	adc.setVoltageRange_mV(ADS1115_RANGE_4096);
	// adc.setConvRate(ADS1115_64_SPS);
	adc.setConvRate(ADS1115_16_SPS);
	adc.setCompareChannels(ADS1115_COMP_0_GND);
	adc.setMeasureMode(ADS1115_CONTINUOUS);
	// adc.setMeasureMode(ADS1115_SINGLE);
	

	dacWrite(25, 255);
	delay(1000);
	voltage255 = (int)adc.getResult_V();

	dacWrite(25, 0);
	delay(1000);
	voltage0 = (int)adc.getResult_V();

	pinMode(MESURE_PIN, INPUT);
	analogSetWidth(ADCRES);
	analogReadResolution(ADCRES);

	ArduinoOTA.begin();
}

//	4096 mv	32767
//	3300 mv	26399 26450
//	26399	511
//	26450	511
void loop() {
	if ((millis() - memMillis) > 4000){
		memMillis = millis();
		targetInt++;
		dacWrite(25, targetInt);
		// DEBUGLOG("voltageADCRawTemp %lu\tvoltageADCRaw %u\n",voltageADCRawTemp,voltageADCRaw);
	}
	delay(50);

	voltageADC = adc.getResult_V();
	voltageADCRawTemp = adc.getRawResult();
	if (voltageADCRawTemp>0)
		voltageADCRaw = ((uint32_t)voltageADCRawTemp*(uint32_t)511/(uint32_t)26450) & 0xFFFF;
	else
		voltageADCRaw = 0;
	// voltageADCRaw = voltageADCRawTemp & 0xFFFF;
	voltageRaw = analogRead(MESURE_PIN);
	voltageRawFiltered = (uint16_t)filterVoltage.updateEstimate(voltageRaw);
	voltageESPFiltered = voltageRawFiltered * VSS / ADCMAX;
	// voltageESPFiltered = filterVoltage.updateEstimate(voltageESP);

	if ((millis() - memMillis2) > 500){
		memMillis2 = millis();
		sendToTeleplot();
		ArduinoOTA.handle();
	}
}

