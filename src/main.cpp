#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <ADS1115_WE.h>
#include <AsyncTCP.h>
#include <mydebug.h>
#include <myfunction.h>
#include <SimpleKalmanFilter.h>
// #include <math.h>

#define SDA_PIN 18
#define SCL_PIN 19
#define I2C_ADDRESS 0x48

#define MESURE1_PIN 35
#define MESURE2_PIN 34
#define MESURE3_PIN 39
#define ADCRES 10


#define R1 100000    // voltage divider resistor value
#define BETA 3950.0  // Beta value
#define T0 298.15    // Temperature in Kelvin for 25 degree Celsius
#define R0 100000.0    // Resistance of Thermistor at 25 degree Celsius 94kOhm


// #define ADCMAX 4095  // Résolution MAX
uint16_t ADCMAX = (1<<ADCRES)-1;  // Résolution MAX
#define VSS 3300      // Tension MAX

#define PORTPLOT 47269

// TwoWire myWire = TwoWire(0);
// ADS1115_WE adc = ADS1115_WE(&Wire, I2C_ADDRESS);
ADS1115_WE adc = ADS1115_WE();

float targetV;
float voltage0;
float voltage255;
unsigned char targetInt = 0;
float voltageADC = 0;
float voltageADCFilter = 0;
// int16_t voltageADCRawTemp = 0;
// uint16_t voltageADCRaw = 0;


uint16_t voltageRaw1 = 0;
uint16_t voltageRawFiltered1 = 0;
float voltageESP1 = 0;
uint16_t voltageRaw2 = 0;
uint16_t voltageRawFiltered2 = 0;
float voltageESP2 = 0;
uint16_t voltageRaw3 = 0;
uint16_t voltageRawFiltered3 = 0;
float voltageESP3 = 0;

// Valable pour une résolution de 9 btis = 512 mesures
// float measureError = 10;
// float processNoise = 0.25;

// Valable pour une résolution de 10 btis = 1024 mesures
float measureError = 30;
float processNoise = 0.22;


// SimpleKalmanFilter filterVoltage(0.1, 0.08); //0.08 Amplitude du bruit
// SimpleKalmanFilter filterVoltage(40, 0.04); //40 Amplitude du bruit pour une résolution sur 12 bits 4096
SimpleKalmanFilter filterVoltage1(measureError, processNoise); //10 Amplitude du bruit pour une résolution sur 9 bits 512
SimpleKalmanFilter filterVoltage2(measureError, processNoise); //10 Amplitude du bruit pour une résolution sur 9 bits 512
SimpleKalmanFilter filterVoltage3(measureError, processNoise); //10 Amplitude du bruit pour une résolution sur 9 bits 512
SimpleKalmanFilter filterADC(0.5, 0.4);

uint16_t measureErrorMax = 0;
int measureErrorReset = 0;
int measureErrorResetMax = 70;

int16_t step = 20;

enum mode{
	Step,
	Random,
	File
};

mode activeMode = mode::Step;
bool modeFileFirst = false;

TaskHandle_t pADCRead;
void ADCRead( void * parameter){
	const TickType_t taskPeriod = 10; // 20ms <--> 50Hz
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;)  {
		voltageRaw1 = analogRead(MESURE1_PIN);
		voltageRaw2 = analogRead(MESURE2_PIN);
		voltageRaw3 = analogRead(MESURE3_PIN);

		if (measureErrorReset>0){
			measureErrorReset--;
			measureErrorMax = 0;
		}
		else{
			if (abs(voltageRaw1 - voltageRawFiltered1)>measureErrorMax)
				measureErrorMax = abs(voltageRaw1 - voltageRawFiltered1);
			if (abs(voltageRaw2 - voltageRawFiltered2)>measureErrorMax)
				measureErrorMax = abs(voltageRaw2 - voltageRawFiltered2);
			if (abs(voltageRaw3 - voltageRawFiltered3)>measureErrorMax)
				measureErrorMax = abs(voltageRaw3 - voltageRawFiltered3);
		}
		voltageRawFiltered1 = (uint16_t)filterVoltage1.updateEstimate(voltageRaw1);
		voltageRawFiltered2 = (uint16_t)filterVoltage2.updateEstimate(voltageRaw2);
		voltageRawFiltered3 = (uint16_t)filterVoltage3.updateEstimate(voltageRaw3);
		vTaskDelayUntil(&xLastWakeTime, taskPeriod);
	}
}

unsigned long memMillis;
unsigned long memMillis2;

void sendToTeleplot() {
	static char buffer[300];
	static WiFiUDP udpPlot;

	if (!WiFi.isConnected())
		return;

	voltageESP1 = voltageRawFiltered1 * VSS / ADCMAX;
	voltageESP2 = voltageRawFiltered2 * VSS / ADCMAX;
	voltageESP3 = voltageRawFiltered3 * VSS / ADCMAX;

	sprintf(buffer, "targetInt:%u\ntargetV:%.3f\nvoltageADC:%.3f\nvoltageADCFilter:%.3f\nmeasureErrorMax:%u\nvoltageRaw1:%u\nvoltageRawFiltered1:%u\nvoltageESP1:%.3f\nvoltageRaw2:%u\nvoltageRawFiltered2:%u\nvoltageESP2:%.3f\nvoltageRaw3:%u\nvoltageRawFiltered3:%u\nvoltageESP3:%.3f\n",
			targetInt,
			targetV,
			voltageADC,
			voltageADCFilter,
			measureErrorMax,
			voltageRaw1,
			voltageRawFiltered1,
			voltageESP1,
			voltageRaw2,
			voltageRawFiltered2,
			voltageESP2,
			voltageRaw3,
			voltageRawFiltered3,
			voltageESP3);
	udpPlot.beginPacket(WiFi.broadcastIP(), PORTPLOT);
	udpPlot.print(buffer);
	udpPlot.endPacket();
}

void onReceiveDebug(char *data, size_t len) {
	// DEBUGLOG("Receive %d car %c str %s at %lu\n", len, data[0], data, data);
	switch (data[0]) {
		case '+':
			measureErrorResetMax++;
			DEBUGLOG("MeasureErrorResetMax : %d\n", measureErrorResetMax);
			break;
		case '-':
			measureErrorResetMax--;
			DEBUGLOG("MeasureErrorResetMax : %d\n", measureErrorResetMax);
			break;
		case 'o':
			filterADC.setMeasurementError(filterADC.getMeasurementError()+0.1);
			filterADC.reset();
			DEBUGLOG("ADC MeasurementError : %.3f\n", filterADC.getMeasurementError());
			break;
		case 'l':
			filterADC.setMeasurementError(filterADC.getMeasurementError()-0.1);
			filterADC.reset();
			DEBUGLOG("ADC MeasurementError : %.3f\n", filterADC.getMeasurementError());
			break;
		case 'p':
			filterADC.setProcessNoise(filterADC.getProcessNoise()+0.01);
			filterADC.reset();
			DEBUGLOG("ADC ProcessNoise : %.3f\n", filterADC.getProcessNoise());
			break;
		case 'm':
			filterADC.setProcessNoise(filterADC.getProcessNoise()-0.01);
			filterADC.reset();
			DEBUGLOG("ADC ProcessNoise : %.3f\n", filterADC.getProcessNoise());
			break;
		case '4':
			measureError -= 1;
			filterVoltage1.setMeasurementError(measureError);
			filterVoltage1.reset();
			filterVoltage2.setMeasurementError(measureError);
			filterVoltage2.reset();
			filterVoltage3.setMeasurementError(measureError);
			filterVoltage3.reset();
			DEBUGLOG("MeasurementError : %.3f\n", measureError);
			break;
		case '7':
			measureError += 1;
			filterVoltage1.setMeasurementError(measureError);
			filterVoltage1.reset();
			filterVoltage2.setMeasurementError(measureError);
			filterVoltage2.reset();
			filterVoltage3.setMeasurementError(measureError);
			filterVoltage3.reset();
			DEBUGLOG("MeasurementError : %.3f\n", measureError);
			break;
		case '5':
			processNoise-=0.01;
			filterVoltage1.setProcessNoise(processNoise);
			filterVoltage1.reset();
			filterVoltage2.setProcessNoise(processNoise);
			filterVoltage2.reset();
			filterVoltage3.setProcessNoise(processNoise);
			filterVoltage3.reset();
			DEBUGLOG("ProcessNoise : %.3f\n", processNoise);
			break;
		case '8':
			processNoise+=0.01;
			filterVoltage1.setProcessNoise(processNoise);
			filterVoltage1.reset();
			filterVoltage2.setProcessNoise(processNoise);
			filterVoltage2.reset();
			filterVoltage3.setProcessNoise(processNoise);
			filterVoltage3.reset();
			DEBUGLOG("ProcessNoise : %.3f\n", processNoise);
			break;
		case '6':
			step-=5;
			DEBUGLOG("Step : %d\n", step);
			break;
		case '9':
			step+=5;
			DEBUGLOG("Step : %d\n", step);
			break;
		case 's':
			activeMode = mode::Step;
			DEBUGLOG("Mode Step\n");
			break;
		case 'r':
			activeMode = mode::Random;
			DEBUGLOG("Mode Random\n");
			break;
		case 'f':
			// activeMode = mode::File;
			modeFileFirst = true;
			DEBUGLOG("Mode File\n");
			break;
		case '?':
			DEBUGLOG("0			Reset filter\n");
			DEBUGLOG("4/7		MeasurementError : %.3f\n", measureError);
			DEBUGLOG("5/8		ProcessNoise : %.3f\n", processNoise);
			DEBUGLOG("Calibration 0=%.3fV		255=%.3fV\n", voltage0, voltage255);
			DEBUGLOG("-/+		MeasureErrorResetMax : %d\n", measureErrorResetMax);
			DEBUGLOG("6/9		Step : %d\n", step);
			DEBUGLOG("l/o		ADC MeasurementError : %.3f\n", filterADC.getMeasurementError());
			DEBUGLOG("m/p		ADC ProcessNoise : %.3f\n", filterADC.getProcessNoise());
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

	// adc.setPermanentAutoRangeMode(false);
	// adc.setVoltageRange_mV(ADS1115_RANGE_4096);
	adc.setPermanentAutoRangeMode(true);
	// adc.setConvRate(ADS1115_64_SPS);
	adc.setConvRate(ADS1115_16_SPS);
	adc.setCompareChannels(ADS1115_COMP_0_GND);
	adc.setMeasureMode(ADS1115_CONTINUOUS);
	// adc.setMeasureMode(ADS1115_SINGLE);
	

	dacWrite(25, 255);
	delay(1000);
	voltage255 = (int)adc.getResult_mV();

	dacWrite(25, 0);
	delay(1000);
	voltage0 = (int)adc.getResult_mV();

	pinMode(MESURE1_PIN, INPUT);
	pinMode(MESURE2_PIN, INPUT);
	pinMode(MESURE3_PIN, INPUT);
	analogSetWidth(ADCRES);
	analogReadResolution(ADCRES);

	ArduinoOTA.begin();
	xTaskCreate(ADCRead,"ADCRead",1024,NULL,1,&pADCRead);
}

//	4096 mv	32767
//	3300 mv	26399 26450
//	26399	511
//	26450	511
// voltageADCRawTemp = adc.getRawResult();
// if (voltageADCRawTemp>0)
// 	voltageADCRaw = ((uint32_t)voltageADCRawTemp*(uint32_t)511/(uint32_t)26450) & 0xFFFF;
// else
// 	voltageADCRaw = 0;
// voltageADCRaw = voltageADCRawTemp & 0xFFFF;

uint16_t indice ;
uint16_t lastIndice;
float lastMesure;
uint8_t linecount;

void loop() {
	if (activeMode == mode::File || modeFileFirst){
		if (modeFileFirst){
			modeFileFirst = false;
			targetInt = 0;
			indice = 0;
			lastIndice = 0;
			lastMesure = 0;
			linecount = 0;
			activeMode = mode::File;
		}
		else
			targetInt = (targetInt + 1) & 0xFF;
		measureErrorReset = measureErrorResetMax;
		dacWrite(25, targetInt);
	}
	else
		if ((millis() - memMillis) > 10000){
			memMillis = millis();
			switch (activeMode)
			{
			case mode::Random:
				targetInt = esp_random() & 0xFF;
				break;
			case mode::Step:
				targetInt = ((int16_t)targetInt + step) & 0xFF;
				break;	
			}
			measureErrorReset = measureErrorResetMax;
			dacWrite(25, targetInt);
		}
	// targetV = voltage0 + (voltage255-voltage0)*targetInt/255;
	delay(200);
	// delay(500);
	for(int i = 0; i<3; i++){
		voltageADC = adc.getResult_mV();
		voltageADCFilter = filterADC.updateEstimate(voltageADC);
		delay(100);
	}
	sendToTeleplot();
	ArduinoOTA.handle();
	if (activeMode == mode::File){
		// long double Rt = R1 * voltageADCFilter / (VSS - voltageADCFilter);
		// long double temperature = (1 / ((1 / T0) + (log(Rt / R0) / BETA))) - 273.15;
		uint16_t indiceMesure = (int16_t)(0.5+(float)(voltageRawFiltered1 + voltageRawFiltered2 + voltageRawFiltered3)/3);
		float mesure = voltageADCFilter;
		// SI dernière mesure on prolonge pour avoir jusqu'au ADCMAX (valuer max d'échantillonage)
		if (targetInt == 0xFF && indiceMesure<ADCMAX){
			mesure = lastMesure; // + (ADCMAX-lastIndice) * (mesure-lastMesure)/(indiceMesure-lastIndice);
			indiceMesure = ADCMAX;
		}
		float value;
		while(indice<=indiceMesure){
			if (indice == 0)
				value = indiceMesure;
			else
				value = lastMesure + (indice-lastIndice)*(mesure-lastMesure)/(indiceMesure-lastIndice);

			float Rt = R1 * value / (VSS - value);
			float temperature = (1 / ((1 / T0) + (log(Rt / R0) / BETA))) - 273.15;
			// DEBUGLOG("%u ; %u ; %u ; %lu ; %.1f\n",
			// 	targetInt,
			// 	indice,
			// 	(uint16_t) (value + 0.5),
			// 	(uint32_t) Rt,
			// 	temperature);
			DEBUGLOG("%d,\t",
				(int16_t) (temperature*10));
			linecount++;

			if (linecount==16) {
				DEBUGLOG("\n");
				linecount = 0;
			}

			indice++;
		}
		if (indiceMesure>lastIndice){
			lastIndice = indice - 1;
			lastMesure = mesure;
		}
		
		if (targetInt == 0xFF)
			activeMode=mode::Step;
	}
}
