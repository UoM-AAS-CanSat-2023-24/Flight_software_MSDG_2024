/*
 Name:		Flight_software_MSDG_2024.ino
 Created:	3/27/2024 5:22:23 PM
 Author:	Elliot winterbottom, PUT YOUR NAME HERE!!

 This is the main flight software code for the manchester cansat project 2024. 
	+it runs on the Visual Micro extension for Visual studio
	+it is set to upload onto the Teensy 4.0 board 
	+ Follow this guide to install the correct drivers https://www.youtube.com/watch?v=RcEzUjNr634

	Current problems to solve:

 
*/

 

// this doesn't include lib for voltage sensor 

//auto included 
#include <DFRobot_LWLP.h>
#include <Wire.h>
#include <sh2.h>
#include <sh2_err.h>
#include <sh2_hal.h>
#include <sh2_SensorValue.h>
#include <sh2_util.h>
#include <shtp.h>
#include <Adafruit_PMTK.h>
#include <NMEA_data.h>
#include <bmp3_defs.h>
#include <bmp3.h>
#include <Printers.h>


// manually included 
#include <Adafruit_GPS.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <XBee.h>
#include <DS1307RTC.h>
#include <TimeLib.h>
#include <EEPROM.h>



//---constants---
#define SERVO_PIN_CAMERA 0
#define SERVO_PIN_PARACHUTE 1
#define SERVO_PIN_AEROBREAK 2

#define FIXED_CAM_PIN 6
#define PID_CAM_PIN 7

#define BUZZER_PIN 12
#define LED_PIN 11

#define BMP_SCL 19
#define BMP_SDA 18
#define BNO08X_SDA 19
#define BNO08X_SCL 18
#define SEALEVELPRESSURE_HPA (1013.25)
#define BNO08X_RESET -1
#define GPSSerial Serial2
#define GPSECHO  true
#define BQ34Z100 0x55 

const int STATE_EEPROM_ADDRESS = 0;
const int ALTITUDE_EEPROM_ADDRESS = STATE_EEPROM_ADDRESS + 1;


//---global variables---

// state vars
bool flightState = 0;// 0 for ascent, 1 for descent
bool camera1State = 0;// 0 for inactive, 1 for active this is the PID cam and also controls the state of the PID loop
bool camera2State = 0;  // 0 for inactive, 1 for active this cam only actives alt > 600m
bool groundedState = 0; // 0 for not grounded 1 for grounded
bool missionTimeState = 1; // 0 for UTC 1 for GPS
bool simState = 0; // 0 for actual flight, 1 for simulation State
bool simEnableState = 0; // 0 for not 1 for enabled 
bool telemetryState = 0; // 0 for inactive 1 for active 
bool hsDeployedSate = 0; // 0 for not deployed, 1 for deployed
bool pcDelpoyedSate = 0; // 0 for not deployed, 1 for deployed 



unsigned long start = millis();
long int programTime; 
long int lastTransmissionTime; // the last recorded program time at which telemetry was sent
long int lastPIDUpdateTime; // last time PID algorithm was updated 
int telemetryMissionTime[3]; // stores the current time sent with any telemetry 
int deltaT; // time used in PID algorithm

float altitudeOffset = 0;
float gpsGround = 0;
float previousAltitude = 0;
float previousPressure = 0;


// global objects
XBeeWithCallbacks xbee = XBeeWithCallbacks(); // xbee with callbacks allows for error reporting from xbee chip 
DFRobot_LWLP lwlp;
Adafruit_BMP3XX bmp;
Adafruit_BNO08x  bno085(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
Adafruit_GPS GPS(&GPSSerial);


//---structs ---
struct PacketStruct { // struct to represent general telemetry packet. variables are declared in the same order they should be transmitted 
	int teamID = 2045;
	int missionTime[3];
	int packetCount;
	char mode;
	char* state;
	float altitude;
	int airSpeed;
	char hsDeployed;
	char pcDeployed;
	float temperature;
	float pressure;
	float voltage;
	int   gpsTime[3];
	float gpsAltitude;
	float gpsLatitude;
	float gpsLongitude;
	int   gpsStats;
	float tiltX;
	float tiltY;
	float rotZ;
	char* cmdEcho;
} txPacket;

struct RxPacket
{
	int packetCount = 0;
	char* cmdType = "";
	char* strCmdData = "";
	tmElements_t tm = {0,0,0,0,0,0,0}; // this is a time format used to conver to epochs. it's a struct of uints8_t in form {sec,min,hr, day of the week(odd ik), day, month, year - 1970
	
	float fltCmdData = 0.0;
} currentRxPacket; // global Rx packet


struct euler_t
{
	float yaw;
	float pitch;
	float roll;
} ypr;




//---functions---

//----EEEPROM---

void SaveToEEEPROM(bool flightStateArg,float altArg)
{
	EEPROM.update(STATE_EEPROM_ADDRESS, flightStateArg);
	EEPROM.put(ALTITUDE_EEPROM_ADDRESS, altArg);
}

void RestoreFromEEEPROM()
{
	flightState = EEPROM.read(STATE_EEPROM_ADDRESS);
	EEPROM.get(ALTITUDE_EEPROM_ADDRESS, txPacket.altitude);

}

void ResetEEEPROM()
{
	EEPROM.update(STATE_EEPROM_ADDRESS, 0);
	EEPROM.put(ALTITUDE_EEPROM_ADDRESS, 0.0);

}

//---Telemtry---  
void SendData(char* data) // actual sending of data. Char pointer points to start memory address of string so string arguments can be used
{
	uint8_t* payload = (uint8_t*)data; // payload as in the datat to be sent not to be confused with CANSAT which is rocket payload
	uint32_t payload_len = strlen(data); // assumes our data is string (char array) 
	Tx16Request request; // creates new tx request 
	request.setAddress16(0x0001); // sets address for other xbee (ground station xbee has it's address set as 1 while 
	request.setPayload(payload);
	request.setPayloadLength(payload_len);
	xbee.send(request);
}


void SendTelemetry(PacketStruct packetArg , bool simStateArg)
{
	static int sPacketCount = 0;// static means that it doesn't reset when function goes out of scope i.e. it will keep its value until the MC resets.



	static char sBuf[127]; // a buffer for the data telemetry 
	
	if (!simStateArg) // seperate sim mode telemetry from 
	{
		snprintf(sBuf, 127, "%d,%d:%d:%d,%d,%c,%s,%0.1f,%d,%c,%c,%0.1f,%0.1f,%0.1f,%d:%d:%d,%0.1f,%0.4f,%0.4f,%d,%0.2f,%0.2f,%0.2f,%s", packetArg.teamID, packetArg.missionTime[0], packetArg.missionTime[1], packetArg.missionTime[2], packetArg.packetCount, packetArg.mode, packetArg.state, packetArg.altitude, packetArg.airSpeed, packetArg.hsDeployed, packetArg.pcDeployed, packetArg.temperature, packetArg.pressure, packetArg.voltage, packetArg.gpsTime[0], packetArg.gpsTime[1], packetArg.gpsTime[2], packetArg.gpsAltitude, packetArg.gpsLatitude, packetArg.gpsLongitude, packetArg.gpsStats, packetArg.tiltX, packetArg.tiltY, packetArg.rotZ, packetArg.cmdEcho);
	}
	else
	{
		snprintf(sBuf, 127, "%d,SIMP,%d:%d:%d,%f", packetArg.teamID, packetArg.missionTime[0], packetArg.missionTime[1], packetArg.missionTime[2], packetArg.pressure);
	}
	SendData(sBuf); // here our data passed to function does not need to be referenced as we are passing an array 
	sPacketCount += 1; // updates packet count
	Serial.println(sBuf);
}

void ParsePacket(char* strPacket)
{
	// this function is built on the bases that ALL incoming data is in the format <CMD, 2045, command type, command data>
	static int sPacketCount = 1;
	currentRxPacket.packetCount = sPacketCount;
	sPacketCount += 1;

	char* token;

	//Serial.println("split up output:");
	token = strtok(strPacket, " ,<>");
	//Serial.println(token);
	token = strtok(NULL, " ,<>");
	//Serial.println(token);
	token = strtok(NULL, " ,<>");
	// Serial.println(token);

	currentRxPacket.cmdType = token;

	if (strcmp(token, "CX") == 0) // can't just compare strings in C as our token is technically  a pointer and the compiler has a cry. so instead we use the C-native string compare fnction
	{

		token = strtok(NULL, " ,<>");
		currentRxPacket.strCmdData = token;
		currentRxPacket.fltCmdData = 0;
		currentRxPacket.tm.Second = 0;
		currentRxPacket.tm.Minute = 0;
		currentRxPacket.tm.Hour = 0; 
		currentRxPacket.tm.Day = 0;
		currentRxPacket.tm.Month = 0;
		currentRxPacket.tm.Year = 0;// resets other values 


	}
	else if (strcmp(token, "ST") == 0) 
	{
		token = strtok(NULL, " ,<>");
		if (strcmp(token, "GPS") == 0) // checks to see if it's "GPS" 
		{
			currentRxPacket.strCmdData = token; // all strings need a null char 
			currentRxPacket.fltCmdData = 0;
			currentRxPacket.tm.Second = 0;
			currentRxPacket.tm.Minute = 0;
			currentRxPacket.tm.Hour = 0;
			currentRxPacket.tm.Day = 0;
			currentRxPacket.tm.Month = 0;
			currentRxPacket.tm.Year = 0;// resets other values 
		}

		else  // if it's not further parses it. this only works bc the ST command MUST be one or the other 
		{
			Serial.print(token); // to see if 
			currentRxPacket.strCmdData = "0";
			currentRxPacket.fltCmdData = 0; // resets other values 

			token = strtok(token, " /:");
			currentRxPacket.tm.Hour = (uint8_t)atoi(token); // these lines parse for UTC time given in hours:mins:secs/days/months/years 

			token = strtok(NULL, " /:");
			currentRxPacket.tm.Minute = (uint8_t)atoi(token);

			token = strtok(NULL, " /:");
			currentRxPacket.tm.Second = (uint8_t)atoi(token);

			token = strtok(NULL, " /:");
			currentRxPacket.tm.Day = (uint8_t)atoi(token);

			token = strtok(NULL, " /:");
			currentRxPacket.tm.Month = (uint8_t)atoi(token);

			token = strtok(NULL, " /:");
			currentRxPacket.tm.Year = (uint8_t)(atoi(token)-1970); // gotta be offset from epoch start year :)

		}
	}

	else if (strcmp(token, "SIM") == 0)
	{
		token = strtok(NULL, " ,<>");
		currentRxPacket.strCmdData = token;
		currentRxPacket.fltCmdData = 0;
		currentRxPacket.tm.Second = 0;
		currentRxPacket.tm.Minute = 0;
		currentRxPacket.tm.Hour = 0;
		currentRxPacket.tm.Day = 0;
		currentRxPacket.tm.Month = 0;
		currentRxPacket.tm.Year = 0;// resets other values  


	}
	else if (strcmp(token, "CAL") == 0)
	{
		token = strtok(NULL, " ,<>");
		currentRxPacket.strCmdData = "0";
		currentRxPacket.fltCmdData = 0;
		currentRxPacket.tm.Second = 0;
		currentRxPacket.tm.Minute = 0;
		currentRxPacket.tm.Hour = 0;
		currentRxPacket.tm.Day = 0;
		currentRxPacket.tm.Month = 0;
		currentRxPacket.tm.Year = 0;// resets other values 

	}
	else if (strcmp(token, "BCN") == 0)
	{
		token = strtok(NULL, " ,<>");
		currentRxPacket.strCmdData = token;
		currentRxPacket.fltCmdData = 0;
		currentRxPacket.tm.Second = 0;
		currentRxPacket.tm.Minute = 0;
		currentRxPacket.tm.Hour = 0;
		currentRxPacket.tm.Day = 0;
		currentRxPacket.tm.Month = 0;
		currentRxPacket.tm.Year = 0;// resets other values 

	}
	else if (strcmp(token, "SIMP") == 0)
	{
		token = strtok(NULL, " ,<>");
		currentRxPacket.strCmdData = "0";
		currentRxPacket.fltCmdData = strtof(token, NULL); // conversion from string to float.
		currentRxPacket.tm.Second = 0;
		currentRxPacket.tm.Minute = 0;
		currentRxPacket.tm.Hour = 0;
		currentRxPacket.tm.Day = 0;
		currentRxPacket.tm.Month = 0;
		currentRxPacket.tm.Year = 0;// resets other values 

	}
	else if (strcmp(token,"ERSE")== 0)
	{
		currentRxPacket.strCmdData = "0";
		currentRxPacket.fltCmdData = 0.0; // conversion from string to float.
		currentRxPacket.tm.Second = 0;
		currentRxPacket.tm.Minute = 0;
		currentRxPacket.tm.Hour = 0;
		currentRxPacket.tm.Day = 0;
		currentRxPacket.tm.Month = 0;
		currentRxPacket.tm.Year = 0;// resets other values 
	}

	else
	{
		Serial.print("command not known");
	}

}

void ReceiveTelemetry(Rx16Response& response, uintptr_t)  // function for handling recieved data
{


	Rx16Response rx16 = Rx16Response();
	if (xbee.getResponse().isAvailable()) {
		if (xbee.getResponse().getApiId() == RX_16_RESPONSE) {
			xbee.getResponse().getRx16Response(rx16); // checks for and then gets the Rx16 type response. I'm not sure if this rx is strictly necerssary but don't want to have to fix it if i remove it
		}
	}
	xbee.readPacket(); // reeds a packet pased on the response 

	char dataReadBuf[rx16.getDataLength() + 1]; // creates a new data buffer of the length of the got data which will be deleted after this function goes out of scope


	for (int i = 0; i < rx16.getDataLength(); i++) {

		dataReadBuf[i] = (char)rx16.getData(i); // casts data in from byte to char.
		dataReadBuf[i + 1] = '\0'; // sets second to last to null char so that it behaves like an actual string.
		//Serial.print(dataReadBuf[i]);
	}
	// Serial.print(dataReadBuf);
	ParsePacket(dataReadBuf);
	//Serial.write(response.getData(), response.getDataLength()); // we must also specify the length of our string as it doesn't auto come with a null character at the end (ALTERNATE METHOD OF GETTING DATA)
}

//---Sensors---

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) 
{

	float sqr = sq(qr);
	float sqi = sq(qi);
	float sqj = sq(qj);
	float sqk = sq(qk);

	ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
	ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
	ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

	if (degrees) {
		ypr->yaw *= RAD_TO_DEG;
		ypr->pitch *= RAD_TO_DEG;
		ypr->roll *= RAD_TO_DEG;
	}
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) 
{
	quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) 
{
	quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setReports(void) {
	Serial.println("Setting desired reports");
	if (!bno085.enableReport(SH2_GAME_ROTATION_VECTOR)) {
		Serial.println("Could not enable game vector");
	}
	/*if (!bno085.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
		Serial.println("Could not enable gyroscope");
	}*/
}

float GetTemperature() // temperature sensor 
{
	float temperature = 0;
	if (bmp.performReading()) {
		temperature = bmp.temperature;
	}
	return temperature;
}

float GetPressure() 
{
	float pressure = 0;
	if (bmp.performReading()) {
		pressure = bmp.pressure / 1000.0;
		
	}
	return pressure;
}

float GetBMPaltitude() 
{
	float altitude = 0;
	if (bmp.performReading()) {
		altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
		
	}
	return altitude;
}

float GetPitotVelocity() 
{
	DFRobot_LWLP::sLwlp_t data;
	data = lwlp.getData();
	//Get pressure difference in unit pa 
	float velocity = 0;
	if (abs(data.presure) > 0.2) {
		float density = 1.293;
		velocity = sqrt((2 * abs(data.presure)) / density);
		
	}
	return velocity;
}

float GetTiltX() 
{
	bno085.enableReport(SH2_GAME_ROTATION_VECTOR);
	while (!bno085.getSensorEvent(&sensorValue)) {
		delay(10);
	}
	while (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
		bno085.getSensorEvent(&sensorValue);
	}
	switch (sensorValue.sensorId) {
	case SH2_GAME_ROTATION_VECTOR:
		break;
	}
	return sensorValue.un.gameRotationVector.i;
}

float GetTiltY() 
{
	bno085.enableReport(SH2_GAME_ROTATION_VECTOR);
	while (!bno085.getSensorEvent(&sensorValue)) {
		delay(10);
	}
	while (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) {
		bno085.getSensorEvent(&sensorValue);
	}
	switch (sensorValue.sensorId) {
	case SH2_GAME_ROTATION_VECTOR:
		break;
	}
	return sensorValue.un.gameRotationVector.j;
}

float GetRotationRate() 
{
	bno085.enableReport(SH2_GYROSCOPE_CALIBRATED);
	while (!bno085.getSensorEvent(&sensorValue)) {
		delay(10);
	}
	while (sensorValue.sensorId != SH2_GYROSCOPE_CALIBRATED) {
		bno085.getSensorEvent(&sensorValue);
	}
	switch (sensorValue.sensorId) {
	case SH2_GYROSCOPE_CALIBRATED:
		
		break;
	}
	return sensorValue.un.gyroscope.z;
}

float GetLatitude() {
	float latitude = 0;
	if (GPS.fix) {
		latitude = GPS.latitude;
		
	}
	return latitude;
}

float GetLongitude() {
	float longitude = 0;
	if (GPS.fix) {
		longitude = GPS.longitude;
		
	}
	return longitude;
}

int* GetGPStime() {
	//char date_array[] = { (char)GPS.hour, ':', (char)GPS.minute, ':', (char)GPS.seconds};
	//return date_array;
	int hour = GPS.hour;
	int min = GPS.minute;
	int sec = GPS.seconds;
	int time_array[] = { hour, min, sec };
	return time_array;
}


int GetGPShours() {
	return GPS.hour;
}
int GetGPSmins() {
	return GPS.minute;
}
int GetGPSsecs() {
	return GPS.seconds;
}



float GetGPSaltitude() {
	float altitude = 0;
	if (GPS.fix) {
		altitude = GPS.altitude;
	}
	return altitude;
}

int GetSatellites() {
	int satellites = 0;
	if (GPS.fix) {
		satellites = (int)GPS.satellites;
		
	}
	return satellites;
}

int GetVoltage()
{
	Wire.beginTransmission(BQ34Z100);
	Wire.write(0x08);
	Wire.endTransmission();

	Wire.requestFrom(BQ34Z100, 1);

	unsigned int low = Wire.read();

	Wire.beginTransmission(BQ34Z100);
	Wire.write(0x09);
	Wire.endTransmission();

	Wire.requestFrom(BQ34Z100, 1);

	unsigned int high = Wire.read();

	unsigned int high1 = high << 8;

	int voltage = high1 + low;
	return voltage;//in mV
}

bool CheckGPSfix() {
	
	return GPS.fix;
}

void SetupSensors() {
	

	

	//BMP388
	if (!bmp.begin_I2C()) {//0x77
		Serial.println("Failed to initialise BMP388");
		while (1);
	}
	bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
	bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
	bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
	bmp.setOutputDataRate(BMP3_ODR_50_HZ);
	


	delay(100);

	//LWLP5000:
	while (lwlp.begin() != 0) {//0x01
		Serial.println("Failed to initialise LWLP5000");
		delay(1000);
	}
	



	delay(100);

	//BNO085:
	while (!bno085.begin_I2C(0x4A)) {
		Serial.println("Failed to initialise BNO085");
		delay(1000);
	}
	//setReports();
	


	delay(100);

	//GPS
	while (!GPS.begin(9600)) {
		Serial.println("Failed to initialise GPS");
		delay(1000);
	}
	CheckGPSfix();
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_OFF);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	// help

	

	//BQ34Z100G1
	Wire.begin();
	Wire.beginTransmission(BQ34Z100);
	byte error = Wire.endTransmission();
	if (error == 0) Serial.println("BQ34Z100 online - battery voltage");
	else Serial.println("Failed to initialise BQ34Z1000");



	Serial.println("\nSensor check complete!\n\n");
	

}

//---checkers---

bool CheckDescent()// returns 1 if descent is detected, this is placeholder and a more sophisticated check will likely be needed 
{
	static int descentCounter = 0;
	if (txPacket.altitude < previousAltitude)
	{
		descentCounter += 1;
	}
	else
	{
		descentCounter = 0;
	}
	if(descentCounter >= 3)
	{
		return 1; 
	}
	else
	{
		return 0;
	}
}

bool CheckGrounded()
{
	static int groundedCounter = 0;
	if(round(txPacket.altitude*10)/10 == round(previousAltitude*10)/10 )// rounded to nearest 0.1 of a meter 
	{
		groundedCounter += 1;
	}
	else if (round(GetGPSaltitude()*10)/10 <= gpsGround)
	{
		groundedCounter += 1;
	}
	else
	{
		groundedCounter = 0;
	}

	if(groundedCounter >= 10)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}


//---Hardware actuators--- 

void SoundBuzzer()
{
	digitalWrite(BUZZER_PIN, HIGH); // turn on

}

void SilienceBuzzer()
{
	digitalWrite(BUZZER_PIN, LOW); // turn off 
}

void ReadSensors(bool simSateArg, bool missionTimeSateArg)
{
	if (simSateArg == 1) 
	{
		txPacket.pressure = (currentRxPacket.fltCmdData);
		txPacket.altitude = (((pow((SEALEVELPRESSURE_HPA * 0.1) / currentRxPacket.fltCmdData, 1 / 5.257) - 1) * (GetTemperature() + 273.15)) / 0.0065)-altitudeOffset; // calc altitude from simmed data no filter required 
		
	}
	else
	{
		txPacket.pressure = GetPressure()*0.5+previousPressure*0.5;
		txPacket.altitude = (GetBMPaltitude() - altitudeOffset)*0.5+previousAltitude*0.5; // averaging of last output provides low pass filter 

	}

	if(missionTimeSateArg == 0)
	{
		txPacket.missionTime[0] = GetGPShours();
		txPacket.missionTime[1] = GetGPSmins();
		txPacket.missionTime[2] = GetGPSsecs();
	}
	else
	{
		txPacket.missionTime[0] = hour();
		txPacket.missionTime[1] = minute();
		txPacket.missionTime[2] = second();
	}

	if(simState)
	{
		txPacket.mode = 'S';
	}
	else
	{
		txPacket.mode = 'F';
	}

	
	if(groundedState == 1)
	{
		txPacket.state = "ASCENDING";
	}
	else if (pcDelpoyedSate == 1)
	{
		txPacket.state = "PC_DEPLOYED";
	}
	else if(hsDeployedSate ==1)
	{
		txPacket.state = "HS_DEPLOYED";
	}
	else if(flightState == 1)
	{
		txPacket.state = "DESCENDING";
	}
	else if (flightState == 0)
	{
		txPacket.state = "ASCENDING";
	}
	else
	{
		txPacket.state = "UNDEFINED";
	}
	


	txPacket.airSpeed = GetPitotVelocity();



	if (hsDeployedSate) 
	{
		txPacket.hsDeployed = 'P';
	}
	else
	{
		txPacket.hsDeployed = 'N';
	}

	if (pcDelpoyedSate)
	{
		txPacket.hsDeployed = 'C';
	}
	else
	{
		txPacket.hsDeployed = 'N';
	}
	
	txPacket.temperature = GetTemperature();
	txPacket.voltage = GetVoltage();
	txPacket.gpsTime[0] = GetGPShours();
	txPacket.gpsTime[1] = GetGPSmins();
	txPacket.gpsTime[2] = GetGPSsecs();
	txPacket.gpsAltitude = GetGPSaltitude();
	txPacket.gpsLatitude = GetLatitude();
	txPacket.gpsLongitude = GetLongitude();
	txPacket.gpsStats = GetSatellites();
	txPacket.tiltX = GetTiltX();
	txPacket.tiltY = GetTiltY();
	txPacket.rotZ = GetRotationRate();
	txPacket.cmdEcho = currentRxPacket.cmdType;
}


// commands 
void DoCommand()
{
	static int lastRxPacketCount = 0; 

	if(currentRxPacket.packetCount == lastRxPacketCount) // checks if the last packet number is the same (true if no new command)
	{



	}
	else if(currentRxPacket.packetCount > lastRxPacketCount) // new command!
	{
		if (strcmp(currentRxPacket.cmdType,"CX") == 0) //  looks confusing but == 0 means they are the same.
		{
			if(strcmp(currentRxPacket.strCmdData,"ON")==0)
			{
				telemetryState = 1; 
			}
			else if(strcmp(currentRxPacket.strCmdData, "OFF") == 0)
			{
				telemetryState = 0;
			}
			else
			{
				Serial.println("Unrecognised CX command");
			}
		}

		else if (strcmp(currentRxPacket.cmdType, "ST") == 0)
		{
			if(strcmp(currentRxPacket.strCmdData, "GPS") == 0)
			{
				missionTimeState = 1;
			}
			else // else if isn't really possible here 
			{
				RTC.set(makeTime(currentRxPacket.tm));
				missionTimeState = 0;
			}
		}
		
		else if (strcmp(currentRxPacket.cmdType, "SIM") == 0)
		{
			if(strcmp(currentRxPacket.cmdType, "ENABLE") == 0)
			{
				simEnableState = 1;
			}
			else if (strcmp(currentRxPacket.cmdType, "ACTIVATE") == 0) 
			{
				if(simEnableState = 1)
				{
					simState = 1;
				}
				else
				{
					Serial.println("SIM NOT ENABLED COMMAND IGNORED");
				}
			}
			else if(strcmp(currentRxPacket.cmdType, "DISABLE") == 0)
			{
				simEnableState = 0;
				simState = 0;
			}
			else
			{
				Serial.println("UNRECOGNISED SIM COMMAND");
			}


		}
		else if (strcmp(currentRxPacket.cmdType, "CAL") == 0)
		{
			if(simState)
			{
				altitudeOffset = currentRxPacket.fltCmdData;
				gpsGround = GetGPSaltitude();
			}
			else
			{
				altitudeOffset = GetBMPaltitude();
			}
		}

		else if (strcmp(currentRxPacket.cmdType, "BCN") == 0)
		{
			if (strcmp(currentRxPacket.strCmdData, "ON") == 0)
			{
				SoundBuzzer();
			}
			else if (strcmp(currentRxPacket.strCmdData, "OFF") == 0)
			{
				SilienceBuzzer();
			}
			else
			{
				Serial.println("Unrecognised BCN command");
			}
		}
		
		else if (strcmp(currentRxPacket.cmdType, "ERSE") == 0)
		{
			ResetEEEPROM();
		}
		
		else 
		{
			Serial.println("unrecognised command "); 
		}
	}
	else
	{
		Serial.println("command flow error");
	}

	lastRxPacketCount = currentRxPacket.packetCount;

}

void DeployAerobreak()
{

}

void EjectAerobreak()
{

}

void DeployParachute()
{

}




void setup() 
{
	programTime = millis(); // set starting times
	lastTransmissionTime = millis();


	Serial3.begin(115200); // THIS NEEDS TO BE CHANGED BASED ON CHARS WIRING 
	xbee.setSerial(Serial3); // this uses a different serial (for Teensy 4.1 Rx1 and Tx1)
	Serial.begin(115200);// this serial is simply for outputs to the serial port of a laptop
	xbee.onPacketError(printErrorCb, (uintptr_t)(Print*)(&Serial)); // does some weird shite to print out our errors  on the serial port. these will need commenting out on the final version
	xbee.onTxStatusResponse(printErrorCb, (uintptr_t)(Print*)(&Serial));
	xbee.onOtherResponse(printResponseCb, (uintptr_t)(Print*)(&Serial));
	xbee.onRx16Response(ReceiveTelemetry, 0); // ties our telemetry handling function to any 16 byte Rx events


	//SetupSensors(); THIS NEEDS TO BE UNCOMMENTED WHEN ACTUALLY RUN 
  
	pinMode(BUZZER_PIN, OUTPUT); // setting up buzzer pin 
	

	
	flightState = 0; // auto assume ascent state 
	groundedState = 0;


	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if (timeStatus() != timeSet) 
	{
		Serial.println("Unable to sync with the RTC");
	}
	 

	RestoreFromEEEPROM();
}


// the loop function runs over and over again until power down or reset
void loop() 
{
	xbee.loop();

	//sensor_poll(flightState, simState);
	programTime = millis();


	switch (flightState) // case Statements used as they are likely more effecient than if statments for if equals Statements 
	{
	case 0: // ascending 

		if (txPacket.altitude >= 600)
		{
			camera2State = 1;
		} // assumed altitude in meteres 
		else;
		
		if (CheckDescent() == 1)
		{
			flightState = 1;
			DeployAerobreak();
			SaveToEEEPROM(flightState, txPacket.altitude);
		}
		else;
	


		break;

	case 1: // descending 
		if (CheckGrounded() == 1)
		{
			SoundBuzzer();
			groundedState = 1 ;
		}
		else;

		if (txPacket.altitude <= 100) // again assumed altitude given in meters
		{
			DeployParachute();
			EjectAerobreak();
			SaveToEEEPROM(flightState, txPacket.altitude);

		}
		else;
		
	}


	if (programTime - lastPIDUpdateTime > deltaT) 
	{
		switch (camera1State) 
		{
		case 0: // inactive camera

			

			break;
		case 1: // active camera

				//PID_set();

			break;
		}

		lastPIDUpdateTime = programTime;
	}
	else;


	if (programTime - lastTransmissionTime >= 1000) 
	{
		switch (telemetryState)
		{
		case 0: // inactive telemetry




			break;
		case 1: // active telemetry 


			SendTelemetry(txPacket, simState);


			break;
		}

		lastTransmissionTime = programTime;
	}
	else;

	ReadSensors(simState, missionTimeState);

	DoCommand();

	previousAltitude = txPacket.altitude; 
	previousPressure = txPacket.pressure;
  
}
