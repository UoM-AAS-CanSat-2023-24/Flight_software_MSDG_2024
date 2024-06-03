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


//---global variables---

// state vars
bool flightState =0;// 0 for ascent, 1 for descent
bool camera1State = 0;// 0 for inactive, 1 for active this is the PID cam and also controls the state of the PID loop
bool camera2State = 0;  // 0 for inactive, 1 for active this cam only actives alt > 600m
bool groundedState =0; // 0 for not grounded 1 for grounded
bool missionTimeState = 1; // 0 for UTC 1 for GPS
bool simState = 0; // 0 for actual flight, 1 for simulation State
bool telemetryState = 0; // 0 for inactive 1 for active 


unsigned long start = millis();
long int programTime; 
long int lastTransmissionTime; // the last recorded program time at which telemetry was sent
long int lastPIDUpdateTime; // last time PID algorithm was updated 
int telemetryMissionTime[3]; // stores the current time sent with any telemetry 
int deltaT; // time used in PID algorithm


// Sensor reading data  IK it's grim t that these are global but I can't think of a better way of doing this 
float satelliteAltitude; // this could change to float depending on size of input from sensor 
float previousAltitude;

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

// TELEMETRY 
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
	static int sPacketCount = 0;
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
	if (satelliteAltitude < previousAltitude)
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
	if(satelliteAltitude = previousAltitude) // again a place holder meaning a much more robust check is likely to be needed 
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

void ReadSensors(bool simSateArg, bool missionTimeSate)
{
	if (simSateArg == 1) 
	{
		txPacket.pressure = currentRxPacket.fltCmdData;
	}
	else
	{
		txPacket.pressure = GetPressure();
		txPacket.altitude = ((pow((SEALEVELPRESSURE_HPA * 0.1) / GetPressure(), 1 / 5.257) - 1) * (GetTemperature() + 273.15)) / 0.0065;

	}
	if (missionTimeSate == 1) 
	{

	}
	else 
	{

	}
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
	

	//EEPROM_data_load();
	flightState = 0; // auto assume ascent state 


	setSyncProvider(RTC.get);   // the function to get the time from the RTC
	if (timeStatus() != timeSet) 
	{
		Serial.println("Unable to sync with the RTC");
	}
	 


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

		if (satelliteAltitude >= 600)
		{
			camera2State = 1;
		} // assumed altitude in meteres 
		else;
		
		if (CheckDescent() == 1)
		{
			flightState = 1; 
			//aerobreak_deploy();
		}


		break;

	case 1: // descending 
		if (CheckGrounded() == 1)
		{
			SoundBuzzer();
		}
		else if (satelliteAltitude <= 100) // again assumed altitude given in meters
		{
			//parachute_deploy();
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

		//EEPROM_data_save(); // only saves data every second instead of every clock cycle
		//SD_data_save();// ^^^

		lastTransmissionTime = programTime;
	}
	else;

	previousAltitude = satelliteAltitude; 
  
}
