/*
 Name:		Flight_software_MSDG_2024.ino
 Created:	3/27/2024 5:22:23 PM
 Author:	Elliot winterbottom, PUT YOUR NAME HERE!!

 This is the main flight software code for the manchester cansat project 2024. 
	+it runs on the Visual Micro extension for Visual studio
	+it is set to upload onto the Teensy 4.0 board 
	+ Follow this guide to install the correct drivers https://www.youtube.com/watch?v=RcEzUjNr634

 This code is formatted as such:
	+camelCase is used for variables
	+ALLCAPS is used for constants
	+PascalCase is used for classes
	+snake_case is used for functions excluding the setup and loop functions.

	Current problems to solve:
	- Which time to be used for which mechanisms?
	- pulling data from sensors (Ask Emily)
	- pushing data to telemetry (ask Matej)
	-
 
*/


//---constants---
#define SERVO_PIN_CAMERA 0
#define SERVO_PIN_PARACHUTE 1
#define SERVO_PIN_AEROBREAK 2

#define FIXED_CAM_PIN 6
#define PID_CAM_PIN 7

#define BUZZER_PIN 12
#define LED_PIN 11


//---global variables---
bool flightState;// 0 for ascent, 1 for descent
bool cameraState;// 0 for inactive, 1 for active
bool simState; // 0 for actual flight, 1 for simulation State
bool telemetryState; // 0 for inactive 1 for active 

long int programTime; // needs to be set in millis or micros depending on what louis says 
long int lastTransmissionTime; // the last recorded program time at which telemetry was sent
long int lastPIDUpdateTime; // last time PID algorithm was updated 
int telemetryMissionTime[3]; // stores the current time sent with any telemetry 
int deltaT; // time used in PID algorithm

double satelliteAltitude; // this could change to float depending on size of input from sensor 
double previousAltitude;


//---function prototypes--- OK I swear these are meant to be prototypes but c++ really doesn't like it when you try and compile while calling functions that aren't defined 

void sensor_setup() {} // sets up sensors one I2c busses and sets default values.
void sensor_poll(bool flightStateArg, bool simStateArg) {} // polls sensors based on the State of the Statellite 

void telemetry_transmit() {} // transmits data 
void telemetry_receive() {} // receive data for sim mode
void telemetry_command_check() {}

void EEPROM_data_save() {} // saves data from sensors to EEPROM along with the current State of the cansat
void EEPROM_data_load() {}// loads above data from EEEPROM if one exists 
void SD_data_save() {}

void buzzer_toggle() {} // activates and deactivates the buzzer

void aerobreak_deploy() {} // deploys the heat sheild  and parachute respectively 
void parachute_deploy() {}

void PID_set() {} // sets values for PID loop and will update servo motor control



//---function definitions---
bool descent_check()// returns 1 if descent is detected, this is placeholder and a more sophisticated check will likely be needed 
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

bool grounded_check()
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



// the setup function runs once when you press reset or power the board
void setup() 
{
	programTime = millis(); // set starting times
	lastTransmissionTime = millis();

	sensor_setup();  
	EEPROM_data_load();
	flightState = 0;



}

// the loop function runs over and over again until power down or reset
void loop() 
{

	
	telemetry_command_check();
	sensor_poll(flightState, simState);
	programTime = millis();


	switch (flightState) // case Statements used as they are likely more effecient than if statments for if equals Statements 
	{
	case 0: // ascending 

		if (satelliteAltitude >= 600)
		{
			cameraState = 1;
		} // assumed altitude in meteres 
		else;
		
		if (descent_check() == 1)
		{
			flightState = 1; 
			aerobreak_deploy();
		}


		break;

	case 1: // descending 
		if (grounded_check() == 1)
		{
			buzzer_toggle();
		}
		else if (satelliteAltitude <= 100) // again assumed altitude given in meters
		{
			parachute_deploy();
		}
		else;
		
	}


	if (programTime - lastPIDUpdateTime > deltaT) 
	{
		switch (cameraState) 
		{
		case 0: // inactive camera

			PID_set();

			break;
		case 1: // active camera



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


			telemetry_transmit();


			break;
		}

		EEPROM_data_save(); // only saves data every second instead of every clock cycle
		SD_data_save();// ^^^

		lastTransmissionTime = programTime;
	}
	else;

	previousAltitude = satelliteAltitude; 
  
}
