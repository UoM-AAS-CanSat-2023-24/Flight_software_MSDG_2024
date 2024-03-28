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
bool flightSate;// 0 for ascent, 1 for descent
bool cameraSate;// 0 for inactive, 1 for active
bool simSate; // 0 for actual flight, 1 for simulation state
bool groundedSate; // 0 for airborne 1 for grounded 
bool telemetrySate; // 0 for inactive 1 for active 

long int programTime; // needs to be set in millis or micros depending on what louis says 
int telemetryTime[3]; // stores the current time sent with any telemetry 

double satelliteAltitude; // this could change to float depending on size of input from sensor 



//---function prototypes---

void sensor_setup(); // sets up sensors one I2c busses 
void sensor_poll(bool flightSateArg, bool simSateArg ); // polls sensors based on the state of the satellite 

void telemetry_transmit(); // transmits data 
void telemetry_receive(); // receive data for sim mode

void EEPROM_data_save(); // saves data from sensors to EEPROM
void EEPROM_data_load();
void SD_data_save();

void buzzer_control(); // activates and deactivates the buzzer

void aerobreak_deploy(); // deploys the heat sheild  and parachute respectively 
void parachute_deploy();

void PID_set(); // sets values for PID loop and will update servo motor control





//---function definitions---




// the setup function runs once when you press reset or power the board
void setup() 
{
	

}

// the loop function runs over and over again until power down or reset
void loop() 
{



	switch (flightSate) // case statements used as they are likely more effecient than if statments for massively reapeted satements 
	{
	case 0: // ascending 



		break;

	case 1: // descending 



		break;
	}

	switch (cameraSate)
	{
	case 0: // inactive camera




		break;
	case 1: // active camera



		break;
	}

	switch (telemetrySate)
	{
	case 0: // inactive telemetry




		break;
	case 1: // active telemetry 



		break;
	}
  
}
