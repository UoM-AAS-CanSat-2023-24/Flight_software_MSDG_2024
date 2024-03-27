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
 
*/


//---constants---


//---global variables---
bool flightSate;// 0 for ascent, 1 for descent
bool cameraSate;// 0 for inactive, 1 for active
bool simSate; // 0 for actual flight, 1 for simulation state
bool groundedSate; // 0 for airborne 1 for grounded 



//---function prototypes---



//---function definitions---


// the setup function runs once when you press reset or power the board
void setup() 
{
	

}

// the loop function runs over and over again until power down or reset
void loop() 
{


	switch (flightSate)
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
  
}
