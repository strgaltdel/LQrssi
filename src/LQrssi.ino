/*
###################################################################################
#######																		#######
#######		  	"LQrssi", an "arduino sport sketch" to output 	            #######
#######	number of frames which were lost or in failsafe state during flight	#######
#######					and rssi value of sat receivers				        #######
#######	      	like r-xsr which can representate rssi on channels			#######
#######	      scaling from SBUS into rssi value was verified by R-XSR		#######	
#######		     thanks to all who gave valuable infos & inputs 		    #######
#######																		#######
#######																		#######
#######																		#######
#######	 Rev 1.00															#######
#######	 02 Feb 2021														#######
#######	 by strgaltdel														#######
###################################################################################

		This program is free software: you can redistribute it and/or modify
		it under the terms of the GNU General Public License as published by
		the Free Software Foundation, either version 3 of the License, or
		(at your option) any later version.

		This program is distributed in the hope that it will be useful,
		but WITHOUT ANY WARRANTY; without even the implied warranty of
		MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
		GNU General Public License for more details.

		You should have received a copy of the GNU General Public License
		along with this program.  If not, see <https://www.gnu.org/licenses/>.
	
*************************************************************************************/

/*
special thanks to "RealTadango
this sketch is based on his SPORT lib and link quality sensor
https://github.com/RealTadango/FrSky

in addition, this sketch needs the "moving average lib" by Sofian Audrey
https://github.com/sofian/MovingAverage



 some ID information:
0x00	// Physical ID  0 - Vario (altimeter high precision)
0xA1	// Physical ID  1 - FLVSS Lipo sensor (can be sent with one or two cell voltages)
0x22	// Physical ID  2 - FAS-40S current sensor
0x83	// Physical ID  3 - GPS / altimeter (normal precision)
0xE4	// Physical ID  4 - RPM
0x45	// Physical ID  5 - SP2UART(Host)
0xC6	// Physical ID  6 - SPUART(Remote)
0x67	// Physical ID  7 -
0x48	// Physical ID  8 - acc-oXs 
0xE9	// Physical ID  9 -
0x6A	// Physical ID 10 - airspeed
0xCB	// Physical ID 11 -
0xAC	// Physical ID 12 -
0x0D	// Physical ID 13 -
0x8E	// Physical ID 14 -
0x2F	// Physical ID 15 -
0xD0	// Physical ID 16 -
0x71	// Physical ID 17 - FrSky ESC
0xF2	// Physical ID 18 -
0x53	// Physical ID 19 -
0x34	// Physical ID 20 - unisense
0x95	// Physical ID 21 - unisense
0x16	// Physical ID 22 -
0xB7	// Physical ID 23 - IMU ACC (x,y,z)
0x98	// Physical ID 24 -
0x39	// Physical ID 25 - Power Box / RF internal rssi swr etc..
0xBA	// Physical ID 26 - RB10
0x1B	// Physical ID 27 - Fuel (ArduPilot/Betaflight)
*/



#include <SPort.h>										// tadango sport lib https://github.com/RealTadango/FrSky
#include <MovingAverage.h>								// Sofian Audry lib  https://github.com/sofian/MovingAverage



														// **********************  user customizing:
#define SPORT_PIN 4									// Arduino SPort Pin
#define channel_investigate 15							// which channel shall be evaluated
// #define RX_PRIMARY									// remark if you want to monitor Sat Rx
														// **********************  end of  customizing	


// define sensor IDs
#ifdef RX_PRIMARY
	// primary Rx:
	#define PHYSICAL_ID 		0x12					// phys. Sensor main ID
	#define APP_ID_Quality 		0x5100					// APP_ID link quality classic tadango
	#define APP_ID_FrameLoss 	0x5102					// APP_ID sumup FL blocks
	#define APP_ID_FailSafe		0x5104					// APP_ID sumup FS blocks
#else
	// sat Rx:
	#define PHYSICAL_ID 		0x14					// phys. Sensor main ID
	#define APP_ID_Quality 		0x5110					// APP_ID link quality classic tadango
	#define APP_ID_FrameLoss 	0x5112					// APP_ID sumup FL blocks
	#define APP_ID_FailSafe		0x5114					// APP_ID sumup FS blocks
	#define APP_ID_RSSI			0x5116					// APP_ID RSSI (calculated from channel value)
#endif



#define SBUS_BAUD 100000								// sbus "speed"
#define SBUS_THRESHOLD 5
#define SBUS_BUFFER 25									// number of SBus bytes per frame
#define FRAMELOSS_BUFFER 100							// buffer for transmission-quality calculation

#define SBUS_MIN 172.0									// sbus val for -100%
#define SBUS_MAX 1811.0									// sbus val for +100%
#define PWM_CENTER 1500									// FrSky center position (µs)
#define PWM_TRAVEL 512									// FrSky PWM travel from center to 100% (µs)

float sbus_Travel  = (SBUS_MAX-SBUS_MIN)/2;				// sbus travel from center to 100%
float sbus_Center  = SBUS_MIN + sbus_Travel;			// native sbus "center" or "zero%" value
float sbus_Scaling = sbus_Travel/PWM_TRAVEL;			// scaling sbus <> PWM



														// Sport ala tadango
SPortHub hub(PHYSICAL_ID, SPORT_PIN);
SimpleSPortSensor sensor_quality(APP_ID_Quality);
SimpleSPortSensor sensor_fl(APP_ID_FrameLoss);
SimpleSPortSensor sensor_fs(APP_ID_FailSafe);
#ifndef RX_PRIMARY
	SimpleSPortSensor sensor_rssi(APP_ID_RSSI);			// only used by SAT Rx
#endif

byte sbusBuffer[SBUS_BUFFER];							// bytes in sbus frame
long sbusIndex = 0;										// "pointer"
byte sbusPreviousValue = 0;
bool sbusInFrame = false;								// frame detection
bool evaluation = false;								// start evaluation condition after reading complete frame

bool lastFrameLossesBuffer[FRAMELOSS_BUFFER];
int lastFrameLossesIndex = 0;


int16_t chSBus[30];										// channel values (native SBUS)
int16_t chPWM[30];										// channel values (PWM)
int16_t chVal[30];  									// channel values (percent)


														// prepare moving average calc
#define MOVING_AVERAGE_ITERATIONS 140					// the more the smoother
static  MovingAverage avgRSSI(MOVING_AVERAGE_ITERATIONS);
float   rssi_avg;										// averaged variable



// *******************************************   here we go ******************************************************

void setup() {											//register sensors (ala tadango)
    Serial.begin(SBUS_BAUD, SERIAL_8E2);

	hub.registerSensor(sensor_quality);
	hub.registerSensor(sensor_fl);
	hub.registerSensor(sensor_fs);
#ifndef RX_PRIMARY	
	hub.registerSensor(sensor_rssi);
#endif
    hub.begin();
	

}

void loop() {

	hub.handle();										// push telemtry	
														// read SBus frame
    while (Serial.available()) 
		{												// Read MSB value from S.Bus stream
        byte value = Serial.read();
														// Only start a new frame if everything lines up correctly
        if(!sbusInFrame && value == 0x0F && sbusPreviousValue == 0x00) {
            sbusIndex = 0;
            sbusInFrame = true;
			evaluation = true;
        }
														// When in frame, store the value in the buffer
        if(sbusInFrame) {
            sbusBuffer[sbusIndex] = value;
            sbusIndex++;
            
														// If all 25 bytes are received in the frame, handle it
            if(sbusIndex == 25) {
                sbusInFrame = false; 					// Stop capturing
				if(evaluation) {
					handleSBusFrame();						// investigate finished frame
				}
            }
        }

        sbusPreviousValue = value;
    }
}




void handleSBusFrame() {   								// evaluate frame


  bool framelost = sbusBuffer[23] & 0x04;				// frameloss detected ?
  bool failsafe((sbusBuffer[23] >> 3) & 0x0001);		// failsafe detected ?
 
  if (failsafe) {										// if so: inc failsafe counter
	  sensor_fs.value++;
  } 
  else if (framelost) {									// if so: inc frameloss counter
	  sensor_fl.value++;
  }
  

  #ifndef RX_PRIMARY
	ChannelCalc();										// evaluate channel value from sat-rx
  #endif
														// an now, evaluate link quality ( last 100 frames)
  lastFrameLossesBuffer[lastFrameLossesIndex] = framelost;
  lastFrameLossesIndex++;
  if(lastFrameLossesIndex >= FRAMELOSS_BUFFER) {
    lastFrameLossesIndex = 0;
  }
  
  int lastLost = 0;
  
  for(int i = 0; i < FRAMELOSS_BUFFER; i++) {
      if(lastFrameLossesBuffer[i]) {
        lastLost++;
      }
  }
														// average frame losses (percent in FRAMELOSS_BUFFER )
  sensor_quality.value = 100 - (((double)lastLost / (double)FRAMELOSS_BUFFER) * (double)100);
  
  evaluation = false;										// flag evaluation finished
}





//************************************************   do some research in channel values   *****************************************************

void ChannelCalc() {
	int i;
	float tmpcalc = 0;

	//************************************ convert buffer values into native SBus channel values  *************************************/
	chSBus[0]  = (int16_t) (((sbusBuffer[1] |sbusBuffer[2] <<8) & 0x07FF));
	chSBus[1]  = (int16_t) (((sbusBuffer[2]	>>3 |sbusBuffer[3] <<5) & 0x07FF));
	chSBus[2]  = (int16_t) (((sbusBuffer[3]	>>6 |sbusBuffer[4] <<2 |sbusBuffer[5]<<10) & 0x07FF));
	chSBus[3]  = (int16_t) (((sbusBuffer[5]	>>1 |sbusBuffer[6] <<7)	& 0x07FF));
	chSBus[4]  = (int16_t) (((sbusBuffer[6]	>>4 |sbusBuffer[7] <<4) & 0x07FF));
	chSBus[5]  = (int16_t) (((sbusBuffer[7]	>>7 |sbusBuffer[8] <<1 |sbusBuffer[9]<<9) & 0x07FF));
	chSBus[6]  = (int16_t) (((sbusBuffer[9]  >>2 |sbusBuffer[10]<<6) & 0x07FF));
	chSBus[7]  = (int16_t) (((sbusBuffer[10] >>5 |sbusBuffer[11]<<3) & 0x07FF));
	chSBus[8]  = (int16_t) (((sbusBuffer[12] |sbusBuffer[13]<<8) & 0x07FF));
	chSBus[9]  = (int16_t) (((sbusBuffer[13] >>3 |sbusBuffer[14]<<5) & 0x07FF));
	chSBus[10] = (int16_t) (((sbusBuffer[14] >>6 |sbusBuffer[15]<<2 |sbusBuffer[15]<<10) & 0x07FF));
	chSBus[11] = (int16_t) (((sbusBuffer[16] >>1 |sbusBuffer[17]<<7) & 0x07FF));			
	chSBus[12] = (int16_t) (((sbusBuffer[17] >>4 |sbusBuffer[18]<<4) & 0x07FF));
	chSBus[13] = (int16_t) (((sbusBuffer[18] >>7 |sbusBuffer[19]<<1 |sbusBuffer[20]<<9) & 0x07FF));
	chSBus[14] = (int16_t) (((sbusBuffer[20] >>2 |sbusBuffer[21]<<6) & 0x07FF));
	chSBus[15] = (int16_t) (((sbusBuffer[21] >>5 |sbusBuffer[22]<<3) & 0x07FF));
	
	//**************************  			and now, built values of interest  		 	************************************************			

	// single channel eval	

	chVal[channel_investigate] =  (((float)(chSBus[channel_investigate] - SBUS_MIN)/sbus_Travel)-1)*100.5;		// channel percentage value	
	chPWM[channel_investigate] = (chSBus[channel_investigate] -sbus_Center)*sbus_Scaling+PWM_CENTER;			// PWM value
	
																												// convert ch value into rssi value (scaling) & built moving average
	if(chVal[channel_investigate] > (-60)) {
		rssi_avg = avgRSSI.update((chVal[channel_investigate]*0.40)+77);										// scaling chVal > -60% (rssi>50)
	}
	else {
		rssi_avg = avgRSSI.update((chVal[channel_investigate]*0.8)+98);											// scaling chVal < -60% (rssi<50)
	}
	sensor_rssi.value =  (int)rssi_avg ;																		// new (SPORT) rssi value
	

	// all channels:
	// (in case you want to examine other things....)
/*			
	for (i=0; i< 16; i++) {		
		tmpcalc= (((chSBus[i] - 172.0)/820.0)-1.0)*100.5;
		chVal[i] = (int) tmpcalc;																				// channel percentage value
		chPWM[i] = (chSBus[i] -sbus_Center)*sbus_Scaling+PWM_CENTER;											// PWM value
	}			
*/

}