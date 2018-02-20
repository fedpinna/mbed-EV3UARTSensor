// EV3UARTSensor.h
//
// Interface to LEGO Mindstorms EV3 UART Sensors
// 
// Author: Lawrie Griffiths (lawrie.griffiths@ntlworld.com)
// Copyright (C) 2014 Lawrie Griffiths
// Modified and adapted for use in mbed by Federico Pinna(fedepinna13@gmail.com)

#include <mbed.h>
#include <string>


/** Example EV3UARTSensor class.
 * @code
 * #include "mbed.h"
 * #include "EV3UARTSensor.h"
 *
 * RawSerial serial3(PTC17,PTC16);
 * DigitalIn sw(SW2);
 * DigitalOut ledg(LED_GREEN,1);
 * EV3UARTSensor sensor;
 * float sample[3];
 *
 * int main(){
 *
 *  initSystemClock();
 *	sensor.begin(serial3);
 *  sensor.connect(ledg); //sensor.connect(); optional
 *
 *  while(true){
 *
 *   sensor.check_for_data();
 *
 *   if(!sw){
 *   	sensor.set_mode(ColColor);
 *   	delay_ms(20);
 *   }
 *
 *   sensor.fetch_sample(sample,0);
 *   printf("Color: %.3f",sample[0]);
 *   delay_ms(100);
 *
 *  }
 *
 * }
 * @endcode
 */

// Message values
#define   BYTE_ACK                      0x04
#define   BYTE_NACK                     0x02
#define   CMD_SELECT                    0x43
#define   CMD_TYPE                      0x40
#define   CMD_MODES                     0x49
#define   CMD_SPEED                     0x52
#define   CMD_MASK                      0xC0
#define   CMD_INFO                      0x80
#define   CMD_LLL_MASK                  0x38
#define   CMD_LLL_SHIFT                 3
#define   CMD_MMM_MASK                  0x07
#define   CMD_DATA                      0xc0
#define   CMD_WRITE                     0x44

#define   TYPE_COLOR                    29

// Values for status
#define RESET 0
#define STARTED 1
#define DATA_MODE 2

// Maximum number of modes supported
#define MAX_MODES 10

// The maximum number of data items in a sample
#define MAX_DATA_ITEMS 10

// The time between heartbeats in milliseconds
#define HEART_BEAT 100

// Set to get message debbugging
//#define DEBUG

/**
* Represent a specific sensor mode
**/
using namespace std;





enum SensorModes{ColReflect,ColAmbient,ColColor,RefRaw,RGBRaw,ColCal};



class EV3UARTMode {
	public:
		EV3UARTMode();
		string name;                      // The mode name
		string symbol;                    // The unit symbol
		uint8_t sets;                        // The number of samples
		uint8_t data_type;                   // The data type 0= 8bits, 1=16 bits, 2=32 bits, 3=float
		uint8_t figures;                     // Number of significant digits
		uint8_t decimals;                    // Number of decimal places
		float raw_low, raw_high;          // Low and high values for raw data
		float si_low, si_high;            // Low and high values for SI data
		float pct_low, pct_high;	      // Low and high values for Percentage values
		string get_data_type_string();    // Get the data type as a string
};

/**
* Represent a generic EV3 UART Sensor
**/

class EV3UARTSensor {
	public:
		EV3UARTSensor(); // Create the sensor and specify the pins for SoftwareSerial
		void begin(RawSerial &serial);                                   // Start communicating with the sensor
		void connect();
		void connect(DigitalOut &led);
		void end();														// End communication with
		void check_for_data();                         // Called from Arduino loop to process all data from the sensor
		int16_t get_number_of_modes();                     // Number of modes supported
		void set_mode(SensorModes mode);                       // Set the sensor to the specific mode
		int16_t get_current_mode();                        // The current sensor mode
		int16_t sample_size();                             // The number of items in a sample for the current mode
		void fetch_sample(float* sample, int16_t offset);  // Fetch a sample in the current mode
		int16_t get_status();                              // Get the status of the connection
		EV3UARTMode* get_mode(int16_t mode);               // Get the EV3UARTMode object for a specific mode
		void reset();                                  // Make the sensor reset
		void send_write(uint8_t* bb, int16_t len);            // Send a WRITE command to the sensor
		int16_t get_type();                                // Get the LEGO type code for the sensor
		uint32_t get_speed();
	private:
		void send_nack();
	    uint8_t read_byte();                              // Read a byte from the sensor (synchronous)
		uint32_t get_long(uint8_t* bb, int16_t offset);  // Helper method to get a long value
		string get_string(uint8_t* bb, int16_t len);          // Helper method to get a String value
		float get_float(uint8_t* bb, int16_t len);            // Helper method to get a float value
		int16_t exp2(int16_t val);                             // Helper method for powers of 2
		string get_info_type(int16_t val);                 // Helper method to get type of INFO message
		string get_data_type(int16_t val);                 // Helper method to get the data type as a string
		void send_select(uint8_t mode);                   // Send a CMD_SELECT command t change modes
		int16_t get_int(uint8_t* bb, int16_t offset);             // Helper method to get an int
		uint32_t speed;                           // The required bit rate of the sensor
		uint8_t mode;                                     // The current sensor mode
		uint8_t status;                                   // The current status of the connection
		uint8_t modes;                                    // The number of modes supported
		uint8_t views;                                    // The number of views supported
		uint32_t last_nack;                       // The time of the last heartbeat NACK
		uint8_t type;                                     // The internal type encoding of the sensor
		RawSerial *ss;                            // Reference to the SoftwareSerial object
		int16_t data_errors;                               // Total number of data errors
		float value[MAX_DATA_ITEMS];                   // The current value
		int16_t num_samples;                               // The current number of samples
		EV3UARTMode* mode_array[MAX_MODES] ;           // An array of EV3UARTMode objects
		uint8_t consecutive_errors;                       // Number of sonsective errors
		int16_t recent_messages;                           // Number of recent messages
		Ticker heart;
};
