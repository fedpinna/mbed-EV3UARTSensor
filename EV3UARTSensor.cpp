// EV3UARTSensor.c
//
// Interface to LEGO Mindstorms EV3 UART Sensors
// 
// Author: Lawrie Griffiths (lawrie.griffiths@ntlworld.com)
// Copyright (C) 2014 Lawrie Griffiths
// Modified and adapted for use in mbed by Federico Pinna(fedepinna13@gmail.com)

#include "EV3UARTSensor.h"
#include "SysTimer.h"


/**
 * Create a mode object
**/
EV3UARTMode::EV3UARTMode() {
}

/**
 * Get the mode data type as a string
**/
string EV3UARTMode::get_data_type_string() {
  switch(data_type) {
    case 0: return "Data8";
    case 1: return "Data16";
    case 2: return "Data32";
    case 3: return "DataF";
    default: return "Invalid";
  }
}

/**
 * Create the sensor with the specified RX and TX pins
 * Speed starts at 2400 baud
**/
void EV3UARTSensor::send_nack(){
	if(ss->writeable())
	ss->putc(BYTE_NACK);
}

/*
void EV3UARTSensor::sync_nack(){

	if (this->status == DATA_MODE && (tt.read_ms() - tstart)>HEART_BEAT) {
	    this->ss->putc(BYTE_NACK);
	    tstart=tt.read_ms();
	  }

}*/

EV3UARTSensor::EV3UARTSensor(){
  ss = NULL;
  status = RESET;
  speed = 2400;
  mode = -1;
  num_samples = 1;
}




/**
 * Get the mode object for a specific mode
**/
EV3UARTMode* EV3UARTSensor::get_mode(int16_t mode) {
  return mode_array[mode];
}

/**
 * Start communication with the sensor
**/
void EV3UARTSensor::begin(RawSerial &serial) {
  ss= &serial;
  ss->baud(2400);
}

/**
 * End communication with the sensor
**/
void EV3UARTSensor::end() {
  //ss->close();
}

/**
 * Reset the sensor. It will revert to mode zero.
**/
void EV3UARTSensor::reset() {
  status = RESET;
  this->speed = 2400;
  //ss->close();
  ss->baud(2400);
}

/**
 * This does most of the work. Must be called very frequently. The metadata is processed and the mode objects created.
 * When this is complete, the bit rate is increased and the data is processed.
**/
void EV3UARTSensor::connect(){
	while(status!=DATA_MODE)
		this->check_for_data();
}

void EV3UARTSensor::connect(DigitalOut &led){
	while(status!=DATA_MODE){
		this->check_for_data();
		led=!led;
	}
	led = 1;
}

void EV3UARTSensor::check_for_data() {
  // Send heart beat messages if in data mode
  // Process bytes from the sensor


   //uint32_t primaskValue = 0U;
   //primaskValue = DisableGlobalIRQ();

  if(ss->readable()) {
    uint8_t cmd = ss->getc();
	if (this->status == DATA_MODE) {
#ifdef DEBUG
	  Serial.print("Data received ");
      Serial.println(cmd, HEX);
#endif	 
      // If in data mode, process the data command and set the current value(s)
      if ((cmd & CMD_MASK) == CMD_DATA) {
    	uint8_t lll = (cmd & CMD_LLL_MASK) >> CMD_LLL_SHIFT;
    	uint8_t l = this->exp2(lll); // Number of extra bytes
    	uint8_t mode = (cmd & 7); // The current mode
    	uint8_t checksum = 0xff ^ cmd;
    	uint8_t bb[l];
#ifdef DEBUG
		//Serial.print("Data message: mode: ");
		//Serial.print(mode);
		//Serial.print(" length: ");
		//Serial.println(l);
#endif
        for(int i=0;i<l;i++) {
          bb[i] = this->read_byte();
          checksum ^= bb[i];
        }
        uint8_t sum = this->read_byte();
        // The Color sensor calculates checksums incorrectly in RGB mode
        if ((this->type == TYPE_COLOR && mode == 4) || checksum == sum) {
#ifdef DEBUG
        //Serial.print("Valid data message : ");
        //Serial.print("Data is ");
#endif
		  this->consecutive_errors = 0;
		  recent_messages++;
		  // Extract the data from the message using type information from INFO messages
		  for(int i=0;i<num_samples;i++) {	  
		    switch(mode_array[mode]->data_type) {
		      case 0: this->value[i] = (float) bb[i]; break;
              case 1: this->value[i] = (float) get_int(bb,i*2); break;
              case 2: this->value[i] = (float) get_long(bb,i*4); break;
              case 3: this->value[i] = get_float(bb,i*4); break;	
			}
#ifdef DEBUG
          //    Serial.print(this->value[i]);
         //     Serial.print(" ");
#endif
		  }
#ifdef DEBUG
          //Serial.println();
#endif
        } else {
#ifdef DEBUG
         /* Serial.println("Data checksum error");
		  Serial.print("  Message: ");
		  Serial.print(cmd,HEX);
		  Serial.print(" ");
		  for(int  i=0;i<l;i++) {
		    Serial.print(bb[i],HEX);
			Serial.print(" ");
	      }
		  Serial.print("  Calculated: ");
		  Serial.print(checksum, HEX);
		  Serial.print("  Received: ");
		  Serial.println(sum, HEX);*/
#endif	
		  this->data_errors++;
		  printf("%d\n",data_errors);
		  // If more than 6 errors occur in a row, reset the connection
		  if (this->consecutive_errors++ > 6) reset();
        }
	  }
	  // Ignore all messages except CMD_TYPE until we get a valid CMD_TYPE message
	} else if (this->status == STARTED || cmd == CMD_TYPE) {
	  if (cmd == BYTE_ACK) {
	    // Ack receive
#ifdef DEBUG
		/*Serial.println("Ack received");
		Serial.print("Setting speed to: ");
        Serial.println(this->speed);*/
#endif
        // An ACK is sent by the sensor after all the metadata is sent
		// Send an ACK back, wait a while, and then change the speed
		// to the one given in the CMD_SPEED message
        //uint8_t c;
    	while(ss->readable())
    		ss->getc();
    	ss->putc(BYTE_ACK);
 	 	delay_ms(10);
 	 	ss->baud(speed);
 	 	this->status = DATA_MODE;
		this->data_errors = 0;
		this->consecutive_errors = 0;
		this->recent_messages = 0;
		ss->putc(BYTE_NACK);
		this->heart.attach_us(callback(this,&EV3UARTSensor::send_nack),95000);

	  } else if (cmd == CMD_TYPE) {
#ifdef DEBUG
	    //Serial.println("Type command");
#endif
        // Tpe command is the first metadata command. Extract the type field
        uint8_t checksum = 0xff ^ cmd;
        uint8_t type = this->read_byte();
        checksum ^= type;
        if (checksum == this->read_byte()) {
		  this->type = type;
          this->status = STARTED;
#ifdef DEBUG
		  //Serial.print("Valid type message : ");
		  //Serial.println(type);
#endif		  
        } else {
#ifdef DEBUG		
		  //Serial.println("Type invalid checksum");
#endif
        	//printf("Type invalid checksum\n");
		}
	  } else if (cmd == CMD_MODES) {
	    // The mode command comes after the type command.
		// Extract the number of modes and views
		uint8_t checksum = 0xff ^ cmd;
		uint8_t modes = this->read_byte();
        checksum ^= modes;
        uint8_t views = this->read_byte();
        checksum ^= views;
        if (checksum == this->read_byte()) {
		  this->views = views;
		  this->modes = modes + 1;
		  // Create a mode object for each of the modes
		  for(int i =0;i<=modes && i < MAX_MODES;i++) {
		    this->mode_array[i] = new EV3UARTMode();;
		  }
#ifdef DEBUG
          /*Serial.print("Valid modes message: ");
          Serial.print("modes = ");
          Serial.print(modes);
          Serial.print(" views = ");
          Serial.println(views);*/
#endif
        } else {
#ifdef DEBUG
          //Serial.println("Invalid modes checksum");
#endif		  
        	//printf("Invalid modes checksum\n");
        }
	  } else if (cmd == CMD_SPEED) {
	    // The speed command comes after the MODES command
		// Extract the bit rate to use in data mode
		uint8_t checksum = 0xff ^ cmd;
		uint8_t bb[4];
        for(int i=0;i<4;i++) {
          uint8_t b = this->read_byte();
          checksum ^= b;
          bb[i] = b;
        }
        if (checksum == this->read_byte()) {
	      this->speed  = this->get_long(bb, 0);

	      //printf("speed %d\n",this->speed);
#ifdef DEBUG		  
         /* Serial.print("Valid speed message: ");
          Serial.print("Speed: ");
          Serial.println(speed);*/

#endif		  
        } else {
          //Serial.println("Invalid speed checksum");
        	//printf("Invalid speed checksum\n");
        }
	  } else if ((cmd & CMD_MASK) == CMD_INFO) {
#ifdef DEBUG
       // Serial.print("Info message: ");
#endif
        // A series of INFO commands are given for each mode
		// Modes count down from the highest to zero
		uint8_t lll = (cmd & CMD_LLL_MASK) >> CMD_LLL_SHIFT;
		uint8_t l = this->exp2(lll);
		uint8_t mode = (cmd & CMD_MMM_MASK);
		uint8_t checksum = 0xFF ^ cmd;
		uint8_t type = this->read_byte();
        checksum ^= type;
        uint8_t bb[l];
        for(int i=0;i<l;i++) {
          uint8_t b = this->read_byte();
          checksum ^= b;
          bb[i] = b;
        } 
		string modeName, symbol;
		float low, high;
        if (checksum != this->read_byte()) {
          //printf("Invalid info checksum\n");
        } else {
          switch(type) {
            case 0:
			    // The mode name
			    modeName = this->get_string(bb,l);
				this->mode_array[mode]->name = modeName;
#ifdef DEBUG
              /*Serial.print("Name: ");
              Serial.println(modeName);*/
#endif			  
              break;
            case 1:
			  // The range of raw values
			  low = this->get_float(bb,0);
			  high = this->get_float(bb,4);
			  mode_array[mode]->raw_low = low;
			  mode_array[mode]->raw_high = high;
#ifdef DEBUG			
             /* Serial.print("Raw: ");
              Serial.print(low);
              Serial.print(" ");
              Serial.println(high);*/
#endif			  
              break;
            case 2:
			  // The range of percentage values
			  low = this->get_float(bb,0);
			  high = this->get_float(bb,4);
			  mode_array[mode]->pct_low = low;
			  mode_array[mode]->pct_high = high;
#ifdef DEBUG			
             /* Serial.print("Pct: ");
              Serial.print(low);
              Serial.print(" ");
              Serial.println(high);*/
#endif			  
              break;        
            case 3:
			  // The range of SI values
			  low = this->get_float(bb,0);
			  high = this->get_float(bb,4);
			  mode_array[mode]->si_low = low;
			  mode_array[mode]->si_high = high;
#ifdef DEBUG			
              /*Serial.print("Si: ");
              Serial.print(low);
              Serial.print(" ");
              Serial.println(high);*/
#endif			  
              break;
            case 4:
			  // The unit symbol
			  symbol = this->get_string(bb,l);
			  mode_array[mode]->symbol = symbol;
#ifdef DEBUG			
             /* Serial.print("Symbol: ");
              Serial.println(symbol);*/
#endif			  
              break;
            case 0x80:
			  // The data format including number of data items,
			  // the data time and the number of signicant digits
			  mode_array[mode]->sets = bb[0];
			  mode_array[mode]->data_type = bb[1];
			  mode_array[mode]->figures = bb[2];
			  mode_array[mode]->decimals = bb[3];
#ifdef DEBUG			
           /*   Serial.print("Format: ");
              Serial.print(" Sets: ");
              Serial.print(bb[0]);
              Serial.print(" Data Type: ");
              Serial.print(this->get_data_type(bb[1]));
              Serial.print(" ");
              Serial.print(" Figures: ");
              Serial.print(bb[2]);
              Serial.print(" Decimals: ");
              Serial.print(bb[3]); 
			  // After a format commmand for mode 0 is given, an ACK will be sent
			  // and we reply to it, change the speed, and go into data mode.
              Serial.print(" Mode ");
              Serial.println(mode);*/
#endif         
             break;
          }
        }
	  }
	}
  }

}

/**
 * Utility method to read a byte synchronously
**/
uint8_t EV3UARTSensor::read_byte() {
  while(!ss->readable());
  return ss->getc();
}

/**
 * Utility method to read a long from a byte array
**/
uint32_t EV3UARTSensor::get_long(uint8_t* bb, int16_t offset) {
   return ((uint32_t) bb[offset]) | ((uint32_t) bb[offset+1] << 8) |
         ((uint32_t) bb[offset+2] << 16) | ((uint32_t) bb[offset+3] << 24);
}

/**
 * Utility method to get a string from a byte array
**/
string EV3UARTSensor::get_string(uint8_t* bb, int16_t len) {
  string s = "";
  for(int i=0;i<len;i++) {
    if (bb[i] == 0) break;
    s += (char) bb[i];
  }
  return s;
}

/**
 * Utility method to get a float from a byte array
**/
float EV3UARTSensor::get_float(uint8_t *bb, int16_t offset) {
  union Data {
    uint32_t l;
    float f;
  } data;
  
  data.l = this->get_long(bb,offset);
  return data.f;
}

/**
 * Utility method to get an int from a byte array
**/
int16_t EV3UARTSensor::get_int(uint8_t* bb, int16_t offset) {
  return ((int16_t) bb[offset]) | ((int16_t) bb[offset+1] << 8);
}

/**
 * Utility method t return a small power of 2
**/
int16_t EV3UARTSensor::exp2(int16_t val) {
  switch(val) {
    case 0: return 1;
    case 1: return 2;
    case 2: return 4;
    case 3: return 8;
    case 4: return 16;
    case 5: return 32;
    default: return 0;
  }
}

/**
 * Debugging method to convert INFO message type to a string
**/
string EV3UARTSensor::get_info_type(int16_t val) {
  switch(val) {
    case 0: return "Name";
    case 1: return "Raw";
    case 2: return "Pct";
    case 3: return "Si";
    case 4: return "Symbol";
    case 0x80: return "Format";
    default: return "Invalid";
  }
}

/**
 * Utility method to convert a data type to a string
**/  
string EV3UARTSensor::get_data_type(int16_t val) {
  switch(val) {
    case 0: return "Data8";
    case 1: return "Data16";
    case 2: return "Data32";
    case 3: return "DataF";
    default: return "Invalid";
  }
}

/**
 * Set the sensor mode
**/
void EV3UARTSensor::set_mode(SensorModes mode) {
  this->send_select(mode);
  this->mode = mode;
  this->num_samples = mode_array[mode]->sets;
}


/**
 * Send mode select command to the sensor
**/
void EV3UARTSensor::send_select(uint8_t mode) {
  uint8_t checksum = 0xff ^ CMD_SELECT;
  ss->putc(CMD_SELECT);
  ss->putc(mode);
  checksum ^= mode;
  ss->putc(checksum);
}

/**
 * Send a write command command to the sensor
**/
void EV3UARTSensor::send_write(uint8_t* bb, int16_t len) {
  uint8_t b = CMD_WRITE | (len << CMD_LLL_SHIFT);
  uint8_t checksum = 0xFF ^ b;
  ss->putc(b);
  for(int i=0;i<len;i++) {
    ss->putc(bb[i]);
	checksum ^= bb[i];
  }
  ss->putc(checksum);
}

/**
 * Return the sample size for the current mode
**/
int16_t EV3UARTSensor::sample_size() {
  return this->num_samples;
}

/**
 * Get the current sensor mode
**/
int16_t EV3UARTSensor::get_current_mode() {
  return this->mode;
}

/**
 * Get the number of sensor modes
**/
int16_t EV3UARTSensor::get_number_of_modes() {
  return this->modes;
}

/**
 * Get the sensor type
**/
int16_t EV3UARTSensor::get_type() {
  return this->type;
}

/**
 * Get the status of the connection
**/
int16_t EV3UARTSensor::get_status() {
  return this->status;
}

/**
 * Fetch a sample in the current mode
**/
void EV3UARTSensor::fetch_sample(float* sample, int16_t offset) {
  for(int i=0;i<num_samples;i++) sample[offset+i] = this->value[i];
}

uint32_t EV3UARTSensor::get_speed(){

	return this->speed;
}
