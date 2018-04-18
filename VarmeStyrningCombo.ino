/*


*/
#include "ApplicationMonitor.h"
#include <Ethernet.h>
#include <SPI.h>
#include "VarmeStyrningCombo.h"
#include <OneWire.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <EEPROM.h>

#define Serialprint Serial.print
#define Serialprintln Serial.println
#define Serialwrite Serial.write

//     RFM 01 Define section
#define RF_PORT	PORTD
#define RF_DDR	DDRD
#define RF_PIN	PIND


//Can probably reuse SPI interface at some point
#define SDI_RFM		6	// RF01  SDI_RFM,  arduino  13 cannot be changed
#define SCK_RFM		5	// RF01  SCK_RFM,  arduino  12 cannot be changed
#define CS_RFM		3	// RF01  nSEL, arduino  11 cannot be changed
#define SDO_RFM		2	// RF01  SDO_RFM,  arduino  10 cannot be changed

#define MSG_LEN   8
#define MSG_PENDING 0
#define MSG_WAIT 1
#define MSG_READ 2
unsigned char msg_buf[MSG_LEN];  // recv buf
unsigned int msg_state = MSG_WAIT;
//     END RFM 01 Define section

//     Ethernet Define section

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
byte mac[] = {  
  0x00, 0x4A, 0x3B, 0x2C, 0xDE, 0x02 };

// if you don't want to use DNS (and reduce your sketch size)
// use the numeric IP instead of the name for the server:
//IPAddress server(74,125,232,128);  // numeric IP for Google (no DNS)
char emon_server[] = "emoncms.org";    // name address for Google (using DNS)

// Initialize the Ethernet client library
// with the IP address and port of the server
// that you want to connect to (port 80 is default for HTTP):
EthernetClient client;

char httpreq[130];
int bytecnt = 0;
unsigned long sendtime;

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);

//END     Ethernet Define section

//        WH2 define section

// Read data from 433MHz receiver on digital pin 7
#define RF_IN 7
#define RF_IN_RAW PIND7
#define RF_IN_PIN PIND

#define COUNTER_RATE 3200-1 // 16,000,000Hz / 3200 = 5000 interrupts per second, ie. 200us between interrupts
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval)   (interval >= 2 && interval <= 3)
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval)  (interval >= 7 && interval <= 8)
// worst case packet length
// 6 bytes x 8 bits x (1.5 + 1) = 120ms; 120ms = 200us x 600
#define HAS_TIMED_OUT(interval) (interval > 600)
// we expect 1ms of idle time between pulses
// so if our pulse hasn't arrived by 1.2ms, reset the wh2_packet_state machine
// 6 x 200us = 1.2ms
#define IDLE_HAS_TIMED_OUT(interval) (interval > 6)
// our expected pulse should arrive after 1ms
// we'll wh2_accept it if it arrives after
// 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 4)
// Shorthand for tests
//#define RF_HI (digitalRead(RF_IN) == HIGH)
//#define RF_LOW (digitalRead(RF_IN) == LOW)
#define RF_HI (bit_is_set(RF_IN_PIN, RF_IN_RAW))
#define RF_LOW (bit_is_clear(RF_IN_PIN, RF_IN_RAW))

// wh2_flags 
#define GOT_PULSE 0x01
#define LOGIC_HI  0x02
volatile byte wh2_flags = 0;
volatile byte wh2_packet_state = 0;
volatile int wh2_timeout = 0;
byte wh2_packet[5];
byte wh2_calculated_crc;

ISR(TIMER1_COMPA_vect)
{
  static byte sampling_state = 0;
  static byte count;   
  static boolean was_low = false; 
    
  switch(sampling_state) {
    case 0: // waiting
      wh2_packet_state = 0;
      if (RF_HI) {
        if (was_low) {
          count = 0;
          sampling_state = 1;
          was_low = false;
        }
      } else {
        was_low = true;  
      }
      break;
    case 1: // acquiring first pulse
      count++;
      // end of first pulse
      if (RF_LOW) {
        if (IS_HI_PULSE(count)) {
          wh2_flags = GOT_PULSE | LOGIC_HI;
          sampling_state = 2;
          count = 0;        
        } else if (IS_LOW_PULSE(count)) {
          wh2_flags = GOT_PULSE; // logic low
          sampling_state = 2;
          count = 0;      
        } else {
          sampling_state = 0;
        }    
      }   
      break;
    case 2: // observe 1ms of idle time
      count++;
      if (RF_HI) {
         if (IDLE_HAS_TIMED_OUT(count)) {
           sampling_state = 0;
         } else if (IDLE_PERIOD_DONE(count)) {
           sampling_state = 1;
           count = 0;
         }
      }     
      break;     
  }
  
  if (wh2_timeout > 0) {
    wh2_timeout++; 
    if (HAS_TIMED_OUT(wh2_timeout)) {
      wh2_packet_state = 0;
      wh2_timeout = 0;
    }
  }
}

// END        WH2 define section


//        Temperature sensor define section
OneWire  ds(8);
byte temp_addr[8] = {  0x28, 0xFF, 0x7E, 0x5B, 0x10, 0x14, 0x00, 0x56 };
float celsius, bor_temp;

// END        Temperature sensor define section


// Motor shield define section

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

// END Motor shield define section


// SHUNT define section
#define SHUNT_NORMAL  0x00
#define SHUNT_OPEN_END  0x01
#define SHUNT_CLOSE_END 0x02
#define ADJUSTMENT_PERIOD 2*60*1000UL
byte end_point;
int end_point_count = 0;
int movement_count = 0;

// End SHUNT define section


// END End sensor define section
#define END_IN 9
#define END_IN_RAW PINB1
#define END_IN_PIN PINB
// END End sensor define section

void software_Reboot()
{
  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_15ms);
  while(1)
  {
  }
}

Watchdog::CApplicationMonitor ApplicationMonitor;

#define SETTEMP_ADR  0
#define END_POINT_ADR 1

void setup() {
  Serial.begin(9600);

  Serialprintln(F("Start! "));
  ApplicationMonitor.Dump(Serial);

  Serialprintln("Read stored desired temperature & latest end direction of motor from NVRAM");
  bor_temp = 45;
  byte temp = EEPROM.read(SETTEMP_ADR);
  if(temp > 1 && temp < 70)
  {
    bor_temp = (float) temp;
    Serialprint(F("Set temp: "));
    Serialprintln(temp);    
  }

  if(END_IN_PIN & B00000010)
  {
    Serialprint(F("Binary 1"));
  }
  else
  {
    Serialprint(F("Binary 0"));
  }
  

  end_point = SHUNT_NORMAL;
  temp = EEPROM.read(END_POINT_ADR);
  if(temp == SHUNT_OPEN_END || temp == SHUNT_CLOSE_END)
  {
    if (!(END_IN_PIN & B00000010))
    {
      end_point = temp;
    }
    else
    {
      //Clear endpoint
      EEPROM.write(END_POINT_ADR, end_point);
    }
  }
    
    Serialprint(F("Shunt state: "));
    Serialprintln(end_point);
  

  //Init powermeter receiver
  rf01_init();

  Serialprintln(F("RF01 init "));

  //Init interrupt
  pinMode(RF_IN, INPUT);
  TCCR1A = 0x00;
  TCCR1B = 0x09; // CTC, Prescaler = 1
  TCCR1C = 0x00;
  OCR1A = COUNTER_RATE; 
  TIMSK1 = 0x02;

  //initiate temp reading
  ds.reset();
  ds.select(temp_addr);
  ds.write(0x44, 0);

  // start the Ethernet connection:
  if (Ethernet.begin(mac) == 0) {
    Serialprint(F("Failed to configure Ethernet using DHCP\n"));
    delay(5*1000*1000); //Wait 5 minutes
    software_Reboot(); 
  }
  delay(1000);
  Serialprintln(F("DHCP ok! "));

// start the Ethernet connection and the server:
  server.begin();
  Serialprint(F("server is at "));
  Serialprintln(Ethernet.localIP());    

  //Initialize end sensor pin (PULL-UP)
  pinMode(9, INPUT_PULLUP);


  //Initialize motor and make sure it is still
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(255);
  myMotor->run(RELEASE);


  // Setup variables
  sendtime=millis();
  
  // enable interrupts
  sei();

  ApplicationMonitor.EnableWatchdog(Watchdog::CApplicationMonitor::Timeout_8s);
  
}

void loop() {
  //Process any message from powermeters
  //Send values to emoncms
  ApplicationMonitor.SetData(1);
  process_power();
 
  //Process any message from weather sensors
  //Send values to emoncms
  ApplicationMonitor.SetData(2);
  process_wh2();
  
  //Look for any new value for desired temperature
  //Store it in NVRAM
  ApplicationMonitor.SetData(3);
  process_client();
  
  //Compare to desired value and if needed drive motor
  //If end state is reached, store the direction it was driven
  ApplicationMonitor.SetData(4);
  adjust_shunt();


  //Reset WD
  ApplicationMonitor.IAmAlive();

  //reset board every 24hrs
  if( millis() > 24*60*60*1000UL )
  {
    software_Reboot();
  }
}


void adjust_shunt(void)
{  
  static unsigned long next_adjust = ADJUSTMENT_PERIOD;
  static boolean normal_ops = true;
  byte  i;
  
  if( (long) (millis()-next_adjust) >= 0   || next_adjust == 0)
  {
    next_adjust = next_adjust + ADJUSTMENT_PERIOD;  
  
    readtemp();
    //write to Emon
    writeTempToEmon((int) 10*bor_temp, (int) 10*celsius);  //convert to int

    //what is the temp diff
    float diff = bor_temp-celsius;
    Serialprint("Temperature diff: ");
    Serialprintln(diff, 2);
    
    //is it within tolerance then return without action
    if(fabs(diff) < 2.0)
      return;

    //if for any reason we are not in an end-point. Lets clear the flag
    
    if( (end_point != SHUNT_NORMAL) && (END_IN_PIN & B00000010))
    {
        end_point = SHUNT_NORMAL;
        EEPROM.write(END_POINT_ADR, end_point);
    }
    
    //is end-point reached?
    if(end_point != SHUNT_NORMAL)
    {
      if( (end_point == SHUNT_OPEN_END && diff > 0) || (end_point == SHUNT_CLOSE_END && diff < 0)  )
      {
        Serialprintln(F("At endpoint!"));
        return;
      } 
      else
      {
        if(diff > 0)
        {
          //open shunt
          myMotor->run(FORWARD);
        }
        else
        {
          myMotor->run(BACKWARD);
          //close shunt
        }
        //drive the shunt until we are clear of the endpoint
        while(!(END_IN_PIN & B00000010))
        { 
        }
        delay(200);  //run for a while longer to make sure we avoid hysteresis
        //Clear the end_point!
        end_point = SHUNT_NORMAL;
        EEPROM.write(END_POINT_ADR, end_point);
      }
    }
    if(diff > 0)
    {
      //open shunt
      myMotor->run(FORWARD);
    }
    else
    {
      myMotor->run(BACKWARD);
      //close shunt
    }
    for(i=0;i<5;i++)
     {
      delay(50);
      if(!(END_IN_PIN & B00000010))
      {
        end_point = (diff > 0) ? SHUNT_OPEN_END : SHUNT_CLOSE_END;
        EEPROM.write(END_POINT_ADR, end_point);
        Serialprintln(F("End point reached!"));
        end_point_count++;
        break;  //break will break the nearest loop, in this case the for loop
      }
    }    
    movement_count++;
    myMotor->run(RELEASE);
    //drive
  }  
}

void readtemp(void)
{
  byte present = 0;
  byte i;
  byte data[12];
  byte addr[8];
  
    
  present = ds.reset();
  ds.select(temp_addr);    
  ds.write(0xBE);         // Read Scratchpad

  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time

  celsius = (float)raw / 16.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");

  //initiate temp reading
  ds.reset();
  ds.select(temp_addr);
  ds.write(0x44, 0);
}


int rfm_error = 0;

void process_client(void)
{
    /*
  GET /settemp/30 HTTP/1.1
  Host: courel.asuscomm.com
  */

  #define BUFFER_SIZE  20    //must fit entrie command plus args
  #define TEMP_COMMAND  "GET \/settemp\/"  //remember to escape special chars!
  static unsigned long next_poll = 0;
  char recv_buf[BUFFER_SIZE];
  
  if( (long) (millis()-next_poll) >= 0   || next_poll == 0)
  {
    next_poll = next_poll + 5000;  //wait 5 seconds
    
    EthernetClient client = server.available();
    if (client) {
      Serialprintln(F("new client"));
      // an http request ends with a blank line
      boolean currentLineIsBlank = true;
      byte pos = 0;
      
      while (client.connected()) {
        if (client.available()) {
          char c = client.read();
          if( (pos+1) < BUFFER_SIZE)    //need room for null term
              recv_buf[pos++]=c;
          Serial.write(c);
          // if you've gotten to the end of the line (received a newline
          // character) and the line is blank, the http request has ended,
          // so you can send a reply
 
          if (c == '\n' && currentLineIsBlank) {
            client.print(F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n\r\n<!DOCTYPE HTML>\r\n<html>\r\n"));
            /*
            // send a standard http response header
            client.println("HTTP/1.1 200 OK");
            client.println("Content-Type: text/html");
            client.println("Connection: close");  // the connection will be closed after completion of the response
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            */
            
            // output the number of errors per timeunit
            client.print("Uptime (mins): ");
            client.print( millis()/(60*1000UL));
            client.println("<br />");
            client.print("MTBF: ");
            client.print( millis()/1000/rfm_error);
            client.println("<br />");
            client.print(F("Current Temperature: "));
            client.print( celsius, 1);
            client.println("<br />");
            client.print(F("Set Temperature: "));
            client.print( bor_temp, 1);
            client.println("<br />");
            client.print(F("Shunt state: "));
            switch(end_point)
            {
            case SHUNT_NORMAL:
              client.print("NORMAL");
              break;              
            case SHUNT_OPEN_END:
              client.print("FULLY OPEN");
              break;              
            case SHUNT_CLOSE_END:
              client.print("CLOSED");
              break;              
            } 
            client.println("<br />");
            client.print(F("Movement/End point count: "));
            client.print( movement_count);
            client.print( "/");
            client.print( end_point_count);
            client.println("<br />");

            client.println("</html>");
            break;
          }
          if (c == '\n') {
            // you're starting a new line
            currentLineIsBlank = true;
            recv_buf[pos]='\0';
            if(strncmp(recv_buf,TEMP_COMMAND,sizeof(TEMP_COMMAND)-1) == 0 )
            {
              //correct command received
              Serialprintln(F("Set Temperature command received!"));
              //get temp integer
              int settemp=atoi(&recv_buf[sizeof(TEMP_COMMAND)-1]);
              if(settemp > 1 && settemp < 70)
              {
                Serialprint("Set temperature to: ");
                Serialprintln(settemp);
                bor_temp = (float) settemp;                
                EEPROM.write(SETTEMP_ADR, (byte) settemp);
              }
              else
                Serialprintln("Set temperature outside limits 1-70 degress..");
            }
            pos = 0;
          } else if (c != '\r') {
            // you've gotten a character on the current line
            currentLineIsBlank = false;
          }
        }
      } //while
      // give the web browser time to receive the data
      delay(1);
      // close the connection:
      client.stop();
      Serial.println("client disconnected");
      Ethernet.maintain();
    }
  } 
}


Sensorstruct sensors[7] = 
{
  { 0xFB8F,0, 15000
  },
  { 0x04E6,1, 0
  },
  { 0x04DA,2, 0
  },
  { 0x04F3,3, 0
  },
  { 0x0469,4, 0
  },
  { 0xF296,10, 3680
  },
  { 0xFE85,11, 3680
}  
};

/*
unsigned int getSensorMax(unsigned int ADR)
{
  return sensors[id].max.value;
}
*/

Sensorstruct *getSensorData(unsigned int ADR)
{
  for(int i=0;i<sizeof(sensors)/sizeof(sensors[0]);i++)
  {
    if(sensors[i].adr == ADR)
    {
      return &sensors[i];
    }
  }
  Serialprint(F("Sensor id not defined\n"));
  Serialprintln(ADR, HEX);
  
  return 0;
}

void writeTempToEmon(int a, int b)
{
  if(bytecnt>0)
  {
    sprintf(&httpreq[bytecnt++],",");
  }  
  bytecnt+=sprintf(&httpreq[bytecnt],"[%u,%u,%d,%d]",(unsigned int) (millis()-sendtime)/1000, 13, a, b);

  writeToEmon();
}

void writePowerToEmon(unsigned char id, int a)
{
  if(bytecnt>0)
  {
    sprintf(&httpreq[bytecnt++],",");
  }  
  bytecnt+=sprintf(&httpreq[bytecnt],"[%u,%u,%u]",(unsigned int) (millis()-sendtime)/1000, id, a);

  writeToEmon();
}

void writeWH2ToEmon(unsigned char id, int a, byte hum)
{
  if(bytecnt>0)
  {
    sprintf(&httpreq[bytecnt++],",");
  }  
  bytecnt+=sprintf(&httpreq[bytecnt],"[%u,%u,%u,%u]",(unsigned int) (millis()-sendtime)/1000, id, a, hum);

  writeToEmon();
}


void writeToEmon(void)
{
  static int  conn_fails=0;
  
  ApplicationMonitor.SetData(100);
  if(bytecnt<100)
    return;

  if(bytecnt>sizeof(httpreq))
  {
     Serialprint(F("Reset - Httpreq buffer overflow"));
     software_Reboot();      
  }  
  
  client.flush();
  ApplicationMonitor.SetData(101);
  client.stop();
  ApplicationMonitor.SetData(102);
  
  if (client.connect(emon_server, 80)) 
  {
      ApplicationMonitor.SetData(103);
      Serialprint(F("connected\n"));
      client.print(F("GET /input/bulk.json?data=["));  // make sure there is a [space] between GET and /input
      client.print(httpreq);
      client.print(F("]&apikey="));
      client.print(F("355b01d73c69ac734602b31cdbb99405"));         //assuming MYAPIKEY is a char or string
      client.println(F(" HTTP/1.1"));   //make sure there is a [space] BEFORE the HTTP
      client.println(F("Host: emoncms.org"));
      client.print(F("User-Agent: Arduino-ethernet"));
      client.println(F("Connection: close"));     //    Although not technically necessary, I found this helpful
      client.println();    
      sendtime = millis();
      conn_fails = 0;
      bytecnt = 0;
  }
   else
    {
      ApplicationMonitor.SetData(104);
      Serialprint(F("Failed to connect!\n"));
      client.stop();
      conn_fails++;
      if(conn_fails > 10)
      {
         ApplicationMonitor.SetData(104);
         Serialprint(F("Reset - Connfailure 10 times.."));
         software_Reboot();
      }        
    }
    Serialprint(httpreq);
}

void process_wh2(void)
{
    if (wh2_flags) {
    if (wh2_accept()) {
      // calculate the CRC
      wh2_calculate_crc();

      Serialprint("Sensor ID: 0x");
      Serialprint(wh2_sensor_id(), HEX);
      Serialprint(" | ");
      Serialprint(wh2_humidity(), DEC);
      Serialprint("% | ");
      Serialprint(wh2_temperature(), DEC);
      Serialprint(" | ");
      Serialprint((wh2_valid() ? "OK\n" : "BAD\n"));      
      
    if(wh2_valid())
    {
      Sensorstruct *senptr = getSensorData(wh2_sensor_id());
      if (senptr)
      {
        writeWH2ToEmon(senptr->emon_id, wh2_temperature(), wh2_humidity());
      } 
   }
  }
  
  wh2_flags = 0x00; 
 }
}


void process_power ( void )
{
  if(cc_rxdata() == MSG_PENDING)
  {
    Sensorstruct *senptr = getSensorData((unsigned int) (msg_buf[0]  << 8 | msg_buf[1])); 
    if (senptr)
    {
      int a;
      a=msg_buf[2]<<8;
      a|=msg_buf[3];
      a-=0x7FFF;
      a=abs(a);

      if(a > senptr->max_value)
      {
        rfm_error++;
        Serialprint(F("**** High power reading... Skipping\n"));      
      }
      else
      {
        Serialprint(a);
        Serialprint(F(" Watts\n"));      
        writePowerToEmon(senptr->emon_id, a);
      }
      
    }
    for(int i=0;i<MSG_LEN;i++)
    {
         Serialprint(" ");
         Serialprint(msg_buf[i], HEX);
    }
   Serial.write('\n');
   msg_state=MSG_WAIT;

  }   
}

#ifndef cbi
#define cbi(sfr, bit)     (_SFR_BYTE(sfr) &= ~_BV(bit)) 
#endif
#ifndef sbi
#define sbi(sfr, bit)     (_SFR_BYTE(sfr) |= _BV(bit))  
#endif

void rf01_trans(unsigned short wert)
{	unsigned char i;

	cbi(RF_PORT, CS_RFM);
	for (i=0; i<16; i++)
	{	if (wert&32768)
			sbi(RF_PORT, SDI_RFM);
		else
			cbi(RF_PORT, SDI_RFM);
		sbi(RF_PORT, SCK_RFM);
		wert<<=1;
		_delay_us(0.2);
		cbi(RF_PORT, SCK_RFM);
	}
	sbi(RF_PORT, CS_RFM);
}

void rf01_init(void)
{	unsigned char i;

	RF_PORT=(1<<CS_RFM);
	RF_DDR=(1<<SDI_RFM)|(1<<SCK_RFM)|(1<<CS_RFM);

	for (i=0; i<11; i++)
		_delay_ms(10);			// wait until POR done

  rf01_trans(0x892c); //433, Xtal osc, 9.5pf, 67kHz, disable CLK

// Low Duty Cycle Command OFF
  rf01_trans(0xea00); //Wake up timer command
  rf01_trans(0xcc00); //Low duty cycle command OFF


  rf01_trans(0xc69f); //Keep offset, +15/-16, AFC on
  rf01_trans(0xc46a); //Fast clk recovery, digital filter, DQD=2
  rf01_trans(0xc88a); //3918.5 bps
  rf01_trans(0xc080); //Clk recovery lock output, 0 LNA gain, -103 RSSI, disable receiver
  rf01_trans(0xce88); //FIFO = 8, Disable FIFO fill, Disable FIFO function
  rf01_trans(0xce8b); //FIFO = 8, Enable FIFO fill, Enable FIFO function
  rf01_trans(0xc200); //1 MHz, battery threshold=0
  rf01_trans(0xa618); //433.9 MHz
  rf01_trans(0xce88); //Set FIFO mode
  rf01_trans(0xce8b); //FIFO = 8, Enable FIFO fill, Enable FIFO function  
  rf01_trans(0xc081); //Clk recovery lock output, 0 LNA gain, -103 RSSI, enable receiver
}

unsigned char ManchesterDecode(unsigned char in)
{
switch(in)
{
case 0b10101010:
  return 0;
case 0b10101001:
  return 1;
case 0b10100110:
  return 2;
case 0b10100101:
  return 3;
case 0b10011010:
  return 4;
case 0b10011001:
  return 5;
case 0b10010110:
  return 6;
case 0b10010101:
  return 7;
case 0b01101010:
  return 8;
case 0b01101001:
  return 9;
case 0b01100110:
  return 10;
case 0b01100101:
  return 11;
case 0b01011010:
  return 12;
case 0b01011001:
  return 13;
case 0b01010110:
  return 14;
case 0b01010101:
  return 15;
default:
  return 255;
}
}

//This really has to be interrupt driven if we are doing a lot of other stuff...

unsigned int cc_rxdata()
{
unsigned char c,j,i;
unsigned int stat;
static unsigned char msgbyte = 0;
static unsigned int bytecnt = 0;
static boolean msg_of = false;

      cbi(RF_PORT, SDI_RFM);
      cbi(RF_PORT, CS_RFM);
      if(RF_PIN&(1<<SDO_RFM))  //Has 8 bits been received?
      {
        if(msg_state == MSG_PENDING)    //MSG overflow?
          msg_of = true;
        else
          msg_state = MSG_READ;
        stat=0;
        for (j=0; j<16; j++)	// read and discard status register
        {	
                stat<<=1;
                sbi(RF_PORT, SCK_RFM);
//	        asm("nop");
                if(RF_PIN&(1<<SDO_RFM))
                  stat|=1;
		cbi(RF_PORT, SCK_RFM);
	}
	c=0;
	for (j=0; j<8; j++)
	{	c<<=1;
		if (RF_PIN&(1<<SDO_RFM))
		  c|=1;
		sbi(RF_PORT, SCK_RFM);
		_delay_us(0.2);
		cbi(RF_PORT, SCK_RFM);
	}
	sbi(RF_PORT, CS_RFM);
        msgbyte |= ManchesterDecode(c);  
        bytecnt++;
        if( !(bytecnt & 0x01) )  //read entire byte?
        {  //Yes, put it in buffer
          msg_buf[(bytecnt >> 1)-1] = msgbyte;
//          Serialprint(" ");
//          Serialprint(msgbyte, HEX);
          msgbyte = 0;
        }
        else
        {  //shift nibble and wait for next
          msgbyte = msgbyte << 4;
        }
        if(stat & 0x4000)  //FIFO overflow?
        {
          Serialprintln("FIFO buffer overflow");
	  rf01_trans(0xCE89);			// restart FIFO (wait for start-word)
	  rf01_trans(0xCE8b);			// enable FIFO
          bytecnt = 0;
          msg_state = MSG_WAIT;   //
        }

        if(bytecnt == MSG_LEN*2)    //did we get all 8 bytes?
        {
//          Serial.write('\n');
	  rf01_trans(0xCE89);			// restart FIFO (wait for start-word)
	  rf01_trans(0xCE8b);			// enable FIFO
          bytecnt = 0;
          msg_state = MSG_PENDING;   //flag we have a new message
        }
    }
    sbi(RF_PORT, CS_RFM);
    return msg_state;
}


// processes new pulse
boolean wh2_accept()
{
  static byte packet_no, bit_no, history;

  // reset if in initial wh2_packet_state
  if(wh2_packet_state == 0) {
     // should history be 0, does it matter?
    history = 0xFF;
    wh2_packet_state = 1;
    // enable wh2_timeout
    wh2_timeout = 1;
  } // fall thru to wh2_packet_state one
 
  // acquire preamble
  if (wh2_packet_state == 1) {
     // shift history right and store new value
     history <<= 1;
     // store a 1 if required (right shift along will store a 0)
     if (wh2_flags & LOGIC_HI) {
       history |= 0x01;
     }
     // check if we have a valid start of frame
     // xxxxx110
     if ((history & B00000111) == B00000110) {
       // need to clear packet, and counters
       packet_no = 0;
       // start at 1 becuase only need to acquire 7 bits for first packet byte.
       bit_no = 1;
       wh2_packet[0] = wh2_packet[1] = wh2_packet[2] = wh2_packet[3] = wh2_packet[4] = 0;
       // we've acquired the preamble
       wh2_packet_state = 2;
    }
    return false;
  }
  // acquire packet
  if (wh2_packet_state == 2) {

    wh2_packet[packet_no] <<= 1;
    if (wh2_flags & LOGIC_HI) {
      wh2_packet[packet_no] |= 0x01;
    }

    bit_no ++;
    if(bit_no > 7) {
      bit_no = 0;
      packet_no ++;
    }

    if (packet_no > 4) {
      // start the sampling process from scratch
      wh2_packet_state = 0;
      // clear wh2_timeout
      wh2_timeout = 0;
      return true;
    }
  }
  return false;
}


void wh2_calculate_crc()
{
  wh2_calculated_crc = crc8(wh2_packet, 4);
}

bool wh2_valid()
{
  return (wh2_calculated_crc == wh2_packet[4]);
}

int wh2_sensor_id()
{
  return (wh2_packet[0] << 4) + (wh2_packet[1] >> 4);
}

byte wh2_humidity()
{
  return wh2_packet[3];
}

/* Temperature in deci-degrees. e.g. 251 = 25.1 */
int wh2_temperature()
{
  int temperature;
  temperature = ((wh2_packet[1] & B00000111) << 8) + wh2_packet[2];
  // make negative
  if (wh2_packet[1] & B00001000) {
    temperature = -temperature;
  }
  return temperature;
}

uint8_t crc8( uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  // Indicated changes are from reference CRC-8 function in OneWire library
  while (len--) {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i; i--) {
      uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
      crc <<= 1; // changed from right shift
      if (mix) crc ^= 0x31;// changed from 0x8C;
      inbyte <<= 1; // changed from right shift
    }
  }
  return crc;
}

