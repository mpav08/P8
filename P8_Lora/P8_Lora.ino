/* Heltec Automation send communication test example
 *
 * Function:
 * 1. Send data from a esp32 device over hardware 
 *  
 * Description:
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */


//Including Libraries
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "DHT.h"
#include <Adafruit_BMP085.h>

//Define I2C pins from ESP-32
const int SCLpin = 42;
const int SDApin = 41;

//Define DHT pin and type
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//Define Pressure sensor
Adafruit_BMP085 bmp;

//define variables taken from sensor
float temperature = 0;
float humidity = 0;
float pressure = 0;

//LoRa configurations
#define RF_FREQUENCY                                868100000 // Hz

#define TX_OUTPUT_POWER                             5        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             0         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000 // Receiving Timeout
#define BUFFER_SIZE                                 30 // Define the payload size here

// Buffers for transmitting and receiving
char txpacket[BUFFER_SIZE]; 
char rxpacket[BUFFER_SIZE]; 

double txNumber;  // Counter or placeholder for transmitted data (unused here)

bool lora_idle=true;  // Flag to track LoRa status

// / Radio event structure and callback declarations
static RadioEvents_t RadioEvents;
void OnTxDone( void );  // Callback when transmission is done
void OnTxTimeout( void ); // Callback for transmission timeout

void setup() {
    //Initialize serial for debugging
    Serial.begin(115200);

    // Initialize the Heltec Board
    Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);

    //begin DHT sensor
    dht.begin();
    
    // Initialize LoRa radio events
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    
    // Initialize the LoRa Radio
    Radio.Init( &RadioEvents );
    Radio.SetChannel( RF_FREQUENCY );
    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
    //Begin Wire for I2C comms
    Wire.begin(SDApin, SCLpin);

    //Checking if the sensor works. Neccessary to run 
    if (!bmp.begin()) {
      Serial.println("Could not find BMP180 sensor!");
    while (1);
    } 
   }



void loop()
{
  //Read from sensors
  pressure = bmp.readPressure();
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  
  //Check if LoRa is ready to transmit
	if(lora_idle == true)
	{
    delay(250);
		
    //start a package
		sprintf(txpacket, "%.1f,%.f,%.f", temperature, humidity, pressure);  
   
    //Show in Serial the package ready to be send
		Serial.printf("\r\nsending packet \"%s\" , length %d\r\n",txpacket, strlen(txpacket));

		//send the package out	
    Radio.Send( (uint8_t *)txpacket, strlen(txpacket) ); 
    lora_idle = false; // Mark the radio as busy
	}
  // Process radio IRQs (e.g., TxDone, TxTimeout)
  Radio.IrqProcess( );
}

// Callback when transmission is successful
void OnTxDone( void )
{
	Serial.println("TX done......");
	lora_idle = true;
}

// Callback when transmission times out
void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}