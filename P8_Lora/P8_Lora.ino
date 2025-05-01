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
#include <SparkFun_WM8960_Arduino_Library.h> 
#include <driver/i2s.h>
#include <arduinoFFT.h>

//Define I2C pins from ESP-32
const int SCLpin = 42;
const int SDApin = 41;

//Define DHT pin and type
#define DHTPIN 46
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define I2S_WS 5
#define I2S_SD 4
#define I2S_SDO 3
#define I2S_SCK 2

// Use I2S Processor 0.
#define I2S_PORT I2S_NUM_0

#define FFT_SIZE 8192 //8192 seems to be the limit. Otherwise it causes DRAM issues. Remember it needs to be a multiple of 2. It may be possible to increase this using PSRAM if the esp32 has it.Otherwise more efficient memmory usage might be needed.
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FFT_SIZE, 44100.0);
int16_t fftBuffer[FFT_SIZE];              // Accumulation buffer for FFT
int fftBufferIndex = 0;                   // Tracks number of accumulated samples
// Read more about the library here: https://github.com/kosme/arduinoFFT/wiki/api


// Define input buffer length. Warning i2s can only handle a bufferlength of up to 1024.
#define bufferLen 1024
int16_t sBuffer[bufferLen];
WM8960 codec; // creates an instance of the codec.

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

    //Begin Wire for I2C comms
    Wire.begin(SDApin, SCLpin);

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


    //Checking if the sensor works. Neccessary to run 
    if (!bmp.begin()) {
      Serial.println("Could not find BMP180 sensor!");
    while (1);
    }
    if (codec.begin() == false) //Begin communication over I2C.
    {
      Serial.println("The device did not respond. Please check wiring.");
      while (1); // Freeze
    }
    Serial.println("Device is connected properly.");
    codec_setup();
    // Set up I2S
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT); 
   }



void loop()
{
  //Read from sensors
  pressure = bmp.readPressure()/1000;
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  size_t bytesIn = 0; //Stores the number of bytes read from the I2S interface
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, sizeof(sBuffer), &bytesIn, portMAX_DELAY); // Reads the data into the sBuffer

  //Checks if the read operation was successful.
  if (result == ESP_OK) {
    int samples_read = bytesIn / sizeof(int16_t); // Converts the number of bytes read into the number of 16-bit audio samples

    // Copy samples from the sBuffer into the larger fftBuffer
    for (int i = 0; i < samples_read && fftBufferIndex < FFT_SIZE; i++) {
      fftBuffer[fftBufferIndex++] = sBuffer[i];
    }

    // Once enough samples are collected, perform FFT
    if (fftBufferIndex == FFT_SIZE) {
      for (int i = 0; i < FFT_SIZE; i++) {
        float voltage = (fftBuffer[i] / 32768.0) * 3.3; // Converts the buffer values to their equivalent voltage
        float pascal = voltage / 0.01258925412; // Converts the Voltage to pascal, using the microphone sensitivity
        vReal[i] = pascal * 0.5 * (1 - cos(2 * PI * i / (FFT_SIZE - 1))); // Applies the Hann window and saves it to the vReal buffer.
        vImag[i] = 0;
      }

      // Apply the FFT function, calculate the magnitude.
      FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);
      //Creates third octave bands and calculates the A-weighted SPL values for them.
      computeThirdOctaveBandsAWeightedSPL(vReal, FFT_SIZE, 44100.0);

      fftBufferIndex = 0; // Reset buffer index after processing
    }
  }
  
  
  //Check if LoRa is ready to transmit
	if(lora_idle == true)
	{
    delay(250);
		
    //start a package
		sprintf(txpacket, "%.1f,%.f,%.1f", temperature, humidity, pressure);  
   
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
	//Serial.println("TX done......");
	lora_idle = true;
}

// Callback when transmission times out
void OnTxTimeout( void )
{
    Radio.Sleep( );
    Serial.println("TX Timeout......");
    lora_idle = true;
}

// The following three functions are used to setup the codec and i2s communication between the esp32 and the codec.
void codec_setup() // This loop has been simplified. Keep in mind that it is set up to only take Lin1 and send out mono audio
{
  // Basic codec setup
  codec.enableVREF();
  codec.enableVMID();

  // Enable left mic path
  codec.enableLMIC();

  // Connect Lin1 to the inverting input of the left PGA
  codec.connectLMN1();

  // Unmute PGA input
  codec.disableLINMUTE();

  // Set PGA volume
  codec.setLINVOLDB(0.00);

  // No boost on input
  codec.setLMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);

  // Connect PGA output to boost mixer and enable it
  codec.connectLMIC2B();
  codec.enableAINL();

  // Analog bypass from boost mixer to output mixer
  codec.enableLB2LO();
  codec.setLB2LOVOL(WM8960_OUTPUT_MIXER_GAIN_0DB);
  codec.enableLOMIX();

  // Clock setup for 44.1kHz sampling
  codec.enablePLL();
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26);
  codec.setWL(WM8960_WL_16BIT);

  // Peripheral (slave) mode
  codec.enablePeripheralMode();

  // Enable ADC (left only)
  codec.enableAdcLeft();
  codec.disableDacLeft(); // Not needed
  codec.disableLoopBack();
  codec.enableDacMute();  // Default state

  // Headphone output setup
  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Buffer ground reference
  codec.setHeadphoneVolumeDB(0.00);
}
// Remember we have set I2S_CHANNEL_FMT_ONLY_LEFT. Meaning we have mono audio from Lin1
void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_driver_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
    .mclk_multiple = i2s_mclk_multiple_t(I2S_MCLK_MULTIPLE_512),
    .bits_per_chan = i2s_bits_per_chan_t(I2S_BITS_PER_CHAN_DEFAULT)
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}
void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .mck_io_num = I2S_PIN_NO_CHANGE,
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_SDO,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

/* Takes the vReal values from the complexToMagnitude() function and splits into frequency bands.
The SPL of each band is then calculated. Aftewards, A-weigting is applied to each band.
Lastly the Total A-weighted SPL for the whole frequency is calculated.
All these walues are printed to serial. */ 
void computeThirdOctaveBandsAWeightedSPL(float *vReal, int fftSize, float samplingFrequency) {
  const float refPressure = 0.00002; // Reference pressure in Pa for 0 dB SPL (20μPa)

  const int numBands = 30; // The number of frequency bands

  // The center frequencies of each band
  const float centerFrequencies[numBands] = {
    25, 31.5, 40, 50, 63, 80, 100, 125, 160,
    200, 250, 315, 400, 500, 630, 800, 1000,
    1250, 1600, 2000, 2500, 3150, 4000, 5000,
    6300, 8000, 10000, 12500, 16000, 20000
  };

// The A-weigting factor for each frequency band
  const float aWeighting[numBands] = {
    -44.7, -39.4, -34.6, -30.2, -26.2, -22.5, -19.1, -16.1, -13.4,
    -10.9, -8.6, -6.6, -4.8, -3.2, -1.9, -0.8, 0.0,
    0.6, 1.0, 1.2, 1.3, 1.2, 1.0, 0.5,
    -0.1, -1.1, -2.5, -4.3, -6.6, -9.3
  };

  //What do they
  float bandEnergy[numBands] = {0}; // used to accumulate the energy/power of each frequency band
  float totalLinearAWeightedPower = 0;

  // Frequency resolution per FFT bin
  float binWidth = samplingFrequency / fftSize;

  // Accumulate FFT magnitudes into third-octave bands (only the positive bins)
  for (int i = 1; i < fftSize / 2; i++) {
    float freq = i * binWidth;

      // The following loop calculates the frequency band for each center frequency (±1/6 octave)
    for (int band = 0; band < numBands; band++) {
      float fCenter = centerFrequencies[band];
      float fLower = fCenter / pow(2.0, 1.0 / 6.0);
      float fUpper = fCenter * pow(2.0, 1.0 / 6.0);

      // If the FFT bin frequency is inside a band, its power is added
      if (freq >= fLower && freq < fUpper) {
        // Converts FFT magnitude to pressure, computes power, and accumulates it
        float pressure = 2.0*vReal[i] / fftSize; // vReal is multiplied by 2, because we are dealing with a single sided spectrum
        float power = pressure * pressure;
        bandEnergy[band] += power;
        break;
      }
    }
  }

  // The following calculates the SPL and applies the A-weigthing
  Serial.println("Third-Octave Band SPL (A-weighted):");
  for (int band = 0; band < numBands; band++) {
    if (bandEnergy[band] > 0) {
      // Converts energy to SPL
      float SPL = 10.0 * log10(bandEnergy[band] / (refPressure * refPressure));
      // Converts SPL to A-weighted SPL
      float SPLA = SPL + aWeighting[band];

      Serial.print("Band ");
      Serial.print(centerFrequencies[band], 1);
      Serial.print(" Hz: ");
      Serial.print(SPLA, 1);
      Serial.println(" dB SPLA");
      // Collect the calculated A-weighted SPL and calculates the power for all frequency bands 
      totalLinearAWeightedPower += pow(10, SPLA / 10.0);
    }
  }

  // converts the total power to A-weighted SPL
  float totalSPLA = 10.0 * log10(totalLinearAWeightedPower);
  Serial.print("Total SPLA: ");
  Serial.print(totalSPLA, 1);
  Serial.println(" dB SPLA");
}