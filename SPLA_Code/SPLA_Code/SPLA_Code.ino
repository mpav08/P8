#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> 
#include <driver/i2s.h>
#include <arduinoFFT.h>

// Connections to I2S.
#define I2S_WS 25
#define I2S_SD 17
#define I2S_SDO 4
#define I2S_SCK 16

// Use I2S Processor 0.
#define I2S_PORT I2S_NUM_0

// Fourier Transform
#define FFT_SIZE 1024
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FFT_SIZE, 44100.0);
// Read more about the library here: https://github.com/kosme/arduinoFFT/wiki/api


// Define input buffer length. Warning i2s can only handle a bufferlength of up to 1024.
#define bufferLen 1024
int16_t sBuffer[bufferLen];
WM8960 codec; // creates an instance of the codec.

void setup(){
  Serial.begin(115200);
  Wire.begin();
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

void loop() {
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, sizeof(sBuffer), &bytesIn, portMAX_DELAY);
  if (result == ESP_OK) {
    int samples_read = bytesIn / sizeof(int16_t);

    if (samples_read == FFT_SIZE) {

      for (int i = 0; i < FFT_SIZE; i++) {
        float voltage = (sBuffer[i] / 32768.0) * 3.3; // Converts the i2s signal saved to the sBuffer to its equivalent voltage.
        float pascal = voltage / 0.01258925412; // Converts the measured voltage to its equivalent pressure in pascal. this is done based on the microphone sensitivty.

        // Applies the Hann window
        vReal[i] = pascal * 0.5 * (1 - cos(2 * PI * i / (FFT_SIZE - 1))); 
        vImag[i] = 0;
      }

      FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);
      computeThirdOctaveBandsAWeightedSPL(vReal, FFT_SIZE, 44100.0);
    }
  }
}


// The following three functions are used to setup the codec and i2s communication between the esp32 and the codec.
void codec_setup()
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
  const float refPressure = 0.00002; // Reference pressure in Pa for 0 dB SPL

  const int numBands = 30;
  float centerFrequencies[numBands] = {
    25, 31.5, 40, 50, 63, 80, 100, 125, 160,
    200, 250, 315, 400, 500, 630, 800, 1000,
    1250, 1600, 2000, 2500, 3150, 4000, 5000,
    6300, 8000, 10000, 12500, 16000, 20000
  };

  float aWeighting[numBands] = {
    -44.7, -39.4, -34.6, -30.2, -26.2, -22.5, -19.1, -16.1, -13.4,
    -10.9, -8.6, -6.6, -4.8, -3.2, -1.9, -0.8, 0.0,
    0.6, 1.0, 1.2, 1.3, 1.2, 1.0, 0.5,
    -0.1, -1.1, -2.5, -4.3, -6.6, -9.3
  };

  float bandEnergy[numBands] = {0};
  float totalLinearAWeightedPower = 0;

  float binWidth = samplingFrequency / fftSize;

  // Accumulate FFT magnitudes into third-octave bands
  for (int i = 1; i < fftSize / 2; i++) {
    float freq = i * binWidth;

    for (int band = 0; band < numBands; band++) {
      float fCenter = centerFrequencies[band];
      float fLower = fCenter / pow(2.0, 1.0 / 6.0);
      float fUpper = fCenter * pow(2.0, 1.0 / 6.0);

      if (freq >= fLower && freq < fUpper) {
        float pressure = vReal[i] / fftSize;
        float power = pressure * pressure;
        bandEnergy[band] += power;
        break;
      }
    }
  }

  Serial.println("Third-Octave Band SPL (A-weighted):");
  for (int band = 0; band < numBands; band++) {
    if (bandEnergy[band] > 0) {
      float SPL = 10.0 * log10(bandEnergy[band] / (refPressure * refPressure));
      float SPLA = SPL + aWeighting[band];

      Serial.print("Band ");
      Serial.print(centerFrequencies[band], 1);
      Serial.print(" Hz: ");
      Serial.print(SPLA, 1);
      Serial.println(" dB SPLA");

      totalLinearAWeightedPower += pow(10, SPLA / 10.0);
    }
  }

  float totalSPLA = 10.0 * log10(totalLinearAWeightedPower);
  Serial.print("Total SPLA: ");
  Serial.print(totalSPLA, 1);
  Serial.println(" dB SPLA");
}


