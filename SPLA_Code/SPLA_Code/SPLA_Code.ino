#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> 
#include <driver/i2s.h>
#include <arduinoFFT.h>

// Connections to I2S
#define I2S_WS 25
#define I2S_SD 17
#define I2S_SDO 4
#define I2S_SCK 16

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// Fourier Transform
#define FFT_SIZE 1024
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, FFT_SIZE, 44100.0);
// Read more about the library here: https://github.com/kosme/arduinoFFT/wiki/api


// Define input buffer length
#define bufferLen 1024
int16_t sBuffer[bufferLen];

WM8960 codec;

int db_SPL;








void setup(){
  Serial.begin(115200);
  Wire.begin();
  if (codec.begin() == false) //Begin communication over I2C
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
        float voltage = (sBuffer[i] / 32768.0) * 3.3;
        float pascal = voltage / 0.01258925412;
        vReal[i] = pascal * 0.5 * (1 - cos(2 * PI * i / (FFT_SIZE - 1))); // Hann window
        vImag[i] = 0;
      }

      FFT.compute(vReal, vImag, FFT_SIZE, FFT_FORWARD);
      FFT.complexToMagnitude(vReal, vImag, FFT_SIZE);

      computeThirdOctaveBands(vReal);
    }
  }
}


void codec_setup()
{
  // General setup needed
  codec.enableVREF();
  codec.enableVMID();

  // Setup signal flow to the ADC
  codec.enableLMIC();
  codec.enableRMIC();

  // Connect from INPUT1 to "n" (aka inverting) inputs of PGAs.
  codec.connectLMN1();
  codec.connectRMN1();

  // Disable mutes on PGA inputs (aka INTPUT1)
  codec.disableLINMUTE();
  codec.disableRINMUTE();

  // Set pga volumes
  codec.setLINVOLDB(0.00); // Valid options are -17.25dB to +30dB (0.75dB steps)
  codec.setRINVOLDB(0.00); // Valid options are -17.25dB to +30dB (0.75dB steps)

  // Set input boosts to get inputs 1 to the boost mixers
  codec.setLMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);
  codec.setRMICBOOST(WM8960_MIC_BOOST_GAIN_0DB);

  // Connect from MIC inputs (aka pga output) to boost mixers
  codec.connectLMIC2B();
  codec.connectRMIC2B();

  // Enable boost mixers
  codec.enableAINL();
  codec.enableAINR();

  // Connect LB2LO (booster to output mixer (analog bypass)
  codec.enableLB2LO();
  codec.enableRB2RO();

  // Disconnect from DAC outputs to output mixer
  codec.disableLD2LO();
  codec.disableRD2RO();

  // Set gainstage between booster mixer and output mixer
  codec.setLB2LOVOL(WM8960_OUTPUT_MIXER_GAIN_0DB); 
  codec.setRB2ROVOL(WM8960_OUTPUT_MIXER_GAIN_0DB); 

  // Enable output mixers
  codec.enableLOMIX();
  codec.enableROMIX();

  // CLOCK STUFF, These settings will get you 44.1KHz sample rate, and class-d 
  // freq at 705.6kHz
  codec.enablePLL(); // Needed for class-d amp clock
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h
  //codec.setADCDIV(0); // Default is 000 (what we need for 44.1KHz)
  //codec.setDACDIV(0); // Default is 000 (what we need for 44.1KHz)
  codec.setWL(WM8960_WL_16BIT);

  codec.enablePeripheralMode();
  //codec.enableMasterMode();
  //codec.setALRCGPIO(); // Note, should not be changed while ADC is enabled.

  // Enable ADCs, and disable DACs
  codec.enableAdcLeft();
  codec.enableAdcRight();
  codec.disableDacLeft();
  codec.disableDacRight();
  codec.disableDacMute();

  //codec.enableLoopBack(); // Loopback sends ADC data directly into DAC
  codec.disableLoopBack();

  // Default is "soft mute" on, so we must disable mute to make channels active
  codec.enableDacMute(); 

  codec.enableHeadphones();
  codec.enableOUT3MIX(); // Provides VMID as buffer for headphone ground

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

void computeThirdOctaveBands(float* spectrum) {
  const float sampleRate = 44100.0;
  const float binWidth = sampleRate / FFT_SIZE;

  struct Band {
    float center;
    float low;
    float high;
  };

  Band bands[] = {
    {31.5, 28.0, 35.5}, {40.0, 35.5, 44.5}, {50.0, 44.5, 56.0},
    {63.0, 56.0, 71.0}, {80.0, 71.0, 89.0}, {100.0, 89.0, 112.0},
    {125.0, 112.0, 140.0}, {160.0, 140.0, 180.0}, {200.0, 180.0, 225.0},
    {250.0, 225.0, 280.0}, {315.0, 280.0, 355.0}, {400.0, 355.0, 450.0},
    {500.0, 450.0, 560.0}, {630.0, 560.0, 710.0}, {800.0, 710.0, 900.0},
    {1000.0, 900.0, 1120.0}, {1250.0, 1120.0, 1400.0}, {1600.0, 1400.0, 1800.0},
    {2000.0, 1800.0, 2250.0}, {2500.0, 2250.0, 2800.0}, {3150.0, 2800.0, 3550.0},
    {4000.0, 3550.0, 4500.0}, {5000.0, 4500.0, 5600.0}, {6300.0, 5600.0, 7100.0},
    {8000.0, 7100.0, 9000.0}, {10000.0, 9000.0, 11200.0}, {12500.0, 11200.0, 14000.0},
    {16000.0, 14000.0, 18000.0}
  };

  int numBands = sizeof(bands) / sizeof(Band);

  for (int b = 0; b < numBands; b++) {
    float power_sum = 0;
    int binCount = 0;

    for (int k = 0; k < FFT_SIZE / 2; k++) {
      float freq = k * binWidth;
      if (freq >= bands[b].low && freq <= bands[b].high) {
        power_sum += spectrum[k] * spectrum[k];
        binCount++;
      }
    }

    if (binCount > 0) {
      float rms = sqrt(power_sum / binCount);
      float db_spl = 20 * log10(rms / 20e-6);
      Serial.printf("Band %.0f Hz: %.2f dB SPL\n", bands[b].center, db_spl);
    }
  }

  Serial.println("--------");
}


