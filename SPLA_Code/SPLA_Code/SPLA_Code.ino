#include <Wire.h>
#include <SparkFun_WM8960_Arduino_Library.h> 
#include <driver/i2s.h>


// Connections to I2S
#define I2S_WS 25
#define I2S_SD 17
#define I2S_SDO 4
#define I2S_SCK 16

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

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

void loop(){
  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  if (result == ESP_OK)
  {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / sizeof(int16_t);
    if (samples_read > 0) {
      float pascal_rms = 0;
      for (int16_t i = 0; i < samples_read; ++i)
      {
        float voltage = (sBuffer[i] / 32768.0) * 3.3; //Converts the input buffer values to their equivalent measured voltage
        float pascal = voltage / 0.01258925412; // Applies the microphone sensitivity to the voltage, to convert voltage to pascal
        pascal_rms += pascal * pascal; // This squares the the pascal and adds it to the pascal_rms
      }
      pascal_rms = sqrt(pascal_rms / samples_read); // This takes the squared and summed pascal, and averages it while squaring it. use samples_read not bufferLen
      db_SPL = 20 * log10(pascal_rms / (20e-6));  // Converts pascal to SPL
     
      Serial.printf("dB:");
      Serial.println(db_SPL);
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