/*
AQSettings.h header contains struct for active board, sensor, and software operating parameters.
By Elliott Dicus (manylabs.org), updated by Ken McGary (ken@circuitsci.com), 
typically used with the echem328 smart gas sensor module and echemMon serial monitor and control software
Released under Creative Commons SA-BY 4.0 license
See circuitsci.com for more information.
*/

#ifndef AQ_SETTINGS_H
#define AQ_SETTINGS_H

#define SENSITIVITY_TABLE_LENGTH 9
#define ZERO_CURRENT_TABLE_LENGTH 9
#define DATA_FIELDS_ARRAY_SIZE 24

// Settings Struct
struct AQSettings {
  uint8_t verification_check;

  uint16_t dataSetId;
  uint16_t boardNumber; // board serial number

  uint32_t readInterval; // milliseconds between readings
  uint16_t startupDelay; // in seconds
  uint8_t triggerMode;
  int8_t triggerQualify; // debounce qualify time in milliseconds, polarity indicates trigger polatory, 0 = external disabled.

  float sensorSensitivity; // nA per ppm @ 20degC as measured at Alphasense factory
  float sensorOffset;      // nA @ 20degC as measured at Alphasense factory

  uint32_t sensorNumber; // sensor serial number
  uint8_t sensorType; // Sensor type, 0=C0-B4, 1=H2S-B4, 2=NO-B4, 3=NO2-B4, 4=O3-B4, 5=SO2-B4, add more to echem.h case switch

  uint8_t adcGain; // ADC gain setting options are 0-3 corresponding to 2/3x, 1x, 2x, and 4x

  uint8_t afeBias;
  uint8_t afeBiasPolarity;
  uint8_t afeLoadResistor;  // AFE Rload setting in ohms, options are 0, 1, 2, 3 corresponding to 10, 33, 50, 100 ohms
  uint8_t afeGainResistor;  // 0 = external, 1 = 350K, 2 = 120K, see datatsheet for the rest

  float adcOffset0; // mV @ 20degC, agcGain = 0 as measured at CircuitSci factory or on v2diff reading with open sensor socket
  float adcOffset1; // mV @ 20degC, agcGain = 1  as measured at CircuitSci factory or on v2diff reading with open sensor socket
  float adcOffset2; // mV @ 20degC, agcGain = 2  as measured at CircuitSci factory or on v2diff reading with open sensor socket
  float adcOffset3; // mV @ 20degC, agcGain = 3  as measured at CircuitSci factory or on v2diff reading with open sensor socket
  float auxOffset;
  
  float filterFrequency;
  float filterQfactor;

  float sensitivity_factor_values[ SENSITIVITY_TABLE_LENGTH ];
  float zero_current_factor_values[ ZERO_CURRENT_TABLE_LENGTH ];
  
  uint16_t avgNumber; // Number of adc samples to average for each sensor reading
  uint16_t avgDelay; // Milliseconds from end of one averaging sample to beginning of next, sample itself takes ~20ms w/16MHz MCU clock.

  uint8_t data_fields [ DATA_FIELDS_ARRAY_SIZE ];
  
  float gasTemp;
  

};

extern AQSettings g_settings;

#endif //AQ_SETTINGS_H
