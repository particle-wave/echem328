/*
AQSensorValues.h header contains struct for current sensor values.
By Elliott Dicus (manylabs.org) 
typically used with the echem328 smart gas sensor module and echemMon serial monitor and control software
Released under Creative Commons SA-BY 4.0 license
See circuitsci.com for more information.
*/
#ifndef AQ_SENSOR_VALUES_H
#define AQ_SENSOR_VALUES_H

/* Sensor Values */
struct AQSensorValues {
  float tempC;
  float humidity;

  float echemVdd;
  float echemVin;
  float ppmraw;
  float ppm;
  float sensorCurrent;
  float volt1diff;
  float volt2diff;
  float volt3;
  float tempDelta;
};

#endif // AQ_SENSOR_VALUES_H
