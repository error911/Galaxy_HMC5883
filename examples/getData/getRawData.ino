/*!
 * @license     The MIT License (MIT)
 * @author      Roman Murnik
 * @version     V0.0.2
 * @date        2024-02-02
 * @url         https://github.com/error911/Galaxy_HMC5883
 */
#include <Galaxy_HMC5883.h>

Galaxy_HMC5883 compass(HMC5883L_ADDRESS);

void setup()
{
  Serial.begin(9600);
  while (!compass.begin())
  {
    Serial.println("Could not find a valid sensor!");
    delay(2000);
  }

  Serial.println("Initialize process...");
  // Compass signal gain range, default to be 1.3 Ga
  compass.setRange(HMC5883L_RANGE_1_3GA);
  Serial.println(compass.getRange());

  // Measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  Serial.println(compass.getMeasurementMode());

  // Data collection frequency
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  Serial.println(compass.getDataRate());

  // Sensor status
  compass.setSamples(HMC5883L_SAMPLES_8);
  Serial.println(compass.getSamples());

  delay(1000);
}

void loop()
{
  /**
   * @brief  Set declination angle on your location and fix heading
   * @n      You can find your declination on: http://magnetic-declination.com/
   * @n      (+) Positive or (-) for negative
   * @n      For Bytom / Poland declination angle is 4'26E (positive)
   * @n      Formula: (deg + (min / 60.0)) / (180 / PI);
   */

  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
  compass.setDeclinationAngle(declinationAngle);

  Vector_t mag = compass.readRaw();
  float heading = compass.getHeadingDegrees();

  Serial.print(heading); Serial.print("\t"); Serial.print("\t");
  Serial.print(mag.x); Serial.print("\t");
  Serial.print(mag.y); Serial.print("\t");
  Serial.print(mag.z); Serial.print("\t");
  Serial.println();

  delay(100);
}
