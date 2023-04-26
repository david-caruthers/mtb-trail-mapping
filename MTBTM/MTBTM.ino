
//v3 added IMU fields for rotation vector and remove individual x/y/z printing
// Basic demo for readings from Adafruit BNO08x
#include <Adafruit_BNO08x.h>

#include <Arduino.h>
// This demo explores two reports (SH2_ARVR_STABILIZED_RV and SH2_GYRO_INTEGRATED_RV) both can be used to give
// quartenion and euler (yaw, pitch roll) angles.  Toggle the FAST_MODE define to see other report.
// Note sensorValue.status gives calibration accuracy (which improves over time)


#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <SD.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
// For SPI mode, we need a CS pin
#define BNO08X_CS 10
#define BNO08X_INT 9
//#define FAST_MODE

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1
#define SEALEVELPRESSURE_HPA (1007)
#define CIRCUMFERENCE 5236  //in kilofeet, assuming 10 in wheel radius

//HES
volatile byte half_revolutions;
unsigned int spd;
unsigned long distance;
unsigned long timeold;
const int hall_pin = 2;

File myFile;
char filename[] = "gudstuff.txt";  // filename (without extension) should not exceed 8 chars
const int chipSelect = 53;

//SD.exists() function?


unsigned long myTime;

Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif

void setup(void) {

  Serial1.begin(9600);
  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");

  myTime = millis();

  Serial.begin(115200);

  while (!Serial)
    delay(10);  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit BNO08x/BMP388 test!");

  //HES
  //attachInterrupt(0, magnet_detect, RISING);//Initialize the intterrupt pin (Arduino digital pin 2)

  pinMode(hall_pin, INPUT);
  half_revolutions = 0;

  distance = 0;
  timeold = 0;
  spd = 0;

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
    // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
                           //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
                           //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }
  Serial.println("BNO08x Found!");

  // for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
  //   Serial.print("Part ");
  //   Serial.print(bno08x.prodIds.entry[n].swPartNumber);
  //   Serial.print(": Version :");
  //   Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
  //   Serial.print(".");
  //   Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
  //   Serial.print(".");
  //   Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
  //   Serial.print(" Build ");
  //   Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  // }

  setReports(reportType, reportIntervalUs);

  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("Reading events");
  delay(100);
}

// Here is where you define the sensor outputs you want to receive
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");

  // if (!bno08x.enableReport(SH2_ACCELEROMETER,3000)) {
  //   Serial.println("Could not enable accelerometer");
  // }
  // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED,report_interval)) {
  //   Serial.println("Could not enable gyroscope");
  // }
  // if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
  //   Serial.println("Could not enable magnetic field calibrated");
  // }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 2500)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 2500)) {
    Serial.println("Could not enable rotation vector");
  }
  //  if (! bno08x.enableReport(reportType, report_interval)) {
  //   Serial.println("Could not enable stabilized remote vector");
  // }
}
//HES
void magnet_detect()  //This function is called whenever a magnet/interrupt is detected by the arduino
{
  half_revolutions++;
  Serial.println(half_revolutions);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}


void loop() {

  //add button conditional for start of data logging. should this be in setup? or just use as trigger?
  //delay(10);

  // make a string for assembling the data to log:
  char dataString[490] = "";
  // String dataString = "";

  for (int i = 0; i < 7; i++) {

    myTime = millis();
    // myFile.print("Time, ");
    // myFile.print(myTime);
    // myFile.print(" , ");

    char currtime[10] = "";
    snprintf(currtime, sizeof(currtime), "%lu", myTime);
    strcat(dataString, currtime);
    strcat(dataString, ",");
    // Serial.print("Time, ");
    // Serial.print(myTime);
    // Serial.print(" , ");

    // if (bno08x.wasReset()) {
    //   Serial.print("sensor was reset ");
    //   setReports(reportType, reportIntervalUs);
    // }

    // if (!bno08x.getSensorEvent(&sensorValue)) {
    //   return;
    // }

    // if (!bmp.performReading()) {
    //   Serial.println("Failed to perform reading :(");
    //   return;
    // }
    // //Serial.print("Time, "); Serial.print(myTime); Serial.print(" , ");

    char bmpout[7] = "";
    dtostrf(bmp.readAltitude(SEALEVELPRESSURE_HPA), 5, 1, bmpout);
    // snprintf(bmpout, sizeof(bmpout), "\tTemp, %f, Press, %f, Alt, %f,",
    //          bmp.temperature, bmp.pressure/100, bmp.readAltitude(SEALEVELPRESSURE_HPA));
    strcat(dataString, bmpout);
    strcat(dataString, ",");

    // myFile.print("\tTemp, ");
    // myFile.print(bmp.temperature);
    // // //myFile.print(" *C,\t");

    // myFile.print(", Press, ");
    // myFile.print(bmp.pressure / 100.0);
    // // //myFile.print(" hPa,\t");

    // myFile.print(", Alt, ");
    // myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    // //myFile.print(" m, ");

    // Serial.print(", Temp, ");
    // Serial.print(bmp.temperature); myFile.print(" , ");
    // // //Serial.print(" *C,\t");

    // Serial.print(", Pressure, ");
    // Serial.print(bmp.pressure / 100.0);
    // Serial.print(" hPa,\t");

    // Serial.print("Altitude, ");
    // Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    // Serial.print(" , ");




    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {

          // case SH2_ARVR_STABILIZED_RV:
          //   quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
          // case SH2_GYRO_INTEGRATED_RV:
          //   // faster (more noise?)
          //   quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          //   break;
        case SH2_ROTATION_VECTOR:
          break;
        case SH2_LINEAR_ACCELERATION:
          break;
      }
      //   static long last = 0;
      //   long now = micros();
      //   myTime = millis();
      //   // Serial.print(now - last);             Serial.print("\t <= deltaT ");
      //   //Serial.print(",Time, "); Serial.print(myTime);  Serial.print(",");

      //   myFile.print("\t ypr, ");
      //   myFile.print(ypr.yaw);                myFile.print("\t, ");
      //   myFile.print(ypr.pitch);              myFile.print("\t, ");
      //   myFile.print(ypr.roll);             myFile.print("\t,");
      //   myFile.print(sensorValue.status);   myFile.print("\t, ");

      //   // Serial.print("\t ypr, ");
      //   // Serial.print(ypr.yaw);                Serial.print("\t, ");
      //   // Serial.print(ypr.pitch);              Serial.print("\t, ");
      //   // Serial.print(ypr.roll);             Serial.print("\t, ");
      //   // // last = now;
      //   // Serial.print(sensorValue.status);   Serial.print("\t, "); // This is accuracy in the range of 0 to 3

      //   switch (sensorValue.sensorId) {



      // case SH2_ROTATION_VECTOR:
      char rotreal[7] = "";
      char roti[7] = "";
      char rotj[7] = "";
      char rotk[7] = "";
      //char rotstatus[2] = "";

      dtostrf(sensorValue.un.rotationVector.real, 6, 2, rotreal);
      dtostrf(sensorValue.un.rotationVector.i, 6, 2, roti);
      dtostrf(sensorValue.un.rotationVector.j, 6, 2, rotj);
      dtostrf(sensorValue.un.rotationVector.k, 6, 2, rotk);
      //dtostrf(sensorValue.status, 1, 0, rotstatus);
      // snprintf(rotvect, sizeof(rotvect), "\tRotVect rijk, %f, %f, %f, %f, %f,",
      //          sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i,
      //          sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, sensorValue.status);
      strcat(dataString, rotreal);
      strcat(dataString, ",");
      strcat(dataString, roti);
      strcat(dataString, ",");
      strcat(dataString, roti);
      strcat(dataString, ",");
      strcat(dataString, roti);
      strcat(dataString, ",");

      // strcat(dataString, rotstatus);
      // strcat(dataString, ",");


      // // myFile.print("\tRotVect rijk, ");
      // // myFile.print(sensorValue.un.rotationVector.real);
      // // myFile.print(" , ");
      // // myFile.print(sensorValue.un.rotationVector.i);
      // // myFile.print(" , ");
      // // myFile.print(sensorValue.un.rotationVector.j);
      // // myFile.print(" , ");
      // // myFile.print(sensorValue.un.rotationVector.k);
      // // myFile.print(" , ");
      // // myFile.print(sensorValue.status);   myFile.print(" , ");

      // // Serial.print("\tRotationVector rijk, ");
      // // Serial.print(sensorValue.un.rotationVector.real);
      // // Serial.print(" , ");
      // // Serial.print(sensorValue.un.rotationVector.i);
      // // Serial.print(" , ");
      // // Serial.print(sensorValue.un.rotationVector.j);
      // // Serial.print(" , ");
      // // Serial.print(sensorValue.un.rotationVector.k);

      // // break;

      // // case SH2_ACCELEROMETER:
      // // myFile.print("\tAccelerometer xyz, ");
      // // myFile.print(sensorValue.un.accelerometer.x);
      // // myFile.print(", ");
      // // myFile.print(sensorValue.un.accelerometer.y);
      // // myFile.print(", ");
      // // myFile.print(sensorValue.un.accelerometer.z);
      // // myFile.print(" , ");

      // // Serial.print("\t Accelerometer xyz, ");
      // // Serial.print(sensorValue.un.accelerometer.x);
      // // Serial.print(", ");
      // // Serial.print(sensorValue.un.accelerometer.y);
      // // Serial.print(", ");
      // // Serial.print(sensorValue.un.accelerometer.z);
      // // break;

      // // case SH2_LINEAR_ACCELERATION:
      // // myTime = millis();
      // // Serial.print("Time: ");
      // // Serial.print(myTime);

      // // myFile.print("\tLAccel xyz, ");
      // // myFile.print(sensorValue.un.linearAcceleration.x);
      // // myFile.print(", ");
      // // myFile.print(sensorValue.un.linearAcceleration.y);
      // // myFile.print(", ");
      // // myFile.print(sensorValue.un.linearAcceleration.z);
      // // myFile.print(" , ");
      // // myFile.print(sensorValue.status);   myFile.print(" , ");

      char linaccx[7] = "";
      char linaccy[7] = "";
      char linaccz[7] = "";
      char linaccstatus[2] = "";

      dtostrf(sensorValue.un.linearAcceleration.x, 6, 2, linaccx);
      dtostrf(sensorValue.un.linearAcceleration.y, 6, 2, linaccy);
      dtostrf(sensorValue.un.linearAcceleration.z, 6, 2, linaccz);
      dtostrf(sensorValue.status, 1, 0, linaccstatus);

      strcat(dataString, linaccx);
      strcat(dataString, ",");
      strcat(dataString, linaccy);
      strcat(dataString, ",");
      strcat(dataString, linaccz);
      strcat(dataString, ",");
      strcat(dataString, linaccstatus);
      strcat(dataString, ",");

      // snprintf(linacc, sizeof(linacc), "\tLinAcc xyz, %f , %f , %f , %f,",
      //      sensorValue.un.linearAcceleration.x, sensorValue.un.linearAcceleration.y,
      //     sensorValue.un.linearAcceleration.z, sensorValue.status);
      //      strcat(dataString,linacc);
      // Serial.print("\t LinearAccel xyz, ");
      // Serial.print(sensorValue.un.linearAcceleration.x);
      // Serial.print(", ");
      // Serial.print(sensorValue.un.linearAcceleration.y);
      // Serial.print(", ");
      // Serial.println(sensorValue.un.linearAcceleration.z);
      //   break;
      strcat(dataString, "\n");
    }
  }

  // // HES
  //       char spdprint[6] = "";
  //  char dist[10] = "";

  // if (half_revolutions >= 1) {
  //    distance += CIRCUMFERENCE;
  //    spd = (CIRCUMFERENCE*1.0)/(millis() - timeold); //in feet per second
  //    timeold = millis();
  //    half_revolutions = 0;

  //  }
  //  else{
  //    spd = 0;

  //  }
  //  snprintf(dist, sizeof(dist), "%lu",distance);
  //    dtostrf(spd,5,2,spdprint);

  //    Serial.print("Spd:");
  //    Serial.print(spdprint);
  //    Serial.print("Dist:");
  //    Serial.println(distance); //in kilofeet

  //   strcat(dataString, distance);
  //   strcat(dataString, ",");
  //   strcat(dataString, spd);
  //   strcat(dataString, ",");


  // }


  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(filename, FILE_WRITE);


  // if the file is available, write to it:
  if (myFile) {
    myFile.println(dataString);
    //myFile.write("\n");
    myFile.close();
    // print to the serial port too:
    // Serial.println(dataString);
    //Serial.println("\n");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial1.println("error opening datalog.txt");
  }
}
