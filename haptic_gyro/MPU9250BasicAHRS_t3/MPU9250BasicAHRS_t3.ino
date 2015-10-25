#include <Adafruit_DRV2605.h>
#include <i2c_t3.h>
#include <SPI.h>

#ifndef GRYO_H
#include "gyro.h"
#endif

//Address for I2C Mux
#define TCA9548A_ADDRESS 0x70
#define MPU_CHANNEL 6

#define SerialDebug true
#define HAPTIC_ACTIVE 0
#define HAPTIC_UPDATE_RATE 200

Adafruit_DRV2605 drv;
struct MotorGrid {
    uint8_t upMotor;
    uint8_t downMotor;
    uint8_t rightMotor;
    uint8_t leftMotor;
    uint8_t frontMotor;
    uint8_t backMotor;
};
struct MotorGrid motorGrid; 

uint32_t count = 0, sumCount = 0, count_haptic = 0; // used to control display output rate
float deltat = 0.0f, sum = 0.0f;        // integration interval for both filter schemes
uint32_t lastUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;        // used to calculate integration interval
float y, p, r;
uint32_t delt_t = 0, delt_t_haptic = 0;

void setup() {
    // Setup for Master mode, pins 18/19, external pullups, 400kHz
    delay(5000);
    Serial.begin(9600);
    Serial.println("Serial Started");
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    Serial.println("Wire Started");
    
    // Set up the interrupt pin, its set as active high, push-pull
    pinMode(intPin, INPUT);
    digitalWrite(intPin, LOW);
    pinMode(adoPin, OUTPUT);
    digitalWrite(adoPin, HIGH);
    pinMode(myLed, OUTPUT);
    digitalWrite(myLed, HIGH);
    Serial.println("Did digital writes");
    
    
    //Add test for I2C MUX access
    Wire.beginTransmission(TCA9548A_ADDRESS);
    uint8_t error = Wire.endTransmission();
    if (error != 0){
        Serial.println("Can't Access I2C MUX. Check hookups");
        while(1);
    } else {
        Serial.println("Connected to I2C MUX");
    }
   
    //loop through each haptic channel and initialize
    if (HAPTIC_ACTIVE) {
        for (int i = 0; i < 8; i++) {
            selectI2CChannels(i);
            drv.begin();
            drv.selectLibrary(1);
      
            // I2C trigger by sending 'go' command 
            // default, internal trigger when sending GO command
            drv.setMode(DRV2605_MODE_INTTRIG);  
        }
    }
  
    selectI2CChannels(MPU_CHANNEL);  
   
    if (testMPUConnection()) { // WHO_AM_I should always be 0x68

      Serial.println("MPU9250 is online...");

      selfTestMPU();
      delay(5000);
    
      calibrateMPU(); // Calibrate gyro and accelerometers, load biases in bias registers
      delay(1000); 
    
      initMPU9250(); 
      Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
    
      // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
      if (testMagConnection()) {
          Serial.println("Magnometer is online...");
      } else {
          Serial.println("Could not connect to Magnometer");
          while(1) ; // Loop forever if communication doesn't happen
      }
      delay(1000); 
    
      // Get magnetometer calibration from AK8963 ROM
      initMag(); 
      Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer
    
      calibrateMag();
      delay(2000); // add delay to see results before serial spew of data
    
    }  else {
        Serial.println("Could not connect to MPU9250");
        while(1) ; // Loop forever if communication doesn't happen
    }
}

void loop() {  
    selectI2CChannels(MPU_CHANNEL);
    // If intPin goes high, all data registers have new data
    if (checkMPUInterrupt() & 0x01) {  // On interrupt, check if data ready interrupt
        getAccelData();
        getGyroData();
        getMagData();
    }
  
    Now = micros();
    deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate = Now;

    sum += deltat; // sum for averaging filter update rate
    sumCount++;
 
    makeQuaternion(deltat);
    y = getYaw();
    p = getPitch();
    r = getRoll();

    delt_t_haptic = millis() - count_haptic;
    if (HAPTIC_ACTIVE & (delt_t_haptic > HAPTIC_UPDATE_RATE)) {
        translateYPR2Waveforms(y, p, r);
        adjustMotors();
        count_haptic = millis();
    }
    
    delt_t = millis() - count;
    if(SerialDebug & (delt_t > 500)) {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(y, 2);
        Serial.print(", ");
        Serial.print(p, 2);
        Serial.print(", ");
        Serial.println(r, 2);
        if (HAPTIC_ACTIVE) {
            Serial.print("Motor Left(0): ");Serial.print(motorGrid.leftMotor);Serial.print(" Motor Right(1): ");Serial.print(motorGrid.rightMotor);
            Serial.print(" Motor Up(2): ");Serial.print(motorGrid.upMotor);Serial.print(" Motor Down(3): ");Serial.print(motorGrid.downMotor);
            Serial.print(" Motor Front(4): ");Serial.print(motorGrid.frontMotor);Serial.print(" Motor Back(5): ");Serial.println(motorGrid.backMotor);
        }
        Serial.print("rate = "); Serial.print((float)sumCount/sum, 2); Serial.println(" Hz");
        count = millis(); 
    }
   
    // With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and 
    // >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
    // The filter update rate is determined mostly by the mathematical steps in the respective algorithms, 
    // the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
    // an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
    // filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively. 
    // This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
    // This filter update rate should be fast enough to maintain accurate platform orientation for 
    // stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
    // produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
    // The 3.3 V 8 MHz Pro Mini is doing pretty well!

    //digitalWrite(myLed, !digitalRead(myLed));
    sumCount = 0;
    sum = 0;   
}


/*===================================================================================================================
====== Set of useful function for haptic controls
===================================================================================================================
Function to access the correct haptic motor on the wire, and set it's waveform 
* Here is the location of haptic motor to channel 
* left motor = channel 0
* right motor = channel 1
* up motor = channel 2
* down motor = channel 3
* front motor = channel 4
* back motor = channel 7
*
* FYI MPU is on channel 6
*/
void adjustMotors() {
    if (motorGrid.leftMotor != 0.0) {
        selectI2CChannels(0);
        playWaveform(motorGrid.leftMotor);
    }
    if (motorGrid.rightMotor != 0.0) {
        selectI2CChannels(1);
        playWaveform(motorGrid.rightMotor);
    }
    if (motorGrid.upMotor != 0.0) {
        selectI2CChannels(2);
        playWaveform(motorGrid.upMotor);
    }
    if (motorGrid.downMotor != 0.0) {
        selectI2CChannels(3);
        playWaveform(motorGrid.downMotor);
    }
    if (motorGrid.frontMotor != 0.0) {
        selectI2CChannels(4);
        playWaveform(motorGrid.frontMotor);
    }
    if (motorGrid.backMotor != 0.0) {
        selectI2CChannels(7);
        playWaveform(motorGrid.backMotor);
    }
}
  
void playWaveform(uint8_t effect) {
    drv.setWaveform(0, effect);  // play effect 
    drv.setWaveform(1, 0);       // end waveform

    // play the effect!
    drv.go();
    delay(30);
}

uint8_t getPercentileWaveform(float angle) {
    uint8_t waveFormId = 0;
    float absAngle = abs(angle);
    if (absAngle/180 <= 0.20){
        waveFormId = 123;
    } else if (absAngle/180 <= 0.40) {
        waveFormId = 122;  
    } else if (absAngle/180 <= 0.60) {
        waveFormId = 121;  
    } else if (absAngle/180 <= 0.80) {
        waveFormId = 120;  
    } else {
        waveFormId = 119;
    }
  
  return waveFormId;
}

void translateYPR2Waveforms(float y, float p, float r){
  
    float yawWVId = getPercentileWaveform(y);
    if (y == 0.0){
        motorGrid.leftMotor = 0;
        motorGrid.rightMotor = 0;
    } else if (y > 0.0){
        motorGrid.leftMotor = yawWVId;
        motorGrid.rightMotor = 0;
    } else {
        motorGrid.leftMotor = 0;
        motorGrid.rightMotor = yawWVId;
    }
    
    float rollWVId = getPercentileWaveform(r);
    if (r == 0.0){
        motorGrid.downMotor = 0;
        motorGrid.upMotor = 0;
    } else if (r > 0.0){
        motorGrid.downMotor = rollWVId;
        motorGrid.upMotor = 0;
    } else {
        motorGrid.downMotor = 0;
        motorGrid.upMotor = rollWVId;
    } 
    
    float pitchWVId = getPercentileWaveform(p);
    if (p == 0.0){
        motorGrid.frontMotor = 0;
        motorGrid.backMotor = 0;
    } else if (p > 0.0) {
        motorGrid.frontMotor = pitchWVId;
        motorGrid.backMotor = 0;
    } else {
        motorGrid.frontMotor = 0;
        motorGrid.backMotor = pitchWVId;
    }
}

void selectI2CChannels(uint8_t channels) {
    Wire.beginTransmission(TCA9548A_ADDRESS);
    Wire.write(1 << channels); 
    Wire.endTransmission();
}


