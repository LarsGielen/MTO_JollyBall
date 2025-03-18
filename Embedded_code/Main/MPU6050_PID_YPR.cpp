#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// MPU6050 object
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float ypr_cal[3];
// Orientation/motion vars
VectorFloat gravity;    // [x, y, z]            gravity vector
Quaternion q;           // [w, x, y, z]         quaternion container

float yprOffset[3] = {0, 0, 0}; // calibration offsets

// Calibration vars
const int calibrationSamples = 100;
bool calibrated = false;
bool sensorOffsetCalibrated = false;

// Raw accel/gyro values for offset calibration
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Sensor offsets
int16_t accelOffsets[3] = {0, 0, 0};  // [x, y, z]
int16_t gyroOffsets[3] = {0, 0, 0};   // [x, y, z]

            static float filteredYaw = 0;
            static float filteredPitch = 0;
            static float filteredRoll = 0;
const float alpha = 0.05; // Low alpha value for stronger smoothing
std::array<float,3> results;

// MPU6050 Register addresses
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

// DLPF Configuration options
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
void configureDLPF();
void calibrateSensorOffsets();
void finetuneOffsets();
void calibrateYPR();
void setup_MPU6050();
std::array<float,3> readout_MPU6050();

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup_MPU6050() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock
    
    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo/Micro/Pro Micro
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // Configure DLPF
    Serial.println(F("Configuring DLPF settings..."));
    configureDLPF();
    
    // Start with sensor offset calibration
    Serial.println(F("Starting sensor offset calibration in 3 seconds..."));
    Serial.println(F("Keep the MPU6050 stationary on a level surface"));
    delay(3000);
    calibrateSensorOffsets();
    
    // Now initialize the DMP with the calibrated offsets
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Supply the calibrated offsets to the DMP
    mpu.setXAccelOffset(accelOffsets[0]);
    mpu.setYAccelOffset(accelOffsets[1]);
    mpu.setZAccelOffset(accelOffsets[2]);
    mpu.setXGyroOffset(gyroOffsets[0]);
    mpu.setYGyroOffset(gyroOffsets[1]);
    mpu.setZGyroOffset(gyroOffsets[2]);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        Serial.println(F("Starting YPR calibration in 3 seconds..."));
        Serial.println(F("Keep the MPU6050 stationary in its mounted position"));
        delay(3000);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// ================================================================
// ===                    DLPF CONFIGURATION                    ===
// ================================================================

void configureDLPF() {
    // Set the Digital Low Pass Filter (DLPF)
    // This affects both gyro and accelerometer bandwidth
    //
    // DLPF_CFG | Bandwidth | Delay   | Fs
    // ---------+-----------+---------+------
    // 0        | 260 Hz    | 0.0 ms  | 8 kHz
    // 1        | 184 Hz    | 2.0 ms  | 1 kHz
    // 2        | 94 Hz     | 3.0 ms  | 1 kHz
    // 3        | 44 Hz     | 4.9 ms  | 1 kHz
    // 4        | 21 Hz     | 8.5 ms  | 1 kHz
    // 5        | 10 Hz     | 13.8 ms | 1 kHz
    // 6        | 5 Hz      | 19.0 ms | 1 kHz
    
    // Set DLPF to lowest bandwidth (5 Hz) for maximum stability
    mpu.setDLPFMode(MPU6050_DLPF_BW_5);
    
    // Set the sample rate divider for the slowest possible rate
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // Gyroscope Output Rate = 8kHz when DLPF is disabled (DLPF_CFG = 0 or 7), 1kHz when enabled
    // With DLPF enabled, setting maximum divider of 255 gives:
    // Sample Rate = 1000 / (1 + 255) = ~3.9 Hz
    // mpu.setRate(255);
    
    // Configure additional settings
    // Full-scale range: ±250°/s gives maximum sensitivity
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    
    // Full-scale range: ±2g gives maximum sensitivity
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    
    Serial.println(F("DLPF configured for maximum stability:"));
    Serial.println(F("- DLPF Bandwidth: 5 Hz"));
    Serial.println(F("- Sample Rate: ~3.9 Hz"));
    Serial.println(F("- Gyro Range: ±250°/s"));
    Serial.println(F("- Accel Range: ±2g"));
}

// ================================================================
// ===              LOW-LEVEL REGISTER WRITE FUNCTION           ===
// ================================================================


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void readout_MPU6050(float* ypr){
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        if (!calibrated) {
            calibrateYPR();
        } else {
            // apply calibration offsets
            ypr[0] = ypr[0] - yprOffset[0];
            ypr[1] = ypr[1] - yprOffset[1];
            ypr[2] = ypr[2] - yprOffset[2];
        }
    }
}

// ================================================================
// ===              SENSOR OFFSET CALIBRATION ROUTINE           ===
// ================================================================

void calibrateSensorOffsets() {
    int16_t accelSums[3] = {0, 0, 0};
    int16_t gyroSums[3] = {0, 0, 0};
    const int numCalibrationReadings = 200;
    
    Serial.println(F("Calibrating accelerometer and gyroscope offsets..."));
    
    // Take multiple readings and average them
    for (int i = 0; i < numCalibrationReadings; i++) {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        accelSums[0] += ax;
        accelSums[1] += ay;
        accelSums[2] += az;
        gyroSums[0] += gx;
        gyroSums[1] += gy;
        gyroSums[2] += gz;
        
        if (i % 50 == 0) {
            Serial.print(".");
        }
        delay(10);
    }
    
    // Calculate offsets (negative of the average)
    accelOffsets[0] = -accelSums[0] / numCalibrationReadings;
    accelOffsets[1] = -accelSums[1] / numCalibrationReadings;
    // For Z, we want 16384 (1g) after offset
    accelOffsets[2] = -(accelSums[2] / numCalibrationReadings - 16384);
    
    gyroOffsets[0] = -gyroSums[0] / numCalibrationReadings;
    gyroOffsets[1] = -gyroSums[1] / numCalibrationReadings;
    gyroOffsets[2] = -gyroSums[2] / numCalibrationReadings;
    
    Serial.println();
    Serial.println(F("Accel offsets [X,Y,Z]: "));
    Serial.print(accelOffsets[0]); Serial.print(", ");
    Serial.print(accelOffsets[1]); Serial.print(", ");
    Serial.println(accelOffsets[2]);
    
    Serial.println(F("Gyro offsets [X,Y,Z]: "));
    Serial.print(gyroOffsets[0]); Serial.print(", ");
    Serial.print(gyroOffsets[1]); Serial.print(", ");
    Serial.println(gyroOffsets[2]);
    
    // Fine-tune offsets through multiple iterations
    finetuneOffsets();
    
    sensorOffsetCalibrated = true;
}

// ================================================================
// ===              OFFSET FINE-TUNING ROUTINE                  ===
// ================================================================

void finetuneOffsets() {
    const int numIterations = 5;
    const int numReadings = 50;
    
    Serial.println(F("Fine-tuning offsets..."));
    
    for (int iteration = 0; iteration < numIterations; iteration++) {
        // Apply current offsets
        mpu.setXAccelOffset(accelOffsets[0]);
        mpu.setYAccelOffset(accelOffsets[1]);
        mpu.setZAccelOffset(accelOffsets[2]);
        mpu.setXGyroOffset(gyroOffsets[0]);
        mpu.setYGyroOffset(gyroOffsets[1]);
        mpu.setZGyroOffset(gyroOffsets[2]);
        
        delay(100);
        
        // Take new readings with offsets applied
        int32_t accelSums[3] = {0, 0, 0};
        int32_t gyroSums[3] = {0, 0, 0};
        
        for (int i = 0; i < numReadings; i++) {
            mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            
            accelSums[0] += ax;
            accelSums[1] += ay;
            accelSums[2] += az;
            gyroSums[0] += gx;
            gyroSums[1] += gy;
            gyroSums[2] += gz;
            
            delay(10);
        }
        
        // Calculate remaining error
        int16_t accelError[3];
        int16_t gyroError[3];
        
        accelError[0] = accelSums[0] / numReadings;
        accelError[1] = accelSums[1] / numReadings;
        accelError[2] = accelSums[2] / numReadings - 16384; // Z should read 1g (16384)
        
        gyroError[0] = gyroSums[0] / numReadings;
        gyroError[1] = gyroSums[1] / numReadings;
        gyroError[2] = gyroSums[2] / numReadings;
        
        // Adjust offsets further
        accelOffsets[0] -= accelError[0] / 8; // Divide by 8 for smoother convergence
        accelOffsets[1] -= accelError[1] / 8;
        accelOffsets[2] -= accelError[2] / 8;
        
        gyroOffsets[0] -= gyroError[0] / 4;
        gyroOffsets[1] -= gyroError[1] / 4;
        gyroOffsets[2] -= gyroError[2] / 4;
        
        Serial.print(F("Iteration "));
        Serial.print(iteration + 1);
        Serial.println(F(" completed"));
    }
    
    // Set final offsets
    mpu.setXAccelOffset(accelOffsets[0]);
    mpu.setYAccelOffset(accelOffsets[1]);
    mpu.setZAccelOffset(accelOffsets[2]);
    mpu.setXGyroOffset(gyroOffsets[0]);
    mpu.setYGyroOffset(gyroOffsets[1]);
    mpu.setZGyroOffset(gyroOffsets[2]);
    
    Serial.println(F("Final Accel offsets [X,Y,Z]: "));
    Serial.print(accelOffsets[0]); Serial.print(", ");
    Serial.print(accelOffsets[1]); Serial.print(", ");
    Serial.println(accelOffsets[2]);
    
    Serial.println(F("Final Gyro offsets [X,Y,Z]: "));
    Serial.print(gyroOffsets[0]); Serial.print(", ");
    Serial.print(gyroOffsets[1]); Serial.print(", ");
    Serial.println(gyroOffsets[2]);
    
    Serial.println(F("Sensor offset calibration complete!"));
}

// ================================================================
// ===                    YPR CALIBRATION ROUTINE               ===
// ================================================================

void calibrateYPR() {
    static int sampleCount = 0;
    static float yprSum[3] = {0, 0, 0};
    
    if (sampleCount < calibrationSamples) {
        // Accumulate samples
        yprSum[0] += ypr_cal[0];
        yprSum[1] += ypr_cal[1];
        yprSum[2] += ypr_cal[2];
        
        sampleCount++;
        
        if (sampleCount % 10 == 0) {
            Serial.print("YPR Calibrating... ");
            Serial.print(sampleCount);
            Serial.print("/");
            Serial.println(calibrationSamples);
        }
    } else {
        // Calculate average offsets
        yprOffset[0] = yprSum[0] / calibrationSamples;
        yprOffset[1] = yprSum[1] / calibrationSamples;
        yprOffset[2] = yprSum[2] / calibrationSamples;
        
        Serial.println("YPR Calibration complete!");
        Serial.print("YPR Offsets - Yaw: ");
        Serial.print(yprOffset[0]);
        Serial.print(", Pitch: ");
        Serial.print(yprOffset[1]);
        Serial.print(", Roll: ");
        Serial.println(yprOffset[2]);
        
        calibrated = true;
    }
}
