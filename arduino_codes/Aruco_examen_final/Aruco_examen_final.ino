#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define ENC_A 34
#define DAC_A 13
//#define ADC_PIN A2

#define DIR_A1 14
#define DIR_A2 12

#define ENC_B 36
#define DAC_B 27

#define DIR_B1 26
#define DIR_B2 25

const float ppr = 11.0;
const float reduction_ratio = 26.0;


volatile unsigned long pulseCount_A = 0;
volatile unsigned long lastInterruptTime_A = 0;

volatile unsigned long pulseCount_B = 0;
volatile unsigned long lastInterruptTime_B = 0;

const int pwmMaxValue = 255;

float Kp_B = 0.4;
float Ki_B = 0.1;
float Kd_B = 0.011;

float Kp_A = 0.415;
float Ki_A = 0.1;
float Kd_A = 0.01;

const float outputMin = 0;
const float outputMax = pwmMaxValue;

float integral = 0;
float lastError = 0;
float lastMeasurement = 0;

float setpoint_A = 0;
float setpoint_B = 0;

const int updateInterval = 10;
const float derivativeFilterConstant = 0.9;

Adafruit_MPU6050 mpu;

float yaw = 0.0;
unsigned long lastTime = 0;
float gyroZ_offset = -0.01;

bool flag = true;
float giro_initial = 0;
unsigned long lastTime_recto = 0;
int action = 6;

void handleEncoder_B() {
    unsigned long interruptTime_B = micros();
    if (interruptTime_B - lastInterruptTime_B > 100) {
        pulseCount_B++;
        lastInterruptTime_B = interruptTime_B;
    }
}

void handleEncoder_A() {
    unsigned long interruptTime_A = micros();
    if (interruptTime_A - lastInterruptTime_A > 100) {
        pulseCount_A++;
        lastInterruptTime_A = interruptTime_A;
    }
}

float calculateRPM_A() {
    noInterrupts();
    unsigned long currentCount_A = pulseCount_A;
    pulseCount_A = 0;
    interrupts();

    float rpm_A = (currentCount_A * (60000.0 / updateInterval)) / (ppr * reduction_ratio);
    return rpm_A;
}

float calculateRPM_B() {
    noInterrupts();
    unsigned long currentCount_B = pulseCount_B;
    pulseCount_B = 0;
    interrupts();

    float rpm_B = (currentCount_B * (60000.0 / updateInterval)) / (ppr * reduction_ratio);
    return rpm_B;
}

float computePID(float setpoint, float measurement, float dt, float Kp, float Kd, float Ki) {
    float error = setpoint - measurement;
    
    float proportional = Kp * error;
    
    integral += error * dt;
    integral = constrain(integral, -outputMax / Ki, outputMax / Ki);
    float integralTerm = Ki * integral;

    float output = proportional + integralTerm;
    output = constrain(output, outputMin, outputMax);

    if (output > outputMin && output < outputMax) {
        integral += error * dt;
    }

    output *= 2;

    return output;
}

void setup() {
    Serial.begin(115200);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(DAC_A, OUTPUT);

    pinMode(ENC_B, INPUT_PULLUP);
    pinMode(DAC_B, OUTPUT);
    
    pinMode(DIR_A1,OUTPUT);
    pinMode(DIR_A2,OUTPUT);

    digitalWrite(DIR_A1,1);
    digitalWrite(DIR_A2,0);

    pinMode(DIR_B1,OUTPUT);
    pinMode(DIR_B2,OUTPUT);

    digitalWrite(DIR_B1,1);
    digitalWrite(DIR_B2,0);
    
    //analogReadResolution(12);
    attachInterrupt(digitalPinToInterrupt(ENC_A), handleEncoder_A, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_B), handleEncoder_B, RISING);

    if (!mpu.begin()) {
      while (1)
      {
        delay(10);
        Serial.println("Mpu Failed to start!");
      }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    lastTime = millis();
}
void loop() {
    static unsigned long lastUpdateTime = 0;
    unsigned long currentTime = millis();

    if(Serial.available() > 0){
      String receivedData = Serial.readStringUntil('\n');
      receivedData.trim();
      action = receivedData.toInt();
      lastTime = millis();
    }

    setpoint_A = 70;
    setpoint_B = 70;

    switch (action){
      case 0:{
        //Izquierda 90
        digitalWrite(DIR_A1,1);
        digitalWrite(DIR_A2,0);
        digitalWrite(DIR_B1,0);
        digitalWrite(DIR_B2,1);
        bool giro = giro_mpu(82);
        if (giro == false){
          action = 5;
          flag = true;
          lastTime_recto = millis();
        }
      }
      break;
      case 1:{
        //Derecha 90
        digitalWrite(DIR_A1,0);
        digitalWrite(DIR_A2,1);
        digitalWrite(DIR_B1,1);
        digitalWrite(DIR_B2,0);
        bool giro = giro_mpu(80);
        if (giro == false){
          action = 5;
          flag = true;
          lastTime_recto = millis();
        }
      }
      break;
      case 2:{
        //180
        digitalWrite(DIR_A1,0);
        digitalWrite(DIR_A2,1);
        digitalWrite(DIR_B1,1);
        digitalWrite(DIR_B2,0);
        bool giro = giro_mpu(170);
        if (giro == false){
          action = 5;
          flag = true;
          lastTime_recto = millis();
        }
      }
      break;
      case 3:{
        //Izquierda 90
        digitalWrite(DIR_A1,1);
        digitalWrite(DIR_A2,0);
        digitalWrite(DIR_B1,0);
        digitalWrite(DIR_B2,1);
        bool giro = giro_mpu(82);
        if (giro == false){
          action = 5;
          flag = true;
          lastTime_recto = millis();
        }
      }
      break;
      case 4:{
        //180
        digitalWrite(DIR_A1,0);
        digitalWrite(DIR_A2,1);
        digitalWrite(DIR_B1,1);
        digitalWrite(DIR_B2,0);
        bool giro = giro_mpu(170);
        if (giro == false){
          action = 6;
          flag = true;
          lastTime_recto = millis();
        }
      }
      break;
      case 5:{
        //Avance
        digitalWrite(DIR_A1,1);
        digitalWrite(DIR_A2,0);
        digitalWrite(DIR_B1,1);
        digitalWrite(DIR_B2,0);
      }
      break;
      case 6:{
        //Parada
        setpoint_A = 0;
        setpoint_B = 0;
      }
      break;
    }

    if (currentTime - lastUpdateTime >= updateInterval) {
        float rpm_A = calculateRPM_A();
        float rpm_B = calculateRPM_B();
        
        float dt = (currentTime - lastUpdateTime) / 1000.0;
        
        float pwmValue_A = computePID(setpoint_A, rpm_A, dt, Kp_A, Kd_A, Ki_A);
        analogWrite(DAC_A, (int)pwmValue_A);

        float pwmValue_B = computePID(setpoint_B, rpm_B, dt, Kp_B, Kd_B, Ki_A);
        analogWrite(DAC_B, (int)pwmValue_B);
        
        lastUpdateTime = currentTime;
    }
}

bool giro_mpu(int objetivo_ang){
  /*----------MPU INFO GET----------*/
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  /*----------MPU TIME CALCULATION----------*/
  unsigned long currentTime_m = millis();
  float deltaTime = (currentTime_m - lastTime) / 1000.0;
  lastTime = currentTime_m;

  /*----------DEGREES TRANSFORMATION----------*/
  yaw += (gyro.gyro.z - gyroZ_offset) * 180.0 / PI * deltaTime;

  if(flag==true){
    giro_initial = abs(yaw); // Modificar segun la direccion del giro
    flag = false;
  }

  float diff_angulo = abs(giro_initial-abs(yaw));

  if( diff_angulo >= objetivo_ang){ // Modificar segun la direccion del giro 170 o 82
    return false;
  }
  //Poner movimiento de los motores aqui
  return true;
}