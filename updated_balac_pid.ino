#include <M5StickC.h>

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float target = -96.5;
float P, I, D, preP;

float power = 0;
float dt, preTime;

bool started = false;

float roll, pitch, yaw;
float now;
float loopfreq;

bool led = false;

static float pRatio = 1.0;


//---------------------------------------------------------------------------------------------------
// IMU algorithm update

/* This is a modified version of the original MahonyAHRSupdateIMU algorithm (which can be found in M5StickC/src/utility/MahonyAHRS.cpp) */
void myMahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float *pitch,float *roll,float *yaw, float samplefrequency) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  static float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;          // quaternion of sensor frame relative to auxiliary frame
 
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Apply proportional feedback
    gx += 2.0f * halfex;
    gy += 2.0f * halfey;
    gz += 2.0f * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * (1.0f / samplefrequency));    // pre-multiply common factors
  gy *= (0.5f * (1.0f / samplefrequency));
  gz *= (0.5f * (1.0f / samplefrequency));
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;


  *pitch = asin(-2 * q1 * q3 + 2 * q0* q2); // pitch
  *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1); // roll
  *yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);  //yaw

  *pitch *= RAD_TO_DEG;
    *yaw   *= RAD_TO_DEG;
    // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
    //  8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
    // - http://www.ngdc.noaa.gov/geomag-web/#declination
    *yaw   -= 8.5;
    *roll  *= RAD_TO_DEG;

}

/* Modified version of the M5.MPU6886.GetAhrsData to accept the input of the sample frequency */
void myGetAhrsData(float *pitch,float *roll,float *yaw, float samplefrequency){

  float accX = 0; 
  float accY = 0;
  float accZ = 0;

  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;

  M5.MPU6886.getGyroData(&gyroX,&gyroY,&gyroZ);
  M5.MPU6886.getAccelData(&accX,&accY,&accZ);
  
  myMahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ,pitch,roll,yaw,samplefrequency);
}

void drawScreen() {
    M5.Lcd.setCursor(0, 0);
    if (!started) 
    {
      M5.Lcd.printf("Press ^ btn\n");
      M5.Lcd.printf("  to start\n\n");
    }
    M5.Lcd.printf("Set:%6.1f\n", target);
    M5.Lcd.printf("roll:%6.1f\n", roll);
    M5.Lcd.printf("now:%4.1f\n", target - roll);
    M5.Lcd.printf("Power:%2.1f\n", power);
    //M5.Lcd.printf("Bat:%5.1fV\n", M5.Axp.GetBatVoltage());
    //M5.Lcd.printf("Cur:%5.1f\n", M5.Axp.GetBatCurrent());
    M5.Lcd.printf("freq:%5.2f  ",loopfreq);

}

void wheelA(int8_t val)
{
    if (val!=0) val += 5 *(val/abs(val));
    val = constrain(val, -100, 100);
    
    Wire.beginTransmission(0x38);
    Wire.write(0x00);
    Wire.write(val);
    Wire.endTransmission();
}

void wheelB(int8_t val)
{
    Wire.beginTransmission(0x38);
    Wire.write(0x01);
    Wire.write(val);
    Wire.endTransmission();
}

void setup() {
    // put your setup code here, to run once:
    M5.begin();
    Wire.begin(0, 26, 400000);
    M5.Lcd.setRotation(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Axp.ScreenBreath(8);
    M5.MPU6886.Init();
    wheelA(0);
    wheelB(0);
    pinMode(GPIO_NUM_10, OUTPUT);
    digitalWrite(GPIO_NUM_10, HIGH);

    Kp = 10.0;
    Ki = 0.6;
    Kd = 0.05;
    
    drawScreen();
}

void loop() {

    M5.update();
    if (M5.BtnB.wasReleased()) {
        // ボタンBを押すと再起動
        esp_restart();
    }

    if (M5.BtnA.wasReleased()) 
    {
        started = true;
        M5.Lcd.fillScreen(BLACK);
    }

    dt = (micros() - preTime) / 1000000; // 処理時間を求める
    preTime = micros(); // 処理時間を記録
    loopfreq = 1/dt;

    myGetAhrsData(&pitch, &roll, &yaw, loopfreq);

//    Serial.printf("%8.1f,%8.1f,%8.1f\n", roll, pitch, yaw);
    drawScreen();
//    delay(1);

    if (!started) 
    {
        return;
    }
    now = target - roll;
    
    if (abs(now) > 40) {
        // +-40度を超えたら倒れたとみなす
        power = 0;
        wheelA(0);
        wheelB(0);
        return;
    }

    // 目標角度から現在の角度を引いて偏差を求める
    // pRatioで割ることでKp,Ki,Kdの量を調整できる
    P = (target - roll) / pRatio;
    // 偏差を積分する
    I += P * dt;
    // 偏差を微分する
    D = (P - preP) / dt;
    // 偏差を記録する
    preP = P;
    // 積分部分が大きくなりすぎると出力が飽和するので大きくなり過ぎたら0に戻す(アンチワインドアップ)
    if (200 < abs(I * Ki)) {
        I = 0;
        led = !led;
        digitalWrite(GPIO_NUM_10, led ? LOW : HIGH);
        // anti windup
    }
    // 出力を計算する
    power = Kp*P + Ki * I + Kd * D;

    power += 9 * (power / abs(power)); // add minimum PWM duty cycle
    
    power = constrain(power, -100, 100);
    // モーターの出力を決定する
    
    wheelA(-power);
    wheelB(power);

}
