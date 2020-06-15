#include <M5StickC.h>

float Kp = 10.0;
float Ki = 0.0;
float Kd = 0.0;
float target = -98.9;
float P, I, D, preP;

float power = 0;
float dt, preTime;

bool started = false;

float roll, pitch, yaw;
float now;
bool led = false;

static float pRatio = 15.0;

void drawScreen() {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("S:%6.1f\n", target);
    M5.Lcd.printf("roll:%6.1f\n", roll);
    M5.Lcd.printf("now:%4.1f\n", target - roll);
    M5.Lcd.printf("Power:%5.1f\n", power);
    M5.Lcd.printf("Kp:%5.1f\n", Kp);
    M5.Lcd.printf("Ki:%5.1f\n", Ki);
    M5.Lcd.printf("Kd:%6.3f\n", Kd);
    M5.Lcd.printf("Bat:%5.1fV\n", M5.Axp.GetBatVoltage());
    M5.Lcd.printf("Cur:%5.1f\n", M5.Axp.GetBatCurrent());
}

void wheel(uint8_t val) {
    Wire.beginTransmission(0x38);
    Wire.write(0x00);
    Wire.write(-val);
    Wire.endTransmission();
    Wire.beginTransmission(0x38);
    Wire.write(0x01);
    Wire.write(val);
    Wire.endTransmission();
}

void setup() {
    // put your setup code here, to run once:
    M5.begin();
    Wire.begin(0, 26);
    M5.Lcd.setRotation(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Axp.ScreenBreath(8);
    M5.MPU6886.Init();
    wheel(0);
    pinMode(GPIO_NUM_10, OUTPUT);
    digitalWrite(GPIO_NUM_10, HIGH);

    drawScreen();
}

void loop() {

    M5.update();
    if (M5.BtnB.wasReleased()) {
        // ボタンBを押すと再起動
        esp_restart();
    }

    if (M5.BtnA.wasReleased()) {
        // BtnAを押して3秒後にモーター駆動開始
        Serial.println("BtnA.wasReleased() == TRUE");

        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("3");
        delay(1000);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("2");
        delay(1000);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("1");
        // ボタンを押したら直立させて待ち残り1秒になったら現在の角度を基準に設定する
        float temp = 0.0;
        for (int i = 0; i < 10; i++) {
            M5.MPU6886.getAhrsData(&pitch, &roll, &yaw);
            temp += roll;
            delay(10);
        }
        target = temp / 10.0;
        delay(800);
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.print("0");
        preTime = micros();
        started = true;
    }
    M5.MPU6886.getAhrsData(&pitch, &roll, &yaw);

    Serial.printf("%8.1f,%8.1f,%8.1f\n", roll, pitch, yaw);
    drawScreen();
    delay(1);
    if (!started) {
        return;
    }
    now = target - roll;
    dt = (micros() - preTime) / 1000000; // 処理時間を求める
    preTime = micros(); // 処理時間を記録

    if (-1 < now && now < 1) {
        // +-1以内は制御しない
        power = 0;
        P = 0;
               // Iを0にすると慣性力で倒れるのでIのみ維持する
               //I = 0;
        D = 0;
        wheel(0);
        return;
    }
    if (now < -40 || 40 < now) {
        // +-40度を超えたら倒れたとみなす
        power = 0;
        wheel(power);
        P = 0;
        I = 0;
        D = 0;
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
    power = Kp * P + Ki * I + Kd * D;
    power = constrain(power, -100, 100);
    // モーターの出力を決定する
    wheel(power);

}

