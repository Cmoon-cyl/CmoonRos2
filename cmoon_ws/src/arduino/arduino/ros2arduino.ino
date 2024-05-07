#include <ArduinoJson.h>

int enA = 5;
int in1 = 4;
int in2 = 3;
char mode[10] = "s";
int speed = 0;
int speedL=0;
int speedR=0;

// 初始化串行通信的数据速率
void setup() {
  Serial.begin(9600);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void loop() {
  StaticJsonDocument<200> jsondata;
  // 检查是否有可用的串行数据
  if (Serial.available()) {
    // 从串行接口读取并解析JSON
    DeserializationError error = deserializeJson(jsondata, Serial);

    // 如果解析失败，打印错误并返回
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
      return;
    }


    Serial.println("接收到: ");
    if (jsondata.containsKey("mode")) {
      strcpy(mode, jsondata["mode"]);
      Serial.print("mode:");
      Serial.println(mode);
    }
    if (jsondata.containsKey("speed")) {
      speed = jsondata["speed"];
      Serial.print("speed:");
      Serial.println(speed);
    }
    if (jsondata.containsKey("speedL")) {
      speedL = jsondata["speedL"];
      Serial.print("speedL:");
      Serial.println(speedL);
    }
    if (jsondata.containsKey("speedR")) {
      speedR = jsondata["speedR"];
      Serial.print("speedR:");
      Serial.println(speedR);
    }

    if (strcmp(mode, "s") == 0) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      log();
    } else if (strcmp(mode, "l") == 0) {
      analogWrite(enA, speed);
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      log();
    } else if (strcmp(mode, "r") == 0) {
      analogWrite(enA, speed);
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      log();
    } else {
      Serial.println("no mode or unsupported mode");
    }
  }
}

void log() {
  Serial.print("set => mode:");
  Serial.print(mode);
  Serial.print("  speed:");
  Serial.println(speed);
}
