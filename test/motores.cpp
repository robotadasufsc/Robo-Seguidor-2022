#include <Arduino.h>

#define PWMA 19
#define PWMB 21
#define esq1 2
#define esq2 4
#define dir1 16
#define dir2 17

void setup(){
    Serial.begin(1115200);
    pinMode(esq1, OUTPUT);
    pinMode(esq2, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);

    // config ponte H
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    ledcSetup(0, 5000, 12); // canal para esquerdo
    ledcSetup(1, 5000, 12); // canal para o direito
    ledcAttachPin(PWMA, 1);
    ledcAttachPin(PWMB, 0);
}

void loop(){

    digitalWrite(esq1, LOW);
    digitalWrite(esq2, HIGH);
    digitalWrite(dir1, LOW);
    digitalWrite(dir2, HIGH);

    for(int i = 0; i < 4095; i++){
        ledcWrite(0, i);
        ledcWrite(1, i);
        delay(5);
    }

    ledcWrite(0, 0);
    ledcWrite(1, 0);

    delay(1000);

    digitalWrite(esq1, HIGH);
    digitalWrite(esq2, LOW);
    digitalWrite(dir1, HIGH);
    digitalWrite(dir2, LOW);

    for(int i = 0; i < 4095; i ++){
        ledcWrite(0, i);
        ledcWrite(1, i);
        delay(5);
    }
}