#include <Arduino.h>
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t qtdSensores = 8;
uint16_t sensorValues[qtdSensores];

void setup() {
    Serial.begin(115200);
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){39, 34, 35, 32, 33, 25, 26, 27}, qtdSensores);
    qtr.setEmitterPin(14);

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH); // turn off Arduino's LED to indicate we are through with calibration
    for (uint16_t i = 0; i < 100; i++){
        qtr.calibrate();
    }
    digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

    // print the calibration minimum values measured when emitters were on
    for (uint8_t i = 0; i < qtdSensores; i++){
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    // print the calibration maximum values measured when emitters were on
    for (uint8_t i = 0; i < qtdSensores; i++){
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(' ');
    }
    Serial.println();
    Serial.println();
    delay(1000);
}

void loop(){
    // read calibrated sensor values and obtain a measure of the line position
    // from 0 to 5000 (for a white line, use readLineWhite() instead)
    uint16_t position = qtr.readLineWhite(sensorValues);

    // print the sensor values as numbers from 0 to 1/000, where 0 means maximum
    // reflectance and 1000 means minimum reflectance, followed by the line
    // position
    for (uint8_t i = 0; i < qtdSensores; i++){
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println(position);

    delay(250);
}