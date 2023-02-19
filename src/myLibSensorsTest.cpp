#include <Arduino.h>
#include <QTRSensors.h>

// biblitoeca pronta 
QTRSensors qtr;
uint16_t sensorValues[8];

// meus testes
const uint8_t qtdSensores = 8;
const int pinSensores[] = {39, 34, 35, 32, 33, 25, 26, 27};
long int lastPosition;                                              // ultima posição do sensor em relação a linha 
int linha = 1000, pista = 0;                                        // valores que vamos usar para 0 e 4095 lidos dos sensores
int tolLinha = 0.7 * linha;                                         // valor de tolerencia para saber se o sensor esta sob uma linha
int max_[] = {0, 0, 0, 0, 0, 0, 0, 0};                              // valores maximos lidos na calibração
int min_[] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};      // valores minimos lidos na calibração
int pesos[] = {3500, 2500, 1500, 500, -500, -1500, -2500, -3500};   // peso de cada sensor usado para calcular media ponderada

void isValid(){
    // verifica se as medidas fazem sentido
    for(int i = 0; i < qtdSensores; i++){
        if(min_[i] >= max_[i]){
            Serial.println("Erro. Refaça as medidas!");
            while(true){ // pisca o led para sinalizar o erro
                digitalWrite(2, HIGH);
                delay(250);
                digitalWrite(2, LOW);
                delay(250);
            }
        }
    }
}

/*
    não achei uma forma boa de calcular pois quando mexemos no carrinho,
    um univo ponto em 4095 acaba deixando o valor do preto muito alto.
    isso acaba deixando a calibração imprecisa.
*/
void setMaxAndMin(){
    // acende o led para sinalizar quando começa e quando acaba
    int start = millis();
    digitalWrite(2, HIGH);
    // durante 5 segundos salva as menores e maiores medidas lidas de cada sensor
    while((millis() - start) < 5000){
        for(int i = 0; i < qtdSensores; i++){
            int x = analogRead(pinSensores[i]);
            if(x > max_[i])
                max_[i] = x;
            if(x < min_[i])
                min_[i] = x;
        }
    }
    digitalWrite(2, LOW);

    isValid();
}

// calcula a media de 10 medidas na parte escura da pista
void getMax(){
    for(int i = 0; i < qtdSensores; i++){
        int m = 0;
        for(int j = 0; j < 10; j++){
            m += analogRead(pinSensores[i]);
            delay(5);
        }
        max_[i] = m/10; // salva essa media para cada sensor
    }
}

// calcula a media de 10 medidas na parte clara da pista
void getMin(){
    for(int i = 0; i < qtdSensores; i++){
        int m = 0;
        for(int j = 0; j < 10; j++){
            m += analogRead(pinSensores[i]);
            delay(5);
        }
        min_[i] = m/10; // salva essa media para cada sensor
    }
}

/*
    Tira a medida do quanto cada sensor recebe na parte preta da pista,
    depois espera 2 segundos e mede quanto cada um recebe de pista. 
*/
void setMaxAndMinOneTime(){
    digitalWrite(2, HIGH);
    Serial.println("Medindo a itensidade da luz para o preto linha...");
    delay(2000);
    digitalWrite(2, LOW);
    getMax();
    delay(100);
    digitalWrite(2, HIGH);
    Serial.println("Medindo a itensidade da luz para o branco pista...");
    delay(2000);
    digitalWrite(2, LOW);
    getMin();

    isValid();
    
}

/*
    le quanto cada sensor esta medindo, e conforme os valores
    maximos e minimos salvos deixa todos na mesma escala
*/
int read(int i, bool lineWhite){
    int value = analogRead(pinSensores[i]);

    if(lineWhite){
        value = map(value, min_[i], max_[i], pista, linha);
    }else{
        value = map(value, min_[i], max_[i], linha, pista);
    }

    // garante que nenhum valor vai estrapolar os limites
    if (value < pista)
        value = pista;

    if (value > linha)
        value = linha;

    return value;
}

/*
    calcula a media ponderada das medidas para saber
    onde a linha esta
*/
int searchLine(){
    long int sum = 0, p = 0;
    bool sobLinha = false;
    for(int i = 0; i < qtdSensores; i++){
        int x = read(i, false);
        sum += x * pesos[i];
        p += x;
        if(x > tolLinha) // pelo menos um sensor deve detectar a linha
            sobLinha = true;
    }
    if(sobLinha){
        lastPosition = sum/p;
    }else{
        // caso o sensor nao detecte a linha 
        // ele satura no lado onde a linha foi vista por utlimo
        if(abs(lastPosition - pesos[0]) < abs(lastPosition - pesos[7])){
            lastPosition = pesos[0];
        }else{
            lastPosition = pesos[7];
        }
    }
    return lastPosition;
}

void setup(){
    Serial.begin(115200);
    pinMode(2, OUTPUT);

    for(int i = 0; i < qtdSensores; i++){
        pinMode(pinSensores[i], INPUT);
    }

    // minha calibrcao
    setMaxAndMinOneTime();
    //setMaxAndMin();

    Serial.println("O resultado da medida foi:");

    Serial.print("Minimos: \t");
    for(int i = 0; i < qtdSensores; i++){
        Serial.print(min_[i]);
        Serial.print("\t");
    }
    Serial.println();
    Serial.print("Maximos: \t");
    for(int i = 0; i < qtdSensores; i++){
        Serial.print(max_[i]);
        Serial.print("\t"); 
    }
    Serial.println();

    delay(1000);

    // calibracao da biblioteca pronta
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){39, 34, 35, 32, 33, 25, 26, 27}, qtdSensores);
    qtr.setEmitterPin(14);

    digitalWrite(2, HIGH);
    for(int i = 0; i < 100; i++){
        qtr.calibrate();
    }
    digitalWrite(2, LOW);

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
}

void loop(){

    int m1, m2; 
    long int t1, t2;

    long int start = millis();
    m1 = 3500 - qtr.readLineWhite(sensorValues);
    t1 = millis() - start;

    start = millis();
    m2 = searchLine();
    t2 = millis() - start;

    // calculando a posicao da linha 
    Serial.printf("M1: %04d \t T1: %04d \t M2: %04d \t T2: %04d \t Erro: %04d \n", m1, t1, m2, t2, abs(m1-m2));
    delay(50);
}