#include <Arduino.h>

const uint8_t qtdSensores = 8;
const int pinSensores[] = {39, 34, 35, 32, 33, 25, 26, 27};

long int lastPosition;                                          // ultima posição do sensor em relação a linha 
int preto = 100, branco = 0;                                    // valores que vamos usar para 0 e 4095 lidos dos sensores
int max_[] = {0, 0, 0, 0, 0, 0, 0, 0};                          // valores maximos lidos na calibração
int min_[] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};  // valores minimos lidos na calibração
int pesos[] = {-400, -300, -200, -100, 100, 200, 300, 400};     // peso de cada sensor usado para calcular media ponderada

/*
    não achei uma forma boa de calcular pois quando mexemos no carrinho,
    um univo ponto em 4095 acaba deixando o valor do preto muito alto.
    isso acaba deixando a calibração imprecisa.
*/
void getMaxAndMin(){
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
}

// calcula a media de 10 medidas na parte escura da pista
void getMax(){
    for(int i = 0; i < qtdSensores; i++){
        int m = 0;
        for(int j = 0; j < 10; j++){
            m += analogRead(pinSensores[i]);
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
        }
        min_[i] = m/10; // salva essa media para cada sensor
    }
}

/*
    Tira a medida do quanto cada sensor recebe na parte prata da pista,
    depois espera 2 segundos e mede quando cada um recebe de branco. 
*/
void getMaxAndMinOneTime(){
    digitalWrite(2, HIGH);
    Serial.println("Medindo a itensidade da luz para o preto...");
    delay(2000);
    digitalWrite(2, LOW);
    getMax();
    delay(100);
    digitalWrite(2, HIGH);
    Serial.println("Medindo a itensidade da luz para o branco...");
    delay(2000);
    digitalWrite(2, LOW);
    getMin();

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
    le quanto cada sensor esta medindo, e conforme os valores
    maximos e minimos salvos deixa todos na mesma escala
*/
int read(int i, bool lineWhite){
    int value = analogRead(pinSensores[i]);

    if(lineWhite){
        value = map(value, min_[i], max_[i], branco, preto);
    }else{
        value = map(value, min_[i], max_[i], preto, branco);
    }

    // garante que nenhum valor vai estrapolar os limites
    if (value < branco)
        value = branco;

    if (value > preto)
        value = preto;

    return value;
}

/*
    calcula a media ponderada das medidas para saber
    onde a linha esta
*/
int searchLine(){
    long int posicao = 0, p = 0;
    for(int i = 0; i < qtdSensores; i++){
        int x = read(i, false);
        posicao += x * pesos[i];
        p += x;
    }
    if(abs(p) > 30){
        lastPosition = posicao/p;
    }else{
        // caso o sensor nao detecte a linha 
        // ele satura no lado onde a linha foi vista por utlimo
        if(lastPosition > 0){
            lastPosition = pesos[7];
        }else{
            lastPosition = pesos[0];
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

    getMaxAndMinOneTime();
}

void loop(){
    Serial.print(searchLine());
    Serial.println();
    delay(5);
}