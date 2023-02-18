#include <Arduino.h>
#include <QTRSensors.h>
#include <BluetoothSerial.h>

QTRSensors qtr;
BluetoothSerial SerialBT;

// multtasks
TaskHandle_t observer;

// constantes do PID
float kp = 170,   // ganho proporcional
ki = 0.065,   	// ganho integral
kd = 470;     	// ganho derivativo

// pinos da ponte H
#define PWMA 19
#define PWMB 21
#define esq1 2
#define esq2 4
#define dir1 16
#define dir2 17

// velocidade do carrinho
bool largada = false;
int32_t speed = 3500;

// velocidade dos motores esquerdo e direito
int32_t velEsq = speed, velDir = speed;

// constante para ajustar quanto a roda gira pra trás nas curvas mais fechadas
float cte = 0.7;

// tempo em que a ultima intercessão ocorreu
unsigned long int t_intercessao;

// config da linha de sensores
const uint8_t qtdSensores = 8;
uint16_t sensorValues[qtdSensores];

// variaveis para o PID
int P, I, D, erroPassado, erroSomado = 0, erro, PID;

// variaveis para execoes
#define botao 18        	// pino do botão
#define sensoresquerdo 13   // pino do sensor esquerdo
#define sensordireito 36	// pino do sensor direito
#define intercessao 4000	// se a soma do valor dos sensores for maior do que esse valor o robô considera uma intercessão
int total;              	// valor da soma dos sensores
int volta = 0;          	// define quantas voltas o robo deu na pista

// codigo para alterar o valor das constantes(kp,ki,kd) pelo celular
String texto = "";
void recebeDados(){
  // recebe as chars e soma em um texto
  char a = SerialBT.read();
  texto += a;

  // separa as constantes quando recebe o texto todo
  if(a == '}'){
	// muda as constantes
	kp = (texto.substring(1, texto.indexOf('/'))).toFloat();
	// kp1 = (texto.substring(texto.indexOf('A')+1, texto.indexOf('/'))).toFloat();

	ki = (texto.substring(texto.indexOf('/')+1, texto.indexOf('%'))).toFloat();
	// ki1 = (texto.substring(texto.indexOf('B')+1, texto.indexOf('%'))).toFloat();

	kd = (texto.substring(texto.indexOf('%')+1, texto.indexOf('&'))).toFloat();
	// kd1 = (texto.substring(texto.indexOf('C')+1, texto.indexOf('&'))).toFloat();

	cte = (texto.substring(texto.indexOf('&')+1, texto.indexOf('*'))).toFloat();
	speed = (texto.substring(texto.indexOf('*')+1, texto.indexOf('}'))).toFloat();

	// printa os valores
	Serial.print(kp); 	Serial.print("\t");
	// Serial.print(kp1); 	Serial.print("\t");

	Serial.print(ki); 	Serial.print("\t");
	// Serial.print(ki1); 	Serial.print("\t");

	Serial.print(kd); 	Serial.print("\t");
	// Serial.print(kd1); 	Serial.print("\t");

	Serial.print(cte); 	Serial.print("\t");
	Serial.print(speed);  Serial.print("\t");

	// limpa a variavel para o proximo loop
	// Serial.println(texto);
	texto = "";
  }
}

// codigo para enviar o valor das constantes para o celular
void enviaDados(void * pvParameters){
  for(;;){ // loop perpétuo
	// caso ele receba algum dado ele altera as constantes
	if(SerialBT.available()){
  	recebeDados();
	}else{
  	// senao ele envia os dados para montar o grafico
  	delay(150);
  	// caso queirm mudar oq ele envia mude aqui
  	SerialBT.printf("{%d}", PID);
	}
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Esp32 seguidor v1");

  pinMode(sensordireito, INPUT);
  pinMode(sensoresquerdo, INPUT);
  pinMode(botao, INPUT);
  pinMode(esq1, OUTPUT);
  pinMode(esq2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(2, OUTPUT);
  // configuração da linha de sensores
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){39, 34, 35, 32, 33, 25, 26, 27}, qtdSensores);
  qtr.setEmitterPin(14);

  // inicia as variaveis da biblioteca
  digitalWrite(2, HIGH); // turn off Arduino's LED to indicate we are through with calibration
  for (uint16_t i = 0; i < 100; i++){
    qtr.calibrate();
  }
  digitalWrite(2, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // config ponte H
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  ledcSetup(0, 5000, 12); // canal para esquerdo
  ledcSetup(1, 5000, 12); // canal para o direito
  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 0);

  // inicia a task bluetooth
  xTaskCreatePinnedToCore(
	enviaDados,   // Função da tarefa
	"taskBT", 	// nome da tarefa
	10000,    	// Tamanho (bytes)
	NULL,     	// parâmetro da tarefa
	2,        	// prioridade da tarefa (tem q ser 2 pra n dar conflito)
	&observer,	// observa a tarefa criada
	0);       	// nucleo
}

void calculaPID(){

  // define os valores do PID
  P = erro * kp;
  I = (erroSomado)* ki;
  D = ((erro - erroPassado)) * kd;

  // regula o valor máximo e mínimo que I pode assumir
  if((I>=(speed/2))&&(erro >=0)){
	I = (speed/2);
  }else if((I<=-(speed/2))&&(erro <=0)){
	I = -(speed/2);
  // calcula a soma de todos os erros anteriores (integral)
  }else{
	erroSomado += erro;
  }

  // calcula o PID
  PID = (P) + (I) + (D);

  // regula o valor máximo e mínimo que o PID pode assumir
  if(PID>speed*2){
	PID = speed*2;
  }else if(PID<(-speed*2)){
	PID = -speed*2;
  }

  // salva o erro passado
  erroPassado = erro;
}

void motors(){
  // controle de curva com PID
  if(PID > 0){
	velDir = speed - PID;
	velEsq = speed;
  }else{
	velEsq = speed + PID;
	velDir = speed;
  }

  // caso seja necessário dar a ré em uma das rodas
  if(velDir < 0){
	// esquerda
	digitalWrite(esq1, HIGH); // roda esquerda gira pra frente
	digitalWrite(esq2, LOW);

	// direita
	digitalWrite(dir1, LOW);  // roda direita gira pra traz
	digitalWrite(dir2, HIGH);

	velDir *= cte;
  }else if(velEsq < 0){
	// esquerda
	digitalWrite(esq1, LOW);  // roda esquerda gira pra traz
	digitalWrite(esq2, HIGH);

	// direita
	digitalWrite(dir1, HIGH); // roda direita gira pra frente
	digitalWrite(dir2, LOW);

	velEsq *= cte;
  }else{
	// esquerda
	digitalWrite(esq1, LOW);  // roda esquerda gira pra frente
	digitalWrite(esq2, HIGH);

	// direita
	digitalWrite(dir1, LOW);  // roda direita gira pra frente
	digitalWrite(dir2, HIGH);
  }
  // ajusta o valor do PWM para evitar a zona morta dos motores
  ledcWrite(1, map(abs(velEsq), 0, speed, 400, speed));
  ledcWrite(0, map(abs(velDir), 0, speed, 400, speed));
}

// caso o robo chegue na linha de largada ou chegada(sendor direito)
void Linha_de_chegada() {
  if(largada == false){
	while(digitalRead(botao) == 1){ // espera o botão ser pressionado para ligar o robô
  	ledcWrite(1, 0);
  	ledcWrite(0, 0);
	}
	delay(1000);
	largada = true;
  }
  if(largada == true){
	// caso N O seja intercessao
	if((digitalRead(sensordireito)==0) and (millis() > (t_intercessao+700))){
  	while(digitalRead(sensordireito) == 0){}// espera o robo sair de cima da linha
  	volta++;
	}
	// depois de passar pela linha de chegada
	if(volta == 2){
  	t_intercessao = millis();
  	volta++;
	}
	// espera um tempo para parar no final da pista (não pode parar em cima da linha)
	if((millis() >= t_intercessao + 100) and (volta == 3)){
  	volta = 0;
  	largada = false;
  	digitalWrite(2, HIGH);
	}
  }
}


void loop() {
  // calcula a posição da linha
  uint16_t position = qtr.readLineBlack(sensorValues);

  // soma os valores dos sensores
  total = 0;
  for (uint8_t i = 0; i < qtdSensores; i++) {
	total += sensorValues[i];
  }
 
  // caso haja intercessao
  if (!(total >= intercessao)){
	t_intercessao = millis();
  }
 
  // verifica as sinalizações da pista
  //Linha_de_chegada();

  // calcula o erro
  erro = (((int)position-3500)/100);
  Serial.println();

  // calcula o PID
  calculaPID();

  // controla a velocidade dos motores
  motors();
}