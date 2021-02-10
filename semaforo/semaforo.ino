/*
  Descrição 01: Faz muito tempo que não brinco com arduino... 
  então vamos ver o que eu consigo fazer kkk
*/

#define LED_vermelho 12
#define LED_amarelo 11
#define LED_verde 10

/* Configurando as portas */
void setup() {
  pinMode(LED_vermelho, OUTPUT);
  pinMode(LED_amarelo, OUTPUT);
  pinMode(LED_verde, OUTPUT);
  digitalWrite(LED_verde, LOW);
  digitalWrite(LED_amarelo, LOW);
}

void loop() {
  digitalWrite(LED_verde, HIGH);
  digitalWrite(LED_vermelho, LOW);
  delay(3000);
  digitalWrite(LED_verde, LOW);
  digitalWrite(LED_amarelo, HIGH);
  delay(1000);
  digitalWrite(LED_amarelo, LOW);
  digitalWrite(LED_vermelho, HIGH);
  delay(3000);
}

