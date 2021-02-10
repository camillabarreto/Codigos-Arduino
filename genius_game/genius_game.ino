/*
  Genius game: Sequence memory (3-color version: red, green and yellow)
  Start Date:   13/01/2021
  Last Update:  20/01/2021
*/

/* setting the pinout */
#define LED_RED 12
#define LED_YELLOW 11
#define LED_GREEN 10
#define BUT_RED 7
#define BUT_YELLOW 6
#define BUT_GREEN 5
#define BUZZER 9
#define SOUND false

/* defining the global variables */
byte step = 1;
byte* seq = NULL;
byte leds[] = {LED_GREEN, LED_YELLOW, LED_RED};
byte nColors = sizeof(leds)/sizeof(leds[0]);
byte freq[] = {430, 480, 530};

void setup() {
  Serial.begin(9600);
  
  /* Se o pino de entrada analógica 0 é deixado desconectado,
  o ruído aleatório analógico irá causar a chamada de randomSeed()
  gerar sementes aleatórias diferentes cada vez que o sketch roda.
  randomSeed() basicamente "embaralha" a função random(). */
  randomSeed(analogRead(0));
  
  /* configuring the ports */
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  
  pinMode(BUT_RED, INPUT);
  pinMode(BUT_YELLOW, INPUT);
  pinMode(BUT_GREEN, INPUT);

  pinMode(BUZZER, OUTPUT);
}

void begin (){
  //display: iniciando
  //pisca os leds em sequencia
  //display: pronto?
  //apaga os leds
}

void sequence() {
  
  int randNumber = random(nColors); //número aleatório entre 0 e nColors-1
  // Serial.print("numero aleatorio: ");
  // Serial.println(randNumber);
  
  byte old [step];
  if(seq != NULL){
    for (byte i = 0; i < step; i++) {
      old[i] = seq[i];
    }  
  }
  
  delete [] seq;
  seq = new byte [step];
  
  if(old != NULL){
    for (byte i = 0; i < step-1; i++) {
      seq[i] = old[i];
    }  
  }
  seq[step-1] = randNumber;
  
  
  for (byte i = 0; i < step; i++) {
    delay(500);
    digitalWrite(leds[seq[i]], HIGH);
    if(SOUND){
      tone(BUZZER, freq[seq[i]], 200);
    }
    delay(500);
    digitalWrite(leds[seq[i]], LOW);
  }
  
}

bool play() {
  byte playSeq[step];
  byte greenState = digitalRead(BUT_GREEN);
  byte yellowState = digitalRead(BUT_YELLOW);
  byte redState = digitalRead(BUT_RED);
  bool rigth = true;
  
  for (byte i = 0; i < step; i++) {
    while(!(redState == HIGH) and !(yellowState == HIGH) and !(greenState == HIGH)){
      redState = digitalRead(BUT_RED);
      yellowState = digitalRead(BUT_YELLOW);
      greenState = digitalRead(BUT_GREEN);
    }
    
    if (redState == HIGH){
      Serial.println("acendeu vermelho");
      playSeq[i] = 2;
      digitalWrite(leds[2], HIGH);
      delay(250);
      digitalWrite(leds[2], LOW);
    }
    else if (yellowState == HIGH){
      Serial.println("acendeu amarelo");
      playSeq[i] = 1;
      digitalWrite(leds[1], HIGH);
      delay(250);
      digitalWrite(leds[1], LOW);
    }
    else if (greenState == HIGH){
      Serial.println("acendeu verde");
      playSeq[i] = 0;
      digitalWrite(leds[0], HIGH);
      delay(250);
      digitalWrite(leds[0], LOW);
    }
    
    if(playSeq[i] != seq[i]){
      rigth = false;
      break;
    }
    
    while((redState == HIGH) or (yellowState == HIGH) or (greenState == HIGH)){
      redState = digitalRead(BUT_RED);
      yellowState = digitalRead(BUT_YELLOW);
      greenState = digitalRead(BUT_GREEN);
    }
  }
  Serial.println("OK");
  
  //confere
  return rigth;
}

void loop() {
  
  Serial.println("COMEÇOU");
  // begin();
  step = 1;
  seq = NULL;
  bool rigth = true;
  while(rigth){
    delay(1500);
    Serial.print("STEP: ");
    Serial.println(step);
    sequence();
    rigth = play();
    step++;
    
  }
  Serial.println("ERROU");
  digitalWrite(leds[0], HIGH);
  digitalWrite(leds[1], HIGH);
  digitalWrite(leds[2], HIGH);
  delay(2000);
  digitalWrite(leds[0], LOW);
  digitalWrite(leds[1], LOW);
  digitalWrite(leds[2], LOW);
  delay(250);
  digitalWrite(leds[0], HIGH);
  digitalWrite(leds[1], HIGH);
  digitalWrite(leds[2], HIGH);
  delay(250);
  digitalWrite(leds[0], LOW);
  digitalWrite(leds[1], LOW);
  digitalWrite(leds[2], LOW);

}
