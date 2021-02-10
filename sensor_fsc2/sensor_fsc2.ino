void setup() {
  Serial.begin(9600);
}

void loop() {
  float s0 = analogRead(A0),t0, v0; 
  float s2 = analogRead(A2), t2, v2;
    
  v0 = s0 * (5.0/1023); //Tensão do sensor 0
  t0 = v0 * 100; //Temperatura do sensor 0

  v2 = s2 * (5.0/1023); //Tensão do sensor 2
  t2 = v2 * 100; //Temperatura do sensor 2

  Serial.print("T0: ");
  Serial.print(t0);
  Serial.print("C     /     ");

 
  Serial.print("T2: ");
  Serial.print(t2);
  Serial.print("C \n");
 
  delay(2000);
}

