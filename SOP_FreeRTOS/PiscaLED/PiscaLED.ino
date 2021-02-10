/*
* 1º Exemplo: PiscaLED
*   Criar um Software Timer do tipo auto-reload com período de 2 segundos. 
*   Sua função callback é chamada a cada 2 segundos e troca .
* Conclusão: 
*   O LED ficará alternando automaticamente entre desligado (2 segundos) e ligado (2 segundos).
*/

#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <timers.h>

const byte porta = 5; //Porta onde o LED está conectado - Ativo alto

/*
* Software Timer Callback Function: no estouro do Software Timer essa função será executada
*/
static void acendeLED(TimerHandle_t temp) {
  //Se LED está apagado, então deve ser acender
  //Se LED está aceso, então deve ser apagar
  if (digitalRead(porta) == HIGH) {
    digitalWrite(porta, LOW);
  }
  else {
    digitalWrite(porta, HIGH);
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  
  //Configurando porta 5 como saída para controlar o LED
  pinMode(porta, OUTPUT);
  
  //Cria Software Timer do tipo Auto Reload com período de 2 segundos
  TimerHandle_t timer = xTimerCreate("Timer_1", 2000 / portTICK_PERIOD_MS, pdTRUE, 0, acendeLED);

  if (timer != NULL){ 
    //Iniciando o Software Timer
    BaseType_t timer_start = xTimerStart(timer, 0);
    if (timer_start == pdPASS) {
      Serial.println("SUCESSO: xTimerStart()");
    }
  }
  else Serial.println("FALHA: xTimerStart()");

}

void loop() {
  //Não faz nada
}
