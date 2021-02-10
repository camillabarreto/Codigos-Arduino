/*
* 2º Exemplo: ApagaLED
*   Criar um Software Timer do tipo one-shot com período de 10 segundos.
*   Sua função callback vai apagar um LED (inicialmente aceso).
*   Criar uma interrupção que é chamada ao pressionar um botão.
*   A tarefa da interrupção acende o LED e reinicia o Software Timer.
* Conclusão:
*   Se o botão for pressionado seguidamente em intervalos menores que 10 segundos o LED permanece aceso.
*   Se o botão não for pressionado por um intervalo de 10 segundos o LED será desligado.
*   Quando o LED estiver apagado e o botão for pressionado, o LED acende e o timer é reiniciado.
*/

#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <timers.h>
#include <semphr.h>

const byte porta = 5; //Porta onde o LED está conectado
const byte interruptPin = 2; //Porta onde o botão está conectado
SemaphoreHandle_t semaforo = NULL; //Semáforo utilizado para sinalizar a Tarefa de Interrupção
TimerHandle_t timer = NULL; //Software Timer utilizado para controlar tempo do LED ligado

/*
* Software Timer Callback Function: no estouro do Software Timer essa função será executada
*/
static void apagaLED(TimerHandle_t temp) {
  Serial.println("Apagando LED");
  digitalWrite(porta, LOW);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  
  //Configurando porta 5 como saída para acender o LED
  pinMode(porta, OUTPUT);
  digitalWrite(porta, HIGH);
  
  //Criando Semáforo Mutex
  semaforo = xSemaphoreCreateBinary();
  
  //Criando Tarefa para a Interrupção INT0
  xTaskCreate(TaskINT0, (const portCHAR *)"Task da INT0", 128, NULL, 1, NULL); 
  
  //Criando Software Timer do tipo One Shot com período de 10 segundos
  timer = xTimerCreate("Timer_1", 5000 / portTICK_PERIOD_MS, pdFALSE, 0, apagaLED);

  if (timer != NULL) {
    //Iniciando o Software Timer
    BaseType_t timer_start = xTimerStart(timer, 0);
    if (timer_start != pdPASS) {
      Serial.println("FALHA: xTimerStart()");
    }
  }else Serial.println("FALHA: xTimerCreate()");
  
  //Ativando resistor na porta da Interrupção
  pinMode(interruptPin, INPUT_PULLUP);
  
  //Ativando Interrupção na porta 2
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, LOW);
}

void loop() {
  //Não faz nada
}

/*
*Task: Quando ocorrer uma Interrupção essa Tarefa será executada
*/
void TaskINT0(void *pvParameters){
  for (;;){
    //Aguardando sinalização no Semáforo
    xSemaphoreTake(semaforo, portMAX_DELAY);
    
    //Acendendo LED
    digitalWrite(porta, HIGH);
    
    //Reiniciando Software Timer
    BaseType_t timer_reset = xTimerReset(timer, 10 / portTICK_PERIOD_MS);
    if(timer_reset != pdPASS){
      Serial.println("FALHA: xTimerReset()");
    }else Serial.println("SUCESSO: xTimerReset()");
  }
}

void interrupt( void ){
  BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdFALSE;
  //Enviando sinalizaça no Semáforo para acionar a TarefaTaskINT0
  xSemaphoreGiveFromISR( semaforo, &xHigherPriorityTaskWoken );
}
