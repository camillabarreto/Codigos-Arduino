/*
  Sistema INPUT/OUTPUT
*/

#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <timers.h>
#include <semphr.h>

#define period 100 //Período do bit
#define my_mac B1101 //Meu valor de MAC
#define interruptPin 2 //Porta de recepção de dados
#define outputPin 5 //Porta de transmissão de dados
#define stx B10; //start of text
#define etx B11; //end of text
#define bcc B10000000; //byte de redundância

SemaphoreHandle_t free_input = NULL; //Semáforo utilizado para liberar a TASK INPUT
SemaphoreHandle_t free_output = NULL; //Semáforo utilizado para liberar a TASK OUTPUT
SemaphoreHandle_t free_interrupt = NULL; //Semáforo utilizado para liberar a TASK INT0
TimerHandle_t output_timer = NULL; //Software Timer utilizado para sinalizar a TASK OUTPUT
TimerHandle_t input_timer = NULL; //Software Timer utilizado para sinalizar a TASK INPUT
TimerHandle_t synchronize_timer = NULL; //Software Timer utilizado para sincronizar o início da leitura


//Struct dos quadros
struct frame{
  byte mac_dest;
  byte mac_font;
  byte port_dest;
  byte port_font;
  byte data[10];
};

//QueueHandle_t RxQueue = xQueueCreate(10, sizeof(frame)); //Fila de quadros cujo o destino é o meu endereço
//QueueHandle_t FwQueue = xQueueCreate(10, sizeof(frame)); //Fila os quadros cuja o destino não é o meu e que serão restransmitidos pela task emissora
QueueHandle_t TxQueue = xQueueCreate(3, sizeof(frame)); //Fila dos dados provindos de outros aplicações


/*
  Função responsável por enviar um byte em uma sequência pré-determinada
  1. Start bit
  2. Byte (data)
  3. End bit (x3)
*/
void enviarByte(byte data){
  byte control = B10000000; //byte de controle
  
  xTimerReset(output_timer, 0);
  
  //STX - Start bit
  digitalWrite(outputPin, LOW);
  
  //DADOS
  for(int i = 0; i<8; i++){
    xSemaphoreTake(free_output, portMAX_DELAY);
    if(data & control){
      digitalWrite(outputPin, HIGH);
    }else{
      digitalWrite(outputPin, LOW);
    }
    control = control>>1;
  }
  
  //ETX - End bit
  xSemaphoreTake(free_output, portMAX_DELAY);
  digitalWrite(outputPin, HIGH);
  xSemaphoreTake(free_output, portMAX_DELAY);
  xSemaphoreTake(free_output, portMAX_DELAY);
}

byte lerByte(){
  byte control = B10000000; //byte de controle
  byte data = B00000000;
  for (int i=0; i<8; i++) {
    xSemaphoreTake(free_input, portMAX_DELAY);
    if(digitalRead(interruptPin) == HIGH){
      data = data | control;
      Serial.print("1");
    }else Serial.print("0");
    control = control>>1;
  }
  xSemaphoreTake(free_input, portMAX_DELAY);
  xTimerStop(input_timer, 0);
  return data;
}

/*
  Timer responsável por liberar a TASK OUTPUT
  Tipo: Auto-reload
*/
void output_timer_function(TimerHandle_t timer) {
  //semáforo free_output
  xSemaphoreGive(free_output);
}

/*
  Timer responsável por liberar a TASK INPUT
  Tipo: Auto-reload
*/
void input_timer_function(TimerHandle_t timer) {
  //semáforo free_input
  xSemaphoreGive(free_input);
}

/*
  Timer responsável por sincronizar o início da leitura INPUT
  Tipo: One-shot
*/
void synchronize_timer_function(TimerHandle_t timer) {
  //start input_timer
  if(xTimerStart(input_timer, 0) != pdPASS) Serial.println("FALHA: xTimerStart()");
}

/*
  Tarefa responsável por ler a entrada de dados e tratá-los
*/
void Task_input(void *pvParameters) {
  byte data = B00000000; //byte onde os dados serão armazenados
  byte control = B10000000; //byte de controle  
  byte quadro[6]; //STX, MACS, PORTS, DATA, BCC, ETX
  
  for(int i = 0; i<6; i++){
    xSemaphoreTake(free_interrupt, portMAX_DELAY); //aguardar interrupção
    xTimerReset(synchronize_timer, 0); //iniciar timer de sincronização
    quadro[i] = lerByte(); //leitura do byte
    attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, LOW); //habilitar interrupção
  }
  xTimerStop(input_timer, 0); //parar o timer de leitura

  //dados recebidos devem ser: 2, 220, 13, 170, 128, 3
  Serial.println("\n\n------------------------");
  Serial.print("STX : ");
  Serial.println(quadro[0]);
  Serial.print("MAC DEST : ");
  Serial.println(quadro[1]>>4);
  Serial.print("PORT : ");
  Serial.println(quadro[2]);
  Serial.print("DATA : ");
  char a = quadro[3];
  Serial.println(a);
  Serial.println(quadro[3], DEC);
  Serial.print("BCC : ");
  Serial.println(quadro[4]);
  Serial.print("ETX : ");
  Serial.println(quadro[5]);


  //verificar se o MAC dest é o mesmo que o meu
  if((quadro[1]>>4) ^ my_mac){
    Serial.println("\n\nNao eh meu MAC");
    //então envia para a fila da transmissão
  }else{
    Serial.println("\n\nEh meu MAC");
    //então envia para a fila da tarefa
  }
  
  for(;;){
    //não faz nada
  }
}

/*
  Tarefa responsável por enviar os dados
*/
void Task_output(void *pvParameters) {
  byte quadro[6];
  byte control = B10000000; //byte de controle
  struct frame f;
  
  //Aguarda frame da fila
  while (xQueueReceive(TxQueue, &f, 0) == errQUEUE_EMPTY) {}
  
  //Construindo quadro
  quadro[0] = stx;
  quadro[1] = (f.mac_dest<<4) | f.mac_font; //unindo os MACS dest e font
  quadro[2] = (f.port_dest<<4) | f.port_font; //unindo as PORTS dest e font
  quadro[3] = f.data[0];
  quadro[4] = bcc;
  quadro[5] = etx;
  
  //Iniciando a transmissão dos bytes
  for(int i = 0; i<6; i++){
    enviarByte(quadro[i]);
  }

  //Desabilita o output_timer
  xTimerStop(output_timer, 0);
  
  for(;;){  
    //não faz nada
  }
}

/*
  Aplicação
*/
void Task_sendMessage(void *pvParameters) {
  char userInput;
  int i = 0;
  byte control = B10000000; //byte de controle
  struct frame f;

  Serial.print("Entre com a mensagem: ");

  for (;;) {
    while (!Serial.available()) {}
    userInput = Serial.read();
    Serial.println(userInput);
    if (userInput != '\n') {
      f.data[i] = userInput;
    } else {
      f.data[i] = 0;
      break;
    }
    i++;
    delay(10);
  }

  f.mac_dest = B1101; //endereço de destino
  f.mac_font = B1100; //endereço de fonte
  f.port_dest = B1101; //porte de destino
  f.port_font = B1100;

  xQueueSendToBack(TxQueue, &f, 0);
  for (;;) {}

}

/*
  Sinaliza a Task_input e desativa a interrupção
*/
void interrupt( void ) {
  Serial.println("\ninterrupt");
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  xSemaphoreGiveFromISR(free_interrupt, &xHigherPriorityTaskWoken );
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}
  
  Serial.println("INICIANDO");
  //Criar os 3 timers: input_timer, output_timer e synchronize_timer
  //Criar os 3 semáforos: free_input, free_output e free_interrupt
  //Criar as 2 tarefas: Task_input e Task_output
  //Configurar os 2 pinos: inputPin e interruptPin 
  //Ativar interrupção
  //Start no output_timer

  output_timer = xTimerCreate("output_timer", period / portTICK_PERIOD_MS, pdTRUE, 0, output_timer_function);
  input_timer = xTimerCreate("input_timer", period / portTICK_PERIOD_MS, pdTRUE, 0, input_timer_function);
  synchronize_timer = xTimerCreate("synchronize_timer", (period/2)/ portTICK_PERIOD_MS, pdFALSE, 0, synchronize_timer_function);

  free_input = xSemaphoreCreateBinary();
  free_output = xSemaphoreCreateBinary();
  free_interrupt = xSemaphoreCreateBinary();

  xTaskCreate(Task_input, "Task_input", 128, NULL, 1, NULL);
  xTaskCreate(Task_output, "Task_output", 128, NULL, 1, NULL);
  xTaskCreate(Task_sendMessage, "Task_sendMessage", 100, NULL, 1, NULL);
  
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, HIGH);
  
  pinMode(interruptPin, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, LOW);
}

void loop() {
  //Não faz nada
}
