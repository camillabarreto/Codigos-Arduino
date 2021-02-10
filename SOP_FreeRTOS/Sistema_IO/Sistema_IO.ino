/*
  Sistema INPUT/OUTPUT
  Discentes:  Camilla Barreto de Sousa
              Jeneffer Farias Bo
*/

#include <Arduino_FreeRTOS.h>
#include <FreeRTOSVariant.h>
#include <timers.h>
#include <semphr.h>

#define period 80 //Período do bit
#define my_mac B1101 //Meu valor de MAC
#define interruptPin 2 //Porta de recepção de dados
#define outputPin 5 //Porta de transmissão de dados
#define stx B10; //start of text
#define etx B11; //end of text
#define bcc B10000000; //byte de redundância

SemaphoreHandle_t free_interrupt = NULL; //Semáforo utilizado para liberar a TASK INPUT
SemaphoreHandle_t free_input = NULL; //Semáforo utilizado para liberar a leitura da TASK INPUT
SemaphoreHandle_t free_output = NULL; //Semáforo utilizado para liberar a TASK OUTPUT
TimerHandle_t output_timer = NULL; //Software Timer utilizado para sinalizar a TASK OUTPUT
TimerHandle_t input_timer = NULL; //Software Timer utilizado para sinalizar a TASK INPUT
TimerHandle_t synchronize_timer = NULL; //Software Timer utilizado para sincronizar o início da leitura da TASK INPUT


struct frame {
  byte mac_dest; //Mac destino
  byte mac_font; //Mac fonte
  byte port_dest; //Porta destino
  byte port_font; //Porta fonte
  byte data[10]; //Dados
};

QueueHandle_t RxQueue = xQueueCreate(1, sizeof(frame)); //Fila de frames cujo o destino é o meu endereço
QueueHandle_t FwQueue = xQueueCreate(1, sizeof(frame)); //Fila de frames cuja o destino não é o meu e que serão restransmitidos
QueueHandle_t TxQueue = xQueueCreate(1, sizeof(frame)); //Fila de frames provindos da aplicação

/*
  Timer responsável por liberar a escrita da TASK OUTPUT
  Tipo: Auto-reload
*/
void output_timer_function(TimerHandle_t timer) {
  xSemaphoreGive(free_output);
}

/*
  Timer responsável por sincronizar o início da leitura INPUT
  Tipo: One-shot
*/
void synchronize_timer_function(TimerHandle_t timer) {
  xTimerStart(input_timer, 0);
}

/*
  Timer responsável por liberar a leitura da TASK INPUT
  Tipo: Auto-reload
*/
void input_timer_function(TimerHandle_t timer) {
  xSemaphoreGive(free_input);
}

/*
  Função responsável por enviar um byte em uma sequência pré-determinada
  1. Start bit
  2. Byte (data)
  3. End bit (x3)
*/
void enviarByte(byte data) {
  byte control = B10000000; //byte de controle

  xTimerReset(output_timer, 0); //reiniciando timer de transmissão

  //Start bit
  digitalWrite(outputPin, LOW);

  //DADOS
  for (int i = 0; i < 8; i++) {
    xSemaphoreTake(free_output, portMAX_DELAY); //aguardando time da escrita
    if (data & control) {
      digitalWrite(outputPin, HIGH);
    } else {
      digitalWrite(outputPin, LOW);
    }
    control = control >> 1;
  }
  xSemaphoreTake(free_output, portMAX_DELAY);
  
  //Stop bit
  digitalWrite(outputPin, HIGH);
  xSemaphoreTake(free_output, portMAX_DELAY); 
  xSemaphoreTake(free_output, portMAX_DELAY);
  xSemaphoreTake(free_output, portMAX_DELAY);
  xTimerStop(output_timer, 0); //parando o timer de transmissão
}

/*
  Função responsável por ler o byte de dados recebido
  OBS: a leitura deve ser sincronizada para o "centro" do bit
*/
byte lerByte() {
  byte control = B10000000; //byte de controle
  byte data = B00000000; //dado será armazenado aqui
  xTimerReset(synchronize_timer, 0); //iniciando timer de sincronização
  for (int i = 0; i < 8; i++) {
    xSemaphoreTake(free_input, portMAX_DELAY); //aguardando time da leitura
    if (digitalRead(interruptPin) == HIGH) {
      data = data | control;
      //Serial.print("1");
    }//else Serial.print("0");
    control = control >> 1;
  }
  xSemaphoreTake(free_input, portMAX_DELAY);
  xTimerStop(input_timer, 0); //parando o timer de recepção
  attachInterrupt(digitalPinToInterrupt(interruptPin), interrupt, LOW); //habilitando a interrupção
  return data;
}

/*
  Tarefa responsável por ler a entrada de dados e tratá-los
*/
void Task_input(void *pvParameters) {
  byte data = B00000000; //byte onde os dados serão armazenados
  byte control = B10000000; //byte de controle
  byte quadro[15]; //STX, MACS, PORTS, DATA, BCC, ETX

  for (;;) {
    for (int i = 0; i < 15; i++) {
      xSemaphoreTake(free_interrupt, portMAX_DELAY); //aguardar interrupção
      quadro[i] = lerByte(); //leitura do byte
    }
    
    frame f;
    f.mac_dest = quadro[1] >> 4; //endereço de destino
    f.mac_font = B1100; //endereço de fonte
    f.port_dest = quadro[2] >> 4; //porte de destino
    f.port_font = B1100;
    for(int i = 0; i<10; i++){
      f.data[i] = quadro[i+3];
    }
    
    //verificar se o MAC dest é o mesmo que o meu
    if ((quadro[1] >> 4) ^ my_mac) {
      Serial.println("\nNao eh meu MAC");
      //então envia para a fila da transmissão
      xQueueSendToBack(FwQueue, &f, 0);
    } else {
      Serial.println("\nEh meu MAC");
      //então envia para a fila da tarefa
      xQueueSendToBack(RxQueue, &f, 0);
    }
  }
}

/*
  Tarefa responsável por enviar os dados provindos da aplicação
*/
void Task_output(void *pvParameters) {
  byte quadro[6]; //quadro será construido para o envio do frame
  byte control = B10000000; //byte de controle
  struct frame f; //frame recebido da TxQueue (aplicação) ou da FwQueue (recepção)

  for (;;) {
    //Aguardando frame de uma das filas
    while ((xQueueReceive(TxQueue, &f, 0) == errQUEUE_EMPTY) && (xQueueReceive(FwQueue, &f, 0) == errQUEUE_EMPTY)) {}

    quadro[0] = stx;
    quadro[1] = (f.mac_dest << 4) | f.mac_font; //unindo os MACS dest e font
    quadro[2] = (f.port_dest << 4) | f.port_font; //unindo as PORTS dest e font
    for(int i = 0; i<10; i++){
      quadro[i+3] = f.data[i];
    }
    quadro[13] = bcc;
    quadro[14] = etx;

    //Iniciando a transmissão dos bytes
    for (int i = 0; i < 15; i++) {
      enviarByte(quadro[i]);
    }
  }
}

/*
  Tarefa de Aplicação para receber uma mensagem do usuário através do teclado e imprimir na tela as respostas
*/
void Task_sendMessage(void *pvParameters) {
  char userInput;
  byte control = B10000000; //byte de controle
  struct frame f;

  for (;;) {
    Serial.println("\n______________________");
    Serial.print("Entre com a mensagem: ");
    for(int j = 0; j<10; j++) f.data[j] = B0;
    
    int i = 0;
    for (;;) {
      while (!Serial.available()) {}
      userInput = Serial.read();
      Serial.print(userInput);
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
    f.port_dest = B1101; //porta de destino
    f.port_font = B1100; //porta de fonte
    
    xQueueSendToBack(TxQueue, &f, 0); //enviando frame para a TxQueue, onde será transmitido na rede
    
    //aguardando a resposta
    while (xQueueReceive(RxQueue, &f, 0) == errQUEUE_EMPTY) {}

    Serial.print("Mensagem recebida: ");
    for(int i = 0; i<10; i++){
      if(f.data[i] != B0){
        Serial.print((char)f.data[i]);
      }else break;
    }
    
    /*
    Serial.print("\nMAC DEST : ");
    Serial.println(f.mac_dest);
    Serial.print("MAC FONT : ");
    Serial.println(f.mac_font);
    Serial.print("PORT DEST : ");
    Serial.println(f.port_dest);
    Serial.print("PORT FONT : ");
    Serial.println(f.port_font);
    /**/
  }

}

/*
  Sinaliza a Task_input e desativa a interrupção
*/
void interrupt( void ) {
  //Serial.println("\ninterrupt");
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

  output_timer = xTimerCreate("output_timer", period / portTICK_PERIOD_MS, pdTRUE, 0, output_timer_function);
  input_timer = xTimerCreate("input_timer", period / portTICK_PERIOD_MS, pdTRUE, 0, input_timer_function);
  synchronize_timer = xTimerCreate("synchronize_timer", (period / 2) / portTICK_PERIOD_MS, pdFALSE, 0, synchronize_timer_function);

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