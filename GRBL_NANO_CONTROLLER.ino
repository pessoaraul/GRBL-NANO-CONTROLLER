/*
   Desenvolvido por Raul Pessoa

   Este software tem como objetvo controlar uma fresadora CNC, que usa o firmware GRBL como interpretador de código G.
   O programa foi desenvolvido de forma independente e todos os direitos dele são reservados exclusivamente ao autor,
   cópias, transferencias e módificações deste código são permanentemente proibidas exceto em casos onde haja concentimento do autor.

   Dentre as funcionalidade do Software estão:
      Envios de Gcodes ao GRBL via comunicação Serial
      Controle dos eixos da máquina atravez de um módulo joystck e um teclado matricial
      Monitorar os status de funcionamento do GRBL
      Monitorar os deslocamentos dos eixos da máquina
      Monitorar a velocidade do Spindle de corte da máquina
      Monitorar a temperatura os drivers de controle dos motores de passo da máquina
      Suporte ao controle de um sistema de refrigeração a a, da parte eletrônica da máquina
      Suporte ao contrle do sistema de exaustao da máquina

   Hardware necessário:
      Arduino Nano
      Módulo Micro Sd
      Módulo Joystick
      Teclado Matricial integrado a um módulo i2c
      Display LCD com módulo i2c
      5 push buttons
      2 sensores de temperatura Lm35
      Módulo Sensor Infravermelho


    Pinos de Ligação do Módulo Micro Sd
      SD card attached to SPI bus as follows:
 *    * MOSI - pin 11
 *    * MISO - pin 12
 *    * CLK - pin 13
 *    * CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

    Pinos de Ligação Display LCD I2c
      Módulo Display pad I2c
       VCC - 5V
       GND - GND
       ADS - pin A4
       SCL - pin A5

    Pinos de Ligação Teclado Matricial I2c
      Módulo key pad I2c
       VCC - 5V
       GND - GND
       ADS - pin A4
       SCL - pin A5

    Pinos de Ligação Sensor LM35
     VCC - 5V
     GND - GND
     Out - pin A0/A1

    Pinos de Ligação Joystick
     VCC - 5V
     GND - GND
     vx - pinA2
     vy - pinA3
     u - pin 6

    Pinos de Ligação Sensor INfravermelho
    Pinos de Ligação Teclado Push-Buttons

*/

//---- Bibliotecas----
#include <LiquidCrystal_I2C.h>                            // Biblioteca display lcd com médulo I2c
#include <Keypad_I2C.h>                          // Biblioteca teclado matricial com módulo I2c
#include <Keypad.h>                              // Biblioteca teclado matricial
#include <Wire.h>                                         // Biblioteca para comunicação seria com médul I2c
#include <SPI.h>
#include <SD.h>                                  //Biblioteca módulo micro SD

//----Mapeamento de Hardware----
#define tempSensor1   16                                  // pino lm35_1 Driver 
#define tempSensor2   17
#define joystick_x    18                                  // pino joystick X
#define joystick_y    19                                  // pino joystick Y
#define rpmSensor     2                                   // pino sensor infraverelho                                                        // pino para relé do sistema ciclone 
#define cooler        3                                   // pino para relé do cooler
#define releCiclone   5                                   // pino para ativação refregeração
#define botNext       10                                  // pino botão Next
#define botBack       9                                   // pino botão Back
#define botControl    8                                   // pino botão Controle
#define botJoystick   6                                   // pino botão Joystick
#define botSend       7                                   // pino botão Enviar


//----Instaciamento dos objetos----
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // cria um objeto chamado lcd para executaar as funções do display

const byte linhas = 4;
const byte colunas = 4;
char teclas[linhas][colunas] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte pinLinhas[linhas] = {0, 1, 2, 3};
byte pinColunas[colunas] = {4, 5, 6, 7};
int i2cendereco = 0x20;
Keypad_I2C teclado = Keypad_I2C( makeKeymap(teclas), pinLinhas, pinColunas, linhas, colunas, i2cendereco);//cria um objeto chamadp teclado para execução das funções do teclado matricial

//----Funções----
void changeMenu();                                        //carrega e faz a mudança dos menus do programa
void showMenu();                                          //mostra o menu carregado no diplay
void temperatura();                                       //executa uma rotina de leitura da temperatura dos drivers
void tacometro();                                         //executa uma rotina de leitura da velocidade de rotação do spindle
void incrementoRPM();                                     //incremento usado na função tacometro
void jog();                                               // função de controle dos eixos da máquina através do joystick e/ ou teclado matricial
void exaustor();                                          // função que faz o acionamento do aspirador ciclone
void SDcard();                                            // função que executa uma rotina de chamada para as funções do módulo sd
void getPosition();                                       //recebe e interpreta os relatórios de posição do grbl
void retorna0();                                          //comando para que a máquina retorne para a posição 0maquina
void reset0();                                            //comando para transformar a posição atual na posição 0maquina
void homing();                                            //executa a rotina de ciclo homing. Sondagem nos 3 eixos
void probe();                                             //executa uma rotina de sondagem do eixo Z
void destrava();                                          //envia comando ao grbl parabdestravar o grbl
void posicao();                                           //imorime no display as posições dos eixos em tempo real
byte menuSD();
String getName(byte i);
byte contaArquivos();
void showTextFile( String linha1, String linha2);
void enviaArquivo(byte indexiile);
bool getOk();
void workDisplay(unsigned long execTimer);
void terminado();

//----Variáveis Globais----
short int x_jog, y_jog, z_jog;
short int feedRate;
short int H, M, S;
short int exaus;
short int fan;
long rpm = 0;                                            // variável que armazena o número de rotações
float temp1;
float temp2;
String nome;
char statusdeposicao[10];                                 // status da posição da mquina
char x_Mpos[9], y_Mpos[9], z_Mpos[9];                     // Coordenadas Absolutas da maquina
char x_Wpos[9], y_Wpos[9], z_Wpos[9];                     // coordenadas instantãneas da maquina
char menu = 0x01;                                         // cariável que recebe o menu
char tecla;
char set1 = 0x00, set2 = 0x00, set3 = 0x00, set4 = 0x00;  // flag para botão de controle dos periféricos
char set5 = 0x00, set6 = 0x00, set7 = 0x00;
bool confirma_ok = false;
boolean f_botNext, f_botBack, f_botControl, f_botJoystick, f_botSend, f_botMenu;     // botão de controle do jog
unsigned long execTime;
unsigned long rpmTime;

//----Arquivos para cartão SD-----

void setup() {

  Serial.begin(115200);                                     // inicia  monitor srial com taxa de 9600
  lcd.begin (20, 4);                                        // inicia o objeto lcd
  teclado.begin();

  pinMode(joystick_x, INPUT);
  pinMode(joystick_y, INPUT);
  pinMode(releCiclone, OUTPUT);                           // inicia o pino do ciclone como saída
  pinMode(cooler, OUTPUT);                                // inicia o pino do cooler como saída

  for (char i = 6; i < 11; i++) pinMode(i, INPUT_PULLUP); // laço for para os pinos dos botões

  x_jog  =  0;
  y_jog  =  0;
  z_jog  =  0;
  feedRate = 0;
  exaus = 0;
  fan = 0;
  f_botNext      =   0x00;                                    // limpa flag do botão next
  f_botBack      =   0x00;                                    // limpa fleg do botão back
  f_botJoystick  =   0x00;                                    // limpa flag do botão enter
  f_botControl   =   0x00;                                    // limpa flag do botão control
  f_botSend      =   0x00;                                    // limpa flag do botão send
  f_botMenu      =   0x00;

  digitalWrite(releCiclone, LOW);                             // inicia o pino ciclone em nível baixo
  digitalWrite(cooler, LOW);                                  // inicia o pino cooler em nível baixo

  lcd.setCursor(6, 1);
  lcd.print(F("GRBL NANO"));                                  // Tela Inicial do Programa
  lcd.setCursor(3, 2);
  lcd.print(F("CNC CONTROLLER"));

  while (digitalRead(botJoystick)) {}                         // Aguarda o Joystick ser precionado para liberar o programa
  delay(1000);
  while (!digitalRead(botJoystick)) {}
  delay(1000);
  lcd.clear();

  pinMode(4, OUTPUT);
  SD.begin(4);                                                // Inicializa o cartão SD, se o cartão não for inserido
  if (!SD.begin(4)) {}                                        // o programa irá esperar até que ele seja.

}// end setup

void loop () {

  changeMenu();
  showMenu();
  tecla = teclado.getKey();
  if (tecla == '*') {
    exaus++;
    if (exaus > 1) exaus = 0;
    switch (exaus) {
      case 1:
        digitalWrite(releCiclone, 1);

        break;
      case 0:
        digitalWrite(releCiclone, 0);

        break;
    }
  }
  if (tecla == '0') {
    fan++;
    if (fan > 1) fan = 0;
    switch (fan) {
      case 1:
        digitalWrite(cooler, 1);

        break;
      case 0:
        digitalWrite(cooler, 0);

        break;
    }
  }
}// end loop

//---- Fuções para Cotrole dos Menus----
void changeMenu() {

  if (!digitalRead(botNext)) f_botNext = 0x01;
  if (!digitalRead(botBack)) f_botBack = 0x01;

  if (digitalRead(botNext) && f_botNext) {

    f_botNext = 0x00;

    lcd.clear();
    menu++;

    if (menu > 0x08) menu = 0x01;
  }
  if (digitalRead(botBack) && f_botBack) {

    f_botBack = 0x00;

    lcd.clear();
    menu--;

    if (menu < 0x01) menu = 0x08;
  }
}

void showMenu() {

  switch (menu)  {
    case 0x01:
      destrava();

      break;
    case 0x02:
      SDcard();

      break;
    case 0x03:
      jog();

      break;
    case 0x04:
      retorna0();

      break;
    case 0x05:
      reset0();

      break;
    case 0x06:
      homing();

      break;
    case 0x07:
      probe();

      break;
    case 0x08:
      exaustor();

      break;
  }
}

//----Funções de Controle da CNC
void SDcard() {

  byte i;
  lcd.setCursor(0, 0);
  lcd.print(F("Menu SD Card"));

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    lcd.clear();
    f_botControl = 0x00;
    set1++;

    if (set1 > 0x02) set1 = 0x01;
    while (set1 == 0x01) {

      enviaArquivo(menuSD());

      if (!digitalRead(botControl)) f_botControl = 0x01;
      if (digitalRead(botControl) && f_botControl) delay(500); break;
    } loop();
  }
}
void jog() {
  short int desloca = 0;
  float incremento = 0;
  float velocidade = 0;

  lcd.setCursor(0, 0);
  lcd.print(F("Axis Motion"));
  lcd.setCursor(0, 1);
  lcd.print(F("Press Joystck"));

  if (!digitalRead(botJoystick)) f_botJoystick = 0x01;
  if (digitalRead(botJoystick) && f_botJoystick) {

    f_botJoystick = 0x00;
    set2++;
    lcd.clear();

    if (set2 > 0x02) set2 = 0x01;

    while (set2 == 0x01) {

      tecla = teclado.getKey();
      if (tecla == '5') {
        desloca++;
        if (desloca > 6) desloca = 0;
      } if (tecla == 'B') {
        desloca--;
        if (desloca < 0) desloca = 6;
      }
      if (tecla == 'D') {
        feedRate++;
        if (feedRate > 6) feedRate = 0;
      }
      if (tecla == '#') {
        feedRate--;
        if (feedRate < 0) feedRate = 6;
      }

      if (desloca == 0) incremento = 0.01;
      if (desloca == 1) incremento = 0.1;
      if (desloca == 2) incremento = 0.5;
      if (desloca == 3) incremento = 1;
      if (desloca == 4) incremento = 5;
      if (desloca == 5) incremento = 10;
      if (desloca == 6) incremento = 20;

      if (feedRate == 0) velocidade = 1;
      if (feedRate == 1) velocidade = 10;
      if (feedRate == 2) velocidade = 50;
      if (feedRate == 3) velocidade = 100;
      if (feedRate == 4) velocidade = 250;
      if (feedRate == 5) velocidade = 500;
      if (feedRate == 6) velocidade = 800;

      posicao();
      lcd.setCursor(0, 0);
      lcd.print(F("Inc "));
      lcd.print(incremento);
      lcd.setCursor(0, 1);
      lcd.print(F("Vel "));
      lcd.print(velocidade);
      switch (tecla) {
        case '1':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("X")); Serial.print(-incremento);
          Serial.print(F("Y")); Serial.println(incremento);
          posicao();

          break;
        case '2':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("Y")); Serial.println(incremento);
          posicao();

          break;
        case '3':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("X")); Serial.print(incremento);
          Serial.print(F("Y")); Serial.println(incremento);
          posicao();

          break;
        case '4':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("X")); Serial.println(-incremento);
          posicao();

          break;
        case '6':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("X")); Serial.println(incremento);
          posicao();

          break;
        case '7':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("X")); Serial.print(-incremento);
          Serial.print(F("Y")); Serial.println(-incremento);
          posicao();

          break;
        case '8':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("Y")); Serial.println(-incremento);
          posicao();

          break;
        case '9':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("X")); Serial.print(incremento);
          Serial.print(F("Y")); Serial.println(-incremento);
          posicao();

          break;
        case 'A':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("Z")); Serial.println(incremento);
          posicao();

          break;
        case 'C':
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print(F("Inc "));
          lcd.print(incremento);
          lcd.setCursor(0, 1);
          lcd.print(F("Vel "));
          lcd.print(velocidade);
          Serial.print(F("G91"));
          Serial.print(F("Z")); Serial.println(-incremento);
          posicao();

          break;
      }
      y_jog = map(analogRead(joystick_x), 0, 1023, -incremento, incremento);
      x_jog = map(analogRead(joystick_y), 0, 1023, -incremento, incremento);

      if (x_jog != 0 || y_jog != 0) {
        getPosition();
        Serial.print(F("$J=G21G91"));
        if (x_jog != 0) {
          Serial.print("X");
          Serial.print(x_jog);
        }
        if (y_jog != 0) {
          Serial.print("Y");
          Serial.print(y_jog);
        }
        Serial.print(F("F"));
        Serial.println(velocidade);
        getPosition();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Inc"));
        lcd.print(incremento);
        lcd.setCursor(0, 1);
        lcd.print(F("Vel "));
        lcd.print(velocidade);
        posicao();
        delay(250);
      }
      if (!digitalRead(botNext)) {
        getPosition();
        z_jog = incremento;
        Serial.print(F("$J=G21G91Z"));
        Serial.print(z_jog);
        Serial.print(F("F"));
        Serial.println(velocidade);
        lcd.setCursor(10, 3);
        lcd.print(F("Z:"));
        lcd.print(z_jog );
        lcd.setCursor(12, 3);
        lcd.print(z_Mpos);
        delay(100);
      }
      if (!digitalRead(botBack)) {
        getPosition();
        z_jog = -incremento;
        Serial.print(F("$J=G21G91Z"));
        Serial.print(z_jog);
        Serial.print(F("F"));
        Serial.println(velocidade);
        lcd.setCursor(10, 3);
        lcd.print(F("Z:"));
        lcd.print(z_jog );
        lcd.setCursor(12, 3);
        lcd.print(z_Mpos);
        delay(100);
      }
      if (!digitalRead(botJoystick)) f_botJoystick = 0x01;
      if (digitalRead(botJoystick) && f_botJoystick) break;
    }
  }
}
void retorna0() {

  char retorna_eixo = 0x00;
  lcd.setCursor(6, 1);
  lcd.print(F("Return to Zero"));

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    f_botControl = 0x00;
    set3++;
    lcd.clear();

    if (set3 > 0x02) set3 = 0x01;
    while (set3 == 0x01) {
      if (!digitalRead(botNext)) {
        retorna_eixo++;
        lcd.clear();
        delay(200);
        if (retorna_eixo > 0x03) retorna_eixo = 0x00;
      }

      if (!digitalRead(botBack)) {
        retorna_eixo--;
        lcd.clear();
        delay(200);
        if (retorna_eixo < 0x00) retorna_eixo = 0x03;
      }
      switch (retorna_eixo) {
        case 0:
          lcd.setCursor(0, retorna_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Return XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Return X");
          lcd.setCursor(1, 2);
          lcd.print("Return Y");
          lcd.setCursor(1, 3);
          lcd.print("Return Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G90 G0 X0 Y0 Z0"));
            f_botSend = 0x00;
          }
          posicao();

          break;
        case 0x01:
          lcd.setCursor(0, retorna_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Return XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Return X");
          lcd.setCursor(1, 2);
          lcd.print("Return Y");
          lcd.setCursor(1, 3);
          lcd.print("Return Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G90 G0 X0"));
            f_botSend = 0x00;
          }
          posicao();

          break;
        case 0x02:
          lcd.setCursor(0, retorna_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Return XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Return X");
          lcd.setCursor(1, 2);
          lcd.print("Return Y");
          lcd.setCursor(1, 3);
          lcd.print("Return Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G90 G0 Y0"));
            f_botSend = 0x00;
          }
          posicao();

          break;
        case 0x03:
          lcd.setCursor(0, retorna_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Return XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Return X");
          lcd.setCursor(1, 2);
          lcd.print("Return Y");
          lcd.setCursor(1, 3);
          lcd.print("Return Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G90 G0 Z0"));
            f_botSend = 0x00;
          }
          posicao();

          break;
      }
      if (!digitalRead(botControl)) f_botControl = 0x01;
      if (digitalRead(botControl) && f_botControl)break;
    }
  }
}
void reset0() {

  char zera_eixo = 0x00;
  lcd.setCursor(0, 0);
  lcd.print(F("Reset Zero"));

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    f_botControl = 0x00;
    set4++;
    lcd.clear();

    if (set4 > 0x02) set4 = 0x01;
    while (set4 == 0x01) {
      tecla = teclado.getKey();
      if (!digitalRead(botNext)) {
        zera_eixo++;
        lcd.clear();
        delay(200);
        if (zera_eixo > 0x03) zera_eixo = 0x00;
      }

      if (!digitalRead(botBack)) {
        zera_eixo--;
        lcd.clear();
        delay(200);
        if (zera_eixo < 0x00) zera_eixo = 0x03;
      }
      switch (zera_eixo) {
        case 0:
          lcd.setCursor(0, zera_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Zerar XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Zerar X");
          lcd.setCursor(1, 2);
          lcd.print("Zerar Y");
          lcd.setCursor(1, 3);
          lcd.print("Zerar Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G10 P0 L20 X0 Y0 Z0"));
            f_botSend = 0x00;
          }
          if (getOk()) {
            lcd.setCursor(10, 0);
            lcd.print(F("Ok"));
          }

          break;
        case 1:
          lcd.setCursor(0, zera_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Zerar XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Zerar X");
          lcd.setCursor(1, 2);
          lcd.print("Zerar Y");
          lcd.setCursor(1, 3);
          lcd.print("Zerar Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G10 P0 L20 X0"));
            f_botSend = 0x00;
          }
          if (getOk()) {
            lcd.setCursor(10, 0);
            lcd.print(F("Ok"));
          }

          break;
        case 2:
          lcd.setCursor(0, zera_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Zerar XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Zerar X");
          lcd.setCursor(1, 2);
          lcd.print("Zerar Y");
          lcd.setCursor(1, 3);
          lcd.print("Zerar Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G10 P0 L20 Y0"));
            f_botSend = 0x01;
          }
          if (getOk()) {
            lcd.setCursor(10, 0);
            lcd.print(F("Ok"));
          }

          break;
        case 3:
          lcd.setCursor(0, zera_eixo);
          lcd.print(">");
          lcd.setCursor(1, 0);
          lcd.print("Zerar XYZ");
          lcd.setCursor(1, 1);
          lcd.print("Zerar X");
          lcd.setCursor(1, 2);
          lcd.print("Zerar Y");
          lcd.setCursor(1, 3);
          lcd.print("Zerar Z");
          if (!digitalRead(botSend)) f_botSend = 0x01;
          if (digitalRead(botSend) && f_botSend) {
            Serial.println(F("G10 P0 L20 Z0"));
            f_botSend = 0x00;
          }
          if (getOk()) {
            lcd.setCursor(10, 0);
            lcd.print(F("Ok"));
          }
          break;
      }
      if (!digitalRead(botControl)) f_botControl = 0x01;
      if (digitalRead(botControl) && f_botControl)break;
    }
  }
}
void homing() {

  lcd.setCursor(0, 0);
  lcd.print(F("Start Homing"));

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    f_botControl = 0x00;
    set5++;
    Serial.println(F("$H"));
    lcd.clear();

    if (set5 > 0x02) set5 = 0x01;
    while (set5 == 0x01) {
      posicao();
      if (!digitalRead(botControl)) f_botControl = 0x01;
      if (digitalRead(botControl) && f_botControl)break;
    }
  }
}
void probe() {

  lcd.setCursor(0, 0);
  lcd.print(F("Probe Function"));

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    f_botControl = 0x00;
    set6++;

    if (set6 > 0x02)set6 = 0x01;
    while (set6 == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Executando"));
      Serial.println(F("G38.2 Z-40 F2"));
      if (confirma_ok) {
        Serial.println(F("G92 Z20"));
        if (confirma_ok) {
          Serial.println(F("G0 Z20"));
          if (confirma_ok) {
            getPosition();
            lcd.setCursor(0, 1);
            lcd.print("Z:");
            lcd.print(z_Mpos);
          }
        }
      }
      if (!digitalRead(botControl)) f_botControl = 0x01;
      if (digitalRead(botControl) && f_botControl) break;
    }
  }
}
void destrava() {

  getPosition();
  lcd.setCursor(0, 0);
  lcd.print(F("Status Machine"));
  lcd.setCursor(0, 1);
  lcd.print(statusdeposicao);

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    Serial.println(F("$x"));
    return;
  }
}
void posicao() {

  unsigned long pos;
  getPosition();
  if (( millis() - pos) >= 250) {
    lcd.setCursor(10, 3);
    lcd.print(F("Z:")); lcd.print(z_Mpos);
    lcd.setCursor(10, 1);
    lcd.print(F("X:")); lcd.print(x_Mpos);
    lcd.setCursor(10, 2);
    lcd.print(F("Y:")); lcd.print(y_Mpos);
  }
}
void exaustor() {
  lcd.setCursor(0, 0);
  lcd.print(F("Exaustor Ciclone"));

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    f_botControl = 0x00;
    set7++;

    if (set7 > 2) set7 = 0x01;

    switch (set7) {

      case 0x01:
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(F("Status"));
        lcd.print("  On");
        digitalWrite(releCiclone, 1);
        break;

      case 0x02:
        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print(F("Status"));
        lcd.print("  Off");
        digitalWrite(releCiclone, 0);
        break;
    }
  }
}

void tacometro() {

  rpmTime =  millis();
  rpm = 0;                                                // zera o número de rotações
  attachInterrupt(0, incrementoRPM, FALLING);             // detecta as interrumções no sensor ligado ao pino 2
  delay(100);                                             // aguarda 1 segundo
  detachInterrupt(0);                                     // desabilita as interrupções

  if ((millis() - rpmTime) - 1000) {
    rpm = rpm * 600;                                        // rpm = número de interrupções * 60
    rpmTime = millis();
  }
  /*
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Spindle Speed");
    lcd.setCursor(0, 1);
    lcd.print(rpm);
    lcd.print(" rpm");
  */
}

void incrementoRPM() {
  rpm++;
}
/*
void temperatura() {

  int leitura1 = analogRead(tempSensor1);                    // lê os sensores lm35
  float voltagem1 = leitura1 * (5.0 / 1023);                 // calcula a tnsão lida no sensor
  temp1 = voltagem1 * 100;                                   // temperatura = tensão*100

  lcd.setCursor(0, 0);                                     // orienta o cursor do dlispay lcd
  lcd.print(F("Drivers Temperature"));                        // imprime um texto no display

  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) {

    f_botControl = 0x00;
    set5++;

    if (set5 > 1) set5 = 0x01;
  }
  switch (set5) {
    case 0x01:
      lcd.setCursor(0, 1);
      lcd.print("Driver X: ");
      lcd.print(temp1);
      break;
  }

  if (temp1 > 55) {
    digitalWrite(cooler, 1);
  } else {
    digitalWrite(cooler, 0);
  }
  delay(10);                             // aguarda 0.01 segndo
}
*/

void getPosition() {

  char contador[85];
  char caractere;
  byte index = 0;
  bool mensagem = false;
  int i = 0;
  int c = 0;

  Serial.println("?");  // Ask the machine status
  while (Serial.available()) {
    caractere = Serial.read();
    contador[index] = caractere;
    if (contador[index] == '>') mensagem = true;
    if (index > 0) {
      if (contador[index] == 'k' && contador[index - 1] == 'o') {
        confirma_ok = true;
      }
    }
    index++;
    delay(1);
  }

  if (!mensagem)  return;
  i++;
  while (c < 9 && contador[i] != '|') {
    statusdeposicao[c++] = contador[i++];
    statusdeposicao[c] = 0;
  }
  while (contador[i++] != ':') ; // skip until the first ':'
  c = 0;
  while (c < 8 && contador[i] != ',') {
    x_Mpos[c++] = contador[i++];  // get MposX
    x_Mpos[c] = 0;
  }
  c = 0;
  i++;
  while (c < 8 && contador[i] != ',') {
    y_Mpos[c++] = contador[i++];  // get MposY
    y_Mpos[c] = 0;
  }
  c = 0;
  i++;
  while (c < 8 && contador[i] != '|') {
    z_Mpos[c++] = contador[i++];  // get MposZ
    z_Mpos[c] = 0;
  }
  while (contador[i++] != ':') ; // skip until the next ':'
  c = 0;
  while (c < 8 && contador[i] != ',') {
    x_Wpos[c++] = contador[i++];  // get WposX
    x_Wpos[c] = 0;
  }
  c = 0;
  i++;
  while (c < 8 && contador[i] != ',') {
    y_Wpos[c++] = contador[i++];  // get WposY
    y_Wpos[c] = 0;
  }
  c = 0;
  i++;
  while (c < 8 && contador[i] != '>') {
    z_Wpos[c++] = contador[i++];  // get WposZ
    z_Wpos[c] = 0;
  }

  if (z_Wpos[0] == '-') {
    z_Wpos[5] = '0';
    z_Wpos[6] = 0;
  } else {
    z_Wpos[4] = '0';
    z_Wpos[5] = 0;
  }
}
//--------- Funções para o Módulo Sd Card----
byte menuSD() {

  unsigned long lastUpdate;
  byte indexfile = 1;
  byte fc = contaArquivos();

  nome = getName(indexfile);
  showTextFile(F("Arquivos SD"), "->" + (String)nome, "", F("Click to Start"));

  while (true) {

    tecla = teclado.getKey();
    if (tecla == '#') {
      exaus++;
      if (exaus > 1) exaus = 0;
      switch (exaus) {
        case 1:
          digitalWrite(releCiclone, 1);
          Serial.println(exaus);

          break;
        case 0:
          digitalWrite(releCiclone, 0);
          Serial.println(exaus);

          break;
      }
    }
    if (tecla == '0') {
      fan++;
      if (fan > 1) fan = 0;
      switch (fan) {
        case 1:
          digitalWrite(cooler, 1);

          break;
        case 0:
          digitalWrite(cooler, 0);

          break;
      }
    }
    if (indexfile < fc && digitalRead(botNext) == LOW) {
      indexfile++;
      nome = getName(indexfile);
      lcd.setCursor(0, 1);
      lcd.print(F("-> "));
      lcd.print(nome);
      for (int i = nome.length() + 4 ; i < 20 ; i++) {
        lcd.print(" ");
      } delay(100);
    }
    if (indexfile > 1 && digitalRead(botBack) == LOW) {
      indexfile--;
      nome = getName(indexfile);
      lcd.setCursor(0, 1);
      lcd.print(F("-> ")); lcd.print(nome);
      for (int i = nome.length() + 4; i < 20; i++) {
        lcd.print(" ");
      } delay(100);
    }
    if (!digitalRead(botSend)) f_botSend = 0x01;
    if (indexfile > 0 && digitalRead(botSend) && f_botSend) {
      return indexfile;
    }
    if (!digitalRead(botControl)) f_botControl = 0x01;
    if (digitalRead(botControl) && f_botControl) break;
  } loop();
}
String getName(byte i) {

  byte x = 0;
  String resultado;
  File root = SD.open("/");
  while (resultado == "") {
    File entry = root.openNextFile();
    if (!entry) {}
    else {
      if (!entry.isDirectory()) {
        x++;
        if (x == i) resultado = entry.name();
      }
      entry.close();
    }
  }
  root.close();
  return resultado;
}
byte contaArquivos() {
  byte c = 0;
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      root.rewindDirectory();
      root.close();
      return c;
      break;
    } else {
      if (!entry.isDirectory()) c++;
      entry.close();
    }
  }
}
void showTextFile( String linha1, String linha2, String linha3, String linha4) {

  lcd.setCursor(0, 0);
  lcd.print(linha1);
  for (int i = linha1.length(); i < 20; i++) {
    lcd.print(" ");
  }
  lcd.setCursor(0, 1);
  lcd.print(linha2);
  for (int i = linha2.length(); i < 20; i++) {
    lcd.print(" ");
  }
  lcd.setCursor(0, 2);
  lcd.print(linha3);
  for (int i = linha3.length(); i < 20; i++) {
    lcd.print(" ");
  }
  lcd.setCursor(0, 3);
  lcd.print(linha4);
  for (int i = linha4.length(); i < 20; i++) {
    lcd.print(" ");
  }
}
void enviaArquivo(byte indexiile) {

  String linha = "";

  File dataFile;
  unsigned long atualizar;

  String arquivo;
  arquivo = getName(indexiile);
  dataFile = SD.open(arquivo);
  if (!dataFile) {
    showTextFile(F("File"), "", "", F("Erro ao Executar"));
    delay(100);
    return;
  }
  Serial.println(F("G90"));
  Serial.println(F("G21"));
  Serial.println(F("G92 G0 X0  Y0  Z0"));
  delay(250);

  getOk();
  execTime = millis();
  while (dataFile.available()) {
    if (confirma_ok) {
      confirma_ok = false;
      linha = dataFile.readStringUntil('\n');
      Serial.print(linha);
    } else {}
    if (millis() - atualizar >= 250) {
      atualizar = millis();
      workDisplay(execTime);
    }
  }
  delay(2000);
  terminado();
}
bool getOk() {

  char ult_carac, ant_carac;
  ult_carac = 64;
  ant_carac = 64;

  while (Serial.available()) {
    ult_carac = Serial.read();
    if (ant_carac == 'o' && ult_carac == 'k') {
      confirma_ok = true;
      return confirma_ok;
    }
    ant_carac = ult_carac;
    delay(1);
  }
}
void workDisplay(unsigned long execTime) {

  unsigned long tempo = millis() - execTime;
  char timeString[9];
  char p[3];
  tempo = tempo / 1000;
  H = floor(tempo / 3600);
  tempo = tempo - (H * 3600);
  M = floor(tempo / 60);
  S = tempo - (M * 60);
  sprintf (timeString, "%02d:%02d:%02d", H, M, S);

  getPosition();
  //temperatura();
  tacometro();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Speed:")); lcd.print(rpm); lcd.print(F("rpm"));
  lcd.setCursor(0, 15);
  lcd.print(F("|")); lcd.print(statusdeposicao);
  lcd.setCursor(0, 1);
  lcd.print(timeString);
  /*
    lcd.setCursor(0, 2);
    lcd.print(F("T:")); lcd.print(temp1); lcd.write(byte(223)); lcd.print(F("C"));
  */
  lcd.setCursor(11, 1);
  lcd.print(F("X:")); lcd.print(x_Mpos);
  lcd.setCursor(11, 2);
  lcd.print(F("Y:")); lcd.print(y_Mpos);
  lcd.setCursor(11, 3);
  lcd.print(F("Z:")); lcd.print(z_Mpos);

}
void terminado() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(nome);
  lcd.setCursor(0, 1);
  lcd.print(F("Terminado"));
  lcd.setCursor(0, 2);
  lcd.print(F("Tempo "));
  lcd.print(H); lcd.print(F(":"));
  lcd.print(M); lcd.print(F(":"));
  lcd.print(S);
  digitalWrite(releCiclone, 0);
  delay(1000);
  if (!digitalRead(botControl)) f_botControl = 0x01;
  if (digitalRead(botControl) && f_botControl) loop();
}
