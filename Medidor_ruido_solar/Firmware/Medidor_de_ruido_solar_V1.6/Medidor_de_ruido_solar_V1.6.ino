/*  EA5IOT Medidor de rudio solar V1.6
    Copyright (C) 2018  EA5IOT

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "lcd7920_STM.h"
#include <EEPROM.h>

extern const PROGMEM LcdFont font5x7_STM;                                                                   // in glcd10x10.cpp

#define LCD_USE_SPI     (false)                                                                             // no SPI yet for STM32
#define MOSIpin         PB15
#define SCLKpin         PB13
#define LED             PC13
#define Boton1          PB9
#define Boton2          PB8
#define Boton3          PB7
#define Boton4          PB6
#define ADC             PB0
#define Timer1Ch2       PA9                                                                                 // Salida PWM para el control de la retroiluminación de la pantalla, PA9 pwm 5V out (open drain)
#define Y0              13                                                                                  // inicio escala
#define Y1              21                                                                                  // valores escala

typedef union
{
  float Float;
  byte Byte[4];
} Float;

static Lcd7920 lcd(SCLKpin, MOSIpin, LCD_USE_SPI);

int X_ant = 0;
int X_punta = 0;
int Memoria[128];

const int Puntos_Calibracion = 23;                                                                          // Cantidad de puntos de calibracion, de 5db en 5db
const int DBm_Maximo = -20;                                                                                 // DBm máximos
const int DBm_Minimo = - (((Puntos_Calibracion - 1) * 5) + (DBm_Maximo * -1));                              // El valor de DBm mínimos se calcula en función de los DBm máximos y la cantidad de puntos de calibración
Float TablaCalibracion[Puntos_Calibracion];

float Cero;
float Dato;
unsigned int ValorNuevo = 0;                                                                                // Se va incrementando en 1 para tener constancia de la actualización del ADC.
unsigned int ValorAnterior = 0;                                                                             // Se incremeenta manualmente para comprobar si el ADC se ha actualizado.
float DBm;
float DBrel;
unsigned long TiempoPunta = 0;
unsigned long TiempoPuntaPasado = 0;
unsigned long TiempoMemoria = 0;
unsigned long TiempoMemoriaPasado = 0;
int TiempoEnc = 0;
bool XigualAnt = true;
int Menu = 0;
int SubMenu = 0;
bool SubMenu1 = false;
int Seleccion = 0;
const int MenuCalibracion = 2;
const int Opciones[] = {4-1, 23-1, 1-1, 5-1, 5-1};                                                          // Opciones de los Menus -> Principal, Calibracion dB, Filtro Señal y Escalas
bool BorrarMenuCalibracion = false;
bool BorrarPantalla = true;                                                                                 // Borra la pantalla de medidas una sola vez cuando entra por primera vez al menu principal
bool CambioEscala = false;
bool BuscaCero = false;
bool InicioMedia = false;
bool Iniciar_Barra = true;
unsigned long TiempoInicioMedia = 0;
float Media = 0.0;
int MediaCont = 0;
unsigned int K1 = 20;
unsigned int K2 = 40;
unsigned int K3 = 80;
unsigned int K4 = 160;
unsigned int K5 = 320;
unsigned int K = K5;                                                                                        // Ganancia filtro señal
unsigned int Escala1 = 1;
unsigned int Escala2 = 3;
unsigned int Escala3 = 5;
unsigned int Escala4 = 10;
unsigned int Escala5 = 20;
int Escala = 20;                                                                                            // Fondo de escala en db
const float EscalaADC = 3.3 / 4096.0;
float IncCero = (float)Escala / 127.0;
bool Pulsador1_Ant = HIGH;
bool Pulsador2_Ant = HIGH;
bool Pulsador3_Ant = HIGH;
bool Pulsador4_Ant = HIGH;
bool Pulsador1 = false;
bool Pulsador2 = false;
bool Pulsador3 = false;
bool Pulsador4 = false;
bool Pulsador1Repeticion = false;
bool Pulsador2Repeticion = false;
bool Pulsador3Repeticion = false;
bool Pulsador4Repeticion = false;
unsigned long TiempoPulsador1 = 0;
unsigned long TiempoPulsador2 = 0;
unsigned long TiempoPulsador3 = 0;
unsigned long TiempoPulsador4 = 0;
unsigned long TiempoInicioRepeticionPulsacion = 1000;                                                       // Tiempo de inicio de repetición de las pulsaciones de los botones en milisegundos
const unsigned long TiempoRepeticionPulsacion = 50;                                                         // Tiempo de repetición de las pulsaciones de los botones en milisegundos
unsigned long TiempoInicioRepeticionPulsador1 = 0;
unsigned long TiempoInicioRepeticionPulsador2 = 0;
unsigned long TiempoInicioRepeticionPulsador3 = 0;
unsigned long TiempoInicioRepeticionPulsador4 = 0;
unsigned long Menu1Parpadeo = 0;
bool Menu1ParpadeoOK = false;
unsigned int Contraste = 128;
bool GrabarMemoria = false;

bool Debug = false;
unsigned long DebugMilis = 0;
int Test1 = 0;
int Test2 = 0;
int Test4 = 0;
float Test3 = 0.0;

void setup()
{
  afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY);                                                                 // Se deshabilitan los pines JTAG, pero se permiten los pines para ST-LINK
  
  pinMode(LED, OUTPUT);                                                                                     // Pata PC13 como salida
  pinMode(Boton1, INPUT_PULLUP);
  pinMode(Boton2, INPUT_PULLUP);
  pinMode(Boton3, INPUT_PULLUP);   
  pinMode(Boton4, INPUT_PULLUP);   
  pinMode(ADC, INPUT);

  CargarValoresEEprom();
  IniciarContadores();                                                                                      // Se inician los contadores antes del primer mensaje para que el PWM de control de contraste funcione

  delay(500);                                                                                               // Esperamos a que el LCD se inicialice internamente para aceptar datos
  lcd.begin(true);                                                                                          // Se inicia la pantalla
  lcd.setFont(&font5x7_STM);                                                                                // Elegimos la fuente de texto
  lcd.clear();
  lcd.setCursor(22, 4);
  lcd.print("EA5IOT SUN NOISE METER");
  lcd.setCursor(32, 35);
  lcd.print("VERSION 1.6");
  lcd.flush();                                                                                              // Se actualiza la pantalla para que aparezca el mensaje
  delay(5000);                                                                                              // Tiempo que aparece el mensaje en pantalla (5 segundos)
  lcd.clear();
  lcd.flush();
  
  for (int i = 0; i < 128; i++) Memoria[i] = 0;                                                             // Se inicializa el buffer de memoria del osciloscopio
  PintarEscala();

  Cero = DBm = DBm_Minimo;
  Dato = TablaCalibracion[0].Float;
  
  Timer4.resume();                                                                                          // Se activan las interrupciones de tiempo para llamar a la rutina de leervalor
}

void loop()
{
  LeerPulsadores();
  
  if (BuscaCero)
  {
    if (CalcularMedia(DBm, 50))
    {
      Cero = Media;
      BuscaCero = false;
    };
  }
  else if (CambioEscala)
    {
      PintarEscala();
      //BuscaCero = true;                                                                                   // Descomentar si se quiere hacer un autocero cada vez que se cambie de escala
      CambioEscala = false;
    };

  if ((Menu == 2) && (SubMenu == 1) && (SubMenu1 == true) && (Seleccion < Puntos_Calibracion))                       
  {
    if (CalcularMedia(Dato, 20))                                                                            // Se calcula la media de x valores del ADC
    {
      TablaCalibracion[Seleccion].Float = Media;                                                            // Se almacena el valio medio del punto de calibracion.      
      Seleccion++;                                                                                          // Avanzamos al siguiente punto de calibracion.
      if (Seleccion < Puntos_Calibracion) SubMenu1 = false;                                                 // Si ya se han recogido los Puntos_Calibracion, dejamos el último mensaje en pantalla fijo.
    };
  };

  if (GrabarMemoria) CargarValoresEEprom();                                                                 // Grabamos los nuevos datos a la EEPROM

  ActualizarPantalla();                                                                                     // La actualización de la pantalla influye en la señal, por lo tanto en BuscaCero, por lo tanto hay que hacer BuscaCero y Actualizar la pantalla a la vez.
}

void LeerValor(void)
{
  int c, d, ii;
  float fx, dif, x, a0, a1, a2, a3, y0, y1, y2, y3, k;

  Dato += ((float)K*0.001)*((float)analogRead(ADC) - Dato);                                                 // Se filtra la lectura del ADC
  if (Dato >= TablaCalibracion[Puntos_Calibracion - 1].Float) Dato = TablaCalibracion[Puntos_Calibracion - 1].Float;
  ii = 0;
  d = 0;
  for (int i = 0; i < Puntos_Calibracion; i++)                                                              // Busqueda del indice para apuntar a los valores de calibración
  {
    if (Dato >= TablaCalibracion[i].Float)
    {
      ii = i;                                                                                               // Buscamos el valor mas cercano al dato leido según la tabla de calibración para aplicar los coeficientes mas cercanos de calibración
      d = i;
    };
  }; 
  
  if (ii < 1)
  {
    ii = 1;
    y0 = TablaCalibracion[ii - 1].Float - 5.0; //(TablaCalibracion[ii].Float - TablaCalibracion[ii - 1].Float);
    y1 = TablaCalibracion[ii - 1].Float;
    y2 = TablaCalibracion[ii].Float;
    y3 = TablaCalibracion[ii + 1].Float;
  } else if (ii >= (Puntos_Calibracion - 2))
  {
    ii = Puntos_Calibracion - 2;
    y0 = TablaCalibracion[ii - 1].Float;
    y1 = TablaCalibracion[ii].Float;
    y2 = TablaCalibracion[ii + 1].Float;
    y3 = TablaCalibracion[ii + 1].Float + 7.0; //(TablaCalibracion[ii + 1].Float - TablaCalibracion[ii].Float);
  } else 
  {
    y0 = TablaCalibracion[ii - 1].Float;
    y1 = TablaCalibracion[ii].Float;
    y2 = TablaCalibracion[ii + 1].Float;
    y3 = TablaCalibracion[ii + 2].Float;    
  };
  
  Test1 = ii;                                                                                               // Debug
  c = 0;                                                                                                    // Debug

  a0 = -0.5 * y0 + 1.5 * y1 - 1.5 * y2 + 0.5 * y3;
  a1 = y0 - 2.5 * y1 + 2.0 * y2 - 0.5 * y3;
  a2 = -0.5 * y0 + 0.5 * y2;
  a3 = y1;

  x = (Dato - TablaCalibracion[ii].Float) / (TablaCalibracion[ii + 1].Float - TablaCalibracion[ii].Float);  // Se aproxima el valor inicial de x0
  if (Dato < TablaCalibracion[2].Float) k = 0.02; else k = 0.004;
  
  do
  {
    fx = (a0 * x * x * x) + (a1 * x * x) + (a2 * x) + a3;                                                   // fx(x1)
    dif = fx - Dato;
    x -= dif * k;
    c++;                                                                                                    // Debug, contador de número de iteraciones
  } while ((abs(dif) >= 0.1) && (c < 50));                                                                  // Se evalua la ecuación en x1

  Test4 = c;
  Test3 = fx;
  DBm = -130.0;
  DBm += 5.0 * d;
  DBm += 5.0 * x;                                                                                           // Interpolamos la lectura del ADC y se convierte a DBm 
  if (DBm < DBm_Minimo) DBm = DBm_Minimo;
  else if (DBm > DBm_Maximo) DBm = DBm_Maximo;
  
  ValorNuevo++;                                                                                             // Unsigned int, de manera que va de 0 a 65535, para tener la indicacion de las actualizaciones del ADC.

  return;
}

void ActualizarPantalla(void)
{
  if (Menu != MenuCalibracion)
  {
    int barra;

    DBrel = DBm - Cero;
    barra = (int)(DBrel / (Escala / 127.0));                                                                // Se representa el valor en dB con el fondo de escala "Escala"
      
    if (barra < 0)
    {
      barra = 0;
    } else if (barra > 127) barra = 127;

    if (BorrarMenuCalibracion == true)                                                                      // Hemos vuelto a la pantalla principal de medidas
    {
      lcd.clear();
      PintarEscala();
      //BuscaCero = true;                                                                                   // Descomentar si se quiere hacer un buscaceros cuando volvalmos del menu principal
      BorrarMenuCalibracion = false;
    };
    
    PintarBarra(barra, Iniciar_Barra);
    PintarPunta(barra);
    PintarMemoriaOsciloscopio(barra);
  } else PintarMenu();

  lcd.flush();                                                                                              // Se envía la pantalla pintada en memoria a la pantalla física

  return;
}

void CargarValoresEEprom(void)
{
  int poscero;

  poscero = EEPROM.read(0);
  
  if ((poscero == 100) && (GrabarMemoria == false))                                                          // ¿Está inicializada la EEPROM?
  {
    K1 = EEPROM.read(1); 
    K2 = EEPROM.read(2);   
    K3 = EEPROM.read(3);  
    K4 = EEPROM.read(4);  
    K5 = EEPROM.read(5);   
    Escala1 = EEPROM.read(6); 
    Escala2 = EEPROM.read(7);   
    Escala3 = EEPROM.read(8);   
    Escala4 = EEPROM.read(9);   
    Escala5 = EEPROM.read(10);   
    Contraste = EEPROM.read(11);
    
    for (int i = 0; i < Puntos_Calibracion; i++)
    {
      for (int ii = 0; ii < 4; ii++)
      {
        TablaCalibracion[i].Byte[ii] = EEPROM.read(12 + (i * 4) + ii);       
      };
    };
    Escala = Escala5;
    IncCero = (float)Escala / 127.0;
  } else
  {
    if (GrabarMemoria == false)
    {
      for (int i = 0; i < Puntos_Calibracion; i++)                                                          // Inicialización de la tabla de calitración para medidores nuevos
      {
        TablaCalibracion[i].Float = (4096.0 / Puntos_Calibracion) * i + (4096.0 / Puntos_Calibracion);      // Se suma el menor valor, para que sirva de offset simulando la lectura mínima de ruido
      };
    } else Timer4.pause();
    EEPROM.write(0, 100);                                                                                   // Grabamos el código de inicialización en la primera posición   
    EEPROM.write(1, K1);  
    EEPROM.write(2, K2);  
    EEPROM.write(3, K3);   
    EEPROM.write(4, K4);  
    EEPROM.write(5, K5);  
    EEPROM.write(6, Escala1);  
    EEPROM.write(7, Escala2);   
    EEPROM.write(8, Escala3);   
    EEPROM.write(9, Escala4);   
    EEPROM.write(10, Escala5); 
    EEPROM.write(11, Contraste);
    for (int i = 0; i < Puntos_Calibracion; i++)
    {
      for (int ii = 0; ii < 4; ii++)
      {
        EEPROM.write(12 + (i * 4) + ii, TablaCalibracion[i].Byte[ii]);  
      };
    };
    if (GrabarMemoria == true)
    {
      GrabarMemoria = false;
      Timer4.resume();
    };
  };

  return;
}

bool CalcularMedia(float dato, unsigned int cantidadValores)                                                // Rutina no bloqueante, procesa un dato cada vez que se llama. Devuelve true cuando ha terminado de procesar los datos.
{
  if (InicioMedia == false)
  {
    TiempoInicioMedia = millis();
    InicioMedia = true;
    Media = 0.0;
    MediaCont = 0;
  } else if ((millis() - TiempoInicioMedia) > 200)                                                          // Esperamos x milisegunos para comenzar a buscar el cero para que los pulsadores se estabilicen y no distorsionen la señal
  {
    if (ValorNuevo != ValorAnterior)                                                                        // ValorNuevo es un contador que se incrementa en uno cada vez que el ADC hace una conversion, se usa para comprobar que el ADC haya hecho una conversión nueva
    {
      Media += dato;
      MediaCont++;
      if (MediaCont == cantidadValores)                                                                     // Si hemos recogido el maximo de valores, hacemos la media
      {
        Media /= MediaCont;
        InicioMedia = false;
        return true;                                                                                        // Devolvemos TRUE para indicar que el valor que contenga CERO es el calculo de la media
      };
      ValorAnterior = ValorNuevo;
    };
  };

  return false;                                                                                             // Devolvemos FALSE para indicar que se esta calculando la media
}

void IniciarContadores(void)
{
  Timer4.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  Timer4.pause();
  Timer4.setPeriod(20000);                                                                                   // Se llama a la rutina de la interrupción cada 20000 microsegundos = 50 veces por segundo
  Timer4.setCompare1(1);
  Timer4.attachCompare1Interrupt(LeerValor);
  //Timer4.resume();

  TIMER1->regs.gen->CR1 = 0x0000;                                                                           // Se deshabilita el contador
  TIMER1->regs.gen->PSC = 0x0000;                                                                           // Hay que poner un cero en el preescaler del TIMER1 para que cuente a 72Mhz, porque las librerias lo arrancan a 2 (36 Mhz)
  TIMER1->regs.gen->CCMR1 |= 0x6800;                                                                        // Output Compare CH2
  TIMER1->regs.gen->CCER |= 0x0010;                                                                         // CH2 Output Compare ON - Polarity HIGH
  pinMode(Timer1Ch2, PWM);                                                                                  // No hay manera de acceder al registro BDTR para activar el OUTPUT COMPARE, asi hace lo mismo
  TIMER1->regs.gen->CCR2 = Contraste * 256;                                                                 // PWM para controlar el contraste de la pantalla
  GPIOA->regs->CRH &= 0xFFFFFF3F;                                                                           // Se hace un and para poner los ceros
  GPIOA->regs->CRH |= 0x000000C0;                                                                           // Se hace un or para poner los unos, alternate function open drain PA9 para PWM de 5V
  TIMER1->regs.gen->CR1 = 0x0001;                                                                           // Se habilita el contador para que genere el PWM

  return;
}

void LeerPulsadores(void)
{
  bool pulsador1, pulsador2, pulsador3, pulsador4;

  pulsador1 = digitalRead(Boton1);                                                                          // pulsador1 = Menu
  if ((pulsador1 == LOW) && (Pulsador1_Ant == HIGH))                                                        // ¿Se acaba de pulsar el botón?.
  {
    if (Pulsador1 == false)                                                                                 // ¿Ha sido la primera pulsación?.
    {
      TiempoPulsador1 = millis();                                                                           // Se inicia el contador de tiempo de pulsación del botón.
      Pulsador1 = true;
      if (Menu == MenuCalibracion) Pulsador1 = false;                                                       // Si estamos en el menu de calibracion desactivamos el tiempo de pulsacion larga.
      else Pulsador1 = true;                                                                                // Se memoriza que ha sido la primera pulsación.
    };
  } else if ((pulsador1 == LOW) && (Pulsador1_Ant == LOW))                                                  // ¿Se acaba de soltar el botón?.
  {
    if (((millis() - TiempoPulsador1) > 1000) && (Menu != MenuCalibracion))                                 // ¿La pulsación es larga?.
    {
      Menu = MenuCalibracion;                                                                               // Activamos el menú de calibracion por pulsación larga.
      SubMenu = 0;
      Pulsador1 = false;                                                                                    // Ya puede pulsarse por primera vez el botón de nuevo.
      TiempoInicioRepeticionPulsacion = 65535;                                                              // Para desactivar la repeticion de las teclas en los menus de calibracion
      BorrarPantalla = true;
    };
  } else if ((pulsador1 == HIGH) && (Pulsador1_Ant == LOW) && ((millis() - TiempoPulsador1) < 1000))
  {
    if (Menu == MenuCalibracion)
    {
      BorrarMenuCalibracion = true;                                                                         // Para borrar el menu de calibracion cuando se vuelve al menu normal   
      //BorrarPantalla = true;                                                                              // Para que la proxima vez que entre en el menu de calibracion borre la pantalla de medidas una sola vez y no este continuamente borrando el menu de calibracion
      TiempoInicioRepeticionPulsacion = 1000;                                                               // Para activar la repeticion de las teclas en los menus de medida
      SubMenu1 = false;

      if ((SubMenu == 1) && (Seleccion == Puntos_Calibracion))                                              // Al salir del menu de calibracion de db grabamos los datos a la eeprom si se han completado todos los puntos de calibracion
      {
        GrabarMemoria = true;
      } else if (SubMenu == 2)                                                                              // Al salir del menu de ajuste de contraste grabamos el contraste en la eeprom.
        {
          GrabarMemoria = true;
        } else if ((SubMenu == 3) || (SubMenu == 4))                                                        // Al salir del menu de ajuste del filtro de señal...
          {
            Escala = Escala5;
            K = K5;
            CambioEscala = true;
            GrabarMemoria = true;
          };
      Seleccion = 0;                                                                                        // Preparamos a qué submenu se preseleccionara la proxima vez que se entre en los menus
                                                                                                            // Esta línea debajo del if anterior obligatoriamente porque preguntamos por Seleccion.
    };

    Menu++;                                                                                                 // Avanzamos en el menú.
    if (Menu > (MenuCalibracion - 1))
    {
      Menu = 0;                                                                                             // Vamos recorriendo los distintos menus.

      lcd.setCursor(Y1, 117);
      lcd.print("    ");
      if (Escala < 10) lcd.setCursor(Y1, 117+5);
      else lcd.setCursor(Y1, 117);
      lcd.textInvert(false);
      lcd.print(Escala);                                                                                    // Se borra el fondo negro del Menu1 en la indicación de fondo de escala
    };
    Pulsador1 = false;                                                                                      // Ya puede pulsarse por primera vez el botón de nuevo.
    //CambioEscala = true;
    Iniciar_Barra = true;
  };

  pulsador2 = digitalRead(Boton2);                                                                          // pulsador2 = Subir
  if ((pulsador2 == LOW) && (Pulsador2_Ant == HIGH))                                                        // ¿Se acaba de pulsar el botón?.
  {
    if (Pulsador2 == false)                                                                                 // ¿Ha sido la primera pulsación?.
    {
      TiempoInicioRepeticionPulsador2 = millis();                                                           // Se inicia el contador de tiempo de pulsación del botón.
      Pulsador2 = true;                                                                                     // Se memoriza que ha sido la primera pulsación.
    };
  } else
  { 
    if ((pulsador2 == LOW) && (Pulsador2_Ant == LOW))                                                       // ¿Sigue pulsador el botón?
    {
      if ((millis() - TiempoInicioRepeticionPulsador2) > TiempoInicioRepeticionPulsacion)                   // Se repite la acción de aumentar a razón de "TiempoRepeticionPulsacion" milisegundos.
      {
        if (Pulsador2Repeticion == false)
        {
          TiempoPulsador2 = millis();
          Pulsador2Repeticion = true;
        };
        if ((millis() - TiempoPulsador2) > TiempoRepeticionPulsacion)
        {
          Pulsadores_Subir(Menu);                                                                                      // Acción de aumentar, dependendiendo del menú ser hará una cosa u otra.
          Pulsador2Repeticion = false;
        };
      };
    } else if ((pulsador2 == HIGH) && (Pulsador2_Ant == LOW))                                               // ¿Se acaba de soltar el boton?
    {
      Pulsadores_Subir(Menu);                                                                                          // Acción de aumentar, dependendiendo del menú se hará una cosa u otra.
      Pulsador2 = false;
      Pulsador2Repeticion = false;
    };
  };

  pulsador3 = digitalRead(Boton3);                                                                          // pulsador3 = Bajar
  if ((pulsador3 == LOW) && (Pulsador3_Ant == HIGH))                                                        // ¿Se acaba de pulsar el botón?.
  {
    if (Pulsador3 == false)                                                                                 // ¿Ha sido la primera pulsación?.
    {
      TiempoInicioRepeticionPulsador3 = millis();                                                           // Se inicia el contador de tiempo de pulsación del botón.
      Pulsador3 = true;                                                                                     // Se memoriza que ha sido la primera pulsación.
    };
  } else
  { 
    if ((pulsador3 == LOW) && (Pulsador3_Ant == LOW))                                                       // ¿Sigue pulsador el botón?
    {
      if ((millis() - TiempoInicioRepeticionPulsador3) > TiempoInicioRepeticionPulsacion)                   // Se repite la acción de aumentar a razón de "TiempoRepeticionPulsacion" milisegundos.
      {
        if (Pulsador3Repeticion == false)
        {
          TiempoPulsador3 = millis();
          Pulsador3Repeticion = true;
        };
        if ((millis() - TiempoPulsador3) > TiempoRepeticionPulsacion)
        {
          Pulsadores_Bajar(Menu);                                                                                      // Acción de disminuir, dependendiendo del menú se hará una cosa u otra.
          Pulsador3Repeticion = false;
        };
      };
    } else if ((pulsador3 == HIGH) && (Pulsador3_Ant == LOW))                                               // ¿Se acaba de soltar el boton?
    {
      Pulsadores_Bajar(Menu);                                                                                          // Acción de disminuir, dependendiendo del menú se hará una cosa u otra.
      Pulsador3 = false;
      Pulsador3Repeticion = false;
    };
  };

  pulsador4 = digitalRead(Boton4);                                                                          // pulsador4 = Seleccion
  if ((pulsador4 == LOW) && (Pulsador4_Ant == HIGH))                                                        // ¿Se acaba de pulsar el botón?.
  {
    if (Pulsador4 == false)                                                                                 // ¿Ha sido la primera pulsación?.
    {
      Pulsador4 = true;                                                                                     // Se memoriza que ha sido la primera pulsación.
      DebugMilis = millis();
    };
  } else if ((pulsador4 == HIGH) && (Pulsador4_Ant == LOW))                                                 // ¿Se acaba de soltar el boton?
  {
    Pulsadores_Seleccionar(Menu);                                                                                      // Acción de seleccionar, dependendiendo del menú se hará una cosa u otra.
    Pulsador4 = false;
  };

  Pulsador1_Ant = pulsador1;
  Pulsador2_Ant = pulsador2;
  Pulsador3_Ant = pulsador3;
  Pulsador4_Ant = pulsador4;

  if (Menu == 1)                                                                                            // Parpadeo texto fondo de escala en el menu1
  {
    if ((millis() - Menu1Parpadeo) > 500)
    {
      lcd.setCursor(Y1, 117);
      lcd.print("    ");
      if (Escala < 10) lcd.setCursor(Y1, 117+5);
      else lcd.setCursor(Y1, 117);
      Menu1Parpadeo = millis();     
      if (Menu1ParpadeoOK == true)
      {
        lcd.textInvert(true);
        lcd.print(Escala);
        lcd.textInvert(false);
        Menu1ParpadeoOK = false;
      } else
      {
        lcd.textInvert(false);
        lcd.print(Escala);
        Menu1ParpadeoOK = true;
      };
    };
  };

  return;
}

void PintarBarra(int valor, bool iniciar)
{
  if (!iniciar)
  {
    if (valor > X_ant)
    {
      for (int i = X_ant; i <= valor; i++)
      {
        lcd.fastVline(i, 0, 12, PixelSet);
      };    
    } else if (valor < X_ant)
    {
      for (int i = X_ant; i > valor; i--)
      {
        lcd.fastVline(i, 0, 12, PixelClear);
      };
    };
  } else
  {
    for (int i = 0; i <= 128; i++)
    {
      if (i <= valor)
      {
        lcd.fastVline(i, 0, 12, PixelSet);
      } else lcd.fastVline(i, 0, 12, PixelClear);
    };
    Iniciar_Barra = false;
  };

  X_ant = valor;

  return;
}

void PintarEscala(void)
{
  lcd.fillbox(1, 13, 127, 6, PixelClear);

  if (Escala == 1)
  {
    for (int i = 0; i <= 10 * 2; i++)
    {
      lcd.fastVline(i * 127 / 10, Y0, 3, PixelSet);
      if ((i == 0) || (i == 10))
      {
        lcd.fastVline(i * 127 / 10, Y0, 6, PixelSet);
      };
    };
  } else if ((Escala == 2) || (Escala == 3))
    {
      for (int i = 0; i < Escala * 5; i++)
      {
        lcd.fastVline(i * 127 / (Escala * 5), Y0, 3, PixelSet);
        if (i <= 3)
        {
          lcd.fastVline(i * 127 / Escala, Y0, 6, PixelSet);
        };
      };
    } else if (Escala > 3)
      {
        for (int i = 0; i <= Escala * 2; i++)
        {
          if (Escala < 15)
          {
            lcd.fastVline(i * 127 / (Escala * 2), Y0, 3, PixelSet);
          };
          if (i <= Escala)
          {
            lcd.fastVline(i * 127 / Escala, Y0, 6, PixelSet);
          };
        };
      };

  lcd.setCursor(Y1, 117);
  lcd.print("    ");
  if (Escala < 10) lcd.setCursor(Y1, 117+5);
  else if (Escala < 100) lcd.setCursor(Y1, 117);
  else lcd.setCursor(Y1, 117-5);

  if (Menu == 1) lcd.textInvert(true);
  lcd.print(Escala);
  if (Menu == 1) lcd.textInvert(false);

  return;
}

void PintarMemoriaOsciloscopio(int valor)
{
  if (TiempoMemoria > 200)                                                                                  // Se pinta un valor nuevo cada x milisegundos
  {
    lcd.fillbox(0, Y1, 117, 7, PixelClear);                                                                 // lcd.fillbox(X, Y, cantidad de x a la derecha respecto de X, cantidad de y abajo respecto de Y)
    lcd.setCursor(Y1, 0);
    lcd.print(DBrel, 2);
    lcd.print("dB");
    lcd.setCursor(Y1, 55);
    lcd.print(DBm, 2);
    lcd.print("dBm");

    for (int i = 127; i > 0; i--)
    {
      lcd.line(i, 63 - Memoria[i], i - 1, 63 - Memoria[i - 1], PixelClear);                                 // Borro la pantalla anterior
    };

    Memoria[0] = valor/4;                                                                                   // Maximo 32 puntos porque 128/4 = 32
    for (int i = 127; i > 0; i--)
    {
      Memoria[i] = Memoria[i - 1];                                                                          // Se mueven los puntos a la derecha
    };

    for (int i = 127; i > 0; i--)
    {
      lcd.line(i, 63 - Memoria[i], i - 1, 63 - Memoria[i - 1], PixelSet);                                   // Pinto la pantalla nueva
    };

    if (Debug)
    {
      lcd.fillbox(0, Y1 + 10, 128, 33, PixelClear);
      lcd.setCursor(Y1 + 9, 0);
      lcd.print(Dato, 2);
      lcd.setCursor(Y1 + 18, 0);
      lcd.print(Test1);
      lcd.print(">");
      lcd.print(TablaCalibracion[Test1].Float, 2);
      lcd.setCursor(Y1 + 27, 0);
      lcd.print(Test2);
      lcd.print(">");
      lcd.print(TablaCalibracion[Test2].Float, 2);
      lcd.setCursor(Y1 + 36, 0);
      lcd.print(Test4);
      lcd.print("  ");
      lcd.print(Test3, 7);
    };

    TiempoMemoria = 0;

    digitalWrite(LED, !digitalRead(LED));
  };

  TiempoMemoria += millis() - TiempoMemoriaPasado;                                                          // Se incrementa el tiempo transcurrido en milisegundos
  TiempoMemoriaPasado = millis();

  return;
}

void PintarPunta(int valor)
{
  if (valor > X_punta)                                                                                      // Si la barra es mayor que el punto, se actualiza el valor del punto
  {
    X_punta = valor;
    TiempoPunta = 0;
    XigualAnt = true;                                                                                       // Se habilita el pintado del punto
  } else if (valor < X_punta)                                                                               // Si la barra es menor que el punto
  {
    if (TiempoPunta > 2000)                                                                                 // Tiempo en milisegundos que permanece pintado el punto
    {
      lcd.fastVline(X_punta, 0, 12, PixelClear);                                                            // Se borra la barra pasado el tiempo de espera
      TiempoPunta = 0;
      X_punta = valor;
      XigualAnt = false;                                                                                    // Deshabilitamos el pintado del punto porque lo acabamos de borrar
    } else if (XigualAnt)                                                                                   // Si en la pasada anterior el punto y la barra eran iguales, ahora la barra es menor, por lo que habrá borrado el punto, asi que hay que pintarlo
    {
      lcd.fastVline(X_punta, 0, 12, PixelSet);
      XigualAnt = false;
    };
  } else if (valor == X_punta)                                                                              // Si la barra y el punto son iguales se memoriza, por si la siguiente pasada es menor y hay que pintar el punto al borrarlo la barra
  {
    TiempoPunta = 0;
    XigualAnt = true;
  };  
  
  TiempoPunta += millis() - TiempoPuntaPasado;                                                              // Se incrementa el tiempo transcurrido en milisegundos
  TiempoPuntaPasado = millis();
  
  return;
}

void PintarMenu(void)
{
  if (BorrarPantalla == true)                                                                               // Borra la pantalla de medidas antes de pintar la pantalla de menu principal por primera vez
  {
    lcd.clear();
    BorrarPantalla = false;
  };
  
  switch (SubMenu)
  {
    case 0:                                                                                                 // menu principal
      lcd.fillbox(0, 0, 128, 9, PixelSet);
      lcd.textInvert(true);
      lcd.setCursor(1, 40);
      lcd.print("MAIN MENU");
      lcd.textInvert(false);          
      switch (Seleccion)
      {
        case 0:                
          lcd.fillbox(0, 12, 128, 9, PixelSet);
          lcd.textInvert(true);
          lcd.setCursor(13, 20);          
          lcd.print("DBM READINGS CAL.");
          lcd.textInvert(false);

          lcd.fillbox(0, 22, 128, 9, PixelClear);
          lcd.setCursor(23, 20);          
          lcd.print("SCREEN CONSTRAST");

          lcd.fillbox(0, 32, 128, 9, PixelClear);
          lcd.setCursor(33, 10);          
          lcd.print("SIGNAL FILTER ADJUST");

          lcd.fillbox(0, 42, 128, 9, PixelClear);
          lcd.setCursor(43, 30);          
          lcd.print("METER SCALES");
        break;
                
        case 1:       
          lcd.fillbox(0, 12, 128, 9, PixelClear);
          lcd.setCursor(13, 20);          
          lcd.print("DBM READINGS CAL.");

          lcd.fillbox(0, 22, 128, 9, PixelSet);
          lcd.textInvert(true);          
          lcd.setCursor(23, 20);          
          lcd.print("SCREEN CONSTRAST");
          lcd.textInvert(false);          

          lcd.fillbox(0, 32, 128, 9, PixelClear);
          lcd.setCursor(33, 10);          
          lcd.print("SIGNAL FILTER ADJUST");

          lcd.fillbox(0, 42, 128, 9, PixelClear);
          lcd.setCursor(43, 30);          
          lcd.print("METER SCALES");
        break;

        case 2:
          lcd.fillbox(0, 12, 128, 9, PixelClear);
          lcd.setCursor(13, 20);          
          lcd.print("DBM READINGS CAL.");

          lcd.fillbox(0, 22, 128, 9, PixelClear);
          lcd.setCursor(23, 20);          
          lcd.print("SCREEN CONSTRAST");

          lcd.fillbox(0, 32, 128, 9, PixelSet);
          lcd.textInvert(true);
          lcd.setCursor(33, 10);          
          lcd.print("SIGNAL FILTER ADJUST");
          lcd.textInvert(false);          

          lcd.fillbox(0, 42, 128, 9, PixelClear);
          lcd.setCursor(43, 30);          
          lcd.print("METER SCALES");
        break;

        case 3:
          lcd.fillbox(0, 12, 128, 9, PixelClear);
          lcd.setCursor(13, 20);          
          lcd.print("DBM READINGS CAL.");

          lcd.fillbox(0, 22, 128, 9, PixelClear);
          lcd.setCursor(23, 20);          
          lcd.print("SCREEN CONSTRAST");

          lcd.fillbox(0, 32, 128, 9, PixelClear);
          lcd.setCursor(33, 10);          
          lcd.print("SIGNAL FILTER ADJUST");

          lcd.fillbox(0, 42, 128, 9, PixelSet);
          lcd.textInvert(true);
          lcd.setCursor(43, 30);          
          lcd.print("METER SCALES");
          lcd.textInvert(false);          
        break;

        default:
        break;
      };
    break;

    case 1:                                                                                                 // calibrar sensor
      if (SubMenu1)
      {
        lcd.setCursor(1, 0);
        lcd.textInvert(true);
        if (Seleccion < Puntos_Calibracion)
        {
          lcd.box(0, 0, 128, 8, PixelSet);
          lcd.print("                 WAIT                   ");
        } else
        {
          lcd.box(0, 0, 128, 8, PixelSet);
          lcd.print("        ALL DATA SAVED           ");
        };
        lcd.textInvert(false);        
      } else
      {
        lcd.box(0, 0, 128, 8, PixelClear);
        lcd.setCursor(1, 0);      
        lcd.print("APPLY ");
        lcd.print((Seleccion * 5) + DBm_Minimo);
        lcd.print(" dBm ON RF SMA              ");        
      };

      lcd.setCursor(15, 0);
      lcd.print("CHANGE FREQUENCY FOR");
      lcd.setCursor(25, 0);
      lcd.print("MAXIMUN ADC READING");
      lcd.setCursor(40, 0);
      
      if (Seleccion < Puntos_Calibracion)
      {
        lcd.print("ADC READING:");
        lcd.setCursor(40, 70);
        lcd.print(Seleccion);
        lcd.print(" > ");
        lcd.print(Dato, 2);
        delay(100);                                                                                         // Este delay es necesario para que las indicaciones de la variable Dato sean iguales aquí y en la pantalla principal, ¿PORQUE?
        lcd.print("    ");
        lcd.setCursor(50, 0);
        lcd.print("PRESS SELECT WHEN DONE");
      } else
      {
        lcd.fillbox(0, 10, 128, 50, PixelClear);
        lcd.setCursor(50,0);
        lcd.print("PRESS MENU                   ");
      };
    break;

    case 2:                                                                                                 // contraste pantalla
      lcd.setCursor(10, 18);
      lcd.print("ADJUST CONSTRAST");
      lcd.box(17, 19, 94, 8, PixelSet);
      lcd.setCursor(20, 17);
      lcd.textInvert(true);
      lcd.print("ADJUST CONSTRAST");
      lcd.textInvert(false);
      lcd.fillbox(0, 37, (Seleccion / 2), 10, PixelSet);
      lcd.fillbox((Seleccion / 2), 37, (128 - (Seleccion / 2)), 10, PixelClear);
      lcd.setCursor(55, 10);
      lcd.print("PRESS MENU WHEN DONE");
    break;

    case 3:                                                                                                 // filtro de ruido
      lcd.fillbox(0, 0, 128, 9, PixelSet);
      lcd.textInvert(true);
      lcd.setCursor(1, 10);
      lcd.print("SCALE SIGNAL FILTERS");
      lcd.textInvert(false);

      if (SubMenu1)
      {
        if (Escala == 1)
        {
          lcd.fillbox(81, 12, 40, 9, PixelClear);
          lcd.setCursor(13, 84);
          lcd.print((float)0.001*K1, 3);
        } else if (Escala == 2)
        {
          lcd.fillbox(81, 22, 40, 9, PixelClear);
          lcd.setCursor(23, 84);
          lcd.print((float)0.001*K2, 3);                
        } else if (Escala == 3)
        {
          lcd.fillbox(81, 32, 40, 9, PixelClear);
          lcd.setCursor(33, 84);
          lcd.print((float)0.001*K3, 3);
        } else if (Escala == 4)
        {
          lcd.fillbox(81, 42, 40, 9, PixelClear);      
          lcd.setCursor(43, 84);
          lcd.print((float)0.001*K4, 3);        
        } else if (Escala == 5)
        {
          lcd.fillbox(81, 52, 40, 9, PixelClear);
          lcd.setCursor(53, 84);
          lcd.print((float)0.001*K5, 3);               
        };
      } else
      {   
        switch (Seleccion)
        {
          case 0:
            lcd.fillbox(0, 12, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(13, 1);          
            lcd.print("Scale 1 Filter > ");
            lcd.print((float)0.001*K1, 3);
            lcd.textInvert(false);

            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Scale 2 Filter > ");
            lcd.print((float)0.001*K2, 3);

            lcd.fillbox(0, 32, 128, 9, PixelClear);
            lcd.setCursor(33, 1);          
            lcd.print("Scale 3 Filter > ");
            lcd.print((float)0.001*K3, 3);

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Scale 4 Filter > ");
            lcd.print((float)0.001*K4, 3);

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Scale 5 Filter > ");
            lcd.print((float)0.001*K5, 3);
          break;

          case 1:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Scale 1 Filter > ");
            lcd.print((float)0.001*K1, 3);

            lcd.fillbox(0, 22, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(23, 1);          
            lcd.print("Scale 2 Filter > ");
            lcd.print((float)0.001*K2, 3);
            lcd.textInvert(false);

            lcd.fillbox(0, 32, 128, 9, PixelClear);
            lcd.setCursor(33, 1);          
            lcd.print("Scale 3 Filter > ");
            lcd.print((float)0.001*K3, 3);

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Scale 4 Filter > ");
            lcd.print((float)0.001*K4, 3);

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Scale 5 Filter > ");
            lcd.print((float)0.001*K5, 3);       
          break;

          case 2:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Scale 1 Filter > ");
            lcd.print((float)0.001*K1, 3);
          
            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Scale 2 Filter > ");
            lcd.print((float)0.001*K2, 3);

            lcd.fillbox(0, 32, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(33, 1);
            lcd.print("Scale 3 Filter > ");
            lcd.print((float)0.001*K3, 3);
            lcd.textInvert(false);

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);         
            lcd.print("Scale 4 Filter > ");
            lcd.print((float)0.001*K4, 3);

            lcd.fillbox(0, 52, 128, 9, PixelClear); 
            lcd.setCursor(53, 1);          
            lcd.print("Scale 5 Filter > ");
            lcd.print((float)0.001*K5, 3);      
          break;

          case 3:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Scale 1 Filter > ");
            lcd.print((float)0.001*K1, 3);

            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Scale 2 Filter > ");
            lcd.print((float)0.001*K2, 3);

            lcd.fillbox(0, 32, 128, 9, PixelClear);          
            lcd.setCursor(33, 1);          
            lcd.print("Scale 3 Filter > ");
            lcd.print((float)0.001*K3, 3);

            lcd.fillbox(0, 42, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(43, 1);          
            lcd.print("Scale 4 Filter > ");
            lcd.print((float)0.001*K4, 3);
            lcd.textInvert(false);          

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Scale 5 Filter > ");
            lcd.print((float)0.001*K5, 3);        
          break;

          case 4:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Scale 1 Filter > ");
            lcd.print((float)0.001*K1, 3);

            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Scale 2 Filter > ");
            lcd.print((float)0.001*K2, 3);

            lcd.fillbox(0, 32, 128, 9, PixelClear);
            lcd.setCursor(33, 1);          
            lcd.print("Scale 3 Filter > ");
            lcd.print((float)0.001*K3, 3);

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Scale 4 Filter > ");
            lcd.print((float)0.001*K4, 3);

            lcd.fillbox(0, 52, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(53, 1);          
            lcd.print("Scale 5 Filter > ");
            lcd.print((float)0.001*K5, 3);
            lcd.textInvert(false);                   
          break;

          default:
          break;
        };
      };
    break;

    case 4:
      lcd.fillbox(0, 0, 128, 9, PixelSet);
      lcd.textInvert(true);
      lcd.setCursor(1, 30);
      lcd.print("METER SCALES");
      lcd.textInvert(false);
      
      if (SubMenu1)
      {
        if (Escala == 1)
        {
          lcd.fillbox(70, 12, 40, 9, PixelClear);
          lcd.setCursor(13, 75);
          lcd.print(Escala1);
          lcd.print(" dB");
        } else if (Escala == 2)
        {
          lcd.fillbox(70, 22, 40, 9, PixelClear);
          lcd.setCursor(23, 75);
          lcd.print(Escala2);
          lcd.print(" dB");                 
        } else if (Escala == 3)
        {
          lcd.fillbox(70, 32, 40, 9, PixelClear);
          lcd.setCursor(33, 75);
          lcd.print(Escala3);
          lcd.print(" dB");
        } else if (Escala == 4)
        {
          lcd.fillbox(70, 42, 40, 9, PixelClear);      
          lcd.setCursor(43, 75);
          lcd.print(Escala4);
          lcd.print(" dB");          
        } else if (Escala == 5)
        {
          lcd.fillbox(70, 52, 40, 9, PixelClear);
          lcd.setCursor(53, 75);
          lcd.print(Escala5);
          lcd.print(" dB");                
        };
      } else
      {
        switch (Seleccion)
        {
          case 0:
            lcd.fillbox(0, 12, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(13, 1);          
            lcd.print("Full Scale 1 > ");
            lcd.print(Escala1);
            lcd.print(" dB");
            lcd.textInvert(false);

            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Full Scale 2 > ");
            lcd.print(Escala2);
            lcd.print(" dB");

            lcd.fillbox(0, 32, 128, 9, PixelClear);
            lcd.setCursor(33, 1);          
            lcd.print("Full Scale 3 > ");
            lcd.print(Escala3);
            lcd.print(" dB");

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Full Scale 4 > ");
            lcd.print(Escala4);
            lcd.print(" dB");

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Full Scale 5 > ");
            lcd.print(Escala5);
            lcd.print(" dB");
          break;

          case 1:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Full Scale 1 > ");
            lcd.print(Escala1);
            lcd.print(" dB");

            lcd.fillbox(0, 22, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(23, 1);          
            lcd.print("Full Scale 2 > ");
            lcd.print(Escala2);
            lcd.print(" dB");
            lcd.textInvert(false);

            lcd.fillbox(0, 32, 128, 9, PixelClear);
            lcd.setCursor(33, 1);          
            lcd.print("Full Scale 3 > ");
            lcd.print(Escala3);
            lcd.print(" dB");

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Full Scale 4 > ");
            lcd.print(Escala4);
            lcd.print(" dB");

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Full Scale 5 > ");
            lcd.print(Escala5);
            lcd.print(" dB");      
          break;

          case 2:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Full Scale 1 > ");
            lcd.print(Escala1);
            lcd.print(" dB");
          
            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Full Scale 2 > ");
            lcd.print(Escala2);
            lcd.print(" dB");

            lcd.fillbox(0, 32, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(33, 1);
            lcd.print("Full Scale 3 > ");
            lcd.print(Escala3);
            lcd.print(" dB");
            lcd.textInvert(false);

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Full Scale 4 > ");
            lcd.print(Escala4);
            lcd.print(" dB");

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Full Scale 5 > ");
            lcd.print(Escala5);
            lcd.print(" dB");      
          break;

          case 3:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Full Scale 1 > ");
            lcd.print(Escala1);
            lcd.print(" dB");

            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Full Scale 2 > ");
            lcd.print(Escala2);
            lcd.print(" dB");

            lcd.fillbox(0, 32, 128, 9, PixelClear);          
            lcd.setCursor(33, 1);          
            lcd.print("Full Scale 3 > ");
            lcd.print(Escala3);
            lcd.print(" dB");

            lcd.fillbox(0, 42, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(43, 1);
            lcd.print("Full Scale 4 > ");          
            lcd.print(Escala4);
            lcd.print(" dB");
            lcd.textInvert(false);          

            lcd.fillbox(0, 52, 128, 9, PixelClear);
            lcd.setCursor(53, 1);          
            lcd.print("Full Scale 5 > ");
            lcd.print(Escala5);
            lcd.print(" dB");        
          break;

          case 4:
            lcd.fillbox(0, 12, 128, 9, PixelClear);        
            lcd.setCursor(13, 1);          
            lcd.print("Full Scale 1 > ");
            lcd.print(Escala1);
            lcd.print(" dB");

            lcd.fillbox(0, 22, 128, 9, PixelClear);
            lcd.setCursor(23, 1);          
            lcd.print("Full Scale 2 > ");
            lcd.print(Escala2);
            lcd.print(" dB");

            lcd.fillbox(0, 32, 128, 9, PixelClear);
            lcd.setCursor(33, 1);          
            lcd.print("Full Scale 3 > ");
            lcd.print(Escala3);
            lcd.print(" dB");

            lcd.fillbox(0, 42, 128, 9, PixelClear);
            lcd.setCursor(43, 1);          
            lcd.print("Full Scale 4 > ");
            lcd.print(Escala4);
            lcd.print(" dB");

            lcd.fillbox(0, 52, 128, 9, PixelSet);
            lcd.textInvert(true);
            lcd.setCursor(53, 1);          
            lcd.print("Full Scale 5 > ");          
            lcd.print(Escala5);
            lcd.print(" dB");
            lcd.textInvert(false);                   
          break;

          default:
          break;
        };         
      };
       
    break;

    default:   
    break;    
  };
  
  return;
}

void Pulsadores_Bajar(int menu)
{
  switch (menu)
  {
    case 0:
      Cero += IncCero;                                                                                      // Esta al reves para que el efecto visual sea al derecho
    break;
    
    case 1:
      CambioEscala = true;
      if (Escala == Escala5)
      {
        K = K4;
        Escala = Escala4;       
      } else if (Escala == Escala4)
      {
        K = K3;
        Escala = Escala3;        
      } else if (Escala == Escala3)
      {
        K = K2;
        Escala = Escala2;             
      } else if (Escala == Escala2)
      {
        K = K1;
        Escala = Escala1;       
      } else if (Escala == Escala1)
      {
        K = K5;
        Escala = Escala5;       
      };

      IncCero = (float)Escala / 127.0;
    break;

    case 2:
      switch (SubMenu)
      {
        case 0:                                                                                             // menu principal
          Seleccion++;
          if (Seleccion > Opciones[SubMenu]) Seleccion = 0;
        break;

        case 1:                                                                                             // calibrar sensor
        break;

        case 2:                                                                                             // contraste pantalla
          Seleccion--;
          if (Seleccion < 1) Seleccion = 1;
          Contraste = Seleccion;
          TIMER1->regs.gen->CCR2 = Contraste * 256;
        break;

        case 3:                                                                                             // filtro de señal
          if (SubMenu1)
          {
            switch (Escala)
            {
              case 1:
                K1--;
                if (K1 < 1) K1 = 1;
              break;
              
              case 2:
                K2--;
                if (K2 < 1) K2 = 1;
              break;
              
              case 3:
                K3--;
                if (K3 < 1) K3 = 1;              
              break;
              
              case 4:
                K4--;
                if (K4 < 1) K4 = 1;              
              break;
              
              case 5:
                K5--;
                if (K5 < 1) K5 = 1;              
              break;

              default:
              break;
            };           
          } else
          {
            Seleccion++;
            if (Seleccion > Opciones[SubMenu]) Seleccion = 0;
            Escala = Seleccion + 1;            
          }; 
        break;

        case 4:                                                                                             // fondo de escala
          if (SubMenu1)
          {
            switch (Escala)
            {
              case 1:
                Escala1--;
                if (Escala1 < 1) Escala1 = 1;
              break;
              
              case 2:
                Escala2--;
                if (Escala2 < 1) Escala2 = 1;
              break;
              
              case 3:
                Escala3--;
                if (Escala3 < 1) Escala3 = 1;              
              break;
              
              case 4:
                Escala4--;
                if (Escala4 < 1) Escala4 = 1;              
              break;
              
              case 5:
                Escala5--;
                if (Escala5 < 1) Escala5 = 1;              
              break;

              default:
              break;
            };           
          } else
          {
            Seleccion++;
            if (Seleccion > Opciones[SubMenu]) Seleccion = 0;
            Escala = Seleccion + 1;            
          };           
        break;        
        
        default:
        break;
      };
    break;

    default:
    break;
  };
  
  return;
}

void Pulsadores_Subir(int menu)
{
  switch (menu)
  {
    case 0:
      Cero -= IncCero;                                                                                      // Esta al reves para que el efecto visual sea al derecho
    break;
    
    case 1:
      CambioEscala = true;
      if (Escala == Escala5)
      {
        K = K1;
        Escala = Escala1;       
      } else if (Escala == Escala4)
      {
        K = K5;
        Escala = Escala5;        
      } else if (Escala == Escala3)
      {
        K = K4;
        Escala = Escala4;             
      } else if (Escala == Escala2)
      {
        K = K3;
        Escala = Escala3;       
      } else if (Escala == Escala1)
      {
        K = K2;
        Escala = Escala2;       
      };
      
      IncCero = (float)Escala / 127.0;
    break;

    case 2:
      switch (SubMenu)
      {
        case 0:                                                                                             // menu principal
          Seleccion--;
          if (Seleccion < 0) Seleccion = Opciones[SubMenu];        
        break;

        case 1:                                                                                             // calibrar sensor
        break;

        case 2:                                                                                             // contraste pantalla
          Seleccion++;
          if (Seleccion > 255) Seleccion = 255;
          Contraste = Seleccion;
          TIMER1->regs.gen->CCR2 = Contraste * 256;
        break;

        case 3:                                                                                             // filtro de señal
          if (SubMenu1)
          {
            switch (Escala)
            {
              case 1:
                K1++;
                if (K1 > 900) K1 = 900;
              break;
              
              case 2:
                K2++;
                if (K2 > 900) K2 = 900;
              break;
              
              case 3:
                K3++;
                if (K3 > 900) K3 = 900;              
              break;
              
              case 4:
                K4++;
                if (K4 > 900) K4 = 900;              
              break;
              
              case 5:
                K5++;
                if (K5 > 900) K5 = 900;              
              break;

              default:
              break;
            };
          } else        
          {
            Seleccion--;
            if (Seleccion < 0) Seleccion = Opciones[SubMenu];
            Escala = Seleccion + 1;
          };        
        break;

        case 4:                                                                                             // fondo de escala
          if (SubMenu1)
          {
            switch (Escala)
            {
              case 1:
                Escala1++;
                if (Escala1 > 100) Escala1 = 100;
              break;
              
              case 2:
                Escala2++;
                if (Escala2 > 100) Escala2 = 100;
              break;
              
              case 3:
                Escala3++;
                if (Escala3 > 100) Escala3 = 100;              
              break;
              
              case 4:
                Escala4++;
                if (Escala4 > 100) Escala4 = 100;              
              break;
              
              case 5:
                Escala5++;
                if (Escala5 > 100) Escala5 = 100;              
              break;

              default:
              break;
            };
          } else
          {
            Seleccion--;
            if (Seleccion < 0) Seleccion = Opciones[SubMenu];
            Escala = Seleccion + 1;
          };       
        break;        
        
        default:
        break;
      };
    break;  

    default:
    break;
  };
  
  return;
}

void Pulsadores_Seleccionar(int menu)
{
  switch (menu)
  {
    case 0:
      BuscaCero = true;

      if (Debug)
      {
        Test2++;
        if (Test2 > (Puntos_Calibracion - 1)) Test2 = 0;
      };      
    break;

    case 1:
      if (Debug == false)
      {
        if ((millis() - DebugMilis) > 2000) Debug = true;
      } else
      {
        if ((millis() - DebugMilis) > 2000) Debug = false;
        lcd.fillbox(0, Y1 + 9, 128, 34, PixelClear);
      };
    break;

    case 2:
      switch (SubMenu)
      {
        case 0:                                                                                             // Menu Principal
          SubMenu = Seleccion + 1;                                                                          // Se selecciona el siguiente Menu, 1 = dbm cal, 2 = contraste, 3 = filtro de señal, 4 = fondo de escala.
          BorrarPantalla = true;
          
          Seleccion = 0;                                                                                    // Nuevo SubMenu = 1, Seleccion apunta al primer dato del menu de calibracion.
          SubMenu1 = false;                                                                                 // Nuevo SubMenu = 1, Se prepara la variable SubMenu1 para ir cambiando de punto de calibración.
          if (SubMenu == 2)                                                                                 // Nuevo SubMenu = 2
          {
            TiempoInicioRepeticionPulsacion = 1000;                                                         // Se activa la repetición de los pulsadores para ajustar el contraste mas comodamente.
            Seleccion = Contraste;                                                                          // Ponemos el PWM al valor grabado en le EEPROM
          };
          if ((SubMenu == 3) || (SubMenu == 4))                                                             // Nuevo SubMenu = 3 o 4
          {
            Escala = 1;                                                                                     // Inicializamos Escala para que coincida con la primera indicación en pantalla
          };
        break;

        case 1:                                                                                             // Calibracion lecturas db.
          SubMenu1 = true;
        break;

        case 2:                                                                                             // Contraste pantalla.
        break;

        case 3:                                                                                             // Filtro de señal
          SubMenu1 = !SubMenu1;
        break;

        case 4:                                                                                             // Fondo de escala
          SubMenu1 = !SubMenu1;
        break;

        default:
        break;
      };
    break;

    default:
    break;
  };
  
  return;
}
