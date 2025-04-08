

#include "Arduino.h" //Librería principal para trabajar con funciones bases de arduino
#include <MPU6050_tockn.h> //Para leer los datos del acelerómetro
#include "LedControl.h" // Para controlar las matrices led con  MAX7219
#include "Delay.h" //librería personalizada para manejar retrasos sin bloquear


//Definición de pines y constantes
#define  MATRIX_A  1 //define cómo estan conectadas las 2 matrices led
#define MATRIX_B  0

MPU6050 mpu6050(Wire); //Crea un objeto de tipo MPU6050 utilizando Wire(I2C que utiliza el sensor para comunicarse)

// Fijan el movimiento del acelerómetro según su valor (Utilizadas posteriormente para condiciones)
#define ACC_THRESHOLD_LOW -25
#define ACC_THRESHOLD_HIGH 25

// Pines de control para las matrices led (SPI)
#define PIN_DATAIN 5 //Pin de envío de bits a la matriz led
#define PIN_CLK 4 //Define el pin 4 como reloj
#define PIN_LOAD 6 //Define el pin 6 como LOAD (Se activa para mandar los datos a la matriz)

// Acelerómetro
#define PIN_X  mpu6050.getAngleX() //Inclinación en el eje X (Adelante, atrás)
#define PIN_Y  mpu6050.getAngleY() //Inclinación lateral (Izquierda, derecha)

// Pines de señal del rotatory encoder
#define PIN_ENC_1 3 //Con estos 2 pines, sabe hacía qué
#define PIN_ENC_2 2 //Dirección está girando y cuánto ha girado

#define PIN_ENC_BUTTON 7 //Pin donde está el botón integrado al rotatory encoder

#define PIN_BUZZER 14 //Pin que controla el buzzer

#define ROTATION_OFFSET 90 //Ajustar la orientación de la matriz led

#define DEBOUNCE_THRESHOLD 500 //Creamos este valor en milisegundo para evitar el rebote mecánico del botón

#define DELAY_FRAME 100 //Velocidad de actualización de la animación

#define DEBUG_OUTPUT 1 //Para poder definir condiciones según si estamos depurando o no

#define MODE_HOURGLASS 0 //Modo normal del reloj de arena
#define MODE_SETMINUTES 1 //Permite ajustar los minutos del reloj
#define MODE_SETHOURS 2 //Permite ajustar las horas del reloj

byte delayHours = 0; //Tiempo en horas que va a contabilizar el reloj
byte delayMinutes = 1; //Tiempo en minutos que va a contabilizar el reloj

int mode = MODE_HOURGLASS; //Variable que guarda el estado actual del reloj
int gravity; //Variable que va a guardar el valor de la gravedad detectada 

LedControl lc = LedControl(PIN_DATAIN, PIN_CLK, PIN_LOAD, 2); // lc de tipo LedControl que controla la matriz con las variables ya definidas
NonBlockDelay d; //Un objeto de tipo NonBlockDelay que está definida en el archivo

int resetCounter = 0; //Contador de programa para saber si el reloj se giró o reseteó
bool alarmWentOff = false; //Para indicar si la alarma ya sonó (Evitamos que suene más de una vez)


// Calcula cuánto tiempo en segundos debe pasar para la siguiente caida de particula
long getDelayDrop() {
  //Ya que tenemos exactamente 60 particulas, no es necesario multiplicar el tiempo total entre 60 y dividirlo entre 60, ya que se cancelan dejando la fórmula que ya tenemos

  return delayMinutes + delayHours * 60; //Multiplicamos las horas por 60 para poder obtener el tiempo en minutos, así sumando con los minutos y obtener el tiempo total
}


#if DEBUG_OUTPUT //Como ya comentamos antes, este bloque de código se ejecutará solo si estamos en modo debug, de lo contrario se pasará por ato
void printmatrix() { //Dibuja un matriz 8x8 en el monitor serial para visualizar cómo están encendidos los leds
  Serial.println(" 0123-4567 "); //Ennumeración de las columnas
  for (int y = 0; y<8; y++) { //Recorre las filas (0 - 7)
   
    if (y == 4) { //Cuando llega a 4, refleja la linea divisora de la matriz
      Serial.println("|----|----|");
    }
    Serial.print(y); // Imprime cada fila

    for (int x = 0; x<8; x++) { //Recorre las columnas
      if (x == 4) {
        Serial.print("|"); //Divisor en 4
      }
      Serial.print(lc.getXY(0,x,y) ? "X" :" "); //lc.getXY es una función que devuelve True si está encendido en esa posición
    }
    Serial.println("|");
  }
  Serial.println("-----------");
}
#endif


//Funciones auxiliares para movernos dentro de una matriz 2D
//coord viene de nuestra estructura en ledControl.h

coord getDown(int x, int y) { // Baja una posición en diagonal hacia la izquierda
  coord xy;
  xy.x = x-1;
  xy.y = y+1;
  return xy;
}
coord getLeft(int x, int y) { //Se mueve una columna a la izquierda
  coord xy;
  xy.x = x-1;
  xy.y = y;
  return xy;
}
coord getRight(int x, int y) { //Se mueve una fila hacia abajo
  coord xy;
  xy.x = x;
  xy.y = y+1;
  return xy;
}


//Simulación del movimiento de una partícula
//Addr para saber en qué matriz está, "x" y "y" para saber la posición de la particula

bool canGoLeft(int addr, int x, int y) { //Puedes ir a la izquierda
  if (x == 0) return false; // Si x == 0 significa que ya está en el borde izquierdo y no puede moverse más
  return !lc.getXY(addr, getLeft(x, y)); // Si la fila izquierda está vacía entonces puede moverse ahí
}
bool canGoRight(int addr, int x, int y) { //Puedes ir a la derecha
  if (y == 7) return false; // Si y == 7 significa que está en el borde derecho, por lo tanto no se puede mover más
  return !lc.getXY(addr, getRight(x, y)); // Si la fila derecha está vacía, puede moverse ahí
}
bool canGoDown(int addr, int x, int y) { //Puedes ir abajo
  if (y == 7) return false; // Está en el borde inferior (No se puede mover más)
  if (x == 0) return false; // Está en el borde izquierdo (No se puede mover más)
  if (!canGoLeft(addr, x, y)) return false; //Si no puede ir a la izquierda
  if (!canGoRight(addr, x, y)) return false; //Si no puede ir a la derecha
  return !lc.getXY(addr, getDown(x, y)); // Revisar si la celda de abajo diagonal está libre
}


//El método lc.setXY enciende o apaga un led en la posición de la matriz

void goDown(int addr, int x, int y) { //Mueve la partícula en diagonal hacia abajo a la izquierda
  lc.setXY(addr, x, y, false); //Apaga el led
  lc.setXY(addr, getDown(x,y), true); //Enciende el led abajo a la izquierda
}
void goLeft(int addr, int x, int y) { //Mueve la partícula una columna a la izquierda
  lc.setXY(addr, x, y, false); //Apaga el led actual
  lc.setXY(addr, getLeft(x,y), true); //Enciende el led a la izquierda
}
void goRight(int addr, int x, int y) { //Mueve la partícula una columna a la derecha
  lc.setXY(addr, x, y, false); //Apaga el led actual
  lc.setXY(addr, getRight(x,y), true); //Enciende el led a la derecha
}


//Contamos cuántos leds están encendidos
int countParticles(int addr) { 
  int c = 0; //Contador de matrices encendidas
  for (byte y=0; y<8; y++) { //Doble bucle for para contar todas las posiciones de la matriz 8x8
    for (byte x=0; x<8; x++) {
      if (lc.getXY(addr, x, y)) { //Si el led de esa posición está encendido, suma 1 al contador
        c++;
      }
    }
  }
  return c;
}

//Mover una partícula individual dentro de la matriz led
bool moveParticle(int addr, int x, int y) {
  if (!lc.getXY(addr,x,y)) { //Si no hay una partícula en esa dirección, no hay nada que mover y se sale de la función
    return false;
  }

  bool can_GoLeft = canGoLeft(addr, x, y); //Consulta si puede moverse a la izquierda
  bool can_GoRight = canGoRight(addr, x, y); // o derecha

  if (!can_GoLeft && !can_GoRight) {
    return false; // Está atrapada, por lo tanto, no se mueve
  }

  bool can_GoDown = canGoDown(addr, x, y); //Consulta si puede ir abajo diagonal hacia la izquierda

  if (can_GoDown) { // Si puede ir abajo
    goDown(addr, x, y); //Va abajo
  } else if (can_GoLeft&& !can_GoRight) { //Si puede ir a la izquierza pero no a la derecha
    goLeft(addr, x, y); // Va a la izquierda
  } else if (can_GoRight && !can_GoLeft) { //Si puede ir a la derecha pero no a la izquierda
    goRight(addr, x, y); //Va a la derecha
  } else if (random(2) == 1) { //Podemos ir a la izquierda o a la derecha, pero, no abajo. Generamos un random para movernos hacia una dirección
    goLeft(addr, x, y); //Si el random salio el numero 1, Va a la izquierda
  } else {
    goRight(addr, x, y); //Sino, a la derecha
  }
  return true; //Retorna True si sí se ha podido mover
}


//Rellena la cantidad de matrices con un max de partículas siguiendo una diagonal en zigzag
void fill(int addr, int maxcount) { //maxcount, representa cuántos leds queremos encender
  int n = 8;
  byte x,y;
  int count = 0;
  
  //slice representa la suma de x+y (La diagonal actual)
  for (byte slice = 0; slice < 2*n-1; ++slice) { //Recorre las diagonales de la matriz desde la superior izquierda hasta la inferior derecha
    byte z = slice < n ? 0 : slice-n + 1; 
    for (byte j = z; j <= slice-z; ++j) { //Recorre cada punto de la diagonal, para cada slice, genera pares x,y
      //Calcula las coordenadas reales de cada punto
      y = 7-j; //Empieza desde la parte inferior de la matriz
      x = (slice-j); //Mantiene la suma x+y constrate par acada slice
      lc.setXY(addr, x, y, (++count <= maxcount)); //Incrementa el contador count y si aún no supero maxcount, enciende el led x,y
    }
  }
}



/**
 * Detecta la orientación del reloj de arena utilizando el MPU6050
 *
 *     | up | right | left | down |
 * --------------------------------
 * 400 |    |       | y    | x    |
 * 330 | y  | x     | x    | y    |
 * 260 | x  | y     |      |      |
 */
int getGravity() {

  //Obtiene los angulos de inclinación de los ejes X e Y
  int x = mpu6050.getAngleX();
  int y = mpu6050.getAngleY();

  if (y < ACC_THRESHOLD_LOW)  { return 90;   } //Si el eje Y está inclinado negativamente, se inclinó a la izquierda
  if (x > ACC_THRESHOLD_HIGH) { return 0;  } //Si el eje X se inclinó positivamente, está en posición vertica normal
  if (y > ACC_THRESHOLD_HIGH) { return 270; } //Si el eje Y está inclinado positivamente, se inclinó hacia la derecha
  if (x < ACC_THRESHOLD_LOW)  { return 180; } //Si el eje X se inclinó negativamente, el reloj está dado vuelta
}


int getTopMatrix() { // Si el reloj está a 90 grados MATRIX_A es la de arriba y MATRIX_B la de abajo
  return (getGravity() == 90) ? MATRIX_A : MATRIX_B;
}
int getBottomMatrix() { // En otro caso invierte esa lógica
  return (getGravity() != 90) ? MATRIX_A : MATRIX_B;
}


//Se encarga de reiniciar el reloj de arena limpiando ambas matrices led
void resetTime() {
  for (byte i=0; i<2; i++) {
    lc.clearDisplay(i);//Limpia ambas matrices
  }
  fill(getTopMatrix(), 60); //Llama a la función fill, para llenar la matriz que está actualmente arriba, usa getTopMatrix para saber cuál está arriba según la orientación
  d.Delay(getDelayDrop() * 1000);
}



//Se encarga de hacer que las particulas "caigan"
bool updateMatrix() {
  int n = 8; //Tamaño de la matriz 8x8
  bool somethingMoved = false; //Guarda si alguna partícula se ha movido durante la ejecución
  byte x,y;
  bool direction;
  for (byte slice = 0; slice < 2*n-1; ++slice) { //recorre la matriz en orden diagonal, empezando por arriba y bajando
    direction = (random(2) == 1); // si podemos movernos tanto a la izquierda como a la derecha, randomiza el movimiento

    byte z = slice<n ? 0 : slice-n + 1; //Calcula el límite inferior de la diagonal según el slice actual

    for (byte j = z; j <= slice-z; ++j) {//Recorre cada punto de la diagonal actual

      //Calcula X y Y para cada posición, cambiando el orden de recorrido según la posición
      y = direction ? (7-j) : (7-(slice-j)); 
      x = direction ? (slice-j) : j;

      //Por cada coordenada, intenta mover una partícula, si una se mueve somethingmoved retorna TRUE
      if (moveParticle(MATRIX_B, x, y)) {
        somethingMoved = true;
      };
      if (moveParticle(MATRIX_A, x, y)) {
        somethingMoved = true;
      }
    }
  }
  return somethingMoved;
}



//Intenta hacer caer una partícula desde la matriz superior a la inferior
boolean dropParticle() {

  if (d.Timeout()) { //Verifica si ha pasado el tiempo necesario para dejar caer la siguiente partícula

    d.Delay(getDelayDrop() * 1000); //Programa el siguiente delay para que la siguiente partícula caiga, calculando el tiempo total en segundos según el timepo impuesto por el usuario
    if (gravity == 0 || gravity == 180) { //Solo permite que caiga una partícula si el reloj está ubicado verticalmente

      //solo se transfiere una partícula si hay una en el centro de la matriz superior y el lugar correspondiente en la matriz inferior está libre, o viceversa
      if ((lc.getRawXY(MATRIX_A, 0, 0) && !lc.getRawXY(MATRIX_B, 7, 7)) ||
          (!lc.getRawXY(MATRIX_A, 0, 0) && lc.getRawXY(MATRIX_B, 7, 7))
      ) {
        //Estas líneas hacen el "traspado" de la partícula entre matriz visualmente
        lc.invertRawXY(MATRIX_A, 0, 0);
        lc.invertRawXY(MATRIX_B, 7, 7);
        tone(PIN_BUZZER, 440, 10); //Reproduce un pequeño beep para indicar que la partícula ya cayó
        return true; //Indica que la particula cayó correctamente
      }
    }
  }
  return false;
}


//Hace sonar el buzzer 5 veces para indicar que todas las partículas ya calleron
void alarm() {
  for (int i=0; i<5; i++) {
    tone(PIN_BUZZER, 440, 200);
    delay(1000);
  }
}


//Supervisa si el dispositivo fue movido o agitado
void resetCheck() {
  int z = analogRead(A3); //Lee el valor analógico de un sensor conectado a A3

  if (z > ACC_THRESHOLD_HIGH || z < ACC_THRESHOLD_LOW) { //Si la lectura está fuera de los valores normales, se interpreta como una inclinación extrema o movimiento
    resetCounter++;
    Serial.println(resetCounter);
  } else {
    resetCounter = 0;
  }
  if (resetCounter > 20) { //Si el valor estuvo fuera de rango por más de 20 segundos seguidos, entonces, se considera una intención de resetear el reloj
    Serial.println("RESET!");
    resetTime();
    resetCounter = 0;
  }
}

//Muestra una "X" grande centrada en la matriz, y luego superpone una letra M o H si corresponde

void displayLetter(char letter, int matrix) {
  // Serial.print("Letter: ");
  // Serial.println(letter);
  lc.clearDisplay(matrix); //Limpia completamente la matriz

  //Esto dibuja una especie de X decorativa, que aparece tanto en el modo minutos como horas
  lc.setXY(matrix, 1,4, true);
  lc.setXY(matrix, 2,3, true);
  lc.setXY(matrix, 3,2, true);
  lc.setXY(matrix, 4,1, true);

  lc.setXY(matrix, 3,6, true);
  lc.setXY(matrix, 4,5, true);
  lc.setXY(matrix, 5,4, true);
  lc.setXY(matrix, 6,3, true);

  //Añade trazos que forman una M sobre la X de fondo.
  if (letter == 'M') {
    lc.setXY(matrix, 4,2, true);
    lc.setXY(matrix, 4,3, true);
    lc.setXY(matrix, 5,3, true);
  }

  //Dibuja detalles específicos que identifican la letra H.
  if (letter == 'H') {
    lc.setXY(matrix, 3,3, true);
    lc.setXY(matrix, 4,4, true);
  }
}


//Llaman a fill() para llenar la parte superior con una cantidad de granitos correspondiente a los minutos u horas configurados.
void renderSetMinutes() {
  fill(getTopMatrix(), delayMinutes);
  displayLetter('M', getBottomMatrix());
}
void renderSetHours() {
  fill(getTopMatrix(), delayHours);
  displayLetter('H', getBottomMatrix());
}



//Estas funciones controlan el ajuste del temporizador

void knobClockwise() { //Se llama cuando girás el encoder hacia la derecha.
  Serial.println("Clockwise");
  if (mode == MODE_SETHOURS) { //Si estamos en modo MODE_SETHOURS aumenta delayHours hasta 64
    delayHours = constrain(delayHours+1, 0, 64);
    renderSetHours(); //Actualiza la visualización 
  } else if(mode == MODE_SETMINUTES) { //La misma lógica pero con minutos
    delayMinutes = constrain(delayMinutes+1, 0, 64);
    renderSetMinutes();
  }
  Serial.print("Delay: ");
  Serial.println(getDelayDrop()); //Imprime el nuevo delay en segundos calculado con getDelayDrop
}


void knobCounterClockwise() { //Se llama cuando girás el encoder hacia la izquierda.
  Serial.println("Counterclockwise");
  if (mode == MODE_SETHOURS) {
    delayHours = constrain(delayHours-1, 0, 64); //Disminuye delayHours hasta 0
    renderSetHours(); //Actualiza la visualización
  } else if (mode == MODE_SETMINUTES) {
    delayMinutes = constrain(delayMinutes-1, 0, 64);
    renderSetMinutes();
  }
  Serial.print("Delay: ");
  Serial.println(getDelayDrop());
}



volatile int lastEncoded = 0; //último encoded
volatile long encoderValue = 0; //valor del encoder
long lastencoderValue = 0; //último valor del encoder
long lastValue = 0;

//encargada de leer el estado del encoder rotatorio y traducirlo en una acción
void updateEncoder() {
  //Lee el estado de los pines del enconder
  int MSB = digitalRead(PIN_ENC_1); //MSB = most significant bit
  int LSB = digitalRead(PIN_ENC_2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //Convina MSB y LSN en un valor (0 - 3) que representa el estado actual del encoder

  int sum  = (lastEncoded << 2) | encoded; //Detecta la dirección del giro
  
  //Estos patrones corresponden a secuencias de cambio que indican si el giro fue hacia un lado u otro
  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue--; 
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue++;

  //Solo ejecuta las funciones knobClockwise() o knobCounterClockwise() cada 4 pasos, para evitar múltiples lecturas por un solo click.
  if ((encoderValue % 4) == 0) {
    int value = encoderValue / 4;
    if (value > lastValue) knobClockwise();
    if (value < lastValue) knobCounterClockwise();
    lastValue = value;
  }
  lastEncoded = encoded; //Guarda el estado para compararlo la próxima vez
}



//callback del botón del encoder

volatile unsigned long lastButtonPushMillis;
void buttonPush() {

  if((long)(millis() - lastButtonPushMillis) >= DEBOUNCE_THRESHOLD) { //Esto evita que se activen múltiples pulsaciones por el mismo toque
    mode = (mode+1) % 3; //Cambia el modo ciclicamente (HOUR_GLASS, SETMINUTES, SETHOURS)
    Serial.print("Switched mode to: ");
    Serial.println(mode);
    lastButtonPushMillis = millis(); // Registra el tiempo de pulsación

    //Maneja la interfaz según el modo
    if (mode == MODE_SETMINUTES) {
      lc.backup(); // Guarda el estado actual (Solo necesitamos hacerlo cuando pasa de HOURGLASS a SETMINUTES)
      renderSetMinutes(); // muestra letra 'M' y cantidad de minutos
    }
    if (mode == MODE_SETHOURS) {
      renderSetHours(); //Muestra la letra H y cantidad de horas
    }
    if (mode == MODE_HOURGLASS) {
      lc.clearDisplay(0);
      lc.clearDisplay(1);
      lc.restore(); //Recupera el estado antes del ajuste
      resetTime(); //Recarga partículas
    }
  }
}



/**
 * Setup
 */
void setup() {
mpu6050.calcGyroOffsets(true); //Calibra los offsets del giroscopio
 // Serial.begin(9600);
mpu6050.begin(); //Comienza la comunicación con el MPU6050
  
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB
  // }

  // Configura el encoder rotatorio y su botón
  pinMode(PIN_ENC_1, INPUT);
  pinMode(PIN_ENC_2, INPUT);
  pinMode(PIN_ENC_BUTTON, INPUT);
  digitalWrite(PIN_ENC_1, HIGH); //Activa resistencias Pull Up
  digitalWrite(PIN_ENC_2, HIGH); //Activa resistencias Pull Up
  digitalWrite(PIN_ENC_BUTTON, HIGH); //turn pullup resistor on
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_1), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_2), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_BUTTON), buttonPush, RISING);

  // Serial.println(digitalPinToInterrupt(PIN_ENC_1));
  // Serial.println(digitalPinToInterrupt(PIN_ENC_2));
  // Serial.println(digitalPinToInterrupt(PIN_ENC_BUTTON));

  randomSeed(analogRead(A0)); //Inicializa las semillas de aleatoridad

  // Activa ambos displays MATRIX_A y MATRIX_B
  for (byte i=0; i<2; i++) { 
    lc.shutdown(i,false);
    lc.setIntensity(i,0); //Configura la intencidad al mínimo (0)
  }

  resetTime(); //Carga el reloj de arena (Llena la parte superior de partículas y espera el primer tick)
}



/**
 * Main loop
 */
void loop() {

  //Actualiza datos del acelerómetro
  mpu6050.update();

  //Captura la inclinación del dispositivo
  //Imprime los ángulos para el debug ( Por si estamos calibrando )
  Serial.println("angleX : ");
  Serial.println(mpu6050.getAngleX());
  Serial.println("\tangleY : ");
  Serial.println(mpu6050.getAngleY());
 
  delay(DELAY_FRAME); //Hace una pequeña pausa para evitar conflictos con el monitor en serie
  
  //Calcula la gravedad y rota la matriz
  gravity = getGravity(); //Detecta si el reloj está abajo o no
  lc.setRotation((ROTATION_OFFSET + gravity) % 360); //Ajusta la orientación visual de la matriz

  //Modo de configuración (horas/minutos)

  //Si estamos en modo de configuración, no se ejecuta el "flujo de arena", sino que muestra la letra M o H con el valor correspondiente.
  if (mode == MODE_SETMINUTES) {
    renderSetMinutes(); return;
  } else if (mode == MODE_SETHOURS) {
    renderSetHours(); return;
  }

  // resetCheck();
  //Simula el movimiento de las partículas dentro de la matriz.
  bool moved = updateMatrix(); 
  bool dropped = dropParticle(); // pasa una partícula de una matriz a la otra (si toca).

  // Activa una alarma cuando termina el tiempo
  if (!moved && !dropped && !alarmWentOff && (countParticles(getTopMatrix()) == 0)) {
    alarmWentOff = true;
    alarm();
  }
  //  Resetea la alarma si se reanuda el movimiento
  if (dropped) {
    alarmWentOff = false;
  }
}
