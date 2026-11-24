/*
  ============================================================
  MONITOR DE CLORO
  ============================================================
  Conexiones:
    LCD 1602 I2C (0x27) + RTC DS3231 (0x68) ... A4 (SDA) / A5 (SCL)
    Modulo SD (SPI) ........................... CS D10, MOSI D11, MISO D12, SCK D13
    Joystick .................................. VERT A1, HORZ A0 (sin uso), SEL D2
    LED RGB catodo comun ...................... R D3, G D5, B D6
    Buzzer .................................... D4
    Reservado para LoRa (futuro) .............. D7, D8 (CS), D9 + bus SPI

  Funcionamiento main
    - Modo producción, donde las lecturas son cada 60seg y modo prueba, para
      simulaciones en Wokwi donde las lecturas son cada 2seg.
      El modo prueba tiene un rango de 70% lecturas "normales", 15% > 3.0ppm
      y 15% < 5.0ppm, con tal de probar alertas y confirmaciones.
    - Cada lectura se guarda en la SD con registros de largo fijo (25 bytes),
      asi se pueden leer las ultimas 720 lecturas sin gastar RAM, las lecturas
      deberian tener fecha y hora gracias al bendito modulo RTC que venia en
      la caja, asi que hay que instalarlo SI O SI, o las fechas van a quedar
      ambiguas en el log.
    - Para producción, alerta si ppm < 3.0 o ppm > 5.0.
    - Todos los valores de ppm se manejan como entero x10 (4.2 ppm = 42).
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ---------------- Configuracion ----------------
#define DEBUG_SERIAL   1      // 1 = imprime eventos por el monitor serial
#define JOY_INVERTIR_Y 0      // pon 1 si arriba/abajo te queda al reves

const int  UMBRAL_BAJO_X10 = 30;          // 3.0 ppm
const int  UMBRAL_ALTO_X10 = 50;          // 5.0 ppm
const unsigned long INTERVALO_PROD_MS   = 60000UL;
const unsigned long INTERVALO_PRUEBA_MS = 2000UL;
const unsigned long VERDE_MS = 5000UL;    // duracion del verde al confirmar
const unsigned long MSG_MS   = 2500UL;    // duracion de mensajes en pantalla
const uint16_t N_DUMP = 720;              // 12 h a 1 lectura por minuto

// ---------------- Pines ----------------
const byte PIN_SD_CS  = 10;
const byte PIN_JOY_Y  = A1;               // HORZ (A0) queda libre
const byte PIN_JOY_SW = 2;
const byte PIN_RGB_R  = 3;
const byte PIN_RGB_G  = 5;
const byte PIN_RGB_B  = 6;
const byte PIN_BUZZER = 4;

// ---------------- Formato del registro (25 bytes) ----------------
// "2026-11-24 15:30:05,04.2\n"
const byte REC = 25;

// ---------------- Tipos de alerta y estados de pantalla ----------------
const byte TIPO_NINGUNA = 0, TIPO_BAJO = 1, TIPO_ALTO = 2;
const byte E_MENU = 0, E_VIVO = 1, E_PROMEDIO = 2, E_RANGO = 3, E_MSG = 4;
const byte N_ITEMS = 6;

// ---------------- Objetos y variables globales ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;                           // si tienes DS1307: RTC_DS1307 rtc;

bool rtcOk = false, sdOk = false;
bool modoPrueba = false;                  // arranca en Produccion
bool daltonismo = false;                  // paleta accesible (se guarda en EEPROM)

int  ultimaX10 = 0;
DateTime ultimaHora;
bool lecturaValida = false, lecturaNueva = false;
unsigned long ultimaMuestra = 0;

byte tipoAlerta = TIPO_NINGUNA;
bool confirmada = false;
unsigned long verdeHasta = 0;
bool overlay = false;                     // pantalla de alerta visible
bool buzzerOn = false;

byte estado = E_MENU;
byte itemSel = 0;
byte pagProm = 0;
unsigned long msgHasta = 0;

// Resultados del analisis de las ultimas 720 lecturas
byte  resCodigo = 1;                      // 0 ok, 1 sin datos, 2 error SD
int   resN, resProm, resMin, resMax, resBajo, resAlto;

// ============================================================
//  Utilidades
// ============================================================
DateTime ahora() {
  return rtcOk ? rtc.now() : DateTime(2000, 1, 1, 0, 0, 0);
}

const char* archivoLog() {
  return modoPrueba ? "PRUEBA.CSV" : "LOG.CSV";   // prueba no ensucia el log real
}

unsigned long intervaloMs() {
  return modoPrueba ? INTERVALO_PRUEBA_MS : INTERVALO_PROD_MS;
}

bool alertaPendiente() {
  return tipoAlerta != TIPO_NINGUNA && !confirmada;
}

// ============================================================
//  Sensor (simulado)
// ============================================================
int leerCloro() {
  // TODO: reemplazar por la lectura real del sensor (analogRead + calibracion).
  // Debe devolver ppm x10 (ej. 42 = 4.2 ppm).
  int r = random(100);
  if (r < 70) return random(UMBRAL_BAJO_X10, UMBRAL_ALTO_X10 + 1);  // normal 3.0-5.0
  if (r < 85) return random(10, UMBRAL_BAJO_X10);                   // bajo   1.0-2.9
  return random(UMBRAL_ALTO_X10 + 1, 71);                           // alto   5.1-7.0
}

// ============================================================
//  PUNTO DE ENGANCHE FUTURO: LoRaWAN -> servidor de red -> Telegram
//  Se llama una vez cada vez que aparece una alerta nueva.
// ============================================================
void enviarAlerta(byte tipo, int ppmX10) {
  // (vacia por ahora)
}

// ============================================================
//  Registro en SD
// ============================================================
bool registroValido(const uint8_t* b) {
  return b[4] == '-' && b[19] == ',' && b[22] == '.' && b[24] == '\n' &&
         isDigit(b[20]) && isDigit(b[21]) && isDigit(b[23]);
}

void guardarLectura() {
  if (!sdOk) sdOk = SD.begin(PIN_SD_CS);
  if (!sdOk) return;

  int x = constrain(ultimaX10, 0, 999);
  char buf[REC + 1];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d,%02d.%d\n",
           (int)ultimaHora.year(), (int)ultimaHora.month(), (int)ultimaHora.day(),
           (int)ultimaHora.hour(), (int)ultimaHora.minute(), (int)ultimaHora.second(),
           x / 10, x % 10);

  File f = SD.open(archivoLog(), FILE_WRITE);
  if (!f) { sdOk = false; return; }

  // Si un corte de energia dejo un registro a medias, lo completamos
  // con puntos para que los siguientes queden alineados.
  uint32_t resto = f.size() % REC;
  if (resto) {
    for (uint32_t i = resto; i < (uint32_t)(REC - 1); i++) f.write((uint8_t)'.');
    f.write((uint8_t)'\n');
  }
  f.write((const uint8_t*)buf, REC);
  f.close();
}

// Lee las ultimas N_DUMP lecturas y calcula promedio, min, max y conteos.
byte analizar() {
  resN = 0; resProm = 0; resMin = 9999; resMax = -1; resBajo = 0; resAlto = 0;
  if (!sdOk) return 2;

  File f = SD.open(archivoLog(), FILE_READ);
  if (!f) return 1;

  uint32_t total = f.size() / REC;
  uint32_t n = total < N_DUMP ? total : N_DUMP;
  f.seek((total - n) * REC);

  uint8_t b[REC];
  long suma = 0;
  for (uint32_t i = 0; i < n; i++) {
    if (f.read(b, REC) != REC) break;
    if (!registroValido(b)) continue;
    int x = (b[20] - '0') * 100 + (b[21] - '0') * 10 + (b[23] - '0');
    suma += x;
    resN++;
    if (x < resMin) resMin = x;
    if (x > resMax) resMax = x;
    if (x < UMBRAL_BAJO_X10) resBajo++;
    else if (x > UMBRAL_ALTO_X10) resAlto++;
  }
  f.close();

  if (resN == 0) return 1;
  resProm = (int)((suma + resN / 2) / resN);
  return 0;
}

// Copia las ultimas 720 lecturas a un archivo nuevo MMDDHHMM.CSV
// Devuelve 0 ok, 1 sin datos, 2 error de SD.
byte crearDump(char* nombre) {
  if (!sdOk) return 2;

  DateTime t = ahora();
  snprintf(nombre, 13, "%02d%02d%02d%02d.CSV",
           (int)t.month(), (int)t.day(), (int)t.hour(), (int)t.minute());

  File src = SD.open(archivoLog(), FILE_READ);
  if (!src) return 1;
  uint32_t total = src.size() / REC;
  if (total == 0) { src.close(); return 1; }
  uint32_t n = total < N_DUMP ? total : N_DUMP;

  if (SD.exists(nombre)) SD.remove(nombre);
  File dst = SD.open(nombre, FILE_WRITE);
  if (!dst) { src.close(); return 2; }

  dst.print(F("fecha_hora,ppm\n"));
  src.seek((total - n) * REC);

  uint8_t b[REC];
  uint16_t copiados = 0;
  for (uint32_t i = 0; i < n; i++) {
    if (src.read(b, REC) != REC) break;
    if (!registroValido(b)) continue;
    dst.write(b, REC);
    copiados++;
  }
  dst.close();
  src.close();
  return copiados ? 0 : 1;
}

// ============================================================
//  Alertas y lecturas
// ============================================================
void evaluarAlerta() {
  byte tipo = TIPO_NINGUNA;
  if (ultimaX10 < UMBRAL_BAJO_X10) tipo = TIPO_BAJO;
  else if (ultimaX10 > UMBRAL_ALTO_X10) tipo = TIPO_ALTO;

  if (tipo != tipoAlerta) {            // alerta nueva, cambio de tipo o fin de alerta
    tipoAlerta = tipo;
    confirmada = false;
    if (tipo != TIPO_NINGUNA) {
#if DEBUG_SERIAL
      Serial.println(tipo == TIPO_BAJO ? F("[ALERTA] cloro BAJO") : F("[ALERTA] cloro ALTO"));
#endif
      enviarAlerta(tipo, ultimaX10);
    }
  }
}

void tomarLectura() {
  ultimaMuestra = millis();
  ultimaX10 = leerCloro();
  ultimaHora = ahora();
  lecturaValida = true;
  lecturaNueva = true;
#if DEBUG_SERIAL
  Serial.print(F("[LECTURA] "));
  Serial.print(ultimaX10 / 10); Serial.print('.'); Serial.print(ultimaX10 % 10);
  Serial.println(F(" ppm"));
#endif
  guardarLectura();
  evaluarAlerta();
}

// ============================================================
//  Salidas: LED RGB (catodo comun) y buzzer
// ============================================================
void setRGB(bool r, bool g, bool b) {
  digitalWrite(PIN_RGB_R, r);
  digitalWrite(PIN_RGB_G, g);
  digitalWrite(PIN_RGB_B, b);
}

/*
  Paleta normal ........ azul = normal, rojo = fuera de rango, verde = confirmada
  Paleta daltonismo .... azul = normal, amarillo = fuera de rango, blanco = confirmada
    (azul/amarillo es el eje que se distingue bien en deuteranopia y protanopia;
     ademas el amarillo PARPADEA mientras la alerta no esta confirmada)
*/
void actualizarSalidas() {
  unsigned long t = millis();
  bool pendiente = alertaPendiente();
  bool verde = (long)(verdeHasta - t) > 0;

  if (verde) {
    setRGB(daltonismo, true, daltonismo);
  } else if (tipoAlerta != TIPO_NINGUNA) {
    if (daltonismo) {
      bool on = !pendiente || ((t / 250) % 2 == 0);
      setRGB(on, on, false);
    } else {
      setRGB(true, false, false);
    }
  } else {
    setRGB(false, false, true);
  }

  // Buzzer: suena (intermitente) solo mientras la alerta NO esta confirmada
  if (pendiente) {
    bool on = (t % 500UL) < 250UL;
    if (on != buzzerOn) {
      buzzerOn = on;
      if (on) tone(PIN_BUZZER, 2000); else noTone(PIN_BUZZER);
    }
  } else if (buzzerOn) {
    noTone(PIN_BUZZER);
    buzzerOn = false;
  }
}

// ============================================================
//  Entradas: joystick
// ============================================================
bool botonPresionado() {
  static bool previo = false;
  static unsigned long t = 0;
  bool ahoraPresionado = digitalRead(PIN_JOY_SW) == LOW;
  bool evento = false;
  if (ahoraPresionado && !previo && millis() - t > 60) {
    evento = true;
    t = millis();
  }
  previo = ahoraPresionado;
  return evento;
}

int8_t joyDir() {
  static bool activo = false;
  static unsigned long ultimo = 0;
  int v = analogRead(PIN_JOY_Y);
  int8_t d = 0;
  if (v < 300) d = -1; else if (v > 700) d = 1;
  if (JOY_INVERTIR_Y) d = -d;
  if (d == 0) { activo = false; return 0; }
  if (!activo || millis() - ultimo > 300) {
    activo = true;
    ultimo = millis();
    return d;
  }
  return 0;
}

// ============================================================
//  Pantalla LCD
// ============================================================
void print2(int v) {
  if (v < 10) lcd.print('0');
  lcd.print(v);
}

void imprimirPpm(int x10) {
  lcd.print(x10 / 10);
  lcd.print('.');
  lcd.print(x10 % 10);
}

void imprimirItem(byte i) {
  switch (i) {
    case 0: lcd.print(F("Lectura en vivo")); break;
    case 1: lcd.print(F("Crear dump 12h")); break;
    case 2: lcd.print(F("Promedio 12h")); break;
    case 3: lcd.print(F("Rango de ppm")); break;
    case 4: lcd.print(modoPrueba ? F("Modo: PRUEBA 2s") : F("Modo: PROD. 60s")); break;
    case 5: lcd.print(daltonismo ? F("Daltonismo: ON") : F("Daltonismo: OFF")); break;
  }
}

void dibujarMenu() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print('>'); imprimirItem(itemSel);
  lcd.setCursor(0, 1); lcd.print(' '); imprimirItem((itemSel + 1) % N_ITEMS);
}

void dibujarVivo() {
  lcd.clear();
  if (!lecturaValida) { lcd.print(F("Sin lectura")); return; }
  lcd.setCursor(0, 0);
  lcd.print(F("Cloro: ")); imprimirPpm(ultimaX10); lcd.print(F(" ppm"));
  lcd.setCursor(15, 0); lcd.print(modoPrueba ? 'T' : 'P');
  lcd.setCursor(0, 1);
  print2(ultimaHora.hour()); lcd.print(':');
  print2(ultimaHora.minute()); lcd.print(':');
  print2(ultimaHora.second()); lcd.print(' ');
  lcd.print(tipoAlerta == TIPO_BAJO ? F("BAJO")
          : (tipoAlerta == TIPO_ALTO ? F("ALTO") : F("NORMAL")));
  if (!sdOk) { lcd.setCursor(15, 1); lcd.print('!'); }   // '!' = problema con la SD
}

void dibujarAlerta() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(tipoAlerta == TIPO_BAJO ? F("!ALERTA: BAJO!") : F("!ALERTA: ALTO!"));
  lcd.setCursor(0, 1);
  imprimirPpm(ultimaX10); lcd.print(F(" ppm Pulsar"));
}

void dibujarPromedio() {
  lcd.clear();
  if (resCodigo == 2) { lcd.print(F("Error SD")); return; }
  if (resCodigo == 1) { lcd.print(F("Sin datos aun")); return; }
  if (pagProm == 0) {
    lcd.print(F("Prom: ")); imprimirPpm(resProm); lcd.print(F(" ppm"));
    lcd.setCursor(0, 1);
    lcd.print(F("N:")); lcd.print(resN);
    lcd.print(F(" Fuera:")); lcd.print(resBajo + resAlto);
    lcd.setCursor(15, 1); lcd.print('>');
  } else {
    lcd.print(F("Min:")); imprimirPpm(resMin);
    lcd.print(F(" Max:")); imprimirPpm(resMax);
    lcd.setCursor(0, 1);
    lcd.print(F("Baj:")); lcd.print(resBajo);
    lcd.print(F(" Alt:")); lcd.print(resAlto);
    lcd.setCursor(15, 1); lcd.print('<');
  }
}

void dibujarRango() {
  lcd.clear();
  lcd.print(F("Rango normal:"));
  lcd.setCursor(0, 1);
  imprimirPpm(UMBRAL_BAJO_X10); lcd.print(F(" - "));
  imprimirPpm(UMBRAL_ALTO_X10); lcd.print(F(" ppm"));
}

void iniciarMensaje() {
  estado = E_MSG;
  msgHasta = millis() + MSG_MS;
}

void mensaje(const __FlashStringHelper* l1, const __FlashStringHelper* l2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
  iniciarMensaje();
}

void mensajeRam(const __FlashStringHelper* l1, const char* l2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
  iniciarMensaje();
}

void volverMenu() {
  estado = E_MENU;
  dibujarMenu();
}

void redibujar() {
  if (estado == E_MSG) estado = E_MENU;   // un mensaje tapado por una alerta se descarta
  switch (estado) {
    case E_MENU:     dibujarMenu();     break;
    case E_VIVO:     dibujarVivo();     break;
    case E_PROMEDIO: dibujarPromedio(); break;
    case E_RANGO:    dibujarRango();    break;
  }
}

// ============================================================
//  Menu y logica de interfaz
// ============================================================
void ejecutarItem() {
  switch (itemSel) {
    case 0:
      estado = E_VIVO;
      dibujarVivo();
      break;
    case 1: {
      char nombre[13];
      byte r = crearDump(nombre);
      if (r == 0)      mensajeRam(F("Dump OK"), nombre);
      else if (r == 1) mensaje(F("Sin datos"), F("para copiar"));
      else             mensaje(F("Error SD"), F("Revise tarjeta"));
      break;
    }
    case 2:
      resCodigo = analizar();
      pagProm = 0;
      estado = E_PROMEDIO;
      dibujarPromedio();
      break;
    case 3:
      estado = E_RANGO;
      dibujarRango();
      break;
    case 4:
      modoPrueba = !modoPrueba;
      tomarLectura();                      // lectura inmediata con el nuevo intervalo
      dibujarMenu();
      break;
    case 5:
      daltonismo = !daltonismo;
      EEPROM.update(0, daltonismo ? 1 : 0);
      dibujarMenu();
      break;
  }
}

void confirmarAlerta() {
  confirmada = true;
  verdeHasta = millis() + VERDE_MS;
  overlay = false;
  noTone(PIN_BUZZER);
  buzzerOn = false;
#if DEBUG_SERIAL
  Serial.println(F("[ALERTA] confirmada"));
#endif
  mensaje(F("Alerta"), F("confirmada"));
}

void manejarUI(bool btn, int8_t dir) {
  switch (estado) {
    case E_MENU:
      if (dir) {
        itemSel = (itemSel + (dir > 0 ? 1 : N_ITEMS - 1)) % N_ITEMS;
        dibujarMenu();
      } else if (btn) {
        ejecutarItem();
      }
      break;
    case E_VIVO:
      if (btn) volverMenu();
      else if (lecturaNueva) dibujarVivo();
      break;
    case E_PROMEDIO:
      if (btn) volverMenu();
      else if (dir) { pagProm = 1 - pagProm; dibujarPromedio(); }
      break;
    case E_RANGO:
      if (btn) volverMenu();
      break;
    case E_MSG:
      if (btn || (long)(millis() - msgHasta) >= 0) volverMenu();
      break;
  }
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_JOY_SW, INPUT_PULLUP);
  setRGB(false, false, true);

#if DEBUG_SERIAL
  Serial.begin(9600);
#endif

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print(F("Monitor de cloro"));
  lcd.setCursor(0, 1); lcd.print(F("Iniciando..."));

  randomSeed(analogRead(A3) + micros());

  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // hora de compilacion
  }
  sdOk = SD.begin(PIN_SD_CS);
  daltonismo = (EEPROM.read(0) == 1);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("RTC:")); lcd.print(rtcOk ? F("OK") : F("ERROR"));
  lcd.setCursor(0, 1); lcd.print(F("SD: ")); lcd.print(sdOk ? F("OK") : F("ERROR"));
  delay(1500);

  tomarLectura();
  volverMenu();
}

void loop() {
  if (millis() - ultimaMuestra >= intervaloMs()) tomarLectura();

  bool btn = botonPresionado();
  int8_t dir = joyDir();

  if (alertaPendiente()) {
    if (btn) {
      confirmarAlerta();                       // el boton solo confirma mientras hay alerta
    } else if (!overlay || lecturaNueva) {
      overlay = true;
      dibujarAlerta();
    }
  } else {
    if (overlay) { overlay = false; redibujar(); }
    manejarUI(btn, dir);
  }

  lecturaNueva = false;
  actualizarSalidas();
}
