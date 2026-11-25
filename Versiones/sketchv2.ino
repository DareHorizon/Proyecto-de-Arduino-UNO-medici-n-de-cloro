/*
  ============================================================
  MONITOR DE CLORO  (v2)
  ============================================================
  Conexiones:
    LCD 1602 I2C (0x27) + RTC DS3231 (0x68) ... A4 (SDA) / A5 (SCL)
    Modulo SD (SPI) ........................... CS D10, MOSI D11, MISO D12, SCK D13
    Joystick .................................. VERT A1, HORZ A0, SEL D2
    LED RGB catodo comun ...................... R D3, G D5, B D6
    Buzzer .................................... D4
    Reservado para LoRa (futuro) .............. D7, D8 (CS), D9 + bus SPI

  Funcionamiento main
    - Modo producción, donde las lecturas son cada 60seg y modo prueba, para
      simulaciones en Wokwi donde las lecturas son cada 2seg.
      El modo prueba tiene un rango de 70% lecturas "normales", 15% < 3.0ppm
      y 15% > 5.0ppm, con tal de probar alertas y confirmaciones.
      ([FIX] v2: el comentario decia "15% > 3.0" y "15% < 5.0", estaba al reves)
    - Cada lectura se guarda en la SD con registros de largo fijo (25 bytes),
      asi se pueden leer las ultimas 720 lecturas sin gastar RAM, las lecturas
      deberian tener fecha y hora gracias al bendito modulo RTC que venia en
      la caja, asi que hay que instalarlo SI O SI, o las fechas van a quedar
      ambiguas en el log.
    - Para producción, alerta si ppm < 3.0 o ppm > 5.0.
    - Todos los valores de ppm se manejan como entero x10 (4.2 ppm = 42).

  ============================================================
  CAMBIOS v2   (busca las etiquetas en el codigo)
    [FIX] = correccion de un error   [QOL] = mejora de uso   [OPT] = optimizacion
  ------------------------------------------------------------
  [FIX] Boton: el rebote al SOLTAR podia generar un segundo clic (Wokwi simula
        rebote en SEL por defecto). Ahora hay antirrebote real.
  [FIX] SD: el reintento nunca podia reconectar (SD.begin() falla siempre la 2a
        vez si no se llama SD.end() antes) y, sin tarjeta, bloqueaba la interfaz
        hasta ~2 s en CADA lectura. Ahora reintenta cada 30 s en segundo plano,
        o al instante cuando pides un dump o el promedio.
  [FIX] Sin RTC todas las lecturas quedaban con 2000-01-01 00:00:00. Ahora usa
        2000-01-01 + tiempo desde el encendido, asi el log al menos queda ordenado.
  [QOL] Joystick horizontal (A0): izquierda = volver, derecha = entrar / pagina.
  [QOL] Mantener arriba/abajo: primer paso inmediato, despues repite mas rapido.
  [QOL] Al confirmar una alerta vuelves a la pantalla donde estabas (antes
        siempre al menu; molestaba si estabas mirando la lectura en vivo).
  [QOL] 60 s sin tocar nada -> vuelve sola a "Lectura en vivo".
  [QOL] 5 min sin tocar nada -> apaga la luz del LCD. El primer toque solo la
        enciende (no ejecuta nada). Una alerta siempre la enciende.
  [QOL] Recordatorio: si una alerta confirmada sigue activa 30 min, vuelve a sonar.
  [QOL] "Promedio": 3a pagina con el periodo que cubren los datos (inicio/fin),
        numero de pagina y flechas arriba/abajo como pista de navegacion.
  [QOL] El menu dice "24m" en vez de "12h" en modo prueba (720 lecturas x 2 s).
  [QOL] Nuevo item "Sonido" para silenciar el buzzer. NO se guarda y vuelve a ON
        al pasar a Produccion, para que nunca quede mudo en uso real por olvido.
  [QOL] "Lectura en vivo" marca '?' si falta el RTC (antes solo '!' para la SD).
  [QOL] Comandos por monitor serial (util en Wokwi, donde no puedes sacar la SD):
          d = volcar las ultimas 720 lecturas    s = estado y estadisticas
          t AAAA-MM-DD HH:MM:SS = ajustar reloj  ? = ayuda
  [QOL] Un dump sin lecturas validas ya no deja un archivo vacio en la SD.
  [OPT] La lectura de las ultimas 720 lecturas estaba duplicada (analizar y
        crearDump). Ahora comparten abrirUltimas() / copiarUltimas(); esta ultima
        tambien sirve para el volcado por serial.
  [OPT] imprimirPpm() sirve para LCD y Serial (antes el serial lo armaba a mano).
  [OPT] snprintf reemplazado por ponerNum(): ~1.5 KB menos de flash.
  [OPT] La alerta se evalua ANTES de escribir en la SD: reacciona aunque la SD tarde.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>

// ---------------- Configuracion ----------------
#define DEBUG_SERIAL    1     // 1 = imprime eventos por el monitor serial
#define COMANDOS_SERIAL 1     // [QOL] 1 = acepta comandos por el monitor serial (d, s, t, ?)
#define JOY_INVERTIR_Y  0     // pon 1 si arriba/abajo te queda al reves
#define JOY_USAR_X      1     // [QOL] 1 = usa HORZ (A0). Pon 0 si A0 no esta conectado (leeria ruido)
#define JOY_INVERTIR_X  0     // [QOL] pon 1 si izquierda/derecha te queda al reves
#define USAR_SERIAL     (DEBUG_SERIAL || COMANDOS_SERIAL)

const long BAUDIOS = 9600;                // con 115200 el comando 'd' tarda ~2 s en vez de ~20 s

const int  UMBRAL_BAJO_X10 = 30;          // 3.0 ppm
const int  UMBRAL_ALTO_X10 = 50;          // 5.0 ppm
const unsigned long INTERVALO_PROD_MS   = 60000UL;
const unsigned long INTERVALO_PRUEBA_MS = 2000UL;
const unsigned long VERDE_MS = 5000UL;    // duracion del verde al confirmar
const unsigned long MSG_MS   = 2500UL;    // duracion de mensajes en pantalla
const uint16_t N_DUMP = 720;              // 12 h a 1 lectura por minuto

// [QOL] Tiempos de interfaz. Pon 0 en cualquiera para desactivar esa funcion.
const unsigned long REPOSO_MS      = 60000UL;    // inactividad -> "Lectura en vivo"
const unsigned long LUZ_MS         = 300000UL;   // inactividad -> apaga la luz del LCD (5 min)
const unsigned long RECORDAR_MS    = 1800000UL;  // alerta confirmada que sigue activa -> vuelve a sonar (30 min)
const unsigned long REPETIR_INI_MS = 400UL;      // mantener joystick: espera antes de empezar a repetir
const unsigned long REPETIR_MS     = 150UL;      // mantener joystick: velocidad de repeticion
const unsigned long REBOTE_MS      = 30UL;       // [FIX] antirrebote del boton
const unsigned long REINTENTO_SD_MS = 30000UL;   // [FIX] reintento de la SD en segundo plano

// ---------------- Pines ----------------
const byte PIN_SD_CS  = 10;
const byte PIN_JOY_Y  = A1;
const byte PIN_JOY_X  = A0;               // [QOL] antes quedaba libre
const byte PIN_JOY_SW = 2;
const byte PIN_RGB_R  = 3;                // OJO: D3 comparte Timer2 con tone(); usar solo digitalWrite aqui
const byte PIN_RGB_G  = 5;
const byte PIN_RGB_B  = 6;
const byte PIN_BUZZER = 4;

// ---------------- Formato del registro (25 bytes) ----------------
// "2026-11-24 15:30:05,04.2\n"
const byte REC = 25;

// ---------------- Tipos de alerta y estados de pantalla ----------------
const byte TIPO_NINGUNA = 0, TIPO_BAJO = 1, TIPO_ALTO = 2;
const byte E_MENU = 0, E_VIVO = 1, E_PROMEDIO = 2, E_RANGO = 3, E_MSG = 4;
const byte N_ITEMS = 7;                   // [QOL] +1 por "Sonido"
const byte N_PAG_PROM = 3;                // [QOL] +1 pagina "periodo" en Promedio
const byte CH_FLECHAS = 1;                // [QOL] caracter propio del LCD (slot 1; el 0 se confunde con '\0')

// ---------------- Objetos y variables globales ----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
RTC_DS3231 rtc;                           // si tienes DS1307: RTC_DS1307 rtc;

bool rtcOk = false, sdOk = false;
bool modoPrueba = false;                  // arranca en Produccion
bool daltonismo = false;                  // paleta accesible (se guarda en EEPROM)
bool sonido = true;                       // [QOL] NO se guarda: siempre arranca en ON

int  ultimaX10 = 0;
DateTime ultimaHora;
bool lecturaValida = false, lecturaNueva = false;
unsigned long ultimaMuestra = 0;

byte tipoAlerta = TIPO_NINGUNA;
bool confirmada = false;
unsigned long confirmadaEn = 0;           // [QOL] para el recordatorio
unsigned long verdeHasta = 0;
bool overlay = false;                     // pantalla de alerta visible
bool buzzerOn = false;

byte estado = E_MENU;
byte estadoTrasMsg = E_MENU;              // [QOL] pantalla a la que vuelve un mensaje
byte itemSel = 0;
byte pagProm = 0;
unsigned long msgHasta = 0;

unsigned long ultimaActividad = 0;        // [QOL] ultimo toque del usuario
bool luzOn = true;                        // [QOL] luz del LCD
unsigned long ultimoIntentoSD = 0;        // [FIX] para espaciar los reintentos

// Resultados del analisis de las ultimas 720 lecturas
byte  resCodigo = 1;                      // 0 ok, 1 sin datos, 2 error SD
int   resN, resProm, resMin, resMax, resBajo, resAlto;
byte  resIni[4], resFin[4];               // [QOL] dia, mes, hora, minuto de la 1a y ultima lectura

// ============================================================
//  Utilidades
// ============================================================
DateTime ahora() {
  if (rtcOk) return rtc.now();
  // [FIX] Sin RTC: 2000-01-01 + tiempo desde el encendido (antes quedaba fijo en 00:00:00)
  return DateTime(2000, 1, 1, 0, 0, 0) + TimeSpan((int32_t)(millis() / 1000UL));
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

// [OPT] Sirve para el LCD y para el Serial (ambos heredan de Print)
void imprimirPpm(Print& p, int x10) {
  p.print(x10 / 10);
  p.print('.');
  p.print(x10 % 10);
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
//  Se llama una vez cada vez que aparece una alerta nueva
//  ([QOL] v2: y tambien en cada recordatorio, ver RECORDAR_MS).
// ============================================================
void enviarAlerta(byte tipo, int ppmX10) {
  // (vacia por ahora)
}

// ============================================================
//  Registro en SD
// ============================================================

// [FIX] Conecta (o reconecta) la SD.
//  - Sin 'forzar' lo intenta como maximo cada REINTENTO_SD_MS, porque SD.begin()
//    sin tarjeta puede bloquear hasta ~2 s (en modo prueba eso pasaba cada lectura).
//  - SD.end() es necesario: sin el, SD.begin() siempre falla en el 2o intento
//    porque la raiz quedo "abierta" del intento anterior.
bool asegurarSD(bool forzar) {
  if (sdOk) return true;
  if (!forzar && millis() - ultimoIntentoSD < REINTENTO_SD_MS) return false;
  ultimoIntentoSD = millis();
  SD.end();
  sdOk = SD.begin(PIN_SD_CS);
  return sdOk;
}

// [OPT] Escribe 'v' con exactamente 'n' digitos (ceros a la izquierda) y devuelve
// el puntero al final. Reemplaza a snprintf, que por si solo ocupaba ~1.5 KB de
// flash (espacio que conviene guardar para la libreria LoRaWAN futura).
char* ponerNum(char* p, unsigned int v, byte n) {
  for (byte i = n; i > 0; i--) { p[i - 1] = '0' + v % 10; v /= 10; }
  return p + n;
}

bool registroValido(const uint8_t* b) {
  return b[4] == '-' && b[19] == ',' && b[22] == '.' && b[24] == '\n' &&
         isDigit(b[20]) && isDigit(b[21]) && isDigit(b[23]);
}

void guardarLectura() {
  if (!asegurarSD(false)) return;           // [FIX] antes: SD.begin() en cada lectura

  // [OPT] Se arma a mano en vez de snprintf (ver ponerNum). Mismo formato:
  // "2026-11-24 15:30:05,04.2\n"
  int x = constrain(ultimaX10, 0, 999);
  char buf[REC];
  char* p = buf;
  p = ponerNum(p, ultimaHora.year(), 4);   *p++ = '-';
  p = ponerNum(p, ultimaHora.month(), 2);  *p++ = '-';
  p = ponerNum(p, ultimaHora.day(), 2);    *p++ = ' ';
  p = ponerNum(p, ultimaHora.hour(), 2);   *p++ = ':';
  p = ponerNum(p, ultimaHora.minute(), 2); *p++ = ':';
  p = ponerNum(p, ultimaHora.second(), 2); *p++ = ',';
  p = ponerNum(p, x / 10, 2);              *p++ = '.';
  p = ponerNum(p, x % 10, 1);              *p   = '\n';

  File f = SD.open(archivoLog(), FILE_WRITE);
  if (!f) { sdOk = false; ultimoIntentoSD = millis(); return; }

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

// [OPT] Abre el log y se ubica en la primera de las ultimas N_DUMP lecturas.
// Devuelve cuantos registros hay que leer (0 = no hay archivo o esta vacio).
// Siempre se puede llamar f.close() despues (cerrar dos veces es seguro).
uint16_t abrirUltimas(File& f) {
  f = SD.open(archivoLog(), FILE_READ);
  if (!f) return 0;
  uint32_t total = f.size() / REC;
  uint16_t n = total < N_DUMP ? (uint16_t)total : N_DUMP;
  if (n) f.seek((total - n) * REC);
  return n;
}

// [OPT] Escribe las ultimas N_DUMP lecturas validas en 'destino' (un archivo o el Serial).
// Devuelve cuantas copio.
uint16_t copiarUltimas(Print& destino) {
  File src;
  uint16_t n = abrirUltimas(src);
  uint8_t b[REC];
  uint16_t copiados = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (src.read(b, REC) != REC) break;
    if (!registroValido(b)) continue;
    destino.write(b, REC);
    copiados++;
  }
  src.close();
  return copiados;
}

// [QOL] Lee 2 digitos ASCII y guarda dia, mes, hora y minuto de un registro
byte dig2(const uint8_t* p) { return (p[0] - '0') * 10 + (p[1] - '0'); }
void guardarFechaCorta(const uint8_t* b, byte* d) {
  d[0] = dig2(b + 8);  d[1] = dig2(b + 5);     // dia, mes
  d[2] = dig2(b + 11); d[3] = dig2(b + 14);    // hora, minuto
}

// Lee las ultimas N_DUMP lecturas y calcula promedio, min, max y conteos.
byte analizar() {
  resN = 0; resProm = 0; resMin = 9999; resMax = -1; resBajo = 0; resAlto = 0;
  if (!asegurarSD(true)) return 2;          // [FIX] reintenta al momento (quizas recien pusiste la tarjeta)

  File f;
  uint16_t n = abrirUltimas(f);             // [OPT] codigo compartido con copiarUltimas()
  uint8_t b[REC];
  long suma = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (f.read(b, REC) != REC) break;
    if (!registroValido(b)) continue;
    int x = (b[20] - '0') * 100 + (b[21] - '0') * 10 + (b[23] - '0');
    if (resN == 0) guardarFechaCorta(b, resIni);   // [QOL] periodo cubierto
    guardarFechaCorta(b, resFin);
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
  if (!asegurarSD(true)) return 2;          // [FIX] reintenta al momento

  DateTime t = ahora();
  char* p = nombre;                         // [OPT] sin snprintf: "MMDDHHMM.CSV"
  p = ponerNum(p, t.month(), 2);
  p = ponerNum(p, t.day(), 2);
  p = ponerNum(p, t.hour(), 2);
  p = ponerNum(p, t.minute(), 2);
  strcpy(p, ".CSV");

  if (SD.exists(nombre)) SD.remove(nombre);
  File dst = SD.open(nombre, FILE_WRITE);
  if (!dst) return 2;

  dst.print(F("fecha_hora,ppm\n"));
  uint16_t copiados = copiarUltimas(dst);   // [OPT] codigo compartido
  dst.close();
  if (!copiados) { SD.remove(nombre); return 1; }   // [QOL] no deja un dump vacio
  return 0;
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
  } else if (RECORDAR_MS && tipo != TIPO_NINGUNA && confirmada &&
             millis() - confirmadaEn >= RECORDAR_MS) {
    // [QOL] Recordatorio: la alerta se confirmo pero el cloro sigue fuera de
    // rango demasiado tiempo -> vuelve a quedar pendiente (pantalla + buzzer).
    confirmada = false;
#if DEBUG_SERIAL
    Serial.println(F("[ALERTA] recordatorio: sigue fuera de rango"));
#endif
    enviarAlerta(tipo, ultimaX10);
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
  imprimirPpm(Serial, ultimaX10);           // [OPT] misma funcion que el LCD
  Serial.println(F(" ppm"));
#endif
  evaluarAlerta();                          // [OPT] antes de la SD: la alerta no espera a la tarjeta
  guardarLectura();
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
  // [QOL] ...y el sonido no esta silenciado desde el menu (el LED sigue avisando)
  if (pendiente && sonido) {
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

// [FIX] Antirrebote real. Antes solo se ignoraban 60 ms despues de presionar,
// asi que el rebote al SOLTAR (si el clic duraba mas de 60 ms) contaba como un
// segundo clic. Ahora el estado tiene que mantenerse REBOTE_MS para aceptarse.
bool botonPresionado() {
  static bool lectura = false, estable = false;
  static unsigned long cambio = 0;
  bool l = digitalRead(PIN_JOY_SW) == LOW;
  if (l != lectura) { lectura = l; cambio = millis(); }
  if (estable != lectura && millis() - cambio >= REBOTE_MS) {
    estable = lectura;
    return estable;                         // evento solo al presionar, no al soltar
  }
  return false;
}

// [QOL] Un eje del joystick. Se generalizo joyDir() para leer tambien HORZ.
struct EjeJoy {
  byte pin;
  bool invertir;
  bool activo;
  unsigned long ultimo;
  unsigned long espera;
};
EjeJoy ejeY = { PIN_JOY_Y, JOY_INVERTIR_Y != 0, false, 0, 0 };
// En Wokwi HORZ da 1023 a la IZQUIERDA y 0 a la DERECHA: se invierte por defecto
// para que izquierda = -1 (volver) y derecha = +1 (entrar).
EjeJoy ejeX = { PIN_JOY_X, JOY_INVERTIR_X == 0, false, 0, 0 };

// Devuelve -1, 0 o +1. Con 'repetir', mantener el joystick repite el paso:
// el primero es inmediato, luego espera REPETIR_INI_MS y despues cada REPETIR_MS
// (antes era siempre cada 300 ms: lento para recorrer el menu).
int8_t leerEje(EjeJoy& e, bool repetir) {
  int v = analogRead(e.pin);
  int8_t d = 0;
  if (v < 300) d = -1; else if (v > 700) d = 1;
  if (e.invertir) d = -d;
  if (d == 0) { e.activo = false; return 0; }
  unsigned long t = millis();
  if (!e.activo) {
    e.activo = true; e.ultimo = t; e.espera = REPETIR_INI_MS;
    return d;
  }
  if (repetir && t - e.ultimo >= e.espera) {
    e.ultimo = t; e.espera = REPETIR_MS;
    return d;
  }
  return 0;                                 // el eje X no repite: mantener "derecha" no ejecuta 2 veces
}

// ============================================================
//  Pantalla LCD
// ============================================================
void print2(int v) {
  if (v < 10) lcd.print('0');
  lcd.print(v);
}

// [QOL] "dd/mm hh:mm" (15 caracteres con el prefijo de 4)
void imprimirFechaCorta(const byte* f) {
  print2(f[0]); lcd.print('/'); print2(f[1]); lcd.print(' ');
  print2(f[2]); lcd.print(':'); print2(f[3]);
}

void imprimirItem(byte i) {
  switch (i) {
    case 0: lcd.print(F("Lectura en vivo")); break;
    // [QOL] En modo prueba las 720 lecturas son 720 x 2 s = 24 min, no 12 h
    case 1: lcd.print(modoPrueba ? F("Crear dump 24m") : F("Crear dump 12h")); break;
    case 2: lcd.print(modoPrueba ? F("Promedio 24m") : F("Promedio 12h")); break;
    case 3: lcd.print(F("Rango de ppm")); break;
    case 4: lcd.print(modoPrueba ? F("Modo: PRUEBA 2s") : F("Modo: PROD. 60s")); break;
    case 5: lcd.print(daltonismo ? F("Daltonismo: ON") : F("Daltonismo: OFF")); break;
    case 6: lcd.print(sonido ? F("Sonido: ON") : F("Sonido: OFF")); break;   // [QOL]
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
  lcd.print(F("Cloro: ")); imprimirPpm(lcd, ultimaX10); lcd.print(F(" ppm"));
  lcd.setCursor(15, 0); lcd.print(modoPrueba ? 'T' : 'P');
  lcd.setCursor(0, 1);
  print2(ultimaHora.hour()); lcd.print(':');
  print2(ultimaHora.minute()); lcd.print(':');
  print2(ultimaHora.second()); lcd.print(' ');
  lcd.print(tipoAlerta == TIPO_BAJO ? F("BAJO")
          : (tipoAlerta == TIPO_ALTO ? F("ALTO") : F("NORMAL")));
  lcd.setCursor(15, 1);
  if (!sdOk)       lcd.print('!');           // '!' = problema con la SD
  else if (!rtcOk) lcd.print('?');           // [QOL] '?' = sin RTC (hora = tiempo desde el encendido)
}

void dibujarAlerta() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(tipoAlerta == TIPO_BAJO ? F("!ALERTA: BAJO!") : F("!ALERTA: ALTO!"));
  lcd.setCursor(0, 1);
  imprimirPpm(lcd, ultimaX10); lcd.print(F(" ppm Pulsar"));
}

void dibujarPromedio() {
  lcd.clear();
  if (resCodigo == 2) { lcd.print(F("Error SD")); return; }
  if (resCodigo == 1) { lcd.print(F("Sin datos aun")); return; }
  if (pagProm == 0) {
    lcd.print(F("Prom: ")); imprimirPpm(lcd, resProm); lcd.print(F(" ppm"));
    lcd.setCursor(0, 1);
    lcd.print(F("N:")); lcd.print(resN);
    lcd.print(F(" Fuera:")); lcd.print(resBajo + resAlto);
  } else if (pagProm == 1) {
    lcd.print(F("Min:")); imprimirPpm(lcd, resMin);
    lcd.print(F(" Max:")); imprimirPpm(lcd, resMax);
    lcd.setCursor(0, 1);
    lcd.print(F("Baj:")); lcd.print(resBajo);
    lcd.print(F(" Alt:")); lcd.print(resAlto);
  } else {
    // [QOL] Periodo real que cubren los datos (sirve para ver si hubo cortes:
    // en 12 h sin cortes, Ini y Fin deberian estar separados 12 h).
    lcd.print(F("Ini ")); imprimirFechaCorta(resIni);
    lcd.setCursor(0, 1);
    lcd.print(F("Fin ")); imprimirFechaCorta(resFin);
  }
  // [QOL] Numero de pagina arriba y flechas abajo (antes '>' / '<', que sugerian
  // mover a los lados cuando en realidad se cambiaba con arriba/abajo).
  lcd.setCursor(15, 0); lcd.print(pagProm + 1);
  lcd.setCursor(15, 1); lcd.write(CH_FLECHAS);
}

void dibujarRango() {
  lcd.clear();
  lcd.print(F("Rango normal:"));
  lcd.setCursor(0, 1);
  imprimirPpm(lcd, UMBRAL_BAJO_X10); lcd.print(F(" - "));
  imprimirPpm(lcd, UMBRAL_ALTO_X10); lcd.print(F(" ppm"));
}

void iniciarMensaje() {
  estado = E_MSG;
  estadoTrasMsg = E_MENU;                   // [QOL] por defecto vuelve al menu (confirmarAlerta lo cambia)
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

void redibujar() {
  if (estado == E_MSG) estado = estadoTrasMsg;   // un mensaje tapado por una alerta se descarta
  switch (estado) {
    case E_MENU:     dibujarMenu();     break;
    case E_VIVO:     dibujarVivo();     break;
    case E_PROMEDIO: dibujarPromedio(); break;
    case E_RANGO:    dibujarRango();    break;
  }
}

// [QOL] Cambia de pantalla y la dibuja
void irA(byte e) {
  estado = e;
  redibujar();
}

void volverMenu() {
  irA(E_MENU);
}

// [QOL] Luz del LCD: encenderla tambien cuenta como actividad
void encenderLuz() {
  if (!luzOn) { lcd.backlight(); luzOn = true; }
  ultimaActividad = millis();
}

// [QOL] Reposo: sin tocar nada vuelve a la lectura en vivo y luego apaga la luz.
// Nunca actua con una alerta pendiente.
void revisarInactividad() {
  if (alertaPendiente()) return;
  unsigned long quieto = millis() - ultimaActividad;
  if (REPOSO_MS && quieto >= REPOSO_MS && estado != E_VIVO) irA(E_VIVO);
  if (LUZ_MS && luzOn && quieto >= LUZ_MS) { lcd.noBacklight(); luzOn = false; }
}

// ============================================================
//  Menu y logica de interfaz
// ============================================================
void ejecutarItem() {
  switch (itemSel) {
    case 0:
      irA(E_VIVO);
      break;
    case 1: {
      char nombre[13];
      lcd.clear(); lcd.print(F("Copiando..."));   // [QOL] el dump tarda; avisa que no se colgo
      byte r = crearDump(nombre);
      if (r == 0)      mensajeRam(F("Dump OK"), nombre);
      else if (r == 1) mensaje(F("Sin datos"), F("para copiar"));
      else             mensaje(F("Error SD"), F("Revise tarjeta"));
      break;
    }
    case 2:
      lcd.clear(); lcd.print(F("Calculando..."));  // [QOL] idem
      resCodigo = analizar();
      pagProm = 0;
      irA(E_PROMEDIO);
      break;
    case 3:
      irA(E_RANGO);
      break;
    case 4:
      modoPrueba = !modoPrueba;
      if (!modoPrueba) sonido = true;      // [QOL] seguridad: en Produccion el buzzer vuelve a ON
      tomarLectura();                      // lectura inmediata con el nuevo intervalo
      dibujarMenu();
      break;
    case 5:
      daltonismo = !daltonismo;
      EEPROM.update(0, daltonismo ? 1 : 0);
      dibujarMenu();
      break;
    case 6:                                // [QOL] silenciar / activar buzzer
      sonido = !sonido;
      dibujarMenu();
      break;
  }
}

void confirmarAlerta() {
  // [QOL] Recordar donde estaba el usuario para volver ahi despues del mensaje
  byte volverA = (estado == E_MSG) ? estadoTrasMsg : estado;
  confirmada = true;
  confirmadaEn = millis();                 // [QOL] para el recordatorio
  verdeHasta = millis() + VERDE_MS;
  overlay = false;
  noTone(PIN_BUZZER);
  buzzerOn = false;
#if DEBUG_SERIAL
  Serial.println(F("[ALERTA] confirmada"));
#endif
  mensaje(F("Alerta"), F("confirmada"));
  estadoTrasMsg = volverA;                 // [QOL] antes siempre volvia al menu
}

// dx: eje horizontal ([QOL]) -1 = izquierda (volver), +1 = derecha (entrar / siguiente)
void manejarUI(bool btn, int8_t dir, int8_t dx) {
  switch (estado) {
    case E_MENU:
      if (dir) {
        itemSel = (itemSel + (dir > 0 ? 1 : N_ITEMS - 1)) % N_ITEMS;
        dibujarMenu();
      } else if (btn || dx > 0) {
        ejecutarItem();
      }
      break;
    case E_VIVO:
      if (btn || dx < 0) volverMenu();
      else if (lecturaNueva) dibujarVivo();
      break;
    case E_PROMEDIO:
      if (btn || dx < 0) volverMenu();
      else if (dir || dx > 0) {
        // [QOL] 3 paginas; arriba/abajo recorren en ambos sentidos, derecha avanza
        int8_t paso = dir ? dir : 1;
        pagProm = (pagProm + (paso > 0 ? 1 : N_PAG_PROM - 1)) % N_PAG_PROM;
        dibujarPromedio();
      }
      break;
    case E_RANGO:
      if (btn || dx < 0) volverMenu();
      break;
    case E_MSG:
      if (btn || dx < 0 || (long)(millis() - msgHasta) >= 0) irA(estadoTrasMsg);
      break;
  }
}

// ============================================================
//  [QOL] Comandos por monitor serial
//  Escribe el comando y pulsa Enter:
//    d                       vuelca las ultimas 720 lecturas del log activo (CSV)
//    s                       estado (modo, RTC, SD) y estadisticas
//    t 2026-11-24 15:30:00   ajusta el RTC
//    ?                       ayuda
//  OJO: el volcado bloquea mientras transmite (~20 s a 9600 baudios, ~2 s a 115200);
//  durante ese tiempo no se atiende el joystick.
// ============================================================
#if COMANDOS_SERIAL
void ayudaSerial() {
  Serial.println(F("Comandos: d = volcar lecturas | s = estado | ? = ayuda"));
  Serial.println(F("          t AAAA-MM-DD HH:MM:SS = ajustar reloj"));
}

void estadoSerial() {
  Serial.print(F("Modo: ")); Serial.print(modoPrueba ? F("PRUEBA") : F("PRODUCCION"));
  Serial.print(F(" | RTC: ")); Serial.print(rtcOk ? F("OK") : F("ERROR"));
  resCodigo = analizar();                  // (puede reconectar la SD)
  Serial.print(F(" | SD: ")); Serial.println(sdOk ? F("OK") : F("ERROR"));
  if (resCodigo) { Serial.println(resCodigo == 2 ? F("Error SD") : F("Sin datos")); return; }
  Serial.print(F("N=")); Serial.print(resN);
  Serial.print(F(" prom=")); imprimirPpm(Serial, resProm);
  Serial.print(F(" min=")); imprimirPpm(Serial, resMin);
  Serial.print(F(" max=")); imprimirPpm(Serial, resMax);
  Serial.print(F(" bajo=")); Serial.print(resBajo);
  Serial.print(F(" alto=")); Serial.println(resAlto);
}

void ajustarHora(const char* p) {
  while (*p == ' ') p++;
  // Formato fijo "AAAA-MM-DD HH:MM:SS": revisa que haya digitos donde corresponde
  bool ok = strlen(p) >= 19 && p[0] == '2' && p[1] == '0';
  for (byte i = 0; ok && i < 19; i++) {
    if (i == 4 || i == 7 || i == 10 || i == 13 || i == 16) continue;
    if (!isDigit(p[i])) ok = false;
  }
  DateTime t(p);                           // RTClib entiende este formato
  if (!ok || !t.isValid()) { Serial.println(F("Formato: t 2026-11-24 15:30:00")); return; }
  if (!rtcOk) { Serial.println(F("Sin RTC")); return; }
  rtc.adjust(t);
  Serial.println(F("Reloj ajustado"));
}

void procesarComando(char* c) {
  switch (c[0]) {
    case 'd': case 'D': {
      if (!asegurarSD(true)) { Serial.println(F("Error SD")); break; }
      Serial.println(F("fecha_hora,ppm"));
      uint16_t n = copiarUltimas(Serial);
      Serial.print(F("# ")); Serial.print(n); Serial.println(F(" lecturas"));
      break;
    }
    case 's': case 'S': estadoSerial(); break;
    case 't': case 'T': ajustarHora(c + 1); break;
    default: ayudaSerial(); break;
  }
}

// Junta caracteres sin bloquear; procesa la linea al recibir Enter
void leerSerial() {
  static char buf[24];
  static byte n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n) { buf[n] = '\0'; procesarComando(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    }
  }
}
#endif

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

#if USAR_SERIAL
  Serial.begin(BAUDIOS);
#endif

  lcd.init();
  lcd.backlight();
  // [QOL] Caracter propio: flechas arriba/abajo (se usa en "Promedio")
  byte flechas[8] = { 0x04, 0x0E, 0x1F, 0x00, 0x1F, 0x0E, 0x04, 0x00 };
  lcd.createChar(CH_FLECHAS, flechas);
  lcd.clear();                              // createChar deja el cursor en la CGRAM
  lcd.setCursor(0, 0); lcd.print(F("Monitor de cloro"));
  lcd.setCursor(0, 1); lcd.print(F("Iniciando..."));

  randomSeed(analogRead(A3) + micros());

  rtcOk = rtc.begin();
  if (rtcOk && rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));   // hora de compilacion
  }
  sdOk = SD.begin(PIN_SD_CS);
  ultimoIntentoSD = millis();               // [FIX]
  daltonismo = (EEPROM.read(0) == 1);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(F("RTC:")); lcd.print(rtcOk ? F("OK") : F("ERROR"));
  lcd.setCursor(0, 1); lcd.print(F("SD: ")); lcd.print(sdOk ? F("OK") : F("ERROR"));
  delay(1500);

#if COMANDOS_SERIAL
  ayudaSerial();
#endif

  tomarLectura();
  volverMenu();
  ultimaActividad = millis();               // [QOL]
}

void loop() {
  if (millis() - ultimaMuestra >= intervaloMs()) tomarLectura();

#if COMANDOS_SERIAL
  leerSerial();                             // [QOL]
#endif

  bool btn = botonPresionado();
  int8_t dir = leerEje(ejeY, true);
#if JOY_USAR_X
  int8_t dx = leerEje(ejeX, false);         // [QOL]
#else
  int8_t dx = 0;
#endif

  // [QOL] Luz del LCD: una alerta pendiente siempre la enciende. Si estaba
  // apagada, el primer toque solo la enciende y no hace nada mas (asi no
  // ejecutas algo a ciegas).
  if (alertaPendiente() && !luzOn) encenderLuz();
  if (btn || dir || dx) {
    bool dormida = !luzOn;
    encenderLuz();
    if (dormida) { btn = false; dir = 0; dx = 0; }
  }
  revisarInactividad();

  if (alertaPendiente()) {
    if (btn) {
      confirmarAlerta();                       // el boton solo confirma mientras hay alerta
    } else if (!overlay || lecturaNueva) {
      overlay = true;
      dibujarAlerta();
    }
  } else {
    if (overlay) { overlay = false; redibujar(); }
    manejarUI(btn, dir, dx);
  }

  lecturaNueva = false;
  actualizarSalidas();
}
