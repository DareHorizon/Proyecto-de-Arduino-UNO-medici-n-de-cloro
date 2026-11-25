#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// PINES
#define PIN_POT_CLORO   34    // Simula sonda ORP/Cloro
#define PIN_BOTON_ACK   18    // Botón de confirmación (Pabellonero)
#define PIN_BALIZA_ROJA 19    // Alerta visual (<3 ppm o >5 ppm)
#define PIN_LED_OK      23    // Rango normal (3.0 a 5.0 ppm)

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// PARAMETROS
const float CLORO_MIN = 3.0;
const float CLORO_MAX = 5.0;

// VARIABLES
float cloro_ppm = 0.0;
bool en_alerta = false;
bool alerta_confirmada = false;
unsigned long ultimo_muestreo = 0;
const unsigned long INTERVALO_MUESTREO = 2000; // 2 segundos en simulación

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BOTON_ACK, INPUT_PULLUP);
  pinMode(PIN_BALIZA_ROJA, OUTPUT);
  pinMode(PIN_LED_OK, OUTPUT);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Error al iniciar SSD1306"));
  }

  display.clearDisplay();
  display.setTextColor(WHITE);
  Serial.println(F("--- Sistema IoT: Monitoreo de Cloro Inicializado ---"));
}

void loop() {
  // Detección de acuse de recibo del operario
  if (digitalRead(PIN_BOTON_ACK) == LOW && en_alerta) {
    alerta_confirmada = true;
    digitalWrite(PIN_BALIZA_ROJA, LOW); // Apaga la baliza al confirmar
    Serial.println(F("[ACK] Operario confirmó la alerta. Estado: REVISADA"));
    delay(250); // Antirrebote básico
  }

  // CICLO TEMPORIZADO
  if (millis() - ultimo_muestreo >= INTERVALO_MUESTREO) {
    ultimo_muestreo = millis();

    // Lectura analógica escalada a 0.0 - 10.0 ppm
    int adc_val = analogRead(PIN_POT_CLORO);
    cloro_ppm = (adc_val / 4095.0) * 10.0;

    // MAQUINA DE ESTADO
    if (cloro_ppm < CLORO_MIN) {
      en_alerta = true;
      digitalWrite(PIN_LED_OK, LOW);
      if (!alerta_confirmada) digitalWrite(PIN_BALIZA_ROJA, HIGH);

      Serial.printf("[CRÍTICO] Cloro Bajo: %.2f ppm (< 3.0 ppm) - Riesgo Sanitario\n", cloro_ppm);
    } 
    else if (cloro_ppm > CLORO_MAX) {
      en_alerta = true;
      digitalWrite(PIN_LED_OK, LOW);
      if (!alerta_confirmada) digitalWrite(PIN_BALIZA_ROJA, HIGH);

      Serial.printf("[ALERTA] Cloro Alto: %.2f ppm (> 5.0 ppm) - Sobredosificación\n", cloro_ppm);
    } 
    else {
      // ESTADO OPTIMO
      en_alerta = false;
      alerta_confirmada = false;
      digitalWrite(PIN_BALIZA_ROJA, LOW);
      digitalWrite(PIN_LED_OK, HIGH);

      Serial.printf("[NORMAL] Cloro en rango: %.2f ppm (Optimo 3-5 ppm)\n", cloro_ppm);
    }

    // PANTALLA LED
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("CLORACION DEL AGUA");
    display.drawLine(0, 10, 128, 10, WHITE);

    display.setTextSize(2);
    display.setCursor(10, 16);
    display.printf("%.2f ppm", cloro_ppm);

    display.setTextSize(1);
    display.setCursor(0, 38);
    if (cloro_ppm < CLORO_MIN) {
      display.print("ALERTA: CLORO BAJO");
    } else if (cloro_ppm > CLORO_MAX) {
      display.print("ALERTA: CLORO ALTO");
    } else {
      display.print("ESTADO: EN RANGO OK");
    }

    display.setCursor(0, 52);
    if (en_alerta) {
      display.print(alerta_confirmada ? "Status: REVISADA" : "Status: SIN CONFIRMAR");
    } else {
      display.print("Red: 3.0 - 5.0 ppm");
    }
    display.display();
  }
}