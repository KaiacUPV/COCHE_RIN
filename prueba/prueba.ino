#define BLYNK_TEMPLATE_ID "TMPL5RI12qGXX"
#define BLYNK_TEMPLATE_NAME "Quickstart Template"
#define BLYNK_AUTH_TOKEN "Ud6HBga4wFKKEy97he586quXD6iOL7HY"
#define TRIG_PIN 26
#define ECHO_PIN 27
// Pines del L293D para motor A
#define IN1 4
#define IN2 18
#define ENA 5  // PWM canal 0

// Pines del L293D para motor B
#define IN3 12
#define IN4 32
#define ENB 13 // PWM canal 1

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Wi-Fi
char ssid[] = "Kaiac";
char pass[] = "62mari2lasucla";

// Variables globales para joystick
volatile int x = 0;
volatile int y = 0;

// Variables para sensor 1
volatile int pulseCount1 = 0;
unsigned long lastTime1 = 0;
float rpm1 = 0;
const int sensorPin1 = 16;

// Variables para sensor 2
volatile int pulseCount2 = 0;
unsigned long lastTime2 = 0;
float rpm2 = 0;
const int sensorPin2 = 17;

// Variables para distancia
unsigned long lastDistTime = 0;
float distancia_cm = 0;

// ISR para sensor 1
void IRAM_ATTR pulseISR1() {
  pulseCount1++;
}

// ISR para sensor 2
void IRAM_ATTR pulseISR2() {
  pulseCount2++;
}

// Lectura del eje X (V4)
BLYNK_WRITE(V4) {
  x = param.asInt();
  Serial.print("X: ");
  Serial.println(x);
}

// Lectura del eje Y (V5)
BLYNK_WRITE(V5) {
  y = param.asInt();
  Serial.print("Y: ");
  Serial.println(y);
}

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;



// Tarea sensores RPM y ultrasonido
void taskSensores(void *pvParameters) {
  const float pulsosPorVuelta = 20.0;
  for (;;) {
    unsigned long currentTime = millis();

    // Sensor 1 RPM cada 1 segundo
    if (currentTime - lastTime1 >= 1000) {
      portENTER_CRITICAL_ISR(&mux);
      int count1 = pulseCount1;
      pulseCount1 = 0;
      portEXIT_CRITICAL_ISR(&mux);

      rpm1 = (count1 * 60.0) / pulsosPorVuelta;
      Serial.print("Velocidad sensor 1: ");
      Serial.print(rpm1);
      Serial.println(" RPM");
      Blynk.virtualWrite(V10, rpm1);
      lastTime1 = currentTime;
    }

    // Sensor 2 RPM cada 1 segundo
    if (currentTime - lastTime2 >= 1000) {
      portENTER_CRITICAL_ISR(&mux);
      int count2 = pulseCount2;
      pulseCount2 = 0;
      portEXIT_CRITICAL_ISR(&mux);

      rpm2 = (count2 * 60.0) / pulsosPorVuelta;
      Serial.print("Velocidad sensor 2: ");
      Serial.print(rpm2);
      Serial.println(" RPM");
      Blynk.virtualWrite(V11, rpm2);
      lastTime2 = currentTime;
    }

    // Ultrasonido cada 300 ms
    if (currentTime - lastDistTime >= 300) {
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      long duracion = pulseIn(ECHO_PIN, HIGH, 25000); // timeout 25ms
      distancia_cm = duracion * 0.0343 / 2.0;

      Serial.print("Distancia: ");
      Serial.print(distancia_cm);
      Serial.println(" cm");
      Blynk.virtualWrite(V9, distancia_cm);
      lastDistTime = currentTime;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// Tarea control motores
void taskMotores(void *pvParameters) {
  for (;;) {
    int velocidad = abs(y - 128) * 2;
    velocidad = constrain(velocidad, 0, 255);

    // Motor A
    if (y > 140) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (y < 115) {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      velocidad = 0;
    }

    // Motor B
    if (y > 140) {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    } else if (y < 115) {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

    // PWM (velocidad en ambos motores) - nueva API
    ledcWrite(ENA, velocidad);  // ENA = pin 5
    ledcWrite(ENB, velocidad);  // ENB = pin 13

    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pines motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Nueva API PWM
  ledcAttach(ENA, 1000, 8);  // pin ENA, frecuencia 1kHz, resolución 8 bits
  ledcAttach(ENB, 1000, 8);  // pin ENB, frecuencia 1kHz, resolución 8 bits

  // Sensores RPM
  pinMode(sensorPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin1), pulseISR1, FALLING);

  pinMode(sensorPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin2), pulseISR2, FALLING);

  // Ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // WiFi
  WiFi.begin(ssid, pass);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectado");
  Serial.print("📶 Dirección IP: ");
  Serial.println(WiFi.localIP());

  // Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("🔗 Conectando a Blynk...");

  lastTime1 = millis();
  lastTime2 = millis();
  lastDistTime = millis();

  // Crear tareas FreeRTOS
  xTaskCreatePinnedToCore(
    taskSensores,
    "Sensores",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  xTaskCreatePinnedToCore(
    taskMotores,
    "Motores",
    2048,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  Blynk.run();
  delay(10);
}
