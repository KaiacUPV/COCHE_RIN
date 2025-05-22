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
int x = 0;
int y = 0;

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

void setup() {
  Serial.begin(115200);
  delay(1000); // Pequeña pausa para iniciar el monitor

  //motores
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Canal PWM 0 → ENA, Canal PWM 1 → ENB
  ledcAttach(ENA, 1000, 8);  // canal 0
  ledcAttach(ENB, 1000, 8);  // canal 1


  pinMode(sensorPin1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin1), pulseISR1, FALLING);

  pinMode(sensorPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(sensorPin2), pulseISR2, FALLING);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Conexión manual al WiFi (opcional, para mostrar estado)
  WiFi.begin(ssid, pass);
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectado");
  Serial.print("📶 Dirección IP: ");
  Serial.println(WiFi.localIP());

  // Iniciar conexión a Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("🔗 Conectando a Blynk...");

  lastTime1 = millis();
  lastTime2 = millis();
  lastDistTime = millis();
}

void loop() {
  Blynk.run();
  unsigned long currentTime = millis();

  // Sensor 1 RPM
  if (currentTime - lastTime1 >= 1000) {
    const float pulsosPorVuelta = 20.0;
    rpm1 = (pulseCount1 * 60.0) / pulsosPorVuelta;
    Serial.print("Velocidad sensor 1: ");
    Serial.print(rpm1);
    Serial.println(" RPM");
    Blynk.virtualWrite(V10, rpm1);
    pulseCount1 = 0;
    lastTime1 = currentTime;
  }

  // Sensor 2 RPM
  if (currentTime - lastTime2 >= 1000) {
    const float pulsosPorVuelta = 20.0;
    rpm2 = (pulseCount2 * 60.0) / pulsosPorVuelta;
    Serial.print("Velocidad sensor 2: ");
    Serial.print(rpm2);
    Serial.println(" RPM");
    Blynk.virtualWrite(V11, rpm2);
    pulseCount2 = 0;
    lastTime2 = currentTime;
  }

  // Sensor de distancia ultrasónico (cada 300 ms)
  if (currentTime - lastDistTime >= 300) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duracion = pulseIn(ECHO_PIN, HIGH, 25000); // Timeout 25 ms = 400 cm máx
    distancia_cm = duracion * 0.0343 / 2.0;

    Serial.print("Distancia: ");
    Serial.print(distancia_cm);
    Serial.println(" cm");

    Blynk.virtualWrite(V9, distancia_cm);
    lastDistTime = currentTime;
  }



  //motores
    // Movimiento del joystick (modo diferencial)
  int velocidad = abs(y - 128) * 2;  // convierte 0-255 en 0-255 relativo
  velocidad = constrain(velocidad, 0, 255);

  // Dirección del motor A
  if (y > 140) { // adelante
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (y < 115) { // atrás
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    velocidad = 0;
  }

  // Dirección del motor B (igual que A, simplificado)
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

  // PWM (velocidad en ambos motores)
  ledcWrite(ENA, velocidad); // canal 0
  ledcWrite(ENB, velocidad); // canal 1

}
