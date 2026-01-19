#include <ESP32Servo.h>

// =============================
// PINES
// =============================
#define PIN_PIR_DERECHO    26
#define PIN_PIR_IZQUIERDO  25
#define PIN_IR_PUERTA      27

#define SERVO_PUERTA       14
#define SERVO_DESVIO       12

// =============================
// ÁNGULOS
// =============================
#define PUERTA_CERRADA  0
#define PUERTA_ABIERTA  90

#define DESVIO_IZQ     35
#define DESVIO_CENTRO  75
#define DESVIO_DER     110

// =============================
#define RETARDO_PIR 3500
#define MAX_EVENTOS 10

Servo servoPuerta;
Servo servoDesvio;

// =============================
// SENSOR PUERTA
// =============================
bool objetoPuerta = false;

// =============================
// ESTRUCTURA COLA
// =============================
struct EventoMalo {
  String carril;
  bool pirArmado;       // 🔑 PIR listo (vio LOW)
  bool pirDetectado;    // flanco detectado
  unsigned long t0;
};

EventoMalo cola[MAX_EVENTOS];
int head = 0;
int tail = 0;

// =============================
// COLA HELPERS
// =============================
bool colaVacia() {
  return head == tail;
}

bool colaLlena() {
  return ((tail + 1) % MAX_EVENTOS) == head;
}

void pushEvento(String carril) {
  if (!colaLlena()) {
    cola[tail] = { carril, false, false, 0 };
    tail = (tail + 1) % MAX_EVENTOS;
    Serial.println("Evento en cola: " + carril);
  }
}

EventoMalo& frontEvento() {
  return cola[head];
}

void popEvento() {
  head = (head + 1) % MAX_EVENTOS;
}

// =============================
// SETUP
// =============================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_PIR_IZQUIERDO, INPUT);
  pinMode(PIN_PIR_DERECHO, INPUT);
  pinMode(PIN_IR_PUERTA, INPUT);

  servoPuerta.attach(SERVO_PUERTA);
  servoDesvio.attach(SERVO_DESVIO);

  servoPuerta.write(PUERTA_CERRADA);
  servoDesvio.write(DESVIO_CENTRO);

  Serial.println("ESP32 lista (PIR ARMADO)");
}

// =============================
// LOOP
// =============================
void loop() {

  // =============================
  // PUERTA SIEMPRE
  // =============================
  if (digitalRead(PIN_IR_PUERTA) == HIGH && !objetoPuerta) {
    objetoPuerta = true;
    servoPuerta.write(PUERTA_ABIERTA);
    delay(800);
    servoPuerta.write(PUERTA_CERRADA);
  }
  if (digitalRead(PIN_IR_PUERTA) == LOW) {
    objetoPuerta = false;
  }

  // =============================
  // SERIAL → COLA
  // =============================
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "MALO|IZQUIERDA") pushEvento("IZQUIERDA");
    if (cmd == "MALO|DERECHA")   pushEvento("DERECHA");
  }

  // =============================
  // PROCESAR EVENTO
  // =============================
  if (!colaVacia()) {
    EventoMalo &e = frontEvento();

    int pinPIR = (e.carril == "IZQUIERDA") ? PIN_PIR_IZQUIERDO : PIN_PIR_DERECHO;
    bool pirEstado = digitalRead(pinPIR);

    // 1️⃣ Esperar que PIR esté en LOW (armar)
    if (!e.pirArmado) {
      if (!pirEstado) {
        e.pirArmado = true;
        Serial.println("PIR armado " + e.carril);
      }
      return;
    }

    // 2️⃣ Esperar flanco LOW → HIGH
    if (e.pirArmado && !e.pirDetectado) {
      if (pirEstado) {
        e.pirDetectado = true;
        e.t0 = millis();
        Serial.println("PIR activado " + e.carril);
      }
      return;
    }

    // 3️⃣ Esperar retardo
    if (e.pirDetectado && millis() - e.t0 >= RETARDO_PIR) {

      if (e.carril == "IZQUIERDA") servoDesvio.write(DESVIO_IZQ);
      else                         servoDesvio.write(DESVIO_DER);

      delay(400);
      servoDesvio.write(DESVIO_CENTRO);

      Serial.println("Desvío ejecutado " + e.carril);

      popEvento();
    }
  }
}
