from ultralytics import YOLO
import cv2
import time
import serial

# =============================
# Serial ESP32
# =============================
esp32 = serial.Serial("COM5", 115200, timeout=1)  # cambia COM
time.sleep(2)

# =============================
# Modelo
# =============================
model = YOLO("best.pt")

# =============================Stevebnestubo aqui 
# Cámara
# =============================
cap = cv2.VideoCapture(1)

# =============================
# Parámetros de decisión
# =============================
FRAMES_ESTABLES = 8
UMBRAL_CAMBIO = 0.05
UMBRAL_MALO = 0.70  # 80 %

# =============================
# Estado por carril
# =============================
estado = {
    "IZQUIERDA": {
        "decidido": False,
        "area_anterior": None,
        "frames_estables": 0,
        "malo_fuerte": False,      # ← si alguna vez fue malo >= 80 %
        "mejor_clase": None        # ← clase final a enviar
    },
    "DERECHA": {
        "decidido": False,
        "area_anterior": None,
        "frames_estables": 0,
        "malo_fuerte": False,
        "mejor_clase": None
    }
}

# =============================
# Loop principal
# =============================
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame, verbose=False)
    detecciones = results[0].boxes

    ancho = frame.shape[1]
    carriles_detectados = set()

    if detecciones is not None and len(detecciones) > 0:
        for i in range(len(detecciones)):
            clase = int(detecciones.cls[i])
            nombre_clase = model.names[clase].upper()
            conf = float(detecciones.conf[i])

            x1, y1, x2, y2 = detecciones.xyxy[i]
            area = (x2 - x1) * (y2 - y1)
            cx = (x1 + x2) / 2

            carril = "IZQUIERDA" if cx < ancho / 2 else "DERECHA"
            carriles_detectados.add(carril)

            if estado[carril]["decidido"]:
                continue

            # =============================
            # Regla de MALO definitivo
            # =============================
            if nombre_clase == "MALO" and conf >= UMBRAL_MALO:
                estado[carril]["malo_fuerte"] = True
                estado[carril]["mejor_clase"] = "MALO"

            # =============================
            # Estabilidad de la caja
            # =============================
            area_prev = estado[carril]["area_anterior"]

            if area_prev is None:
                estado[carril]["area_anterior"] = area
                estado[carril]["frames_estables"] = 0
                estado[carril]["mejor_clase"] = nombre_clase
                continue

            cambio = abs(area - area_prev) / area_prev

            if cambio < UMBRAL_CAMBIO:
                estado[carril]["frames_estables"] += 1
            else:
                estado[carril]["frames_estables"] = 0

            estado[carril]["area_anterior"] = area

            # =============================
            # DECISIÓN FINAL
            # =============================
            if estado[carril]["frames_estables"] >= FRAMES_ESTABLES:
                # si alguna vez fue malo fuerte → MALO
                if estado[carril]["malo_fuerte"]:
                    clase_final = "MALO"
                else:
                    clase_final = estado[carril]["mejor_clase"]

                mensaje = f"{clase_final}|{carril}\n"
                esp32.write(mensaje.encode())
                print(mensaje.strip())

                estado[carril]["decidido"] = True

    # =============================
    # Reset SOLO del carril que quedó vacío
    # =============================
    for carril in estado:
        if carril not in carriles_detectados:
            estado[carril]["decidido"] = False
            estado[carril]["area_anterior"] = None
            estado[carril]["frames_estables"] = 0
            estado[carril]["malo_fuerte"] = False
            estado[carril]["mejor_clase"] = None

    annotated_frame = results[0].plot()
    cv2.imshow("YOLO Inference", annotated_frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
esp32.close()
cv2.destroyAllWindows()
