# Proyecto: Teclado HID Bluetooth con Joystick (XIAO ESP32-C3)

Este proyecto convierte un Seeed Studio XIAO ESP32-C3 en un teclado HID Bluetooth especializado. Utiliza un joystick analógico para la selección de teclas mediante un sistema de "direcciones" y botones físicos para gestionar "capas" de caracteres.

## 📋 Especificaciones de Hardware (XIAO ESP32-C3)

Para que el proyecto funcione en la placa XIAO, se ha utilizado el siguiente mapeo de pines:

| Componente | Pin XIAO (Serigrafía) | GPIO ESP32-C3 | Función |
| :--- | :--- | :--- | :--- |
| **Joystick Eje X** | A0 | GPIO 2 | Entrada Analógica (ADC1_CH2) |
| **Joystick Eje Y** | A1 | GPIO 3 | Entrada Analógica (ADC1_CH3) |
| **Botón Capa 1** | D2 / A2 | GPIO 4 | Entrada Digital (Pull-up) |
| **Botón Capa 2** | D3 / A3 | GPIO 5 | Entrada Digital (Pull-up) |
| **Botón Extra** | D8 | GPIO 6 | Entrada Digital (Pull-up) |

---

## 🔍 Explicación Detallada del Código

### 1. `pruebaWroom.c` (Lógica de Aplicación y Hardware)

Este archivo es el "cerebro" del dispositivo. Se encarga de leer los sensores físicos y decidir qué tecla enviar por Bluetooth.

#### **A. Configuración de Hardware (Pines y ADC)**
*   **Joystick (ADC1)**: Los ejes X e Y están en los canales 2 y 3 (GPIO 2 y 3 del XIAO). Se configura con una resolución de **12 bits** (valores de 0 a 4095) y **11dB de atenuación**, lo que permite mapear el voltaje de 0-3.3V al rango numérico completo del ESP32-C3.
*   **Botones (GPIO)**: Los pines 4, 5 y 6 se configuran con **Pull-up interno**. Esto significa que el pin lee `1` por defecto y `0` cuando el botón conecta el pin a tierra (GND).

#### **B. El Sistema de "Capas" y Direcciones**
*   **Diccionario (`diccionario[4][8]`)**: Es una matriz de 4 capas (seleccionadas con botones) por 8 direcciones (seleccionadas con el joystick). Cada celda contiene un `keycode` HID (por ejemplo, `0x04` para la letra 'A').
*   **`leer_joystick()`**: Esta función toma los valores RAW del ADC y los compara con umbrales calibrados. Divide el movimiento del joystick en 8 sectores (como una rosa de los vientos). Si el joystick está cerca del centro (valor ~2000), devuelve `9` (reposo).

#### **C. El Bucle Principal (`app_main`)**
Corre infinitamente cada 20ms:
1.  **Detecta la Capa**: Lee los botones 1 y 2 para formar un número binario del 0 al 3.
2.  **Lee la Dirección**: Llama a `leer_joystick()`.
3.  **Debounce y Envío**: Para evitar enviar múltiples letras por un solo toque, el código espera a que la lectura sea estable (`debounce_counter`). Cuando se confirma, llama a `send_keyboard_pulse()`, que envía el código de la tecla y luego un código "vacío" (0) para simular que la tecla se ha soltado.

---

### 2. `nimble.c` (Pila Bluetooth Low Energy)

Este archivo implementa el estándar **HOGP** (HID Over GATT Profile), que permite que el ESP32 aparezca como un teclado estándar ante cualquier sistema operativo.

#### **A. El Mapa de Reporte HID (`hid_report_map`)**
Es el descriptor más crítico. Es un array de bytes que le dice al ordenador: *"Soy un teclado, enviaré informes de 8 bytes: el primero para modificadores (Ctrl/Shift), el segundo reservado, y los otros 6 para teclas presionadas"*. Sin esto, el ordenador vería datos pero no sabría que son pulsaciones de teclas.

#### **B. Servicios GATT**
El código define tres servicios primarios:
1.  **HID Service (`0x1812`)**: Contiene las características de protocolo, el mapa de reporte y el reporte de entrada (donde se envían las teclas).
2.  **Device Information Service (`0x180A`)**: Informa al PC quién fabricó el dispositivo (PnP ID).
3.  **Battery Service (`0x180F`)**: Informa el nivel de batería (fijo al 100% en este código).

#### **C. Seguridad y Emparejamiento**
En `ble_hid_init`, se configura la seguridad:
*   **Bonding**: Permite que el dispositivo "recuerde" al PC para que no tengas que emparejarlo cada vez que lo enciendes.
*   **Just Works**: Como el dispositivo no tiene pantalla para mostrar un PIN, utiliza el método de emparejamiento más simple.

#### **D. Eventos de GAP (`ble_gap_event`)**
Maneja el ciclo de vida de la conexión:
*   **`CONNECT`**: Cuando un PC se conecta, el ESP32 inicia la solicitud de seguridad (encriptación).
*   **`DISCONNECT`**: Si se pierde la conexión, el código reinicia automáticamente los **Advertisements** (anuncios) para que el dispositivo vuelva a ser visible.
*   **`SUBSCRIBE`**: El PC debe "suscribirse" a las notificaciones del reporte HID antes de que el ESP32 pueda enviarle teclas. La variable `notify_state` rastrea este estado.

#### **E. Envío de Datos (`ble_hid_send_report`)**
Es la función que finalmente empaqueta los datos. Toma el modificador y las teclas, los mete en un buffer de memoria (`os_mbuf`) y usa `ble_gatts_notify_custom` para "empujar" ese dato hacia el PC conectado de forma asíncrona.

---

## 🚀 Cómo Compilar y Flashear

1. Asegúrate de tener instalado el entorno **ESP-IDF** (v4.4 o superior).
2. Configura el target para el ESP32-C3:
   ```bash
   idf.py set-target esp32c3
   ```
3. Compila y flashea:
   ```bash
   idf.py build flash monitor
   ```

## 🔍 Notas de Calibración
Si el joystick no responde con precisión, los valores de comparación en la función `leer_joystick` deben ajustarse basándose en los valores RAW obtenidos durante la fase de diagnóstico (herramienta de testeo). Actualmente, se asume un centro cercano a `2000` y extremos en `0` y `4000`.
