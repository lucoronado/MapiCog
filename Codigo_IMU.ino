// =======================================================
// === ESP32 SLAVE: SISTEMA IMU CON SINCRONIZACIÓN ===
// =======================================================
// Este código implementa un sistema de captura de datos IMU (Unidad de Medición Inercial)
// que se sincroniza con un ESP32 Master mediante pulsos GPIO.
//
// COMPONENTES PRINCIPALES:
// 1. Sensor BMI160 (acelerómetro + giroscopio de 6 ejes)
// 2. Sincronización por pulsos desde ESP32 Master
// 3. Almacenamiento en Firebase Realtime Database
// 4. Calibración automática del giroscopio
// 5. Calibración manual del acelerómetro
//
// FUNCIONAMIENTO:
// - El ESP32 Master envía pulsos de 100ms por GPIO23
// - Cada pulso alterna entre iniciar/detener la grabación (toggle)
// - Durante la grabación, se capturan datos cada 200ms
// - Los datos se almacenan en buffer y se envían a Firebase por lotes
// - Sistema NTP para timestamps precisos
// =======================================================

#include <Wire.h>                    // Comunicación I2C con el sensor
#include <BMI160Gen.h>               // Librería del sensor BMI160
#include <WiFi.h>                    // Conexión WiFi
#include "time.h"                    // Manejo de tiempo y NTP
#include <Firebase_ESP_Client.h>     // Cliente Firebase para ESP32
#include <vector>                    // Contenedor dinámico para el buffer

// =======================================================
// === CONFIGURACIÓN DE PINES ===
// =======================================================
#define TRIGGER_PIN 23  // GPIO23 - Recibe pulsos de sincronización del Master

// =======================================================
// === CONFIGURACIÓN WIFI ===
// =======================================================
const char* ssid = "Redmi Note 13";      // Nombre de la red WiFi
const char* password = "Jimena2004";     // Contraseña de la red WiFi

// =======================================================
// === CONFIGURACIÓN FIREBASE ===
// =======================================================
FirebaseData fbdo;        // Objeto para operaciones con Firebase
FirebaseAuth auth;        // Objeto de autenticación (no usado en este caso)
FirebaseConfig config;    // Configuración de Firebase
String sessionPath = "";  // Ruta de la sesión actual en Firebase

// ESTRUCTURA EN FIREBASE:
// /imu_data/
//   └── session_YYYYMMDD_HHMMSS/
//       ├── batch_0/
//       │   ├── 0: "timestamp,ax,ay,az,gx,gy,gz"
//       │   ├── 1: "timestamp,ax,ay,az,gx,gy,gz"
//       │   └── ...
//       ├── batch_1/
//       └── ...

// =======================================================
// === CONFIGURACIÓN NTP (SINCRONIZACIÓN DE TIEMPO) ===
// =======================================================
const char* ntpServer = "time.google.com";  // Servidor NTP de Google
const long gmtOffset_sec = -18000;          // GMT-5 (Colombia: -5 horas × 3600 seg/hora)
const int daylightOffset_sec = 0;           // Sin horario de verano

// =======================================================
// === CONFIGURACIÓN DE MUESTREO ===
// =======================================================
const unsigned long SAMPLE_INTERVAL = 200;  // Intervalo entre muestras: 200ms (5 Hz)
const int BUFFER_SIZE = 50;                 // Tamaño del buffer antes de enviar a Firebase

// POR QUÉ 200ms:
// - Balance entre resolución temporal y cantidad de datos
// - 5 muestras por segundo es suficiente para movimientos humanos
// - Reduce la carga en Firebase y el consumo de datos

// POR QUÉ 50 MUESTRAS:
// - 50 muestras × 200ms = 10 segundos de datos por lote
// - Optimiza el uso de memoria y red
// - Reduce el número de escrituras a Firebase (ahorro de costos)

// =======================================================
// === VARIABLES DE ESTADO DEL SISTEMA ===
// =======================================================
volatile bool isRecording = false;       // ¿Está grabando actualmente?
unsigned long lastSample = 0;            // Timestamp de la última muestra capturada
unsigned long recordingStartTime = 0;    // Timestamp del inicio de la grabación
int totalSampleCount = 0;                // Contador total de muestras en la sesión
int batchNumber = 0;                     // Número del lote actual (para Firebase)

// =======================================================
// === VARIABLES DE SINCRONIZACIÓN ===
// =======================================================
// Estas variables se usan en la ISR para detectar pulsos válidos del Master
volatile bool pulso_sync_detectado = false;      // Flag: se detectó un pulso válido
volatile unsigned long tiempo_inicio_pulso = 0;  // Timestamp del flanco de subida

// =======================================================
// === BUFFER DE DATOS IMU ===
// =======================================================
std::vector<String> imuBuffer;  // Buffer dinámico para almacenar lecturas

// FORMATO DE CADA ENTRADA:
// "YYYY-MM-DD HH:MM:SS.mmm,ax,ay,az,gx,gy,gz"
// Ejemplo: "2024-01-15 10:30:45.123,100,-50,1000,10,-5,2"

// =======================================================
// === CALIBRACIÓN DEL ACELERÓMETRO (MANUAL) ===
// =======================================================
// Estos valores fueron obtenidos previamente mediante un proceso de calibración
// donde se midió el sensor en 6 posiciones estáticas (cada eje ±1g)

// OFFSETS: Corrección de bias (desviación cuando debería estar en cero)
float accelOffsetX = 422.50;   // Offset en X (en unidades ADC del sensor)
float accelOffsetY = -728.00;  // Offset en Y
float accelOffsetZ = 94.50;    // Offset en Z

// SCALES: Corrección de ganancia (cuando debería medir 1g)
float accelScaleX = 1.0030;    // Factor de escala en X
float accelScaleY = 1.0058;    // Factor de escala en Y
float accelScaleZ = 1.0046;    // Factor de escala en Z

// APLICACIÓN:
// accel_calibrado = (accel_raw - offset) × scale

// =======================================================
// === CALIBRACIÓN DEL GIROSCOPIO (AUTOMÁTICA) ===
// =======================================================
// El giroscopio se calibra automáticamente en el setup()
// midiendo el bias cuando el sensor está en reposo

float gyroOffsetX = 0.0;  // Offset en X (se calcula en setup)
float gyroOffsetY = 0.0;  // Offset en Y (se calcula en setup)
float gyroOffsetZ = 0.0;  // Offset en Z (se calcula en setup)

// POR QUÉ AUTOMÁTICA:
// - El giroscopio solo necesita corrección de bias (no de escala)
// - El bias puede variar con temperatura, por eso se calibra al inicio
// - Más simple y confiable que calibración manual

// =======================================================
// === ISR: DETECCIÓN DE PULSO DE SINCRONIZACIÓN ===
// =======================================================
// Esta función se ejecuta en cada cambio de estado del pin TRIGGER_PIN
// Detecta pulsos de aproximadamente 100ms del ESP32 Master

void IRAM_ATTR isrSyncPulso() {
    static unsigned long tiempo_ultimo_pulso = 0;
    bool estado_pin = digitalRead(TRIGGER_PIN);
    
    if (estado_pin == HIGH) {
        // ===== FLANCO DE SUBIDA (inicio del pulso) =====
        tiempo_inicio_pulso = millis();
    } 
    else if (estado_pin == LOW && tiempo_inicio_pulso > 0) {
        // ===== FLANCO DE BAJADA (fin del pulso) =====
        
        // Calcular duración del pulso
        unsigned long duracion_pulso = millis() - tiempo_inicio_pulso;
        
        // VALIDACIÓN DEL PULSO:
        // 1. Duración entre 70-130ms (pulso nominal: 100ms ± 30ms de tolerancia)
        // 2. Al menos 200ms desde el último pulso válido (anti-rebote)
        
        if (duracion_pulso >= 70 && duracion_pulso <= 130) {
            if (millis() - tiempo_ultimo_pulso > 200) {
                // Pulso válido detectado
                pulso_sync_detectado = true;
                tiempo_ultimo_pulso = millis();
            }
        }
        tiempo_inicio_pulso = 0;  // Resetear para siguiente pulso
    }
}

// =======================================================
// === CALIBRACIÓN AUTOMÁTICA DEL GIROSCOPIO ===
// =======================================================
// Toma 200 muestras del giroscopio en reposo y calcula el promedio
// Este promedio es el bias que se restará de todas las lecturas futuras

void calibrarGiroscopio() {
    Serial.println("Iniciando calibración del giroscopio...");
    Serial.println("IMPORTANTE: Mantener el sensor completamente inmóvil");
    
    delay(1000);  // Espera inicial para estabilización
    
    const int numSamples = 200;  // 200 muestras
    long sumGx = 0, sumGy = 0, sumGz = 0;
    
    // Acumular muestras
    for (int i = 0; i < numSamples; i++) {
        int ax, ay, az, gx, gy, gz;
        BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);
        
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        
        delay(10);  // 200 muestras × 10ms = 2 segundos total
    }
    
    // Calcular promedios (estos son los offsets)
    gyroOffsetX = sumGx / (float)numSamples;
    gyroOffsetY = sumGy / (float)numSamples;
    gyroOffsetZ = sumGz / (float)numSamples;
    
    Serial.println("Calibración completada:");
    Serial.print("Gyro Offset X: "); Serial.println(gyroOffsetX);
    Serial.print("Gyro Offset Y: "); Serial.println(gyroOffsetY);
    Serial.print("Gyro Offset Z: "); Serial.println(gyroOffsetZ);
}

// =======================================================
// === APLICAR CALIBRACIÓN A LOS DATOS ===
// =======================================================
// Toma los datos crudos del sensor y aplica las correcciones de calibración
// Los parámetros se pasan por referencia y se modifican directamente

void aplicarCalibracion(int &ax, int &ay, int &az, int &gx, int &gy, int &gz) {
    // ===== CALIBRAR ACELERÓMETRO =====
    // Fórmula: valor_calibrado = (valor_raw - offset) × scale
    float accelX = (ax - accelOffsetX) * accelScaleX;
    float accelY = (ay - accelOffsetY) * accelScaleY;
    float accelZ = (az - accelOffsetZ) * accelScaleZ;
    
    // ===== CALIBRAR GIROSCOPIO =====
    // Fórmula: valor_calibrado = valor_raw - offset
    // (No necesita factor de escala porque la ganancia es inherentemente precisa)
    float gyroX = gx - gyroOffsetX;
    float gyroY = gy - gyroOffsetY;
    float gyroZ = gz - gyroOffsetZ;
    
    // ===== CONVERTIR DE VUELTA A ENTEROS =====
    // Firebase almacenará estos valores como strings, pero mantener como int
    // reduce el tamaño de datos y es suficiente precisión
    ax = (int)accelX;
    ay = (int)accelY;
    az = (int)accelZ;
    gx = (int)gyroX;
    gy = (int)gyroY;
    gz = (int)gyroZ;
}

// =======================================================
// === OBTENER TIMESTAMP ACTUAL ===
// =======================================================
// Genera un timestamp con formato: "YYYY-MM-DD HH:MM:SS.mmm"
// Usa NTP para tiempo real + millis() para precisión de milisegundos

String getTimestamp() {
    struct tm timeinfo;
    
    // Intentar obtener tiempo del servidor NTP
    if (!getLocalTime(&timeinfo)) {
        // Si falla, usar millis() como fallback
        Serial.println("Error obteniendo tiempo NTP, usando millis()");
        return String(millis());
    }

    // Formatear fecha y hora
    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &timeinfo);
    
    // Agregar milisegundos (los 3 dígitos menos significativos de millis)
    unsigned long ms = millis() % 1000;

    // Combinar todo en el formato final
    char timestamp[30];
    snprintf(timestamp, sizeof(timestamp), "%s.%03lu", buffer, ms);
    
    return String(timestamp);
}

// =======================================================
// === CREAR IDENTIFICADOR DE SESIÓN ===
// =======================================================
// Genera un ID único basado en la fecha y hora actual
// Formato: "session_YYYYMMDD_HHMMSS"
// Ejemplo: "session_20240115_103045"

String crearSesion() {
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    
    char sessionID[50];
    strftime(sessionID, sizeof(sessionID), "session_%Y%m%d_%H%M%S", &timeinfo);
    
    return String(sessionID);
}

// =======================================================
// === ENVIAR BUFFER A FIREBASE ===
// =======================================================
// Envía el contenido completo del buffer a Firebase Realtime Database
// Incluye reintentos automáticos y reconexión WiFi si es necesario

bool enviarBufferInmediato() {
    // Si el buffer está vacío, no hay nada que enviar
    if (imuBuffer.size() == 0) return true;

    // ===== VERIFICAR CONEXIÓN WIFI =====
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi desconectado, intentando reconectar...");
        WiFi.reconnect();
        
        // Esperar hasta 10 segundos (20 intentos × 500ms)
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }
        
        // Si no se pudo reconectar, abortar
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\nError: No se pudo reconectar a WiFi");
            return false;
        }
        Serial.println("\nWiFi reconectado");
    }

    // ===== PREPARAR DATOS JSON =====
    FirebaseJson json;
    
    // Agregar cada entrada del buffer al JSON
    // Formato: {"0": "timestamp,ax,ay,az,gx,gy,gz", "1": "...", ...}
    for (int i = 0; i < imuBuffer.size(); i++) {
        json.set(String(i), imuBuffer[i]);
    }

    // ===== CONSTRUIR RUTA EN FIREBASE =====
    // Ejemplo: "/imu_data/session_20240115_103045/batch_0"
    String path = sessionPath + "/batch_" + String(batchNumber);
    
    // ===== INTENTAR ENVIAR CON REINTENTOS =====
    for (int intento = 1; intento <= 3; intento++) {
        // Esperar entre reintentos (excepto el primero)
        if (intento > 1) {
            Serial.print("Reintento ");
            Serial.print(intento);
            Serial.println("/3");
            delay(2000);
        }
        
        // Intentar escribir en Firebase
        if (Firebase.RTDB.setJSON(&fbdo, path.c_str(), &json)) {
            // ===== ÉXITO =====
            Serial.print("Lote ");
            Serial.print(batchNumber);
            Serial.print(" enviado (");
            Serial.print(imuBuffer.size());
            Serial.println(" muestras)");
            
            batchNumber++;      // Incrementar número de lote
            imuBuffer.clear();  // Limpiar buffer
            return true;
        } else {
            // ===== ERROR =====
            Serial.print("Error en intento ");
            Serial.print(intento);
            Serial.print(": ");
            Serial.println(fbdo.errorReason());
        }
    }
    
    // Después de 3 intentos fallidos, limpiar buffer para evitar pérdida de memoria
    Serial.println("ADVERTENCIA: Lote descartado después de 3 intentos fallidos");
    imuBuffer.clear();
    return false;
}

// =======================================================
// === INICIAR GRABACIÓN ===
// =======================================================
// Se ejecuta cuando se recibe un pulso del Master estando en reposo

void iniciarGrabacion() {
    Serial.println("=== INICIANDO GRABACIÓN ===");
    
    isRecording = true;
    
    // Crear nueva sesión en Firebase con timestamp actual
    sessionPath = "/imu_data/" + crearSesion();
    Serial.print("Sesión creada: ");
    Serial.println(sessionPath);
    
    // Resetear todas las variables de estado
    imuBuffer.clear();
    recordingStartTime = millis();
    totalSampleCount = 0;
    batchNumber = 0;
    lastSample = millis();
    
    Serial.println("Sistema listo para capturar datos");
}

// =======================================================
// === DETENER GRABACIÓN ===
// =======================================================
// Se ejecuta cuando se recibe un pulso del Master estando grabando

void detenerGrabacion() {
    Serial.println("=== DETENIENDO GRABACIÓN ===");
    
    // Enviar cualquier dato que quede en el buffer
    if (imuBuffer.size() > 0) {
        Serial.print("Enviando últimas ");
        Serial.print(imuBuffer.size());
        Serial.println(" muestras...");
        enviarBufferInmediato();
    }
    
    isRecording = false;
    
    // Mostrar estadísticas de la sesión
    unsigned long duracion = millis() - recordingStartTime;
    Serial.print("Duración total: ");
    Serial.print(duracion / 1000.0);
    Serial.println(" segundos");
    Serial.print("Total de muestras: ");
    Serial.println(totalSampleCount);
    Serial.print("Lotes enviados: ");
    Serial.println(batchNumber);
}

// =======================================================
// === SETUP INICIAL ===
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("\n=== SISTEMA IMU CON SINCRONIZACIÓN ===");

    // ===== CONECTAR A WIFI =====
    Serial.print("Conectando a WiFi: ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    // Si no se conectó después de 20 segundos, detener
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nERROR: No se pudo conectar a WiFi");
        Serial.println("Sistema detenido. Reiniciar ESP32.");
        while(1);  // Bucle infinito (requiere reinicio manual)
    }
    
    Serial.println("\nWiFi conectado");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    // ===== CONFIGURAR NTP (SINCRONIZACIÓN DE TIEMPO) =====
    Serial.println("Sincronizando reloj con servidor NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(2000);  // Esperar sincronización inicial
    
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        Serial.println("Reloj sincronizado correctamente");
        Serial.print("Hora actual: ");
        Serial.println(getTimestamp());
    } else {
        Serial.println("ADVERTENCIA: No se pudo sincronizar el reloj");
    }

    // ===== CONFIGURAR FIREBASE =====
    Serial.println("Configurando Firebase...");
    config.database_url = "mapicog-default-rtdb.firebaseio.com";
    config.signer.tokens.legacy_token = "QQsdJ9zQwDLaWzDuQ7edmisbmhARXnIaG25dZZqO";
    
    // Timeouts extendidos para conexión estable
    config.timeout.serverResponse = 30 * 1000;      // 30 segundos
    config.timeout.socketConnection = 30 * 1000;    // 30 segundos
    config.timeout.sslHandshake = 30 * 1000;        // 30 segundos

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);  // Reconectar automáticamente si se pierde WiFi
    
    // Optimizar buffers SSL para mejor rendimiento
    fbdo.setBSSLBufferSize(4096, 1024);
    
    Serial.println("Firebase configurado");

    // ===== CONFIGURAR SENSOR IMU =====
    Serial.println("Inicializando sensor BMI160...");
    Wire.begin(21, 22);  // SDA=GPIO21, SCL=GPIO22
    
    // Configurar pin de sincronización
    pinMode(TRIGGER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), isrSyncPulso, CHANGE);
    Serial.println("Pin de sincronización configurado (GPIO23)");

    // Inicializar BMI160 en modo I2C, dirección 0x68
    if (!BMI160.begin(BMI160GenClass::I2C_MODE, 0x68)) {
        Serial.println("ERROR: No se pudo inicializar el sensor BMI160");
        Serial.println("Verificar conexiones I2C y dirección del sensor");
        while (1);  // Detener si no hay sensor
    }
    
    Serial.println("Sensor BMI160 inicializado correctamente");
    
    // ===== CALIBRACIÓN AUTOMÁTICA DEL GIROSCOPIO =====
    calibrarGiroscopio();
    
    // ===== RESERVAR MEMORIA PARA EL BUFFER =====
    // Reservar un poco más que BUFFER_SIZE para evitar reasignaciones frecuentes
    imuBuffer.reserve(BUFFER_SIZE + 10);
    
    Serial.println("\n=== SISTEMA LISTO ===");
    Serial.println("Esperando pulso de sincronización del Master...");
    Serial.println("(GPIO23 debe recibir pulsos de ~100ms)");
}

// =======================================================
// === LOOP PRINCIPAL ===
// =======================================================
void loop() {
    // ===== PARTE 1: DETECCIÓN DE PULSO DE SINCRONIZACIÓN =====
    // Este pulso viene del ESP32 Master y actúa como toggle de grabación
    
    if (pulso_sync_detectado) {
        pulso_sync_detectado = false;  // Limpiar flag
        
        if (!isRecording) {
            // ===== INICIAR NUEVA GRABACIÓN =====
            iniciarGrabacion();
        } else {
            // ===== DETENER GRABACIÓN ACTUAL =====
            detenerGrabacion();
        }
    }

    // ===== PARTE 2: CAPTURA Y ENVÍO DE DATOS IMU =====
    // Solo ejecutar si estamos grabando y ha pasado el intervalo de muestreo
    
    if (isRecording && millis() - lastSample >= SAMPLE_INTERVAL) {
        lastSample = millis();  // Actualizar timestamp de última muestra

        // ===== VERIFICAR MEMORIA DISPONIBLE =====
        // Si queda poca memoria, enviar el buffer inmediatamente
        if (ESP.getFreeHeap() < 30000) {  // Menos de 30KB libres
            Serial.println("ADVERTENCIA: Memoria baja, enviando buffer...");
            enviarBufferInmediato();
        }

        // ===== LEER SENSOR IMU =====
        int ax, ay, az, gx, gy, gz;
        BMI160.readMotionSensor(ax, ay, az, gx, gy, gz);
        
        // ax, ay, az: Aceleración en X, Y, Z (en unidades ADC del sensor)
        // gx, gy, gz: Velocidad angular en X, Y, Z (en unidades ADC del sensor)
        
        // ===== APLICAR CALIBRACIÓN =====
        aplicarCalibracion(ax, ay, az, gx, gy, gz);

        // ===== FORMATEAR DATOS =====
        // Formato: "timestamp,ax,ay,az,gx,gy,gz"
        String entry = getTimestamp() + "," +
                       String(ax) + "," + String(ay) + "," + String(az) + "," +
                       String(gx) + "," + String(gy) + "," + String(gz);

        // ===== AGREGAR AL BUFFER =====
        imuBuffer.push_back(entry);
        totalSampleCount++;

        // Debug cada 10 muestras
        if (totalSampleCount % 10 == 0) {
            Serial.print("Muestras capturadas: ");
            Serial.print(totalSampleCount);
            Serial.print(" | Buffer: ");
            Serial.print(imuBuffer.size());
            Serial.print("/");
            Serial.println(BUFFER_SIZE);
        }

        // ===== ENVIAR BUFFER SI ESTÁ LLENO =====
        if (imuBuffer.size() >= BUFFER_SIZE) {
            enviarBufferInmediato();
        }
    }
}
