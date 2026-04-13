// =======================================================
// === SISTEMA INTEGRADO: SENSOR ULTRASÓNICO + AUDIO HÁPTICO ===
// =======================================================
// Este sistema combina:
// 1. Un sensor ultrasónico basado en ADC para medir distancias
// 2. Generación de retroalimentación háptica mediante audio Bluetooth
// 3. Control de inicio/pausa mediante botón
// 4. Sincronización con un segundo ESP32 (IMU)
//
// FUNCIONAMIENTO GENERAL:
// - Ciclo de carga/descarga del sensor ultrasónico para medir distancia
// - Generación de tonos de audio que varían según la distancia detectada
// - Promedio de múltiples mediciones para mayor precisión
// - Comunicación Bluetooth A2DP para transmitir el audio
// =======================================================

#include <Arduino.h>
#include <driver/i2s.h>           // Para muestreo ADC de alta velocidad vía DMA
#include "driver/adc.h"           // Control del conversor analógico-digital
#include "BluetoothA2DPSource.h"  // Transmisión de audio Bluetooth
#include "AudioTools.h"           // Herramientas de procesamiento de audio
#include <math.h>                 // Funciones matemáticas (sin, exp, etc.)
#include <esp_task_wdt.h>         // Watchdog timer para evitar cuelgues

// =======================================================
// === CONFIGURACIÓN DE PINES Y CONSTANTES ===
// =======================================================

// PIN DE SINCRONIZACIÓN
// Este pin envía un pulso al segundo ESP32 (IMU) para coordinar mediciones
#define SYNC_OUT_PIN 14

// BOTÓN DE INICIO/PAUSA
// Permite iniciar o detener el sistema con un solo botón
#define BOTON_INICIO 2
volatile bool sistema_activo = false;     // Estado actual del sistema
volatile bool boton_presionado = false;   // Flag para manejar la interrupción

// =======================================================
// === BLUETOOTH A2DP (TRANSMISIÓN DE AUDIO) ===
// =======================================================
BluetoothA2DPSource a2dp_source;

// Variables de control del tono de audio generado
volatile bool tone_is_active = false;        // ¿El tono está sonando?
volatile double current_amplitude = 0.0;     // Amplitud actual del tono
const double TONE_MAX_AMPLITUDE = 10000.0;   // Amplitud máxima permitida

// FRECUENCIA VARIABLE
// La frecuencia del tono cambia según la distancia y voltaje detectados
volatile double current_frequency = 450.0;   // Frecuencia inicial en Hz

// CONFIGURACIÓN DE AUDIO
const double SAMPLE_RATE_AUDIO = 44100.0;    // 44.1 kHz (estándar CD)
static double m_time = 0.0;                  // Tiempo transcurrido en la generación

// =======================================================
// === CONTROL DE SECUENCIA CÍCLICA (TONO + SILENCIO) ===
// =======================================================
// El sistema genera un patrón repetitivo:
// TONO (duración fija) → SILENCIO (duración variable según distancia)

const unsigned long FIXED_TONE_DURATION = 600; // Duración del tono: 600ms constante

// Parámetros calculados dinámicamente según la distancia
double amp_ratio = 0.0;              // Ratio de amplitud (0.0 a 1.0)
unsigned long silence_duration = 0;  // Duración del silencio en ms

// Máquina de estados para controlar la secuencia
enum SequenceStep {
    TONE_PHASE,      // Fase de reproducción del tono
    SILENCE_PHASE    // Fase de silencio
};
volatile SequenceStep current_step = TONE_PHASE;
unsigned long phase_start_time = 0;  // Marca de tiempo del inicio de cada fase

// =======================================================
// === CONFIGURACIÓN DEL SENSOR ULTRASÓNICO ===
// =======================================================

// PINES DE CONTROL DEL SENSOR
#define PIN_CARGA      23  // Activa la carga del condensador (HIGH = carga ON)
#define PIN_DESCARGA   16  // Activa la descarga del condensador (LOW = descarga ON)
#define PIN_NORMAL     4   // Salida de pulsos 40kHz fase normal
#define PIN_INVERTED   5   // Salida de pulsos 40kHz fase invertida (180°)
#define FLAG_PIN       27  // Indicador de lectura ADC activa (para debug/análisis)

// EXPLICACIÓN DEL FUNCIONAMIENTO DEL SENSOR:
// 1. FASE DE CARGA (10.2ms):
//    - Se carga un condensador (PIN_CARGA = HIGH)
//    - Se emiten pulsos de 40kHz (12 pulsos) que duran los primeros 300us de la fase de carga y excitan el transducdor piezo electrico de tramisión
//    - Los pulsos rebotan en objetos cercanos
//    - El eco recibido se captura con el transductor piezo electrico de recepción
//
// 2. FASE DE DESCARGA (0.5ms):
//    - Se descarga el condensador (PIN_DESCARGA = LOW)
//    - Preparación para el siguiente ciclo
//
// 3. LECTURA ADC:
//    - Durante la carga, el ADC muestrea continuamente el voltaje
//    - La distancia se calcula por el tiempo de llegada del eco
//    - La intensidad del eco indica la "densidad" del objeto detectado

// =======================================================
// === CONFIGURACIÓN ADC E I2S ===
// =======================================================
// El ADC se usa junto con I2S para muestreo de alta velocidad con DMA
// Esto permite capturar la señal ultrasónica sin sobrecargar el CPU

#define I2S_PORT        I2S_NUM_0        // Puerto I2S a utilizar
#define ADC_CHANNEL     ADC1_CHANNEL_5   // GPIO33 (Canal 5 del ADC1)
#define SAMPLE_RATE     24000            // 24 kHz -> Frecuencia de muestreo
#define BUFFER_LEN      256              // Tamaño del buffer DMA

uint16_t dmaBuffer[BUFFER_LEN];          // Buffer para almacenar muestras ADC

// =======================================================
// === CONFIGURACIÓN DE TIMING PRECISO ===
// =======================================================

#define DURACION_CARGA_US      10200UL   // Duración de la fase de carga: 10.2ms
#define DURACION_DESCARGA_US   500UL     // Duración de la fase de descarga: 0.5ms

// GENERACIÓN DE PULSOS ULTRASÓNICOS A 40kHz
// Frecuencia: 40,000 Hz → Período: 25 µs → Semiperiodo: 12.5 µs
const uint32_t SEMIPERIODO_UNITS = 125;  // Timer en resolución 0.1µs: 12.5µs = 125 unidades
const uint8_t  NUMERO_PULSOS     = 12;   // Número de pulsos a emitir

// UMBRAL DE DETECCIÓN
// Define el nivel mínimo de señal para considerar válida una detección
const float UMBRAL_VOLTIOS = 0.01;  // 10 mV (ajustable según ruido del sistema y calibración del VCA810)
const float UMBRAL_BITS = (UMBRAL_VOLTIOS / 1.1) * 4095.0;  // Conversión a valor ADC

// =======================================================
// === SISTEMA DE PROMEDIADO DE MÚLTIPLES CICLOS ===
// =======================================================
// Para mayor precisión, se promedian 50 ciclos de medición
// Esto reduce el ruido y mejora la estabilidad de las lecturas

#define CICLOS_PROMEDIO 50    // Número de ciclos a promediar
#define CICLOS_MINIMOS 10     // Mínimo de ciclos válidos para considerar el promedio

// Estructura para almacenar los datos de cada ciclo individual
struct DatosCiclo {
    float distancia;     // Distancia medida en centímetros
    float sumaVoltios;   // Suma de voltajes de las 36 muestras del eco
    bool valido;         // ¿Es válida esta medición?
};

DatosCiclo ciclosBuffer[CICLOS_PROMEDIO];  // Buffer circular de ciclos
int indiceCiclo = 0;                       // Índice actual en el buffer

// Estructura para almacenar el último promedio calculado
struct UltimoPromedioValido {
    bool hayPromedioValido = false;  // ¿Hay un promedio válido disponible?
    int ciclosValidos = 0;           // Cantidad de ciclos válidos en el promedio
    float promedioDistancia = 0;     // Distancia promedio en cm
    float promedioSumaVoltios = 0;   // Voltaje promedio
} ultimoPromedio;

// =======================================================
// === VARIABLES GLOBALES DEL SENSOR ===
// =======================================================

hw_timer_t* timerPulsos = NULL;  // Timer hardware para generar pulsos 40kHz

// Variables de control de los pulsos ultrasónicos
volatile uint8_t contadorPulsos = 0;  // Cuenta los pulsos emitidos
volatile bool pulsosActivos     = false;  // ¿Están los pulsos activos?
volatile bool estadoPulso       = false;  // Estado actual del pulso (HIGH/LOW)
volatile bool en_fase_carga     = true;   // ¿Estamos en fase de carga?

// Control de tiempo para sincronización precisa
volatile uint32_t tiempoInicioCarga = 0;  // Marca de tiempo del inicio de carga

// =======================================================
// === FUNCIÓN DE SINCRONIZACIÓN CON ESP32 IMU ===
// =======================================================
// Envía un pulso de 100ms al segundo ESP32 para coordinar mediciones
void enviarPulsoSync() {
    digitalWrite(SYNC_OUT_PIN, HIGH);
    delay(100);  // Pulso de 100ms (fácil de detectar)
    digitalWrite(SYNC_OUT_PIN, LOW);
}

// =======================================================
// === INTERRUPCIÓN DEL BOTÓN DE INICIO/PAUSA ===
// =======================================================
// Maneja la pulsación del botón con anti-rebote por software
void IRAM_ATTR isrBoton() {
    static unsigned long ultimo_tiempo = 0;
    unsigned long tiempo_actual = millis();
    
    // Anti-rebote: ignorar pulsaciones menores a 300ms
    if (tiempo_actual - ultimo_tiempo > 300) {
        boton_presionado = true;
        ultimo_tiempo = tiempo_actual;
    }
}

// =======================================================
// === GENERACIÓN DE AUDIO BLUETOOTH (CALLBACK) ===
// =======================================================
// Esta función es llamada automáticamente por la librería A2DP
// para solicitar los datos de audio a transmitir

int32_t get_sound_data(Frame *data, int32_t len) {
    // Si el sistema no está activo o el tono está detenido, no generar audio
    if (!tone_is_active || !sistema_activo) {
        return 0;
    }
    
    // Generar 'len' frames de audio (cada frame = muestra estéreo)
    for (int i = 0; i < len; i++) {
        // Generar onda sinusoidal: A * sin(2πft)
        int16_t value = (int16_t)(current_amplitude * sin(2 * M_PI * current_frequency * m_time));
        
        // Duplicar la señal mono a ambos canales (estéreo)
        data[i].channel1 = value;
        data[i].channel2 = value;
        
        // Avanzar el tiempo según la tasa de muestreo
        m_time += 1.0 / SAMPLE_RATE_AUDIO;
        
        // Reiniciar el tiempo cada 10 segundos para evitar errores de precisión
        if (m_time > 10.0) { 
            m_time = 0.0;
            play_tone(amp_ratio);  // Recalcular amplitud
        }
    }
    return len;
}

// =======================================================
// === MODELO MATEMÁTICO: AMPLITUD vs DISTANCIA ===
// =======================================================
// La amplitud del tono disminuye con la distancia
// Usa dos ecuaciones diferentes según el rango de distancia

double calcular_amplitud(double dist) {
    // Rango cercano (10-70 cm): función lineal
    if (dist >= 10.0 && dist <= 70.0) {
        return (-dist * (7.0 / 1300.0)) + (131.0 / 130.0);
    } 
    // Rango lejano (71-180 cm): función exponencial
    else if (dist >= 71.0 && dist <= 180.0) {
        return 1.2 * exp(dist * -0.025) + 0.17;
    }
    return 0.0;  // Fuera de rango
}

// =======================================================
// === MODELO MATEMÁTICO: DURACIÓN DEL SILENCIO vs DISTANCIA ===
// =======================================================
// El silencio entre tonos aumenta con la distancia
// Objetos cercanos → tonos más frecuentes
// Objetos lejanos → tonos más espaciados

unsigned long calcular_duracion_silencio(double dist) {
    unsigned long duration = (unsigned long)(-900.0 * exp(dist * -0.025) + 750.0);
    return (duration > 50) ? duration : 50;  // Mínimo 50ms
}

// =======================================================
// === MODELO MATEMÁTICO: FRECUENCIA ADAPTATIVA ===
// =======================================================
// La frecuencia del tono varía según:
// La intensidad del eco recibido (voltaje)

// Voltaje mínimo esperado para cada distancia
double Vmin_dist(double d) {
    return 0.003 * exp(0.039 * d);
}

// Voltaje máximo esperado para cada distancia
double Vmax_dist(double d) {
    return 0.68 * exp(0.011 * d);
}

// Cálculo de frecuencia mediante interpolación lineal
double calcular_frecuencia(double d, double V) {
    const double FMIN = 450.0;   // Frecuencia mínima: 450 Hz
    const double FMAX = 1000.0;  // Frecuencia máxima: 1000 Hz

    // Fuera del rango de distancia válido
    if (d < 30.0 || d > 150.0) {
        return FMIN;
    }

    double Vmin = Vmin_dist(d);
    double Vmax = Vmax_dist(d);

    // Saturación inferior: voltaje muy bajo
    if (V <= Vmin) {
        return FMIN;
    }

    // Saturación superior: voltaje muy alto
    if (V >= Vmax) {
        return FMAX;
    }

    // Interpolación lineal dentro del rango válido
    // alpha = 0 → frecuencia mínima
    // alpha = 1 → frecuencia máxima
    double alpha = (V - Vmin) / (Vmax - Vmin);
    alpha = constrain(alpha, 0.0, 1.0);

    return FMIN + alpha * (FMAX - FMIN);
}

// =======================================================
// === CONTROL DE REPRODUCCIÓN DEL TONO ===
// =======================================================

// Iniciar la reproducción del tono con una amplitud específica
void play_tone(double amp_ratio) {
    tone_is_active = true;
    current_amplitude = amp_ratio * TONE_MAX_AMPLITUDE; 
}

// Detener la reproducción del tono
void stop_tone() {
    tone_is_active = false;
    current_amplitude = 0.0;
}

// =======================================================
// === PROTOTIPOS DE FUNCIONES ===
// =======================================================
void configurarPines();
void configurarBoton();
void setupTimers();
void setupADC();
void IRAM_ATTR isrPulsos();
void iniciarTrenPulsos();
void iniciarFaseCarga();
void iniciarFaseDescarga();
void calcularPromedioMovilYDetectar(uint16_t* buffer, int numMuestras);
void procesarPromedioY50Ciclos();

// =======================================================
// === ISR: GENERACIÓN DE PULSOS ULTRASÓNICOS A 40kHz ===
// =======================================================
// Esta función se ejecuta cada 12.5 µs para generar la onda cuadrada

void IRAM_ATTR isrPulsos() {
    if (!pulsosActivos) return;

    // Alternar el estado del pulso (HIGH → LOW → HIGH...)
    estadoPulso = !estadoPulso;

    // PIN_NORMAL y PIN_INVERTED están en fase opuesta (180°)
    digitalWrite(PIN_NORMAL,   estadoPulso ? HIGH : LOW);
    digitalWrite(PIN_INVERTED, estadoPulso ? LOW  : HIGH);

    // Contar solo en flanco ascendente para tener un conteo preciso
    if (estadoPulso) {
        contadorPulsos++;
        
        // Si ya emitimos los 12 pulsos, detener
        if (contadorPulsos >= NUMERO_PULSOS) {
            pulsosActivos = false;
            timerAlarmDisable(timerPulsos);

            // Estado de reposo: NORMAL=LOW, INVERTED=HIGH
            digitalWrite(PIN_NORMAL, LOW);
            digitalWrite(PIN_INVERTED, HIGH);
        }
    }
}

// =======================================================
// === INICIAR FASE DE CARGA DEL SENSOR ===
// =======================================================
void iniciarFaseCarga() {
    en_fase_carga = true;
    tiempoInicioCarga = micros();  // Marcar tiempo de inicio

    // Activar pin de carga (condensador se carga)
    pinMode(PIN_CARGA, OUTPUT);
    digitalWrite(PIN_CARGA, HIGH);

    // Desconectar descarga (alta impedancia)
    pinMode(PIN_DESCARGA, INPUT);

    // Iniciar emisión de pulsos ultrasónicos
    iniciarTrenPulsos();
}

// =======================================================
// === INICIAR FASE DE DESCARGA DEL SENSOR ===
// =======================================================
void iniciarFaseDescarga() {
    en_fase_carga = false;

    // Activar pin de descarga (condensador se descarga)
    pinMode(PIN_DESCARGA, OUTPUT);
    digitalWrite(PIN_DESCARGA, LOW);

    // Pin de carga también en LOW durante descarga
    pinMode(PIN_CARGA, OUTPUT);
    digitalWrite(PIN_CARGA, LOW);

    // Detener emisión de pulsos
    pulsosActivos = false;
    timerAlarmDisable(timerPulsos);
    digitalWrite(PIN_NORMAL, LOW);
    digitalWrite(PIN_INVERTED, HIGH);
}

// =======================================================
// === INICIAR TREN DE PULSOS ULTRASÓNICOS ===
// =======================================================
void iniciarTrenPulsos() {
    contadorPulsos = 0;
    estadoPulso    = false;
    pulsosActivos  = true;

    // Estado inicial de los pines
    digitalWrite(PIN_NORMAL, LOW);
    digitalWrite(PIN_INVERTED, HIGH);

    // Habilitar la alarma del timer (inicia la ISR)
    timerAlarmEnable(timerPulsos);
}

// =======================================================
// === ALGORITMO DE DETECCIÓN: PROMEDIO MÓVIL ===
// =======================================================
// Este algoritmo busca el primer punto donde la señal supera el umbral
// usando ventanas móviles de 4 muestras para reducir ruido

void calcularPromedioMovilYDetectar(uint16_t* buffer, int numMuestras) {
    
    // Mínimo de muestras necesarias para procesar
    if (numMuestras < 83) return;

    int muestrasAProcesar = min(numMuestras, 240);
    
    float promedioAnterior = 0;
    bool primerPromedio = true;
    bool umbralDetectado = false;
    
    // Variables para almacenar el resultado
    static uint32_t tiempoDeteccion = 0;
    static float distancia = 0;
    static float sumaTotal = 0;
    static int cantidadSumada = 0;
    
    // Comenzar desde la muestra 80 (ignorar transitorio inicial)
    for (int i = 80; i < muestrasAProcesar && !umbralDetectado; ) {
        if (primerPromedio) {
            // Primera ventana: promediar muestras 80-83
            float suma = 0;
            for (int j = 0; j < 4; j++) {
                suma += (buffer[i + j] & 0x0FFF);  // Máscara de 12 bits
            }
            promedioAnterior = suma * 0.25;  // Dividir por 4
            
            // Verificar si supera el umbral
            if (promedioAnterior > UMBRAL_BITS) {  
                umbralDetectado = true;
                
                // Calcular distancia:
                // tiempo (µs) = índice_muestra / sample_rate
                // distancia (cm) = tiempo * velocidad_sonido / 2
                //                = tiempo * 343 m/s / 2
                //                = tiempo * 0.01715 cm/µs
                tiempoDeteccion = (i * 1000000UL) / SAMPLE_RATE;
                distancia = tiempoDeteccion * 0.01715; 

                // Sumar las siguientes 36 muestras para calcular intensidad del eco
                sumaTotal = 0;
                int muestraFinal = min(i + 36, muestrasAProcesar);
                for (int k = i; k < muestraFinal; k++) {
                    sumaTotal += (buffer[k] & 0x0FFF);
                }
                cantidadSumada = muestraFinal - i;
            }
            primerPromedio = false;
            i += 4;  // Avanzar 4 muestras
        }
        else {
            if (i + 3 > muestrasAProcesar) break;
            
            // Ventanas siguientes: reutilizar promedio anterior + 3 nuevas
            // Esto es más eficiente que recalcular las 4 muestras
            float suma = promedioAnterior;
            for (int j = 0; j < 3; j++) {
                suma += (buffer[i + j] & 0x0FFF);
            }
            promedioAnterior = suma * 0.25;  // Dividir por 4
            
            // Verificar umbral
            if (promedioAnterior > UMBRAL_BITS) {
                umbralDetectado = true;
                
                tiempoDeteccion = (i * 1000000UL) / SAMPLE_RATE;
                distancia = tiempoDeteccion * 0.0171500;
                
                sumaTotal = 0;
                int muestraFinal = min(i + 36, numMuestras);
                for (int k = i; k < muestraFinal; k++) {
                    sumaTotal += (buffer[k] & 0x0FFF);
                }
                cantidadSumada = muestraFinal - i;
            }
            i += 3;  // Avanzar 3 muestras
        }
    }
    
    // Guardar el resultado en el buffer de ciclos
    if (umbralDetectado) {
        // Convertir suma ADC a voltios
        // ADC_ATTEN_DB_0: rango 0-1.1V, 12 bits: 0-4095
        float voltiosPorCuenta = 1.1 / 4095.0;
        float sumaVoltios = sumaTotal * voltiosPorCuenta;
        
        // Almacenar en el buffer circular
        ciclosBuffer[indiceCiclo].distancia = distancia;
        ciclosBuffer[indiceCiclo].sumaVoltios = sumaVoltios;
        ciclosBuffer[indiceCiclo].valido = true;
    } else {
        // Marcar como no válido (sin detección)
        ciclosBuffer[indiceCiclo].valido = false;
    }
    
    // Avanzar al siguiente slot del buffer
    indiceCiclo++;
    
    // Si completamos 50 ciclos, calcular promedio
    if (indiceCiclo >= CICLOS_PROMEDIO) {
        procesarPromedioY50Ciclos();
        indiceCiclo = 0;  // Reiniciar buffer circular
    }
}

// =======================================================
// === CALCULAR PROMEDIO DE LOS ÚLTIMOS 50 CICLOS ===
// =======================================================
void procesarPromedioY50Ciclos() {
    float sumaDistancia = 0;
    float sumaSumaVoltios = 0;
    int ciclosValidos = 0;
    
    // Sumar todos los ciclos marcados como válidos
    for (int i = 0; i < CICLOS_PROMEDIO; i++) {
        if (ciclosBuffer[i].valido) {
            sumaDistancia += ciclosBuffer[i].distancia;
            sumaSumaVoltios += ciclosBuffer[i].sumaVoltios;
            ciclosValidos++;
        }
    }
    
    // Solo actualizar si hay suficientes ciclos válidos
    if (ciclosValidos >= CICLOS_MINIMOS) {
        float promedioDistancia = sumaDistancia / ciclosValidos;  
        float promedioSumaVoltios = sumaSumaVoltios / ciclosValidos;
        
        // Guardar como último promedio válido disponible
        ultimoPromedio.hayPromedioValido = true;
        ultimoPromedio.ciclosValidos = ciclosValidos;
        ultimoPromedio.promedioDistancia = promedioDistancia;
        ultimoPromedio.promedioSumaVoltios = promedioSumaVoltios;
    }
}

// =======================================================
// === CONFIGURACIÓN DEL BOTÓN ===
// =======================================================
void configurarBoton() {
    pinMode(BOTON_INICIO, INPUT_PULLUP);  // Resistencia pull-up interna
    attachInterrupt(digitalPinToInterrupt(BOTON_INICIO), isrBoton, FALLING);
}

// =======================================================
// === CONFIGURACIÓN DE PINES DEL SENSOR ===
// =======================================================
void configurarPines() {
    // Pines de pulsos ultrasónicos
    pinMode(PIN_NORMAL, OUTPUT);
    pinMode(PIN_INVERTED, OUTPUT);
    digitalWrite(PIN_NORMAL, LOW);
    digitalWrite(PIN_INVERTED, HIGH);

    // Pin de depuración (flag de lectura ADC)
    pinMode(FLAG_PIN, OUTPUT);
    digitalWrite(FLAG_PIN, LOW);

    // Estado inicial: sistema en espera
    pinMode(PIN_CARGA, OUTPUT);
    pinMode(PIN_DESCARGA, INPUT);  // Alta impedancia
    digitalWrite(PIN_CARGA, LOW);
}

// =======================================================
// === CONFIGURACIÓN DE TIMER HARDWARE ===
// =======================================================
void setupTimers() {
    // Timer 0, prescaler 8 (80MHz/8 = 10MHz → 0.1µs resolución)
    timerPulsos = timerBegin(0, 8, true);
    timerAttachInterrupt(timerPulsos, &isrPulsos, true);
    timerAlarmWrite(timerPulsos, SEMIPERIODO_UNITS, true);  // Auto-reload
}

// =======================================================
// === CONFIGURACIÓN ADC E I2S ===
// =======================================================
void setupADC() {
    // Configurar ADC: 12 bits, sin atenuación (0-1.1V)
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0);

    // Configurar I2S para muestreo ADC por DMA
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER |
                             I2S_MODE_RX |
                             I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = BUFFER_LEN,
        .use_apll = false
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
    i2s_adc_enable(I2S_PORT);
}

// =======================================================
// === SETUP INICIAL ===
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    // 1. Configurar botón (debe ser primero para control de inicio)
    configurarBoton();

    // 2. Configurar pin de sincronización
    pinMode(SYNC_OUT_PIN, OUTPUT);
    digitalWrite(SYNC_OUT_PIN, LOW);
    
    // 3. Configurar sensor ultrasónico
    configurarPines();
    setupTimers();
    setupADC();
    
    // 4. Inicializar buffer de promediado
    for (int i = 0; i < CICLOS_PROMEDIO; i++) {
        ciclosBuffer[i].valido = false;
    }
    indiceCiclo = 0;

    // 5. Inicializar Bluetooth A2DP
    // Nombre visible: "Kalley K-ABC"
    a2dp_source.start("Kalley K-ABC", get_sound_data);

    // 6. Configurar watchdog (evita cuelgues del sistema)
    esp_task_wdt_init(5, true);   // Timeout: 5 segundos
    esp_task_wdt_add(NULL);       // Monitorear el loop principal
}

// =======================================================
// === LOOP PRINCIPAL ===
// =======================================================
void loop() {
    // =======================================================
    // === PARTE 1: CONTROL DEL BOTÓN (INICIO/PAUSA) ===
    // =======================================================
    if (boton_presionado) {
        boton_presionado = false;
        sistema_activo = !sistema_activo;  // Toggle del estado
        
        if (sistema_activo) {
            // SISTEMA INICIADO
            
            // Enviar señal de sincronización al ESP32 IMU
            enviarPulsoSync();
            
            // Iniciar en fase de carga del sensor
            iniciarFaseCarga();
            
            // Iniciar en fase de tono del audio
            current_step = TONE_PHASE;
            phase_start_time = millis();
            tone_is_active = false;
        } else {
            // SISTEMA PAUSADO
            
            // Enviar señal de pausa al ESP32 IMU
            enviarPulsoSync();
            
            // Detener audio
            stop_tone();
            
            // Poner todos los pines en estado seguro
            digitalWrite(PIN_CARGA, LOW);
            pinMode(PIN_DESCARGA, INPUT);
            digitalWrite(PIN_NORMAL, LOW);
            digitalWrite(PIN_INVERTED, HIGH);
            digitalWrite(FLAG_PIN, LOW);
            
            // Detener timer de pulsos
            pulsosActivos = false;
            timerAlarmDisable(timerPulsos);
        }
    }

    // Si el sistema está pausado, no hacer nada más
    if (!sistema_activo) {
        delay(100);
        return;
    }

    // =======================================================
    // === PARTE 2: LECTURA DEL SENSOR ULTRASÓNICO ===
    // =======================================================
    size_t bytesRead = 0;

    // Señal de depuración: inicio de lectura ADC
    digitalWrite(FLAG_PIN, HIGH);
    
    // Asegurar que estamos en fase de carga
    if (!en_fase_carga) {
        iniciarFaseCarga();
    }

    // Leer buffer ADC mediante I2S DMA
    while (bytesRead < sizeof(dmaBuffer)) {
        size_t bytes = 0;
        i2s_read(I2S_PORT,
                 (uint8_t*)dmaBuffer + bytesRead,
                 sizeof(dmaBuffer) - bytesRead,
                 &bytes,
                 0);  // No bloquear (polling)
        
        bytesRead += bytes;
        
        // Verificar si han transcurrido 10.2ms de carga
        uint32_t tiempoTranscurrido = micros() - tiempoInicioCarga;
        if (tiempoTranscurrido >= DURACION_CARGA_US && en_fase_carga) {
            iniciarFaseDescarga();  // Cambiar a descarga
        }
        
        // Dar tiempo al sistema si no hay datos disponibles
        if (bytes == 0) {
            delayMicroseconds(10);
            esp_task_wdt_reset();  // Resetear watchdog
            yield();               // Permitir otras tareas
        }
    }
    esp_task_wdt_reset();

    // Señal de depuración: fin de lectura ADC
    digitalWrite(FLAG_PIN, LOW);

    // Calcular número de muestras leídas
    int samples = bytesRead / 2;
    
    // Procesar el buffer: buscar eco y calcular distancia
    calcularPromedioMovilYDetectar(dmaBuffer, samples);

    // Asegurar tiempo mínimo de descarga (0.5ms)
    if (!en_fase_carga) {
        uint32_t tiempoEnDescarga = micros() - tiempoInicioCarga - DURACION_CARGA_US;
        if (tiempoEnDescarga < DURACION_DESCARGA_US) {
            delayMicroseconds(DURACION_DESCARGA_US - tiempoEnDescarga);
        }
    }

    // =======================================================
    // === PARTE 3: CONTROL DEL AUDIO HÁPTICO ===
    // =======================================================
    
    // Verificar conexión Bluetooth
    if (!a2dp_source.is_connected()) {
        return; // Esperar hasta que haya conexión
    }

    bool phase_finished = false;

    // Verificar si la fase actual ha terminado
    if (current_step == TONE_PHASE) {
        // Fase de tono: duración fija de 600ms
        if (millis() - phase_start_time >= FIXED_TONE_DURATION) {
            stop_tone();
            phase_finished = true;
        }
    } else if (current_step == SILENCE_PHASE) {
        // Fase de silencio: duración variable según distancia
        if (millis() - phase_start_time >= silence_duration) {
            phase_finished = true;
        }
    }

    // Transición entre fases
    if (phase_finished) {
        if (current_step == TONE_PHASE) {
            stop_tone();  // Seguridad extra
            current_step = SILENCE_PHASE;
        } else {
            current_step = TONE_PHASE;
        }
        phase_start_time = millis();  // Reiniciar contador de fase
    }

    // Iniciar nueva fase de TONO si corresponde
    if (current_step == TONE_PHASE && !tone_is_active) {
        // Usar el último promedio válido calculado
        if (ultimoPromedio.hayPromedioValido) {
            double dist = ultimoPromedio.promedioDistancia;
            double voltaje = ultimoPromedio.promedioSumaVoltios;
            
            // Calcular parámetros del audio según la distancia y voltaje
            amp_ratio = calcular_amplitud(dist);
            silence_duration = calcular_duracion_silencio(dist);

            // Limitar amplitud al máximo permitido
            if (amp_ratio > 1.0) {
                amp_ratio = 1.0;
            }

            // Calcular frecuencia adaptativa
            current_frequency = calcular_frecuencia(dist, voltaje);

            // Limitar duración mínima del silencio
            if (silence_duration < 50) {
                silence_duration = 50;    
            }
            
            // Iniciar reproducción del tono
            play_tone(amp_ratio);

        } else {
            // No hay promedio válido: usar valores por defecto
            current_frequency = 450.0;
            play_tone(0.3);
        }
    }
    
    yield();  // Dar tiempo a otras tareas del sistema
}