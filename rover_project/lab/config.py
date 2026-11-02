"""
config.py — Parámetros centralizados del rover.

Cada valor lleva una etiqueta de procedencia:

  [USUARIO]    Dato que proporcionasteis (pines, PID experimental, PPR...).
               Se usa tal cual, pero conviene confirmarlo con los tests hw_*.
  [DATASHEET]  Fijado por el fabricante del componente (no es una elección).
  [PENDIENTE]  Valor inicial razonable para arrancar. DEBE medirse/ajustarse
               en el rover real (el README indica con qué script).
  [DERIVADO]   Calculado a partir de otros valores.

Para ajustes locales sin tocar este archivo, cread lab/config_local.py y
redefinid ahí lo que necesitéis (se importa al final y tiene prioridad).
"""

import os as _os

_LAB_DIR = _os.path.dirname(_os.path.abspath(__file__))
PROJECT_DIR = _os.path.dirname(_LAB_DIR)

# ---------------------------------------------------------------------------
# 1. Motores (driver de dos entradas por motor: una con PWM, otra a 0)
# ---------------------------------------------------------------------------
MOTOR_A_PIN_1 = 12          # [USUARIO] señal 1 motor A (se usa como "adelante")
MOTOR_A_PIN_2 = 18          # [USUARIO] señal 2 motor A ("atrás")
MOTOR_B_PIN_1 = 13          # [USUARIO] señal 1 motor B
MOTOR_B_PIN_2 = 19          # [USUARIO] señal 2 motor B
MOTOR_PWM_FREQUENCY_HZ = 1000   # [PENDIENTE] frecuencia de PWM
MOTOR_A_INVERT = False      # [MEDIDO 29-10] hw_test_motors: A gira hacia delante
MOTOR_B_INVERT = True       # [MEDIDO 29-10] hw_test_motors: B giraba hacia atrás con orden "adelante"
MOTOR_A_SIDE = "right"      # [MEDIDO 29-10] motor A = rueda DERECHA, motor B = IZQUIERDA
MOTOR_BACKEND = "gpiozero"  # "gpiozero" en la Pi, "mock" para pruebas sin hardware
DUTY_LIMIT = 1.0            # límite absoluto de PWM (0..1) aplicado SIEMPRE en rover.py

# ---------------------------------------------------------------------------
# 2. Encoders (los lee EXCLUSIVAMENTE el servicio C++ encodered)
# ---------------------------------------------------------------------------
ENCODER_A_PIN_1 = 17        # [USUARIO]
ENCODER_A_PIN_2 = 27        # [USUARIO]
ENCODER_B_PIN_1 = 22        # [USUARIO]
ENCODER_B_PIN_2 = 23        # [USUARIO] en una prueba se quedó en 0 -> verificar con encoder_status.py
ENCODER_SOCKET = "/tmp/encoder.sock"         # [USUARIO] AF_UNIX / SOCK_DGRAM
ENCODER_LOCK = "/tmp/encodered.lock"
ENCODER_SERVICE_BINARY = _os.path.join(PROJECT_DIR, "cpp", "encodered")
# Modo de decodificación del servicio: "pulse" | "x1" | "x2" | "x4".
# x1 = flancos de subida del canal 1 (sentido desde el canal 2). Es el que
# menos depende del canal 2 y da 1 cuenta por pulso del canal 1.
# Si cambiáis el modo, cambia el número de cuentas por vuelta (x2 = doble, x4 = cuádruple).
ENCODER_DECODE_MODE = "x1"  # [PENDIENTE] confirmar qué contaba el servicio original
ENCODER_BIAS = "pull-up"    # [PENDIENTE] "pull-up" | "pull-down" | "disabled" | "as-is"
ENCODER_DEBOUNCE_US = 0     # [PENDIENTE] antirrebote del kernel; 0 = desactivado
ENCODER_TIMEOUT_S = 0.03    # timeout de cada petición al socket
ENCODER_RETRIES = 1         # reintentos por lectura
ENCODER_MAX_CONSECUTIVE_FAILURES = 5   # lecturas fallidas seguidas -> parada segura
# Si True, el sentido de giro se toma de la orden aplicada al motor y del
# encoder solo el módulo. Necesario mientras el canal 2 no esté verificado.
ENCODER_SIGN_FROM_COMMAND = True

PULSES_PER_REV_A = 700      # [MEDIDO 29-10] 692.7 en 10 vueltas a mano (≈1 %, dentro del error de contar vueltas)
PULSES_PER_REV_B = 700      # [MEDIDO 29-10] 699.7 en 10 vueltas a mano
WHEEL_DIAMETER_CM = 4.8     # [USUARIO]
TRACK_WIDTH_CM = 10.2       # [MEDIDO 29-10] distancia entre centros de las ruedas
MAX_PLAUSIBLE_RPM = 600     # salto de cuentas mayor que esto se trata como lectura inválida

# ---------------------------------------------------------------------------
# 3. Control de velocidad (PI + feed-forward por motor)
# ---------------------------------------------------------------------------
CONTROL_LOOP_HZ = 50        # frecuencia del único bucle de control

KP_A = 0.0025               # [USUARIO] experimental
KI_A = 0.0015               # [USUARIO] experimental
KD_A = 0.0
KP_B = 0.0010               # [USUARIO] experimental
KI_B = 0.0010               # [USUARIO] experimental
KD_B = 0.0
FF_A_FORWARD_PER_RPM = 0.0100   # [USUARIO] PWM por RPM, motor A
FF_B_FORWARD_PER_RPM = 0.0015   # [USUARIO] PWM por RPM, motor B (ver README: la diferencia 6.7x es sospechosa)
FF_A_BACKWARD_PER_RPM = FF_A_FORWARD_PER_RPM   # [PENDIENTE] marcha atrás
FF_B_BACKWARD_PER_RPM = FF_B_FORWARD_PER_RPM   # [PENDIENTE]
FF_A_STATIC = 0.0           # [PENDIENTE] PWM mínimo para vencer rozamiento estático (pidtest --find-deadband)
FF_B_STATIC = 0.0           # [PENDIENTE]
# Tabla medida [(rpm, pwm), ...] con pidtest.py --ff-fit. Si existe, sustituye a
# FF_*_PER_RPM / FF_*_STATIC (necesaria si la curva RPM-PWM no es una recta).
FF_A_TABLE = None           # [PENDIENTE] pegar la tabla que propone pidtest.py --ff-fit
FF_B_TABLE = None           # [PENDIENTE]
INTEGRAL_LIMIT = 0.4
# Impulso de arranque [AJUSTADO 29-10]: en la curva a la derecha la rueda izquierda (B) se quedaba
# parada con PWM 0.14 al pivotar; el integrador de B tardaba segundos en vencer el rozamiento.
STICTION_KICK_RATE = 1.5    # PWM/s añadido mientras la rueda debería girar y no gira (0 = desactivado)
STICTION_RPM = 3.0          # por debajo de esto se considera parada
STICTION_DELAY_S = 0.06     # espera antes de empezar el impulso        # contribución máxima del término integral (en PWM)
RPM_FILTER_ALPHA = 0.5      # filtro exponencial de la RPM medida (1 = sin filtro)
MAX_RPM = 45.0              # [MEDIDO 29-10] motor A en el suelo: 45 RPM ya pide PWM 0.7-0.93 (en vacío 71 a PWM 1.0)
MAX_ACCEL_RPM_S = 150.0     # rampa de RPM objetivo: sincroniza el arranque de A (arranca a PWM ~0.3) y B (~0.1)

# Detección de motor bloqueado / encoder sin pulsos
STALL_DETECTION = True
STALL_DUTY = 0.45           # PWM a partir del cual esperamos movimiento
STALL_RPM = 2.0             # por debajo de esto, la rueda "no se mueve"
STALL_TIME_S = 0.8          # tiempo continuo antes de parar

# ---------------------------------------------------------------------------
# 4. Sensor de color (I2C)
# ---------------------------------------------------------------------------
COLOR_SENSOR_MODEL = "TCS34725"   # [USUARIO] verificar: i2cdetect -y 1 debe mostrar 29
COLOR_I2C_BUS = 1           # bus I2C estándar del conector de 40 pines (SDA=GPIO2, SCL=GPIO3)
COLOR_I2C_ADDRESS = 0x29    # [DATASHEET] el TCS34725 tiene dirección fija 0x29
COLOR_INTEGRATION_MS = 12.0 # [PENDIENTE] múltiplo de 2.4 ms; menor = más rápido y menos resolución
COLOR_GAIN = 4              # [PENDIENTE] 1, 4, 16 o 60
COLOR_MAX_CONSECUTIVE_FAILURES = 5
SENSOR_FORWARD_OFFSET_CM = 6.0    # [PENDIENTE] distancia del sensor por delante del eje de ruedas (solo doc./simulación)

# ---------------------------------------------------------------------------
# 5. IMU (I2C). El modelo se detecta por su registro WHO_AM_I.
# ---------------------------------------------------------------------------
IMU_MODEL = "auto"          # "auto" | "mpu6050" | "lsm6ds" | "icm20948" | "none"  (rover NikiC: ICM-20948 en 0x69)
IMU_I2C_BUS = 1
IMU_I2C_ADDRESS = None      # None = probar direcciones estándar del modelo
IMU_REQUIRED = False        # True: si no hay IMU, el programa no arranca
IMU_YAW_AXIS = "z"          # [PENDIENTE] eje del giróscopo perpendicular al suelo ("x"|"y"|"z")
IMU_YAW_SIGN = -1           # [MEDIDO 29-10] ICM-20948: girando a la izquierda daba -52 deg/s (imu.py y hw_calibrate_turn) +1 si girar a la IZQUIERDA da velocidad angular positiva (tests/hw_test_imu.py)
IMU_CALIBRATION_SAMPLES = 200   # muestras con el rover quieto para estimar el bias del giróscopo
IMU_GYRO_DEADBAND_DPS = 0.3
IMU_RATE_FILTER_ALPHA = 0.6
IMU_MAX_CONSECUTIVE_FAILURES = 5
IMU_SIGN_CHECK = True       # compara el signo IMU con el giro de los encoders; si es opuesto, desactiva la IMU

# ---------------------------------------------------------------------------
# 5b. Sensor de distancia IR (MCP3008) — valores de rc_pid_lab9_working.py
# ---------------------------------------------------------------------------
DISTANCE_ENABLED = True     # False o --no-distance para desactivarlo
DISTANCE_REQUIRED = True    # si falla la lectura, parada de seguridad
DISTANCE_ADC_CHANNEL = 0    # [USUARIO] rc_pid_lab9_working.py
DISTANCE_CLOCK_PIN = 11     # [USUARIO]
DISTANCE_MOSI_PIN = 10      # [USUARIO]
DISTANCE_MISO_PIN = 9       # [USUARIO]
DISTANCE_SELECT_PIN = 8     # [USUARIO]
DISTANCE_VREF = 3.3         # [USUARIO]
DISTANCE_MEDIAN_SAMPLES = 3 # [USUARIO] mediana de 3 lecturas
DISTANCE_MAX_CONSECUTIVE_FAILURES = 5
OBSTACLE_VOLTAGE = 2.0      # [USUARIO] a 20-25 cm medisteis 2.2-2.4 V (más tensión = más cerca)
CLEAR_VOLTAGE = 1.7         # [USUARIO] por debajo: el obstáculo se ha ido
OBSTACLE_CLEAR_S = 1.0      # [USUARIO] tiempo despejado antes de reanudar (10 ciclos de 0.1 s)
OBSTACLE_RESUME = True      # True: reanuda el seguimiento al quitar el objeto; False: termina el programa
OBSTACLE_MAX_WAIT_S = 60.0  # espera máxima con obstáculo antes de terminar (0 = sin límite)
DISTANCE_CALIBRATION_FILE = _os.path.join(PROJECT_DIR, "calibration", "distance.json")
DISTANCE_CALIBRATION_CM = [15, 20, 25, 30, 35, 40, 50, 60]   # distancias de la calibración

# Convoy (--convoy): seguir la línea manteniendo la separación con el robot de delante
CONVOY_TARGET_CM = 30.0     # separación deseada
CONVOY_STOP_CM = 20.0       # más cerca que esto: parada inmediata (margen por la inercia)
CONVOY_KP = 0.8             # cm/s de velocidad por cm de error de separación
CONVOY_KI = 0.15            # integral: elimina el error cuando el de delante va a velocidad constante
CONVOY_MAX_RPM = 32.0       # velocidad máxima para alcanzar al de delante (solo en tramos centrados)
CONVOY_START_RPM = 20.0     # histéresis de arranque/parada: las ruedas no giran estables por debajo
CONVOY_STOP_RPM = 13.0      #   de ~20 RPM, así que por debajo de STOP se para del todo y espera
CONVOY_DIST_ALPHA = 0.4     # filtro de la distancia medida
CONVOY_MEMORY_S = 3.0       # líder fuera del haz (curvas): tiempo ESPERANDO parado en que se supone parado
                            # donde se vio por última vez; después, vía libre
CONVOY_BLIND_RPM = 22.0     # velocidad prudente cuando no se ve al líder (≈ mínima estable de las ruedas)
CONVOY_SEARCH_FIRST_SWEEP_DEG = 110.0   # en convoy se para/arranca dentro de las curvas: barridos de
CONVOY_SEARCH_SECOND_SWEEP_DEG = 120.0  # búsqueda más amplios (con 75/90 a veces no encontraba la línea)
CONVOY_SEARCH_TIMEOUT_S = 12.0
CONVOY_MEMORY_APPROACH_CM = 10.0  # con distancia estimada, acercarse hasta TARGET-esto (≥ STOP+2) para
                                  # girar con la curva y volver a ver al líder

# ---------------------------------------------------------------------------
# 6. Seguimiento de línea (común a línea negra y colores)
# ---------------------------------------------------------------------------
SURFACE_CALIBRATION_FILE = _os.path.join(PROJECT_DIR, "calibration", "surface.json")
SURFACE_CHANNEL = "clear"   # "clear" (intensidad total) | "sum_rgb"
DARKNESS_FILTER_ALPHA = 0.7 # filtro de la "cobertura" (0 = blanco, 1 = negro)
MIN_CALIBRATION_CONTRAST = 0.25   # negro debe ser al menos un 25 % más oscuro que el fondo

# Histéresis de clasificación (fracciones del contraste calibrado, no valores crudos)
BLACK_ENTER = 0.80
BLACK_EXIT = 0.70
WHITE_ENTER = 0.15
WHITE_EXIT = 0.25

# Seguimiento de borde
EDGE_SIDE = "left"          # "left" | "right": borde de la línea que se sigue (visto en el sentido de avance)
EDGE_SETPOINT = 0.5         # cobertura objetivo (sensor medio sobre la línea)
LINE_KP_DPS = 90.0          # [AJUSTADO 29-10] grados/s por unidad de error (260 zigzagueaba en el rover real)
LINE_KD_DPS = 6.0           # [AJUSTADO 02-11] derivativo: el sensor real es poco ruidoso (min/max 132/133)
                            # y tiene zona de transición gradual; con KD=6 baja la oscilación y las pérdidas
LINE_KI_DPS = 80.0          # [AJUSTADO 29-10] integral: en curvas largas devuelve el sensor al borde (cov≈0.5);
                            # sin ella, con el borde izquierdo, las curvas a la DERECHA dejan el sensor junto al blanco
LINE_I_LIMIT_DPS = 45.0     # límite de la contribución integral (deg/s)
OMEGA_MAX_DPS = 120.0       # giro máximo solicitado por el seguidor
CORRECTING_THRESHOLD = 0.18 # |error| por encima del cual el estado es CORRECTING_*

BASE_RPM = 26.0             # [AJUSTADO 29-10] avance en recta; deja margen hasta MAX_RPM para girar
MIN_FOLLOW_RPM = 12.0       # velocidad mínima de avance mientras se sigue
SLOWDOWN_GAIN = 1.4         # reducción de velocidad por error: v = base * (1 - gain*|e|)
TURN_SLOWDOWN_DPS = 120.0   # v se divide por (1 + |omega|/este valor)
SPEED_TAU_UP_S = 0.8        # [02-11] suavizado de velocidad: acelera despacio...
SPEED_TAU_DOWN_S = 0.1      # ...y frena rápido (antes la velocidad subía y bajaba en cada corrección)
ALLOW_WHEEL_REVERSE = False # [AJUSTADO 29-10] sin marcha atrás al seguir: los encoders no dan sentido y
                            # las inversiones rápidas provocaban el zigzag en el sitio (la búsqueda sí invierte)

# Estabilización con IMU (regulador de VELOCIDAD ANGULAR, no de rumbo fijo)
YAW_RATE_KP = 0.2           # RPM de diferencia por grado/s de error de velocidad angular
YAW_RATE_CORRECTION_LIMIT_RPM = 25.0
RPM_PER_DEGPS = None        # [PENDIENTE] None = derivado de TRACK_WIDTH/WHEEL_DIAMETER; medir con hw_calibrate_turn.py

# Pérdida y búsqueda de línea
LOST_CONFIRM_S = 0.8        # tiempo máximo en fondo claro (girando hacia la línea) antes de declararla perdida
LOST_TURN_DEG = 60.0        # ...o grados girados hacia la línea sin encontrarla (lo que ocurra antes)
LOST_BRAKE_S = 0.25         # parada breve antes de buscar
BLACK_SATURATED_MAX_S = 2.5 # tiempo máximo continuo en negro total (cruce / zona negra) antes de parar
LAST_TURN_WINDOW_S = 0.4    # ventana para recordar el último sentido de corrección
LAST_TURN_MIN_DPS = 35.0    # giro medio mínimo para considerarlo "sentido de curva"
SEARCH_OMEGA_DPS = 70.0     # velocidad angular de búsqueda (giro sobre sí mismo)
SEARCH_FIRST_SWEEP_DEG = 75.0   # primer barrido hacia el lado más probable (desde el rumbo inicial)
SEARCH_SECOND_SWEEP_DEG = 90.0  # segundo barrido hacia el otro lado (ángulo medido desde el rumbo inicial)
SEARCH_TIMEOUT_S = 8.0      # tiempo máximo total de búsqueda
SEARCH_FOUND_COVERAGE = 0.35    # cobertura que indica línea encontrada
MAX_LOSSES = 5              # pérdidas de línea dentro de LOSS_WINDOW_S -> STOPPED (evita ciclos perder/encontrar sin avanzar)
LOSS_WINDOW_S = 10.0

# Calibración automática (girando sobre sí mismo encima de la línea)
AUTO_CALIBRATION_SWEEP_DEG = 50.0
AUTO_CALIBRATION_OMEGA_DPS = 40.0

# Perfiles de velocidad (--perfil). "suave" = valores de arriba.
# Medido en simulación con tus motores: "rapido" ≈ +30 % de velocidad pero ≈ 2x de oscilación lateral.
PROFILES = {
    "suave": {},
    "rapido": {"BASE_RPM": 32.0, "SLOWDOWN_GAIN": 0.8, "TURN_SLOWDOWN_DPS": 200.0},
}

# ---------------------------------------------------------------------------
# 7. Seguimiento de colores
# ---------------------------------------------------------------------------
COLOR_CALIBRATION_FILE = _os.path.join(PROJECT_DIR, "calibration", "colors.json")
COLOR_NAMES = ["red", "green", "blue"]     # [PENDIENTE] los tres colores de vuestra pista
COLOR_MAX_RESIDUAL = 0.12   # error máximo de ajuste para aceptar un color
# Acción por color: speed_scale multiplica BASE_RPM; "stop": True detiene el rover
COLOR_ACTIONS = {
    "red": {"speed_scale": 1.0, "stop": False},
    "green": {"speed_scale": 1.0, "stop": False},
    "blue": {"speed_scale": 1.0, "stop": False},
}

# ---------------------------------------------------------------------------
# 8. Registro de diagnóstico
# ---------------------------------------------------------------------------
LOG_DIR = _os.path.join(PROJECT_DIR, "logs")
LOG_CONSOLE_HZ = 5.0
LOG_QUEUE_SIZE = 2000

# ---------------------------------------------------------------------------
# Overrides locales
# ---------------------------------------------------------------------------
try:
    from config_local import *  # noqa: F401,F403
except ImportError:
    pass


def rpm_per_degps(cfg=None):
    """Diferencia de RPM (derecha - izquierda) necesaria por cada grado/s de giro. [DERIVADO]

    Giro sobre sí mismo: omega[deg/s] = dRPM * 3 * D / W   (D = diámetro rueda, W = vía)
    """
    import sys
    cfg = cfg or sys.modules[__name__]
    if getattr(cfg, "RPM_PER_DEGPS", None):
        return float(cfg.RPM_PER_DEGPS)
    return float(cfg.TRACK_WIDTH_CM) / (3.0 * float(cfg.WHEEL_DIAMETER_CM))


def as_namespace(**overrides):
    """Copia de la configuración como objeto independiente (útil en tests)."""
    import sys
    import types
    mod = sys.modules[__name__]
    ns = types.SimpleNamespace(**{k: getattr(mod, k) for k in dir(mod) if k.isupper()})
    for k, v in overrides.items():
        setattr(ns, k, v)
    return ns
