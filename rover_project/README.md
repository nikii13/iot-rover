# Rover IoT — seguimiento de línea negra (Raspberry Pi)

Proyecto completo del rover: servicio C++ de encoders (`encodered`), control de
motores, PID de velocidad por rueda, IMU, sensor de color, seguimiento de
línea de colores y el nuevo **seguimiento de línea negra** (`lab/black_line_follow.py`).

> **Estado:** compila y supera 117 pruebas automáticas sin hardware (incluido un
> simulador cinemático). **No se ha probado en el rover real.** No debe darse por
> bueno el seguimiento físico hasta completar las pruebas del apartado 8.

---

## 0. Convivencia con el proyecto `iot-rover`

Este proyecto es **independiente** de `~/iot-rover`: va en su propia carpeta
(`~/rover_project`), no modifica los archivos del proyecto original y **usa el mismo servicio
de encoders que ya tenemos instalado** (`encoderd.service`, `/usr/local/bin/encoderd`).

* El cliente Python habla el protocolo de nuestro `encoderd`: cualquier mensaje → `A,B`.
  Para el diagnóstico por canal usa su comando `diag`. **Nunca envía `reset`**, para
  no poner a cero los contadores que pueda estar usando otro programa.
* `scripts/start_services.sh` detecta que `encoderd.service` está activo y **no arranca
  nada más**. El `encodered` de `cpp/` es solo una alternativa para una Pi sin el servicio original: nunca se ejecuta a la vez que el servicio original (ambos usarían los mismos GPIO).
* `scripts/stop_services.sh` **no toca** `encoderd.service`.
* **No ejecutar a la vez** un programa de `iot-rover` y otro de `rover_project`: los dos
  escribirían en los GPIO de los motores. Ejecutar uno cada vez y comparar los logs.

Lo que se ha comprobado de nuestro `encoderd` (iot-rover = cpsy-encoder-final = binario
instalado): cuenta **flancos de subida** de cada canal por separado y devuelve
`max(C1, C2)` por motor. **No da sentido de giro** (las cuentas solo suben), así que
este proyecto toma el sentido de la orden aplicada al motor
(`ENCODER_SIGN_FROM_COMMAND = True`, obligatorio con el servicio original). Los 700 PPR son
flancos de subida de **un** canal por vuelta, que es lo que mide `tests/hw_encoder_ppr.py`.

---

## 1. Contenido

```
rover_project/
├── README.md               este documento
├── Makefile                make build | test | start | stop | check
├── requirements.txt
├── cpp/
│   ├── encoder.cpp         fuente del servicio encodered (GPIO -> socket UNIX)
│   ├── quadrature.hpp      decodificador de encoder (sin hardware, testeable)
│   ├── test_quadrature.cpp pruebas unitarias del decodificador
│   └── Makefile            genera cpp/encodered
├── lab/
│   ├── config.py           TODOS los parámetros, etiquetados [USUARIO]/[DATASHEET]/[PENDIENTE]
│   ├── rover.py            motores (único módulo que escribe PWM en GPIO)
│   ├── encoder_client.py   cliente del socket /tmp/encoder.sock
│   ├── encoder_status.py   Test 1: cuentas y flancos por canal + diagnóstico
│   ├── speedometer.py      cuentas -> RPM
│   ├── speed_control.py    feed-forward + PI(D) por motor, anti-windup
│   ├── pidtest.py          Test 3: PID y medición automática de feed-forward
│   ├── imu.py              IMU (detección por WHO_AM_I), Test 4
│   ├── color_sensor.py     TCS34725 por I2C, Test 5
│   ├── drive_loop.py       ÚNICO bucle de control (encoders -> IMU -> sensor -> trayectoria -> PID -> PWM)
│   ├── line_control.py     seguidor de borde, mezclador con IMU, máquina de estados, búsqueda
│   ├── black_line_follow.py  NUEVO: seguimiento de línea negra
│   ├── color_line_follow.py  seguimiento de 3 colores (reconstruido, ver §3)
│   └── diag_log.py         log CSV no bloqueante
├── tests/
│   ├── test_*.py           pruebas automáticas sin hardware (pytest)
│   ├── sim_rover.py        simulador SOLO para pruebas automáticas
│   └── hw_*.py             pruebas físicas guiadas (requieren el rover)
├── scripts/
│   ├── build.sh  start_services.sh  stop_services.sh  check_system.sh
├── calibration/            calibraciones guardadas (surface.json, colors.json)
└── logs/                   logs CSV de cada ejecución + encodered.log
```

---

## 2. Arquitectura y flujo de comunicación

```
GPIO 17,27,22,23 ──► encodered (C++)  ──AF_UNIX/SOCK_DGRAM /tmp/encoder.sock──► encoder_client.py
                     un hilo, poll():                                            │
                     eventos GPIO + peticiones                                   ▼
                                                                         speedometer.py (RPM)
I2C 0x29 TCS34725 ──► color_sensor.py ──► cobertura ──► line_control.py     │
I2C IMU ────────────► imu.py ──► velocidad angular ───►  (nivel A + B)       │
                                                        target_rpm izq/der   │
                                                               ▼             ▼
                                                    speed_control.py (nivel C, PID por motor)
                                                               ▼
                                                rover.py ──► GPIO 12,18 (A)  13,19 (B)
```

**Protocolo del socket (sin cambios):** el cliente crea su propio socket
`AF_UNIX/SOCK_DGRAM`, hace `bind()` a `/tmp/encoder_client_<pid>_<id>.sock` (para que
el servicio pueda responder), envía `GET` y recibe `MotorA_steps,MotorB_steps`.
Extensiones solo de diagnóstico, que no afectan al formato original: `STATUS`
(flancos por canal, eventos perdidos) y `PING`.

| Pregunta | Respuesta en esta implementación |
|---|---|
| Quién crea el socket | `encodered`, al arrancar (borra antes un socket residual; puede hacerlo porque tiene el bloqueo) |
| Cómo pide Python | `sendto(b"GET")` desde su socket enlazado; `recv` con timeout (`ENCODER_TIMEOUT_S`, 30 ms) y 1 reintento |
| Cómo responde C++ | `sendto` a la dirección del cliente; si el cliente no hizo `bind()` no hay respuesta |
| Interpretación | dos enteros con signo de 64 bits, acumulados desde el arranque del servicio |
| Servicio caído | `EncoderError`; 5 fallos seguidos → **parada de seguridad** de los motores |
| Limpieza | `encodered` borra su socket con SIGINT/SIGTERM; el cliente borra el suyo en `close()` |
| Doble instancia | `flock` sobre `/tmp/encodered.lock`; la segunda instancia sale con código 2 sin tocar el socket |
| Permisos | no necesita root: basta con pertenecer al grupo `gpio` (acceso a `/dev/gpiochip*`) |

**Una sola autoridad sobre el PWM:** el seguidor de línea solo produce RPM objetivo;
`speed_control.py` las convierte en PWM; `drive_loop.py` llama una vez por ciclo a
`Rover.set_motors()`. No hay hilos de control paralelos.

---

## 3. Decisiones de implementación y discrepancias

El proyecto se ha construido a partir de la especificación del laboratorio. Los parámetros que todavía requieren verificación física se indican a continuación:

| Elemento | Decisión | Estado |
|---|---|---|
| Pines motor 12/18, 13/19 y encoder 17/27/22/23 | Según la configuración del laboratorio | [USUARIO] — verificar con `hw_test_motors.py` y `encoder_status.py` |
| Qué señal es "adelante" y qué motor es el izquierdo | Configurable (`MOTOR_*_INVERT`, `MOTOR_A_SIDE`) | [PENDIENTE] — `hw_test_motors.py` lo determina |
| Socket `/tmp/encoder.sock`, DGRAM, `A,B` | Conservado | [USUARIO] |
| Servicio de encoders | **Se usa nuestro `encoderd`** (systemd). `cpp/encodered` es una alternativa que solo se arranca si no hay ningún servicio | ver §0 |
| Librería GPIO del C++ | Nuestro `encoderd` usa `lgpio`; el `encodered` alternativo usa la interfaz de carácter del kernel | |
| `PULSES_PER_REV = 700`, rueda 4,8 cm | Conservados | [USUARIO] — medir con `hw_encoder_ppr.py` |
| PID y feed-forward | Conservados los valores experimentales | [USUARIO] — ver advertencia de abajo |
| Sensor TCS34725, dirección 0x29 | 0x29 es la dirección fija del fabricante; el driver comprueba el registro ID y falla si no es un TCS3472x | verificar con `i2cdetect -y 1` |
| Modelo de IMU | **No se supone**: se detecta por WHO_AM_I (familias MPU-6050/6500/9250 e ICM-206xx, y LSM6DS3/DSL/DSO). Si es otra, el programa lo dice y puede funcionar sin IMU | [PENDIENTE] |
| Algoritmo de 3 colores | **No disponible.** `color_line_follow.py` es una reconstrucción sobre la misma arquitectura, no una copia del original | ver `lab/color_line_follow.py` |
| Nº de sensores de línea | Se ha diseñado para **un único sensor** (el escenario de la especificación). No se utiliza una matriz de sensores | |
| Vía entre ruedas, posición del sensor | No documentadas | [PENDIENTE] |

Si recuperamos los archivos originales (`lab/`, `encoder.cpp`, `rover_full.py`,
`rc_pid_lab9_working.py`, el seguidor de colores), podemos copiarlos en `lab/legacy/` y
comparar: pines, modo de conteo del encoder, factor PPR y acciones por color.
Cualquier diferencia debe resolverse a favor de lo que esté verificado físicamente.

### Dos indicios en los datos experimentales que conviene comprobar primero

1. **Encoders.** "Motor A = 0 pulsos" y "GPIO 23 siempre a 0" encajan con un canal
   muerto. Con decodificación en cuadratura completa (x4), un canal muerto hace que el
   contador sume +1 y −1 alternativamente y se quede en ~0. Por eso el servicio usa por
   defecto **x1** (flancos de subida del canal 1) y Python toma el **sentido de la orden
   aplicada** (`ENCODER_SIGN_FROM_COMMAND = True`). `encoder_status.py` muestra los flancos
   de cada canal y dice cuál falla. Si cambiamos el modo (x2/x4), cambia el número de
   cuentas por vuelta y hay que volver a medir `PULSES_PER_REV`.
2. **Feed-forward.** `FF_A = 0.0100` frente a `FF_B = 0.0015` supone que el motor B daría
   6,7 veces más RPM con el mismo PWM. Es poco probable entre dos motores iguales; es más
   probable que los dos encoders no cuenten igual (modo o canal distinto). En el
   simulador, con motores parecidos, esos valores hacen que el motor B tarde unos 20 s en
   alcanzar el objetivo, porque KI es muy bajo. Eso produce desviaciones y correcciones
   tardías como las observadas. **Conviene medir con `pidtest.py --ff-fit` antes de modificar el seguidor.**

---

## 4. Por qué el rover seguía recto en la curva a la izquierda

Sin el código anterior no se puede confirmar la causa exacta, pero estas causas encajan
con lo observado y el diseño nuevo elimina cada una:

| Causa probable | Cómo se resuelve |
|---|---|
| Lógica binaria "negro = recto, blanco = girar" sobre el **centro** de la línea: con un solo sensor, al salirse no se sabe por qué lado, y el lado de giro queda fijo, así que funciona en un sentido de pista y no en el otro | Se sigue un **borde**: la lectura parcial (gris) dice hacia dónde corregir, de forma continua y simétrica para curvas a izquierda y derecha |
| La IMU mantenía el **rumbo inicial** y se oponía al giro | La IMU regula la **velocidad angular hacia la que pide la línea** (`omega_ref`), nunca un rumbo fijo. En recta, `omega_ref ≈ 0` y corrige la deriva; en curva, refuerza el giro |
| Diferencias entre motores / FF_B incorrecto → la orden de giro no se materializa | PID por rueda con anti-windup + medición de FF; el bucle de IMU compensa lo que quede |
| Velocidad excesiva frente a la frecuencia de lectura | Integración del sensor de 12 ms (≈80 lecturas/s, frente a ≈40 con 24 ms), bucle a 50 Hz, y la velocidad baja automáticamente con el error y con el giro |
| Seguir recto al perder la línea | En fondo claro sigue girando hacia la línea; si en 0,8 s o 60° de giro no aparece → `LINE_LOST` → frenada → búsqueda acotada → `STOPPED` |

---

## 5. Diseño del seguimiento de línea negra

**Cobertura:** `cov = (fondo − C) / (fondo − negro)` con el canal *clear* calibrado
(0 = fondo, 1 = negro). Se filtra (EMA) y se clasifica con histéresis en
`LINE / EDGE / BACKGROUND`. Los umbrales son **fracciones del contraste calibrado**, no
valores crudos.

**Nivel A — borde (EDGE_SIDE = left por defecto):**
`omega_ref = ±(KP·(cov − 0.5) + KD·dcov/dt)`. Más oscuro → girar hacia la línea; más claro →
alejarse. La velocidad de avance baja con |error| y con |omega_ref|.

**Nivel B — mezcla + IMU:** `ΔRPM = RPM_PER_DEGPS·omega_ref + KP_yaw·(omega_ref − omega_IMU)`.
Si una rueda satura, se sacrifica velocidad de avance, no giro. La rueda interior puede
invertirse en curvas muy cerradas (`ALLOW_WHEEL_REVERSE`, desactivado por defecto desde el ajuste del 29-10).

**Nivel C — PID por motor** con los encoders (`speed_control.py`).

**Estados:**

| Estado | Significado | Salida |
|---|---|---|
| `CALIBRATING` | calibración automática girando sobre la línea (`--auto-calibrate`) | |
| `FOLLOWING` | sobre el borde, error pequeño | |
| `CORRECTING_LEFT/RIGHT` | error grande, corrigiendo hacia ese lado | |
| `LINE_LOST` | fondo claro más de 0,8 s o tras girar 60° hacia la línea sin verla: frena 0,25 s y elige lado de búsqueda | → `SEARCHING` |
| `SEARCHING` | giro sobre sí mismo: barrido 1 (75°) hacia el lado probable, barrido 2 (90°) hacia el otro, vuelta al rumbo inicial; máximo 8 s | línea encontrada → `FOLLOWING`; si no → `STOPPED` |
| `STOPPED` | fin, con motivo registrado | |

**Lado de búsqueda:** el giro medio de los últimos 0,4 s, si fue claro (p. ej. una curva
cerrada que se escapó o un cruce de la línea). Si no, el lado opuesto al borde seguido
(con el borde izquierdo, estar en fondo claro implica que la línea está a la derecha).

**Negro continuo** más de 2,5 s (zona negra, cruce perpendicular) → `STOPPED`.

**Colocación inicial:** el sensor sobre el borde izquierdo de la línea (o el derecho con
`--edge right`). Si arranca en fondo claro, busca primero.

---

### Ajuste en el rover real (29-10)

La primera prueba en la pista completa acabó en **zigzag en el sitio**: con
`LINE_KP_DPS = 260` y marcha atrás permitida, las ruedas invertían el sentido varias
veces por segundo. Los motores tardan en invertir, y con los encoders del laboratorio
(que no dan sentido) la velocidad medida durante la inversión sale con el signo
equivocado. Además, salirse unos milímetros al fondo claro, algo normal con un solo
sensor, se trataba como línea perdida a los 0,12 s.

Con las curvas de motor medidas y un retardo realista, el simulador reproduce el
fallo (`tests/test_real_rover_sim.py`). Los valores actuales lo corrigen:
`LINE_KP_DPS = 90`, `LINE_KD_DPS = 0`, `BASE_RPM = 26`, `ALLOW_WHEEL_REVERSE = False`
(la búsqueda sí invierte), y línea perdida solo tras 0,8 s o 60° girando hacia ella.

**Curva a la derecha (segunda prueba):** al pivotar, la rueda derecha (A) se detiene
y la izquierda (B) tiene que arrastrar el rover. Con PWM 0.14 no vencía el rozamiento
estático, y su integrador (`KI_B = 0.001`) tardaba segundos en subir. Resultado: ninguna
rueda giraba, la línea se "perdía" y se repetía el ciclo. Se ha añadido un **impulso de
arranque** (`STICTION_KICK_RATE`): si una rueda debería girar y lleva más de 60 ms
parada, su PWM sube 1,5 por segundo hasta que se mueve. Está reproducido y comprobado
en `tests/test_real_rover_sim.py`.

**Curvas a la derecha más difíciles que a la izquierda (tercera prueba):** es una
limitación de seguir el borde izquierdo con un solo sensor. En una curva a la izquierda,
si el rover se queda corto, el sensor entra en la línea negra, que tiene 2 cm de margen.
En una curva a la derecha se sale hacia el fondo blanco, donde no hay margen ni
información. Con un solo control proporcional, para girar de forma continua el sensor
tiene que quedarse desplazado del borde, y en las curvas a la derecha se queda junto al
blanco. El término integral (`LINE_KI_DPS = 80`) lo devuelve al borde durante la curva.
Si la pista tiene sobre todo curvas a la derecha, conviene seguir el borde derecho
(`--edge right`).

Además: `IMU_YAW_SIGN = -1` (medido dos veces), `TRACK_WIDTH_CM = 10.2`, y la búsqueda
también se limita por el giro medido con las ruedas, por si la IMU fallara.

**IMU:** si `IMU_YAW_SIGN` está invertido, la IMU empuja en el sentido equivocado y el
rover no se centra. El bucle compara la IMU con el giro medido por las ruedas: si
discrepan sistemáticamente, desactiva la IMU en esa ejecución y lo indica en un
`[evento]`.

### Más fluido y más rápido (02-11): qué se probó y qué funcionó

Para ajustar el simulador se usaron las curvas de motor medidas y la zona de transición
del sensor estimada a partir de los logs reales (unos 7–8 mm de radio). Se probó en un
circuito mixto: rectas, curvas de 90° y 180° a ambos lados, S y una curva cerrada de radio 8 cm.

| Configuración | Velocidad media | Oscilación lateral en recta | Pérdidas de línea |
|---|---|---|---|
| Antes (29-10) | 4,8 cm/s | 4,0 mm | 8 |
| **`--perfil suave`** (por defecto) | 4,8 cm/s | **3,1 mm** | **4** |
| **`--perfil rapido`** | **6,2 cm/s** | 8,1 mm | 6 |

**Lo que funcionó:** derivativo sobre la cobertura (`LINE_KD_DPS = 6`) y suavizado de
velocidad (`SPEED_TAU_UP_S` / `SPEED_TAU_DOWN_S`). Antes la velocidad subía y bajaba en
cada pequeña corrección.

**Lo que se probó y se descartó** porque no mejoraba: amortiguación por rumbo con la IMU,
adelanto (lead) en la diferencia de ruedas, bucle a 100 Hz, menos ganancia proporcional
con más integral, y velocidad base de 36 o más.

**Límite físico:** con un solo sensor y unos motores que tardan unos 0,3 s en responder,
la oscilación crece con la velocidad, porque el retraso es fijo en tiempo. Los cambios
que más ayudarían son de hardware: un segundo sensor de línea (el TCS34725 tiene
dirección fija 0x29, así que haría falta un multiplexor I2C como el TCA9548A, o un sensor
de reflectancia analógico en otro canal del MCP3008) o un motor A más rápido.

### Parada por obstáculo (sensor de distancia)

`lab/distance_sensor.py` lee el sensor IR por el MCP3008, con la misma configuración que
`rc_pid_lab9_working.py`: canal 0, pines 11/10/9/8, 3,3 V, mediana de 3 lecturas.
`ObstacleGuard` (en `line_control.py`) envuelve el seguidor. Si el rover avanza y la tensión
llega a ≥ 2,0 V (unos 20 cm), se detiene en el estado `OBSTACLE`. Reanuda cuando la
tensión se mantiene por debajo de 1,7 V durante 1 s. Si el objeto sigue ahí más de 60 s,
termina. Durante la parada no cuentan los temporizadores de línea perdida. Los giros
sobre sí mismo (búsqueda) no se bloquean, igual que en el programa original. Si falla la
lectura del sensor, se produce una parada de seguridad. `--no-distance` lo desactiva.

---

## 5b. Lab 11 — Convoy

**Roles** (`lab/convoy.py`):

```bash
python3 lab/convoy.py --role follower                              # seguidor
python3 lab/convoy.py --role leader --leader-mode line --base-rpm 22  # líder que sigue la línea, despacio
python3 lab/convoy.py --role leader --leader-mode manual           # líder a mano: indica el programa RC original
```

**Antes, en cada seguidor:** calibrar la línea (`black_line_follow.py --calibrate`) y el
sensor de distancia (`distance_sensor.py --calibrate`). El sensor IR da voltios y no es
lineal, así que la calibración crea la tabla de tensión a centímetros
(`calibration/distance.json`). Para comprobarla: `python3 lab/distance_sensor.py`, que
muestra los cm en vivo.

**Cómo funciona el seguidor** (`ConvoyController` en `line_control.py`): sigue la línea igual
que siempre, y la velocidad de avance la decide la distancia al de delante.

| Situación | Comportamiento |
|---|---|
| Líder a la vista | PI sobre la separación: objetivo 30 cm (`CONVOY_TARGET_CM`), dentro de la ventana de 20–40 cm del lab |
| Líder parado | Se para detrás (histéresis: las ruedas no giran estables por debajo de ~20 RPM) y arranca cuando el líder se aleja |
| A menos de 20 cm (`CONVOY_STOP_CM`) | Parada inmediata |
| **El líder sale del haz en una curva** (el IR apunta recto) | Supone que el líder se ha **parado** donde lo vio y descuenta lo que avanza el seguidor (encoders). La estimación es pesimista, así que la distancia real nunca es menor. Se acerca a ~20 cm, gira con la curva y vuelve a verlo |
| Sin verlo 3 s esperando | Supone que el líder se fue y avanza despacio (`CONVOY_BLIND_RPM`) hasta encontrarlo |
| Pierde la línea en una curva | Búsqueda acotada con barridos más amplios (110°/120°) que fuera del convoy; el convoy no la interrumpe |
| Recta y bien centrado | Puede ir hasta `CONVOY_MAX_RPM` para alcanzar al líder; en las correcciones, como máximo la base |

**Velocidad máxima (simulación con los motores del rover; falta medirlo en la pista):**

| Líder | Recta | Curvas (90° R20 + 90° R15) |
|---|---|---|
| 3 cm/s | 30–32 cm ✅ | 22–47 cm, se para a 29,6 cm |
| 4 cm/s | 30 cm ✅ | 24–45 cm (73 % del tiempo en 20–40) |
| 5 cm/s | 30–31 cm ✅ | 29–69 cm (se queda atrás en las curvas) |
| 6 cm/s | 32–37 cm ✅ | — |
| 7 cm/s | 47–54 cm ❌ el seguidor no da más | — |

Conclusión provisional: en recta el convoy aguanta hasta **~6 cm/s** del líder (unos 24 RPM).
En curvas, hasta **~4 cm/s**. Con más velocidad, el seguidor pierde terreno porque frena
en cada corrección de línea, y porque el líder sale del haz del IR. Coincide con la nota
del enunciado: más despacio es más preciso.

---

## 6. Instalación en la Raspberry Pi

```bash
# 0. Si hay un entorno virtual activo "(venv)", salir de él: los paquetes de apt no se ven dentro
deactivate 2>/dev/null || true

# 1. Dependencias del sistema (Raspberry Pi OS Bookworm o posterior)
sudo apt update
sudo apt install -y unzip build-essential python3-gpiozero python3-lgpio python3-smbus2 i2c-tools python3-pytest
# (alternativa: pip install -r requirements.txt  — en Bookworm, dentro de un venv)

# 2. Activar I2C y comprobar permisos
sudo raspi-config nonint do_i2c 0
sudo usermod -aG gpio,i2c $USER      # cerrar sesión y volver a entrar

# 3. Descomprimir y compilar
unzip rover_project.zip && cd rover_project
make build            # genera cpp/encodered
make test             # pruebas C++ y Python sin hardware

# 4. Diagnóstico y servicio de encoders
./scripts/check_system.sh     # gpiochips, i2cdetect, dependencias, estado de encoderd
./scripts/start_services.sh   # con encoderd.service activo: solo confirma que responde
```

Los parámetros de `encodered` (pines, socket, modo) salen de `lab/config.py`, que es la
única fuente. Los ajustes propios van en `lab/config_local.py`, que tiene prioridad; así
no hace falta editar `config.py`:

```python
# lab/config_local.py
MOTOR_A_SIDE = "right"
FF_B_FORWARD_PER_RPM = 0.0091
PULSES_PER_REV_B = 703
```

---

## 7. Ejecución

```bash
cd rover_project
./scripts/start_services.sh

# Línea negra
python3 lab/black_line_follow.py --calibrate          # una vez por pista/iluminación
python3 lab/black_line_follow.py --no-motors          # empujar el rover a mano: ver estados sin moverse
python3 lab/black_line_follow.py --max-time 60        # seguir la línea
python3 lab/black_line_follow.py --edge right --base-rpm 30
python3 lab/black_line_follow.py --auto-calibrate     # calibra girando sobre la línea y sigue

# Línea de colores
python3 lab/color_line_follow.py --calibrate
python3 lab/color_line_follow.py --max-time 60
```

Ctrl+C detiene los motores en cualquier momento.

---

## 8. Pruebas físicas (pendientes) — en este orden

| # | Comando | Qué comprobar / qué ajustar |
|---|---|---|
| 0 | `./scripts/check_system.sh` | `0x29` (TCS34725) y la IMU en `i2cdetect`; usuario en `gpio` |
| 1 | `python3 lab/encoder_status.py` (girar cada rueda a mano) | Los **4 canales** con flancos. Si un canal no tiene flancos, el resumen lo indica |
| 1b | `python3 tests/hw_encoder_ppr.py --turns 10` | `PULSES_PER_REV_A/B` reales en el modo actual |
| 2 | `python3 tests/hw_test_motors.py` (ruedas en el aire) | Propone `MOTOR_A_SIDE`, `MOTOR_*_INVERT`; detecta encoders intercambiados |
| 3 | `python3 lab/pidtest.py --ff-fit` y luego `python3 lab/pidtest.py --rpm 40` | Propone `FF_*_PER_RPM`, `FF_*_STATIC`, `MAX_RPM`; el PID debe quedar a ±2 RPM del objetivo |
| 4 | `python3 lab/imu.py` | Bias y ruido; girando a mano a la izquierda el ángulo **aumenta** (si no, `IMU_YAW_SIGN = -1`) |
| 4b | `python3 tests/hw_calibrate_turn.py` (en el suelo) | `RPM_PER_DEGPS` real y confirmación de signos |
| 5 | `python3 lab/color_sensor.py` | Valores C sobre negro, fondo y borde; sin "SATURADO". Contraste ≥ 25 % |
| 6 | `python3 lab/color_line_follow.py --calibrate` y ejecutar | Sigue la línea de colores; comparar con el comportamiento del programa antiguo |
| 7 | `python3 lab/black_line_follow.py --no-motors`, después sin `--no-motors` | Estados correctos al empujar a mano; seguimiento y recuperación |
| 8 | Pista en **ambos sentidos** (primera curva a la izquierda y a la derecha), con `--edge left` y `--edge right` | Responde a la curva izquierda; tras el final de la línea busca y se para |

**Ajuste del seguidor (en este orden):** `BASE_RPM` bajo (25–30) → `LINE_KP_DPS`
(si oscila: reducirlo; si se sale en curvas: aumentarlo) → `LINE_KD_DPS` → `YAW_RATE_KP` → subir
`BASE_RPM`. En cada ejecución se guarda un CSV en `logs/` con estado, cobertura, RPM
objetivo/medidas, error PID, PWM, `omega` pedida/medida, corrección IMU y motivos de
búsqueda/parada.

---

## 9. Verificación realizada (sin hardware)

* `make -C cpp test`: decodificador pulse/x1/x2/x4, ambos sentidos, canal muerto, flancos perdidos.
* `pytest tests` (117 pruebas):
  * Compatibilidad con nuestro `encoderd` (respuesta `A,B` a todo, `diag`, sin `reset`).
  * Protocolo real de `encodered` compilado en modo `--no-gpio` (`GET`/`STATUS`/`PING`),
    bloqueo de doble instancia y limpieza del socket.
  * Cliente: timeouts, respuestas inválidas, servicio caído.
  * PID: convergencia con FF erróneo, anti-windup, objetivo cero, inversión, límites.
  * Signos del seguidor de borde, histéresis, calibración, y que la IMU **no se opone** a un giro pedido.
  * Simulador con el bucle y el seguidor reales: curvas de 90° y 120° a izquierda y
    derecha (radios de 15, 8 y 5 cm), S, ambos bordes, sin IMU, canal 2 muerto, arranque
    fuera de la línea, búsqueda acotada, parada por fallo de encoders, motor bloqueado,
    negro continuo y excepción.
  * Todos los módulos se importan sin `gpiozero` ni `smbus2`.

**Límites del simulador:** iluminación ideal, sin vibraciones ni latencias de I2C, y
geometría supuesta (sensor 6 cm por delante del eje, línea de 1,9 cm, vía de 13 cm).
Sirve para detectar errores de lógica y de signos, **no** demuestra el comportamiento
real. Para el rover real valen las pruebas del apartado 8.
