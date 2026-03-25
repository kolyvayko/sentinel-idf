# Sentinel — Специфікація системи

**Дата:** 2026-03-25
**Статус:** Затверджено (draft)
**Цільова платформа:** ESP32-C6 + ArduPilot (GUIDED_NOGPS)

---

## 1. Мета проекту

Sentinel — вбудована система автономного перехоплення дронів-цілей. ESP32-C6 визначає напрямок на RF передавач керування цілі (900 MHz або 2.4 GHz) через фазову різницю на виходах AD8302, обчислює кут пеленгу і надсилає attitude команди в ArduPilot через MAVLink. Дрон-носій летить автономно до фізичного контакту з ціллю без GPS.

---

## 2. Апаратний ланцюжок

```
Антена L ──┐
            ├──▶ AD8302 #1 ──▶ VPHS₁ (азимут)   ──▶ ADC CH1 (GPIO1)
Антена R ──┘                   VMAG₁ (опційно)  ──▶ ADC CH3 (GPIO3, опційно)

Антена F ──┐
            ├──▶ AD8302 #2 ──▶ VPHS₂ (елевація)  ──▶ ADC CH2 (GPIO2)  [ФАЗА 1: не підключено]
Антена B ──┘                   VMAG₂ (опційно)

ESP32-C6 UART (TX/RX) ──▶ ArduPilot TELEM порт (57600 baud)
SSD1306 I2C (GPIO22/23) ──▶ дисплей стану
```

**Цільові частоти:** 900 MHz (λ ≈ 33.3 cm) та 2.4 GHz (λ ≈ 12.5 cm)
**Рекомендована відстань між антенами:** λ/2 відповідної частоти
**AD8302 VPHS вихід:** 0 V = −180°, ~0.9 V = 0°, 1.8 V = +180° фазової різниці

---

## 3. Програмна архітектура

### 3.1 Модулі

| Модуль | Файли | Відповідальність |
|--------|-------|-----------------|
| `adc` | `adc/adc.c`, `adc/adc.h` | ADC continuous mode, читання VPHS (і опційно VMAG) |
| `bearing` | `bearing/bearing.c`, `bearing/bearing.h` | VPHS (mV) → азимут/елевація (градуси) |
| `navigator` | `navigator/navigator.c`, `navigator/navigator.h` | Стан машина, P-controller, генерація nav_cmd_t |
| `mavlink` | `mavlink/mavlink_task.c`, `mavlink/mavlink_task.h` | Heartbeat, SET_ATTITUDE_TARGET, режим FC |
| `display` | `display/display.c`, `display/display.h` | SSD1306: кути, стан, режим частоти |

### 3.2 FreeRTOS задачі та черги

```
adc_task        (prio 5, 4KB)  ──[adc_sample_t, depth=1, drop-oldest]──▶
bearing_task    (prio 5, 4KB)  ──[bearing_t,    depth=1, drop-oldest]──▶ navigator_task
                                └─[bearing_t,    depth=1, drop-oldest]──▶ display_task
navigator_task  (prio 4, 4KB)  ──[nav_cmd_t,    depth=2]──▶
mavlink_task    (prio 4, 8KB)
display_task    (prio 3, 4KB)
```

`bearing_task` надсилає в дві окремі черги: `s_bearing_nav_queue` (для navigator) та `s_bearing_disp_queue` (для display). Обидві depth=1, drop-oldest.

Усі черги використовують стратегію drop-oldest (глибина 1 для realtime даних).

### 3.3 Потік даних

```
[ADC] raw VPHS mV
  ↓
[bearing] θ_azimuth, θ_elevation (градуси)
  ↓
[navigator] roll, pitch, thrust (P-controller)
  ↓
[mavlink] SET_ATTITUDE_TARGET @ 20 Hz → ArduPilot FC
```

---

## 4. Алгоритм пеленгу

### 4.1 VPHS → кут

```c
// bearing/bearing.c
// vphs_zero_v: калібрувальне зміщення (SENTINEL_VPHS_ZERO_MV / 1000.0f, типово 0.9f)
float vphs_to_angle(int adc_raw, float antenna_spacing_m, float freq_hz, float vphs_zero_v) {
    float vphs      = adc_raw * 3.3f / 4095.0f;                        // 12-bit ADC, 3.3V ref
    float delta_phi = (vphs - vphs_zero_v) / vphs_zero_v * M_PI;       // фазова різниця [рад]
    float lambda    = 3e8f / freq_hz;
    float sin_theta = delta_phi * lambda / (2.0f * M_PI * antenna_spacing_m);
    sin_theta = fmaxf(-1.0f, fminf(1.0f, sin_theta));                  // clamp
    return asinf(sin_theta) * (180.0f / M_PI);                         // → градуси
}
```

Виклик: `vphs_to_angle(raw, SENTINEL_ANTENNA_SPACING_M, SENTINEL_FREQ_HZ, SENTINEL_VPHS_ZERO_MV / 1000.0f)`

Зміщення 0.9 V підлягає калібруванню при нульовому куті в полі (`SENTINEL_VPHS_ZERO_MV` в `app_config.h`).

### 4.2 Підтримка двох частот

Профіль частоти (λ та рекомендований `d`) обирається через `app_config.h`:

```c
#define SENTINEL_FREQ_HZ        900e6f   // або 2400e6f
#define SENTINEL_ANTENNA_SPACING_M  0.165f  // λ/2 для 900 MHz
```

### 4.3 Elevation (вертикаль)

Модуль `bearing` завжди обчислює `θ_elevation` з VPHS₂, але navigator використовує його лише якщо:

```c
#define SENTINEL_ELEVATION_ENABLED  0   // 0 = горизонт-only (Фаза 1)
```

При `0` — pitch команда фіксована на `SENTINEL_CRUISE_PITCH_DEG`, elevation кут логується але не впливає на керування.

---

## 5. MAVLink інтеграція

**Підключення:** ESP32-C6 UART (57600 baud) → ArduPilot TELEM порт
**Бібліотека:** `mavlink/c_library_v2` (додається як IDF компонент або `components/`)

### 5.1 Послідовність ініціалізації

1. Надсилати `HEARTBEAT` кожну 1 с (тип: `MAV_TYPE_GCS`)
2. Дочекатись `HEARTBEAT` від FC (підтвердження з'єднання)
3. Надіслати `MAV_CMD_DO_SET_MODE` → `COPTER_MODE_GUIDED_NOGPS` (20)
4. Дочекатись `COMMAND_ACK` → розпочати attitude loop

### 5.2 Attitude loop (20 Hz)

```c
mavlink_msg_set_attitude_target_pack(
    system_id, component_id, &msg,
    timestamp_ms,
    target_system, target_component,
    0b00000111,          // type_mask: ігнорувати body rates
    quaternion,          // з euler_to_quaternion(roll, pitch, yaw=0)
    0, 0, 0,             // body rates (ігноруються)
    thrust               // 0.0–1.0
);
```

### 5.3 Watchdog

Якщо `COMMAND_ACK` не отримано протягом 3 с або FC heartbeat зникає — перейти в стан `IDLE`, припинити надсилання attitude команд. FC переходить в LOITER або disarm по своїй логіці.

---

## 6. Навігатор — стан машина

```
IDLE
  │ armed + mode confirmed
  ▼
SEARCHING ◀──────────────────────────────────────────┐
  │ VPHS сигнал > SENTINEL_SIGNAL_THRESHOLD           │ сигнал втрачено > 3 с
  ▼                                                   │
TRACKING ───────────────────────────────────────────-┘
  │ VMAG > SENTINEL_VMAG_INTERCEPT_THRESHOLD
  │ (або |azimuth| < 5° і elevation ≈ 0°)
  ▼
INTERCEPT  (незворотній стан — перехоплення не скасовується)
```

### 6.1 P-controller (TRACKING)

```c
float roll   =  Kp * bearing.azimuth_deg;       // + = вправо
float pitch  = -Kp * bearing.elevation_deg       // elevation=0 → SENTINEL_CRUISE_PITCH_DEG
             + SENTINEL_CRUISE_PITCH_DEG;
float thrust = SENTINEL_CRUISE_THRUST;           // константа (0.55f)
```

`Kp` — стартове значення 0.02 рад/°, підбирається в полі.

### 6.2 INTERCEPT стан

```c
roll   = 0;
pitch  = SENTINEL_INTERCEPT_PITCH_DEG;   // агресивний нахил вперед
thrust = SENTINEL_INTERCEPT_THRUST;       // максимальна тяга (0.85f)
```

---

## 7. VMAG (опціональний канал)

- ADC CH3 (GPIO3) читає VMAG з AD8302 #1
- Нормалізується до `[0.0, 1.0]` де 1.0 = максимальний сигнал
- Використовується ЛИШЕ як додатковий поріг для переходу TRACKING → INTERCEPT
- Не впливає на розрахунок кута пеленгу
- Вмикається через `#define SENTINEL_VMAG_ENABLED 1`
- **Примітка:** VMAG параметри можуть розходитися з фазою — threshold підбирається емпірично в полі

---

## 8. Конфігурація (app_config.h)

Константи бізнес-логіки системи використовують префікс `SENTINEL_`. Константи, що належать конкретному модулю (MAVLink, FreeRTOS задачі), використовують префікс відповідного модуля (`MAVLINK_`, `ADC_TASK_`, `BEARING_TASK_` тощо) — це навмисна конвенція.

```c
// --- Bearing ---
#define SENTINEL_FREQ_HZ                900e6f
#define SENTINEL_ANTENNA_SPACING_M      0.165f   // λ/2 для 900 MHz
#define SENTINEL_VPHS_ZERO_MV           900      // калібрування нульового кута
#define SENTINEL_SIGNAL_THRESHOLD       100      // ADC counts (мінімальний сигнал)
#define SENTINEL_ELEVATION_ENABLED      0        // 0 = Фаза 1 (горизонт only)

// --- Navigator ---
#define SENTINEL_KP                     0.02f
#define SENTINEL_CRUISE_PITCH_DEG       5.0f
#define SENTINEL_CRUISE_THRUST          0.55f
#define SENTINEL_INTERCEPT_PITCH_DEG    20.0f
#define SENTINEL_INTERCEPT_THRUST       0.85f
#define SENTINEL_VMAG_INTERCEPT_THRESHOLD  0.85f
#define SENTINEL_VMAG_ENABLED           0

// --- MAVLink ---
#define MAVLINK_UART_PORT               UART_NUM_1
#define MAVLINK_BAUD_RATE               57600
#define MAVLINK_HEARTBEAT_INTERVAL_MS   1000
#define MAVLINK_ATTITUDE_RATE_MS        50       // 20 Hz
#define MAVLINK_WATCHDOG_MS             3000

// --- Tasks ---
#define ADC_TASK_STACK_SIZE             4096
#define ADC_TASK_PRIORITY               5
#define BEARING_TASK_STACK_SIZE         4096
#define BEARING_TASK_PRIORITY           5
#define NAVIGATOR_TASK_STACK_SIZE       4096
#define NAVIGATOR_TASK_PRIORITY         4
#define MAVLINK_TASK_STACK_SIZE         8192
#define MAVLINK_TASK_PRIORITY           4
#define DISPLAY_TASK_STACK_SIZE         4096
#define DISPLAY_TASK_PRIORITY           3
```

---

## 9. Фази реалізації

### Фаза 1 — PoC: Bearing (один AD8302, горизонт)
**Ціль:** Підтвердити що VPHS → кут працює на реальному обладнанні.

- Підключити AD8302 #1 (ліво-права пара антен) → ADC CH1
- Реалізувати модуль `bearing/` з `vphs_to_angle()`
- `SENTINEL_ELEVATION_ENABLED = 0` — elevation код присутній але неактивний
- SSD1306 показує θ_azimuth в реальному часі
- Тест: повернути антену на відомі кути (0°, ±30°, ±60°), перевірити відповідність
- Калібрування `SENTINEL_VPHS_ZERO_MV`

### Фаза 2 — MAVLink з'єднання
**Ціль:** ESP32-C6 бачить FC і може переключати режим.

- Реалізувати модуль `mavlink/`: heartbeat loop, отримання heartbeat від FC
- Команда `MAV_CMD_DO_SET_MODE` → `GUIDED_NOGPS`
- Верифікація через Mission Planner: FC бачить ESP32 як GCS
- Watchdog: втрата з'єднання → IDLE

### Фаза 3 — Navigator + автономний горизонтальний трекінг
**Ціль:** Дрон автономно стежить за RF джерелом по горизонту.

- Реалізувати модуль `navigator/`: стан машина + P-controller
- `SET_ATTITUDE_TARGET` @ 20 Hz: roll = Kp × azimuth, pitch = cruise_pitch
- Тест на мотузці: дрон стежить за ручним передавачем
- Підбір `SENTINEL_KP`

### Фаза 4 — INTERCEPT + опціональний VMAG
**Ціль:** Дрон переходить в режим перехоплення при наближенні.

- Стан INTERCEPT: max pitch + max thrust
- Опціонально: `SENTINEL_VMAG_ENABLED = 1`, підбір `VMAG_INTERCEPT_THRESHOLD`
- Польові тести з нерухомою ціллю

### Фаза 5 — Elevation + надійність
**Ціль:** Повна 3D навігація і готовність до польових умов.

- Підключити AD8302 #2 (передня-задня пара), `SENTINEL_ELEVATION_ENABLED = 1`
- Перевірити elevation трекінг
- Підтримка двох частот: профіль `SENTINEL_FREQ_HZ` (900 MHz / 2.4 GHz)
- Логування через MAVLink `STATUSTEXT`
- Стрес-тест: втрата сигналу, перешкоди, різні дистанції

---

## 10. Залежності (idf_component.yml)

```yaml
dependencies:
  idf: '>=5.3.0'
  components/ssd1306: ^1.1.2
  # mavlink/c_library_v2 — додати вручну як components/mavlink
```

---

## 11. Поточний стан коду

| Компонент | Стан |
|-----------|------|
| `adc/` | Реалізовано (continuous mode, CH1+CH2) |
| `display/` | Реалізовано (SSD1306, текст) |
| `bearing/` | Не реалізовано |
| `navigator/` | Не реалізовано |
| `mavlink/` | Не реалізовано |
| `app_config.h` | Частково (ADC/display константи, потребує розширення) |

### Передумови для старту Фази 1

Перед початком реалізації `bearing/` необхідно додати до `app_config.h`:
- `SENTINEL_FREQ_HZ`
- `SENTINEL_ANTENNA_SPACING_M`
- `SENTINEL_VPHS_ZERO_MV`
- `SENTINEL_ELEVATION_ENABLED`
- `SENTINEL_SIGNAL_THRESHOLD`
