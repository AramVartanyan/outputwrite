# outputwrite

GPIO / relay / ADC HAL helpers and a non-blocking status-LED indicator for
Espressif chips. Created by Aram Vartanyan.

| Module | Version | Purpose |
|---|---|---|
| `outputwrite.c/.h` | 1.1.0 | GPIO init (`ioInit`, `ioInitPdown`), input read, output write with software polarity (`OutputWrite`), ADC helpers |
| `led_indicator.c/.h` | 1.7.0 | Non-blocking single-LED indicator: GPIO on/off by default, LEDC PWM only for breathing (on-demand); steady base, breathe, blink, heartbeat, transient flashes, fixed patterns (`ledOtaStatus`, `ledIdentify`, ...) |

## Platform support

| Platform | outputwrite | led_indicator |
|---|---|---|
| ESP-IDF 5.x (ESP32 family) | full (incl. calibrated ADC via `esp_adc`) | full |
| ESP-IDF 4.x | full (legacy `esp_adc_cal`) | full |
| ESP8266_RTOS_SDK v3.4 | GPIO part (ADC helpers compile out) | full, since 1.5.3 |

The version gating is automatic: `outputwrite.h` selects the ADC API by
`ESP_IDF_VERSION` (the ESP8266 SDK reports 3.4, so the ADC helpers are
excluded there — that chip's single TOUT ADC is read directly by the
application instead).

## ESP8266_RTOS_SDK notes

- **Component dependencies differ.** The ESP8266 SDK has no separate
  `driver` / `esp_adc` components — the gpio/adc/ledc drivers live inside
  the `esp8266` component. `CMakeLists.txt` therefore registers the library
  per target:
  - `esp8266`: `REQUIRES esp8266 freertos`
  - otherwise: `REQUIRES driver esp_adc freertos`
- **LEDC:** the ESP8266 "ledc" driver is a software-PWM compatibility layer
  with an ESP32-like API, with three quirks handled in `led_indicator.c`:
  - `ledc_timer_config_t` has no `.clk_cfg` member (guarded with
    `#ifndef CONFIG_IDF_TARGET_ESP8266`);
  - the duty scale of `ledc_set_duty` is fixed at 0..8196 regardless of the
    configured resolution (the ESP32 uses timer-resolution units), so the
    engine rescales at its single write point (`LED_DRV_FULL`);
  - `ledc_channel_config` validates the initial `.duty` against the PWM
    period in MICROSECONDS (e.g. 200 at 5 kHz), so 0 is passed there and the
    real level is applied immediately afterwards through `ledc_set_duty`;
  - `ledc_fade_func_install()` is MANDATORY after the channel configuration:
    on this driver it is what actually initialises the underlying PWM object
    (`pwm_init`), and the first `ledc_set_duty` crashes (NULL dereference in
    `pwm_get_duty`) without it. On the ESP32 the call is optional and only
    enables hardware fades.
  - the driver's period is `1e6 / freq_hz` units, so the PWM frequency also
    sets the duty resolution; the default is 1 kHz on the ESP8266 (1000
    units, and easy on the software-PWM ISR — the `led_dim` reference uses
    the same), vs 5 kHz on the ESP32.
  Init order (both platforms): `ledc_timer_config` -> `ledc_channel_config`
  -> `ledc_fade_func_install`, matching the `led_dim` reference.
- Keep this library **canonical**: it is shared between the ESP32 tree
  (`new_project/projects/common`) and the ESP8266 tree
  (`esp8266/project/common`). Platform differences are expressed as
  conditionals in the same files — never as diverging copies.

## led_indicator design

Framework-agnostic: the core knows nothing about Matter/HomeKit. The
application maps its own state to the neutral calls of the public API
(e.g. a Matter lock maps "unlocked" -> `ledSteady(true)`).

esp-idf 4.x and 5.x compatible: software polarity (`activeLow`) + software
breathing/blink ramps; only universal LEDC calls are used.

Concurrency: the FreeRTOS one-shot timer callback is the SOLE owner of the
live state machine. Public API functions only set the desired state and
(re)arm the timer — no mutex is used, so the shared timer-service task is
never blocked. Call the public API from a single context (the app task).

Naming: this library's own functions use CamelCase (`ledInit`, `ledFlash`,
...) to stand apart from the esp-idf lower_snake_case style.

Nothing is read from Kconfig inside the library: each project passes its
own pins/options via `LedConfig`, so the module stays fully reusable.

### Drive model

The LED is a plain **GPIO on/off** output by default. **LEDC PWM is used only
for `ledBreathe()`** — spun up on demand when breathing starts and torn back
down to GPIO (`ledc_stop` + pin reclaim) when it stops, so no PWM timer/ISR runs
outside the breathe window. Every other indication (steady, flash, blink,
heartbeat) is pure on/off.

Why it matters, especially on the ESP8266: the SDK's software PWM is
edge-scheduled — its timer ISR fires ~twice per period *regardless of duty
resolution*, and reschedules itself every period even at a static level. Keeping
PWM to the breathe window alone means a solid or blinking indicator costs only
the FreeRTOS software timer that toggles it (a few callbacks per second, none
while steady) — the indicator stays off the CPU budget of an accessory whose
main job is elsewhere (a window-cover motor, a lock).

`LedConfig.gpioOnly = true` forbids PWM entirely: `ledBreathe()` then degrades to
an on/off square wave instead of a smooth fade. Use it for a secondary indicator
that never needs to breathe. (Real dimming across the whole range belongs in a
lightbulb driver, not a status indicator.)

### Engines

Two internal engines; every public function is a thin wrapper over one of them:
- `start_flash()` — finite, transient pattern (N blinks, optionally grouped with
  a gap), then back to the background. Wrappers: `ledFlash`, `ledFlashGroups`,
  `ledTxFlash`, `ledOverheated`, `ledResetNetwork`, `ledNotifyFlash`,
  `ledIdentify`.
- `start_cycle()` — continuous pattern until stopped. Wrappers: `ledBlink`
  (full↔off, symmetric), `ledHeartbeat` (opposite-of-base pulse, 80/1920 ms),
  `ledOtaStatus` (a slow blink).

The persistent backgrounds — **steady**, **breathe**, **cycle** (blink/heartbeat)
— are mutually exclusive. Transient flashes overlay the steady base only and are
suppressed while a background runs. **While breathing, the library ignores
flash/cycle calls** (breathing owns the LED). `ledFlashBlocking` (wrapped by
`ledResetOk`) is the only blocking call — for terminal paths such as just before
a factory reset, where the pattern must complete before the reset call.

## Typical usage (window-cover firmware)

```c
LedConfig led_cfg = { .gpio = CONFIG_LED_GPIO, .activeLow = true };
ledInit(&led_cfg);
ledSteady(wifi_connected);      // WiFi status as the steady base
ledIdentify();                  // HomeKit identify
ledOtaStatus(true);             // OTA in progress (continuous slow blink)
ledFlashBlocking(3, 100);       // just before hap_reset_to_factory()
```

## Changelog

- **led_indicator 1.7.0** (2026-07-15): internal rearchitecture — public API and
  every blink pattern/timing unchanged. (1) **Drive model:** the LED is now plain
  GPIO on/off by default; **LEDC PWM is used only for `ledBreathe`**, spun up on
  demand and released back to GPIO (`ledc_stop` + pin reclaim) when breathing
  stops — no PWM timer/ISR runs outside the breathe window. A new `pwm_active`
  flag routes `apply_duty` (GPIO vs LEDC). (2) **Two engines, everything a thin
  wrapper:** `start_flash` (finite/transient) and the new `start_cycle`
  (continuous); `ledBlink`/`ledHeartbeat` are now wrappers of `start_cycle`, so
  the `BG_BLINK`/`BG_HEARTBEAT` background modes collapse into a single
  `BG_CYCLE`. `run_background()` is the single source of truth for "apply frame +
  (re)arm". (3) **State** consolidated from ~17 scattered file-scope statics into
  one `s` struct (enums right-sized to `uint8_t`). (4) **Constants:** the ~20
  per-pattern `#define`s (all coincidental duplicates) are gone — literals inline
  in the one-line wrappers; only the engine/computed constants remain. (5) **New
  guarantee:** while breathing, `ledBlink`/`ledFlash`/… calls are ignored
  (breathing owns the LED). (6) **Removed the unused `onPercent` `LedConfig`
  field** — nobody dimmed the steady LED; the "on" level and breathe peak are now
  always full scale. Projects that set `.onPercent` in their initializer must drop
  that line (it becomes a compile error — loud, not silent). **Requires hardware
  validation** — the ESP8266 LEDC teardown/re-init cycle is not yet bench-tested
  (ESP32 is routine).
- **led_indicator 1.6.4** (2026-07-15): `apply_duty` now skips the write when the
  driver duty is unchanged from the last one it pushed (`s_last_drv` cache). On
  the ESP8266 the `ledc` driver queues each duty update through a size-1 fade
  queue drained by a background task; two back-to-back identical writes at boot
  (`ledInit` OFF, then the breathe effect's first frame) raced and the second one
  logged `E ledc: xQueueSend err`. Skipping the redundant write removes that
  benign boot error and saves needless PWM churn on every SDK. `ledInit`
  invalidates the cache (`s_last_drv = 0xFFFFFFFF`) since it reconfigures the
  channel.
- **led_indicator 1.6.3** (2026-07-14): breathing slowed 2x (`BREATHE_PERIOD_MS`
  2500 -> 5000, ~5 s per breath) so it looks more natural; the tick rate is
  unchanged, so no extra load. Also silences the ESP8266 `ledc` driver's
  per-duty INFO spam ("ledc: channel_num ... step_duty ...") via
  `esp_log_level_set("ledc", ESP_LOG_WARN)` in `ledInit` — it flooded the
  console during breathing.
- **led_indicator 1.6.2** (2026-07-14): breathing now rests briefly at the dark
  bottom of each cycle (`BREATHE_BOTTOM_HOLD_MS`, 400 ms) so it reads like a
  real breath — inhale, exhale, pause. The lit top needs no dwell: the squared
  ramp is flat there, so the LED already lingers bright.
- **led_indicator 1.6.1** (2026-07-14): `ledInit` is safe to call more than
  once per boot (reuses the software timer) — e.g. a boot-time OTA indicator
  reconfigured for the main application afterwards.
- **led_indicator 1.6.0** (2026-07-14): `gpioOnly` mode — drive the pin with
  plain GPIO (no PWM, zero recurring ISR) for secondary status LEDs that only
  need on/off + blink/flash.
- **led_indicator 1.5.3** (2026-07-14): ESP8266 default PWM frequency 1 kHz
  (proven-stable per the `led_dim` reference; finer duty than 5 kHz).
- **led_indicator 1.5.2** (2026-07-14): ESP8266 boot-loop fix — mandatory
  `ledc_fade_func_install()` (it performs `pwm_init` on this driver).
- **led_indicator 1.5.1** (2026-07-14): ESP8266 duty-scale fix (0..8196
  driver scale; `ledc_channel_config` period-validation workaround).
- **led_indicator 1.5.0** (2026-07-14): ESP8266_RTOS_SDK v3.4 support
  (`.clk_cfg` guard); dual-target `CMakeLists.txt`.
- **led_indicator 1.4.0** (2026-06-29): framework-agnostic vocabulary,
  grouped flashes, heartbeat.
- **outputwrite 1.1.0** (2026-06-22): ADC helpers for ESP-IDF 5.x.
