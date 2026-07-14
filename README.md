# outputwrite

GPIO / relay / ADC HAL helpers and a non-blocking status-LED indicator for
Espressif chips. Created by Aram Vartanyan.

| Module | Version | Purpose |
|---|---|---|
| `outputwrite.c/.h` | 1.1.0 | GPIO init (`ioInit`, `ioInitPdown`), input read, output write with software polarity (`OutputWrite`), ADC helpers |
| `led_indicator.c/.h` | 1.6.4 | Non-blocking single-LED indicator engine (LEDC PWM): steady base, breathe, blink, heartbeat, transient flashes, fixed patterns (`ledOtaStatus`, `ledIdentify`, ...) |

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

Persistent backgrounds (steady / breathe / blink / heartbeat) are mutually
exclusive; transient effects (`ledFlash`, `ledFlashGroups`) overlay the
steady base and are suppressed while a continuous background runs.
`ledFlashBlocking` is for terminal paths (e.g. right before a factory
reset), where the pattern must complete before the reset call.

### `gpioOnly` — zero-cost mode for a secondary status LED

Set `LedConfig.gpioOnly = true` and the engine drives the pin with plain
`gpio_set_level` instead of PWM. on/off, blink and flash behave identically
(they only ever use full-on/full-off); only breathing and partial-brightness
`ledSteady` collapse to on/off and should not be used in this mode.

Why it matters, especially on the ESP8266: the SDK's software PWM is
edge-scheduled — its timer ISR fires at every waveform edge, i.e. about
twice per period *regardless of duty resolution*. So the number of steps
(200, 1024, ...) does NOT change the CPU cost; only the frequency does
(~2·freq_hz interrupts/second). Worse, the ISR reschedules itself every
period even at a static 0 %/100 % level, so a solid or slowly-blinking LED
still costs ~2·freq_hz interrupts/second for nothing. `gpioOnly` removes the
PWM entirely: the pin is a bare GPIO, and the only remaining work is the
FreeRTOS software timer that toggles it during an active blink/flash (a few
callbacks per second, none while steady). For an accessory whose main job is
elsewhere (a window-cover motor, a lock), this keeps the indicator off the
CPU budget. Use full PWM mode only where real dimming/breathing is wanted
(e.g. a lightbulb, where the LED is the primary function).

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
