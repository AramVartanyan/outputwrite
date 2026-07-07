/*
 led_indicator — non-blocking single-LED status indicator (LEDC PWM)
 Version: 1.0.0  (created 2026-06-19)
 Created by Aram Vartanyan, (C) 2026
 */

#ifndef _LED_INDICATOR_H_
#define _LED_INDICATOR_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LED_INDICATOR_VERSION "1.0.0"

/* Configuration passed at init. Nothing is read from Kconfig inside the
   library — each project supplies its own pins/options, so the module stays
   fully reusable. */
typedef struct {
    int      gpio;          /* LED GPIO number */
    bool     activeLow;     /* true: LED lights on LOW level (software inverted) */
    uint32_t freqHz;        /* PWM frequency; 0 -> 5000 Hz */
    uint8_t  onPercent;     /* steady "on" brightness 1..100; 0 -> 100 */
    uint8_t  ledcTimer;     /* LEDC timer index (0..3) */
    uint8_t  ledcChannel;   /* LEDC channel index */
} LedConfig;

/* Initialize LEDC PWM + the indicator engine. LED starts OFF. */
esp_err_t ledInit(const LedConfig *cfg);

/* ---- persistent backgrounds (bool-controlled, mutually exclusive) ---- */

/* Steady base level. The application maps its own state -> on/off.
   Applied immediately when no breathing/blink/flash effect is running. */
void ledSteady(bool on);

/* Continuous breathing (smooth fade), e.g. while the commissioning window is
   open. on=false returns to the steady base level. */
void ledBreathe(bool on);

/* Continuous square on/off blink at `periodMs` half-period, until turned off
   — a persistent "busy" background (e.g. OTA in progress). Full bright <-> off
   (independent of the steady base). on=false returns to the steady base.
   periodMs=0 -> 500 ms. */
void ledBlink(bool on, uint16_t periodMs);

/* ---- transient effects (overlay the background, then return) ---- */

/* Blink `times` times, toggling around the current steady base
   (opposite-of-base -> base). `periodMs` is each half-phase (full blink =
   2 * periodMs). Non-blocking. Ignored while a continuous background
   (breathe/blink) is active, so it never disturbs them.
   times=0 -> ignored; periodMs=0 -> 60 ms. */
void ledFlash(uint8_t times, uint16_t periodMs);

/* Grouped/repeated blink, non-blocking: repeat `groups` times
   { `blinksPerGroup` blinks at `periodMs` half-phase; then a `gapMs` pause }.
   No pause after the last group. gapMs=0 -> groups run back-to-back.
   Same suppression-during-background rule as ledFlash. */
void ledFlashGroups(uint8_t groups, uint8_t blinksPerGroup, uint16_t periodMs, uint16_t gapMs);

/* Same as ledFlash() but BLOCKING (busy until done). Intended for
   terminal/pre-reboot paths (e.g. factory reset). */
void ledFlashBlocking(uint8_t times, uint16_t periodMs);

/* ---- Convenience wrappers: fixed patterns for a unified vocabulary across
   projects. Call the primitives above directly for custom counts/speeds. ---- */
void ledTxFlash(void);      /* comms activity:        1 blink (80 ms) */
void ledOtaStatus(bool on); /* OTA in progress:       continuous slow blink (500 ms) while on */
void ledOverheated(void);   /* over-temperature:      3x (2 blinks @100 ms, 200 ms gap) */
void ledResetNetwork(void); /* network reset:         3 fast blinks (80 ms) */
void ledNotifyFlash(void);  /* generic notification:  2 blinks (100 ms) — e.g. fabric removed */
void ledIdentify(void);     /* identify:              3x (2 blinks @200 ms, 300 ms gap) */
void ledResetOk(void);      /* factory reset accepted: 4 fast blinks (80 ms), BLOCKING */

#ifdef __cplusplus
}
#endif

#endif /* _LED_INDICATOR_H_ */
