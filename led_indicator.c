/*
 led_indicator — non-blocking single-LED status indicator
 Version: 1.7.0  (updated 2026-07-15)
 Created by Aram Vartanyan, (C) 2026
*/

#include "led_indicator.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "led_ind";

/* ---- PWM resolution / engine tunables (breathe-related + fallbacks) -------- */
#define LED_PWM_RES        LEDC_TIMER_10_BIT
#define LED_MAX_DUTY       (1u << 10)               /* 1024 = full scale (LEDC 100%) */
#ifdef CONFIG_IDF_TARGET_ESP8266
/* The ESP8266 compatibility driver takes 0..8196 regardless of the resolution,
   and its software PWM is proven stable at 1 kHz. See README.md. */
#define LED_DRV_FULL       8196u
#define LED_DEFAULT_FREQ   1000
#else
#define LED_DRV_FULL       LED_MAX_DUTY
#define LED_DEFAULT_FREQ   5000
#endif
#define DEFAULT_PERIOD_MS  60                       /* ledFlash half-phase fallback */
#define DEFAULT_BLINK_MS   500                      /* ledBlink half-period fallback */

#define BREATHE_TICK_MS        25
#define BREATHE_PERIOD_MS      5000                 /* full inhale+exhale (~5 s) */
#define BREATHE_STEPS          (BREATHE_PERIOD_MS / BREATHE_TICK_MS)  /* 200 */
#define BREATHE_HALF           (BREATHE_STEPS / 2)                    /* 100 */
#define BREATHE_BOTTOM_HOLD_MS 400                  /* dwell at the dark bottom of a breath */

/* BG = persistent background mode; FX = transient effect over it; PH = flash phase */
enum { BG_STEADY = 0, BG_BREATHE, BG_CYCLE };
enum { FX_NONE = 0, FX_FLASH };
enum { PH_OPP = 0, PH_BASE, PH_GAP };

/* All module state in one object (was ~17 scattered file-scope statics). */
static struct {
    LedConfig     cfg;
    TimerHandle_t timer;
    uint32_t      last_drv;       /* last duty written to LEDC (skip repeats)      */
    uint8_t       pwm_active;     /* LEDC currently drives the pin (breathe only)  */
    uint8_t       inited;
    uint8_t       base_on;        /* steady level requested by the app             */
    uint8_t       bg;             /* BG_STEADY / BG_BREATHE / BG_CYCLE             */
    uint8_t       fx;             /* FX_NONE / FX_FLASH                            */
    uint16_t      breathe_i;      /* breathe phase index 0..BREATHE_STEPS-1        */
    /* cycle (blink / heartbeat) */
    uint8_t       cycle_use_base; /* true: HIGH=opposite,LOW=base; false: HIGH=on,LOW=off */
    uint8_t       cycle_high;     /* current cycle phase                           */
    uint16_t      cycle_on_ms;
    uint16_t      cycle_off_ms;
    /* flash (finite, grouped) */
    uint8_t       phase;          /* PH_OPP / PH_BASE / PH_GAP                     */
    uint8_t       groups_left;
    uint8_t       blinks_left;
    uint8_t       blinks_per_grp;
    uint16_t      flash_period;
    uint16_t      gap_ms;
} s;

/* ---- low-level output ------------------------------------------------------ */
/* GPIO on/off by default; LEDC duty only while breathing (s.pwm_active). */
static void apply_duty(uint32_t logical)
{
    if (logical > LED_MAX_DUTY) {
        logical = LED_MAX_DUTY;
    }

    if (!s.pwm_active) {                       /* plain GPIO: past half-scale = "on" */
        bool on = logical > (LED_MAX_DUTY / 2);
        gpio_set_level((gpio_num_t)s.cfg.gpio, (on != s.cfg.activeLow) ? 1 : 0);
        return;
    }

    uint32_t d = s.cfg.activeLow ? (LED_MAX_DUTY - logical) : logical; /* software polarity */
    d = d * LED_DRV_FULL / LED_MAX_DUTY;                     /* engine -> driver scale */
    if (d == s.last_drv) {                     /* unchanged: skip (avoids 8266 fade-queue err) */
        return;
    }
    s.last_drv = d;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s.cfg.ledcChannel, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s.cfg.ledcChannel);
}

static void apply_base(void)     { apply_duty(s.base_on ? LED_MAX_DUTY : 0); }
static void apply_opposite(void) { apply_duty(s.base_on ? 0 : LED_MAX_DUTY); }

/* perceptually smooth (squared) breathe ramp, integer math only */
static uint32_t breathe_duty(uint16_t i)
{
    uint16_t pos = (i < BREATHE_HALF) ? i : (uint16_t)(BREATHE_STEPS - i); /* 0..HALF */
    return (uint32_t)LED_MAX_DUTY * pos * pos / (BREATHE_HALF * BREATHE_HALF);
}

static void rearm(uint32_t ms) { xTimerChangePeriod(s.timer, pdMS_TO_TICKS(ms), 0); } /* also (re)starts */
static void stop_timer(void)   { xTimerStop(s.timer, 0); }

/* ---- LEDC lifecycle: active only for the breathe window -------------------- */
static esp_err_t ledc_pwm_start(void)          /* route the pin GPIO -> LEDC */
{
    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LED_PWM_RES,
        .timer_num       = (ledc_timer_t)s.cfg.ledcTimer,
        .freq_hz         = s.cfg.freqHz,
#ifndef CONFIG_IDF_TARGET_ESP8266
        .clk_cfg         = LEDC_AUTO_CLK,          /* absent on ESP8266 — see README.md */
#endif
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config: %d", err);
        return err;
    }

    ledc_channel_config_t ccfg = {
        .gpio_num   = s.cfg.gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)s.cfg.ledcChannel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = (ledc_timer_t)s.cfg.ledcTimer,
#ifdef CONFIG_IDF_TARGET_ESP8266
        .duty       = 0,                           /* validated vs the period on the 8266 */
#else
        .duty       = s.cfg.activeLow ? LED_MAX_DUTY : 0,   /* start OFF */
#endif
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config: %d", err);
        return err;
    }

#ifdef CONFIG_IDF_TARGET_ESP8266
    /* Mandatory pwm_init on the 8266 shim before the first ledc_set_duty. */
    err = ledc_fade_func_install(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_fade_func_install: %d", err);
        return err;
    }
    esp_log_level_set("ledc", ESP_LOG_WARN);       /* silence the per-duty INFO flood */
#endif
    return ESP_OK;
}

static void ledc_pwm_stop(void)                /* route the pin LEDC -> GPIO (reclaim it) */
{
    /* Park the PWM output at the OFF level, then hand the pin back to plain GPIO
       so the on/off engine drives it again. The pwm/fade driver stays installed
       — cheap, and it avoids the risky uninstall on the ESP8266 shim. */
    ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s.cfg.ledcChannel, s.cfg.activeLow ? 1 : 0);
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s.cfg.gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

/* ---- background: apply the current frame and (re)arm/stop the timer -------- */
static void run_background(void)
{
    if (s.bg == BG_BREATHE) {
        apply_duty(breathe_duty(s.breathe_i));
        rearm(BREATHE_TICK_MS);
    } else if (s.bg == BG_CYCLE) {
        if (s.cycle_use_base) {
            if (s.cycle_high) {
                apply_opposite();
            } else {
                apply_base();
            }
        } else {
            apply_duty(s.cycle_high ? LED_MAX_DUTY : 0);
        }
        rearm(s.cycle_high ? s.cycle_on_ms : s.cycle_off_ms);
    } else {                                       /* BG_STEADY */
        apply_base();
        stop_timer();
    }
}

/* Timer callback — sole owner of the running state machine (timer-service task). */
static void led_timer_cb(TimerHandle_t t)
{
    (void)t;

    if (s.fx == FX_FLASH) {
        if (s.phase == PH_OPP) {                    /* opposite half done -> base half */
            s.phase = PH_BASE;
            apply_base();
            rearm(s.flash_period);
        } else if (s.phase == PH_BASE) {            /* one blink done */
            if (s.blinks_left > 0) {
                s.blinks_left--;
            }
            if (s.blinks_left > 0) {
                s.phase = PH_OPP;
                apply_opposite();
                rearm(s.flash_period);
            } else {                                /* group done */
                if (s.groups_left > 0) {
                    s.groups_left--;
                }
                if (s.groups_left == 0) {           /* flash done -> back to background */
                    s.fx = FX_NONE;
                    run_background();
                } else {
                    s.blinks_left = s.blinks_per_grp;
                    if (s.gap_ms > 0) {
                        s.phase = PH_GAP;
                        apply_base();
                        rearm(s.gap_ms);
                    } else {
                        s.phase = PH_OPP;
                        apply_opposite();
                        rearm(s.flash_period);
                    }
                }
            }
        } else {                                    /* PH_GAP done -> next group */
            s.phase = PH_OPP;
            apply_opposite();
            rearm(s.flash_period);
        }
    } else if (s.bg == BG_BREATHE) {                /* breathe step (special: bottom-hold) */
        s.breathe_i = (uint16_t)((s.breathe_i + 1) % BREATHE_STEPS);
        apply_duty(breathe_duty(s.breathe_i));
        rearm(s.breathe_i == 0 ? BREATHE_BOTTOM_HOLD_MS : BREATHE_TICK_MS);
    } else {                                        /* cycle: advance; steady: settle+stop */
        if (s.bg == BG_CYCLE) {
            s.cycle_high = !s.cycle_high;
        }
        run_background();
    }
}

/* ---- engines --------------------------------------------------------------- */
/* Finite, transient: 'groups' x { 'blinks' blinks @ periodMs half-phase; gapMs pause }.
   Suppressed while a persistent background (breathe/cycle) owns the LED. */
static void start_flash(uint8_t groups, uint8_t blinks, uint16_t periodMs, uint16_t gapMs)
{
    if (!s.inited || groups == 0 || blinks == 0) {
        return;
    }
    if (periodMs == 0) {
        periodMs = DEFAULT_PERIOD_MS;
    }
    if (s.bg != BG_STEADY) {            /* breathe/cycle own the LED -> ignore flashes */
        return;
    }
    s.fx             = FX_FLASH;
    s.groups_left    = groups;
    s.blinks_per_grp = blinks;
    s.blinks_left    = blinks;
    s.flash_period   = periodMs;
    s.gap_ms         = gapMs;
    s.phase          = PH_OPP;
    apply_opposite();                   /* first (opposite) half */
    rearm(periodMs);
}

static void cycle_stop(void)            /* BG_CYCLE -> steady (no-op otherwise) */
{
    if (s.bg != BG_CYCLE) {             /* leaves a running breathe untouched */
        return;
    }
    s.bg = BG_STEADY;
    if (s.fx == FX_NONE) {
        run_background();
    }
}

/* Continuous cycle control shared by ledBlink and ledHeartbeat. on=true starts
   it, on=false stops it (-> steady). HIGH phase (on_ms) <-> LOW phase (off_ms):
   use_base=true  -> HIGH=opposite-of-base, LOW=base   (heartbeat pulse)
   use_base=false -> HIGH=full on,          LOW=off     (blink, independent of base) */
static void set_cycle(bool on, bool use_base, uint16_t on_ms, uint16_t off_ms)
{
    if (!s.inited) {
        return;
    }
    if (!on) {
        cycle_stop();
        return;
    }
    if (s.bg == BG_BREATHE) {           /* breathe owns the LED -> ignore */
        return;
    }
    s.bg             = BG_CYCLE;
    s.cycle_use_base = use_base;
    s.cycle_on_ms    = on_ms;
    s.cycle_off_ms   = off_ms;
    s.cycle_high     = true;            /* start on the HIGH phase */
    if (s.fx == FX_NONE) {
        run_background();
    }
}

/* ---- public API ------------------------------------------------------------ */
esp_err_t ledInit(const LedConfig *cfg)
{
    if (!cfg) {
        return ESP_ERR_INVALID_ARG;
    }
    s.cfg = *cfg;
    if (s.cfg.freqHz == 0) {
        s.cfg.freqHz = LED_DEFAULT_FREQ;
    }

    /* Always begin on plain GPIO; LEDC is set up lazily by ledBreathe(). */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s.cfg.gpio),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    s.pwm_active = false;

    /* Safe to call ledInit more than once per boot (e.g. a boot-time OTA
       indicator re-configured for the main app afterwards): reuse the timer. */
    if (s.timer) {
        xTimerStop(s.timer, 0);
    } else {
        s.timer = xTimerCreate("led", pdMS_TO_TICKS(BREATHE_TICK_MS), pdFALSE, NULL, led_timer_cb);
        if (!s.timer) {
            ESP_LOGE(TAG, "alloc failed");
            return ESP_ERR_NO_MEM;
        }
    }

    s.base_on  = false;
    s.bg       = BG_STEADY;
    s.fx       = FX_NONE;
    s.last_drv = 0xFFFFFFFFu;
    apply_duty(0);                       /* explicit OFF (GPIO) */
    s.inited = true;
    ESP_LOGI(TAG, "init gpio=%d activeLow=%d freq=%uHz v%s",
             s.cfg.gpio, (int)s.cfg.activeLow, (unsigned)s.cfg.freqHz, LED_INDICATOR_VERSION);
    return ESP_OK;
}

void ledSteady(bool on)
{
    if (!s.inited) {
        return;
    }
    s.base_on = on;
    if (s.fx == FX_NONE && s.bg == BG_STEADY) {   /* immediate when idle */
        apply_base();
    }
}

void ledBreathe(bool on)
{
    if (!s.inited) {
        return;
    }
    if (on) {
        if (!s.cfg.gpioOnly && !s.pwm_active) {   /* spin LEDC up on demand */
            if (ledc_pwm_start() == ESP_OK) {
                s.pwm_active = true;
                s.last_drv = 0xFFFFFFFFu;
            }
        }
        s.bg = BG_BREATHE;
        s.breathe_i = 0;
        if (s.fx == FX_NONE) {
            run_background();
        }
    } else {
        s.bg = BG_STEADY;
        stop_timer();
        if (s.pwm_active) {                       /* hand the pin back to GPIO */
            ledc_pwm_stop();
            s.pwm_active = false;
            s.last_drv = 0xFFFFFFFFu;
        }
        if (s.fx == FX_NONE) {
            apply_base();
        }
    }
}

void ledBlink(bool on, uint16_t periodMs)
{
    if (periodMs == 0) {
        periodMs = DEFAULT_BLINK_MS;
    }
    set_cycle(on, false, periodMs, periodMs);   /* full <-> off, symmetric */
}

void ledHeartbeat(bool on)
{
    set_cycle(on, true, 80, 1920);              /* opposite-of-base pulse 80 / 1920 ms */
}

void ledFlash(uint8_t times, uint16_t periodMs)
{
    start_flash(1, times, periodMs, 0);
}

void ledFlashGroups(uint8_t groups, uint8_t blinksPerGroup, uint16_t periodMs, uint16_t gapMs)
{
    start_flash(groups, blinksPerGroup, periodMs, gapMs);
}

void ledFlashBlocking(uint8_t times, uint16_t periodMs)
{
    if (!s.inited || times == 0) {
        return;
    }
    if (periodMs == 0) {
        periodMs = DEFAULT_PERIOD_MS;
    }
    /* Terminal/pre-reboot path: drive synchronously (the timer/task is soon gone).
       The base level is frozen for the whole sequence. */
    s.fx = FX_NONE;
    stop_timer();
    bool base = s.base_on;
    for (uint8_t i = 0; i < times; i++) {
        apply_duty(base ? 0 : LED_MAX_DUTY);          /* opposite of base */
        vTaskDelay(pdMS_TO_TICKS(periodMs));
        apply_duty(base ? LED_MAX_DUTY : 0);          /* base */
        vTaskDelay(pdMS_TO_TICKS(periodMs));
    }
}

/* ---- convenience wrappers: fixed patterns, unified across devices ---------- */
void ledTxFlash(void)      { ledFlash(1, 80); }
void ledOtaStatus(bool on) { ledBlink(on, 500); }
void ledOverheated(void)   { ledFlashGroups(3, 2, 100, 200); }
void ledResetNetwork(void) { ledFlash(3, 80); }
void ledNotifyFlash(void)  { ledFlash(2, 100); }
void ledIdentify(void)     { ledFlashGroups(3, 2, 200, 300); }
void ledResetOk(void)      { ledFlashBlocking(4, 80); }
