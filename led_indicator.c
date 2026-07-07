/*
 led_indicator — non-blocking single-LED status indicator (LEDC PWM)
 Version: 1.3.0  (updated 2026-06-28)
 Created by Aram Vartanyan, (C) 2026
 */

#include "led_indicator.h"

#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "led_ind";

/* ---- PWM resolution / continuous-effect tunables ---- */
#define LED_PWM_RES        LEDC_TIMER_10_BIT
#define LED_MAX_DUTY       (1u << 10)               /* 1024 = true full scale (LEDC 100%) */
#define LED_DEFAULT_FREQ   5000
#define DEFAULT_PERIOD_MS  60                       /* ledFlash period fallback */
#define DEFAULT_BLINK_MS   500                      /* ledBlink period fallback */

#define BREATHE_TICK_MS    25
#define BREATHE_PERIOD_MS  2500
#define BREATHE_STEPS      (BREATHE_PERIOD_MS / BREATHE_TICK_MS)  /* 100 */
#define BREATHE_HALF       (BREATHE_STEPS / 2)                    /* 50  */

#define HEARTBEAT_ON_MS    80     /* "alive" pulse width */
#define HEARTBEAT_OFF_MS   1920   /* gap; 80 + 1920 = 2 s period */

/* ---- fixed patterns for the convenience wrappers (unified vocabulary) ---- */
#define TX_BLINKS          1
#define TX_MS              80
#define OTA_PERIOD_MS      500
#define OVERHEAT_GROUPS    3
#define OVERHEAT_BLINKS    2
#define OVERHEAT_MS        100
#define OVERHEAT_GAP_MS    200
#define RESETNET_BLINKS    3
#define RESETNET_MS        80
#define NOTIFY_BLINKS      2
#define NOTIFY_MS          100
#define IDENTIFY_GROUPS    3      /* accessoryIdentify: 2 blinks @200 ms, repeated 3x */
#define IDENTIFY_BLINKS    2
#define IDENTIFY_MS        200
#define IDENTIFY_GAP_MS    300
#define RESET_BLINKS       4
#define RESET_MS           80

/* persistent background mode */
enum { BG_STEADY = 0, BG_BREATHE = 1, BG_BLINK = 2, BG_HEARTBEAT = 3 };
/* transient effect (overlays the background) */
enum { FX_NONE = 0, FX_FLASH = 1 };
/* flash sub-phase */
enum { PH_OPP = 0, PH_BASE = 1, PH_GAP = 2 };

static LedConfig s_cfg;
static uint32_t  s_on_duty = LED_MAX_DUTY;   /* duty for "on" (scaled by onPercent) */
static bool      s_inited  = false;

static TimerHandle_t     s_timer = NULL;

static bool s_base_on = false;     /* steady level requested by the app */
static int  s_bg      = BG_STEADY; /* background mode */
static int  s_fx      = FX_NONE;   /* active transient effect */

static uint16_t s_breathe_i    = 0;   /* breathe phase index 0..BREATHE_STEPS-1 */
static bool     s_blink_on     = false;
static uint16_t s_blink_period = DEFAULT_BLINK_MS;

/* flash (grouped) state */
static int      s_phase          = PH_OPP;
static uint8_t  s_groups_left    = 0;
static uint8_t  s_blinks_left    = 0;
static uint8_t  s_blinks_per_grp = 0;
static uint16_t s_flash_period   = DEFAULT_PERIOD_MS;
static uint16_t s_gap_ms         = 0;

/* ---- low level output ---- */
static void apply_duty(uint32_t logical)
{
    if (logical > LED_MAX_DUTY) {
        logical = LED_MAX_DUTY;
    }
    uint32_t d = s_cfg.activeLow ? (LED_MAX_DUTY - logical) : logical; /* software polarity */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s_cfg.ledcChannel, d);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)s_cfg.ledcChannel);
}

static void apply_base(void)     { apply_duty(s_base_on ? s_on_duty : 0); }
static void apply_opposite(void) { apply_duty(s_base_on ? 0 : s_on_duty); }

/* perceptually smooth (gamma/squared) triangle ramp, integer math only */
static uint32_t breathe_duty(uint16_t i)
{
    uint16_t pos = (i < BREATHE_HALF) ? i : (uint16_t)(BREATHE_STEPS - i); /* 0..HALF */
    return (uint32_t)s_on_duty * pos * pos / (BREATHE_HALF * BREATHE_HALF);
}

static void rearm(uint32_t ms) { xTimerChangePeriod(s_timer, pdMS_TO_TICKS(ms), 0); } /* also (re)starts */
static void stop_timer(void)   { xTimerStop(s_timer, 0); }

/* write the current background and (re)arm/stop the timer accordingly */
static void run_background(void)
{
    if (s_bg == BG_BREATHE) {
        apply_duty(breathe_duty(s_breathe_i));
        rearm(BREATHE_TICK_MS);
    } else if (s_bg == BG_BLINK) {
        apply_duty(s_blink_on ? s_on_duty : 0);
        rearm(s_blink_period);
    } else if (s_bg == BG_HEARTBEAT) {
        /* pulse = opposite of the steady base (works from off AND from on) */
        if (s_blink_on) apply_opposite(); else apply_base();
        rearm(s_blink_on ? HEARTBEAT_ON_MS : HEARTBEAT_OFF_MS);
    } else {
        apply_base();
        stop_timer();
    }
}

/* Timer callback — runs in the FreeRTOS timer-service task and is the sole
   owner of the state machine. Only light LEDC register writes happen here. */
static void led_timer_cb(TimerHandle_t t)
{
    (void)t;

    if (s_fx == FX_FLASH) {
        if (s_phase == PH_OPP) {                 /* opposite half done -> base half */
            s_phase = PH_BASE;
            apply_base();
            rearm(s_flash_period);
        } else if (s_phase == PH_BASE) {         /* one blink done */
            if (s_blinks_left > 0) s_blinks_left--;
            if (s_blinks_left > 0) {
                s_phase = PH_OPP;
                apply_opposite();
                rearm(s_flash_period);
            } else {                             /* group done */
                if (s_groups_left > 0) s_groups_left--;
                if (s_groups_left == 0) {
                    s_fx = FX_NONE;
                    run_background();            /* flash done -> back to background */
                } else {
                    s_blinks_left = s_blinks_per_grp;
                    if (s_gap_ms > 0) {
                        s_phase = PH_GAP;
                        apply_base();
                        rearm(s_gap_ms);
                    } else {
                        s_phase = PH_OPP;
                        apply_opposite();
                        rearm(s_flash_period);
                    }
                }
            }
        } else {                                 /* PH_GAP done -> next group */
            s_phase = PH_OPP;
            apply_opposite();
            rearm(s_flash_period);
        }
    } else if (s_bg == BG_BREATHE) {             /* continuous breathing step */
        s_breathe_i = (uint16_t)((s_breathe_i + 1) % BREATHE_STEPS);
        apply_duty(breathe_duty(s_breathe_i));
        rearm(BREATHE_TICK_MS);
    } else if (s_bg == BG_BLINK) {               /* continuous square blink step */
        s_blink_on = !s_blink_on;
        apply_duty(s_blink_on ? s_on_duty : 0);
        rearm(s_blink_period);
    } else if (s_bg == BG_HEARTBEAT) {           /* asymmetric: short pulse, long gap */
        s_blink_on = !s_blink_on;
        if (s_blink_on) apply_opposite(); else apply_base();   /* pulse = opposite of base */
        rearm(s_blink_on ? HEARTBEAT_ON_MS : HEARTBEAT_OFF_MS);
    } else {                                     /* nothing dynamic -> settle and stop */
        apply_base();
        stop_timer();
    }
}

/* set up a (possibly grouped) flash; a continuous background suppresses it */
static void start_flash(uint8_t groups, uint8_t blinks, uint16_t periodMs, uint16_t gapMs)
{
    if (!s_inited || groups == 0 || blinks == 0) return;
    if (periodMs == 0) periodMs = DEFAULT_PERIOD_MS;
    if (s_bg != BG_STEADY) {            /* breathe/blink have priority -> ignore flashes */
        return;
    }
    s_fx             = FX_FLASH;
    s_groups_left    = groups;
    s_blinks_per_grp = blinks;
    s_blinks_left    = blinks;
    s_flash_period   = periodMs;
    s_gap_ms         = gapMs;
    s_phase          = PH_OPP;
    apply_opposite();                  /* first (opposite) half */
    rearm(periodMs);
}

/* ---- public API ---- */
esp_err_t ledInit(const LedConfig *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    s_cfg = *cfg;
    if (s_cfg.freqHz == 0) s_cfg.freqHz = LED_DEFAULT_FREQ;
    if (s_cfg.onPercent == 0 || s_cfg.onPercent > 100) s_cfg.onPercent = 100;
    s_on_duty = (uint32_t)LED_MAX_DUTY * s_cfg.onPercent / 100;

    ledc_timer_config_t tcfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LED_PWM_RES,
        .timer_num       = (ledc_timer_t)s_cfg.ledcTimer,
        .freq_hz         = s_cfg.freqHz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&tcfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ledc_timer_config failed: %d", err); return err; }

    ledc_channel_config_t ccfg = {
        .gpio_num   = s_cfg.gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = (ledc_channel_t)s_cfg.ledcChannel,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = (ledc_timer_t)s_cfg.ledcTimer,
        .duty       = s_cfg.activeLow ? LED_MAX_DUTY : 0,   /* start OFF */
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ccfg);
    if (err != ESP_OK) { ESP_LOGE(TAG, "ledc_channel_config failed: %d", err); return err; }

    s_timer = xTimerCreate("led", pdMS_TO_TICKS(BREATHE_TICK_MS), pdFALSE, NULL, led_timer_cb);
    if (!s_timer) { ESP_LOGE(TAG, "alloc failed"); return ESP_ERR_NO_MEM; }

    s_base_on = false;
    s_bg = BG_STEADY;
    s_fx = FX_NONE;
    apply_duty(0);                       /* explicit OFF */
    s_inited = true;
    ESP_LOGI(TAG, "init gpio=%d activeLow=%d freq=%uHz v%s",
             s_cfg.gpio, (int)s_cfg.activeLow, (unsigned)s_cfg.freqHz, LED_INDICATOR_VERSION);
    return ESP_OK;
}

void ledSteady(bool on)
{
    if (!s_inited) return;
    s_base_on = on;
    if (s_fx == FX_NONE && s_bg == BG_STEADY) {
        apply_base();                    /* immediate when idle */
    }
}

void ledBreathe(bool on)
{
    if (!s_inited) return;
    if (on) {
        s_bg = BG_BREATHE;
        s_breathe_i = 0;
        if (s_fx == FX_NONE) { apply_duty(breathe_duty(0)); rearm(BREATHE_TICK_MS); }
    } else {
        s_bg = BG_STEADY;
        if (s_fx == FX_NONE) { apply_base(); stop_timer(); }
    }
}

void ledBlink(bool on, uint16_t periodMs)
{
    if (!s_inited) return;
    if (periodMs == 0) periodMs = DEFAULT_BLINK_MS;
    if (on) {
        s_bg = BG_BLINK;
        s_blink_period = periodMs;
        s_blink_on = true;
        if (s_fx == FX_NONE) { apply_duty(s_on_duty); rearm(periodMs); }
    } else {
        s_bg = BG_STEADY;
        if (s_fx == FX_NONE) { apply_base(); stop_timer(); }
    }
}

void ledHeartbeat(bool on)
{
    if (!s_inited) return;
    if (on) {
        s_bg = BG_HEARTBEAT;
        s_blink_on = true;               /* start with the pulse for immediate feedback */
        if (s_fx == FX_NONE) { apply_opposite(); rearm(HEARTBEAT_ON_MS); }
    } else {
        s_bg = BG_STEADY;
        if (s_fx == FX_NONE) { apply_base(); stop_timer(); }
    }
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
    if (!s_inited || times == 0) return;
    if (periodMs == 0) periodMs = DEFAULT_PERIOD_MS;
    /* Stop the engine: the device typically reboots right after, so drive the
       pattern synchronously instead of relying on the (soon-gone) timer. */
    s_fx = FX_NONE;
    stop_timer();
    bool base = s_base_on;

    for (uint8_t i = 0; i < times; i++) {
        apply_duty(base ? 0 : s_on_duty);          /* opposite of base */
        vTaskDelay(pdMS_TO_TICKS(periodMs));
        apply_duty(base ? s_on_duty : 0);          /* base */
        vTaskDelay(pdMS_TO_TICKS(periodMs));
    }
}

/* ---- convenience wrappers: fixed patterns, unified across devices ---- */
void ledTxFlash(void)      { ledFlash(TX_BLINKS, TX_MS); }
void ledOtaStatus(bool on) { ledBlink(on, OTA_PERIOD_MS); }
void ledOverheated(void)   { ledFlashGroups(OVERHEAT_GROUPS, OVERHEAT_BLINKS, OVERHEAT_MS, OVERHEAT_GAP_MS); }
void ledResetNetwork(void) { ledFlash(RESETNET_BLINKS, RESETNET_MS); }
void ledNotifyFlash(void)  { ledFlash(NOTIFY_BLINKS, NOTIFY_MS); }
void ledIdentify(void)     { ledFlashGroups(IDENTIFY_GROUPS, IDENTIFY_BLINKS, IDENTIFY_MS, IDENTIFY_GAP_MS); }
void ledResetOk(void)      { ledFlashBlocking(RESET_BLINKS, RESET_MS); }
