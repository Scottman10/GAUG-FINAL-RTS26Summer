/*
 * Application 1 — Setup, blink, web monitor
 *
 * What this scaffold gives you:
 *   - A WORKING dual-core ESP32 application.
 *   - Core 1 runs a pps_task that toggles an LED at 1 Hz with vTaskDelayUntil,
 *     simulating the 1PPS (one-pulse-per-second) timing output of a GNSS receiver.
 *   - A GPIO interrupt (pps_isr_handler) fires on every rising edge of that
 *     signal — just like a real timing receiver's dedicated PPS input line —
 *     and hands off to a deferred task (pps_capture_task) that does the
 *     actual interval/jitter logging, keeping the ISR itself minimal.
 *   - Core 0 runs an HTTP server that reports PPS state and WCET stats to a browser.
 *   - Wi-Fi is wired up for Wokwi's simulated "Wokwi-GUEST" access point.
 *
 * WIRING NOTE (Wokwi diagram): add a jumper wire from LED_GPIO (GPIO2) to
 * PPS_CAPTURE_GPIO (GPIO4) so the simulated PPS output loops back into a
 * real interrupt input, the way an external GNSS module's PPS pin would
 * be wired into a host MCU.
 *
 * What you do:
 *   1. Rename the task / log / page strings to fit your CHOSEN THEME.
 *      Search for YOURTHEME and replace every occurrence.
 *   2. Customize the HTML in handle_root() to match your theme.
 *   3. Optionally change LED_GPIO and BLINK_PERIOD_MS.
 *   4. Run it, take a screenshot of the web page, drop both in your README.
 *
 * ============================================================
 * Theme: GNSS Receiver — 1PPS Timing Signal
 * ============================================================
 *
 * Real-world context: GNSS (GPS/GLONASS/Galileo/BeiDou) receivers used for
 * precision timing output a "1PPS" (one pulse per second) signal — a sharp
 * edge aligned to the top of each UTC second, accurate to tens of
 * nanoseconds once the receiver has a fix. Downstream systems (network time
 * servers, telecom base stations, phasor measurement units, lab
 * instruments) discipline their local oscillators against this edge, almost
 * always via a hardware interrupt on that edge rather than by polling — the
 * whole point of PPS is that software polling jitter would defeat its
 * precision. Here, the 1 Hz LED toggle stands in for that PPS edge, the
 * GPIO interrupt stands in for the receiver's PPS input line, and the web
 * dashboard stands in for a receiver's status page.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"

/* ---------- Configuration ---------- */
#define LED_GPIO           GPIO_NUM_2
#define PPS_CAPTURE_GPIO   GPIO_NUM_4   /* wire this to LED_GPIO in the Wokwi diagram */
#define BLINK_PERIOD_MS    1000          /* 1 Hz toggle == simulated 1PPS edge */
#define HTTP_PORT          80

#define WIFI_SSID         "Wokwi-GUEST"
#define WIFI_PASS         ""             /* Wokwi virtual AP is open */

#define CONFIG_LOG_DEFAULT_LEVEL_INFO 1
#define CONFIG_LOG_MAXIMUM_LEVEL  5

static const char *TAG = "gnss_pps";

/* Shared state: single bool, atomic read on Xtensa.
 * (W6 will teach us why this is the only case where volatile-without-mutex is OK.) */
static volatile bool pps_high = false;
static volatile uint32_t pulse_count = 0;

/* ---------- ISR / deferred-capture state ---------- *
 * A real GNSS-disciplined clock latches a hardware timestamp the instant
 * the PPS edge arrives, then does everything else (logging, math, network
 * updates) OUTSIDE the ISR. We mirror that split here: pps_isr_handler does
 * only the time-critical latch + notify; pps_capture_task does the rest. */
static volatile int64_t  last_pulse_us     = 0;
static volatile int64_t  last_interval_us  = 0;
static volatile uint32_t isr_edge_count    = 0;
static TaskHandle_t      capture_task_handle = NULL;

/* Every task that writes to the console (ESP_LOGI or printf) takes this
 * first, so a multi-line print — like the WCET table — can't be interleaved
 * with a log line from another task. ISRs never log, so the ISR itself
 * doesn't need it. */
static SemaphoreHandle_t console_mutex = NULL;

/* ---------- WCET instrumentation ---------- *
 * Each task/ISR times its own critical-path body with esp_timer_get_time()
 * (a monotonic microsecond counter) and keeps a running max. This is a
 * measured, empirical WCET-so-far, not a formally verified bound — see
 * TASK_TABLE_AND_WCET.md for the methodology and its limits. */
static volatile int64_t pps_task_wcet_us     = 0;
static volatile int64_t capture_task_wcet_us = 0;
static volatile int64_t isr_wcet_us          = 0;
static volatile uint32_t pps_task_samples     = 0;
static volatile uint32_t capture_task_samples = 0;

/* ---------- PPS task — runs on Core 1 (APP_CPU) ---------- *
 * Stands in for a real GNSS module's PPS output pin, which a real
 * receiver drives with a hardware timer disciplined to the satellite
 * constellation's atomic clocks. */
static void pps_task(void *arg)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    /* vTaskDelayUntil is drift-free: we wake on a fixed schedule
     * even if our work took a variable amount of time. A real PPS
     * signal comes from receiver hardware, not software timing, but
     * this keeps our simulated edge from accumulating jitter. */
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BLINK_PERIOD_MS);

    //============================================================================================================================
    //Replace for loop to introduce fault
    //============================================================================================================================

    for (;;) 
    //for (volatile int i = 0; i < 5000000; i++)
    {
        int64_t t0 = esp_timer_get_time();

        pps_high = !pps_high;
        if (pps_high) {
            pulse_count++;
        }
        gpio_set_level(LED_GPIO, pps_high);
        xSemaphoreTake(console_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "1PPS edge = %s (pulse #%lu)",
                 pps_high ? "RISING" : "FALLING", (unsigned long)pulse_count);
        xSemaphoreGive(console_mutex);

        int64_t dur = esp_timer_get_time() - t0;
        if (dur > pps_task_wcet_us) {
            pps_task_wcet_us = dur;
        }
        pps_task_samples++;

        vTaskDelayUntil(&last_wake, period);

    }
}

/* ---------- PPS interrupt handler — fires on every rising edge ---------- *
 * Runs in interrupt context on whichever core called gpio_install_isr_service()
 * (that's Core 0 here, since app_main() runs on PRO_CPU by default). Kept
 * deliberately tiny: latch a timestamp, compute the interval, wake the
 * deferred task. No logging, no blocking calls — exactly what you want an
 * ISR to look like when a real downstream clock is disciplining itself off
 * this edge. */
static void IRAM_ATTR pps_isr_handler(void *arg)
{
    int64_t t0 = esp_timer_get_time();

    int64_t now = t0;
    last_interval_us = now - last_pulse_us;
    last_pulse_us = now;
    isr_edge_count++;

    BaseType_t higher_prio_woken = pdFALSE;
    //comment out to induce fault
    vTaskNotifyGiveFromISR(capture_task_handle, &higher_prio_woken);

    int64_t dur = esp_timer_get_time() - t0;
    if (dur > isr_wcet_us) {
        isr_wcet_us = dur;   /* best-effort: fine for a coursework metric,
                                 not a substitute for a logic-analyzer trace */
    }

    if (higher_prio_woken) {
        portYIELD_FROM_ISR();
    }


}

/* ---------- Deferred capture task — does the work the ISR shouldn't ---------- *
 * Blocks on a task notification given from pps_isr_handler, then does the
 * "slow" work: compute jitter against the ideal 1 s spacing and log it.
 * This is the FreeRTOS analogue of a Linux kernel PPS driver's tasklet /
 * bottom half. */
static void pps_capture_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t t0 = esp_timer_get_time();

        int64_t interval_us = last_interval_us;
        int64_t jitter_us = interval_us - 1000000; /* deviation from ideal 1 s */
        xSemaphoreTake(console_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "PPS capture: edge #%lu, interval=%lld us, jitter=%lld us",
                 (unsigned long)isr_edge_count, (long long)interval_us, (long long)jitter_us);
        xSemaphoreGive(console_mutex);

        int64_t dur = esp_timer_get_time() - t0;
        if (dur > capture_task_wcet_us) {
            capture_task_wcet_us = dur;
        }
        capture_task_samples++;
    }
}

/* ---------- Stats reporter — the easy way to collect WCET evidence ---------- *
 * Low-priority, does nothing time-critical: just wakes every REPORT_PERIOD_MS
 * and prints a small table of running-max WCET for every task/ISR straight
 * to the terminal (Wokwi's Serial Monitor). No browser, no IP address, no
 * network required — just watch (or save) the console output and copy the
 * table into your evidence doc. Uses printf() rather than ESP_LOGI so the
 * table isn't broken up by per-line log prefixes/timestamps. Runs on Core 0
 * since it only reads state; it never perturbs the tasks it reports on. */
#define REPORT_PERIOD_MS  10000
static void print_wcet_table(void)
{
    int64_t uptime_s = esp_timer_get_time() / 1000000;

    /* Held for the whole table: this is the only thing standing between a
     * clean multi-line print and another task's ESP_LOGI landing in the
     * middle of it, since all writers share the one UART. */
    xSemaphoreTake(console_mutex, portMAX_DELAY);

    printf("\n");
    printf("================== WCET EVIDENCE (uptime %llds) ==================\n",
           (long long)uptime_s);
    printf("%-18s %-6s %-6s %10s %10s\n",
           "Task/ISR", "Core", "Prio", "WCET(us)", "Samples");
    printf("--------------------------------------------------------------\n");
    printf("%-18s %-6d %-6d %10lld %10lu\n",
           "pps_task", 1, 5, (long long)pps_task_wcet_us,
           (unsigned long)pps_task_samples);
    printf("%-18s %-6d %-6s %10lld %10lu\n",
           "pps_isr_handler", 0, "ISR", (long long)isr_wcet_us,
           (unsigned long)isr_edge_count);
    printf("%-18s %-6d %-6d %10lld %10lu\n",
           "pps_capture_task", 0, 6, (long long)capture_task_wcet_us,
           (unsigned long)capture_task_samples);
    printf("================================================================\n");

    xSemaphoreGive(console_mutex);
}

static void wcet_report_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(REPORT_PERIOD_MS);

    for (;;) {
        print_wcet_table();
        vTaskDelayUntil(&last_wake, period);
    }
}

/* ---------- HTTP handler: live JSON state ---------- *
 * Returns: {"pps":true|false,"pulses":N}
 * The page polls this 4x per second via fetch() &mdash; far faster and smoother
 * than a meta-refresh full-page reload, and a realistic pattern for embedded
 * dashboards. */
static esp_err_t handle_state(httpd_req_t *req)
{
    char buf[64];
    int n = snprintf(buf, sizeof(buf),
        "{\"pps\":%s,\"pulses\":%lu}",
        pps_high ? "true" : "false",
        (unsigned long)pulse_count);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

/* ---------- HTTP handler: live WCET stats ---------- *
 * Returns the running-max execution time seen so far for each task/ISR,
 * in microseconds, plus the edge count. Poll this after letting the sim
 * run for a while to gather empirical WCET evidence. */
static esp_err_t handle_wcet(httpd_req_t *req)
{
    char buf[192];
    int n = snprintf(buf, sizeof(buf),
        "{\"pps_task_wcet_us\":%lld,\"capture_task_wcet_us\":%lld,"
        "\"isr_wcet_us\":%lld,\"edges\":%lu}",
        (long long)pps_task_wcet_us,
        (long long)capture_task_wcet_us,
        (long long)isr_wcet_us,
        (unsigned long)isr_edge_count);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_send(req, buf, n);
    return ESP_OK;
}

/* ---------- HTTP handler: root page (HTML shell only) ---------- *
 * The HTML is served once. JavaScript polls /state at 4 Hz and updates the
 * DOM in place &mdash; no full reload, no flicker, no Nyquist aliasing against
 * the 1 Hz PPS edge. */
static esp_err_t handle_root(httpd_req_t *req)
{
    static const char html[] =
        "<!DOCTYPE html>"
        "<html lang=\"en\"><head>"
        "<meta charset=\"utf-8\">"
        "<title>GNSS 1PPS monitor</title>"
        "<style>"
        "  body { font-family: -apple-system, sans-serif; background: #0B0F14; "
        "         color: #E6EDF3; padding: 2rem; }"
        "  h1 { color: #7FDBFF; border-bottom: 3px solid #1F6FEB; "
        "       display: inline-block; padding-bottom: 4px; }"
        "  .state { font-size: 3em; font-weight: 700; margin: 1rem 0; "
        "           transition: color 120ms ease; font-variant-numeric: tabular-nums; }"
        "  .state.on  { color: #3FB950; }"
        "  .state.off { color: #6E7681; }"
        "  .meta { color: #7FDBFF; font-variant-numeric: tabular-nums; }"
        "  .dot { display:inline-block; width: 0.6em; height: 0.6em; "
        "         border-radius: 50%; margin-right: 0.4em; "
        "         vertical-align: middle; transition: background 120ms ease, "
        "         box-shadow 120ms ease; }"
        "  .dot.on  { background: #3FB950; box-shadow: 0 0 10px #3FB950; }"
        "  .dot.off { background: #6E7681; box-shadow: none; }"
        "</style></head>"
        "<body>"
        "<h1>GNSS-01 &middot; 1PPS Timing Monitor</h1>"
        "<p>Receiver status: <strong>fix acquired (simulated)</strong></p>"
        "<p>PPS edge state:</p>"
        "<div id=\"state\" class=\"state off\">"
        "  <span id=\"dot\" class=\"dot off\"></span><span id=\"label\">--</span>"
        "</div>"
        "<p class=\"meta\">Pulses since boot: <span id=\"count\">0</span></p>"
        "<p class=\"meta\">Nominal rate: 1 Hz &nbsp;|&nbsp; Polling dashboard at 4 Hz via "
        "<code>/state</code> JSON endpoint.</p>"
        "<p class=\"meta\">Measured WCET (running max, &micro;s) &mdash; "
        "pps_task: <span id=\"wt\">--</span> &nbsp;|&nbsp; "
        "isr: <span id=\"wi\">--</span> &nbsp;|&nbsp; "
        "capture_task: <span id=\"wc\">--</span></p>"
        "<script>"
        "async function poll(){"
        "  try{"
        "    const r = await fetch('/state',{cache:'no-store'});"
        "    const s = await r.json();"
        "    const cls = s.pps ? 'on' : 'off';"
        "    document.getElementById('state').className = 'state ' + cls;"
        "    document.getElementById('dot').className = 'dot ' + cls;"
        "    document.getElementById('label').textContent = s.pps ? 'HIGH' : 'LOW';"
        "    document.getElementById('count').textContent = s.pulses;"
        "  }catch(e){/* ignore transient network blips */}"
        "}"
        "async function pollWcet(){"
        "  try{"
        "    const r = await fetch('/wcet',{cache:'no-store'});"
        "    const w = await r.json();"
        "    document.getElementById('wt').textContent = w.pps_task_wcet_us;"
        "    document.getElementById('wi').textContent = w.isr_wcet_us;"
        "    document.getElementById('wc').textContent = w.capture_task_wcet_us;"
        "  }catch(e){/* ignore transient network blips */}"
        "}"
        "setInterval(poll, 250);"
        "setInterval(pollWcet, 1000);"
        "poll();"
        "pollWcet();"
        "</script>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* ---------- PPS capture input + interrupt setup ---------- *
 * Configures PPS_CAPTURE_GPIO as an interrupt input and attaches
 * pps_isr_handler to its rising edge. Call this before pps_task starts
 * toggling LED_GPIO, and after capture_task_handle has been created. */
static void pps_capture_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << PPS_CAPTURE_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PPS_CAPTURE_GPIO, pps_isr_handler, NULL);
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = HTTP_PORT;
    cfg.core_id = 0;                    /* networking on Core 0 */
    cfg.task_priority = 5;
    cfg.stack_size = 8192;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) == ESP_OK) {
        httpd_uri_t root = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = handle_root,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &root);

        httpd_uri_t state = {
            .uri = "/state",
            .method = HTTP_GET,
            .handler = handle_state,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &state);

        httpd_uri_t wcet = {
            .uri = "/wcet",
            .method = HTTP_GET,
            .handler = handle_wcet,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &wcet);

        ESP_LOGI(TAG, "HTTP server started on port %d", HTTP_PORT);
    } else {
        ESP_LOGE(TAG, "HTTP server failed to start");
    }
    return server;
}

/* ---------- Wi-Fi event handler ---------- */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        start_webserver();
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
}

/* ---------- app_main — kicks everything off ---------- */
void app_main(void)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "==== App 1 GNSS 1PPS Monitor starting ====");

    /* Must exist before pps_task, pps_capture_task, or wcet_report_task
     * start running, since all three take it before writing to the console. */
    console_mutex = xSemaphoreCreateMutex();

    /* Start Wi-Fi + HTTP on Core 0 (PRO_CPU is networking by default in IDF) */
    wifi_init_sta();

    /* Deferred PPS-capture task on Core 0, alongside the ISR that wakes it.
     * Priority 6 — above the httpd task (5) — because latching/logging a
     * timing edge is more time-critical than serving a web page, mirroring
     * how a real host prioritizes PPS handling over background I/O. */
    xTaskCreatePinnedToCore(
        pps_capture_task,
        "pps_capture",
        3072,              /* a bit more headroom: does logging + math */
        NULL,
        6,
        &capture_task_handle,
        PRO_CPU_NUM        /* Core 0, same core the GPIO ISR service runs on */
    );

    /* Attach the interrupt only after capture_task_handle is valid. */
    pps_capture_init();

    /* Prints WCET evidence to the serial log every 10 s — the easy path
     * for collecting evidence, no networking required. Low priority (1)
     * since it's purely observational and must never perturb the tasks
     * it's reporting on. */
    xTaskCreatePinnedToCore(
        wcet_report_task,
        "wcet_report",
        2048,
        NULL,
        1,
        NULL,
        PRO_CPU_NUM
    );

    /* Pin the real-time PPS task to Core 1 (APP_CPU), just as a real
     * timing receiver isolates its PPS edge generation from anything
     * that could jitter it, like network or UI work. */
    xTaskCreatePinnedToCore(
        pps_task,          /* function */
        "pps",             /* name (max 16 chars) */
        2048,              /* stack — 2048 words = 8 KB */
        NULL,              /* parameters */
        5,                 /* priority */
        NULL,              /* task handle (we don't need it) */
        APP_CPU_NUM        /* Core 1 */
    );

    /* app_main returns; both cores keep running the tasks we created. */
}
