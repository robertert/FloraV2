/**
 * @file bringup_cam.c
 * @brief Podglad obrazu z kamery przez WiFi, w dwoch trybach.
 *
 * DWA TRYBY, BO SLUZA DO DWOCH ROZNYCH RZECZY:
 *
 *   jpeg  podglad do CELOWANIA kamery i oceny ostrosci. Czujnik pakuje
 *         sprzetowo, wiec po sieci leci kilkadziesiat kB zamiast setek,
 *         a przegladarka wyswietla strumien MJPEG bez linii skryptu.
 *   raw   surowe RGB565 do ZBIERANIA ZBIORU. JPEG podprobkowuje
 *         chrominancje 4:2:0 i doklada artefakty blokow, czyli psuje
 *         dokladnie ten sygnal, na ktorym stoi wykrywanie przeszkod
 *         (porownanie chromatycznosci z wzorcem podloza). Prototyp na
 *         komputerze MUSI widziec te same bajty, ktore pozniej zobaczy
 *         algorytm na ESP, inaczej wyniki z Maca nie przenosza sie.
 *
 * STEROWNIK PRZYJMUJE JEDEN FORMAT NA URUCHOMIENIE, wiec przelaczenie
 * trybu zapisuje klucz w NVS i restartuje glowice. Restart zamiast
 * deinit+init w biegu jest swiadomy: przeinicjalizowanie kamery w chwili,
 * gdy uchwyt HTTP jest w polowie wysylania klatki, to gotowa awaria, a
 * dwie sekundy rozruchu nikomu tu nie przeszkadzaja.
 *
 * KOLEJNOSC BAJTOW W RGB565 nie jest zgadnieta. Sterownik esp32-camera
 * dekoduje bufor jako big-endian (`conversions/to_bmp.c`): pierwszy bajt
 * piksela jest gorny, drugi dolny, a kanaly wypadaja tak:
 *
 *     R = hb & 0xF8
 *     G = (hb & 0x07) << 5 | (lb & 0xE0) >> 3
 *     B = (lb & 0x1F) << 3
 *
 * Ta sama arytmetyka siedzi w stronie podgladu i w narzedziach na Macu.
 *
 * KAMERA IDZIE PRZEZ flora_camera, nie przez esp_camera wprost. Format
 * jest polem konfiguracji wspolnego sterownika, wiec nie ma w repo dwoch
 * miejsc, w ktorych ustawia sie pinout.
 *
 * ================== CIEPLO ==================
 * Modul kamery grzeje sie przy ciaglej pracy i trzy rzeczy go studza:
 *
 *  1. ZEGAR. 10 MHz zamiast 20 (FLORA_HEAD_DVP_XCLK_HZ). Czujnik pracuje
 *     z pelna szybkoscia niezaleznie od tego, ile klatek odbieramy, wiec
 *     obnizenie zegara to realnie polowa pracy wewnetrznej.
 *  2. TEMPO STRUMIENIA. Podglad ograniczony do STREAM_MAX_FPS. Bez tego
 *     uchwyt HTTP wysyla tyle klatek, ile uda sie przepchnac przez WiFi,
 *     a radio nadajace bez przerwy to drugie zrodlo ciepla obok czujnika.
 *  3. USYPIANIE. Po IDLE_SLEEP_MS bez zadnego zadania kamera jest
 *     DEINICJALIZOWANA: sterownik zatrzymuje DMA i wylacza XCLK, wiec
 *     czujnik przestaje pracowac. Pierwsze zadanie po ciszy budzi ja z
 *     powrotem, kosztem okolo 300 ms. Modul zostawiony na biurku z
 *     otwarta zapomniana karta przegladarki przestaje byc grzalka.
 * ============================================
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bringup_cam.h"
#include "flora_camera.h"
#include "flora_head_board.h"
/* Tylko po dane dostepowe WiFi i klucze NVS - glowica trzyma je pod tymi
 * samymi nazwami co bryla, zeby nie bylo dwoch miejsc do poprawiania. */
#include "flora_board.h"
#include "flora_store.h"
#include "flora_wifi.h"

static const char *TAG = "podglad";

/**
 * Podglad JPEG: 640x480. Przy 320x240 nie widac, czy obraz jest ostry,
 * czy tylko maly, a celowanie kamery to glowne zadanie tego trybu.
 */
#define JPEG_W          640
#define JPEG_H          480
#define JPEG_Q          12

/**
 * Surowy obraz: 320x240, czyli DOKLADNIE to, na czym bedzie pracowac
 * wykrywanie przeszkod. Jedna klatka to 153,6 kB - po WiFi kilka klatek
 * na sekunde, do zbierania zbioru z nadmiarem.
 */
#define RAW_W           320
#define RAW_H           240

/** Gorna granica tempa podgladu. Do celowania kamera w zupelnosci starcza. */
#define STREAM_MAX_FPS  3
/**
 * Cisza dluzsza niz to usypia kamere.
 *
 * Dziesiec sekund, nie minuta. Sterownik esp32-camera lapie klatki CIAGLE,
 * niezaleznie od tego, ile z nich odbieramy, wiec minuta bezczynnosci to
 * minuta pelnej pracy czujnika za darmo. Pobudka kosztuje okolo sekundy
 * razem z rozgrzewka, a przy zbieraniu zbioru z odstepem 2 s kamera i tak
 * nie zdazy zasnac miedzy klatkami.
 */
#define IDLE_SLEEP_MS   10000
#define IDLE_CHECK_MS   2000

/** Klucz w przestrzeni NVS glowicy. Limit NVS to 15 znakow. */
#define NVS_CAM_MODE    "cam_mode"

typedef enum { MODE_JPEG, MODE_RAW } cam_mode_t;

static cam_mode_t         s_mode;
static httpd_handle_t     s_server;
/* Kamera jest zasobem dzielonym miedzy uchwytami HTTP a timerem usypiania,
 * a init i deinit nie moga sie zdarzyc w srodku pobierania klatki. */
static SemaphoreHandle_t  s_cam_lock;
static volatile int64_t   s_last_use_us;
static esp_timer_handle_t s_idle_timer;

/* ---------------------------------------------------------------- tryb -- */

static cam_mode_t mode_load(void)
{
    char v[8] = "jpeg";
    flora_store_t store = NULL;
    if (flora_store_open(FLORA_HEAD_NVS_NS, &store) == ESP_OK) {
        flora_store_get_str(store, NVS_CAM_MODE, v, sizeof v, "jpeg");
        flora_store_close(store);
    }
    return strcmp(v, "raw") == 0 ? MODE_RAW : MODE_JPEG;
}

static esp_err_t mode_save(const char *v)
{
    flora_store_t store = NULL;
    esp_err_t r = flora_store_open(FLORA_HEAD_NVS_NS, &store);
    if (r != ESP_OK) {
        return r;
    }
    r = flora_store_set_str(store, NVS_CAM_MODE, v);
    flora_store_close(store);
    return r;
}

/* ---------------------------------------------------------------- WiFi -- */

static void on_wifi_state(const flora_wifi_info_t *info, void *ctx)
{
    switch (info->state) {
        case FLORA_WIFI_CONNECTED:
            ESP_LOGI(TAG, "  WiFi: adres " IPSTR, IP2STR(&info->ip));
            break;
        case FLORA_WIFI_CONNECTING:
            ESP_LOGI(TAG, "  WiFi: proba %" PRIu32, info->attempt);
            break;
        case FLORA_WIFI_DISCONNECTED:
            ESP_LOGW(TAG, "  WiFi: rozlaczone (%s), nastepna proba za %" PRIu32 " ms",
                     flora_wifi_reason_str(info->reason), info->next_retry_ms);
            break;
        case FLORA_WIFI_FAILED:
            ESP_LOGE(TAG, "  WiFi: poddaje sie");
            break;
    }
}

static esp_err_t wifi_up(void)
{
    char ssid[33], pass[65];
    flora_store_t store = NULL;

    strncpy(ssid, FLORA_WIFI_SSID_DEFAULT, sizeof(ssid) - 1); ssid[sizeof(ssid) - 1] = 0;
    strncpy(pass, FLORA_WIFI_PASS_DEFAULT, sizeof(pass) - 1); pass[sizeof(pass) - 1] = 0;

    if (flora_store_open(FLORA_NVS_NS_NET, &store) == ESP_OK) {
        flora_store_get_str(store, FLORA_NVS_WIFI_SSID, ssid, sizeof(ssid),
                            FLORA_WIFI_SSID_DEFAULT);
        flora_store_get_str(store, FLORA_NVS_WIFI_PASS, pass, sizeof(pass),
                            FLORA_WIFI_PASS_DEFAULT);
        flora_store_close(store);
    }
    ESP_LOGI(TAG, "  siec \"%s\"", ssid);

    flora_wifi_cfg_t cfg = FLORA_WIFI_DEFAULT_CFG();
    strncpy(cfg.ssid, ssid, sizeof(cfg.ssid) - 1);
    strncpy(cfg.password, pass, sizeof(cfg.password) - 1);
    cfg.on_state = on_wifi_state;

    esp_err_t r = flora_wifi_init(&cfg);
    /* Glowica moze miec WiFi podniesione przez inny test - to nie blad. */
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "  nie udalo sie podniesc WiFi: %s", esp_err_to_name(r));
        return r;
    }

    ESP_LOGI(TAG, "  czekam na adres (do 20 s)...");
    r = flora_wifi_wait_connected(20000);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  brak polaczenia: %s", esp_err_to_name(r));
        ESP_LOGE(TAG, "  ponawianie dziala dalej w tle - podglad wstanie dopiero po restarcie");
    }
    return r;
}

/* -------------------------------------------------------------- kamera -- */

static esp_err_t camera_up(void)
{
    flora_camera_cfg_t cfg = FLORA_CAMERA_DEFAULT_CFG();
    cfg.pins = (flora_camera_pins_t)FLORA_HEAD_DVP_PINS();

    if (s_mode == MODE_RAW) {
        cfg.format = FLORA_CAMERA_RGB565;
        cfg.width  = RAW_W;
        cfg.height = RAW_H;
    } else {
        cfg.format       = FLORA_CAMERA_JPEG;
        cfg.width        = JPEG_W;
        cfg.height       = JPEG_H;
        cfg.jpeg_quality = JPEG_Q;
    }

    esp_err_t r = flora_camera_init(&cfg);

    /* Obnizony zegar jest wyborem termicznym, nie elektrycznym, wiec gdyby
     * czujnik przy nim nie ruszyl, wracamy do katalogowych 20 MHz i mowimy
     * o tym glosno - lepiej ciepla kamera niz zadna. */
    if (r != ESP_OK && cfg.pins.xclk_freq_hz != 20000000) {
        ESP_LOGW(TAG, "  czujnik nie ruszyl przy %d MHz, probuje 20 MHz",
                 cfg.pins.xclk_freq_hz / 1000000);
        ESP_LOGW(TAG, "  UWAGA: modul bedzie sie grzal mocniej - skroc sesje");
        cfg.pins.xclk_freq_hz = 20000000;
        r = flora_camera_init(&cfg);
    }

    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  kamera nie wystartowala: %s", esp_err_to_name(r));
        ESP_LOGE(TAG, "  sprawdz w tej kolejnosci: tasma w gniezdzie (styki do dolu),");
        ESP_LOGE(TAG, "  pinout DVP w flora_head_board.h, wlaczony PSRAM w sdkconfig");
    }
    return r;
}

/* ------------------------------------------------------- usypianie kamery -- */

/**
 * @brief Bierze kamere do uzytku, budzac ja, jesli spi.
 *
 * Po powrocie ESP_OK zasob jest ZABLOKOWANY i trzeba oddac go przez
 * cam_release(). Pobudka kosztuje okolo 300 ms i zdarza sie raz na sesje.
 */
static esp_err_t cam_acquire(void)
{
    xSemaphoreTake(s_cam_lock, portMAX_DELAY);
    if (!flora_camera_is_ready()) {
        esp_err_t r = camera_up();
        if (r != ESP_OK) {
            xSemaphoreGive(s_cam_lock);
            return r;
        }
        ESP_LOGI(TAG, "kamera obudzona");
    }
    s_last_use_us = esp_timer_get_time();
    return ESP_OK;
}

static void cam_release(void)
{
    s_last_use_us = esp_timer_get_time();
    xSemaphoreGive(s_cam_lock);
}

static void idle_check(void *arg)
{
    /* Bez czekania: jesli ktos wlasnie pobiera klatke, to z definicji nie
     * jest to cisza i usypianie moze poczekac do nastepnego taktu. */
    if (xSemaphoreTake(s_cam_lock, 0) != pdTRUE) {
        return;
    }
    if (flora_camera_is_ready() &&
        esp_timer_get_time() - s_last_use_us > (int64_t)IDLE_SLEEP_MS * 1000) {
        flora_camera_deinit();
        ESP_LOGI(TAG, "cisza przez %d s - usypiam kamere (XCLK wylaczony)",
                 IDLE_SLEEP_MS / 1000);
    }
    xSemaphoreGive(s_cam_lock);
}

bool bringup_cam_is_raw_mode(void)
{
    return s_mode == MODE_RAW;
}

esp_err_t bringup_cam_frame_begin(flora_camera_frame_t *out)
{
    esp_err_t r = cam_acquire();
    if (r != ESP_OK) {
        return r;
    }
    r = flora_camera_grab(out, 1000);
    if (r != ESP_OK) {
        cam_release();
    }
    return r;
}

void bringup_cam_frame_end(flora_camera_frame_t *fr)
{
    flora_camera_release(fr);
    cam_release();
}

/* ---------------------------------------------------------- serwer HTTP -- */

#define BOUNDARY "flora_klatka"

static const char *STREAM_TYPE = "multipart/x-mixed-replace;boundary=" BOUNDARY;
static const char *PART_HDR    = "\r\n--" BOUNDARY "\r\n"
                                 "Content-Type: image/jpeg\r\n"
                                 "Content-Length: %u\r\n\r\n";

static const char PAGE_JPEG[] =
    "<!doctype html><meta charset=utf-8><meta name=viewport "
    "content='width=device-width,initial-scale=1'>"
    "<title>Flora - podglad kamery</title>"
    "<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui;"
    "text-align:center}img{max-width:100%;image-rendering:pixelated}"
    "p{margin:8px}a{color:#8cf}</style>"
    "<img src='/stream' alt='strumien z kamery'>"
    "<p>obiektyw 160&deg;, tryb jpeg &middot; <a href='/jpg'>zapisz klatke</a>"
    " &middot; <a href='/focus'>ustaw ostrosc</a>"
    " &middot; <a href='/mode?v=raw'>przelacz na surowy RGB565</a></p>";

/* Strona trybu surowego. Przegladarka nie zna RGB565, wiec konwersje robi
 * kilka linii skryptu na canvasie - ta sama arytmetyka co w sterowniku
 * i w narzedziach na Macu. Wymiary bierzemy z naglowkow odpowiedzi, zeby
 * nie powtarzac ich w trzech miejscach.
 *
 * ODSTEP 700 ms MIEDZY POBRANIAMI, nie requestAnimationFrame. Pierwsza
 * wersja pobierala klatki tak szybko, jak sie dalo, i gdy ruszyla wizja,
 * zaczely konkurowac o te sama kamere - a serwer HTTP ma wyzszy priorytet
 * niz zadanie wizji. Zmierzone: po otwarciu tej strony czas na klatke
 * wizji skoczyl z 233 ms na 1033 ms. Do celowania kamery poltorej klatki
 * na sekunde wystarcza z nadmiarem. */
static const char PAGE_RAW[] =
    "<!doctype html><meta charset=utf-8><meta name=viewport "
    "content='width=device-width,initial-scale=1'>"
    "<title>Flora - surowy obraz</title>"
    "<style>body{margin:0;background:#111;color:#ddd;font:14px system-ui;"
    "text-align:center}canvas{max-width:100%;image-rendering:pixelated}"
    "p{margin:8px}a{color:#8cf}</style>"
    "<canvas id=c></canvas>"
    "<p id=s>lacze sie...</p>"
    "<p>tryb raw (RGB565) &middot; <a href='/raw'>pobierz klatke</a>"
    " &middot; <a href='/focus'>ustaw ostrosc</a>"
    " &middot; <a href='/mode?v=jpeg'>przelacz na podglad JPEG</a></p>"
    "<script>"
    "const c=document.getElementById('c'),x=c.getContext('2d'),"
    "s=document.getElementById('s');let n=0,t0=performance.now();"
    "async function tick(){try{"
    "const r=await fetch('/raw',{cache:'no-store'});"
    "const w=+r.headers.get('X-Width'),h=+r.headers.get('X-Height');"
    "const b=new Uint8Array(await r.arrayBuffer());"
    "if(c.width!==w){c.width=w;c.height=h;}"
    "const im=x.createImageData(w,h),d=im.data;"
    "for(let i=0,j=0,k=0;i<w*h;i++,j+=2,k+=4){const hb=b[j],lb=b[j+1];"
    "d[k]=hb&0xF8;d[k+1]=((hb&7)<<5)|((lb&0xE0)>>3);d[k+2]=(lb&0x1F)<<3;d[k+3]=255;}"
    "x.putImageData(im,0,0);n++;const dt=(performance.now()-t0)/1000;"
    "if(dt>1){s.textContent=w+'x'+h+' RGB565, '+(n/dt).toFixed(1)+' kl/s, '"
    "+(b.length/1024).toFixed(0)+' kB/klatke';n=0;t0=performance.now();}"
    "}catch(e){s.textContent='blad: '+e;}setTimeout(tick,700);}tick();"
    "</script>";

static esp_err_t page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    if (s_mode == MODE_RAW) {
        return httpd_resp_send(req, PAGE_RAW, sizeof(PAGE_RAW) - 1);
    }
    return httpd_resp_send(req, PAGE_JPEG, sizeof(PAGE_JPEG) - 1);
}

/** Jedna klatka JPEG. Do "curl -o kadr.jpg http://<ip>/jpg". */
static esp_err_t jpg_handler(httpd_req_t *req)
{
    if (cam_acquire() != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "kamera nie wstala");
        return ESP_FAIL;
    }
    flora_camera_frame_t fr;
    if (flora_camera_grab(&fr, 1000) != ESP_OK) {
        cam_release();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "brak klatki");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=flora.jpg");
    esp_err_t r = httpd_resp_send(req, (const char *)fr.data, fr.len);
    flora_camera_release(&fr);
    cam_release();
    return r;
}

/**
 * Jedna klatka surowego RGB565, bez zadnego naglowka w tresci - same
 * piksele. Wymiary ida w naglowkach HTTP, bo dzieki temu plik na dysku
 * jest dokladnie tym, co widzi algorytm, a nie formatem do parsowania.
 */
static esp_err_t raw_handler(httpd_req_t *req)
{
    if (cam_acquire() != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "kamera nie wstala");
        return ESP_FAIL;
    }
    flora_camera_frame_t fr;
    if (flora_camera_grab(&fr, 1000) != ESP_OK) {
        cam_release();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "brak klatki");
        return ESP_FAIL;
    }

    char w[8], h[8];
    snprintf(w, sizeof w, "%u", fr.width);
    snprintf(h, sizeof h, "%u", fr.height);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "X-Width", w);
    httpd_resp_set_hdr(req, "X-Height", h);
    /* Big-endian, tak jak dekoduje sterownik - patrz naglowek pliku. */
    httpd_resp_set_hdr(req, "X-Format", "RGB565BE");
    esp_err_t r = httpd_resp_send(req, (const char *)fr.data, fr.len);
    flora_camera_release(&fr);
    cam_release();
    return r;
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t r = httpd_resp_set_type(req, STREAM_TYPE);
    if (r != ESP_OK) {
        return r;
    }
    ESP_LOGI(TAG, "przegladarka podlaczona do strumienia");

    uint32_t frames = 0, bytes = 0;
    int64_t  t0 = esp_timer_get_time();

    while (1) {
        /* Blokade bierzemy NA KLATKE, nie na caly strumien: dzieki temu
         * timer usypiania ma kiedy wejsc, a /raw albo /jpg z drugiej karty
         * nie czeka do konca seansu. */
        int64_t frame_start = esp_timer_get_time();
        if (cam_acquire() != ESP_OK) {
            ESP_LOGE(TAG, "kamera nie wstala - koncze strumien");
            break;
        }
        flora_camera_frame_t fr;
        if (flora_camera_grab(&fr, 1000) != ESP_OK) {
            cam_release();
            ESP_LOGE(TAG, "brak klatki z kamery - koncze strumien");
            break;
        }

        /* 96 bajtow: naglowek czesci ma 75 znakow przy szescioznakowej
         * dlugosci klatki, reszta to zapas na dluzsze liczby. */
        char hdr[96];
        int n = snprintf(hdr, sizeof hdr, PART_HDR, (unsigned)fr.len);

        /* Przegladarka zamyka strumien zwyczajnie przez rozlaczenie, wiec
         * blad zapisu to NORMALNE zakonczenie, nie awaria. */
        r = httpd_resp_send_chunk(req, hdr, n);
        if (r == ESP_OK) {
            r = httpd_resp_send_chunk(req, (const char *)fr.data, fr.len);
        }
        bytes += fr.len;
        flora_camera_release(&fr);
        cam_release();
        if (r != ESP_OK) {
            break;
        }

        /* Dlawienie tempa: bez tego uchwyt wysyla tyle klatek, ile przejdzie
         * przez WiFi, a radio nadajace bez przerwy grzeje modul tak samo
         * skutecznie jak sam czujnik. */
        int64_t took_ms = (esp_timer_get_time() - frame_start) / 1000;
        int64_t budget  = 1000 / STREAM_MAX_FPS;
        if (took_ms < budget) {
            vTaskDelay(pdMS_TO_TICKS(budget - took_ms));
        }

        /* Co 100 klatek tempo i sredni rozmiar klatki: jedno i drugie mowi,
         * czy waskim gardlem jest WiFi, czy sama kamera. */
        if (++frames % 100 == 0) {
            int64_t dt = esp_timer_get_time() - t0;
            float   s  = (float)dt / 1000000.0f;
            ESP_LOGI(TAG, "%" PRIu32 " klatek, %.1f kl/s, srednio %" PRIu32 " kB/klatke",
                     frames, s > 0 ? 100.0f / s : 0.0f, bytes / 100 / 1024);
            bytes = 0;
            t0 = esp_timer_get_time();
        }
    }

    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "strumien zakonczony po %" PRIu32 " klatkach", frames);
    return ESP_OK;
}

/**
 * /focus - jedna proba ustawienia ostrosci.
 *
 * Sluzy tez jako TEST SPRZETU: jesli modul nie ma silniczka, firmware
 * autofokusu nie wystartuje i dostaniesz to na stronie wprost, zamiast
 * zgadywac, czemu tubus sie nie krecil.
 */
static esp_err_t focus_handler(httpd_req_t *req)
{
    if (cam_acquire() != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "kamera nie wstala");
        return ESP_FAIL;
    }
    bool focused = false;
    esp_err_t r = flora_camera_focus_trigger(3000, &focused);
    cam_release();

    const char *msg;
    if (r == ESP_ERR_NOT_SUPPORTED) {
        msg = "Sterownik nie ma wkompilowanej obslugi autofokusu.";
    } else if (r == ESP_ERR_TIMEOUT) {
        msg = "Bez potwierdzenia ostrosci. Prawie na pewno modul nie ma "
              "silniczka, czyli obiektyw jest staloogniskowy - ostrosci nie "
              "ustawisz ani programowo, ani krecac tubusem.";
    } else if (r != ESP_OK) {
        msg = "Blad sterownika.";
    } else {
        msg = "Ostrosc zlapana. Modul MA silniczek, wiec tubusem krecic nie "
              "trzeba i nie wolno.";
    }

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<!doctype html><meta charset=utf-8>"
        "<style>body{background:#111;color:#ddd;font:15px system-ui;"
        "text-align:center;padding:3em}a{color:#8cf}</style><p>");
    httpd_resp_sendstr_chunk(req, msg);
    httpd_resp_sendstr_chunk(req, "</p><p><a href='/'>wroc</a></p>");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

/**
 * /height?mm=500 - wysokosc obiektywu nad ziemia, do NVS, i restart.
 *
 * Bez tej liczby detektor dziala, ale zamiast centymetrow raportuje "nie
 * wiem": zamiana pierscienia obrazu na odleglosc to `rho = h * tan(r/f)`,
 * czyli skaluje sie wprost proporcjonalnie do h. Blad wysokosci przeklada
 * sie jeden do jednego na blad odleglosci, wiec zmierz miarka.
 */
static esp_err_t height_handler(httpd_req_t *req)
{
    char query[32], v[12];
    if (httpd_req_get_url_query_str(req, query, sizeof query) != ESP_OK ||
        httpd_query_key_value(query, "mm", v, sizeof v) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "uzyj /height?mm=500");
        return ESP_FAIL;
    }
    const int mm = atoi(v);
    if (mm < 50 || mm > 3000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                            "podaj 50-3000 mm, bo poza tym to nie jest maszt");
        return ESP_FAIL;
    }

    flora_store_t store = NULL;
    esp_err_t r = flora_store_open(FLORA_HEAD_NVS_NS, &store);
    if (r == ESP_OK) {
        r = flora_store_set_i32(store, "cam_h_mm", mm);
        flora_store_close(store);
    }
    if (r != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "zapis do NVS nie wyszedl");
        return r;
    }

    ESP_LOGW(TAG, "wysokosc kamery -> %d mm, restartuje", mm);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<!doctype html><meta charset=utf-8>"
                            "<meta http-equiv=refresh content='4;url=/'>"
                            "<style>body{background:#111;color:#ddd;font:14px system-ui;"
                            "text-align:center;padding:3em}</style>"
                            "<p>wysokosc zapisana, glowica restartuje sie</p>");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/** /mode?v=jpeg albo /mode?v=raw - zapis do NVS i restart glowicy. */
static esp_err_t mode_handler(httpd_req_t *req)
{
    char query[32], v[8];
    if (httpd_req_get_url_query_str(req, query, sizeof query) != ESP_OK ||
        httpd_query_key_value(query, "v", v, sizeof v) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "uzyj /mode?v=jpeg albo /mode?v=raw");
        return ESP_FAIL;
    }
    if (strcmp(v, "jpeg") != 0 && strcmp(v, "raw") != 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "tryb to jpeg albo raw");
        return ESP_FAIL;
    }

    esp_err_t r = mode_save(v);
    if (r != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "zapis do NVS nie wyszedl");
        return r;
    }

    ESP_LOGW(TAG, "tryb kamery -> %s, restartuje", v);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "<!doctype html><meta charset=utf-8>"
                            "<meta http-equiv=refresh content='4;url=/'>"
                            "<style>body{background:#111;color:#ddd;font:14px system-ui;"
                            "text-align:center;padding:3em}</style>"
                            "<p>tryb zapisany, glowica restartuje sie - strona wroci sama</p>");

    /* Odpowiedz musi wyjsc na kabel PRZED restartem, inaczej przegladarka
     * dostanie zerwane polaczenie i nie pokaze, czy cokolwiek sie udalo. */
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;      /* nieosiagalne */
}

static esp_err_t server_up(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* Domyslne 4 kB nie wystarcza: w tym samym zadaniu siedzi obsluga
     * kamery i sklejanie czesci multipart. */
    cfg.stack_size       = 8192;
    /* Zamknieta karta przegladarki zostawia gniazdo zajete do timeoutu.
     * Bez tego po kilku odswiezeniach serwer przestaje przyjmowac. */
    cfg.lru_purge_enable = true;

    esp_err_t r = httpd_start(&s_server, &cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "  httpd_start: %s", esp_err_to_name(r));
        return r;
    }

    const httpd_uri_t common[] = {
        { .uri = "/",      .method = HTTP_GET, .handler = page_handler },
        { .uri = "/mode",  .method = HTTP_GET, .handler = mode_handler },
        { .uri = "/focus", .method = HTTP_GET, .handler = focus_handler },
        { .uri = "/height", .method = HTTP_GET, .handler = height_handler },
    };
    for (size_t i = 0; i < sizeof common / sizeof common[0]; i++) {
        httpd_register_uri_handler(s_server, &common[i]);
    }

    /* Uchwyty formatu rejestrujemy TYLKO dla aktywnego trybu. Inaczej
     * /jpg w trybie surowym oddalby bajty RGB565 z naglowkiem image/jpeg,
     * czyli plik, ktory wyglada na obraz i nim nie jest - najgorszy
     * mozliwy rodzaj bledu w zbiorze danych. */
    if (s_mode == MODE_RAW) {
        const httpd_uri_t u = { .uri = "/raw", .method = HTTP_GET, .handler = raw_handler };
        httpd_register_uri_handler(s_server, &u);
    } else {
        const httpd_uri_t uris[] = {
            { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler },
            { .uri = "/jpg",    .method = HTTP_GET, .handler = jpg_handler    },
        };
        for (size_t i = 0; i < sizeof uris / sizeof uris[0]; i++) {
            httpd_register_uri_handler(s_server, &uris[i]);
        }
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------- test -- */

/** Wspolny rozruch: blokada, kamera, timer usypiania. */
static esp_err_t camera_stage(void)
{
    s_cam_lock = xSemaphoreCreateMutex();
    if (!s_cam_lock) {
        return ESP_ERR_NO_MEM;
    }

    /* Kamere podnosimy raz na starcie, zeby blad pinoutu albo tasmy wyszedl
     * TERAZ, w logu rozruchowym, a nie przy pierwszym uzyciu. */
    esp_err_t r = camera_up();
    if (r != ESP_OK) {
        return r;
    }
    s_last_use_us = esp_timer_get_time();

    const esp_timer_create_args_t idle_args = {
        .callback = idle_check,
        .name     = "cam_idle",
    };
    if (esp_timer_create(&idle_args, &s_idle_timer) == ESP_OK) {
        esp_timer_start_periodic(s_idle_timer, IDLE_CHECK_MS * 1000ULL);
    } else {
        ESP_LOGW(TAG, "  bez timera usypiania - kamera bedzie pracowac ciagle");
    }
    return ESP_OK;
}

esp_err_t bringup_cam_offline(void)
{
    /* Bez sieci podglad JPEG nie ma odbiorcy, wiec format wymuszamy, a nie
     * czytamy z NVS - inaczej glowica zapamietana w trybie jpeg wstalaby
     * z kamera, ktorej detektor nie umie przetworzyc, i nikt by sie o tym
     * nie dowiedzial az do pierwszego logu bez przeszkod. */
    s_mode = MODE_RAW;
    ESP_LOGI(TAG, "--- kamera bez sieci, tryb raw (RGB565) ---");

    esp_err_t r = camera_stage();
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "  WiFi i serwer podgladu wylaczone "
                      "(FLORA_HEAD_CAM_NET_DEFAULT = 0)");
    }
    return r;
}

esp_err_t bringup_cam_preview(void)
{
    s_mode = mode_load();
    ESP_LOGI(TAG, "--- podglad kamery, tryb %s ---",
             s_mode == MODE_RAW ? "raw (RGB565)" : "jpeg");

    esp_err_t r = wifi_up();
    if (r != ESP_OK) {
        return r;
    }
    if ((r = camera_stage()) != ESP_OK) {
        return r;
    }

    if ((r = server_up()) != ESP_OK) {
        flora_camera_deinit();
        return r;
    }

    flora_wifi_info_t info;
    flora_wifi_get_info(&info);
    ESP_LOGI(TAG, "  obraz na komputerze: http://" IPSTR "/", IP2STR(&info.ip));
    ESP_LOGI(TAG, "  zegar czujnika %d MHz, podglad do %d kl/s, usypianie po %d s",
             FLORA_HEAD_DVP_XCLK_HZ / 1000000, STREAM_MAX_FPS, IDLE_SLEEP_MS / 1000);
    if (s_mode == MODE_RAW) {
        ESP_LOGI(TAG, "  surowa klatka: curl -o kadr.rgb565 http://" IPSTR "/raw",
                 IP2STR(&info.ip));
        ESP_LOGI(TAG, "  powrot do podgladu: http://" IPSTR "/mode?v=jpeg", IP2STR(&info.ip));
    } else {
        ESP_LOGI(TAG, "  jedna klatka do pliku: curl -o kadr.jpg http://" IPSTR "/jpg",
                 IP2STR(&info.ip));
        ESP_LOGI(TAG, "  tryb zbierania zbioru: http://" IPSTR "/mode?v=raw", IP2STR(&info.ip));
    }
    return ESP_OK;
}
