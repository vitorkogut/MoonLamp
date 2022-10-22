#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include <math.h>
#include "string.h"
#include "stdio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/ledc.h"
#include "esp_err.h"
#define min(a,b) fmin(a,b)

// ##################### VARS GLOBAIS #################
#define DEFAULT_SSID "SOL"
#define DEFAULT_PWD "cms201244"
int light_limit = 2500;
int latest_light_level = 0;
int loops_desde_ultimo_movimento = 0;
int loop_periferico_ms = 50;
int val_LED_red = 0;
int val_LED_green = 0;
int val_LED_blue = 0;
int val_LED_fade = 0;
int val_LED_ativa_sensores = 0;
int val_atual_red = 0;
int val_atual_green = 0;
int val_atual_blue = 0;
int constante_fade = 4;
int direcao_fade_red = 0;
int direcao_fade_green = -1;
int direcao_fade_blue = -1;

// ################# PWM CONFIGS ######################
#define PWM_CHANNEL_RED LEDC_CHANNEL_1
#define PWM_CHANNEL_GREEN LEDC_CHANNEL_2
#define PWM_CHANNEL_BLUE LEDC_CHANNEL_3
#define PWM_TIMER LEDC_TIMER_1
#define PWM_BIT_NUM LEDC_TIMER_8_BIT
#define RED_PWM_PIN GPIO_NUM_19
#define GREEN_PWM_PIN GPIO_NUM_21
#define BLUE_PWM_PIN GPIO_NUM_22

int cap_value(int val, int max, int min){
    if(val>max){
        return max;
    }
    if(val<min){
        return min;
    }
    return val;
}

void led_pwm_init(void){
    ledc_channel_config_t ledc_channel_red = {0}, ledc_channel_green = {0}, ledc_channel_blue = {0};

    ledc_channel_red.gpio_num = RED_PWM_PIN;
    ledc_channel_red.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel_red.channel = PWM_CHANNEL_RED;
    ledc_channel_red.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_red.timer_sel = PWM_TIMER;
    ledc_channel_red.duty = 0;
	
    ledc_channel_green.gpio_num = GREEN_PWM_PIN;
    ledc_channel_green.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel_green.channel = PWM_CHANNEL_GREEN;
    ledc_channel_green.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_green.timer_sel = PWM_TIMER;
    ledc_channel_green.duty = 0;
    
    ledc_channel_blue.gpio_num = BLUE_PWM_PIN;
    ledc_channel_blue.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_channel_blue.channel = PWM_CHANNEL_BLUE;
    ledc_channel_blue.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_blue.timer_sel = PWM_TIMER;
    ledc_channel_blue.duty = 0;
	
    ledc_timer_config_t ledc_timer = {0};
    ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledc_timer.duty_resolution = 8;
    ledc_timer.timer_num = PWM_TIMER;
    ledc_timer.freq_hz = 25000;
	
	ESP_ERROR_CHECK( ledc_channel_config(&ledc_channel_red) );
	ESP_ERROR_CHECK( ledc_channel_config(&ledc_channel_green) );
    ESP_ERROR_CHECK( ledc_channel_config(&ledc_channel_blue) );
	ESP_ERROR_CHECK( ledc_timer_config(&ledc_timer) );
}

void led_pwm_set(int red_value, int green_value, int blue_value) {
	//uint32_t red_value_u = cap_value(red_value, 0, 255);
    //uint32_t green_value_u = cap_value(green_value, 0, 255);
    //uint32_t blue_value_u = cap_value(blue_value, 0, 255);
	
	ESP_ERROR_CHECK( ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_RED, red_value) );
	ESP_ERROR_CHECK( ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_RED) );

    ESP_ERROR_CHECK( ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_GREEN, green_value) );
	ESP_ERROR_CHECK( ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_GREEN));

    ESP_ERROR_CHECK( ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_BLUE, blue_value) );
	ESP_ERROR_CHECK( ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL_BLUE) );
}

// ####################### WIFI #######################

#if CONFIG_EXAMPLE_WIFI_ALL_CHANNEL_SCAN
#define DEFAULT_SCAN_METHOD WIFI_ALL_CHANNEL_SCAN
#elif CONFIG_EXAMPLE_WIFI_FAST_SCAN
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#else
#define DEFAULT_SCAN_METHOD WIFI_FAST_SCAN
#endif /*CONFIG_EXAMPLE_SCAN_METHOD*/

#if CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SIGNAL
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#elif CONFIG_EXAMPLE_WIFI_CONNECT_AP_BY_SECURITY
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SECURITY
#else
#define DEFAULT_SORT_METHOD WIFI_CONNECT_AP_BY_SIGNAL
#endif /*CONFIG_EXAMPLE_SORT_METHOD*/

#if CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD
#define DEFAULT_RSSI CONFIG_EXAMPLE_FAST_SCAN_MINIMUM_SIGNAL
#if CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_OPEN
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WEP
#define DEFAULT_AUTHMODE WIFI_AUTH_WEP
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WPA
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA_PSK
#elif CONFIG_EXAMPLE_FAST_SCAN_WEAKEST_AUTHMODE_WPA2
#define DEFAULT_AUTHMODE WIFI_AUTH_WPA2_PSK
#else
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif
#else
#define DEFAULT_RSSI -127
#define DEFAULT_AUTHMODE WIFI_AUTH_OPEN
#endif /*CONFIG_EXAMPLE_FAST_SCAN_THRESHOLD*/

static const char *TAG = "scan";

static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data){
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

/* Initialize Wi-Fi as sta and set scan method */
static void fast_scan(void){
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    // Initialize default station as network interface instance (esp-netif)
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // Initialize and start WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = DEFAULT_SSID,
            .password = DEFAULT_PWD,
            .scan_method = DEFAULT_SCAN_METHOD,
            .sort_method = DEFAULT_SORT_METHOD,
            .threshold.rssi = DEFAULT_RSSI,
            .threshold.authmode = DEFAULT_AUTHMODE,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


// ####################### SERVER HTTP #######################

esp_err_t get_handler_presenca(httpd_req_t *req){
    const char resp[20] = "";
    int secs_desde_ultimo_movimento = (loops_desde_ultimo_movimento * loop_periferico_ms) / 1000;
    sprintf(resp, "%d", secs_desde_ultimo_movimento);    
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}
esp_err_t get_handler_luz(httpd_req_t *req){
    char resp[20] = "";
    sprintf(resp, "%d", latest_light_level);
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t post_handler(httpd_req_t *req){
    char content[100]= "";

    if(req->content_len != 16){
        const char resp[] = "Comando possui tamanho invalido!";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    /* Truncate if content length larger than the buffer */
    size_t recv_size = fmin(req->content_len, sizeof(content));
    int ret = httpd_req_recv(req, content, recv_size);

    char red[3];
    char green[3];
    char blue[3];
    int red_int = 0; 
    int green_int = 0; 
    int blue_int = 0; 
    int fade_int = 0;
    int sensores_int = 0;

    red[0]=content[0];
    red[1]=content[1];
    red[2]=content[2];
    sscanf(red, "%d", &red_int);
    green[0]=content[4];
    green[1]=content[5];
    green[2]=content[6];
    sscanf(green, "%d", &green_int);
    blue[0]=content[8];
    blue[1]=content[9];
    blue[2]=content[10];
    sscanf(blue, "%d", &blue_int);

    fade_int = content[12] - '0';
    sensores_int = content[14] - '0';


    val_LED_red = red_int;
    val_LED_green = green_int;
    val_LED_blue = blue_int;    
    val_LED_fade = fade_int;
    val_LED_ativa_sensores = sensores_int;

    printf("RED: %d\n", red_int);
    printf("GREEN: %d\n", green_int);
    printf("BLUE: %d\n", blue_int);
    fflush(stdout);

    /* Send a simple response */
    const char resp[] = "Comando recebido";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* URI handler structure for GETs*/
httpd_uri_t presenca_get = {
    .uri      = "/sensor_presenca",
    .method   = HTTP_GET,
    .handler  = get_handler_presenca,
    .user_ctx = NULL
};
httpd_uri_t luz_get = {
    .uri      = "/sensor_luz",
    .method   = HTTP_GET,
    .handler  = get_handler_luz,
    .user_ctx = NULL
};

/* URI handler structure for POSTs */
httpd_uri_t uri_post = {
    .uri      = "/set",
    .method   = HTTP_POST,
    .handler  = post_handler,
    .user_ctx = NULL
};

/* Function for starting the webserver */
httpd_handle_t start_webserver(void)
{
    /* Generate default configuration */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    /* Empty handle to esp_http_server */
    httpd_handle_t server = NULL;

    /* Start the httpd server */
    if (httpd_start(&server, &config) == ESP_OK) {
        /* Register URI handlers */
        httpd_register_uri_handler(server, &luz_get);
        httpd_register_uri_handler(server, &presenca_get);
        httpd_register_uri_handler(server, &uri_post);
    }
    /* If server failed to start, handle will be NULL */
    return server;
}

/* Function for stopping the webserver */
void stop_webserver(httpd_handle_t server){
    if (server) {
        /* Stop the httpd server */
        httpd_stop(server);
    }
}

void task_perifericos(void *pvParameters){
    adc1_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_11);

    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = ((1ULL<<GPIO_NUM_34));
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    esp_err_t ret_gpio = gpio_config(&io_conf);

    led_pwm_init();

    while(1==1){ // Loop de iteração com periféricos
        vTaskDelay(loop_periferico_ms / portTICK_PERIOD_MS);

        // ATUALIZAÇÂO DO VALOR DO SENSOR DE LUZ
        int vals = 0;
        for(int i=0;i<5;i++){
            int med=0;
            med = adc1_get_raw(ADC1_CHANNEL_5);
            vals = vals + med;
        }
        latest_light_level = vals/5;

        // ATUALIZAÇÂO DO TEMPO DESDE ULTIMO MOVIEMNTO
        if( gpio_get_level(34) == 0){
            loops_desde_ultimo_movimento++;
        }else{
            loops_desde_ultimo_movimento = 0;
        }
        //printf("%d\n",gpio_get_level(34));

        // CONFIG DO FADE  
        if(val_atual_red < val_LED_red){
            val_atual_red = val_atual_red + constante_fade;
        }
        if(val_atual_red > val_LED_red){
            val_atual_red = val_atual_red - constante_fade;
        }

        if(val_atual_green < val_LED_green){
            val_atual_green = val_atual_green + constante_fade;
        }
        if(val_atual_green > val_LED_green){
            val_atual_green = val_atual_green - constante_fade;
        }

        if(val_atual_blue < val_LED_blue){
            val_atual_blue = val_atual_blue + constante_fade;
        }
        if(val_atual_blue > val_LED_blue){
            val_atual_blue = val_atual_blue - constante_fade;
        }


        // ATUALIZA VALORES DO PWM DOS LEDS
        if(val_LED_ativa_sensores == 0){
            led_pwm_set(val_atual_red, val_atual_green, val_atual_blue);
        }else{
            if(gpio_get_level(34) == 1 && light_limit < latest_light_level){
                led_pwm_set(val_atual_red, val_atual_green, val_atual_blue);
            }else{
                led_pwm_set(0, 0, 0);
            }
        }
        
        // EFEITO DE FADE
        if(val_LED_fade == 1 && direcao_fade_red == 0){
            val_LED_red = val_LED_red + 6;
            if (val_LED_red > 240){
                direcao_fade_red = 1; 
            }
        }else if(val_LED_fade == 1 && direcao_fade_red == 1){
            val_LED_red = val_LED_red - 6;
            if (val_LED_red < 10){
                direcao_fade_red = -1;
                direcao_fade_green = 0; 
            }
        }

        if(val_LED_fade == 1 && direcao_fade_green == 0){
            val_LED_green = val_LED_green + 6;
            if (val_LED_green > 240){
                direcao_fade_green = 1; 
            }
        }else if(val_LED_fade == 1 && direcao_fade_green == 1){
            val_LED_green = val_LED_green - 6;
            if (val_LED_green < 10){
                direcao_fade_green = -1;
                direcao_fade_red = 0; 
            }
        }
        
    }
}



void app_main(void){
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    fast_scan(); // procura a rede para conectar
    start_webserver(); // inicia servidor HTTP

    xTaskCreatePinnedToCore(
                    &task_perifericos,   /* Function to implement the task */
                    "task_perifericos", /* Name of the task */
                    10000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    1);  /* Core where the task should run */
    
}
