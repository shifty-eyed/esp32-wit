#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "wit_c_sdk.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "esp_timer.h"

#define BUF_SIZE 1024

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};

//network
#define WIFI_SSID "imenilenina-bistro"
#define WIFI_PASS "10101010"
#define SERVER_IP "192.168.4.95"
#define SERVER_PORT 9876
static const char *TAG = "imu_tracker1";
static const char *UDP_RECEIVER_TAG = "upd_receiver";
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

typedef struct {
	int16_t type;
	int32_t messageNumber;
	int64_t timestamp;
	int32_t deviceId;
    float accX;
    float accY;
    float accZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float angleX;
    float angleY;
    float angleZ;
    int16_t magX;
    int16_t magY;
    int16_t magZ;
} __attribute__((packed)) SensorData;

static void CmdProcess(char);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
static void Usart1Init(uint32_t baud_rate);
static void CopeCmdData(unsigned char ucData);

static void Usart1_task(void *pvParameters)
{
	unsigned char ucTemp;

	Usart1Init(9600);

	while(1)
	{
		if(uart_read_bytes(UART_NUM_1, &ucTemp, 1, portMAX_DELAY) == 1)
			WitSerialDataIn(ucTemp);
	}
}

static void Usart0_task(void *pvParameters)
{
	unsigned char c;

	while(1)
	{
		if(scanf("%c", &c) != -1)
		{
			CopeCmdData(c);
		}
		else
		{
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
	}
}

static void udp_receive_task(void *pvParameters) {
    char rx_buffer[128];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(SERVER_PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(UDP_RECEIVER_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_RECEIVER_TAG, "Socket created");

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(UDP_RECEIVER_TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_RECEIVER_TAG, "Socket bound, port %d", SERVER_PORT);

        while (1) {
            ESP_LOGI(UDP_RECEIVER_TAG, "Waiting for data");
            struct sockaddr_in6 source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len < 0) {
                ESP_LOGE(UDP_RECEIVER_TAG, "recvfrom failed: errno %d", errno);
                break;
            } else {
                // Null-terminate whatever we received and log it
                rx_buffer[len] = 0;
                inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
                ESP_LOGI(UDP_RECEIVER_TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(UDP_RECEIVER_TAG, "%s", rx_buffer);

				CmdProcess(rx_buffer[0]);
                //message_t *msg = (message_t *)rx_buffer;
                //ESP_LOGI(TAG, "Command: %c, Value: %d", msg->command, msg->value);
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying to connect to the AP...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init() {
    s_wifi_event_group = xEventGroupCreate();
    
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
    
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    
    ESP_LOGI(TAG, "wifi_init_sta finished.");
}

void app_main(void)
{
	float fAcc[3], fGyro[3], fAngle[3];
	int i;
	struct sockaddr_in dest_addr;
	long messageNumber = 0;
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);

	xTaskCreate(Usart0_task, "Usart0_task", 4096, NULL, 5, NULL);
	xTaskCreate(Usart1_task, "Usart1_task", 4096, NULL, 5, NULL);

	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
	WitDelayMsRegister(Delayms);
	AutoScanSensor();

	esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdTRUE, portMAX_DELAY);

	if (!bits || !WIFI_CONNECTED_BIT) {
		ESP_LOGE(TAG, "Failed to connect to the AP. Shutting down...");
		return;
	} else {
		ESP_LOGI(TAG, "Connected to the AP. Starting UDP send task...");
	}

	xTaskCreate(udp_receive_task, "udp_receive_task", 4096, NULL, 5, NULL);


	// Create a UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
    }
    

	while (1) {
		if(s_cDataUpdate) {
			messageNumber++;
			for(i = 0; i < 3; i++) {
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}

			SensorData msg;
			msg.type = 1;
			msg.messageNumber = messageNumber;
			msg.timestamp = esp_timer_get_time();
			msg.deviceId = 1001;

			if(s_cDataUpdate & ACC_UPDATE) {
				msg.accX = fAcc[0];
				msg.accY = fAcc[1];
				msg.accZ = fAcc[2];
				s_cDataUpdate &= ~ACC_UPDATE;
			}

			if(s_cDataUpdate & GYRO_UPDATE) {
				msg.gyroX = fGyro[0];
				msg.gyroY = fGyro[1];
				msg.gyroZ = fGyro[2];
				s_cDataUpdate &= ~GYRO_UPDATE;
			}

			if(s_cDataUpdate & ANGLE_UPDATE) {
				msg.angleX = fAngle[0];
				msg.angleY = fAngle[1];
				msg.angleZ = fAngle[2];
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}

			if(s_cDataUpdate & MAG_UPDATE) {
				msg.magX = sReg[HX];
				msg.magY = sReg[HY];
				msg.magZ = sReg[HZ];
				s_cDataUpdate &= ~MAG_UPDATE;
			}

			 // Send the message
			int err = sendto(sock, &msg, sizeof(SensorData), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));

			//printf("type: %d, deviceId: %ld, m#: %5ld, ts: %9lld,  accX: %5.2f, accY: %5.2f, accZ: %5.2f, gyroX: %5.2f, gyroY: %5.2f, gyroZ: %5.2f, angX: %5.2f, angY: %5.2f, angZ: %5.2f, magX: %d, magY: %d, magZ: %d\n", 
			//msg.type, msg.deviceId, msg.messageNumber, msg.timestamp, msg.accX, msg.accY, msg.accZ, msg.gyroX, msg.gyroY, msg.gyroZ, msg.angleX, msg.angleY, msg.angleZ, msg.magX, msg.magY, msg.magZ);
			

			if (err < 0) {
				ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
			} else {
				//ESP_LOGI(TAG, "Message sent");
			}

			vTaskDelay(pdMS_TO_TICKS(50)); // 20Hz
		}
	}

	if (sock != -1) {
		ESP_LOGE(TAG, "Shutting down socket");
		shutdown(sock, 0);
		close(sock);
    }
}

static void Usart1Init(uint32_t baud_rate) {
    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 4, 5, -1, -1));
}

void CopeCmdData(unsigned char ucData) {
	static unsigned char s_ucData[50], s_ucRxCnt = 0;

	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1]=='\r'||s_ucData[1]=='\n') && (s_ucData[2]=='\r' || s_ucData[2]=='\n'))
		{
			CmdProcess(s_ucData[0]);
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;

		}
	}

}
static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
	printf("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
	printf("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
	printf("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}

static void CmdProcess(char s_cCmd) {
	switch(s_cCmd)
	{
		case 'a':
			if(WitStartAccCali() != WIT_HAL_OK)
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':
			if(WitStartMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK)
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK)
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else
				uart_set_baudrate(UART_NUM_1, 115200);
			break;
		case 'b':
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n");
			else
				uart_set_baudrate(UART_NUM_1, 9600);
			break;
		case 'R':
			if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK)
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'r':
			if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)
				printf("\r\nSet Rate Error\r\n");
			break;
		case 'C':
			if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG|RSW_Q) != WIT_HAL_OK)
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'c':
			if(WitSetContent(RSW_ACC) != WIT_HAL_OK)
				printf("\r\nSet RSW Error\r\n");
			break;
		case 'h':
			ShowHelp();
			break;
	}
}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
	uart_write_bytes(UART_NUM_1, (const char*)p_data, uiSize);
}

static void Delayms(uint16_t usMs) {
	vTaskDelay(usMs / portTICK_PERIOD_MS);
}

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
	int i;
    for(i = 0; i < uiRegNum; i++) {
        switch(uiReg) {
            case AZ: s_cDataUpdate |= ACC_UPDATE; break;
            case GZ: s_cDataUpdate |= GYRO_UPDATE; break;
            case HZ: s_cDataUpdate |= MAG_UPDATE; break;
            case Yaw: s_cDataUpdate |= ANGLE_UPDATE; break;
            default: s_cDataUpdate |= READ_UPDATE; break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void) {
	int i, iRetry;

	for(i = 1; i < 10; i++) {
		uart_set_baudrate(UART_NUM_1, c_uiBaud[i]);
		iRetry = 2;
		do {
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Delayms(100);
			if(s_cDataUpdate != 0)
			{
				printf("%lu baud find sensor\r\n\r\n", c_uiBaud[i]);
				ShowHelp();
				return ;
			}
			iRetry--;
		} while(iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

