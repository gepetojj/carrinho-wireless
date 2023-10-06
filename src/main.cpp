#include <ArduinoJSON.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <esp_now.h>
// #include <DistanceSensor.hh>
#include <Motor.hh>
#include <Preferences.h>
#include <WiFi.h>

#include "ESPAsyncWebServer.h"
#include "pages.hh"

#define DEVICE_NAME "Carrinho Wireless"
#define PREF_WIFI_SETUP_KEY "wifiMenu"

#define LEFT_WHEEL_IN1_PIN 32
#define LEFT_WHEEL_IN2_PIN 33
#define LEFT_WHEEL_VEL_PIN 25

#define RIGHT_WHEEL_IN1_PIN 21
#define RIGHT_WHEEL_IN2_PIN 19
#define RIGHT_WHEEL_VEL_PIN 18

#define FRONT_SENSOR_TRIG_PIN 22
#define FRONT_SENSOR_ECHO_PIN 23

#define BUZZER_PIN 2
#define RESET_PIN 4

Motor leftWheel(LEFT_WHEEL_IN1_PIN, LEFT_WHEEL_IN2_PIN, LEFT_WHEEL_VEL_PIN, false);
Motor rightWheel(RIGHT_WHEEL_IN1_PIN, RIGHT_WHEEL_IN2_PIN, RIGHT_WHEEL_VEL_PIN, false);

// DistanceSensor frontSensor(FRONT_SENSOR_TRIG_PIN, FRONT_SENSOR_ECHO_PIN);

DynamicJsonDocument doc(1024);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

bool WIFI_MENU_SELECTED;
bool ERASING_MEMORY = false;

u_int16_t MOTORS_VELOCITY = 255;
u_int8_t FORWARD_MOVEMENT_LIMIT = 17;

typedef struct ControllerPayload
{
	uint8_t direction;
	uint8_t command;
} ControllerPayload;

ControllerPayload CONTROLLER_PAYLOAD;

// !! Métodos auxiliares para conexão ao WiFi !!

void startWiFiMenu()
{
	WiFi.mode(WIFI_AP);
	WiFi.softAP(DEVICE_NAME);
	Serial.println("[WiFi Menu] Modo WiFi alterado para AP.");
}

void registerWiFiMenu()
{
	server.reset();
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
			  { req->send_P(200, "text/html", MAIN_PAGE); });
	server.on("/console", HTTP_GET, [](AsyncWebServerRequest *req)
			  { req->send_P(200, "text/html", CONSOLE_PAGE); });
	server.on("/changeVel", HTTP_GET, [](AsyncWebServerRequest *req)
			  {
		if (req->hasParam("vel")) {
			const int velocity = std::atoi(req->getParam("vel")->value().c_str());
			MOTORS_VELOCITY = 255 * velocity / 100;
			Serial.printf("[Motors] Velocidade alterada para %i\n", MOTORS_VELOCITY);
			req->send(200, "text/plain", "Seus dados foram recebidos.");
		} else {
			req->send(400, "text/plain", "Velocidade não foi recebida, tente novamente.");
		} });

	server.begin();

	Serial.print("[WiFi Menu] Conectado. IP: ");
	Serial.println(WiFi.localIP());
	Serial.print("[WiFi Menu] MAC: ");
	Serial.println(WiFi.macAddress());
}

// !! Métodos do WebSocket !!

void startENMenu()
{
	WiFi.mode(WIFI_STA);
	Serial.println("[EN Menu] Modo WiFi alterado para STA.");
}

void onWSMessageHandler(void *arg, uint8_t *data, size_t len)
{
	AwsFrameInfo *info = (AwsFrameInfo *)arg;

	// Recebe uma mensagem por websocket indicando a direção desejada do movimento do carrinho
	if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
	{
		data[len] = 0;
		if (strcmp((char *)data, "forward") == 0)
		{
			leftWheel.moveForward(MOTORS_VELOCITY);
			rightWheel.moveForward(MOTORS_VELOCITY);
		}
		else if (strcmp((char *)data, "backward") == 0)
		{
			leftWheel.moveBackward(MOTORS_VELOCITY);
			rightWheel.moveBackward(MOTORS_VELOCITY);
		}
		else if (strcmp((char *)data, "right") == 0)
		{
			leftWheel.moveBackward(MOTORS_VELOCITY);
			rightWheel.moveForward(MOTORS_VELOCITY);
		}
		else if (strcmp((char *)data, "left") == 0)
		{
			leftWheel.moveForward(MOTORS_VELOCITY);
			rightWheel.moveBackward(MOTORS_VELOCITY);
		}
		else if (strcmp((char *)data, "stop") == 0)
		{
			leftWheel.stopMovement();
			rightWheel.stopMovement();
		}

		else if (strcmp((char *)data, "enmenu") == 0)
		{
			WIFI_MENU_SELECTED = false;
			preferences.putBool(PREF_WIFI_SETUP_KEY, false);
			startENMenu();
		}
		else if (strcmp((char *)data, "reset") == 0)
		{
			ERASING_MEMORY = true;
		}
	}
}

void onEventHandler(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
	switch (type)
	{
	case WS_EVT_DATA:
		onWSMessageHandler(arg, data, len);
		break;

	case WS_EVT_ERROR:
		Serial.println("[WebSocket] Houve um erro na comunicacao WebSocket.");
		break;

	default:
		Serial.println("[WebSocket] Um evento nao tratado foi disparado.");
		break;
	}
}

// !! Métodos do ESP-NOW !!

void onENMessageHandler(const uint8_t *mac, const uint8_t *data, int len)
{
	memcpy(&CONTROLLER_PAYLOAD, data, sizeof(CONTROLLER_PAYLOAD));
	Serial.printf("[ESP-NOW] Recebido payload de %i bytes do controle remoto.\n", len);

	if (CONTROLLER_PAYLOAD.direction == 0x04)
	{
		leftWheel.moveForward(MOTORS_VELOCITY);
		rightWheel.moveForward(MOTORS_VELOCITY);
	}
	else if (CONTROLLER_PAYLOAD.direction == 0x08)
	{
		leftWheel.moveBackward(MOTORS_VELOCITY);
		rightWheel.moveBackward(MOTORS_VELOCITY);
	}
	else if (CONTROLLER_PAYLOAD.direction == 0x02)
	{

		leftWheel.moveBackward(MOTORS_VELOCITY);
		rightWheel.moveForward(MOTORS_VELOCITY);
	}
	else if (CONTROLLER_PAYLOAD.direction == 0x01)
	{
		leftWheel.moveForward(MOTORS_VELOCITY);
		rightWheel.moveBackward(MOTORS_VELOCITY);
	}
	else if (CONTROLLER_PAYLOAD.direction == 0x00)
	{
		leftWheel.stopMovement();
		rightWheel.stopMovement();
	}

	if (CONTROLLER_PAYLOAD.command == 0x09)
	{
		WIFI_MENU_SELECTED = true;
		preferences.putBool(PREF_WIFI_SETUP_KEY, true);
		startWiFiMenu();
	}
}

// !! Métodos do lifecycle do microcontrolador !!

void reset(void *)
{
	Serial.printf("[Reset] Iniciando tarefa no core %i\n", xPortGetCoreID());
	while (true)
	{
		if (digitalRead(RESET_PIN) == HIGH || ERASING_MEMORY == true)
		{
			ERASING_MEMORY = true;
			Serial.println("[Reset] Reset acionado!! Apagando memoria em 3 segundos.");

			for (int x = 0; x < 3; x++)
			{
				digitalWrite(BUZZER_PIN, HIGH);
				vTaskDelay(200);
				digitalWrite(BUZZER_PIN, LOW);
				vTaskDelay(1000);
			}

			preferences.clear();
			ESP.restart();
		}
	}
}

void setup()
{
	Serial.begin(115200);
	preferences.begin("prefs", false);

	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(RESET_PIN, INPUT_PULLDOWN);

	// Inicia tarefas
	xTaskCreatePinnedToCore(reset, "Reset", 4000, NULL, 0, NULL, 0);
	// xTaskCreatePinnedToCore(sensorsAlarmTask, "SensorsAlarm", 8000, NULL, 0, NULL, 0);

	// Se menu WiFi estiver selecionado, inicia WiFi e as páginas do servidor
	// Se não, registra as páginas do servidor e inicia somente o ESP-NOW
	WIFI_MENU_SELECTED = preferences.getBool(PREF_WIFI_SETUP_KEY, true);
	if (WIFI_MENU_SELECTED)
	{
		startWiFiMenu();
	}
	else
	{
		startENMenu();
	}

	registerWiFiMenu();
	// Inicializa o servidor WebSocket
	ws.onEvent(onEventHandler);
	server.addHandler(&ws);

	// Inicializa o ESP-NOW
	if (esp_now_init() != ESP_OK)
	{
		Serial.println("[EN Menu] Erro iniciando o procotolo ESP-NOW.");
		return;
	}

	esp_now_register_recv_cb(onENMessageHandler);
}

void loop()
{
	ws.cleanupClients();

	doc["wifiMenuSelected"] = WIFI_MENU_SELECTED;
	doc["erasingMemory"] = ERASING_MEMORY;
	doc["motorsVelocity"] = MOTORS_VELOCITY;
	doc["wifiStatus"] = WiFi.status();
	doc["espHeap"] = ESP.getHeapSize();
	doc["espFreeHeap"] = ESP.getFreeHeap();
	doc["clients"] = ws.count();
	doc["ip"] = WiFi.localIP();
	doc["mac"] = WiFi.macAddress();

	String obj = "";
	serializeJson(doc, obj);
	ws.textAll(obj);

	delay(100);
}
