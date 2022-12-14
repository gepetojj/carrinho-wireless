#include <ArduinoJSON.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WiFi.h>

#include "ESPAsyncWebServer.h"
#include "ESPmDNS.h"
#include "distanceSensor.hh"
#include "motor.hh"
#include "pages.hh"

#define DEVICE_NAME "Carrinho Wireless"
#define PREF_SETUP_KEY "setupDone"
#define PREF_SSID_KEY "lastSSID"
#define PREF_PASS_KEY "lastPASS"

#define SERVICE_NAME "_descobrimento"
#define SERVICE_PROTOCOL "_udp"
#define SERVICE_PORT 5600

// Convenções:
// NP - Normalmente positivo
// NN - Normalmente negativo
// VEL - Pino que controla a velocidade (variação na frequência)

#define LEFT_WHEEL_NP_PIN 18
#define LEFT_WHEEL_NN_PIN 19
#define LEFT_WHEEL_VEL_PIN 21

#define RIGHT_WHEEL_NP_PIN 17
#define RIGHT_WHEEL_NN_PIN 5
#define RIGHT_WHEEL_VEL_PIN 16

#define FRONT_SENSOR_TRIG_PIN 22
#define FRONT_SENSOR_ECHO_PIN 23

#define BUZZER_PIN 2
#define RESET_PIN 4

Motor leftWheel(LEFT_WHEEL_NP_PIN, LEFT_WHEEL_NN_PIN, LEFT_WHEEL_VEL_PIN, false);
Motor rightWheel(RIGHT_WHEEL_NP_PIN, RIGHT_WHEEL_NN_PIN, RIGHT_WHEEL_VEL_PIN, true);

DistanceSensor frontSensor(FRONT_SENSOR_TRIG_PIN, FRONT_SENSOR_ECHO_PIN);

DynamicJsonDocument doc(1024);
DNSServer dns;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

bool SETUP_DONE;
bool TRY_CONNECTION;
String SSID;
String PASS;

// !! Métodos auxiliares para conexão ao WiFi !!

class CaptiveRequestHandler : public AsyncWebHandler
{
public:
	CaptiveRequestHandler() {}
	virtual ~CaptiveRequestHandler() {}

	bool canHandle(AsyncWebServerRequest *req)
	{
		return true;
	}

	void handleRequest(AsyncWebServerRequest *req)
	{
		req->send_P(200, "text/html", CAPTIVE_PAGE);
	}
};

void setupCaptiveServer()
{
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
			  { req->send_P(200, "text/html", CAPTIVE_PAGE); });

	server.on("/set", HTTP_GET, [](AsyncWebServerRequest *req)
			  {
		if (req->hasParam("ssid") && req->hasParam("pass")) {
			SSID = req->getParam("ssid")->value();
			PASS = req->getParam("pass")->value();
			TRY_CONNECTION = true;
			Serial.println("[CaptiveServer] Dados recebidos.");
			req->send(200, "text/plain", "Seus dados foram recebidos, agora o carrinho está tentando se conectar.");
		} else {
			req->send(400, "text/plain", "SSID e/ou Senha não foram recebidos, tente novamente.");
		} });

	server.addHandler(new CaptiveRequestHandler()).setFilter(ON_AP_FILTER);
	server.begin();
}

void startCaptivePortal()
{
	WiFi.mode(WIFI_AP);
	WiFi.softAP(DEVICE_NAME);
	Serial.println("[CaptivePortal] Modo WiFi alterado para AP.");

	setupCaptiveServer();
	Serial.println("[CaptivePortal] Servidor iniciado.");

	dns.start(53, "*", WiFi.softAPIP());
	Serial.println("[CaptivePortal] DNS iniciado.");
}

void tryWiFiConnection()
{
	WiFi.mode(WIFI_STA);
	Serial.println("[WiFi] Modo WiFi alterado para STA.");
	Serial.print("[WiFi] Tentando conectar-se ao SSID: ");
	Serial.println(SSID);

	const char *ssid = SSID.c_str();
	const char *pass = PASS.c_str();
	WiFi.begin(ssid, pass);

	unsigned short counter = 0;
	while (WiFi.status() != WL_CONNECTED)
	{
		counter++;
		delay(1000);

		if (counter >= 10)
		{
			Serial.println("[WiFi] Maximo de tentativas atingido, dados podem estar incorretos.");
			SETUP_DONE = false;
			TRY_CONNECTION = false;
			preferences.putBool(PREF_SETUP_KEY, SETUP_DONE);

			startCaptivePortal();
			break;
		}
	}

	if (WiFi.status() == WL_CONNECTED)
	{
		SETUP_DONE = true;
		preferences.putBool(PREF_SETUP_KEY, true);

		String ssid_check = preferences.getString(PREF_SSID_KEY, "");
		String pass_check = preferences.getString(PREF_PASS_KEY, "");

		if (ssid_check != SSID)
		{
			preferences.putString(PREF_SSID_KEY, SSID);
		}
		if (pass_check != PASS)
		{
			preferences.putString(PREF_PASS_KEY, PASS);
		}
	}
}

// !! Métodos do WebSocket !!

void onDataHandler(void *arg, uint8_t *data, size_t len)
{
	AwsFrameInfo *info = (AwsFrameInfo *)arg;

	// Recebe uma mensagem por websocket indicando a direção desejada do movimento do carrinho
	if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
	{
		data[len] = 0;
		if (strcmp((char *)data, "forward") == 0)
		{
			if (doc["frontSensor"] <= 16)
			{
				Serial.println("[Motors] Movimento para frente cancelado (distancia).");
				leftWheel.stopMovement();
				rightWheel.stopMovement();
				return;
			}

			Serial.println("[Motors] Movimentando motores para frente.");
			leftWheel.moveForward();
			rightWheel.moveForward();
		}
		else if (strcmp((char *)data, "backward") == 0)
		{
			Serial.println("[Motors] Movimentando motores para tras.");
			leftWheel.moveBackward();
			rightWheel.moveBackward();
		}
		else if (strcmp((char *)data, "right") == 0)
		{
			if (doc["frontSensor"] <= 9)
			{
				Serial.println("[Motors] Movimento para a direita cancelado (distancia).");
				leftWheel.stopMovement();
				rightWheel.stopMovement();
				return;
			}

			Serial.println("[Motors] Movimentando motores para a direita.");
			leftWheel.moveForward();
			rightWheel.moveBackward();
		}
		else if (strcmp((char *)data, "left") == 0)
		{
			if (doc["frontSensor"] <= 9)
			{
				Serial.println("[Motors] Movimento para a esquerda cancelado (distancia).");
				leftWheel.stopMovement();
				rightWheel.stopMovement();
				return;
			}

			Serial.println("[Motors] Movimentando motores para a esquerda.");
			leftWheel.moveBackward();
			rightWheel.moveForward();
		}
		else if (strcmp((char *)data, "stop") == 0)
		{
			leftWheel.stopMovement();
			rightWheel.stopMovement();
		}
	}
}

void onEventHandler(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
	switch (type)
	{
	case WS_EVT_DATA:
		onDataHandler(arg, data, len);
		break;

	case WS_EVT_ERROR:
		Serial.println("[WebSocket] Houve um erro na comunicacao WebSocket.");
		break;

	default:
		break;
	}
}

// !! Métodos do lifecycle do microcontrolador !!

void setup()
{
	Serial.begin(115200);
	preferences.begin("prefs", false);

	// Inicia o pino do buzzer
	pinMode(BUZZER_PIN, OUTPUT);

	// Inicia o pino do reset
	pinMode(RESET_PIN, INPUT_PULLUP);

	// SETUP_DONE corresponde a existência de SSID && PASS
	SETUP_DONE = preferences.getBool(PREF_SETUP_KEY, false);
	SSID = preferences.getString(PREF_SSID_KEY, "");
	PASS = preferences.getString(PREF_PASS_KEY, "");

	if (SETUP_DONE)
	{
		Serial.println("[Setup] Usando dados inseridos previamente...");
		Serial.print("[Setup] SSID: ");
		Serial.println(SSID);
		Serial.print("[Setup] PASS: ");
		Serial.println(PASS);
		tryWiFiConnection();
	}
	else
	{
		Serial.println("[Setup] Nenhum dado previo foi encontrado, iniciando Captive Portal.");
		startCaptivePortal();
	}

	Serial.println("[Setup] Aguardando Captive Portal.");
	while (!SETUP_DONE)
	{
		dns.processNextRequest();
		delay(10);

		if (TRY_CONNECTION)
		{
			Serial.println("[Setup] Tentando conectar-se com dados recebidos do usuario.");
			tryWiFiConnection();
		}
	}

	// Inicia o serviço de descobrimento do carrinho
	if (!MDNS.begin(DEVICE_NAME))
	{
		Serial.println("[Setup] Não foi possível iniciar o servico de descobrimento.");
	}
	else
	{
		Serial.println("[Setup] Servico de descobrimento iniciado.");
		MDNS.addService(SERVICE_NAME, SERVICE_PROTOCOL, SERVICE_PORT);
	}

	// Remove os handlers do Captive Server
	server.reset();
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
			  { req->send_P(200, "text/html", MAIN_PAGE); });
	server.begin();

	// Inicializa o servidor WebSocket
	ws.onEvent(onEventHandler);
	server.addHandler(&ws);

	Serial.print("[Setup] Conectado. IP: ");
	Serial.println(WiFi.localIP());
}

void loop()
{
	// Caso o botão reset seja pressionado, limpa a memória do ESP e reinicia
	if (digitalRead(RESET_PIN) == LOW)
	{
		Serial.println("[Loop] Reset pressionado!! Apagando memoria em 10 segundos.");

		for (int x = 0; x < 10; x++)
		{
			digitalWrite(BUZZER_PIN, HIGH);
			delay(200);
			digitalWrite(BUZZER_PIN, LOW);
			delay(1000);
		}

		preferences.clear();
		ESP.restart();
		return;
	}

	ws.cleanupClients();

	// Envia para os clientes as leituras dos sensores em cm
	doc["frontSensor"] = frontSensor.readDistance();

	String obj = "";
	serializeJson(doc, obj);
	ws.textAll(obj);

	// Alerta com o buzzer dependendo da distância do sensor
	if (doc["frontSensor"] <= 12)
	{
		analogWrite(BUZZER_PIN, 120);
		delay(100);
		analogWrite(BUZZER_PIN, LOW);
	}
	else if (doc["frontSensor"] > 12 && doc["frontSensor"] <= 20)
	{
		analogWrite(BUZZER_PIN, 120);
		delay(200);
		analogWrite(BUZZER_PIN, LOW);
	}
	else if (doc["frontSensor"] > 20 && doc["frontSensor"] <= 28)
	{
		analogWrite(BUZZER_PIN, 120);
		delay(400);
		analogWrite(BUZZER_PIN, LOW);
	}

	delay(200);
}
