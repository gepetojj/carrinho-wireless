#include <ArduinoJSON.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <WiFi.h>

#include "ESPAsyncWebServer.h"
#include "ESPmDNS.h"
#include "distanceSensor.h"
#include "motor.h"
#include "pages.h"

#define AP_NAME "Carrinho Wireless"
#define PREF_SETUP_KEY "setupDone"
#define PREF_SSID_KEY "lastSSID"
#define PREF_PASS_KEY "lastPASS"

#define MDNS_DEVICE_NAME "Carrinho Wireless"
#define SERVICE_NAME "descobrimento"
#define SERVICE_PROTOCOL "udp"
#define SERVICE_PORT 5622

// Convenções:
// NP - Normalmente positivo
// NN - Normalmente negativo
// VEL - Pino que controla a velocidade (variação na frequência)

#define LEFT_WHEEL_NP_PIN 25
#define LEFT_WHEEL_NN_PIN 26
#define LEFT_WHEEL_VEL_PIN 33

#define RIGHT_WHEEL_NP_PIN 14
#define RIGHT_WHEEL_NN_PIN 27
#define RIGHT_WHEEL_VEL_PIN 12

#define FRONT_SENSOR_TRIG_PIN 32
#define FRONT_SENSOR_ECHO_PIN 35

#define BACK_SENSOR_TRIG_PIN 5
#define BACK_SENSOR_ECHO_PIN 17

Motor leftWheel(LEFT_WHEEL_NP_PIN, LEFT_WHEEL_NN_PIN, LEFT_WHEEL_VEL_PIN);
Motor rightWheel(RIGHT_WHEEL_NP_PIN, RIGHT_WHEEL_NN_PIN, RIGHT_WHEEL_VEL_PIN);

DistanceSensor frontSensor(FRONT_SENSOR_TRIG_PIN, FRONT_SENSOR_ECHO_PIN);
// DistanceSensor backSensor(BACK_SENSOR_TRIG_PIN, BACK_SENSOR_ECHO_PIN);

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

String wifiConnectionsProcessor(const String &var)
{
	int networks = WiFi.scanNetworks();
	if (networks == 0)
	{
		Serial.println("[WiFi Scan] Nenhuma conexão WiFi foi encontrada.");
		return "Erro! Tente novamente.";
	}

	for (int i = 0; i < networks; i++)
	{
		if (var == ("NW_" + String(i)))
		{
			return WiFi.SSID(i);
		}
	}

	return "Opção inválida.";
}

void setupCaptiveServer()
{
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *req)
			  { req->send_P(200, "text/html", CAPTIVE_PAGE, wifiConnectionsProcessor); });

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
	WiFi.softAP(AP_NAME);
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
			Serial.println("[Motors] Movimentando motores para a direita.");
			leftWheel.moveForward();
			rightWheel.moveBackward();
		}
		else if (strcmp((char *)data, "left") == 0)
		{
			Serial.println("[Motors] Movimentando motores para a esquerda.");
			leftWheel.moveBackward();
			rightWheel.moveForward();
		}
		else if (strcmp((char *)data, "stop") == 0)
		{
			Serial.println("[Motors] Parando motores.");
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

	default:
		break;
	}
}

// !! Métodos do lifecycle do microcontrolador !!

void setup()
{
	Serial.begin(115200);
	preferences.begin("prefs", false);

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
	if (!MDNS.begin(MDNS_DEVICE_NAME))
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
	ws.cleanupClients();

	// Envia para os clientes as leituras dos sensores em cm
	doc["frontSensor"] = frontSensor.readDistance();
	doc["backSensor"] = 0; // backSensor.readDistance();

	String obj = "";
	serializeJson(doc, obj);
	ws.textAll(obj);

	delay(200);
}
