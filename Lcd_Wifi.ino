#include <WiFiEsp.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

#define DEBUG

#define AP_SSID "embA"
#define AP_PASS "embA1234"
#define SERVER_NAME "10.10.14.46"
#define SERVER_PORT 5000
#define LOGID "KSW_ARD"
#define PASSWD "PASSWD"

#define WIFIRX 6
#define WIFITX 7

#define CMD_SIZE 50
#define ARR_CNT 7

char sendId[10] = "KSW_ARD";
char lcdLine1[17] = "Smart IoT By KSW";
char lcdLine2[17] = "WiFi Connecting!";
char sendBuf[CMD_SIZE];
SoftwareSerial wifiSerial(WIFIRX, WIFITX);
WiFiEspClient client;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();
  lcdDisplay(0, 0, lcdLine1);
  lcdDisplay(0, 1, lcdLine2);

#ifdef DEBUG
  Serial.begin(115200);
#endif
  wifi_Setup();
}

void loop() {
  if (client.available()) {
    socketEvent();
  }
}

void socketEvent() {
  int i = 0;
  char *pToken;
  char *pArray[ARR_CNT] = {0};
  char recvBuf[CMD_SIZE] = {0};
  int len;

  sendBuf[0] = '\0';
  len = client.readBytesUntil('\n', recvBuf, CMD_SIZE);
  client.flush();
#ifdef DEBUG
  Serial.print("recv : ");
  Serial.println(recvBuf);
#endif
  pToken = strtok(recvBuf, "[@]");
  while (pToken != NULL)
  {
    pArray[i] =  pToken;
    if (++i >= ARR_CNT)
      break;
    pToken = strtok(NULL, "[@]");
  }
  // Check if the received message is DATA
  if (!strcmp(pArray[1], "DATA")) {
    int turbidity = atoi(pArray[2]);
    int temperature = atof(pArray[3]);
    int level1 = atoi(pArray[4]);
    int level2 = atoi(pArray[5]);
    // Display data on the LCD
    sprintf(lcdLine1, "Tb:%d Tp:%d", turbidity, temperature);
    sprintf(lcdLine2, "Lv1:%dLv2:%d", level1, level2);

    lcdDisplay(0, 0, lcdLine1);
    lcdDisplay(0, 1, lcdLine2);
  }
}
void wifi_Setup() {
  wifiSerial.begin(38400);
  wifi_Init();
  server_Connect();
}
void wifi_Init()
{
  do {
    WiFi.init(&wifiSerial);
    if (WiFi.status() == WL_NO_SHIELD) {
#ifdef DEBUG_WIFI
      Serial.println("WiFi shield not present");
#endif
    }
    else
      break;
  } while (1);

#ifdef DEBUG_WIFI
  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(AP_SSID);
#endif
  while (WiFi.begin(AP_SSID, AP_PASS) != WL_CONNECTED) {
#ifdef DEBUG_WIFI
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(AP_SSID);
#endif
  }

#ifdef DEBUG_WIFI
  Serial.println("You're connected to the network");
  printWifiStatus();
#endif
}
int server_Connect()
{
#ifdef DEBUG_WIFI
  Serial.println("Starting connection to server...");
#endif

  if (client.connect(SERVER_NAME, SERVER_PORT)) {
#ifdef DEBUG_WIFI
    Serial.println("Connect to server");
#endif
    client.print("["LOGID":"PASSWD"]");
  }
  else
  {
#ifdef DEBUG_WIFI
    Serial.println("server connection failure");
#endif
  }
}

void printWifiStatus()
{
  // print the SSID of the network you're attached to

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void lcdDisplay(int x, int y, char *str) {
  int len = 16 - strlen(str);
  lcd.setCursor(x, y);
  lcd.print(str);
  for (int i = len; i > 0; i--)
    lcd.write(' ');
}
