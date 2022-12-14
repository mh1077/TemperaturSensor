// Speichermesser
// Programm für die Anzeige eines Wärmespeichers
// Das Programm benutzt den Temperaturfühler DS18B20 als Messgerät
// Durch zwei "OneWire" Ports können 6 Temperatursensoren die Speichertemperatur ermitteln
// und zwei weitere Sensoren den Vor- und Rücklauf des Heizkreises messen.
// Dadurch lässt sich sehr einfach feststellen, ob ein angeschlossener Holzoffen 
// noch weiter gefeuert werden muss oder ob bereits die gewünschte Wäremmenge
// im Speicher ist.
//
// V1.0 - Ersterstellung Michael Hörlein, 20.11.22
// V1.1 - Läuft bei Otto - Programmiert, 4.12.22
// 
// Todo:
// - Temperaturen in einer Datei speichern -> Log aufbauen
// - Logfiles über ftp/http erreichbar machen
// - Zeit über NTP in das Messgerät einlesen
// - Fehlererkennung an den Messchleifen einbauen
// - Schnittstelle für Anzeigegerät zur Verfügung stellen
// - Anzeigegerät bauen


// Temperatursensoren am Speichersensor:
// Adressen:
/*

Sensor gefunden, Index = 0 Adresse = 28C8990C00000076
Sensor gefunden, Index = 1 Adresse = 28D8EB0F0000004B
Sensor gefunden, Index = 2 Adresse = 28E4910B00000005
Sensor gefunden, Index = 3 Adresse = 28CEE30D0000009B
Sensor gefunden, Index = 4 Adresse = 28E92110000000AC
Sensor gefunden, Index = 5 Adresse = 28DF092E5A2001E6

Messung 2.12.22, ca. 18:00:

Temperatur Speichersensor 0 = 34.56 °C
Temperatur Speichersensor 1 = 34.56 °C
Temperatur Speichersensor 2 = 35.50 °C
Temperatur Speichersensor 3 = 34.69 °C
Temperatur Speichersensor 4 = 34.31 °C
Temperatur Speichersensor 5 = 42.44 °C

Temperatur Ofensensor 0 = -127.00 °C
Temperatur Ofensensor 1 = -127.00 °C

Sortierte Werte der Speichersensoren:
34.31
34.56
34.56
34.69
35.50
42.44


Messung vom 2.12.22 ca. 21:00:
Temperaturen vom Speicher

Sensor	Temperatur
1	67.00
2	50.81
3	40.56
4	38.44
5	35.56
6	34.13

Temperaturen vom Speicher unsortiert nach Index
Sensor	Temperatur
0	35.56	Addresse = 28C8990C00000076
1	38.44	Addresse = 28D8EB0F0000004B
2	50.81	Addresse = 28E4910B00000005
3	40.56	Addresse = 28CEE30D0000009B
4	34.13	Addresse = 28E92110000000AC
5	67.00	Addresse = 28DF092E5A2001E6

ToDo:
- Für Vorlauf und Rücklauf zwei unabhängige 1-Wire Busse bauen und Temperaturen messen
- FTP gängig machen
- 

*/

#define VERSION "V1.1"

#include <OneWire.h>            // One Wire Library
#include <DallasTemperature.h>  // Dallas Temperatur Library für DS18B20
#include <WiFi.h>               // WiFi Library
#include <FS.h>                 // this needs to be first, or it all crashes and burns...
#include <LittleFS.h>
#include <WiFiManager.h>        // WiFiManager Library
#include <ArduinoJson.h>        // Arduino JSON library   
#include <TickerScheduler.h>    // Ticker Library von toshik
 
#include <SimpleFTPServer.h>    // FTP Server von Renzo Mischianti

//needed for NTP 
#include "time.h"

#include "main.h"               // Eigene Headers

// JSON configuration file
#define JSON_CONFIG_FILE "/config.json"
 
// Flag for saving data
bool shouldSaveConfig = false;
 
// Define WiFiManager Object
WiFiManager wm;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS_SPEICHER 5
#define ONE_WIRE_BUS_OFEN 2
#define BUTTON_TO_RESET_WM 26   // If pressed on Start, Wifi Credentials are cleared

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWireSpeicher(ONE_WIRE_BUS_SPEICHER);
OneWire oneWireOfen(ONE_WIRE_BUS_OFEN);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensorsSpeicher(&oneWireSpeicher);
DallasTemperature sensorsOfen(&oneWireOfen);

// arrays to hold device addresses
DeviceAddress dASensorsSpeicher[10], dASensorsOfen[10];

char dASensorsSpeicherAsChar[10][20];
String dASensorsSpeicherAsString[10];

uint8_t maxSensorsSpeicher = 0;
uint8_t maxSensorsOfen = 0;

uint8_t addressSensorsSpeicher[10];
uint8_t addressSensorsOfen[10];

float temperatureCSensorsSpeicher[10];
float temperatureCSensorsSpeicherunsorted[10];
float temperatureCSensorsOfen[10];

int count = 0;

TickerScheduler ts(2);

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

//NTP
char ntpServer[40] = "ntp.metas.ch";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 7200;

//FTPServer
FtpServer ftpSrv;   
#define FTP_DEBUG             // to see ftp verbose on serial

//Datenspeicher für Logger
String  sLogg[1440];         // Stringarray für Daten, wird zu Mitternacht in eine Datei geschrieben
u_int   iZeigerLoggWrite = 0; // Zeiger zeigt auf die entsprechende Zelle in sLogg in die das nächste mal geschrieben wird.
bool    fDataSaved = false;   // Flag wird gesetzt, wenn Daten für den letzten Tag in das Flash gespeichert wurden

//IPAdresse vom localhost

IPAddress localIPAdress;      // IP Adresse des Geräts

void setup(void)  
{
  // Warte auf Start
  delay(1000);

  // Pin für Button konfigurieren
  pinMode(BUTTON_TO_RESET_WM, INPUT_PULLUP);

  // start serial port
  Serial.begin(9600);
  Serial.println("...s...");
  Serial.println("Der Gerät! Misst Temperatur eines Wäremsepeichers und des Vorlaufs und Rücklaufs ");
  Serial.println("zu einem Ofen. So kann festgestellt werden, ob noch Wärme benötigt wird, oder ");
  Serial.println("der Speicher bereits genügend gefüllt ist. Michael Hörlein, Winter 22.");
  Serial.print("Version: ");
  Serial.println(VERSION);
  Serial.println("Prüfe Temperatursensoren...");

  // Start up the library
  sensorsOfen.begin();
  sensorsSpeicher.begin();

  //Wie viele Temperatursensoren sind angeschlossen?

  maxSensorsSpeicher = sensorsSpeicher.getDeviceCount();
  maxSensorsOfen = sensorsOfen.getDeviceCount();

  Serial.print("Auf der Messleitung für den Speicher sind: ");
  Serial.print(maxSensorsSpeicher);
  Serial.println(" Sensoren angeschlossen.");

  Serial.print("Auf der Messleitung für den Ofen sind: ");
  Serial.print(maxSensorsOfen);
  Serial.println(" Sensoren angeschlossen.");
  
  for(uint8_t i = 0; i<10; i++)
  {
    if (sensorsSpeicher.getAddress(dASensorsSpeicher[i],i)) 
    {
      Serial.print("Sensor gefunden, Index = ");
      Serial.print(i);
      Serial.print(" Adresse = ");
      printAddress(dASensorsSpeicher[i]);
      dASensorsSpeicherAsString[i] = printAddressAsString(dASensorsSpeicher[i]);
      Serial.println("");
    }
    else 
    {
      Serial.print("Kein Sensor gefunden, Index = ");
      Serial.println(i);
    }
  }

  for(uint8_t i = 0; i<10; i++)
  {
    if (sensorsOfen.getAddress(dASensorsOfen[i], i)) 
    {
      Serial.print("Sensor gefunden, Index = ");
      Serial.print(i);
      Serial.print(" Adresse = ");
      printAddress(dASensorsOfen[i]);
      Serial.println("");
    }
    else 
    {
      Serial.print("Kein Sensor gefunden, Index = ");
      Serial.println(i);
    }
  }

  //Daten aus Datei lesen
  mountAndReadData();

  Serial.println("Konfiguriere WiFi....");

  // Change to true when testing to force configuration every time we run
  bool forceConfig = false;
 
  // Explicitly set WiFi mode
  WiFi.mode(WIFI_STA);
 
  // Reset settings if Button is pressed = 0
  if(digitalRead(BUTTON_TO_RESET_WM) == 0)
  { // Button is pressed
    Serial.println("Configbutton pressed -> Reset WifiCredentials....");
    wm.resetSettings();
  }
 
  // Set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);
 
  // Set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wm.setAPCallback(configModeCallback);
 
  // Text Captive Portal
  WiFiManagerParameter custom_text_info1("<br><b>Temperatursensor für Warmwasserspeicher</b><br>Einstellmen&uuml;<br>");
  WiFiManagerParameter custom_text_info2("Gebaut Winter 22 für Otto.<br><br>");
  WiFiManagerParameter custom_text_info3("Bitte das entsprechende WiFi wählen, Passwort eingeben und speichern.");
  WiFiManagerParameter custom_text_ntpserver("<br>Zeitserver zum Synchronisieren auf die Normalzeit (MEZ):");
  WiFiManagerParameter custom_ntp_server("NTP_Server", "ntp.server.de", ntpServer, 40);

  // Add all defined parameters
  wm.addParameter(&custom_text_info1);
  wm.addParameter(&custom_text_info2);
  wm.addParameter(&custom_text_info3);
  wm.addParameter(&custom_text_ntpserver);
  wm.addParameter(&custom_ntp_server);

  char ssid[23];
  snprintf(ssid, 23, "MCUDEVICE-%llX", ESP.getEfuseMac());
 
  if (forceConfig)
    // Run if we need a configuration
  {
    if (!wm.startConfigPortal(ssid))
    {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  else
  {
    if (!wm.autoConnect())
    {
      wm.resetSettings();
      Serial.println("failed to connect and hit timeout! cleared settings and start again...");
      delay(3000);
      // if we still have not connected restart and try all over again
      ESP.restart();
      delay(5000);
    }
  }
 
  // If we get here, we are connected to the WiFi
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  localIPAdress = WiFi.localIP();
 
  // Lets deal with the user config values
 
  // Save the custom parameters to FS
  if (shouldSaveConfig)
  {
    //Speichere die Daten....
    DataSaving();
  }

  // Wifi sollte sich immer wieder neu verbinden ..
  WiFi.setAutoReconnect(true);

  // NTP Konfigurieren
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Ticker starten
  // Temperatursensoren auslesen und Werte sortieren
  ts.add(0, 10000, [](void *) { messureAndSort(); }, nullptr, true);
  //ts.add(1, 3600000, [](void *) { syncTime(); }, nullptr, false);

  // Webserver starten
  server.begin();

  // FTPServer starten

  /*
  if(LittleFS.begin(true))
  {
    ftpSrv.setCallback(_callback);
    ftpSrv.setTransferCallback(_transferCallback);

    Serial.println("LittleFS opened!");
    ftpSrv.begin("user","user","Willkommen zum Datendownload vom Temperatursensor!");  
  }
  */
}

void loop(void)
{
  // Webserver 
  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) 
  {                                         // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) 
    {                                       // loop while the client's connected
      currentTime = millis();
      if (client.available()) 
      {                                      // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') 
        {                   
          // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) 
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            //client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>Speichertemperaturen</h1>");
            
            // Temperaturen vom Speicher
            client.println("<h2>Temperaturen vom Speicher</h2>");
            client.println("<table>");
            client.println("<tr>");
            client.println("<td>Sensor</td>");
            client.println("<td>Temperatur</tr>");
            client.println("</tr>");

            for (int i = 0; i < 6; i++)
            {
              client.print("<tr>");
              client.print("<td>");
              client.print(i+1);
              client.println("</td>");
              client.print("<td>");
              client.print(temperatureCSensorsSpeicher[5-i]);
              client.print("</td>");
              client.println("</tr>");
            }

            client.println("</table>");

            // Temperaturen vom Speicher unsortiert nach Index
            client.println("<h2>Temperaturen vom Speicher unsortiert nach Index</h2>");
            client.println("<table>");
            client.println("<tr>");
            client.println("<td>Sensor</td>");
            client.println("<td>Temperatur</tr>");
            client.println("</tr>");

            for (int i = 0; i < 6; i++)
            {
              client.print("<tr>");
              client.print("<td>");
              client.print(i);
              client.println("</td>");
              client.print("<td>");
              client.print(temperatureCSensorsSpeicherunsorted[i]);
              client.print("</td>");
              client.print("<td>");
              client.print("Addresse = ");
              client.print(dASensorsSpeicherAsString[i]);
              client.print("</td>");
              client.println("</tr>");
            }

            client.println("</table>");

            // Temperaturen vom Ofen
            client.println("<h2>Temperaturen vom Ofen</h2>");
            client.println("<table>");
            client.println("<tr>");
            client.println("<td>Sensor</td>");
            client.println("<td>Temperatur</tr>");
            client.println("</tr>");

            for (int i = 0; i < 2; i++)
            {
              client.print("<tr>");
              client.print("<td>");
              switch(i) 
              {
                case 0:
                  client.print("Vorlauf");
                  break;
                case 1:
                  client.print("R&uuml;cklauf");
                  break;
                default:
                  break;
              }
              client.println("</td>");
              client.print("<td>");
              client.print(temperatureCSensorsOfen[1-i]);
              client.print("</td>");
              client.println("</tr>");
            }

            client.println("</table>");

            client.println("<br>");

            localIPAdress = WiFi.localIP();

            client.print("<a href=\"ftp:////");
            client.print(localIPAdress);
            client.println("//\">Datendownload</a>");

            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else 
          { 
            // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') 
        {  
          // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  // FTP Zugriff Händeln
  //ftpSrv.handleFTP();

  

  // Ticker
  ts.update();
}

// Sortierfunktionen
void swap(float* xp, float* yp)
{
    float temp = *xp;
    *xp = *yp;
    *yp = temp;
}
 
// Function to perform Selection Sort
void selectionSort(float arr[], int n)
{
    int i, j, min_idx;
 
    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {
 
        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (arr[j] < arr[min_idx])
                min_idx = j;
 
        // Swap the found minimum element
        // with the first element
        swap(&arr[min_idx], &arr[i]);
    }
}

//Funktionen für den WifiManager
 
// Callback notifying us of the need to save configuration
void saveConfigCallback()
{
  Serial.println("Should save config");
  shouldSaveConfig = true;
}
 
// Called when config mode launched 
void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println("Entered Configuration Mode");
 
  Serial.print("Config SSID: ");
  Serial.println(myWiFiManager->getConfigPortalSSID());
 
  Serial.print("Config IP Address: ");
  Serial.println(WiFi.softAPIP());
}

// Routine zum Messen und Sortieren der Temperaturen, 
// wird zyklisch durch den Ticker alle 10 Sekunden aufgerufen

void messureAndSort(void) 
{
  Serial.println("messureAndSort() running...");

  // Temperaturen der Sensoren abfragen, Messung starten
  sensorsSpeicher.requestTemperatures(); 
  sensorsOfen.requestTemperatures();

  for (int i = 0 ; i < 6; i++ )
  {
    temperatureCSensorsSpeicher[i] = sensorsSpeicher.getTempCByIndex(i);
    temperatureCSensorsSpeicherunsorted[i] = temperatureCSensorsSpeicher[i];

    Serial.print("Temperatur Speichersensor ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(temperatureCSensorsSpeicher[i]);
    Serial.println(" °C");
  }

  Serial.println("");

  for (int i = 0 ; i < 2; i++ )
  {
    temperatureCSensorsOfen[i] = sensorsOfen.getTempCByIndex(i);

    Serial.print("Temperatur Ofensensor ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(temperatureCSensorsOfen[i]);
    Serial.println(" °C");
  }

  Serial.println("");

  // Werte sortieren

  selectionSort(temperatureCSensorsSpeicher, 6);
  Serial.println("Sortierte Werte der Speichersensoren:");
  for (int i = 0; i < 6; i++) 
  {
    Serial.println(temperatureCSensorsSpeicher[i]);
  }

  selectionSort(temperatureCSensorsOfen, 2);
  Serial.println("Sortierte Werte der Ofensensoren:");
  for (int i = 0; i < 2; i++) 
  {
    Serial.println(temperatureCSensorsOfen[i]);
  }
}

// Routine zum messen, sortieren und speichern

void messureSortAndSave(void)
{
  // Zuerst messen
  messureAndSort();

  // Werte abspeichern
  // Zeit abfragen
  time_t now;
  char strftime_buf[64];
  struct tm time;

  localtime_r(&now, &time);

  // Prüfen auf Datumswechsel
  if( time.tm_hour == 0 && time.tm_min == 0 && !fDataSaved )
  {
    // Datumswechsel -> Daten abspeichern
    saveDataToFile();
    iZeigerLoggWrite = 0;
  }

  if( time.tm_hour == 23 && time.tm_min == 59 && fDataSaved)
  {
    fDataSaved = false;
  }
  
  // Daten ins Ram schreiben
  char buffer[1048];
  int iZeigerZelle = 0;

  // Datum Tag, Monat, Jahr
  sprintf(buffer, "%d02" , time.tm_mday);
  sLogg[iZeigerLoggWrite] = buffer;
  sLogg[iZeigerLoggWrite] += ".";
  sprintf(buffer, "%d02", time.tm_mon);
  sLogg[iZeigerLoggWrite] += buffer; 
  sLogg[iZeigerLoggWrite] += "."; 
  sprintf(buffer, "%d04", time.tm_year+1900);
  sLogg[iZeigerLoggWrite] = buffer;
  sLogg[iZeigerLoggWrite] += " ";

  // Uhrzeit
  sprintf(buffer, "%d02", time.tm_hour);
  sLogg[iZeigerLoggWrite] += buffer; 
  sLogg[iZeigerLoggWrite] += ":";
  sprintf(buffer, "%d02", time.tm_min);
  sLogg[iZeigerLoggWrite] += buffer; 
  sLogg[iZeigerLoggWrite] += ":";
  sprintf(buffer, "%d02", time.tm_sec);
  sLogg[iZeigerLoggWrite] += buffer; 
  sLogg[iZeigerLoggWrite] += ";";

  // Zuerst die Ofentemperaturen
  for(int i = 0; i<6; i++)
  {
    sprintf(buffer, "%f.1", temperatureCSensorsSpeicher[i]);
    sLogg[iZeigerLoggWrite] += buffer; 
    sLogg[iZeigerLoggWrite] += ";";
  }

  // Nachher VL und RL
  for(int i = 0; i<2; i++)
  {
    sprintf(buffer, "%f.1", temperatureCSensorsOfen[i]);
    sLogg[iZeigerLoggWrite] += buffer; 
    sLogg[iZeigerLoggWrite] += ";";
  }

  // Zeiger auf die nächste Zeile erhöhen
  iZeigerLoggWrite++;
}

void saveDataToFile(void)
{
  Serial.println("Informationen zum Dateisystem:");
  Serial.printf("- Bytes total:   %ld\n", LittleFS.totalBytes());
  Serial.printf("- Bytes genutzt: %ld\n\n", LittleFS.usedBytes());

  File file = LittleFS.open("Datafile.csv","w", true);
  
  for(int i = 0; i <= iZeigerLoggWrite; i++)
  {
    for(int x = 0; x < sizeof(sLogg[i]); x++)
    {
      char c = sLogg[i][x];
      file.write(c);
    } 
    file.write('\r');
    file.write('\n');
  }

  file.close();
  fDataSaved = true;
}

void syncTime(void)
{
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

//Routine Schreibt die Daten ins EEProm vom ESP
int DataSaving(void)
{
  Serial.println("Saving Data to File...");
  DynamicJsonDocument json(2048);

  json["ntp_server"] = ntpServer;

  File configFile = LittleFS.open(JSON_CONFIG_FILE, "w");
  if (!configFile) {
     Serial.println("failed to open config file for writing");
     return -2;
  }

  serializeJson(json, configFile);

  configFile.close();
  
  Serial.println("Data saved:");
  serializeJson(json, Serial);
  Serial.println("");
  Serial.println("Data stored, File closed!");

  return 0;
}

void mountAndReadData(void) {

  //Filesystem mounten und Datei suchen, wenn Datei nicht vorhanden dann Neuerstellung
  if (LittleFS.begin(true)) {
    Serial.println("mounted file system");
    if (LittleFS.exists(JSON_CONFIG_FILE)) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = LittleFS.open(JSON_CONFIG_FILE, "r");
      if (configFile) {
        Serial.println("opened config file");

        DynamicJsonDocument jsonDocument(2048);

        // ArduinoJson 6
        DeserializationError error = deserializeJson(jsonDocument, configFile);
        Serial.print("\n-deserializeJson() -> output:");
        Serial.print(error.c_str());
        
        if (!error) 
        {
          Serial.println("\nparsed json =>  ");

          JsonVariant jvntpserver = jsonDocument["ntp_server"];
          if (!jvntpserver.isNull()) {
            Serial.print(" ntpserver=");
            Serial.print(jvntpserver.as<const char*>());
            strcpy(ntpServer, jvntpserver.as<const char*>());
          } else {
            strcpy(ntpServer, "ntp.metas.ch");
            Serial.print(" no value => ntp.metas.ch");
          }
        } else {
          Serial.println("\n->Error: unable to parse json!\nLoading Values!");
          strcpy(ntpServer, "ntp.metas.ch");

          //Konfiguration WLAN Credentials löschen
          wm.resetSettings();
          wm.setConnectTimeout(3);   //Timeout kleiner, muss eh neu konfiguriert werden
        }
      }
    }
  } else {
    //Config File exsistiert nicht!
    Serial.println("failed to mount FS");
    strcpy(ntpServer, "ntp.metas.ch");
    Serial.println("failed to load json config");
  }
  //end read
}


//Callbacks für FTPServer

void _callback(FtpOperation ftpOperation, unsigned int freeSpace, unsigned int totalSpace){
  Serial.println("Informationen zum Dateisystem:");
  Serial.printf("- Bytes total:   %ld\n", LittleFS.totalBytes());
  Serial.printf("- Bytes genutzt: %ld\n\n", LittleFS.usedBytes());
  switch (ftpOperation) {
    case FTP_CONNECT:
      Serial.println(F("FTP: Connected!"));
      break;
    case FTP_DISCONNECT:
      Serial.println(F("FTP: Disconnected!"));
      break;
    case FTP_FREE_SPACE_CHANGE:
      Serial.printf("FTP: Free space change, free %u of %u!\n", freeSpace, totalSpace);
      break;
    default:
      break;
  }
}

void _transferCallback(FtpTransferOperation ftpOperation, const char* name, unsigned int transferredSize){
  switch (ftpOperation) {
    case FTP_UPLOAD_START:
      Serial.println(F("FTP: Upload start!"));
      break;
    case FTP_UPLOAD:
      Serial.printf("FTP: Upload of file %s byte %u\n", name, transferredSize);
      break;
    case FTP_TRANSFER_STOP:
      Serial.println(F("FTP: Finish transfer!"));
      break;
    case FTP_TRANSFER_ERROR:
      Serial.println(F("FTP: Transfer error!"));
      break;
    default:
      break;
  }
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

// function to convert a device address into String
String printAddressAsString(DeviceAddress deviceAddress)
{
  String deviceAddressAsString;

  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    char temp[3] = "";
    //if (deviceAddress[i] < 16) deviceAddressAsString += String("0");
    sprintf(temp, "%02X" , deviceAddress[i]);
    deviceAddressAsString += String(temp);
  }

  return deviceAddressAsString;
}