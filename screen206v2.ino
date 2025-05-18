#include <arduino.h>
#include <Wire.h>
#include <LittleFS.h>
#include <U8g2lib.h>            // Bibliothèque Ecran Oled
#include <Adafruit_GFX.h>       // Bibliothèque Ecran Oled 2                                         
#include <Adafruit_SSD1306.h>   // Bibliothèque Ecran Oled 2
#include <SPI.h>                // Bibliotheque SPI
#include <RTClib.h>             // Bibliothèque RTC
#include <DHT.h>                // Bibliothèque Sonde de Temp/hum
#include <Adafruit_BME280.h>    // Bibliothèque barometre
#include <Adafruit_MPU6050.h>   // Bibliothèque accelerometre
#include <Adafruit_Sensor.h>
#include <Chrono.h>             // Bibliotheque Chronometre
#include <cstdio>               // pour sprintf et sscanf
#include "Image.h"              // Fichier Image en BITMAP
#include "Bot.h"                // Fichier Image en BITMAP
#include "meteo.h"              // Fichier Image en BITMAP

// ---- OLED 1 (3.12 inches, 256x64, SSD1322) ----
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 15, /* dc=*/ 16, /* reset=*/ 17);

// ---- OLED 2 (0.96 inch, 128x64, SSD1306 emulation) ----
#define SCREEN_WIDTH 128 // OLED Largeur d'affichage, en pixels
#define SCREEN_HEIGHT  64 // OLED hauteur d'affichage, en pixels
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT,  &Wire, -1);

// ---- RTC ----
RTC_DS3231 rtc;

// ---- DHT11 ----
#define DHTPIN1 14
#define DHTPIN2 27
#define DHTTYPE DHT11
DHT htin(DHTPIN1, DHTTYPE);
DHT htout(DHTPIN2, DHTTYPE);

// ---- MPU6050 ----
Adafruit_MPU6050 mpu;

// ---- BME280 ----
Adafruit_BME280 bme;

// ---- Variable Bouton et Menu ----
unsigned long currentMillis;
int mode = 0; // mode oled2 0: Main, 1: Sec, 2: ...
int BUTTON_PIN_1 = 13;
int BUTTON_PIN_2 = 4;
int reading1 = HIGH;
int reading2 = HIGH;
int button1State = HIGH;
int button2State = HIGH;
int lastButton1State = HIGH;
int lastButton2State = HIGH;
unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long debounceDelay = 50;
unsigned long lastPressTime1 = 0;
unsigned long lastPressTime2 = 0;
const unsigned long longPressThreshold = 1500; // Seuil pour un appui long
int Antirebond = 200;

// ---- Variables Temps ----
int currentYear; 
int currentMonth; 
int currentDay; 
int currentHour; 
int currentMinute; 
int currentSecond;
bool isNight;

// ---- Variables du Chronometre ----
int hours;
int minutes;
int seconds;
int milli;
int smillis;

// ---- Variables for DHT sensors ----
unsigned long lastMeasurementTime = 0;
const unsigned long measurementInterval = 2000; // 2 secondes
float tempint; 
float humint; 
float tempext; 
float humext;

// ---- Variable Barometre ----
unsigned long lastUpdatebarMillis = 0;
const unsigned long updatebarInterval = 2000; // Mettre à 500 ms pour une mise à jour toutes les secondes
float altitude;
float pression;
float tempbar;
float humbar;

// ---- Variable principal Bot ----
unsigned long currentanimMillis;  // Variable commune pour l'horodatage actuel
unsigned long previousOledUpdateMillis = 0;
unsigned long oledUpdateInterval = 2500;         
unsigned long randomAnimationPreviousMillis = 0;
unsigned long randomAnimationInterval = 10000;
bool animationEnCours = false; 

// ---- BOT mode regard lateraux ----
unsigned long lookLPreviousMillis = 0;
const long lookLInterval = 2000;
int lookLState = 0;
unsigned long lookRPreviousMillis = 0;
const long lookRInterval = 2000;  // Durée entre les étapes de clignement
int lookRState = 0;  // 0: ouvert, 1: fermé

// ---- BOT mode clignement ----
unsigned long blinkPreviousMillis = 0;
unsigned long blinkInterval = 5000;
int blinkState = 0;

// ---- BOT mode surprise ----
unsigned long surprisePreviousMillis = 0;
const long surpriseInterval = 3000;
bool surpriseDisplayed = false;

// ---- BOT mode tremblement ----
unsigned long shakePreviousMillis = 0;
const long shakeInterval = 200;  // Intervalle de 250 ms pour l'effet de tremblement
int shakeStep = 0;
bool isShaking = false;

// ---- BOT mode saut ----
static unsigned long lastBumpTime = 0;
static float lastAccelZ = 0;  // Stocke la dernière valeur de l'accélération
static unsigned long bumpPreviousMillis = 0; // Rendre cette variable statique
static int bumpStep = 0;
static bool isBumping = false;
float threshold = 5.00;  // Seuil de déclenchement en m/s²
float setThreshold;
const long bumpInterval = 250;  // Intervalle pour l'effet de sautillement
const long refreshbump = 50;

// ---- BOT mode frein ----
//int breakState = 0;                   // État du freinage (0 = non, 1 = en cours)
//unsigned long breakPreviousMillis = 0; // Temps précédent pour le freinage
//unsigned long breakInterval = 200;    // Intervalle pour le freinage (ajustez comme nécessaire)
//float thresholdbreak = 5.00;  // Seuil de déclenchement en m/s²
//float setThresholdbreak = 2.00;              // Seuil d'accélération pour le freinage (ajustez comme nécessaire)

// ---- BOT mode sommeil ----
unsigned long lastMovementTime = 0; // Dernière fois que le mouvement a été détecté
const unsigned long sleepDelay = 120000; // Temps avant sommeil en millisecondes
bool isSleeping = false; // État de sommeil
bool displayFirstBitmap = true; // Indique quel bitmap afficher
unsigned long animationInterval = 300; // Intervalle d'animation en millisecondes
unsigned long lastAnimationMillis = 0; // Timestamp de la dernière animation

// ---- Variables for MPU6050 acceleration ----
unsigned long lastUpdateMillis = 0;
const unsigned long updateInterval = 50;
float accelX = 0;  // X acceleration values
float accelY = 0;  // Y acceleration values
float accelZ = 9.80;  // Z acceleration values
float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;

// ---- Variable Chronomètre ----
Chrono chronometre206;

// ---- Variables de gestion du temps ----
int setSecondes, setMinutes, setHeures, setJours, setMois, setAnnees;
int modeReglage = 0;  // 0 = pas en mode réglage, 1 = en mode réglage
int etapeReglage = 0; // 0 = heures, 1 = minutes, 2 = secondes, etc.
bool clignoter = false;
unsigned long previousMillis = 0;
const long intervalClignotement = 500; // Intervalle pour faire clignoter

// ---- Variables moniteur serie ----
unsigned long lastUpdateserialMillis = 0;
const unsigned long updateserialInterval = 500; // Mettre à 500 ms pour une mise à jour toutes les secondes

bool calibrating = true;      // Indique si on est en phase de calibration
unsigned long calibrationTime = 500;  // Temps de calibration en ms
unsigned long calibrationStart;       // Temps de début de la calibration

void setup(){

  Serial.begin(115200);  // ---- Initialise la communication série ----
  Serial.println("Démarrage du programme...");
      
  Wire.begin();  // ---- Initialize I2C communication ----

  u8g2.begin();  // ---- Initialize OLED 1 (3.12-inch SSD1322) ----
  Serial.println("Initialisation de l'écran OLED 1 OK");
    

  if(!display2.begin(SSD1306_SWITCHCAPVCC, 0x3C))  { // ---- Initialize OLED 2 (0.96-inch SSD1306 emulation) ----// Addresse i2c
    Serial.println(F("SSD1306 allocation failed"));                 
    for(;;);}  
  Serial.println("Initialisation de l'écran OLED 2 OK");

if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

// Restaurer les paramètres depuis l'EEPROM
  loadSettingsFromLittleFS();

// ---- Debut séquence de chargement ----                                                           
  u8g2.clearBuffer();                                               
  display2.clearDisplay();                                           
  u8g2.drawXBMP(0, 0, 256, 64, logo206oled1);     
  u8g2.setFont(u8g2_font_helvB08_tr);
  u8g2.drawStr(220, 64, "V2.1.2");                
  u8g2.sendBuffer();                                                 
  Serial.println("Affichage du logo OK");   
 
 // ---- Initialize ecran1 in ----                       
  display2.drawBitmap(2, 16, charge0, 124, 16, WHITE);
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Ecran 1");                
  display2.display();                              
  delay(350);   

// ---- Initialize ecran2 in ----                       
  display2.drawBitmap(2, 16, charge10, 124, 16, WHITE);
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Ecran 2");                
  display2.display();                              
  delay(350);    

  // ---- Initialize DHT in ----                       
  display2.drawBitmap(2, 16, charge20, 124, 16, WHITE);
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Sonde Interieur");                //
  display2.display();                                          
    htin.begin();
    Serial.println("Initialisation des capteurs DHTin OK");       
  delay(350);                  

// ---- Initialize DHT out ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge30, 124, 16, WHITE); 
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Sonde Exterieur");                
  display2.display();
    htout.begin();
    Serial.println("Initialisation des capteurs DHTout OK");
  delay(350); 

// ---- Initialize MPU6050 ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge40, 124, 16, WHITE);  
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Accelerometre");                
  display2.display();
    if (!mpu.begin(0x69)) {
    Serial.println("Erreur: La connexion au MPU6050 a échoué.");
    while (1);  // Bloque le programme si l'accelerometre ne fonctionne pas
    } 
    else {
    Serial.println("MPU6050 connecté avec succès.");
    }

  delay(400); 
    accelZ = 9.80;             // Initialisation pour éviter les faux déclenchements
    lastAccelZ = 9.80;         // Initialisation de la valeur précédente à la gravité
    calibrationStart = millis();  // Début de la calibration

// ---- Initialize BME280 ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge50, 124, 16, WHITE);  
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Barometre");                
  display2.display();
    if(!bme.begin(0x76)) {
    Serial.println(F("Erreur: Le BME ne peut pas être initialisé."));
    while(1);   // Bloque le programme si le BME ne fonctionne pas
    } 
    else {
    Serial.println(F("BME initialisé correctement."));
    }
  delay(400); 

// ---- Buttons ----  
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge60, 124, 16, WHITE);  
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Boutons");                
  display2.display();
    pinMode(BUTTON_PIN_1, INPUT_PULLUP);
    pinMode(BUTTON_PIN_2, INPUT_PULLUP);
    initializeButtons();
    Serial.println("Initialisation des boutons OK"); 
  delay(350);   

// ---- Initialisation du module DS 3231 ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge70, 124, 16, WHITE);     
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Puce Temps Reel");             
  display2.display();
    if (!rtc.begin()) {
    Serial.println("Erreur: Le RTC ne peut pas être initialisé.");
    while (1);  // Bloque le programme si RTC ne fonctionne pas
    } else {
    Serial.println("RTC initialisé correctement.");
    }
  delay(400);

// ---- Initialisation du Reglage de l'heure ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge80, 124, 16, WHITE); 
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Reglage de l'heure");                 
  display2.display();
  // **********************************************
  // Réglage d'une date/heure, au televersement
  // **********************************************
    //uint16_t  setAnnees     = 2024;     // de 2000 à 2099
    //uint8_t   setMois      = 10;        // de 1 à 12
    //uint8_t   setJours      = 7;       // de 1 à 31
    //uint8_t   setHeures     = 0;       // de 0 à 23
    //uint8_t   setMinutes   = 59;       // de 0 à 59
    //uint8_t   setSecondes  = 00;        // de 0 à 59
    //rtc.adjust(DateTime(setAnnees, setMois, setJours, setHeures, setMinutes, setSecondes));
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  ///////////////////////////////////////////////////////////////////////////////////
    adjustVarTime();
    adjustVarDate();
  delay(400); 

// ---- Initialisation du Chronometre ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge90, 124, 16, WHITE); 
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Chronometre");                 
  display2.display();
    chronometre206 = Chrono(Chrono::MILLIS, false);    // Ne pas démarrer le chronomètre automatiquement
  delay(350); 

// ---- Initialisation Terminé ----
  display2.clearDisplay(); 
  display2.drawBitmap(2, 16, charge100, 124, 16, WHITE);  
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(10, 1);
  display2.print("Chargement");
  display2.setCursor(10, 40);
  display2.print("Terminer");                
  display2.display();
  delay(1000);                                                       
  u8g2.clearBuffer();                                                
  display2.clearDisplay();                                           
    Serial.println("Fin de setup().");
}


/////////////////////////////DEBUG/////A ACTIVER EN FIN DE BOUCLE//////////////////////////////////////
void serial() { 

    if (millis() - lastUpdateserialMillis >= updateserialInterval) {

    // Affichage de l'horloge'
    //Serial.print(F("Horloge = "));
    //Serial.print((currentHour), DEC); 
    //Serial.print(':'); 
    //Serial.print((currentMinute), DEC);
    //Serial.print(':'); 
    //Serial.print((currentSecond), DEC);
    //Serial.println();

    // Affichage de la date
    //Serial.print(F("Date = "));
    //Serial.print((currentDay), DEC); 
    //Serial.print('/'); 
    //Serial.print((currentMonth), DEC);
    //Serial.print('/'); 
    //Serial.print((currentYear), DEC);
    //Serial.println();

    // Affichage du chronometre
    //Serial.print(F("Chronometre = "));
    //Serial.print((hours), DEC); 
    //Serial.print(':'); 
    //Serial.print((minutes), DEC);
    //Serial.print(':'); 
    //Serial.print((seconds), DEC);
    //Serial.print(':'); 
    //Serial.print((milli), DEC);
    //Serial.print((smillis), DEC);
    //Serial.println(); 

    // Affichage de la TEMPÉRATURE
    //Serial.print(F("Température bar = "));
    //Serial.print((tempbar), DEC);
    //Serial.println(F(" °C"));

    // Affichage du TAUX D'HUMIDITÉ
    //Serial.print(F("Humidité bar = "));
    //Serial.print((humbar), DEC);
    //Serial.println(F(" %"));
    
    // Affichage de la TEMPÉRATURE
    //Serial.print(F("Température int = "));
    //Serial.print((tempint), DEC);
    //Serial.println(F(" °C"));

    // Affichage du TAUX D'HUMIDITÉ
    //Serial.print(F("Humidité int = "));
    //Serial.print((humint), DEC);
    //Serial.println(F(" %"));
  
    // Affichage de la TEMPÉRATURE
    //Serial.print(F("Température ext = "));
    //Serial.print((tempext), DEC);
    //Serial.println(F(" °C"));

    // Affichage du TAUX D'HUMIDITÉ
    //Serial.print(F("Humidité ext = "));
    //Serial.print((humext), DEC);
    //Serial.println(F(" %"));

    // Affichage de la PRESSION ATMOSPHÉRIQUE
    //Serial.print(F("Pression atmosphérique = "));
    //Serial.print((pression), DEC);
    //Serial.println(F(" hPa"));

    // Affichage de l'estimation d'ALTITUDE
    //Serial.print(F("Altitude estimée = "));
    //Serial.print((altitude), DEC);
    //Serial.println(F(" m"));

    // Affichage accelX
    //Serial.print(F("accelX = "));
    //Serial.print((accelX));
    //Serial.println();

    // Affichage accelY
    //Serial.print(F("accelY = "));
    //Serial.print((accelY));
    //Serial.println();

    // Affichage accelZ
    //Serial.print(F("accelZ = "));
    //Serial.print((accelZ));
    //Serial.println();

    //Serial.print("État sleepbitmap : ");
    //Serial.println(displayFirstBitmap ? "ON" : "OFF");

    lastUpdateserialMillis = millis(); // Update the last update timestamp
    //Serial.println();
  }
}
////////////////////////////////////////////END DEBUG////////////////////////////////////////////

////////////////////////////////////////////FONCTIONS////////////////////////////////////////////
void loadSettingsFromLittleFS() {
  File file = LittleFS.open("/settings.txt", "r");
  if (!file) {
    Serial.println("Failed to open settings file");
    setThreshold = 8.00; // Valeur par défaut
    //setThresholdbreak = 8.00; // Valeur par défaut
    return;
  }

  // Lire la première ligne pour setThreshold
  String line = file.readStringUntil('\n');
  setThreshold = line.toFloat();

  // Lire la deuxième ligne pour setThresholdbreak
  //line = file.readStringUntil('\n');
  //setThresholdbreak = line.toFloat();

  file.close();
  threshold = setThreshold;
  //thresholdbreak = setThresholdbreak;
}

void saveSettingsToLittleFS() {
  File file = LittleFS.open("/settings.txt", "w");
  if (!file) {
    Serial.println("Failed to open settings file");
    return;
  }

  // Écrire setThreshold dans le fichier
  file.println(String(setThreshold));
  // Écrire setThresholdbreak dans le fichier
  //file.println(String(setThresholdbreak));

  file.close();
}

void miseenpage() {
  u8g2.drawLine(0, 40, 256, 40);
  u8g2.drawLine(232, 40, 232, 64);
  u8g2.drawLine(100, 40, 100, 64);
  u8g2.drawLine(138, 0, 138, 40);
}

void displayTime() {
    adjustVarTime();
  u8g2.setFont(u8g2_font_profont29_tn); // Large font
  u8g2.setCursor(5, 28); // Position in the display
  u8g2.printf("%02d:%02d:%02d", currentHour, currentMinute, currentSecond);
}

void adjustVarTime(){
    DateTime currentTime = rtc.now();
    currentHour = currentTime.hour();
    currentMinute = currentTime.minute();
    currentSecond = currentTime.second();
}

void displayDate() {
    adjustVarDate();    
  u8g2.setFont(u8g2_font_profont17_tn); // Smaller font for date
  u8g2.setCursor(5, 59); // Position in the display
  u8g2.printf("%02d/%02d/%02d", currentDay, currentMonth, currentYear);
}

void adjustVarDate(){
    DateTime currentTime = rtc.now();
    currentYear = currentTime.year();
    currentMonth = currentTime.month();
    currentDay = currentTime.day();
}

void displayChrono() {
  unsigned long elapsedTime = chronometre206.elapsed();
  hours = elapsedTime / 3600000;
  minutes = (elapsedTime % 3600000) / 60000;
  seconds = (elapsedTime % 60000) / 1000;
  milli = (elapsedTime % 1000) / 100;
  smillis = (elapsedTime % 100) / 10;

  u8g2.drawXBMP(104, 43, 20, 20, chrono);
  u8g2.setFont(u8g2_font_profont17_tn); // Smaller font for chrono
  u8g2.setCursor(128, 59); // Position in the display
  u8g2.printf("%02d:%02d:%02d:%01d%01d", hours, minutes, seconds, milli, smillis); // Format the time 
}

void readAcceleration() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (currentMillis - lastUpdateMillis >= updateInterval) {

      // Stocker les valeurs arrondies à trois décimales sous forme d'entiers en millièmes
      int accelX_milli = round(a.acceleration.x * 1000);
      int accelY_milli = round(a.acceleration.y * 1000);
      int accelZ_milli = round(a.acceleration.z * 1000);

      // Utiliser ces valeurs pour afficher les résultats avec trois décimales
      //printf("Accélération X : %.3f\n", accelX_milli / 1000.0);
      //printf("Accélération Y : %.3f\n", accelY_milli / 1000.0);
      //printf("Accélération Z : %.3f\n", accelZ_milli / 1000.0);

      // Inverser les valeurs si nécessaire
      accelX_milli = -accelX_milli;
      accelY_milli = -accelY_milli;

      // Mettre à jour les valeurs pour les utiliser dans des calculs ultérieurs, si besoin
      accelX = accelX_milli / 1000.0;
      accelY = accelY_milli / 1000.0;
      accelZ = accelZ_milli / 1000.0;

      gyroX = g.gyro.x;
      gyroY = g.gyro.y;
      gyroZ = g.gyro.z;

      accelX *= -1;  // Invert the value of AccelX
      accelY *= -1;  // Invert the value of AccelY
      
    lastUpdateMillis = currentMillis;
  }
}

void readBarometre() {

  if (currentMillis - lastUpdatebarMillis >= updatebarInterval) {

  tempbar = bme.readTemperature();
  humbar = bme.readHumidity();
  pression = bme.readPressure() / 100.0F;  
  altitude = bme.readAltitude(1029);

    lastUpdatebarMillis = currentMillis;
  }
}

void adjustBrightness() {
  if (isNight) {
    display2.ssd1306_command(0x81);  // Commande pour régler le contraste oled2
    display2.ssd1306_command(1);  // Valeur de contraste entre 0 et 255 oled2
    u8g2.setContrast(1); // Valeur de contraste entre 0 et 255 oled1
  } else {
      display2.ssd1306_command(0x81);  // Commande pour régler le contraste oled2
      display2.ssd1306_command(255);  // Valeur de contraste entre 0 et 255 oled2
      u8g2.setContrast(255); // Valeur de contraste entre 0 et 255 oled1
    }
}

void temphum(){

if (currentMillis - lastMeasurementTime >= measurementInterval) {
    lastMeasurementTime = currentMillis;

  tempint = htin.readTemperature();
  humint = htin.readHumidity();
  tempext = htout.readTemperature();
  humext = htout.readHumidity();

  if (isnan(tempint)) {
    tempint = tempbar;  // Remplacer par tempbar si NaN
  } 
  if (isnan(humint)) {
    humint = humbar;  // Remplacer par humbar si NaN
  }
}
  
  u8g2.setFont(u8g2_font_helvB10_tr);         // Choisir une police de caractères
  u8g2.drawStr(145, 15, "INT");               // Afficher "Temp:"
  char tempintBuffer[10];                     // Convertir la température en chaîne de caractères
  dtostrf(tempint, 4, 1, tempintBuffer);      // Convertit en chaîne (avec 1 décimale)
  u8g2.drawStr(174, 15, tempintBuffer);       // Afficher la température
  u8g2.drawCircle(204, 6, 2, U8G2_DRAW_ALL);  // Ajouter le symbole °C après la température
  u8g2.drawStr(207, 15, "C");        
  char humintBuffer[10];
  dtostrf(humint, 3, 0, humintBuffer);        // Convertit en chaîne (avec 1 décimale)
  u8g2.drawStr(218, 15, humintBuffer);        // Afficher l'humidité
  u8g2.drawStr(240, 15, "%");                 // Ajouter le symbole pourcentage
    
  u8g2.drawStr(143, 34, "EXT");               // Afficher "Temp:"
  char tempextBuffer[10];                     // Convertir la température en chaîne de caractères
  dtostrf(tempext, 4, 1, tempextBuffer);      // Convertit en chaîne (avec 1 décimale)
  u8g2.drawStr(174, 34, tempextBuffer);       // Afficher la température
  u8g2.drawCircle(204, 25, 2, U8G2_DRAW_ALL); // Ajouter le symbole °C après la température
  u8g2.drawStr(207, 34, "C");                
  char humextBuffer[10];
  dtostrf(humext, 3, 0, humextBuffer);        // Convertit en chaîne (avec 1 décimale)
  u8g2.drawStr(218, 34, humextBuffer);        // Afficher l'humidité
  u8g2.drawStr(240, 34, "%");                 // Ajouter le symbole pourcentage
}

void updateDayNightStatus(int currentHour, int currentMonth) {
    // Initialisation des heures de lever et coucher du soleil selon la saison
    int sunriseHour, sunsetHour;
    
    if (currentMonth == 12 || currentMonth == 1 || currentMonth == 2) {
        // Hiver
        sunriseHour = 8;
        sunsetHour = 17;
    } else if (currentMonth == 3 || currentMonth == 4 || currentMonth == 5) {
        // Printemps
        sunriseHour = 7;
        sunsetHour = 20;
    } else if (currentMonth == 6 || currentMonth == 7 || currentMonth == 8) {
        // Été
        sunriseHour = 6;
        sunsetHour = 21;
    } else if (currentMonth == 9 || currentMonth == 10 || currentMonth == 11) {
        // Automne
        sunriseHour = 7;
        sunsetHour = 19;
    }
    
    // Vérifier si l'heure actuelle est dans la plage de nuit
    if (currentHour < sunriseHour || currentHour >= sunsetHour) {
        isNight = true;  // Il fait nuit
    } else {
        isNight = false;  // Il fait jour
    }
}

void meteo() {
    // Mettre à jour l'état jour/nuit en fonction de l'heure et du mois
    updateDayNightStatus(currentHour, currentMonth);
    
    if (currentMonth == 12 || currentMonth == 1 || currentMonth == 2) {
        iconmeteohiver();  // Afficher les icônes pour la période hivernale
    } 
    else if (currentMonth == 3 || currentMonth == 4 || currentMonth == 5) {
        iconmeteoprintemps(); // Afficher les icônes pour la période printanière
    }
    else if (currentMonth == 6 || currentMonth == 7 || currentMonth == 8) {
        iconmeteoete(); // Afficher les icônes pour la période estivale
    }
    else if (currentMonth == 9 || currentMonth == 10 || currentMonth == 11) {
        iconmeteoautomne(); // Afficher les icônes pour la période automnale
    }
}

void iconmeteoete() {
    // Affichage d'icônes basé sur les conditions estivales
    if (tempext > 30) {  // Température très élevée
        u8g2.drawXBMP(233, 41, 23, 23, hot);  // Afficher l'icône de chaleur
    } 
    else if (pression < 985 || (pression < 995 && humext > 70)) {  
        u8g2.drawXBMP(233, 41, 23, 23, pluie);  // Pluie
    } 
    else if (pression < 995) {  
        u8g2.drawXBMP(233, 41, 23, 23, nuage);  // Nuageux
    } 
    else if (pression < 1015) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, nuagelune);  // Afficher icône nuage avec lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, nuagesol);  // Afficher icône soleil avec nuages
        }
    } 
    else if (pression >= 1015) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, lune);  // Afficher l'icône de la lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, soleil);  // Afficher l'icône du soleil
        }
    }
}

void iconmeteohiver() {
    // Affichage d'icônes basé sur les conditions hivernales
    if (tempext < 0 && pression < 1000) {  
        u8g2.drawXBMP(233, 41, 23, 23, neige);  // Afficher l'icône de neige
    } 
    else if (tempext < 0) {  
        u8g2.drawXBMP(233, 41, 23, 23, gel);  // Afficher l'icône de gel
    } 
    else if (pression < 990 || (pression < 1000 && humext > 75)) {  
        u8g2.drawXBMP(233, 41, 23, 23, pluie);  // Pluie
    } 
    else if (pression < 1000) {  
        u8g2.drawXBMP(233, 41, 23, 23, nuage);  // Nuageux
    } 
    else if (pression < 1025) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, nuagelune);  // Nuage avec lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, nuagesol);  // Soleil avec nuages
        }
    } 
    else if (pression >= 1025) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, lune);  // Lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, soleil);  // Soleil
        }
    }
}

void iconmeteoprintemps() {
    if (tempext > 25) {  
        u8g2.drawXBMP(233, 41, 23, 23, hot);  // Chaleur
    } 
    else if (pression < 985 || (pression < 995 && humext > 70)) {  
        u8g2.drawXBMP(233, 41, 23, 23, pluie);  // Pluie
    } 
    else if (pression < 995) {  
        u8g2.drawXBMP(233, 41, 23, 23, nuage);  // Nuageux
    } 
    else if (pression < 1020) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, nuagelune);  // Nuage avec lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, nuagesol);  // Soleil avec nuages
        }
    } 
    else if (pression >= 1020) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, lune);  // Lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, soleil);  // Soleil
        }
    }
}

void iconmeteoautomne() {
    if (pression < 985 || (pression < 995 && humext > 80)) {   
        u8g2.drawXBMP(233, 41, 23, 23, pluie);  // Pluie
    } 
    else if (pression < 1005) {  
        u8g2.drawXBMP(233, 41, 23, 23, nuage);  // Nuageux
    } 
    else if (pression < 1020) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, nuagelune);  // Nuage avec lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, nuagesol);  // Soleil avec nuages
        }
    } 
    else if (pression >= 1020) {  
        if (isNight) {
            u8g2.drawXBMP(233, 41, 23, 23, lune);  // Lune
        } else {
            u8g2.drawXBMP(233, 41, 23, 23, soleil);  // Soleil
        }
    }
}

void drawFace() {
  display2.clearDisplay();
  display2.drawBitmap(10, 15, eye_L, 40, 17, WHITE);  // Oeil gauche
  display2.drawBitmap(78, 15, eye_R, 40, 17, WHITE);  // Oeil droit
  display2.drawBitmap(29, 45, smile, 70, 7, WHITE);   // Bouche
  display2.display();
}

void checkRandomAnimation() {
if (!animationEnCours && !isShaking && !isBumping) {  // Si aucune animation n'est en cours

  if (currentanimMillis - randomAnimationPreviousMillis >= randomAnimationInterval) {
    randomAnimationPreviousMillis = currentanimMillis;

    int randomAnimation = random(0, 4);     // Choisir une animation aléatoire
    
    switch(randomAnimation) {
      case 0:
        drawBlinkingEyes();  // Clignement des yeux
        break;
      case 1:
        drawSurpriseFace();  // Visage surpris
        break;
      case 2:
        lookL();
        break;
      case 3:
        lookR();
        break;
    }
    randomAnimationInterval = random(1500, 10000);
    }
  }
}

void lookL() {
  animationEnCours = true;

  if (lookLState == 0 && currentanimMillis - lookLPreviousMillis >= lookLInterval) {
    lookLPreviousMillis = currentanimMillis;
    
    display2.clearDisplay();
    display2.drawBitmap(10, 15, eye_L_look_L, 40, 17, WHITE);  // Oeil gauche
    display2.drawBitmap(78, 15, eye_R_look_L, 40, 17, WHITE);  // Oeil droit
    display2.drawBitmap(29, 45, smile, 70, 7, WHITE);          // Bouche
    display2.display();

    lookLState = 1;
    oledUpdateInterval = random(1000, 4000);
  } 

  if (lookLState == 1 && currentanimMillis - lookLPreviousMillis >= oledUpdateInterval) {  
    drawFace();  // Retourner à l'état normal du visage
    lookLState = 0;  // Remettre l'état à zéro pour la prochaine animation
    oledUpdateInterval = 0;
    lookLPreviousMillis = currentanimMillis;  // Réinitialiser le temps précédent
  }
  animationEnCours = false;
}

void lookR() {
  animationEnCours = true;

  if (lookRState == 0 && currentanimMillis - lookRPreviousMillis >= lookRInterval) {
    lookRPreviousMillis = currentanimMillis;

    display2.clearDisplay();
    display2.drawBitmap(10, 15, eye_L_look_R, 40, 17, WHITE);  // Oeil gauche
    display2.drawBitmap(78, 15, eye_R_look_R, 40, 17, WHITE);  // Oeil droit
    display2.drawBitmap(29, 45, smile, 70, 7, WHITE);          // Bouche
    display2.display();

    lookRState = 1;
    oledUpdateInterval = random(1000, 4000);
  } 

  if (lookRState == 1 && currentanimMillis - lookRPreviousMillis >= oledUpdateInterval) {  
    drawFace();  // Retourner à l'état normal du visage
    lookRState = 0;  // Remettre l'état à zéro pour la prochaine animation
    oledUpdateInterval = 0;
    lookRPreviousMillis = currentanimMillis;  // Réinitialiser le temps précédent
  }
  animationEnCours = false;
}
 
void detectSleep() {
  // Si un mouvement est détecté, réinitialiser le timer de sommeil
  if (isMovementDetected(accelX, accelY, gyroX, gyroY, gyroZ)) {
    resetMovementTimer(); // Réinitialise le timer de mouvement
    if (isSleeping) {
      wakeUp(); // Réveille le robot si le mode sommeil est activé
    }
  }
  
  // Si aucun mouvement n'est détecté depuis sleepDelay, activer le mode sommeil
  if (!isSleeping && (currentanimMillis - lastMovementTime > sleepDelay)) {
    isSleeping = true;              // Active le mode sommeil
    lastAnimationMillis = currentanimMillis; // Initialiser le timer d'animation de sommeil
  }

  // Si le mode sommeil est activé, exécuter l'animation de sommeil
  if (isSleeping) {
    sleepAnimation();
  }
}

void sleepAnimation() {
  if (isSleeping) {
    // Alterne entre les deux images de l'animation de sommeil
    if (currentanimMillis - lastAnimationMillis >= animationInterval) {
      display2.clearDisplay(); // Efface l'affichage avant de dessiner

      // Affiche l'image selon l'état actuel
      if (displayFirstBitmap) {
        showFirstAnimationFrame(); // Affiche la première image de l'animation
      } else {
        showSecondAnimationFrame(); // Affiche la deuxième image de l'animation
      }
      display2.display(); // Actualise l'affichage

      // Alterne entre les deux images pour la prochaine animation
      displayFirstBitmap = !displayFirstBitmap;
      lastAnimationMillis = currentanimMillis; // Met à jour le temps de la dernière animation
    }
  }
}

void drawBlinkingEyes() {
  if (blinkState == 0 && currentanimMillis - blinkPreviousMillis >= blinkInterval) {
    blinkPreviousMillis = currentanimMillis;
    animationEnCours = true;
    display2.clearDisplay();
    display2.drawBitmap(10, 15, eye_L_BLINK, 40, 17, WHITE); 
    display2.drawBitmap(78, 15, eye_R_BLINK, 40, 17, WHITE); 
    display2.drawBitmap(29, 45, smile, 70, 7, WHITE);
    display2.display();

    blinkState = 1;
    oledUpdateInterval = random(500, 1000);
  }

  if (blinkState == 1 && currentanimMillis - blinkPreviousMillis >= oledUpdateInterval) {
    drawFace();
    blinkState = 0;
    oledUpdateInterval = 0;
    blinkPreviousMillis = currentanimMillis;
  }
  animationEnCours = false;
}

void drawFilledOval(int16_t x, int16_t y, int16_t width, int16_t height, uint16_t color) {
  for (int16_t i = -width / 2; i <= width / 2; i++) {
      for (int16_t j = -height / 2; j <= height / 2; j++) {
          if (pow((float)i / (width / 2), 2) + pow((float)j / (height / 2), 2) <= 1) {
              display2.drawPixel(x + i, y + j, color);
          }
      }
  }
}

void drawSurpriseFace() {
  if (currentanimMillis - surprisePreviousMillis >= surpriseInterval) {
    surprisePreviousMillis = currentanimMillis;

    if (!surpriseDisplayed) {
      animationEnCours = true;
      display2.clearDisplay();
      display2.drawBitmap(10, 15, eye_L, 40, 17, WHITE);  // Oeil gauche
      display2.drawBitmap(78, 15, eye_R, 40, 17, WHITE);  // Oeil droit
      display2.drawBitmap(29, 45, Bouchesurprise, 70, 7, WHITE);
      display2.display();
      surpriseDisplayed = true;
      oledUpdateInterval = random(1000, 2500);
    } else {
      drawFace();
      surpriseDisplayed = false;      
      oledUpdateInterval = 0;
    }
  }
  animationEnCours = false;
}

void detectBumpAndJumpEyes() {
    // Pendant la phase de calibration, on ignore les chocs
    if (calibrating) {
        if (millis() - calibrationStart < calibrationTime) {
            lastAccelZ = accelZ;  // Met à jour lastAccelZ sans déclencher isBumping
            return;
        } else {
            calibrating = false; // Fin de la calibration
        }
    }

    if (currentanimMillis - lastBumpTime >= refreshbump) {  // Vérifie chaque 50 ms
        lastBumpTime = currentanimMillis;

        // Vérifie si le changement d'accélération dépasse le seuil
        if (!isBumping && (abs(accelZ - lastAccelZ) > threshold)) {
            isBumping = true;         // Activer l'animation de sautillement
            bumpStep = 0;             // Réinitialiser l'animation
            bumpPreviousMillis = currentanimMillis;  // Démarrer le timer
        }
        lastAccelZ = accelZ;  // Sauvegarde la dernière valeur
    }

    // Gestion de l'animation de saut
    if (isBumping) {
        if (currentanimMillis - bumpPreviousMillis >= bumpInterval) {
            bumpPreviousMillis = currentanimMillis;

            // Appeler la fonction pour dessiner le visage avec un saut
            drawFaceWithJump(bumpStep % 2 == 0 ? 10 : -10);
            bumpStep++;

            // Arrêter l'animation après 8 étapes
            if (bumpStep >= 8) {
                isBumping = false;  
                bumpStep = 0;  // Réinitialiser bumpStep
            }
        }
    }
}

void drawFaceWithJump(int offsetY) {
  display2.clearDisplay();
  display2.drawBitmap(10, 15 + offsetY, eye_L_croix, 40, 17, WHITE);
  display2.drawBitmap(78, 15 + offsetY, eye_R_croix, 40, 17, WHITE);
  display2.drawBitmap(29, 45 + offsetY, smile, 70, 7, WHITE);
  display2.display();
}

//void detectBraking() {
    // Calcul de la variation de l'accélération sur l'axe X
//    static float previousAccelX = accelX;
//    float deceleration = previousAccelX - accelX;
//    previousAccelX = accelX;

    // Vérifie si la décélération dépasse le seuil pour déclencher le freinage
//    if (deceleration > thresholdbreak) {
//        if (breakState == 0) { // Démarre l'animation de freinage
//            breakPreviousMillis = currentanimMillis; // Enregistre le temps de départ
//            breakState = 1;
//        }
//    }

    // Si l'animation de freinage est en cours, afficher le visage de freinage
//    if (breakState == 1 && currentanimMillis - breakPreviousMillis < breakInterval) {
//        display2.clearDisplay();
//        display2.drawBitmap(10, 15, eye_L, 40, 17, WHITE);  // Oeil gauche
//        display2.drawBitmap(78, 15, eye_R, 40, 17, WHITE);  // Oeil droit
//        display2.drawBitmap(45, 29, bouchebreak, 43, 43, WHITE);
//        display2.display();
//    } else if (breakState == 1) {
        // Rétablit l'affichage normal après l'animation de freinage
//        drawFace();
//        breakState = 0;
//    }
//}

void checkTemperatureAndShakeFace() {
    // Détection de température anormale pour activer le tremblement une seule fois
    if (!isShaking && (tempint < 5)) {
        isShaking = true;  // Active l'animation de tremblement
        shakeStep = 0;     // Réinitialise l'animation
        shakePreviousMillis = currentanimMillis;  // Enregistre le temps de départ
    }

    // Si l'animation de tremblement est active
    if (isShaking) {
        // Vérifie l'intervalle de temps pour chaque étape de tremblement
        if (currentanimMillis - shakePreviousMillis >= shakeInterval) {
            shakePreviousMillis = currentanimMillis;  // Met à jour le dernier temps
            
            // Calcule les offsets pour chaque étape de tremblement
            int offsetX = (shakeStep % 2 == 0) ? 2 : -2;
            int offsetY = (shakeStep % 2 == 0) ? 2 : -2;
            drawFaceWithShake(offsetX, offsetY);
            shakeStep++;
            
            // Arrête l'animation après 10 étapes
            if (shakeStep >= 10) {
                isShaking = false;  // Désactive l'animation de tremblement
            }
        }
    }
}

void drawFaceWithShake(int offsetX, int offsetY) {
    display2.clearDisplay();
    display2.drawBitmap(10 + offsetX, 15 + offsetY, eye_L, 40, 17, WHITE);  // Oeil gauche
    display2.drawBitmap(78 + offsetX, 15 + offsetY, eye_R, 40, 17, WHITE);  // Oeil droit
    display2.drawBitmap(29 + offsetX, 45 + offsetY, Bouchetempshake, 70, 7, WHITE);   // Bouche
    display2.display();
}

bool isMovementDetected(float ax, float ay, float gx, float gy, float gz) {
  const float movementThreshold = 0.80; // Seuil pour la détection de l'accélération
  const float gyroThreshold = 0.60; // Seuil pour la détection du gyroscope (valeurs plus petites pour rotations lentes)

  // Vérifier si les valeurs d'accélération dépassent le seuil
  bool accelMovement = abs(ax) > movementThreshold || abs(ay) > movementThreshold;

  // Vérifier si les valeurs du gyroscope dépassent le seuil
  bool gyroMovement = abs(gx) > gyroThreshold || abs(gy) > gyroThreshold || abs(gz) > gyroThreshold;

  // Retourner vrai si un mouvement est détecté soit par l'accéléromètre, soit par le gyroscope
  return accelMovement || gyroMovement;
}

void showFirstAnimationFrame() {
  display2.drawBitmap(10, 15, eye_L_BLINK, 40, 17, WHITE); // Œil gauche
  display2.drawBitmap(78, 15, eye_R_BLINK, 40, 17, WHITE); // Œil droit
  display2.drawBitmap(29, 45, Bouchesleep, 70, 7, WHITE); // Bouche
}

// Fonction pour afficher le deuxième état d'animation
void showSecondAnimationFrame() {
  display2.drawBitmap(10, 15, eye_L_BLINK, 40, 17, WHITE); // Œil gauche
  display2.drawBitmap(78, 15, eye_R_BLINK, 40, 17, WHITE); // Œil droit
  display2.drawBitmap(29, 45, Bouchesleep2, 70, 7, WHITE); // Bouche alternative
}

void resetMovementTimer() {
  lastMovementTime = millis(); // Réinitialise le temps du dernier mouvement
  isSleeping = false; // Réinitialise l'état de sommeil
}

void wakeUp() {
  isSleeping = false;           // Désactiver le mode sommeil
  lastMovementTime = currentanimMillis; // Réinitialiser le timer de mouvement
  display2.clearDisplay();       // Effacer l'affichage au réveil
  drawFace();                    // Affiche le visage au réveil
  display2.display();
}

void afficherValeur(const char* label, float valeur, int x, int y) {
  char buffer[10];
  dtostrf(valeur, 4, 2, buffer); // Convertit la valeur en chaîne de caractères (4 caractères minimum, 2 décimale)
  display2.setCursor(x, y);
  display2.print(label);
  display2.print(buffer);
}

void oled2main() {
  currentanimMillis = millis();

  // Gestion du mode sommeil
  detectSleep();

  if (isSleeping) {
    // Si le robot est en mode sommeil, exécute uniquement l'animation de sommeil
    sleepAnimation();
  } else {
    // Si le robot n'est pas en mode sommeil, exécuter les autres animations
    if (!isShaking && !isBumping) {
      drawFace();               // Mise à jour de la position des yeux selon le gyroscope
      checkRandomAnimation();    // Vérifier si une animation aléatoire doit être lancée
    }
    if (!isShaking) {
      detectBumpAndJumpEyes();   // Réaction aux bosses
    }
    //detectBraking();             // Détection de freinage
    checkTemperatureAndShakeFace(); // Vérifier la température pour activer le tremblement
  }
}

void  oled2sec(){     
  display2.clearDisplay();  
  display2.setTextSize(2);
  display2.setTextColor(WHITE);
  
  // Afficher les angles cumulés plutôt que les vitesses angulaires
  display2.setCursor(2, 0);
  display2.print("Altitude:");
  afficherValeur("", altitude, 0, 17);
  display2.setCursor(112, 17);
  display2.print("m");
  display2.setCursor(2, 34);
  display2.print("Pression");
  afficherValeur("", pression, 0, 50);
  display2.setCursor(92, 50);
  display2.print("hpa");
  
  display2.display();  // Mettre à jour l'affichage
}

void oled2ter() {    
  display2.clearDisplay();  
  display2.setTextSize(2);
  display2.setTextColor(WHITE);
  
  // Afficher les angles cumulés plutôt que les vitesses angulaires
  afficherValeur("aX:", accelX, 2, 0);
  afficherValeur("aY:", accelY, 2, 22);
  afficherValeur("aZ:", accelZ, 2, 44);
  
  display2.display();  // Mettre à jour l'affichage
}

void oled2quar() {    
  display2.clearDisplay();  
  display2.setTextSize(2);
  display2.setTextColor(WHITE);
  
  // Afficher les angles cumulés plutôt que les vitesses angulaires
  afficherValeur("GX:", gyroX, 2, 0);
  afficherValeur("GY:", gyroY, 2, 22);
  afficherValeur("GZ:", gyroZ, 2, 44);
  
  display2.display();  // Mettre à jour l'affichage
}

void functionButton2() {
    Serial.println("Bouton 2 pressé");
  if (chronometre206.isRunning()) {
    chronometre206.stop(); // Arrêter le chronomètre
  } else {
    chronometre206.resume(); // Démarrer ou reprendre le chronomètre
  }
}

void functionBothButtonsPressed() {
    if (!chronometre206.isRunning()) {   // Vérifie si le chronomètre n'est pas en cours d'exécution avant d'activer le mode réglage ou réinitialiser
        Serial.println("Les deux boutons sont pressés ensemble");
        resetChrono();
    }
}

void resetChrono() {
    Serial.println("Reset Chronometre");
  if (!chronometre206.isRunning()) { // Si le chronomètre est à l'arrêt
    chronometre206.restart(0);
    chronometre206 = Chrono(Chrono::MILLIS, false); // Réinitialise et stoppe le chronomètre
  }
}

void initializeButtons() {
  reading1 = digitalRead(BUTTON_PIN_1);
  reading2 = digitalRead(BUTTON_PIN_2);
  lastButton1State = reading1;
  lastButton2State = reading2;
}

void incrementerValeur() {
  switch (etapeReglage) {
    case 0: setHeures = (setHeures + 1) % 24; break;
    case 1: setMinutes = (setMinutes + 1) % 60; break;
    case 2: setSecondes = (setSecondes + 1) % 60; break;
    case 3: setJours = (setJours % 31) + 1; break;
    case 4: setMois = (setMois % 12) + 1; break;
    case 5: if (setAnnees < 2040) {setAnnees++;} 
            else {setAnnees = 2020;} // Redémarre à 2020 après 2040
            break;
    case 6: if (setThreshold < 25) {setThreshold++;}
            else {setThreshold = 1;}
            break;
   // case 7: if (setThresholdbreak < 15) {setThresholdbreak++;}
   //         else {setThresholdbreak = -15;}
   //         break;
  }
}

void afficherReglage() {
  if (modeReglage) {  // ---- Gestion du clignotement en mode réglage ----
    if (currentMillis - previousMillis >= intervalClignotement) {
      previousMillis = currentMillis;
      clignoter = !clignoter; // Inverse l'état de clignotement
    }
  }

  u8g2.clearBuffer();
  miseenpage();

  char heuresStr[3], minutesStr[3], secondesStr[3];
  char joursStr[3], moisStr[3], anneesStr[5]; // Années avec 4 chiffres
  char thresholdStr[10];
  //char thresholdbreakStr[10];

  sprintf(heuresStr, "%02d", setHeures);
  sprintf(minutesStr, "%02d", setMinutes);
  sprintf(secondesStr, "%02d", setSecondes);
  u8g2.setFont(u8g2_font_profont29_tn);
  u8g2.setCursor(5, 28);
  u8g2.printf(
    "%s:%s:%s", 
    (etapeReglage == 0 && clignoter) ? "  " : heuresStr, // Heures clignotantes
    (etapeReglage == 1 && clignoter) ? "  " : minutesStr, // Minutes clignotantes
    (etapeReglage == 2 && clignoter) ? "  " : secondesStr // Secondes clignotantes
  );
  sprintf(joursStr, "%02d", setJours);
  sprintf(moisStr, "%02d", setMois);
  sprintf(anneesStr, "%04d", setAnnees); // Années avec 4 chiffres
  u8g2.setFont(u8g2_font_profont17_tn);
  u8g2.setCursor(5, 59);
  u8g2.printf(
    "%s/%s/%s", 
    (etapeReglage == 3 && clignoter) ? "  " : joursStr,  // Jours clignotants
    (etapeReglage == 4 && clignoter) ? "  " : moisStr,   // Mois clignotants
    (etapeReglage == 5 && clignoter) ? "    " : anneesStr // Années clignotantes
  );

  u8g2.setFont(u8g2_font_profont12_tr);
  u8g2.drawStr(160, 12, "REGLAGES");

  sprintf(thresholdStr, "%.1f", setThreshold);

  u8g2.setFont(u8g2_font_profont10_tr);
  u8g2.drawStr(142, 22, "Detec saut: ");
  u8g2.setCursor(217, 22);
  u8g2.printf(
    "%s",
    (etapeReglage == 6 && clignoter) ? "     " : thresholdStr
  );
  u8g2.drawStr(236, 22, " m/s²"); 

  //sprintf(thresholdbreakStr, "%.1f", setThresholdbreak);

  //u8g2.setFont(u8g2_font_profont10_tr);
  //u8g2.drawStr(142, 32, "Detec frein: ");
  //u8g2.setCursor(217, 32);
  //u8g2.printf(
  //  "%s",
  //  (etapeReglage == 7 && clignoter) ? "     " : thresholdbreakStr
  //);
  //u8g2.drawStr(236, 32, " m/s²"); 

  u8g2.sendBuffer();  // Envoi à l'écran
}

//////////////////////////////////////////END FONCTION/////////////////////////////////////////////////

void loop() {
    currentMillis = millis();
    reading1 = digitalRead(BUTTON_PIN_1); // Lecture d'états bouton1
    reading2 = digitalRead(BUTTON_PIN_2); // Lecture d'états bouton2
    bool bothButtonsPressed = (reading1 == LOW) && (reading2 == LOW); // Appui simultané (LOW = pressé)

if (modeReglage) {    // Mode réglage activé
  if (bothButtonsPressed) {   // Gestion de l'appui simultané des deux boutons en mode reglage
    rtc.adjust(DateTime(setAnnees, setMois, setJours, setHeures, setMinutes, setSecondes));
    threshold = setThreshold;
    //thresholdbreak = setThresholdbreak;
    saveSettingsToLittleFS();
    modeReglage = 0;
    Serial.println("Nouvelles valeurs ajustées dans le RTC");
    clignoter = false;
    delay(Antirebond); // Anti-rebond
  } else {  
      if (reading2 == LOW && reading1 == HIGH) { // Gestion du bouton 2 en mode reglage 
          if (millis() - lastPressTime2 < longPressThreshold) {
            incrementerValeur();
            delay(Antirebond); // Anti-rebond
          }
        lastPressTime2 = millis();
      }
    lastButton2State = reading2;

      if (reading1 == LOW && reading2 == HIGH) {  // Gestion du bouton 1 en mode reglage 
          if (millis() - lastPressTime1 < longPressThreshold) {
            Serial.println("Bouton 1 pressé");
            etapeReglage = (etapeReglage + 1) % 7; // Passer au champ suivant (heures -> minutes -> secondes -> jours -> mois -> année)
            delay(Antirebond); // Délai anti-rebond
          } 
        lastPressTime1 = millis();
      }
    lastButton1State = reading1;
    }
} else {
    if (bothButtonsPressed) {
        if (chronometre206.elapsed() == 0) { //obtient le temps écoulé
            if (modeReglage == 0) { // Si on n'est PAS en mode réglage
                modeReglage = 1; // Passer en mode réglage
                setHeures = currentHour;
                setMinutes = currentMinute;
                setSecondes = currentSecond;
                setJours = currentDay;
                setMois = currentMonth;
                setAnnees = currentYear;
                setThreshold = threshold;
                //setThresholdbreak = thresholdbreak;
                etapeReglage = 0; // Commence par régler les heures
                Serial.println("Entrée en mode réglage");
                delay(Antirebond); // Anti-rebond
            }
        } else {
            Serial.println("Le chronomètre n'est pas a 0, ne peut pas entrer en mode réglage. si il est a l'arret = reset");
            functionBothButtonsPressed();
            delay(Antirebond); // Anti-rebond
          }
    } else{
        if (reading1 == LOW && reading2 == HIGH) { // Gestion du bouton 1
            if (millis() - lastPressTime1 < longPressThreshold) {
              Serial.println("Bouton 1 pressé");
              mode = (mode + 1) % 4; // Alterne entre les modes
              delay(Antirebond); // Délai anti-rebond
            }
          lastPressTime1 = millis();
        }
        lastButton1State = reading1;
        if (reading2 == LOW && reading1 == HIGH) { // Gestion du bouton 2
            if (millis() - lastPressTime2 < longPressThreshold) {
              functionButton2(); // Gérer le chronomètre
              delay(Antirebond); // Anti-rebond
            }
          lastPressTime2 = millis();
        }
      lastButton2State = reading2;
    }
}

  // ---- Affichage (3.12 pouces) ----
  u8g2.clearBuffer(); // Efface le tampon avant d'afficher sur OLED 1 
  if (modeReglage) {  // Affichage conditionnel basé sur le mode
    afficherReglage(); // Affichage en mode réglage
  } else {
    adjustBrightness(); // Ajustement de la luminosité en fonction du capteur de lumière   
    readAcceleration();
    readBarometre();
    miseenpage();
    displayTime();  // Affiche l'heure
    displayDate();  // Affiche la date
    displayChrono(); // Affiche le chronomètre
    temphum(); // Affiche la température et l'humidité
    meteo();
    }

  u8g2.sendBuffer(); // Envoie le tampon au OLED 1

  switch (mode) {
    case 0: // Main
      if (currentMillis - previousOledUpdateMillis >= oledUpdateInterval) {
        previousOledUpdateMillis = currentMillis;
        oled2main();
      } break;
    case 1: // Sec
        oled2sec();
        break;
    case 2: // ter
        oled2ter();
        break;
    case 3: // quar
        oled2quar();
        break;
  }

  //serial(); // DEBUG serial PC 
}