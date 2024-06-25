#include <Wire.h> // for I2C communication
#include <WiFi.h> // for WiFi connection
#include <ESP_Mail_Client.h> // for sending emails using ESP32 SMTP
#include <LiquidCrystal_I2C.h> // for LCD display
#include <ESP8266SAM.h> // for audio output
#include <AudioOutputI2S.h> // for I2S audio output
#include <DFRobot_BMX160.h> // for magnetometer
#include <RTClib.h> // for real-time clock

// WiFi credentials
const char* ssid = ""; // WiFi SSID
const char* password = ""; // WiFi password
int x = 0;  // Initial value. This acts as a counter.
bool metal1 = false; // Ferromagnetic metal detection flag
bool metal2 = false; // Non-ferromagnetic metal detection flag
bool metal3 = false; // Copper detection flag
bool metal4 = false; // Brass detection flag
bool metal5 = false; // Gold detection flag

// SMTP server settings
#define SMTP_HOST "smtp.gmail.com" // SMTP server host
#define SMTP_PORT 465 // SMTP server port

// Email credentials
#define AUTHOR_EMAIL "" // Sender email address
#define AUTHOR_PASSWORD "ldvc dknh xtgl tqyr" // Sender email password
#define RECIPIENT_EMAIL "" // Recipient email address

// Multiplexer I2C address
#define MUX_ADDR 0x70

// BMX160 I2C address
#define BMX160_ADDR 0x68

// RTC module definition
RTC_DS3231 rtc;

// Days of the week for the RTC module
char daysOfWeek[7][12] = {
  "Sunday",
  "Monday",
  "Tuesday",
  "Wednesday",
  "Thursday",
  "Friday",
  "Saturday"
};

// LCD display I2C address
#define LCD_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

const int sensorPin = 17;  // Hall sensor connection pin

int buzzerPin = 5; // Buzzer pin

// I2S pins - Audio module connection pins. Should not be changed
const int I2S_DOUT = 25;
const int I2S_BCLK = 27;
const int I2S_LRC = 26;

const byte npulse = 3; // Number of pulses sent by the sensor

const byte pin_pulse = 32; // GPIO pin number where the pulse pin is connected
const byte pin_cap = 33; // GPIO pin number where the capacitor is connected
const byte pin_LED1 = 12; // GPIO pin number where the red LED is connected. If this LED is on, the material is ferromagnetic
const byte pin_LED2 = 14; // GPIO pin number where the green LED is connected. If this LED is on, the material is non-ferromagnetic
const byte pin_LED3 = 13; // GPIO pin number where the yellow LED is connected

// Manyetometre için obje oluşturma
DFRobot_BMX160 bmx160;
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLUMNS, LCD_ROWS);

//Multiplexer kanalı seçimi
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

AudioOutputI2S *out = NULL;
ESP8266SAM *sam = NULL;

/* Declare the global used SMTPSession object for SMTP transport */
SMTPSession smtp;

/* Declare the message class */
SMTP_Message message;

/* Declare the Session_Config for user defined session credentials */
Session_Config config;

/* Callback function to get the Email sending status */
void smtpCallback(SMTP_Status status) {
  Serial.println(status.info());

  if (status.success()) {
    Serial.println("----------------");
    Serial.printf("Message sent success: %d\n", status.completedCount());
    Serial.printf("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");

    for (size_t i = 0; i < smtp.sendingResult.size(); i++) {
      SMTP_Result result = smtp.sendingResult.getItem(i);
      Serial.printf("Message No: %d\n", i + 1);
      Serial.printf("Status: %s\n", result.completed ? "success" : "failed");
      Serial.printf("Recipient: %s\n", result.recipients.c_str());
      Serial.printf("Subject: %s\n", result.subject.c_str());
    }
    Serial.println("----------------\n");
    smtp.sendingResult.clear();
  }
}

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  Wire.begin(); // Start I2C communication

  pinMode(3, OUTPUT); // Set pin 3 as output

  pinMode(pin_pulse, OUTPUT); // Set pulse pin as output
  digitalWrite(pin_pulse, LOW); // Initialize pulse pin to low
  pinMode(pin_cap, INPUT); // Set capacitor pin as input
  pinMode(pin_LED1, OUTPUT); // Set LED1 pin as output
  digitalWrite(pin_LED1, LOW); // Initialize LED1 to off
  pinMode(pin_LED2, OUTPUT); // Set LED2 pin as output
  digitalWrite(pin_LED2, LOW); // Initialize LED2 to off
  pinMode(pin_LED3, OUTPUT); // Set LED3 pin as output
  digitalWrite(pin_LED3, HIGH); // Initialize LED3 to on

  // Initialize the LCD display (Multiplexer channel 2)
  selectMuxChannel(2);
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the LCD backlight
  lcd.print("Metal Detector"); // Display "Metal Detector" on the LCD
  delay(2000); // Wait for 2 seconds
  lcd.clear(); // Clear the LCD


  selectMuxChannel(3);
  // SETUP RTC MODULE
  if (!rtc.begin()) {
    Serial.println("RTC module is NOT found");
    Serial.flush();
    while (1);
  }

  // automatically sets the RTC to the date & time on PC this sketch was compiled
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  // BMX160 sensörünü başlat (Multiplexer kanal 1)
  selectMuxChannel(1);
  if (!bmx160.begin()) {
    Serial.println("BMX160 initialization failed");
    while (1);
  }

  // Connect to WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Initialize the audio output
  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  out->SetRate(16000); // Ensure the sample rate matches
  out->begin();

  // Initialize the SAM object
  sam = new ESP8266SAM();

  // Configure the mail client
  MailClient.networkReconnect(true);
  smtp.debug(1);
  smtp.callback(smtpCallback);

  config.server.host_name = SMTP_HOST;
  config.server.port = SMTP_PORT;
  config.login.email = AUTHOR_EMAIL;
  config.login.password = AUTHOR_PASSWORD;
  config.login.user_domain = "";
  config.time.ntp_server = F("pool.ntp.org,time.nist.gov");
  config.time.gmt_offset = 3;
  config.time.day_light_offset = 0;

 pinMode(buzzerPin, OUTPUT); // Set buzzer pin as output
}

void sendMail(const char* subject, const char* body) {
  message.sender.name = "ESP32 Metal Detector";
  message.sender.email = AUTHOR_EMAIL;
  message.subject = subject;
  message.addRecipient("", RECIPIENT_EMAIL);
  message.text.content = body;
  message.text.charSet = "us-ascii";
  message.text.transfer_encoding = Content_Transfer_Encoding::enc_7bit;
  message.priority = esp_mail_smtp_priority::esp_mail_smtp_priority_low;
  message.response.notify = esp_mail_smtp_notify_success | esp_mail_smtp_notify_failure | esp_mail_smtp_notify_delay;

  if (!smtp.connect(&config)) {
    Serial.printf("Connection error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
    return;
  }

  if (!MailClient.sendMail(&smtp, &message)) {
    Serial.printf("Error, Status Code: %d, Error Code: %d, Reason: %s\n", smtp.statusCode(), smtp.errorCode(), smtp.errorReason().c_str());
  }
}

const int nmeas = 256;  // Number of measurements
long int sumsum = 0; // Sum of 64 totals
long int skip = 0;   // Number of skips
long int diff = 0;    // Difference between total and average
long int flash_period = 0; // Flash period 
unsigned long prev_flash = 0; // Previous flash timestamp

void loop() {
  x++;
  //Serial.println(x);
  
  // Initialize variables
  int minval = 4095;
  int maxval = 0;
  long unsigned int sum = 0;
  
  // Read value from hall sensor, should be low if metal is detected
  int sensorValue = digitalRead(sensorPin);

  selectMuxChannel(1);
  sBmx160SensorData_t Omagn;
  bmx160.getAllData(&Omagn, NULL, NULL);

  float x1 = Omagn.x;
  float y = Omagn.y;
  float z = Omagn.z;

  // Additional oscillator circuit code is necessary for this magnetometer and hall sensor. Conditions must be determined according to it
  float magStrength = sqrt(x1 * x1 + y * y + z * z);

  //Serial.println(magStrength);

  // Declare formattedTime and dateTime outside the if block
  String formattedTime;
  String dateTime;

  selectMuxChannel(3);
  DateTime now = rtc.now();
  formattedTime = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  dateTime = String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + " " + daysOfWeek[now.dayOfTheWeek()];

  selectMuxChannel(2);
  lcd.setCursor(0, 1);
  lcd.print(formattedTime);

  if (x == 10) {
    selectMuxChannel(2);
    lcd.setCursor(0, 0);
    lcd.print("Hello!");
    digitalWrite(buzzerPin, HIGH); //buzzer çalışsın
    delay(500); 
    sam->Say(out, "Metal detector started");
    delay(1000);
  }
// Measurement loop
for (int imeas = 0; imeas < nmeas + 2; imeas++) {
  // Send pulse to capacitor and read
  pinMode(pin_cap, OUTPUT);
  digitalWrite(pin_cap, LOW);
  delayMicroseconds(20);
  pinMode(pin_cap, INPUT);

  // Pulse sending loop
  for (int ipulse = 0; ipulse < npulse; ipulse++) {
    digitalWrite(pin_pulse, HIGH);
    delayMicroseconds(3);
    digitalWrite(pin_pulse, LOW);
    delayMicroseconds(3);
  }

  // Read and update minimum, maximum, and sum values
  int val = analogRead(pin_cap);
  minval = min(val, minval);
  maxval = max(val, maxval);
  sum += val;

  // Calculate flash period and control LEDs
  unsigned long timestamp = millis();
  byte ledstat = 0;
  if (timestamp < prev_flash + 10) {
    if (diff > 0) ledstat = 1;
    if (diff < 0) ledstat = 2;
  }
  if (timestamp > prev_flash + flash_period) {
    if (diff > 0) ledstat = 1;
    if (diff < 0) ledstat = 2;
    prev_flash = timestamp;   
  }
  if (flash_period > 1000) ledstat = 0;

  // Set LEDs according to their statuses
  if (ledstat == 0) { // No metal
    digitalWrite(pin_LED1, LOW);
    digitalWrite(pin_LED2, LOW);
    digitalWrite(pin_LED3, HIGH);
  }
  if (ledstat == 1) { // Ferromagnetic metal
    digitalWrite(pin_LED1, HIGH);
    digitalWrite(pin_LED2, LOW);
    digitalWrite(pin_LED3, LOW);
  }
  if (ledstat == 2) { // Non-ferromagnetic metal
    digitalWrite(pin_LED1, LOW);
    digitalWrite(pin_LED2, HIGH);
    digitalWrite(pin_LED3, LOW);
  }
}

// Subtract minimum and maximum values
sum -= minval; sum -= maxval;

// Calculate average value and difference
if (sumsum == 0) sumsum = sum << 6; 
long int avgsum = (sumsum + 32) >> 6; 
diff = sum - avgsum;

// Update skip count based on the absolute value of difference
if (abs(diff) < avgsum >> 10) {      
  sumsum = sumsum + sum - avgsum;
  skip = 0;
} else {
  skip++;
}

// Check skip count and reset if necessary
if (skip > 64) {     
  sumsum = sum << 6;
  skip = 0;
}

// Calculate flash period based on the difference
if (diff == 0) flash_period = 1000000;
else flash_period = avgsum / (2 * abs(diff));

  Serial.print(nmeas); 
  Serial.print(" ");
  Serial.print(minval); 
  Serial.print(" ");
  Serial.print(maxval); 
  Serial.print(" ");
  Serial.print(sum); 
  Serial.print(" ");
  Serial.print(avgsum); 
  Serial.print(" ");
  Serial.print(diff); 
  Serial.print(" ");
  Serial.print(flash_period); 
  Serial.println();
  

  if (flash_period < 25 && flash_period > 10 && !metal1) { 
    digitalWrite(buzzerPin, LOW);
    selectMuxChannel(2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("It's aluminum!");
    lcd.setCursor(0, 1);
    lcd.print(formattedTime);
    sam->Say(out, "Aluminum detected");
    metal1 = true;

    String subject = "Metal Detected: Aluminum";
    String body = "Aluminum detected at " + formattedTime + " " + dateTime;
    sendMail(subject.c_str(), body.c_str());
    delay(500);
  }
  
  if (flash_period < 35 && flash_period > 25 && !metal2) {
    digitalWrite(buzzerPin, LOW); //buzzer çalışmasın
    selectMuxChannel(2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Iron detected");
    lcd.setCursor(0, 1);
    lcd.print(formattedTime);
    sam->Say(out, "Iron detected");
    metal2 = true;

    String subject = "Metal Detected: Iron";
    String body = "Iron detected at " + formattedTime + " " + dateTime;
    sendMail(subject.c_str(), body.c_str());
    delay(500);
  }
  
  if (flash_period < 70 && diff < -200 && !metal3) {
    digitalWrite(buzzerPin, LOW);
    selectMuxChannel(2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gold detected");
    lcd.setCursor(0, 1);
    lcd.print(formattedTime);
    sam->Say(out, "Gold detected");
    metal3 = true;

    String subject = "Metal Detected: Gold";
    String body = "Gold detected at " + formattedTime + " " + dateTime;
    sendMail(subject.c_str(), body.c_str());
    delay(500);
  }
  
  if (flash_period < 10 && !metal4) { 
    digitalWrite(buzzerPin, LOW);
    selectMuxChannel(2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Copper detected");
    lcd.setCursor(0, 1);
    lcd.print(formattedTime);
    sam->Say(out, "Copper detected");
    metal4 = true;

    String subject = "Metal Detected: Copper";
    String body = "Copper detected at " + formattedTime + " " + dateTime;
    sendMail(subject.c_str(), body.c_str());
    delay(500);
  }
  
  if (flash_period > 35 && flash_period < 45 && !metal5) { 
    digitalWrite(buzzerPin, LOW);
    selectMuxChannel(2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Brass detected");
    lcd.setCursor(0, 1);
    lcd.print(formattedTime);
    sam->Say(out, "Brass detected");
    metal5 = true;

    String subject = "Metal Detected: Brass";
    String body = "Brass detected at " + formattedTime + " " + dateTime;
    sendMail(subject.c_str(), body.c_str());
    delay(500);
  }
}