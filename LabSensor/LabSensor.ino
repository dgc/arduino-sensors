/* LabSensor.ino */

#define OUT_CHANNEL "Arduino_Test"
#define CONTROL_CHANNEL OUT_CHANNEL "_Control"
#define DEFAULT_PERIOD 60
#define DEFAULT_MAC { 0x90, 0xA2, 0xDA, 0x0D, 0x0A, 0x77 } /* 0x77 / 0x3B */
#define MQTT_SERVER { 152, 78, 131, 193 }
#define MQTT_PORT 1883
#define NTP_PERIOD (60 * 15) /* 15 minutes */

#define DHT22_PIN 5

#define SETTINGS_MARKER 12783 /* Randomly chosen 15 bit number */

#define DIGITAL_PIN_OFFSET 2
#define DIGITAL_PIN_COUNT 14
#define ANALOG_PIN_OFFSET 16
#define ANALOG_PIN_COUNT 6
#define PERIOD_OFFSET 22
#define MAC_OFFSET 24

#define DEBUG

#ifdef DEBUG
#define DPRINT(x) Serial.print(x)
#define DPRINTLN(x) Serial.println(x)
#else /* DEBUG */
#define DPRINT(x) /* */
#define DPRINTLN(x) /* */
#endif /* DEBUG */

#include <stdio.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <JsonParser.h>
#include <dht.h>
#include <SimpleTimer.h>
#include <Time.h>

using namespace ArduinoJson::Parser;

// DHT support

dht DHT;

// EEPROM support

int getInt8(int offset) {
  return EEPROM.read(offset);
}

void setInt8(int offset, int value) {
  EEPROM.write(offset, value & 255);
}

int getInt16(int offset) {
  return (EEPROM.read(offset + 1) << 8) | EEPROM.read(offset);
}

void setInt16(int offset, int value) {
  EEPROM.write(offset + 1, (value >> 8) & 255);
  EEPROM.write(offset, value & 255);
}

IPAddress timeServer(152, 78, 128, 60); // ntp0.soton.ac.uk NTP server

byte mac[] = DEFAULT_MAC;

byte server[] = MQTT_SERVER;

int active_read_sensors = 0;
int active_write_sensors = 0;

SimpleTimer timer;
int timer_id;

EthernetClient ethClient;
PubSubClient client(server, MQTT_PORT, processConfiguration, ethClient);

boolean validSettings() {
  return getInt16(0) == SETTINGS_MARKER;
}

void resetSettings() {

  setInt16(0, SETTINGS_MARKER);

  for (int i = 0; i < DIGITAL_PIN_COUNT; i++)
    setInt8(DIGITAL_PIN_OFFSET + i, 0);

  for (int i = 0; i < ANALOG_PIN_COUNT; i++)
    setInt8(ANALOG_PIN_OFFSET + i, 0);

  //  for (int i = 0; i < 6; i++)
  //    setInt8(MAC_OFFSET + i, 

  setInt16(PERIOD_OFFSET, DEFAULT_PERIOD);
}

// ---------------------------------------------------------------------

// NTP code derived from http://arduino.cc/en/Tutorial/UdpNTPClient

unsigned int localPort = 2390;      // local port to listen for UDP packets

const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message

byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets

// A UDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)

  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void checkForNTPResponse() {

  if ( Udp.parsePacket() ) {

    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;

    // now convert NTP time into everyday time:

    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;

    DPRINT("Time set (via NTP) to ");
    DPRINTLN(epoch);

    // Set the onboard clock to the received time.
    setTime(epoch);
  }
}

// ---------------------------------------------------------------------

void scheduledNTP() {
  sendNTPpacket(timeServer);
  delay(1000);
  timer.setTimeout(1000, checkForNTPResponse);
}

// ---------------------------------------------------------------------

int analogRead2(int pin) {

  // Read once to switch the ADC channel.
  analogRead(pin);
  delay(50);

  // Read a second time to get an accurate reading.
  return analogRead(pin);
}

void publishString(PubSubClient &client, char *topic, String message) {

  int len = message.length() + 1;

  char arr[len];

  message.toCharArray(arr, len);

  client.publish(topic, arr);
}

void sensorRead() {
  //DPRINTLN("sensorRead(): start");
  String message = "";

  int dht_status = DHT.read22(DHT22_PIN);


  message = message + "{\"timestamp\":" + now();
  message = message + ",\"A0\":" + analogRead2(A0);
  message = message + ",\"A1\":" + analogRead2(A1);

  if (dht_status == DHTLIB_OK) {
    message = message + ",\"DHT_TEMP\":" + ((int) DHT.temperature);
    message = message + ",\"DHT_HUMIDITY\":" + ((int) DHT.humidity);
  }

  message = message + "}";
  //message = message + ",\"A0\":" + analogRead2(A0) + "}";

  DPRINTLN(message);
  publishString(client, OUT_CHANNEL, message);
  //DPRINTLN("sensorRead(): end");
}

void processConfiguration(char* topic, byte* payload, unsigned int length) {

  DPRINTLN((char *) payload);

  char arr[length + 1];

  strcpy(arr, (char *) payload);
  arr[length] = '\0';

  DPRINTLN(arr);

  JsonParser<32> parser;

  JsonObject root = parser.parse(arr);

  if (root.success()) {

    long period = root["period"];

    if (period > 0) {
      timer.deleteTimer(timer_id);
      setInt16(PERIOD_OFFSET, period);
      timer_id = timer.setInterval(period * 1000, sensorRead);
    }
  }
}

void setup()
{
#ifdef DEBUG

  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  char buf[20];

  sprintf(buf, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  DPRINT("MAC address: ");
  DPRINTLN(buf);

#endif /* DEBUG */

  // Reset to default settings if the EEPROM hasn't been set yet.

  if (!validSettings())
    resetSettings();

  DPRINT("Sample period: ");
  DPRINTLN(getInt16(PERIOD_OFFSET));

  int ethernetStatus = Ethernet.begin(mac);

  if (ethernetStatus == 0) {
    DPRINTLN("Could not configure the ethernet connection.");
  } 
  else {
    DPRINTLN("Connected to ethernet via DHCP.");
  }

  if (client.connect("arduinoClient")) {
    DPRINTLN("Connected to MQTT server.");
    client.subscribe(CONTROL_CHANNEL);
  }   
  else {
    DPRINTLN("Could not connect to server.");
  }

  timer_id = timer.setInterval(((long) getInt16(PERIOD_OFFSET)) * 1000, sensorRead);
  timer.setInterval(((long) NTP_PERIOD) * 1000, scheduledNTP);
  //  setInt32(0, 64738);
  //  Serial.println(getInt32(0));

  // Get initial NTP time

  Udp.begin(localPort);

  sendNTPpacket(timeServer);
  delay(1000);
  checkForNTPResponse();
}

void loop()
{
  //DPRINTLN("loop(): start");
  timer.run();
  //DPRINTLN("loop(): done timer");
  client.loop();
  //DPRINTLN("loop(): done client");
}




















