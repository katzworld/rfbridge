#include <Arduino.h>
#include <string.h>
#include <WiFi.h>
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 1024
#endif
#include <PubSubClient.h>
#include <rtl_433_ESP.h>
#include <LittleFS.h>

// rtl_433_ESP owns its CC1101 instance internally; reuse it for TX (replay).
extern CC1101 radio;

// ================= WiFi (EDIT THESE) =================
const char* WIFI_SSID     = "xxxxxxx";
const char* WIFI_PASSWORD = "xxxxxxxxxx";

// ================= MQTT =================
const char*   MQTT_BROKER = "192.168.x.x";
const uint16_t MQTT_PORT   = 1883;
const char*   MQTT_CLIENT  = "esp32-rf433";
const char*   HA_PREFIX    = "homeassistant";
const char*   MQTT_USERNAME = "mqtt";  // optional
const char*   MQTT_PASSWORD = "mqtt";  // optional

// State topics
const char* TOPIC_STATE = "RF/remote/state";  // full JSON of last signal (attributes)
const char* TOPIC_LAST  = "RF/remote/last";   // compact summary (sensor state)
const char* TOPIC_TX         = "RF/remote/+/tx";       // replay commands (wildcard subscribe)
const char* TOPIC_INVERT_SET = "RF/remote/invert/set"; // invert OOK command
const char* TOPIC_INVERT     = "RF/remote/invert";     // invert OOK state

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

// Static discovery: one sensor showing the last received code.
const char DISCOVERY_LAST[] =
  R"({"name":"RF Remote Last Code","state_topic":"RF/remote/last","json_attributes_topic":"RF/remote/state","unique_id":"rf_remote_last","device":{"identifiers":["esp32-rf433"],"name":"ESP32 RF433 Remote Bridge","manufacturer":"ESP32","model":"CC1101 433MHz"}})";

const char DISCOVERY_INVERT[] =
  R"({"name":"RF Invert OOK","state_topic":"RF/remote/invert","command_topic":"RF/remote/invert/set","payload_on":"ON","payload_off":"OFF","state_on":"ON","state_off":"OFF","unique_id":"rf_remote_invert","device":{"identifiers":["esp32-rf433"],"name":"ESP32 RF433 Remote Bridge","manufacturer":"ESP32","model":"CC1101 433MHz"}})";

// Learned remote codes -> dynamically published binary_sensor + button discovery.
#define MAX_CODES  32
#define MAX_PULSES 256   // pulse/gap pairs retained for replay (full frame)
#define KEY_MAX    48

#ifndef RF_INVERT_OOK
#  define RF_INVERT_OOK 0   // default OOK polarity (0=normal, 1=inverted)
#endif

#define RAW_CAP       (MAX_PULSES * 12 + 128)  // room for a full "p,g," list
#define RAW_BUCKET    100                      // us; coarse bucket for a stable fingerprint
#define EDGE_MIN      100                      // us; edge pulses shorter than this are noise
#define MIN_RAW_PULSES 4                       // ignore undecoded signals with fewer pulses
#define PERSIST_MAGIC 0x52463433               // "RF43"
#define PERSIST_FILE  "/codes.bin"

struct CodeEntry {
  char key[KEY_MAX];
  char name[64];
  uint16_t pulses[MAX_PULSES * 2];  // interleaved mark,space,... timings (us)
  uint16_t count;                   // number of pulse/gap pairs
};
CodeEntry codes[MAX_CODES];
int codeCount = 0;

struct PendingOff {
  char key[KEY_MAX];
  unsigned long offAt;
};
PendingOff pending[MAX_CODES];
int pendingCount = 0;

bool invertOok = RF_INVERT_OOK;
static bool stateDirty = false;

struct PersistedCode {
  char key[KEY_MAX];
  char name[64];
  uint16_t count;
  uint16_t pulses[MAX_PULSES * 2];
};
static PersistedCode persistBuf;

static char rawTrain[RAW_CAP];
static char rawJson[RAW_CAP + 256];

// djb2 hash (stable across reboots)
uint32_t hashString(const char* s) {
  uint32_t h = 5381;
  while (*s) h = ((h << 5) + h) + (unsigned char)*s++;
  return h;
}

void delayUs(uint32_t us) {
  while (us > 16000) { delayMicroseconds(16000); us -= 16000; }
  if (us) delayMicroseconds(us);
}

bool rawFingerprint(const int* pulse_us, const int* gap_us, unsigned int num_pulses, uint32_t& hashOut) {
  if (num_pulses < MIN_RAW_PULSES) return false;

  // Trim trailing/leading noise edges (short pulses).
  unsigned int n = num_pulses;
  while (n > 0 && pulse_us[n - 1] < EDGE_MIN) n--;
  unsigned int start = 0;
  while (start + 1 < n && pulse_us[start] < EDGE_MIN) start++;

  if (n - start < MIN_RAW_PULSES) return false;

  // Hash the quantized sequence; include the length so different lengths differ.
  uint32_t h = 5381;
  h = ((h << 5) + h) + (unsigned char)(n - start);
  char tmp[16];
  for (unsigned int i = start; i < n; i++) {
    int p = (pulse_us[i] + RAW_BUCKET / 2) / RAW_BUCKET;
    int g = (gap_us[i] + RAW_BUCKET / 2) / RAW_BUCKET;
    int l = snprintf(tmp, sizeof(tmp), "%d,%d,", p, g);
    for (int j = 0; j < l; j++) h = ((h << 5) + h) + (unsigned char)tmp[j];
  }
  hashOut = h;
  return true;
}

void saveState() {
  File f = LittleFS.open(PERSIST_FILE, "w");
  if (!f) { Serial.println("FS: open for write failed"); return; }
  uint32_t magic = PERSIST_MAGIC;
  f.write((uint8_t*)&magic, sizeof(magic));
  f.write((uint8_t*)&codeCount, sizeof(codeCount));
  f.write((uint8_t*)&invertOok, sizeof(invertOok));
  for (int i = 0; i < codeCount; i++) {
    memset(&persistBuf, 0, sizeof(persistBuf));
    strncpy(persistBuf.key, codes[i].key, KEY_MAX - 1);
    strncpy(persistBuf.name, codes[i].name, sizeof(persistBuf.name) - 1);
    persistBuf.count = codes[i].count;
    memcpy(persistBuf.pulses, codes[i].pulses, sizeof(codes[i].pulses));
    f.write((uint8_t*)&persistBuf, sizeof(persistBuf));
  }
  f.close();
  Serial.printf("FS: saved %d codes (invert=%d)\n", codeCount, invertOok);
}

void loadState() {
  if (!LittleFS.exists(PERSIST_FILE)) return;
  File f = LittleFS.open(PERSIST_FILE, "r");
  if (!f) return;
  uint32_t magic = 0;
  int storedCount = 0;
  if (f.read((uint8_t*)&magic, sizeof(magic)) != sizeof(magic) || magic != PERSIST_MAGIC) { f.close(); return; }
  if (f.read((uint8_t*)&storedCount, sizeof(storedCount)) != sizeof(storedCount)) { f.close(); return; }
  f.read((uint8_t*)&invertOok, sizeof(invertOok));
  codeCount = storedCount > MAX_CODES ? MAX_CODES : storedCount;
  for (int i = 0; i < codeCount; i++) {
    if (f.read((uint8_t*)&persistBuf, sizeof(persistBuf)) != sizeof(persistBuf)) break;
    strncpy(codes[i].key, persistBuf.key, KEY_MAX - 1); codes[i].key[KEY_MAX - 1] = '\0';
    strncpy(codes[i].name, persistBuf.name, sizeof(codes[i].name) - 1); codes[i].name[sizeof(codes[i].name) - 1] = '\0';
    codes[i].count = persistBuf.count;
    memcpy(codes[i].pulses, persistBuf.pulses, sizeof(codes[i].pulses));
  }
  f.close();
  Serial.printf("FS: loaded %d codes (invert=%d)\n", codeCount, invertOok);
}

// ================= rtl_433_ESP =================
#ifndef RF_MODULE_FREQUENCY
#  define RF_MODULE_FREQUENCY 433.92
#endif

#define JSON_MSG_BUFFER 1024
char messageBuffer[JSON_MSG_BUFFER];
rtl_433_ESP rf;

// Decoded callback always fires before the raw callback for the same signal.
static bool signalDecoded = false;
static char decodedKey[KEY_MAX];
static char decodedName[64];
static char decodedSummary[96];
static char decodedJson[JSON_MSG_BUFFER];

void connectWiFi() {
  Serial.printf("WiFi: connecting to \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi: connected, IP ");
  Serial.println(WiFi.localIP());
}

// Human-readable PubSubClient mqtt.state() return codes
const char* mqttStateText(int rc) {
  switch (rc) {
    case -4: return "connection timeout";
    case -3: return "connection lost";
    case -2: return "network connect failed";
    case -1: return "disconnected";
    case  1: return "bad protocol version";
    case  2: return "client ID rejected";
    case  3: return "server unavailable";
    case  4: return "bad username/password";
    case  5: return "NOT AUTHORIZED (check user/pass or ACL)";
    default: return "unknown";
  }
}

bool mqttConnect() {
  if (MQTT_USERNAME[0] != '\0') {
    return mqtt.connect(MQTT_CLIENT, MQTT_USERNAME, MQTT_PASSWORD);
  }
  return mqtt.connect(MQTT_CLIENT);
}

void connectMQTT() {
  Serial.printf("MQTT: connecting to %s:%u (user=%s) ...\n", MQTT_BROKER, MQTT_PORT, MQTT_USERNAME[0] != '\0' ? MQTT_USERNAME : "(anonymous)");
  while (!mqtt.connected()) {
    if (mqttConnect()) {
      Serial.println("MQTT: connected");
    } else {
      Serial.printf("MQTT: connect failed rc=%d (%s), retry in 3s\n", mqtt.state(), mqttStateText(mqtt.state()));
      delay(3000);
    }
  }
}

void mqttSubscribe() {
  mqtt.subscribe(TOPIC_TX);
  mqtt.subscribe(TOPIC_INVERT_SET);
  Serial.printf("MQTT: subscribed to %s and %s\n", TOPIC_TX, TOPIC_INVERT_SET);
}

void publishBinarySensorDiscovery(const char* key, const char* name) {
  char topic[128];
  snprintf(topic, sizeof(topic), "%s/binary_sensor/rf_remote_%s/config", HA_PREFIX, key);
  char payload[512];
  snprintf(payload, sizeof(payload),
    R"({"name":"%s","state_topic":"RF/remote/%s","payload_on":"ON","payload_off":"OFF","unique_id":"rf_remote_%s","device":{"identifiers":["esp32-rf433"],"name":"ESP32 RF433 Remote Bridge","manufacturer":"ESP32","model":"CC1101 433MHz"}})",
    name, key, key);
  mqtt.publish(topic, payload, true);
}

void publishButtonDiscovery(const char* key, const char* name) {
  char topic[128];
  snprintf(topic, sizeof(topic), "%s/button/rf_remote_tx_%s/config", HA_PREFIX, key);
  char payload[512];
  snprintf(payload, sizeof(payload),
    R"({"name":"Replay %s","command_topic":"RF/remote/%s/tx","payload_press":"PRESS","unique_id":"rf_remote_tx_%s","device":{"identifiers":["esp32-rf433"],"name":"ESP32 RF433 Remote Bridge","manufacturer":"ESP32","model":"CC1101 433MHz"}})",
    name, key, key);
  mqtt.publish(topic, payload, true);
}

void publishInvertState() {
  if (mqtt.connected()) {
    mqtt.publish(TOPIC_INVERT, invertOok ? "ON" : "OFF", true);
  }
}

// Publish Home Assistant auto-discovery configs (retained)
void publishDiscovery() {
  char topic[128];
  snprintf(topic, sizeof(topic), "%s/sensor/rf_remote_last/config", HA_PREFIX);
  mqtt.publish(topic, DISCOVERY_LAST, true);
  snprintf(topic, sizeof(topic), "%s/switch/rf_remote_invert/config", HA_PREFIX);
  mqtt.publish(topic, DISCOVERY_INVERT, true);
  for (int i = 0; i < codeCount; i++) {
    publishBinarySensorDiscovery(codes[i].key, codes[i].name);
    publishButtonDiscovery(codes[i].key, codes[i].name);
  }
  publishInvertState();
  Serial.println("MQTT: Home Assistant discovery configs published");
}

CodeEntry* findCode(const char* key) {
  for (int i = 0; i < codeCount; i++)
    if (strcmp(codes[i].key, key) == 0) return &codes[i];
  return nullptr;
}

CodeEntry* ensureCode(const char* key, const char* name) {
  CodeEntry* e = findCode(key);
  if (e) return e;
  if (codeCount >= MAX_CODES) {
    Serial.println("WARN: too many codes, ignoring new one");
    return nullptr;
  }
  e = &codes[codeCount++];
  memset(e, 0, sizeof(*e));
  strncpy(e->key, key, KEY_MAX - 1);
  strncpy(e->name, name, sizeof(e->name) - 1);
  publishBinarySensorDiscovery(key, name);
  publishButtonDiscovery(key, name);
  char t[64];
  snprintf(t, sizeof(t), "RF/remote/%s", key);
  if (mqtt.connected()) mqtt.publish(t, "OFF");
  Serial.printf("MQTT: discovered new code %s (%s)\n", key, name);
  stateDirty = true;  // persist the new code
  return e;
}

void storePulses(CodeEntry* e, const int* pulse_us, const int* gap_us, unsigned int num_pulses) {
  unsigned int n = num_pulses > MAX_PULSES ? MAX_PULSES : num_pulses;
  for (unsigned int i = 0; i < n; i++) {
    e->pulses[i * 2]     = (uint16_t)constrain(pulse_us[i], 0, 65535);
    e->pulses[i * 2 + 1] = (uint16_t)constrain(gap_us[i], 0, 65535);
  }
  e->count = (uint16_t)n;
}

void scheduleOff(const char* key, unsigned long delayMs) {
  if (pendingCount >= MAX_CODES) return;
  strncpy(pending[pendingCount].key, key, KEY_MAX - 1);
  pending[pendingCount].key[KEY_MAX - 1] = '\0';
  pending[pendingCount].offAt = millis() + delayMs;
  pendingCount++;
}

void pulseCode(const char* key) {
  char t[64];
  snprintf(t, sizeof(t), "RF/remote/%s", key);
  if (mqtt.connected()) mqtt.publish(t, "ON");
  scheduleOff(key, 500);
}

void processPendingOff() {
  unsigned long now = millis();
  for (int i = 0; i < pendingCount; ) {
    if ((long)(now - pending[i].offAt) >= 0) {
      char t[64];
      snprintf(t, sizeof(t), "RF/remote/%s", pending[i].key);
      if (mqtt.connected()) mqtt.publish(t, "OFF");
      pending[i] = pending[pendingCount - 1];
      pendingCount--;
    } else {
      i++;
    }
  }
}

void publishSignal(const char* key, const char* name, const char* summary, const char* fullJson) {
  (void)name;
  if (!mqtt.connected()) return;
  mqtt.publish(TOPIC_STATE, fullJson, true);
  mqtt.publish(TOPIC_LAST, summary, true);
  pulseCode(key);
}

// Extract a top-level JSON value for `key` into `out` (null-terminated).
// Handles both quoted strings and bare scalars (numbers/bools).
static bool jsonGetValue(const char* json, const char* key, char* out, size_t outLen) {
  char needle[32];
  snprintf(needle, sizeof(needle), "\"%s\":", key);
  const char* p = strstr(json, needle);
  if (!p) return false;
  p += strlen(needle);
  while (*p == ' ' || *p == '\t') p++;
  bool quoted = (*p == '"');
  if (quoted) p++;
  size_t i = 0;
  while (*p && *p != (quoted ? '"' : ',') && *p != '}') {
    if (i + 1 < outLen) out[i++] = *p;
    p++;
  }
  out[i] = '\0';
  return true;
}

void rtl_433_Callback(char* message) {
  signalDecoded = true;

  char model[32] = "?", id[24] = "?", cmd[24] = "", rssi[16] = "";
  jsonGetValue(message, "model", model, sizeof(model));
  jsonGetValue(message, "id", id, sizeof(id));
  jsonGetValue(message, "cmd", cmd, sizeof(cmd));
  jsonGetValue(message, "rssi", rssi, sizeof(rssi));

  // Stable key = hash of the message with the volatile rssi/duration tail removed.
  char seed[JSON_MSG_BUFFER];
  const char* rssiPos = strstr(message, "\"rssi\"");
  size_t n = rssiPos ? (size_t)(rssiPos - message) : strlen(message);
  if (n >= sizeof(seed)) n = sizeof(seed) - 1;
  memcpy(seed, message, n);
  seed[n] = '\0';
  while (n > 0 && (seed[n - 1] == ',' || seed[n - 1] == ' ')) seed[--n] = '\0';

  snprintf(decodedKey, sizeof(decodedKey), "d%08lx", (unsigned long)hashString(seed));

  if (id[0] && strcmp(id, "?") != 0) {
    if (cmd[0]) snprintf(decodedName, sizeof(decodedName), "%s %s Cmd %s", model, id, cmd);
    else        snprintf(decodedName, sizeof(decodedName), "%s %s", model, id);
  } else {
    snprintf(decodedName, sizeof(decodedName), "%s", model);
  }

  snprintf(decodedSummary, sizeof(decodedSummary), "%s id=%s cmd=%s", model, id, cmd);
  strncpy(decodedJson, message, sizeof(decodedJson) - 1);
  decodedJson[sizeof(decodedJson) - 1] = '\0';

  Serial.printf("DECODED %s | id:%s cmd:%s | rssi:%s\n", model, id, cmd, rssi);
}

void rtl_433_RawCallback(const int* pulse_us, const int* gap_us,
                         unsigned int num_pulses, unsigned long duration_us,
                         int rssi) {
  char key[KEY_MAX], name[64], summary[96];

  if (signalDecoded) {
    signalDecoded = false;
    strncpy(key, decodedKey, KEY_MAX - 1); key[KEY_MAX - 1] = '\0';
    strncpy(name, decodedName, sizeof(name) - 1); name[sizeof(name) - 1] = '\0';
    strncpy(summary, decodedSummary, sizeof(summary) - 1); summary[sizeof(summary) - 1] = '\0';
    snprintf(rawJson, sizeof(rawJson), "%s", decodedJson);
  } else {
    signalDecoded = false;

    uint32_t h = 0;
    if (!rawFingerprint(pulse_us, gap_us, num_pulses, h)) {
      Serial.printf("RAW ignored (too few pulses) | n:%u rssi:%d\n", num_pulses, rssi);
      return;   // noise, not a real signal
    }
    snprintf(key, sizeof(key), "r%08lx", (unsigned long)h);
    snprintf(name, sizeof(name), "RF Raw %s", key);

    // Full pulse train (rounded to 100us buckets) for analysis.
    size_t tpos = 0;
    unsigned int n = num_pulses > MAX_PULSES ? MAX_PULSES : num_pulses;
    for (unsigned int i = 0; i < n && tpos + 24 < sizeof(rawTrain); i++)
      tpos += snprintf(rawTrain + tpos, sizeof(rawTrain) - tpos, "%d,%d,",
                       (pulse_us[i] + RAW_BUCKET / 2) / RAW_BUCKET,
                       (gap_us[i] + RAW_BUCKET / 2) / RAW_BUCKET);
    if (tpos) rawTrain[tpos - 1] = '\0';

    snprintf(rawJson, sizeof(rawJson),
      "{\"model\":\"raw\",\"key\":\"%s\",\"rssi\":%d,\"duration_us\":%lu,\"pulses\":%u,\"train_100us\":\"%s\"}",
      key, rssi, (unsigned long)duration_us, num_pulses, rawTrain);

    snprintf(summary, sizeof(summary), "raw %s", key);

    Serial.printf("RAW | %s | rssi:%d | dur:%luus | pulses:%u | train:%s\n",
                  key, rssi, (unsigned long)duration_us, num_pulses, rawTrain);
  }

  // Retain the raw pulse train so this code can be replayed later.
  CodeEntry* e = ensureCode(key, name);
  if (e) storePulses(e, pulse_us, gap_us, num_pulses);

  publishSignal(key, name, summary, rawJson);
}

void transmitRaw(const uint16_t* pulses, uint16_t count) {
  if (!pulses || count == 0) return;

  rf.disableReceiver();

  int16_t st = radio.setOOK(true);
  if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(10);
  if (st == RADIOLIB_ERR_NONE) st = radio.transmitDirectAsync();  // GDO0 -> async data in, CMD_TX

  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("TX: radio setup failed rc=%d\n", st);
    radio.receiveDirectAsync();
    rf.enableReceiver();
    return;
  }

  pinMode(RF_MODULE_GDO0, OUTPUT);
  digitalWrite(RF_MODULE_GDO0, LOW);
  delayMicroseconds(200);

  for (uint16_t i = 0; i < count; i++) {
    uint16_t p = pulses[i * 2];
    uint16_t g = pulses[i * 2 + 1];
    uint16_t markUs  = invertOok ? g : p;   // swap mark/space when inverted
    uint16_t spaceUs = invertOok ? p : g;
    if (markUs > 0) {
      digitalWrite(RF_MODULE_GDO0, HIGH);
      delayUs(markUs);
    }
    digitalWrite(RF_MODULE_GDO0, LOW);
    if (spaceUs > 0) delayUs(spaceUs);
  }
  digitalWrite(RF_MODULE_GDO0, LOW);

  pinMode(RF_MODULE_GDO0, INPUT);
  radio.receiveDirectAsync();
  rf.enableReceiver();
  Serial.printf("TX: replayed %u pulses, back to RX\n", count);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, TOPIC_INVERT_SET) == 0) {
    bool on = (length == 2 && payload[0] == 'O' && payload[1] == 'N') ||
              (length == 1 && payload[0] == '1');
    if (on != invertOok) {
      invertOok = on;
      stateDirty = true;
    }
    publishInvertState();
    Serial.printf("INVERT: %s\n", invertOok ? "ON" : "OFF");
    return;
  }

  const char* prefix = "RF/remote/";
  const char* suffix = "/tx";
  size_t plen = strlen(prefix);
  if (strncmp(topic, prefix, plen) != 0) return;

  const char* p = topic + plen;
  const char* end = strstr(p, suffix);
  if (!end) return;

  char key[KEY_MAX];
  size_t klen = (size_t)(end - p);
  if (klen >= KEY_MAX) klen = KEY_MAX - 1;
  memcpy(key, p, klen);
  key[klen] = '\0';

  CodeEntry* e = findCode(key);
  if (e && e->count > 0) {
    Serial.printf("TX: replaying %s (%s)\n", key, e->name);
    transmitRaw(e->pulses, e->count);
  } else {
    Serial.printf("TX: unknown/empty code %s\n", key);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing rtl_433_ESP (CC1101 @ 433.92 MHz, OOK) -> MQTT/HA");

  if (!LittleFS.begin(true)) {
    Serial.println("FS: LittleFS mount failed");
  }
  loadState();

  connectWiFi();
  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  connectMQTT();
  mqttSubscribe();
  publishDiscovery();

  rf.initReceiver(RF_MODULE_RECEIVER_GPIO, RF_MODULE_FREQUENCY);
  rf.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER);
  rf.setRawPulsesCallback(rtl_433_RawCallback);
  rf.enableReceiver();

  Serial.printf("Listening -> publishing to %s and %s\n", TOPIC_LAST, TOPIC_STATE);
}

void loop() {
  if (!mqtt.connected()) {
    static unsigned long lastAttempt = 0;
    if (millis() - lastAttempt > 5000) {
      lastAttempt = millis();
      Serial.println("MQTT: disconnected, reconnecting...");
      if (mqttConnect()) {
        publishDiscovery();  // re-publish discovery after reconnect
        mqttSubscribe();
      }
    }
  }
  mqtt.loop();
  rf.loop();
  processPendingOff();
  if (stateDirty) {
    stateDirty = false;
    saveState();
  }
}
