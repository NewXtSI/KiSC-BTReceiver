#include <Arduino.h>
#include "FS.h"
#include <LittleFS.h>
#include <WiFi.h>
#define ESP32DEBUGGING
#include <ESP32Logger.h>

#include <Arduino.h>

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include "../KiSC-ESP-Now-Protocol/include/kiscprotov2.h"

I2SStream out;
BluetoothA2DPSink a2dp_sink(out);
A2DPSimpleExponentialVolumeControl vc;

KiSCProtoV2Slave kisc("BTAudio");

// I2S Output
#define I2S_OUT_BCK_PIN     27      // Weiß
#define I2S_OUT_WS_PIN      26      // Gelb
#define I2S_OUT_DATA_PIN    25      // Rot

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
  if (id == ESP_AVRC_MD_ATTR_PLAYING_TIME) {
    uint32_t playtime = String((char*)text).toInt();
    Serial.printf("==> Playing time is %d ms (%d seconds)\n", playtime, (int)round(playtime/1000.0));
  }
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr){
  Serial.println(a2dp_sink.to_str(state));
}

void audio_state_changed(esp_a2d_audio_state_t state, void *ptr){
  Serial.println(a2dp_sink.to_str(state));
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    AudioLogger::instance().begin(Serial, AudioLogger::Info);
//    WRITE_PERI_REG(CONFIG_BROWNOUT_DET, 0);  // disable brownout detector
    delay(100);
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    DBGINI(&Serial)
    DBGINI(&Serial, ESP32Timestamp::TimestampSinceStart)
    DBGLEV(Verbose)
    DBGSTA
    DBGLOG(Info,    "---------------------------------------------------------------"
                    "---------")
    DBGLOG(Info,    "Enabled debug levels:")
    DBGLOG(Error,   "Error")
    DBGLOG(Warning, "Warning")
    DBGLOG(Info,    "Info")
    DBGLOG(Verbose, "Verbose")
    DBGLOG(Debug,   "Debug")
    DBGLOG(Info,    "---------------------------------------------------------------"
                    "---------")

    DBGLOG(Info, "Total heap:      %6.1f kByte", ESP.getHeapSize() / 1024.0);
    DBGLOG(Info, "Free heap:       %6.1f kByte", ESP.getFreeHeap() / 1024.0);
    DBGLOG(Info, "CPU frequency:   %6d MHz", ESP.getCpuFreqMHz());
    bool psRamInited = psramInit();
    DBGLOG(Verbose, "PSRAM Found (Init: %s) : %s", psRamInited ? "true" : "false", psramFound() ? "true" : "false");
    if (psramFound()) {
        DBGLOG(Info, "Total PSRAM:     %6.1f kByte", ESP.getPsramSize() / 1024.0);
        DBGLOG(Info, "Free PSRAM:      %6.1f kByte", ESP.getFreePsram() / 1024.0);
    }
    auto i2sconfig = out.defaultConfig(TX_MODE);
    i2sconfig.port_no = 0;
    i2sconfig.is_master = true;
    i2sconfig.pin_bck = I2S_OUT_BCK_PIN;
    i2sconfig.pin_ws = I2S_OUT_WS_PIN;
    i2sconfig.pin_data = I2S_OUT_DATA_PIN;
    out.begin(i2sconfig);
    a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_PLAYING_TIME );
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);

    a2dp_sink.set_on_connection_state_changed(connection_state_changed);
    a2dp_sink.set_on_audio_state_changed(audio_state_changed);
    a2dp_sink.set_auto_reconnect(true);
//    a2dp_sink.set_volume_control(&vc);
    a2dp_sink.set_mono_downmix(true);
//    a2dp_sink.set_volume(100);
    a2dp_sink.start("MyMusic");

    kisc.onNetwork([]() {
        kisc.dumpNetwork();
    });
    kisc.setType(KiSCPeer::SlaveType::BTAudio);
    kisc.start();
}

void loop() {
  // put your main code here, to run repeatedly:
    delay(2000);
    a2dp_sink.set_volume(a2dp_sink.get_volume()+10);
    DBGLOG(Info, "Volume: %d", a2dp_sink.get_volume());
}