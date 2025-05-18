#ifndef EEPROM_SAFEMANAGER_H
#define EEPROM_SAFEMANAGER_H

#include <EEPROM.h>
#include <Arduino.h>

#define EEPROM_SIZE 512
#define EEPROM_MAGIC 0xBEEF2061

struct EEPROMData {
  uint32_t magic;
  uint32_t crc;
  int brightness;
  // Ajoute ici d'autres champs à sauvegarder
};

class EEPROM_SafeManager {
public:
  EEPROMData data;

  void begin() {
    EEPROM.begin(EEPROM_SIZE);
    if (!load()) {
      Serial.println("EEPROM corrompue ou non initialisée. Réinitialisation...");
      setDefault();
      save();
    } else {
      Serial.println("EEPROM chargée avec succès.");
    }
  }

  void setDefault() {
    data.magic = EEPROM_MAGIC;
    data.brightness = 128;  // valeur par défaut
    updateCRC();
  }

  void save() {
    updateCRC();
    EEPROM.put(0, data);
    EEPROM.commit();
  }

  bool load() {
    EEPROM.get(0, data);
    if (data.magic != EEPROM_MAGIC) return false;
    uint32_t savedCRC = data.crc;
    data.crc = 0;
    uint32_t computedCRC = calculateCRC((uint8_t*)&data, sizeof(data));
    data.crc = savedCRC;
    return savedCRC == computedCRC;
  }

private:
  void updateCRC() {
    data.crc = 0;
    data.crc = calculateCRC((uint8_t*)&data, sizeof(data));
  }

  uint32_t calculateCRC(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        if (crc & 1)
          crc = (crc >> 1) ^ 0xEDB88320;
        else
          crc >>= 1;
      }
    }
    return crc ^ 0xFFFFFFFF;
  }
};

#endif
