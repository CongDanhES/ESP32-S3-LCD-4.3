#include "SD/waveshare_sd_card.hpp"
#include <ESP_IOExpander_Library.h>
#include "defined.hpp"

ESP_IOExpander_CH422G *expander = NULL;

// Initial Setup
void setup(){
    Serial.begin(115200);

    // Serial.println("Initialize IO expander");
    displayDevice.init(); // Initialize display device

    /* Initialize IO expander */
    // expander = new ESP_IOExpander_CH422G(0 ,EXAMPLE_I2C_ADDR, EXAMPLE_I2C_SCL_PIN, EXAMPLE_I2C_SDA_PIN);
    // expander->init();
    // expander->begin();

    // Serial.println("Set the IO0-7 pin to output mode.");
    
    // expander->digitalWrite(TP_RST , HIGH);
    // expander->digitalWrite(LCD_RST , HIGH);
    // expander->digitalWrite(LCD_BL , HIGH);

    // // Use extended GPIO for SD card
    // expander->digitalWrite(SD_CS, LOW);

    // // Turn off backlight
    // expander->digitalWrite(LCD_BL, LOW);

    // // When USB_SEL is HIGH, it enables FSUSB42UMX chip and gpio19, gpio20 wired CAN_TX CAN_RX, and then don't use USB Function 
    // expander->digitalWrite(USB_SEL, LOW);

    // Initialize SPI
    SPI.setHwCs(false);
    SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_SS);
    if (!SD.begin(SD_SS)) {
        Serial.println("Card Mount Failed"); // SD card mounting failed
        return;
    }
    uint8_t cardType = SD.cardType();

    if (cardType == CARD_NONE) {
        Serial.println("No SD card attached"); // No SD card connected
        return;
    }

    Serial.print("SD Card Type: "); // SD card type
    if (cardType == CARD_MMC) {
        Serial.println("MMC");
    } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
    } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN"); // Unknown Type
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize); // SD card size
}

// Main Loop
void loop() {
  delay(5000);
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
      Serial.println("No SD card attached"); // No SD card connected
      return;
  }

  Serial.print("SD Card Type: "); // SD card type
  if (cardType == CARD_MMC) {
      Serial.println("MMC");
  } else if (cardType == CARD_SD) {
      Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN"); // Unknown Type
  }
}
