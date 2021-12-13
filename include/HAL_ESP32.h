/*
THIS IS THE HARDWARE ABSTRACT LAYER
FOR DIYBMS ESP32 CONTROLLER PCB - THIS IS THE LARGER
PCB WITH RS485/CANBUS/TFT DISPLAY
*/

#include <Arduino.h>
#include "driver/i2c.h"
#include <driver/can.h>
#include "esp32-hal-i2c.h" 
#include <SPI.h>

//#define GREEN_LED 2

// 0 is the BOOT button
#define RESET_WIFI_PIN 0
#define PFC_INTERRUPT_PIN 33

#ifndef HAL_ESP32_H_
#define HAL_ESP32_H_

// GPIO34 (input only pin)
#define TCA9534A_INTERRUPT_PIN GPIO_NUM_34
#define TCA9534APWR_ADDRESS 0x38      // does not exist not on our board
#define TCA9534APWR_INPUT 0x00
#define TCA9534APWR_OUTPUT 0x01
#define TCA9534APWR_POLARITY_INVERSION 0x02
#define TCA9534APWR_CONFIGURATION 0x03
#define TCA9534APWR_INPUTMASK B10010000

// PCF8574A
// i2c address 0x20 is the PCF8574A chip on the left center of my board, connected to all output relays and output MOSFETS
#define PCF8574A_INTERRUPT_PIN GPIO_NUM_34
#define PCF8574A_ADDRESS 0x20
#define PCF8574A_INPUTMASK B10000000    // input mask, pin 7 is input to interrupt from PCF8574B all other pins are outputs

// PCF8574B
// i2c address 0x25 is the PCF8574B chip on the lower edge of my board, intended for input and to controll a bistable relay
#define PCF8574B_INTERRUPT_PIN GPIO_NUM_34
#define PCF8574B_ADDRESS 0x3D           //board 002 = 0x25
#define PCF8574B_INPUTMASK B00111111    // pin6 + 7 control bistable relay the rest can be used as input

// GPIO39 (input only pin)
#define TCA6408_INTERRUPT_PIN GPIO_NUM_34
#define TCA6408_ADDRESS 0x27    // does not exist not on our board
#define TCA6408_INPUT 0x00
#define TCA6408_OUTPUT 0x01
#define TCA6408_POLARITY_INVERSION 0x02
#define TCA6408_CONFIGURATION 0x03
// All pins on TCA6408 are outputs - prevents interrupts triggering randomly due to static/ESD
#define TCA6408_INPUTMASK B00000000

#define VSPI_SCK GPIO_NUM_18

#define TOUCH_IRQ GPIO_NUM_36
#define TOUCH_CHIPSELECT GPIO_NUM_4
#define SDCARD_CHIPSELECT GPIO_NUM_5

#define RS485_RX GPIO_NUM_21
#define RS485_TX GPIO_NUM_22
#define RS485_ENABLE GPIO_NUM_25

// RTC DS3231
// I2C Slave Address
#define DS3231_ADDRESS 0x68

// DS3231 Register Addresses
#define DS3231_REG_TIMEDATE 0x00
#define DS3231_REG_ALARMONE 0x07
#define DS3231_REG_ALARMTWO 0x0B

#define DS3231_REG_CONTROL 0x0E
#define DS3231_REG_STATUS 0x0F
#define DS3231_REG_AGING 0x10

#define DS3231_REG_TEMP 0x11

// DS3231 Register Data Size if not just 1
#define DS3231_REG_TIMEDATE_SIZE 7
#define DS3231_REG_ALARMONE_SIZE 4
#define DS3231_REG_ALARMTWO_SIZE 3

#define DS3231_REG_TEMP_SIZE 2

// DS3231 Control Register Bits
#define DS3231_A1IE 0
#define DS3231_A2IE 1
#define DS3231_INTCN 2
#define DS3231_RS1 3
#define DS3231_RS2 4
#define DS3231_CONV 5
#define DS3231_BBSQW 6
#define DS3231_EOSC 7
#define DS3231_AIEMASK (_BV(DS3231_A1IE) | _BV(DS3231_A2IE))
#define DS3231_RSMASK (_BV(DS3231_RS1) | _BV(DS3231_RS2))

// DS3231 Status Register Bits
#define DS3231_A1F 0
#define DS3231_A2F 1
#define DS3231_BSY 2
#define DS3231_EN32KHZ 3
#define DS3231_OSF 7
#define DS3231_AIFMASK (_BV(DS3231_A1F) | _BV(DS3231_A2F))

#define CONFIG_I2CDEV_TIMEOUT 1000 // I2C port timeout
#define DS3231_12HOUR_FLAG 0x40
#define DS3231_12HOUR_MASK 0x1F
#define DS3231_PM_FLAG 0x20
#define DS3231_24HOUR_MASK 0x3F

struct TouchScreenValues
{
    bool touched;
    uint8_t pressure;
    uint16_t X;
    uint16_t Y;
};

// Derived classes
class HAL_ESP32
{
public:
    SPIClass vspi;
    HAL_ESP32()
    {
        vspi = SPIClass(VSPI);
        xVSPIMutex = xSemaphoreCreateMutex();
        xDisplayMutex = xSemaphoreCreateMutex();
        xi2cMutex = xSemaphoreCreateMutex();
        RS485Mutex = xSemaphoreCreateMutex();
    }

    void ConfigureI2C(void (*PCF8574AInterrupt)(void), void (*PCF8574BInterrupt)(void));
    void SetOutputState(uint8_t outputId, RelayState state);
    uint8_t ReadPCF8574AInputRegisters();
    uint8_t ReadPCF8574BInputRegisters();
    
    void readDS3231_RTC(struct tm *localRTC);
    void writeDS3231_RTC(struct tm *localRTC);
    void parseTimeDateToTM(Diybms_eeprom_settings *_mysettings,  struct tm *localRTC);
    void parseTMtoTimeDate(Diybms_eeprom_settings *_mysettings, struct tm *localRTC);
    void setTimeDateMan(Diybms_eeprom_settings *_mysettings,  struct tm *localRTC);

    void Led(uint8_t bits);
    void ConfigurePins(void (*WiFiPasswordResetInterrput)(void));
    void TFTScreenBacklight(bool Status);
    void SwapGPIO0ToOutput();
    void CANBUSEnable(bool value);
    void ConfigureCAN();

    bool IsVSPIMutexAvailable()
    {
        if (xVSPIMutex == NULL)
            return false;

        return (uxSemaphoreGetCount(xVSPIMutex) == 1);
    }

    bool GetDisplayMutex()
    {
        if (xDisplayMutex == NULL)
            return false;

        // Wait 50ms max
        bool reply = (xSemaphoreTake(xDisplayMutex, (TickType_t)50 / portTICK_PERIOD_MS) == pdTRUE);
        if (!reply)
        {
            ESP_LOGE(TAG, "Unable to get Display mutex");
        }
        return reply;
    }
    bool ReleaseDisplayMutex()
    {
        if (xDisplayMutex == NULL)
            return false;

        return (xSemaphoreGive(xDisplayMutex) == pdTRUE);
    }

    bool GetVSPIMutex()
    {
        if (xVSPIMutex == NULL)
            return false;

        // Wait 50ms max
        bool reply = (xSemaphoreTake(xVSPIMutex, (TickType_t)50 / portTICK_PERIOD_MS) == pdTRUE);
        if (!reply)
        {
            ESP_LOGE(TAG, "Unable to get VSPI mutex");
        }
        return reply;
    }
    bool ReleaseVSPIMutex()
    {
        if (xVSPIMutex == NULL)
            return false;

        return (xSemaphoreGive(xVSPIMutex) == pdTRUE);
    }

    bool Geti2cMutex()
    {
        if (xi2cMutex == NULL)
            return false;

        // Wait 100ms max
        bool reply = (xSemaphoreTake(xi2cMutex, (TickType_t)100 / portTICK_PERIOD_MS) == pdTRUE);
        if (!reply)
        {
            ESP_LOGE(TAG, "Unable to get I2C mutex");
        }
        return reply;
    }
    bool Releasei2cMutex()
    {
        if (xi2cMutex == NULL)
            return false;

        return (xSemaphoreGive(xi2cMutex) == pdTRUE);
    }

    bool GetRS485Mutex()
    {
        if (RS485Mutex == NULL)
            return false;

        // Wait 100ms max
        bool reply = (xSemaphoreTake(RS485Mutex, (TickType_t)100 / portTICK_PERIOD_MS) == pdTRUE);
        if (!reply)
        {
            ESP_LOGE(TAG, "Unable to get RS485 mutex");
        }
        return reply;
    }
    bool ReleaseRS485Mutex()
    {
        if (RS485Mutex == NULL)
            return false;

        return (xSemaphoreGive(RS485Mutex) == pdTRUE);
    }

    // Infinite loop flashing the LED RED/WHITE
    void Halt(RGBLED colour)
    {
        ESP_LOGE(TAG, "SYSTEM HALTED");

        while (true)
        {
            Led(RGBLED::Red);
            delay(700);
            Led(colour);
            delay(300);
        }
    }

    TouchScreenValues TouchScreenUpdate()
    {
        /*
            Features and Specification of XPT2046
            * 12 bit SAR type A/D converter with S/H circuit
            * Low voltage operations (VCC = 2.2V - 3.6V)
            * Low power consumption (260uA)
            * 125 kHz sampling frequency
            * On-chip reference voltage - 2.5V
            * Pen pressure measurement
            * On-chip thermo sensor
            * Direct battery measurement
            * 4-wire I/F

            DataSheet
            https://components101.com/admin/sites/default/files/component_datasheet/XPT2046-Datasheet.pdf

            // BIT7(MSB) BIT6 BIT5 BIT4 BIT3 BIT2      BIT1 BIT0(LSB)
            // S         A2   A1   A0   MODE SER/——DFR PD1  PD0

            // S = START BIT
            // A2/A1/A0 = Channel address bits 011 = Z1, 100=Z2, 001=X, 101=Y
            // MODE = 12bit/8bit (12bit=low)
            // SER/DFR = SingleEnded/Differential ADC mode (differential mode)
            // PD0 and PD1 control power delivery, 0/0 = all enabled, 1=1=disabled.
        */
        // Z is touch (depth)
        // const uint8_t ADD_TEMPERATURE = 0B00000000;
        // const uint8_t ADD_VBATTERY = 0B00100000;
        const uint8_t ADD_Z1 = 0B00110000;
        const uint8_t ADD_Z2 = 0B01000000;
        const uint8_t ADD_X = 0B00010000;
        const uint8_t ADD_Y = 0B01010000;
        const uint8_t STARTBIT = 0B10000000;
        // internal reference voltage is only used in the single-ended mode for battery monitoring, temperature measurement
        const uint8_t PWR_ALLOFF = 0B00000000;
        const uint8_t PWR_REFOFF_ADCON = 0B00000001;
        // const uint8_t PWR_REFON_ADCOFF = 0B00000010;
        // const uint8_t PWR_REFON_ADCON = 0B00000011;

        // Differential Reference Mode (SER/DFR low) should be used for reading X/Y/Z
        // Single ended mode (SER/DFR high) should be used for temperature and battery
        const uint8_t ADC_DIFFERENTIAL = 0B00000000;
        // const uint8_t ADC_SINGLEENDED = 0B00000100;

        const uint8_t ADC_8BIT = 0B00001000;
        const uint8_t ADC_12BIT = 0B00000000;

        TouchScreenValues reply;
        reply.touched = false;
        reply.pressure = 0;
        reply.X = 0;
        reply.Y = 0;

        // Landscape orientation screen
        // X is zero when not touched, left of screen is about 290, right of screen is about 3900
        // Y is 3130 when not touched, top of screen is 250, bottom is 3150

        if (GetVSPIMutex())
        {
            // Slow down to 2Mhz SPI bus
            vspi.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

            digitalWrite(TOUCH_CHIPSELECT, LOW);

            // We don't need accurate touch pressure for this application, so
            // only read Z2 register, we just want a boolean value at the end of the day

            // The transfers are always a step behind, so the transfer reads the previous value/command
            vspi.write(STARTBIT | PWR_REFOFF_ADCON | ADC_8BIT | ADC_DIFFERENTIAL | ADD_Z1);

            // Read Z2 (1 byte) and request X
            reply.pressure = vspi.transfer(STARTBIT | PWR_REFOFF_ADCON | ADC_12BIT | ADC_DIFFERENTIAL | ADD_X);

            // Read X (2 bytes) and request Y
            reply.X = vspi.transfer16(STARTBIT | PWR_REFOFF_ADCON | ADC_12BIT | ADC_DIFFERENTIAL | ADD_Y) >> 3;
            // Take Y reading, then power down after this sample
            reply.Y = vspi.transfer16(STARTBIT | PWR_ALLOFF | ADC_8BIT | ADC_DIFFERENTIAL | ADD_Z2) >> 3;

            // Final transfer to ensure device goes into sleep
            // In order to turn the reference off, an additional write to the XPT2046 is required after the channel has been converted.  Page 24 datasheet
            // uint16_t Z2 =
            vspi.transfer(0);

            digitalWrite(TOUCH_CHIPSELECT, HIGH);
            vspi.endTransaction();

            ReleaseVSPIMutex();

            // Screen connected and not touched
            // Touch = 128, x=0 , y around 3130-3140

            // 135 controls the minimum amount of pressure needed to "touch"
            // X also needs to be greater than zero
            reply.touched = reply.pressure > 135 && reply.X > 0;

            ESP_LOGD(TAG, "Touch = touch=%i pressure=%u x=%u y=%u", reply.touched, reply.pressure, reply.X, reply.Y);
        }

        return reply;
    }

    bool IsScreenAttached()
    {
        TouchScreenValues v = TouchScreenUpdate();

        return !(v.pressure == 0 && v.X == 0 && v.Y == 0);
    }

    void ConfigureVSPI()
    {
        vspi.endTransaction();
        vspi.end();
        // VSPI
        // GPIO23 (MOSI), GPIO19(MISO), GPIO18(CLK) and GPIO5 (CS)
        vspi.begin(18, 19, 23, -1);
        // Don't use hardware chip selects on VSPI
        vspi.setHwCs(false);
        vspi.setBitOrder(MSBFIRST);
        vspi.setDataMode(SPI_MODE0);
        // 10mhz
        vspi.setFrequency(10000000);
    }

    uint8_t LastPCF8574AValue()
    {
        return PCF8574A_Value;
    }

private:
    SemaphoreHandle_t xVSPIMutex = NULL;
    SemaphoreHandle_t xDisplayMutex = NULL;
    SemaphoreHandle_t xi2cMutex = NULL;
    SemaphoreHandle_t RS485Mutex = NULL;

    // Copy of pin state for PCF8574A
    uint8_t PCF8574A_Value;
    // Copy of pin state for PCF8574B
    uint8_t PCF8574B_Value;

    esp_err_t test4i2cDevice(i2c_port_t i2c_num, uint8_t dev);
    
    esp_err_t writeBytePCF8574(i2c_port_t i2c_num, uint8_t dev, uint8_t data);
    esp_err_t writeByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg, uint8_t data);
    uint8_t readBytePCF8574(i2c_port_t i2c_num, uint8_t dev);
    uint8_t readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg);

    uint8_t BcdToUint8(uint8_t val);
    uint8_t Uint8ToBcd(uint8_t val);
    esp_err_t readRegDS3231(i2c_port_t i2c_num, uint8_t chip_addr, uint8_t Reg_Adr, uint8_t *data_rd, size_t num_bytes);
    esp_err_t setRegDS3231(i2c_port_t i2c_num, uint8_t *Reg_Adr, size_t out_reg_size, uint8_t *data_rd, size_t num_bytes);
    
};

#endif
