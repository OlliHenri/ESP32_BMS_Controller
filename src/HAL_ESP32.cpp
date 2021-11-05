
#include "defines.h"
#include "HAL_ESP32.h"

uint8_t HAL_ESP32::readBytePCF8574(i2c_port_t i2c_num, uint8_t dev)
{
    // We use the native i2c commands for ESP32 as the Arduino library
    // seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {
        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Send start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (B01000000 | ((dev << 1) | I2C_MASTER_READ)), I2C_MASTER_NACK);
        // i2c_master_write_byte(cmd, (B01000000 | ((dev << 1) | I2C_MASTER_READ)), I2C_MASTER_ACK);
        //  Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, &data, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(300)));
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        //ESP_LOGD(TAG, "I2C read PCF8574 reply %i", ret);
        //ESP_LOGD(TAG, "I2C read PCF8574_Value : %i", data);
        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
        return data;
    }
    else
    {
        return 0;
    }
}

uint8_t HAL_ESP32::readByte(i2c_port_t i2c_num, uint8_t dev, uint8_t reg)
{
    // We use the native i2c commands for ESP32 as the Arduino library
    // seems to have issues with corrupting i2c data if used from multiple threads
    if (Geti2cMutex())
    {

        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Select the correct register on the i2c device
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
        i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK);
        // Send repeated start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, I2C_MASTER_NACK);
        // Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // esp_err_t ret =
        ESP_ERROR_CHECK_WITHOUT_ABORT(i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100)));

        // ESP_LOGD(TAG,"I2C reply %i",ret);

        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();
        return data;
    }
    else
    {
        return 0;
    }
}

// i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeBytePCF8574(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t data)
{
    if (Geti2cMutex())
    {
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
        uint8_t buffer[2];
        buffer[0] = data;
        i2c_master_write(cmd, buffer, 1, I2C_MASTER_NACK);

        // i2c_master_write_byte(cmd, i2cregister, true);
        // i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();

        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

// i2c: Writes a single byte to a slave devices register
esp_err_t HAL_ESP32::writeByte(i2c_port_t i2c_num, uint8_t deviceAddress, uint8_t i2cregister, uint8_t data)
{
    if (Geti2cMutex())
    {
        // We use the native i2c commands for ESP32 as the Arduino library
        // seems to have issues with corrupting i2c data if used from multiple threads
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | I2C_MASTER_WRITE, I2C_MASTER_NACK);
        uint8_t buffer[2];
        buffer[0] = i2cregister;
        buffer[1] = data;
        i2c_master_write(cmd, buffer, 2, I2C_MASTER_NACK);

        // i2c_master_write_byte(cmd, i2cregister, true);
        // i2c_master_write_byte(cmd, data, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));
        i2c_cmd_link_delete(cmd);

        Releasei2cMutex();
        return ret;
    }
    else
    {
        return ESP_ERR_INVALID_STATE;
    }
}

uint8_t HAL_ESP32::ReadPCF8574AInputRegisters()
{
    PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
    return PCF8574A_Value & PCF8574A_INPUTMASK;
}

uint8_t HAL_ESP32::ReadPCF8574BInputRegisters()
{
    PCF8574B_Value = readBytePCF8574(i2c_port_t::I2C_NUM_0, PCF8574B_ADDRESS);
    return PCF8574B_Value & PCF8574B_INPUTMASK;
}

void HAL_ESP32::SetOutputState(uint8_t outputId, RelayState state)
{
    //ESP_LOGD(TAG, "ACHTUNG ACHTUNG ACHTUNG ACHTUNG ACHTUNG ACHTUNG");
    //PCF8574A_Value = 222;
    PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
    //ESP_LOGI(TAG, "PCF8574A read value: %i", PCF8574A_Value);

    // Relays connected to PCF8574A
    // P0 = BUZZER (outputId=0)
    // P1 = RELAY1 (outputId=1)
    // P2 = RELAY2 (outputId=2)
    // P3 = EXT_MOSFET (outputId=3)
    // P3 = EXT_MOSFET (outputId=4)
    // P3 = EXT_MOSFET (outputId=5)
    // P3 = EXT_MOSFET (outputId=6)
    // P7 interrupt input from PCF8674B

    if (outputId <= 3)
    {
        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        uint8_t bit = outputId; // + 4;
        PCF8574A_Value = (state == RelayState::RELAY_ON) ? (PCF8574A_Value | (1 << bit)) : (PCF8574A_Value & ~(1 << bit));
        // ESP_ERROR_CHECK_WITHOUT_ABORT(writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_Value));
        //esp_err_t ret = 
                  writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_Value);
        //ESP_LOGD(TAG, "PCF8574A reply fault code : %i", ret);
        // TODO: Check return value
        //PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        //ESP_LOGD(TAG, "PCF8574A reply PCF8574A_Value = %i", PCF8574A_Value);
    }
}

void HAL_ESP32::Led(uint8_t bits)
{
    // Clear LED pins
    PCF8574B_Value = PCF8574B_Value & B11111000; // To Do set bitvalue correct if LEDs
    // Set on
    PCF8574B_Value = PCF8574B_Value | (bits & B00000111);
    // esp_err_t ret =
    // ESP_ERROR_CHECK_WITHOUT_ABORT(writeBytePCF8574(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_Value));

    // ESP_LOGD(TAG,"PCF8574B LED reply %i",ret);
    // TODO: Check return value
}

void HAL_ESP32::ConfigureCAN()
{
    // Initialize configuration structures using macro initializers
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(gpio_num_t::GPIO_NUM_16, gpio_num_t::GPIO_NUM_17, CAN_MODE_NORMAL);
    g_config.mode = CAN_MODE_NORMAL;

    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = {.acceptance_code = 0, .acceptance_mask = 0xFFFFFFFF, .single_filter = true};

    // Filter out all messages except 0x305 and 0x307
    // https://docs.espressif.com/projects/esp-idf/en/v3.3.5/api-reference/peripherals/can.html
    // 01100000101 00000 00000000 00000000 = 0x60A00000  (0x305)
    // 01100000111 00000 00000000 00000000 = 0x60E00000  (0x307)
    // 00000000010 11111 11111111 11111111 = 0x005FFFFF
    //          ^ THIS BIT IS IGNORED USING THE MASK SO 0x305 and 0x307 are permitted
    f_config.acceptance_code = 0x60A00000;
    f_config.acceptance_mask = 0x005FFFFF;

    // Install CAN driver
    if (can_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN driver installed.  Filter=%u Mask=%u", f_config.acceptance_code, f_config.acceptance_mask);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to install CAN driver");
    }

    // Start CAN driver
    if (can_start() == ESP_OK)
    {
        ESP_LOGI(TAG, "CAN driver started");
    }
    else
    {
        ESP_LOGE(TAG, "Failed to start CAN driver");
    }
}

// Control Silent mode control input on TJA1051T/3
// True = enable CANBUS
void HAL_ESP32::CANBUSEnable(bool value)
{
    // Pin P5
    // Low = Normal mode
    // High = Silent
    PCF8574B_Value = PCF8574B_Value & B11011111;

    if (value == false)
    {
        // Set on
        PCF8574B_Value = PCF8574B_Value | B00100000;
    }

    ESP_ERROR_CHECK_WITHOUT_ABORT(writeBytePCF8574(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574A_Value));
}

// Control TFT backlight LED
void HAL_ESP32::TFTScreenBacklight(bool value)
{
    // Clear LED pins
    PCF8574B_Value = PCF8574B_Value & B11110111;

    if (value == true)
    {
        // Set on
        PCF8574B_Value = PCF8574B_Value | B00001000;
    }

    // esp_err_t ret =
    ESP_ERROR_CHECK_WITHOUT_ABORT(writeBytePCF8574(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_Value));
    // TODO: Check return value
    // ESP_LOGD(TAG,"PCF8574B reply %i",ret);
}

void HAL_ESP32::ConfigurePins(void (*WiFiPasswordResetInterrupt)(void))
{
    // GPIO39 is interrupt pin from PCF8574A (doesnt have pull up/down resistors, but extern)
    pinMode(PCF8574A_INTERRUPT_PIN, INPUT);

    // GPIO34 is interrupt pin from PCF8574B (doesnt have pull up/down resistors, but extern)
    pinMode(PCF8574B_INTERRUPT_PIN, INPUT);

    // BOOT Button on ESP32 module is used for resetting wifi details
    pinMode(GPIO_NUM_0, INPUT_PULLUP);
    attachInterrupt(GPIO_NUM_0, WiFiPasswordResetInterrupt, CHANGE);

    // For touch screen
    // GPIO_NUM_36 no internal PULLUP
    pinMode(TOUCH_IRQ, INPUT);
    // attachInterrupt(GPIO_NUM_36, TFTScreenTouch, FALLING);

    // Configure the CHIP SELECT pins as OUTPUT and set HIGH
    pinMode(TOUCH_CHIPSELECT, OUTPUT);
    digitalWrite(TOUCH_CHIPSELECT, HIGH);
    pinMode(SDCARD_CHIPSELECT, OUTPUT);
    digitalWrite(SDCARD_CHIPSELECT, HIGH);

    pinMode(RS485_ENABLE, OUTPUT);
    // Enable receive
    digitalWrite(RS485_ENABLE, LOW);
}

void HAL_ESP32::SwapGPIO0ToOutput()
{
    // BOOT Button on ESP32 module is used for resetting wifi details
    detachInterrupt(GPIO_NUM_0);
    pinMode(GPIO_NUM_0, OUTPUT);
    digitalWrite(GPIO_NUM_0, HIGH);
}

// test if i2c device is present
esp_err_t HAL_ESP32::test4PCF8574(i2c_port_t i2c_num, uint8_t dev)
{
    esp_err_t ret = -1;

    if (Geti2cMutex())
    {
        uint8_t data;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Send start, and read the register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
        // Read single byte and expect NACK in reply
        i2c_master_read_byte(cmd, &data, i2c_ack_type_t::I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        // esp_err_t ret =

        ret = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(100));

        i2c_cmd_link_delete(cmd);
        Releasei2cMutex();
        ESP_LOGI(TAG, "Test I2C for PCF8574A");
    }
    return ret;
}

void HAL_ESP32::ConfigureI2C(void (*PCF8574AInterrupt)(void), void (*PCF8574BInterrupt)(void))
{
    ESP_LOGI(TAG, "Configure I2C");

    // SDA / SCL
    // ESP32 = I2C0-SDA / I2C0-SCL
    // I2C Bus 1: uses GPIO 27 (SDA) and GPIO 26 (SCL);
    // I2C Bus 2: uses GPIO 33 (SDA) and GPIO 32 (SCL);
    //  Ollis I2C bus:  GPIO 21 (SDA) and GPIO 22 (SCL);

    // Initialize
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = gpio_num_t::GPIO_NUM_21; // 27;
    conf.scl_io_num = gpio_num_t::GPIO_NUM_22; // 26;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    // conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // conf.scl_pullup_en = GPIO_PULLUP_ENABLE;

    conf.master.clk_speed = 100000; // A BIT SLOWER FOR OUR DEVICES 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));

    ESP_LOGI(TAG, "Config i2c passed");

    // https://datasheet.lcsc.com/szlcsc/1809041633_Texas-Instruments-TCA9534APWR_C206010.pdf
    // TCA9534APWR Remote 8-Bit I2C and Low-Power I/O Expander With Interrupt Output and Configuration Registers
    // https://lcsc.com/product-detail/Interface-I-O-Expanders_Texas-Instruments-TCA9534APWR_C206010.html
    // A0/A1/A2 are LOW, so i2c address is 0x38

    // PINS
    // P0= BLUE
    // P1= RED
    // P2= GREEN
    // P3= DISPLAY BACKLIGHT LED
    // P4= SPARE on J13
    // P5= Canbus RS
    // P6= SPARE on J13
    // P7= ESTOP (pull to ground to trigger)
    // INTERRUPT PIN = ESP32 IO34

    // BIT  76543210
    // PORT 76543210
    // MASK=10000000

    // search for PCF8574A
    esp_err_t ret = -1;
    if (ESP_OK == test4PCF8574(I2C_NUM_0, PCF8574A_ADDRESS))
    {

        ret = writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, PCF8574A_INPUTMASK);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PCF8574A Error");
            // Halt(RGBLED::Purple);
        }
        else
        {
            ESP_LOGI(TAG, "wrote INPUT MASK at PCF8574A");
            attachInterrupt(PCF8574A_INTERRUPT_PIN, PCF8574BInterrupt, FALLING);
            ESP_LOGI(TAG, "PCF8574A interrupt attached");
        }
        /*         // toggle pins 0 to 2 to test the routine
                    delay(1000);
                    writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10000111 );
                        delay(1000);
                    writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10000000 );
                        delay(1000);
                    writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10000111 );
                        delay(1000);
                    writeBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS, B10001000 );  */

        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        ESP_LOGI(TAG, "PCF8574A read value: %i", PCF8574A_Value);
        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574A_ADDRESS);
        ESP_LOGI(TAG, "PCF8574A read value: %i", PCF8574A_Value);
        ESP_LOGD(TAG, "ESP_OK = 0, I2C reply %i", ret);
    }
    else
    {
        ESP_LOGE(TAG, "PCF8574A Error not found");
    }
    /*
Now for the PCF8574B
*/ // P0=EXT_IO_A
    // P1=EXT_IO_B
    // P2=EXT_IO_C
    // P3=EXT_IO_D
    // P4=RELAY 1
    // P5=RELAY 2
    // P6=RELAY 3 (SSR)
    // P7=EXT_IO_E

    if (ESP_OK == test4PCF8574(I2C_NUM_0, PCF8574B_ADDRESS))
    {
        // if PCF8574B is found set Input mask else abort
        ret = writeBytePCF8574(I2C_NUM_0, PCF8574B_ADDRESS, PCF8574B_INPUTMASK);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "PCF8574B Error");
            // Halt(RGBLED::Purple);
        }
        else
        {
            ESP_LOGI(TAG, "wrote INPUT MASK at PCF8574B");
            attachInterrupt(PCF8574B_INTERRUPT_PIN, PCF8574BInterrupt, FALLING);
            ESP_LOGI(TAG, "PCF8574B interrupt attached");
        }
        PCF8574A_Value = readBytePCF8574(I2C_NUM_0, PCF8574B_ADDRESS);
        ESP_LOGI(TAG, "PCF8574B read value: %i", PCF8574B_Value);
        ESP_LOGD(TAG, "ESP_OK = 0, I2C reply %i", ret);
    }
    else
    {
        ESP_LOGE(TAG, "PCF8574B Error not found");
    }
}
