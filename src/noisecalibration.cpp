/*
 *
 *  Decibel Calibration Station
 *  for NoisePollution App
 *  Copyright (C) 2022 Alex Agrafiotis
 *
 */
#include "Arduino.h"

#define sDEBUG
#include <sDebug.h>

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// lib for buttons
#include "InputDebounce.h"

// lib for mic
#include <driver/i2s.h>

//  INMP441 I2S microphone Connections to
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 1 for mic
#define I2S_PORT I2S_NUM_1

// Define input buffer length
#define bufferLen 1000

#define MAX_DB 120

#define BUTTON1 5
#define BUTTON2 18
#define BUTTON3 19
#define BUTTON_DEBOUNCE_DELAY 20 // [ms]

#define SIREN 4

BluetoothSerial SerialBT;

// RTOS TASKS
TaskHandle_t t_readMic;
TaskHandle_t t_makeNoise;
TaskHandle_t t_displayDecibels;
TaskHandle_t t_sendRecieveBtData;

// init buttons
static InputDebounce button1, button2, button3;

// mic samples buffer
int16_t sBuffer[bufferLen];

// global vars
bool state1, prev_state1;
bool state2, prev_state2;
bool state3, prev_state3;

bool button1_pressed = false;
bool button2_pressed = false;
bool button3_pressed = false;

bool record = false;

float avg_DB = 0;
float mean_DB = 0;
int samples_DB = 0;

// functions
void i2s_install()
{
    // Set up I2S Processor configuration
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), // I2S_COMM_FORMAT_I2S
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = bufferLen,
        .use_apll = false};

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void test()
{
    //! add test func
}

void i2s_setpin()
{
    // Set I2S pin configuration
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD};

    i2s_set_pin(I2S_PORT, &pin_config);
}

void checkButtons()
{

    // check button 1
    state1 = button1.process(millis());

    if (state1 && !prev_state1)
    {
        button1_pressed = true;
    }

    prev_state1 = state1;

    // check button 2
    state2 = button2.process(millis());

    if (state2 && !prev_state2)
    {
        button2_pressed = true;
    }

    prev_state2 = state2;

    // check button 3
    state3 = button3.process(millis());

    if (state3 && !prev_state3)
    {
        button3_pressed = true;
    }

    prev_state3 = state3;
}

float readLiveDB()
{

    float _decibels = 0;
    // False print statements to "lock range" on serial plotter display
    int rangelimit = 120;
    Serial.print(0);
    Serial.print(" ");
    Serial.print(rangelimit);
    Serial.print(" ");

    // Get I2S data and place in data buffer
    size_t bytesIn = 0;
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

    if (result == ESP_OK)
    {
        // Read I2S data buffer
        // int16_t samples_read = bytesIn / 8;
        int16_t samples_read = bufferLen;

        if (samples_read > 0)
        {
            float _mean = 0;
            int _counter_non_zero_values = 0;
            for (int16_t i = 0; i < samples_read; ++i)
            {
                if (sBuffer[i] != 0)
                {
                    _counter_non_zero_values++;
                    _mean += (abs(sBuffer[i])); //? abs()
                }
            }

            // Average the data reading
            _mean /= _counter_non_zero_values;

            //! decibel formula
            //? 20*Math.log10(_mean/32768) + 110; (in java)
            _decibels = 20 * log10(_mean / 32768) + MAX_DB;

            return _decibels;
        }
    }
}

float calculateAvgDB()
{
    float _avg_DB = 0;

    _avg_DB = mean_DB / samples_DB;

    mean_DB = 0;
    samples_DB = 0;

    return _avg_DB;
}

// TASKS
void readMic(void *parameter)
{
    float _decibels = 0;
    float _minDb = 10000;
    float _maxDb = -10000;

    for (;;)
    {

        _decibels = readLiveDB();

        if (_decibels < _minDb)
            _minDb = _decibels;

        if (_decibels > _maxDb)
            _maxDb = _decibels;

        PRINT(_decibels);
        PRINT(_maxDb);
        PRINT(_maxDb);

        if (record)
        {
            mean_DB += _decibels;
            samples_DB++;
        }

        // avgDB = _mean / _samples;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void makeNoise(void *parameter)
{

    for (;;)
    {

        digitalWrite(SIREN, !digitalRead(SIREN));
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void sendRecieveBtData(void *parameter)
{
    // BT begin
    SerialBT.begin("ESP32test");
    String bt_command = "";

    for (;;)
    {

        if (SerialBT.available())
        {
            bt_command = SerialBT.readString();
            bt_command.trim();
            PRINT(bt_command);
        }

        if (bt_command == "ON")
        {
            digitalWrite(2, HIGH);
        }
        else if (bt_command == "OFF")
        {
            digitalWrite(2, LOW);
        }
        else if (bt_command == "RECORD")
        {
            record = true;
        }
        else if (bt_command == "STOP")
        {
            bt_command = "";
            record = false;
            SerialBT.println(calculateAvgDB());
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    // setup input buttons (debounced)
    button1.setup(BUTTON1, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
    button2.setup(BUTTON2, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
    button3.setup(BUTTON3, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);

    // OUTPUTS
    pinMode(2, OUTPUT);

    pinMode(SIREN, OUTPUT);
    digitalWrite(SIREN, LOW);

    // Set up I2S
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);

    // init serial
    sDEBUG_BEGIN(115200);

    xTaskCreatePinnedToCore(readMic, "t_readMic", 10000, NULL, 4, &t_readMic, 1);
    xTaskCreatePinnedToCore(makeNoise, "t_makeNoise", 10000, NULL, 1, &t_makeNoise, 0);
    xTaskCreatePinnedToCore(sendRecieveBtData, "t_sendRecieveBtData", 10000, NULL, 4, &t_sendRecieveBtData, 0);
}

void loop()
{

    checkButtons();
}