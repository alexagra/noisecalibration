/*
 *
 *  Decibel Calibration Station
 *  for NoisePollution App
 *  Copyright (C) 2022 Alex Agrafiotis
 *   mac:  30:C6:F7:20:A6:C4
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
#define I2S_WS 32
#define I2S_SD 33
#define I2S_SCK 25

// Use I2S Processor 1 for mic
#define I2S_PORT I2S_NUM_1

// Define input buffer length
#define bufferLen 1000

#define MAX_DB 120

#define SIREN_M0 4  // not working on pcb ------> with no siren
#define SIREN_M1 5  // 3v
#define SIREN_M2 18 // 6v
#define SIREN_M3 19 // 12v

BluetoothSerial SerialBT;

// RTOS TASKS
TaskHandle_t t_readMic;
TaskHandle_t t_makeNoise;
TaskHandle_t t_displayDecibels;
TaskHandle_t t_sendRecieveBtData;

// mic samples buffer
int16_t sBuffer[bufferLen];

// global vars
bool state1, prev_state1;
bool state2, prev_state2;
bool state3, prev_state3;

bool siren_m0_state = false;
bool siren_m1_state = false;
bool siren_m2_state = false;
bool siren_m3_state = false;
bool record = false;

float live_decibels = 0;
float avg_DB = 0;
float mean_DB = 0;
int samples_DB = 0;

// functions
void turnOnMode(int m)
{
    (m == 0) ? siren_m0_state = true : siren_m0_state = false;
    (m == 1) ? siren_m1_state = true : siren_m1_state = false;
    (m == 2) ? siren_m2_state = true : siren_m2_state = false;
    (m == 3) ? siren_m3_state = true : siren_m3_state = false;

    record = true;

    PRINT(siren_m0_state);
    PRINT(siren_m1_state);
    PRINT(siren_m2_state);
    PRINT(siren_m3_state);
}
void stopRecord()
{
    siren_m0_state = false;
    siren_m1_state = false;
    siren_m2_state = false;
    siren_m3_state = false;

    record = false;
}

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

float readLiveDB()
{

    // Get I2S data and place in data buffer
    size_t bytesIn = 0;
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

    if (result == ESP_OK)
    {
        // Read I2S data buffer
        // int16_t samples_read = bytesIn / 8;
        int16_t samples_read = bufferLen;
        float _decibels = 0;

        if (samples_read > 0)
        {
            float _mean = 0;
            int _counter_non_zero_values = 0;
            for (int16_t i = 0; i < samples_read; ++i)
            {
                if (sBuffer[i] != 0)
                {
                    _counter_non_zero_values++;
                    _mean += (abs(sBuffer[i]));
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
    // Set up I2S
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);

    float _minDb = 10000;
    float _maxDb = -10000;

    for (;;)
    {

        live_decibels = readLiveDB();

        if (live_decibels < _minDb)
            _minDb = live_decibels;

        if (live_decibels > _maxDb)
            _maxDb = live_decibels;

        // PRINT(live_decibels);
        // PRINT(_minDb);
        // PRINT(_maxDb);

        if (record)
        {
            mean_DB += live_decibels;
            samples_DB++;
        }

        // avgDB = _mean / _samples;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void makeNoise(void *parameter)
{

    // OUTPUTS RELAYS NEGATIVE LOGIC
    digitalWrite(SIREN_M1, HIGH); // 3v
    digitalWrite(SIREN_M2, HIGH); // 6v
    digitalWrite(SIREN_M3, HIGH); // 12v

    // pinMode(SIREN_M0, OUTPUT);
    pinMode(SIREN_M1, OUTPUT);
    pinMode(SIREN_M2, OUTPUT);
    pinMode(SIREN_M3, OUTPUT);

    stopRecord();

    for (;;)
    {
        // digitalWrite(SIREN_M0, siren_m0_state); // no siren
        digitalWrite(SIREN_M1, !siren_m1_state); // 3v
        digitalWrite(SIREN_M2, !siren_m2_state); // 6v
        digitalWrite(SIREN_M3, !siren_m3_state); // 12v

        DELAY(10);
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
            stopRecord();
            DELAY(50);
            if (bt_command == "RECORD0")
            {
                turnOnMode(0);
            }
            else if (bt_command == "RECORD1")
            {
                turnOnMode(1);
            }
            else if (bt_command == "RECORD2")
            {
                turnOnMode(2);
            }
            else if (bt_command == "RECORD3")
            {
                turnOnMode(3);
            }
            else if (bt_command == "STOP")
            {
                bt_command = "";
                stopRecord();
                SerialBT.println(calculateAvgDB());
            }
            else
            {
                SerialBT.println("ERROR");
                PRINT("ERROR");
            }
        }

        DELAY(5);
    }
}

void setup()
{
    // init serial
    sDEBUG_BEGIN(115200);

    xTaskCreatePinnedToCore(readMic, "t_readMic", 10000, NULL, 4, &t_readMic, 1);
    xTaskCreatePinnedToCore(makeNoise, "t_makeNoise", 10000, NULL, 1, &t_makeNoise, 0);
    xTaskCreatePinnedToCore(sendRecieveBtData, "t_sendRecieveBtData", 10000, NULL, 4, &t_sendRecieveBtData, 0);
}

void loop()
{
}