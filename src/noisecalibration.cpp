/*
 *
 *  Decibel Calibration Station
 *  for NoisePollution App
 *  Copyright (C) 2022 Alex Agrafiotis
 *
 */

#define DEBUG
#include <serialDebugger.h>

// lib for buttons
#include "InputDebounce.h"

// lib for mic
#include <driver/i2s.h>

// lib for speaker and SD
#include "Arduino.h"
// #include <Audio.h>
#include <SD.h>
#include <FS.h>

// microSD Card Reader connections
#define SD_CS 5
#define SPI_MOSI 23
#define SPI_MISO 19
#define SPI_SCK 18

// I2S Speaker Connections
#define I2S_DOUT 26
#define I2S_BCLK 27
#define I2S_LRC 14

//  INMP441 I2S microphone Connections to
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 1 for mic
#define I2S_PORT I2S_NUM_1

// Define input buffer length
#define bufferLen 3584 //? 3584 on phones || default is 64

#define BUTTON1 5
#define BUTTON2 18
#define BUTTON3 19
#define BUTTON_DEBOUNCE_DELAY 20 // [ms]

#define SIREN 4

static InputDebounce button1, button2, button3;

// mic samples buffer
int16_t sBuffer[bufferLen];

// RTOS
TaskHandle_t t_readMic;
TaskHandle_t t_makeNoise;
TaskHandle_t t_displayDecibels;

// global vars
bool state1, prev_state1;
bool state2, prev_state2;
bool state3, prev_state3;

bool button1_pressed = false;
bool button2_pressed = false;
bool button3_pressed = false;

void i2s_install()
{
    // Set up I2S Processor configuration
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 44100,
        .bits_per_sample = i2s_bits_per_sample_t(16),
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S), // I2S_COMM_FORMAT_I2S
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
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

void checkButtons()
{

    // check button 1
    state1 = button1.process(millis());

    if (state1 && !prev_state1)
    {
        debugln("1");
        button1_pressed = true;
    }

    prev_state1 = state1;

    // check button 2
    state2 = button2.process(millis());

    if (state2 && !prev_state2)
    {
        debugln("2");
        button2_pressed = true;
    }

    prev_state2 = state2;

    // check button 3
    state3 = button3.process(millis());

    if (state3 && !prev_state3)
    {
        debugln("3");
        button3_pressed = true;
    }

    prev_state3 = state3;
}

void readMic(void *parameter)
{

    for (;;)
    {
        // False print statements to "lock range" on serial plotter display
        // Change rangelimit value to adjust "sensitivity"
        int rangelimit = 200;
        Serial.print(rangelimit * -1);
        Serial.print(" ");
        Serial.print(rangelimit);
        Serial.print(" ");

        // Get I2S data and place in data buffer
        size_t bytesIn = 0;
        esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

        if (result == ESP_OK)
        {
            // Read I2S data buffer
            int16_t samples_read = bytesIn / 8;
            if (samples_read > 0)
            {
                float mean = 0;

                for (int16_t i = 0; i < samples_read; ++i)
                {
                    mean += (sBuffer[i]); //? abs()
                }

                // Average the data reading
                mean /= samples_read;
                // Serial.println(samples_read);

                // Print to serial plotter
                Serial.println(mean);

                //! decibel formula
                //? 20*Math.log10(mean/32768) + 110; (in java)
            }
        }

        // vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

void makeNoise(void *parameter)
{

    for (;;)
    {
        digitalWrite(SIREN, 1);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        digitalWrite(SIREN, 0);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void setup()
{

    xTaskCreatePinnedToCore(readMic, "t_readMic", 10000, NULL, 1, &t_readMic, 1);
    xTaskCreatePinnedToCore(makeNoise, "t_makeNoise", 10000, NULL, 1, &t_makeNoise, 0);

    // setup input buttons (debounced)
    button1.setup(BUTTON1, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
    button2.setup(BUTTON2, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);
    button3.setup(BUTTON3, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_INT_PULL_UP_RES);

    // OUTPUTS
    pinMode(SIREN, OUTPUT);
    digitalWrite(SIREN, LOW);

    // Set up I2S
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);

    // Set microSD Card CS as OUTPUT and set HIGH
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);

    // Initialize SPI bus for microSD Card
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // init serial
    Serial.begin(115200);
}

void loop()
{
    checkButtons();
}