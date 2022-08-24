// ############################################################
// Libraries
// ############################################################

#include <stdio.h> // printf

#include "scd4x_i2c.h"
#include "sensirion_common.h"
#include "sensirion_i2c_hal.h"

#include "pico/stdlib.h"
#include <stdlib.h>
#include "common/pimoroni_i2c.hpp"
#include "pico/sleep.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "hardware/clocks.h"
#include "hardware/rosc.h"
#include "hardware/structs/scb.h"
#include "hardware/watchdog.h"

using namespace pimoroni;

// ############################################################
// Configurables
// ############################################################

// --- SCD41 ---
const uint SCD41_SDA_PIN = 4; // GPIO pin number on the Pico for SCD41 SDA
const uint SCD41_SCL_PIN = 5; // GPIO pin number on the Pico for SCD41 SCL

// --- Buzzer ---
const uint BUZZER_PIN = 15; // GPIO pin number on the Pico

// NOTICE: buzzer selection!

// Use this with the 5V Buzzer by Adafruit (https://www.adafruit.com/product/1536)
// #define BUZZER_TYPE_BUZZER3V5V

// Use this with any regular piezo buzzer
// #define BUZZER_TYPE_PIEZO

// --- Measurement timings ---
const int8_t MEASURE_INTERVAL_MINUTES = 5;         // co2 measurement interval in minutes
const int8_t MEASURE_HIGH_NOTICE_SLEEP_FACTOR = 5; // when high co2 is detected, sleep for this fraction of measurement interval

// --- CO2 levels ---
const uint16_t CO2_LEVEL_NORMAL_UPPER = 1500; // co2 level: normal upper threshold in ppm
const uint16_t CO2_LEVEL_HIGH_UPPER = 2500;   // co2 level: high upper threshold in ppm. Anything higher is considered dangerous!

// --- Battery level ---
const float BATTERY_LOW_WARNING_PERCENT = 15.0f; // battery low warning percent

const bool CHECK_FOR_FIRST_MEASURE = false; // treats first measurement as not trusted, ie sleep after first measurement as if high co2 happened
const bool ALLOW_LED = true;
const uint16_t RESET_AFTER_SUCCESSFUL_MEASUREMENTS = 500; // Reset Pico after n amount of successful measurements. 0 means no restart.

// ############################################################
// Constants
// ############################################################

#ifndef PICO_DEFAULT_LED_PIN
#warning Requires a board with a regular LED
#else
const uint ONBOARD_LED_PIN = PICO_DEFAULT_LED_PIN;
#endif

const uint VSYS_PIN = 29;
const uint CHARGING_PIN = 24;

const float BATTERY_CONVERSION_FACTOR = 3.0f * 3.3f / (1 << 12);
const float FULL_BATTERY_VOLTAGE = 4.2f;  // these are our reference voltages for a full/empty battery, in volts; the values could vary by battery size/manufacturer so you might need to adjust them.
const float EMPTY_BATTERY_VOLTAGE = 2.8f; // these are our reference voltages for a full/empty battery, in volts; the values could vary by battery size/manufacturer so you might need to adjust them.
const uint32_t DEFAULT_SLEEP_TIME_MS = 5000;

// ############################################################
// Version
// ############################################################

static const unsigned VERSION_MAJOR = 1;
static const unsigned VERSION_MINOR = 3;
static const unsigned VERSION_REVISION = 0;

// ############################################################
// Classes
// ############################################################

class Buzzer3v5v
{
private:
    uint32_t _tone_gap_ms;
    uint _buzzer_pin;

public:
    Buzzer3v5v(uint pin_number)
    {
        _tone_gap_ms = 300;
        _buzzer_pin = pin_number;
        gpio_init(_buzzer_pin);
        gpio_set_dir(_buzzer_pin, GPIO_OUT);
    }

    uint32_t get_tone_gap_ms()
    {
        return _tone_gap_ms;
    }

    void set_tone_gap_ms(uint32_t ms)
    {
        _tone_gap_ms = ms;
    }

    void be_quiet()
    {
        gpio_put(_buzzer_pin, 0);
    }

    void beep()
    {
        gpio_put(_buzzer_pin, 1);
    }

    void beep_boop(uint times, bool cont)
    {
        if (cont)
        {
            beep();
            for (uint i = 0; i < times; i++)
            {
                sleep_ms(_tone_gap_ms);
            }

            be_quiet();
        }
        else
        {
            for (uint i = 0; i < times; i++)
            {
                beep();
                sleep_ms(_tone_gap_ms);
                be_quiet();
                sleep_ms(_tone_gap_ms);
            }
        }

        be_quiet();
    }
};

class PiezoBuzzer
{
private:
    uint32_t _tone_gap_ms;
    uint _buzzer_pin;
    int _duty_cicle;
    uint _slice_number;
    uint _channel;

    uint32_t pwm_set_freq_duty(uint slice_number, uint channel, uint32_t frequency, int duty)
    {
        uint32_t clock = 125000000;
        uint32_t divider16 = clock / frequency / 4096 + (clock % (frequency * 4096) != 0);

        if (divider16 / 16 == 0)
        {
            divider16 = 16;
        }

        uint32_t wrap = clock * 16 / divider16 / frequency - 1;
        pwm_set_clkdiv_int_frac(slice_number, divider16 / 16, divider16 & 0xF);
        pwm_set_wrap(slice_number, wrap);
        pwm_set_chan_level(slice_number, channel, wrap * duty / 100);
        return wrap;
    }

public:
    PiezoBuzzer(uint pin_number)
    {
        _tone_gap_ms = 300;
        _buzzer_pin = pin_number;
        _duty_cicle = 5000;

        gpio_set_function(_buzzer_pin, GPIO_FUNC_PWM);
        _slice_number = pwm_gpio_to_slice_num(_buzzer_pin);
        _channel = pwm_gpio_to_channel(_buzzer_pin);

        be_quiet();
    }

    uint32_t get_tone_gap_ms()
    {
        return _tone_gap_ms;
    }

    void set_tone_gap_ms(uint32_t ms)
    {
        _tone_gap_ms = ms;
    }

    int get_duty_cicle()
    {
        return _duty_cicle;
    }

    void set_duty_cicle(int duty_cicle)
    {
        _duty_cicle = duty_cicle;
    }

    void play_tone(uint32_t frequency)
    {
        pwm_set_freq_duty(_slice_number, _channel, frequency, _duty_cicle);
        pwm_set_enabled(_slice_number, true);
    }

    void be_quiet()
    {
        pwm_set_enabled(_slice_number, false);
    }

    void beep_boop(uint times, bool cont, uint32_t frequency = 2000)
    {
        if (cont)
        {
            play_tone(frequency);
            for (uint i = 0; i < times; i++)
            {
                sleep_ms(_tone_gap_ms);
            }

            be_quiet();
        }
        else
        {
            for (uint i = 0; i < times; i++)
            {
                play_tone(frequency);
                sleep_ms(_tone_gap_ms);
                be_quiet();
                sleep_ms(_tone_gap_ms);
            }
        }

        be_quiet();
    }
};

// ############################################################
// Global variables
// ############################################################

#if defined(BUZZER_TYPE_BUZZER3V5V) and defined(BUZZER_TYPE_PIEZO)
#warning Multiple buzzer type selected!
#elif defined(BUZZER_TYPE_BUZZER3V5V)
Buzzer3v5v buzzer(BUZZER_PIN);
#elif defined(BUZZER_TYPE_PIEZO)
PiezoBuzzer buzzer(BUZZER_PIN);
#else
#warning Buzzer selection mandatory!
#endif

static bool awake = true;

I2C *i2c = new I2C(SCD41_SDA_PIN, SCD41_SCL_PIN);

uint scb_orig;
uint sleep_en0_orig;
uint sleep_en1_orig;
uint enabled0_orig;
uint enabled1_orig;
uint wake_en0_orig;
uint wake_en1_orig;
uint intr_orig;
uint inte_orig;
uint intf_orig;
uint ints_orig;

// ############################################################
// Functions
// ############################################################

void recover_from_sleep()
{
    // Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    // reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = sleep_en0_orig;
    clocks_hw->sleep_en1 = sleep_en1_orig;
    // clocks_hw->enabled0 = enabled0_orig;
    // clocks_hw->enabled1 = enabled1_orig;
    clocks_hw->wake_en0 = wake_en0_orig;
    clocks_hw->wake_en1 = wake_en1_orig;
    // clocks_hw->intr = intr_orig;
    clocks_hw->inte = inte_orig;
    clocks_hw->intf = intf_orig;
    // clocks_hw->ints = ints_orig;

    // reset clocks
    clocks_init();
    stdio_init_all();
    sleep_ms(DEFAULT_SLEEP_TIME_MS);
    return;
}

void save_clock_registers()
{
#ifdef DEBUG
    printf("[TRC] Saving clocks.\n");
#endif

    scb_orig = scb_hw->scr;
    sleep_en0_orig = clocks_hw->sleep_en0;
    sleep_en1_orig = clocks_hw->sleep_en1;
    enabled0_orig = clocks_hw->enabled0;
    enabled1_orig = clocks_hw->enabled1;
    wake_en0_orig = clocks_hw->wake_en0;
    wake_en1_orig = clocks_hw->wake_en1;
    intr_orig = clocks_hw->intr;
    inte_orig = clocks_hw->inte;
    intf_orig = clocks_hw->intf;
    ints_orig = clocks_hw->ints;
}

static void sleep_callback(void)
{
    awake = true;

    return;
}

static void rtc_sleep(int8_t minutes, int8_t seconds)
{
    datetime_t t = {
        .year = 2020,
        .month = 06,
        .day = 05,
        .dotw = 5,
        .hour = 15,
        .min = 00,
        .sec = 00,
    };

    datetime_t t_alarm = {
        .year = 2020,
        .month = 06,
        .day = 05,
        .dotw = 5,
        .hour = 15,
        .min = minutes,
        .sec = seconds,
    };

    rtc_init();
    sleep_run_from_xosc();
    if (!rtc_set_datetime(&t))
    {
        printf("[ERR] Could not set datetime!\n");
        printf("[INF] Regular sleep...\n");
        sleep_ms(((minutes * 60) + seconds) * 1000);
        return;
    }

    awake = false;

#ifdef DEBUG
    printf("[TRC] Deinit I2C.\n");
#endif
    i2c->~I2C();
    delete i2c;

#ifdef DEBUG
    printf("[DBG] RTC sleep start...\n");
#endif

    uart_default_tx_wait_blocking();
    sleep_goto_sleep_until(&t_alarm, &sleep_callback);

    uart_default_tx_wait_blocking();
    recover_from_sleep();

#ifdef DEBUG
    printf("[TRC] Reinit I2C.\n");
#endif
    i2c = new I2C(SCD41_SDA_PIN, SCD41_SCL_PIN);
    sensirion_i2c_hal_init(i2c);

#ifdef DEBUG
    printf("[DBG] Awakened from RTC sleep.\n");
#endif
}

// Turn off LED
void turn_off_led()
{
    gpio_put(ONBOARD_LED_PIN, false);
}

// Turn on LED
void turn_on_led()
{
    if (ALLOW_LED)
    {
        gpio_put(ONBOARD_LED_PIN, true);
    }
}

void blink_led(uint times, uint32_t blink_gap_ms = 100)
{
    if (!ALLOW_LED)
    {
        return;
    }

    turn_off_led();
    for (uint i = 0; i < times; i++)
    {
        turn_on_led();
        sleep_ms(blink_gap_ms);
        turn_off_led();
    }
}

// Get battery status
void battery_status(bool *charging, float *voltage, float *percentage)
{
    *charging = gpio_get(CHARGING_PIN);
    *voltage = adc_read() * BATTERY_CONVERSION_FACTOR;
    *percentage = 100.0f * (*voltage - EMPTY_BATTERY_VOLTAGE) / (FULL_BATTERY_VOLTAGE - EMPTY_BATTERY_VOLTAGE);

    if (*percentage >= 100.0f)
    {
        *percentage = 100.0f;
    }
    else if (*percentage <= 0.0f)
    {
        *percentage = 0.0f;
    }
}

void scd41_reinit()
{
    printf("[INF] SCD41 wake up and init.\n");

    int16_t error;
    error = scd4x_wake_up();
#ifdef DEBUG
    printf("[TRC] scd4x_wake_up  %i\n", error);
#endif
    error = scd4x_stop_periodic_measurement();
#ifdef DEBUG
    printf("[TRC] scd4x_stop_periodic_measurement %i\n", error);
#endif
    error = scd4x_reinit();
#ifdef DEBUG
    printf("[TRC] scd4x_reinit %i\n", error);
#endif
    if (error)
    {
        printf("[ERR] Error executing scd4x_reinit(): %i\n", error);
    }
}

void scd41_start_periodic_measurement(bool sleep)
{
    scd41_reinit();

    printf("[INF] SCD41 start periodic measurement...\n");
    sleep_ms(1000);

    int16_t error;
    uint32_t sleep_time;
    if ((MEASURE_INTERVAL_MINUTES * 60) / MEASURE_HIGH_NOTICE_SLEEP_FACTOR < 60)
    {
        error = scd4x_start_periodic_measurement();
#ifdef DEBUG
        printf("[TRC] scd4x_start_periodic_measurement %i\n", error);
#endif
        if (error)
        {
            printf("[ERR] Error executing scd4x_start_periodic_measurement(): %i\n", error);
        }
        sleep_time = 5000;
    }
    else
    {
        error = scd4x_start_low_power_periodic_measurement();
#ifdef DEBUG
        printf("[TRC] scd4x_start_low_power_periodic_measurement %i\n", error);
#endif
        if (error)
        {
            printf("[ERR] Error executing scd4x_start_low_power_periodic_measurement(): %i\n", error);
        }
        sleep_time = 30000;
    }

    if (sleep)
    {
        sleep_ms(sleep_time);
    }
}

void scd41_power_down()
{
    printf("[INF] Powering down scd41.\n");

    int16_t error;
    error = scd4x_stop_periodic_measurement();
#ifdef DEBUG
    printf("[TRC] scd4x_stop_periodic_measurement %i\n", error);
#endif
    error = scd4x_power_down();
#ifdef DEBUG
    printf("[TRC] scd4x_power_down %i\n", error);
#endif
    uart_default_tx_wait_blocking();
}

void scd41_self_test()
{
    scd41_reinit();
    printf("[INF] Running scd41 selftest.\n");

    uint16_t sensor_status;
    int16_t error;
    error = scd4x_perform_self_test(&sensor_status);
#ifdef DEBUG
    printf("[TRC] scd4x_perform_self_test %i\n", error);
#endif
    if (error)
    {
        printf("[ERR] Error executing scd4x_perform_self_test(): %i\n", error);
    }
    else if (sensor_status)
    {
        printf("[ERR] Malfunction detected: %i\n", sensor_status);
        buzzer.beep_boop(10, false);
    }
    else
    {
        printf("[INF] No malfunction detected.\n");
    }
}

void software_reset()
{
    watchdog_reboot(0, SRAM_END, 0);
    while (true)
    {
    }
}

// Setup before main loop
void setup()
{
    stdio_init_all();
    printf("[INF] Setup start.\n");

    // save clocks
    save_clock_registers();

    // --- LED ---
#ifdef DEBUG
    printf("[TRC] Init onboard LED.\n");
#endif
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);
    turn_off_led();

    turn_on_led();
    buzzer.be_quiet();

    // --- Charging pin ---
#ifdef DEBUG
    printf("[TRC] Init charging pin.\n");
#endif
    gpio_init(CHARGING_PIN);
    gpio_set_dir(CHARGING_PIN, GPIO_IN);

    // --- VSYS pin ---
#ifdef DEBUG
    printf("[TRC] Init VSYS pin.\n");
#endif
    adc_init();
    adc_gpio_init(VSYS_PIN);
    adc_select_input(3);

    // --- SCD41 ---
#ifdef DEBUG
    printf("[TRC] Init I2C.\n");
#endif
    sensirion_i2c_hal_init(i2c);

    int16_t error;
    uint16_t serial_0;
    uint16_t serial_1;
    uint16_t serial_2;
    error = scd4x_get_serial_number(&serial_0, &serial_1, &serial_2);
#ifdef DEBUG
    printf("[TRC] scd4x_get_serial_number %i\n", error);
#endif
    if (error)
    {
        printf("[ERR] Error executing scd4x_get_serial_number(): %i\n", error);
        printf("[ERR] SCD41 not found!\n");
    }
    else
    {
        printf("[INF] Serial: 0x%04x%04x%04x\n", serial_0, serial_1, serial_2);
    }

    scd41_self_test();
    scd41_start_periodic_measurement(true);

    turn_off_led();
    printf("[INF] Setup finished.\n");
}

// Main loop
void loop()
{
    bool firstmeasure = CHECK_FOR_FIRST_MEASURE;
    bool measured = false;
    bool stopped = false;
    uint loop_count = 1U;
    uint successful_measurement_count = 0U;
    uint co2_high_count = 0U;
    int16_t error = 0;

    while (true)
    {
        uart_default_tx_wait_blocking();
#ifdef DEBUG
        printf("[DBG] Loops: %d, successful measurements: %d\n", loop_count, successful_measurement_count);
#endif

        if (RESET_AFTER_SUCCESSFUL_MEASUREMENTS != 0 && successful_measurement_count >= RESET_AFTER_SUCCESSFUL_MEASUREMENTS)
        {
            printf("[INF] Restart point reached. Restarting...\n");

            if (stopped)
            {
                scd41_reinit();
            }

            uart_default_tx_wait_blocking();
            software_reset();
        }

        if (stopped)
        {
            scd41_reinit();
            scd41_start_periodic_measurement(true);
            stopped = false;
        }

        bool charging;
        float voltage;
        float percentage;
        battery_status(&charging, &voltage, &percentage);
        printf("[INF] Charging: %d, voltage: %.2f V, percentage: %.0f %%\n", charging, voltage, percentage);

        if (!charging && percentage <= BATTERY_LOW_WARNING_PERCENT)
        {
            printf("[INF] Battery level low!\n");
            buzzer.beep_boop(2, false);
            sleep_ms(2000);
            buzzer.beep_boop(2, false);
        }

#ifdef DEBUG
        printf("[DBG] Trying to measure...\n");
#endif
        bool co2_high = false;

        sleep_ms(DEFAULT_SLEEP_TIME_MS);

        bool ready = false;
        uint16_t dr;
        error = scd4x_get_data_ready_status(&dr);
#ifdef DEBUG
        printf("[TRC] scd4x_get_data_ready_status %i\n", error);
#endif
        if (error)
        {
            printf("[ERR] Error executing scd4x_get_data_ready_status(): %i\n", error);
        }
        else
        {
            ready = true;
#ifdef DEBUG
            printf("[DBG] Data ready: %d\n", dr);
#endif
        }

        if (ready)
        {
            blink_led(1);
            uint16_t co2;
            int32_t temperature;
            int32_t humidity;
            error = scd4x_read_measurement(&co2, &temperature, &humidity);
#ifdef DEBUG
            printf("[TRC] scd4x_read_measurement %i\n", error);
#endif
            if (error)
            {
                printf("[ERR] Error executing scd4x_read_measurement(): %i\n", error);
            }
            else if (co2 == 0)
            {
                printf("[ERR] Invalid sample detected.\n");
            }
            else
            {
                printf("[INF] CO2: %u, Temperature: %ldC, Humidity: %ld mRH\n", co2, temperature, humidity);
                measured = true;
                successful_measurement_count++;

                if (co2 <= CO2_LEVEL_NORMAL_UPPER)
                {
                    printf("[INF] CO2 level good.\n");
                }
                else if (co2 <= CO2_LEVEL_HIGH_UPPER)
                {
                    printf("[INF] CO2 level high.\n");
                    co2_high = true;
                    co2_high_count++;
                    buzzer.beep_boop(5, false);
                }
                else
                {
                    printf("[INF] CO2 level danger.\n");
                    co2_high = true;
                    co2_high_count++;
                    buzzer.beep_boop(30, true);
                }
            }
        }
        else
        {
            printf("[INF] SCD41 not ready: %i\n", error);
            measured = false;
            blink_led(2);
        }

        if (measured)
        {
            measured = false;
#ifdef DEBUG
            printf("[DBG] CO2 high: %d, High count: %d\n", co2_high, co2_high_count);
#endif

            if (!co2_high && co2_high_count < 1)
            {
                co2_high_count = 0;

                if (firstmeasure)
                {
                    firstmeasure = false;
                    int8_t min = MEASURE_INTERVAL_MINUTES / MEASURE_HIGH_NOTICE_SLEEP_FACTOR;
                    int8_t sec = 10;

                    if (min == 0)
                    {
                        min = 0;
                        sec = (MEASURE_INTERVAL_MINUTES * 60) / MEASURE_HIGH_NOTICE_SLEEP_FACTOR;
                    }

                    printf("[INF] First measurement. rtc_sleep start. (%dmin %dsec)\n", min, sec);
                    scd41_power_down();
                    stopped = true;
                    rtc_sleep(min, sec);
                }
                else
                {
                    int8_t min = MEASURE_INTERVAL_MINUTES;
                    int8_t sec = 10;
                    printf("[INF] rtc_sleep start. (%dmin %dsec)\n", min, sec);
                    scd41_power_down();
                    stopped = true;
                    rtc_sleep(min, sec);
                }
            }
            else
            {
                co2_high_count--;
                int8_t min = MEASURE_INTERVAL_MINUTES / MEASURE_HIGH_NOTICE_SLEEP_FACTOR;
                int8_t sec = 10;

                if (min == 0)
                {
                    min = 0;
                    sec = (MEASURE_INTERVAL_MINUTES * 60) / MEASURE_HIGH_NOTICE_SLEEP_FACTOR;
                }

                printf("[INF] rtc_sleep start. (%dmin %dsec)\n", min, sec);
                scd41_power_down();
                stopped = true;
                rtc_sleep(min, sec);
            }
        }
        else
        {
            printf("[INF] No measurement, sleep and retry.\n");
            sleep_ms(DEFAULT_SLEEP_TIME_MS);
        }

        loop_count++;
    }
}

// ############################################################
// Main
// ############################################################

int main(void)
{
    setup();
    uart_default_tx_wait_blocking();
    loop();

    return 0;
}
