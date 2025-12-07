// Included libraries
#include "mbed.h" // Main Mbed OS library
#include "arm_math.h" // CMSIS-DSP library
#include <cmath>

using namespace std::chrono_literals; // for 19ms literal

// LSM6DSL Register Definitions
#define WHO_AM_I     (0x0F) // ID register - should return 0x6A
#define CTRL1_XL     (0x10) // Accelerometer control register to configure range
#define OUTX_L_XL    (0x28) // XL X-axis (low byte)
#define OUTX_H_XL    (0x29) // XL X-axis (high byte)
#define OUTY_L_XL    (0x2A) // XL Y-axis (low byte)
#define OUTY_H_XL    (0x2B) // XL Y-axis (high byte)
#define OUTZ_L_XL    (0x2C) // XL Z-axis (low byte)
#define OUTZ_H_XL    (0x2D) // XL Z-axis (high byte)
#define LSM6DSL_ADDR (0x6A << 1) // Equals 0xD4

// DSP Parameters
#define FFT_SIZE    128 // FFT Size (Must be power of 2: 128, 256, 512)
#define SAMPLE_RATE 52.0f // Sensor ODR in Hz

// Objects
arm_rfft_fast_instance_f32 S; // FFT instance
I2C i2c(PB_11, PB_10);  // I2C2: SDA = PB11, SCL = PB10

// ---- UI objects (LED + Button) ----
DigitalOut status_led(LED1);   // single on-board LED
DigitalIn  user_button(BUTTON1); // USER button (active-low)

// Frequency states
typedef enum{

    NORMAL,
    TREMOR,
    DYSKINESIA,
    OUT_OF_RANGE

} OscillationState;

// Movement states
typedef enum{

    IDLE,
    WALKING,
    FREEZED

} MovementState;

// Global variables
const float ACC_SENSITIVITY = 0.061f; // mg/LSB for ±2g
const float FREQ_BIN_SIZE = SAMPLE_RATE / FFT_SIZE; // Derived Resolution: 52 / 128 = 0.40625 Hz per bin
const float WALK_MIN_FREQ = 1.0f; // walking band lower bound
const float WALK_MAX_FREQ = 3.0f; // walking band upper bound
const float LOW_FREQ_TH   = 1.0f; // not walking
const float BIG_DROP_TH   = -1.0f; // big decrease
const float BIG_CHANGE_TH =  0.5f; // noticeable change
volatile float current_dominant_freq = 0.0f; // Holds calculated frequency for printing
volatile float prev_dominant_freq = 0.0f; // Holds frequency captured for processing

// FFT Buffers
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float fft_mag[FFT_SIZE / 2];
int fft_idx = 0;

// ---- Extra globals for UI ----
OscillationState g_osc_state = NORMAL;
MovementState    g_move_state = IDLE;
bool g_reset_flag = false; // set by long button press

// Function prototypes
void write_register( uint8_t reg, uint8_t value );
uint8_t read_register( uint8_t reg );
int16_t read_16bit_value( uint8_t reg_low, uint8_t reg_high );
bool configure_device();
void fft( float input_signal );
OscillationState determine_oscillation_state( float frequency );
MovementState determine_movement_state( float frequency );

// UI helpers
void update_led_pattern();
void handle_button();

int main(){

    // Configure the device 
    if( !configure_device() ){

        printf("Device configuration failed!\r\n");
        while(1);

    }

    // Initialize the FFT library
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    // Sample interval counter
    int sample_interval = 0;
    
    // Main loop
    while (1){

        // Read raw accelerometer values, convert to g, and remove gravity offset
        float acc_x_g = ( read_16bit_value(OUTX_L_XL, OUTX_H_XL) * ACC_SENSITIVITY / 1000.0f );
        float acc_y_g = ( read_16bit_value(OUTY_L_XL, OUTY_H_XL) * ACC_SENSITIVITY / 1000.0f );
        float acc_z_g = ( read_16bit_value(OUTZ_L_XL, OUTZ_H_XL) * ACC_SENSITIVITY / 1000.0f ) - 1.0f; // Subtract 1g for gravity
        float acc_magnitude = sqrtf( acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g ); // Magnitude

        // Process accelerometer magnitude through FFT
        fft( acc_magnitude );

        // Sample accelerometer dominant frequency every ~3 seconds
        if( sample_interval++ >= (int)(3.0f * SAMPLE_RATE) ){

            // Reset sample interval
            sample_interval = 0;
        
            // Determine oscillation state
            OscillationState state = determine_oscillation_state( current_dominant_freq );

            // --- keep EXACT same prints as original ---
            if( state == NORMAL ) printf("NORMAL (%.2f Hz)\r\n", current_dominant_freq);
            else if( state == TREMOR ) printf("TREMOR (%.2f Hz)\r\n", current_dominant_freq);
            else if( state == DYSKINESIA ) printf("DYSKINESIA (%.2f Hz)\r\n", current_dominant_freq);
            else printf("OUT OF RANGE (%.2f Hz)\r\n", current_dominant_freq);

            // Determine movement state
            MovementState move_state = determine_movement_state( current_dominant_freq );
            if( move_state == IDLE ) printf("Movement State: IDLE\r\n");
            else if( move_state == WALKING ) printf("Movement State: WALKING\r\n");
            else if( move_state == FREEZED ) printf("Movement State: FREEZED\r\n");

            // Save states for LED UI
            g_osc_state  = state;
            g_move_state = move_state;

        }

        // Handle long-press button reset (no change to prints)
        handle_button();

        // Update LED pattern based on latest states
        update_led_pattern();

        // Wait before next sample
        ThisThread::sleep_for(19ms); // ~19ms for 52Hz sampling

    }

};

// Write a value to a register
void write_register( uint8_t reg, uint8_t value ){

    char data[2] = {(char)reg, (char)value};
    i2c.write(LSM6DSL_ADDR, data, 2);

}

// Read a value from a register
uint8_t read_register( uint8_t reg ){

    char data = reg;
    i2c.write(LSM6DSL_ADDR, &data, 1, true); // No stop
    i2c.read(LSM6DSL_ADDR, &data, 1);
    return (uint8_t)data;
    
}

// Read a 16-bit value (combines low and high byte registers)
int16_t read_16bit_value( uint8_t low_reg, uint8_t high_reg ){

    // Read low byte
    char low_byte = read_register(low_reg);
    
    // Read high byte
    char high_byte = read_register(high_reg);
    
    // Combine the bytes (little-endian: low byte first)
    return (high_byte << 8) | low_byte;

}

// Configure the LSM6DSL device
bool configure_device(){

    // Setup I2C at 400kHz
    i2c.frequency(400000);
    
    // Verify devicd id   
    if( read_register(WHO_AM_I) != 0x6A ) return false;
    
    // Configure the accelerometer (52 Hz, ±2g range)
    write_register(CTRL1_XL, 0x30); // ODR = 52 Hz (0011), FS = ±2g (00)

    return true; 

}

// Process input signal through FFT to find dominant frequency
void fft( float input_signal ){
    
    // Add sample to FFT input buffer
    fft_input[fft_idx] = input_signal;
    fft_idx++;

    // Check if buffer is full
    if (fft_idx >= FFT_SIZE) {

        // 1. Compute FFT (Real -> Complex)
        arm_rfft_fast_f32(&S, fft_input, fft_output, 0);
        
        // 2. Compute Magnitude (Complex -> Real)
        arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);
        
        // 3. Find Dominant Frequency (Peak)
        float max_val = 0.0f;
        int max_bin = 0;
        
        // Start at i=1 to skip DC bias (0 Hz) 
        for (int i = 1; i < FFT_SIZE / 2; i++) {

            if (fft_mag[i] > max_val) {

                max_val = fft_mag[i];
                max_bin = i;

            }

        }
        
        // 4. Convert Bin Index to Hz
        // Add threshold to filter noise
        if (max_val > 0.3f) current_dominant_freq = max_bin * FREQ_BIN_SIZE;
        else current_dominant_freq = 0.0f;
        
        // Reset index to start filling buffer again
        fft_idx = 0;

    }

}

// Determine Oscillation State based on Frequency
OscillationState determine_oscillation_state( float frequency ){

    // Define thresholds (in Hz)
    if( frequency >= 3.0f && frequency <= 5.0f ) return TREMOR;
    else if( frequency > 5.0f && frequency <= 7.0f ) return DYSKINESIA;
    else if ( frequency > 7.0f) return OUT_OF_RANGE;
    else return NORMAL;

}

// Determine Movement State based on Frequency
MovementState determine_movement_state( float frequency ){

    // Initalize initial state
    static MovementState current_state = IDLE;

    // Initalize iniitial previous frequency
    static float prev_dominant_freq_local = 0.0f;

    // ---- handle reset request from long button press ----
    if (g_reset_flag) {
        current_state = IDLE;
        prev_dominant_freq_local = 0.0f;
        g_reset_flag = false;
        return current_state;
    }

    // Calculate frequency change 
    float delta = frequency - prev_dominant_freq_local;

    switch (current_state) {

        case IDLE:

            // IDLE -> WALKING if we clearly enter walking band with change
            if (frequency >= WALK_MIN_FREQ && frequency <= WALK_MAX_FREQ && fabsf(delta) > BIG_CHANGE_TH) current_state = WALKING;

            // stay IDLE
            else  current_state = IDLE;

            break;

        case WALKING:

            // WALKING -> FREEZED: big DROP into low frequency
            if (frequency < LOW_FREQ_TH && delta < BIG_DROP_TH) current_state = FREEZED;

            // WALKING -> IDLE: just smoothly slowed down
            else if (frequency < LOW_FREQ_TH && fabsf(delta) <= BIG_CHANGE_TH) current_state = IDLE;

            // stay WALKING
            else current_state = WALKING;

            break;

        case FREEZED:

            // FREEZED -> WALKING: walking band with noticeable change
            if (frequency >= WALK_MIN_FREQ && frequency <= WALK_MAX_FREQ && fabsf(delta) > BIG_CHANGE_TH) current_state = WALKING;

            // FREEZED -> IDLE: stays low without big changes
            else if (frequency < LOW_FREQ_TH && fabsf(delta) <= BIG_CHANGE_TH) current_state = IDLE;

            // else remain FREEZED
            else current_state = FREEZED;

            break;

        default:

            current_state = IDLE; // safety fallback
            break;

    }

    // Save previous frequency
    prev_dominant_freq_local = frequency;

    // Return current state
    return current_state;
    
}

// -------- LED UI: use g_osc_state + g_move_state to drive a single LED --------
// Priority: FREEZED (FOG) > DYSKINESIA > TREMOR > WALKING > IDLE/NORMAL
void update_led_pattern() {

    const int TICK_MS = 19;  // called once per loop

    static int elapsed_ms = 0;
    static int last_mode  = -1;

    // mode:
    // 0 = IDLE/NORMAL
    // 1 = WALKING
    // 2 = TREMOR
    // 3 = DYSKINESIA
    // 4 = FREEZED
    int mode = 0;

    if (g_move_state == FREEZED) {
        mode = 4;
    } else if (g_osc_state == DYSKINESIA) {
        mode = 3;
    } else if (g_osc_state == TREMOR) {
        mode = 2;
    } else if (g_move_state == WALKING) {
        mode = 1;
    } else {
        mode = 0;
    }

    if (mode != last_mode) {
        elapsed_ms = 0;
        last_mode  = mode;
    } else {
        elapsed_ms += TICK_MS;
    }

    switch (mode) {

        case 4: {
            // FREEZED (FOG): very slow blink (1s ON, 1s OFF)
            int period  = 2000;
            int on_time = 1000;
            int t = elapsed_ms % period;
            status_led = (t < on_time) ? 1 : 0;
            break;
        }

        case 3: {
            // DYSKINESIA: double-flash (ON 100ms, OFF 100ms, ON 100ms, OFF 500ms)
            int period = 800;
            int t = elapsed_ms % period;
            if ((t < 100) || (t >= 200 && t < 300)) {
                status_led = 1;
            } else {
                status_led = 0;
            }
            break;
        }

        case 2: {
            // TREMOR: fast blink (~4 Hz)
            int period  = 250;
            int on_time = 125;
            int t = elapsed_ms % period;
            status_led = (t < on_time) ? 1 : 0;
            break;
        }

        case 1: {
            // WALKING: solid ON
            status_led = 1;
            break;
        }

        case 0:
        default: {
            // IDLE / NORMAL: LED OFF
            status_led = 0;
            break;
        }
    }
}

// -------- Button handling: long press -> reset states --------
//  - short press: ignored
//  - long press (~>=600ms): set g_reset_flag, real reset done in determine_movement_state()
void handle_button() {

    static bool prev_pressed = false;
    static int  press_ticks  = 0;

    // BUTTON1 is active-low (pressed = 0)
    bool pressed = (user_button.read() == 0);

    if (pressed) {
        press_ticks++;
    }

    // Just released
    if (!pressed && prev_pressed) {

        int duration_ms = press_ticks * 19; // approximate duration in ms

        if (duration_ms >= 600) {
            // Long press detected: request reset of movement state machine
            g_reset_flag = true;
            printf("[BUTTON] Long press detected. Resetting states...\r\n");
        }

        // Reset counter after release
        press_ticks = 0;
    }

    prev_pressed = pressed;
}


/* maybe not needed

#define CTRL2_G      (0x11) // Gyroscope control register to configure range
#define OUTX_L_G     (0x22) // Gyro X-axis (low byte)
#define OUTX_H_G     (0x23) // Gyro X-axis (high byte)
#define OUTY_L_G     (0x24) // Gyro Y-axis (low byte)
#define OUTY_H_G     (0x25) // Gyro Y-axis (high byte)
#define OUTZ_L_G     (0x26) // Gyro Z-axis (low byte)
#define OUTZ_H_G     (0x27) // Gyro Z-axis (high byte)

//const float GYRO_SENSITIVITY = 8.75f; // mdps/LSB for ±250 dps

--- configure device function
// Configure the gyroscope (52 Hz, ±250 dps range)
write_register(CTRL2_G, 0x30); // ODR = 52 Hz (0011), FS = ±250 dps (00)
---

// Read raw gyroscope values and convert to dps
float gyro_x_dps = read_16bit_value(OUTX_L_G, OUTX_H_G) * GYRO_SENSITIVITY / 1000.0f;
float gyro_y_dps = read_16bit_value(OUTY_L_G, OUTY_H_G) * GYRO_SENSITIVITY / 1000.0f;
float gyro_z_dps = read_16bit_value(OUTZ_L_G, OUTZ_H_G) * GYRO_SENSITIVITY / 1000.0f;
float gyro_magnitude = sqrtf( gyro_x_dps * gyro_x_dps + gyro_y_dps * gyro_y_dps + gyro_z_dps * gyro_z_dps ); // Magnitude

// Process gyroscope magnitude through FFT
fft( gyro_magnitude );

*/
