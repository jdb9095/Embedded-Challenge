// Included libraries
#include "mbed.h" // Main Mbed OS library
#include "arm_math.h" // CMSIS-DSP library

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

// Function prototypes
void write_register( uint8_t reg, uint8_t value );
uint8_t read_register( uint8_t reg );
int16_t read_16bit_value( uint8_t reg_low, uint8_t reg_high );
bool configure_device();
void fft( float input_signal );
OscillationState determine_oscillation_state( float frequency );
MovementState determine_movement_state( float frequency );

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

            if( state == NORMAL ) printf("NORMAL (%.2f Hz)\r\n", current_dominant_freq);
            else if( state == TREMOR ) printf("TREMOR (%.2f Hz)\r\n", current_dominant_freq);
            else if( state == DYSKINESIA ) printf("DYSKINESIA (%.2f Hz)\r\n", current_dominant_freq);
            else printf("OUT OF RANGE (%.2f Hz)\r\n", current_dominant_freq);

            // Determine movement state
            MovementState move_state = determine_movement_state( current_dominant_freq );
            if( move_state == IDLE ) printf("Movement State: IDLE\r\n");
            else if( move_state == WALKING ) printf("Movement State: WALKING\r\n");
            else if( move_state == FREEZED ) printf("Movement State: FREEZED\r\n");

        }

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
    static float prev_dominant_freq = 0.0f;

    // Calculate frequency change 
    float delta = frequency - prev_dominant_freq;

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
    prev_dominant_freq = frequency;

    // Return current state
    return current_state;
    
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