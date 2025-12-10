// Included libraries
#include "mbed.h" // Main Mbed OS library
#include "arm_math.h" // CMSIS-DSP library
// BLE includes
#include "events/mbed_events.h"
#include "ble/BLE.h"
#include "ble/GattServer.h"
#include "ble/Gap.h"
// Advertising builder helper
#include "ble/gap/AdvertisingDataBuilder.h"
#include <cmath>
#include <string.h>  // ADDED: for strcpy, strlen

using namespace std::chrono_literals; // for 19ms literal

// LSM6DSL Register Definitions
#define WHO_AM_I     (0x0F) // ID register - should return 0x6A
#define CTRL1_XL     (0x10) // Accelerometer control register to configure range
#define CTRL2_G      (0x11) // Gyroscope control register to configure range
#define OUTX_L_XL    (0x28) // XL X-axis (low byte)
#define OUTX_H_XL    (0x29) // XL X-axis (high byte)
#define OUTY_L_XL    (0x2A) // XL Y-axis (low byte)
#define OUTY_H_XL    (0x2B) // XL Y-axis (high byte)
#define OUTZ_L_XL    (0x2C) // XL Z-axis (low byte)
#define OUTZ_H_XL    (0x2D) // XL Z-axis (high byte)
#define OUTX_L_G     (0x22) // Gyro X-axis (low byte)
#define OUTX_H_G     (0x23) // Gyro X-axis (high byte)
#define OUTY_L_G     (0x24) // Gyro Y-axis (low byte)
#define OUTY_H_G     (0x25) // Gyro Y-axis (high byte)
#define OUTZ_L_G     (0x26) // Gyro Z-axis (low byte)
#define OUTZ_H_G     (0x27) // Gyro Z-axis (high byte)
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

// BLE / GATT globals
static events::EventQueue event_queue(16 * EVENTS_EVENT_SIZE);
// Rename BLE instance variable to avoid collision with the `ble` namespace
static ble::BLE &ble_instance = ble::BLE::Instance();
static bool ble_ready = false;

// Symptom string constants and buffer for BLE
const char* SYMPTOM_NONE = "NONE";
const char* SYMPTOM_TREMOR = "TREMOR";
const char* SYMPTOM_DYSKINESIA = "DYSKINESIA";
const char* SYMPTOM_FOG = "FOG";

#define MAX_SYMPTOM_LEN 12
static uint8_t symptom_string[MAX_SYMPTOM_LEN];

// Single characteristic pointer for symptom string
static GattCharacteristic *symptomCharPtr = nullptr;

// Manage BLE reconnection
class MyGapEventHandler : public ble::Gap::EventHandler {
public:
    void onConnectionComplete(const ble::ConnectionCompleteEvent &event) override {
        (void)event;
        printf("BLE connected\r\n");
    }

    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) override {
        (void)event;
        printf("BLE disconnected, restarting advertising\r\n");
        // Restart advertising on disconnection
        ble::BLE::Instance().gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
    }
};

static MyGapEventHandler gap_event_handler;

// Forward declarations for BLE callbacks
void schedule_ble_events(ble::BLE::OnEventsToProcessCallbackContext*);
void on_ble_init_complete(ble::BLE::InitializationCompleteCallbackContext*);

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
const float GYRO_SENSITIVITY = 8.75f; // mdps/LSB for ±250 dps
const float FREQ_BIN_SIZE = SAMPLE_RATE / FFT_SIZE; // Derived Resolution: 52 / 128 = 0.40625 Hz per bin
const float WALK_MIN_FREQ = 1.0f; // walking band lower bound
const float WALK_MAX_FREQ = 3.0f; // walking band upper bound
const float LOW_FREQ_TH   = 1.0f; // not walking
const float BIG_DROP_TH   = -1.0f; // big decrease
const float BIG_CHANGE_TH =  0.5f; // noticeable change
volatile float current_dominant_freq = 0.0f; // Holds calculated frequency for printing
volatile float current_fft_magnitude = 0.0f; // ADDED: Holds FFT magnitude for intensity
volatile float prev_dominant_freq = 0.0f; // Holds frequency captured for processing
volatile float current_gyro_avg = 0.0f;  // Average gyro magnitude over window for tremor confirmation

// FFT Buffers
float fft_input[FFT_SIZE];
float fft_output[FFT_SIZE];
float fft_mag[FFT_SIZE / 2];
int fft_idx = 0;

// ---- Step detection variables ----
volatile int step_count = 0;          // Total steps
volatile int steps_in_window = 0;     // Steps in current 3-second window
float acc_vertical_prev = 1.0f;       // Previous vertical acceleration
float acc_vertical_avg = 1.0f;        // Running average for baseline
bool step_detected = false;           // Flag for step state machine
int step_cooldown = 0;                // Cooldown counter between steps

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
MovementState determine_movement_state( float frequency, int steps );
void detect_step( float acc_z );  // ADDED
uint8_t calculate_intensity( float magnitude );  // ADDED

// UI helpers
void update_led_pattern();
void handle_button();

int main(){

    // Configure the device 
    if( !configure_device() ){

        printf("Device configuration failed!\r\n");
        while(1);

    }

    // Initialize BLE
    // start event queue in a background thread so BLE events are processed
    static Thread ble_event_thread;
    ble_event_thread.start(callback(&event_queue, &events::EventQueue::dispatch_forever));
    ble_instance.onEventsToProcess(schedule_ble_events);
    ble_instance.init(on_ble_init_complete);

    // Initialize the FFT library
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    // Sample interval counter
    int sample_interval = 0;
    
    printf("=== Parkinson's Monitor Started ===\r\n");
    
    // Main loop
    while (1){

        // Read raw accelerometer values, convert to g
        float acc_x_g = ( read_16bit_value(OUTX_L_XL, OUTX_H_XL) * ACC_SENSITIVITY / 1000.0f );
        float acc_y_g = ( read_16bit_value(OUTY_L_XL, OUTY_H_XL) * ACC_SENSITIVITY / 1000.0f );
        float acc_z_g = ( read_16bit_value(OUTZ_L_XL, OUTZ_H_XL) * ACC_SENSITIVITY / 1000.0f );
        
        // ADDED: Read gyroscope for enhanced detection
        float gyro_x_dps = read_16bit_value(OUTX_L_G, OUTX_H_G) * GYRO_SENSITIVITY / 1000.0f;
        float gyro_y_dps = read_16bit_value(OUTY_L_G, OUTY_H_G) * GYRO_SENSITIVITY / 1000.0f;
        float gyro_z_dps = read_16bit_value(OUTZ_L_G, OUTZ_H_G) * GYRO_SENSITIVITY / 1000.0f;
        
        // Calculate magnitudes
        float acc_z_nograv = acc_z_g - 1.0f; // Remove gravity for FFT
        float acc_magnitude = sqrtf( acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_nograv * acc_z_nograv );
        float gyro_magnitude = sqrtf( gyro_x_dps * gyro_x_dps + gyro_y_dps * gyro_y_dps + gyro_z_dps * gyro_z_dps );
        
        // Track gyro average for tremor/dyskinesia confirmation
        static float gyro_sum = 0.0f;
        static int gyro_count = 0;
        gyro_sum += gyro_magnitude;
        gyro_count++;
        
        if (gyro_count >= (int)(3.0f * SAMPLE_RATE)) {
            current_gyro_avg = gyro_sum / gyro_count;
            gyro_sum = 0.0f;
            gyro_count = 0;
        }
        
        // Detect steps using vertical acceleration 
        float acc_total_mag = sqrtf( acc_x_g * acc_x_g + acc_y_g * acc_y_g + acc_z_g * acc_z_g );
        detect_step( acc_total_mag );

        // Process accelerometer magnitude through FFT (accel only for clean walking/FOG detection)
        fft( acc_magnitude );

        // Sample accelerometer dominant frequency every ~3 seconds
        if( sample_interval++ >= (int)(3.0f * SAMPLE_RATE) ){

            // Reset sample interval
            sample_interval = 0;
        
            // Determine oscillation state
            OscillationState state = determine_oscillation_state( current_dominant_freq );
            
            // Calculate intensity from FFT magnitude
            uint8_t intensity = calculate_intensity( current_fft_magnitude );

            // Teleplot output for visualization
            printf(">frequency:%.2f\r\n", current_dominant_freq);
            printf(">intensity:%d\r\n", intensity);
            printf(">steps:%d\r\n", steps_in_window);
            printf(">gyro_avg:%.2f\r\n", current_gyro_avg);  // For tuning tremor threshold
            printf(">osc_state:%d\r\n", (int)state);  // 0=NORMAL, 1=TREMOR, 2=DYSKINESIA, 3=OUT_OF_RANGE
            
            // Determine movement state - MODIFIED to use steps
            MovementState move_state = determine_movement_state( current_dominant_freq, steps_in_window );
            printf(">move_state:%d\r\n", (int)move_state);  // 0=IDLE, 1=WALKING, 2=FREEZED
            
            // Also print human-readable for debugging
            if( state == NORMAL ) printf("NORMAL (%.2f Hz)\r\n", current_dominant_freq);
            else if( state == TREMOR ) printf("TREMOR (%.2f Hz, intensity: %d)\r\n", current_dominant_freq, intensity);
            else if( state == DYSKINESIA ) printf("DYSKINESIA (%.2f Hz, intensity: %d)\r\n", current_dominant_freq, intensity);
            else printf("OUT OF RANGE (%.2f Hz)\r\n", current_dominant_freq);

            if( move_state == IDLE ) printf("Movement State: IDLE (steps: %d)\r\n", steps_in_window);
            else if( move_state == WALKING ) printf("Movement State: WALKING (steps: %d)\r\n", steps_in_window);
            else if( move_state == FREEZED ) printf("Movement State: FREEZED\r\n");
            
            // Reset steps for next window
            steps_in_window = 0;
            
            // Update BLE characteristic with symptom string (only if not NONE)
            if (ble_ready && symptomCharPtr) {
                const char* symptom_to_send = nullptr;
                
                // Priority: FOG > DYSKINESIA > TREMOR (skip NONE)
                if (move_state == FREEZED) {
                    symptom_to_send = SYMPTOM_FOG;
                } else if (state == DYSKINESIA) {
                    symptom_to_send = SYMPTOM_DYSKINESIA;
                } else if (state == TREMOR) {
                    symptom_to_send = SYMPTOM_TREMOR;
                }
                
                // Only send if we have a symptom to report
                if (symptom_to_send != nullptr) {
                    strcpy((char*)symptom_string, symptom_to_send);
                    ble_instance.gattServer().write(
                        symptomCharPtr->getValueHandle(),
                        symptom_string,
                        strlen(symptom_to_send) + 1
                    );
                    printf("BLE Sent: %s\r\n", symptom_to_send);
                }
            }

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
    uint8_t low_byte = read_register(low_reg);
    
    // Read high byte
    uint8_t high_byte = read_register(high_reg);
    
    // Combine the bytes (little-endian: low byte first)
    return (int16_t)((high_byte << 8) | low_byte);

}

// Configure the LSM6DSL device
bool configure_device(){

    // Setup I2C at 400kHz
    i2c.frequency(400000);
    
    // Verify device id   
    if( read_register(WHO_AM_I) != 0x6A ) return false;
    
    // Configure the accelerometer (52 Hz, ±2g range)
    write_register(CTRL1_XL, 0x30); // ODR = 52 Hz (0011), FS = ±2g (00)
    
    // Configure the gyroscope (52 Hz, ±250 dps range)
    write_register(CTRL2_G, 0x30); // ODR = 52 Hz (0011), FS = ±250 dps (00)

    return true; 

}

// Step detection using Z-axis acceleration threshold crossing
void detect_step( float acc_z ){
    
    // Update running average (slow adaptation)
    acc_vertical_avg = acc_vertical_avg * 0.99f + acc_z * 0.01f;
    
    // Decrease cooldown
    if( step_cooldown > 0 ) step_cooldown--;
    
    // Step detection: look for acceleration going above threshold then back down
    // A step causes a brief spike in vertical acceleration
    float threshold_high = acc_vertical_avg + 0.10f;  // Above baseline
    float threshold_low = acc_vertical_avg + 0.03f;   // Return to near baseline
    
    if( !step_detected && step_cooldown == 0 ){
        // Looking for rising edge (foot strike)
        if( acc_z > threshold_high && acc_vertical_prev <= threshold_high ){
            step_detected = true;
        }
    }
    else if( step_detected ){
        // Looking for falling edge (foot lift)
        if( acc_z < threshold_low ){
            step_detected = false;
            step_count++;
            steps_in_window++;
            step_cooldown = 15;  // ~290ms cooldown at 52Hz
        }
    }
    
    acc_vertical_prev = acc_z;
}

// Calculate intensity level (1-255) from FFT magnitude
uint8_t calculate_intensity( float magnitude ){
    
    if( magnitude < 0.3f ) return 1;  // Minimum intensity
    
    // Scale magnitude to 1-255 range
    // Typical tremor/dyskinesia magnitudes: 0.3 to 10
    float scaled = (magnitude - 0.3f) / 9.7f;  // Normalize to 0-1
    if( scaled > 1.0f ) scaled = 1.0f;
    
    return (uint8_t)(scaled * 254.0f) + 1;  // Map to 1-255
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
        if (max_val > 0.3f){
            current_dominant_freq = max_bin * FREQ_BIN_SIZE;
            current_fft_magnitude = max_val;  // ADDED: Store magnitude
        }
        else{
            current_dominant_freq = 0.0f;
            current_fft_magnitude = 0.0f;
        }
        
        // Reset index to start filling buffer again
        fft_idx = 0;

    }

}

// Determine Oscillation State based on Frequency + Gyro Confirmation
OscillationState determine_oscillation_state( float frequency ){

    // Gyro threshold - tremor/dyskinesia typically > 15 dps average rotation
    // This helps distinguish real tremor from walking arm swing
    bool has_rotation = (current_gyro_avg > 15.0f);

    // Define thresholds (in Hz) - require rotation for tremor/dyskinesia
    if( frequency >= 3.0f && frequency <= 5.0f && has_rotation ) return TREMOR;
    else if( frequency > 5.0f && frequency <= 7.0f && has_rotation ) return DYSKINESIA;
    else if ( frequency > 7.0f) return OUT_OF_RANGE;
    else return NORMAL;

}

// Determine Movement State based on Frequency AND Steps
MovementState determine_movement_state( float frequency, int steps ){

    // Initialize initial state
    static MovementState current_state = IDLE;

    // Initialize initial previous frequency
    static float prev_dominant_freq_local = 0.0f;
    
    // Track consecutive walking windows for FOG detection
    static int walking_window_count = 0;

    // ---- handle reset request from long button press ----
    if (g_reset_flag) {
        current_state = IDLE;
        prev_dominant_freq_local = 0.0f;
        walking_window_count = 0;
        step_count = 0;
        g_reset_flag = false;
        return current_state;
    }

    // Calculate frequency change 
    float delta = frequency - prev_dominant_freq_local;
    
    // Use steps to help determine walking
    bool has_steps = (steps >= 1);
    bool has_motion = (current_fft_magnitude > 0.5f);
    bool in_walk_freq = (frequency >= WALK_MIN_FREQ && frequency <= WALK_MAX_FREQ);

    switch (current_state) {

        case IDLE:

            // IDLE -> WALKING if steps detected OR frequency in walking band with change
            if( has_steps || (in_walk_freq && has_motion) ){
                current_state = WALKING;
                walking_window_count = 1;
            }
            else{
                current_state = IDLE;
            }
            break;

        case WALKING:

            walking_window_count++;
            
            // WALKING -> FREEZED: Need sustained walking first, then sudden stop
            // FOG requires: walking for at least 2 windows (~6 sec), then sudden frequency drop with no steps
            if( walking_window_count >= 2 && frequency < LOW_FREQ_TH && !has_steps && delta < BIG_DROP_TH ){
                current_state = FREEZED;
                walking_window_count = 0;
            }
            // WALKING -> IDLE: gradual stop (low freq, no steps, small change)
            else if( frequency < LOW_FREQ_TH && !has_steps && fabsf(delta) <= BIG_CHANGE_TH ){
                current_state = IDLE;
                walking_window_count = 0;
            }
            // Stay WALKING if still have steps or in walking frequency band
            else if( has_steps || in_walk_freq ){
                current_state = WALKING;
            }
            // Otherwise transition to IDLE
            else{
                current_state = IDLE;
                walking_window_count = 0;
            }
            break;

        case FREEZED:

            // FREEZED -> WALKING: resume walking (steps or walking frequency)
            if( has_steps || (in_walk_freq && fabsf(delta) > BIG_CHANGE_TH) ){
                current_state = WALKING;
                walking_window_count = 1;
            }
            // FREEZED -> IDLE: stays low without big changes for extended time
            else if (frequency < LOW_FREQ_TH && fabsf(delta) <= BIG_CHANGE_TH){
                static int freeze_duration = 0;
                freeze_duration++;
                if( freeze_duration >= 3 ){  // ~9 seconds in freeze
                    current_state = IDLE;
                    freeze_duration = 0;
                }
            }
            // else remain FREEZED
            else{
                current_state = FREEZED;
            }
            break;

        default:

            current_state = IDLE; // safety fallback
            walking_window_count = 0;
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
//  - short press: show step count
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
        else if (duration_ms >= 50) {
            // Short press - show current status
            printf("[BUTTON] Total steps: %d, Current symptom: %s\r\n", 
                   step_count, (char*)symptom_string);
        }

        // Reset counter after release
        press_ticks = 0;
    }

    prev_pressed = pressed;
}

// Schedule BLE events on the event queue
void schedule_ble_events(ble::BLE::OnEventsToProcessCallbackContext* context) {
    event_queue.call(callback(&ble::BLE::Instance(), &ble::BLE::processEvents));
}

// BLE initialization complete - create GATT service and start advertising
void on_ble_init_complete(ble::BLE::InitializationCompleteCallbackContext* params) {

    if (params->error) {
        printf("BLE initialization failed\r\n");
        return;
    }

    ble::BLE &local_ble = params->ble;

    // Create ONE string characteristic for symptom name
    strcpy((char*)symptom_string, SYMPTOM_NONE);  // Initialize to "NONE"
    symptomCharPtr = new GattCharacteristic(
        UUID("0000a001-0000-1000-8000-00805f9b34fb"),
        symptom_string, 1, MAX_SYMPTOM_LEN,
        GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ
    );

    GattCharacteristic *char_table[] = { symptomCharPtr };

    // Custom service UUID
    const UUID service_uuid("0000a000-0000-1000-8000-00805f9b34fb");
    GattService custom_service(service_uuid, char_table, sizeof(char_table) / sizeof(char_table[0]));

    local_ble.gattServer().addService(custom_service);

    // Build advertising payload using AdvertisingDataBuilder
    Gap &gap = local_ble.gap();
    // handle disconnnection and restart advertising
    gap.setEventHandler(&gap_event_handler);
    const uint8_t service_uuid_bytes[16] = { 0xFB,0x34,0x9B,0x5F,0x80,0x00,0x00,0x80,0x00,0x10,0x00,0x00,0xA0,0x00,0x00,0x00 };
    uint8_t adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder adv_builder(adv_buffer);
    adv_builder.clear();
    adv_builder.setFlags();
    adv_builder.setName("Parkinsons Monitor");

    // Add 128-bit service UUID using mbed::Span
    mbed::Span<const unsigned char> svc_span(reinterpret_cast<const unsigned char*>(service_uuid_bytes), sizeof(service_uuid_bytes));
    adv_builder.addData(ble::adv_data_type_t::COMPLETE_LIST_128BIT_SERVICE_IDS, svc_span);

    // Set advertising parameters for legacy connectable undirected advertising
    gap.setAdvertisingParameters(ble::LEGACY_ADVERTISING_HANDLE, ble::AdvertisingParameters(ble::advertising_type_t::CONNECTABLE_UNDIRECTED));

    // Set payload and start advertising using the legacy advertising handle
    gap.setAdvertisingPayload(ble::LEGACY_ADVERTISING_HANDLE, adv_builder.getAdvertisingData());
    gap.startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);

    ble_ready = true;
    printf("BLE initialization SUCCESS!\r\n");

}