/* 
 * TODO:
 * 1. Include a low-pass filter to sense general touch input
 * 2. Determine if the baseline time constants have changed after the sensor has been touched
 * 
 */

/*
 *                       Teensy 2.0 (Atmega 32U4)
 *                        _______
 *                    ___|       |___
 *               Gnd | o |       | o | Vcc
 * (interior) SS PB0 | o |_______| o | PF0 ADC0       (interior)
 *  PE6     SCLK PB1 | o           o | PF1 ADC1          AREF
 *  AIN0    MOSI PB2 | o           o | PF4 ADC4
 *  INT6    MISO PB3 | o o       o o | PF5 ADC5
 * RTS OC1C OC0A PB7 | o   _____   o | PF6 ADC6
 * OC0B SCL INT0 PD0 | o  |     |  o | PF7 ADC7
 *      SDA INT1 PD1 | o  |     |  o | PB6 ADC13 OC1B OC4B
 *     RXD1 INT2 PD2 | o  |_____|  o | PB5 ADC12 OC1A OC4B
 *     TXD1 INT3 PD3 | o           o | PB4 ADC11
 *    !OC4A OC3A PC6 | o    [   ]  o | PD7 ADC10 T0 OC4D
 *     OC4A ICP3 PC7 | o o o o o o o | PD6 ADC9  T1 !OC4D (LED on PD6)
 *                   |___|_|_|_|_|___|
 *                       | | | | |
 *            CTS XCK1 PD5 | | | PD4 ADC8 ICP1
 *                       Vcc | RST
 *                          GND
 * 
 *                        _______
 *                    ___|       |___
 *               Gnd | o |       | o | Vcc
 * (interior)      0 | o |_______| o | 21 A0        (interior)
 *     24          1 | o           o | 20 A1           AREF
 *                 2 | o           o | 19 A2
 *                 3 | o o       o o | 18 A3
 *        PWM      4 | o   _____   o | 17 A4
 *        PWM INT0 5 | o  |     |  o | 16 A5
 *            INT1 6 | o  |     |  o | 15 A6  PWM
 *         RX INT2 7 | o  |_____|  o | 14 A7  PWM
 *         TX INT3 8 | o           o | 13 A8
 *        PWM      9 | o    [   ]  o | 12 A9  PWM
 *        PWM     10 | o o o o o o o | 11 A10 (LED on 11)
 *                   |___|_|_|_|_|___|
 *                       | | | | |
 *                      23 | | | 22, A11
 *                       Vcc | RST
 *                          GND
 */


/*
 *  Clock Divisor Table
 * 0 | 0 | 0 No clock source
 * 0 | 0 | 1 clk/1
 * 0 | 1 | 0 clk/8
 * 0 | 1 | 1 clk/64
 * 1 | 0 | 0 clk/256
 * 1 | 0 | 1 clk/1024
 * 1 | 1 | 0 
 * 1 | 1 | 1 
 * 
 */


/*
 * Cyclical Buffer Operation
 * 
 * We define a buffer with n elements numbered from 0 to n-1.
 * The buffer elements contain readings taken sequentially.
 * We define an index to track which element is being written/overwritten.
 * We also define a buffer sum that keeps track of the sum of all elements within the buffer.
 * When a reading is taken, the value is placed in an element and the write index is incremented.
 * The removed element is subtracted from the buffer sum and the reading is added.
 * When the index reaches the end of the buffer, the index wraps around to zero.
 * 
 *      buffer, index = x          (n-4) (n-2)
 *  0  1  2  3  4  5  6     x   (n-5) (n-3) (n-1)
 * [#][#][#][#][#][#][#]...[#]...[#][#][#][#][#]
 * 
 * 
 * buffer_sum = buffer_sum + reading
 * buffer_sum = buffer_sum - buffer[x]
 * buffer[x] = reading;
 * 
 * Caveats:
 * 1. Alays add to the buffer sum before subtracting, especially if the buffer sum is an unsigned quantity.
 * 2. Avoid adding and subtracting quantities in the same line to avoid order of operations conflicts and variable wrap-around.
 * 
 */

// Length of the cyclical buffer used for the low-pass filter reading
// Higher values produce a "smoother" output but slow responsiveness to changes in touch
#define CYC_BUFF_LEN 128 

#define MHz_TO_MICROS 16 // Scaling clock ticks in MHz to microseconds
#define DT_TO_DELAY 1000 // 1000 Microseconds per 1 Hz
#define SAMPLE_RATE 100  // Hz, approximate

// A (red) and B (blue) electrode LED pins
#define REDLED 9    // Pin 9
#define BLUELED 10  // Pin 10

float dt = 1.0/SAMPLE_RATE;     // Time change

volatile uint8_t state = 0;     // State of timer interrupt (intital condition)

uint8_t idx = 0;                // Index of the cyclical buffer
uint32_t buffer_overflow = 0;   // Number of times the cyclical buffer has overflowed

// Wait "n" number of milliseconds to begin analyzing readings
// This gives the buffer time to flush dummy (zero) readings and populate with real data
uint32_t start_after = 2000;

uint16_t cyc_buffer[CYC_BUFF_LEN][2] = { { 0, }, { 0, } }; // Definition of the cyclical buffer
uint32_t cyc_buffer_sum[2] = { 0, };                       // Definition of the buffer sum

uint16_t tau_A = 0; // Signal A rise time
uint16_t tau_B = 0; // Signal B rise time

// A and B electrode signal reading
float sigA = 0.0;
float sigB = 0.0;

// The A and B scaled readings
float sigAScaled = 1.0;
float sigBScaled = 1.0;

// The brightness values of the red and blue LEDs
uint8_t ledA;
uint8_t ledB;

// The minimum and maximum LED brightness values
uint8_t minLEDBrightness[] = { 0, 0 };
uint8_t maxLEDBrightness[] = { 127, 127 };

// The minimum and maximum A and B signal values
float minSignalValue[] = { 1.0, 1.0 };
float maxSignalValue[] = { 60.0, 60.0 };

// The time to wait before returning the initial values
uint32_t delay_time = (uint32_t)(DT_TO_DELAY*dt);

// Observed baseline time constant (microseconds)
float baseA = 18.0;
float baseB = 18.0;


// Serial.print delimiter
String delim = " ";

void setup() {
    // Start the serial port
    Serial.begin(115200);
    
    // Enable the built-in LED
    // The LED will remain on to signify the running program
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Set the PWM-enabled LED pins
    // The colored LEDs will show the output from each electrode
    pinMode(REDLED, ledA);
    pinMode(BLUELED, ledB);
    
    TCCR1A = 0; // Clear initial settings (use 16 bit timing)
    TCCR1B = (1 << WGM12) | (1 << CS10); // Clear timer on Compare and set prescaler to 1
    
    // Output Compare Register reset ticks
    OCR1A = 8191; // 8191 clock ticks, about 1 ms
    OCR1B = 5000;
    
    TCNT1 = 0; // Reset timer counter 1
    
    TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // (1 << TOIE1) | Enable TIMER1 overflow interrupt and output compare interrupt
        
    DDRD &= ~((1 << DDD0) | (1 << DDD1)); // Set input on pins 5 and 6 (Signal in)
    DDRB |= (1 << DDB6) | (1 << DDB5);    // Set output on pins 14 and 15 (Signal out)
    DDRD |= (1 << DDD2) | (1 << DDD3); // Set output on pins 7 and 8 (ISR out)
    DDRF |= (1 << DDF0); // Set output on pin 21 (Trigger out)
    
    EICRA |= (1 << ISC10) | (1 << ISC00); // Fire interrupts on a change in external input
    //EICRA |= (1 << ISC10) | (1 << ISC00) | (1 << ISC11) | (1 << ISC01); // Fire interrupts on a rising edge external input
    EIMSK |= (1 << INT1) | (1 << INT0); // Enable external interrupts
    
    sei(); // Set enable interrupts
    
    // Wait for a specified time to fill and cycle the data buffer
    delay(start_after);
    
    // Turn on the built-in LED
    digitalWrite(LED_BUILTIN, HIGH);
}

/*
 * TODO:
 * self-calibration
 * 
 * The sensor subtracts an offset reading from the A and B signals when there is no touch.
 * The sensor is allowed to adjust the baseline offset when
 * 
 * Map the exponential filter alpha between 0 and 1
 * 
 */

void loop() {
    // Delay the loop
    delay(delay_time);
    
    // Get the windowed moving average
    sigA = (float)cyc_buffer_sum[0]/(MHz_TO_MICROS*CYC_BUFF_LEN);
    sigB = (float)cyc_buffer_sum[1]/(MHz_TO_MICROS*CYC_BUFF_LEN);
        
    // Subtract the baseline from the scaled
    sigAScaled = sigA - baseA;
    sigBScaled = sigB - baseB;
    
    // Set the minimum value of the scaled signals to 1.0 if the value is less than 1.0
    sigAScaled = (sigAScaled < 1.0)? 1.0 : sigAScaled;
    sigBScaled = (sigBScaled < 1.0)? 1.0 : sigBScaled;
    
    // Map the LED output values 
    ledA = (uint8_t)map(sigAScaled, minSignalValue[0], maxSignalValue[0], minLEDBrightness[0], maxLEDBrightness[0]);
    ledB = (uint8_t)map(sigBScaled, minSignalValue[1], maxSignalValue[1], minLEDBrightness[1], maxLEDBrightness[1]);
    
    // Output the LED brightness
    analogWrite(REDLED, ledA);
    analogWrite(BLUELED, ledB);
    
    // Print the A and B raw signal values, the estimated touch location and estimated touch pressure
    Serial.print(sigA);
    Serial.print(delim);
    Serial.print(sigB);
    Serial.print(delim);
    Serial.print((log(sigAScaled) - log(sigBScaled)));
    Serial.print(delim);
    Serial.println((sigAScaled + sigBScaled)/2.0 - 1.0);
}

/*
 * External Interrupt INT0 (Digital pin 5)
 */
ISR(INT0_vect) {
    tau_A = TCNT1; // Record the signal A rise time
    PORTD |= (1 << DDD2);
}

/*
 * External Interrupt INT1 (Digital pin 6)
 */
ISR(INT1_vect) {
    tau_B = TCNT1; // Record the signal B rise time
    PORTD |= (1 << DDD3);
}

/*
 * TIMER1 Compare channel A vector
 */
ISR(TIMER1_COMPA_vect) {
    
    // Invert digital pins 9 and 10
    PORTB ^= (1 << DDB6) | (1 << DDB5);

    // Put the tau_A and tau_B readings into the cyclical buffer (add before subtracting)
    cyc_buffer_sum[0] += tau_A;
    cyc_buffer_sum[0] -= cyc_buffer[idx][0];
    cyc_buffer_sum[1] += tau_B;
    cyc_buffer_sum[1] -= cyc_buffer[idx][1];
    cyc_buffer[idx][0] = tau_A;
    cyc_buffer[idx][1] = tau_B;
    
    idx++; // Increment the buffer index and reset to zero if it reaches the end
    if (idx >= CYC_BUFF_LEN) {
        idx = 0;
        buffer_overflow++;
    }
}

/*
 * TIMER1 Compare channel B vector
 */
ISR(TIMER1_COMPB_vect) {
    // Do nothing
}
