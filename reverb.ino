/*
 * Boite à beep / echo
 */

#include "koin.h"

#define audio_in 0
#define audio_out 9
#define ptt_in A3
#define toggle_echo A2
#define ptt_out 6
#define callsign A2
#define F_SAMP_PWM (78125/1)
#define F_SAMP_AUDIO 16000

typedef void (*func_t)(void);
volatile func_t src_ptr;

/* Buffer for sample effects */
const unsigned int buff_size = 1700;
unsigned int buff_pos = 0;
char buff[buff_size] = { NULL };

enum play_mode { NONE, MIC, ROGER };
uint8_t samp_mode = NONE;
uint8_t dc = 126; // DC offset remove
uint8_t play = 0;
bool echo = false;
unsigned int i=0;

int8_t get_sample_from_mic() {
  ADCSRA |= (1 << ADSC);
  int8_t sample = ADCH - 127;
  sample = -sample; // Avoid loopback
  return sample;
}

int8_t get_sample_from_mem() {
  int8_t sample;
  sample = pgm_read_byte(&koin[i]) - 127;
  i = i+1;
  if (i == koin_length) {
    i = 0;
    play = play - 1;
  }
  return sample;
}

int8_t sample_echo(int8_t sample) {
  int8_t old_sample = buff[buff_pos];
  int8_t new_sample = sample + old_sample / 3;
  buff[buff_pos] = new_sample;
  buff_pos = buff_pos + 1;
  if (buff_pos == buff_size) buff_pos = 0;
  
  return new_sample;
}

void output_sample(int8_t sample) {
  OCR1AL = sample + 128;
 // analogWrite(5, sample+128);
}

void process_audio()
{
  int8_t sample;
  switch (samp_mode) {
    case NONE:
      sample = 0;
      break;
    case MIC:
      sample = get_sample_from_mic();
      break;
    case ROGER:
      sample = get_sample_from_mem();
      break;
  }

  if (echo) sample = sample_echo(sample);
  output_sample(sample);
  
}

void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs) {
  DIDR0 |= (1 << adcpin); // disable digital input 
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;              // clear ADMUX register
  ADMUX |= (adcpin & 0x0f);    // set analog input pin
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);  // If reflvl == true, set AREF=1.1V (Internal ref); otherwise AREF=AVCC=(5V)
  ADMUX |= (1 << ADLAR);
  ADCSRA |= 0x06;
  ADCSRA |= (1 << ADEN);  // enable ADC
}

void timer1_start(uint32_t fs)  //PWM
{  // Timer 1: OC1A and OC1B in PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  TCCR1B |= (1 << CS10) | (1 << WGM13) | (1 << WGM12); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;
  ICR1L = min(255, F_CPU / fs);  // PWM value range (fs>78431):  Fpwm = F_CPU / [Prescaler * (1 + TOP)]
  OCR1A = 0;
  OCR1B = 0;
}

void timer2_start(uint32_t fs) //Audio
{  // Timer 2: interrupt mode
  ASSR &= ~(1 << AS2);  // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21); // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
  TCCR2B |= (1 << CS22);  // Set C22 bits for 64 prescaler
  TIMSK2 |= (1 << OCIE2A);  // enable timer compare interrupt TIMER2_COMPA_vect
  uint8_t ocr = ((F_CPU / 64) / fs) - 1;   // OCRn = (F_CPU / pre-scaler / fs) - 1;
  OCR2A = ocr;
}

// Audio sampling interrupt
ISR(TIMER2_COMPA_vect) {
  process_audio();  
}

void setup()
{
  pinMode(ptt_in, INPUT_PULLUP);
  pinMode(toggle_echo, INPUT_PULLUP);
  pinMode(audio_out, OUTPUT);
  pinMode(ptt_out, OUTPUT);
  digitalWrite(ptt_out, LOW);
  adc_start(audio_in, true, F_SAMP_AUDIO);
  // Start timers to begin audio sample and rendering
  timer1_start(F_SAMP_PWM);
  timer2_start(F_SAMP_AUDIO);
  Serial.begin(230400);

 // TCCR0B = TCCR0B & 0b11111001;
  
  samp_mode = NONE;
  
}

void loop()
{
  if (digitalRead(ptt_in) == LOW) {
    if (samp_mode == NONE) {
      samp_mode = MIC;
      digitalWrite(ptt_out, HIGH);
      while (digitalRead(ptt_in) == LOW);
    }
  } else if (samp_mode == MIC) {
    play = 2;
    samp_mode = ROGER;
  }

  if (samp_mode == ROGER &&  play == 0) {
    samp_mode = NONE;
    digitalWrite(ptt_out, LOW);
  }

  if (digitalRead(toggle_echo) == LOW) {
    while (digitalRead(toggle_echo) == LOW);
    echo = !echo;
    delay(10);
  }
  
}
