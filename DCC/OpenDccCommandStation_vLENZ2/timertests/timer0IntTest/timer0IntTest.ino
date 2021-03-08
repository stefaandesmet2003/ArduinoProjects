//timer tests
#define MY_TICK_PERIOD 1000L
#define F_CPU 16000000L

unsigned long int micro_timerval, timerval;
void setup_test1()
{
  // timer 0
  // telt de timer hier tot 255 of tot OCR0A?
  // de timer genereert een interrupt als ie tot OCR0A
  // heeft geteld, en in de isr wordt OCR0A aangepast zodat ie exact 1 ms later opnieuw fired

  TCCR0A = (0<< COM0A1)   // 00 = normal port mode
         | (0<< COM0A0)
         | (0<< COM0B1)   // 00 = normal port mode
         | (0<< COM0B0)
         | (0<< WGM01)    // WGM = 000: normale mode
         | (0<< WGM00);
  TCCR0B = (0<< FOC0A)    // 0 = no forced compare match
         | (0<< FOC0B)    // 0 = no forced compare match
         | (0<< WGM02)    // 
         | (0<<CS02)      // CS = 000: stopped
         | (1<<CS01)      //      001 = run, 010 = div8, 011=div64, 100=div256, 101=div1024. 
         | (1<<CS00);     //      110 = ext, 111 = ext
  OCR0A =  F_CPU / 64L * MY_TICK_PERIOD / 1000000L;               // 8 Bit compare on A
  TCNT0=0x00;
  TIMSK0 |= (1<<OCIE0A);   // Timer0 Compare Int.
  
}

ISR(TIMER0_COMPA_vect)
  {
    PORTB ^= 1<<5;
    //OCR0A +=  F_CPU / 64L * MY_TICK_PERIOD / 1000000L;   
    micro_timerval++;
    if (micro_timerval == 5)
      {
        micro_timerval = 0;
        timerval++;
      }
  }

void setup() {
  // put your setup code here, to run once:
  timerval = 0;
  micro_timerval = 0;
  DDRB |= 1<<5;

  setup_test1();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
