/*
 * measure_wave.ino
 *
 * Objective: Measure the digital waveform arriving at ICP1/PB0 (Arduino pin 8)
 * as accurately as possible, and provide timings on the serial port.
 *
 * Method: Set up Timer1 without prescaling, and use its Input Capture Unit to
 * trigger timestamping of input changes with 16MHz precision. Wait until
 * enough events/timestamps have been recorded, and then output the recorded
 * timestamps on the serial port.
 *
 * The two ISRs used here have an (internal) runtime of ~52 and ~42 processor
 * cycles, indicating that incoming waveforms changing in < ~94 processor
 * cycles (i.e. < ~6 usecs, which corresponds to ~83 KHz with a 50% duty cycle)
 * will not be accurately measured because of missed interrupts.
 * Further experiments suggest that we're still missing occasional interrupts
 * for waveforms that change in < ~8.5 usecs. In other words, the results seem
 * to be accurate for frequencies below ~59 KHz and duty cycles near 50%.
 * Further away from 50% duty cycle, the frequency limit will be somewhat lower
 * E.g. for a ~38 KHz wave, duty cycles within the ~12% - ~88% range seem to be
 * fairly consistently and accurately detected.
 */

/*
 * Store 16-bit timestamps of input changes. The first timestamp is from the
 * first rising edge, the next is from the following falling edge, and so on.
 * Dedicate half the RAM (512 bytes) to hold samples, this leaves us with
 * room for 256 samples.
 */
volatile unsigned short timestamps[256];
volatile unsigned short syncstamp;
volatile unsigned short i = 0;
volatile bool finished = false; // Set to true when sampling phase finished
volatile bool syncfound = false; // Set to true when sampling phase finished
const unsigned short timeout_msecs = 5000; // Upper bound on sampling phase

void setup(void)
{
	Serial.begin(115200);
	Serial.println(F("=== Start ==="));
	Serial.flush();

	cli(); // Disable interrupts while setting up

	// Set up:
	//  - DDRB0 = 0 (PORTB0/ICP1 in input mode)
        PORTB = 0x00;
	DDRB &= ~B1; // DDRB7:0
	DDRB |= 0x20; // DDRB7:0 - LED Pin 13
	DDRB |= 0x10; // DDRB7:0 - Pin 12
        
	//  - Timer1:
	//     - WGM13:0 = 0b0000 (normal mode)
	//     - CS12:0 = 0b001 (no prescaling)
	//     - TOIE1 = 1 (enable overflow interrupt)
	//  - Input capture unit:
	//     - ICNC1 = 0 (disable noise canceled)
	//     - ICES1 = 1 (rising edge detect)
	//     - ICIE1 = 1 (enable interrupt on input capture)
	TCCR1A = B00000000; // COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
	TCCR1B = B00000010; // ICNC1 ICES1 - WGM13 WGM12 [CS12 CS11 CS10] - Trigger on falling edge, Clock Select = CLKio/8 
	TCCR1C = B00000000; // FOC1A FOC1B - - - - - -
	TIMSK1 = B00100000; // - - ICIE1 - - OCIE1B OCIE1A TOIE1, overflow interrupt disabled.

	//  - Reset Timer/Counter1 and Input Capture Register 1
	TCNT1 = 0x0000;
	ICR1 = 0x0000;

	//  - Reset interrupt flags (by writing a logical 1 into them)
	TIFR1 = B00100001; // - - ICF1 - - OCF1B OCF1A TOV1

	sei(); // Re-enable interrupts
}

void enableCapture(boolean en)
{
  cli(); // Disable interrupts while setting up  
  if(en)
  {
    //  - Reset Timer/Counter1 and Input Capture Register 1
    TIMSK1 = B00100000; // Enable interrupts    
  }
  else
  {
    TIMSK1 = B00000000; // Disable interrupts
  }
  TCNT1 = 0x0000;
  ICR1 = 0x0000;  
  TIFR1 = B00100001; // Kill pending interrupts (1 clears)  
  sei(); // Re-enable interrupts
}

// Timer1 Capture Event interrupt handler; triggers when ICF1 is set
// Measuring TCNT1 at the start and end of this suggests runtime == 52 cycles
ISR(TIMER1_CAPT_vect)
{
	// Store ICR1 register into timestamps array
	unsigned short t = ICR1;

        if(!syncfound)
        {
          if(TCCR1B & _BV(ICES1))
          {
            // Rising edge detected
            syncstamp = t;
            if(syncstamp > 20000)
            {
              syncfound = true;
              i=0;
            }
          }
          TCNT1 = 0x0000;
          ICR1 = 0x0000;
          TCCR1B ^= _BV(ICES1); // Toggle edge to trigger on
        }
        else
        {
  	  // Change trigger edge
          if(TCCR1B & _BV(ICES1))
          {
            // Rising edge detected
            TCNT1 = 0x0000;
  	    ICR1 = 0x0000;
          }
          else
          {
            // Falling edge detected
            timestamps[i++] = t;
          }
          TCCR1B ^= _BV(ICES1); // Toggle edge to trigger on
  
  	  if (i == 24) 
          {
            TIMSK1 = B00000000; // Disable interrupts
            TCNT1 = 0x0000;
            ICR1 = 0x0000;  
            TIFR1 = B00100001; // Kill pending interrupts (1 clears)  
            finished = true; // Signal completion
            i=0;
            syncfound=false;
            return;
  	  }
        }
}

void loop(void)
{
  //continuousTriggerAndCapTest();
  //singleTriggerAndCapTest();
  rfCapTest();
}

void rfCapTest(void)
{
  unsigned long pingpong[2];
  unsigned char k;
  while(true)
  {    
    if(finished)
    {
      k=++k%2;
      pingpong[k]=0;
      for(int j=0;j<24;j++)
      {
        pingpong[k] += (timestamps[j]>1000?1:0)<<j;
      }
      if(pingpong[0]==pingpong[1])
        Serial.println(pingpong[k],HEX);
      finished = false;
      enableCapture(true);     
    }  
  }

}

void continuousTriggerAndCapTest(void)
{
  while(true)
  {    
    digitalWrite(12, HIGH);   // Trigger 1527
    if(finished)
    {
      unsigned char bits[24];
      for(int j=0;j<24;j++)
      {
        bits[j]=timestamps[j]>1000?1:0;
      }
      Serial.print(F("Bits: "));
      for(int j=0;j<24;j++)
      {      
        Serial.print(bits[j]);
      }
      Serial.println(F(" "));

      finished = false;

      enableCapture(true);     
      
      digitalWrite(12, LOW);   // Trigger 1527
      digitalWrite(12, HIGH);   // Trigger 1527
    }  
  }
}

void singleTriggerAndCapTest(void)
{
	digitalWrite(12, HIGH);   // Trigger 1527
	delay(88);
	digitalWrite(12, LOW);    

	// Broadcast the received samples over the serial port
	Serial.println(F("Timestamps:"));
	Serial.println(F("--BEGIN--"));
        Serial.println(syncstamp, DEC);
        Serial.println(F("---"));
	for (unsigned char j = 0; j < 24; j++) {
		Serial.println(timestamps[j], DEC);
	}
	Serial.println(F("--END--"));
	Serial.println(F("Finished printing timestamps."));

	while(true){}
}
