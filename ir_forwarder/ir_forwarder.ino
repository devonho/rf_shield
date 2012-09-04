/*
 * IR Remote Control signal forwarder
 *
 * Objective:
 *
 * This program forwards IR pulses (modulated on a 38 KHz carrier wave) from a
 * 3-pin IR receiver (such as a PNA4602, IRM-3638N3, or similar) with its data
 * pin connected to ICP1/PB0 (Arduino pin #8) to an IR LED (connected to
 * OC2A/PB3 (Arduino pin #11)). The forwarded IR pulses are also printed to the
 * Serial console as a series of pulse timings (in usecs).
 *
 * Mode of operation:
 *
 * The IR receiver filters out the 38 KHz carrier wave and pulls the ICP1 pin
 * low whenever there an IR pulse is detected (i.e. the 38 KHz carrier is
 * present), thus triggering Timer1's Input Capture logic and the corresponding
 * TIMER1_CAPT interrupt. The TIMER1_CAPT ISR first turns on/off the carrier
 * wave hooked up to the IR LED (see below) so as to replicate the IR pulse
 * there, and then records the length of the last IR pulse.
 *
 * The IR LED is fed by a 38 KHz carrier wave that is modulated by calling the
 * IR_toggle() macro. The carrier wave is generated by running Timer/Counter2
 * in CTC mode with the value of OCR2A register chosen such that the timer is
 * automatically reset twice in each 38 KHz cycle. The COM2A1:0 bits then
 * determine whether the OC2A output (to which the IR LED is connected) is
 * either toggled (producing the 38 KHz carrier) - or cleared (producing no
 * carrier) on each timer reset. Thus, the generation of the 38 KHz carrier
 * wave itself does not consume any CPU time; only modulating it (by calling
 * IR_toggle() from the TIMER1_CAPT ISR consumes CPU time.
 *
 * The only remaining part of the program is printing the IR pulse timings to
 * the serial port. The IR timings are recorded by the TIMER1_CAPT ISR into a
 * simple ring buffer. The main loop of the program then monitors this ring
 * buffer, and prints the recorded timings as ASCII decimal integers (unit is
 * usec) to the serial port (@115200 baud).
 *
 * Physical considerations:
 *
 * The point of an IR forwarder is to make an IR signal available where is
 * normally would not reach. Hence, the IR receiver should be located where
 * the IR signal is available, and the IR LED where the signal is not (yet)
 * available. More importantly, the IR receiver must not be able to detect
 * the IR pulses emitted by the IR LED, as that will mess things up.
 * This means that there will probably be considerably physical distance
 * between the IR receiver and the IR LED. In this case, it is recommended to
 * place the Arduino as close to the IR receiver as possible, and then run a
 * cable (which type?) to the IR LED. If you need to amplify the IR LED signal
 * (e.g. to boost the range of the forwarded signal), then use a simple
 * transistor circuit, such as https://www.sparkfun.com/products/10732 .
 *
 * Author: Johan Herland <johan@herland.net>
 * License: GNU GPL v2 or later
 */

volatile unsigned short ring_buffer[256] = { 0 };
volatile byte rb_write; // current write position in ring buffer
byte rb_read; // current read position in ringbuffer

void setup(void)
{
	Serial.begin(115200);

	cli(); // Disable interrupts while setting up

	/*
	 * 38 KHz carrier wave for IR output LED
	 *
	 * Set up Timer/Counter2 in CTC mode (with TOP == OCR2A), and make it
	 * trigger at a rate of 2 x 38 KHz, toggling OC2A/PORTB3/pin11 at
	 * every trigger to produce a 38 KHz (50% duty cycle) square wave.
	 * Modulate this carrier wave by switching between toggling
	 * (COM2A1:0 == 0b01) and clearing (COM1A1:0 == 0b10) behavior (see the
	 * IR_on()/IR_off() macros below).
	 */

	// OC2A/PORTB3/pin11 in output mode
	DDRB |= _BV(DDB3);

	// Timer2:
	//  - COM2A1:0 = 0b10 (Clear OC2A on Compare Match)
	//  - WGM22:0 = 0b010 (CTC mode, reset on OCR2A Match)
	//  - CS22:0 = 0b001 (no prescaling)
	// No timer-related interrupts
	TCCR2A = B10000010; // COM2A1 COM2A0 COM2B1 COM2B0 - - WGM21 WGM20
	TCCR2B = B00000001; // FOC2A FOC2B - - WGM22 CS22 CS21 CS20
	TIMSK2 = B00000000; // - - - - - OCIE2B OCIE2A TOIE2

	// Output Compare Register 2 A:
	//  - OCR2A = 209 (match every ~13.15 usec (~= 2 x 38 KHz)
	// Reset Timer/Counter2
	OCR2A = 209;
	TCNT2 = 0x00;

	// Reset interrupt flags (by writing a logical 1 into them)
	TIFR2 = B00000111; // - - - - - OCF2B OCF2A TOV2

	/*
	 * Capture changes from IR receiver
	 *
	 * Set up Timer/Counter1 in normal mode with a prescaler chosen so that
	 * we (a) get useful precision on timings (on the order of a few usecs)
	 * and also (b) get no more than a single overflow between two
	 * successive timings in a single IR command (We don't care about
	 * multiple overflows between two IR commands, as the inter-command
	 * time is uninteresting).
	 *
	 * A 64 x prescaler gives us a resolution of (64 * 1/16 MHz =) 4 usec,
	 * and an overflow every (4 usec * 2^16 =) ~262 msec. That should be
	 * enough to fairly accurately record IR remote control commands with
	 * shortest pulse at ~500 usecs, and longest pulses < ~50 msecs.
	 *
	 * Additionally, we set up the incoming IR pulses to be captured and
	 * timestamped in ICR1, and raise the TIMER1_CAPT interrupt, so that
	 * the ICR1 timings can be copied into a ring_buffer that communicates
	 * the timings to the program's main loop.
	 */

	//  ICP1/PORTB0/pin8 in input mode
	DDRB &= ~_BV(DDB0);

	//  - Timer1:
	//     - WGM13:0 = 0b0000 (normal mode)
	//     - CS12:0 = 0b011 (64 x prescaling)
	//     - TOIE1 = 1 (enable overflow interrupt)
	//  - Input capture unit:
	//     - ICNC1 = 1 (enable noise canceling)
	//     - ICES1 = 0 (falling edge detect - pin is pulled LOW on IR pulse)
	//     - ICIE1 = 1 (enable interrupt on input capture)
	TCCR1A = B00000000; // COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
	TCCR1B = B10000011; // ICNC1 ICES1 - WGM13 WGM12 CS12 CS11 CS10
	TCCR1C = B00000000; // FOC1A FOC1B - - - - - -
	TIMSK1 = B00100001; // - - ICIE1 - - OCIE1B OCIE1A TOIE1

	//  - Reset Timer/Counter1 and Input Capture Register 1
	TCNT1 = 0x0000;
	ICR1 = 0x0000;

	//  - Reset interrupt flags (by writing a logical 1 into them)
	TIFR1 = B00100001; // - - ICF1 - - OCF1B OCF1A TOV1

	sei(); // Re-enable interrupts

	Serial.println(F("Ready"));
}

/*
 * Modulate the IR output carrier by switching between
 *  - COM2A1:0 == 0b01 (Toggle OC2A on compare match, i.e. carrier present)
 *  - COM2A1:0 == 0b10 (Clear OC2A on compare match, i.e. carrier absent)
 * The other bits in TCCR2A must not be changed from their initialized value.
 */
#define IR_toggle()  (TCCR2A ^= (_BV(COM2A1) | _BV(COM2A0)))

// Timer1 Capture Event interrupt handler. Triggered on every incoming IR pulse
// event, typically no more often than every ~500 usecs.
ISR(TIMER1_CAPT_vect)
{
	// Read ICR1 register
	unsigned short t = ICR1;
	// Change trigger edge
	TCCR1B ^= _BV(ICES1);
	// Replicate pulse change on IR output LED
	IR_toggle();
	// The timestamp value 0 is reserved for Timer1 overflows
	if (t == 0)
		t--; // Wrap back by one timer count (4 usec)
	// Store pulse change timestamp in ring buffer
	ring_buffer[rb_write++] = t;
}

// Timer1 Overflow interrupt handler. With 64 x prescaler and 16 MHz system
// clock this is triggered every ~262 msecs.
ISR(TIMER1_OVF_vect)
{
	// Store 0 value into ring_buffer
	ring_buffer[rb_write++] = 0;
}

void loop(void)
{
	if (rb_read == rb_write)
		return; // Nothing has been added to the ring buffer

	// Process the next pulse in the ring buffer
	unsigned short start = ring_buffer[rb_read - 1];
	unsigned short end = ring_buffer[rb_read];
	unsigned long usecs = 0; // Calculate #usecs in this pulse

	if (end) { // pulse has ended (i.e. end is not a overflow sentinel)
		if (!start) { // an overflow occured during this pulse
			// find the real start of the pulse in preceding slot
			start == ring_buffer[rb_read - 2];
			usecs += 65536l;
			/*
			 * If start is still == 0, we know that at least TWO
			 * timer overflows occured during this pulse, meaning
			 * that the pulse must be > ~262 msecs. That is not a
			 * pulse, but rather a pause between IR commands.
			 * Hence, we report the start of a new IR command
			 * instead of an insane pulse length.
			 */
			if (!start) // ALSO an overflow sentinel
				Serial.println("---"); // Between IR commands
		}

		if (start) {
			// We have a definite start and end time. Calculate
			// pulse length in usecs (using 4 usecs per timer
			// count), and output to serial port.
			usecs += end;
			if (start > usecs) // Should not happen
				Serial.println("***");
			usecs -= start;
			usecs *= 4;
			Serial.println(usecs, DEC);
		}
	}

	rb_read++;
}
