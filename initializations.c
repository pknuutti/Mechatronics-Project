#include "can.h"
#include "resolvers.h"
#include "valves.h"

void initADC()
{

   /*
   * -------------------------------------------------------------------------------
   * ADC initialization
   * ADC Clock frequency: 125,000 kHz
   * ADC Voltage Reference: AVCC pin
   * ADC High Speed Mode: Off
   * --------------------------------------------------------------------------------
   */

   /*
   * Digital input buffers on ADC0: Off, ADC1: Off, ADC2: Off, ADC3: Off
   * ADC4: Off, ADC5: Off, ADC6: Off, ADC7: Off
   * When an analog signal is applied to the ADC7..0 pin and the digital input from
   * this pin is not needed, this bit should be written logic one to reduce power
   * consumption in the digital input buffer.
   */

   // Digital signal is only needed for ADC3 and ADC4 as the joystick buttons are connected to these two.
   DIDR0=(1<<ADC7D) | (1<<ADC6D) | (1<<ADC5D) | (0<<ADC4D) | (0<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D);

   // Set the reference voltage to AVCC.
	ADMUX |= (0<<REFS1) | (1<<REFS0);

   /* Set divisor value to 128. ADC sample rate will be 16,000,000/128 = 125,000 Hz
    * According to AT90CAN128 datasheet the sample rate should be between 50kHz - 200kHz
    * when using 10 bit accuracy. The sample rate can be higher when using only 8 bits.
    */
   ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

   /* Enable A/D converter */
   ADCSRA |= (1<<ADEN);

}

int16_t readADCChannel(int8_t channelNumber)
{
   ADMUX = (0<<REFS1) | (1<<REFS0) | (1<<ADLAR) | (0<<MUX4) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0);
	if (channelNumber == 0)
		ADMUX |= (0<<MUX4) | (0<<MUX3) | (0<<MUX2) | (0<<MUX1) | (0<<MUX0);
	if (channelNumber == 2)
		ADMUX |= (0<<MUX4) | (0<<MUX3) | (0<<MUX2) | (1<<MUX1) | (0<<MUX0);
	if (channelNumber == 6)
		ADMUX |= (0<<MUX4) | (0<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);


   /* ADSC will read as one as long as a conversion is in progress. When the conversion is complete,
   * it returns to zero. Writing zero to this bit has no effect.
   *
   * This while loop will ensure that the conversion is complete.
   */
	while( !(ADCSRA & (1<<ADSC)) );

	/* Return 10bit ADC conversion result */
	uint16_t tenBitADC = (ADCH<<8) | ADCL;

	return tenBitADC;
}

void initPorts()
{
	PORTA = 0x00;
	DDRA = 0x00;

	PORTB = 0x00;
	DDRB = 0x00;

	PORTC = 0x00;
	DDRC = (1<<PIN1) | (1<<PIN0); // Set PC0 and PC1 as outputs (+5V out)

	PORTD = 0x00; 
	DDRD = (1<<PIN3); // Set PD3 (TX1 = USB Tx) as output

	PORTE = 0x00; 
	DDRE = (1<<PIN1); // Set PE1 (TX0 = RS232 Tx) as output

	PORTF = 0x00;
	DDRF = 0x00;

}

void initCAN()
{
    can_init(BITRATE_250_KBPS);
    can_set_mode(NORMAL_MODE);

	/* -----------------------------------------------------------
	 * Set CAN message filters. Each message is placed to it's own
	 * place within the AT90CAN128 memory
     * ----------------------------------------------------------- */
	
	/* -----------------------------------------------------------
	 * Filters for the resolvers
	 * ----------------------------------------------------------- */

    can_filter_t slewJointResolver;
    slewJointResolver.id = SLEW_JOINT_RESOLVER_ID;
    slewJointResolver.mask = SLEW_JOINT_RESOLVER_ID;
    slewJointResolver.flags.rtr = 0;
    slewJointResolver.flags.extended = 1;
    can_set_filter(0, &slewJointResolver);
	
    can_filter_t boomJointResolver;
    boomJointResolver.id = BOOM_JOINT_RESOLVER_ID;
    boomJointResolver.mask = BOOM_JOINT_RESOLVER_ID;
    boomJointResolver.flags.rtr = 0;
    boomJointResolver.flags.extended = 1;
    can_set_filter(1, &boomJointResolver);

    can_filter_t armJointResolver;
    armJointResolver.id = ARM_JOINT_RESOLVER_ID;
    armJointResolver.mask = ARM_JOINT_RESOLVER_ID;
    armJointResolver.flags.rtr = 0;
    armJointResolver.flags.extended = 1;
    can_set_filter(2, &armJointResolver);

    can_filter_t bucketJointResolver;
    bucketJointResolver.id = BUCKET_JOINT_RESOLVER_ID;
    bucketJointResolver.mask = BUCKET_JOINT_RESOLVER_ID;
    bucketJointResolver.flags.rtr = 0;
    bucketJointResolver.flags.extended = 1;
    can_set_filter(3, &bucketJointResolver);

	/* -----------------------------------------------------------
	 * Filters for the valves
	 * ----------------------------------------------------------- */

	/*
	can_filter_t slewJointValve;
    slewJointValve.id = SLEW_JOINT_VALVE_AVEF_ID;
    slewJointValve.mask = SLEW_JOINT_VALVE_AVEF_ID;
    slewJointValve.flags.rtr = 0;
    slewJointValve.flags.extended = 3;
    can_set_filter(4, &slewJointValve);

	can_filter_t boomJointValve;
    boomJointValve.id = BOOM_JOINT_VALVE_AVEF_ID;
    boomJointValve.mask = BOOM_JOINT_VALVE_AVEF_ID;
    boomJointValve.flags.rtr = 0;
    boomJointValve.flags.extended = 3;
    can_set_filter(5, &boomJointValve);

	can_filter_t armJointValve;
    armJointValve.id = ARM_JOINT_VALVE_AVEF_ID;
    armJointValve.mask = ARM_JOINT_VALVE_AVEF_ID;
    armJointValve.flags.rtr = 0;
    armJointValve.flags.extended = 3;
    can_set_filter(6, &armJointValve);

	can_filter_t bucketJointValve;
    bucketJointValve.id = BUCKET_JOINT_VALVE_AVEF_ID;
    bucketJointValve.mask = BUCKET_JOINT_VALVE_AVEF_ID;
    bucketJointValve.flags.rtr = 0;
    bucketJointValve.flags.extended = 3;
    can_set_filter(7, &bucketJointValve);
	*/
}




