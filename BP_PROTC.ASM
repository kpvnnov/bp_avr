;***********************************************************
;  Bp_protection - file is programme for processor AT90S2333-8AC
;                  It is davice, which it is protected BP_005
;		   from long measurement time ( more 2 minutes)
;		   and big pressure ( more 300 לל.נע.סע.).
;
;  Clock = 1000 kHz
;
;***********************************************************
;
;  Processor pins:
;
;  VCC = 5v
;
;  RESET - from VIP
;
;  PB0 - Output(Low - passive buzzer)
;  PB1 - Output(Low - passive buzzer)/Output Timer 1
;  PB2 - SS (SPI Slave Select input)
;  PB3 - MOSI (SPI Bus Slave Input)
;  PB4 - MISO (SPI Bus Slave Output)
;  PB5 - SCK (SPI Bus Serial Clock)
;
;  AVCC = +5V through LPF
;  AREF = +5va
;
;  PC0 - ADC0 Pressure channel_0
;  PC1 - ADC1 Pressure channel_2 (GND, temporarily)
;  PC2 - ADC2 V battery (GND, temporarily)
;  PC3 - ADC3 (GND, temporarily)
;  PC4 - ADC4 (GND, temporarily)
;  PC5 - ADC5 (GND, temporarily)
;
;  PD0 - RESET for TMS320F206
;  PD1 - TX (UART Output [ADC Value {2 bytes - LSB first}])
;  PD2 - INT0 - C+5va
;  PD3( 1) - INT1 - Key
;  PD4( 2) - Output - Int3 of TMS320F206
;  PD5( 9) - Output - "R" of buffer register controling to valve and pump
;  PD6(10) - Output_1 " Second valve control"
;  PD7(11) - Output_2 " Second valve control"
;
;***********************************************************

	.include "2333def.inc"

	.def	ShiftCount=r20
	.def	Pressure295Timer=r21
	.def	Pressure320Timer=r22
	.def 	Flags=r23
		.equ 	IDLE_FLAG=0
		.equ    CALK_SHIFT_FLAG=1
		.equ	FATAL_ERROR_FLAG=2
	     	.equ	TIMER_FLAG=3
	     	.equ	BUZZER_FLAG=4
	.def	ADC_L=r24
	.def	ADC_H=r25

	.equ 	PRESSURE_5=0x000F
	.equ 	PRESSURE_10=0x001E
	.equ 	PRESSURE_15=0x002D
	.equ 	PRESSURE_295=0x0378
	.equ 	PRESSURE_320=0x03C3

	.equ	SHIFT_COUNT_CONST=16
	.equ	CALC_SHIFT_DELAY=38

	.equ 	TIMER_CONST=58594
	.equ	BUZZER_CONST=127

	; Port D
		.equ	BIT_RS_FOR_TMS=0
		.equ	BIT_TX=1
		.equ	BIT_C_5VA=2
		.equ	BIT_KEY=3
		.equ	BIT_INT3_OF_TMS=4
		.equ	BIT_R_BR=5
		.equ	BIT_VALVE_CONTROL_1=6
		.equ	BIT_VALVE_CONTROL_2=7
	; Port B
		.equ	BIT_BUZZER=0
		
	;SRAM
	.equ 	PressureShift=0x60
	.equ 	PressureShiftHigh=0x61
	.equ 	MinuteCounter=0x62

;********************************************************************
; Interrupts vectors
;********************************************************************

        rjmp 	Reset 		; Reset Handler
        rjmp 	Ext_int0 	; IRQ0 Handler
        rjmp 	Ext_int1 	; IRQ1 Handler
        rjmp 	TIM1_capt 	; Timer1 Capture Handler
        rjmp 	TIM1_comp 	; Timer1 compare Handler
        rjmp 	TIM1_ovf 	; Timer1 Overflow Handler
        rjmp 	TIM0_ovf 	; Timer0 Overflow Handler
        rjmp 	SPI_stc 	; SPI Transfer Complete Handler
        rjmp 	UART_rxc 	; UART RX Complete Handler
        rjmp 	UART_dre 	; UDR Empty Handler
        rjmp 	UART_txc 	; UART TX Complete Handler
        rjmp 	ADC_int  	; ADC Conversion Complete Interrupt Handler
        rjmp 	EE_rdy   	; EEPROM Ready Handler
        rjmp 	ANA_comp 	; Analog Comparator Handler

;********************************************************************

Ext_int0:               ;C+5va
	push 	r16

	ldi 	r16,0
	out 	ADMUX,r16

	ldi 	r16,0b10000101		;Prescaler=32, Int Enable
	out 	ADCSR,r16

	in 	r16,GIMSK
	andi 	r16,~(1<<INT0)
	out 	GIMSK,r16		;Int0 Disable

	in 	r16,TIMSK
	ori 	r16,1<<TOIE0
	out 	TIMSK,r16		;Enable Timer0 Overflow interrupt (ADC Period Starts)
	ldi 	r16,0x04
	out 	TCCR0,r16		;Run Timer0 (Prescaler=256) ~15Hz

	ori 	Flags,1<<IDLE_FLAG	  ;Set Idle Flag
	ori 	Flags,1<<CALK_SHIFT_FLAG  ;Set Calculate shift Flag
	ldi	ShiftCount,SHIFT_COUNT_CONST
	ldi	Pressure295Timer,225
	ldi	Pressure320Timer,15

	pop 	r16
	reti

;********************************************************************

ADC_int:
	push 	r16

	cbi ADCSR,ADIE

	in 	ADC_L,ADCL		; r24:r25 = ADC Value
	in 	ADC_H,ADCH
	
	out	UDR,ADC_L
	out	UDR,ADC_H

	sbrc	Flags,CALK_SHIFT_FLAG
	rjmp	Calculat_shift

	lds 	r18,PressureShift
	clr 	r17

	cp	ADC_L,r18
	cpc	ADC_H,r17
	brlo	ADC_Lower_Than_Shift		;Compare with Shift

	sub 	ADC_L,r18			;Substract Low byte
	sbc 	ADC_H,r17			;Substract With Carry High byte
	rjmp	ADC_Further

ADC_Lower_Than_Shift:
	ldi	ADC_H,0
	ldi	ADC_L,0

ADC_Further:
	ldi 	r17,HIGH(PRESSURE_15)
	cpi 	ADC_L,LOW(PRESSURE_15)	; Compare to PRESSURE_15
	cpc 	ADC_H,r17
	brlo 	Pressure_less_15

	sbrc	Flags,TIMER_FLAG
	rjmp	Next_check


		ori	Flags,1<<TIMER_FLAG
		;Start Timer1 (16 bit) minutes counter
		in 	r16,TIMSK
		ori 	r16,1<<OCIE1	;Enable Timer1 Compare Match Interrupt
		out 	TIMSK,r16

		;Timer1 (16 bit) minutes counter
		ldi 	r16,HIGH(TIMER_CONST)
		out 	OCR1H,r16
		ldi 	r16,LOW(TIMER_CONST)
		out 	OCR1L,r16
		
		ldi	r16,0
		out 	TCCR1A,r16

		ldi 	r16,0b00001101	;Clear On Compare & Prescaler=1024
		out 	TCCR1B,r16	;Start Timer1

Next_check:
	ldi 	r17,HIGH(PRESSURE_295)
	cpi 	ADC_L,LOW(PRESSURE_295)	; Compare to PRESSURE_295
	cpc 	ADC_H,r17
	brlo 	Pressure_less_295

	dec	Pressure295Timer
	breq	Pressure_more_295

	ldi 	r17,HIGH(PRESSURE_320)
	cpi 	ADC_L,LOW(PRESSURE_320)	; Compare to PRESSURE_320
	cpc 	ADC_H,r17
	brlo 	Pressure_less_320

	dec	Pressure320Timer
	brne	Before_return

	;Write EEPROM
	ldi 	r17,0x02
	rjmp	Fatal_error_occur

Pressure_more_295:
	;Write EEPROM
	ldi 	r17,0x01
	rjmp	Fatal_error_occur

Pressure_less_295:
	ldi	Pressure295Timer,225
	ldi	Pressure320Timer,15
	rjmp 	Before_return

Pressure_less_320:
	ldi	Pressure320Timer,15
	rjmp 	Before_return

Pressure_less_15:
	andi    Flags,~(1<<TIMER_FLAG)
	andi    Flags,~(1<<BUZZER_FLAG)
	;Stop Timer1 (16 bit) minutes counter

	in 	r16,TIMSK
	andi 	r16,~(1<<OCIE1)		;Disable Timer1 Compare Match Interrupt
	out 	TIMSK,r16

	ldi 	r16,0b00001000		;Clear On Compare & Prescaler=0
	out 	TCCR1B,r16		;Stop Timer1
	
	ldi	r16,0
	out	TCNT1H,r16		;Clear Timer contents
	out	TCNT1L,r16
	sts	MinuteCounter,r16		;Clear Minute Counter
	
	cbi	PORTB,BIT_BUZZER

	sbic	PIND,BIT_C_5VA
	andi    Flags,~(1<<IDLE_FLAG)

Before_return:
	in 	r16,TIMSK
	ori 	r16,1<<TOIE0
	out 	TIMSK,r16		;Enable Timer0 Overflow interrupt
	ldi 	r16,0x04
	out 	TCCR0,r16		;Run Timer0 (Prescaler=256) ~15Hz

	pop 	r16
	reti

Calculat_shift:
	dec 	ShiftCount
	cpi	ShiftCount,CALC_SHIFT_DELAY
	brsh	Before_return

	ldi 	r17,HIGH(PRESSURE_10)
	cpi 	ADC_L,LOW(PRESSURE_10)	;Compare to PRESSURE 15
	cpc 	ADC_H,r17
	brsh	Bad_shift

	lds 	r17,PressureShift
	lds 	r18,PressureShiftHigh
	add	r17,ADC_L
	sts	PressureShift,r17
	adc	r18,ADC_H
	sts	PressureShiftHigh,r18
	cpi	ShiftCount,0
	brne	Before_return

	ror	r18
	ror	r17
	lsr     r17
	lsr     r17
	sts	PressureShift,r17
	andi	Flags,~(1<<CALK_SHIFT_FLAG)
	rjmp	Before_return

Bad_shift:
	ldi	r16,PRESSURE_5
	sts 	PressureShift,r16
	andi	Flags,~(1<<CALK_SHIFT_FLAG)

	rjmp	Before_return

Fatal_error_occur:
	ldi 	r16,0
	out 	EEAR,r16		;Set EEPROM Address
	out 	EEDR,r17		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

	cbi	PORTD,BIT_R_BR
	sbi	PORTB,BIT_BUZZER

	ori	Flags,1<<BUZZER_FLAG
	cbi     PORTD,BIT_INT3_OF_TMS
	nop
	sbi     PORTD,BIT_INT3_OF_TMS

		ori	Flags,1<<TIMER_FLAG
		ori	Flags,1<<BUZZER_FLAG
		;Start Timer1 (16 bit) buzzer counter
		in 	r16,TIMSK
		andi 	r16,~(1<<OCIE1)		;Disable Timer1 Compare Match Interrupt
		out 	TIMSK,r16

		;Timer1 (16 bit) buzzer counter
		ldi 	r16,HIGH(BUZZER_CONST)
		out 	OCR1H,r16
		ldi 	r16,LOW(BUZZER_CONST)
		out 	OCR1L,r16
		ldi 	r16,0b01000000
		out	TCCR1A,r16		;Timer Output = Toggle Line
		ldi 	r16,0b00001001		;Clear On Compare & Prescaler=1
		out 	TCCR1B,r16		;Start Timer1

	rjmp	Before_return

;********************************************************************
Power_down_mode:
	ldi 	r16,0
	out 	ADCSR,r16		;ADC=OFF
	out 	TIMSK,r16		;Time interrupts disable!
	ldi 	r16,0b00001000		;Clear On Compare & Prescaler=0
	out 	TCCR1B,r16		;Stop Timer1
	ldi 	r16,(1<<INT0)
	out 	GIMSK,r16		;Int0 interrupts enable only!

	in 	r16,MCUCR
	ori 	r16,(1<<SM)+(1<<SE)	;Set bits SleepMode(PowerDown)
	out 	MCUCR,r16               ;and SleepEnable

	cbi	PORTD,BIT_RS_FOR_TMS

	sei
	sleep
	nop
	nop
	nop
	sbrs 	Flags,IDLE_FLAG
	rjmp 	Power_down_mode
	rjmp 	Idle_mode

;********************************************************************

Idle_mode:
	ldi 	r16,0
	out 	GIMSK,r16		;Int0 & Int1 interrupts disable!

	in 	r16,MCUCR
	andi 	r16,~(1<<SM)		;Clear bit SleepMode (Idle)
	ori 	r16,(1<<SE)		;Set bit SleepEnable
	out 	MCUCR,r16

	sei
	sleep
	nop
	nop
	nop

	sbrs 	Flags,IDLE_FLAG
	rjmp 	Power_down_mode
	rjmp 	Idle_mode

;*******************************************************************

TIM0_ovf:		;Timer0 interrupt (~1/15Hz after ADC Convertion)
	push 	r16

	in 	r16,TIMSK
	andi 	r16,~(1<<TOIE0)
	out 	TIMSK,r16		;Disable Timer interrupts

	ldi	r16,0
	out 	TCCR0,r16		;Stop Timer0
	out	TCNT0,r16

	cbi	ADCSR,ADFR		;Clear Free Run bit
	sbi 	ADCSR,ADSC		;Start ADC Conversion
	sbi	ADCSR,ADIE		;Enable ADC Interrupt

	pop 	r16
	reti

;*******************************************************************

TIM1_comp:		;Two minutes counter

	push 	r16

	lds 	r16,MinuteCounter
	inc 	r16
	andi 	r16,0b00000001
	sts	MinuteCounter,r16
	brne 	Timer1_Exit

	;Fatal Error (Time exceeds 2 minutes)
	;Write EEPROM
	ldi 	r17,0x00
	rjmp	Fatal_error_occur
Timer1_Exit:
	pop 	r16
	reti

;********************************************************************
;*       Initialization
;********************************************************************

Reset:
Ext_int1:
TIM1_capt:
TIM1_ovf:
SPI_stc:
UART_rxc:
UART_dre:
UART_txc:
ANA_comp:
EE_rdy:
	cli

	ldi 	r16,RAMEND
	out 	SP,r16			;Stack Pointer

	ldi 	r16,0b00010010
	out 	DDRB,r16
	ldi 	r16,0b11101101
	out 	PORTB,r16

	ldi 	r16,0
	out 	DDRC,r16
	out 	PORTC,r16

	ldi 	r16,0b11110011
	out 	DDRD,r16
	ldi 	r16,0b00111101
	out 	PORTD,r16

	ldi 	r16,0b0001000
	out 	MCUCR,r16		;Int edges

	ldi r16,12	;(12 - 19200 bps [4 MHz],  4800 bps [1MHz])
	out UBRR,r16
	ldi r16,0b00001000
	out UCR,r16

	clr 	Flags
	sts 	PressureShift,Flags
	sts 	PressureShiftHigh,Flags

	;Timer1 (16 bit) minutes counter
	ldi 	r16,HIGH(TIMER_CONST)
	out 	OCR1H,r16
	ldi 	r16,LOW(TIMER_CONST)
	out 	OCR1L,r16

	ldi	Pressure295Timer,225
	ldi	Pressure320Timer,15
	ldi	ShiftCount,16

	;Read EEPROM
	ldi 	r16,0
	out 	EEAR,r16		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	r16,EEDR		;r16 = EEPROM Value

	subi	r16,3
	brlo	Fatal_error_was 	;If was Fatal Error.

	rjmp 	Power_down_mode

;*******************

Fatal_error_was:
	cbi	PORTD,BIT_R_BR
	ori	Flags,1<<FATAL_ERROR_FLAG

	sbic    PIND,BIT_KEY
	rjmp 	Power_down_mode

	ldi 	r16,0x05
	out 	TCCR0,r16		;Run Timer0 (Prescaler=1024) ~7Hz
	ldi	r17,0x7F
sbi PORTD,7
Wait_key_1:
	in	r16,TIFR
	sbrs    r16,TOV0
	rjmp	Wait_key_1

	in	r16,TIFR
	out     TIFR,r16

	sbic    PIND,BIT_KEY
	rjmp 	Power_down_mode

	dec	r17
	brne	Wait_key_1

cbi PORTD,7

	ldi	r17,0x0F
Wait_key_2:
	in	r16,TIFR
	sbrs    r16,TOV0
	rjmp	Wait_key_2

	in	r16,TIFR
	out     TIFR,r16

	dec	r17
	brne	Wait_key_2

sbi PORTD,7
	sbis    PIND,BIT_KEY
	rjmp 	Power_down_mode

	;Write EEPROM
	ldi 	r16,0
	out 	EEAR,r16		;Set EEPROM Address
	ldi 	r16,0x0FF
	out 	EEDR,r16		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

	rjmp 	Power_down_mode
