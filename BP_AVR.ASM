;***********************************************************
;  Bp_protection - file is programme for processor AT90S4434-8AC
;                  It is davice, which it is protected BP_005
;		   from long measurement time ( more 2 minutes)
;		   and big pressure ( more 300 мм.рт.ст.).
;
;  Clock = 3,6864 mHz
;
;***********************************************************
;
;  Processor pins:
;
;  VCC = 5v
;
;  RESET - Internal
;  AVCC(pin27) = +5V through LPF
;  AREF(pin29) = +5va(Весь размах равен 340 мм.рт.ст.)
;
;  PA0(pin37) - GND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;  PA1(pin36) - ADC1 Pressure channel_2
;  PA2(pin35) - GND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;  PA3(pin34) - ADC3 Pressure channel_0
;  PA4(pin33) - Disable RS_TMS input ( Pull up )
;  PA5(pin32) - no connect           ( Pull up )
;  PA6(pin31) - no connect           ( Pull up )
;  PA7(pin30) - no connect           ( Pull up )
;
;
;  PB0(pin40) - Buzz_av1 output(Low - passive buzzer)
;  PB1(pin41) - Buzz_av2 output(High - passive buzzer)
;  PB2(pin42) - no connect
;  PB3(pin43) - INT2 ( Key signal)
;  PB4(pin44) - SS (SPI Slave Select input)
;  PB5(pin1) - MOSI (SPI Bus Slave Input)
;  PB6(pin2) - MISO (SPI Bus Slave Output)
;  PB7(pin3) - SCK (SPI Bus Serial Clock)
;
;  PC0(pin19) - RS_TMS_I output
;  PC1(pin20) - GEN_STOP output
;  PC2(pin21) - CLK_STOP output
;  PC3(pin22) - INT3 (TMS) output - запрос обмена у TMS
;  PC4(pin23) - R_BR  output
;  PC5(pin24) - Clk32768 is checked  output
;  PC6(pin25) - CRISTAL_32768 input
;  PC7(pin26) - CRISTAL_32768 output
;
;  PD0(pin 9) -
;  PD1(pin10) - TX (UART Output [ADC Value {2 bytes - LSB first}])
;  PD2(pin11) - INT0 - C+5va
;  PD3(pin12) - INT1 - INT1(TMS) - провал во вх. питанию
;  PD4(pin13) - VALVE21 output
;  PD5(pin14) - VALVE22 output
;  PD6(pin15) - ERR0 output
;  PD7(pin16) - ERR1 output
;
;***********************************************************

	.include "4434def.inc"

	; Variables
	.def	Tmp0=r16
	.def	Tmp1=r17
	.def	Tmp2=r18
	.def	Tmp3=r19
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

	; I/O Registers
	; for GIMSK
	.equ	INT0_EN=1<<INT0
	.equ	INT1_EN=1<<INT1

	; for MCUCR
	.equ	IDLE_MODE_MCUCR=0b00001000
	.equ	POWER_DOWN_MODE_MCUCR=0b00101000
	.equ	POWER_SAVE_MODE_MCUCR=0b00111000

	; for WDTCR
	.equ	WATCHDOG_05_SEC=0b00001101
	.equ	WATCHDOG_2_SEC=0b00001111

	; for TIMSK
	.equ	TIM2_OV_EN=0b01000000
	.equ	TIM0_OV_EN=0b00000001

	; Timers
	; for TCCR0
	.equ	TIM0_PRESCALER_0=0
	; for TCCR1B
	.equ	TIM1_PRESCALER_0=0
	; for ASSR
	.equ	AS2_EN=0b00001000
	; for TCCR2
	.equ	TIM2_PRESCALER_32=0b00000011

	.equ	USBR_19_2=11
	.equ	USB_TRANSMIT_ONLY=0b00001000

	; Port A
  		.equ	BIT_PRESSURE_CH_2=1
  		.equ	BIT_PRESSURE_CH_0=3
  		.equ	BIT_DISABLE_RS_TMS=4

	.equ	IO_PIN_PORTA_FOR_RS=0b11100000
	.equ	PORTA_FOR_RS=0b00010000

	; Port B
  		.equ	BIT_BUZZ_AV1=0
  		.equ	BIT_BUZZ_AV1=1
  		.equ	BIT_KEY_SIGNAL=3

	.equ	IO_PIN_PORTB_FOR_RS=0b00000011
	.equ	PORTB_FOR_RS=0b00001110

	; Port C
  		.equ	BIT_RS_TMS_I=0
  		.equ	BIT_GEN_STOP=1
  		.equ	BIT_CLK_STOP=2
  		.equ	BIT_INT3_TMS=3
  		.equ	BIT_R_BR=4
  		.equ	BIT_CLK_32768=5

	.equ	IO_PIN_PORTC_FOR_RS=0b00111111
	.equ	PORTC_FOR_RS_TMS_CLK_RUN=0b00001111
	.equ	PORTC_FOR_TMS_RUN=0b00011110
	.equ	PORTC_FOR_CLK_STOP=0b00001010
	.equ	PORTC_FOR_GEN_STOP=0b00001001
	.equ	PORTC_FOR_GEN_RUN=0b00001011

	; Port D
		.equ	BIT_C_5VA=2
  		.equ	BIT_LOW_BATTERY=3
  		.equ	BIT_VALVE21=4
  		.equ	BIT_VALVE22=5
  		.equ	BIT_ERR0=6
  		.equ	BIT_ERR1=7

	.equ	IO_PIN_PORTD_FOR_RS=0b11110010
	.equ	PORTD_FOR_RS=0b00001101

	; SRAM
	.equ 	PressureShift=0x60
	.equ 	PressureShiftHigh=0x61
	.equ 	SecondsCounter=0x62
	.equ 	MinutesCounter=0x63
	.equ 	HoursCounter=0x64
	.equ 	ErrorInterruptFlag=0x65

	; Conctants
	.equ 	PRESSURE_5=0x000F
	.equ 	PRESSURE_10=0x001E
	.equ 	PRESSURE_15=0x002D
	.equ 	PRESSURE_295=0x0378
	.equ 	PRESSURE_320=0x03C3

	.equ	SHIFT_COUNT_CONST=38
	.equ	CALC_SHIFT_DELAY=8

	.equ 	TIMER_CONST=58594
	.equ	BUZZER_CONST=127


;********************************************************************
; Interrupts vectors
;********************************************************************

        rjmp 	Reset 		; Reset Handler
        rjmp 	Ext_int0 	; IRQ0 Handler
        rjmp 	Ext_int1 	; IRQ1 Handler
        rjmp 	TIM2_comp 	; Timer1 compare Handler
        rjmp 	TIM2_ovf 	; Timer1 Overflow Handler
        rjmp 	TIM1_capt 	; Timer1 Capture Handler
        rjmp 	TIM1_compa 	; Timer1 compare Handler
        rjmp 	TIM1_compb 	; Timer1 compare Handler
        rjmp 	TIM1_ovf 	; Timer1 Overflow Handler
        rjmp 	TIM0_ovf 	; Timer0 Overflow Handler
        rjmp 	SPI_stc 	; SPI Transfer Complete Handler
        rjmp 	UART_rxc 	; UART RX Complete Handler
        rjmp 	UART_udre 	; UDR Empty Handler
        rjmp 	UART_tx 	; UART TX Complete Handler
        rjmp 	ADC_int  	; ADC Conversion Complete Interrupt Handler
        rjmp 	EE_rdy   	; EEPROM Ready Handler
        rjmp 	ANA_comp 	; Analog Comparator Handler

;********************************************************************
;*       Initialization
;********************************************************************

TIM2_comp: 	; Timer1 compare Handler
TIM1_capt: 	; Timer1 Capture Handler
TIM1_compa: 	; Timer1 compare Handler
TIM1_compb: 	; Timer1 compare Handler
TIM1_ovf: 	; Timer1 Overflow Handler
UART_rxc: 	; UART RX Complete Handler
UART_udre: 	; UDR Empty Handler
UART_tx: 	; UART TX Complete Handler
EE_rdy:   	; EEPROM Ready Handler
ANA_comp: 	; Analog Comparator Handler

	cli
	ldi	Tmp0,1
	sts	ErrorInterruptFlag,Tmp0

Reset: 		; Reset Handler

	cli
	sbis	MCUSR,PORF
	rjmp    Init_start

;Power_down_mode
	ldi 	Tmp0,WATCHDOG_05_SEC
	out	WDTCR,Tmp0
	wdr

	ldi 	Tmp0,0
	out 	GIMSK,Tmp0		;Int0 and Int1 interrupts disable!
	out 	TIMSK,Tmp0		;Time interrupts disable!
	out 	ADCSR,Tmp0		;ADC=OFF

	ldi 	Tmp0,TIM0_PRESCALER_0
	out 	TCCR0,Tmp0		;Timer0 stop!

	ldi 	Tmp0,TIM1_PRESCALER_0
	out 	TCCR1B,Tmp0		;Timer1 stop!

	ldi	Tmp0,AS2_EN
	out 	ASSR,Tmp0		;Asynchronous operation of timer2
	ldi	Tmp0,TIM2_PRESCALER_32
	out 	TCCR2,Tmp0		;Timer2 0,25 сек

	ldi 	Tmp0,IO_PIN_PORTA_FOR_RS
	out 	DDRA,Tmp0
	ldi 	Tmp0,PORTA_FOR_RS
	out 	PORTA,Tmp0

	ldi 	Tmp0,IO_PIN_PORTB_FOR_RS
	out 	DDRB,Tmp0
	ldi 	Tmp0,PORTB_FOR_RS
	out 	PORTB,Tmp0

	ldi 	Tmp0,IO_PIN_PORTC_FOR_RS
	out 	DDRC,Tmp0
	ldi 	Tmp0,PORTC_FOR_TMS_RS_CLK_RUN
	out 	PORTC,Tmp0

	ldi 	Tmp0,IO_PIN_PORTD_FOR_RS
	out 	DDRD,Tmp0
	ldi 	Tmp0,PORTD_FOR_RS
	out 	PORTD,Tmp0

	ldi	Tmp0,0
	sts	ErrorInterruptFlag,Tmp0		; Clear ErrorFlag

	ldi 	Tmp0,POWER_DOWN_MODE_MCUCR
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
	rjmp 	Power_down_mode

;*****
Init_start:

	wdr
	ldi 	Tmp0,WATCHDOG_2_SEC
	out	WDTCR,Tmp0

	ldi 	Tmp0,RAMEND
	out 	SP,Tmp0			;Stack Pointer

	ldi 	Tmp0,IO_PIN_PORTA_FOR_RS
	out 	DDRA,Tmp0
	ldi 	Tmp0,PORTA_FOR_RS
	out 	PORTA,Tmp0

	ldi 	Tmp0,IO_PIN_PORTB_FOR_RS
	out 	DDRB,Tmp0
	ldi 	Tmp0,PORTB_FOR_RS
	out 	PORTB,Tmp0

	ldi 	Tmp0,IO_PIN_PORTC_FOR_RS
	out 	DDRC,Tmp0
	ldi 	Tmp0,PORTC_FOR_GEN_RUN
	out 	PORTC,Tmp0

	ldi 	Tmp0,IO_PIN_PORTD_FOR_RS
	out 	DDRD,Tmp0
	ldi 	Tmp0,PORTD_FOR_RS
	out 	PORTD,Tmp0

	ldi Tmp0,USBR_19_2		;11 - 19200 bps [3.6864 MHz]
	out UBRR,Tmp0
	ldi Tmp0,USB_TRANSMIT_ONLY
	out UCR,Tmp0

	ldi 	Tmp0,POWER_SAVE_MODE_MCUCR
	out 	MCUCR,Tmp0





	clr 	Flags
	sts 	PressureShift,Flags
	sts 	PressureShiftHigh,Flags


	ldi	Pressure295Timer,225    ; 15 second
	ldi	Pressure320Timer,15     ; 1 second
	ldi	ShiftCount,SHIFT_COUNT_CONST

	;Read EEPROM
	ldi 	Tmp0,0
	out 	EEAR,Tmp0		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value

	cpi	Tmp0,3
	brlo	Fatal_error_was 	;If was Fatal Error.

	rjmp 	Power_save_mode_start

;*****

Fatal_error_was:

	sbrs	Tmp0,0
	cbi	PORTC,BIT_ERROR_COD_0
	sbrs	Tmp0,1
	cbi	PORTC,BIT_ERROR_COD_1

	cbi	PORTD,BIT_R_BR
	ori	Flags,1<<FATAL_ERROR_FLAG

	sbic    PIND,BIT_KEY
	rjmp 	Power_save_mode_start

	ldi 	Tmp0,0x05
	out 	TCCR0,Tmp0		;Run Timer0 (Prescaler=1024) ~7Hz
	ldi	r17,0x7F
sbi PORTD,7
Wait_key_1:
	in	Tmp0,TIFR
	sbrs    Tmp0,TOV0
	rjmp	Wait_key_1

	in	Tmp0,TIFR
	out     TIFR,Tmp0

	sbic    PIND,BIT_KEY
	rjmp 	Power_save_mode_start

	dec	r17
	brne	Wait_key_1

cbi PORTD,7

	ldi	r17,0x0F
Wait_key_2:
	in	Tmp0,TIFR
	sbrs    Tmp0,TOV0
	rjmp	Wait_key_2

	in	Tmp0,TIFR
	out     TIFR,Tmp0

	dec	r17
	brne	Wait_key_2

sbi PORTD,7
	sbis    PIND,BIT_KEY
	rjmp 	Power_save_mode_start

	;Write EEPROM
	ldi 	Tmp0,0
	out 	EEAR,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,0x0FF
	out 	EEDR,Tmp0		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

	rjmp 	Power_save_mode_start

;**********************************************************************
;
;**********************************************************************
Power_save_mode_start:

	ldi 	Tmp0,0
	out 	ADCSR,Tmp0		;ADC=OFF
	out 	TIMSK,Tmp0		;Time interrupts disable!
	ldi 	Tmp0,(1<<INT0)
	out 	GIMSK,Tmp0		;Int0 interrupts enable only!

	ldi 	Tmp0,POWER_SAVE_MODE_MCUCR
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
	sbrs 	Flags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;*****

Idle_mode_start:
	ldi 	Tmp0,0
	out 	GIMSK,Tmp0		;Int0 & Int1 interrupts disable!

	in 	Tmp0,MCUCR
	andi 	Tmp0,~(1<<SM)		;Clear bit SleepMode (Idle)
	ori 	Tmp0,(1<<SE)		;Set bit SleepEnable
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
	sbrs 	Flags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;********************************************************************

Ext_int0:               ;C+5va
	push 	Tmp0

	ldi 	Tmp0,0
	out 	ADMUX,Tmp0

	ldi 	Tmp0,0b10000101		;Prescaler=32, Int Enable
	out 	ADCSR,Tmp0

	in 	Tmp0,GIMSK
	andi 	Tmp0,~(1<<INT0)
	out 	GIMSK,Tmp0		;Int0 Disable

	in 	Tmp0,TIMSK
	ori 	Tmp0,1<<TOIE0
	out 	TIMSK,Tmp0		;Enable Timer0 Overflow interrupt (ADC Period Starts)
	ldi 	Tmp0,0x04
	out 	TCCR0,Tmp0		;Run Timer0 (Prescaler=256) ~15Hz

	ori 	Flags,1<<IDLE_FLAG	  ;Set Idle Flag
	ori 	Flags,1<<CALK_SHIFT_FLAG  ;Set Calculate shift Flag
	ldi	ShiftCount,SHIFT_COUNT_CONST
	ldi	Pressure295Timer,225    ; 15 second
	ldi	Pressure320Timer,15     ; 1 second

	pop 	Tmp0
	reti

;********************************************************************

Ext_int1:               ;Low battery

	pop 	Tmp0
	reti
;********************************************************************

TIM2_ovf: 	; Timer1 Overflow Handler

;*******************************************************************

TIM0_ovf:		;Timer0 interrupt (~1/15Hz after ADC Convertion)
	push 	Tmp0

	in 	Tmp0,TIMSK
	andi 	Tmp0,~(1<<TOIE0)
	out 	TIMSK,Tmp0		;Disable Timer interrupts

	ldi	Tmp0,0
	out 	TCCR0,Tmp0		;Stop Timer0
	out	TCNT0,Tmp0

	cbi	ADCSR,ADFR		;Clear Free Run bit
	sbi 	ADCSR,ADSC		;Start ADC Conversion
	sbi	ADCSR,ADIE		;Enable ADC Interrupt

	pop 	Tmp0
	reti

;*******************************************************************
SPI_stc: 	; SPI Transfer Complete Handler

;********************************************************************
ADC_int:
	push 	Tmp0

	cbi 	ADCSR,ADIE

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
		in 	Tmp0,TIMSK
		ori 	Tmp0,1<<OCIE1	;Enable Timer1 Compare Match Interrupt
		out 	TIMSK,Tmp0

		;Timer1 (16 bit) minutes counter
		ldi 	Tmp0,HIGH(TIMER_CONST)
		out 	OCR1H,Tmp0
		ldi 	Tmp0,LOW(TIMER_CONST)
		out 	OCR1L,Tmp0

		ldi	Tmp0,0
		out 	TCCR1A,Tmp0

		ldi 	Tmp0,0b00001101	;Clear On Compare & Prescaler=1024
		out 	TCCR1B,Tmp0	;Start Timer1

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

	in 	Tmp0,TIMSK
	andi 	Tmp0,~(1<<OCIE1)		;Disable Timer1 Compare Match Interrupt
	out 	TIMSK,Tmp0

	ldi 	Tmp0,0b00001000		;Clear On Compare & Prescaler=0
	out 	TCCR1B,Tmp0		;Stop Timer1

	ldi	Tmp0,0
	out	TCNT1H,Tmp0		;Clear Timer contents
	out	TCNT1L,Tmp0
	sts	MinuteCounter,Tmp0		;Clear Minute Counter

	cbi	PORTB,BIT_BUZZER

	sbic	PIND,BIT_C_5VA
	andi    Flags,~(1<<IDLE_FLAG)

Before_return:
	in 	Tmp0,TIMSK
	ori 	Tmp0,1<<TOIE0
	out 	TIMSK,Tmp0		;Enable Timer0 Overflow interrupt
	ldi 	Tmp0,0x04
	out 	TCCR0,Tmp0		;Run Timer0 (Prescaler=256) ~15Hz

	pop 	Tmp0
	reti

Calculat_shift:
	dec 	ShiftCount
	cpi	ShiftCount,CALC_SHIFT_DELAY
	brsh	Before_return

	sbi	PORTD,BIT_R_BR

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
	ldi	Tmp0,PRESSURE_5
	sts 	PressureShift,Tmp0
	andi	Flags,~(1<<CALK_SHIFT_FLAG)

	rjmp	Before_return

Fatal_error_occur:
	ldi 	Tmp0,0
	out 	EEAR,Tmp0		;Set EEPROM Address
	out 	EEDR,r17		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

	sbrs	r17,0
	cbi	PORTC,BIT_ERROR_COD_0
	sbrs	r17,1
	cbi	PORTC,BIT_ERROR_COD_1

	cbi	PORTD,BIT_R_BR
	sbi	PORTB,BIT_BUZZER

	ori	Flags,1<<BUZZER_FLAG
	cbi     PORTD,BIT_INT3_OF_TMS
	nop
	sbi     PORTD,BIT_INT3_OF_TMS

		ori	Flags,1<<TIMER_FLAG
		ori	Flags,1<<BUZZER_FLAG
		;Start Timer1 (16 bit) buzzer counter
		in 	Tmp0,TIMSK
		andi 	Tmp0,~(1<<OCIE1)		;Disable Timer1 Compare Match Interrupt
		out 	TIMSK,Tmp0

		;Timer1 (16 bit) buzzer counter
		ldi 	Tmp0,HIGH(BUZZER_CONST)
		out 	OCR1H,Tmp0
		ldi 	Tmp0,LOW(BUZZER_CONST)
		out 	OCR1L,Tmp0
		ldi 	Tmp0,0b01000000
		out	TCCR1A,Tmp0		;Timer Output = Toggle Line
		ldi 	Tmp0,0b00001001		;Clear On Compare & Prescaler=1
		out 	TCCR1B,Tmp0		;Start Timer1

	rjmp	Before_return

