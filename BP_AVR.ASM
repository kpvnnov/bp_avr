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
;  PA2(pin35) - GND !!!!!!!!!!!!!!!!!!! <-> ADC2 Pressure channel_0
;  PA3(pin34) - ADC3 Pressure channel_0 <-> GND !!!!!!!!!!!!!!!!!!!
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
	.def 	InternalFlags=r0                  	; for RS
		.equ 	ACTIVE_TMS_FLAG=0               ; 0
		.equ    CALK_SHIFT_FLAG=1               ; 0
		.equ	PRESSURE_MORE_15_FLAG=2      	; 0
		.equ	FATAL_ERROR_FLAG=3              ; 0
	     	.equ	BUZZER_FLAG=4                   ; 0
	.def 	SystemFlags=r1                          ;
	     	.equ	SYSTEM_RS_FLAG=0                ; 1
	     	.equ	LOW_BATTERY_FLAG=1              ; 0
	     	.equ	SECOND_FLAG=2                   ; 0
	     	.equ	MEASUREMENT_FLAG=3              ; 0
	     	.equ	FATAL_MEASUREMENT_ERROR_FLAG=4  ; 0
		.equ	BAD_SHIFT_FLAG=5
	.def 	SystemFlags,SECOND_FLAG
	.def 	PreSecondsCounter=r2
	.def 	SecondsCounter=r3
	.def 	MinutesCounter=r4
	.def 	HoursCounter=r5
	.def	LowBatteryTimer=r6
	.def	TwoMinutesTimer=r7
	.def	CalcShiftTimer=r8
	.def	Tmp0=r16
	.def	Tmp1=r17
	.def	Tmp2=r18
	.def	Tmp3=r19
	.def	ADCShiftPr0=r20
	.def	ADCShiftPr2=r21
	.def	ADC_L_pr0=r22
	.def	ADC_H_pr0=r23
	.def	ADC_L_pr2=r24
	.def	ADC_H_pr2=r25

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
	.equ	TIM0_PRESCALER_256=4
	.equ	TIM0_PRESCALER_1024=5
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
	.equ	ADM_PR0=3
	.equ	ADM_PR2=1

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
  		.equ	BIT_INT3_OF_TMS=3
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
	.equ	CalcShiftTimer=0x60
	.equ 	PressureShiftPr0=0x61
	.equ 	PressureShiftHighPr0=0x62
	.equ 	PressureShiftPr0=0x63
	.equ 	PressureShiftHighPr0=0x64
	.equ	Pressure295Timer=0x65
	.equ	Pressure320Timer=0x66

	;EEPROM
	.equ	FATAL_ERROR_ADDRESS=0
	.equ	ERR_INT_FLAG_ADDRESS=1

	; Conctants
	.equ 	PRESSURE_5=0x000F
	.equ 	PRESSURE_10=0x001E
	.equ 	PRESSURE_15=0x002D
	.equ 	PRESSURE_295=0x0378
	.equ 	PRESSURE_320=0x03C3

	.equ	SHIFT_TIMER_MAX_VALUE=8		; 2-е сек
	.equ	CALC_SHIFT_DELAY=4              ; 1-а сек на выч. смещени
	.equ	SECOND_1=4
	.equ	SECOND_15=60

	.equ	WHITE_10_MKS=12
	.equ	TIM0_1_MS=239		 	;255-16  prescaler=256
	.equ	TIM0_5_MS=175              	;255-80  prescaler=256

	.equ	FATAL_ERROR_315=0
	.equ	FATAL_ERROR_295=1
	.equ	FATAL_ERROR_2S=2
	.equ	NOT_FATAL_ERROR=0x0FF

	.equ	PRE_SECOND_INITIAL_VALUE=4
	.equ    SECOND_COUNTER_MAX_VALUE=60
	.equ    MINUTES_COUNTER_MAX_VALUE=60
	.equ    HOURS_COUNTER_MAX_VALUE=24

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

Ext_int0: 	; IRQ0 Handler
	cli
	ldi 	Tmp1,0x01
	rjmp	Int_error
TIM2_comp: 	; Timer1 compare Handler
	cli
	ldi 	Tmp1,0x02
	rjmp	Int_error
TIM1_capt: 	; Timer1 Capture Handler
	cli
	ldi 	Tmp1,0x03
	rjmp	Int_error
TIM1_compa: 	; Timer1 compare Handler
	cli
	ldi 	Tmp1,0x04
	rjmp	Int_error
TIM1_compb: 	; Timer1 compare Handler
	cli
	ldi 	Tmp1,0x05
	rjmp	Int_error
TIM1_ovf: 	; Timer1 Overflow Handler
	cli
	ldi 	Tmp1,0x06
	rjmp	Int_error
UART_rxc: 	; UART RX Complete Handler
	cli
	ldi 	Tmp1,0x07
	rjmp	Int_error
UART_udre: 	; UDR Empty Handler
	cli
	ldi 	Tmp1,0x08
	rjmp	Int_error
UART_tx: 	; UART TX Complete Handler
	cli
	ldi 	Tmp1,0x09
	rjmp	Int_error
EE_rdy:   	; EEPROM Ready Handler
	cli
	ldi 	Tmp1,0x0A
	rjmp	Int_error
ANA_comp: 	; Analog Comparator Handler
	cli
	ldi 	Tmp1,0x0B
	rjmp	Int_error

Int_error:
	ldi 	Tmp0,ERR_INT_FLAG_ADDRESS
	out 	EEAR,Tmp0		;Set EEPROM Address
	out 	EEDR,Tmp1		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

Reset: 		; Reset Handler
	cli
	sbis	MCUSR,PORF
	rjmp    Init_start

;Power_down_mode
	wdr
	ldi 	Tmp0,WATCHDOG_05_SEC
	out	WDTCR,Tmp0

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

	ldi 	Tmp0,USBR_19_2		;11 - 19200 bps [3.6864 MHz]
	out 	UBRR,Tmp0
	ldi 	Tmp0,USB_TRANSMIT_ONLY
	out 	UCR,Tmp0

	ldi 	Tmp0,POWER_SAVE_MODE_MCUCR
	out 	MCUCR,Tmp0

	clr 	InternalFlags
	clr 	SystemFlags
	sbr 	SystemFlags,(1<<SYSTEM_RS_FLAG)
	clr 	PreSecondsCounter,Tmp0
	clr 	SecondsCounter,Tmp0
	clr 	MinutesCounter,Tmp0
	clr 	HoursCounter,Tmp0

	ldi 	Tmp0,0
	sts 	PressureShift,Tmp0
	sts 	PressureShiftHigh,Tmp0

	ldi 	Tmp0,60                ;???????????????
	sts	Pressure295Timer,Tmp0   ; 15 second
	ldi 	Tmp0,4                 ;???????????????
	sts	Pressure320Timer,Tmp0   ; 1 second
	ldi 	Tmp0,SHIFT_TIMER_MAX_VALUE
	mov	CalcShiftTimer,Tmp0

Init_white_INT1_1:
	nop
	sbis    PORTD,BIT_LOW_BATTERY
	rjmp    Init_white_INT1_1

	;Read Fatal error from EEPROM and write to PORTD
	ldi 	Tmp0,FATAL_ERROR_ADDRESS
	out 	EEAR,Tmp0		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	sbrs	Tmp0,0
	cbi	PORTD,BIT_ERR0
	sbrs	Tmp0,1
	cbi	PORTD,BIT_ERR1

	cpi	Tmp0,NOT_FATAL_ERROR
	brne	Fatal_error_was 	;If was Fatal Error.

	rjmp 	Power_save_mode_start

;*****

Fatal_error_was:


	sbr	InternalFlags,(1<<FATAL_ERROR_FLAG)

	sbic    PIND,BIT_KEY
	rjmp 	Power_save_mode_start

	ldi 	Tmp0,TIM0_PRESCALER_1024
	out 	TCCR0,Tmp0		;Run Timer0 (Prescaler=1024)
	ldi	Tmp1,200		; 14 сек.
Wait_key_1:
	in	Tmp0,TIFR
	sbrs    Tmp0,TOV0
	rjmp	Wait_key_1

	in	Tmp0,TIFR
	out     TIFR,Tmp0

	sbic    PIND,BIT_KEY
	rjmp 	Power_save_mode_start

	dec	Tmp1
	brne	Wait_key_1

	ldi	Tmp1,71		; 5 сек.
Wait_key_2:
	in	Tmp0,TIFR
	sbrs    Tmp0,TOV0
	rjmp	Wait_key_2

	in	Tmp0,TIFR
	out     TIFR,Tmp0

	dec	Tmp1
	brne	Wait_key_2

	ldi 	Tmp0,TIM0_PRESCALER_0	; Stop Timer0
	out 	TCCR0,Tmp0

	sbis    PIND,BIT_KEY
	rjmp 	Power_save_mode_start

	;Write EEPROM
	ldi 	Tmp0,FATAL_ERROR_ADDRESS
	out 	EEAR,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,NOT_FATAL_ERROR
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
	out 	GIMSK,Tmp0		;Int0 and Int1 interrupts disable
	out 	TIMSK,Tmp0		;Time interrupts disable!
	sbi 	TIMSK,TOIE2     	;Timer2 interrupts enable!

	ldi 	Tmp0,POWER_SAVE_MODE_MCUCR
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
	nop
	nop
	cli
	sbrs 	InternalFlags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;*****

Idle_mode_start:

	ldi 	Tmp0,IDLE_MODE_MCUCR
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
	nop
	nop
	cli
	sbrs 	InternalFlags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;********************************************************************

Ext_int1:               ;Low battery

	pusp 	Tmp0

	cbi	PORTC,BIT_INT3_OF_TMS
	cbi	PORTC,BIT_R_BR

	sbr	SystemFlags,(1<<LOW_BATTERY_FLAG)
	nop
	nop

	sbi	PORTC,BIT_INT3_OF_TMS

	pop 	Tmp0
	reti

;********************************************************************

TIM2_ovf: 	; Timer2 Overflow Handler

	wdr     ; Watchdog reset
		; System watch
	cbr  	SystemFlags,(1<<SECOND_FLAG)
	inc	PreSecondsCounter
	cpi	PreSecondsCounter,PRE_SECOND_INITIAL_VALUE
	brne    Timer2_pin_low_battery_question
	clr	PreSecondsCounter
	sbr  	SystemFlags,(1<<SECOND_FLAG)

	inc     SecondsCounter
	cpi	SecondsCounter,SECOND_COUNTER_MAX_VALUE
	brne    Timer2_pin_low_battery_question
	clr	SecondsCounter

	inc     MinutesCounter
	cpi     MinutesCounter,MINUTES_COUNTER_MAX_VALUE
	brne    Timer2_pin_low_battery_question
	clr	MinutesCounter

	inc	HoursCounter
	cpi     HoursCounter,HOURS_COUNTER_MAX_VALUE
	brne    Timer2_pin_low_battery_question
	clr	HoursCounter

Timer2_pin_low_battery_question:
	sbic    PORTD,BIT_LOW_BATTERY
	rjmp    Timer2_max_low_battery_timer
	dec	LowBatteryTimer
	brpl    Timer2_key_question
	clr	LowBatteryTimer			; Set LowBatteryTimer=0
	rjmp	Timer2_key_question

Timer2_max_low_battery_timer:
	ldi	Tmp0,MAX_LOW_BATTERY_TIMER     	; Set LowBatteryTimer=max
	mov	LowBatteryTimer,Tmp0

Timer2_key_question:
	sbic	PORTB,BIT_KEY_SIGNAL
	rjmp	Timer2_key_1

	ldi	Tmp0,MAX_LOW_BATTERY_TIMER     	; Set LowBatteryTimer=max
	mov	LowBatteryTimer,Tmp0

	sbrs	InternalFlag,ACTIVE_TMS_FLAG
	rjmp	Timer2_key_0_low_battery_1

	sbrs	InternalFlags,PRESSURE_MORE_15_FLAG
	rjmp	Timer2_key_1

	cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	sbi     PORTC,BIT_R_BR
	rjmp	Timer2_key_1

Timer2_key_0_low_battery_1:
	sbic    PORTD,BIT_LOW_BATTERY
	sbr	SystemFlags,(1<<MEASUREMENT_FLAG)

Timer2_key_1:

	sbrc	InternalFlag,ACTIVE_TMS_FLAG
	rjmp	Timer2_C5VA_question

	cpi	PreSecondsCounter,0
	brne	Timer2_quit

	cpi	LowBatteryTimer,0
	breq	Timer2_quit

	sbr	InternalFlags,(1<<ACTIVE_TMS_FLAG) ; ACTIVE_TMS_FLAG = 1
	sbi	PORTC,BIT_GEN_STOP              ; GEN_STOP = 1
	sbi	PORTC,BIT_R_BR                  ; R_BR = 1
	cli
	ldi	Tmp0,TIM0_PRESCALE_256
	out	TCCR0,Tmp0
	ldi	Tmp0,TIM0_5_MS
	out	TCNT0,Tmp0			; 20000 cycle
	in	Tmp0,TIFR
	andi	Tmp0,0x01
	out     TIFR,Tmp0
	sbi	TIMSK,TOIE0
	rjmp	Timer2_quit

Timer2_C5VA_question:

	sbic	PORTD,BIT_C_5VA
	rjmp	Timer2_bit_c5va_1

	dec	TwoMinutesTimer

	ldi 	Tmp0,ADM_PR2		; ADCMUX = Pr2
	out 	ADMUX,Tmp0
	in 	Tmp0,ADCSR              ; ADC flag reset
	out 	ADCSR,Tmp0
	ldi 	Tmp0,0b10000110		;Prescaler=64, Int Enable
	out 	ADCSR,Tmp0
	sbi 	ADCSR,ADSC		;Start ADC Conversion
	sbi	ADCSR,ADIE		;Enable ADC Interrupt
	rjmp	Timer2_quit

Timer2_bit_c5va_1:

	ldi 	Tmp0,0b00000110		;Prescaler=64, Int Disable
	out 	ADCSR,Tmp0
	cbi     PORTC,BIT_R_BR 		; Impulse R_BR
	sbi     PORTC,BIT_R_BR

	ldi     Tmp0,TWO_MINUTES
	mov	TwoMinutesTimer,Tmp0

Timer2_quit:
	reti

;*******************************************************************

TIM0_ovf:		;Timer0 interrupt
	push 	Tmp0

	sbi	PORTC,BIT_CLK_STOP

	ldi	Tmp0,WHITE_10_MKS
Tim0_white_10_mks:
	dec	Tmp0
	brpl	Tim0_white_10_mks

	cbi  	TIMSK,TOIE0             ;Disable Timer interrupts
	ldi	Tmp0,0
	out 	TCCR0,Tmp0		;Stop Timer0
	out	TCNT0,Tmp0

	cbi	PORTC,BIT_RS_TMS_I

	pop 	Tmp0
	reti

;*******************************************************************
SPI_stc: 	; SPI Transfer Complete Handler





;********************************************************************
ADC_int:

	in 	ADC_L_pr0,ADCL		; r24:r25 = ADC Value
	in 	ADC_H_pr0,ADCH
	out	UDR,ADC_L_pr0
	out	UDR,ADC_H_pr0

	in	Tmp0,ADMUX
	cpi	Tmp0,ADM_PR0
	breq    ADC_calc_shift_flag_question
ADC_next_channel:
	mov	ADC_L_pr2,ADC_L_pr0
	mov	ADC_H_pr2,ADC_H_pr0
	ldi 	Tmp0,ADM_PR0		; ADCMUX = Pr0
	out 	ADMUX,Tmp0
	sbi 	ADCSR,ADSC		;Start ADC Conversion
	rjmp	ADC_quit

ADC_calc_shift_flag_question:

	sbrc	InternalFlags,CALK_SHIFT_FLAG
	rjmp	Calculat_shift

	ldi	Tmp0,0
	sub 	ADC_L_pr2,ADCShiftPr2		;Substract Low byte
	sbc 	ADC_H_pr2,Tmp0			;Substract With Carry High byte
	brsh	ADC_sub_shift_pr0
	ldi	ADC_H_pr2,0
	ldi	ADC_L_pr2,0

ADC_sub_shift_pr0:
	sub 	ADC_L_pr0,ADCShiftPr0		;Substract Low byte
	sbc 	ADC_H_pr0,Tmp0			;Substract With Carry High byte
	brsh	ADC_compare_pr0_pr2
	ldi	ADC_H_pr0,0
	ldi	ADC_L_pr0,0

ADC_compare_pr0_pr2:
	cp	ADC_L_pr0,ADC_L_pr2
	cpc	ADC_H_pr0,ADC_H_pr2
	brsh	ADC_Further
	mov     ADC_L_pr0,ADC_L_pr2
	mov	ADC_H_pr0,ADC_H_pr2

ADC_Further:
	ldi 	Tmp1,HIGH(PRESSURE_15)
	cpi 	ADC_L_pr0,LOW(PRESSURE_15)	; Compare to PRESSURE_15
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_15

	sbrc	InternalFlags,PRESSURE_MORE_15_FLAG
	rjmp	Next_check
	sbr	InternalFlags,(1<<PRESSURE_MORE_15_FLAG)

Next_check:
	ldi 	Tmp1,HIGH(PRESSURE_295)
	cpi 	ADC_L_pr0,LOW(PRESSURE_295)	; Compare to PRESSURE_295
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_295

	dec	Pressure295Timer
	breq	Pressure_more_295

	ldi 	Tmp1,HIGH(PRESSURE_320)
	cpi 	ADC_L_pr0,LOW(PRESSURE_320)	; Compare to PRESSURE_320
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_320

	dec	Pressure320Timer
	brne	ADC_quit

	;Write EEPROM
	ldi 	Tmp1,0x02
	rjmp	Fatal_error_occur

Pressure_more_295:
	;Write EEPROM
	ldi 	Tmp1,0x01
	rjmp	Fatal_error_occur

Pressure_less_295:
	ldi	Pressure295Timer,SECOND_1=4
	ldi	Pressure320Timer,SECOND_15=60
	rjmp 	ADC_quit

Pressure_less_320:
	ldi	Pressure320Timer,SECOND_15=60
	rjmp 	ADC_quit

Pressure_less_15:
	cbr     InternalFlags,(1<<PRESSURE_MORE_15_FLAG)
	cbr     InternalFlags,(1<<BUZZER_FLAG)

ADC_quit
	reti


Calculat_shift:
	cbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
	dec 	CalcShiftTimer
	cpi	CalcShiftTimer,CALC_SHIFT_DELAY
	brsh	ADC_quit

	lds 	Tmp0,PressureShiftPr0
	lds 	Tmp1,PressureShiftHighPr0
	add	Tmp0,ADC_L_pr0
	sts	PressureShiftPr0,Tmp0
	adc	Tmp1,ADC_H_pr0
	sts	PressureShiftHighPr0,Tmp1

	lds 	Tmp0,PressureShiftPr2
	lds 	Tmp1,PressureShiftHighPr2
	add	Tmp0,ADC_L_pr2
	sts	PressureShiftPr2,Tmp0
	adc	Tmp1,ADC_H_pr2
	sts	PressureShiftHighPr2,Tmp1

	cpi	CalcShiftTimer,0
	brne	ADC_quit

	lds 	Tmp1,PressureShiftPr2
	lds 	Tmp2,PressureShiftHighPr2
	ror	Tmp2
	ror	Tmp1
	ror	Tmp2
	ror	Tmp1
	mov	ADCShiftPr2,Tmp1
	andi    Tmp2,0x0F
	ldi 	Tmp0,HIGH(PRESSURE_12)
	cpi 	Tmp1,LOW(PRESSURE_12)	;Compare to PRESSURE 12
	cpc 	Tmp2,Tmp0
	brlo	Calc_shift_pr0
	ldi	ADCShiftPr2,PRESSURE_12
	sbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
Calc_shift_pr0:
	lds 	Tmp1,PressureShiftPr0
	lds 	Tmp2,PressureShiftHighPr0
	ror	Tmp2
	ror	Tmp1
	ror	Tmp2
	ror	Tmp1
	mov	ADCShiftPr0,Tmp1
	andi    Tmp2,0x0F
	ldi 	Tmp0,HIGH(PRESSURE_12)
	cpi 	Tmp1,LOW(PRESSURE_12)	;Compare to PRESSURE 12
	cpc 	Tmp2,Tmp0
	brlo	Calc_shift_flag_reset
	ldi	ADCShiftPr0,PRESSURE_12
	sbr	SystemFlags,(1<<BAD_SHIFT_FLAG)

Calc_shift_flag_reset:
	cbr	InternalFlags,(1<<CALK_SHIFT_FLAG)
	rjmp	ADC_quit

Fatal_error_occur:
	ldi 	Tmp0,FATAL_ERROR_ADDRESS
	out 	EEAR,Tmp0		;Set EEPROM Address
	out 	EEDR,Tmp1		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

	sbrs	Tmp1,0
	cbi	PORTD,BIT_ERR0
	sbrs	Tmp1,1
	cbi	PORTD,BIT_ERR1

	cbi	PORTC,BIT_R_BR
	sbi	PORTB,BIT_BUZZER

	sbr	InternalFlags,(1<<BUZZER_FLAG)
	cbi     PORTC,BIT_INT3_OF_TMS
	sbr	SystemFlags,(1<<FATAL_MEASUREMENT_ERROR_FLAG)
	nop
	nop
	nop
	sbi     PORTD,BIT_INT3_OF_TMS
	rjmp	ADC_quit


***********************

