;***********************************************************
;  Bp_protection - file is programme for processor AT90S4434-8AC
;                  It is davice, which it is protected BP_005
;		   from long measurement time ( more 2 minutes)
;		   and big pressure ( more 300 ��.��.��.).
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
;  AREF(pin29) = +5va(���� ������ ����� 340 ��.��.��.)
;
;  PA0(pin37) - GND !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;  PA1(pin36) - ADC1 Pressure channel_2
;  PA2(pin35) - GND !!!!!!!!!!!!!!!!!!! 
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
;  PC3(pin22) - INT3 (TMS) output - ������ ������ � TMS
;  PC4(pin23) - R_BR  output
;  PC5(pin24) - Clk32768 is checked  output
;  PC6(pin25) - CRISTAL_32768 input
;  PC7(pin26) - CRISTAL_32768 output
;
;  PD0(pin 9) -
;  PD1(pin10) - TX (UART Output [ADC Value {2 bytes - LSB first}])
;  PD2(pin11) - INT0 - C+5va
;  PD3(pin12) - INT1 - INT1(TMS) - ������ �� ��. �������
;  PD4(pin13) - VALVE21 output
;  PD5(pin14) - VALVE22 output
;  PD6(pin15) - ERR0 output
;  PD7(pin16) - ERR1 output
;
;***********************************************************

	.include "4434def.inc"

	; Variables

	.def 	PreSecondsCounter=r0
	.def 	SecondsCounter=r1
	.def 	MinutesCounter=r2
	.def 	HoursCounter=r3
	.def	LowBatteryTimer=r4
	.def	TwoMinutesTimer=r5
	.def	CalcShiftTimer=r6
	.def	ADCShiftPr0=r7
	.def	ADCShiftPr2=r8
	.def	ADC_L_pr0=r9
	.def	ADC_H_pr0=r10
	.def	ADC_L_pr2=r11
	.def	ADC_H_pr2=r12
	.def	SPIByteCounter=r13
	.def    SPICommand=r14
	.def	OneMinutesTimer=r15
	.def	Tmp0=r16
	.def	Tmp1=r17
	.def	Tmp2=r18
	.def	Tmp3=r19
	.def 	InternalFlags=r20                  	; for RS
		.equ 	ACTIVE_TMS_FLAG=0               ; 0
		.equ    CALK_SHIFT_FLAG=1               ; 0
		.equ	PRESSURE_MORE_15_FLAG=2      	; 0
	     	.equ	BUZZER_FLAG=3                   ; 0
		.equ	FIRST_BYTE_OF_SPI=4		; 1
	.def 	SystemFlags=r21                          ;
	     	.equ	SYSTEM_RS_FLAG=0                ; 1
	     	.equ	LOW_BATTERY_FLAG=1              ; 0
	     	.equ	SECOND_FLAG=2                   ; 0
	     	.equ	MEASUREMENT_FLAG=3              ; 0
	     	.equ	FATAL_ERROR_FLAG=4  		; 0
		.equ	BAD_SHIFT_FLAG=5                ; 0
		.equ	TEST_FLAG=6                     ; 0
	.def	ThirtySecondsTimer=r22
	.def    MeasurementFlagTimer=r23

	; I/O Registers
	; for GIMSK
	.equ	INT0_EN=1<<INT0
	.equ	INT1_EN=1<<INT1

	; for MCUCR
	.equ	IDLE_MODE_MCUCR=0b01001000
	.equ	POWER_DOWN_MODE_MCUCR=0b01101000
	.equ	POWER_SAVE_MODE_MCUCR=0b01111000

	; for WDTCR
	.equ	WATCHDOG_025_SEC=0b00001100	;0b00001101
	.equ	WATCHDOG_2_SEC=0b00001111
	.equ	WATCHDOG_RS1=0b00011111
	.equ	WATCHDOG_RS2=0b00010111

	; for TIMSK

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
	
	; FOR SPCR
	.equ	SPCR_SPI_EN_I_EN=0b11000000

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
  		.equ	BIT_BUZZ_AV2=1
  		.equ	BIT_KEY_SIGNAL=3
  		.equ	BIT_SPI_SS=4

	.equ	IO_PIN_PORTB_FOR_RS=0b01000011
	.equ	PORTB_FOR_RS=0b01001110

	; Port C
  		.equ	BIT_RS_TMS_I=0
  		.equ	BIT_GEN_STOP=1
  		.equ	BIT_CLK_STOP=2
  		.equ	BIT_INT3_OF_TMS=3
  		.equ	BIT_R_BR=4
  		.equ	BIT_CLK_32768=5

	.equ	IO_PIN_PORTC_FOR_RS=0b00111111
	.equ	PORTC_FOR_TMS_RS_CLK_RUN=0b00011111
	.equ	PORTC_FOR_TMS_RUN=0b00011110
	.equ	PORTC_FOR_CLK_STOP=0b00001010
	.equ	PORTC_FOR_GEN_STOP=0b00001001
	.equ	PORTC_FOR_GEN_RUN=0b00011011

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
	.equ 	PressureShiftPr0=0x61
	.equ 	PressureShiftHighPr0=0x62
	.equ 	PressureShiftPr2=0x63
	.equ 	PressureShiftHighPr2=0x64
	.equ	Pressure295Timer=0x65
	.equ	Pressure320Timer=0x66
	.equ    EEPROMByteCounter=0x67
	.equ    FatalError=0x68
	.equ	SPINewTime=0x6A			; 3-�� ����
	.equ	SPISerialNumber=0x70		; 5-�� ����
	.equ	SPINewSerialNumber=0x75		; 5-�� ����

	;EEPROM
	.equ	FATAL_ERROR_ADDRESS=0
	.equ	ERR_INT_ADDRESS=1
	.equ	SERIAL_NUMBER_ADDRESS=2

	; Conctants
	.equ 	PRESSURE_5=0x000F
	.equ 	PRESSURE_10=0x001E
	.equ 	PRESSURE_12=0x0024
	.equ 	PRESSURE_15=0x002D
	.equ 	PRESSURE_295=0x0378
	.equ 	PRESSURE_320=0x03C3

	.equ	SHIFT_TIMER_MAX_VALUE=8		; 2-� ���
	.equ	CALC_SHIFT_DELAY=4              ; 1-� ��� �� ���. �������
	.equ	SECOND_1=4
	.equ	SECOND_15=60
	.equ    THIRTY_SECONDS=120
	.equ	TWO_MINUTES=118
	.equ	ONE_MINUTES=240
	.equ    TWO_HOURS=120

	.equ	WHITE_10_MKS=12
	.equ	WHITE_30_MKS=40
	.equ	WHITE_8_MKS=10
	.equ	TIM0_1_MS=239		 	;255-16  prescaler=256
	.equ	TIM0_5_MS=175              	;255-80  prescaler=256
	.equ    TIMER1_4_KHZ=0x0FF8D		; 0x0FFFF-((250*3.6468)/8)
	.equ    TIMER1_PRESCALE_8=2

	.equ	FATAL_ERROR_2S=0
	.equ	FATAL_ERROR_295=1
	.equ	FATAL_ERROR_315=2
	.equ	C5VA_ERROR=4
	.equ	NOT_FATAL_ERROR=0x0FF

	.equ	PRE_SECOND_INITIAL_VALUE=32
	.equ    SECOND_COUNTER_MAX_VALUE=60
	.equ    MINUTES_COUNTER_MAX_VALUE=60
	.equ    HOURS_COUNTER_MAX_VALUE=24
	.equ    MAX_LOW_BATTERY_TIMER=20

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
ANA_comp: 	; Analog Comparator Handler
	cli
	ldi 	Tmp1,0x0B
	rjmp	Int_error

Int_error:
	ldi 	Tmp0,LOW(ERR_INT_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(ERR_INT_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	out 	EEDR,Tmp1		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE
	
		rjmp 	Idle_mode_start ;!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Reset: 		; Reset Handler
	cli
	in	Tmp0,MCUSR
;	sbrs	Tmp0,PORF
	rjmp    Init_start

Power_down_mode:
	wdr
	ldi 	Tmp0,WATCHDOG_025_SEC
	out	WDTCR,Tmp0

	ldi 	Tmp0,TIM0_PRESCALER_0
	out 	TCCR0,Tmp0		;Timer0 stop!

	ldi 	Tmp0,TIM1_PRESCALER_0
	out 	TCCR1B,Tmp0		;Timer1 stop!

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

	ldi	Tmp0,AS2_EN
	out 	ASSR,Tmp0		;Asynchronous operation of timer2
Wait_update_assr0:	
	in	Tmp0,ASSR
	andi	Tmp0,((1<<TCN2UB)|(1<<TCR2UB))
	brne	Wait_update_assr0
	ldi	Tmp0,TIM2_PRESCALER_32
	out 	TCCR2,Tmp0		;Timer2 0,25 ���
	
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
;	ldi 	Tmp0,WATCHDOG_2_SEC	;!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	ldi 	Tmp0,WATCHDOG_RS1	;!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	ldi 	Tmp1,WATCHDOG_RS2	;!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	out	WDTCR,Tmp0		
	out	WDTCR,Tmp1		;!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	ldi 	Tmp0,LOW(RAMEND)
	out 	SPL,Tmp0			;Stack Pointer
	ldi 	Tmp0,HIGH(RAMEND)
	out 	SPH,Tmp0			;Stack Pointer

	ldi	Tmp0,0
	ldi	XH,0
	ldi	XL,23
Init_zero0:
	st	-X,Tmp0
	cpi	XL,0
	brne	Init_zero0
	
	ldi	Tmp0,32
	ldi	Tmp1,0
	ldi	XH,0
	ldi	XL,0x60
Init_zero1:
	st	X+,Tmp1
	dec	Tmp0
	brne	Init_zero1

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
	ldi 	Tmp0,PORTC_FOR_TMS_RUN
	out 	PORTC,Tmp0

	ldi 	Tmp0,IO_PIN_PORTD_FOR_RS
	out 	DDRD,Tmp0
	ldi 	Tmp0,PORTD_FOR_RS
	out 	PORTD,Tmp0
	
	ldi	Tmp0,AS2_EN
	out 	ASSR,Tmp0		;Asynchronous operation of timer2
Wait_update_assr:	
	in	Tmp0,ASSR
	andi	Tmp0,((1<<TCN2UB)|(1<<TCR2UB))
	brne	Wait_update_assr
	ldi	Tmp0,TIM2_PRESCALER_32
	out 	TCCR2,Tmp0		;Timer2 0,25 ���

	ldi 	Tmp0,1<<TOIE2     	;Timer2 interrupts enable!
	out 	TIMSK,Tmp0		;Time interrupts disable!
	
	in	Tmp0,SPSR
	ldi	Tmp0,SPCR_SPI_EN_I_EN	;SPI initialyzation
	out	SPCR,Tmp0
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	
	ldi	Tmp2,0x007
Wiat02:		
	ldi	Tmp1,0x0FF
Wiat01:	
	ldi	Tmp0,0x0FF
Wiat00:	
	dec	Tmp0
	brne	Wiat00
	dec	Tmp1
	brne	Wiat01
	dec	Tmp2
	brne	Wiat02
			
Init_white_INT1_1:
	nop
	wdr
	sbis    PIND,BIT_LOW_BATTERY
	rjmp    Init_white_INT1_1
	cbi	PORTD,BIT_ERR1
	
	sbi	PORTD,BIT_ERR0

	ldi 	Tmp0,USBR_19_2		;11 - 19200 bps [3.6864 MHz]
	out 	UBRR,Tmp0
	ldi 	Tmp0,USB_TRANSMIT_ONLY
	out 	UCR,Tmp0
	out	UDR,Tmp0

	ori 	SystemFlags,(1<<SYSTEM_RS_FLAG)
	ori	InternalFlags,(1<<ACTIVE_TMS_FLAG) ; ACTIVE_TMS_FLAG = 1

	ldi     Tmp0,TWO_MINUTES
	mov	TwoMinutesTimer,Tmp0
	ldi     Tmp0,MAX_LOW_BATTERY_TIMER
	mov	LowBatteryTimer,Tmp0
	ldi     Tmp0,SHIFT_TIMER_MAX_VALUE
	mov	CalcShiftTimer,Tmp0
	ldi     Tmp0,1
	mov	SPIByteCounter,Tmp0
	ldi     Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0
	ldi     Tmp0,TWO_HOURS
	mov	MeasurementFlagTimer,Tmp0	

	ldi	Tmp0,SECOND_15
	sts	Pressure295Timer,Tmp0
	ldi	Tmp0,SECOND_1
	sts	Pressure320Timer,Tmp0
	
		cbi	PORTD,BIT_ERR1			;!!!!!!!!!!!!!!!!!!
		cbi	PORTD,BIT_ERR0			;!!!!!!!!!!!!!!!!!!
		cbi	PORTD,BIT_VALVE21		;!!!!!!!!!!!!!!!!!!
		cbi	PORTD,BIT_VALVE22		;!!!!!!!!!!!!!!!!!!

	;Read Fatal error from EEPROM and write to PORTD
	ldi 	Tmp0,LOW(FATAL_ERROR_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(FATAL_ERROR_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	sts	FatalError,Tmp0

	sbrs	Tmp0,0
	cbi	PORTD,BIT_ERR0
	sbrs	Tmp0,1
	cbi	PORTD,BIT_ERR1

	cpi	Tmp0,NOT_FATAL_ERROR
	brne	Fatal_error_was 	;If was Fatal Error.

	sbic    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

;*****

Fatal_error_was:

	sbr	SystemFlags,(1<<FATAL_ERROR_FLAG)

	sbic    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

	ldi 	Tmp0,TIM0_PRESCALER_1024
	out 	TCCR0,Tmp0		;Run Timer0 (Prescaler=1024)
	ldi	Tmp1,200		; 14 ���.
Wait_key_1:
	in	Tmp0,TIFR
	sbrs    Tmp0,TOV0
	rjmp	Wait_key_1

	in	Tmp0,TIFR
	out     TIFR,Tmp0

	sbic    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

	dec	Tmp1
	brne	Wait_key_1

	ldi	Tmp1,71		; 5 ���.
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

	sbis    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

	;Write EEPROM
	ldi 	Tmp0,LOW(FATAL_ERROR_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(FATAL_ERROR_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,NOT_FATAL_ERROR
	out 	EEDR,Tmp0		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE
	sts	FatalError,Tmp0
	cbr	SystemFlags,(1<<FATAL_ERROR_FLAG)
	sbr	SystemFlags,1<<TEST_FLAG

	rjmp 	Idle_mode_start

;**********************************************************************
;
;**********************************************************************
Power_save_mode_start:

	ldi 	Tmp0,0
	out 	ADCSR,Tmp0		;ADC=OFF
	out 	GIMSK,Tmp0		;Int0 and Int1 interrupts disable
		
	sbic	PINB,BIT_SPI_SS				
	ori	InternalFlags,1<<FIRST_BYTE_OF_SPI
	
	ldi	Tmp0,0			;Asynchronous operation of timer2
	out 	OCR2,Tmp0		;Timer2 0,25 ���
Wait_update_assr2:	
	in	Tmp0,ASSR
	andi	Tmp0,(1<<OCR2UB)
	brne	Wait_update_assr2

	ldi 	Tmp0,POWER_SAVE_MODE_MCUCR
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
	nop
	
;	wdr	; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	nop
	cli
	sbrs 	InternalFlags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;*****

Idle_mode_start:

	
	sbic	PINB,BIT_SPI_SS				
	ori	InternalFlags,1<<FIRST_BYTE_OF_SPI

	ldi 	Tmp0,IDLE_MODE_MCUCR
	out 	MCUCR,Tmp0

	sei
	sleep
	nop
	nop
	nop
;	nop
		
;	wdr	; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	nop
	cli
	sbrs 	InternalFlags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;********************************************************************

Ext_int1:               ;Low battery

	push 	Tmp0

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
	sbic	PINB,BIT_SPI_SS				
	ori	InternalFlags,1<<FIRST_BYTE_OF_SPI
		
	cbr  	SystemFlags,(1<<SECOND_FLAG)
	inc	PreSecondsCounter
	ldi	Tmp0,PRE_SECOND_INITIAL_VALUE
	cp	PreSecondsCounter,Tmp0
	brne    Timer2_pin_low_battery_question
	clr	PreSecondsCounter
	sbr  	SystemFlags,(1<<SECOND_FLAG)
	
	ldi	Tmp1,1<<BIT_CLK_32768	
	in	Tmp0,PORTC		
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0

	inc     SecondsCounter
	ldi	Tmp0,SECOND_COUNTER_MAX_VALUE
	cp	SecondsCounter,Tmp0
	brne    Timer2_pin_low_battery_question
	clr	SecondsCounter
		
	dec	MeasurementFlagTimer
	brne    Timer2_minutes_inc
	ldi	Tmp0,TWO_HOURS
	mov	MeasurementFlagTimer,Tmp0
	sbr	SystemFlags,(1<<MEASUREMENT_FLAG)
Timer2_minutes_inc:
	inc     MinutesCounter
	ldi	Tmp0,MINUTES_COUNTER_MAX_VALUE
	cp      MinutesCounter,Tmp0
	brne    Timer2_pin_low_battery_question
	clr	MinutesCounter

	inc	HoursCounter
	ldi	Tmp0,HOURS_COUNTER_MAX_VALUE
	cp      HoursCounter,Tmp0
	brne    Timer2_pin_low_battery_question
	clr	HoursCounter

Timer2_pin_low_battery_question:

	sbic    PIND,BIT_LOW_BATTERY
	rjmp    Timer2_max_low_battery_timer
	dec	LowBatteryTimer
	brne    Timer2_key_question
	clr	LowBatteryTimer			; Set LowBatteryTimer=0
	rjmp	Timer2_key_question

Timer2_max_low_battery_timer:
	ldi	Tmp0,MAX_LOW_BATTERY_TIMER     	; Set LowBatteryTimer=max
	mov	LowBatteryTimer,Tmp0

Timer2_key_question:
	sbic	PINB,BIT_KEY_SIGNAL
	rjmp	Timer2_key_1

	ldi	Tmp0,MAX_LOW_BATTERY_TIMER     	; Set LowBatteryTimer=max
	mov	LowBatteryTimer,Tmp0

	sbrs	InternalFlags,ACTIVE_TMS_FLAG
	rjmp	Timer2_key_0_low_battery_1

	sbrs	InternalFlags,PRESSURE_MORE_15_FLAG
	rjmp	Timer2_key_1

	cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	sbi     PORTC,BIT_R_BR
	rjmp	Timer2_key_1

Timer2_key_0_low_battery_1:
	sbic    PIND,BIT_LOW_BATTERY
	sbr	SystemFlags,(1<<MEASUREMENT_FLAG)

Timer2_key_1:
	cpi	ThirtySecondsTimer,0
	breq    Timer2_zero_to_30SecTime
	dec     ThirtySecondsTimer
	sbrc	SystemFlags,TEST_FLAG
	rjmp    Timer2_zero_to_30SecTime
	cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	sbi     PORTC,BIT_R_BR
	cbr	SystemFlags,(1<<MEASUREMENT_FLAG)
	rjmp	Timer2_FATAL_ERROR_FLAG_check

Timer2_zero_to_30SecTime:
	clr     ThirtySecondsTimer

Timer2_FATAL_ERROR_FLAG_check:
	sbrs	SystemFlags,FATAL_ERROR_FLAG
	rjmp	Timer2_ACTIVE_TMS_FLAG_check
	cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	sbi     PORTC,BIT_R_BR
	cbr	SystemFlags,(1<<MEASUREMENT_FLAG)

Timer2_ACTIVE_TMS_FLAG_check:
	sbrc	InternalFlags,ACTIVE_TMS_FLAG
	rjmp	Timer2_C5VA_question

	sbrs    SystemFlags,SECOND_FLAG
	rjmp	Timer2_quit

	ldi	Tmp0,0
	cp	LowBatteryTimer,Tmp0
	breq	Timer2_quit_
	
	sbr	InternalFlags,(1<<ACTIVE_TMS_FLAG) ; ACTIVE_TMS_FLAG = 1
	sbi	PORTC,BIT_GEN_STOP              ; GEN_STOP = 1
	sbi	PORTC,BIT_R_BR                  ; R_BR = 1
	ldi	Tmp0,TIM0_PRESCALER_256
	out	TCCR0,Tmp0
	ldi	Tmp0,TIM0_5_MS
	out	TCNT0,Tmp0			; 20000 cycle
	in	Tmp0,TIFR
	andi	Tmp0,(1<<TOV0)
	out     TIFR,Tmp0
	in	Tmp0,TIMSK		;Enable Timer0 interrupts
	ori  	Tmp0,(1<<TOIE0)
	out	TIMSK,Tmp0	
	rjmp	Timer2_quit

Timer2_C5VA_question:

	sbic	PIND,BIT_C_5VA
	rjmp	Timer2_bit_c5va_1

	dec	OneMinutesTimer
	brne    Timer2_bit_c5va_0
	ldi	Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0
	sbrc	SystemFlags,TEST_FLAG
	rjmp    Timer2_bit_c5va_0
	ldi 	Tmp0,LOW(FATAL_ERROR_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(FATAL_ERROR_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	ldi	Tmp1,C5VA_ERROR
	out 	EEDR,Tmp1		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE
	sts	FatalError,Tmp1
	cbi     PORTC,BIT_R_BR 		; Impulse R_BR
	sbi     PORTC,BIT_R_BR
	sbr	SystemFlags,(1<<FATAL_ERROR_FLAG)

	ldi	Tmp0,LOW(TIMER1_4_KHZ)		; Timer1 enable
	out	TCNT1L,Tmp0
	ldi	Tmp0,HIGH(TIMER1_4_KHZ)
	out	TCNT1H,Tmp0
	ldi	Tmp0,TIMER1_PRESCALE_8
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK		; Enable Timer1 interrupts
	ori  	Tmp0,(1<<TOIE1)
	out	TIMSK,Tmp0
	
Timer2_bit_c5va_0:
	ldi 	Tmp0,ADM_PR2		; ADCMUX = Pr2
	out 	ADMUX,Tmp0
	in 	Tmp0,ADCSR              ; ADC flag reset
	out 	ADCSR,Tmp0
	ldi 	Tmp0,0b10000110		;Prescaler=64, Int Enable
	out 	ADCSR,Tmp0
	sbi 	ADCSR,ADSC		;Start ADC Conversion
	sbi	ADCSR,ADIE		;Enable ADC Interrupt
Timer2_quit_:	
	rjmp	Timer2_quit

Timer2_bit_c5va_1:

	ldi 	Tmp0,0b00000110		;Prescaler=64, Int Disable
	out 	ADCSR,Tmp0
	sbr	InternalFlags,(1<<CALK_SHIFT_FLAG)
	cbi     PORTC,BIT_R_BR 		; Impulse R_BR
	sbi     PORTC,BIT_R_BR

	ldi 	Tmp0,0
	sts 	PressureShiftPr0,Tmp0
	sts 	PressureShiftHighPr0,Tmp0
	sts 	PressureShiftPr2,Tmp0
	sts 	PressureShiftHighPr2,Tmp0

	ldi     Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0

Timer2_quit:
	reti

;*******************************************************************
TIM1_ovf: 	; Timer1 Overflow Handler
	sbic	PINB,BIT_BUZZ_AV1
	rjmp	TIM1_buzz_av1_0
	sbi	PORTB,BIT_BUZZ_AV1
	cbi	PORTB,BIT_BUZZ_AV2
	reti
TIM1_buzz_av1_0:
	cbi	PORTB,BIT_BUZZ_AV1
	sbi	PORTB,BIT_BUZZ_AV2
	reti
;*******************************************************************

TIM0_ovf:		;Timer0 interrupt

;	push 	Tmp0

	sbi	PORTC,BIT_CLK_STOP

	ldi	Tmp0,WHITE_10_MKS
Tim0_white_10_mks:
	dec	Tmp0
	brne	Tim0_white_10_mks

	in	Tmp0,TIMSK		;Disable Timer0 interrupts
	andi  	Tmp0,~(1<<TOIE0)
	out	TIMSK,Tmp0
	ldi	Tmp0,0
	out 	TCCR0,Tmp0		;Stop Timer0
	out	TCNT0,Tmp0

	cbi	PORTC,BIT_RS_TMS_I

;	pop 	Tmp0
	reti

;*******************************************************************
SPI_stc: 	; SPI Transfer Complete Handler

		sbi	PORTD,BIT_ERR1			;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	;	in	Tmp0,SPSR			;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	sbrs	InternalFlags,FIRST_BYTE_OF_SPI
	rjmp	SPI_second_byte
	in	Tmp0,SPDR
	mov     Tmp1,Tmp0
	swap	Tmp1
	eor	Tmp1,Tmp0
	breq	SPI_good_command
SPI_bad_command:
	ldi	Tmp1,0x0EE	;0x0FF
	out	SPDR,Tmp1
	ldi	Tmp1,1
	mov 	SPIByteCounter,Tmp1
	ldi    	Tmp0,0x0F
	mov	SPICommand,Tmp0
	andi	InternalFlags,~(1<<FIRST_BYTE_OF_SPI)
		sbi	PORTD,BIT_ERR0			;!!!!!!!!!!!!!!!!!!
	rjmp	SPI_quit

SPI_good_command:
	cbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	andi    Tmp0,0x0F
	mov	SPICommand,Tmp0
	ldi	r30,LOW(SPI_jump_first_byte)
	ldi	r31,HIGH(SPI_jump_first_byte)
	add	r30,Tmp0
	adc	r31,Tmp1
	ijmp

SPI_jump_first_byte:
	rjmp	SPI_system_stat_command
	rjmp	SPI_TMS_powerdown_command
	rjmp	SPI_read_time_command
	rjmp	SPI_read_serial_number_command
	rjmp	SPI_read_fatal_error_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_set_test_flag_command
	rjmp	SPI_set_time_command
	rjmp	SPI_write_serial_number_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command

SPI_system_stat_command:
	out	SPDR,SystemFlags
	ldi	Tmp1,1
	mov 	SPIByteCounter,Tmp1
		sbi	PORTD,BIT_VALVE21		;!!!!!!!!!!!!!!!!!!
	rjmp	SPI_quit

SPI_TMS_powerdown_command:
	ldi	Tmp1,0x11
	out	SPDR,Tmp1
	ldi	Tmp1,1
	mov 	SPIByteCounter,Tmp1
		sbi	PORTD,BIT_VALVE22		;!!!!!!!!!!!!!!!!!!
	rjmp	SPI_quit

SPI_read_time_command:
	out	SPDR,SecondsCounter
	ldi	Tmp1,3
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_read_serial_number_command:
	lds	Tmp0,SPISerialNumber
	out	SPDR,Tmp0
	ldi	Tmp1,5
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_read_fatal_error_command:
	lds	Tmp0,FatalError
	out	SPDR,Tmp0
	ldi	Tmp1,1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_set_test_flag_command:
	ldi	Tmp1,0x77
	out	SPDR,Tmp1
	ldi	Tmp1,1
	mov 	SPIByteCounter,Tmp1
	sbr	SystemFlags,1<<TEST_FLAG
	rjmp	SPI_quit

SPI_set_time_command:
	ldi	Tmp1,0x88
	out	SPDR,Tmp1
	ldi	Tmp1,3
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_write_serial_number_command:
	ldi	Tmp1,0x99
	out	SPDR,Tmp1
	ldi	Tmp1,5
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_erroneous_command:
	rjmp	SPI_bad_command


SPI_second_byte:
		cbi	PORTD,BIT_ERR1			;!!!!!!!!!!!!!!!!!!
	dec	SPIByteCounter
	ldi	Tmp1,0
	ldi	r30,LOW(SPI_jump_second_byte)
	ldi	r31,HIGH(SPI_jump_second_byte)
	add	r30,SPICommand
	adc	r31,Tmp1
	ijmp

SPI_jump_second_byte:
	rjmp	SPI_system_stat
	rjmp	SPI_TMS_powerdown
	rjmp	SPI_read_time_next_byte
	rjmp	SPI_read_serial_number
	rjmp	SPI_read_fatal_error
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous
	rjmp	SPI_set_test_flag
	rjmp	SPI_set_time
	rjmp	SPI_write_serial_number
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous

SPI_system_stat:
	cbr 	SystemFlags,(1<<SYSTEM_RS_FLAG)
		cbi	PORTD,BIT_VALVE21		;!!!!!!!!!!!!!!!!!!
SPI_read_fatal_error:
		cbi	PORTD,BIT_ERR0			;!!!!!!!!!!!!!!!!!!
SPI_set_test_flag:
SPI_erroneous:
	ori	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	rjmp	SPI_quit

SPI_TMS_powerdown:
		cbi	PORTD,BIT_VALVE22		;!!!!!!!!!!!!!!!!!!
	in	Tmp0,SPDR
	mov	MeasurementFlagTimer,Tmp0
	cbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
	ldi	Tmp0,WHITE_30_MKS
SPI_wait_30_mks:
	dec	Tmp0
	brne	SPI_wait_30_mks
	cbi	PORTC,BIT_CLK_STOP
	cbr	InternalFlags,(1<<ACTIVE_TMS_FLAG)
	cbr	SystemFlags,(1<<MEASUREMENT_FLAG)
	ldi	Tmp0,WHITE_8_MKS
SPI_wait_8_mks:
	dec	Tmp0
	brne	SPI_wait_8_mks
	cbi	PORTC,BIT_GEN_STOP
	sbi	PORTC,BIT_RS_TMS_I
	ori	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	rjmp	SPI_quit

SPI_read_time_next_byte:
	ldi	r30,LOW(SPI_read_time)
	ldi	r31,HIGH(SPI_read_time)
	add	r30,SPIByteCounter
	adc	r31,Tmp1
	ijmp
SPI_read_time:
	rjmp	SPI_read_time_end
	rjmp	SPI_read_time_hours_byte
	rjmp	SPI_read_time_minutes_byte
	rjmp	SPI_quit
SPI_read_time_end:
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	rjmp	SPI_quit
SPI_read_time_hours_byte:
	out	SPDR,HoursCounter
	rjmp	SPI_quit
SPI_read_time_minutes_byte:
	out	SPDR,MinutesCounter
	rjmp	SPI_quit

SPI_read_serial_number:
	cp	SPIByteCounter,Tmp1
	breq	SPI_reading_ser_num_is_finish
	ldi	XL,LOW(SPISerialNumber+5)
	ldi	XH,HIGH(SPISerialNumber+5)
	sub	XL,SPIByteCounter
	sbc	XH,Tmp1
	ld	Tmp0,X
	out	SPDR,Tmp0
	rjmp	SPI_quit
SPI_reading_ser_num_is_finish:
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	rjmp	SPI_quit

SPI_set_time:
	in	Tmp0,SPDR
	ldi	YL,LOW(SPINewTime+2)
	ldi	YH,HIGH(SPINewTime+2)
	sub	YL,SPIByteCounter
	sbc	YH,Tmp1
	st	Y,Tmp0
	cpse	SPIByteCounter,Tmp1
	rjmp	SPI_quit
	ld	HoursCounter,Y
	ld	MinutesCounter,-Y
	ld	SecondsCounter,-Y
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	rjmp	SPI_quit

SPI_write_serial_number:
	in	Tmp0,SPDR
	ldi	XL,LOW(SPINewSerialNumber+4)
	ldi	XH,HIGH(SPINewSerialNumber+4)
	sub	XL,SPIByteCounter
	sbc	XH,Tmp1
	st	X,Tmp0
	cpse	SPIByteCounter,Tmp1
	rjmp	SPI_quit
	sbi	EECR,EERIE	; Enable interrupt ready EEPROM
	ldi	Tmp0,4+1
	sts	EEPROMByteCounter,Tmp0
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
	rjmp	SPI_quit

SPI_quit:
	reti

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
	mov	ADC_H_pr2,Tmp0
	mov	ADC_L_pr2,Tmp0

ADC_sub_shift_pr0:
	sub 	ADC_L_pr0,ADCShiftPr0		;Substract Low byte
	sbc 	ADC_H_pr0,Tmp0			;Substract With Carry High byte
	brsh	ADC_compare_pr0_pr2
	mov	ADC_H_pr0,Tmp0
	mov	ADC_L_pr0,Tmp0

ADC_compare_pr0_pr2:
	cp	ADC_L_pr0,ADC_L_pr2
	cpc	ADC_H_pr0,ADC_H_pr2
	brsh	ADC_Further
	mov     ADC_L_pr0,ADC_L_pr2
	mov	ADC_H_pr0,ADC_H_pr2

ADC_Further:
	ldi 	Tmp1,HIGH(PRESSURE_15)
	ldi	Tmp2,LOW(PRESSURE_15)	; Compare to PRESSURE_15
	cp 	ADC_L_pr0,Tmp2
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_15

	ldi	Tmp0,THIRTY_SECONDS
	mov     ThirtySecondsTimer,Tmp0

	ldi	Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0

	sbrs    SystemFlags,BAD_SHIFT_FLAG
	rjmp	ADC_dec_TwoMinutesTimer
	cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	sbi     PORTC,BIT_R_BR

ADC_dec_TwoMinutesTimer:
	mov	Tmp0,PreSecondsCounter
	cpi     Tmp0,0
	brne	Press_more_15_flag_check
	dec	TwoMinutesTimer
Press_more_15_flag_check:
	sbrc	InternalFlags,PRESSURE_MORE_15_FLAG
	rjmp	Two_minutes_check
	sbr	InternalFlags,(1<<PRESSURE_MORE_15_FLAG)
	ldi     Tmp0,TWO_MINUTES
	mov	TwoMinutesTimer,Tmp0

Two_minutes_check:
	mov	Tmp0,TwoMinutesTimer
	cpi     Tmp0,0
	brne	Press_more_295_check
	;Write EEPROM
	ldi 	Tmp1,FATAL_ERROR_2S
	rjmp	Fatal_error_occur

Press_more_295_check:
	ldi 	Tmp1,HIGH(PRESSURE_295)
	ldi	Tmp2,LOW(PRESSURE_295)	; Compare to PRESSURE_295
	cp 	ADC_L_pr0,Tmp2
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_295

	lds	Tmp0,Pressure295Timer
	dec	Tmp0
	sts	Pressure295Timer,Tmp0
	cpi	Tmp0,0
	breq	Pressure_more_295

	ldi 	Tmp1,HIGH(PRESSURE_320)
	ldi	Tmp2,LOW(PRESSURE_320)	; Compare to PRESSURE_320
	cp 	ADC_L_pr0,Tmp2
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_320

	lds	Tmp0,Pressure320Timer
	dec	Tmp0
	sts	Pressure320Timer,Tmp0
	cpi	Tmp0,0
	brne	ADC_quit

	;Write EEPROM
	ldi 	Tmp1,FATAL_ERROR_315
	rjmp	Fatal_error_occur

Pressure_more_295:
	;Write EEPROM
	ldi 	Tmp1,FATAL_ERROR_295
	rjmp	Fatal_error_occur

Pressure_less_295:
	ldi	Tmp0,SECOND_15
	sts	Pressure295Timer,Tmp0
	ldi	Tmp0,SECOND_1
	sts	Pressure320Timer,Tmp0
	rjmp 	ADC_quit

Pressure_less_320:
	ldi	Tmp0,SECOND_15
	sts	Pressure320Timer,Tmp0
	rjmp 	ADC_quit

Pressure_less_15:
	cbr     InternalFlags,(1<<PRESSURE_MORE_15_FLAG)
	cbi	PORTB,BIT_BUZZ_AV1
	sbi	PORTB,BIT_BUZZ_AV2
	ldi	Tmp0,0		; Timer1 disable
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK		; Disable Timer interrupts
	andi  	Tmp0,~(1<<TOIE1)
	out	TIMSK,Tmp0
	
ADC_quit:
	reti

Calculat_shift:
	cbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
	ldi	Tmp0,CALC_SHIFT_DELAY
	dec 	CalcShiftTimer
	cp	CalcShiftTimer,Tmp0
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

	ldi	Tmp0,0
	cp	CalcShiftTimer,Tmp0
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
	ldi	Tmp0,PRESSURE_12
	mov	ADCShiftPr2,Tmp0
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
	ldi	Tmp0,PRESSURE_12
	mov	ADCShiftPr0,Tmp0
	sbr	SystemFlags,(1<<BAD_SHIFT_FLAG)

Calc_shift_flag_reset:
	cbr	InternalFlags,(1<<CALK_SHIFT_FLAG)
	rjmp	ADC_quit

Fatal_error_occur:
	ldi 	Tmp0,LOW(FATAL_ERROR_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(FATAL_ERROR_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	out 	EEDR,Tmp1		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE
	sts	FatalError,Tmp1

	sbrs	Tmp1,0
	cbi	PORTD,BIT_ERR0
	sbrs	Tmp1,1
	cbi	PORTD,BIT_ERR1

	cbi	PORTC,BIT_R_BR

	cbi     PORTC,BIT_INT3_OF_TMS
	sbr	SystemFlags,(1<<FATAL_ERROR_FLAG)

	ldi	Tmp0,LOW(TIMER1_4_KHZ)		; Timer1 enable
	out	TCNT1L,Tmp0
	ldi	Tmp0,HIGH(TIMER1_4_KHZ)
	out	TCNT1H,Tmp0
	ldi	Tmp0,TIMER1_PRESCALE_8
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK		; Enable Timer1 interrupts
	ori  	Tmp0,(1<<TOIE1)
	out	TIMSK,Tmp0
	
	sbi     PORTD,BIT_INT3_OF_TMS
	rjmp	ADC_quit


;***********************
EE_rdy:   	; EEPROM Ready Handler
	cli
	lds	Tmp3,EEPROMByteCounter
	dec	Tmp3
	brlo    EE_writing_EEPROM_is_finished
	sts	EEPROMByteCounter,Tmp3

	ldi	Tmp2,0
	ldi	XL,LOW(SPINewSerialNumber+4)
	ldi	XH,HIGH(SPINewSerialNumber+4)
	sub	XL,Tmp3
	sbc	XH,Tmp2

	ldi 	Tmp0,LOW(SERIAL_NUMBER_ADDRESS+4)
	ldi 	Tmp1,HIGH(SERIAL_NUMBER_ADDRESS+4)
	sub	Tmp0,Tmp3
	sbc	Tmp1,Tmp2
	out 	EEARL,Tmp0		;Set EEPROM Address
	out 	EEARH,Tmp1		;Set EEPROM Address
	ld	Tmp2,X
	out 	EEDR,Tmp2		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE

	reti

EE_writing_EEPROM_is_finished:
	ldi	Tmp3,0
	sts	EEPROMByteCounter,Tmp3

	ldi	XL,LOW(SPISerialNumber)
	ldi	XH,HIGH(SPISerialNumber)
	ldi 	ZL,LOW(SERIAL_NUMBER_ADDRESS)
	ldi 	ZH,HIGH(SERIAL_NUMBER_ADDRESS)

	out 	EEARL,ZL		;Set EEPROM Address
	out 	EEARH,ZH		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	st	X+,Tmp0

	adiw	ZL,1
	out 	EEARL,ZL		;Set EEPROM Address
	out 	EEARH,ZH		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	st	X+,Tmp0

	adiw	ZL,1
	out 	EEARL,ZL		;Set EEPROM Address
	out 	EEARH,ZH		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	st	X+,Tmp0

	adiw	ZL,1
	out 	EEARL,ZL		;Set EEPROM Address
	out 	EEARH,ZH		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	st	X+,Tmp0

	adiw	ZL,1
	out 	EEARL,ZL		;Set EEPROM Address
	out 	EEARH,ZH		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	st	X,Tmp0

	cbi	EECR,EERIE	; Disable interrupt ready EEPROM
	reti





