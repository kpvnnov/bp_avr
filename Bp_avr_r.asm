;$Id: Bp_avr_r.asm,v 1.2 2003-07-10 08:09:55 zykov Exp $
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
		.equ 	ACTIVE_TMS_FLAG=0               ; 1
		.equ    CALK_SHIFT_FLAG=1               ; 0
		.equ	PRESSURE_MORE_15_FLAG=2      	; 0
	     	.equ	BUZZER_FLAG=3                   ; 0
		.equ	FIRST_BYTE_OF_SPI=4		; 1
		.equ	EXTERNAL_BUZZER_FLAG=5		; 0
		.equ	LAST_STAT_KEY=6			; 0
		.equ	LAST_STAT_C5VA=7		; 1
	.def 	SystemFlags=r21                         ;
	     	.equ	SYSTEM_RS_FLAG=0                ; 1
	     	.equ	LOW_BATTERY_FLAG=1              ; 0
	     	.equ	SECOND_FLAG=2                   ; 0
	     	.equ	THIRTY_SECONDS_FLAG=3           ; 0
	     	.equ	FATAL_ERROR_FLAG=4  		; 0
		.equ	BAD_SHIFT_FLAG=5                ; 0
		.equ	TEST_FLAG=6                     ; 0
		.equ	KEY_IS_PRESSED=7		; 0
	.def	ThirtySecondsTimer=r22
	.def	Flag15mmTimer=r23

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
	.equ	USBR_115=1
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
  		.equ	BIT_WDI=1
  		.equ	BIT_CLK_STOP=2
  		.equ	BIT_INT3_OF_TMS=3
  		.equ	BIT_R_BR=4
  		.equ	BIT_CLK_32768=5

	.equ	IO_PIN_PORTC_FOR_RS=0b00111111
	.equ	PORTC_FOR_TMS_RS_CLK_STOP=0b00011001
	.equ	PORTC_FOR_TMS_RS_CLK_RUN=0b00011101
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
	.equ	PORTD_FOR_RS=0b11001101

	; SRAM
	.equ 	PressureShiftPr0=0x61
	.equ 	PressureShiftHighPr0=0x62
	.equ 	PressureShiftPr2=0x63
	.equ 	PressureShiftHighPr2=0x64
	.equ	Pressure295Timer=0x65
	.equ	Pressure320Timer=0x66
	.equ	FirstThreshOfMaxPressL=0x67
	.equ	FirstThreshOfMaxPressH=0x68
	.equ	SecondThreshOfMaxPressL=0x69
	.equ	SecondThreshOfMaxPressH=0x6A
	.equ    EEPROMByteCounter=0x6B
	.equ    FatalError=0x6C
	.equ	ChildModeWord=0x6D
	.equ	BuzzerCounter0=0x6E
	.equ	BuzzerCounter1=0x6F
	.equ	StatusIsSaved=0x70
	.equ	SPINewTime=0x72			; 3-ть байт
	.equ	SPIBufferForTms=0x80		; 64-ть байт

	;EEPROM
	.equ	FATAL_ERROR_ADDRESS=0
	.equ	ERR_INT_ADDRESS=1
	.equ	CHILD_MODE_WORD_ADDRESS=2

	; Conctants
	.equ 	PRESSURE_5=0x000F
	.equ 	PRESSURE_10=0x001E
	.equ 	PRESSURE_12=0x0023	;24
	.equ 	PRESSURE_15=0x002B	;2D
	.equ 	PRESSURE_200=0x0249	;378
	.equ 	PRESSURE_210=0x0267	;3C3
	.equ 	PRESSURE_295=0x035F	;378
	.equ 	PRESSURE_320=0x03A8	;3C3
	.equ	DROWN_UP_MODE_CODE=0xA5
	.equ	CHILD_MODE_CODE=0x5A
	.equ	ERROR_TO_CHILD_MODE_CODE=0x55

	.equ	SHIFT_TIMER_MAX_VALUE=8		; 2-е сек
	.equ	CALC_SHIFT_DELAY=4              ; 1-а сек на выч. смещени
	.equ	SECOND_1=4
	.equ	SECOND_15=60
	.equ	THREE_SECOND=12
	.equ    THIRTY_SECONDS=120
	.equ	TWO_MINUTES=120			; 118 !!!!!!!!!!!!!!!!!!!!!!
	.equ	ONE_MINUTES=240
	.equ    TWO_HOURS=120
	.equ	THREE_MINUTES=3
	.equ	FOUR_MINUTES=4

	.equ	WHITE_10_MKS=12
	.equ	WHITE_30_MKS=40
	.equ	WHITE_8_MKS=10
	.equ	TIM0_1_MS=239		 	; 255-16  prescaler=256
	.equ	TIM0_5_MS=175              	; 255-80  prescaler=256
	.equ    TIMER1_4_KHZ=0x0FF8D		; 0x0FFFF-((250*3.6468)/8)
	.equ    TIMER1_PRESCALE_8=2

	.equ	FATAL_ERROR_2S=0
	.equ	FATAL_ERROR_295=1
	.equ	FATAL_ERROR_315=2
	.equ	C5VA_ERROR=4
	.equ	NOT_FATAL_ERROR=0x0FF

	.equ	PRE_SECOND_INITIAL_VALUE=4
	.equ    SECOND_COUNTER_MAX_VALUE=60
	.equ    MINUTES_COUNTER_MAX_VALUE=60
	.equ    HOURS_COUNTER_MAX_VALUE=24
	.equ    MAX_LOW_BATTERY_TIMER=20

	.equ	SIZE_BUFFER_FOR_TMS=7

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
        rjmp 	TIM0_ovf	; Timer0 Overflow Handler
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
Ext_int1:
TIM2_ovf:
EE_rdy:
ADC_int:
SPI_stc:
TIM0_ovf:
TIM1_ovf:
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
	;ldi 	Tmp0,LOW(ERR_INT_ADDRESS)
	;out 	EEARL,Tmp0		;Set EEPROM Address
	;ldi 	Tmp0,HIGH(ERR_INT_ADDRESS)
	;out 	EEARH,Tmp0		;Set EEPROM Address
	;out 	EEDR,Tmp1		;Set Data To be written
	;sbi 	EECR,EEMWE
	;sbi 	EECR,EEWE

	;rjmp 	Idle_mode_start ;!!!!!!!!!!!!!!!!!!!!!!!!!!!!

Reset: 		; Reset Handler
	cli

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
	ldi 	Tmp0,PORTC_FOR_TMS_RS_CLK_RUN	;10.07.2003
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
	out 	TCCR2,Tmp0		;Timer2 0,25 сек

	ldi 	Tmp0,1<<TOIE2     	;Timer2 interrupts enable!
	out 	TIMSK,Tmp0		;Time interrupts disable!

	in	Tmp0,SPSR
	ldi	Tmp0,SPCR_SPI_EN_I_EN	;SPI initialyzation
	out	SPCR,Tmp0
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)

	ldi	Tmp2,0x0FF
Wiat00:
	ldi	Tmp1,1<<BIT_WDI
	in	Tmp0,PORTC
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0
	dec	Tmp2
	brne	Wiat00

	ldi 	Tmp0,USBR_115		;11 - 19200 bps [3.6864 MHz]
	out 	UBRR,Tmp0
	ldi 	Tmp0,USB_TRANSMIT_ONLY
	out 	UCR,Tmp0
	out	UDR,Tmp0
	
Init_white_INT1_1:
	ldi	Tmp1,1<<BIT_WDI
	in	Tmp0,PORTC
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0
	nop
	wdr
	sbis    PIND,BIT_LOW_BATTERY
	rjmp    Init_white_INT1_1
		
	sbr	InternalFlags,(1<<ACTIVE_TMS_FLAG) 	; ACTIVE_TMS_FLAG = 1
	sbi	PORTC,BIT_R_BR                  	; R_BR = 1
	sbi	PORTC,BIT_CLK_STOP			; CLK_STOP = 1
	ldi	Tmp0,WHITE_10_MKS
White_10_mks_0:
	dec	Tmp0
	brne	White_10_mks_0
	cbi	PORTC,BIT_RS_TMS_I
	
Wiat03:	
	ldi	Tmp1,1<<BIT_WDI
	in	Tmp0,PORTC
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0
	ldi	Tmp2,0x0FF
Wiat02:
	dec	Tmp2
	brne	Wiat02
	
	in	Tmp0,PIND
	out	UDR,tmp0
	rjmp    Wiat03


; Дальше программа не идет !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
; Дальше программа не идет !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
; Дальше программа не идет !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!







