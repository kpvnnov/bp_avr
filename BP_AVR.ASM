;$Id: BP_AVR.ASM,v 1.41 2002-09-09 13:00:20 zykov Exp $
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
;  PC1(pin20) - WDI output
;  PC2(pin21) - CLK_STOP output
;  PC3(pin22) - INT3 (TMS) output - запрос обмена у TMS
;  PC4(pin23) - R_BR  output
;  PC5(pin24) - Clk32768 is checked  output
;  PC6(pin25) - CRISTAL_32768 input
;  PC7(pin26) - CRISTAL_32768 output
;
;  PD0(pin 9) - RX (UART input)
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
		.equ	UART_FLAG=6                     ; 0
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
	;.equ	USB_RECEIVE_ONLY=0b10010000
	;.equ	USB_RX_INT_EN_TX_EN=0b10011000	

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
	.equ	PORTC_FOR_TMS_RS_CLK_RUN=0b00011111
	.equ	PORTC_FOR_TMS_RUN=0b00011110

	; Port D
		.equ	BIT_RX_UART=0
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
	.equ	SPIWhiteTimer=0x71
	.equ	SPINewTime=0x72			; 3-и байта
	.equ	TwoMinutesTimerOfError=0x73	; Таймер двухминутной блокировки в случае ошибки
	.equ	CheckVariablesBuffer=0x80	; 15-ть байт
	.equ	SPIBufferForTms=0x90		; 8-мь байт
	.equ	SPIBuffer=0xA0			; 15-ть байт
	
	.equ	CHECK_VARIABLES_BUFFER_SIZE=15
	.equ	SIZE_BUFFER_FOR_TMS=7
	.equ	SIZE_SPI_BUFFER=15-1	; -1 потому что 
					; первый байт посылаем сразу
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
	.equ	ONE_SECOUNDS=1
	.equ	TWO_MINUTES=125			; 120 / 118 !!!!!!!!!!!!!!!!!!!!
	.equ	ONE_MINUTES=240
	.equ    TWO_HOURS=120
	.equ	THREE_MINUTES=3
	.equ	FOUR_MINUTES=4

	.equ	WHITE_10_MKS=12
	.equ	WHITE_30_MKS=40
	.equ	WHITE_100_MKS=150
	.equ	WHITE_8_MKS=10
	.equ	WHITE_2_MKS=2	
	.equ	TIM0_1_MS=239		 	; 255-16  prescaler=256
	.equ	TIM0_5_MS=175              	; 255-80  prescaler=256
	.equ    TIMER1_4_KHZ=0x0FF8D		; 0x0FFFF-((250*3.6468)/8)
	.equ    TIMER1_PRESCALE_8=2
	
	;это для версии с временной блокировкой после фатальной ошибки		
						;БЫЛО
	.equ	FATAL_ERROR_2S=128		;FATAL_ERROR_2S=0
	.equ	FATAL_ERROR_295=129		;FATAL_ERROR_295=1
	.equ	FATAL_ERROR_315=130		;FATAL_ERROR_315=2
	.equ	C5VA_ERROR=132			;C5VA_ERROR=4
	.equ	NOT_FATAL_ERROR=0x0EE

	.equ	PRE_SECOND_INITIAL_VALUE=4
	.equ    SECOND_COUNTER_MAX_VALUE=60
	.equ    MINUTES_COUNTER_MAX_VALUE=60
	.equ    HOURS_COUNTER_MAX_VALUE=24
	.equ    MAX_LOW_BATTERY_TIMER=20

;BYTE_NUMBER mast be even.
	.equ	SYSTEM_STATE_BYTE_NUMBER=4
	.equ	TMS_POWERDOWN_BYTE_NUMBER=2
	.equ	READ_TIME_BYTE_NUMBER=6
	.equ	SEND_ACKNOW_BYTE_NUMBER=2	
	.equ	READ_FATAL_ERROR_BYTE_NUMBER=2
	.equ	READ_BUFFER_TMS_BYTE_NUMBER=SIZE_BUFFER_FOR_TMS+1
	.equ	SET_TIME_BYTE_NUMBER=4
	.equ	WRITE_CHILD_MODE_BYTE_NUMBER=2
	.equ	WRITE_BUFFER_TMS_BYTE_NUMBER=SIZE_BUFFER_FOR_TMS+1
	.equ	SET_BUZZER_BYTE_NUMBER=2
	.equ	READ_VARIABLE_AVR_BYTE_NUMBER=CHECK_VARIABLES_BUFFER_SIZE+1
	.equ	READ_VERSION_BYTE_NUMBER=80
	.equ	ERROR_COMMAND_BYTE_NUMBER=2	
			
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
;*       Program_version
;********************************************************************        
Program_version:
	.db 	"$Revision: 1.41 $ $Date: 2002-09-09 13:00:20 $",0
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
	ldi 	Tmp0,PORTC_FOR_TMS_RS_CLK_RUN
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
	
	ldi	Tmp3,0xFF
Init_white_1:
	ldi	Tmp1,1<<BIT_WDI	
	in	Tmp0,PORTC
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0	
		
	nop
	wdr
	dec	Tmp3
	brne    Init_white_INT1_1
	rjmp    Init_white_1
Init_white_INT1_1:
	in	Tmp0,PORTC
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0
	nop
	wdr
	sbis    PIND,BIT_LOW_BATTERY
	rjmp    Init_white_INT1_1
	
	ldi 	Tmp0,PORTC_FOR_TMS_RUN
	out 	PORTC,Tmp0
	
	ldi 	Tmp0,USBR_115		;1 - 115200 bps [3.6864 MHz]
	out 	UBRR,Tmp0
	ldi 	Tmp0,USB_TRANSMIT_ONLY
	out 	UCR,Tmp0
	out	UDR,Tmp0

	ori 	SystemFlags,(1<<SYSTEM_RS_FLAG)
	ori	InternalFlags,(1<<ACTIVE_TMS_FLAG) ; ACTIVE_TMS_FLAG = 1
	sbr	InternalFlags,(1<<LAST_STAT_C5VA)

	ldi     Tmp0,TWO_MINUTES
	mov	TwoMinutesTimer,Tmp0
	ldi     Tmp0,MAX_LOW_BATTERY_TIMER
	mov	LowBatteryTimer,Tmp0
	ldi     Tmp0,0
	mov	CalcShiftTimer,Tmp0
	ldi     Tmp0,1
	mov	SPIByteCounter,Tmp0
	ldi     Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0
	clr     ThirtySecondsTimer
	
	ldi	Tmp0,SECOND_15
	sts	Pressure295Timer,Tmp0
	ldi	Tmp0,SECOND_1
	sts	Pressure320Timer,Tmp0

	;Read child mode
	ldi 	Tmp0,LOW(CHILD_MODE_WORD_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(CHILD_MODE_WORD_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	sts	ChildModeWord,Tmp0
	cpi	Tmp0,DROWN_UP_MODE_CODE
	brne	Init_Child_Mode

	ldi 	Tmp0,HIGH(PRESSURE_295)
	sts	FirstThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_295)
	sts	FirstThreshOfMaxPressL,Tmp0

	ldi 	Tmp0,HIGH(PRESSURE_320)
	sts	SecondThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_320)
	sts	SecondThreshOfMaxPressL,Tmp0
	rjmp	Init_error_check

Init_Child_Mode:
	ldi 	Tmp0,HIGH(PRESSURE_200)
	sts	FirstThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_200)
	sts	FirstThreshOfMaxPressL,Tmp0

	ldi 	Tmp0,HIGH(PRESSURE_210)
	sts	SecondThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_210)
	sts	SecondThreshOfMaxPressL,Tmp0
	
	lds	Tmp0,ChildModeWord
	cpi	Tmp0,CHILD_MODE_CODE
	breq	Init_error_check
	ldi	Tmp0,ERROR_TO_CHILD_MODE_CODE
	sts	ChildModeWord,Tmp0
	
Init_error_check:
	;sbi	PORTD,BIT_ERR0
	;sbi	PORTD,BIT_ERR1
		;cbi	PORTD,BIT_ERR1			;!!!!!!!!!!!!!!!!!!
		;cbi	PORTD,BIT_ERR0			;!!!!!!!!!!!!!!!!!!
		;cbi	PORTD,BIT_VALVE21		;!!!!!!!!!!!!!!!!!!
		;cbi	PORTD,BIT_VALVE22		;!!!!!!!!!!!!!!!!!!

	;Read Fatal error from EEPROM and write to PORTD
	ldi 	Tmp0,LOW(FATAL_ERROR_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(FATAL_ERROR_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		;Tmp0 = EEPROM Value
	sts	FatalError,Tmp0

	;sbrs	Tmp0,0
	;cbi	PORTD,BIT_ERR0
	;sbrs	Tmp0,1
	;cbi	PORTD,BIT_ERR1

	cpi	Tmp0,NOT_FATAL_ERROR
	brne	Fatal_error_was 	;If was Fatal Error.

	sbic    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

;*****

Fatal_error_was:

	sbr	SystemFlags,(1<<FATAL_ERROR_FLAG)
	ldi     Tmp0,ONE_SECOUNDS
	sts 	TwoMinutesTimerOfError,Tmp0		

	sbic    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

	ldi 	Tmp0,TIM0_PRESCALER_1024
	out 	TCCR0,Tmp0		;Run Timer0 (Prescaler=1024)
	ldi	Tmp1,200		; 14 сек.
Wait_key_1:
	ldi	Tmp3,1<<BIT_WDI
	in	Tmp2,PORTC
	eor	Tmp2,Tmp3
	out	PORTC,Tmp2
	in	Tmp0,TIFR
	sbrs    Tmp0,TOV0
	rjmp	Wait_key_1

	in	Tmp0,TIFR
	out     TIFR,Tmp0

	sbic    PINB,BIT_KEY_SIGNAL
	rjmp 	Idle_mode_start

	dec	Tmp1
	brne	Wait_key_1

	ldi	Tmp1,71		; 5 сек.
Wait_key_2:
	ldi	Tmp3,1<<BIT_WDI
	in	Tmp2,PORTC
	eor	Tmp2,Tmp3
	out	PORTC,Tmp2
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

	;sbi	PORTD,BIT_ERR0
	;sbi	PORTD,BIT_ERR1

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
	out 	OCR2,Tmp0		;Timer2 0,25 сек
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
;	nop
;	nop
;
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
	
Idle_mode_white:
	sei
	ldi	Tmp0,255
Idle_mode_white_2:	
	sbic	PINB,BIT_SPI_SS
	rjmp	Idle_mode_check_ss
	dec	Tmp0
	brne	Idle_mode_white_2
	nop
	
	ldi	Tmp0,255
Idle_mode_white_3:	
	sbic	PINB,BIT_SPI_SS
	rjmp	Idle_mode_check_ss
	dec	Tmp0
	brne	Idle_mode_white_3
	nop
		
	cli
	lds	Tmp0,SPIWhiteTimer
	dec	Tmp0
	sts	SPIWhiteTimer,Tmp0
	cpi	Tmp0,0
	brne	Idle_mode_check_ss
	ldi	Tmp0,255
	sts	SPIWhiteTimer,Tmp0
	nop				; Время ожидания конца обмена по SPI > 64 ms
Idle_mode_check_ss:
	cli
	sbis	PINB,BIT_SPI_SS
	rjmp	Idle_mode_white
	ori	InternalFlags,1<<FIRST_BYTE_OF_SPI
	ldi	Tmp0,255
	sts	SPIWhiteTimer,Tmp0

;	wdr	; !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	lds	Tmp0,EEPROMByteCounter
	cpi	Tmp0,0
	brne	Idle_mode_not_check_eeprom
	sbic	EECR,EEWE
	;rjmp	Idle_mode_not_check_eeprom
	sbi	EECR,EERIE		; Enable interrupt ready EEPROM	
Idle_mode_not_check_eeprom:	
	nop
	cli
	sbrs 	InternalFlags,ACTIVE_TMS_FLAG
	rjmp    Power_save_mode_start
	rjmp    Idle_mode_start

;********************************************************************

Ext_int1:               ;Low battery

	push 	Tmp0

	cbi	PORTC,BIT_R_BR
	sbi	PORTC,BIT_R_BR
	sbr	SystemFlags,(1<<LOW_BATTERY_FLAG)
	nop
	nop

	pop 	Tmp0
	reti

;********************************************************************

TIM2_ovf: 	; Timer2 Overflow Handler

	wdr     ; Watchdog reset
		; System watch
	cbi     PORTC,BIT_WDI
	
	sbic    PIND,BIT_RX_UART		
	sbr	SystemFlags,(1<<UART_FLAG)
	
;	ldi	Tmp1,1<<BIT_WDI
;	in	Tmp0,PORTC
;	eor	Tmp0,Tmp1
;	out	PORTC,Tmp0

	sbrs	SystemFlags,SECOND_FLAG
	rjmp	TIM2_C_5VA_check
	lds	Tmp0,TwoMinutesTimerOfError		;Удержание Сброса 2-е минуты при ФАТАЛЬНОЙ ошибке
	dec	Tmp0
	sts	TwoMinutesTimerOfError,Tmp0
	cpi	Tmp0,0	
	brne	TIM2_C_5VA_check
	cbr	SystemFlags,(1<<FATAL_ERROR_FLAG)	;!!!!!!!!!!!!!!!!!!!!!!
	
TIM2_C_5VA_check:
	in	Tmp2,PIND
	sbrs	Tmp2,BIT_C_5VA
	rjmp	TIM2_change_flag
	sbrc	InternalFlags,LAST_STAT_C5VA
	rjmp	TIM2_change_flag
	ldi	Tmp0,THIRTY_SECONDS
	mov     ThirtySecondsTimer,Tmp0
	sbr	SystemFlags,(1<<THIRTY_SECONDS_FLAG)

TIM2_change_flag:
	cbr	InternalFlags,(1<<LAST_STAT_C5VA)
	sbrc	Tmp2,BIT_C_5VA
	sbr	InternalFlags,(1<<LAST_STAT_C5VA)
	
	sbic	PINB,BIT_SPI_SS
	ori	InternalFlags,1<<FIRST_BYTE_OF_SPI

	cbr  	SystemFlags,(1<<SECOND_FLAG)
	inc	PreSecondsCounter
	ldi	Tmp0,PRE_SECOND_INITIAL_VALUE
	cp	PreSecondsCounter,Tmp0
	brne    Timer2_pin_low_battery_question
	clr	PreSecondsCounter		; Секунда	
	sbr  	SystemFlags,(1<<SECOND_FLAG)
	sbi     PORTC,BIT_R_BR			; Отпустить R_BR

	ldi	Tmp0,0
	cp	LowBatteryTimer,Tmp0
	breq	Timer2_second_inc
	
	push	Tmp3	
	rcall	Tms_to_active
	pop	Tmp3
	
Timer2_second_inc:	
	ldi	Tmp1,1<<BIT_CLK_32768
	in	Tmp0,PORTC
	eor	Tmp0,Tmp1
	out	PORTC,Tmp0

	inc     SecondsCounter
	ldi	Tmp0,SECOND_COUNTER_MAX_VALUE
	cp	SecondsCounter,Tmp0
	brne    Timer2_pin_low_battery_question
	clr	SecondsCounter

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

	sbic    PIND,BIT_LOW_BATTERY		; Check low battery
	rjmp    Timer2_max_low_battery_timer
	ldi	Tmp0,0
	cpse	LowBatteryTimer,Tmp0
	dec	LowBatteryTimer
	rjmp	Timer2_key_question	

Timer2_max_low_battery_timer:
	ldi	Tmp0,MAX_LOW_BATTERY_TIMER     		; Set LowBatteryTimer=max
	mov	LowBatteryTimer,Tmp0

Timer2_key_question:
	in	Tmp2,PINB
	sbrc	Tmp2,BIT_KEY_SIGNAL
	rjmp	Timer2_key_1
		
	sbrs	InternalFlags,ACTIVE_TMS_FLAG		; Key is press
	rjmp	Timer2_key_0_active_tms_0

	sbrs	InternalFlags,PRESSURE_MORE_15_FLAG
	rjmp	Timer2_key_0_active_tms_0

	cbi     PORTC,BIT_R_BR 				; Impulse R_BR
	sbi     PORTC,BIT_R_BR
	rjmp	Timer2_key_1

Timer2_key_0_active_tms_0:
	sbrs	InternalFlags,LAST_STAT_KEY
	rjmp	Timer2_key_1
	sbr	SystemFlags,(1<<KEY_IS_PRESSED)
	ldi	Tmp0,MAX_LOW_BATTERY_TIMER     		; Set LowBatteryTimer=max
	mov	LowBatteryTimer,Tmp0	

Timer2_key_1:
	cbr	InternalFlags,(1<<LAST_STAT_KEY)
	sbrc	Tmp2,BIT_KEY_SIGNAL
	sbr	InternalFlags,(1<<LAST_STAT_KEY)
	;out	UDR,SystemFlags				;!!!!!!!!!!!!!!!!!!
	;out	UDR,ThirtySecondsTimer			;!!!!!!!!!!!!!!!!!!
	;out	UDR,InternalFlags				;!!!!!!!!!!!!!!!!!!
	sbrs	SystemFlags,THIRTY_SECONDS_FLAG
	rjmp    Timer2_FATAL_ERROR_FLAG_check
	cpi	ThirtySecondsTimer,0
	breq    Timer2_zero_to_30SecTime
	dec     ThirtySecondsTimer
	sbic	PIND,BIT_C_5VA
	rjmp    Timer2_FATAL_ERROR_FLAG_check
	cbi     PORTC,BIT_R_BR 				; Impulse R_BR
	rjmp	Timer2_FATAL_ERROR_FLAG_check

Timer2_zero_to_30SecTime:
	clr     ThirtySecondsTimer
	cbr	SystemFlags,(1<<THIRTY_SECONDS_FLAG)

Timer2_FATAL_ERROR_FLAG_check:
	sbrs	SystemFlags,FATAL_ERROR_FLAG
	rjmp	Timer2_ACTIVE_TMS_FLAG_check
	cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	
Timer2_ACTIVE_TMS_FLAG_check:
	sbrs	InternalFlags,ACTIVE_TMS_FLAG
	rjmp	Timer2_quit_

Timer2_C5VA_question:
	sbic	PIND,BIT_C_5VA
	rjmp	Timer2_bit_c5va_1

	dec	OneMinutesTimer
	brne    Timer2_bit_c5va_0
	ldi	Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0

	ldi 	Tmp1,C5VA_ERROR
	rjmp	Fatal_error_occur

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
	cbr     InternalFlags,(1<<PRESSURE_MORE_15_FLAG)
	cbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
	;cbi     PORTC,BIT_R_BR 		; Impulse R_BR
	;sbi     PORTC,BIT_R_BR

	ldi 	Tmp0,0
	sts 	PressureShiftPr0,Tmp0
	sts 	PressureShiftHighPr0,Tmp0
	sts 	PressureShiftPr2,Tmp0
	sts 	PressureShiftHighPr2,Tmp0

	ldi     Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0
	ldi     Tmp0,TWO_MINUTES
	mov	TwoMinutesTimer,Tmp0
	
Timer2_quit:
	sbi     PORTC,BIT_WDI
	reti

;*******************************************************************
TIM1_ovf: 	; Timer1 Overflow Handler
	lds	Tmp0,BuzzerCounter0
	dec	Tmp0
	sts	BuzzerCounter0,Tmp0
	cpi	Tmp0,0
	brne	TIM1_buzz_is_enabled
	lds	Tmp0,BuzzerCounter1
	dec	Tmp0
	sts	BuzzerCounter1,Tmp0
	cpi	Tmp0,0
	brne	TIM1_buzz_is_enabled
	sbrs	InternalFlags,EXTERNAL_BUZZER_FLAG
	rjmp	TIM1_buzz_is_enabled
TIM1_buzz_is_stopped:				; if EXTERNAL_BUZZER_FLAG=1
	cbr	InternalFlags,(1<<EXTERNAL_BUZZER_FLAG)
	cbi	PORTB,BIT_BUZZ_AV1
	sbi	PORTB,BIT_BUZZ_AV2
	ldi	Tmp0,0				; Timer1 disable
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK			; Disable Timer interrupts
	andi  	Tmp0,~(1<<TOIE1)
	out	TIMSK,Tmp0
	reti

TIM1_buzz_is_enabled:
	ldi	Tmp0,LOW(TIMER1_4_KHZ)		; Timer1 enable
	out	TCNT1L,Tmp0
	ldi	Tmp0,HIGH(TIMER1_4_KHZ)
	out	TCNT1H,Tmp0

	sbic	PINB,BIT_BUZZ_AV1
	rjmp	TIM1_buzz_av1_0
	sbi	PORTB,BIT_BUZZ_AV1
	sbi	PORTB,BIT_BUZZ_AV2
	reti
TIM1_buzz_av1_0:
	cbi	PORTB,BIT_BUZZ_AV1
	cbi	PORTB,BIT_BUZZ_AV2
	reti
;*******************************************************************

TIM0_ovf:		;Timer0 interrupt

;	push 	Tmp0
;	pop 	Tmp0
	reti

;*******************************************************************

SPI_stc: 	; SPI Transfer Complete Handler

		;sbi	PORTD,BIT_ERR1			;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	nop
	in	Tmp0,SPDR
	sbrs	InternalFlags,FIRST_BYTE_OF_SPI
	rjmp	SPI_second_byte
	mov     Tmp1,Tmp0
	swap	Tmp1
	eor	Tmp1,Tmp0
	breq	SPI_good_command
SPI_bad_command:
SPI_erroneous_command:
	ldi	Tmp1,0x0F0		;was 0xEE
	out	SPDR,Tmp1
	ldi	Tmp1,ERROR_COMMAND_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	ldi    	Tmp0,0x0F
	mov	SPICommand,Tmp0
	andi	InternalFlags,~(1<<FIRST_BYTE_OF_SPI)
		;sbi	PORTD,BIT_ERR0			;!!!!!!!!!!!!!!!!!!
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
	rjmp	SPI_send_acknowledge_command
	rjmp	SPI_read_fatal_error_command
	rjmp	SPI_read_buffer_tms_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_set_time_command
	rjmp	SPI_write_child_mode_command
	rjmp	SPI_write_buffer_tms_command
	rjmp	SPI_set_buzzer_command
	rjmp	SPI_read_variable_command
	rjmp	SPI_read_version_command
	rjmp	SPI_erroneous_command
	rjmp	SPI_erroneous_command

SPI_system_stat_command:
	out	SPDR,SystemFlags
	sts	SPIBuffer+SIZE_SPI_BUFFER-1,SystemFlags
	lds	Tmp0,ChildModeWord
	sts	SPIBuffer+SIZE_SPI_BUFFER-2,Tmp0
	ldi	Tmp1,SYSTEM_STATE_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	cbr	SystemFlags,(1<<UART_FLAG)
	rjmp	SPI_quit

SPI_TMS_powerdown_command:
	ldi	Tmp1,0xEE
	out	SPDR,Tmp1
	ldi	Tmp1,TMS_POWERDOWN_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_read_time_command:
	ldi	Tmp0,TWO_MINUTES
	sub	Tmp0,TwoMinutesTimer
	out	SPDR,Tmp0
	ldi	Tmp0,0xEE
	sts	SPIBuffer+SIZE_SPI_BUFFER-1,Tmp0
	sts	SPIBuffer+SIZE_SPI_BUFFER-2,HoursCounter
	sts	SPIBuffer+SIZE_SPI_BUFFER-3,MinutesCounter
	sts	SPIBuffer+SIZE_SPI_BUFFER-4,SecondsCounter
	ldi	Tmp1,READ_TIME_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_send_acknowledge_command:
	ldi	Tmp1,0xEE
	out	SPDR,Tmp1
	ldi	Tmp1,SEND_ACKNOW_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	ldi	Tmp1,0xFF
	lds	Tmp0,StatusIsSaved
	andi	Tmp0,((1<<KEY_IS_PRESSED)|(1<<SYSTEM_RS_FLAG))
	eor	Tmp0,Tmp1
	and	SystemFlags,Tmp0
	rjmp	SPI_quit

SPI_read_fatal_error_command:
	lds	Tmp0,FatalError
	out	SPDR,Tmp0
	ldi	Tmp1,READ_FATAL_ERROR_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_read_buffer_tms_command:
	lds	Tmp1,SPIBufferForTms
	out	SPDR,Tmp1
	ldi	Tmp1,READ_BUFFER_TMS_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_set_time_command:
	ldi	Tmp1,0x88
	out	SPDR,Tmp1
	ldi	Tmp1,SET_TIME_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_write_child_mode_command:
	ldi	Tmp1,0x99
	out	SPDR,Tmp1
	ldi	Tmp1,WRITE_CHILD_MODE_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_write_buffer_tms_command:
	ldi	Tmp1,0xAA
	out	SPDR,Tmp1
	ldi	Tmp1,WRITE_BUFFER_TMS_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_set_buzzer_command:
	ldi	Tmp1,0xBB
	out	SPDR,Tmp1
	ldi	Tmp1,SET_BUZZER_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit

SPI_read_variable_command:
	ldi	Tmp0,TWO_MINUTES
	sub	Tmp0,TwoMinutesTimer
	out	SPDR,Tmp0
	sts	SPIBuffer+8,HoursCounter
	sts	SPIBuffer+7,MinutesCounter
	sts	SPIBuffer+6,SecondsCounter
	sts	SPIBuffer+5,ADCShiftPr2
	sts	SPIBuffer+4,ADCShiftPr0	
	sts	SPIBuffer+3,ADC_L_pr2
	sts	SPIBuffer+2,ADC_H_pr2
	sts	SPIBuffer+1,ADC_L_pr0
	sts	SPIBuffer+0,ADC_H_pr0
	ldi	Tmp1,READ_VARIABLE_AVR_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit
	
SPI_read_version_command:
	mov	Tmp0,r0
	ldi	ZL,LOW(Program_version*2)
	ldi	ZH,HIGH(Program_version*2)
	lpm
	out	SPDR,r0
	mov	r0,Tmp0
	ldi	Tmp1,READ_VERSION_BYTE_NUMBER-1
	mov 	SPIByteCounter,Tmp1
	rjmp	SPI_quit


SPI_second_byte:
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
	rjmp	SPI_send_acknowledgement
	rjmp	SPI_read_fatal_error
	rjmp	SPI_read_buffer_tms
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous
	rjmp	SPI_set_time
	rjmp	SPI_write_child_mode
	rjmp	SPI_write_buffer_tms
	rjmp	SPI_set_buzzer
	rjmp	SPI_read_variable
	rjmp	SPI_read_version
	rjmp	SPI_erroneous
	rjmp	SPI_erroneous

SPI_system_stat:
SPI_read_time_next_byte:
SPI_read_variable:
	cp	SPIByteCounter,Tmp1
	breq	SPI_quit_and_flag_is_set_up_0
	ldi	XL,LOW(SPIBuffer+SIZE_SPI_BUFFER)
	ldi	XH,HIGH(SPIBuffer+SIZE_SPI_BUFFER)
	sub	XL,SPIByteCounter
	sbc	XH,Tmp1
	ld	Tmp0,X
	out	SPDR,Tmp0
	rjmp	SPI_quit
SPI_quit_and_flag_is_set_up_0:
	rjmp	SPI_quit_and_flag_is_set_up
	
SPI_TMS_powerdown:
		;cbi	PORTD,BIT_VALVE22		;!!!!!!!!!!!!!!!!!!
	cbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
	ldi	Tmp0,WHITE_100_MKS
SPI_wait_100_mks:
	dec	Tmp0
	brne	SPI_wait_100_mks
	cbi	PORTC,BIT_CLK_STOP			;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	cbr	InternalFlags,(1<<ACTIVE_TMS_FLAG)	;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
;	ldi	Tmp0,WHITE_8_MKS
;SPI_wait_8_mks:
;	dec	Tmp0
;	brne	SPI_wait_8_mks
;	sbi	PORTC,BIT_RS_TMS_I			;!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	rjmp	SPI_quit_and_flag_is_set_up

SPI_read_buffer_tms:
	cp	SPIByteCounter,Tmp1
	breq	SPI_quit_and_flag_is_set_up_0
	ldi	XL,LOW(SPIBufferForTms+SIZE_BUFFER_FOR_TMS-1)
	ldi	XH,HIGH(SPIBufferForTms+SIZE_BUFFER_FOR_TMS-1)
	sub	XL,SPIByteCounter
	sbc	XH,Tmp1
	ld	Tmp0,X
	out	SPDR,Tmp0
	rjmp	SPI_quit

SPI_set_time:
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
	rjmp	SPI_quit_and_flag_is_set_up

SPI_write_child_mode:

		;cbi	PORTD,BIT_ERR0		;!!!!!!!!!!!!!!!!!!!!
		;cbi	PORTD,BIT_ERR1		;!!!!!!!!!!!!!!!!!!!!
	sts	ChildModeWord,Tmp0
	cpi	Tmp0,DROWN_UP_MODE_CODE
	breq	SPI_write_drown_up_mode
	ldi	Tmp0,CHILD_MODE_CODE
	sts	ChildModeWord,Tmp0
SPI_write_drown_up_mode:
	sbi	EECR,EERIE	; Enable interrupt ready EEPROM
	ldi	Tmp0,1
	sts	EEPROMByteCounter,Tmp0
	rjmp	SPI_quit_and_flag_is_set_up

SPI_write_buffer_tms:
	ldi	XL,LOW(SPIBufferForTms+SIZE_BUFFER_FOR_TMS-1)
	ldi	XH,HIGH(SPIBufferForTms+SIZE_BUFFER_FOR_TMS-1)
	sub	XL,SPIByteCounter
	sbc	XH,Tmp1
	st	X,Tmp0
	cpse	SPIByteCounter,Tmp1
	rjmp	SPI_quit
		;ldi	Tmp1,1<<BIT_ERR1		;!!!!!!!!!!!!!!!!!!
		;in	Tmp0,PORTD			;!!!!!!!!!!!!!!!!!!
		;eor	Tmp0,Tmp1			;!!!!!!!!!!!!!!!!!!
		;out	PORTD,Tmp0			;!!!!!!!!!!!!!!!!!!
	rjmp	SPI_quit_and_flag_is_set_up

SPI_set_buzzer:
	sts	BuzzerCounter1,Tmp0
	sbr	InternalFlags,(1<<EXTERNAL_BUZZER_FLAG)

	ldi	Tmp0,LOW(TIMER1_4_KHZ)		; Timer1 enable
	out	TCNT1L,Tmp0
	ldi	Tmp0,HIGH(TIMER1_4_KHZ)
	out	TCNT1H,Tmp0
	ldi	Tmp0,TIMER1_PRESCALE_8
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK		; Enable Timer1 interrupts
	ori  	Tmp0,(1<<TOIE1)
	out	TIMSK,Tmp0
	rjmp	SPI_quit_and_flag_is_set_up
	
SPI_read_version:
	cp	SPIByteCounter,Tmp1
	breq	SPI_quit_and_flag_is_set_up	
	mov	Tmp0,r0
	ldi	ZL,LOW(Program_version*2+READ_VERSION_BYTE_NUMBER-1)
	ldi	ZH,HIGH(Program_version*2+READ_VERSION_BYTE_NUMBER-1)
	sub	ZL,SPIByteCounter
	sbc	ZH,Tmp1
	lpm
	out	SPDR,r0
	mov	r0,Tmp0
	rjmp	SPI_quit
	
SPI_erroneous:
	sbic	PINB,BIT_SPI_SS
	rjmp	SPI_quit_and_flag_is_set_up
	ldi	Tmp1,0x0F0
	out	SPDR,Tmp1
	rjmp	SPI_quit
SPI_read_fatal_error:
		;cbi	PORTD,BIT_ERR0			;!!!!!!!!!!!!!!!!!!
SPI_send_acknowledgement:
SPI_quit_and_flag_is_set_up:
	sbr	InternalFlags,(1<<FIRST_BYTE_OF_SPI)
SPI_quit:
	ldi	Tmp0,WHITE_10_MKS*2
SPI_quit_white:
	reti

;********************************************************************
Tms_to_active:
	sbr	InternalFlags,(1<<ACTIVE_TMS_FLAG) 	; ACTIVE_TMS_FLAG = 1	
	sbi	PORTC,BIT_CLK_STOP			; CLK_STOP = 1
	ldi	Tmp0,WHITE_10_MKS
Tms_to_active_white_1:
	dec	Tmp0
	brne	Tms_to_active_white_1
	cbi	PORTC,BIT_RS_TMS_I
	ldi	Tmp0,WHITE_8_MKS
Tms_to_active_white_2:
	dec	Tmp0
	brne	Tms_to_active_white_2
	cbi     PORTC,BIT_INT3_OF_TMS
	ldi	Tmp0,WHITE_2_MKS
Tms_to_active_white_3:
	dec	Tmp0
	brne	Tms_to_active_white_3
	sbi     PORTC,BIT_INT3_OF_TMS
	ret
	
;********************************************************************
ADC_int:

	in 	ADC_L_pr0,ADCL		; r9:r10 = ADC Value
	in 	ADC_H_pr0,ADCH
	in 	Tmp0,ADCSR              ; ADC flag reset
	out 	ADCSR,Tmp0
	ldi 	Tmp0,0b00000000		; Prescaler=64, Int Enable
	out 	ADCSR,Tmp0

	in	Tmp0,ADMUX
	cpi	Tmp0,ADM_PR0
	breq    ADC_calc_shift_flag_question
ADC_next_channel:

	mov	ADC_L_pr2,ADC_L_pr0
	mov	ADC_H_pr2,ADC_H_pr0
	ldi 	Tmp0,0b10000110		;Prescaler=64, Int Enable
	out 	ADCSR,Tmp0
	ldi 	Tmp0,ADM_PR0		; ADCMUX = Pr0
	out 	ADMUX,Tmp0
	sbi 	ADCSR,ADSC		;Start ADC Conversion
	sbi	ADCSR,ADIE		;Enable ADC Interrupt
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
	;out	UDR,ADC_L_pr0			;!!!!!!!!!!!!!!!!!!
	;out	UDR,ADC_H_pr0			;!!!!!!!!!!!!!!!!!!
	ldi 	Tmp1,HIGH(PRESSURE_15)
	ldi	Tmp2,LOW(PRESSURE_15)	; Compare to PRESSURE_15
	cp 	ADC_L_pr0,Tmp2
	cpc 	ADC_H_pr0,Tmp1
	brlo 	Pressure_less_15_0

		;ldi	Tmp1,1<<BIT_VALVE21
		;in	Tmp0,PORTD		;!!!!!!!!!!!!!!!!!!
		;eor	Tmp0,Tmp1
		;out	PORTD,Tmp0
		
	ldi	Flag15mmTimer,THREE_SECOND
		
	ldi	Tmp0,ONE_MINUTES
	mov	OneMinutesTimer,Tmp0

	sbrs    SystemFlags,BAD_SHIFT_FLAG
	rjmp	ADC_dec_TwoMinutesTimer
	;cbi     PORTC,BIT_R_BR 			; Impulse R_BR
	;sbi     PORTC,BIT_R_BR

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

	;ldi 	Tmp1,HIGH(PRESSURE_295)
	;ldi	Tmp2,LOW(PRESSURE_295)	; Compare to PRESSURE_295
	lds	Tmp1,FirstThreshOfMaxPressL
	lds	Tmp2,FirstThreshOfMaxPressH
	cp 	ADC_L_pr0,Tmp1
	cpc 	ADC_H_pr0,Tmp2
	brlo 	Pressure_less_295

	lds	Tmp0,Pressure295Timer
	dec	Tmp0
	sts	Pressure295Timer,Tmp0
	cpi	Tmp0,0
	breq	Pressure_more_295

	;ldi 	Tmp1,HIGH(PRESSURE_320)
	;ldi	Tmp2,LOW(PRESSURE_320)	; Compare to PRESSURE_320
	lds	Tmp1,SecondThreshOfMaxPressL
	lds	Tmp2,SecondThreshOfMaxPressH
	cp 	ADC_L_pr0,Tmp1
	cpc 	ADC_H_pr0,Tmp2
	brlo 	Pressure_less_320

	lds	Tmp0,Pressure320Timer
	dec	Tmp0
	sts	Pressure320Timer,Tmp0
	cpi	Tmp0,0
	brne	ADC_quit

	;Write EEPROM
	ldi	Tmp0,0
	sts	Pressure320Timer,Tmp0
	ldi 	Tmp1,FATAL_ERROR_315
	rjmp	Fatal_error_occur

Pressure_less_15_0:
	rjmp	Pressure_less_15

Pressure_more_295:
	;Write EEPROM
	ldi	Tmp0,0
	sts	Pressure295Timer,Tmp0
	ldi 	Tmp1,FATAL_ERROR_295
	rjmp	Fatal_error_occur

Pressure_less_295:
	ldi	Tmp0,SECOND_15
	sts	Pressure295Timer,Tmp0
	ldi	Tmp0,SECOND_1
	sts	Pressure320Timer,Tmp0
	rjmp 	ADC_quit

Pressure_less_320:
	ldi	Tmp0,SECOND_1
	sts	Pressure320Timer,Tmp0
	rjmp 	ADC_quit

Pressure_less_15:
	cpi	Flag15mmTimer,0
	brne	ADC_dec_Flag15mmTimer
	sbrs	InternalFlags,PRESSURE_MORE_15_FLAG
	rjmp	Pressure_less_15_flag_0
	cbr     InternalFlags,(1<<PRESSURE_MORE_15_FLAG)
	ldi	Tmp0,THIRTY_SECONDS
	mov     ThirtySecondsTimer,Tmp0
	sbr	SystemFlags,(1<<THIRTY_SECONDS_FLAG)
		
Pressure_less_15_flag_0:
	cbi	PORTB,BIT_BUZZ_AV1
	sbi	PORTB,BIT_BUZZ_AV2
	ldi	Tmp0,0		; Timer1 disable
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK		; Disable Timer interrupts
	andi  	Tmp0,~(1<<TOIE1)
	out	TIMSK,Tmp0

ADC_quit:
	reti
	
ADC_dec_Flag15mmTimer:
	dec	Flag15mmTimer
	rjmp	ADC_quit
	
Calculat_shift:
	cbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
	ldi	Tmp0,CALC_SHIFT_DELAY
	inc 	CalcShiftTimer
	cp	CalcShiftTimer,Tmp0
	brlo	ADC_quit

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

	ldi	Tmp0,SHIFT_TIMER_MAX_VALUE
	cp	CalcShiftTimer,Tmp0
	brne	ADC_quit

	lds 	Tmp1,PressureShiftPr2
	lds 	Tmp2,PressureShiftHighPr2
	clc
	ror	Tmp2
	ror	Tmp1
	clc
	ror	Tmp2
	ror	Tmp1
	mov	ADCShiftPr2,Tmp1
	ldi 	Tmp0,HIGH(PRESSURE_15)
	cpi 	Tmp1,LOW(PRESSURE_15)	;Compare to PRESSURE 12
	cpc 	Tmp2,Tmp0
	brlo	Calc_shift_pr0
	ldi	Tmp0,LOW(PRESSURE_15)
	mov	ADCShiftPr2,Tmp0
	sbr	SystemFlags,(1<<BAD_SHIFT_FLAG)
Calc_shift_pr0:
	lds 	Tmp1,PressureShiftPr0
	lds 	Tmp2,PressureShiftHighPr0
	clc
	ror	Tmp2
	ror	Tmp1
	clc
	ror	Tmp2
	ror	Tmp1
	mov	ADCShiftPr0,Tmp1
	ldi 	Tmp0,HIGH(PRESSURE_15)
	cpi 	Tmp1,LOW(PRESSURE_15)	;Compare to PRESSURE 12
	cpc 	Tmp2,Tmp0
	brlo	Calc_shift_flag_reset
	ldi	Tmp0,LOW(PRESSURE_15)
	mov	ADCShiftPr0,Tmp0
	sbr	SystemFlags,(1<<BAD_SHIFT_FLAG)

Calc_shift_flag_reset:
	cbr	InternalFlags,(1<<CALK_SHIFT_FLAG)
	clr	CalcShiftTimer
	rjmp	ADC_quit

Fatal_error_occur:
	sbrc	SystemFlags,FATAL_ERROR_FLAG
	rjmp	ADC_quit
	ldi 	Tmp0,LOW(FATAL_ERROR_ADDRESS)
	out 	EEARL,Tmp0		;Set EEPROM Address
	ldi 	Tmp0,HIGH(FATAL_ERROR_ADDRESS)
	out 	EEARH,Tmp0		;Set EEPROM Address
	out 	EEDR,Tmp1		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE
	sts	FatalError,Tmp1

	;sbrs	Tmp1,0
	;cbi	PORTD,BIT_ERR0
	;sbrs	Tmp1,1
	;cbi	PORTD,BIT_ERR1

	cbi	PORTC,BIT_R_BR

	sbr	SystemFlags,(1<<FATAL_ERROR_FLAG)
	ldi     Tmp0,TWO_MINUTES
	sts 	TwoMinutesTimerOfError,Tmp0	

	ldi	Tmp0,LOW(TIMER1_4_KHZ)		; Timer1 enable
	out	TCNT1L,Tmp0
	ldi	Tmp0,HIGH(TIMER1_4_KHZ)
	out	TCNT1H,Tmp0
	ldi	Tmp0,TIMER1_PRESCALE_8
	out	TCCR1B,Tmp0
	in	Tmp0,TIMSK		; Enable Timer1 interrupts
	ori  	Tmp0,(1<<TOIE1)
	out	TIMSK,Tmp0

	cbr	InternalFlags,(1<<EXTERNAL_BUZZER_FLAG)
	rjmp	ADC_quit
	
;***********************
EE_rdy:   	; EEPROM Ready Handler
	cli
	lds	Tmp3,EEPROMByteCounter
	dec	Tmp3
	brmi    EE_writing_EEPROM_is_finished
	sts	EEPROMByteCounter,Tmp3

	ldi 	ZL,LOW(CHILD_MODE_WORD_ADDRESS)
	ldi 	ZH,HIGH(CHILD_MODE_WORD_ADDRESS)
	out 	EEARL,ZL		;Set EEPROM Address
	out 	EEARH,ZH		;Set EEPROM Address
	lds	Tmp2,ChildModeWord
	out 	EEDR,Tmp2		;Set Data To be written
	sbi 	EECR,EEMWE
	sbi 	EECR,EEWE
	
	cbi	EECR,EERIE		; Disable interrupt ready EEPROM	
	
	;ldi	Tmp1,1<<BIT_ERR0	
	;in	Tmp0,PORTD
	;eor	Tmp0,Tmp1
	;out	PORTD,Tmp0
		
	reti

EE_writing_EEPROM_is_finished:
	ldi	Tmp3,0xFF
	sts	EEPROMByteCounter,Tmp3

	;sbi	PORTD,BIT_ERR1		;!!!!!!!!!!!!!!!!!!!!
		
	ldi 	ZL,LOW(CHILD_MODE_WORD_ADDRESS)
	ldi 	ZH,HIGH(CHILD_MODE_WORD_ADDRESS)
	out 	EEARL,ZL		; Set EEPROM Address
	out 	EEARH,ZH		; Set EEPROM Address
	sbi 	EECR,EERE
	in 	Tmp0,EEDR		; Tmp0 = EEPROM Value
	cbi	EECR,EERIE		; Disable interrupt ready EEPROM
	sts	ChildModeWord,Tmp0
	cpi	Tmp0,DROWN_UP_MODE_CODE
	brne	EE_Child_Mode

	ldi 	Tmp0,HIGH(PRESSURE_295)
	sts	FirstThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_295)
	sts	FirstThreshOfMaxPressL,Tmp0

	ldi 	Tmp0,HIGH(PRESSURE_320)
	sts	SecondThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_320)
	sts	SecondThreshOfMaxPressL,Tmp0
	reti

EE_Child_Mode:
	
	ldi 	Tmp0,HIGH(PRESSURE_200)
	sts	FirstThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_200)
	sts	FirstThreshOfMaxPressL,Tmp0

	ldi 	Tmp0,HIGH(PRESSURE_210)
	sts	SecondThreshOfMaxPressH,Tmp0
	ldi	Tmp0,LOW(PRESSURE_210)
	sts	SecondThreshOfMaxPressL,Tmp0
	
	lds	Tmp0,ChildModeWord
	cpi	Tmp0,CHILD_MODE_CODE
	breq	EE_quit
	ldi	Tmp0,ERROR_TO_CHILD_MODE_CODE
	sts	ChildModeWord,Tmp0
EE_quit:
	reti




