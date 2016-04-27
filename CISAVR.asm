
.include "m328def.inc"

;	pins are: (graphic is Ic pin layout)

	
;	01 RST  6		5 WR#	28	SCK
;	02 SI  0		4 RD#	27	SDA
;	03 TXE  1		3 BUS3	26
;	04 PWR0	2		2 BUS2	25
;	05 RXF	3       1 BUS1	24
;	06 CIS_XFER	4	0 BUS0	23
;	07 VCC			  GND	22 
;	08 GND			  AREF	21
;	09 XTL			  AVCC	20
;	10 XTL			5 OPT2	19	SCK	
;	11 CIS_CLK	5	4 STEP	18	MISO
;	12 ACAP	6		3 BUS7	17	MOSI
;	13 ACAP	7		2 BUS6	16  SS
;	14 BUS4	0		1 BUS5	15

	; port pin usage
	; PD0 OUT	SI/WU		; send immediate wake up
	; PD1 IN	TXE#		; FIFO out busy
	; PD2 IN	PWR0		; USB active		 (int0)
	; PD3 IN	RXF#		; FIFO message avail (int1)
	; PD4 OUT   CIS_XFER	; cis strobe
	; PD5 OUT	CIS_CLK		; cis clock
	; PD6 PD7 Analog capture

.equ SI 		= PORTD0
.equ TXE		= PORTD1
.equ PWR0 		= PORTD2
.equ RXF		= PORTD3
.equ CIS_XFER	= PORTD4
.equ CIS_CLK	= PORTD5

.equ SI_DD 		= DDD0
.equ TXE_DD		= DDD1
.equ PWR0_DD 	= DDD2
.equ RXF_DD		= DDD3
.equ CIS_XDD	= DDD4
.equ CIS_CDD	= DDD5
; System Process equates
;----------------------------------------------------------------------
.equ NOOFTASKS 		= 1  	; tasks are numbered 0 - 2  
.equ STACKSIZE 		= 16 	; hope there is room
							; 

.equ ProcMainID		= 0		; Task ID of the main process


.equ F_CPU = 16000000


.def zero		= r1	; keep a zero handy
.def ashift		= r2	; temporary shift register (cause 88 breaks things)
.def nibbleMask	= r3	; should always be 0x0F
.def IOMask		= r4	; mask for port C reads and writes
//.def r5				; temporary store for reads and writes
//.def r6
.def byteflip	= r7	; place to flip bytes
.def runlow		= r8
.def runhigh	= r9

.def mtr_delayL	= r11
.def mtr_delayH	= r12
.def chkcmnd	= r13	; checksum of command
.def chksum		= r14	; simple checksum of recieved bytes
.def ssreg    	= r15

.def shiftbuf	= R16
.def temp3		= R17	;temporary register
.def TASKSTATE	= R18	;temporary register
.def temp1		= R19	;temporary register
.def idx		= R20	;temporary register
.def c_tmp     	= r21 	; passed arguments
.def ARGL     	= r22 	; passed arguments
.def ARGH     	= r23 	; to OS, usually a pointer
.def ACC      	= r24 	; Old fashioned double wideaccumulator
.def BCC      	= r25 	;  

; TASKSTATE semephors and task no 76543210
;                                 KSSSIITT
.equ keywaiting		= 7	; a keypress occured		
.equ task2Semaphore	= 6 ; to be used by
.equ task1Semaphore	= 5 ; file load and
.equ task0Semaphore	= 4 ; playback
; bits 3 and 2 are task ID when timer was interrupted
; bits 0 and 1 are task ID of current process

;==============================================================================
;               
;                     D A T A   S E G M E N T 
;
;==============================================================================
.equ MSGQ_SIZE = 64

.DSEG                		; Start data segment 
.ORG sram_start            	; Start data segment 

.ORG RAMEND-(NOOFTASKS * STACKSIZE); - STACKSIZE
STKRESERVE: .BYTE (NOOFTASKS * STACKSIZE); - STACKSIZE
.ORG STKRESERVE
MSGQEND:
.ORG MSGQEND-MSGQ_SIZE
MSGQ:   .BYTE MSGQ_SIZE             	;Buffer for writing LCD
.ORG MSGQ - ((NOOFTASKS * 4)+4)	; OS globals
world_save:	.byte 4
TTSL:   .BYTE NOOFTASKS *2		; 0x60 task timers
SPTS:   .BYTE NOOFTASKS *2		; 0x68 task saved stack pointers

; interrupt table
.cseg

.org 0
	rjmp reset		; reset device

.org INT0addr	; External Interrupt 0
	rjmp USBDown
	
.org INT1addr	; External Interrupt 1
	rjmp USBRead



; ***** INTERRUPT VECTORS ************************************************
.org	PCI0addr	; Pin Change Interrupt Request 0
	reti
.org	PCI1addr	; Pin Change Interrupt Request 0
	reti
.org	PCI2addr	; Pin Change Interrupt Request 1
	reti
.org	WDTaddr		; Watchdog Time-out Interrupt
	reti
.org	OC2Aaddr	; Timer/Counter2 Compare Match A
	reti
.org	OC2Baddr	; Timer/Counter2 Compare Match A
	reti
.org	OVF2addr	; Timer/Counter2 Overflow
	reti
.org	ICP1addr	; Timer/Counter1 Capture Event
	reti
.org	OC1Aaddr	; Timer/Counter1 Compare Match A
	rjmp NuLine
.org	OC1Baddr	; Timer/Counter1 Compare Match B
	reti
.org	OVF1addr	; Timer/Counter1 Overflow
	reti
.org	OC0Aaddr	; TimerCounter0 Compare Match A
	reti ;rjmp OvPtTST
.org	OC0Baddr	; TimerCounter0 Compare Match B
	rjmp OvPtTST
.org	OVF0addr	; Timer/Couner0 Overflow
	reti ;rjmp OvPtTST
.org	SPIaddr		; SPI Serial Transfer Complete
	reti
.org	URXCaddr	; USART Rx Complete
	reti
.org	UDREaddr	; USART, Data Register Empty
	reti
.org	UTXCaddr	; USART Tx Complete
	reti
.org	ADCCaddr	; ADC Conversion Complete
	reti
.org	ERDYaddr	; EEPROM Ready
	reti
.org	ACIaddr		; Analog Comparator
	rjmp CISflip
.org	TWIaddr		; Two-wire Serial Interface
	reti
.org	SPMRaddr	; Store Program Memory Read
	reti

.org INT_VECTORS_SIZE
;**********************************************************************
;
;		Y E   O L D E   I N T E R U P T E   H A N D L E R S
;
;**********************************************************************
_saveWorld:
	; save all upper registers in case of deferment
	; a trick to save a 16 bit value in a register
	; for a HW function we do no use.
	; make sure output compare A is off :-) if this
;	BRID PC+2
;	nop
	sts world_save,r4
	sts world_save+1,r0 
;	out OCR1AH,r4			; store the last 2 values in an unused	 
;	out OCR1AL,r0			; timer compare
	pop r4					; get the return address for placing at
	pop r0					; the bottom of the swich queue
	sts world_save+2,r4
	sts world_save+3,r0 
;
;	out OCR1BH,r4			; save the return address for later	 
;	out OCR1BL,r0			; (note, one could use this as a debug
	lds r4,world_save		
	lds r0,world_save+1		
;	in r4,OCR1AH			; restore user's register
;	in r0,OCR1AL			; 

	; save all but one register pair onto the stack

sz_save_all_st:				; special marker to keep from leaking memory
	push ZH
	push ZL
	push YH
	push YL
	push XH			 
	push XL			 	
	push BCC
	push ACC
	push ARGH		
	push ARGL	 
    push c_tmp		; fflags are used for inter process communication
	push idx		; R20				
	push temp1		; R19	
;	push temp2		; R18
	push temp3		; R17
	push shiftbuf	; R16

	; only a few low registers are not timers or belong only
	; to the MIDI task.
	push ssreg				; sreg at time of interrupt
	push r5					; usually result from divison
	push r4

	push r0					; we could defer inside an LPM
sz_save_all_end:
.equ sz_save_all = (sz_save_all_end -sz_save_all_st)

	; unfortunatally without using a second 16 bit temp it is
	; diffucault to restore a register pair and use push/pop
	; this work around take some explainig. At this point all
	; registers are on the stack.
	
	lds r4,world_save+2		
	lds r0,world_save+3		

;	in r4,OCR1BH			; push the return addres of the interrupt 
;	in r0,OCR1BL			; routine onto the stack
	push r0					; caller's return is on stack and registers are
	push r4					; saved for context switch at the end of the
							; satallite routine

	lds r4,world_save		
	lds r0,world_save+1		
;	in r4,OCR1AH			; the user arguments are still in the timer 
;	in r0,OCR1AL			; 
	push r0					; push them onto the stack so the satallite
	push r4					; routine can restore them
	
	rcall saveMoon			; return through save moon to restore temp register
	; fatal error if the code gets here
	; ldi RESULT,ERR_OS
	; rjmp OSERR

saveMoon:
;	BRID PC+2
;	nop
	pop r4					; put address of _save world into dev/null
	pop r0	
	pop r4					; restore user's args
	pop r0		
	ret						; back to interupt handler from save world with
							; all registers intact
; Interrupt 0 -- USB active

USBDown:
	; the PWREN# line went from low to high

	sbr TASKSTATE,1 << task0Semaphore
	reti

; Interrupt 1 -- Read Request
USBRead:
;	in ssreg,SREG
;	rcall _saveWorld

	; a new command is in the buffer
	ldi XL,low(MSGQ)			; t11
	ldi XH,high(MSGQ)			; t12
	clr chksum					; t13

	cbi PORTC,PINC4				; t14 read in checksum byte
	out PORTD,temp3				; t15 tick 01 set clock bit on port
								; keep clock high for read and command
								; selection? 
	nop
	in ACC,PINC					; t16 latch the two nybbles
	in BCC,PINB					;t1
	sbi PORTC,PINC4				;t2

	andi ACC,0x0F				;t3 convert the two nybbles to a single
	swap BCC					;t4 byte. -- this should catch the read
	andi BCC,0xF0				;t5 busy pre charge time T6
	or ACC,BCC
	mov chkcmnd,ACC				;t6 compare this to the calculated checksum
								; to involk a command.
	nop
	nop 
	nop	
	
USBRD001:
	cbi PORTC,PINC4				;t15 RD# strobe
	nop							;t16
	nop							;t16
	in ACC,PINC					;t1 latch the two nybbles
	in BCC,PINB					;t2
	sbi PORTC,PINC4				;t3

	andi ACC,0x0F				;t4 convert the two nybbles to a single
	swap BCC					;t5 byte. -- this should catch the read
	andi BCC,0xF0				;t6 busy pre charge time T6
	or ACC,BCC					;t7

	ldi c_tmp,high(MSGQEND-1)	;t1
	cpi XL,low(MSGQEND-1)		;t2
	cpc XH,c_tmp				;t3
	breq USBRD010				;t4
	st X+,ACC					;t5
	ldi c_tmp,high(MSGQ+7)		;t6 commands are 6 bytes
	cpi XL,low(MSGQ+7)			;t7
	cpc XH,c_tmp				;t8
	brge USBRD010				;t9 keep scanning for set parameters
	add chksum,ACC				;t10
USBRD010:
	sbis PIND,PIND3				;t12
	rjmp USBRD001				;t13 loop until FIFO empty

	; convert the checsum to octal and save it in buffer

	ldi ACC,' '					;t14 0 byte terminator.
	st X+,ACC					;t15

	ldi ACC,'\\'						;t14 0 byte terminator.
	st X+,ACC					;t15

	mov BCC,chksum
	clr ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	ori ACC,'0'
	st X+,ACC					;t15

	clr ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	ori ACC,'0'
	st X+,ACC					;t15

	clr ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	ori ACC,'0'
	st X+,ACC					;t15
	
	ldi ACC,' '						;t14 0 byte terminator.
	st X+,ACC					;t15

	ldi ACC,'\\'						;t14 0 byte terminator.
	st X+,ACC					;t15

	clr ACC
	mov BCC,chkcmnd

	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	ori ACC,'0'
	st X+,ACC					;t15

	clr ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	ori ACC,'0'
	st X+,ACC					;t15

	clr ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	lsl BCC
	rol ACC
	ori ACC,'0'
	st X+,ACC					;t15
	
	clr ACC						;t14 0 byte terminator.
	st X+,ACC					;t15

	sbr TASKSTATE,1 << task1Semaphore	;t16
	ldi	ACC, 1 << INT1			;t1
;	out GIFR,ACC				;t2 clear Read Busy cycle

	; test echo the string back
;	rcall USBWriteMSG

;	rjmp KEXIT				; restore SREG, other registers and reti
	ret

;	reti

;----------------------------------------------------------------------
USBWriteMSG:
	ldi XL,low(MSGQ)
	ldi XH,high(MSGQ)

	sbic PIND,PIND1
	nop						; TXE# busy


	ldi c_tmp,0b00011111	; activate bus for write
	out DDRB,c_tmp
	ldi c_tmp,0b00111111
	out DDRC,c_tmp


USBWR001:					; write loop
	ld ACC,X+
	cpi ACC,0
	breq EXIT_WR
	mov BCC,ACC
	swap BCC
	andi BCC,0x0F
	in c_tmp,PORTB			; save the scanner state
	andi c_tmp,0x30
	or BCC,c_tmp
	out PORTB,BCC			; drive the bus hard

	andi ACC,0x0F
	in c_tmp,PORTC			; save the RW state
	andi c_tmp,0x30
	or ACC,c_tmp
	ori ACC,0b00100000		; enable write pulse
	out PORTC,ACC			; drive the bus hard
	nop						; settle time
	cbi PORTC,PINC5			; latch the data
	nop						; data stable
	sbic PIND,PIND1			; buffer full?
	nop						
	rjmp USBWR001

EXIT_WR:
	in c_tmp,PORTB			; clear the last byte from the
	andi c_tmp,0b00110000	; bus
	out PORTB,c_tmp
	in c_tmp,PORTC
	andi c_tmp,0b00110000
	out PORTC,c_tmp

	sbi PORTC,PINC5			
	nop
	cbi PORTC,PINC5	
	nop

	ldi c_tmp,0b00010000	; restore tristate
	out DDRB,c_tmp
	ldi c_tmp,0b00110000	
	out DDRC,c_tmp

	cbi PORTD,PIND0			; send a SI message?
	nop
	nop
	sbi PORTD,PIND0
	ret

USB_WriteZ:					; write string from program memory

	sbic PIND,PIND1
	nop						; TXE# busy

	ldi c_tmp,0b00011111	; activate bus for write
	out DDRB,c_tmp
	ldi c_tmp,0b00111111
	out DDRC,c_tmp

WRITEZ00:
	lpm
	mov ACC,r0
	cpi ACC,0
	breq EXIT_WRZ
	mov BCC,ACC
	adiw ZL,1

	swap BCC
	andi BCC,0x0F
	in c_tmp,PORTB			; save the scanner state
	andi c_tmp,0x30
	or BCC,c_tmp
	out PORTB,BCC			; drive the bus hard

	andi ACC,0x0F
	in c_tmp,PORTC			; save the RW state
	andi c_tmp,0x30
	or ACC,c_tmp
	ori ACC,0b00100000		; enable write pulse
	out PORTC,ACC			; drive the bus hard
	nop						; settle time
	cbi PORTC,PINC5			; latch the data
	nop						; data stable
	sbic PIND,PIND1			; buffer full?
	nop						
	rjmp WRITEZ00

EXIT_WRZ:
	ret


NuLine:
	in ssreg,SREG  ;if we have time
	push c_tmp
;	cbi PORTD,CIS_CLK
	lds r5,TCNT1L			; itk + 2   clear the pulses
	lds r6,TCNT1H			; itk + 4
	out TCNT0,zero
	sub r5,runlow
	sbc r6,runhigh
;	st X+,r5				; itk + 10  save the data into the ring
	sbi PORTD,CIS_XFER
;	sbrc XH,3
;	ldi XH,high(SRAM_START)
;	andi XH,0x07			; dont overwrite the stack
;	st X+,r6				; itk + 12
	sbi PORTD,CIS_XFER
;	sbrc XH,3
;	ldi XH,high(SRAM_START)
;	andi XH,0x07			; dont overwrite the stack
	sts TCNT1H,zero		; t1t2 this will clear the timer
	sts TCNT1L,zero		; t3t4 and 6track the number of pixels actually scanned
	mov runlow,zero		; t5 and 6track the number of pixels actually scanned
	mov runhigh,zero		;t6 this will clear the timer
;	ldi XL,low(SRAM_START)	;t7 tclk10  ; this sets up the ring
;	sbi PORTD,CIS_CLK	
;	ldi XH,high(SRAM_START)	;t8 tclk11
;	ldi YL,low(SRAM_START)	;t9 tclk12
	ldi c_tmp,0x07
	out TIFR0,c_tmp
	cbi PORTD,CIS_XFER		;t11
;	ldi YH,high(SRAM_START)	;t10 tclk13
	;set the X and Y ring pointers to 0 (start of ring)
;	cbi PORTD,CIS_CLK
	out SREG,ssreg
	reti

OvPtTST:
	sbi PORTB,PB4	
	cbi PORTB,PB4
	reti

CISflip:
;	cbi PORTD,CIS_CLK 		; CIS_XFER
							; at least three clocks to get here

	reti

CleanSignal:
	lds r5,TCNT1L			; itk + 2   clear the pulses
	lds r6,TCNT1H			; itk + 4
	sub r5,runlow
	sbc r6,runhigh
	cp r5,zero
	cpc r6,zero
	breq L0602
	lds runlow,TCNT1L
	lds runhigh,TCNT1H
	st X+,r5				; itk + 10  save the data into the ring
	sbrc XH,3
	ldi XH,high(SRAM_START)
	st X+,r6				; itk + 12
	sbrc XH,3
	ldi XH,high(SRAM_START)
;	andi XH,0x07			; dont overwrite the stack
L0602:
;	cbi PORTD,CIS_XFER
	reti					; itk + 16  will extend clock a few microseconds

;----------------------------------------------------------------------
;********************************************************************
;* Init program
;********************************************************************
;------------------------------------------------------------------------------------------
reset:			;initialization of processor and variables to right values



	ldi ACC, 1 << CIS_CDD | 1 << CIS_XDD | 1 << SI_DD
	out DDRD,ACC       		


	CLI
	ldi	ACC,low(RAMEND)		; initialization of stack
	out	SPL,ACC

	ldi	ACC,high(RAMEND)		; initialization of stack
	out	SPH,ACC

	clr ACC					; slam all interrupts off
	out	EIMSK,ACC

	WDR

	; disable watchdog for testing
;	ldi ACC,1 << WDTOE | 1 << WDE | 1 << WDP2 | 1 << WDP1 | 1 << WDP0
;	out WDTCR,ACC
	ldi ACC,0 << WDE | 1 << WDP2 | 1 << WDP1 | 1 << WDP0
;	out WDTCR,ACC
;WDTCSR

	clr	XH					;
	clr	YH					;
	clr	ZH					; ROM pointer

			
		
   	; PC0,1,2,3 IO			; low nybble
	; PC4 OUT	RD# 		; read low strobe, clocks data in when low to
	;						; high
	; PC5 OUT	WR			; write strobe on high to low
	ldi ACC,0b00110000
	out DDRC,ACC       	
	ldi ACC,0b00010000
	out PORTC,ACC      

	; PB0,1,2,3 IO			; high nybble
	; PB4 OUT				; step pulse
	; PB5 IN				; Scan abt

	ldi ACC,0b00010000
	out DDRB,ACC       	

	ldi ACC,0b00100000
	out PORTB,ACC      

	;set the motor timout
	ser ACC
	mov mtr_delayL,ACC
	mov mtr_delayH,ACC


;Set up the MCU Control register (3-29)
;
	; this sets the interrupts for a one bit scanner
	; the two bit scanner does not have interrupts
	
	ldi	ACC, 1 << ISC11 | 0 << ISC10 | 1 << ISC01 | 1 << ISC00

	sts	EICRA,ACC

	ldi	ACC, 1 << INT1 | 1 << INT0 
	out	EIMSK,ACC
	
	CLI 					; busy till line is scanned
;	out TCCR1A,BCC			; No comapre match or PWM
;	ldi ACC,1 << CS12 | 1 << CS11	
;	sts TCCR1B,ACC			; set timer to trip on write 0 to T1

	; cascade clocks to generate proper waveform for idle mode
	; this may also be practical for scanning in the run length mode 
;	ldi ACC, 1 << COM0B1 | 1 << WGM01 | 1 << WGM00
;	out TCCR0A,ACC

;	ldi ACC, 1 << WGM02 | 1 << CS00
;	out TCCR0B,ACC

;	ldi ACC,15
;	out OCR0A,ACC

;	ldi ACC,3
;	out OCR0B,ACC


;	ldi ZH,high(3648)   ; tick 15 number of pixels in a tight scan line
;	ldi ZL,low(3648) 	; tick 16

;	sts OCR1AH,ZH		; this will clear the timer on interrupt
;	sts OCR1AL,ZL		; track the number of pixels actually scanned
	


;	ldi ACC,1 << OCF1A		; enable interrupt
;	sts TIMSK1,ACC

	clr ACC
	mov zero,ACC		; keep a zero in register for faster clears to
						; io locations
	mov runlow,zero		; t5 and 6track the number of pixels actually scanned
	mov runhigh,zero		; tick 03 and 4this will clear the timer

	ldi ACC,0x0F
	mov nibbleMask,ACC	; should always be 0x0F
	ldi ACC,0b00110000
	mov IOMask,ACC		; mask for port C reads and writes

;	ldi ACC,1 << ACIE
;	out ACSR,ACC

	ldi XL,low(SRAM_START)	; tclk10  ; this sets up the ring
	ldi XH,high(SRAM_START)	; tclk11
	ldi YL,low(SRAM_START)	; tclk12
	ldi YH,high(SRAM_START)	; tclk13

.if 0
	SEI					; let the timer handle the counting


; this code seems to be part of an attempt to use the
; analog compare to log in the data this does not work
; as the input signal is pulsed every clock from 0
W: 
	cp XL,YL			; test the ring for advance
	cpc XH,YH
	breq W
	brlo W
	; test the FIFO  for clear 
	sbic PIND,PIND1
	rjmp W						; TXE# busy


TwentyFourOut:
	ld ARGL,Y+
	sbrc YH,3
	ldi YH,high(SRAM_START)
	ld ARGH,Y+
	sbrc YH,3
	ldi YH,high(SRAM_START)
;	andi YH,0x03
;	brne L0794
;	inc YH
L0794:
	; write the timer
	mov byteflip,ARGL		; itk + 5
	swap byteflip			; itk + 6
	and byteflip,nibbleMask	; itk + 7
	and ARGL,nibbleMask		; itk + 8
	or byteflip,IOMask		; itk + 9
	out PortB,ARGL			; itk +10
	out PortC,byteflip		; itk +11
	cbi PORTC,PINC5			; itk +12
	and ARGH,nibbleMask		; itk +13
	or ARGH,IOMask			; itk +14
	out PortB,zero			; itk +15
	out PortC,ARGH			; itk +16
	cbi PORTC,PINC5			; itk +17	


rjmp W

	CLI
.endif


;	rjmp PWR0WAIT_SYNC		;StartPulse      ; tick blah
	ldi idx,0b00000000  	; blank for state reset
	ldi temp1, 1 << PWR0 | 1 << SI	; default port D state

	ldi temp3,1 << CIS_CLK | 1 << PWR0 | 1 << SI

	; wait for USB online PWREN# == 0

	; Enable Comparitor
	in ACC,ACSR
;	cbi ACSR,ACD


	; main polling routine, scanner will output clocks wile idle
	; this is to keep the array from saturating and pulling too
	; much power.

PWR0WAIT_SYNC:
						; tick 00 (tick 16)
	out PORTD,temp3		; tick 01 ; set clock state bit and SI
	ldi temp3,1 << CIS_XFER | 1 << PWR0 | 1 << SI
	nop 				; tick 03
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	out PORTD,temp3		; tick 06 start pulse at end of clock
	ldi temp3,1 << CIS_CLK | 1 << CIS_XFER | 1 << PWR0 | 1 << SI
	ldi shiftbuf,0		; tick 08 set first three bits on line start
	ldi XL, low(SRAM_START)  ; tick 09 ring tail 
	ldi XH, high(SRAM_START) ; tick 10 
	ldi ZL,low(3648)	; tick 11
	ldi ZH,high(3648)	; tick 12 number of pixels in a tight scan line
	ldi YL,low(SRAM_START)		; tick 13 ring head
	ldi YH,high(SRAM_START); tick 14 
	clr c_tmp           ; tick 15 simple semaphore for blocking FIFO
						; 		  and PWREN# lines 
	nop					; tick 16 
		
	out PORTD,temp3		; tick 01 set clock bit on port
	ldi temp3,1 << CIS_XFER | 1 << PWR0 | 1 << SI
	nop 	  	 		; tick 03
	clr ARGH     		; tick 04 used for ring count
	out PORTD,temp3     ; tick 05 clear clock pulse
	out PORTD,temp1     ; tick 06 clear start pulse
	ldi temp3,1 << CIS_CLK | 1 << PWR0 | 1 << SI
	nop					; tick 08 activate bus for write
	nop					; tick 09
	nop					; tick 10
	nop					; tick 11 bus is now in write mode
	nop					; tick 12 
	nop					; tick 13 
	nop					; tick 14 
	nop					; tick 15 
	nop					; tick 16 

	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 clear start pulse on
	nop     	 		; tick 03 
	nop     	 		; tick 04 
	out PORTD,temp1     ; tick 05 clear clock pulse
	nop				    ; tick 06 
	nop 				; tick 07 
	nop					; tick 08
	nop					; tick 09 
	nop					; tick 10 
	nop					; tick 11 
	nop					; tick 12 
	nop					; tick 13 
	nop					; tick 14 
	rjmp logBit4		; tick 15 the first three bit times
	nop					; tick 16 used to write the FIFO 

PWR0WAIT:

	; use an un rolled loop as the timing it tight we have
	; 16 clocks to clock the array at 1 MHz
; logBit7

	out PORTD,temp3		; tick 01 set clock bit on port
	st X+,shiftbuf		; tick 02,03 save new byte
	;nop
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse

	clr shiftbuf 		; tick 06 prep byte for next scan
	sbrc XH,3			; tick 07 
	ldi XH,high(SRAM_START)
	nop					; tick 08 
;	nop					; tick 09 

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16 

; log bit 6

	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop 				; tick 03 
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06
	nop					; tick 06
	nop					; tick 08
	nop					; tick 09

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16 

; log bit 5
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop 				; tick 03 
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06
	nop					; tick 07
	nop					; tick 08
	nop					; tick 09

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16 

; log bit 4
logBit4:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop 				; tick 03 
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	ldi c_tmp,0xFF			; tick 06 load motor counting
	cp  mtr_delayL,c_tmp	; tick 07 semaphore and test
	cpc mtr_delayH,c_tmp	; tick 08 against -1
	brne MOTOR_STEP			; tick 09 branch if motor active 					

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	rjmp logBit3		; tick 15 
						; tick 16 

MOTOR_STEP:
; bit 4 
	sub mtr_delayL,c_tmp	; tick 11 counting semaphore is in
	sbc mtr_delayH,c_tmp 	; tick 12 c_tmp from calling routine
	cp  mtr_delayL,c_tmp	; tick 13 compare delay count to
	cpc mtr_delayH,c_tmp	; tick 14 see if the motor step is done
	breq MS050				; tick 15
	nop 					; tick 16 continue to output clock pulses
							; for the duration of the motor step period
	
	out PORTD,temp3			; tick 01 set clock bit on port
	nop						; tick 02 
	nop 					; tick 03 
	nop 					; tick 04
	out PORTD,temp1			; tick 05 clear clock pulse
	nop						; tick 06
	nop						; tick 07
	nop						; tick 08
MotorIdle:
	nop						; tick 09

	nop						; tick 10
	nop						; tick 11
	nop						; tick 12
	nop						; tick 13
	nop						; tick 14

	rjmp PWR0WAIT_SYNC 		; tick 15,16   ; while busy?


MS050:	
	out PORTD,temp3			; tick 01 set clock bit on port
	nop						; tick 02 
	nop 					; tick 03 
	nop 					; tick 04
	out PORTD,temp1			; tick 05 clear clock pulse
	sbis PINB,PINB4			; tick 06
	rjmp MotorIdle	 		; tick 07,08 done advancing motor
;MotorActive
	nop						; tick 07
	nop						; tick 08
	nop						; tick 09


	cbi PORTB,PINB4				; tick 10 clear the pulse bit
;	ldi ARGL,low(0xFFFF-200)	; tick 11
;	ldi ARGH,high(0xFFFF-200)	; tick 12
;	mov mtr_delayL,ARGL			; tick 13
;	mov mtr_delayH,ARGH		 	; tick 14 set duty cycle
	nop						; tick 11
	nop						; tick 12
	nop						; tick 13
	nop						; tick 14

	rjmp PWR0WAIT_SYNC 			; tick 15,16

; log bit 3
logBit3:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop 				; tick 03 
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06
	nop					; tick 06
	nop					; tick 08
	nop					; tick 09

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16 

; log bit 2
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop 				; tick 03 
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06
	nop					; tick 06
	nop					; tick 08
	nop					; tick 09

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16 

; log bit 1
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop 				; tick 03 
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06
	nop					; tick 06
	nop					; tick 08
	nop					; tick 09

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16 

; log bit 0
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop					; tick 03 
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	subi ZL,8		 	; tick 06
	sbci ZH,0			; tick 07
	breq logByte		; tick 08
	nop

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14
	rjmp PWR0WAIT		; tick 15,16 

logByte:
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf		; tick 14		;

	ldi ZH,high(3648)   ; tick 15 number of pixels in a tight scan line
	ldi ZL,low(3648) 	; tick 16

;bit7 redux
	; buffer save the last byte of data
	out PORTD,temp3		; tick 01 set clock bit on port
	st X+,shiftbuf		; tick 02 save new byte
	sbrc XH,3			; tick 03 
	ldi XH,high(SRAM_START)
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	nop						; tick 06
	nop						; tick 07
	nop						; tick 08
	nop						; tick 09

	nop						; tick 10
	sbic PIND,PIND2			; tick 11
	rjmp PWR0WAIT_SYNC		; tick 12,13
	nop						; tick 12
	nop						; tick 13
	; check for pending data in FIFO
	sbic PIND,PIND3			; tick 14
	rjmp PWR0WAIT_SYNC		; tick 15,16

USBDataIN:


	; wait for a read request to issue us a command

	; commands are
	; 0 scanner on -> scan state READY | SENSOR_ABT
	; 1 scan single -> returns one line of CIS data bit packed
	; 2 scan step	-> same as 1 but issue a step pulse
	; 3 single pulse -> READY | SENSOR_ABT
	; 5 set step speed ASCII (pulse width) -> READY | SENSOR_ABT
	; 6 read step speed (pulse width) -> width ascii
	; 7 scan RLE 	-> return RLE data
	; 8 scan RLE with step

	rcall USBRead 		; get it

	; got it
	cp chkcmnd,chksum
	brne BADCMD

	; good.
ck10:
	ldi c_tmp,0x86		; single scan
	cp chkcmnd,c_tmp
	brne ck11
	rjmp SCAN10
ck11:
	ldi c_tmp,0x87
	cp chkcmnd,c_tmp
	brne ck01
	rjmp SCAN11
ck01:
	ldi c_tmp,0x9D
	cp chkcmnd,c_tmp
	brne ck48	
	rjmp STEP01	; - Step motor 1 pulse width.

ck48:
	ldi c_tmp,0x9C
	cp chkcmnd,c_tmp
	brne ck20	
	rjmp STAT00	; - Step motor 1 pulse width.

ck20:

BADCMD:
	ldi ZL,low(ERRORSTR *2)     ; indicate card detected
	ldi ZH,high(ERRORSTR *2)

	rcall USB_WriteZ
	rcall USBWriteMSG

	
forever:
	WDR
	rjmp PWR0WAIT_SYNC

	
	
	
	
 		
;------------------------------------------------------------------------------------------
;********************************************************************
;* Main program
;********************************************************************
;		SEI					;enable interrupts globally
SCAN10:

;		simple test to see if we can clock the system for scan rate of 1 MHz

				
StartPulse: 
						; tick 00 (tick 16)
	out PORTD,temp3		; tick 01 set clock bit on port
	ldi temp3,1 << CIS_XFER | 1 << PWR0 | 1 << SI
	nop 				; tick 03
	nop 				; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	out PORTD,temp3		; tick 06 start pulse at end of clock
	ldi temp3,1 << CIS_CLK | 1 << CIS_XFER | 1 << PWR0 | 1 << SI
	ldi shiftbuf,7		; tick 08 set first three bits on line start
	ldi XL, low(SRAM_START)  ; tick 09 ring tail 
	ldi XH, high(SRAM_START) ; tick 10 
	ldi ZL,low(3648)	; tick 11
	ldi ZH,high(3648)	; tick 12 number of pixels in a tight scan line
	ldi YL,low(SRAM_START)		; tick 13 ring head
	ldi YH,high(SRAM_START); tick 14 
	clr c_tmp           ; tick 15 simple semaphore for blocking FIFO
						; 		  and PWREN# lines 
	nop					; tick 16 
		
	out PORTD,temp3		; tick 01 set clock bit on port
	ldi temp3,1 << CIS_XFER | 1 << PWR0 | 1 << SI
	nop 	  	 		; tick 03
	clr ARGH     		; tick 04 used for ring count
	out PORTD,temp3     ; tick 05 clear clock pulse
	out PORTD,temp1     ; tick 06 clear start pulse
	ldi temp3,1 << CIS_CLK | 1 << PWR0 | 1 << SI
	ldi c_tmp,0b00011111; tick 08 activate bus for write
	out DDRB,c_tmp		; tick 09
	ldi c_tmp,0b00111111; tick 10
	out DDRC,c_tmp		; tick 11 bus is now in write mode
	nop					; tick 12 
	nop					; tick 13 
	nop					; tick 14 
	nop					; tick 15 
	nop					; tick 16 

	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 clear start pulse on
	nop     	 		; tick 03 
	nop     	 		; tick 04 
	out PORTD,temp1     ; tick 05 clear clock pulse
	nop				    ; tick 06 
	nop 				; tick 07 
	nop					; tick 08
	nop					; tick 09 
	nop					; tick 10 
	nop					; tick 11 
	nop					; tick 12 
	nop					; tick 13 
	nop					; tick 14 
	rjmp bit4			; tick 15 the first three bit times
							; tick 16 used to write the FIFO 
								

							
TightScan:
bit7:		
	out PORTD,temp3		; tick 01 set clock bit on port
	st X+,shiftbuf		; tick 02,03 save new byte
	;nop
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse

	clr shiftbuf 		; tick 06 prep byte for next scan
	sbrc XH,3			; tick 07 
	ldi XH,high(SRAM_START)
	nop					; tick 08 
;	nop					; tick 09 

	in ashift,ACSR		; t10 m 88 fix shift bit 5 into cary
	rol ashift			; t11
	rol ashift			; t12
	rol ashift      	; t13
	rol shiftbuf  		; tick 14

	nop		; tick 15,16  
	nop


;gets complicate here -- If we can write we do one branch
; if the Fifo is busy we need to do a different branch the tick count
; and sample point must remain constant.

; We  implement a classic ring here; saving the
; two output bytes into the ring tail. This would be pointed to by X
; Y points to the ring head, which if the tranmitter is ready
; and the ring not empty would supply the FIFO

bit6:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 save the high nybble in the ring
	nop					; tick 03  
	inc ARGH			; tick 04 bump the ring counter -- We do this
						;		  here since there are not enough ticks
						;		  to do the next save
	out PORTD,temp1		; tick 05 clear clock pulse


	nop					; tick 06 Store the low Nybble into the ring
	nop					; tick 07 we have already counted it

	nop					; tick 08 
	nop					; tick 09 

	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13

	rol shiftbuf  		; tick 14

	in c_tmp,PIND		; tick 15 get the FIFO state
	nop			 		; tick 16 Blocking semaphore in bit 2

bit5:
	out PORTD,temp3		; tick 01 set clock bit on port
	andi c_tmp,0b0000110; tick 02 -- Fifo clear, we can write
	brne FIFO_BLOCKED	; tick 03 if bit 2 (TXE#) or bit 3 (PWREN#)
						; is set then no write is allowed
						;         bit 3 PWREN# is overlayed with the
						;		  blocking semaphore which means
						;		  that the rest of the scan line is
						;		  buffered, and will be sent (If possible)
						;		  when the transmitter is ready

	ld ACC,Y+			; tick 04 get next byte from the ring
						; tick 05							 
	out PORTD,temp1		; tick 06 clear clock pulse
	sbrc YH,3				; tick 07
	ldi YH,high(SRAM_START) ; tick 08 on overflow
	mov BCC,ACC			; tick 08 get next byte from the ring
	nop					; tick 09

	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift      ; t13
	rol shiftbuf  		; tick 14

	swap BCC		; tick 15 fifo is clear start the write 
	andi BCC,0x0F	; tick 16 -- WR active set in bit 7 tick 09

;bit4 USB write mux

	out PORTD,temp3		; tick 01 set clock bit on port
	in c_tmp,PORTB		; tick 02 merge the data out
	andi c_tmp,0x30		; tick 03
	or BCC,c_tmp		; tick 04
	out PORTD,temp1		; tick 05 clear clock pulse
	out PORTB,BCC		; tick 06 drive the low nybble onto the bus

	andi ACC,0x0F		; tick 07
	in c_tmp,PORTC		; tick 08 save the RW state
	andi c_tmp,0x30		; tick 09

	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift      ; t13
	rol shiftbuf  	; tick 14

	or ACC,c_tmp	; t15
	ori ACC,0b00100000	; tick 16 enable write pulse

; bit3 USB write mux
	out PORTD,temp3		; tick 01 set clock bit on port
	out PORTC,ACC		; tick 02 drive the bus hard
	nop					; tick 03 settle time
	cbi PORTC,PINC5		; tick 04 latch the data
	out PORTD,temp1		; tick 05 clear clock pulse

	rjmp bit3a			; tick 06 resume scanning data bytes
						; tick 07


FIFO_BLOCKED:	
						; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
    nop                 ; tick 06 
	nop					; tick 07 
	nop					; tick 08		
	nop					; tick 09
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16

bit4:
	out PORTD,temp3		; tick 01 set clock bit on port

	nop					; tick 02
	nop					; tick 03
	nop					; tick 04
	
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06 
	nop					; tick 07 
	nop					; tick 08		
	nop					; tick 09
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15 
	nop					; tick 16

; resume scan here when there is a mux write happening

bit3:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop					; tick 03 
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06 
	nop					; tick 07 
bit3a:
	nop					; tick 08		
	nop					; tick 09
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15
	nop					; tick 16

bit2:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop					; tick 03 
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06 
	nop					; tick 07 
	nop					; tick 08		
	nop					; tick 09
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15
	nop					; tick 16

bit1:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop					; tick 03 
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06 
	nop					; tick 07 
	nop					; tick 08		
	nop					; tick 09
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf  		; tick 14
	nop					; tick 15
	nop					; tick 16

bit0:
	out PORTD,temp3		; tick 01 set clock bit on port
	nop					; tick 02 
	nop					; tick 03 
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	subi ZL,8		 	; tick 06
	sbci ZH,0			; tick 07
	breq TightExit		; tick 08
	nop

	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf		; tick 14		;
	rjmp TightScan		; tick 15/16 	; 
							

TightExit:
	in ashift,ACSR	; t10 m 88 fix shift bit 5 into cary
	rol ashift		; t11
	rol ashift		; t12
	rol ashift       ; t13
	rol shiftbuf		; tick 14		;

	ldi ZH,high(3648)   ; tick 15 number of pixels in a tight scan line
	ldi ZL,low(3648) 	; tick 16

;bit7 redux
	; buffer save the last byte of data
	out PORTD,temp3		; tick 01 set clock bit on port
	st X+,shiftbuf		; tick 02 save new byte
	sbrc XH,3			; tick 03 
	ldi XH,high(SRAM_START)
	nop					; tick 04 
	out PORTD,temp1		; tick 05 clear clock pulse
	nop					; tick 06 delay so ring loop is
	nop		 			; tick 07 in sync with clock pulses
	nop					; tick 08 
	nop		 			; tick 09 
	nop					; tick 10 
	nop					; tick 11
	nop					; tick 12 
	nop					; tick 13
	nop					; tick 14 
	nop					; tick 15
	nop					; tick 16 


HASRINGDATA:
	out PORTD,temp3		; t1

	in c_tmp,PIND		; tick 02 
	nop					; tick 03
	andi c_tmp,0b0000110; tick 04 Blocking semaphore in bit 2
	out PORTD,temp1		; tick 05 clear clock pulse

	breq FIFO_OUT		; tick 06 
	nop		 			; tick 07 in sync with clock pulses
	nop					; tick 08 
	nop		 			; tick 09 
	nop					; tick 10 
	nop					; tick 11
	nop					; tick 12 
	nop					; tick 13
	nop					; tick 14 
	rjmp HASRINGDATA						


FIFO_OUT:
	clc					; tick 07	
	cp YL,XL			; tick 08
	cpc YH,XH			; tick 09
	breq RINGDEPL		; tick 10
		
	ld ACC,Y+			; tick 11  buffered data is preped and
	sbrc YH,3			; tick 12  ready to send
	ldi YH,high(SRAM_START)
	mov BCC,ACC			; tick 13
	swap BCC			; tick 14
	andi BCC,0x0F		; tick 15
	in c_tmp,PORTB		; tick 16 save the scanner state

	out PORTD,temp3		; t1
	andi c_tmp,0x30		; tick 02
	or BCC,c_tmp		; tick 03
	out PORTB,BCC		; tick 04 drive the bus hard
	out PORTD,temp1		; tick 05 clear clock pulse

	andi ACC,0x0F		; tick 07
	in c_tmp,PORTC		; tick 08 save the RW state
	andi c_tmp,0x30		; tick 09
	or ACC,c_tmp		; tick 10
	ori ACC,0b00100000	; tick 12 enable write pulse
	out PORTC,ACC		; tick 13 drive the bus hard
	nop					; tick 14 settle time
	cbi PORTC,PINC5		; tick 15 latch the data

	rjmp HASRINGDATA	; tick 16,t17

RINGDEPL:
	; send a sync EOL packet

	; restore bus to input mode
	in c_tmp,PORTB			; t11 clear the last byte from the
	andi c_tmp,0b00110000	; t12 bus
	out PORTB,c_tmp			; t13
	in c_tmp,PORTC			; t14
	andi c_tmp,0b00110000	; t15	
	out PORTC,c_tmp			; t16

	out PORTD,temp3			; t1
	ldi c_tmp,0b00010000	; t2 restore tristate
	out DDRB,c_tmp			; t3
	ldi c_tmp,0b00110000	; t4
	out PORTD,temp1			; tick 05 clear clock pulse
	out DDRC,c_tmp			; t6


	cbi PORTD,PIND0			; t7 send a SI message?

	nop						; t8
	sbi PORTD,PIND0			; t9
	nop						; t10
	nop						; t11
	nop						; t12
	nop						; t13

; check for a motor tick timer

	ldi c_tmp,0xFF			; tick 14
	cp mtr_delayL,c_tmp		; tick 15
	cpc mtr_delayH,c_tmp	; tick 16
	breq SKIP_SYNC000		; tick 17

	sbi PORTB,PINB4			; tick 18 turn the motor on

SKIP_SYNC000:
	rjmp PWR0WAIT_SYNC 		; t19,20 (starts at t11) StartPulse     
							; tick blah

SCAN11: ;- scan and return one line of single bit data bit packed -step motor one pulse width
	; set motor state
	ldi ARGL,low(0xFFFF-1)
	ldi ARGH,high(0xFFFF-1)
	mov mtr_delayL,ARGL
	mov mtr_delayH,ARGH

	rjmp SCAN10	; t9 (starts at t11) StartPulse     

SCAN20: ;- scan and return one line of two bit data interleaved (twin bit array)
SCAN21: ;- scan and return one line of two bit data interleaved  - step motor one pulse width
SCAN18: ;- scan and return one line from array one
SCAN19: ;- scan and return one line from array one, step motor one pulse width
SCAN28: ;- scan and return one line from array two
SCAN29: ;- scan and return one line from array two, step motor one pulse width
	; disable comparator
	rjmp PWR0WAIT_SYNC 		; t9 (starts at t11) StartPulse     

STEP01:	; - Step motor 1 pulse width.
	; start motor tick pulse counter	
	; read mtr_delayL & H from EEPROM

	ldi ARGL,low(0xFFFF-200)
	ldi ARGH,high(0xFFFF-200)
	mov mtr_delayL,ARGL
	mov mtr_delayH,ARGH

	sbi PORTB,PINB4			; turn the motor on

	rjmp PWR0WAIT_SYNC 		; t9 (starts at t11) StartPulse     

STEP33:	; 000 - set the step pulse width in CIS clock event ticks.
STEP22:	; - return the STEP pulse width stored in the EEPROM

STAT00:	; - return the state of the scanner as ASCII
	; compare port b to assert mask
	; return abort if line asserted

	; compare to step counter
	; return busy if in step pulse	

	ldi ZL,low(READYSTR *2)     ; indicate card detected
	ldi ZH,high(READYSTR *2)

	rcall USB_WriteZ
	rcall EXIT_WR
	rjmp PWR0WAIT_SYNC

STAT99:	; - return the state of the microcontoller IO pins


SETA01:	; - set abort bit to assert high
SETA00:	; - set abort bit to assert low
SETC01:	; - turn CIS clock on
SETC02:	; - turn CIS clock off

SETC20:	; - set CIS clock line low
SETC21:	; - set CIS clock line high
SETC40:	; - set SP clock line low
SETC41:	; - set SP clock line high

;Optional commands that work only with the existing single bit interface

RLE010: ;- return a scan line as Run length encoded data into the FIFO
	ldi BCC,0b00000000  	; blank for state reset		
	ldi ACC,1 << CS12 | 1 << CS11	
	CLI 					; busy till line is scanned
;	out TCCR1A,BCC			; No comapre match or PWM
;	out TCCR1B,ACC			; set timer to trip on write 0 to T1

	rjmp PWR0WAIT_SYNC		;StartPulse      ; tick blah

RLE011: ;- return a scan line as Run length encoded data into the FIFO, with step

COMP22: ;- read the ADC voltage on the neg input of the comparator.

;				E		R			R		O			R
;ERRORSTR: .db 0x04,0x05,0x05,0x02,0x05,0x02,0x04,0x0F,0x05,0x02 ,0, 0
ERRORSTR:	.db "Error: Bad command ",0x00 
READYSTR:	.db "READY",0x00
BUSY_STR:	.db "BUSY",0x00,0x00
ABORTSTR:	.db "ABORT",0x00
