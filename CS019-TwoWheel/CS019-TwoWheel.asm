; CS019B05, 08/07/09, changing code to slow down new serve motors from Hillhouse products.

	list      p=16f73            ; list directive to define processor
	#include <p16f73.inc>        ; processor specific variable definitions

;	__CONFIG   _CP_OFF & _WDT_ON & _BODEN_ON & _PWRTE_ON & _HS_OSC
	__CONFIG   _CP_ALL & _WDT_ON & _BODEN_ON & _PWRTE_ON & _HS_OSC

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.

;*********************************************************************
; Definitions
;*********************************************************************
;
MSEC5_LSB	equ	0xa8	
;NORMAL 
MSEC5_MSB	equ	0x61

;SIM MSEC5_MSB	equ	0xfe

; msec5 init value to count up to 1 second
; 200 5msec ticks (255 - 200)
;TICK_CNT	equ	0x37
TICK_CNT	equ	0x57
; remote block init value to count down to 1/2 second
BLOCK_CNT	equ	0x53
; I/O, Interrupt and Option Definitions
;
;OPTIONVAL	equ	88h		;10001000b: portB no pull-up, tmr0 int
OPTIONVAL	equ	0c8h		;11001000b: portB no pull-up,  RB0/INT rising edge, tmr0 int
;OPTIONVAL	equ	088h		;11001000b: portB no pull-up,  RB0/INT falling edge, tmr0 int
INTCONVAL	equ	0a0h		; set GIE, TOIE

					; port A:
SPINPBIT	equ	00h		; analog input for ball spin     a/d channel 0
SPDBIT		equ	01h		; analog input for ball speed    a/d channel 1
BATTERY		equ	02h		; analog input for battery level a/d channel 2
FEEDBIT		equ	03h		; analog input for feed rate     a/d channel 3
SWEEPEN		equ	04h		; analog input for sweep enable switch a/d channel 4 (PORTA bit 5)
TRISAVAL	equ	3fh		;00111111b: A0 - A5 as input
ADCON1VAL	equ	2		; RA0,1,2,3,5 are analog	;MJZ DEBUG, COMMENTED THIS LINE
;ADCON1VAL	equ	6		;MJZ DEBUG, ADDED THIS LINE, CHANGED ALL ANALOG INPUTS TO DIGITAL INPUTS
ADCON0VAL	equ	081h		;10000001b: fosc/32, channel 0
LINESWBIT	equ 04h			;MJZ ADDED LINE, 2-LINE LIMIT SWITCH INPUT BIT, LOW=SWITCH ON, HIGH=SWITCH OFF

					; port B:
TRISBVAL	equ	0f1h		;11110001b: B1,B2,B3 as output
RED_LED		equ	1		;  bit 1 controls red led
GREEN_LED	equ	2		;  bit 2 controls green led
FEED_LED	equ	3		;  bit 3 controls feed led
RED_LED_ON	equ	2		;  value to turn on red led
GREEN_LED_ON	equ	4		;  value to turn on green led

					; port C:
PWMOUTBIT	equ	00h		; software PWM output for feed motor
SWEEP_EN	equ	03h		; sweep motor enable (digital out)
ELEV_EN		equ	04h		; elevation motor enable (digital out)
BEEP_OFF	equ	0x40		; bit 6 on disables beep
TRISCVAL	equ	0a0h		;10100000b: C7,C5 as input
PWM1		equ	2
PWM2		equ	1

FLASH_TIME	equ	30h
PAUSE_TIME	equ	4

;***** VARIABLE DEFINITIONS
w_temp		equ	0x20		; variable used for context saving 
status_temp	equ	0x21		; variable used for context saving
temp		equ	0x22
pwr_on_code	EQU	0x23
t_top_pwm	equ	0x24		; target value for top PWM
c_top_pwm	equ	0x25		; current value for top PWM
t_bot_pwm	equ	0x26		; target value for bot PWM
c_bot_pwm	equ	0x27		; current value for bot PWM
top_inited	equ	0x28		; top motor init done
bot_inited	equ	0x29		; bottom motor init done
feed_inited	equ	0x2a		; feed motor init done
sweep_inited	equ	0x2b		; sweep motor init done

stackw		equ	0x2c		; stack to push/pop the W-register
stacks		equ	0x2d		; stack to push/pop the STATUS-reg
counter		equ	0x2e		; counter: input frequency
					;   f1 = crystalfreq. / 4 / 255
counter2	equ	0x2f		; counter2: input frequency
					;   f2 = f1 / 128
pwmdesired	equ	0x30		; target PWM of feed motor value 0..255
pwmmax		equ	0x31		; register to support generation PWM
pwmhelp		equ	0x32		; used as temp storage of pwmdesired

a2d_chan	equ	0x33		; next channel to acquire a2d

seconds		equ	0x34		; seconds since poweron
msec5		equ	0x35		; number of 5msec intrs
init_done	equ	0x36		; power up initialization done
soft_pwm	equ	0x37		; software PWM module enabled

spin_a2d	equ	0x38		; a2d value from a2d chan 0
spd_a2d		equ	0x39		; a2d value from a2d chan 1
bat_a2d		equ	0x3a		; a2d value from a2d chan 2
feed_a2d	equ	0x3b		; a2d value from a2d chan 3
sweep_a2d	equ	0x3c		; a2d value from a2d chan 4

calc_pwm	equ	0x3d		; temporarily variable for calculating PWM
delta_c		equ	0x3e		; temp var
delta_s		equ	0x3f		; temp var

wait_cnt	equ	0x40		; counter for wait routine
;err_no		equ	0x41		; motor error number	;MJZ COMMENTED THIS LINE
					; 0= no error, 2= feed motor error, 3= top serve motor error
					;              4= bottom serve err, 5= sweep motor error
err_no		equ	0x41		; motor error number	;MJZ ADDED THIS LINE
					; 0= no error, 2= feed motor error, 3= top serve motor error
					;              4= bottom serve err, 5= POWER OFF ERROR
flash_code	equ	0x42		; local variable used by motor_err()
led_bits	equ	0x43		; memory copy of LED bits on PORTB
rbif_wait	equ	0x44		; delay for rechecking rbif

power_level	equ	0x45		; power level for set led: 0 is > 2.6v, 1 is > 2.5V, 2 is > 2.4v, 3 is < 2.4v
power_level_r	equ	0x46		; lowest power level for this 16 second period
power_level_tmp	equ	0x47		; temp var 
delay		equ	0x48		; delay var

mult1		equ	0x49		; temp var for mult_sub routine
mult2		equ	0x4a		; temp var for mult_sub routine
mult3		equ	0x4b		; temp var for mult_sub routine
rbif_bits	equ	0x4c		; bit mask for checking motor errors
sweep_is_on	equ	0x4d		; sweep motor on flag
r1		equ	0x4e		; result of mult1 x mult2
r2		equ	0x4f		; result of mult1 x mult2

remote_toggle	equ	0x50		; toggle bits for remote control
FEED_BIT	equ	0
SWEEP_BIT	equ	1
REMOTE_BIT	equ	5
FEED_TOGGLE	equ	0x01		; bit 0:  0 = feed off, 1 = feed on
SWEEP_TOGGLE	equ	0x02		; bit 1:  0 = sweep off, 1 = sweep on
REMOTE_ON	equ	0x20		; bit 5:  0 = remote disabled, 1= enabled
remote_block	equ	0x51		; time to block remote transmission input
FF_count	equ	0x52		; remaining FF's to send for beep
last_sweep	equ	0x53		;
beep_count	equ	0x54		; DEBUG



linesw_toggle	equ 0x55	;MJZ ADDED THIS LINE, BIT 4 IS 2-LINE SWITCH TOGGLE BIT
sweep_timer		equ 0x56	;MJZ ADDED THIS LINE, REACHES 0 MEANS NO 2-LINE SWITCH DETECTED BETWEEN SIDE TO SIDE SWEEPS
feed_rate_type	equ 0x57	;MJZ ADDED THIS LINE, BIT 0=0 FOR NORMAL FEED RATE ALGORITHM, 
							;MJZ ADDED THIS LINE, BIT 0=1 FOR RESTRICTED MAX RATE IN 2-LINE MODE



;----------------------------------------------------------------------
; PWM-module constant
PWMADJUSTVAL	equ	.22
	; correction number, defined by the following factors:
	; time from timer interrupt to executing PC 004 + 3 cycles
	; computing time from PC=004 to required edge   +18 cycles
	; lost timer cycles due to writing the time     + 2 cycles
	; cal desired PWM value to timer loading value  + 2 cycles
	; time from timer loading to gen required edge  - 1 cycle
	; valid value for hardware (unknown diff to the data sheet)
	; 3+18+2+2-1=22
	; value value for PICSIM version 5.11 (error of PICSIM):
	; 0+18+2+2-1=21
PWMMAXVAL	equ	.29
	; loading value for pwmmax
	; If n is the maximum length of a high pulse, which has to be
	; generated by the skipping method, then is PWMMAXVAL = n+1.
	; The max length of a low pulse using the skip method is n-1.



;**********************************************************************
		ORG     0x000             ; processor reset vector
		clrf    PCLATH            ; ensure page bits are cleared
  		goto    main              ; go to beginning of program


		ORG     0x004             ; interrupt vector location
		movwf   w_temp            ; save off current W register contents
		movf	STATUS,w          ; move status register into W register
		bcf     STATUS,RP0        ; ensure file register bank set to 0
		movwf	status_temp       ; save off contents of STATUS register

; isr code can go here or be located as a call subroutine elsewhere
		btfsc	INTCON,T0IF      ; check for timer0 interrupt
		call	PwmInt

		bcf     STATUS,RP0        ; ensure file register bank set to 0
		movf    status_temp,w     ; retrieve copy of STATUS register
		movwf	STATUS            ; restore pre-isr STATUS register contents
		swapf   w_temp,f
		swapf   w_temp,w          ; restore pre-isr W register contents
		retfie                    ; return from interrupt

main
; init code 
	;configuration of the PWM module
	clrf	INTCON			; disable all intrs
	clrf	CCP1CON			; CCP1 Module is off
	clrf	CCP2CON			; CCP2 Module is off
	clrf	TMR0			; reset timer
	clrf	pwmdesired		; reset value of software PWM is 0
	clrf	t_top_pwm
	clrf	t_bot_pwm
	clrf	c_top_pwm
	clrf	c_bot_pwm
	clrf	top_inited
	clrf	bot_inited
	clrf	feed_inited
	clrf	sweep_inited
;	clrw
;	clrf	PORTC			; reset all output to 0 before port C
	movlw	BEEP_OFF		; reset all output to 0 except BEEP_OFF before port C
	movwf	PORTC			; is changed from input to output 
					; to suppress an uncontrolled
					; spike
	clrf	PORTB			; turn off red led, green led, and feed led
	clrf	pwr_on_code		; reset type code
	movlw	PWMMAXVAL		; set support register
	movwf	pwmmax			;
	; configuration of the PIC
	bsf	STATUS,RP0		; register page 1
	movlw	TRISAVAL		; configure ...
	movwf	TRISA			; ...port A
	movlw	TRISBVAL		; configure ...
	movwf	TRISB			; ...port B
	movlw	TRISCVAL		; configure ...
	movwf	TRISC			; ...port C
	movlw	ADCON1VAL		; set inputs of ...
	movwf	ADCON1			; ...adc
	movlw	OPTIONVAL		; configure ...
	movwf	OPTION_REG		; ...PIC
	movf	PCON,w			; pick up NOT_POR and NOT_BOR
	andlw	3
	bsf	PCON,NOT_POR
	bsf	PCON,NOT_BOR
	bcf	STATUS,RP0		; register page 0
	movwf	pwr_on_code
	movlw	0x18			; get NOT_TO and NOT_PD
	andwf	STATUS,w
	iorwf	pwr_on_code,f			; or with NOT_POR and NOT_BOR
	clrwdt
;
; init variables
	clrf	seconds
	clrf	init_done		; power on init not done
	clrf	soft_pwm		; software pwm module not enabled
	clrf	led_bits
	movlw	MSEC5_LSB		; init  timer1 to count up to 5 msec
	movwf	TMR1L
	movlw	MSEC5_MSB
	movwf	TMR1H
	movlw	0x01			; enables TMR1, FOSC/4, 1:1 prescale
	movwf	T1CON	
	movlw	TICK_CNT		; init msec5 so it will roll over after 1 second
	movwf	msec5

	clrf	sweep_timer		;MJZ ADDED THIS LINE
	clrf	feed_rate_type	;MJZ ADDED THIS LINE


; on power up
;
at0sec
	call	sec0init
;	movlw	0xf8			; SIM ONLY
;	movwf	msec5			; SIM ONLY
; main idle loop
;
idle
	clrwdt				; toggle watchdog
	btfss	PIR1,TMR1IF		; wait for timer1 to overflow
	goto	idle
	movlw	MSEC5_LSB		; reset  timer1 to count up to 5 msec
	movwf	TMR1L
	movlw	MSEC5_MSB
	movwf	TMR1H
	bcf	PIR1,TMR1IF		; clear timer1 overflow flag
;	call	chk_rbif		; check if RB7:RB4 pins changed - motors error	;MJZ COMMENTED THIS LINE
	call	chk_rbif		;MJZ ADDED THIS LINE, CHECK IF RB7:RB4 PINS CHANGED - MOTOR ERRORS OR POWER OFF
	call	chk_intf		;MJZ ADDED THIS LINE, CHECK FOR BALL DROPPED
	call	chk_remote
	call	chk_2linesw		;MJZ ADDED LINE, CHECK IF 2-LINE LIMIT SWITCH IS ON, NEEDS TO BE CHECKED EVERY 5MS
							;TO ENSURE STOP POSITION ACCURACY & CONSISTANCY
	incfsz	msec5,f			; if msec5 overflowed,
	goto	not_second
;	movlw	0x55			; DEBUG
;	btfss	remote_toggle,REMOTE_BIT	; DEBUG
;	call	putchar			; DEBUG

	decf	sweep_timer			;MJZ ADDED THIS LINE, SWEEP TIMER COUNTDOWN SECONDS

	btfsc	power_level,0		;MJZ ADDED THIS LINE
	call	toggle_green_led	;MJZ ADDED THIS LINE, IF POWER_LEVEL=1 (OR 3), TOGGLE GREEN LED (POWER_LEVEL 3 NEVER GETS HERE)
	btfsc	power_level,1		;MJZ ADDED THIS LINE
	call	toggle_red_led		;MJZ ADDED THIS LINE, IF POWER_LEVEL=2 (OR 3), TOGGLE RED LED (POWER_LEVEL 3 NEVER GETS HERE)

	bsf	rbif_bits,0			; start checking for sweep motor error again
	movlw	TICK_CNT		; init msec5 so it will roll over after 1 second
	movwf	msec5
;	movlw	0xf8			; SIM ONLY
;	movwf	msec5			; SIM ONLY
	decfsz	init_done,w		;   and init not done
	goto	i_second		;   do init stuff
	incf	seconds,f		; msec5 overflowed, actually toggled every 1 seconds
	movf	seconds,w
	andlw	0x0f			; update battery led every 16 seconds
	btfsc	STATUS,Z
	call	update_bat_led

not_second:
	movlw	7
	andwf	msec5,w
	addwf	PCL,f
	goto	tick0
	goto	tick1
	goto	tick2
	goto	tick3
	goto	tick4
	goto	tick5
	goto	tick6
	goto	tick7

i_second:
	incf	seconds,w		; msec5 overflowed, actually toggled every 1.25 seconds
	movwf	seconds
	addwf	PCL,f
	goto	at0sec			; 0 second - this won't happen
	goto	at1sec			; 1 second
	goto	at2sec			; 2 seconds
	goto	at3sec			; 3 seconds
	goto	at4sec
	goto	at5sec
	goto 	at6sec
	goto	at7sec
	goto	at8sec
	goto	at9sec
	goto	at10sec
	goto	at11sec
	goto	at12sec
	goto	at13sec
	goto	at14sec
	goto	at15sec



	goto	idle
; end of main

;--- software PWM Generator
;
PwmInt
	bcf	INTCON,T0IF		; clear interrupt flag
	btfsc	PORTC,PWMOUTBIT		; which edge is required?
	goto	Lowpulse		; -> goto falling edge
Highpulse
	comf	pwmdesired,W		; get desired PWM value
	movwf	pwmhelp			; store val for the foll low pulse
	addwf	pwmmax,F		; calc number of inst's to skip
	btfss	STATUS,C		; which method to use?
	goto 	HighImpInt		; -> using interrupt
HighImpShrt
	movf	pwmmax,W		; get number of inst's to skip
	addwf	PCL,F			; skip n instructions
	bsf	PORTC,PWMOUTBIT		; rising edge, 28 cycles hi pulse
	bsf	PORTC,PWMOUTBIT		; 27 cycles
	bsf	PORTC,PWMOUTBIT		; 26 cycles
	bsf	PORTC,PWMOUTBIT		; 25 cycles
	bsf	PORTC,PWMOUTBIT		; 24 cycles
	bsf	PORTC,PWMOUTBIT		; 23 cycles
	bsf	PORTC,PWMOUTBIT		; 22 cycles
	bsf	PORTC,PWMOUTBIT		; 21 cycles
	bsf	PORTC,PWMOUTBIT		; 20 cycles
	bsf	PORTC,PWMOUTBIT		; 19 cycles
	bsf	PORTC,PWMOUTBIT		; 18 cycles
	bsf	PORTC,PWMOUTBIT		; 17 cycles
	bsf	PORTC,PWMOUTBIT		; 16 cycles
	bsf	PORTC,PWMOUTBIT		; 15 cycles
	bsf	PORTC,PWMOUTBIT		; 14 cycles
	bsf	PORTC,PWMOUTBIT		; 13 cycles
	bsf	PORTC,PWMOUTBIT		; 12 cycles
	bsf	PORTC,PWMOUTBIT		; 11 cycles
	bsf	PORTC,PWMOUTBIT		; 10 cycles
	bsf	PORTC,PWMOUTBIT		; 9 cycles
	bsf	PORTC,PWMOUTBIT		; 8 cycles
	bsf	PORTC,PWMOUTBIT		; 7 cycles
	bsf	PORTC,PWMOUTBIT		; 6 cycles
	bsf	PORTC,PWMOUTBIT		; 5 cycles
	bsf	PORTC,PWMOUTBIT		; 4 cycles
	bsf	PORTC,PWMOUTBIT		; 3 cycles
	bsf	PORTC,PWMOUTBIT		; 2 cycles
	bsf	PORTC,PWMOUTBIT		; 1 cycle
	bcf	PORTC,PWMOUTBIT		; fall edge, start of the following
					; low pulse using the interrupt
	incf	counter,F		; trigger counter, cause there was
					; a rising edge
	comf	pwmhelp,W		; get required low pulse length
	addlw	PWMADJUSTVAL+5		; calculate timer loading value
					; Edge was generated 5 cycles before
					; usual point of time
	movwf	TMR0			; put value into timer
	goto 	LowImpInt2		; low pulse using int is running
HighImpInt				; high pulse using interrupt
	addlw	PWMADJUSTVAL		; calculate timer loading value
	movwf	TMR0			; put value into timer
HighImpInt2
	bsf	PORTC,PWMOUTBIT		; generate rising edge
	incf	counter,F		; trigger counter, because there was a rising edge
	movlw	PWMMAXVAL-1		; "repair"...
	movwf	pwmmax			; ...support register
	return				; return to main isr


Lowpulse
	comf	pwmhelp,W		; get required pulse length
	addwf	pwmmax,F		; calc number of inst's to skip
	btfss	STATUS,C		; which method is to use?
	goto 	LowImpInt		; ->using interrupt
LowImpShrt
	movf	pwmmax,W		; get number of inst's to skip
	addwf	PCL,F			; skip n instructions
	bcf	PORTC,PWMOUTBIT		; falling edge, 27 cycles low pulse
	bcf	PORTC,PWMOUTBIT		; 26 cycles
	bcf	PORTC,PWMOUTBIT		; 25 cycles
	bcf	PORTC,PWMOUTBIT		; 24 cycles
	bcf	PORTC,PWMOUTBIT		; 23 cycles
	bcf	PORTC,PWMOUTBIT		; 22 cycles
	bcf	PORTC,PWMOUTBIT		; 21 cycles
	bcf	PORTC,PWMOUTBIT		; 20 cycles
	bcf	PORTC,PWMOUTBIT		; 19 cycles
	bcf	PORTC,PWMOUTBIT		; 18 cycles
	bcf	PORTC,PWMOUTBIT		; 17 cycles
	bcf	PORTC,PWMOUTBIT		; 16 cycles
	bcf	PORTC,PWMOUTBIT		; 15 cycles
	bcf	PORTC,PWMOUTBIT		; 14 cycles
	bcf	PORTC,PWMOUTBIT		; 13 cycles
	bcf	PORTC,PWMOUTBIT		; 12 cycles
	bcf	PORTC,PWMOUTBIT		; 11 cycles
	bcf	PORTC,PWMOUTBIT		; 10 cycles
	bcf	PORTC,PWMOUTBIT		; 9 cycles
	bcf	PORTC,PWMOUTBIT		; 8 cycles
	bcf	PORTC,PWMOUTBIT		; 7 cycles
	bcf	PORTC,PWMOUTBIT		; 6 cycles
	bcf	PORTC,PWMOUTBIT		; 5 cycles
	bcf	PORTC,PWMOUTBIT		; 4 cycles
	bcf	PORTC,PWMOUTBIT		; 3 cycles
	bcf	PORTC,PWMOUTBIT		; 2 cycles
	bcf	PORTC,PWMOUTBIT		; 1 cycles
	bsf	PORTC,PWMOUTBIT		; rising edge; start of the next
					; high pulse using the interrupt
	comf	pwmdesired,W		; get desired PWM value
	movwf	pwmhelp			; store val for the next lo pulse
	addlw	PWMADJUSTVAL+5		; calculate timer loading value
					; Edge was gen'd 5 cycles before
					; usual point of time.
	movwf	TMR0			; put value into timer
	goto 	HighImpInt2		; high pulse using int is running
LowImpInt				; low pulse using interrupt
	addlw	PWMADJUSTVAL		; calculate timer loading value
	movwf	TMR0			; put value into timer
LowImpInt2
	bcf	PORTC,PWMOUTBIT		; generate falling edge
	movlw	PWMMAXVAL		; "repair" ...
	movwf	pwmmax			; ... support register
	return				; return to main isr




; this is really part of main loop, moved here so PWNINT can reside below 0x100

; enable top motor 20KHz, 10% duty cycle
at1sec

	clrf	TMR2			; clear timer2
	clrf	T2CON			; hardware pwm prescaler is one, timer 2 off
;	movlw	0x02			; hardware pwm prescaler is 16, timer 2 off
;	movwf	T2CON
	bsf	STATUS,RP0		; register page 1
	movlw	0xff			; period is 50 us if prescaler is 1, 82 us if prescaler is 16
	movwf	PR2			;
	bcf	STATUS,RP0		; register page 0
;	movlw	0x7f
	movlw	0x19
	movwf	t_top_pwm
	movwf	t_bot_pwm
	movwf	c_top_pwm
	movwf	c_bot_pwm
;	movlw	0x7f			; 50% duty cycle
	movlw	0x19			; 10% duty cycle
	movwf	CCPR1L			;
	movwf	CCPR2L
	bsf	STATUS,RP0		; bank1
	bcf	TRISC,PWM1		; make pin for pwm1 output
	bcf	TRISC,PWM2		; make pin for pwm2 output
	clrf	PIE1			; disable peripheral interrupts
	bcf	STATUS,RP0		; bank0
	clrf	PIR1			; clear peripheral interrupts flags
	movlw	0x0c			; PWM mode, 2LSBs of duty cycle = 0 (top)
;	movlw	0x3c			; DEBUG PWM mode, 2LSBs of duty cycle = 11b (top)
	movwf	CCP1CON
;	movlw	0x0c			; PWM mode, 2LSBs of duty cycle = 0 (bottom)
;	movwf	CCP2CON
	bsf	T2CON,TMR2ON		; timer2 starts to increment

; fall through to atcom

at2sec
at3sec

	goto	atcom

at4sec

;; Top motor duty cycle set to pot
;	incf	top_inited,f		; set flag to allow top PWM to adjust to pot, 06/15/05 MJZ change
; Bottom motor duty cycle set to pot
;	incf	bot_inited,f		; set flag to allow bottom PWM to adjust to pot
	goto	atcom

at5sec

; Bottom motor 10% duty cycle
	movlw	0x19			; 06/15/05 MJZ change for ramping up bottom serve motor at power-on, now original 10% duty cycle 
							; got changed before we got here.
	movwf	t_bot_pwm		; 06/15/05 MJZ change for ramping up bottom serve motor at power-on
	movwf	c_bot_pwm		; 06/15/05 MJZ change for ramping up bottom serve motor at power-on
	movlw	0x19			; 06/15/05 MJZ change for ramping up bottom serve motor at power-on
	movwf	CCPR2L			; 06/15/05 MJZ change for ramping up bottom serve motor at power-on

	movlw	0x0c			; PWM mode, 2LSBs of duty cycle = 0
;	movlw	0x3c			; DEBUG PWM mode, 2LSBs of duty cycle = 11b (bottom)
	movwf	CCP2CON
;	movlw	0x0c			; PWM mode, 2LSBs of duty cycle = 0
;	movwf	CCP1CON
	goto	atcom

at6sec
	
	goto	atcom

at7sec

;; Top motor duty cycle set to pot, 06/15/05 MJZ change
	incf	top_inited,f		; set flag to allow top PWM to adjust to pot, 06/15/05 MJZ change

;; Bottom motor duty cycle set to pot,	; 06/15/05 MJZ changes, set to pot sooner than original
	incf	bot_inited,f		; set flag to allow bottom PWM to adjust to pot, 06/15/05 MJZ change, see above line

	goto	atcom

; fall through to atcom
at8sec
;; Bottom motor duty cycle set to pot
;	incf	bot_inited,f		; set flag to allow bottom PWM to adjust to pot, 06/15/05 MJZ change, move to "at7sec"
; Top motor duty cycle set to pot
;	incf	top_inited,f		; set flag to allow top PWM to adjust to pot
	goto	atcom

at9sec
at10sec
at11sec
	goto	atcom
at12sec
; sweep motor control
;	incf	sweep_inited,f		; set flag to allow sweep motor control
;	bcf	rbif_bits,0		;  don't check for sweep motor error
;	incf	init_done,f		; set initialization done flag

; fall through to atcom
at13sec
;	movf	feed_a2d,w
;	sublw	0x3f			; if input < 1.25V (no borrow), power feed motor to pot
;	btfsc	STATUS,C		; else force to 1.25V for 1 sec, then set to pot
;	goto	under_1_25
;	movlw	0x3f
;	movwf	feed_a2d
;under_1_25
;	call	init_feed_pwm
	goto	atcom

at14sec
;	movf	feed_a2d,w
;	sublw	0x7f			; if input < 2.5V (no borrow), power feed motor to pot
;	btfsc	STATUS,C		; else force to 2.5V for 1 sec, then set to pot
;	goto	under_2_5
;	movlw	0x7f
;	movwf	feed_a2d
;under_2_5
;	call	init_feed_pwm
	goto	atcom



atcom:
	goto	idle

at15sec:
; if feed motor a2d input > 1.0V, enable feed motor to pot
	incf	feed_inited,f		; enable feed motor if input > 1.0V
	incf	init_done,f		; set initialization done flag
	goto	idle			; goto main idle loop

sec0init:
	; init usart
	bsf	STATUS,RP0		; goto bank 1
;	movlw	0xff			; 1200 baud
	movlw	0x81			; 2400 baud
	movwf	SPBRG
	movlw	0x20			; 8-bit transimit, transmitter enabled
	movwf	TXSTA			; asynchronous mode, low speed mode
	bcf	STATUS,RP0		; goto bank 0
	movlw	0x90			; 8-bit receive, receiver enabled
	movwf	RCSTA			; serial port enabled

	call	usart_off		; turn off usart, disable beep

;	movlw	0x0f			; change to 0x0d - don't check feed motor
	movlw	0x0d
	movwf	rbif_bits		; check all motor errors
	clrf	TMR0
	clrf	a2d_chan
	call	acq_a2d			; acquire chan0 a2d - BALL SPIN
	movwf	spin_a2d		; save value
	sublw	0x8f			; if spin_a2d < 143
	movlw	0x8f
	btfsc	STATUS,C		;  skip if borrow - spin_a2d > 143
	movwf	spin_a2d		;  if spin_a2d < 143, spin_a2d = 143
	movf	spin_a2d,w		; 
	addlw	6	
	movlw	0xfa			; if spin_a2d > 250
	btfsc	STATUS,C		; 
	movwf	spin_a2d		;  spin_a2d = 250

	movlw	1
	movwf	a2d_chan
	call	acq_a2d			; acquire chan1 a2d - SPEED 
	movwf	spd_a2d
	sublw	0x8f			; if speed_a2d < 143
	movlw	0x8f
	btfsc	STATUS,C		;  skip if borrow - speed_a2d > 143
	movwf	spd_a2d			;  if speed_a2d < 143, speed_a2d = 143
	movf	spd_a2d,w		; 
	addlw	6	
	movlw	0xfa			; if spd_a2d > 250
	btfsc	STATUS,C		; 
	movwf	spd_a2d			;  spd_a2d = 250

	movlw	2
	movwf	a2d_chan
	call	acq_a2d			; acquire chan2 a2d - battery
;	movlw	0xff			; SIM - force battery ok
	movwf	bat_a2d		
	movlw	3
	movwf	a2d_chan
	call	acq_a2d			; acquire chan3 a2d - feed rate
	movwf	feed_a2d
	sublw	0x14			; 20 - feed_a2d
	btfss	STATUS,C		;  skip if no borrow - feed_a2d < 20
	goto    feed_ok
	movf	feed_a2d,w
	sublw	0x0f			; 15 - feed_a2d
	movlw	0x14
	btfss	STATUS,C		;  skip if no borrow - feed _a2d < 11
	movwf	feed_a2d		;  if 15 < feed_a2d < 20, speed_a2d = 20
feed_ok

	movlw	4
	movwf	a2d_chan
	call	acq_a2d			; acquire chan4 a2d - sweep enable
	movwf	sweep_a2d
	
;	btfsc	PORTB,0			;MJZ COMMENTED THIS LINE
;	goto	power_is_off		; if RB0 is set, power switch is off	;MJZ COMMENTED THIS LINE
	btfsc	PORTB,4			;MJZ ADDED THIS LINE, POWER SWITCH NOW ON RB4
	goto	power_is_off	;MJZ ADDED THIS LINE, POWER SWITCH NOW ON RB4

	movf	bat_a2d,w		; get battery level
;	sublw	0x7a			; if battery less than 2.4V (0x7a - bat_a2d)
	sublw	0x84			; at power on, battery must be at least 2.6V to power up motors
	btfsc	STATUS,C		;    skip if borrow (> 2.6V)	;MJZ DEBUG, COMMENTED THIS LINE
;	btfss	STATUS,C		;MJZ DEBUG, ADDED THIS LINE FOR SIMULATION TO GET PAST NO BATTERY INPUT
	goto	bat_dead		;  battery exhausted, shut down all motors
	bsf	PORTC,ELEV_EN		; else enable elevation motor
	clrf	PORTB			; on 1st power up, getting motor error
	clrf	INTCON			;  so make sure conditions are cleared
	clrf	sweep_is_on		; clear sweep motor on flag
	clrf	linesw_toggle	;MJZ CLEAR 2-LINE SWITCH TOGGLE BYTE
	call	init_bat_level
; sweep motor control
	incf	sweep_inited,f		; set flag to allow sweep motor control

	bsf	pwr_on_code,NOT_TO	; ignore watchdog timeout reset
	movf	pwr_on_code,w		; change bits to code : 0 = pwr on reset, 
					;			1= BOR, 2=wdt, 3=reset
	movwf	temp
	clrf	pwr_on_code
	btfss	temp,NOT_POR
	goto	pwr_on_reset
	incf	pwr_on_code,f
	btfss	temp,NOT_BOR
	goto	npwr_on_reset
	incf	pwr_on_code,f
	btfsc	temp,NOT_TO
	incf	pwr_on_code,f
npwr_on_reset:				; not power on reset (and not watchdog timeout)
	movlw	6
	movwf	flash_code
	call	flash_error
	goto	npwr_on_reset

pwr_on_reset:

	clrf	remote_toggle		; feed and sweep toggle are both off
	bsf	remote_toggle,REMOTE_BIT	; remote switch is off
	clrf	remote_block		; blocking of remote xmitter is off
	clrf	FF_count		; no FF's to send
	clrf	beep_count
	
	return

tick0:
	clrf	a2d_chan
	call	acq_a2d			; acquire chan0 a2d - BALL SPIN
	movwf	spin_a2d		; save value
	sublw	0x8f			; if spin_a2d < 143
	movlw	0x8f
	btfsc	STATUS,C		;  skip if borrow - spin_a2d > 143
	movwf	spin_a2d		;  if spin_a2d < 143, spin_a2d = 143
	movf	spin_a2d,w		; 
	addlw	6	
	movlw	0xfa			; if spin_a2d > 250
	btfsc	STATUS,C		; 
	movwf	spin_a2d		;  spin_a2d = 250
	goto	idle

tick1:
	movlw	1
	movwf	a2d_chan
	call	acq_a2d			; acquire chan1 a2d - SPEED 
;	movlw	0xfa			; SIM - force speed max
	movwf	spd_a2d
; 08/07/09 start
;	sublw	0x8f			; if spd_a2d < 143, 06/20/05 MJZ change to increase minimum serve motor speed for pressureless balls
;	movlw	0x8f			; 06/20/05 MJZ change to increase minimum serve motor speed for pressureless balls
;	sublw	0x98			; if spd_a2d < 152, 06/20/05 MJZ change to increase minimum serve motor speed for pressureless balls
;	movlw	0x98			; 06/20/05 MJZ change to increase minimum serve motor speed for pressureless balls
	sublw	0x8f
	movlw	0x8f
; 08/07/09 end
	btfsc	STATUS,C		;  skip if borrow - spd_a2d > 143
	movwf	spd_a2d			;  if spd_a2d < 143, spd_a2d = 143
	movf	spd_a2d,w		; 
; 08/07/09 start
	addlw	6	
	movlw	0xfa			; if spd_a2d > 250
	btfsc	STATUS,C		; 
	movwf	spd_a2d			;  spd_a2d = 250
;	addlw	0x20	
;	movlw	0xe0			; if spd_a2d > 224, approx 85% of original max of 250
;	btfsc	STATUS,C		; 
;	movwf	spd_a2d			;  spd_a2d = 224
; 08/07/09 end
	goto	idle

tick2:
	movlw	2
	movwf	a2d_chan
	call	acq_a2d			; acquire chan2 a2d - battery
;	movlw	0xff			; SIM - force battery ok
	movwf	bat_a2d		
;	sublw	0x7a			; if battery less than 2.4V (0x7a - W)	
;	btfsc	STATUS,C		;  	skip if borrow set ( > 2.4V )
;	goto	bat_dead		;  battery exhausted, shut down all motors
	; update battery led here if battery level changed

	goto	idle

tick3:
	movlw	3
	movwf	a2d_chan
	call	acq_a2d			; acquire chan3 a2d - feed rate
;	movlw	0xff			; SIM - force feed ok
	movwf	feed_a2d
	sublw	0x14			; 20 - feed_a2d
	btfss	STATUS,C		;  skip if no borrow - feed_a2d < 20
;	goto    feed_ok1		;MJZ COMMENTED THIS LINE
	goto	chk_feed_rate_type	;MJZ ADDED LINE, IF > 20, CHECK IF MUST RESTRICT MAXIMUM FEED RATE FOR 2-LINE MODE
	movf	feed_a2d,w
	sublw	0x0f			; 15 - feed_a2d
	movlw	0x14
	btfss	STATUS,C		;  skip if no borrow - feed _a2d < 11
	movwf	feed_a2d		;  if 17 < feed_a2d < 20, speed_a2d = 20
	goto	feed_ok1		;MJZ ADDED LINE
;feed_ok1					;MJZ COMMENTED THIS LINE
;	goto	idle			;MJZ COMMENTED THIS LINE
chk_feed_rate_type:				;MJZ ADDED LINE
	btfss	feed_rate_type,0	;MJZ ADDED LINE, IF SET, MUST RESTRICT MAXIMUM FEED RATE FOR 2-LINE MODE
	goto	feed_ok1			;MJZ ADDED LINE
	movf	feed_a2d,w			;MJZ ADDED LINE
	sublw	0x57				;MJZ ADDED LINE, MAXIMUM FEED RATE FOR 2-LINE MODE
	btfsc	STATUS,C			;MJZ ADDED LINE, SKIP IF BORROW, IF FEED RATE HIGHER THAN 0X57 (first try was 0x40)
	goto	feed_ok1			;MJZ ADDED LINE, OK IF FEED RATE ALREADY LESS THAN 0X57
	movlw	0x57				;MJZ ADDED LINE
	movwf	feed_a2d			;MJZ ADDED LINE, RESTRICT FEED RATE TO 0X57 (first try was 0x40) MAXIMUM FOR 2-LINE MODE
feed_ok1:						;MJZ ADDED LINE
	goto idle					;MJZ ADDED LINE

tick4:
	movlw	4
	movwf	a2d_chan
	call	acq_a2d			; acquire chan4 a2d - sweep enable
	movwf	sweep_a2d
	call	update_remote		; check if remote enable switch had changed
	goto	idle

tick5:
	call	calc_bot_pwm
	call	calc_top_pwm
	goto	idle

tick6:
	call	update_top_pwm
;	call	update_top_pwm
;	call	update_bot_pwm
	call	update_bot_pwm
	goto	idle

tick7:
	call	update_feed_pwm
	call	update_sweep
;	call	update_bat_led
	call	record_bat_level
	goto	idle


update_top_pwm:
	decfsz	top_inited,w		; if TOP MOTOR PWM inited,
;	goto	upd_exit		; 06/09/05 MJZ commented this line, see next line
	call	top_not_inited	; 06/09/05 MJZ added this line, at power-on, ramp up top serve motor regardless of speed pot
	movf	c_top_pwm,w		;  load current top pwm value into W
	subwf	t_top_pwm,w		;  W = (target - current)
	btfsc	STATUS,Z		;  if current top pwm value equal target top pwm value
	goto	upd_exit		;   done
	btfss	STATUS,C		;  if current top pwm value < than target ( borrow cleared)
	goto	dec_top			;   	(goto dec_top if borrow set)
	andlw	0xf8			;  if delta < 8
	btfss	STATUS,Z
	goto	inc_top_by_8
	incf	c_top_pwm,w		;   incr top pwm value and save to W
	goto	set_top
inc_top_by_8:				; else incr top pwm value by 8
;	movlw	8
;	btfss	init_done,0		;      if init_done flag is set
	movlw	4			;      else inc by 4
	addwf	c_top_pwm,w
	goto	set_top
dec_top:
	sublw	0			; figure out delta to decrement
	andlw	0xf8			; if delta < 8
	btfss	STATUS,Z
	goto	dec_top_by_8
	decf	c_top_pwm,w		;  dec top pwm value and save to W
	goto	set_top
dec_top_by_8
;	movlw	8
;	btfss	init_done,0		; if init_done flag is not set
	movlw	4			;      dec by 4
	subwf	c_top_pwm,w		; else dec top pwm value by 8
set_top:
	movwf	c_top_pwm		;  save current top pwm value
	movwf	CCPR1L			;  write new duty cycle value to PWM module 1
upd_exit:
	return

update_bot_pwm:				; check if bottom pwm duty cycle need updating
	decfsz	bot_inited,w		; if BOTTOM MOTOR PWM inited,
;	goto	update_done		; 06/15/05 MJZ change, see next line
	call	bot_not_inited	; 06/15/05 MJZ change for ramping up bottom serve motor at power-on
	movf	c_bot_pwm,w		;  load current bottom pwm value into W
	subwf	t_bot_pwm,w		;  W = target - current
	btfsc	STATUS,Z		;  if current bottom pwm value = target bottom pwm value
	goto	update_done		;   goto exit
	btfss	STATUS,C		;  if current bottom pwm value < target ( no borrow)
	goto    dec_bot			; 
	andlw	0xf8			;  if delta < 8
	btfss	STATUS,Z
	goto	inc_bot_by_8
	incf	c_bot_pwm,w		;   incr bottom pwm value and save to W
	goto	set_bot
inc_bot_by_8:
;	movlw	8			; else inc bot pwm value by 8
;	btfss	init_done,0		;      if init_done flag is set
	movlw	4			;      else inc by 4
	addwf	c_bot_pwm,w
	goto	set_bot

dec_bot:
	sublw	0			; figure out delta to decr
	andlw	0xf8			; if delta < 8
	btfss	STATUS,Z
	goto	dec_bot_by_8
	decf	c_bot_pwm,w		;  dec bottom pwm value and save to W
	goto	set_bot
dec_bot_by_8
;	movlw	8			; else dec bot pwm value by 8
;	btfss	init_done,0		;      if init_done flag is set
	movlw	4			;      else dec by 4
	subwf	c_bot_pwm,w
set_bot:
	movwf	c_bot_pwm		;  save current bottom pwm value
	movwf	CCPR2L			;  write new duty cycle value to PWM module 2
update_done:
	return

; if soft_pwm == 0
;   if input >= 1.0 volt, enable software pwm
;   if input < 1.0 volt, do nothing
; if soft_pwm == 1
;   if input < 1.0 volt, soft_pwm = 0 and disable software pwm
;   if input > 1.0, adjust pwmdesired
; check a2d of feed control
update_feed_pwm:
	decfsz	feed_inited,w
	goto	chk_feed_xit
	btfsc	remote_toggle,REMOTE_BIT ; if remote switch is on
	goto	no_remote_feed
	btfss	remote_toggle,FEED_BIT	;    and feed toggle is off
	goto	feed_off		;        disable feed
no_remote_feed:
	movf	feed_a2d,w
	sublw	0x13			; if input < 0.4V (no borrow)
	btfss	STATUS,C
	goto	over1v
feed_off:
	decfsz	soft_pwm,w		;  and soft_pwm was on
	goto	chk_feed_xit
	; disable software pwm
	bcf	INTCON,GIE		;  disable GLOBAL INTERRUPT ENABLE
	clrf	soft_pwm
	clrf	pwmdesired
	bcf	PORTC,PWMOUTBIT		;  force feed motor off
	bcf	led_bits,FEED_LED
	bcf	PORTB,FEED_LED		;  turn feed led off
	goto	chk_feed_xit
over1v
	decfsz	soft_pwm,w		; if soft_pwm was off
	goto	enable_feed
	goto	update_feed
enable_feed:
	; enable software pwm
	bsf	PORTB,FEED_LED		; turn feed led on
	bsf	led_bits,FEED_LED
	clrf	TMR0			; reset timer 0
	movlw	INTCONVAL		; enable timer0 intr
	movwf	INTCON
	incf	soft_pwm,f		; indicate feed pwm active by setting soft_pwm
	movlw	0x43			; got straight to 26% duty cycle
	movwf	delta_s
	goto	set_pwm_value

update_feed:				; check if feed motor pwm duty cycle need updating

	movlw	0x07			; if feed_a2d > 248
	addwf	feed_a2d,W		;
	btfss	STATUS,C		; and
	goto	calc_feed
	movlw	0x20			;  if pwmdesired > 223
	addwf	pwmdesired,W		;
	btfss	STATUS,C		;  
	goto	calc_feed
	incfsz	pwmdesired,w
	movwf	pwmdesired
	goto	chk_feed_xit		;  if > 233, step up by one to 255
	
calc_feed:
	movlw	0x14			; calc delta c
	subwf	feed_a2d,w
	movwf	delta_c			; delta_c = feed_a2d - 20
	addlw	0x43			;  W = 67 + delta_c
	movwf	delta_s			;    save in delta_s
	bcf	STATUS,C
	rrf	delta_c,F		;  ( div by 2 )
	bcf	STATUS,C
	rrf	delta_c,F		;  ( div by 4 )
	movf	delta_c,W
	subwf	delta_s,F		;         - delta_c / 4
	bcf	STATUS,C
	rrf	delta_c,F		;  ( div by 8 )
	bcf	STATUS,C
	rrf	delta_c,F		;  ( div by 16 )
	movf	delta_c,W
	addwf	delta_s,F		;         + delta_c / 16
	bcf	STATUS,C
	rrf	delta_c,F		;  ( div by 32 )
	bcf	STATUS,C
	rrf	delta_c,W		;  ( div by 64 )
	subwf	delta_s,W		;         - delta_c / 64

	movwf	delta_s			; delta_s is now new feed pwm value
	movf	pwmdesired,w
	subwf	delta_s,w		; W = new pwm value - pwmdesired
	btfss	STATUS,C		; skip if no borrow: new pwm value > pwmdesired
	goto	set_pwm_value		;    new pwm value is smaller, ok to ramp dowm immediately
	andlw	0xfc			;
	btfsc	STATUS,Z		; if delta < 4
	goto	set_pwm_value		;    set to new pwm value
	movlw	3			; else incr pwmdesired by 3
	btfss	init_done,0		; 	if not init_done, incr by 2
	movlw	2
	addwf	pwmdesired,f
	goto	chk_feed_xit
	
set_pwm_value
	movf	delta_s,w
	movwf	pwmdesired		; save current feed pwm value
chk_feed_xit:
	return

; at 13 seconds, if pot > 25% duty cycle, init feed pwm to 25% duty cycle
; at 14 seconds, if pot > 50% duty cycle, init feed pwm to 50% duty cycle
init_feed_pwm:				; this gets called before feed_inited is set
	movf	feed_a2d,w
	sublw	0x20			; if input < 0.4V (no borrow)
	btfss	STATUS,C
	goto	over_pt4_v
	decfsz	soft_pwm,w		;  and soft_pwm was on
	goto	init_feed_xit
	; disable software pwm
	bcf	INTCON,GIE		;  disable GLOBAL INTERRUPT ENABLE
	clrf	soft_pwm
	clrf	pwmdesired
	bcf	PORTC,PWMOUTBIT		;  force feed motor off
	bcf	led_bits,FEED_LED
	bcf	PORTB,FEED_LED		;  turn feed led off
	goto	init_feed_xit
over_pt4_v
	decfsz	soft_pwm,w		; if soft_pwm was off
	goto	init_enable_feed
	goto	init_update_feed
init_enable_feed:
	; enable software pwm
	bsf	PORTB,FEED_LED		; turn feed led on
	bsf	led_bits,FEED_LED
	clrf	TMR0			; reset timer 0
	movlw	INTCONVAL		; enable timer0 intr
	movwf	INTCON
	incf	soft_pwm,f		; indicate feed pwm active by setting soft_pwm
init_update_feed:				; check if feed motor pwm duty cycle need updating
	movf	feed_a2d,w		; *** go straight to feed_a2d value
	movwf	pwmdesired		; save current feed pwm value
init_feed_xit:
	return



update_sweep:
; turn on/off sweep motor based on a2d
	decfsz	sweep_inited,w		; if sweep not inited exit
	goto	sweep_exit
	btfss	remote_toggle,REMOTE_BIT; if remote not enable,
	goto	sweep_remote
	movf	sweep_a2d,w
;	sublw	0x33			;  if sweep switch < 1.0V (no borrow)
	sublw	0x99			;MJZ ADDED LINE, CHANGED TO < 3.0V TO TURN SWEEP ON REGARDLESS OF WHICH RESISTOR DIVIDER CIRCUIT ACTIVATED
	btfss	STATUS,C
	goto	sweep_off
sweep_on
	btfsc	sweep_is_on,0		;   if motor already enabled, exit
	goto	sweep_exit
	call	linesw_toggle_init	;MJZ ADDED LINE, IF SWEEP MOTOR WAS JUST ENABLED, INIT LINESW_TOGGLE TO MATCH 2-LINE SWITCH VALUE
	bsf	PORTC,SWEEP_EN		;    enable sweep motor
	bcf	rbif_bits,0		;    don't check for sweep motor error for 1 second	
	incf	sweep_is_on,f
	goto	sweep_exit1
sweep_off
	btfss	sweep_is_on,0
	goto	sweep_exit
	bcf	PORTC,SWEEP_EN		; else disable sweep motor
	clrf	sweep_is_on
sweep_exit1:
;	btfss	remote_toggle,REMOTE_BIT; if remote not enable,
;	goto	sweep_exit
;	movf	pwr_on_code,W		; 	if timeout or brown out
;	btfsc	STATUS,Z
;	goto	sweep_exit
;	movwf	flash_code		; 		flash the reset type code 2 time
;	call	flash_error
;	movf	pwr_on_code,W
;	movwf	flash_code
;	call	flash_error
;	bcf	led_bits,GREEN_LED
sweep_exit:
	return

sweep_remote:				; remote switch on
	btfss	remote_toggle,SWEEP_BIT ;  set SWEEP output based on SWEEP_BIT
	goto	sweep_off
	goto	sweep_on

; check if remote enable switch had changed
update_remote:
	decfsz	init_done,W		; if init not done, don't bother
	goto	update_rmxit
	movf	remote_toggle,W		; pick up remote_toggle byte
	andlw	REMOTE_ON
	xorwf	PORTC,W			; exclusive or with PORTC
	andlw	REMOTE_ON		; did REMOTE_ON change value
	btfsc	STATUS,Z		; if no change, exit
	goto	update_rmxit
;	movlw	0x20
;	movwf	remote_block		; flush usart
	movlw	REMOTE_ON
	xorwf	remote_toggle,F
	btfsc	remote_toggle,REMOTE_BIT ; if remote switch just turned on,
	goto	remote_off

;	call	usart_enable
	bsf	STATUS,RP0		; goto bank 1
	movlw	0x81			; 2400 baud
	movwf	SPBRG
	movlw	0x20			; 8-bit transimit, transmitter enabled
	movwf	TXSTA			; asynchronous mode, low speed mode
	bcf	STATUS,RP0		; goto bank 0
	movlw	0x90			; 8-bit receive, receiver enabled
	movwf	RCSTA			; serial port enabled
	movf	RCREG,W			; flush input data
	movf	RCREG,W			; flush input data

;	movlw	REMOTE_ON
;	iorlw	FEED_TOGGLE
;	movwf	remote_toggle		; then set remote and feed toggles
	bsf	remote_toggle,FEED_BIT	;   set feed toggle
	btfsc	sweep_is_on,0		;      if sweep is on
	bsf	remote_toggle,SWEEP_BIT	;         set sweep bit
	movf	sweep_is_on,W		; copy sweep_is_on flag to last_sweep
	movwf	last_sweep
	movlw	BLOCK_CNT-8		; short beep
	movwf	remote_block
	call	usart_disable
update_rmxit:
	return
remote_off:
;	call	usart_off
	bcf	remote_toggle,SWEEP_BIT	; turn off sweep_toggle
	goto	usart_off
	return
;
; acquire reading on analog channel
; enters: (a2d_chan) = chanel number
; exits: W = result
acq_a2d
	swapf	a2d_chan,F		; move channel number
	bcf	STATUS,C		;  clear carry bit before rotate
	rrf	a2d_chan,W		;  bits 5-3 in Wreg
	iorlw	ADCON0VAL		; or in FOSC/32, ADON
	movwf	ADCON0			;  set ADC configuration and selecting analog chan
					;;acquisition delay - incr channel number for next op
	swapf	a2d_chan,F		; restore channel number
	incf	a2d_chan,F		; incr channel number
	movlw	5			; mod 5 - only channels 0-4 valid
	subwf	a2d_chan,W
	btfsc	STATUS,Z
	movwf	a2d_chan		;      wrap to 0 if next chan is 5

	movlw	0x18			; need 19.6us acquisition time
	movwf	delay
delaylp:
	decfsz	delay,f
	goto	delaylp
WaitNoInt
	movf	TMR0,W			; waiting till enough time
	sublw	0d0h			; for one conversion before start
	btfss	STATUS,C		; of the next timer interrupt
	goto	WaitNoInt		; (Conv can be disturbed by an intr)
	bsf	ADCON0,GO		; start ADC
WaitAdc
	btfsc	ADCON0,GO		; waiting until ACDC...
	goto	WaitAdc			; ... is ready
; result is waiting in ADRES, but first select next channel and start acquisition
	swapf	a2d_chan,F		; restore channel number
	bcf	STATUS,C		;  clear carry bit before rotate
	rrf	a2d_chan,W		;  bits 5-3 in Wreg
	iorlw	ADCON0VAL		; or in FOSC/32, ADON
	movwf	ADCON0			;  set ADC configuration and selecting analog chan
	swapf	a2d_chan,F		; restore channel number

	movf	ADRES,W			; put result into W-reg
	clrwdt				; clear watch dog timer
	return

; calculate desired bottom pwm
calc_bot_pwm
; first calculate speed based on no spin
; 08/07/09 start
;	movlw	0x8f
	movlw	0x8f			; minimum speed pot read allowed
; 08/07/09 end
	subwf	spd_a2d,w
	movwf	delta_s			; delta_s = speed_a2d - 143
; 08/07/09 start
;	addlw	0x3d			; calc_pwm = 61 + delta_s
;	addlw	0x3d			; calc_pwm = 61 + delta_s, 61/255 = 24% duty cycle minimum pwm/speed?
	addlw	0x4a			; calc_pwm = 61 + delta_s, 72/255 = 29% duty cycle minimum pwm/speed?
; 08/07/09 end
	movwf	calc_pwm
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 2)
; 08/07/09 start
;	movf	delta_s,w
;	addwf	calc_pwm,F		;               + delta_s / 2
; 08/07/09 end
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	movf	delta_s,w
	addwf	calc_pwm,F		;               + delta_s / 4
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 8)
; 08/07/09 start
	movf	delta_s,w
	addwf	calc_pwm,F		;               + delta_s / 8
; 08/07/09 end
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 16)
; 08/07/09 start
	subwf	calc_pwm,f		;               - delta_s / 16
; 08/07/09 end
;	bcf	STATUS,C
;	rrf	delta_s,W		; (div by 32)
;	subwf	calc_pwm,f		;               - delta_s / 32
; 08/07/09 end
; now adjust based on spin
	movf	spin_a2d,w
	sublw	0xba			; 186 - spin_a2d
	btfsc	STATUS,Z		; if 186 <= spin_a2d <= 191, then no spin
	goto	no_spin1
	btfsc	STATUS,C		; if spin_a2d < 186, then we have back spin (skip if borrow)
	goto	adjust_backspin1
	addlw	5			;
	btfsc	STATUS,C		; if spin_a2d is 187,188,189,190 or 191, then no spin
	goto	no_spin1
	goto	adjust_topspin1
adjust_backspin1:			; if spin_a2d < 186 then we have back spin
	movlw	0xc5
	subwf	spd_a2d,W		; speed_a2d - 197
	btfsc	STATUS,C		; skip if borrow (slow speed)
	goto	high_spd1
	movlw	0x8f			; slow speed
	subwf	spd_a2d,w
	movwf	delta_s			; delta_s = speed_a2d - 143
	movwf	mult1			; mult1 = 	delta_s
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	movf	delta_s,w
	addwf	mult1,f			;		+ delta_s / 4
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 8)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 16)
	movf	delta_s,w
	subwf	mult1,f			;		- delta_s / 16
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 32)
	addwf	mult1,f			;		+ delta_s / 32
	movlw	0x0b
	movwf	mult2
	call	mult_sub		; W = 11 * mult1 / 64
	movwf	mult1			; mult1 is maxspin for this speed
	goto	adjust_bs1
high_spd1:
	movlw	0xc5
	subwf	spd_a2d,W		; speed_a2d - 197
	movwf	delta_s			; delta_s = speed_a2d - 197
	movwf	mult1
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	movf	delta_s,w
	addwf	mult1,f			;		+ delta_s / 4
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 8)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 16)
	movf	delta_s,w
	subwf	mult1,f			;		- delta_s / 16
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 32)
	addwf	mult1,f			;		+ delta_s / 32
	movlw	0x0b
	movwf	mult2
	call	mult_sub		; W = 11 * mult1 / 64
	sublw	0x0b			; W = 11 - 11 * mult1 / 64
	movwf	mult1			; mul1 is maxspin for this speed
adjust_bs1:
	movf	spin_a2d,w
	sublw	0xba			; 186 - spin_a2d
	movwf	delta_s			; amount of back spin desired (delta back_spin)
	movwf	mult2			; max delta spin = delta_s
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 2)
	addwf	mult2,f			;		+ delta_s / 2
	call	mult_sub		; W = maxspin * (delta back_spin) / 64
	addwf	calc_pwm,f		; calc_pwm = calc_pwm + maxspin * (delta back_spin) / 64
	goto	no_spin1

adjust_topspin1:
	movlw	0xbf			; mult1 = spin_a2d - 191
	subwf	spin_a2d,w
	movwf	mult1			; max top spin = delta_spin
	movwf	delta_s
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 8)
	movf	delta_s,w
	addwf	mult1,f			;		+ delta_spin / 8 
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 16)
	movf	delta_s,w
	subwf	mult1,f			;		- delta_spin / 16
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 32)
	addwf	mult1,f			;               + delta_spin / 32

	movlw	0xc5
	subwf	spd_a2d,W		; delta speed = speed_a2d - 197
	btfss	STATUS,C		; skip if no borrow (fast speed)
	goto	slow_spd
					; fast speed:
	movwf	delta_s			; maxspin = delta_speed / 2
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	movf	delta_s,w
	movwf	mult2
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 4)
	bcf	STATUS,C
	rrf	delta_s,W		; (div by 8)
	subwf	mult2,W			;           - delta_speed / 8
	addlw	0x29			;	    + 41
	goto	ts_com1

slow_spd
	movlw	0x29			; slow speed: maxspin = 41
ts_com1
	subwf	calc_pwm,w
	movwf	mult2			; mult2 = calc_pwm - maxspin
	call	mult_sub		; W = (max top spin) * (calc_pwm - maxspin) / 64
	subwf	calc_pwm,f		; calc_pwm = calc_pwm - (196 - spin_a2d) * (calc_pwm - maxspin) / 64
no_spin1
;	movf	calc_pwm,w		; DEBUG - force 100% PWM if 96%
;	sublw	0xf3			; DEBUG
;	btfss	STATUS,C		; DEBUG if 96%, goto force 100%
;	goto	force255		; DEBUG
	movf	calc_pwm,w
	movwf	t_bot_pwm
	sublw	0x1f			; 41 - t_bot_pwm
	btfss	STATUS,C		; skip if no borrow
	return
	movlw	0x1f
	movwf	t_bot_pwm
	return				;  make at least 41
;force255				; DEBUG
;	movlw	0xff			; DEBUG
;	movwf	t_bot_pwm		; DEBUG
;	return				; DEBUG

; calculate desired top pwm
calc_top_pwm
; first calculate speed based on no spin
; 08/07/09 start
;	movlw	0x8f
	movlw	0x8f			; minimum speed pot read allowed
; 08/07/09 end
	subwf	spd_a2d,w
	movwf	delta_s			; delta_s = speed_a2d - 143
; 08/07/09 start
;	addlw	0x3d			; calc_pwm = 61 + delta_s
;	addlw	0x3d			; calc_pwm = 61 + delta_s, 61/255 = 24% duty cycle minimum pwm/speed?
	addlw	0x4a			; calc_pwm = 61 + delta_s, 74/255 = 29% duty cycle minimum pwm/speed?
; 08/07/09 end
	movwf	calc_pwm
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 2)
; 08/07/09 start
;	movf	delta_s,w
;	addwf	calc_pwm,F		;               + delta_s / 2
; 08/07/09 end
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	movf	delta_s,w
	addwf	calc_pwm,F		;               + delta_s / 4
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 8)
; 08/07/09 start
	movf	delta_s,w
	addwf	calc_pwm,F		;               + delta_s / 8
; 08/07/09 end
	bcf	STATUS,C
	rrf	delta_s,F		; (div by 16)
; 08/07/09 start
	subwf	calc_pwm,f		;               - delta_s / 16
; 08/07/09 end
;	bcf	STATUS,C
;	rrf	delta_s,W		; (div by 32)
;	subwf	calc_pwm,f		;               - delta_s / 32
; 08/07/09 end
; now adjust based on spin
	movf	spin_a2d,w
	sublw	0xba			; 186 - spin_a2d
	btfsc	STATUS,Z		; if spin_a2d is 186, then no spin
	goto	no_spin
	btfsc	STATUS,C		; if spin_a2d < 186, then we have back spin (skip if borrow)
	goto	adjust_backspin
	addlw	5			;
	btfsc	STATUS,C		; if spin_a2d is 187-191, then no spin
	goto	no_spin
adjust_topspin:				; if spin_a2d > 197 then we have top spin
	movlw	0xc5
	subwf	spd_a2d,W		; speed_a2d - 197
	btfsc	STATUS,C		; skip if borrow (slow speed)
	goto	high_spd
	movlw	0x8f			; slow speed
	subwf	spd_a2d,w
	movwf	delta_s			; delta_s = speed_a2d - 143
	movwf	mult1			; mult1 = 	delta_s
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	movf	delta_s,w
	addwf	mult1,f			;		+ delta_s / 4
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 8)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 16)
	movf	delta_s,w
	subwf	mult1,f			;		- delta_s / 16
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 32)
	addwf	mult1,f			;		+ delta_s / 32
	movlw	0x0b
	movwf	mult2
	call	mult_sub		; W = 11 * mult1 / 64
	movwf	mult1			; mult1 is maxspin for this speed
	goto	adjust_ts
high_spd:
	movlw	0xc5
	subwf	spd_a2d,W		; speed_a2d - 197
	movwf	delta_s			; delta_s = speed_a2d - 197
	movwf	mult1
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	movf	delta_s,w
	addwf	mult1,f			;		+ delta_s / 4
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 8)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 16)
	movf	delta_s,w
	subwf	mult1,f			;		- delta_s / 16
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 32)
	addwf	mult1,f			;		+ delta_s / 32
	movlw	0x0b
	movwf	mult2
	call	mult_sub		; W = 11 * mult1 / 64
	sublw	0x0b			; W = 11 - 11 * mult1 / 64
	movwf	mult1			; mul1 is maxspin for this speed
adjust_ts:
	movlw	0xbf			; spin_a2d - 191
	subwf	spin_a2d,w
	movwf	delta_s			; amount of top spin desired (delta top_spin)
	movwf	mult2			; max delta spin = delts_s
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 8)
	movf	delta_s,w
	addwf	mult2,f			;		+ delta_s / 8
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 16)
	movf	delta_s,w
	subwf	mult2,f			;		- delta_s / 16
	bcf	STATUS,C
	rrf	delta_s,w		; (div by 32)
	addwf	mult2,f			;		+ delta_s / 32
	call	mult_sub		; W = maxspin * (delta top_spin) / 64
	addwf	calc_pwm,f
	goto	no_spin

adjust_backspin:
	movf	spin_a2d,w
	sublw	0xba			; mult1 = 186 - spin_a2d
	movwf	mult1
	bcf	STATUS,C		; max backspin = (186 - spin_a2d) * 3 / 2
	rrf	mult1,w
	addwf	mult1,f

	movlw	0xc5
	subwf	spd_a2d,W		; delta speed = speed_a2d - 197
	btfss	STATUS,C		; skip if no borrow (fast speed)
	goto	slow_spd1
					; fast speed:
	movwf	delta_s			; maxspin =
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 2)
	movf	delta_s,w		;           delta_speed / 2
	movwf	mult2
	bcf	STATUS,C
	rrf	delta_s,f		; (div by 4)
	bcf	STATUS,C		
	rrf	delta_s,w		; (div by 8)
	subwf	mult2,w			;           - delta_speed / 8
	addlw	0x29			;           + 41
	goto	bs_com

slow_spd1
	movlw	0x29
bs_com
	subwf	calc_pwm,w
	movwf	mult2			; mult2 = calc_pwm - maxspin
	call	mult_sub		; W = (max back spin) * (calc_pwm - maxspin) / 64
	subwf	calc_pwm,f		; calc_pwm -= W
no_spin
;	movf	calc_pwm,w		; DEBUG - force 100% PWM if 96%
;	sublw	0xf3			; DEBUG
;	btfss	STATUS,C		; DEBUG if 96%, goto force 100%
;	goto	force255_1		; DEBUG
	movf	calc_pwm,w
	movwf	t_top_pwm
	return
;force255_1				; DEBUG
;	movwf	0xff			; DEBUG
;	movwf	t_top_pwm		; DEBUG
;	return				; DEBUG

power_is_off:
; here at power on if detected the power switch is off
;	btfsc	PORTB,0			; if RB0/INT bit is not set, power had been turned back on
;	clrwdt				;  let wdt reset processor

	clrf	PORTB			; turn off all LED
	movf	bat_a2d,w		; get battery level
  	sublw	99h			; battery input at least 3.0V? (battery at least 13.2V)
	btfsc	STATUS,C		;    skip if no borrow (> 3.0V)
	goto	charg_lt_3_0		; charging, below 3.0V
	btfsc	STATUS,Z
	goto	charg_lt_3_0
	movf	bat_a2d,w
;	sublw	0a3h			; battery input at least 3.2V? (battery at least 14.1V), MJZ COMMENTED THIS LINE
	sublw	09fh			;MJZ ADDED THIS LINE, battery input at least 3.1V? (battery at least 13.7V)
	btfsc	STATUS,C		;    skip if no borrow (> 3.2V)
	goto	charg_lt_3_2		; charging, below 3.2V
	btfsc	STATUS,Z
	goto	charg_lt_3_2
	movf	bat_a2d,w
;	sublw	0a8h			; battery input at least 3.3V? (battery at least 14.5V), MJZ COMMENTED THIS LINE
	sublw	0a3h			;MJZ ADDED THIS LINE, battery input at least 3.2V? (battery at least 14.1V)
	btfsc	STATUS,C		;    skip if no borrow (> 3.3V)
	goto	charg_lt_3_3
	btfsc	STATUS,Z
	goto	charg_lt_3_3
					; we are here because we are charging and power > 3.3V
	bsf	PORTB,GREEN_LED		;   green on and red off
wait_reset_lp:
	clrwdt
;	btfss	PORTB,0		; if RB0/INT bit is not set, power had been turned back on, MJZ COMMENTED THIS LINE
	btfss	PORTB,4		;MJZ ADDED THIS LINE, POWER SWITCH NOW ON RB4
	bsf	PORTB,RED_LED		;  set red led
	goto	wait_reset_lp		; loop forever until charging power removed and system reset
					; 
					; 

charg_lt_3_3:				; charging, 3.2V < power level < 3.3V
	goto	flash_grn		; we want red off and flashing green
charg_lt_3_2:				; charging, 3.0V < power level < 3.2V
;	bsf	PORTB,RED_LED		; we want red on and flashing green, MJZ COMMENTED THIS LINE
	goto	flash_red		;MJZ ADDED THIS LINE, we want flashing red and green off
flash_grn:
	bsf	PORTB,GREEN_LED		; turn on green led
	movlw	GREEN_LED_ON
	movwf	mult2			; bits to toggle
	goto	flash_com

flash_red:					;MJZ ADDED THIS LINE
	bsf PORTB,RED_LED		;MJZ ADDED THIS LINE, turn on red led
	movlw	RED_LED_ON		;MJZ ADDED THIS LINE
	movwf	mult2			;MJZ ADDED THIS LINE
	goto	flash_com		;MJZ ADDED THIS LINE

charg_lt_3_0:				; charging, power level <= 3.0V
;	bsf	PORTB,GREEN_LED		; turn on green led, MJZ COMMENTED THIS LINE
;	movlw	GREEN_LED_ON|RED_LED_ON	; toggle both red and green, only 1 on at a time, MJZ COMMENTED THIS LINE
;	movwf	mult2			;MJZ COMMENTED THIS LINE

	bsf	PORTB,RED_LED		;MJZ ADDED THIS LINE, we want red on and green off
	movlw	0				;MJZ ADDED THIS LINE
	movwf	mult2			;MJZ ADDED THIS LINE



flash_com:
	movlw	0x40
	movwf	mult1
fgrn_loop:
	movlw	PAUSE_TIME
	call	wait
	movlw	PAUSE_TIME
	call	wait
	movlw	PAUSE_TIME
	call	wait
	movf	mult2,w			; get bits to toggle
	xorwf	PORTB,F			; toggle leds
	decfsz	mult1,f
	goto	fgrn_loop
;	btfsc	PORTB,0			; if RB0/INT bit is not set, power had been turned back on	;MJZ COMMENTED THIS LINE
	btfsc	PORTB,4			;MJZ ADDED THIS LINE, POWER SWITCH NOW ON RB4
	goto	pwr_off_lp		;  let wdt reset the system
	movlw	2
	movwf	a2d_chan		; get battery level again
	call	acq_a2d
	movwf	bat_a2d		
	goto	power_is_off


; check if RB7:RB4 pins had changed state
chk_rbif:
;	btfss	INTCON,INTF		; if RB0/INT flag is set, check if power switch is turned off
;	btfss	PORTB,0			; if power switch is turned off	;MJZ COMMENTED THIS LINE
	btfss	PORTB,4			;MJZ ADDED THIS LINE, POWER SWITCH NOW ON RB4
	goto	chk_rbif3
;	btfss	PORTB,0			; check power switch again	;MJZ COMMENTED THIS LINE
	btfss	PORTB,4			;MJZ ADDED THIS LINE, POWER SWITCH NOW ON RB4
	goto	chk_rbif3
	clrf	INTCON			; disable all intrs
	clrf	CCP1CON			; CCP1 Module is off
	clrf	CCP2CON			; CCP2 Module is off
;	clrf	PORTC			; disable feed motor pwm
	movlw	BEEP_OFF		; beep is off
	movwf	PORTC			; disable feed motor pwm
					; disable bottom serve motor pwm
					; disable top serve motor pwm
					; sweep motor disabled
					; elevation motor disabled
	clrf	PORTB			; turn off all LED
	clrwdt
	bsf	PORTB,GREEN_LED
;	movlw	0x40
	movlw	0x80
	movwf	mult1
pwr_off_loop:
	movlw	PAUSE_TIME
	call	wait
	movlw	6			; red led on + green led on
	xorwf	PORTB,F			; toggle red and green led, only 1 on
	decfsz	mult1,f
	goto	pwr_off_loop

	clrf	PORTB			; turn off all leds
pwr_off_lp:
;	btfsc	PORTB,0			; if RB0/INT bit is not set, power had been turned back on
;	clrwdt				;  let wdt reset processor
	goto	pwr_off_lp

chk_rbif3:
;	btfss	INTCON,INTF		; if RB0/INT flag is set,	;MJZ COMMENTED THIS LINE
	goto	chk_rbif1



;MJZ ADDED LINE, PREVIOUS VERSION HAD POWER SWITCH ON RB0/INT0.
;MJZ ADDED LINE, CHK_RBIF2 WOULD HAVE BEEN RUN ONLY IF A POWER OFF ON RB0/INT0 SET INTF, BUT
;MJZ ADDED LINE, WHEN RB0 WAS READ, THE POWER OFF CONDITION HAD ALREADY GONE AWAY, I.E. A SPURIOUS SIGNAL SET INTF.
;MJZ ADDED LINE, THE INTF WAS CHECKED FOR IN CHK_RBIF3, SO TO BE SAFE THAT LINE IS NOW COMMENTED SO CHK_RBIF2 WOULD NEVER RUN.
;MJZ ADDED LINE, POWER OFF IS NOW DETECTED ON RB4 WHICH USED TO BE SWEEP MOTOR ERROR DETECTION.
;MJZ ADDED LINE, SWEEP MOTOR ERROR USED TO BE ON RB4, BUT NONE OF THAT LOGIC IS TAKEN OUT, BECAUSE THE POWER OFF DETECTION CODE
;MJZ ADDED LINE, IS EXECUTED BEFORE MOTOR ERROR CODE, SO IF RB4 IS SET, A POWER OFF IS DETECTED.
;MJZ ADDED LINE, BALL DROP IS NOW DETECTED ON RB0/INT0.



chk_rbif2:
	movlw	0xf9
	andwf	PORTB,F			; clear red and green led
	movlw	PAUSE_TIME
	call	wait
	bsf	PORTB,GREEN_LED
	movlw	PAUSE_TIME
	call	wait
	movlw	6
	xorwf	PORTB,F
	movlw	PAUSE_TIME
	call	wait
	movlw	6
	xorwf	PORTB,F
	movlw	0xf9
	andwf	PORTB,F
	andwf	led_bits,F
;	bcf	INTCON,INTF			; MJZ commented this line, INTF now used for ball drop detection.
chk_rbif1:
	btfss	INTCON,RBIF		; if RBIF is clear, return
	return
	swapf	PORTB,w			; get RB7:RB4 in lower bits
	bcf	INTCON,RBIF		;  clear RBIF bit in INTCON
;	andlw	0x0f			;  clear upper bits
	andwf	rbif_bits,w		;  and with mask
	btfsc	STATUS,Z		;  if zero, return
	return
	movlw	6			; wait 10us
	movwf	rbif_wait
rbif_lp
	swapf	PORTB,w			; double checking, get RB7:RB4 in lower bits
;	andlw	0x0f			;  clear upper bits
	andwf	rbif_bits,w		;  and with mask
	btfsc	STATUS,Z		;  if zero, return
	return
	decfsz	rbif_wait,f
	goto	rbif_lp
	swapf	PORTB,w			; get RB7:RB4 in lower bits
;	andlw	0x0f			;  clear upper bits
	andwf	rbif_bits,w		; and with mask
	btfsc	STATUS,Z		;  if zero, return
	return
	movwf	err_no			; save as error number
; motor error, disable all motors, flash error code
;
motor_error
	; disable all motors
	clrwdt
	clrf	INTCON			; disable all intrs
	clrf	CCP1CON			; CCP1 Module is off
	clrf	CCP2CON			; CCP2 Module is off
;	clrf	PORTC			; disable feed motor pwm
	movlw	BEEP_OFF		; disable feed motor pwm
	movwf	PORTC			; disable bottom serve motor pwm
					; disable top serve motor pwm
					; sweep motor disabled
					; eleveation motor disabled
	clrf	PORTB			; turn off all LED
moterr_lp
	movlw	2
	movwf	flash_code
	movf	err_no,w		; get motor error bits
	btfsc	err_no,1		; feed motor error
	call	flash_error		;  flash 2
	movlw	3
	movwf	flash_code
	btfsc	err_no,3		; top serve motor error
	call	flash_error		;  flash 3
	movlw	4
	movwf	flash_code
	btfsc	err_no,2		; bottom serve motor error
	call	flash_error		;  flash 4
	movlw	5
	movwf	flash_code
;	btfsc	err_no,0		; feed motor error	;MJZ COMMENTED THIS LINE, SHOULD BE SWEEP MOTOR ERROR
	btfsc	err_no,0		; MJZ ADDED THIS LINE, SWEEP MOTOR ERROR
	call	flash_error		;  flash 5
	goto	moterr_lp

flash_error:
;	movwf	flash_code		; for err_no times,
	bcf	PORTB,GREEN_LED		;   turn off green led
	movlw	FLASH_TIME
	call	wait
flash_greenled
	bsf	PORTB,GREEN_LED		;   turn on green led
	movlw	FLASH_TIME
	call	wait
	bcf	PORTB,GREEN_LED		;   turn off green led
	movlw	FLASH_TIME
	call	wait
	decfsz	flash_code,f
	goto	flash_greenled

	movlw	PAUSE_TIME
	movwf	flash_code
pause
	movlw	FLASH_TIME		; now pause
	call	wait
	decfsz	flash_code,f
	goto	pause
	return


; battery below 2.4V - disable all motors, LEDs green off and red flashing
bat_dead
	; disable all motors
	clrwdt
	clrf	INTCON			; disable all intrs
	clrf	CCP1CON			; CCP1 Module is off
	clrf	CCP2CON			; CCP2 Module is off
;	clrf	PORTC			; disable feed motor pwm
	movlw	BEEP_OFF		; disable feed motor pwm
	movwf	PORTC			; disable bottom serve motor pwm
					; disable top serve motor pwm
					; sweep motor disabled
					; elevation motor disabled
	clrf	PORTB			; turn off all LED

;flash_redled				;MJZ COMMENTED THIS LINE
;	movlw	RED_LED_ON		;MJZ COMMENTED THIS LINE
;	xorwf	PORTB,f			; toggle red LED, MJZ COMMENTED THIS LINE
;	movlw	FLASH_TIME		;MJZ COMMENTED THIS LINE
;	call	wait			;MJZ COMMENTED THIS LINE
;	goto	flash_redled	;MJZ COMMENTED THIS LINE


bat_dead_loop				;MJZ ADDED THIS LINE
	bsf		PORTB,RED_LED	;MJZ ADDED THIS LINE, TURN ON RED LED
	clrwdt
	goto	bat_dead_loop	;MJZ ADDED THIS LINE

	

; wait (W) number of timer1 overflows
; if (W) = 255, wait time is about 1.2 seconds
wait
	movwf	wait_cnt
waitlp
	clrwdt
	btfss	PIR1,TMR1IF		; wait for timer1 to overflow
	goto	waitlp
	movlw	MSEC5_LSB		; reset  timer1 to count up to 5 msec
	movwf	TMR1L
	movlw	MSEC5_MSB
	movwf	TMR1H
	bcf	PIR1,TMR1IF		; clear timer1 overflow flag
	decfsz	wait_cnt,f		;  decrement wait_cnt
	goto	waitlp
	return

; wait (W) number of timer1 overflows withou clearing watch dog timer
; if (W) = 255, wait time is about 1.2 seconds
wait_no_clrwdt
	movwf	wait_cnt
waitlp1
	btfss	PIR1,TMR1IF		; wait for timer1 to overflow
	goto	waitlp1
	movlw	MSEC5_LSB		; reset  timer1 to count up to 5 msec
	movwf	TMR1L
	movlw	MSEC5_MSB
	movwf	TMR1H
	bcf	PIR1,TMR1IF		; clear timer1 overflow flag
	decfsz	wait_cnt,f		;  decrement wait_cnt
	goto	waitlp1
	return

; initialize battery level on power up	
init_bat_level:
	clrf	power_level
	movf	bat_a2d,w
	sublw	0x84			; 0x84 - bat_a2d
	btfss	STATUS,C		; if no borrow, battery a2d is less than 2.6V
	goto	init_pl_com			; if borrow, over 2.6V: power_level_tmp = 0
	incf	power_level,f
	movf	bat_a2d,w
	sublw	0x80			; 0x80 - bat_a2d
	btfss	STATUS,C		; if no borrow, battery a2d is less than 2.5V
	goto	init_pl_com		; if borrow, over 2.5V: power_level_tmp = 1
; less than 2.5V, about 10 mins left
	incf	power_level,f
	movf	bat_a2d,w
	sublw	0x7a			; if battery less than 2.4V (0x7a - W)	
	btfsc	STATUS,C		;  	skip if borrow set ( > 2.4V ): power_level_tmp = 2
	incf	power_level,f	;       if borrow, less then 2.4V: power_level_tmp = 3
init_pl_com:
	movf	power_level,w
	movwf	power_level_r
; fall through to set battery leds


; update battery leds. Called every 16 seconds. Take the lowest value from the last 16 second period
update_bat_led:
	movf	power_level_r,w
	movwf	power_level
	andlw	0x03
	btfsc	STATUS,Z		; power level 0: > 2.6V
	goto	over2_6v
	decfsz	power_level,f		;
	goto	under2_5v
	goto	over2_5v		; power level 1: > 2.5V


under2_5v
	decfsz	power_level,f
	goto	bat_dead			; power level 3: < 2.4V ;MJZ DEBUG, COMMENTED THIS LINE
;	goto	over2_6v			;MJZ DEBUG, ADDED THIS LINE

;between_2.4v_2.5v				; less than 2.5V, about 10 mins left (GREEN OFF RED ON), MJZ COMMENTED THIS LINE
;	btfss	led_bits,GREEN_LED	; if green led on, turn it off, MJZ COMMENTED THIS LINE
;	goto	grn_off				;MJZ COMMENTED THIS LINE
;	bcf	led_bits,GREEN_LED		;MJZ COMMENTED THIS LINE
;	bcf	PORTB,GREEN_LED			;MJZ COMMENTED THIS LINE
between_2.4v_2.5v:				;MJZ ADDED THIS LINE, BETWEEN 2.4V & 2.5V, GREEN OFF, RED ON(FLASHING)
	bcf		PORTB,GREEN_LED		;MJZ ADDED THIS LINE
	bsf		PORTB,RED_LED		;MJZ ADDED THIS LINE
	goto	bat_led_xit			;MJZ ADDED THIS LINE


grn_off:
	btfsc	led_bits,RED_LED	; if red led off, turn it on
	goto	bat_led_xit
	bsf	led_bits,RED_LED
	bsf	PORTB,RED_LED
	goto	bat_led_xit



;over2_5v:						; between 2.5v and 2.7V - 1/3 capacity (GREEN ON RED ON), MJZ COMMENTED THIS LINE
;	btfsc	led_bits,GREEN_LED	; if green led off, turn it on, MJZ COMMENTED THIS LINE
;	goto	grn_off				;MJZ COMMENTED THIS LINE
;	bsf		led_bits,GREEN_LED	;MJZ COMMENTED THIS LINE
;	bsf		PORTB,GREEN_LED		;MJZ COMMENTED THIS LINE
;	goto	grn_off				; goto turn red led on if necessary, MJZ COMMENTED THIS LINE
over2_5v:						;MJZ ADDED THIS LINE, BETWEEN 2.5V & 2.7V - 1/3 CAPACITY (GREEN ON(FLASHING), RED OFF)
	bcf		PORTB,RED_LED		;MJZ ADDED THIS LINE, TURN OFF RED LED
	bsf		PORTB,GREEN_LED		;MJZ ADDED THIS LINE, TURN ON GREEN LED
	goto	bat_led_xit			;MJZ ADDED THIS LINE


;over2_6v:						; greater than 2.6V, battery good	(GREEN ON RED OFF), MJZ COMMENTED THIS LINE
;	btfsc	led_bits,GREEN_LED	; if green led off, turn it on, MJZ COMMENTED THIS LINE
;	goto	grn_on
;	bsf	led_bits,GREEN_LED		;MJZ COMMENTED THIS LINE
;	bsf	PORTB,GREEN_LED			;MJZ COMMENTED THIS LINE
over2_6v:						;MJZ ADDED THIS LINE, GREATER THAN 2.6V, BATTERY GOOD (GREEN ON, RED OFF)
	bcf		PORTB,RED_LED		;MJZ ADDED THIS LINE, TURN OFF RED LED
	bsf		PORTB,GREEN_LED		;MJZ ADDED THIS LINE, TURN ON GREEN LED
	goto	bat_led_xit			;MJZ ADDED THIS LINE	


grn_on:
	btfss	led_bits,RED_LED	; if red led on, turn it off
	goto	bat_led_xit
	bcf	led_bits,RED_LED
	bcf	PORTB,RED_LED

bat_led_xit:
	movf	power_level_r,w
	movwf	power_level
	movlw	3
	movwf	power_level_r
	return	


record_bat_level:
; within a 16second time period, record the lowest power level
	clrf	power_level_tmp
	movf	bat_a2d,w
	sublw	0x84			; 0x84 - bat_a2d
	btfss	STATUS,C		; if no borrow, battery a2d is less than 2.6V
	goto	pl_com			; if borrow, over 2.6V: power_level_tmp = 0
	incf	power_level_tmp,f
	movf	bat_a2d,w
	sublw	0x80			; 0x80 - bat_a2d
	btfss	STATUS,C		; if no borrow, battery a2d is less than 2.5V
	goto	pl_com			; if borrow, over 2.5V: power_level_tmp = 1
; less than 2.5V, about 10 mins left
	incf	power_level_tmp,f
	movf	bat_a2d,w
	sublw	0x7a			; if battery less than 2.4V (0x7a - W)	
	btfsc	STATUS,C		;  	skip if borrow set ( > 2.4V ): power_level_tmp = 2
	incf	power_level_tmp,f	;       if borrow, less then 2.4V: power_level_tmp = 3
pl_com:
	movf	power_level_r,w		; power_level_r - power_level_tmp
	subwf	power_level_tmp,w
	btfsc	STATUS,C		; if borrow => power_level_tmp < power_level_r
	goto	no_bat_adjust
	movf	power_level_tmp,w	;   power_level_r = new power level
	movwf	power_level_r
no_bat_adjust:
	return	

debug_led:
	movf	feed_a2d,w
	movwf	flash_code
	rrf	flash_code,F
	rrf	flash_code,F
	rrf	flash_code,F
	rrf	flash_code,F
	rrf	flash_code,F
	rrf	flash_code,W
	andlw	3
	addlw	1
	movwf	flash_code
	call	flash_error
	movf	feed_a2d,w
	movwf	flash_code
	rrf	flash_code,F
	rrf	flash_code,F
	rrf	flash_code,W
	andlw	7
	addlw	1
	movwf	flash_code
	call	flash_error
	movf	feed_a2d,w
	andlw	7
	addlw	1
	movwf	flash_code
	call	flash_error

	movlw	3
	movwf	a2d_chan
	call	acq_a2d			; acquire chan3 a2d - feed rate
	movwf	feed_a2d
	clrwdt
	goto	debug_led

; special 16 bit multiply routine
;  temp:mult2 = mult2 x mult1
; returns temp:mult2 / 64
; returnw r2:r1 = mult2 x mult1
;
mult_sub:
	movf	mult1,W			; if either mult1 or mult2 is 0
	btfsc	STATUS,Z
	goto	mult_ret_0		; returns 0
	movf	mult2,W			; if either mult1 or mult2 is 0
	btfsc	STATUS,Z
	goto	mult_ret_0		; returns 0
	clrf	mult2
	clrf	temp
mult_lp:
	addwf	mult2,f			; add mult2 mult1 times (mult2 x mult1)
	btfsc	STATUS,C
	incf	temp,f
	decfsz	mult1,F
	goto	mult_lp
	movf	mult2,w
	movwf	r1
	movf	temp,w
	movwf	r2
	rlf	mult2,f			; div result by 64
	rlf	temp,f
	rlf	mult2,f
	rlf	temp,w
	return

mult_ret_0
	retlw	0



; special 16 bit multiply routine
;  temp:mult2 = mult2 x mult1
; returns (temp:mult2 + 63) / 64
;
mult_sub1:
	movf	mult1,W			; if either mult1 or mult2 is 0
	btfsc	STATUS,Z
	goto	mult_ret_0		; returns 0
	movf	mult2,W			; if either mult1 or mult2 is 0
	btfsc	STATUS,Z
	goto	mult_ret_0		; returns 0
	clrf	mult2
	clrf	temp
mult_lp1:
	addwf	mult2,f			; add mult2 mult1 times (mult2 x mult1)
	btfsc	STATUS,C
	incf	temp,f
	decfsz	mult1,F
	goto	mult_lp1
	movlw	0x3f
	addwf	mult2,f			
	btfsc	STATUS,C
	incf	temp,f			; temp:mult2 += 63
	rlf	mult2,f			; div result by 64
	rlf	temp,f
	rlf	mult2,f
	rlf	temp,w
	return

; enable usart
usart_enable:
	; init usart
	bsf	STATUS,RP0		; goto bank 1
;	movlw	0xff			; 1200 baud
	movlw	0x81			; 2400 baud
	movwf	SPBRG
	movlw	0x20			; 8-bit transimit, transmitter enabled
	movwf	TXSTA			; asynchronous mode, low speed mode
	bcf	STATUS,RP0		; goto bank 0
	movlw	0x90			; 8-bit receive, receiver enabled
	movwf	RCSTA			; serial port enabled
	return

; disable usart
usart_disable
	bsf	STATUS,RP0		; goto bank 1
	clrf	TXSTA			; disable transmitter
	bcf	STATUS,RP0		; goto bank 0
	clrf	RCSTA			; serial port disabled, receiver disabled
;	bsf	PORTC,6			; beep is 30 ff's
	bcf	PORTC,6			; beep is 30 00's
	return

; disable usart with no beep
usart_off
	bsf	STATUS,RP0		; goto bank 1
	clrf	TXSTA			; disable transmitter
	bcf	STATUS,RP0		; goto bank 0
	clrf	RCSTA			; serial port disabled, receiver disabled
;	bcf	PORTC,6			; no beep - beep is 30 ff's
	bsf	PORTC,6			; no beep - beep is 30 00's
	return

; if there is data in RCREG, return in W (returned as 0x30 + data)
getchar:
	clrw
	btfss	PIR1,RCIF		; if not input data, return 0 in W
	return
	movf	RCREG,W			; else read input data
	addlw	0x30
	return

; get transmits status, 0 = not ok to send, 1 = ok to send
xmit_status:
	clrw
	btfsc	PIR1,TXIF		; if xmit not empty, skip
	movlw	1
	return

; send data in (W) to usart
putchar:
	movwf	TXREG			; write data to usart
	return

; called every 5 msec to handle anything related to the remote control
chk_remote:
	btfsc remote_toggle,REMOTE_BIT	; if remote not enabled, exit
	return
	movf	remote_block,W		; if remote blocking,
	btfsc	STATUS,Z		;
	goto	no_blocking
	sublw	0x42			;  if 100msec had elapsed,
	btfsc	STATUS,Z		;
;	bcf	PORTC,6			;   turn off beep - beep is 30 ff's
	bsf	PORTC,6			;   turn off beep - beep is 30 00's
	decfsz	remote_block,F		;  if last block,
	return
	call	usart_enable		;   re-enable usart
	call	getchar			;   flush
	call	getchar			;    input fifo
	return

no_blocking:
	call	getchar
	andlw	0xff			; if no input, exit
	btfsc	STATUS,Z
	return
	sublw	0x38	
	btfsc	STATUS,Z		; got 8 from transmitter - toggle sweep function
	goto	sweep_toggled
	sublw	0x06
	btfsc	STATUS,Z		; got 2 from transmiiter - toggle feed functions
	goto	feed_toggled
	movlw	0xa0
	movwf	remote_block
;no long beep	call	usart_disable
	call	usart_off		; no long beep
	return

set_remote_block
	movlw	BLOCK_CNT
	movwf	remote_block
	call	usart_disable
	return

sweep_toggled:
	movlw	0x1e			; set up to send 30 FF's
	movwf	FF_count
	movlw	SWEEP_TOGGLE
	xorwf	remote_toggle,F
;	movlw	8			; DEBUG
;	movwf	beep_count		; DEBUG
	goto	set_remote_block

feed_toggled:
	movlw	0x1e			; set up to send 30 FF's
	movwf	FF_count
	movlw	FEED_TOGGLE
	xorwf	remote_toggle,F
;	movlw	2			; DEBUG
;	movwf	beep_count		; DEBUG
	goto	set_remote_block



; Current sweep/2-line switch configuration.
; Sweep Enable SPST (On/Off) rocker, Sweep On / Sweep Off, input ot uC.
; Sweep Mode SPDT (On/Off/On) rocker, Narrow 2-line / Continuous Sweep / Wide 2-line, either On position allows respective 2-line
; limit switch to be input to the uC.
; Sweep Enable switch  must be on for sweep motor to turn on in any sweep mode.
; Continuous sweep is when Sweep Mode Switch is off (center position), not allowing any 2-line limit switch detection.
; Narrow or Wide 2-Line mode is when Sweep Mode Switch is on in either direction allowing either narrow or wide limit
; switch detection.
; If sweep is on in continuous mode and then it is switched to any 2-line mode, a ball could be released before the sweep stops at 
; a 2-line limit switch, because we do not know we are in 2-line mode until the first 2-line limit switch is detected.
; When a 2-line limit switch is detected (change of state), the feed motor speed is restricted so ball feed rate is slower than
; side to side sweep.


chk_2linesw:					; called every 5ms
	btfss	sweep_is_on,0		; If sweep motor on flag is on, I.E. sweep switch or remote turned sweep on
	goto	normal_feed_rate	; If sweep motor on flag is off, set feed rate to normal.

	btfss	PORTA,LINESWBIT
	call	restrict_feed_rate

; THE FOLLOWING TWO LINES CORRECTED THE PROBLEM IF IN 2-LINE MODE AND 2-LINE LIMIT SWITCH ON (MACHINE STOPPED
; AT 2-LINE), WHEN 2-LINE ROCKER WAS TURNED OFF, SWEEP MOTOR WOULD NOT START BACK UP UNTIL SWEEP SWITCH WAS TOGGLED

	btfsc	PORTA,LINESWBIT		; IF 2-LINE LIMIT SWITCH IS OFF...
	bsf		PORTC,SWEEP_EN		; TURN ON SWEEP MOTOR

	

	movf	linesw_toggle,W		; GET 2-LINE LIMIT SWITCH TOGGLE FLAG BYTE
	andlw	0x10				; BIT 4 IS THE ACTUAL FLAG, GET RID OF OTHER BITS (THERE SHOULDN'T BE ANY)
	xorwf	PORTA,W				; COMPARE TO PORTA
	andlw	0x10				; GET RID OF EXTRA BITS AGAIN
	btfsc	STATUS,Z			; DID 2-LINE LIMIT SWITCH CHANGE STATE?
	goto	linesw_no_change	;
	goto	linesw_change		;
linesw_no_change:				; IF 2-LINE LIMIT SWITCH DID NOT CHANGE STATE
	btfss	PORTA,LINESWBIT		; If 2-Line limit switch is on, don't do anything, otherwise start sweep timer.
	return
	movlw	0					;
	xorwf	sweep_timer,W		; IF SWEEP TIMER IS 0, 6 SECONDS SINCE LAST LINE SWITCH CLOSURE DETECTED
	btfss	STATUS,Z			; THEN ASSUME NOT 2-LINE MODE AND SET FEED RATE BACK TO NORMAL ALGORITHM
	return						; IF TIMER NOT EXPIRED, HAVEN'T WAITED LONG ENOUGH
	movlw	0x00				; 0=NORMAL FEED RATE ALGORITHM
	movwf	feed_rate_type		; SET FEED RATE TO NORMAL ALGORITHM
	return						;
linesw_change:					; IF 2-LINE SWITCH DID CHANGE STATE
;	movlw	0x01				; 1=RESTRICTED FEED RATE FOR 2-LINE MODE
;	movwf	feed_rate_type		; SET FEED RATE TO RESTRICTED FOR 2-LINE MODE
	movlw	0x06				; 8 SECONDS
	movwf	sweep_timer			; START TIMER AT 6 SECONDS (REALLY x SEC, BECAUSE OF MISCALCULATION WITH 5MS TIMER)
	movlw	0x10				;
	xorwf	linesw_toggle,F		; IF CHANGE (TOGGLE), PUT NEW VALUE IN LINESW_TOGGLE
;	btfss	PORTA,LINESWBIT		; CHECK FOR 2-LINE LIMIT SWITCH ON (LOW)
;	bcf		PORTC,SWEEP_EN		; IF 2-LINE LIMIT SWITCH IS ON, TURN OFF SWEEP MOTOR, THEN LEAVE ROUTINE
;	return						; IF 2-LINE LIMIT SWITCH IS OFF, LEAVE ROUTINE
	btfsc	PORTA,LINESWBIT
	return
	movlw	0x01
	movwf	feed_rate_type
	bcf		PORTC,SWEEP_EN
	return
linesw_toggle_init:				; WHEN SWEEP IS JUST TURNED ON, MUST MATCH LINESW_TOGGLE VALUE TO 2-LINE SWITCH VALUE
	btfsc	PORTA,4				;
	goto	linesw_toggle_set	;
	goto	linesw_toggle_clr	;
linesw_toggle_set:				;
	movlw	0x10				;
	movwf	linesw_toggle		;
	return						;	
linesw_toggle_clr:				;
	clrf	linesw_toggle		;
	return						;

; IF THERE WAS A LINESW_TOGGLE CHANGE AND 2-LINE LIMIT SWITCH IS NOW OFF (SWEEPING), START TIMER AT 0 (CLEAR), IF NOT
; ANOTHER TOGGLE WITHIN APPROX. 6 SECONDS, THEN 2-LINE MODE IS NOW OFF AND USE NORMAL FEED RATE ALGORITHM.
;

chk_intf:						; CALLED EVERY 5MS
	btfss	sweep_is_on,0		; IF SWEEP MOTOR ON FLAG IS ON
	return						;
;	btfsc	INTCON,INTF			; CHECK FOR RB0/INT FLAG SET
;	btfsc	PORTB,0				; Flag was set, now double check ball detect switch input still on.
;	return						;
;	goto	ball_dropped		; Double checked that ball was dropped.
	btfss	INTCON,INTF			; If RBO/INT flag set, double check switch is still on (debounce/noise filter).
	return						; If RBO/INT flag not set, return.
	btfss	PORTB,0				; If ball detect switch is still on, then ball was dropped.
	goto	ball_dropped		;
	bcf		INTCON,INTF			; If ball detect switch is no longer on, it was a spurious detection, clear INTF and return.
	return
ball_dropped:					;
	bsf		PORTC,SWEEP_EN		; ...THEN TURN ON SWEEP MOTOR
	bcf		INTCON,INTF			;
	return						;
normal_feed_rate
	movlw	0x00				; 0=Normal feed rate algorithm
	movwf	feed_rate_type		; If sweep motor on flag is not on, set feed rate to normal.
	return						;
restrict_feed_rate
	movlw	0x01
	movwf	feed_rate_type
	return

; WHEN 2-LINE MODE (LINE LIMIT SWITCH) IS DETECTED, MUST REDUCE MAXIMUM BALL FEED RATE SO THAT TIME BETWEEN
; BALLS FED IS LONGER THAN TIME TO SWEEP SIDE TO SIDE (APPROX. 5 SECONDS), I.E. LIMIT FEED MOTOR SPEED.
; WHEN 2-LINE MODE IS NOT DETECTED FOR LONGER THAN A SIDE TO SIDE SWEEP, MUST GO BACK TO NORMAL FEED RATE ALGORITHM.



toggle_green_led:
	movlw	GREEN_LED_ON	; value to turn on green LED
	xorwf	PORTB,f			; toggle green LED
	return

toggle_red_led:
	movlw	RED_LED_ON		; value to turn on red LED
	xorwf	PORTB,f			; toggle red LED
	return

; The following lines were a part of the fix for the ball jam problem at power-on.
; If the machine was powered on and a ball that was already in the chute got jammed in the server wheels, the machine
; would not shut down with an overcurrent error if the speed pot was set fairly low.
; The top serve motor is the first to turn on and would cause the jam.
; The jammed top server motor would just continue to draw excessive current until it's free-wheeling diode would heat up and fail.
; The changes dated 06/09/05 make the top serve motor spin up quickly at power-on, regardless of the pot setting.
; Then if a ball gets jammed at power-on, enough current is present to trip the overcurrent circuit.
top_not_inited:				; 06/09/05 MJZ added this line
	movlw	0x8f			; 06/09/05 MJZ added this line
	movwf	t_top_pwm		; 06/09/05 MJZ added this line
	return					; 06/09/05 MJZ added this line

; The following lines and all lines dated 06/15/05 are changes that, at power-on, ramp up the bottom serve motor to fairly 
; fast speed, regardless of the pot setting, before it is allowed to be set to the pot. When the changes, 06/09/05, were added
; to ramp up the top serve motor at power-on, the top serve motor, if the speed pot was set low, would slow down before the 
; bottom serve motor would start spinning. The bottom serve motor would pull a ball, that was already in the chute, into the
; wheels and the serve motors would jam on the ball without overcurrent detection, because both motors are drawing too little
; current. Now the bottom serve motor will draw enough current at the ball jam time to trip the overcurrent detection circuit.
bot_not_inited:				; 06/15/05 MJZ added this line
	movlw	0x8f			; 06/15/05 MJZ added this line
	movwf	t_bot_pwm		; 06/15/05 MJZ added this line
	return					; 06/15/05 MJZ added this line


	END                       ; directive 'end of program'


