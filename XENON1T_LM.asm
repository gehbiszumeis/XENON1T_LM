;************************************************************************
;									*
;    Files Required: P16F877A.INC					*
;									*
;   Assembler Code for the readout of the XENON1T capacitive level-	*
;    meter system.							*
;   Basic principle:							*	
;   - Readout of 6+1 UTI chips in a row using a multiplexer		*
;   - Using capture module to measure UTI phases (propto capacitance)	*
;   - Using USART module to communicate with Slow Control via RS485	*    
;************************************************************************
;									*
;   Author: CHRISTOPHER WERNER GEIS (ChG)				*
;   University: Institut für Physik,					*
;		Johannes Gutenberg-Universität Mainz			*
;   Email:  geisch@uni-mainz.de, christopher.geis@gmx.de		*
;   License: GNU General Public License V3.0				*
;                                                                       *
;************************************************************************
;   History:								*
;   - V0.0:	2015/07/01  Kickoff					*
;   - V0.1:	2015/07/15  Multiplexing implemented			*
;   - V0.2:	2015/09/01  USART module setup (buggy) communication    *
;			    over RS485 bus possible			*
;   - V0.3:	2015/11/15  CCP1 module setup (buggy) measurement of UTI*
;			    phases possible				*
;   - V0.4:	2015/12/01  USART communication works bugfree		*
;   - V0.5:	2015/12/15  CCP1 capture works bugfree			*
;   - V1.0:	2015/12/16  First released code with full functionality	*
;   - V1.1:	2016/01/23  Added also 0D CR character response after   *
;			    Enable/Disable Power down mode		*
;									*
;   - V2.0:	2016/04/23  Adjustments according to usage of an updated*
;			    readout board				*
;   - V2.1:	2016/06/01  Allows nor to select (in code) for UTI fast *
;			    mode					*
;************************************************************************
    
 errorlevel -302		; Switches off BANKSEL warning of MPLAB IDE

 list		p=16f877A	; list directive to define processor
 #include	<p16f877a.inc>	; processor specific variable definitions
	
 __CONFIG _CP_OFF & _WDT_OFF & _BODEN_OFF & _PWRTE_OFF & _HS_OSC & _WRT_OFF & _LVP_OFF & _CPD_OFF ;& _DEBUG_ON
;__CONFIG _HS_OSC 

; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.


		
;***** BIT DEFINITIONS 
GotNewData			EQU	0		; bit indicates that new data was received (not used)
BunchAcknowledgement		EQU	1		; bit indicates that 1 bunch (4 Phase Period values of one UTI chip) got measured			
RemoveFirstValue		EQU	2		; bit indicates that 1 phase value was measured (has to be removed)

;***** VARIABLE DEFINITIONS
w_temp				EQU	0x7D		; variable used for context saving 
status_temp			EQU	0x7E		; variable used for context saving
pclath_temp			EQU	0x7F		; variable used for context saving			

loops1				EQU	0x24		; loop variable for WAIT routine
loops2				EQU	0x25		; loop variable for WAIT routine
loops3				EQU	0x26		; loop variable for WAIT routine
ByteCounter			EQU	0x27		; Wild Card (Debugging Purpose)
FlagByte			EQU	0x28		; Flagbyte, see BIT DEFINITIONS 
ReceivedByte			EQU	0x29		; Storage for received bytes by RS485		
NumberCapturedRisingEdges	EQU	0x2A		; Counter of number of captured edges by the CCP1 compare module
NumberUTIChips			EQU	0x2B		; Number of UTI chips (6, 4 SLM, 2 LLM)
TransmissionCounter		EQU	0x2C		; Number of transmitted bytes (Debugging Purpose)
	
;***** CONTSTANTS DEFINITIONS 

	
	
; CAPT_NEW_H:CAPT_NEW_L stores the NEW captured value and the result of the 
; subtraction between this capture and the previous.
;    CAPT_NEW_H:CAPT_NEW_L  =  CAPT_NEW_H:CAPT_NEW_L - CAPT_OLD_H:CAPT_OLD_L
;
; After all computations the new capture value is moved to the CAPT_OLD_H:CAPT_OLD_L
; in preperation for the next capture value.
;
CAPT_NEW_H			EQU     0x06A           ; 
CAPT_NEW_L			EQU     0x06B           ;
CAPT_OLD_H			EQU     0x06C           ;
CAPT_OLD_L			EQU     0x06D           ;
	
;**********************************************************************

; ---------------------------------------------------------------------
; This code executes when a reset occurs
	
		ORG     0x000             ; processor reset vector
		nop			  ; nop required for icd
ResetCode	clrf	PCLATH		  ; Select program memory page zero  
		goto	SetupConfig
	
;----------------------------------------------------------------------------
;This code executes when an interrupt occurs.

		ORG	0x0004		;place code at interrupt vector

InterruptCode				;do interrupts here
		movwf	w_temp		;Copy W to TEMP register
		swapf	STATUS,W	;Swap status to be saved into W
		clrf	STATUS		;bank 0, regardless of current bank, Clears IRP,RP1,RP0
		movwf	status_temp	;Save status to bank zero STATUS_TEMP register
		movf	PCLATH, W	;Only required if using pages 1, 2 and/or 3
		movwf	pclath_temp	;Save PCLATH into W
		clrf	PCLATH		;Page zero, regardless of current page
		
		bcf	INTCON,PEIE	;Disable peripheral interrupts at beginning of ISR
		bcf	INTCON,GIE	;Disable global interrupts at beginning of ISR (redundant)
		clrf	STATUS		;Select Bank 0

		btfsc	PIR1,TMR1IF	; TMR1IF overflow interrupt occured?
		bcf	PIR1,TMR1IF	; Yes, clear IF flag
		btfsc   PIR1, CCP1IF	; CCP1 Interrupt occurred? (Capture)
		bsf	STATUS,RP0	; change to bank 1 if CCP1IF is set
		btfsc	PIE1,CCP1IE	;test CCP1IE only if CCP1IF is set
		goto    CAPTURE		; ;if CCP1IF and CCP1IF set, do capture, Service the CCP1 Interrupt
		btfsc	PIR1,RCIF	;test RCIF receive interrupt
		bsf	STATUS,RP0	;change to bank1 if RCIF is set
		btfsc	PIE1,RCIE	;test RCIE only if RCIF is set
		call	ReceiveSerial	;if RCIF and RCIE set, do receive
		clrf	STATUS		;Select Bank 0
		;btfsc	PIR1,TXIF	;test for TXIF transmit interrupt
;		bsf	STATUS,RP0	;change to bank1 if TXIF is set
;		btfsc	PIE1,TXIE	;test TXIE only if TXIF is set
		;call	TransmitSerial	;if TXIF and TXIE set, do transmit

END_ISR		
		bsf	INTCON,PEIE	;Enable peripheral interrupts at end of ISR
		bsf	INTCON,GIE	;Enable global interrupts at end of ISR (redundant)

		movf	pclath_temp,W	;Restore PCLATH
		movwf	PCLATH		;Move W into PCLATH
		swapf	status_temp,W	;Swap STATUS_TEMP register into W
					;(sets bank to original state)
		movwf	STATUS		;Move W into STATUS register
		swapf	w_temp,F	;Swap W_TEMP
		swapf	w_temp,W	;Swap W_TEMP into W
		
		retfie			;return from interrupt

;----------------------------------------------------------------------------
; Setting Port Configuration Bits	
	
SetupConfig:
		
	; Port A
	BANKSEL PORTA
	clrf    PORTA		    ; PORTA is just 6 bit wide
;	movlw	B'000100'
;	movlw	B'000001'	    ; Start with 2nd UTI port since Port1 is broken
	movlw	B'000000'	    ; Start with 1st UTI port with Board SN002
	movwf	PORTA

	BANKSEL ADCON1
	movlw   B'000110'	    ; Configure all pins as digital I/O
	movwf   ADCON1
	BANKSEL	CMCON		    
	movlw   D'7'		    ; Switch off comparator module 
	movwf	CMCON
	BANKSEL TRISA
	movlw   B'000000'	    ; All inputs except RA0, RA1, RA2
	movwf   TRISA

	;Port B
	BANKSEL	PORTB
	clrf	PORTB
	BANKSEL	TRISB
	movlw	B'00000000'	    ; Configure all pins as outputs
	movwf	TRISB
	
	;Port C
	BANKSEL PORTC
	clrf	PORTC
	;bsf	PORTC,4		    ; uncomment if UTI in fast mode
	BANKSEL TRISC
	movlw   B'11100101'	    ; All inputs except RC1, RC3 and RC4; C6:7 hast to be set for USART, C2 has to be set for CCP
	movwf   TRISC
	
	;Port E
	BANKSEL PORTE
	clrf	PORTE
	BANKSEL	TRISE		    ; Configure all pins as outputs
	clrf	TRISE
	;bcf	TRISE,PSPMODE	    ; Configure PORTE pins as general purpose I/O ports
	;bcf	TRISE,TRISE0	    ; Configure RE0 as output
	;bcf	TRISE,TRISE1	    ; Configure RE1 as output
	
	;Special Registers

;	BANKSEL	OPTION_REG
;	clrf	OPTION_REG

	BANKSEL SPBRG
	movlw	D'77'		    ; Set Baud rate of 9600Baud (ASYNC): Formula Fosc/(16*(77+1)) = 9615 Baud (High Baud Rate selected): 1% error: 960 characters
;	movlw	D'38'		    ; Set Baud rate of 19200Baud (ASYNC): Formula Fosc/(16*(38+1)) = 9615 Baud (High Baud Rate selected): 1% error: 960 characters
	;movlw   D'19'		    ; Set Baud rate of 9600Baud (ASYNC): Formula Fosc/(64*(19+1)) = 9375 Baud (not High Baud Rate selected)
	;movlw   D'155'		    ; Set Baud rate of 19200Baud (SYNC): Formula Fosc/(4*(155+1)) = 19231 Baud
	movwf	SPBRG		    

	BANKSEL TXSTA
	clrf	TXSTA
	bsf	TXSTA,BRGH	    ; Enable High Speed Baud Rate (just asynchronous)
	bcf	TXSTA,SYNC	    ; Select synchronous/asynchronous transmission
	bcf	TXSTA,TX9	    ; Select 8bit transmission
	bsf	TXSTA,TXEN	    ; Disables transmission
	bcf     TXSTA,CSRC	    ; Enables/Disables Synchronous Master/Slave mode (just synchronous)
;	movlw	B'00000100'
;	movwf	TXSTA
	
	BANKSEL RCSTA	
	clrf	RCSTA
	bsf	RCSTA,SPEN	    ; Enables USART function
	bcf	RCSTA,SREN	    ; Enables/Disables single receive
	bsf     RCSTA,CREN          ; Enables continuous receive (overrides SREN)
;	movlw	B'10010000'
;	movwf	RCSTA
	
	BANKSEL	TMR1H
	clrf	TMR1		    ; clearing 16bit timer register
	clrf	TMR1H
	
	BANKSEL	CCPR1H
	clrf	CCPR1		    ; clearing 16bit capture register
	clrf	CCPR1H

	BANKSEL	T1CON
	bcf	T1CON,TMR1CS	    ; Timer1 in Timer mode Fosc/4
	bsf	T1CON,T1CKPS0	    ;
	bsf	T1CON,T1CKPS1	    ; Prescaler 1:1
	bsf	T1CON,TMR1ON	    ; Enables Timer1

	BANKSEL CCP1CON
	movlw	B'00000101'	    ; Enables Capture Mode, every rising edge
;	movlw	B'00000100'	    ; Enables Capture Mode, every falling edge
	movwf	CCP1CON

	; Clear variables
	clrf	w_temp
	clrf	status_temp
	clrf	pclath_temp

	clrf	loops1
	clrf	loops2
	clrf	loops3
	clrf	ByteCounter
	clrf	FlagByte
	clrf	ReceivedByte
	clrf	NumberCapturedRisingEdges
	clrf	NumberUTIChips
	clrf    TransmissionCounter

	clrf	CAPT_NEW_H
	clrf	CAPT_NEW_L
	clrf	CAPT_OLD_H
	clrf	CAPT_OLD_L
	
	; This code block can be used to use indirect addressing of the Multiplexer and not decf
	; -> It allows to measure in sequences which are different to changes of +/-1
;	movlw	B'000100'
;	movwf	0x30
;	movlw	B'000101'
;	movwf	0x31
;	movlw	B'000001'
;	movwf	0x32
;	movlw	B'000010'
;	movwf	0x33
;	movlw	B'000011'
;	movwf	0x34
;	movlw	B'000110'
;	movwf	0x35
	
	
	clrf	FSR		    
	movlw	0x30		    ; Load indirect adressing pointer to Register 0x30
	movwf	FSR
	
;	movf	INDF,W		    ; Needed in indirect addressing of Multiplexer is used
;	movwf	PORTA
	
	BANKSEL	PIE1
	bcf	PIE1,TXIE	    ; enables/disables USART transmit interupt
	bsf	PIE1,RCIE	    ; enables/disables USART receive interupt
	bcf	PIE1,CCP1IE	    ; disables Capture interupts
	bcf	PIE1,TMR1IE	    ; disables Timer 1 interrupts
;	movlw	B'00110000' 
;	movwf	PIE1

	;Port D
	BANKSEL	PORTD
	call	DisablePowerDown    ; Switch on all UTI chips 
	BANKSEL	TRISD
	clrf	TRISD		    ; PORTD all outputs

	BANKSEL INTCON
	bsf	INTCON,PEIE	    ; Peripheral Interrupt Enable bit (necessary for interrupt of RS485 TX)	
	bsf	INTCON,GIE	    ; global interrupt enable bit (necessary for interrupt of RS485 TX)
;	movlw	B'11000000'
;	movwf	INTCON
	
;--------------------------------------------------------------------------
; Main routine
	
MAIN

    btfsc   FlagByte,GotNewData	    ; Check if data got received and is ready to transmit
    call    TransmitSerial	    ; if so then go transmit the data

    goto    MAIN

;----------------------------------------------------------------------------
;Check if data received and if so, parse which command was send and do the corresponding
; actions    
    
ReceiveSerial
    BANKSEL PORTE
    bcf	    PORTE,RE0			    ;RE0 -> notRE = 0 -> Receiver enabled
    bcf	    PORTE,RE1			    ;RE1 -> DE = 0 -> Driver enabled (reception)
    
;    BANKSEL	PIR1
;    btfss	PIR1,RCIF	    ;check if data
;    return			    ;return if no data
    btfsc	RCSTA,OERR	    ;if overrun error occurred
    goto	ErrSerialOverr	    ; then go handle error
    btfsc	RCSTA,FERR	    ;if framing error occurred
    goto	ErrSerialFrame	    ; then go handle error

    movf	RCREG,W		    ;get received data
    movwf	ReceivedByte	    ; Save received byte finally to memory
    movlw	0x58
    xorwf	ReceivedByte,0	    ; 1. Command: 'X' Asking for Levelmeter Data
    btfss	STATUS,Z	    ; Is Command  == 'X'
    goto	NOT_X		    ; No, ask for 2. or 3. Command
;    bsf		PORTC,RC1   ;indicate new data received
    clrf	ByteCounter	    ;clear number of bytes having recognized to be received at every new data request
    clrf	TransmissionCounter ;clear number of transmitted bytes at every new data request
    movlw       0x30		    
    movwf       FSR		    ;set indirect addressing pointer again back to 0x30 at every new data request
    movlw	D'6'
    movwf	NumberUTIChips	    ;reset number of UTI chips to D'6' at every new data request
    movlw	D'5'
    movwf	NumberCapturedRisingEdges  ;reset number of rising edges to be captures to D'5' at every new data request
    bsf		STATUS,RP0	    ; Change to bank 1 for PIE1
    bsf		PIE1,CCP1IE	    ; Enable interrupts for CCP1 capture module -> Enable measurement
NOT_X
    movlw	0x30
    xorwf	ReceivedByte,0	    ; 2. Command: '0' Put UTI chips in power down mode
    btfss	STATUS,Z	    ; Is Command == '0'
    goto	$+2		    ; No, ask for 3. Command
    call	EnablePowerDown	    ; Yes, set UTI chips in PD mode
    movlw	0x31
    xorwf	ReceivedByte,0	    ; 3. Command: '1' Wake UTI chips up from power down mode
    btfss	STATUS,Z	    ; Is Command = '1'
    return			    ; No, do nothing
    call	DisablePowerDown    ; Yes, wake UTI chips up from PD mode
    
    return

;error because OERR overrun error bit is set
;can do special error handling here - this code simply clears and continues

ErrSerialOverr:
    bcf		RCSTA,CREN	    ;reset the receiver logic	
    bsf		RCSTA,CREN	    ;enable reception again
    return

;error because FERR framing error bit is set
;can do special error handling here - this code simply clears and continues

ErrSerialFrame:
    movf	RCREG,W		    ;discard received data that has error
    return

;----------------------------------------------------------------------------
;Transmit data in WREG when the transmit register is empty.

TransmitSerial
    
    BANKSEL PORTE
    incf    TransmissionCounter,F	    ; increment Number of transmitted bytes 
    incf    TransmissionCounter,F	    ; two times since per phase measurement to bytes have to be measured
    
    btfsc   FlagByte,RemoveFirstValue	    ; Check if it was first phase of the measurement
    goto    NO_TRANSMISSION		    ; Don't send those values
  
    bsf	    PORTE,RE0			    ;RE0 -> notRE = 1 -> Receiver disabled
    bsf	    PORTE,RE1			    ;RE1 -> DE = 1 -> Driver enabled (transmission)

;    call    GetLowValue2Transmit	    ;Returns lower byte of the data to transmit
    movf     CAPT_NEW_L,W		    ; Load captured value (in case you won't use the function 'GetLowValue2Transmit'
    movwf   INDF			    ; Save value in memory
    incf    FSR,F			    ; increment indirect addresing pointer for next register
    
;    BANKSEL TXSTA			
;    bsf	    TXSTA,TXEN		    ; Enabling TX only before transmission (doesn't work)

    BANKSEL	PIR1			    ;select bank 0
    btfss	PIR1,TXIF		    ;check if transmitter busy
    goto	$-1			    ;wait until transmitter is not busy
    movwf	TXREG			    ;and transmit the data
    
;    call    GetHighValue2Transmit	    ; Returns higher byte of the data to transmit
    movf     CAPT_NEW_H,W		    ; Load captured value (in case you won't use the function 'GetHighValue2Transmit'
    movwf	INDF			    ; Save value in memory
    incf	FSR,F			    ; increment indirect addresing pointer for next register
;    
    BANKSEL	PIR1			    ;select bank 0
    btfss	PIR1,TXIF		    ;check if transmitter busy
    goto	$-1			    ;wait until transmitter is not busy
    movwf	TXREG			    ;and transmit the data

    BANKSEL	TXSTA		
    btfss	TXSTA,TRMT		    ;The Transmit Shift Register Status bit (TRMT) indicates
    goto	$-1			    ;if the transmission was complete, HAS to be tested, doesn't work without

;    
;    BANKSEL TXSTA
;    bcf	    TXSTA,TXEN		    ; Disabling TX after transmission (doesn't work)
    
    BANKSEL PORTC
    bcf	    PORTC,RC1			    ;switch off indicator LED
    bcf	    FlagByte,GotNewData		    ;indicate no data received
    
    BANKSEL PORTE
    bcf	    PORTE,RE0			    ;RE0 -> notRE = 0 -> Receiver enabled
    bcf	    PORTE,RE1			    ;RE1 -> DE = 0 -> Driver enabled (reception)

    btfss   FlagByte,BunchAcknowledgement   ; Check if BunchAcknowledgment has to be sent
    goto    NO_BUNCH_ACK		    ; No, Bunch not yet finished
    call    TransmitAcknowledgement	    ; Send bunch acknowledgement
    bcf	    FlagByte,BunchAcknowledgement   ; clear flag for next bunch acknowledgement

NO_BUNCH_ACK
    movlw   D'0'			    ; For xorlw	
    xorwf   NumberUTIChips,0		    ; Check if NumberUTIchips decremented to zero already
    btfsc   STATUS,Z			    ; Yes or no?
    call    TransmitCycleAcknowledgement    ; If yes, send acknowledgement that cycle was done

NO_TRANSMISSION
    bcf	    PORTC,RC1			    ;switch off indicator LED
    bcf	    FlagByte,GotNewData		    ;indicate no data received
    bcf	    FlagByte,RemoveFirstValue	    ;indicate that the first value was skipped
   
;    goto    MAIN
    return
    
GetLowValue2Transmit		    ; Returns lower byte of the data to transmit
    movf     CAPT_NEW_L,W 
;    movlw    0x3C
;    movlw   D'4'
    return
    
GetHighValue2Transmit		    ; Returns higher byte of the data to transmit
    movf     CAPT_NEW_H,W 
;    movlw    0x33
;    movlw   D'4'
    return

TransmitCycleAcknowledgement
    BANKSEL PORTE
    bsf	    PORTE,RE0			    ;RE0 -> notRE = 1 -> Receiver disabled
    bsf	    PORTE,RE1			    ;RE1 -> DE = 1 -> Driver enabled (transmission)

    movlw	0x0D			    ; Carriage Return Character: CR = 0x0D
    
    BANKSEL	PIR1			    ;select bank 0
    btfss	PIR1,TXIF		    ;check if transmitter busy
    goto	$-1			    ;wait until transmitter is not busy
    movwf	TXREG			    ;and transmit the data
    
    BANKSEL	TXSTA
    btfss	TXSTA,TRMT		    ;The Transmit Shift Register Status bit (TRMT) indicates
    goto	$-1			    ;if the transmission was complete, HAS to be tested, doesn't work without

    BANKSEL PORTE
    bcf	    PORTE,RE0			    ;RE0 -> notRE = 0 -> Receiver enabled
    bcf	    PORTE,RE1			    ;RE1 -> DE = 0 -> Driver enabled (reception)
    
    return
    
TransmitAcknowledgement
    BANKSEL PORTE
    bsf	    PORTE,RE0			    ;RE0 -> notRE = 1 -> Receiver disabled
    bsf	    PORTE,RE1			    ;RE1 -> DE = 1 -> Driver enabled (transmission)

    movlw	0x06			    ; Acknowledgement Character: ACK = 0x06
    
    BANKSEL	PIR1			    ;select bank 0
    btfss	PIR1,TXIF		    ;check if transmitter busy
    goto	$-1			    ;wait until transmitter is not busy
    movwf	TXREG			    ;and transmit the data
    
    BANKSEL	TXSTA
    btfss	TXSTA,TRMT		    ;The Transmit Shift Register Status bit (TRMT) indicates
    goto	$-1			    ;if the transmission was complete, HAS to be tested, doesn't work without

    BANKSEL PORTE
    bcf	    PORTE,RE0			    ;RE0 -> notRE = 0 -> Receiver enabled
    bcf	    PORTE,RE1			    ;RE1 -> DE = 0 -> Driver enabled (reception)
    
    return

LED_TimerBlink
    BANKSEL PORTC			    ; Just for debugging
    movlw   B'00001000'			    ; Toggles LED if called
    call    WAIT
    xorwf   PORTC,1
    return

CAPTURE
    BANKSEL CCPR1L
    movf    CCPR1L, W        ; New capture value (low byte)
    movwf   CAPT_NEW_L       ;
    movf    CCPR1H, W        ; New capture value (high byte)
    movwf   CAPT_NEW_H       ;
    movf    CAPT_OLD_L, W    ;
    subwf   CAPT_NEW_L, F    ; Subtract the low bytes of the 2 captures
    btfss   STATUS, C        ; Did a borrow occur?
    decf    CAPT_NEW_H, F    ; YES, Decrement old capture (high byte)
    movf    CAPT_OLD_H, W    ; New capture value (low byte)
    subwf   CAPT_NEW_H, F    ; Subtract the low bytes of the 2 captures
LOAD_OLD    
    movf    CCPR1L, W  ; New capture value (low byte)
    movwf   CAPT_OLD_L       ;
    movf    CCPR1H, W        ; New capture value (high byte)
    movwf   CAPT_OLD_H       ;
END_CAPTURE

    bsf	    PORTC,RC1			; switch on indicator LED 
    bsf	    FlagByte,GotNewData		; transmission flag bit
    
    decfsz  NumberCapturedRisingEdges	; Counts number of captured edges down from 5
    goto    REMOVE_FIRST_VALUE		; If it is the first value, this one has to be removed (no full phase)
    ;bcf	    PORTC,RC1
    decfsz  NumberUTIChips		; Counts number of UTI chips down to zero. Already all chips processed?
    goto    NEXT_UTI			; No, change Multiplexer controll bits for the next chip
    goto    CYCLE_DONE			; Yes, One measurement cycle is done
CYCLE_DONE
;    clrf    PORTA			; Reset multiplexer control bits
;    movlw	B'000001'		; to desired value
    movlw	B'000000'		; to desired value
;    movlw	B'000100'
    movwf	PORTA
;    movlw   0x30			; in case indirect addressing of multiplexer is used
;    movwf   FSR			; reset multiplexer control bits
;    movf    INDF,W
;    movwf   PORTA
    movlw   D'5'			; Set number of edges to be captured to 5 (4 UTI phases + 1 incomplete phase at the beginning)
    movwf   NumberCapturedRisingEdges
    bsf	    FlagByte,BunchAcknowledgement   ; Enable Bunch Acknowledgement also at the end of every cycle
    bsf	    STATUS,RP0			; If all four phase periods of the UTI have been transmitted
    bcf	    PIE1,CCP1IE			; disable CCP1 capture interrupt 
    BANKSEL PIR1
    bcf     PIR1, CCP1IF		; Clear CCP1 Interrupt Flag (has to be here!!!!)

    goto    END_ISR
NEXT_UTI
    incf    PORTA,F			; Go to the next higher input of the multiplexer
;    decf    PORTA,F			; Go to the next lower input of the multiplexer
    incf    ByteCounter,F		; increment number of captured values which are not skipped
;    incf    FSR,F			; in case indirect addressing of multiplexer is used
;    movf    INDF,W			; set multiplexer to next value
;    movwf   PORTA
    bsf	    FlagByte,BunchAcknowledgement; Enable Bunch Acknowledgement
    movlw   D'5'
    movwf   NumberCapturedRisingEdges	; ; Set number of edges to be captured to 5 (4 UTI phases + 1 incomplete phase at the beginning)
    
REMOVE_FIRST_VALUE
    BANKSEL PORTC
    movlw   D'4'			; First capture value is always rubbish (part of a period)
    xorwf   NumberCapturedRisingEdges,0	; and gives wrong phase time
    btfsc   STATUS,Z			; If it is the first captured edge
;    bcf	    PORTC,RC1		; no tranmsmission will be enabled
    bsf	    FlagByte,RemoveFirstValue	; Since it is the first value of each measurement it can be skipped (incomplete phase)
;    bcf	    PORTC,RC1	    
    BANKSEL PIR1
    bcf     PIR1, CCP1IF		; Clear CCP1 Interrupt Flag (Disable capture interrupt)

    goto    END_ISR

EnablePowerDown
    BANKSEL PORTC
    bsf	    PORTC,RC3	    ; switch on levelmeter PD mode LED (LED2)
    bcf	    PORTD,RD0	    ; switch off levelmeter usual mode LED (LED3)
    clrf    PORTD	    ; Set all UTI Power Down pins to 0 -> Power Down mode activated
    call    WAIT
    call    TransmitAcknowledgement ; Send acknowledgement that command was computed
    call    TransmitCycleAcknowledgement ; SC needs 0D character to find end of transmission
    return
    
DisablePowerDown
    BANKSEL PORTC
    bcf	    PORTC,RC3	    ; switch off levelmeter PD mode LED (LED2)
    bsf	    PORTD,RD0	    ; switch on levelmeter usual mode LED (LED3)
    movlw   0xFF	    ; set all UTI Power Down pins to 1 -> Power Down mode deactivated
    movwf   PORTD
    call    WAIT
    call    TransmitAcknowledgement ; Send acknowledgement that command was computed
    call    TransmitCycleAcknowledgement ; SC needs 0D character to find end of transmission
    return
    
;; Delay = 1 seconds
;; Clock frequency = 12 MHz
;
;; Actual delay = 1 seconds = 3000000 cycles
;; Error = 0 %
;
;WAIT
;			;2999995 cycles
;	movlw	0x1A
;	movwf	loops1
;	movlw	0x8B
;	movwf	loops2
;	movlw	0x07
;	movwf	loops3
;WAIT_0
;	decfsz	loops1, f
;	goto	$+2
;	decfsz	loops2, f
;	goto	$+2
;	decfsz	loops3, f
;	goto	WAIT_0
;
;			;1 cycle
;	nop
;
;			;4 cycles (including call)
;	return
    
    ; Delay = 0.001 seconds
; Clock frequency = 12 MHz

; Actual delay = 0.001 seconds = 3000 cycles
; Error = 0 %

WAIT	;2993 cycles
	movlw	0x56
	movwf	loops1
	movlw	0x03
	movwf	loops2
WAIT_0
	decfsz	loops1, f
	goto	$+2
	decfsz	loops2, f
	goto	WAIT_0
	;3 cycles
	goto	$+1
	nop
	;4 cycles (including call)
	return
    
; ****** LED Blink ********

;		BANKSEL PORTC
;		bsf	PORTC,3			; LED blink 1 sek an, 1 sek aus
;		;bcf	PORTC,1
;		movlw	D'1'			; 8 * 125 ms= 1s
;		call	WAIT
;		bcf	PORTC,3
;		;bsf     PORTC,1
;		movlw   D'1'
;		call	WAIT	
;======================================================
    END                       ; directive 'end of program'
