;******************************************************************************
;   This file is a basic template for assembly code for a PIC18F4550. Copy    *
;   this file into your project directory and modify or add to it as needed.  *
;                                                                             *
;   The PIC18FXXXX architecture allows two interrupt configurations. This     *
;   template code is written for priority interrupt levels and the IPEN bit   *
;   in the RCON register must be set to enable priority levels. If IPEN is    *
;   left in its default zero state, only the interrupt vector at 0x008 will   *
;   be used and the WREG_TEMP, BSR_TEMP and STATUS_TEMP variables will not    *
;   be needed.                                                                *
;                                                                             *
;   Refer to the MPASM User's Guide for additional information on the         *
;   features of the assembler.                                                *
;                                                                             *
;   Refer to the PIC18FXX50/XX55 Data Sheet for additional                    *
;   information on the architecture and instruction set.                      *
;                                                                             *
;******************************************************************************
;                                                                             *
;    Filename:    PRACTICA 4                                          *
;    Date:        04-10-16                                                   *
;    File Version: 1.0                                                        *
;                                                                             *
;    Author:   Jose Luis Bravo Leon                             *
;    Company:   Academia de Computacion                            *
;                                                                             * 
;******************************************************************************
;                                                                             *
;    Files required: P18F4550.INC                                             *
;                                                                             *
;******************************************************************************

	LIST P=18F4550, F=INHX32	;directive to define processor
	#include <P18F4550.INC>		;processor specific variable definitions

;******************************************************************************
;Configuration bits

	CONFIG PLLDIV   = 5         ;(20 MHz crystal on PICDEM FS USB board)
    CONFIG CPUDIV   = OSC1_PLL2	
    CONFIG USBDIV   = 2         ;Clock source from 96MHz PLL/2
    CONFIG FOSC     = HSPLL_HS
    CONFIG FCMEN    = OFF
    CONFIG IESO     = OFF
    CONFIG PWRT     = OFF
    CONFIG BOR      = ON
    CONFIG BORV     = 3
    CONFIG VREGEN   = ON		;USB Voltage Regulator
    config WDT      = OFF
    config WDTPS    = 32768
    config MCLRE    = ON
    config LPT1OSC  = OFF
    config PBADEN   = OFF		;NOTE: modifying this value here won't have an effect
        							  ;on the application.  See the top of the main() function.
        							  ;By default the RB4 I/O pin is used to detect if the
        							  ;firmware should enter the bootloader or the main application
        							  ;firmware after a reset.  In order to do this, it needs to
        							  ;configure RB4 as a digital input, thereby changing it from
        							  ;the reset value according to this configuration bit.
    config CCP2MX   = ON
    config STVREN   = ON
    config LVP      = OFF
    config ICPRT    = OFF       ; Dedicated In-Circuit Debug/Programming
    config XINST    = OFF       ; Extended Instruction Set
    config CP0      = OFF
    config CP1      = OFF
    config CP2      = OFF
    config CP3      = OFF
    config CPB      = OFF
    config CPD      = OFF
    config WRT0     = OFF
    config WRT1     = OFF
    config WRT2     = OFF
    config WRT3     = OFF
    config WRTB     = OFF       ; Boot Block Write Protection
    config WRTC     = OFF
    config WRTD     = OFF
    config EBTR0    = OFF
    config EBTR1    = OFF
    config EBTR2    = OFF
    config EBTR3    = OFF
    config EBTRB    = OFF
;******************************************************************************
; Definicion de variables

R0 EQU 0x10


;******************************************************************************
;Reset
; Este codigo se ejecutara cuando ocurra un RESET.

RESET_VECTOR	ORG		0

		goto	INICIO		; IR AL INICIO DEL PROGRAMA PRINCIPAL

;******************************************************************************

;******************************************************************************
; Inicio del programa principal a partir de la direccion 0x1000
; 
	ORG		0x1000
INICIO:  					; *** El programa principal se captura aqui **
		call 	C_puertos	; programa puertos
		call 	C_tabla		; creacion de tabla
		movlw	0x0a
		movwf	R0
		call 	c_timer
leer:
		movf 	PORTA,0
		andlw	0x0f
		cpfsgt	R0
		goto 	borra
		movwf	FSR0L
		movf	INDF0,0
		movwf	PORTB
		goto	leer
borra:
		clrf	PORTB
		goto	leer
	
						; *** Fin del programa principal **	


;******************************************************************************
; Espacio para subrutinas
C_puertos:
		movlw	0x0f
		movwf	ADCON1
		movwf	TRISA
		clrf	TRISB
		return

C_tabla:
		lfsr 0,0
		movlw	0x01
		movwf	POSTINC0,0
		movlw	0x02
		movwf	POSTINC0,0 
		movlw	0x03
		movwf	POSTINC0,0
		movlw	0x04
		movwf	POSTINC0,0
		movlw	0x05
		movwf	POSTINC0,0
		movlw	0x06
		movwf	POSTINC0,0
		movlw	0x07
		movwf	POSTINC0,0
		movlw	0x08
		movwf	POSTINC0,0
		movlw	0x09
		movwf	POSTINC0,0
		movlw	0x10
		movwf	POSTINC0,0
		RETURN

		
c_timer
		movlw 0x10
		movwf	TMR0H
		movlw	0x20
		movwf	TMR0L
		RETURN

;******************************************************************************

					



;******************************************************************************
;Fin del programa
	END
