CBI DDRD, 2    ; Set PD2 as Input
SBI DDRB, 2    ; Set PB2 as Output/LED1
SBI DDRB, 3    ; Set PB3 as Output/LED2
SBI DDRB, 4    ; Set PB4 as Output/LED3

CBI PORTB, 2;
CBI PORTB, 3;
CBI PORTB, 4;
RJMP Detect;

Detect: SBIC PIND, 2;
            JMP Detect;If the button is not pressed, this sentence will be executed, and then the button will be detected continuously
            CALL AntiV;Call anti-vibration delay if key press is detected
            SBIC PIND,2;Check again, if the button is high and not pressed, it will not light up
     JMP LIGHT;
     JMP Detect;

LIGHT:      SBI PORTB, 2    ; Turn ON LED -> PB2
                CALL Delay;
  CBI PORTB, 2; TURN OFF LED PB2
                SBI PORTB, 3    ; Turn ON LED -> PB3
  CALL Delay;
  CBI PORTB, 3; TURN OFF LED PB3
                SBI PORTB, 4    ; Turn ON LED -> PB4
  CALL Delay;
  CBI PORTB, 4; TURN OFF LED PB4
                JMP Detect; TURN BACK TO DETECT THE SWITCH


;initial delay number for the register, L1=25,L2=26
Delay:
Delay1: LDI  R18, 241
Delay2: LDI  R19, 245
Delay3: LDI  R20, 158
Delay4: LDI  R21, 2
Loop: DEC  R21
      BRNE Loop;r21-1!=0 jump to loop,if r21-1=0 proceed down
      DEC  R20
      BRNE Delay4
      DEC  R19
      BRNE Delay3
	  DEC  R18
      BRNE Delay2
      RET


AntiV: ;anti-vibration delay
Anti1: LDI  R21, 1
Anti2: LDI  R22, 10
Anti3: LDI  R23, 10
Anti: DEC  R23
      BRNE Anti
      DEC  R22
      BRNE Anti3
      DEC  R21
      BRNE Anti2
      RET