// <editor-fold defaultstate="collapsed" desc="uMOGI2 Configuration Bit Settings">

// PIC24FJ256GB108 - uMOGI2 Configuration Bit Settings
// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 170))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = HS             // Primary Oscillator Select (HS oscillator mode selected)
#pragma config DISUVREG = ON            // Internal USB 3.3V Regulator Disable bit (Regulator is enabled)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config FNOSC = PRIPLL           // Oscillator Select (Primary oscillator (XT, HS, EC) with PLL module (XTPLL,HSPLL, ECPLL))
#pragma config PLL_96MHZ = ON           // 96MHz PLL Disable (Enabled)
#pragma config PLLDIV = DIV3            // USB 96 MHz PLL Prescaler Select bits (Oscillator input divided by 3 (12MHz input))
#pragma config IESO = OFF               // Internal External Switch Over Mode (IESO mode (Two-speed start-up)disabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR32             // WDT Prescaler (Prescaler ratio of 1:32)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx2               // Comm Channel Select (Emulator functions are shared with PGEC2/PGED2)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

// </editor-fold>

#include <xc.h>         // �ltal�nos header a PIC24 csal�dhoz
#include <stdio.h>      // sprintf miatt
#include <stdlib.h>     // malloc
#include <string.h>     // memset
#include <math.h>

// Hardver tulajdons�gok:

// PLL ut�ni �rajel
#define SYS_FREQ        32000000L
#define FCY             SYS_FREQ/2

// M�sodlagos �rajel forr�s - �rakvarc
#define F_SEC_OSC       32768

// Delay f�ggv�nyek
#define DELAY_MS(ms)    __delay_ms(ms);
#define DELAY_US(us)    __delay_us(us);
#include <libpic30.h>   // __delay_ms

// LED-ek
#define LED1        _LATG6
#define LED2        _LATG7
#define LED3        _LATG8
#define LED4        _LATG9
#define LEDR        _LATD15
#define LEDG        _LATF4
#define LEDB        _LATF5

// Nyom�gombok
#define SW1         !_RC1
#define SW2         !_RC3
#define SW3         !_RE8
#define SW4         !_RE9

// <editor-fold defaultstate="collapsed" desc="LCD Settings">

// LCD l�bak
#define LCD_DATA    LATE // bit 0..7
#define LCD_RS      _LATG0
#define LCD_RW      _LATG1
#define LCD_E       _LATF1
#define LCD_BF      _RE7

// LCD makr�k
#define lcdPutChar(c) lcdWrite(c, 1)    // karakter k�ld�se
#define lcdPutCmd(d)  lcdWrite(d, 0)    // utas�t�s k�ld�se
#define lcdClear()    lcdWrite(0x01,0)  // LCD t�rl�se
#define lcdGoHome()   lcdWrite(0x02,0)  // LCD k�ld�se az 1. sorba
#define lcdGo2Row()   lcdWrite(0xC0,0)  // LCD k�ld�se a 2. sorba

// magyar �kezetes karakterek
const unsigned char hu_char[] = {
    0x02, 0x04, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00, // �
    0x02, 0x04, 0x0E, 0x11, 0x1F, 0x10, 0x0E, 0x00, // �
    0x02, 0x04, 0x0C, 0x04, 0x04, 0x04, 0x0E, 0x00, // �
    0x02, 0x04, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00, // �
    0x02, 0x04, 0x11, 0x11, 0x11, 0x13, 0x0D, 0x00, // �
    0x0A, 0x00, 0x11, 0x11, 0x11, 0x13, 0x0D, 0x00, // �
    0x05, 0x0A, 0x11, 0x11, 0x11, 0x13, 0x0D, 0x00, // ?
    0x05, 0x0A, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x00, // ?
};

void lcdInit();
void lcdWrite(uint8_t c, uint8_t rs);
void lcdPutStr(char *s);
void lcdLoadHuChars(void);

// </editor-fold>

void tmr5Init (){
    TMR5 = 0x0000; // Timer1 be�ll�t�sa
    T5CONbits.TCKPS = 3; // T1 elooszt� 1:254
    T5CONbits.TON=1;
        
}
void tmr3Init (){
    TMR3 = 0x0000; // Timer3 be�ll�t�sa
    T3CONbits.TCKPS = 2; // T3 el?oszt� 1:64
    T3CONbits.TON=1;
}
void timer_10Hz_init() {
    // Timer1 konfigur�l�sa
    T1CONbits.TON = 0; // timer kikapcsol�sa
    T1CONbits.TCS = 0; // �rajelforr�s: FCY -> 16MHz
    T1CONbits.TGATE = 0; // gated �zemm�d tilt�sa -> minden �rajelre sz�mol
    T1CONbits.TCKPS = 0b11; // 256-os el?oszt� -> 62500Hz
    PR1 = 6249; // peri�dus be�ll�t�sa -> 62500Hz/(1+62499)=1Hz
    TMR1 = 0; // sz�ml�l� null�z�sa

    // interrupt konfigur�l�sa
    _T1IP = 1; // 1-es priorit�s
    _T1IE = 1; // enged�lyez�s
    _T1IF = 0; // flag t�rl�s

    // �ra ind�t�sa
    T1CONbits.TON = 1;
}
double tizedmasodpercek = 0;


void pwmInitMotor(int duty){
    OC1CON1 = 0; // OC modul be�ll�t�sainak t�rl�se
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0; // TMR2-vel m?k�dik
    OC1CON2bits.SYNCSEL = 0x0C; // szinkroniz�l�s TMR2-vel
    PR2 = 1600 - 1; // TMR2 peri�dusa
    TMR2 = 0; // TMR2 t�r�se
    OC1R = duty; // kit�lt�s
    OC1CON1bits.OCM = 6; // Edge Aligned PWM mode
    T2CONbits.TON = 1; // TMR2 indul
}
void pwmset(int duty){
    if(duty<0){
        duty = 0;
    }
    OC1R=duty;
}
void cninit(){
    _CN14IE=0;
    _CN13IE=1;
    _CNIE=1;
    _CNIP=1;
    _CNIF=0;
}



float velocity_i;
float vtarget;
float kp = 28;
float e;
float eint;
float u;
int pwm;
volatile int32_t lepes=0;
char LCD[80];

int main() {
    //Inicializ�l�s - setup

    // �rajel forr�s be�ll�t�sa
    // PLL kimenete
    CLKDIVbits.CPDIV = 0; // 1:1 32MHz
    // megv�rjuk am�g a PLL modul v�gez
    while (!OSCCONbits.LOCK) Nop();

    // szoftveres WDT kikapcsol�sa
    RCONbits.SWDTEN = 0;

    // �rakvarc enged�lyez�se
    __builtin_write_OSCCONL(OSCCON | 0x02);

    // Perif�ria - l�b �sszerendel�s PPS (pp.135)
    __builtin_write_OSCCONL(OSCCON & 0xbf);  //PPSUnLock;
    _RP14R=18; //ENA <- OC1
    __builtin_write_OSCCONL(OSCCON | 0x40); //PPSLock;


    // L�bak ir�nya
    _TRISG6 = 0; // LED1 kimenet
    _TRISG7 = 0; // LED2 kimenet
    _TRISG8 = 0; // LED3 kimenet
    _TRISG9 = 0; // LED4 kimenet
    _TRISD15 = 0; // LEDR kimenet
    _TRISF4 = 0; // LEDG kimenet
    _TRISF5 = 0; // LEDB kimenet
    _TRISC1 = 1; // SW1 bemenet
    _TRISC3 = 1; // SW2 bemenet
    _TRISE8 = 1; // SW3 bemenet
    _TRISE9 = 1; // SW4 bemenet
    
    _TRISB8=0;
    _TRISB9=0;
    //_TRISB12=0;
    // _TRISB13=0;
    _TRISB14=0;
    _TRISD4=1;
    _TRISD5=1;

    lcdInit(); // LCD modul inicializ�l�sa
    lcdLoadHuChars(); // magyar karakterek bet�lt�se
    lcdPutStr("Hell� uMOGI2  :)"); // sz�veg ki�rat�sa
    DELAY_MS(2000);
    lcdClear();
    pwm=200;
    _LATB14=1;
    tmr5Init();
    tmr3Init();
    pwmInitMotor(pwm);
    cninit();
    timer_10Hz_init();
    
    

   
    
    
    // V�gtelen ciklus - super loop
    
    while (1) {
        
        LEDR = !LEDR;
        if(SW1){
            __delay_ms(10);
            while(SW1);
            __delay_ms(10);
            ;
            vtarget+=2;
            if(vtarget>36) vtarget=36;
        }
        if(SW2){
            __delay_ms(10);
            while(SW2);
            __delay_ms(10);
            vtarget-=1;
            if(vtarget<0) vtarget=0;
        }
        _LATB8=1;
        _LATB9=0;
        sprintf(LCD,"vt: %2f\nv: %0.4f",vtarget/2,velocity_i);
        lcdGoHome();
        lcdPutStr(LCD);
        pwmset(u);
        DELAY_MS(500);
    }

    return 0;
}

void _ISR _CNInterrupt(){
 //a magas B alacsonyx ++ A alacsony B magas ++ m�sik k�t eset -- sz�ml�l�t �s ez a sz�ml�l� modnja majd meg mennyit mozdult el a kezdeti �rt�khez k�pest
    
    LED2 = _RD5;
    LED3 = _RD4;
 
    if((_RD5 == 1 && _RD4 == 0) || (_RD5 == 0 && _RD4 == 1) ){
        lepes++;
    }
    else if ((_RD5 == 1 && _RD4 == 1) || (_RD5 == 0 && _RD4 == 0 )){
        lepes--;
    }
    
    _CNIF=0;
}


volatile int32_t lepes_i=0;
void _ISR _T1Interrupt(){
    tizedmasodpercek++;
    velocity_i= (lepes-lepes_i)*3.14*10/16/33;
    lepes_i = lepes;
    _T1IF = 0; // interrupt flag t�rl�se
    e = vtarget - velocity_i;
    u = kp *e ;
}


// <editor-fold defaultstate="collapsed" desc="LCD modul">

/**
 * <b>LCD inicializ�l�sa</b><br>
 * l�bak be�ll�t�sa<br>
 * ind�t�s 8 bites �zemm�dban<br>
 */
void lcdInit() {

    // az LCD vez�rl? vonalainak be�ll�t�sa
    TRISE &= 0xff00; // D0-D7 kimenet
    _TRISG0 = 0; // RS kimenet
    _TRISG1 = 0; // RW kimenet
    _TRISF1 = 0; // E kimenet
    LCD_RS = 0; // RS alacsony
    LCD_RW = 0; // RW alacsony
    LCD_E = 0; // E alacsony

    DELAY_MS(50); // v�r az eszk�z be�ll�s�ra

    lcdPutCmd(0x38); // 2 soros display, 5x8 karakter
    lcdPutCmd(0x08); // display off
    lcdPutCmd(0x01); // k�perny? t�rl�se, kurzor alaphelyzetbe �ll�t�s
    lcdPutCmd(0x06); // automatikus inkrement�l�s, nem l�pteti a kijelz?t
    lcdPutCmd(0x0C); // a display bekapcsol�sa; kurzor �s villog�s kikapcsolva

    DELAY_MS(3);
}

/**
 * adat vagy utas�t�s k�ld�se
 * @param c adat/karakter
 * @param rs 0 - utas�t�s, 1 - adat
 */
void lcdWrite(uint8_t c, uint8_t rs) {

    uint8_t BF; // BusyFlag (foglalts�g) figyel�se
    TRISE |= 0x00ff; // l�bak bemenetek
    LCD_RW = 1; // beolvas�s
    LCD_RS = 0; // utas�t�s
    LCD_DATA &= 0xff00;
    do {
        LCD_E = 1; // E magas
        Nop();
        Nop();
        Nop();
        BF = LCD_BF;
        LCD_E = 0; // E alacsony
    } while (BF);

    TRISE &= 0xff00; // l�bak kimenetek
    LCD_RW = 0; // �r�s
    LCD_RS = rs ? 1 : 0; // adat vagy parancs
    LCD_DATA = (LCD_DATA & 0xff00) | c; // 8 bit k�ld�se
    LCD_E = 1; // E magas
    Nop();
    Nop();
    Nop();
    LCD_E = 0; // E alacsony
}

/**
 * ki�r egy karakterf�z�rt az LCD-re
 * @param s Karaktert�mb
 */
void lcdPutStr(char *s) {
    while (*s) {
        char c = *s;
        switch (c) { // magyar karakterek cser�je a felt�lt�ttre
            case '�': c = 0x00;
                break;
            case '�': c = 0x01;
                break;
            case '�': c = 0x02;
                break;
            case '�': c = 0x03;
                break;
            case '�': c = 0x04;
                break;
            case '�': c = 0x05;
                break;
           // case '?': c = 0x06;
                break;
          //  case '?': c = 0x07;
                break;
            case '�': c = 0xEF;
                break;
        }
        if (c == '\n')
            lcdGo2Row(); // kurzor mozgat�sa a m�sodik sor elej�re
        else
            lcdPutChar(c); // karakter ki�rat�sa
        s++;
    }
}


// magyar �kezetes karakterek felt�lt�se a CGRAM-ba

void lcdLoadHuChars(void) {
    int i;
    lcdPutCmd(0x40); // kurzor a CGRAM elej�re (0. char)
    for (i = 0; i < 64; i++) {
        lcdPutChar(hu_char[i]); // defini�lhat� karakterek felt�lt�se
    } // �kezetes karakterekkel
    lcdPutCmd(0x80); // kurzor vissza, a DDRAM elej�re
}

// </editor-fold>