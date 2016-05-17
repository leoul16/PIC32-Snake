////////////////////////////////////////////////////////////////////////////////
// ECE 2534:        Lab 3 Leoul Yiheyis
//
// File name:       main.c
//
// Description:     Implementation of Snake Game
//
// How to use:      Plug in OLED, ADC to PIC32 and enjoy!


// Comment this define out to *not* use the OledDrawGlyph() function
#define USE_OLED_DRAW_GLYPH

#ifdef USE_OLED_DRAW_GLYPH
// forward declaration of OledDrawGlyph() to satisfy the compiler
void OledDrawGlyph(char ch);
#endif

#define _PLIB_DISABLE_LEGACY
#include <plib.h>
#include "PmodOLED.h"
#include "OledChar.h"
#include "OledGrph.h"
#include "delay.h"
#include <time.h>

// Configure the microcontroller
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_8         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx1      // ICE/ICD Comm Channel Select
#pragma config DEBUG    = OFF           // Debugger Disabled for Starter Kit

// Intrumentation for the logic analyzer (or oscilliscope)
#define MASK_DBG1  0x1;
#define MASK_DBG2  0x2;
#define MASK_DBG3  0x4;
#define DBG_ON(a)  LATESET = a
#define DBG_OFF(a) LATECLR = a
#define DBG_INIT() TRISECLR = 0x7

// Definitions for the ADC averaging. How many samples (should be a power
// of 2, and the log2 of this number to be able to shift right instead of
// divide to get the average.
#define NUM_ADC_SAMPLES 32
#define LOG2_NUM_ADC_SAMPLES 5


// Computed ADC values
int ADC_UD, ADC_LR;


// Return value check macro
#define CHECK_RET_VALUE(a) { \
  if (a == 0) { \
    LATGSET = 0xF << 12; \
    return(EXIT_FAILURE) ; \
  } \
}
    typedef struct _position {
        int x;
        int y;
    }position;


// Global variable to count number of times in timer2 ISR
unsigned int timer2_ms_value = 0;

enum States {WELC, OPTIONS, INIT_SNAKE, INIT_APPLE, READ, MOVE, DONE};
enum Direction {UP, DOWN, LEFT, RIGHT, NONE}direc, lastdirec;
// Global definitions of our user-defined characters
char diamond_char = 0x00;
BYTE blank[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
char blank_char = 0x01;
char headc = 0x02;
char bodc = 0x03;
char applec = 0x03;

BYTE apple[8] = {0x00, 0x30, 0x48, 0x48, 0x4C, 0x4E, 0x31, 0x00};
BYTE head[8] = {0x08, 0x36, 0x22, 0x45, 0x41, 0x62, 0x36, 0x08};
BYTE body[8] = {0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

unsigned char BTN1Hist = 0x00;
unsigned char BTN2Hist = 0x00;
unsigned char BTN3Hist = 0x00;



void __ISR(_ADC_VECTOR, IPL7SRS) _ADCHandler(void) {
    if(ReadActiveBufferADC10() == 0)
    {
        ADC_LR = ReadADC10(0);
        ADC_UD = ReadADC10(1);

    }

    else if (ReadActiveBufferADC10() == 1)
    {
        ADC_LR = ReadADC10(8);
        ADC_UD = ReadADC10(9);

    }
    INTClearFlag(INT_AD1);
}


// The interrupt handler for timer2
// IPL4 medium interrupt priority
// SOFT|AUTO|SRS refers to the shadow register use
void __ISR(_TIMER_3_VECTOR, IPL4AUTO) _Timer3Handler(void) {
    DBG_ON(MASK_DBG1);
    
    timer2_ms_value++; // Increment the millisecond counter.


    BTN1Hist <<= 1; //updates history
    if (PORTG & 0x40)
        BTN1Hist |= 0x01;

    BTN2Hist <<= 1;
    if (PORTG & 0x80)
        BTN2Hist |= 0x01;

    BTN3Hist <<= 1;
    if (PORTA & 0x01)
        BTN3Hist |= 0x01;

    DBG_OFF(MASK_DBG1);
    INTClearFlag(INT_T3); // Acknowledge the interrupt source by clearing its flag.
}

// Initialize timer2 and set up the interrupts

void initTimer2() {
    // Configure Timer 2 to request a real-time interrupt once per millisecond.
    // The period of Timer 2 is (16 * 625)/(10 MHz) = 1 s.
    OpenTimer3(T3_ON | T3_IDLE_CON | T3_SOURCE_INT | T3_PS_1_16 | T3_GATE_OFF, 624);
    INTSetVectorPriority(INT_TIMER_3_VECTOR, INT_PRIORITY_LEVEL_4);
    INTClearFlag(INT_T3);
    INTEnable(INT_T3, INT_ENABLED);
}

//***********************************************
//* IMPORTANT: THE ADC CONFIGURATION SETTINGS!! *
//***********************************************

// ADC MUX Configuration
// Only using MUXA, AN2 as positive input, VREFL as negative input
#define AD_MUX_CONFIG ADC_CH0_POS_SAMPLEA_AN2 | ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEB_AN3

// ADC Config1 settings
// Data stored as 16 bit unsigned int
// Internal clock used to start conversion
// ADC auto sampling (sampling begins immediately following conversion)
#define AD_CONFIG1 ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

// ADC Config2 settings
// Using internal (VDD and VSS) as reference voltages
// Do not scan inputs
// One sample per interrupt
// Buffer mode is one 16-word buffer
// Alternate sample mode off (use just MUXA)
#define AD_CONFIG2 ADC_VREF_AVDD_AVSS | ADC_SCAN_OFF | \
ADC_SAMPLES_PER_INT_2 | \
ADC_BUF_16 | ADC_ALT_INPUT_ON

// ADC Config3 settings
// Autosample time in TAD = 8
// Prescaler for TAD:  the 20 here corresponds to a
// ADCS value of 0x27 or 39 decimal => (39 + 1) * 2 * TPB = 8.0us = TAD
// NB: Time for an AD conversion is thus, 8 TAD for aquisition +
//     12 TAD for conversion = (8+12)*TAD = 20*8.0us = 160us.
#define AD_CONFIG3 ADC_SAMPLE_TIME_8 | ADC_CONV_CLK_20Tcy

// ADC Port Configuration (PCFG)
// Not scanning, so nothing need be set here..
// NB: AN2 was selected via the MUX setting in AD_MUX_CONFIG which
// sets the AD1CHS register (true, but not that obvious...)
#define AD_CONFIGPORT ENABLE_AN2_ANA | ENABLE_AN3_ANA

// ADC Input scan select (CSSL) -- skip scanning as not in scan mode
#define AD_CONFIGSCAN SKIP_SCAN_ALL

// Initialize the ADC using my definitions
// Set up ADC interrupts
void initADC(void) {

    // Configure and enable the ADC HW
    SetChanADC10(AD_MUX_CONFIG);
    OpenADC10(AD_CONFIG1, AD_CONFIG2, AD_CONFIG3, AD_CONFIGPORT, AD_CONFIGSCAN);
    EnableADC10();

    // Set up, clear, and enable ADC interrupts
    INTSetVectorPriority(INT_ADC_VECTOR, INT_PRIORITY_LEVEL_7);
    INTClearFlag(INT_AD1);
    INTEnable(INT_AD1, INT_ENABLED);
}

void initBasic(void)
{
        // Initialize GPIO for LEDs
    TRISGCLR = 0xf000; // For LEDs: configure PortG pins for output
    ODCGCLR = 0xf000; // For LEDs: configure as normal output (not open drain)

    TRISGSET = 0x40;
    TRISGSET = 0x80;
    TRISASET = 0x01;

    TRISBCLR = 1<<6;
    ODCBCLR  = 1<<6;
    LATBCLR  = 1<<6;
    LATBSET  = 1<<6;


    // Initialize Timer1 and OLED for display
    DelayInit();
    OledInit();

    OledDefUserChar(applec, apple);
    OledDefUserChar(headc, head);
    OledDefUserChar(bodc, body);


}
/*
 * SNAKE GAME IMPLEMENTATION
 *
 * Uses an array of struct position as the snake body, with the head always at
 * at the first element (spos[0]). The apple is a single position struct.
 *
 * The game is run through a state machine with apporpriate functions and
 * timing for different screens and game operations.
 */
int main() {
    char oledstring[17];
    unsigned int timer2_ms_value_save;
    unsigned int last_oled_update = 0;
    unsigned int ms_since_last_oled_update;
    unsigned int glyph_pos_x = 0, glyph_pos_y = 2;
    int retValue = 0;

    int won = 0;
    int lost = 0;

    DDPCONbits.JTAGEN = 0;


    initBasic();

    srand( time(NULL));


    // Set up our user-defined characters for the OLED
    retValue = OledDefUserChar(blank_char, blank);
    CHECK_RET_VALUE(retValue);
    retValue = OledDefUserChar(diamond_char, body);
    CHECK_RET_VALUE(retValue);

    // Initialize GPIO for debugging
    DBG_INIT();

    // Initial Timer2 & ADC
    initTimer2();
    initADC();

    // Configure the system for vectored interrupts
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);

    // Enable the interrupt controller
    INTEnableInterrupts();

    //initialize snake pos., apple pos., snake length & geneate new apple
    position spos[32];
    position apos;
    newApple(&apos);
    unsigned int slen;
    int i = 0; //used for for loops


    enum States state = WELC; //initial state
    enum Direction direc = NONE; //initial direction
    int goal = 0; //initialize goal variable

    start: while (state != DONE) //begining of state machine
    {
        switch (state)
        {
            case WELC:  //stays for 5 seconds
                OledClearBuffer();
                    OledSetCursor(0, 0);
                    OledPutString("Snake Game");
                    OledUpdate();
                    OledSetCursor(0,1);
                    OledPutString("Leoul Yiheyis");
                    OledUpdate();
                    OledSetCursor(0,2);
                    OledPutString("November 5, 2015");
                    OledUpdate();
                while ((timer2_ms_value/1000) < 5) //changes when timer2_ms_value = 5000
                {
                    
                    state = OPTIONS;

                }

                OledClearBuffer();
                break;

            case OPTIONS: //Options screen to pick goal

                OledSetCursor(0, 0);
                sprintf(oledstring, "Set Goal:  %4d", goal);
                OledPutString(oledstring);
                OledSetCursor(0, 1);
                OledPutString("BTN 1:  Goal++");
                OledSetCursor(0, 2);
                OledPutString("BTN 2:  Goal--");
                OledSetCursor(0, 3);
                OledPutString("BTN 3:  Accept");
                OledUpdate();

                if (BTN1Hist == 0xFF) //button 1 increments
                {
                    goal++;
                    BTN1Hist = 0x00;
                    OledUpdate();
                }

                if (BTN2Hist == 0xFF) //button 2 decrements
                {
                    goal--;
                    BTN2Hist = 0x00;
                    OledUpdate();
                }

                if (BTN3Hist == 0xFF) //button 3 statrs game
                {
                    OledClearBuffer();
                    state = READ;
                    BTN3Hist = 0x00;
                    

                }
                break;

            case INIT_SNAKE: //Initialize Snake State
                slen = 1; //always starts at 1
                spos[0].x = 1;  //starting coordinates
                spos[0].y = 0;
                spos[1].x = 0;
                spos[1].y = 0;

                lastdirec = RIGHT; //if direc = NONE, then snake moves towards last direc
                direc = NONE;

                newApple(&apos);  //new apple
                newApple(&apos);

                // SETS CURSOR AND DRAWS HEAD
                OledSetCursor(spos[0].x, spos[0].y);
                #ifdef USE_OLED_DRAW_GLYPH
                     OledDrawGlyph(headc);
                      //OledDrawGlyph(sbod);
                     OledUpdate();
                #else
                            //OledDrawGlyph(headc);
                            OledPutChar(headc);
                            //OledUpdate();
                #endif


                // TO DRAW BODY

                    OledSetCursor(spos[1].x, spos[1].y);

                     #ifdef USE_OLED_DRAW_GLYPH
                            OledDrawGlyph(bodc);
                            OledUpdate();
                    #else
                                OledPutChar(bodc);
                    #endif
                

                // TO DRAW APPLE
                OledSetCursor(apos.x, apos.y);
                #ifdef USE_OLED_DRAW_GLYPH
                       //OledDrawGlyph(shead);
                       OledDrawGlyph(applec);
                       OledUpdate();
                #else
                       OledPutChar(applec);
                #endif


                // TO DRAW GAME BOUNDARY AND SCORE

                OledSetCursor(9, 0);
                OledPutString("|");
                OledSetCursor(9, 1);
                OledPutString("|");
                OledSetCursor(9, 2);
                OledPutString("|");
                OledSetCursor(9, 3);
                OledPutString("|");

                OledSetCursor(10, 0);
                OledPutString("Score");
                OledSetCursor(10, 1);
                sprintf(oledstring, "%4d", slen);
                OledPutString(oledstring);
                OledSetCursor(10, 2);
                OledPutString("Goal");
                OledSetCursor(10, 3);
                sprintf(oledstring, "%4d", goal);
                OledPutString(oledstring);
                OledUpdate();



                state = MOVE;
                break;

            case MOVE: // STATE THAT FACILITATES SNAKE MOVEMENT

                if (eatApple(spos, apos, &slen) == 1) // CHECKS IF APPLE IS EATEN
                {
                    newApple(&apos);
                    slen++;
                    displayGame(spos, apos, slen, goal);
                }


                //        M O V E M E N T

                // Algorithm for snake movements

                if (direc == NONE)
                    direc = lastdirec;

                if (direc == RIGHT)
                {
                    for (i = slen; i >= 0; i--)
                    {
                        spos[i+1] = spos[i];

                    }
                    spos[0].x++;
                }
                else if (direc == LEFT)
                {
                    for (i = slen; i >= 0; i--)
                    {
                        spos[i+1] = spos[i];
                    }

                    spos[0].x--;
                }
                else if (direc == UP)
                {
                    for (i = slen; i >= 0; i--)
                    {
                        spos[i+1] = spos[i];
                    }

                    spos[0].y--;
                }
                else if (direc == DOWN)
                {
                    for (i = slen; i >= 0; i--)
                    {
                        spos[i+1] = spos[i];
                    }

                    spos[0].y++;
                }


                // D I S P L A Y
                displayGame(spos, apos, slen, goal);

                 state = READ;


                // if button1 is pressed restart game
                if (BTN1Hist == 0xFF)
                {
                    state = OPTIONS;
                    BTN1Hist = 0x00;
                }



                // WIN CASE
                if (slen == goal) //if length & goal are ==
                {
                    state = DONE;
                    won = 1;
                    break;
                }


                // LOSE CASES

                for (i = 0; i < slen+1; i++) //if snake hits itself
                {
                    if ((spos[0].x == spos[i].x) && (spos[0].y == spos[i].y))
                    {
                        lost = 1;
                        won = 0;

                    }
                }

                if ((spos[0].x == 0) && (direc == LEFT)) //left boundary
                {
                    lost = 1;
                    won = 0;
                    state = DONE;
                    break;
                }
                else if ((spos[0].y == 3) && (direc == DOWN)) // bottom boundary
                {
                    lost = 1;
                    won = 0;
                    state = DONE;
                    break;
                }
                else if ((spos[0].x == 8) && (direc == RIGHT)) // right boundary
                {
                    lost = 1;
                    won = 0;
                    state = DONE;
                    break;
                }
                else if ((spos[0].y == 0) && (direc == UP)) // top boundary
                {
                    lost = 1;
                    won = 0;
                    state = DONE;
                    break;
                }

                 break;

            case READ: //READS ADC
                OledUpdate();

                    ms_since_last_oled_update = timer2_ms_value - last_oled_update;
                    if (ms_since_last_oled_update >= 100) {
                        DBG_ON(MASK_DBG3);
                        //LATGSET = 1 << 12; // Turn LED1 on
                        timer2_ms_value_save = timer2_ms_value;
                        last_oled_update = timer2_ms_value;

                        //IF ADC IS IN MIDDLE
                        if ((ADC_UD < 550) & (ADC_UD > 490) & (ADC_LR < 550) & (ADC_LR > 490))
                        {
                            lastdirec = direc;
                            direc = NONE;
                            state = MOVE;
                            break;
                        }
                        if (ADC_UD < 490)
                        {
                            direc = DOWN;
                            state = MOVE;

                        }
                        if (ADC_UD > 560)
                        {
                            direc = UP;
                            state = MOVE;
                        }
                        if (ADC_LR < 490)
                        {
                            direc = LEFT;
                            state = MOVE;
                        }

                        if (ADC_LR > 550)
                        {
                            direc = RIGHT;
                            state = MOVE;
                        }

                        DBG_OFF(MASK_DBG3);
                    }
                //}

                        if (BTN1Hist == 0xFF)
                        {
                            state = OPTIONS;
                            BTN1Hist = 0x00;
                        }
               break;

            case DONE:
                break;
        }
    

    if (won == 1)
    {
        OledClearBuffer();
        OledSetCursor(0, 0);
        OledPutString("You Won!");
        OledUpdate();

                if (BTN1Hist == 0xFF)
                {
                    BTN1Hist = 0x00;
                    goto start;
                }

    }

    if (lost == 1)
    {
        OledClearBuffer();
        OledSetCursor(0, 0);
        OledPutString("You Lost!");
        OledUpdate();

                if (BTN1Hist == 0xFF)
                {
                    BTN1Hist = 0x00;
                    goto start;
                }

    }

    return (EXIT_SUCCESS);
}
}


/*
 * eatApple()
 *
 * Description:
 *      Lets program know if snake must eat an apple
 *
 * Pre:
 *      spos[] and apos are not NULL
 *
 * Input:
 *      spos[] - position array of snake
 *      apos   - position of single apple
 *
 * Output:
 *      returns 1 if True, 0 if false
 *
 * Post:
 *      N/A
 *
 */
int eatApple(position spos[], position apos)
{
    if ((spos[0].x == apos.x) && (spos[0].y == apos.y))
    {
        //*slen++;
        newApple(&apos);
        return 1;
    }
    else
        return 0;

}


/*
 * newApple()
 *
 * Description:
 *      Produces new apple
 *
 * Pre:
 *      apos is not NULL
 *
 * Input:
 *      *apos   - pointer to position of single apple
 *
 * Output:
 *      N/A
 *
 * Post:
 *      New Apple is made.
 *
 */
void newApple(position *apos)
{

    apos->x = rand() % 8 + 1;
    apos->y = rand() % 3 + 1;

    //apos->x = 2;
    //apos->y = 4;
}

/*
 * displayGame()
 *
 * Description:
 *      Displays game on OLED
 *
 * Pre:
 *      spos[], apos, slen, gol are not NULL
 *
 * Input:
 *      spos[] - position array of snake
 *      apos   - position of single apple
 *      slen   - length of snake
 *      gol    - goal to win gme
 *
 * Output:
 *      N/A
 *
 * Post:
 *      N/A
 *
 */
void displayGame(position spos[], position apos, int slen, int gol)
{
    char oledstr[17];

     OledSetCursor(apos.x, apos.y);
     #ifdef USE_OLED_DRAW_GLYPH
           OledDrawGlyph(applec);
           OledUpdate();
     #else
     #endif

     OledSetCursor(spos[0].x, spos[0].y);
     #ifdef USE_OLED_DRAW_GLYPH
        OledDrawGlyph(headc);
        OledUpdate();
     #else
        OledPutChar(shead);
     #endif

     OledSetCursor(spos[1].x, spos[1].y);
     #ifdef USE_OLED_DRAW_GLYPH
        OledDrawGlyph(bodc);
        OledUpdate();
     #else
        OledPutChar(sbod);
     #endif

     OledSetCursor(spos[slen+1].x, spos[slen+1].y);
     #ifdef USE_OLED_DRAW_GLYPH
        OledDrawGlyph(blank_char);
        OledUpdate();
     #else
        OledPutChar(blank_char);
     #endif


     OledSetCursor(9, 0);
     OledPutString("|");
     OledSetCursor(9, 1);
     OledPutString("|");
     OledSetCursor(9, 2);
     OledPutString("|");
     OledSetCursor(9, 3);
     OledPutString("|");

     OledSetCursor(10, 0);
     OledPutString("Score");
     OledSetCursor(10, 1);
     sprintf(oledstr, "%4d", slen);
     OledPutString(oledstr);
     OledSetCursor(10, 2);
     OledPutString("Goal");
     OledSetCursor(10, 3);
     sprintf(oledstr, "%4d", gol);
     OledPutString(oledstr);
     OledUpdate();

}
