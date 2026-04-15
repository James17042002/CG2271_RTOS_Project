/*
 * slcd_display.c — 4-digit segment LCD driver for FRDM-MCXC444
 *
 * LCD glass: Lumex LCD-S401M16KR (DS1 on board)
 *
 * FRDM-MCXC444 SLCD wiring (from board schematic):
 *
 *   Glass Pin  GPIO    LCD_P   Function
 *   ─────────  ──────  ──────  ────────────────────────────
 *    1         PTE20   P59     COM0  (back-plane)
 *    2         PTE21   P60     COM1  (back-plane)
 *    3         PTB18   P14     COM2  (back-plane)
 *    4         PTB19   P15     COM3  (back-plane)
 *    5         PTC0    P20     D0: 1D / 1E / 1G / 1F  (digit 1 lo)
 *    6         PTC4    P24     D1: 1DP/ 1C / 1B / 1A  (digit 1 hi)
 *    7         PTC6    P26     D2: 2D / 2E / 2G / 2F  (digit 2 lo)
 *    8         PTC7    P27     D3: 2DP/ 2C / 2B / 2A  (digit 2 hi)
 *    9         PTD0    P40     D4: 3D / 3E / 3G / 3F  (digit 3 lo)
 *   10         PTD2    P42     D5: 3DP/ 3C / 3B / 3A  (digit 3 hi)
 *   11         PTD3    P43     D6: 4D / 4E / 4G / 4F  (digit 4 lo)
 *   12         PTD4    P44     D7: COL/ 4C / 4B / 4A  (digit 4 hi)
 *
 * WF8B bit mapping (1/4 duty, 4 COM):
 *   bit0 = COM0 (phase A)
 *   bit1 = COM1 (phase B)
 *   bit2 = COM2 (phase C)
 *   bit3 = COM3 (phase D)
 *
 * "lo" pin encodes: bit0=D  bit1=E  bit2=G  bit3=F
 * "hi" pin encodes: bit0=DP bit1=C  bit2=B  bit3=A
 *
 * 7-segment layout:
 *        _A_
 *       |F |B
 *        _G_
 *       |E |C
 *        _D_   .DP
 *
 * NOTE: JP7 must be SHORTED (default) so that VLL3 is supplied
 *       from P3V3 through the jumper.
 */

#include "slcd_display.h"
#include "board.h"          /* or your MCXC444 device header */
#include <stdint.h>

/* ================================================================
 *  Back-plane LCD pin numbers (index into WF8B[])
 * ================================================================ */
#define BP_COM0   59   /* PTE20 — glass pin 1 */
#define BP_COM1   60   /* PTE21 — glass pin 2 */
#define BP_COM2   14   /* PTB18 — glass pin 3 */
#define BP_COM3   15   /* PTB19 — glass pin 4 */

/* ================================================================
 *  Front-plane pairs: { lo_pin, hi_pin } per digit
 *  lo = D/E/G/F     hi = DP(or COL)/C/B/A
 * ================================================================ */
static const uint8_t fp[4][2] = {
    { 20, 24 },   /* digit 1 (leftmost):  D0=PTC0(P20),  D1=PTC4(P24) */
    { 26, 27 },   /* digit 2:             D2=PTC6(P26),  D3=PTC7(P27) */
    { 40, 42 },   /* digit 3:             D4=PTD0(P40),  D5=PTD2(P42) */
    { 43, 44 },   /* digit 4 (rightmost): D6=PTD3(P43),  D7=PTD4(P44) */
};

/* ================================================================
 *  Character → segment encoding
 * ================================================================ */
typedef struct { uint8_t lo; uint8_t hi; } seg_t;

static seg_t char_to_seg(char c)
{
    /*  7-segment layout:
     *        _A_
     *       |F |B
     *        _G_
     *       |E |C
     *        _D_   .DP
     *
     *  lo pin: bit0=D  bit1=E  bit2=G  bit3=F
     *  hi pin: bit0=DP bit1=C  bit2=B  bit3=A
     */
    switch (c) {
    /* ---- Digits ---- */
    case '0':           return (seg_t){ 0x0B, 0x0E }; /* A B C D E F   */
    case '1':           return (seg_t){ 0x00, 0x06 }; /*   B C         */
    case '2':           return (seg_t){ 0x07, 0x0C }; /* A B   D E   G */
    case '3':           return (seg_t){ 0x05, 0x0E }; /* A B C D     G */
    case '4':           return (seg_t){ 0x0C, 0x06 }; /*   B C     F G */
    case 'S': case 's': case '5': return (seg_t){ 0x0D, 0x0A }; /* A C D F G */
    case '6':           return (seg_t){ 0x0F, 0x0A }; /* A   C D E F G */
    case '7':           return (seg_t){ 0x00, 0x0E }; /* A B C         */
    case '8':           return (seg_t){ 0x0F, 0x0E }; /* A B C D E F G */
    case '9':           return (seg_t){ 0x0D, 0x0E }; /* A B C D   F G */

    /* ---- Uppercase ---- */
    case 'A':           return (seg_t){ 0x0E, 0x0E }; /* A B C   E F G */
    case 'B':           return (seg_t){ 0x0F, 0x02 }; /*     C D E F G */ /* same as b */
    case 'C':           return (seg_t){ 0x0B, 0x08 }; /* A     D E F   */
    case 'D':           return (seg_t){ 0x07, 0x06 }; /*   B C D E   G */ /* same as d */
    case 'E':           return (seg_t){ 0x0F, 0x08 }; /* A     D E F G */
    case 'F':           return (seg_t){ 0x0E, 0x08 }; /* A       E F G */
    case 'G':           return (seg_t){ 0x0F, 0x0C }; /* A   C D E F   */ /* same as 6 */
    case 'H':           return (seg_t){ 0x0E, 0x06 }; /*   B C   E F G */
    case 'I':           return (seg_t){ 0x0A, 0x00 }; /*         E F   */
    case 'J':           return (seg_t){ 0x03, 0x06 }; /*   B C D       */
    case 'K':           return (seg_t){ 0x0E, 0x08 }; /* A       E F G */ /* approx, like F */
    case 'L':           return (seg_t){ 0x0B, 0x00 }; /*       D E F   */
    case 'M':           return (seg_t){ 0x0A, 0x0E }; /* A B C   E F   */ /* approx */
    case 'N':           return (seg_t){ 0x06, 0x02 }; /*     C   E   G */ /* same as n */
    case 'O':           return (seg_t){ 0x0B, 0x0E }; /* A B C D E F   */ /* same as 0 */
    case 'P': case 'p': return (seg_t){ 0x0E, 0x0C }; /* A B   E F G */
    case 'Q':           return (seg_t){ 0x0C, 0x0E }; /* A B C     F G */ /* like 9 top */
    case 'R':           return (seg_t){ 0x06, 0x00 }; /*         E   G */ /* same as r */
    case 'T':           return (seg_t){ 0x0F, 0x00 }; /*       D E F G */ /* same as t */
    case 'U':           return (seg_t){ 0x0B, 0x06 }; /*   B C D E F   */
    case 'V':           return (seg_t){ 0x0B, 0x06 }; /*   B C D E F   */ /* same as U */
    case 'W':           return (seg_t){ 0x0B, 0x06 }; /*   B C D E F   */ /* approx */
    case 'X':           return (seg_t){ 0x0E, 0x06 }; /*   B C   E F G */ /* same as H */
    case 'Y':           return (seg_t){ 0x0D, 0x06 }; /*   B C     F G */ /* like 4+D */
    case 'Z':           return (seg_t){ 0x07, 0x0A }; /* A B   D E   G */ /* same as 2 */

    /* ---- Lowercase ---- */
    case 'a':           return (seg_t){ 0x07, 0x0E }; /* A B C D E   G */
    case 'b':           return (seg_t){ 0x0F, 0x02 }; /*     C D E F G */
    case 'c':           return (seg_t){ 0x07, 0x00 }; /*       D E   G */
    case 'd':           return (seg_t){ 0x07, 0x06 }; /*   B C D E   G */
    case 'e':           return (seg_t){ 0x0F, 0x0A }; /* A B   D E F G */
    case 'f':           return (seg_t){ 0x0E, 0x08 }; /* A       E F G */ /* same as F */
    case 'g':           return (seg_t){ 0x0D, 0x0E }; /* A B C D   F G */ /* same as 9 */
    case 'h':           return (seg_t){ 0x0E, 0x02 }; /*     C   E F G */
    case 'i':           return (seg_t){ 0x00, 0x02 }; /*     C         */
    case 'j':           return (seg_t){ 0x03, 0x06 }; /*   B C D       */ /* same as J */
    case 'k':           return (seg_t){ 0x0E, 0x08 }; /* A       E F G */ /* approx */
    case 'l':           return (seg_t){ 0x0A, 0x00 }; /*         E F   */ /* same as I */
    case 'm':           return (seg_t){ 0x06, 0x02 }; /*     C   E   G */ /* approx */
    case 'n':           return (seg_t){ 0x06, 0x02 }; /*     C   E   G */
    case 'o':           return (seg_t){ 0x07, 0x02 }; /*     C D E   G */
    case 'q':           return (seg_t){ 0x0C, 0x0E }; /* A B C     F G */
    case 'r':           return (seg_t){ 0x06, 0x00 }; /*         E   G */
    case 't':           return (seg_t){ 0x0F, 0x00 }; /*       D E F G */
    case 'u':           return (seg_t){ 0x03, 0x02 }; /*     C D E     */
    case 'v':           return (seg_t){ 0x03, 0x02 }; /*     C D E     */ /* same as u */
    case 'w':           return (seg_t){ 0x03, 0x02 }; /*     C D E     */ /* approx */
    case 'x':           return (seg_t){ 0x0E, 0x06 }; /*   B C   E F G */ /* same as H */
    case 'y':           return (seg_t){ 0x0D, 0x06 }; /*   B C     F G */
    case 'z':           return (seg_t){ 0x07, 0x0A }; /* A B   D E   G */ /* same as 2 */

    /* ---- Symbols ---- */
    case '-':           return (seg_t){ 0x04, 0x00 }; /*             G */
    case '_':           return (seg_t){ 0x01, 0x00 }; /*       D       */
    case ' ':           return (seg_t){ 0x00, 0x00 }; /* blank         */
    default:            return (seg_t){ 0x00, 0x00 }; /* blank         */
    }
}

/* ================================================================
 *  Initialisation
 * ================================================================ */

void SLCD_DisplayInit(void)
{
    /* 1. Enable clocks for SLCD peripheral + all LCD-pin ports */
    SIM->SCGC5 |= SIM_SCGC5_SLCD_MASK
               |  SIM_SCGC5_PORTB_MASK
               |  SIM_SCGC5_PORTC_MASK
               |  SIM_SCGC5_PORTD_MASK
               |  SIM_SCGC5_PORTE_MASK;

    /* Enable MCGIRCLK (slow IRC ~32 kHz) as the SLCD alternate clock source.
     * The SLCD peripheral uses this when SOURCE=1.
     * IMPORTANT: Must use slow IRC (~32 kHz), NOT fast IRC (~4 MHz).
     * A 4 MHz clock produces a frame rate in the kHz range — far too
     * fast for the LCD glass to multiplex properly.                   */
    MCG->C1 |= MCG_C1_IRCLKEN_MASK | MCG_C1_IREFSTEN_MASK;
    MCG->C2 &= ~MCG_C2_IRCS_MASK;         /* select slow IRC (~32 kHz) */

    /* 2. Set PADSAFE, disable LCD before configuring */
    LCD->GCR |= LCD_GCR_PADSAFE_MASK;
    LCD->GCR &= ~LCD_GCR_LCDEN_MASK;

    /* 3. Pin-mux: set every LCD pin to ALT0 (MUX=0 → LCD/analog) */
    /* Back-planes */
    PORTE->PCR[20] = 0;   /* LCD_P59  COM0 */
    PORTE->PCR[21] = 0;   /* LCD_P60  COM1 */
    PORTB->PCR[18] = 0;   /* LCD_P14  COM2 */
    PORTB->PCR[19] = 0;   /* LCD_P15  COM3 */
    /* Front-planes */
    PORTC->PCR[0]  = 0;   /* LCD_P20  D0 — digit 1 lo */
    PORTC->PCR[4]  = 0;   /* LCD_P24  D1 — digit 1 hi */
    PORTC->PCR[6]  = 0;   /* LCD_P26  D2 — digit 2 lo */
    PORTC->PCR[7]  = 0;   /* LCD_P27  D3 — digit 2 hi */
    PORTD->PCR[0]  = 0;   /* LCD_P40  D4 — digit 3 lo */
    PORTD->PCR[2]  = 0;   /* LCD_P42  D5 — digit 3 hi */
    PORTD->PCR[3]  = 0;   /* LCD_P43  D6 — digit 4 lo */
    PORTD->PCR[4]  = 0;   /* LCD_P44  D7 — digit 4 hi */

    /* 4. Enable LCD pins in PEN registers
     *    PEN[0] covers LCD_P0..31,  PEN[1] covers LCD_P32..63 */
    LCD->PEN[0] = (1u << 14)    /* LCD_P14 COM2 */
                | (1u << 15)    /* LCD_P15 COM3 */
                | (1u << 20)    /* LCD_P20 D0 */
                | (1u << 24)    /* LCD_P24 D1 */
                | (1u << 26)    /* LCD_P26 D2 */
                | (1u << 27);   /* LCD_P27 D3 */
    LCD->PEN[1] = (1u << (40 - 32))   /* LCD_P40 D4 */
                | (1u << (42 - 32))   /* LCD_P42 D5 */
                | (1u << (43 - 32))   /* LCD_P43 D6 */
                | (1u << (44 - 32))   /* LCD_P44 D7 */
                | (1u << (59 - 32))   /* LCD_P59 COM0 */
                | (1u << (60 - 32));  /* LCD_P60 COM1 */

    /* 5. Mark back-plane pins in BPEN registers */
    LCD->BPEN[0] = (1u << 14)    /* LCD_P14 COM2 */
                 | (1u << 15);   /* LCD_P15 COM3 */
    LCD->BPEN[1] = (1u << (59 - 32))   /* LCD_P59 COM0 */
                 | (1u << (60 - 32));  /* LCD_P60 COM1 */

    /* 6. Set back-plane phase assignments */
    LCD->WF8B[BP_COM0] = 0x01;   /* phase A */
    LCD->WF8B[BP_COM1] = 0x02;   /* phase B */
    LCD->WF8B[BP_COM2] = 0x04;   /* phase C */
    LCD->WF8B[BP_COM3] = 0x08;   /* phase D */

    /* 7. General Control Register
     *    DUTY    = 3  → 1/4 duty (4 back-planes)
     *    LCLK    = 4  → prescaler for ~30-60 Hz frame rate
     *    SOURCE  = 1  → alternate clock (MCGIRCLK ≈ 32 kHz slow IRC)
     *    LADJ    = 3  → fastest charge-transfer (best for glass load)
     *    VSUPPLY = 1  → VLL3 supplied externally via JP7
     *    CPSEL   = 0  → resistor bias (no charge pump; JP7 shorted)
     */
    LCD->GCR = LCD_GCR_DUTY(3)
             | LCD_GCR_LCLK(4)
             | LCD_GCR_SOURCE_MASK
             | LCD_GCR_LADJ(3)
             | LCD_GCR_VSUPPLY(1);

    /* 8. Clear all front-plane waveform data */
    SLCD_Clear();

    /* 9. Clear PADSAFE, then enable LCD */
    LCD->GCR &= ~LCD_GCR_PADSAFE_MASK;
    LCD->GCR |= LCD_GCR_LCDEN_MASK;
}

/* ================================================================
 *  Clear / Display
 * ================================================================ */

void SLCD_Clear(void)
{
    for (int d = 0; d < 4; d++) {
        LCD->WF8B[fp[d][0]] = 0;
        LCD->WF8B[fp[d][1]] = 0;
    }
}

void SLCD_ShowString(const char *str)
{
    for (int d = 0; d < 4; d++) {
        char c = (str && *str) ? *str++ : ' ';
        seg_t s = char_to_seg(c);
        LCD->WF8B[fp[d][0]] = s.lo;
        LCD->WF8B[fp[d][1]] = s.hi;
    }
}
