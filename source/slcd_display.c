/*
 * slcd_display.c — 4-digit segment LCD driver for FRDM-MCXC444
 *
 * LCD glass: Lumex LCD-S401M16KR (DS1 on board)
 *
 * FRDM-MCXC444 SLCD wiring (from board user manual Table 13):
 *
 *   Glass Pin  GPIO    LCD_P   Function
 *   ─────────  ──────  ──────  ────────────────────────────
 *    1         PTE20   P59     COM0  (back-plane)
 *    2         PTE21   P60     1D / 1E / 1G / 1F  (digit 1 lo)
 *    3         PTB18   P14     1DP/ 1C / 1B / 1A  (digit 1 hi)
 *    4         PTB19   P15     2D / 2E / 2G / 2F  (digit 2 lo)
 *    5         PTC0    P20     COM1  (back-plane)
 *    6         PTC4    P24     2DP/ 2C / 2B / 2A  (digit 2 hi)
 *    7         PTC6    P26     3D / 3E / 3G / 3F  (digit 3 lo)
 *    8         PTC7    P27     COM2  (back-plane)
 *    9         PTD0    P40     3DP/ 3C / 3B / 3A  (digit 3 hi)
 *   10         PTD2    P42     4D / 4E / 4G / 4F  (digit 4 lo)
 *   11         PTD3    P43     COM3  (back-plane)
 *   12         PTD4    P44     COL/ 4C / 4B / 4A  (digit 4 hi)
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
#define BP_COM0   59   /* PTE20 */
#define BP_COM1   20   /* PTC0  */
#define BP_COM2   27   /* PTC7  */
#define BP_COM3   43   /* PTD3  */

/* ================================================================
 *  Front-plane pairs: { lo_pin, hi_pin } per digit
 *  lo = D/E/G/F     hi = DP(or COL)/C/B/A
 * ================================================================ */
static const uint8_t fp[4][2] = {
    { 60, 14 },   /* digit 1 (leftmost):  PTE21, PTB18 */
    { 15, 24 },   /* digit 2:             PTB19, PTC4  */
    { 26, 40 },   /* digit 3:             PTC6,  PTD0  */
    { 42, 44 },   /* digit 4 (rightmost): PTD2,  PTD4  */
};

/* ================================================================
 *  Character → segment encoding
 * ================================================================ */
typedef struct { uint8_t lo; uint8_t hi; } seg_t;

static seg_t char_to_seg(char c)
{
    /*                     A B C D E F G                             */
    switch (c) {
    case '0': case 'O': return (seg_t){ 0x0B, 0x0E }; /* 1 1 1 1 1 1 0 */
    case '1':           return (seg_t){ 0x00, 0x06 }; /* 0 1 1 0 0 0 0 */
    case '2':           return (seg_t){ 0x07, 0x0A }; /* 1 1 0 1 1 0 1 */
    case '3':           return (seg_t){ 0x05, 0x0E }; /* 1 1 1 1 0 0 1 */
    case '4':           return (seg_t){ 0x0C, 0x06 }; /* 0 1 1 0 0 1 1 */
    case '5': case 'S': return (seg_t){ 0x0D, 0x0A }; /* 1 0 1 1 0 1 1 */
    case '6': case 'G': return (seg_t){ 0x0F, 0x0A }; /* 1 0 1 1 1 1 1 */
    case '7':           return (seg_t){ 0x00, 0x0E }; /* 1 1 1 0 0 0 0 */
    case '8':           return (seg_t){ 0x0F, 0x0E }; /* 1 1 1 1 1 1 1 */
    case '9':           return (seg_t){ 0x0D, 0x0E }; /* 1 1 1 1 0 1 1 */
    case 'A':           return (seg_t){ 0x0E, 0x0E }; /* 1 1 1 0 1 1 1 */
    case 'b':           return (seg_t){ 0x0F, 0x02 }; /* 0 0 1 1 1 1 1 */
    case 'C':           return (seg_t){ 0x0B, 0x08 }; /* 1 0 0 1 1 1 0 */
    case 'c':           return (seg_t){ 0x07, 0x00 }; /* 0 0 0 1 1 0 1 */
    case 'd':           return (seg_t){ 0x07, 0x06 }; /* 0 1 1 1 1 0 1 */
    case 'E':           return (seg_t){ 0x0F, 0x08 }; /* 1 0 0 1 1 1 1 */
    case 'F':           return (seg_t){ 0x0E, 0x08 }; /* 1 0 0 0 1 1 1 */
    case 'H':           return (seg_t){ 0x0E, 0x06 }; /* 0 1 1 0 1 1 1 */
    case 'h':           return (seg_t){ 0x0E, 0x02 }; /* 0 0 1 0 1 1 1 */
    case 'I':           return (seg_t){ 0x0A, 0x00 }; /* 0 0 0 0 1 1 0 */
    case 'J':           return (seg_t){ 0x03, 0x06 }; /* 0 1 1 1 0 0 0 */
    case 'L':           return (seg_t){ 0x0B, 0x00 }; /* 0 0 0 1 1 1 0 */
    case 'n':           return (seg_t){ 0x06, 0x02 }; /* 0 0 1 0 1 0 1 */
    case 'o':           return (seg_t){ 0x07, 0x02 }; /* 0 0 1 1 1 0 1 */
    case 'P':           return (seg_t){ 0x0E, 0x0C }; /* 1 1 0 0 1 1 1 */
    case 'r':           return (seg_t){ 0x06, 0x00 }; /* 0 0 0 0 1 0 1 */
    case 't':           return (seg_t){ 0x0F, 0x00 }; /* 0 0 0 1 1 1 1 */
    case 'U':           return (seg_t){ 0x0B, 0x06 }; /* 0 1 1 1 1 1 0 */
    case 'u':           return (seg_t){ 0x03, 0x02 }; /* 0 0 1 1 1 0 0 */
    case 'Y':           return (seg_t){ 0x0D, 0x06 }; /* 0 1 1 0 0 1 1 */
    case '-':           return (seg_t){ 0x04, 0x00 }; /* 0 0 0 0 0 0 1 */
    default:            return (seg_t){ 0x00, 0x00 }; /* blank           */
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
    PORTC->PCR[0]  = 0;   /* LCD_P20  COM1 */
    PORTC->PCR[7]  = 0;   /* LCD_P27  COM2 */
    PORTD->PCR[3]  = 0;   /* LCD_P43  COM3 */
    /* Front-planes */
    PORTE->PCR[21] = 0;   /* LCD_P60  digit 1 lo */
    PORTB->PCR[18] = 0;   /* LCD_P14  digit 1 hi */
    PORTB->PCR[19] = 0;   /* LCD_P15  digit 2 lo */
    PORTC->PCR[4]  = 0;   /* LCD_P24  digit 2 hi */
    PORTC->PCR[6]  = 0;   /* LCD_P26  digit 3 lo */
    PORTD->PCR[0]  = 0;   /* LCD_P40  digit 3 hi */
    PORTD->PCR[2]  = 0;   /* LCD_P42  digit 4 lo */
    PORTD->PCR[4]  = 0;   /* LCD_P44  digit 4 hi */

    /* 4. Enable LCD pins in PEN registers
     *    PEN[0] covers LCD_P0..31,  PEN[1] covers LCD_P32..63 */
    LCD->PEN[0] = (1u << 14)    /* LCD_P14 */
                | (1u << 15)    /* LCD_P15 */
                | (1u << 20)    /* LCD_P20 BP */
                | (1u << 24)    /* LCD_P24 */
                | (1u << 26)    /* LCD_P26 */
                | (1u << 27);   /* LCD_P27 BP */
    LCD->PEN[1] = (1u << (40 - 32))   /* LCD_P40 */
                | (1u << (42 - 32))   /* LCD_P42 */
                | (1u << (43 - 32))   /* LCD_P43 BP */
                | (1u << (44 - 32))   /* LCD_P44 */
                | (1u << (59 - 32))   /* LCD_P59 BP */
                | (1u << (60 - 32));  /* LCD_P60 */

    /* 5. Mark back-plane pins in BPEN registers */
    LCD->BPEN[0] = (1u << 20)    /* LCD_P20 COM1 */
                 | (1u << 27);   /* LCD_P27 COM2 */
    LCD->BPEN[1] = (1u << (43 - 32))   /* LCD_P43 COM3 */
                 | (1u << (59 - 32));  /* LCD_P59 COM0 */

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
