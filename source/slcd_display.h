#ifndef SLCD_DISPLAY_H
#define SLCD_DISPLAY_H

/*
 * slcd_display.h — 4-digit segment LCD driver for FRDM-MCXC444
 * LCD glass: Lumex LCD-S401M16KR (DS1 on board)
 */

void SLCD_DisplayInit(void);
void SLCD_ShowString(const char *str);   /* shows up to 4 chars */
void SLCD_Clear(void);

#endif /* SLCD_DISPLAY_H */
