/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2018 KYOCERA Corporation
 */


#ifndef KC_TOUCH_DISPLAY
#define KC_TOUCH_DISPLAY
//#define CONFIG_INCELL_TOUCH

#ifdef CONFIG_INCELL_TOUCH
extern void cyttsp5_display_touch(void *cyttsp5_data);
extern int cyttsp5_display_watchdog(void *cyttsp5_data);
extern int cyttsp5_display_suspend(void *cyttsp5_data, int order);
extern int cyttsp5_display_resume(void *cyttsp5_data, int order);
#endif

#endif