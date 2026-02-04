//---------------------------- Screen parameters -----------------------------
#define SCREEN_WIDTH  200.0    // Set for landscape mode, don't remove the decimal place!
#define SCREEN_HEIGHT 200.0

#define SCREEN_ROTATION 1     // 0: portrait, 1: landscape, 2: portrait inverted, 3: landscape inverted

// Input data screen parameters
#define HEADER_FONT_SIZE        16
#define DATA_FONT_SIZE          16
#define HEADER_Y_POS            0
#define HEADER_X_POS            0
#define HEADER_WIDTH            SCREEN_WIDTH
#define HEADER_HEIGHT           2 * HEADER_FONT_SIZE

#define INPUTDATA_FONT_SIZE     16
#define INPUTDATA_Y_POS         5*INPUTDATA_FONT_SIZE - INPUTDATA_FONT_SIZE+1
#define INPUTDATA_X_POS         140
#define INPUTDATA_WIDTH         SCREEN_WIDTH - INPUTDATA_X_POS-1
#define INPUTDATA_HEIGHT        SCREEN_WIDTH - INPUTDATA_Y_POS-1

#define NUM_INPUTDATA_LINES     8

// Watch screen parameters
#define CIRCLE_RADIUS         60
#define CIRCLE_CENTER_X       SCREEN_WIDTH / 2
#define CIRCLE_CENTER_Y       SCREEN_HEIGHT / 2 + 10




//---------------------------- End Screen parameters --------------------------