#include "definitions.h"

#ifndef GAMETABLE_H
#define	GAMETABLE_H

#define degToRad(x) x*(PI/180.0)

#define RED 1
#define BLUE 0

#define MAX_X 2000        // Borders
#define MAX_Y 3000
#define TABLE_MARGIN 350

#define INIT_1_X 610
#define INIT_1_Y 220

#define INIT_2_X 1000
#define INIT_2_Y 220

#define CAKE_C_X 2000         // Cake
#define CAKE_C_Y 1500
#define CAKE_R 500
#define CAKE_ZONE_R CAKE_R + 530

#define JUICE_1_X 1050
#define JUICE_2_X 1050
#define JUICE_3_X 800
#define JUICE_4_X 550
#define JUICE_5_X 550
#define JUICE_6_X 1050
#define JUICE_1_Y 900
#define JUICE_2_Y 1200
#define JUICE_3_Y 1350
#define JUICE_4_Y 1200
#define JUICE_5_Y 900
#define JUICE_6_Y 1050
#define JUICE_DUMP_PREC_Y 400
#define JUICE_DUMP_Y 130
#define JUICE_REVERSE_Y 550


#define GIFT_X 0               // Gifts general
#define GIFT_HALF_HEIGHT 50
#define GIFT_HALF_WIDTH 187

#define GIFT_SECURE_FLAP_X 450
#define GIFT_SECURE_FLAP_Y 535
#define GIFT_PREC_X 350
#define GIFT_PREC_Y 500

#define GIFT_1_Y 600             // Gifts particular
#define GIFT_2_Y 1200
#define GIFT_3_Y 1800
#define GIFT_4_Y 2400
#define GIFT_Y_DECAL 80
#define GIFT_Y_DECAL_UP_1 130
#define GIFT_Y_DECAL_DOWN_1 70
#define GIFT_Y_DECAL_UP_2 27
#define GIFT_Y_DECAL_DOWN_2 87
#define GIFT_X_DECAL 360
#define GIFTS_ZONE_X 400

#define GIFT_34_1_X_OFF -10

#define CANDLE_PREC_1_1_X 1390
#define CANDLE_PREC_2_1_X 1460
#define CANDLE_PREC_1_1_Y 2270
#define CANDLE_PREC_2_1_Y 2070

#define CANDLE_PREC_1_2_X 1390
//#define CANDLE_PREC_1_2_X 2050
#define CANDLE_PREC_2_2_X 1460
#define CANDLE_PREC_1_2_Y 730
#define CANDLE_PREC_2_2_Y 930

#define CANDLE_1_1_X  1310
#define CANDLE_2_1_X  1250
#define CANDLE_3_1_X  1270
#define CANDLE_4_1_X  1320
#define CANDLE_1_1_Y  1220
#define CANDLE_2_1_Y  1440
#define CANDLE_3_1_Y  1620
#define CANDLE_4_1_Y  1845

#define CANDLE_1_2_X  1280
#define CANDLE_2_2_X  1215
#define CANDLE_3_2_X  1180
#define CANDLE_4_2_X  1230
#define CANDLE_1_2_Y  1220
#define CANDLE_2_2_Y  1305
#define CANDLE_3_2_Y  1620
#define CANDLE_4_2_Y  1845

#define C2_OFF_X -20
#define C3_OFF_X 15
#define C4_OFF_X 0
#define C2_OFF_Y -10
#define C3_OFF_Y 10
#define C4_OFF_Y -10

#define C1_OFF_1_X 50
#define C2_OFF_1_X 30
#define C3_OFF_1_X 40
#define C4_OFF_1_X 15
#define C1_OFF_1_Y -75
#define C2_OFF_1_Y -38
#define C3_OFF_1_Y 0
#define C4_OFF_1_Y -25


/*#define CANDLE_1_2_X  1280
#define CANDLE_2_2_X  1170
#define CANDLE_3_2_X  1180
#define CANDLE_4_2_X  1230
#define CANDLE_1_2_Y  1220
#define CANDLE_2_2_Y  1440
#define CANDLE_3_2_Y  1620
#define CANDLE_4_2_Y  1845*/

/*
#define CANDLE_1_2_X  1280
#define CANDLE_2_2_X  1220
#define CANDLE_3_2_X  1210
#define CANDLE_4_2_X  1260
#define CANDLE_1_2_Y  1220
#define CANDLE_2_2_Y  1440
#define CANDLE_3_2_Y  1620
#define CANDLE_4_2_Y  1845*/

#define CANDLE_1_1_TH degToRad(-95)
#define CANDLE_2_1_TH degToRad(-90)
#define CANDLE_3_1_TH degToRad(-80)
#define CANDLE_4_1_TH degToRad(-75)

#define CANDLE_1_2_TH degToRad(105)
#define CANDLE_2_2_TH degToRad(95)
#define CANDLE_3_2_TH degToRad(85)
#define CANDLE_4_2_TH degToRad(75)
#define CANDLE_INTERM_X 1240

#define BLUE_PIL_Y -62           // Pillars
#define RED_PIL_Y 3062
#define PIL_1_X -62
#define PIL_2_X 1000
#define PIL_3_X 2062

#define PILAR_HALF_HEIGHT 40
#define PILAR_HALF_WIDTH 40


bool isTableStructureC(float x, float y, float r);
bool isTableStructureR(float x, float y, float hw, float hh);

#endif
