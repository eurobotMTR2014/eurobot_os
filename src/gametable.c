#include "gametable.h"

bool doubleCircleCollision(float x1, float y1, float r1, float x2, float y2, float r2);
bool doubleRectCollision(float x1, float y1, float hw1, float hh1, float x2, float y2, float hw2, float hh2);
bool circleRectCollision(float x1, float y1, float r1, float x2, float y2, float hw2, float hh2);
/*
bool isTableStructureC(float x, float y, float r)
{
    // Table borders
    if (    (x + r) < -TABLE_MARGIN
         || (x - r)  > (MAX_X + TABLE_MARGIN)
         || (y + r) <  -TABLE_MARGIN
         || (y - r) > (MAX_Y + TABLE_MARGIN)
         )
    {
        return true;
    }

    // Cake
    if (doubleCircleCollision(x,y,r, CAKE_C_X,CAKE_C_Y,CAKE_R))
        return true;

    // Gifts
    if (    circleRectCollision(x,y,r, GIFT_X,GIFT_1_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
         || circleRectCollision(x,y,r, GIFT_X,GIFT_2_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
         || circleRectCollision(x,y,r, GIFT_X,GIFT_3_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
         || circleRectCollision(x,y,r, GIFT_X,GIFT_4_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
        )
    {
        return true;
    }

    // Pillars
    if (    circleRectCollision(x,y,r, BLUE_PIL_X,PIL_1_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || circleRectCollision(x,y,r, BLUE_PIL_X,PIL_2_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || circleRectCollision(x,y,r, BLUE_PIL_X,PIL_3_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || circleRectCollision(x,y,r, RED_PIL_X, PIL_1_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || circleRectCollision(x,y,r, RED_PIL_X, PIL_2_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || circleRectCollision(x,y,r, RED_PIL_X, PIL_3_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
        )
    {
        return true;
    }

    return false;
}

bool isTableStructureR(float x, float y, float hw, float hh)
{
    // Table borders
    if (    (x + hw) < -TABLE_MARGIN
         || (x - hw)  > (MAX_X + TABLE_MARGIN)
         || (y + hh) <  -TABLE_MARGIN
         || (y - hh) > (MAX_Y + TABLE_MARGIN)
         )
    {
        return true;
    }

    // Cake
    if (circleRectCollision(CAKE_C_X,CAKE_C_Y,CAKE_R, x,y,hw,hh))
        return true;

    // Gifts
    if (    doubleRectCollision(x,y,hw,hh, GIFT_1_X,GIFT_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, GIFT_2_X,GIFT_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, GIFT_3_X,GIFT_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, GIFT_4_X,GIFT_Y,GIFT_HALF_WIDTH,GIFT_HALF_HEIGHT)
        )
    {
        return true;
    }

    // Pillars
    if (    doubleRectCollision(x,y,hw,hh, BLUE_PIL_X,PIL_1_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, BLUE_PIL_X,PIL_2_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, BLUE_PIL_X,PIL_3_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, RED_PIL_X, PIL_1_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, RED_PIL_X, PIL_2_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
         || doubleRectCollision(x,y,hw,hh, RED_PIL_X, PIL_3_Y,PILAR_HALF_WIDTH,PILAR_HALF_HEIGHT)
        )
    {
        return true;
    }

    return false;
}

bool doubleCircleCollision(float x1, float y1, float r1, float x2, float y2, float r2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float sqdistance = dx*dx + dy*dy;

    float dbRad = r1 + r2;

    if (sqdistance < dbRad * dbRad)
    {
        return true;
    }

    return false;
}

bool doubleRectCollision(float x1, float y1, float hw1, float hh1, float x2, float y2, float hw2, float hh2)
{
    if (y2 - hh2 > y1+ hh1) // 2 en dessous de 1
    	return false;
    if (y2 + hh2 < y1- hh1)  // 2 au dessus de 1
        return false;
    if (x2 - hw2 > x1 + hw1) // 2 a droite de 1
    	return false;
    if (x2 + hw2 < x1 - hw1)  // 2 a gauche de 1
        return false;

    return true;
}

bool circleRectCollision(float x1, float y1, float r1, float x2, float y2, float hw2, float hh2)
{
    float tx1 = x1;
    float ty1 = y1;

    float X0 = x2 - hw2;
    float X1 = x2 + hw2;
    float Y0 = y2 - hh2;
    float Y1 = y2 + hh2;

    if (tx1 < X0)
        tx1 = X0;
    if (tx1 > X1)
        tx1 = X1;

    if (ty1 < Y0)
        ty1 = Y0;
    if (ty1 > Y1)
        ty1 = Y1;

    float dx = tx1-x1;
    float dy = ty1-y1;

    return ( dx*dx + dy*dy < r1*r1 );
}
*/
