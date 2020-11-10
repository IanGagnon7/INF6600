#ifndef __GEN_MAP_H__
#define __GEN_MAP_H__

#include "farm_map.h"

#define SIMU_ACCEL 1 //mettre à un pour avoir tout les comportements normaux. Ne pas dépasser 4.

#define IMG_W_PX x_farm_map_bmp
#define IMG_H_PX y_farm_map_bmp

#define PX_TO_M 1

#define IMG_W_M (PX_TO_M * IMG_W_PX)
#define IMG_H_M (PX_TO_M * IMG_H_PX)

#define STATION_X 275
#define STATION_Y 205

#define BLANK_RGB {0,0,0}

typedef struct
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

typedef struct
{
    double x;
    double y;
} coord_t;


class PathMap {
    public:
        PathMap();
        ~PathMap();

        void takePhoto(coord_t position);
        uint32_t transmitPhotos();


    private:
        rgb_t **img;

        uint32_t dumpId = 0;

        void setPx(int32_t coordX, int32_t coordY);
		void drawDisc(int32_t coordX, int32_t coordY);
		void dumpImage(const char* file_name);
};

#endif
