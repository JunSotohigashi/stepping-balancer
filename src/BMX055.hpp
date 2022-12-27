/**
 * @file BMX055.hpp
 * @brief BMX055を操作するクラスの定義
 * @author Jun Nakanishi
 * @date 2020/06/24
 */

#ifndef BMX055_HPP_
#define BMX055_HPP_

#include "mbed.h"


/*! @class
 @brief BMX055を操作するクラスの実装
*/
class BMX055 {
    public:
        BMX055(PinName pin_SDA, PinName pin_SCL);
        ~BMX055();
        void init();
        float read_Accl_X();
        float read_Accl_Y();
        float read_Accl_Z();
        float read_Gyro_X();
        float read_Gyro_Y();
        float read_Gyro_Z();
        void calc_Attitude();
        float mPitch_Angle = 0;
        float mPitch_Speed = 0;
        float offset_gyro = 0;
        float offset_angle = 0;

    private:
        I2C mI2C;
        Thread mThread;
        EventQueue mEventQueue;
        int mAddr_Acc = 0x19;
        int mAddr_Gyr = 0x69;
        int mAddr_Mag = 0x13;
        float mPitch_Angle_raw = 0;
};

#endif
