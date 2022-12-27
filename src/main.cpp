/**
 * @file main.cpp
 * @brief 倒立振子ロボット制御プログラム
 * @author Jun Nakanishi
 * @date 2020/06/24
 */

#include "NJM4350D.hpp"
#include "BMX055.hpp"
#include "mbed.h"

// ステッピングモータードライバを初期化
NJM4350D *motor1 = new NJM4350D(PB_7, PA_15, PC_3, PC_2);
NJM4350D *motor2 = new NJM4350D(PC_11, PD_2, PC_12, PC_10);
// IMUを初期化
BMX055 *bmx055 = new BMX055(I2C_SDA, I2C_SCL);
// オンボードLED,SWを初期化
DigitalIn Button = USER_BUTTON;
DigitalOut LED = LED1;

// 倒立制御のループを一定周期で実行するために用意
Thread mThread;
EventQueue mEventQueue;

// プロトタイプ宣言
void run(float v, float w);
void balancer();

/**
 * @fn
 * main関数
 * @brief main関数
 */
int main() {
    //ジャイロオフセット取得
    int led = 0;
    while(Button){
        LED.write(led ^= 1);
        ThisThread::sleep_for(100);
    }
    LED.write(0);
    int offset = 0;
    for(int i = 0; i < 100; i++){
        offset -= bmx055->read_Gyro_Y();
        ThisThread::sleep_for(10);
    }
    offset /= 100;
    bmx055->offset_gyro = offset;
    bmx055->offset_angle = 4.0f;
    LED.write(1);
    
    //スタート指示待機
    while(!Button){
        ThisThread::sleep_for(100);
    }
    while(Button){
        ThisThread::sleep_for(10);
    }

    //動作開始
    mThread.start(callback(&mEventQueue, &EventQueue::dispatch_forever));
    mEventQueue.call_every(5, &balancer);
    while(true){
        printf("%d\n", (int)(bmx055->mPitch_Angle * 100.0f));
        ThisThread::sleep_for(10);
  }
}


/**
 * @fn
 * 倒立制御プログラム
 * @brief 倒立状態で指定された速度で走行する
 * @detail 一定周期で呼び出される用に設定する
 */
void balancer(){
        //printf("%d\n", (int)(bmx055->mPitch_Speed * 100));
        static float speed = 0;
        static float d = 0;
        d += speed * 0.005f;
        speed += bmx055->mPitch_Speed * 30.0f * 0.005f;
        speed += bmx055->mPitch_Angle * 400.0f * 0.005f;
        speed += speed * 5.0f * 0.005f;
        //speed += d * 25.0f * 0.005f;

        run(speed, speed * 0.01f);
}


/**
 * @fn
 * 走行
 * @brief 前進速度, 旋回速度から左右のホイールの回転速度を計算し、指示する
 * @param (v) 前進速度(mm/s)
 * @param (w) 旋回速度(rad/s)
 */
void run(float v, float w){
    float dia = 90.0f;  //ホイール直径
    float tread = 124.0f;  //左右のホイール間隔

    motor1->setSpeed((v - tread * 0.5f * w) / (dia * 3.141592654f));
    motor2->setSpeed(-(v + tread * 0.5f * w) / (dia * 3.141592654f));
}
