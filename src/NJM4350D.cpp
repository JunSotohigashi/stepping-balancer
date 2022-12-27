/**
 * @file NJM4350D.cpp
 * @brief NJM4350Dを操作するクラスの実装
 * @author Jun Nakanishi
 * @date 2020/06/24
 */

#include "NJM4350D.hpp"
#include "mbed.h"


/**
 * @fn
 * コンストラクタ
 * @brief ピンの初期化、励磁モードの設定、周期呼び出しの開始を行う
 * @param (pin_enable) NJM4350D INH(11)
 * @param (pin_mode) NJM4350D HSM(10)
 * @param (pin_dir) NJM4350D DIR(6)
 * @param (pin_step) NJM4350D STEP(7)
 */
NJM4350D::NJM4350D(PinName pin_enable, PinName pin_mode, PinName pin_dir, PinName pin_step)
    : mPin_enable(pin_enable),
      mPin_mode(pin_mode),
      mPin_dir(pin_dir),
      mPin_step(pin_step) {
    mPin_enable.write(0); //出力を有効化するよう指示
    mPin_mode.write(0);   //ハーフステップモードに設定
    // 25us毎にupdate関数が呼び出されるよう設定
    mTicker.attach_us(callback(this, &NJM4350D::update), 25);
}


/**
 * @fn
 * デストラクタ
 * @brief 周期実行の停止、出力停止
 */
NJM4350D::~NJM4350D() {
    // update関数の呼び出しを終了
    mTicker.detach();
    //モーター出力を無効化し、シャフトのロックを解除
    mPin_enable = 1;
}


/**
 * @fn
 * ピン状態更新
 * @brief 25us周期で実行される　カウンタの数値と比較し、一定周期でパルスを生成する
 */
void NJM4350D::update() {
    float cmp = abs(rps);
    //目標速度がとても小さいときは0として処理
    if (cmp > 59E-9f) {
        count++;
        if (count > 100.0f / cmp) {
        count = 0;
        if (rps >= 0) {
            one_pulse(0);
            angle_raw++;
        } else {
            one_pulse(1);
            angle_raw--;
        }
        angle = angle_raw * 0.0025f;  //1rev = 400step * 0.0025deg
        }
    }
}


/**
 * @fn
 * 速度設定
 * @brief 目標速度を設定する
 * @param (speed_rps) 目標速度(rps)
 * @retuen 現在の設定速度(rps)
 */
float NJM4350D::setSpeed(float speed_rps) {
    rps = speed_rps;
    return rps;
}


/**
 * @fn
 * 速度取得
 * @brief 目標速度を取得する
 * @retuen 現在の設定速度(rps)
 */
float NJM4350D::getSpeed() {
    return rps;
}


/**
 * @fn
 * 角度取得
 * @brief 現在角度を取得する
 * @retuen 現在の角度(rev)
 */
float NJM4350D::getAngle() {
    return angle;
}


/**
 * @fn
 * パルス生成
 * @brief パルスを生成し、NJM4350Dにステップ送りを指示する
 * @param (dir) 回転方向
 */
void NJM4350D::one_pulse(int dir) {
    mPin_dir.write(dir);
    //ダウンエッジのパルスを生成
    mPin_step.write(0);
    mPin_step.write(1);
}
