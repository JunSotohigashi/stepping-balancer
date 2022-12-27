/**
 * @file BMX055.cpp
 * @brief BMX055を操作するクラスの実装
 * @author Jun Nakanishi
 * @date 2020/06/24
 */

#include "BMX055.hpp"
#include "mbed.h"


/**
 * @fn
 * コンストラクタ
 * @brief スレーブアドレスの設定、各レジスタの初期化、周期呼び出しの開始を行う
 * @param (pin_SDA) SDAピン
 * @param (pin_SCL) SCLピン
 */
BMX055::BMX055(PinName pin_SDA, PinName pin_SCL)
    : mI2C(pin_SDA, pin_SCL),
      mAddr_Acc(0x19 << 1),
      mAddr_Gyr(0x69 << 1),
      mAddr_Mag(0x13 << 1) {
        init();
        // 5ms周期で呼び出し
        // Tickerで呼び出すとI2Cの関数郡が正常に動作しないので、スレッド・イベントキューを使用する
        mThread.start(callback(&mEventQueue, &EventQueue::dispatch_forever));
        mEventQueue.call_every(5, callback(this, &BMX055::calc_Attitude));
}


/**
 * @fn
 * デストラクタ
 * @brief 周期実行の停止、ピンの開放を行う
 */
BMX055::~BMX055() {
    // TODO:周期実行の停止、ピンの開放
}


/**
 * @fn
 * レジスタ初期化
 * @brief 加速度センサ, ジャイロセンサ, 磁気センサ のレジスタを操作し、サンプリング周期やレンジを設定する
 */
void BMX055::init() {
    int delay = 1;  //データ送信の間隔(ms)
    char buffer[2];

    //加速度センサー
    //-----------------------------------------------------//
    buffer[0] = 0x0F;    //PMU_Range レジスタを選択
    buffer[1] = 0x03;    //レンジを +/- 2g に設定
    mI2C.write(mAddr_Acc, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x10;    //PMU_BW レジスタを選択
    buffer[1] = 0x08;    //帯域幅を 7.18Hz に設定
    mI2C.write(mAddr_Acc, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x11;    //PMU_LPW レジスタを選択
    buffer[1] = 0x00;    //ノーマルモードに設定
    mI2C.write(mAddr_Acc, buffer, 2, false);
    ThisThread::sleep_for(delay);

    //ジャイロセンサー
    //-----------------------------------------------------//
    buffer[0] = 0x0F;    //PMU_Range レジスタを選択
    buffer[1] = 0x03;    //レンジを +/- 250deg/s に設定
    mI2C.write(mAddr_Gyr, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x10;    //PMU_BW レジスタを選択
    buffer[1] = 0b10;    //ODR = 1000Hz に設定
    mI2C.write(mAddr_Gyr, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x11;    //PMU_LPW レジスタを選択
    buffer[1] = 0x00;    //ノーマルモードに設定
    mI2C.write(mAddr_Gyr, buffer, 2, false);
    ThisThread::sleep_for(delay);
    
    //磁気センサー
    //-----------------------------------------------------//
    buffer[0] = 0x4B;    //レジスタを選択
    buffer[1] = 0x83;    //ソフトリセット
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x4B;    //レジスタを選択
    buffer[1] = 0x01;    //ソフトリセット
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x4C;    //レジスタを選択
    buffer[1] = 0x00;    //ODR = 10Hz
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x4E;    //レジスタを選択
    buffer[1] = 0x84;    //X, Y, Z軸を有効化
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x51;    //レジスタを選択
    buffer[1] = 0x04;    //No. of Repetitions for X-Y Axis = 9
    mI2C.write(mAddr_Mag, buffer, 2, false);
    ThisThread::sleep_for(delay);
    //-----------------------------------------------------//
    buffer[0] = 0x52;    //レジスタを選択
    buffer[1] = 0x16;    //No. of Repetitions for Z-Axis = 15
    mI2C.write(mAddr_Mag, buffer, 2, false);
}


/**
 * @fn
 * 加速度センサ X軸 読み取り
 * @brief X軸の加速度を読み取る
 * @return 加速度(m/s^2)
 */
float BMX055::read_Accl_X(){
    char buffer[2];
    buffer[0] = 0x02;    //レジスタアドレスを設定
    mI2C.write(mAddr_Acc, buffer, 1, true);
    mI2C.read(mAddr_Acc, buffer, 2, false);    //2byte読み取り
    int16_t raw = (buffer[1] << 8 | (buffer[0] & 0xF0));
    return (float)raw * 0.0098f / 16.0f;
}


/**
 * @fn
 * 加速度センサ Y軸 読み取り
 * @brief Y軸の加速度を読み取る
 * @return 加速度(m/s^2)
 */
float BMX055::read_Accl_Y(){
    char buffer[2];
    buffer[0] = 0x04;    //レジスタアドレスを設定
    mI2C.write(mAddr_Acc, buffer, 1, true);
    mI2C.read(mAddr_Acc, buffer, 2, false);    //2byte読み取り
    int16_t raw = (buffer[1] << 8 | (buffer[0] & 0xF0));
    return (float)raw * 0.0098f / 16.0f;
}


/**
 * @fn
 * 加速度センサ Z軸 読み取り
 * @brief Z軸の加速度を読み取る
 * @return 加速度(m/s^2)
 */
float BMX055::read_Accl_Z(){
    char buffer[2];
    buffer[0] = 0x06;    //レジスタアドレスを設定
    mI2C.write(mAddr_Acc, buffer, 1, true);
    mI2C.read(mAddr_Acc, buffer, 2, false);    //2byte読み取り
    int16_t raw = (buffer[1] << 8 | (buffer[0] & 0xF0));
    return (float)raw * 0.0098f / 16.0f;
}


/**
 * @fn
 * ジャイロセンサ X軸 読み取り
 * @brief X軸の角速度を読み取る
 * @return 角速度(deg/s)
 */
float BMX055::read_Gyro_X(){
    char buffer[2];
    buffer[0] = 0x02;    //レジスタアドレスを設定
    mI2C.write(mAddr_Gyr, buffer, 1, true);
    mI2C.read(mAddr_Gyr, buffer, 2, false);    //2byte読み取り
    int16_t raw = buffer[1] << 8 | buffer[0];
    return (float)raw * 0.0076f;
}


/**
 * @fn
 * ジャイロセンサ Y軸 読み取り
 * @brief Y軸の角速度を読み取る
 * @return 角速度(deg/s)
 */
float BMX055::read_Gyro_Y(){
    char buffer[2];
    buffer[0] = 0x04;    //レジスタアドレスを設定
    mI2C.write(mAddr_Gyr, buffer, 1, true);
    mI2C.read(mAddr_Gyr, buffer, 2, false);    //2byte読み取り
    int16_t raw = buffer[1] << 8 | buffer[0];
    return (float)raw * 0.0076f;
}


/**
 * @fn
 * ジャイロセンサ Z軸 読み取り
 * @brief Z軸の角速度を読み取る
 * @return 角速度(deg/s)
 */
float BMX055::read_Gyro_Z(){
    char buffer[2];
    buffer[0] = 0x06;    //レジスタアドレスを設定
    mI2C.write(mAddr_Gyr, buffer, 1, true);
    mI2C.read(mAddr_Gyr, buffer, 2, false);    //2byte読み取り
    int16_t raw = buffer[1] << 8 | buffer[0];
    return (float)raw * 0.0076f;
}


/**
 * @fn
 * 姿勢計算
 * @brief 各センサの測定値を統合し、センサの姿勢を推定する
 * @detail 結果はメンバ変数に保存する　現在の実装はピッチ角のみ
 */
void BMX055::calc_Attitude(){
    float dt = 0.005f;  //実行周期(s)
    float accl_x = read_Accl_X();  //一時的に加速度を保存する
    float accl_z = read_Accl_Z();
    float accl_pitch = atan2(accl_x, accl_z) / 3.141592654f * 180.0f;  //加速度センサーからのピッチ角を計算
    float accl_amp = sqrt(accl_x * accl_x + accl_z * accl_z);  //加速度の大きさを計算
    
    float gyro_y = -read_Gyro_Y() - offset_gyro;  //符号を反転

    // 相補フィルタのゲイン
    float k = 0.005f;
    // float k = 0.02f * pow(2.718281828f, (-(accl_amp - 9.8f) * (accl_amp - 9.8f)) / 0.5f);  // 正規分布で重み付けを変化させるとき

    float old_mPitch_Angle = mPitch_Angle_raw;
    float new_mPitch_Angle = old_mPitch_Angle + (k * (accl_pitch - old_mPitch_Angle)) + (1.0f-k) * (gyro_y * dt);

    //メンバ変数に結果を保存
    mPitch_Speed = gyro_y;//(new_mPitch_Angle - old_mPitch_Angle) / dt;
    mPitch_Angle_raw = new_mPitch_Angle;
    mPitch_Angle = new_mPitch_Angle + offset_angle;
}
