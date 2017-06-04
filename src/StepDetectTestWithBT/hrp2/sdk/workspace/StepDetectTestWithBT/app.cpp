/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"

using namespace ev3api;

/* 標準define */
#define TRUE  1
#define FALSE 0

#define OK 1
#define NG 0

/* 障害物define */
#define SONER_SOMETHINGTHERE  1

/* 1: Bluetoothを有効にする */
#define BT_ENABLE 1

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define LIGHT_WHITE          20  /* 白色の光センサ値 */
#define LIGHT_BLACK           0  /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  93  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          60  /* 完全停止用モータ制御PWM絶対最大値 */
#define CMD_START           '1'  /* リモートスタートコマンド */
#define DETECTSTEP_VEL        5  /* 階段に衝突したときに検知する角速度 */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* 関数内変数 */
#if BT_ENABLE == 1
static FILE     *bt = NULL;      /* Bluetoothファイルハンドル */
#endif

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);
static void tail_control_slowly(int32_t angle);
static void bt_send(int8_t *p_sendchar, int8_t length);


/* オブジェクトへのポインタ定義 */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

/* メインタスク */
void main_task(intptr_t unused)
{
    int8_t forward;      /* 前後進命令 */
    int8_t turn;         /* 旋回命令 */
    int8_t pwm_L, pwm_R; /* 左右モータPWM出力 */
    int32_t motor_ang_l, motor_ang_r;
    int32_t volt;
    int16_t gyro;
    int8_t buf[50];    /* ディスプレー表示用 */

    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_2);
    sonarSensor = new SonarSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);

#if BT_ENABLE == 1
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);
#endif


    /* 尻尾モーターのリセット */
    tailMotor->reset();
    /* 初期化完了通知 */
    ev3_led_set_color(LED_ORANGE);

    /* スタート待機 */
    while(1)
    {
        /* 完全停止用角度に制御 */
        tail_control(TAIL_ANGLE_STAND_UP);
        /* 10ms wait */
        clock->sleep(10);

        if(touchSensor->isPressed())
        {
            /* タッチセンサが押された */
            break;
        }
    }

    /* スタート通知 */
    ev3_led_set_color(LED_GREEN);

    /* 走行モーターエンコーダーリセット */
    leftMotor->reset();
    rightMotor->reset();

    /* ジャイロセンサーリセット */
    gyroSensor->reset();
    /* 倒立振子API初期化 */
    balance_init();

    while(1)
    {

        /* 中央ボタンを押すと終了 */
        if (ev3_button_is_pressed(ENTER_BUTTON)) break;

        /* バランス走行用角度に制御 */
        tail_control_slowly(TAIL_ANGLE_DRIVE);
#if 0
        /* 障害物検知は一度削除 */
        if (sonar_alert() == SONER_SOMETHINGTHERE)
        {
            /* 障害物を検知したら停止 */
            forward = 0;
            turn = 0;
        }
        else
#endif
        {
            /* 現在、ジャイロセンサに掛かっている角速度[deg/sec]を取得 */
            gyro = gyroSensor->getAnglerVelocity();
            /* ジャイロセンサに掛かっている角速度[deg/sec]をBT経由で送信 */
            sprintf((char *)buf, "gyro_val = %4d\n", gyro);
            /* これでTeratermとかだとgyro_val = って出てくるんじゃないかな */
#if BT_ENABLE == 1
            bt_send(buf, 16);
#endif

            /************ ライントレース ************/
            forward = 30; /* 前進命令 */
            if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2)
            {
                turn =  20; /* 左旋回命令 */
            }
            else
            {
                turn = -20; /* 右旋回命令 */
            }
        }

        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        // gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための */
        /* 左右モータ出力値を得る */
        balance_control(
            (float)forward,
            (float)turn,
            (float)gyro,
            (float)GYRO_OFFSET,
            (float)motor_ang_l,
            (float)motor_ang_r,
            (float)volt,
            (int8_t *)&pwm_L,
            (int8_t *)&pwm_R);

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

        clock->sleep(4); /* 4msec周期起動 */
    }
    leftMotor->reset();
    rightMotor->reset();

#if BT_ENABLE == 1
    ter_tsk(BT_TASK);
    fclose(bt);
#endif

    ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    static int32_t alert = 0;

    int32_t distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(int32_t angle)
{
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM((int)pwm);
}

//*****************************************************************************
// 関数名 : tail_control_slowly
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : ゆっくりモータの角度制御
//*****************************************************************************
static void tail_control_slowly(int32_t angle)
{
    float pwm;
    if(tailMotor->getCount() > 10)
    {
        /* 角度が10度以上離れているときは、ゆっくり一定速度で戻す */
        pwm = 15.0F;
    }
    else
    {
        pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* 比例制御 */
    }
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM((int)pwm);
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    /* 処理なし */
}

//*****************************************************************************
// 関数名 : bt_send
// 引数 : 文字列ポインタ, ポインタ長
// 返り値 : 文字列を順番にポインタ長分送信します
// 概要 :
//*****************************************************************************
static void bt_send(int8_t *p_sendchar, int8_t length)
{
#if BT_ENABLE == 1
    /* ToDo: 引数lengthは、uint8_tに変更 */

    int8_t uc_cnt_sended;   /* 送信中の文字の場所 */
    int8_t uc_ret = OK;  /* 送信エラー判別 */

    uc_cnt_sended = 0;
    if(length <= 0)
    {
        /* 引数例外 */
        uc_ret = NG;
    }

    if(uc_ret == OK)
    {
        while(1)
        {
            if((uc_cnt_sended + 1) > length)
            {
                /* 送信完了 */
                break;
            }
            fputc(p_sendchar[uc_cnt_sended], bt);
            uc_cnt_sended++;
        }
    }
#endif
}


