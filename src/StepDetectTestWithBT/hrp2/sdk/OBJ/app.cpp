/**
 ******************************************************************************
 ** �t�@�C���� : app.cpp
 **
 ** �T�v : 2�֓|���U�q���C���g���[�X���{�b�g��TOPPERS/HRP2�pC++�T���v���v���O����
 **
 ** ���L : sample_cpp (���C���g���[�X/�K�����[�^/�����g�Z���T/�����[�g�X�^�[�g)
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

/* �W��define */
#define TRUE  1
#define FALSE 0

#define OK 1
#define NG 0

/* ��Q��define */
#define SONER_SOMETHINGTHERE  1

/* 1: Bluetooth��L���ɂ��� */
#define BT_ENABLE 1

/* ���L�̃}�N���͌�/���ɍ��킹�ĕύX����K�v������܂� */
#define GYRO_OFFSET           0  /* �W���C���Z���T�I�t�Z�b�g�l(�p���x0[deg/sec]��) */
#define LIGHT_WHITE          20  /* ���F�̌��Z���T�l */
#define LIGHT_BLACK           0  /* ���F�̌��Z���T�l */
#define SONAR_ALERT_DISTANCE 30  /* �����g�Z���T�ɂ���Q�����m����[cm] */
#define TAIL_ANGLE_STAND_UP  93  /* ���S��~���̊p�x[�x] */
#define TAIL_ANGLE_DRIVE      3  /* �o�����X���s���̊p�x[�x] */
#define P_GAIN             2.5F  /* ���S��~�p���[�^������W�� */
#define PWM_ABS_MAX          60  /* ���S��~�p���[�^����PWM��΍ő�l */
#define CMD_START           '1'  /* �����[�g�X�^�[�g�R�}���h */
#define DETECTSTEP_VEL        5  /* �K�i�ɏՓ˂����Ƃ��Ɍ��m����p���x */

/* LCD�t�H���g�T�C�Y */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)

/* �֐����ϐ� */
#if BT_ENABLE == 1
static FILE     *bt = NULL;      /* Bluetooth�t�@�C���n���h�� */
#endif

/* �֐��v���g�^�C�v�錾 */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);
static void tail_control_slowly(int32_t angle);
static void bt_send(int8_t *p_sendchar, int8_t length);


/* �I�u�W�F�N�g�ւ̃|�C���^��` */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

/* ���C���^�X�N */
void main_task(intptr_t unused)
{
    int8_t forward;      /* �O��i���� */
    int8_t turn;         /* ���񖽗� */
    int8_t pwm_L, pwm_R; /* ���E���[�^PWM�o�� */
    int32_t motor_ang_l, motor_ang_r;
    int32_t volt;
    int16_t gyro;
    int8_t buf[50];    /* �f�B�X�v���[�\���p */

    /* �e�I�u�W�F�N�g�𐶐��E���������� */
    touchSensor = new TouchSensor(PORT_1);
    colorSensor = new ColorSensor(PORT_2);
    sonarSensor = new SonarSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD��ʕ\�� */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET sample_cpp", 0, CALIB_FONT_HEIGHT*1);

#if BT_ENABLE == 1
    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth�ʐM�^�X�N�̋N�� */
    act_tsk(BT_TASK);
#endif


    /* �K�����[�^�[�̃��Z�b�g */
    tailMotor->reset();
    /* �����������ʒm */
    ev3_led_set_color(LED_ORANGE);

    /* �X�^�[�g�ҋ@ */
    while(1)
    {
        /* ���S��~�p�p�x�ɐ��� */
        tail_control(TAIL_ANGLE_STAND_UP);
        /* 10ms wait */
        clock->sleep(10);

        if(touchSensor->isPressed())
        {
            /* �^�b�`�Z���T�������ꂽ */
            break;
        }
    }

    /* �X�^�[�g�ʒm */
    ev3_led_set_color(LED_GREEN);

    /* ���s���[�^�[�G���R�[�_�[���Z�b�g */
    leftMotor->reset();
    rightMotor->reset();

    /* �W���C���Z���T�[���Z�b�g */
    gyroSensor->reset();
    /* �|���U�qAPI������ */
    balance_init();

    while(1)
    {

        /* �����{�^���������ƏI�� */
        if (ev3_button_is_pressed(ENTER_BUTTON)) break;

        /* �o�����X���s�p�p�x�ɐ��� */
        tail_control_slowly(TAIL_ANGLE_DRIVE);
#if 0
        /* ��Q�����m�͈�x�폜 */
        if (sonar_alert() == SONER_SOMETHINGTHERE)
        {
            /* ��Q�������m�������~ */
            forward = 0;
            turn = 0;
        }
        else
#endif
        {
            /* ���݁A�W���C���Z���T�Ɋ|�����Ă���p���x[deg/sec]���擾 */
            gyro = gyroSensor->getAnglerVelocity();
            /* �W���C���Z���T�Ɋ|�����Ă���p���x[deg/sec]��BT�o�R�ő��M */
            sprintf((char *)buf, "gyro_val = %4d\n", gyro);
            /* �����Teraterm�Ƃ�����gyro_val = ���ďo�Ă���񂶂�Ȃ����� */
#if BT_ENABLE == 1
            bt_send(buf, 16);
#endif

            /************ ���C���g���[�X ************/
            forward = 30; /* �O�i���� */
            if (colorSensor->getBrightness() >= (LIGHT_WHITE + LIGHT_BLACK)/2)
            {
                turn =  20; /* �����񖽗� */
            }
            else
            {
                turn = -20; /* �E���񖽗� */
            }
        }

        /* �|���U�q����API �ɓn���p�����[�^���擾���� */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        // gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* �|���U�q����API���Ăяo���A�|�����s���邽�߂� */
        /* ���E���[�^�o�͒l�𓾂� */
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

        clock->sleep(4); /* 4msec�����N�� */
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
// �֐��� : sonar_alert
// ���� : ����
// �Ԃ�l : 1(��Q������)/0(��Q������)
// �T�v : �����g�Z���T�ɂ���Q�����m
//*****************************************************************************
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    static int32_t alert = 0;

    int32_t distance;

    if (++counter == 40/4) /* ��40msec�������ɏ�Q�����m  */
    {
        /*
         * �����g�Z���T�ɂ�鋗����������́A�����g�̌��������Ɉˑ����܂��B
         * NXT�̏ꍇ�́A40msec�������x���o����̍ŒZ��������ł��B
         * EV3�̏ꍇ�́A�v�m�F
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* ��Q�������m */
        }
        else
        {
            alert = 0; /* ��Q������ */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// �֐��� : tail_control
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ���s�̊��S��~�p���[�^�̊p�x����
//*****************************************************************************
static void tail_control(int32_t angle)
{
    float pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* ��ᐧ�� */
    /* PWM�o�͖O�a���� */
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
// �֐��� : tail_control_slowly
// ���� : angle (���[�^�ڕW�p�x[�x])
// �Ԃ�l : ����
// �T�v : ������胂�[�^�̊p�x����
//*****************************************************************************
static void tail_control_slowly(int32_t angle)
{
    float pwm;
    if(tailMotor->getCount() > 10)
    {
        /* �p�x��10�x�ȏ㗣��Ă���Ƃ��́A��������葬�x�Ŗ߂� */
        pwm = 15.0F;
    }
    else
    {
        pwm = (float)(angle - tailMotor->getCount()) * P_GAIN; /* ��ᐧ�� */
    }
    /* PWM�o�͖O�a���� */
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
// �֐��� : bt_task
// ���� : unused
// �Ԃ�l : �Ȃ�
// �T�v : Bluetooth�ʐM�ɂ�郊���[�g�X�^�[�g�B Tera Term�Ȃǂ̃^�[�~�i���\�t�g����A
//       ASCII�R�[�h��1�𑗐M����ƁA�����[�g�X�^�[�g����B
//*****************************************************************************
void bt_task(intptr_t unused)
{
    /* �����Ȃ� */
}

//*****************************************************************************
// �֐��� : bt_send
// ���� : ������|�C���^, �|�C���^��
// �Ԃ�l : ����������ԂɃ|�C���^�������M���܂�
// �T�v :
//*****************************************************************************
static void bt_send(int8_t *p_sendchar, int8_t length)
{
#if BT_ENABLE == 1
    /* ToDo: ����length�́Auint8_t�ɕύX */

    int8_t uc_cnt_sended;   /* ���M���̕����̏ꏊ */
    int8_t uc_ret = OK;  /* ���M�G���[���� */

    uc_cnt_sended = 0;
    if(length <= 0)
    {
        /* ������O */
        uc_ret = NG;
    }

    if(uc_ret == OK)
    {
        while(1)
        {
            if((uc_cnt_sended + 1) > length)
            {
                /* ���M���� */
                break;
            }
            fputc(p_sendchar[uc_cnt_sended], bt);
            uc_cnt_sended++;
        }
    }
#endif
}


