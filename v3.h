#ifndef _V3_H
#define _V3_H

#include "basicio.h"
#include "v3def.h"


//デバッグ用シリアルコードを残しておいてもコンパイルが通るようにここでダミー宣言
#ifndef USE_SERIAL
#define serial_init(baud)   {}
#define serial_puts(str)    {}
#define serial_getTxCount() 0
#endif

//送信データを保持する配列
//無線送信できないときにバッファリングするもの
typedef struct {
    uint32_t millis;            //登録時のmillis()値
    uint8_t len;                //_V3FORMAT はLQI値格納のため可変長
                                //MINPACKETSIZE~MAXPACKETSIZEの値をとる
    union {
        _V3FORMAT data;
        uint8_t bs[MAXPACKETSIZE];
    };
} V3DATA;
#ifdef TWELITE_BLUE
#define DATAARRAYLENGTH    500
#endif
#ifdef TWELITE_RED
#define DATAARRAYLENGTH    500
#endif

//アプリから参照できるように
extern _EEPROM_SETTING_STRUCT eepData;
extern uint16_t values[3];

//ADCソースビット
typedef enum {
    ADC_SOURCEBIT_1     = 1,
    ADC_SOURCEBIT_2     = 2,
    ADC_SOURCEBIT_3     = 4,
    ADC_SOURCEBIT_4     = 8,
    ADC_SOURCEBIT_TEMP  = 64,
    ADC_SOURCEBIT_VOLT  = 128
} ADCSOURCEBIT;
void setupAdcMeasurement(ADCSOURCEBIT adcSourceBits);
void setupAdcSample(ADCSAMPLES sample);
uint16_t getAdcValue(ADCSOURCEBIT adcSourceBit);

//起動直後(セットアップモードも含む)
void afterWake(bool_t settingMode);

//アプリとしての初期化処理
void setupApp(bool_t bSleepDevice, uint32_t bitmapWakeStatus);

typedef enum {
    DATA_NONE = 0,
    DATA_NOT_READY = 1,
    DATA_READY = 2,
} APPSTATE;

//アプリとしての測定処理
//計測が終了するまで DATA_NOT_READY を返す
//計測が終了してvalues[]に値を入れたら DATA_READY を返す
//一度 DATA_READY を返すと、以降呼ばれなくなる
//計測データを送信しない場合は DATA_READY の代わりに DATA_NONE を返す
APPSTATE procApp(bool_t bFirst, bool_t bSleepDevice, bool_t bAdcDone);

//スリープ直前の処理
//pSleepTimeを変更することでスリープ時間を変えられる
//リブートの場合はpSleepTimeがNULLになるので注意
void beforeSleep(uint32_t *pSleepTime);


typedef enum {
    LEDEVENT_NONE       = 0,
    LEDEVENT_RADIO_RX   = 1,
    LEDEVENT_RADIO_TX   = 2,
    LEDEVENT_HAVE_DATA  = 4,
    LEDEVENT_SAMPLING   = 8
} LEDEVENTTYPE;

typedef enum {
    LOW_TO_ON,
    HIGH_TO_ON
} LEDPINMODE;

extern void led_pinMode(uint8_t pinNo, LEDPINMODE mode, LEDEVENTTYPE event);

#endif