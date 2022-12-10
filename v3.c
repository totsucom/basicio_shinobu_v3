/*
 * フォーマットバージョン３
 */
#include "v3.h"

/*
2022/12以前
    V3DTV_DATA受信において、
    ・cbId<128の制限を撤廃。転送モードは128以上のIDを使用するため
    ・データサイズがMINPACKETSIZEのとき、送信者アドレスと測定者アドレスが同じ
    　という条件を撤廃。UART通信の場合はLQIが追加されないため、上記条件が満た
    　されないパターンが発生する
2022/12/10
    EEPROM更新のLQI制限を200から150に下げた。使い勝手悪いので
*/

#ifndef USE_SERIAL
    bool_t serial_printf(const char *fmt, ...) { return TRUE; }
#endif


bool_t settingMode;                 //設定モードかどうか

uint16_t values[3];                 //ユーザーアプリ側で計測された値を格納

int16_t cntdwn_for_gettime;         //次回の時刻要求[sec]
int16_t cntdwn_for_send;            //次回のデータ送信[sec]

int16_t cntdwn_for_measAdc;         //次回のchip_volt/tempｗ測定するタイミング
                                    //スリープデバイス用

int16_t cntdwn_for_getdata;         //次回のデータ取得[sec] UARTマスター用

uint32_t calib_value;               //スリープで使用するRCオシレーターの校正値

#define ADC_VOLTTEMP_INTERVAL   21600

//最低限の関数宣言
void led_event(LEDEVENTTYPE event);


/***********************************************************
 * ステータス
 ***********************************************************/
_STATE_STRUCT state;
//uint32_t recently_success_bits;     //直近32回の送信結果を記憶(成功でビットが立つ)
//uint32_t lost_packets_count;        //バッファフルにより失われたパケットの数
//uint32_t unknown_packets_count;     //V3DTV_DATAの不明パケット受信数
//int32_t timeDataLifeSeconds;                //変数timeDataの残り有効期限[秒]
//uint16_t data_count;                    //保持しているデータ数


/***********************************************************
 * 時刻関連
 * 時刻を保持してカウントするのはスリープしないデバイスのみ
 ***********************************************************/
volatile bool_t timeDataAvailable;          //変数timeDataは有効か
_TIME_VALUE_STRUCT timeData;                //現在時刻
volatile bool_t lock_time_for_i2c;

volatile bool_t timeDataFromXXXAvailable;   //変数timeDataFromXXXは有効か
_TIME_VALUE_STRUCT timeDataFromXXX;         //I2C受信データ

//月の最終日を計算する
uint8_t lastDayOfMonth(uint8_t y, uint8_t m) {
    if (m == 4 || m == 6 || m == 9 || m == 11) return 30;
    if (m == 2) return ((y & 3) == 0) ? 29 : 28;
    return 31;  
}

//構造体_TIME_VALUE_STRUCTの値が正しいかチェックする
bool_t verify_TIME_VALUE_STRUCT(_TIME_VALUE_STRUCT *q) {
    return (
        /*q->year < 50 &&
        q->month >= 1 && q->month <= 12 &&
        q->day >= 1 && q->day <= lastDayOfMonth(q->year, q->month) &&
        q->hour < 24 &&
        q->minute < 60 &&        
        q->second < 60 &&*/
        (q->time & 0x80000000) == 0 &&
        q->msec < 1000
    );
}

//変数timeData の時間を進める
void updateTime(uint16_t tick_ms) {
    lock_time_for_i2c = TRUE;   //lock for interrupt
    if (timeDataAvailable) {
        timeData.msec += tick_ms;
        if (timeData.msec >= 1000) {
            timeData.msec -= 1000;
            ++timeData.time;
        }
        /*
        do {
            timeData.msec += tick_ms;
            if (timeData.msec < 1000) break;
            timeData.msec -= 1000;
            ++timeData.second;
            if (timeData.second < 60) break;
            timeData.second -= 60;
            ++timeData.minute;
            if (timeData.minute < 60) break;
            timeData.minute -= 60;
            ++timeData.hour;
            if (timeData.hour < 24) break;
            timeData.hour -= 24;
            ++timeData.day;
            if (timeData.day <= lastDayOfMonth(timeData.year, timeData.month)) break;
            timeData.day = 1;
            ++timeData.month;
            if (timeData.month < 13) break;
            timeData.month = 1;
            ++timeData.year;
        } while (0);*/
    }
    lock_time_for_i2c = FALSE;
}

//_TIME_VALUE_STRUCT型を_V3FORMAT.pass_time型に変換する
//b0-5      秒 6bit
//b6-11     分 6bit
//b12-b16   時 5bit
//b17-b21   日 5bit
//b22-b25   月 4bit
//b26-b30   年 5bit (2022年を0とする)
//b31       1固定
/*uint32_t _TIME_VALUE_STRUCT_to_u32(_TIME_VALUE_STRUCT *p) {
    return (p->second & 0x3f) |
        ((p->minute & 0x3f) << 6) |
        ((p->hour & 0x1f) << 12) |
        ((p->day & 0x1f) << 17) |
        ((p->month & 0x0f) << 22) |
        (((p->year - 22) & 0x1f) << 26) |
        0x80000000;
}*/


/***********************************************************
 * 設定(EEPROM)関連
 ***********************************************************/
bool_t eepDataAvailable;            //変数eepDataは有効か
_EEPROM_SETTING_STRUCT eepData;     //EEPROMに保存されているデータのコピーになる
                                    //コールドウェイクで読み出され保持される

//スリープする
#define SLEEPDEV()      (eepData.modeBits & EMB_SLEEP)

//別のデバイスから測定データを受信する
#define RELAYDEV()      (eepData.modeBits & EMB_RELAY)

//保持しているデータを無線ではなくI2Cスレーブとして相手に渡す
#define I2CSLAVE()      (eepData.modeBits & EMB_I2CSLAVE)

//保持しているデータを無線ではなくシリアルで相手に渡す
#define UARTSLAVE()     (eepData.modeBits & EMB_UARTSLAVE)

//データの受信は無線ではなくシリアルで受け取る
#define UARTMASTER()    (eepData.modeBits & EMB_UARTMASTER)

#define UARTMODE()      (eepData.modeBits & (EMB_UARTMASTER | EMB_UARTSLAVE))



//構造体_EEPROM_SETTING_STRUCTの値が正しいかチェックする
bool_t verify_EEPROM_SETTING_STRUCT(_EEPROM_SETTING_STRUCT *p) {

    //EMB_SLEEP, EMB_I2CSLAVE, EMB_UARTSLAVE, EMB_UARTMASTER は
    //０(=EMB_NOSLEEP)または いずれか１つのみ選択できる
    uint8_t b = p->modeBits & (EMB_SLEEP | EMB_I2CSLAVE | EMB_UARTSLAVE | EMB_UARTMASTER);
    int8_t i,bitCount;
    for (i=0, bitCount=0; i<8; ++i) {
        if (b & 1) ++bitCount;
        b >>= 1;
    }
    if (bitCount > 1) return FALSE;

    //EMB_SLEEP以外(=EMB_NOSLEEP)の場合はEMB_RELAYオプションが選択できる
    if ((p->modeBits & EMB_SLEEP) != 0 && (p->modeBits & EMB_RELAY) != 0)
        return FALSE;

    //転送アドレスの範囲は0～0xfff
    for (i=0; i<4; ++i) {
        if (p->transfer[i].fromAddress > 0xfff ||
            p->transfer[i].toAddress > 0xfff)
            return FALSE;
    }

    return (
        p->moduleAddress == getModuleAddress() &&   //このデバイスに設定されたものか
        p->version == EEPDATAVERSION &&             //データバージョンチェック
        p->myAddress <= 0xfff &&                    //アドレスの範囲は0～0xfff
        p->targetAddress <= 0xfff &&                //アドレスの範囲は0～0xfff
        (p->sleepTime >= MINIMUM_SLEEP_TIME ||                      //スリープ時間はMINIMUM_SLEEP_TIME以上
         (p->sleepTime == 0 && (p->modeBits & EMB_SLEEP) == 0)) &&  //スリープしないデバイスなら0(測定処理なし)を設定できる
        (p->modeBits & EMB_MASK) == p->modeBits                     //余計なビットが設定されていない
    );
}

void saveEEPROMSetting(_EEPROM_SETTING_STRUCT *p) {
    if (&eepData != p)
        memcpy(&eepData, p, sizeof(_EEPROM_SETTING_STRUCT));
    eepData.data_type = V3DTV_EEPDATA;

    //eeprom_write()はiAHI_WriteDataIntoEEPROMsegment()のエイリアス
    iAHI_WriteDataIntoEEPROMsegment(1, 0, (uint8_t *)&eepData,
        sizeof(_EEPROM_SETTING_STRUCT));
}

bool_t loadEEPROMSetting() {
    eepDataAvailable = eeprom_read(1, 0, (uint8_t *)&eepData, sizeof(eepData));
    if (eepDataAvailable) {
        eepDataAvailable = verify_EEPROM_SETTING_STRUCT(&eepData);
        //serial_puts("EEP READ OK, DATA ");
        //serial_puts(eepDataAvailable ? "OK\r\n" : "NG\r\n");
    } else {
        //serial_puts("EEP READ NG\r\n");
    }
    return eepDataAvailable;
}



/***********************************************************
 * 内蔵ADC関連
 * バージョン３からTWELITEチップの電源電圧及び温度を内蔵ADCで測定
 ***********************************************************/
//内蔵ADC測定ステップ
typedef enum {
    ADCS_STANDBY    = 0,
    ADCS_MEASURING  = 1,
    ADCS_GOT_VALUE  = 2,
    ADCS_COMPLETED  = 3
} ADCMEASSTEP;

//ADCSOURCEBITに対応するADCソース配列
ADCSOURCES adcSourceArray[] = {
    ADC_SOURCE_1,
    ADC_SOURCE_2,
    ADC_SOURCE_3, //DIO0
    ADC_SOURCE_4, //DIO1
    0,
    0,
    ADC_SOURCE_TEMP,
    ADC_SOURCE_VOLT
};

bool_t adcTempVoltForSend;  //基本的に6時間毎にTRUEになる
uint8_t adcMeasurementBits; //実際に測定するビット。adcTempVoltForSendとadcExtraMeasurementの合成値

uint8_t adcStep;      //ADCMEASSTEP
uint8_t adcExtraMeasurement;    //ユーザーが指定した追加のADC測定 ADCSOURCEBITの組み合わせ
ADCSAMPLES adcSample;

uint16_t adcExtra[8];
bool_t adcExtraDone;

//内蔵ADCで測定する場合にsetupApp()内から呼び出す
void setupAdcMeasurement(ADCSOURCEBIT adcSourceBits) {
    adcExtraMeasurement = adcSourceBits;
}

void setupAdcSample(ADCSAMPLES sample) {
    adcSample = sample;
}

uint16_t getAdcValue(ADCSOURCEBIT adcSourceBit) {
    uint8_t i;
    for (i = 0; i < 8; ++i) {
        if (adcSourceBit & 1)
            return adcExtra[i];
        adcSourceBit >>= 1;
    }
    return 0;
}

void adcCallback(uint16_t value) {
    uint8_t i, bit = 1;
    for (i = 0; i < 8; ++i) {
        if (adcMeasurementBits & bit) {
            adcMeasurementBits -= bit;
            adcExtra[i] = value;
            adcStep = ADCS_GOT_VALUE;
            break;
        }
        bit <<= 1;
    }
}

//adcMeasurementBitsに合わせて内蔵ADCを開始する
void beginAdc() {
    memset(&adcExtra, 0, sizeof(adcExtra));
    if (adcMeasurementBits == 0) {
        adcStep = ADCS_COMPLETED;
    } else {
        adc_enable(adcSample, ADC_CLOCK_500KHZ, FALSE);
        uint8_t i, bit = 1;
        for (i = 0; i < 8; ++i) {
            if (adcMeasurementBits & bit) {
                adc_attachCallback(FALSE, TRUE, adcSourceArray[i], adcCallback);
                adcStep = ADCS_MEASURING;
                break;
            }
            bit <<= 1;
        }
    }
}

//loop()から呼び出す
void procAdc() {
    if (adcStep != ADCS_GOT_VALUE) return;
    adc_detach();
    if (adcMeasurementBits == 0) {
        adc_disable();
        adcStep = ADCS_COMPLETED;
        adcExtraDone = TRUE;
    } else {
        uint8_t i, bit = 1;
        for (i = 0; i < 8; ++i) {
            if (adcMeasurementBits & bit) {
                adc_attachCallback(FALSE, TRUE, adcSourceArray[i], adcCallback);
                adcStep = ADCS_MEASURING;
                break;
            }
            bit <<= 1;
        }
    }
}


/***********************************************************
 * 計測データと送信データ配列関連
 ***********************************************************/
V3DATA ar_data[DATAARRAYLENGTH];        //送信までデータを保持する配列
int16_t write_data_index;               //次に書き込む配列のインデックス
int16_t read_data_index;                //次に読み出す配列のインデックス(初期値:-1)
int16_t sending_index;                  //送信中のデータインデックス(未送信時:-1)
volatile bool_t lock_data_for_i2c;

//構造体_V3FORMATの値が正しいかチェックする
bool_t verify_V3FORMAT(_V3FORMAT *p) {
    return (
        p->data_type == V3DTV_DATA &&
        p->pass_time != 0xffffffff &&
        p->address <= 0xfff
    );
}

//送信データ配列のpass_timeを進める、または時刻を設定する
void updateDataArray() {
    uint16_t c = state.data_count;
    int16_t i = read_data_index;
    uint32_t now = millis();
    while (c--) {
        V3DATA *p = &ar_data[i];
        if ((p->data.pass_time & 0x80000000) == 0) {
            //pの示すデータは時刻ではなく経過時間を持っている
            //経過時間を進める
            p->data.pass_time += (now - p->millis);
            p->millis = now;
            if (p->data.pass_time & 0x80000000)
                //pass_timeがオーバーフローした
                p->data.pass_time = 0xffffffff; //無効化
            else if (timeDataAvailable)
                //時刻を持っているので置き換える
                p->data.pass_time =
                    (timeData.time - (p->data.pass_time + 500) / 1000) |
                    0x80000000;
        }
        if (++i == DATAARRAYLENGTH) i = 0;
    }
}

//送信データ配列のpass_timeをsleptTimeだけ進める
//また、配列内で保持しているmillis値を０に戻す
//スリープデバイスのウェイク後に使用
void addSleptTimeToDataArray(uint32_t sleptTime) {
    uint16_t c = state.data_count;
    int16_t i = read_data_index;
    while (c--) {
        V3DATA *p = &ar_data[i];
        if ((p->data.pass_time & 0x80000000) == 0) {
            p->data.pass_time += sleptTime;
            p->millis = 0;
            if (p->data.pass_time & 0x80000000)
                //pass_timeがオーバーフローした
                p->data.pass_time = 0xffffffff; //無効化
        }
        if (++i == DATAARRAYLENGTH) i = 0;
    }
}

//updateDataArray()の単発
void updateData(int16_t i) {
    if (i < 0 || i >= DATAARRAYLENGTH) return;
    V3DATA *p = &ar_data[i];
    if ((p->data.pass_time & 0x80000000) != 0) return;
    uint32_t now = millis();
    p->data.pass_time += now - p->millis;
    p->millis = now;
    if (p->data.pass_time & 0x80000000)
        //pass_timeがオーバーフローした
        p->data.pass_time = 0xffffffff; //無効化
}

//次に送信するデータのpass_timeを進める
//updateDataArray()は配列全体を更新するのに対し、この関数は１つの要素に対してのみ更新する
/*bool_t updateSendData() {
    if (data_count == 0 || read_data_index < 0)
        return FALSE;
    V3DATA *p = &ar_data[read_data_index];
    if (p->data.pass_time == 0xffffffff)
        return FALSE;
    if ((p->data.pass_time & 0x80000000) == 0) {
        uint32_t now = millis();
        p->data.pass_time += (now - p->millis);
        p->millis = now;
    }
    return TRUE;
}*/

void setCbId();

//受信した計測データを送信データ配列に追加する
bool_t appendToDataArray(_V3FORMAT *pData, uint8_t len) {

    //データ転送
    //_V3FORMAT *p = (_V3FORMAT *)pu8Data;
    uint8_t i, transfered = 0;
    for (i = 0; i < 4; i++) {
        if (pData->address == eepData.transfer[i].fromAddress) {
            //データ転送のcbIdは一般idを使用する(再送しない)
            setCbId();
            radio_write(eepData.transfer[i].toAddress, DT_V3, (uint8_t *)pData, len);
            transfered = 1;
        }
    }
    if (transfered) led_event(LEDEVENT_RADIO_TX);

    if (sending_index == write_data_index) {
        //バッファがフルで無線送信中にこの関数が呼ばれるとデータは消失(稀)
        ++state.lost_packets_count;
        return FALSE;
    }

    lock_data_for_i2c = TRUE;   //lock for interrupt

    V3DATA *p = &ar_data[write_data_index];
    p->millis = millis();
    p->len = len;
    memcpy(&p->data, pData, len);

    if ((p->data.pass_time & 0x80000000) == 0 && timeDataAvailable) {
        //経過時間を時刻に書き換え
        //p->data.pass_time = timeData.time | 0x80000000;
        p->data.pass_time =
            (timeData.time - (p->data.pass_time + 500) / 1000) |
            0x80000000;
    }

    if (state.data_count == 0) {
        read_data_index = write_data_index;
        ++state.data_count;
    } else if (state.data_count == DATAARRAYLENGTH) {
        ++state.lost_packets_count;
        if (++read_data_index == DATAARRAYLENGTH) read_data_index = 0;
    } else {
        ++state.data_count;
    }
    if (++write_data_index == DATAARRAYLENGTH) write_data_index = 0;

    lock_data_for_i2c = FALSE;
    led_event(LEDEVENT_HAVE_DATA);

    //すぐに送信を試みる
    cntdwn_for_send = transfered ? 2/*転送したときは気持ち待つ*/ : 1;
    return TRUE;
}

//自身で計測したデータを送信データ配列に登録する
bool_t regToDataArray() {

    _V3FORMAT data;
    data.data_type = V3DTV_DATA;
    data.pass_time = timeDataAvailable ? (timeData.time | 0x80000000) : 0;
    data.address = eepData.myAddress;
    if (adcTempVoltForSend) {
        data.chip_volt = adcExtra[7];
        data.chip_temp = adcExtra[6];
    } else {
        data.chip_volt = 0;
        data.chip_temp = 0;
    }
    data.values[0] = values[0];
    data.values[1] = values[1];
    data.values[2] = values[2];

    return appendToDataArray(&data, MINPACKETSIZE);
/*
    if (sending_index == write_data_index) {
        //バッファがフルで無線送信中にこの関数が呼ばれるとデータは消失(稀)
        ++state.lost_packets_count;
        return FALSE;
    }

    lock_data_for_i2c = TRUE;   //lock for interrupt

    V3DATA *p = &ar_data[write_data_index];
    p->millis = millis();
    p->len = MINPACKETSIZE;
    _V3FORMAT *q = &p->data;
    q->data_type = V3DTV_DATA;
    q->pass_time = timeDataAvailable ? (timeData.time | 0x80000000) : 0;
    q->address = eepData.myAddress;
    q->chip_volt = adcVolt;
    q->chip_temp = adcTemp;
    q->values[0] = values[0];
    q->values[1] = values[1];
    q->values[2] = values[2];

    if (state.data_count == 0) {
        read_data_index = write_data_index;
        ++state.data_count;
    } else if (state.data_count == DATAARRAYLENGTH) {
        ++state.lost_packets_count;
        if (++read_data_index == DATAARRAYLENGTH) read_data_index = 0;
    } else {
        ++state.data_count;
    }
    if (++write_data_index == DATAARRAYLENGTH) write_data_index = 0;

    lock_data_for_i2c = FALSE;
    led_event(LEDEVENT_HAVE_DATA);

    //すぐに送信を試みる
    cntdwn_for_send = 1;
    return TRUE;
*/
}


/***********************************************************
 * 無線受信（補助）
 * 受信関数で実行できないことをloop()内で実行するために記憶
 ***********************************************************/

//リブートタイプ
typedef enum {
    REBOOT_NONE = 0,
    REBOOT_SETTING_MODE = 1,
    REBOOT_WARM = 2,
    DEEP_SLEEP  = 3,
    REBOOT_MASK = 7             //MASK
} REBOOTREQUEST;
uint8_t reboot_request;         //REBOOTREQUEST

uint32_t send_eep_to;           //-1以外の時、このアドレスにEEPDATAを送信
uint32_t send_time_to;          //-1以外の時、このアドレスにTIMEDATAを送信
uint32_t send_state_to;         //-1以外の時、このアドレスにSTATEDATAを送信

char *send_broadcast_message;   //NULL以外の時、V3DTV_MESSAGEでブロードキャスト送信する

/***********************************************************
 * 無線受信
 ***********************************************************/

/*
  工場内のデータ転送では無線通信が不安定になるため、送信者側が失敗したと
 判定した場合でもデータが相手に到着している場合が多々ある。
 そのため、V3DTV_DATAの送信IDに送信データ配列インデックスの下位7ビット
 を用いることで、受信側は同一データの連続受信を回避している。
  また、V3DTV_DATA以外の通信はデータIDのb7ビットを立てることで識別している。
*/

//無線重複受信回避のためアドレスやIDを記憶
#pragma pack(1)
typedef struct {
    uint16_t address;
    uint8_t lastDataId;
    uint32_t millis;
} SENDERINFO;
#pragma pack()
#define SENDERARRAYLENGTH   20
SENDERINFO ar_sender[SENDERARRAYLENGTH];

bool_t radioReceivable;     //無線が受信モードになっているか

//重複受信回避用のコールバック関数
bool_t gateFunc(uint32_t u32SrcAddr, uint8_t u8CbId) {
    int8_t i,emptyIndex=-1,oldestIndex=-1;
    uint32_t t,passTime=0,now=millis();
    SENDERINFO *p = &ar_sender[0];
    for (i = 0; i < SENDERARRAYLENGTH; ++i) {
        if (p->address && p->lastDataId >= 128 && (now - p->millis) > 500) {
            //128以上のidはデータ以外の通信なので0.5秒以上経過していればクリアする
            p->address = 0;
        }
        if (p->address == 0) {
            if (emptyIndex == -1)
                //空きバッファを記憶
                emptyIndex = i;
        } else if (p->address == (uint16_t)u32SrcAddr &&
            ((p->lastDataId < 128 && u8CbId < 128) ||
            (p->lastDataId >= 128 && u8CbId >= 128))) {
            p->millis = now;
            if (p->lastDataId == u8CbId)
                //重複している
                return FALSE;
            //重複していないので登録
            p->lastDataId = u8CbId;
            return TRUE;
        } else {
            t = now - p->millis;
            if (t > passTime) {
                //最も古いバッファを記憶
                passTime = t;
                oldestIndex = i;
            }
        }
        ++p;
    }
    //該当するデータが無かったので、空きバッファまたは最も古いバッファに保存
    i = (emptyIndex >= 0) ? emptyIndex : oldestIndex;
    p = &ar_sender[i];
    p->address = (uint16_t)u32SrcAddr;
    p->lastDataId = u8CbId;
    p->millis = now;
    return TRUE;
}

//無線受信処理
void rxFunc(uint32_t u32SrcAddr, bool_t bBroadcast, uint8_t u8CbId,
    uint8_t u8DataType, uint8_t *pu8Data, uint8_t u8Length, uint8_t u8Lqi) {

    led_event(LEDEVENT_RADIO_RX);

    //serial_printf("rxFunc dataType=%d len=%d 1stByte=%d\r\n",
    //    u8DataType, u8Length, u8Length > 0 ? *pu8Data : 0);

    if (u8DataType != DT_V3 || u8Length == 0) {
        //不正なデータタイプ
        //serial_puts("Unknown receive. DataType not DT_V3 or Length=0\r\n");
        return;
    }

    if (!settingMode) {
        switch (*pu8Data) {
        case V3DTV_DATA:
            if (!UARTMASTER() && RELAYDEV() &&
                u8Length > MINPACKETSIZE && u8Length <= MAXPACKETSIZE &&
                verify_V3FORMAT((_V3FORMAT *)pu8Data)) {
                //データ受信

                //送信データ配列に保存
                *(pu8Data + u8Length) = u8Lqi;
                appendToDataArray((_V3FORMAT *)pu8Data, u8Length + 1);
                //serial_printf("data count=%d\r\n", state.data_count);
            } else {
                ++state.unknown_packets_count;
                //serial_puts("Invalid data\r\n");
            }
            break;
        case V3DTV_GET_EEPDATA:
            if (u8Length == 1 && eepDataAvailable) {
                //EEPROMのデータ要求
                //serial_puts("eep data request received\r\n");
                send_eep_to = u32SrcAddr;
            }
            break;
        case V3DTV_GET_TIMEDATA:
            if (u8Length == 1 && timeDataAvailable) {
                //時刻データの要求
                //serial_puts("time request received\r\n");
                send_time_to = u32SrcAddr;
            }
            break;
        case V3DTV_TIMEDATA:
            if (u8Length == sizeof(_TIME_VALUE_STRUCT) &&
                verify_TIME_VALUE_STRUCT((_TIME_VALUE_STRUCT *)pu8Data)) {

                //時刻データ更新
                //serial_puts("time data updating\r\n");

                memcpy(&timeDataFromXXX, pu8Data, sizeof(_TIME_VALUE_STRUCT));
                timeDataFromXXX.msec += 50;//適当
                if (timeDataFromXXX.msec >= 1000) {
                    timeDataFromXXX.msec -= 1000;
                    ++timeDataFromXXX.time;
                }
                timeDataFromXXXAvailable = TRUE;

                /*memcpy(&timeData, pu8Data, sizeof(_TIME_VALUE_STRUCT));
                timeDataAvailable = TRUE;
                timeDataLifeSeconds = TIMEDATALIFESECOND;

                //次回の時刻同期は1時間後
                cntdwn_for_gettime = 3600;*/
            }
            break;
        case V3DTV_GET_STATE:
            //ステータスの要求
            if (u8Length == 1) {
                //serial_puts("status request received\r\n");
                send_state_to = u32SrcAddr;
            }
            break;
        //default:
            //serial_printf("Unknown receive. cbId=%u 1st byte=%u length=%d\r\n",
            //    u8CbId, *pu8Data, u8Length);
        }
    } else {
        if (*pu8Data == V3DTV_SET_EEPDATA) {
            if (u8Length == sizeof(_EEPROM_SETTING_STRUCT) && !bBroadcast &&
                verify_EEPROM_SETTING_STRUCT((_EEPROM_SETTING_STRUCT *)pu8Data)) {

#ifdef USE_I2C
                if (((_EEPROM_SETTING_STRUCT *)pu8Data)->modeBits & EMB_I2CSLAVE) {
                    //I2Cコードが使用されているのにI2Cスレーブとして動作させようとした
                    send_broadcast_message = "EEPROM not updated. This device compiled for I2C. Not I2CS";
                } else
#endif
                if (u8Lqi >= 150) {
                    saveEEPROMSetting((_EEPROM_SETTING_STRUCT *)pu8Data);
                    //serial_puts("EEPROM updated\r\n");

                    send_broadcast_message = "EEPROM updated";

                    //reboot
                    reboot_request = REBOOT_SETTING_MODE;
                } else {
                    //serial_puts("EEPROM not updated. Less LQI\r\n");
                    send_broadcast_message = "EEPROM not updated. Less LQI";
                }
            }
        } else {
            //serial_printf("Unknown receive. cbId=%u 1st byte=%u length=%d\r\n",
            //    u8CbId, *pu8Data, u8Length);
        }
    }
}


/***********************************************************
 * 無線送信
 ***********************************************************/
volatile int16_t txDoneId;  //送信完了したデータIDを記憶
volatile bool_t txSuccess;  //送信成功の可否を記憶

//V3DTV_DATA以外を送信する場合は、送信データIDに128-255を使用する
//そのため、radio_write()などを実行する前にこの関数を呼び出すこと
static uint8_t __id;
void setCbId() {
    if (++__id < 128) __id = 128;
    radio_setCbId(__id);
}
void setUartCbId() {
    if (++__id < 128) __id = 128;
    serial_putc(__id);
}

//無線送信完了処理
void txFunc(uint8_t u8CbId, bool_t bSuccess) {
    if (u8CbId < 128) {
        //V3DTV_DATAに対してのみ行う
        txDoneId = (uint16_t)u8CbId;
        txSuccess = bSuccess;

        //送信成功の可否をビットで記録
        state.recently_success_bits = (state.recently_success_bits << 1) + (bSuccess ? 1 : 0);

        //送信失敗したときは30秒後にリトライ
        //cntdwn_for_send = bSuccess ? 1 : 30;
    }

    //serial_puts("TX done cbId=");
    //serial_printf("%d", u8CbId);
    //serial_puts(bSuccess ? " SUCCESS\r\n" : " NOT REACH\r\n");
}

//送信データ配列に送信対象があれば無線送信を開始する
bool_t sendData() {
    if (state.data_count == 0 || read_data_index < 0)
        return FALSE;

    if (UARTSLAVE()) {
        //if (uartTxGap < V3UART_DATAGAP)
        //    //前回のUART送信から十分な時間が経過していない
        //    return FALSE;

        //経過時間pass_timeを更新する
        updateData(read_data_index);

        V3DATA *p = &ar_data[read_data_index];
        uint8_t cbId = read_data_index & 127;  //データIDは0-127を使用する
        serial_putc(V3UART_FIRSTBYTE);
        serial_putc(cbId);
        serial_putc(p->len);
        serial_write((uint8_t *)&p->data, p->len);

        //送信成功
        if (--state.data_count == 0) {
            read_data_index = -1;
        } else if (++read_data_index == DATAARRAYLENGTH) {
            read_data_index = 0;
        }
        led_event(LEDEVENT_HAVE_DATA);

    } else {
        //serial_puts("sendData()\r\n");

        //経過時間pass_timeを更新する
        updateData(read_data_index);

        sending_index = read_data_index;
        V3DATA *p = &ar_data[sending_index];
        radio_setCbId(sending_index & 127);  //データIDは0-127を使用する
        radio_write(eepData.targetAddress, DT_V3, (uint8_t *)&p->data, p->len);
    }

    led_event(LEDEVENT_RADIO_TX);
    return TRUE;
}

//サブアドレスに送信する
//チェックは行わないので注意
/*void sendSubData(int16_t index) {
    serial_puts("sendSubData()\r\n");

    //経過時間pass_timeを更新する
    updateData(index);

    //sending_index = read_data_index;
    //radio_setCbId(sending_index & 127);  //データIDは0-127を使用する
    setCbId(); //送信到着管理はしないので一般のcbIdを使う
    V3DATA *p = &ar_data[index];
    radio_write(eepData.targetAddress, DT_V3, (uint8_t *)&p->data, p->len);
    led_event(LEDEVENT_RADIO_TX);
}*/

/***********************************************************
 * I2Cスレーブ送受信
 * 無線送信せずにI2Cスレーブとして動作する
 ***********************************************************/
//#ifndef USE_I2C
//本当の割り込みで動作(basicio.c改)
//バージョン3よりコマンド番号の変更および値のビッグエンディアンへ変更

uint8_t ar_i2cs_receive[32];

void prepareFunc(uint8_t cmd) {
    if (cmd == V3I2C_GET_COUNT) {
        //コマンド4  データ数(2byte) を返す
        static uint16_t dc;
        dc  = state.data_count;
        i2cs_write((uint8_t *)&dc, 2);

    } else if (cmd == V3I2C_GET_TIME) {
        //コマンド16  保持している時刻を返す
        //準備が出来ていない場合は全て255になる
        static _TIME_VALUE_STRUCT tv;
        if (timeDataAvailable && !lock_time_for_i2c) {
            memcpy(&tv, &timeData, sizeof(_TIME_VALUE_STRUCT));
        } else {
            memset(&tv, 0xff, sizeof(_TIME_VALUE_STRUCT));
        }
        i2cs_write((uint8_t *)&tv, sizeof(_TIME_VALUE_STRUCT));

    } else if (cmd == V3I2C_GET_DATA) {
        //コマンド60  最初のデータを返す
        //データ長は23バイト固定で、最初の1バイトが以降のデータ長を表す。
        //23バイトに満たない末尾は255で埋められる
        //準備が出来ていない場合は全て255になる
        static uint8_t data[MAXPACKETSIZE + 1];//23byte
        if (!lock_data_for_i2c && txDoneId == -1 && state.data_count >= 0 && read_data_index >= 0) {
            updateData(read_data_index);   //pass_timeを更新
            V3DATA *p = &ar_data[read_data_index];
            data[0] = p->len;
            memcpy(&data[1], &p->data, p->len);
            if (MAXPACKETSIZE > p->len)
                memset(&data[1 + p->len], 255, MAXPACKETSIZE - p->len);
            i2cs_write(&data[0], p->len + 1);

            //無線送信完了を偽装
            sending_index = read_data_index;
            txDoneId = read_data_index & 127;
            txSuccess = TRUE;
        } else {
            memset(&data[0], 255, MAXPACKETSIZE + 1);
            i2cs_write(&data[0], MAXPACKETSIZE + 1);
        }
    }
}

void receivedFunc(uint8_t *p, uint16_t len) {
    if (*p == V3I2C_SET_TIME && len == sizeof(_TIME_VALUE_STRUCT) + 1 &&
        verify_TIME_VALUE_STRUCT((_TIME_VALUE_STRUCT *)(p + 1))) {
        //コマンド7  時刻を設定する
        memcpy(&timeDataFromXXX, p + 1, sizeof(_TIME_VALUE_STRUCT));
        timeDataFromXXX.msec += 10;//適当
        if (timeDataFromXXX.msec >= 1000) {
            timeDataFromXXX.msec -= 1000;
            ++timeDataFromXXX.time;
        }
        timeDataFromXXXAvailable = TRUE;
    }
    else if (*p == V3I2C_REBOOT && len == 2 && *(p + 1) == 32) {
        //コマンド101
        reboot_request = REBOOT_SETTING_MODE;
    }

}
//#endif


/***********************************************************
 * UART送受信
 * データやコマンドを無線でなくUARTで行う場合
 ***********************************************************/

uint16_t uartTxGap;     //前回のUART送信完了(serial_getTxCount()==0)からの経過時間[ms]

uint16_t uartRxGap;     //前回のUART受信完了(serial_getRxCount()==0)からの経過時間[ms]

#define UARTRXBUFSIZE   (sizeof(_EEPROM_SETTING_STRUCT) + 3)
uint16_t uartRxHeaderLength;
uint8_t uartRxBuf[UARTRXBUFSIZE];

bool_t uartRxHasError;

uint16_t uartSlaveDataCount;    //UARTマスターのとき、スレーブが返したdata_count値を記憶
bool_t serialInitialized;

void uartReceiveFunc() {
    if (uartRxGap >= V3UART_DATAGAP) {
        uartRxHasError = FALSE;
        uartRxHeaderLength = 0;
    }
    while (serial_getRxCount()) {
        uint8_t c = serial_getc();
        if (uartRxHasError) continue;
        if (uartRxHeaderLength == 0) {
            if (c != V3UART_FIRSTBYTE) {
                //１バイト目がおかしい
                uartRxHasError = TRUE;
                continue;
            }
        } else if (uartRxHeaderLength == 2) {
            if (c == 0 || c > (UARTRXBUFSIZE - 3)) {
                //データサイズが0またはバッファ長を超えている
                uartRxHasError = TRUE;
                continue;
            }
        } else if (uartRxHeaderLength >= 3 && uartRxBuf[2] == uartRxHeaderLength - 3) {
            //指定されたデータサイズを超えて書き込みしようとしている
            uartRxHasError = TRUE;
            continue;
        }
        uartRxBuf[uartRxHeaderLength++] = c;
    }
}

#define CBID        uartRxBuf[1]
#define DATALEN     uartRxBuf[2]
#define DATATYPE    uartRxBuf[3]
#define DATAPTR     (&uartRxBuf[3])

void uartSlaveRxFunc() {
    if (!uartRxHasError && uartRxGap >= (V3UART_DATAGAP / 2) &&
        uartRxHeaderLength >= 3 && DATALEN == uartRxHeaderLength - 3) {

        switch (DATATYPE) {
            case V3UDTV_GET_COUNT:
                if (DATALEN == 1) {
                    serial_putc(V3UART_FIRSTBYTE);
                    setUartCbId();
                    serial_putc(3);
                    serial_putc(V3UDTV_COUNT);
                    serial_putc(state.data_count >> 8);
                    serial_putc(state.data_count & 255);
                }
                break;
            case V3UDTV_GET_TIMEDATA:
                //スレーブからマスターに時刻を渡すこともできる
                if (timeDataAvailable) {
                    serial_putc(V3UART_FIRSTBYTE);
                    setUartCbId();
                    serial_putc(sizeof(_TIME_VALUE_STRUCT));
                    timeData.data_type = V3UDTV_TIMEDATA;
                    serial_write((uint8_t *)&timeData, sizeof(_TIME_VALUE_STRUCT));
                }
                break;
            case V3UDTV_TIMEDATA:
                if (DATALEN == sizeof(_TIME_VALUE_STRUCT) &&
                    verify_TIME_VALUE_STRUCT((_TIME_VALUE_STRUCT *)DATAPTR)) {

                    memcpy(&timeDataFromXXX, DATAPTR, sizeof(_TIME_VALUE_STRUCT));
                    timeDataFromXXX.msec += 100;//適当
                    if (timeDataFromXXX.msec >= 1000) {
                        timeDataFromXXX.msec -= 1000;
                        ++timeDataFromXXX.time;
                    }
                    timeDataFromXXXAvailable = TRUE;
                }
                break;
            case V3UDTV_GET_DATA:
                if (DATALEN == 1)
                    sendData();
                break;
            case V3UDTV_REBOOT:
                if (DATALEN == 2 && *(DATAPTR + 1) == 32)
                    reboot_request = REBOOT_SETTING_MODE;
                break;
        }
        uartRxHeaderLength = 0;
        uartRxHasError = FALSE;
    }
}

void uartMasterRxFunc() {
    if (!uartRxHasError && uartRxGap >= (V3UART_DATAGAP / 2) &&
        uartRxHeaderLength >= 3 && DATALEN == uartRxHeaderLength - 3) {

        switch (DATATYPE) {
            case V3UDTV_COUNT:
                if (DATALEN == 3)
                    uartSlaveDataCount = (((uint16_t)*DATAPTR) << 8) + *(DATAPTR + 1);
                break;
            case V3UDTV_DATA:
                if (CBID < 128 && RELAYDEV() && (DATALEN >= MINPACKETSIZE &&
                    DATALEN <= MAXPACKETSIZE) && verify_V3FORMAT((_V3FORMAT *)DATAPTR)) {
                    //データ受信

                    //送信データ配列に保存
                    appendToDataArray((_V3FORMAT *)DATAPTR, DATALEN);

                    //すぐに次を読み出してみる
                    cntdwn_for_getdata = 1;
                } else {
                    ++state.unknown_packets_count;
                }
                break;
        }
        uartRxHeaderLength = 0;
        uartRxHasError = FALSE;
    }
}

#undef CBID
#undef DATALEN
#undef DATATYPE
#undef DATAPTR


/***********************************************************
 * LED処理
 ***********************************************************/

#define LEDONCOUNT_RADIORX      50      //x4[ms]
#define LEDONCOUNT_RADIOTX      50
#define LEDONCOUNT_SAMPLING     50

//イベントに関連するDIOをビットで表す
uint32_t led_eventbits_radiorx; //b0:DIO0...b19:DIO19,b20:DO0,b21:DO1
uint32_t led_eventbits_radiotx;
uint32_t led_eventbits_havedata;
uint32_t led_eventbits_sampling;

//LOW_TO_ONのDIOをビットで表す
uint32_t led_direction_bits;

//LEDをOFFするためのカウントダウン
int16_t led_offcount_radiorx;
int16_t led_offcount_radiotx;
int16_t led_offcount_sampling;

void led_pinMode(uint8_t pinNo, LEDPINMODE mode, LEDEVENTTYPE event) {
    if (eepDataAvailable && SLEEPDEV()) return;

    if (pinNo < 20) {
        dio_write(pinNo, (mode == LOW_TO_ON) ? HIGH : LOW);   //LED OFF
        dio_pinMode(pinNo, OUTPUT);
    } else if (pinNo < 22) {
        do_enable(TRUE);
        do_write(pinNo - 20, (mode == LOW_TO_ON) ? HIGH : LOW);   //LED OFF
    } else return;

    uint32_t b = 1 << pinNo;
    uint32_t i = b ^ 0xffffffff;
    led_eventbits_radiorx &= i;
    led_eventbits_radiotx &= i;
    led_eventbits_havedata &= i;
    led_eventbits_sampling &= i;
    if (event & LEDEVENT_RADIO_RX)  led_eventbits_radiorx |= b;
    if (event & LEDEVENT_RADIO_TX)  led_eventbits_radiotx |= b;
    if (event & LEDEVENT_HAVE_DATA) led_eventbits_havedata |= b;
    if (event & LEDEVENT_SAMPLING)  led_eventbits_sampling |= b;

    led_direction_bits &= i;
    if (mode == LOW_TO_ON) led_direction_bits |= b;
}

void led_event(LEDEVENTTYPE event) {
    if (eepDataAvailable && SLEEPDEV()) return;

    uint32_t x,w;
    uint8_t i;
    if ((event & LEDEVENT_RADIO_RX) && led_eventbits_radiorx) {
        x = led_direction_bits;
        w = led_eventbits_radiorx;
        for (i = 0; i < 22; ++i) {
            if (w == 0) break;
            if (w & 1) {
                if (i < 20)
                    dio_write(i, (x & 1) ? LOW : HIGH); //LED ON
                else
                    do_write(i - 20, (x & 1) ? LOW : HIGH); //LED ON
                led_offcount_radiorx = LEDONCOUNT_RADIORX;
            }
            x >>= 1;
            w >>= 1;
        }
    }
    if ((event & LEDEVENT_RADIO_TX) && led_eventbits_radiotx) {
        x = led_direction_bits;
        w = led_eventbits_radiotx;
        for (i = 0; i < 22; ++i) {
            if (w == 0) break;
            if (w & 1) {
                if (i < 20)
                    dio_write(i, (x & 1) ? LOW : HIGH); //LED ON
                else
                    do_write(i - 20, (x & 1) ? LOW : HIGH); //LED ON
                led_offcount_radiotx = LEDONCOUNT_RADIOTX;
            }
            x >>= 1;
            w >>= 1;
        }
    }
    if ((event & LEDEVENT_HAVE_DATA) && led_eventbits_havedata) {
        x = led_direction_bits;
        w = led_eventbits_havedata;
        for (i = 0; i < 22; ++i) {
            if (w == 0) break;
            if (w & 1) {
                uint8_t f = ((state.data_count ? 0 : 1) ^ ((x & 1) ? 1 : 0)) ? LOW : HIGH;
                if (i < 20)
                    dio_write(i, f);
                else
                    do_write(i - 20, f);
            }
            x >>= 1;
            w >>= 1;
        }
    }
    if ((event & LEDEVENT_SAMPLING) && led_eventbits_sampling) {
        x = led_direction_bits;
        w = led_eventbits_sampling;
        for (i = 0; i < 22; ++i) {
            if (w == 0) break;
            if (w & 1) {
                if (i < 20)
                    dio_write(i, (x & 1) ? LOW : HIGH); //LED ON
                else
                    do_write(i - 20, (x & 1) ? LOW : HIGH); //LED ON
                led_offcount_sampling = LEDONCOUNT_SAMPLING;
            }
            x >>= 1;
            w >>= 1;
        }
    }
}

void led_off() {
    if (eepDataAvailable && SLEEPDEV()) return;

    if (led_offcount_radiorx > 0 && --led_offcount_radiorx == 0) {
        uint8_t i;
        uint32_t x = led_direction_bits;
        uint32_t w = led_eventbits_radiorx;
        for (i = 0; i < 22; ++i) {
            if (w & 1) {
                if (i < 20)
                    dio_write(i, (x & 1) ? HIGH : LOW); //LED OFF
                else
                    do_write(i - 20, (x & 1) ? HIGH : LOW); //LED OFF
            }
            x >>= 1;
            w >>= 1;
        }
    }
    if (led_offcount_radiotx > 0 && --led_offcount_radiotx == 0) {
        uint8_t i;
        uint32_t x = led_direction_bits;
        uint32_t w = led_eventbits_radiotx;
        for (i = 0; i < 22; ++i) {
            if (w & 1) {
                if (i < 20)
                    dio_write(i, (x & 1) ? HIGH : LOW); //LED OFF
                else
                    do_write(i - 20, (x & 1) ? HIGH : LOW); //LED OFF
            }
            x >>= 1;
            w >>= 1;
        }
    }
    if (led_offcount_sampling > 0 && --led_offcount_sampling == 0) {
        uint8_t i;
        uint32_t x = led_direction_bits;
        uint32_t w = led_eventbits_sampling;
        for (i = 0; i < 22; ++i) {
            if (w & 1) {
                if (i < 20)
                    dio_write(i, (x & 1) ? HIGH : LOW); //LED OFF
                else
                    do_write(i - 20, (x & 1) ? HIGH : LOW); //LED OFF
            }
            x >>= 1;
            w >>= 1;
        }
    }
}


/***********************************************************
 * 変数初期化
 ***********************************************************/

void initVarsEveryWake() {
    reboot_request = REBOOT_NONE;   //リブートタイプ

    send_eep_to = 0xffffffff;       //-1以外の時、このアドレスにEEPDATAを送信
    send_time_to = 0xffffffff;      //-1以外の時、このアドレスにTIMEDATAを送信
    send_state_to = 0xffffffff;     //-1以外の時、このアドレスにSTATEDATAを送信
    send_broadcast_message = NULL;  //NULL以外の時、V3DTV_MESSAGEでブロードキャスト送信する

    memset(&ar_sender, 0, sizeof(ar_sender)); //無線重複受信回避のためアドレスやIDを記憶

    txDoneId = -1;                  //送信完了したデータIDを記憶

    adcStep = ADCS_STANDBY;         //内蔵ADC測定ステップ
    adcExtraMeasurement = 0;        //追加のADC測定
    adcSample = ADC_SAMPLE_4;
    adcExtraDone = FALSE;
    adcTempVoltForSend = FALSE;
    adcMeasurementBits = 0;

    uartTxGap = V3UART_DATAGAP;     //前回のUART送信完了からの経過時間
    uartRxGap = V3UART_DATAGAP;     //前回のUART受信完了からの経過時間
    uartRxHeaderLength = 0;
    uartRxHasError = FALSE;
    uartSlaveDataCount = 0;
    serialInitialized = FALSE;
    radioReceivable = FALSE;

    led_eventbits_radiorx = 0;      //LED処理用
    led_eventbits_radiotx = 0;
    led_eventbits_havedata = 0;
    led_eventbits_sampling = 0;
    led_direction_bits = 0;
    led_offcount_radiorx = 0;
    led_offcount_radiotx = 0;
    led_offcount_sampling = 0;
}

void initVarsColdWake() {
    cntdwn_for_measAdc = 0;         //次回のchip_volt/tempｗ測定するタイミング
                                    //スリープデバイス用

    memset(&state, 0, sizeof(_STATE_STRUCT));
    state.data_type = V3DTV_STATE;
    //recently_success_bits = 0;      //直近32回の送信結果を記憶
    //lost_packets_count = 0;         //失われたパケットの数
    //unknown_packets_count = 0;      //V3DTV_DATAの不明パケット数

    timeDataAvailable = FALSE;      //変数timeDataは有効か
    lock_time_for_i2c = FALSE;
    timeDataFromXXXAvailable = FALSE;
    eepDataAvailable = FALSE;       //変数eepDataは有効か

    //data_count = 0;                 //保持しているデータ数
    write_data_index = 0;           //次に書き込む配列のインデックス
    read_data_index = -1;           //次に読み出す配列のインデックス(初期値:-1)
    sending_index = -1;             //送信中のデータインデックス(未送信時:-1)
    lock_data_for_i2c = FALSE;

    calib_value = 0;
}



// 起動時や起床時に呼ばれる関数
void setup(bool_t warmWake, uint32_t bitmapWakeStatus) {

    //設定モードはコールドウェイクかつ
    //DIO,起床タイマー,コンパレータ,パルスカウンタ割り込みでない
    settingMode = !warmWake &&
        (bitmapWakeStatus &
            (0xfffff | E_AHI_SYSCTRL_WK0_MASK | E_AHI_SYSCTRL_COMP0_MASK |
             E_AHI_SYSCTRL_PC0_MASK | E_AHI_SYSCTRL_PC1_MASK)) == 0;

//serial_init(SERIAL_BAUD_115200);
//serial_printf("%d\r\n",settingMode);

    //変数初期化
    initVarsEveryWake();

    if (!warmWake) {
        //変数初期化
        initVarsColdWake();

        //EEPROMから設定を読み出す
        loadEEPROMSetting();
    }

    //afterWake(settingMode);

    if (settingMode) {
        //serial_init(SERIAL_BAUD_115200);
        //serial_puts("SETTING MODE\r\n");

        //変数初期化
        //initVarsColdWake();

        //EEPROMから設定を読み出す
        //loadEEPROMSetting();

        //設定モードの無線設定(ロングアドレス)
        radio_setupInit(RADIO_MODE_TXRX, APPID_SETTING, APPCH_SETTING, 3);
        radio_attachCallback(txFunc, rxFunc);

    } else if (eepDataAvailable) {
        //serial_init(UARTMODE()
        //    ? SERIAL_BAUD_9600      //データ通信用
        //    : SERIAL_BAUD_115200);  //デバッグ用


        if (SLEEPDEV()) {
            //スリープするデバイス

            //保持中データの経過時間を更新
            //uint32_t st = (uint32_t)getSleptTime();
            //serial_printf("Slept time = %ums\r\n", st);
            //addSleptTimeToDataArray(st);
            addSleptTimeToDataArray(wakeTimer_getSleptTime()
                ? (uint32_t)wakeTimer_getSleptTime()
                : eepData.sleepTime);

            if (--cntdwn_for_measAdc <= 0) {
                //内蔵ADC開始
                //ウォームウェイクの場合は約６時間毎に測定するが、
                //コールドウェイクだと毎回測定になる
                adcTempVoltForSend = TRUE;

                if (warmWake) {
                    //ウォームウェイクする場合のみクロック補正
                    //32kHz RCオシレーターの校正値を取得(処理時間約0.5ms)
                    calib_value = wakeTimer_getCalibrationValue();
                }
            } else {
                //skipAdc();
            }

            //送信専用の無線設定(ショートアドレス)
            radio_setupInit(RADIO_MODE_TXONLY, APPID, APPCH, 3);
            radio_attachCallback(txFunc, NULL);
            radio_setupShortAddress(eepData.myAddress);

            //ユーザーアプリケーションの初期化
            setupApp(SLEEPDEV(), bitmapWakeStatus);

            adcMeasurementBits = adcExtraMeasurement;
            if (adcTempVoltForSend)
                adcMeasurementBits |= ADC_SOURCEBIT_TEMP | ADC_SOURCEBIT_VOLT;
            beginAdc(); //測定しない場合でも呼び出して良い


            //serial_puts("Initialized SLEEP DEVICE\r\n");
        } else {
            //スリープしないデバイス
            if (RELAYDEV()) {
//serial_init(SERIAL_BAUD_9600);

                if (state.data_count != DATAARRAYLENGTH) {
                    //データ受信可能な無線設定(ショートアドレス)
                    radio_setupInit(RADIO_MODE_TXRX, APPID, APPCH, 3);
                    radioReceivable = TRUE;
                    radio_attachCallback(txFunc, rxFunc);
                    radio_setRxGateCallback(gateFunc);
                } else {
                    //データがいっぱいなので無線受信はOFF
                    radio_setupInit(RADIO_MODE_TXONLY, APPID, APPCH, 3);
                    radioReceivable = FALSE;
                    radio_attachCallback(txFunc, NULL);
                    //radio_setRxGateCallback(gateFunc);//不要
                }
                radio_setupShortAddress(eepData.myAddress);

                //serial_puts("Initialized RELAY DEVICE\r\n");
            } else {
                //データ受信しない無線設定(ショートアドレス)
                radio_setupInit(RADIO_MODE_TXRX, APPID, APPCH, 3);
                radioReceivable = TRUE;
                radio_attachCallback(txFunc, rxFunc);//なにかしらコマンドは受ける
                //radio_setRxGateCallback(gateFunc); //データは無いのでデフォルトのゲートでよい
                radio_setupShortAddress(eepData.myAddress);

                //serial_puts("Initialized NO-SLEEP DEVICE\r\n");
            }
            if (I2CSLAVE()) {
                //無線送信せずにI2Cスレーブとして応える
                i2cs_enable(
                    50,             //uint8_t u16Address,
                    FALSE,          //bool_t b10BitAddress,
                    FALSE,          //bool_t bUseSecondPin,
                    prepareFunc,    //void (*prepareFunc)(uint8_t),
                    receivedFunc,   //void (*receivedFunc)(uint8_t*, uint8_t));
                    &ar_i2cs_receive[0],
                    sizeof(ar_i2cs_receive)
                );
                //serial_puts("I am I2C slave\r\n");
            } else if (UARTMODE()) {
                //UARTマスタまたはスレーブとして動作する
                serial_init(SERIAL_BAUD_9600);
                serialInitialized = TRUE;
            }
            //serial_printf("My address=%d\r\n", eepData.myAddress);

            //ユーザーアプリケーションの初期化
            setupApp(SLEEPDEV(), bitmapWakeStatus);
        }

    } else {
        //serial_puts("EEP NOT AVAILABLE. GOODNIGHT!\r\n");
    }
}

// setup()後、３種類のイベントで呼ばれるループ関数
void loop(EVENTS event) {
    static APPSTATE procState;
    static bool_t doneReg;
    static bool_t firstProc;
    static bool_t ableToSend;               //スリープするデバイスの送信許可フラグ
    static bool_t sleeping;                 //スリープ命令が実行された
    static uint32_t cntdwnForMeasure;

    if (event == EVENT_START_UP) {
        // 最初に呼ばれる

        //変数初期化
        doneReg = FALSE;
        sleeping = FALSE;
        if (settingMode) {
            /*if (send_broadcast_message) {
                sb_clear();
                sb_putc(V3DTV_MESSAGE);
                sb_puts(send_broadcast_message);
                setCbId();
                radio_write(RADIO_ADDR_BROADCAST, DT_V3, (const uint8_t *)sb_getBuffer(),
                    (uint8_t)strlen(sb_getBuffer()));
                led_event(LEDEVENT_RADIO_TX);
                send_broadcast_message = NULL;
            }*/

            if (eepDataAvailable) {

                //設定モードの場合は最初に現在の設定を送信する
                setCbId();
                radio_write(RADIO_ADDR_BROADCAST, DT_V3, (uint8_t *)&eepData,
                    sizeof(_EEPROM_SETTING_STRUCT));
                led_event(LEDEVENT_RADIO_TX);

            } else {
                //設定が存在しない場合
                setCbId();
                radio_puts(RADIO_ADDR_BROADCAST, DT_V3, "\tSETTING NOT AVAILABLE");
                led_event(LEDEVENT_RADIO_TX);
            }
        } else {
            if (!eepDataAvailable) {
                //ウォームウェイクでもEEPROMに設定が無い場合は何もできないので寝る
                //serial_puts("deep sleep\r\n");
                reboot_request = DEEP_SLEEP;
            }
#ifdef USE_I2C
            else if (I2CSLAVE()) {
                //I2Cコードが使用されているのにI2Cスレーブとして起動された
                //EEPROM書き込み時にチェックがあるものの、既に書き込まれた設定だと
                //すり抜けてしまうため、ここで最終確認
                reboot_request = DEEP_SLEEP;
            }
#endif
            else if (SLEEPDEV()) {
                //固有の変数初期化
                procState = DATA_NOT_READY;
                firstProc = TRUE;
                ableToSend = TRUE;

                //データがあればすぐに送信
                //serial_printf("data_count=%d\r\n",state.data_count);
                sendData();

            } else {
                //固有の変数初期化
                procState = DATA_NONE;
                firstProc = FALSE;
                cntdwn_for_send = 0;     //データは持ってないので送信しない
                cntdwn_for_gettime = 10; //時刻取得はすぐに行う
                cntdwnForMeasure = SLEEPTIME_FOR_FIRST_WARMWAKE / 1000; //計測タイミング[秒]
                cntdwn_for_getdata = 30; //データ要求 UARTマスター用
            }
        }

    }
    
    if (sleeping) return;
    
    if (event == EVENT_TICK_SECOND) {
        // 1秒毎に呼ばれる

        //serial_printf("%d/%d %02d:%02d:%02d\r\n",timeData.month,timeData.day,timeData.hour,timeData.minute,timeData.second);
        if (!settingMode && !SLEEPDEV()) {
            //スリープしないデバイス

            if (timeDataAvailable && --state.timeDataLifeSeconds == 0) {
                //保持している時刻データの有効期限が切れた
                timeDataAvailable = FALSE;
            }

            if (eepData.sleepTime && cntdwnForMeasure && --cntdwnForMeasure == 0) {
                //計測開始

                adcTempVoltForSend = (--cntdwn_for_measAdc <= 0) ? TRUE : FALSE;
                adcMeasurementBits = adcExtraMeasurement;
                if (adcTempVoltForSend)
                    adcMeasurementBits |= ADC_SOURCEBIT_TEMP | ADC_SOURCEBIT_VOLT;
                beginAdc(); //測定しない場合でも呼び出して良い

                procState = DATA_NOT_READY;
                firstProc = TRUE;
                doneReg = FALSE;
                cntdwnForMeasure = eepData.sleepTime / 1000;
                if (cntdwn_for_measAdc <= 0 && cntdwnForMeasure != 0) {
                    //電圧,温度は6時間毎
                    cntdwn_for_measAdc = ADC_VOLTTEMP_INTERVAL / cntdwnForMeasure;
                }
            }

            if (!I2CSLAVE() && !UARTSLAVE() && !SLEEPDEV()) {
                //I2Cスレーブ、UARTスレーブでなければ送信処理を行う

                if (--cntdwn_for_gettime == 0) {
                    //時刻取得を試みる
                    setCbId();
                    uint8_t c = V3DTV_GET_TIMEDATA;
                    radio_write(eepData.targetAddress, DT_V3, &c, 1);
                    led_event(LEDEVENT_RADIO_TX);
                } else if (cntdwn_for_gettime <= -60) {
                    //時刻同期できなかった場合は約1分毎にリトライ
                    cntdwn_for_gettime = 1;
                } else if (state.data_count > 0 && cntdwn_for_send > 0 && --cntdwn_for_send == 0) {
                    //データを持っていれば送信する
                    sendData();
                }

                if (UARTMASTER() && state.data_count < DATAARRAYLENGTH) {
                    if (--cntdwn_for_getdata == 0) {
                        //UARTスレーブにデータ要求
                        serial_putc(V3UART_FIRSTBYTE);
                        setUartCbId();
                        serial_putc(1);
                        serial_putc(V3UDTV_GET_DATA);
                        cntdwn_for_getdata = 30;
                    }
                }
            }

            if (RELAYDEV() &&
                ((radioReceivable && state.data_count == DATAARRAYLENGTH) ||
                 (!radioReceivable && state.data_count <= DATAARRAYLENGTH / 2))) {

                //データがいっぱいになったら、無線受信を中止すべくリブートする
                //無線受信を中止状態でデータが半分まで減ったら、受信解除すべくリブートする
                reboot_request = REBOOT_WARM;
            }
        }

    } else if (event == EVENT_TICK_TIMER) {
        // 4ミリ秒毎(デフォルト)に呼ばれる

        if ((reboot_request & REBOOT_MASK) != REBOOT_NONE &&
            (!serialInitialized || serial_getTxCount() == 0) &&
            send_broadcast_message == NULL && radio_txCount() == 0) {
                
            //再起動要求に応える
            beforeSleep(NULL);
            sleeping = TRUE;
            switch (reboot_request & REBOOT_MASK) {
                case REBOOT_SETTING_MODE: reset(); /*sleepTimer(SLEEPTIME_FOR_REBOOT, FALSE);*/ break;
                case REBOOT_WARM: sleepTimer(SLEEPTIME_FOR_REBOOT, TRUE); break;
                case DEEP_SLEEP: deepSleep(); break;
                default: sleeping = FALSE;
            }
            reboot_request = REBOOT_NONE;
            return;
        }

        //LED OFF処理
        led_off();

        if (settingMode) {
            if (send_broadcast_message) {
                sb_clear();
                sb_putc(V3DTV_MESSAGE);
                sb_puts(send_broadcast_message);
                setCbId();
                radio_write(RADIO_ADDR_BROADCAST, DT_V3, (const uint8_t *)sb_getBuffer(),
                    (uint8_t)strlen(sb_getBuffer()));
                led_event(LEDEVENT_RADIO_TX);
                send_broadcast_message = NULL;
            }

            //設定モードの場合はスリープに入る
            //if (millis() == SETTING_MODE_PERIOD) serial_puts("GO TO BED\r\n");
            if (millis() > SETTING_MODE_PERIOD && (!serialInitialized || serial_getTxCount() == 0)) {
                beforeSleep(NULL);
                if (eepDataAvailable) {
                    //初回のウォームウェイクに向けてスリープ
                    sleepTimer(SLEEPTIME_FOR_FIRST_WARMWAKE, TRUE);
                } else {
                    deepSleep();
                }
                sleeping = TRUE;
                return;
            }
        } else {
            if (!SLEEPDEV()) {
                //スリープしない場合

                //時刻を進める
                updateTime(4);  //4[ms]

                if (UARTMODE()) {
                    //UARTを通信に使用している場合は通信ギャップ(パケット間の時間)を計測
                    if (serial_getTxCount())
                        uartTxGap = 0;
                    else if (uartTxGap < V3UART_DATAGAP)
                        uartTxGap += 4;
                    if (serial_getRxCount())
                        uartRxGap = 0;
                    else if (uartRxGap < V3UART_DATAGAP)
                        uartRxGap += 4;

                    //受信処理
                    uartReceiveFunc();
                    if (UARTSLAVE())
                        uartSlaveRxFunc();
                    if (UARTMASTER())
                        uartMasterRxFunc();
                }

                if (timeDataFromXXXAvailable) {
                    //I2CまたはUART、無線から時刻設定が行われた
                    lock_time_for_i2c = TRUE;   //lock for interrupt
                    memcpy(&timeData, &timeDataFromXXX, sizeof(_TIME_VALUE_STRUCT));
                    bool_t b = timeDataAvailable;
                    timeDataAvailable = TRUE;
                    state.timeDataLifeSeconds = TIMEDATALIFESECOND;
                    timeDataFromXXXAvailable = FALSE;
                    lock_time_for_i2c = FALSE;

                    if (!b)
                        //持っているデータのpass_timeに時刻を設定
                        updateDataArray();

                    if (UARTMASTER()) {
                        //UARTスレーブに時刻データを渡す
                        serial_putc(V3UART_FIRSTBYTE);
                        setUartCbId();
                        serial_putc(sizeof(_TIME_VALUE_STRUCT));
                        serial_write((uint8_t *)&timeDataFromXXX, sizeof(_TIME_VALUE_STRUCT));
                    }

                    if (!I2CSLAVE())
                        //次回の時刻同期は1時間後
                        cntdwn_for_gettime = 3600;
                }

                if (send_eep_to <= 0xfff) {
                    //EEPデータの送信要求に応える
                    setCbId();
                    eepData.data_type = V3DTV_EEPDATA;//念のため
                    radio_write(send_eep_to, DT_V3, (uint8_t *)&eepData,
                        sizeof(_EEPROM_SETTING_STRUCT));
                    led_event(LEDEVENT_RADIO_TX);
                    send_eep_to = 0xffffffff;
                }

                if (send_time_to <= 0xfff) {
                    //TIMEデータの送信要求に応える
                    setCbId();
                    timeData.data_type = V3DTV_TIMEDATA;
                    radio_write(send_time_to, DT_V3, (uint8_t *)&timeData,
                        sizeof(_TIME_VALUE_STRUCT));
                    led_event(LEDEVENT_RADIO_TX);
                    send_time_to = 0xffffffff;
                }

                if (send_state_to <= 0xfff) {
                    //STATEデータの送信要求に応える
                    setCbId();
                    radio_write(send_state_to, DT_V3, (uint8_t *)&state,
                        sizeof(_STATE_STRUCT));
                    led_event(LEDEVENT_RADIO_TX);
                    state.lost_packets_count = 0;   //リセット
                    state.unknown_packets_count = 0;//
                    send_state_to = 0xffffffff;
                }
            }
            

            if (sending_index >= 0 && txDoneId != -1) {
                lock_data_for_i2c = TRUE;//lock for interrupt

                //データ送信完了
                if (txSuccess) {
                    //送信成功
                    if (--state.data_count == 0) {
                        read_data_index = -1;
                    } else if (++read_data_index == DATAARRAYLENGTH) {
                        read_data_index = 0;
                    }
                    cntdwn_for_send = 1;    //1秒後に次を送信
                    //serial_printf("Sent. data count=%d\r\n", state.data_count);
                } else {
                    //送信失敗
                    ableToSend = FALSE;     //スリープするデバイスの場合、今回は打ち止め
                    cntdwn_for_send = 60;   //スリープしないデバイスの場合、60秒後にリトライ
                }
                sending_index = -1;
                txDoneId = -1;

                lock_data_for_i2c = FALSE;
                led_event(LEDEVENT_HAVE_DATA);
            }

            //内蔵ADCの処理(beginAdc()を呼ばない場合は内部でスルーしている)
            procAdc();

            if (procState == DATA_NOT_READY) {
                led_event(LEDEVENT_SAMPLING);

                //ユーザーアプリの計測処理
                procState = procApp(firstProc, eepData.modeBits & EMB_SLEEP, adcExtraDone);
                firstProc = FALSE;
                adcExtraDone = FALSE;
                /*if (procState == DATA_READY) {
                    //serial_printf("USER PROCESS %d %d %d\r\n",values[0],values[1],values[2]);
                } else if (procState == DATA_NONE){
                    //ユーザーがデータを破棄した場合
                    doneReg = TRUE;//データは破棄
                    //serial_puts("USER PROCESS DATA_NONE\r\n");
                }*/
            }
            
            if (adcStep == ADCS_COMPLETED) {
                if (procState == DATA_READY && sending_index == -1) {
                    //全てのデータが揃ってかつ送信中でないのでバッファに登録
                    regToDataArray();
                    adcStep = ADCS_STANDBY;
                    doneReg = TRUE;
                } else if (procState == DATA_NONE) {
                    doneReg = TRUE;//データは破棄
                }
            }

            if (SLEEPDEV()) {
                //スリープするタイプのデバイスはここで送信、スリープする

                if (millis() < SLEEPDEV_SEND_TIMELIMIT && ableToSend && sending_index < 0) {
                    //データがあれば送信
                    sendData();
                }

                if (doneReg && sending_index == -1 && (!serialInitialized || serial_getTxCount() == 0)) {
                    int32_t st = eepData.sleepTime;
                    beforeSleep((uint32_t *)&st);

                    st -= millis();
                    if (st < 1000) st = 1000;

                    if (cntdwn_for_measAdc <= 0) {
                        //6時間毎にADC測定を行う
                        cntdwn_for_measAdc = ADC_VOLTTEMP_INTERVAL / (st / 1000);
                    }

                    //データの登録がされ送信してなければスリープに入る
                    //updateDataArray(eepData.sleepTime);
                    updateTime(millis());
                    sleeping = TRUE;
                    if (calib_value)
                        sleepCalibratedTimer(st, TRUE, calib_value);
                    else
                        sleepTimer(st, TRUE);
                }
            }
        }
    }
}


