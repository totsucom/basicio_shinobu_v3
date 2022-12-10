/*
 * フォーマットバージョン３
 * #include "basicio.h" の後で読み込む
 */

#ifndef _V3DEF_H
#define _V3DEF_H

//全装置共通

//ウォームウェイク時の無線設定
#define APPID                   0x76543210
#define APPCH                   13

//コールドウェイク時の無線設定(ロングアドレス、設定用)
#define APPID_SETTING           0x76543211
#define APPCH_SETTING           18

//コールドウェイク時の起床時間
//このタイミングでのみ V3DTV_SET_EEPDATA を受け付ける
#define SETTING_MODE_PERIOD     1000        //[ms]

//アドレスモードを切り替える場合など、スリープを再起動に使用
#define SLEEPTIME_FOR_REBOOT    1000        //[ms]

//電源投入後、最初の測定タイミングg
#define SLEEPTIME_FOR_FIRST_WARMWAKE 10000  //[ms]

//スリープするデバイスの場合でバッファに複数のデータが溜まっている場合、
//millis()値でいつまで送信するか
#define SLEEPDEV_SEND_TIMELIMIT 1000        //[ms]

//_EEPROM_SETTING_STRUCT.sleepTimeに設定可能な最小時間
#define MINIMUM_SLEEP_TIME      3000        //[ms]


//無線通信で使用する u8DataType 値
//バージョン3では1種類に固定した
typedef enum {
    DT_V3   = 5
} DATATYPES;

//バージョン3では、dataType==DT_V3 のとき、データのファーストバイトで
//データの種類を表している
typedef enum {
    V3DTV_MESSAGE       = 9,    //device->user 1byte + text ※9なのは意味がある"\t"で表現できる

    V3DTV_GET_EEPDATA   = 10,   //user->device 1byte
    V3DTV_SET_EEPDATA   = 11,   //user->device _EEPROM_SETTING_STRUCT
    V3DTV_EEPDATA       = 12,   //device->user _EEPROM_SETTING_STRUCT

    V3DTV_GET_TIMEDATA  = 15,   //device->device 1byte
    V3DTV_TIMEDATA      = 16,   //device->device _TIME_VALUE_STRUCT

    V3DTV_GET_STATE     = 20,   //device->device 1byte
    V3DTV_STATE         = 21,   //device->device _STATE_STRUCT

    V3DTV_DATA          = 60,   //device->device _V3FORMAT
    //V3DTV_SUBDATA       = 62,   //device->device _V3FORMAT
} V3DATATYPEVALUE;

//V3DATATYPEVALUEのUART版
typedef enum {
    V3UDTV_GET_COUNT    = 4,    //device(master)->device(slave) 1byte
    V3UDTV_COUNT        = 5,    //device(slave)->device(master) 1+2byte(data_count)

    V3UDTV_GET_TIMEDATA = 15,   //device->device 1byte
    V3UDTV_TIMEDATA     = 16,   //device(master)->device(slave) _TIME_VALUE_STRUCT

    V3UDTV_GET_DATA     = 59,   //device(master)->device(slave) 1byte
    V3UDTV_DATA         = 60,   //device(slave)->device(master) _V3FORMAT

    V3UDTV_REBOOT       = 101   //device(master)->device(slave) 1+1byte(32)
} V3DATATYPEVALUE_UART;

//EMB_UARTSLAVEまたはEMB_UARTMASTERのデータ形式
//0 V3UART_FIRSTBYTE
//1 cbId
//2 後に続くデータ構造体のバイトサイズ
//3 構造体の1バイト目。V3DATATYPEVALUEではなくV3DATATYPEVALUE_UART値をとる
// :

/*
UART通信はRS485半二重通信に対応させるため、無線通信のようにデータがあったら
送信するのではなく、マスター側（通信の下流側）の要求があったばあいにスレーブ
側がデータを送るというような、むしろI2C通信に似た方法をとる
*/

//EMB_UARTSLAVEまたはEMB_UARTMASTERのファーストバイト
#define V3UART_FIRSTBYTE DT_V3

//EMB_UARTSLAVEまたはEMB_UARTMASTERのデータギャップ(ミリ秒)
#define V3UART_DATAGAP   200


//I2Cスレーブ(EMB_I2CSLAVE)時のI2Cコマンド
typedef enum {
    V3I2C_GET_COUNT     = 4,
    V3I2C_GET_TIME      = 6,
    V3I2C_SET_TIME      = 7,
    V3I2C_GET_DATA      = 60,
    V3I2C_REBOOT        = 101   //1byte 32 を渡す
} V3I2CCOMMAND;


//下記 _EEPROM_SETTING_STRUCT.modeBits で使用する値
//デバイスの動作モードを指定する
typedef enum {
    EMB_NOSLEEP     = 0,   //デバイスはスリープしない
    EMB_SLEEP       = 1,   //デバイスはスリープする
    EMB_RELAY       = 2,   //リレー機能を持つ
    EMB_I2CSLAVE    = 4,   //データ送信は無線でなくI2Cスレーブで行う
    EMB_UARTSLAVE   = 8,   //データ送信は無線でなくUARTで行う、下流側
    EMB_UARTMASTER  = 16,  //データ受信は無線でなくUARTで行う、上流側

    //EMB_SLEEP, EMB_I2CSLAVE, EMB_UARTSLAVE, EMB_UARTMASTER は
    //０(=EMB_NOSLEEP)または いずれか１つのみ選択できる

    //EMB_SLEEP以外(=EMB_NOSLEEP)の場合はEMB_RELAYオプションが選択できる

    EMB_MASK    = 31
} EEPMODEBITS;
/*例
 末端の計測デバイス             EMB_SLEEP
 電源の確保された計測デバイス   EMB_NOSLEEP
 中継器                         EMB_RELAY
 レシーバー                     EMB_RELAY | EMB_I2CSLAVE
                                  または
                                EMB_RELAY | EMB_UARTSLAVE
*/

#pragma pack(1)

#define EEPDATAVERSION  30  //書式バージョン3.0

typedef struct {
    uint16_t fromAddress;
    uint16_t toAddress;
} _ADDRESS_FROM_TO;

//デバイスの設定値を記憶
//この構造体はEEPROMに保存するか、V3DTV_SET_EEPDATAまたはV3DTV_EEPDATAの通信で使用する
typedef struct {
    uint8_t data_type;      //EEPROM上ではV3DTV_EEPDATA
                            //通信ではV3DTV_SET_EEPDATAまたはV3DTV_EEPDATAの値をとる
    uint8_t version;        //EEPDATAVERSION
    uint32_t moduleAddress; //モジュールアドレス。設定ミスを防ぐなど識別用
    uint16_t myAddress;     //自身に設定するショートアドレス
    uint16_t targetAddress; //測定データを送信する相手のショートアドレス
    uint32_t sleepTime;     //[ms]
    uint8_t modeBits;       //EEPMODEBITS
    uint8_t tagName[16];    //タグ名、識別以外特に用途は無い。最大16文字で余った末尾は0埋め
    //uint16_t targetSubAddress; //測定データを送信する相手のショートアドレス
    _ADDRESS_FROM_TO transfer[4]; //転送アドレス
} _EEPROM_SETTING_STRUCT;

//V3DTV_TIMEDATA 通信で使用する時刻を表す構造体
typedef struct {
    uint8_t data_type;      //V3DTV_TIMEDATA
    //volatile uint8_t year,month,day; //yearは末尾2桁
    //volatile uint8_t hour,minute,second;
    volatile uint32_t time; //time_t(UNIX TIME)からV3BASEUNIXTIMEを減算した値
    volatile uint16_t msec; //0-999
} _TIME_VALUE_STRUCT;

#define TIMEDATALIFESECOND  (3600*24)   //時刻データの有効期限(24hr)

#define V3BASEUNIXTIME      (1640995200+32400)
                                        //JSTからV3時刻に変換するための差分
                                        //UNIX時刻(32bit)からこの値を減算することで
                                        //時刻の起点を2022/1/1に変更する
                                        //32400はタイムゾーン +9:00

//V3DTV_STATE 状態を表す構造体
typedef struct {
    uint8_t data_type;                  //V3DTV_STATE
    uint32_t recently_success_bits;     //直近32回の送信結果を記憶(成功でビットが立つ)
    uint32_t lost_packets_count;        //バッファフルにより失われたパケットの数
    uint32_t unknown_packets_count;     //V3DTV_DATAの不明パケット受信数
    uint32_t timeDataLifeSeconds;       //変数timeDataの残り有効期限[秒]
    uint16_t data_count;                //保持中のデータ数
} _STATE_STRUCT;



//V3DTV_DATA 通信で使用する構造体
typedef struct {
    uint8_t data_type;      //V3DTV_DATA
    uint32_t pass_time;     //測定からの経過時間[ms]または測定時刻
                            //最上位ビットが0のときは経過時間、1のときは測定時刻を表す
                            //測定時刻はUNIX時刻(32bit)からV3BASEUNIXTIMEを減算し、最上ビットを1にしたもの
                            //経過時間がオーバーフローしたときは0xffffffffの値となり、データが無効であることを示す
    uint16_t address;       //測定したデバイスのショートアドレス
    uint16_t chip_volt;     //TWELITE内蔵ADC値（電源電圧）
    uint16_t chip_temp;     //TWELITE内蔵ADC値（温度）
    uint16_t values[3];     //デバイス固有のセンサー値
    uint8_t lqis[0];        //転送毎に付加されるLQI値。最大5バイト
} _V3FORMAT;

#pragma pack()

#define MINPACKETSIZE   (sizeof(_V3FORMAT)) //17
#define MAXPACKETSIZE   (MINPACKETSIZE + 5) //22



//デバイス一覧
//#define V3_SOUKORECEIVER    1002        //I2CSLAVE          資材倉庫レシーバー



#endif