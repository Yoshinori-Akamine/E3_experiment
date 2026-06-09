#include <mwio4.h>
#include <stdio.h>

//-- ボード番号の定義 --//

#define TX_FPGA_BOARD  2
#define TX_PEV_BOARD   0

//-- 定数定義 --//

#define FULL_FREQUENCY      85000   // フルパルス周波数 [Hz]
#define DEAD_TIME           30      // デッドタイム
#define TIMER0_INTERVAL     5       // タイマー割り込み周期 [us]
#define SMA_LEN             50       // 移動平均の長さ

#define DETECT_COUNT_TH     20      // 低電流連続判定回数
#define OCP_THRESHOLD       30.0f   // OCPしきい値 [A]

//-- 外部制御フラグ（モニタソフトから書き込み）--//

volatile int Transmit_flag_1 = 0;   // 送電許可フラグ（初期値 = 0）

//-- 動作モードフラグ --//

int search_flag = 0;    // サーチパルス動作中
int full_flag   = 0;    // フルパルス動作中
int gate_enable = 0;    // ゲート許可

//-- 保護フラグ --//

volatile int ocp_fault = 0;   // OCPラッチ：1で異常停止

//-- インバータ変数 --//

int search_carrier_max;
int search_u_ref;
int search_v_ref;
int full_carrier_max;
int full_carrier_hlf;

//-- 電流センサ変数 --//

INT32 adc_0_data_peak;
float i1_ampl;
float i1_ampl_lst[SMA_LEN];
float i1_ampl_avg = 0.0f;

volatile float i1_threshold = 0.1f; // 低電流検知しきい値 [A]

int sma_ready       = 0;
int sma_fill_cnt    = 0;
int low_current_cnt = 0;

float range[5] = {0};

//-- デバッグカウンタ --//

int debug_tx_control       = 0;
int debug_search_flag      = 0;
int debug_full_flag        = 0;
int debug_read_current     = 0;
int debug_low_current_cnt  = 0;
int debug_ocp_fault        = 0;

volatile int SEARCH_PULSE_WIDTH = 27;     // サーチパルス幅 [cnt]
volatile int SEARCH_FREQUENCY = 9700;   // サーチパルス周波数 [Hz]
volatile float i1_ampl_w = 1; // 重み調整（基本的には1固定で問題なかった）

volatile int full_carrier_max_tmp = 0;
volatile int full_carrier_hlf_tmp = 0;

volatile int debug1 = 0;
volatile int t_start_flag = 0;
volatile int end_detect = 0;

//----------------------------------------------------------------------
// 電流フィルタ・判定用変数クリア
//----------------------------------------------------------------------
void Clear_Current_Filter(void)
{
    int i;

    for (i = 0; i < SMA_LEN; ++i)
    {
        i1_ampl_lst[i] = 0.0f;
    }

    i1_ampl_avg     = 0.0f;
    sma_ready       = 0;
    sma_fill_cnt    = 0;
    low_current_cnt = 0;
}


//----------------------------------------------------------------------
// 送電側電流包絡線の取得
//----------------------------------------------------------------------
void Read_Current_Envelope(void)
{
    int i;
    float i1_ampl_sum = 0.0f;

    // FPGAのpeak_holdモジュールからピーク電流値を読み出し
    adc_0_data_peak = IPFPGA_read(TX_FPGA_BOARD, 0x17);

    // 電流換算
    i1_ampl = adc_0_data_peak * i1_ampl_w * 125.0f / 8000.0f;

    // OCP判定：30A以上で異常ラッチ
    if (i1_ampl >= OCP_THRESHOLD)
    {
        ocp_fault = 1;
        debug_ocp_fault++;
    }

    // 移動平均リストを左シフト
    for (i = 0; i < SMA_LEN - 1; ++i)
    {
        i1_ampl_lst[i] = i1_ampl_lst[i + 1];
    }
    i1_ampl_lst[SMA_LEN - 1] = i1_ampl;

    // 移動平均を計算
    for (i = 0; i < SMA_LEN; ++i)
    {
        i1_ampl_sum += i1_ampl_lst[i];
    }
    i1_ampl_avg = i1_ampl_sum / SMA_LEN;

    // 起動直後のSMAバッファ充填確認
    if (sma_ready == 0)
    {
        sma_fill_cnt++;
        if (sma_fill_cnt >= SMA_LEN)
        {
            sma_ready = 1;
        }
    }

    debug_read_current++;
}


//----------------------------------------------------------------------
// キャリア同期割り込み：電流包絡線の取得
//----------------------------------------------------------------------
interrupt void Read_Current_Interrupt(void)
{
    int0_ack();
    Read_Current_Envelope();
}


//----------------------------------------------------------------------
// タイマー割り込み：送電制御
//----------------------------------------------------------------------
interrupt void Tx_Control(void)
{
    C6657_timer0_clear_eventflag();

    if (Transmit_flag_1 == 1)
    {
        // OCP発生時はゲートOFFして停止
        if (ocp_fault == 1)
        {
            gate_enable = 0;
            IPFPGA_write(TX_FPGA_BOARD, 0x06, gate_enable);

            search_flag = 0;
            full_flag   = 0;

            debug_tx_control++;
            return;
        }

        // ゲート許可
        gate_enable = 1;
        IPFPGA_write(TX_FPGA_BOARD, 0x06, gate_enable);

        if (full_flag == 1 && t_start_flag < 100)
        {
            gate_enable = 0; //過渡時OCP防止
            IPFPGA_write(TX_FPGA_BOARD, 0x06, gate_enable);

            search_flag = 0;

            t_start_flag = t_start_flag + 1;

            debug_full_flag++;
        }
        else if (t_start_flag >= 98){
            
            if(full_carrier_hlf_tmp < full_carrier_hlf){
                full_carrier_hlf_tmp = full_carrier_hlf_tmp + 1;
            }
            else{
                full_carrier_hlf_tmp = full_carrier_hlf;
            }
            
            IPFPGA_write(TX_FPGA_BOARD, 0x01, full_carrier_max);
            IPFPGA_write(TX_FPGA_BOARD, 0x02, full_carrier_hlf_tmp);
            IPFPGA_write(TX_FPGA_BOARD, 0x03, full_carrier_hlf_tmp);

            gate_enable = 1;
            IPFPGA_write(TX_FPGA_BOARD, 0x06, gate_enable);

            t_start_flag = 100;
            full_flag = 0;
            end_detect = 1;
        }
        else
        {
            //-- サーチパルス：9.7kHz，狭デューティ --//
            search_carrier_max = 100000000 / SEARCH_FREQUENCY;              // = 10000
            search_u_ref       = SEARCH_PULSE_WIDTH;                        // = 500
            search_v_ref       = search_carrier_max - SEARCH_PULSE_WIDTH;   // = 9500
            IPFPGA_write(TX_FPGA_BOARD, 0x01, search_carrier_max);
            IPFPGA_write(TX_FPGA_BOARD, 0x02, search_u_ref);
            IPFPGA_write(TX_FPGA_BOARD, 0x03, search_v_ref);

            search_flag = 1;

            // SMA充填後に低電流を連続判定
            if (end_detect != 1 && sma_ready == 1)
            {
                if (i1_ampl_avg < i1_threshold) // 閾値判定
                {
                    low_current_cnt = low_current_cnt + 1;
                }
                else
                {
                    low_current_cnt = 0;
                }

                // 閾値以下が連続で成立したらフルモードへ遷移
                if (low_current_cnt >= DETECT_COUNT_TH)
                {
                    full_flag       = 1; //フルパルス送電フラグ0→1
                    search_flag     = 0;
                    low_current_cnt = 0;
                }
                else
                {
                    full_flag       = 0;
                    // search_flag     = 0;
                    // low_current_cnt = 0;
                }
            }
            else
            {
                low_current_cnt = 0;
            }

            debug_low_current_cnt = low_current_cnt;
            debug_search_flag++;
        }
    }
    else
    {
        //-- Transmit_flag_1 = 0 → ゲートOFF，全フラグリセット --//
        gate_enable = 0;
        IPFPGA_write(TX_FPGA_BOARD, 0x06, gate_enable);

        search_flag = 0;
        full_flag   = 0;

        Clear_Current_Filter();

        // 送電OFFでOCPラッチ解除
        ocp_fault = 0;
    }

    debug_tx_control++;
}


//----------------------------------------------------------------------
// 初期化
//----------------------------------------------------------------------
void initialize(void)
{
    // サーチパルス用パラメータを事前計算

    // フルパルス用パラメータを事前計算
    full_carrier_max = 100000000 / FULL_FREQUENCY;                  // ≈ 1176
    full_carrier_hlf = 50000000  / FULL_FREQUENCY;                  // ≈ 588

    full_carrier_hlf_tmp = full_carrier_hlf - 400; //過渡時OCP防止

    Clear_Current_Filter();

    // FPGA レジスタ初期設定（サーチパルス設定で待機）
    IPFPGA_write(TX_FPGA_BOARD, 0x01, search_carrier_max);
    IPFPGA_write(TX_FPGA_BOARD, 0x02, search_u_ref);
    IPFPGA_write(TX_FPGA_BOARD, 0x03, search_v_ref);
    IPFPGA_write(TX_FPGA_BOARD, 0x05, DEAD_TIME);
    IPFPGA_write(TX_FPGA_BOARD, 0x06, 0);

    // PEV ボード初期化
    PEV_ad_set_range(TX_PEV_BOARD, range);
    PEV_ad_set_mode(TX_PEV_BOARD, 0);
    PEV_init(TX_PEV_BOARD);

    int_disable();

    PEV_inverter_disable_int(TX_PEV_BOARD);
    PEV_inverter_init(TX_PEV_BOARD, SEARCH_FREQUENCY, DEAD_TIME);
    PEV_inverter_init_int_timing(TX_PEV_BOARD, 1, 0, 0);
    PEV_int_init(TX_PEV_BOARD, 2, 0, 0, 0, 0, 0, 0, 0);

    int0_init_vector(Read_Current_Interrupt, (CSL_IntcVectId)6, FALSE);

    C6657_timer0_init(TIMER0_INTERVAL);
    C6657_timer0_init_vector(Tx_Control, (CSL_IntcVectId)7);
    C6657_timer0_start();

    PEV_inverter_enable_int(TX_PEV_BOARD);
    int0_enable_int();
    C6657_timer0_enable_int();
    int_enable();
}


//----------------------------------------------------------------------
// メイン
//----------------------------------------------------------------------
void MW_main(void)
{
    initialize();

    while (1)
    {
        // Transmit_flag_1 はモニタから書き込む
        // モード遷移とOCP停止は割り込みルーチンが管理する
    }
}
