/*
 * vsasr.h
 *
 *  Created on: 2016/07/31
 *      Author: cryborg21
 *  This code includes the work that is distributed in
 *  https://www.vstone.co.jp/products/scara_robot/download.html#03-3
 */

#ifndef SCARA_VSASR_SAMPLES_SRC_VSASR_H_
#define SCARA_VSASR_SAMPLES_SRC_VSASR_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>


#include <silabs/SLABCP2110.h>
#include <silabs/SLABHIDtoUART.h>
#include <silabs/CP2114_Common.h>

namespace vsasr {

/*--------------		マクロ		----------------*/
/*----	通信に関する設定	----*/
//ロボットのVender ID
#define	VID	(0x10C4)

//ロボットのProduct ID
#define	PID	(0xEA80)

/*----	本体の寸法に関する設定	----*/
//第一関節の軸間距離（mm）
#define	AXISLEN_A	(80.0)
//第二関節の軸間距離（mm）
#define	AXISLEN_B	(80.0)

//ステージの縦幅(mm)
#define	FIELD_H	(210.0)
//ステージの横幅(mm)
#define	FIELD_W	(230.0)
//ステージ中央を原点(0,0)とした、アームの根元（ID1モータの出力軸）のX座標(mm)
#define	X_OFS	(FIELD_W/2 - 53.0)

//上下軸の距離換算係数
#define	HEIGHT_RATE	(148.54)
//上下軸の可動範囲(mm)
#define	HEIGHT_RANGE	(35.0)
//モータ角度から距離に変換するマクロ関数。r=モータ角度値(0.1度単位)
#define	RAD_TO_HEIGHT(r)	( ((double)r/(HEIGHT_RATE*10.0))*HEIGHT_RANGE)
//距離からモータ角度に変換するマクロ関数。h=距離(mm)
#define	HEIGHT_TO_RAD(h)	(short) (h/HEIGHT_RANGE*HEIGHT_RATE*10.0)

//ハンド軸の距離換算係数
#define	WIDTH_RATE	(31.83)
//ハンド軸の可動範囲(mm)
#define	WIDTH_RANGE	(5.0*2.0)
//爪の穴位置の幅(mm)
#define	GROW_W	(5.0*2.0)
//モータ角度から幅に変換するマクロ関数。r=モータ角度値(0.1度単位)、p=爪の取り付け位置（0～3）
#define	RAD_TO_WIDTH(r,p)	( ((double)-r/(WIDTH_RATE*10.0))*WIDTH_RANGE + GROW_W*(p+1))
//幅からサーボ角度に変換するマクロ関数。w=爪の幅(mm)、p=爪の取り付け位置（0～3）
#define	WIDTH_TO_RAD(w,p)	(short) (-(w-GROW_W*(p+1))/WIDTH_RANGE*WIDTH_RATE*10.0)

//アームのモータの可動範囲（0.1度単位）
#define	ARM_RAD_RANGE	(1350)
//ハンド開閉軸のモータの可動範囲（0.1度単位）
#define	HAND_WIDTH_RANGE	(350)

/*----	座標系に関する設定	----*/
//本体の角度
#define	BASE_ANGLE	(180.0)
//本体の位置(mm)
#define	BASE_OFFSET_X	(+0.0)
#define	BASE_OFFSET_Y	(+0.0)

//現在のハンド軸の爪のねじ穴番号（0～3 = ①～④）
#define	CROW_POS	(1)

/*----	その他必要な設定	----*/
//モータの目標位置への移動に対する遷移時間(msec)
#define	MOVE_TIME	(1000)

//モータの現在位置をラジアン角に変換（※モータの角度は0.1度）
#define	SVPOS_TO_RAD(p)	(((double)(p)/1800.0)*M_PI)

class Vsasr {
public:

	int servoNum;		//現在接続されているモータの数（3軸版=3、5軸版=5）
	unsigned int numDevice;//DWORD numDevice=0;
	HID_UART_DEVICE dev;	//通信ハンドル

	/**
	 * コンストラクタ
	 */
	Vsasr(int servo_num);

	/**
	 * デストラクタ
	 */
	~Vsasr();

	/**
	 * アーム先端の目標位置を設定し、その位置へ動かす
	 * @param tx 目標X座標
	 * @param ty 目標Y座標
	 * @param tz 目標Z座標
	 * @param tyaw 目標Yaw(サーボ4の回転角）
	 * @param tw 目標width(ネイルの開き幅)
	 * @param ttime_ms 遷移時間[ms]
	 */
	void setTargetPosition(double tx, double ty, double tz, double tyaw, double tw, unsigned short ttime_ms);

	/**
	 * 任意のサーボの角度を取得する
	 * @param servo_id サーボのID（１〜５）
	 */
	short getAngle(unsigned char servo_id);

	/**
	 * アーム先端の位置を取得する
	 * @param x
	 * @param y
	 * @param z
	 * @param yaw
	 * @param width
	 */
	void getPosition(double &pos_x, double &pos_y, double &pos_z, double &ang_yaw, double &width);

	/**
	 * サーボの数（３or５）を取得する
	 */
	int getServoNum();

	/**
	 * サーボのゲインのON/OFFを設定する
	 * @param on true:on, false:off
	 */
	void setGain(bool on);

private:

	/**
	 * ロボットとの接続・初期化(ゲインのONは行わないことに注意)
	 * @param servo_num
	 */
	void initialize(int servo_num);

	/**
	 * ロボットとの接続解除・ゲインOFF
	 * @param servo_num
	 */
	void finalize();

	int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num);
	int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
	int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num);
	int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len);
	int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num);
	void rad_to_pos(double *x,double *y, double *z ,double *yaw, double *w ,short *sPos,int num);
	void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num);
	int SetTXOpenDrain(HID_UART_DEVICE dev );

	/**
	 * ctrl + c で終了した時の処理
	 * @param sig
	 */
	void sigintFunc(int sig);
};

} // namespace vsasr

#endif /* SCARA_VSASR_SAMPLES_SRC_VSASR_H_ */
