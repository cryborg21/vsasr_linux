/**********************************
	アカデミック スカラロボット C言語サンプル	【直交座標変換】
	Linux版
	2016.08.28
	@cryborg21
  This code includes the work that is distributed in
  https://www.vstone.co.jp/products/scara_robot/download.html#03-3
**********************************/

/*--------------	ヘッダファイル	----------------*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

//CP2110/4 HID USB-to-UART インターフェースライブラリのインクルードファイル
#include <silabs/SLABCP2110.h>
#include <silabs/SLABHIDtoUART.h>
#include <silabs/CP2114_Common.h>

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


/*-------------- 関数のプロトタイプ宣言 ---------------*/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num);
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num);
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len);
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num);
void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num);
int SetTXOpenDrain(HID_UART_DEVICE dev );
void sigintFunc(int sig);

int servoNum = 0;		//現在接続されているモータの数（3軸版=3、5軸版=5）
unsigned int numDevice=0;//DWORD numDevice=0;
HID_UART_DEVICE dev=0;	//通信ハンドル

/*--------------		処理			 ---------------*/
int main()
{

	printf("SCARA-Robot sample 'position to motor-axis-param'.\n");

	// ctrl+c で終了する際の処理
	signal(SIGINT, sigintFunc);

	//現在PCに接続されているロボットの数を取得
	HidUart_GetNumDevices(&numDevice,VID,PID);
	printf("%d device(s) found.\n",numDevice);

	//もし1台も接続されていなかったらプログラムを終了
	if(numDevice==0) return 0;


	//1台目のロボットに接続
	if(HidUart_Open(&dev,0,VID,PID)!=HID_UART_SUCCESS){
		//もし接続が正しく行われなかったら終了
		printf("cannot open device\n");
		return 1;
	}
	else{

		//TX を Open-Drain に変更
		SetTXOpenDrain(dev);


		//軸数を設定。コンソールで数値（3か5）を入力
		do{
			printf("How many motors?(3 or 5):");
			scanf("%d",&servoNum);
			if(servoNum!=3 && servoNum!=5) printf("Does not correspond to %d motors.\n", servoNum);
			else break;
		}while(1);

		printf("%d-axis type\n",servoNum);

		//描画処理の開始
		//ゲインをON
		printf("Gain ON.\n");
		RSTorqueOnOff(dev,1,1,servoNum);

		//メイン処理を開始
		while(1){
			short sPos[2];
			double tx,ty,tz,tyaw,tw;	//目的座標を取得する変数
			int sign=-1;

			//移動先の座標を入力
			printf("Input move positon x (ex.-30):");
			scanf("%lf",&tx);
			printf("Input move positon y (ex. 50):");
			scanf("%lf",&ty);
			printf("Input move positon z (ex. 10):");
			scanf("%lf",&tz);
			if(servoNum>3){
				printf("Input move positon yaw (ex. 90):");
				scanf("%lf",&tyaw);
				printf("Input move positon width (ex. 25):");
				scanf("%lf",&tw);
			}

			//座標を元に、アーム軸（ID1,ID2）の角度を求める
			pos_to_rad(tx,ty,tz,tyaw,tw,sPos,sign,servoNum);

			if(servoNum>3) printf("  move to (%lf,%lf,%lf,%lf,%lf)\n",tx,ty,tz,tyaw,tw);
			else printf("  move to (%lf,%lf,%lf)\n",tx,ty,tz);

			for(int i=0;i<servoNum;i++){
				printf("ID%d=%+d ",i,sPos[i]);
			}
			printf("\n");

			//得られた角度にアームを動かす
			RSMove(dev,sPos,MOVE_TIME/10,1,servoNum);
			sleep(1); // sleep 1 sec

		}

		//ゲインをOFF
		RSTorqueOnOff(dev,0,1,servoNum);
	}


	//ロボットとの通信を切断
	printf("close device\n");
	HidUart_Close(dev);

	return 0;
}

/* ctrl + c で終了した時の処理 */
void sigintFunc(int sig)
{
	//ゲインをOFF
	RSTorqueOnOff(dev,0,1,servoNum);
	printf("Gain OFF.\n");

	//ロボットとの通信を切断
	printf("close device\n");
	HidUart_Close(dev);

	exit(0);
}


/************************************************

	void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num)

	概要：与えられた直交座標（X/Y/Z）とハンド軸の角度・開閉幅を、モータの目標位置に変換する関数。

	引数：
		double x,y,z	… 変換元のX/Y/Z座標
		double yaw		… 変換元のハンド回転軸の角度
		double w		… 変換元のハンド開閉軸の幅
		short sPos		… 計算後のモータ角度を代入する配列変数へのポインタ
		int sign		… アームの折れ曲がる向き。正の値だと時計回り、負の値だと反時計回りに折れ曲がる
		int num			… 使用するモータ数

	戻り値：
		無し

*************************************************/
void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num)
{

	double a,b,tx,ty,lx,ly;		//余弦定理を求める変数
	double thete1,thete2;		//ID1、ID2の角度を求める変数
	double s;					//アームの折れ曲がる向き（符号）に使用する変数。int signの符号に応じて+1.0か-1.0を代入する

	//引数の符号からアームの折れ曲がる向きを決定
	if(sign<0) s=-1.0;
	else s=1.0;

	//本体の位置と角度の設定を反映させる（変換式を逆転させて、向き・位置共に0の位置を基準にした値に変換）
	lx= x-BASE_OFFSET_X;
	ly= y-BASE_OFFSET_Y;
	x = (lx*cos(-BASE_ANGLE/180.0*M_PI)-ly*sin(-BASE_ANGLE/180.0*M_PI));
	y = (lx*sin(-BASE_ANGLE/180.0*M_PI)+ly*cos(-BASE_ANGLE/180.0*M_PI));

	ty = y;
	tx = x+X_OFS;


	//もしリンク長より目標値が遠ければ、根元の軸を目標値までの角度に合わせて、腕を伸ばす
	if(hypot(tx,ty)>=AXISLEN_A+AXISLEN_B){

		sPos[0] = (short) (atan2(ty,tx)/M_PI*1800.f);
		sPos[1] = 0;

	}
	else{

		//余弦定理を演算
		a = acos( (-(tx*tx+ty*ty) + AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B) / (2 * AXISLEN_A * AXISLEN_B));
		b = s*acos( (- AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B + (tx*tx+ty*ty)) / (2 * AXISLEN_A * sqrt(tx*tx+ty*ty)));

		//指定のX/Y座標の符号によって、角度の計算方法を選択
		if(tx<0){
			if(ty>=0) thete1 = M_PI + atan(ty/tx) +b;
			else thete1 = atan(ty/tx) + b - M_PI;
		}
		else thete1 = atan(ty/tx)+b;
		thete2 = -(M_PI-a)*s;


		//求まった角度を360度法に変換
		if(thete1!=0.0) thete1 = thete1 / M_PI * 180.0;
		if(thete2!=0.0) thete2 = thete2 / M_PI * 180.0;

		//変換した角度を0.1度単位に変換し、モータの出力方向に合わせて符号を変更（ID1）
		sPos[0] = (short) (thete1*10.0);
		sPos[1] = (short) (thete2*10.0);


	}
	//アームの2軸の可動範囲制限
	if(sPos[0]<-ARM_RAD_RANGE) sPos[0]=-ARM_RAD_RANGE;
	else if(sPos[0]>ARM_RAD_RANGE) sPos[0]=ARM_RAD_RANGE;

	if(sPos[1]<-ARM_RAD_RANGE) sPos[1]=-ARM_RAD_RANGE;
	else if(sPos[1]>ARM_RAD_RANGE) sPos[1]=ARM_RAD_RANGE;

	//Z座標の変換
	sPos[2] = HEIGHT_TO_RAD(z);

	//ハンドの2軸の変換
	if(num>3){
		//グリップ幅
		sPos[4] = WIDTH_TO_RAD(w,CROW_POS);

		if(sPos[4]<-HAND_WIDTH_RANGE) sPos[4]=-HAND_WIDTH_RANGE;
		else if(sPos[4]>HAND_WIDTH_RANGE) sPos[4]=HAND_WIDTH_RANGE;

		sPos[3] = -sPos[0]-sPos[1];
		sPos[3] += (short) (yaw*10.0);
	}

}



/************************************************

	int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)

	概要：モータのトルクを切り替える関数。IDが連続した複数のモータを同時に設定可能。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		short sMode			… ゲインのON/OFFを指定。0=off、1=ON
		BYTE id				… 切り替えを開始するモータのID（複数のモータを切り替える場合、先頭のモータのID）
		int num				… 切り替えを行うモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
{
	unsigned char	sendbuf[256],*bufp;				//送信バッファ関係
	unsigned char	sum;							//チェックサム計算用
	int				ret;							//戻り値記録用
	unsigned int			data_len=0,len=0;				//パケット長と書き込みサイズ取得用
	unsigned char	i;


	//送信バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	//パケット作成
	//1．ヘッダ・共通部分の作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)0x00;				// サーボID(常に0)
	sendbuf[3]  = (unsigned char)0x00;				// フラグ(常に0)
	sendbuf[4]  = (unsigned char)0x24;				// アドレス(トルクON/OFF 0x24=36)
	sendbuf[5]  = (unsigned char)0x01+1;			// 長さ(1byte)
	sendbuf[6]  = (unsigned char)num;				// モータの個数

	//共通部分のパケット長を記録
	data_len = 7;

	//2．サーボ個別部分のパケット作成
	bufp = &sendbuf[7];								// 送信バッファの個別メッセージ部分の開始アドレスを代入

	//書き換えるモータの個数分だけ、個別のパケットを追加
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータのID
		data_len++;									//パケット長を1byte加算

		*bufp++ = (unsigned char)(sMode&0x00FF);	//ON・OFFフラグ
		data_len++;									//パケット長を1byte加算
	}

	//3．チェックサムの計算
	//チェックサムは、送信バッファの3byte目(サーボID)～終端を1byteずつXORした値です。
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 求まったチェックサムを送信バッファの最後に代入
	data_len++;										//パケット長を1byte加算

	//4．メッセージの送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );

	if(ret!=HID_UART_SUCCESS) return FALSE;

	//5．ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);
}



/************************************************

	int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)

	概要：モータの現在位置を取得する。現在位置の取得は同時に1個のモータしか対応していない

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		BYTE id				… 現在位置を取得するモータのID
		short *getParam		… 取得した現在位置を格納する変数へのポインタ

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
{
	unsigned char	sendbuf[32];
	unsigned char	readbuf[128];
	unsigned char	sum;
	unsigned int i;
	int				ret;
	unsigned int	len, readlen;
	short			angle;

	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)id;				// サーボID
	sendbuf[3]  = (unsigned char)0x0f;				// フラグ(0x0f=指定アドレスから指定バイト取得)
	sendbuf[4]  = (unsigned char)0x2a;				// 取得すつデータのアドレス(現在位置=0x2a)
	sendbuf[5]  = (unsigned char)0x02;				// 取得するデータの長さ(現在位置=2byte)
	sendbuf[6]  = (unsigned char)0x00;				// 個数(0)


	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// チェックサム

	// パケットを送信
	ret = HidUart_Write(dev,sendbuf, 8, &len);

	//ローカルエコーを読み取り
	if(!ReadLocalEcho(dev,sendbuf,len)) return FALSE;

	// もし送信できたパケット長がデータサイズよりも小さい場合、エラー
	if( len < 8 ) return FALSE;


	// 受信バッファの読み込み
	memset( readbuf, 0x00, sizeof( readbuf ));

	//受信バッファのパケット長の計算
	//	Header(2byte) + ID(1byte) + Flags(1byte) + Address(1byte) + Length(1byte) + Count(1byte) + Dada(2byte) + Sum(1byte)
	readlen = (2) + (1) + (1) + (1) + (1) + (1) + (2) + (1);
	len = 0;

	//バッファの受信
	HidUart_Read(dev,readbuf, readlen, &len);

	//受信バッファのパケット長が、計算で求めた長さ(readlen)と異なる場合、エラー
	if( len < readlen)  return FALSE;

	// 受信データのチェックサム確認
	sum = readbuf[2];
	for( i = 3; i < readlen; i++ ){
		sum = sum ^ readbuf[i];
	}

	//チェックサムが異なる場合、エラー
	if( sum ) return FALSE;

	//受信バッファから、読み取った現在位置のデータを取り出す
	angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
	if(getParam) *getParam = angle;

	return TRUE;
}


/************************************************

	int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)

	概要：モータを指定の角度に動かす。IDが連続した複数のモータを一度に動かすことが可能。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		short *sPoss		… 目標位置を記録した配列変数へのポインタ。開始IDの目標値は、バッファの先頭から順に使用される
		unsigned short sTime … 目標位置までの遷移時間(10ミリ秒単位)
		BYTE id				… 動かすモータのID（複数のモータを動かす場合、先頭のモータのID）
		int num				… 動かすモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i;
	int				ret;
	unsigned int	len,data_len;


	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				    // ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				    // ヘッダー2
	sendbuf[2]  = (unsigned char)0;						// ID(0)
	sendbuf[3]  = (unsigned char)0x00;				    // フラグ(0x00)
	sendbuf[4]  = (unsigned char)0x1E;				    // アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04+1;			    // 長さ(4byte)
	sendbuf[6]  = (unsigned char)num;				    // モータの個数

	//共通部分のパケット長を記録
	data_len = 7;

	//個別のデータ作成
	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータID
		data_len++;									//パケット長を1byte加算

		//目標位置をバッファに書き込み(2byte)
		*bufp++ = (unsigned char)(sPoss[i]&0x00FF);
		*bufp++ = (unsigned char)((sPoss[i]&0xFF00)>>8);
		data_len+=2;								//パケット長を2byte加算

		//遷移時間をバッファに書き込み(2byte)
		*bufp++ = (unsigned char)(sTime&0x00FF);
		*bufp++ = (unsigned char)((sTime&0xFF00)>>8);
		data_len+=2;								//パケット長を2byte加算
	}


	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 送信バッファにチェックサムを追加
	data_len++;										//パケット長を1byte加算

	// パケットを送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	//ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);

}


/************************************************

	int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,DWORD data_len)

	概要：
		送信メッセージのローカルエコーを読み出す関数。
		引数 data_len の長さだけメッセージを受信し、 引数 *sendbuf の内容と同一か比較を行う

	引数：
		HID_UART_DEVICE dev		… 通信ハンドル
		unsigned char *sendbuf	… 送信メッセージ内容へのポインタ
		DWORD data_len			… 送信メッセージのサイズ

	戻り値：
		送信メッセージと同一の内容を受信できたらTRUE、そうでなければFALSE

*************************************************/
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len)
{

	unsigned char readbuf[1024];
	unsigned int len=0;
	memset(readbuf,0,sizeof(readbuf));

	//data_len のサイズだけメッセージを受信
	HidUart_Read( dev, (BYTE*) readbuf, data_len, &len );

	//受信メッセージのサイズが異なる場合、エラー
	if(data_len!=len) return FALSE;

	//受信メッセージと送信メッセージを比較
	for(unsigned long i=0;i<len;i++){

		//受信メッセージと送信メッセージが異なる場合、エラー
		if(readbuf[i]!=sendbuf[i]) return FALSE;
	}
	return TRUE;
}




/************************************************

	int int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)

	概要：
		モータのパラメータを書き換える。連続した複数のモータ・複数のアドレスに書き込みが可能
		アドレス・サイズ・モータ数・実データを指定する

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		BYTE address		… データを書き込むアドレス（アドレスの詳細はRS304MDの資料を参照）
		BYTE size			… モータ1個当たりに書き込むデータのサイズ(byte単位)
		BYTE id				… 動かすモータのID（複数のモータを動かす場合、先頭のモータのID）
		BYTE *data			… 書き込みデータ内容の配列変数へのポインタ
		int num				… 動かすモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i,j;
	int				ret;
	unsigned int	len,data_len;


	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				    // ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				    // ヘッダー2
	sendbuf[2]  = (unsigned char)0;						// ID(0)
	sendbuf[3]  = (unsigned char)0x00;				    // フラグ(0x00)
	sendbuf[4]  = (unsigned char)address;				    // アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)size+1;			    // 長さ(4byte)
	sendbuf[6]  = (unsigned char)num;				    // モータの個数

	//共通部分のパケット長を記録
	data_len = 7;

	//個別のデータ作成
	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータID
		data_len++;									//パケット長を1byte加算

		for(j=0;j<size;j++){
			*bufp++ = (unsigned char)data[j];		//データ部を1byteずつパケットに追加
			data_len++;								//パケット長を2byte加算
		}
	}


	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 送信バッファにチェックサムを追加
	data_len++;										//パケット長を1byte加算

	// パケットを送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	//ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);

}

/************************************************

	int SetTXOpenDrain(HID_UART_DEVICE dev )

	概要：
		USB-to-UART（CP2110）のポート設定について、TXをOpen-Drainに設定変更する。
		ロボット本体と通信する際には、必ずこの設定が必要（設定を変更するとロボット本体に記録される）。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル

	戻り値：
		HID_UART_STATUS にて定義されている各数値。HID_UART_SUCCESSであれば成功、それ以外は失敗

*************************************************/
int SetTXOpenDrain(HID_UART_DEVICE dev )
{
	//GPIOの設定で、TX を Open-Drain に設定する必要がある
	BYTE pinConfig[13];
	BOOL useSuspendValues;
	WORD suspendValue;
	WORD suspendMode;
	BYTE rs485Level;
	BYTE clkDiv;

	//現在の Pin Config を取得
	HidUart_GetPinConfig(dev,  pinConfig,  &useSuspendValues, &suspendValue, &suspendMode, &rs485Level, &clkDiv);

	//TX を TX-Open-Drain(0x01) に変更
	pinConfig[10] = 0x01;

	//Pin Config を更新
	return HidUart_SetPinConfig(dev,  pinConfig, useSuspendValues, suspendValue, suspendMode,rs485Level, clkDiv);

}
