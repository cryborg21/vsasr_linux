/*
 * get_position.cpp
 *
 *  Created on: 2016/07/31
 *      Author: cryborg21
*/

#include "vsasr.h"
#include "stdlib.h"

// ループの終了フラグ
bool quit_flag = false;

// ctrl + c を押した時の処理
void signalHandler(int sig)
{
	quit_flag = true;
}

int main(int argc, char* argv[]) {

	// コマンドライン引数の処理
	// 引数が3もしくは5以外である場合はプログラム終了
	if(argc != 2){
		printf("Please set the num of servo (3 or 5).\n");
		return 0;
  }else	if( !(atoi(argv[1])==3) && !(atoi(argv[1])==5) ){
		printf("Please set the num of servo (3 or 5).\n");
 		return 0;
	}
	// サーボ数
	unsigned int servo_num = atoi(argv[1]);

	// シグナルハンドラの登録
	signal(SIGINT, signalHandler);

	double x = 0;
	double y = 0;
	double z = 0;
	double yaw = 0;
	double width = 0;

	vsasr::Vsasr vsasr(servo_num);
	while(1){
		vsasr.getPosition(x, y, z, yaw, width);
		printf("X:%+6.2fmm, Y:%+6.2fmm, Z:%+6.2fmm", x, y, z);
		if (vsasr.getServoNum() == 5)
			printf(", Yaw:%+6.2fdeg, Width:%6.2fmm", yaw, width);

		printf("\n");
		sleep(0.016); // 0.016sec=16ms sleep -> 60Hz
		if(quit_flag) break;
	}

	return 0;
}
