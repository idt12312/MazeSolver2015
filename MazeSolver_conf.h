#ifndef MAZESOLVER_CONF_H_
#define MAZESOLVER_CONF_H_

/***************************************
 * 迷路に関するパラメータ
 ***************************************/
//迷路の大きさ
#define MAZE_SIZE 16

//迷路のゴール座標
//ゴールとしたい座標を並べる
//4個でなくてもよい
#define MAZE_GOAL_LIST { IndexVec(7,7), IndexVec(7,8), IndexVec(8,7), IndexVec(8,8) }


/****************************************
 * 探索アルゴリズムに関するパラメータ
 ****************************************/
//一旦ゴールに到達したあとのk最短経路を計算するときのk
#define SEARCH_DEPTH1 1

//探索が終了し、最終的な走行ルートを計算するときのk
#define SEARCH_DEPTH2 20


//経路のコストを計算するときに使うロボットの走行性能
//90度曲がるブロックを進むのにかかる時間[s]
#define TURN90_TIME 	0.3

//45度曲がるブロックを進むのにかかる時間[s]
#define TURN45_TIME 	0.2

//加速度[m/s^2]
#define ACCELERATION 	6.0

//最大速度[m/s]
#define MAX_VELOCITY 	2.0

//最小速度[m/s]
#define MIN_VELOCITY 	0.5

//1区画の長さ[m]
#define MAZE_1BLOCK_LENGTH 	0.18


#endif /* MAZESOLVER_CONF_H_ */
