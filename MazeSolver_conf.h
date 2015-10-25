#ifndef MAZESOLVER_CONF_H_
#define MAZESOLVER_CONF_H_

/***************************************
 * 迷路に関するパラメータ
 ***************************************/
//迷路の大きさ
#define MAZE_SIZE 16

//迷路のゴール座標
#define MAZE_GOAL1 IndexVec(7,7)
#define MAZE_GOAL2 IndexVec(7,8)
#define MAZE_GOAL3 IndexVec(8,7)
#define MAZE_GOAL4 IndexVec(8,8)


/****************************************
 * 探索アルゴリズムに関するパラメータ
 ****************************************/
//一旦ゴールに到達したあとのk最短経路を計算するときのk
#define SEARCH_DEPTH1 2

//探索が終了し、最終的な走行ルートを計算するときのk
#define SEARCH_DEPTH2 20


#endif /* MAZESOLVER_CONF_H_ */
