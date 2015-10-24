#ifndef AGENT_H_
#define AGENT_H_

#include <stdint.h>

#include <list>
#include <vector>

#include "Maze.h"
#include "ShortestPath.h"

/*
 * 探索時にロボットに動きの指示を与える
 * 探索・最短経路の計算において最も上位に位置する
 *
 * 一区画進むごとにupdateを呼び出して今の座標と壁情報を入力していく
 * updateを実行すると次に進むべき方向が計算される
 *
 * 迷路情報は外に保存をするが、Agent::updateを通して更新をしていく
 */
class Agent {
public:
	typedef enum {
		IDLE, 					//まだ実行されていない
		SEARCHING_NOT_GOAL, 	//探索中　まだ一度もゴールに到達していない
		SEARCHING_REACHED_GOAL, //探索中　一度ゴールに到達したが、追加で探索をしている
		BACK_TO_START, 			//スタートに戻っている
		FINISHED 				//スタート地点に到着をし、最短経路の計算が終わった
	} State;

private:
	Maze* maze;
	State state;
	IndexVec dist;
	std::list<IndexVec> distIndexList;
	Direction nextDir;
	ShortestPath path;
	std::list<IndexVec> mazeGoalList;

	Direction calcNextDirection(const IndexVec &cur, const IndexVec &dist);

public:
	Agent(Maze &_maze) :maze(&_maze), state(Agent::IDLE), path(_maze)
	{
		mazeGoalList.push_back(IndexVec(7,7));
		mazeGoalList.push_back(IndexVec(7,8));
		mazeGoalList.push_back(IndexVec(8,7));
		mazeGoalList.push_back(IndexVec(8,8));
	}

	//状態を更新する
	//cur:今の座標
	//cur_wall:今の座標における壁情報(Done bitは無視される)
	void update(const IndexVec &cur, const Direction &cur_wall);

	//現在の状態を返す
	//updateを呼び出したあとは必ずこれを読んで状態を確認する
	const State &getState() const {return state;}

	//次にロボットが動くべき方向を返す
	//0が帰ってきた場合はバグったときか、終了した時(おそらくロボットは停止すべき)
	//今の向きと180度逆方向が出てくる場合もある
	//その場合はおそらく一旦停止して切り返す必要がある
	const Direction &getNextDirection() const {return nextDir;}


	//強制的にゴールに向かわせる
	//探索に時間がかかりすぎている場合につかう(2分たったら呼び出すとか)
	void forceGotoStart() { dist = IndexVec(0,0); state = Agent::BACK_TO_START; }

	const Path &getShortestPath() const {return path.getShortestTimePath();}
	const std::vector<Path> &getKShortestPath() const {return path.getKShortestDistancePath();}
	const IndexVec& getDist() const { return dist; }
	const std::list<IndexVec> &getDistList() const { return distIndexList; }

	const std::vector<Operation> &getRunSequence() const { return path.getShortestTimePathOperation(); }
	//TODO:途中から再開できるようにしたい

	void reset();

};



#endif /* AGENT_H_ */
