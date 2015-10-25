#ifndef SHORTESTPATH_H_
#define SHORTESTPATH_H_


#include <stdint.h>
#include <list>
#include <vector>
#include "Maze.h"

typedef std::vector<IndexVec> Path;


//ロボットの動きのシーケンス
//opが動きの種類
//「opをn回実行する」という意味
struct Operation {
	//ロボットの動きを表す
	typedef enum {
		FORWARD,
		TURN_RIGHT90,
		TURN_RIGHT45,
		TURN_LEFT90,
		TURN_LEFT45,
		STOP,
	} OperationType;

	OperationType op;
	uint8_t n;
	Operation(OperationType _op = STOP, uint8_t _n = 1) : op(_op), n(_n) {}
};

class ShortestPath {
private:
	//Dijkstraで計算するときに使う
	struct __attribute__ ((__packed__)) Node {
		IndexVec index;
		uint8_t minCost;
		Direction from;

		struct PointerLess {
			bool operator()(const Node* lhs, const Node* rhs) const
			{ return lhs->minCost < rhs->minCost; }
		};
	};

	Maze *maze;
	Node node[MAZE_SIZE][MAZE_SIZE];
	Path shortestDistancePath;
	std::vector< Path > k_shortestDistancePath;
	int shortestTimePath_index;
	std::vector<Operation> shortestTimePath_operationList;
	std::list<IndexVec> needToSearchWallIndex;
	const bool useDiagonalPath;

	//k shortest pathの関数内で使う
	void removeEdge(const IndexVec& start, const IndexVec& end);
	void removeNode(const IndexVec& node);
	bool matchPath(const Path &path1, const Path &path2, int n);

	//TODO:値で返さない
	//Pathを入れるとOperationのリストを返す
	const std::list<Operation> convertOperationList(const Path &path);
	//Operationのリストから合計コストを計算する
	float evalOperationList(const std::list<Operation> &actionList);

public:
	ShortestPath(Maze &_maze, bool _useDiagonalPath = false)
		: maze(&_maze), shortestTimePath_index(-1), useDiagonalPath(_useDiagonalPath){ }
	void clear() {shortestDistancePath.clear(); needToSearchWallIndex.clear(); shortestTimePath_index=-1;}

	//startからgoalへの最短経路を計算し、shortestDistancePathに格納する
	//goalListを与えた場合、goalListに含まれる座標のうち一番近い座標への道のりを計算する
	//onlyUseFoundWall=trueのとき、未探索壁を通らない経路を生成する
	int calcShortestDistancePath(const IndexVec &start, const IndexVec &goal, bool onlyUseFoundWall);
	int calcShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, bool onlyUseFoundWall);
	const Path &getShortestDistancePath() const { return shortestDistancePath; }

	//k最短経路を計算する
	//calcShortestDistancePathと振る舞いは同じ
	//結果はk_shortestDistancePathに格納
	int calcKShortestDistancePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall);
	int calcKShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall);
	const std::vector< Path > &getKShortestDistancePath() const { return k_shortestDistancePath; }

	//時間に関して最短(だろう)経路を計算する
	//内部でk_shortestDistancePathを実行し、k個のpathの走行時間を計算する
	//その内で一番コスト(走行時間)が小さいものをShortestTimePathとする
	//最短経路のindex(k_shortestDistancePathの)をshortestTimePath_indexに格納する
	int calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall);
	int calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall);
	const Path &getShortestTimePath() const { return k_shortestDistancePath[shortestTimePath_index]; }
	const std::vector<Operation> &getShortestTimePathOperation() const { return shortestTimePath_operationList; }

	//kShortestDistancePath上の未探索壁がある座標リストを計算する。
	//この座標が追加で探索すべき座標になる
	//calcKShortestDistancePathを実行してから実行する
	void calcNeedToSearchWallIndex();
	const std::list<IndexVec> &getNeedToSearchIndex() const { return needToSearchWallIndex; }
};


#endif /* SHORTESTPATH_H_ */
