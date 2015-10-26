#ifndef SHORTESTPATH_H_
#define SHORTESTPATH_H_


#include <stdint.h>
#include <list>
#include <vector>
#include "Maze.h"

typedef std::vector<IndexVec> Path;



/**************************************************************
 * Operation
 *	ロボットがすべき動作を表現
 *	走行ルートを表現するのに使う
 *	opが動きの種類で、「opをn回実行する」という意味
 **************************************************************/
struct Operation {
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


/**************************************************************
 * ShortestPath
 *	最短経路あたりのアルゴリズム
 *	・Dijkstra'sAlgorithmによる最短経路の計算
 *	・Yes'sAlgorithmによるk最短経路の計算
 *	・ロボットの走行パラメータに基づく経路走行時間の見積もり
 **************************************************************/
class ShortestPath {
private:

	//Dijkstraで計算するときに使う
	//経路のコストを計算する用
	struct __attribute__ ((__packed__)) Node {
		IndexVec index;
		uint8_t minCost;
		Direction from;

		//Node*に対する比較演算子<
		//Dijkstra'sAlgorithmの中のpriority_queueで使う
		struct PointerLess {
			bool operator()(const Node* lhs, const Node* rhs) const
			{ return lhs->minCost < rhs->minCost; }
		};
	};

	Node node[MAZE_SIZE][MAZE_SIZE];

	Maze *maze;

	//色々と計算した経路を保存しとく
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

	//Pathを入れるとOperationのリストを返す
	const std::list<Operation> convertOperationList(const Path &path);
	//Operationのリストから合計コストを計算する
	float evalOperationList(const std::list<Operation> &actionList);

public:
	ShortestPath(Maze &_maze, bool _useDiagonalPath = false)
: maze(&_maze), shortestTimePath_index(-1), useDiagonalPath(_useDiagonalPath)
{
		clear();
}

	void clear() {
		shortestDistancePath.clear();
		needToSearchWallIndex.clear();
		shortestTimePath_index=-1;
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				node[i][j].index.x = j;
				node[i][j].index.y = i;
				node[i][j].from = 0;
				node[i][j].minCost = 0;
			}
		}
	}

	//startからgoalへの最短経路を計算し、shortestDistancePathに格納する
	//goalListを与えた場合、goalListに含まれる座標のうち一番近い座標への道のりを計算する
	//onlyUseFoundWall=trueのとき、未探索壁を通らない経路を生成する
	int calcShortestDistancePath(const IndexVec &start, const IndexVec &goal, bool onlyUseFoundWall);
	int calcShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, bool onlyUseFoundWall);
	inline const Path &getShortestDistancePath() const { return shortestDistancePath; }

	//k最短経路を計算する
	//calcShortestDistancePathと振る舞いは同じ
	//結果はk_shortestDistancePathに格納
	int calcKShortestDistancePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall);
	int calcKShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall);
	inline const std::vector< Path > &getKShortestDistancePath() const { return k_shortestDistancePath; }

	//時間に関して最短(だろう)経路を計算する
	//内部でk_shortestDistancePathを実行し、k個のpathの走行時間を計算する
	//その内で一番コスト(走行時間)が小さいものをShortestTimePathとする
	//最短経路のindex(k_shortestDistancePathの)をshortestTimePath_indexに格納する
	int calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall);
	int calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall);
	inline const Path &getShortestTimePath() const { return k_shortestDistancePath[shortestTimePath_index]; }
	inline const std::vector<Operation> &getShortestTimePathOperation() const { return shortestTimePath_operationList; }

	//kShortestDistancePath上の未探索壁がある座標リストを計算する。
	//この座標が追加で探索すべき座標になる
	//calcKShortestDistancePathを実行してから実行する
	void calcNeedToSearchWallIndex();
	inline const std::list<IndexVec> &getNeedToSearchIndex() const { return needToSearchWallIndex; }
};


#endif /* SHORTESTPATH_H_ */
