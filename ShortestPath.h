#ifndef SHORTESTPATH_H_
#define SHORTESTPATH_H_

#include <list>
#include <vector>

#include "Maze.h"
#include "Operation.h"

typedef std::vector<IndexVec> Path;


/**************************************************************
 * ShortestPath
 *	最短経路あたりのアルゴリズム
 *	・歩数マップによる最短経路の計算
 *	・Yes'sAlgorithmによるk最短経路の計算
 *	・ロボットの走行パラメータに基づく経路走行時間の見積もり
 **************************************************************/
class ShortestPath {
private:
	Maze *maze;

	//色々と計算した経路を保存しとく
	Path shortestDistancePath;
	std::vector< Path > k_shortestDistancePath;
	int shortestTimePath_index;
	OperationList shortestTimePath_operationList;
	float shortestTimePath_cost;
	std::list<IndexVec> needToSearchWallIndex;

	//k shortest pathの関数内で使う
	void removeEdge(const IndexVec& start, const IndexVec& end);
	void removeNode(const IndexVec& node);
	bool matchPath(const Path &path1, const Path &path2, int n);

public:
	ShortestPath(Maze &_maze, bool _useDiagonalPath = false)
: maze(&_maze), shortestTimePath_index(-1)
{
		clear();
}

	void clear() {
		shortestDistancePath.clear();
		needToSearchWallIndex.clear();
		shortestTimePath_index=-1;
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
	int calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall, bool useDiagonalPath);
	int calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall, bool useDiagonalPath);
	inline const Path &getShortestTimePath() const { return k_shortestDistancePath[shortestTimePath_index]; }
	inline const OperationList &getShortestTimePathOperation() const { return shortestTimePath_operationList; }
	inline float getShortestTimePathCost() const { return shortestTimePath_cost; }

	//kShortestDistancePath上の未探索壁がある座標リストを計算する。
	//この座標が追加で探索すべき座標になる
	//calcKShortestDistancePathを実行してから実行する
	void calcNeedToSearchWallIndex();
	inline const std::list<IndexVec> &getNeedToSearchIndex() const { return needToSearchWallIndex; }
};


#endif /* SHORTESTPATH_H_ */
