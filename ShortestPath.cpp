#include <cstdio>
#include <cfloat>
#include <cmath>
#include <utility>
#include <algorithm>

#include "MazeSolver_conf.h"
#include "ShortestPath.h"


int ShortestPath::calcShortestDistancePath(const IndexVec &start, const IndexVec &goal, bool onlyUseFoundWall)
{
	std::list<IndexVec> goalList;
	goalList.push_back(goal);
	return calcShortestDistancePath(start, goalList,onlyUseFoundWall);
}


int ShortestPath::calcShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, bool onlyUseFoundWall)
{
	shortestDistancePath.clear();

	maze->updateStepMap(goalList.front(), onlyUseFoundWall);

	if (maze->getStepMap(start) == 0xff) return false;

	//歩数マップを下る方向に
	IndexVec cur = start;
	while (1) {
		shortestDistancePath.push_back(cur);

		//goalListのどこかにたどり着いたらおわり
		auto it = std::find(goalList.begin(), goalList.end(), cur);
		if (it != goalList.end()) {
			break;
		}

		const uint8_t curStep = maze->getStepMap(cur);
		for (int i=0;i<4;i++) {
			if (maze->getWall(cur)[i]) continue;

			if (cur.canSum(IndexVec::vecDir[i])) {
				const IndexVec neighbor = cur + IndexVec::vecDir[i];
				const uint8_t neighborStep = maze->getStepMap(neighbor);
				if (neighborStep == curStep-1) {
					cur = neighbor;
					break;
				}
			}
		}
	}

	return true;
}

void ShortestPath::removeNode(const IndexVec& node)
{
	maze->updateWall(node, Direction(0xff));
}


void ShortestPath::removeEdge(const IndexVec& start, const IndexVec& end)
{
	const IndexVec dxdy = end - start;
	for (int i=0;i<4;i++) {
		if (dxdy == IndexVec::vecDir[i]) {
			maze->updateWall(start, Direction(0x11<<i));
			break;
		}
	}
}

bool ShortestPath::matchPath(const Path &path1, const Path &path2, int n)
{
	bool result = true;

	for (int i=0;i<n;i++) {
		if (path1[i] != path2[i]) result = false;
	}

	return result;
}

int ShortestPath::calcKShortestDistancePath(const IndexVec &start, const IndexVec &goal, int _k, bool onlyUseFoundWall)
{
	std::list<IndexVec> goalList;
	goalList.push_back(goal);
	return calcKShortestDistancePath(start, goalList, _k, onlyUseFoundWall);
}

//Yen's k shortest path algorithm
int ShortestPath::calcKShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, int _k, bool onlyUseFoundWall)
{
	//k=1の時は最短経路のみを計算しておわり
	if (_k == 1) {
		if (calcShortestDistancePath(start, goalList, onlyUseFoundWall) == 0) return 0;
		k_shortestDistancePath.clear();
		k_shortestDistancePath.push_back(shortestDistancePath);
		return 1;
	}

	//mazeを一旦退避
	//書き換えるようにあたらしいものをつくって差し替える
	//TODO:差し替えるてもちゃんとstepmapが更新されるかチェック
	//TODO:差し替えではなく、変更部分だけを後で修復したほうがはやいと思う
	Maze *tmpMaze = maze;
	Maze newMaze(*tmpMaze);
	maze = &newMaze;

	k_shortestDistancePath.clear();
	std::list< Path > B;

	if (calcShortestDistancePath(start, goalList, onlyUseFoundWall) == 0) return 0;
	k_shortestDistancePath.push_back(shortestDistancePath);


	for (int k=1;k<_k;k++) {
		for (size_t i=0;i<k_shortestDistancePath[k-1].size();i++) {
			IndexVec &spurNode = k_shortestDistancePath[k-1][i];
			IndexVec &spurGoal = k_shortestDistancePath[k-1].back();

			if (maze->getWall(spurNode).nWall() > 1) continue;

			Path rootPath(k_shortestDistancePath[k-1].begin(), k_shortestDistancePath[k-1].begin()+i+1);

			for (const Path &p :k_shortestDistancePath) {
				if (matchPath(p, rootPath, rootPath.size())) {
					if (maze->getWall(p[i]).nWall() > 1) continue;
					//i+1とiを結ぶノードを切断
					removeEdge(p[i], p[i+1]);
				}
			}

			//spurNodeを残して、それまでのrootPath上のNodeを削除する
			//直後のspurNode->ゴールまでの最短経路の計算で無駄な経路を含まないため
			for (const IndexVec &rootPathNode : rootPath) {
				if (rootPathNode == spurNode) continue;
				//rootPathNodeを削除する
				removeNode(rootPathNode);
			}

			//ゴールまでいける場合
			if (calcShortestDistancePath(spurNode, spurGoal, onlyUseFoundWall) != 0) {
				auto &spurPath = shortestDistancePath;
				rootPath.pop_back();
				std::copy(spurPath.begin(),spurPath.end(), std::back_inserter(rootPath));

				//唯一になるようにいれる
				auto rootPath_Pos_inB = std::find(B.begin(), B.end(), rootPath);
				if (rootPath_Pos_inB == B.end()) {
					B.push_back(rootPath);
				}
			}

			//削除したpathとnodeを戻す
			//placement new でもっかいコンストラクタを読んでる
			maze = new(maze) Maze(*tmpMaze);
		}

		//BからAにすでに含まれているものを削除する
		for (auto it=B.begin();it!=B.end();) {
			std::vector<IndexVec> Bitem((*it).begin(),(*it).end());
			if (std::find(k_shortestDistancePath.begin(),k_shortestDistancePath.end(), Bitem) != k_shortestDistancePath.end()) {
				it = B.erase(it);
				continue;
			}
			it++;
		}

		if (B.empty()) break;

		B.sort( [](const Path &x, const Path &y){return x.size() < y.size();} );

		k_shortestDistancePath.push_back(B.front());
		B.pop_front();
	}

	maze = tmpMaze;

	return k_shortestDistancePath.size();
}

int ShortestPath::calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall, bool useDiagonalPath)
{
	std::list<IndexVec> goalList;
	goalList.push_back(goal);
	return calcShortestTimePath(start, goalList, k, onlyUseFoundWall, useDiagonalPath);
}

int ShortestPath::calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall, bool useDiagonalPath)
{
	if (calcKShortestDistancePath(start, goalList, k, onlyUseFoundWall) == 0) return false;
	std::vector<uint32_t> costs;

	float minCost = FLT_MAX;
	OperationList minCost_opList;
	for (int i=k_shortestDistancePath.size()-1;i>=0;i--) {
		minCost_opList.loadFromPath(k_shortestDistancePath[i], useDiagonalPath);
		const float cost = minCost_opList.eval();
		if (cost < minCost) {
			minCost = cost;
			shortestTimePath_operationList = minCost_opList;
			shortestTimePath_index = i;
		}
	}
	shortestTimePath_cost = minCost;

	return true;
}

void ShortestPath::calcNeedToSearchWallIndex()
{
	//K shortest path上の未探索座標を列挙
	needToSearchWallIndex.clear();
	for (auto &path : k_shortestDistancePath) {
		for (size_t i=0;i<path.size()-1;i++) {
			IndexVec dxdy = path[i+1] - path[i];
			for (int j=0;j<4;j++) {
				if (dxdy == IndexVec::vecDir[j]) {
					if (!maze->getWall(path[i])[j+4]) {
						//唯一になるようにいれる
						auto it = std::find(needToSearchWallIndex.begin(), needToSearchWallIndex.end(), path[i]);
						if (it == needToSearchWallIndex.end()) {
							needToSearchWallIndex.push_back(path[i]);
						}
					}
				}
			}
		}
	}
}
