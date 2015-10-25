#include <stdio.h>
#include <float.h>

#include <queue>
#include <algorithm>

#include "MazeSolver_conf.h"
#include "ShortestPath.h"

int ShortestPath::calcShortestDistancePath(const IndexVec &start, const IndexVec &goal, bool onlyUseFoundWall)
{
	std::list<IndexVec> goalList;
	goalList.push_back(goal);
	return calcShortestDistancePath(start, goalList,onlyUseFoundWall);
}

//Dijkstra's algorithm
int ShortestPath::calcShortestDistancePath(const IndexVec &start, const std::list<IndexVec> &goalList, bool onlyUseFoundWall)
{
	for (int i=0;i<MAZE_SIZE;i++) {
		for (int j=0;j<MAZE_SIZE;j++) {
			node[i][j].index.x = j;
			node[i][j].index.y = i;
			node[i][j].from = 0;
			node[i][j].minCost = 0;
		}
	}

	shortestDistancePath.clear();
	node[start.y][start.x].minCost = 1;
	std::priority_queue<Node*, std::vector<Node*>, Node::PointerLess> q;
	std::list<IndexVec> indexList;

	q.push(&node[start.y][start.x]);
	indexList.push_back(start);

	while (!q.empty()) {
		Node* doneNode = q.top(); q.pop();

		//取り出したdoneNodeにあたるindexListの要素を消す
		auto it = std::find(indexList.begin(), indexList.end(), doneNode->index);
		if (it != indexList.end()) {
			it = indexList.erase(it);
		}

		for (int i=0;i<4;i++) {
			const IndexVec cur = doneNode->index;
			//壁がある
			if (maze->wall[cur.y][cur.x][i]) continue;
			//未探索の壁がある
			if (onlyUseFoundWall && !maze->wall[cur.y][cur.x][i+4]) continue;

			if (!cur.canSum(IndexVec::vecDir[i])) continue;
			IndexVec toIndex(cur + IndexVec::vecDir[i]);


			const int16_t cost = doneNode->minCost + 1;
			Node* toNode = &node[toIndex.y][toIndex.x];
			if (toNode->minCost == 0  || cost < toNode->minCost) {
				toNode->minCost = cost;
				toNode->from = Direction(0x01<<i);
				if (std::find(indexList.begin(), indexList.end(), toIndex) == indexList.end()) {
					q.push(toNode);
					indexList.push_back(toIndex);
				}
			}
		}
	}

	IndexVec nearestGoal;
	uint8_t minCost = 0xff;
	for (auto &goal : goalList) {
		if (node[goal.y][goal.x].from != 0 && node[goal.y][goal.x].minCost < minCost) {
			minCost = node[goal.y][goal.x].minCost;
			nearestGoal = goal;
		}
	}
	//ゴールまで行けなかった
	if (minCost == 0xff) {
		return false;
	}

	needToSearchWallIndex.clear();
	IndexVec index(nearestGoal);
	while (1) {
		shortestDistancePath.push_back(index);
		if (index == start) break;
		Direction dir = node[index.y][index.x].from;

		for (int i=0;i<4;i++) {
			if (dir[i]) {
				index += IndexVec::vecDir[(i+2)%4];
				continue;
			}
		}
	}

	std::reverse(shortestDistancePath.begin(), shortestDistancePath.end());

	if(k_shortestDistancePath.empty()) k_shortestDistancePath.push_back(shortestDistancePath);

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
	//mazeを一旦退避
	//書き換えるようにあたらしいものをつくって差し替える
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

			if (maze->wall[spurNode.y][spurNode.x].nWall() > 1) continue;

			Path rootPath(k_shortestDistancePath[k-1].begin(), k_shortestDistancePath[k-1].begin()+i+1);

			for (Path &p :k_shortestDistancePath) {
				if (matchPath(p, rootPath, rootPath.size())) {
					if (maze->wall[p[i].y][p[i].x].nWall() > 1) continue;
					//i+1とiを結ぶノードを切断
					removeEdge(p[i], p[i+1]);
				}
			}

			//spurNodeを残して、それまでのrootPath上のNodeを削除する
			//直後のspurNode->ゴールまでの最短経路の計算で無駄な経路を含まないため
			for (IndexVec &rootPathNode : rootPath) {
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

		B.sort( [](const Path &x, const Path &y){return x.size() < y.size();} );

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

		k_shortestDistancePath.push_back(B.front());
		B.pop_front();
	}

	maze = tmpMaze;

	return k_shortestDistancePath.size();
}

const std::list<Operation> ShortestPath::convertOperationList(const Path &path)
{
	std::vector<Operation> opList;
	std::list<Operation> result;

	int8_t robotDir = 0;
	for (size_t i=0;i<path.size()-1; i++) {
		IndexVec dxdy = path[i+1] - path[i];
		int8_t dir;
		for (int j=0;j<4;j++) {
			if (dxdy == IndexVec::vecDir[j]) {
				dir = j;
			}
		}

		int8_t dirDiff = dir - robotDir;
		if (dirDiff == 0) {
			opList.push_back(Operation(Operation::FORWARD));
		}
		else if (dirDiff == 1 || dirDiff == -3) {
			opList.push_back(Operation(Operation::TURN_RIGHT90));
			opList.push_back(Operation::FORWARD);
		}
		else if (dirDiff == -1 || dirDiff == 3) {
			opList.push_back(Operation(Operation::TURN_LEFT90));
			opList.push_back(Operation(Operation::FORWARD));
		}
		else {
			while(1);
		}
		robotDir = dir;
	}

	//FORWARDの圧縮
	result.push_back(opList[0].op);
	for (size_t i=1;i<opList.size();i++) {
		if (opList[i].op == Operation::FORWARD && opList[i].op == result.back().op) {
			result.back().n++;
		}
		else {
			result.push_back(opList[i].op);
		}
	}

	return result;

}

float ShortestPath::evalOperationList(const std::list<Operation> &actionList)
{
	float cost = 0.0;
	for (auto &operation : actionList) {
		if (operation.op == Operation::FORWARD) {
			if (operation.n == 1) cost += 1.0;
			else if (operation.n == 2) cost += 1.8;
			else if (operation.n == 3) cost += 2.5;
			else if (operation.n == 4) cost += 3;
			else if (operation.n == 5) cost += 3.5;
			else if (operation.n >= 6) cost += 0.5 * (operation.n-6) + 4;
		}
		else if (operation.op == Operation::TURN_LEFT90) {
			cost += 2;
		}
		else if (operation.op == Operation::TURN_RIGHT90) {
			cost += 2;
		}
	}

	return cost;
}

//TODO:未探索壁を使わない場合がおかしい?
//TODO:コストの計算 or kShortestPathの列挙がおかしい
int ShortestPath::calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall)
{
	std::list<IndexVec> goalList;
	goalList.push_back(goal);
	return calcShortestTimePath(start, goalList, k, onlyUseFoundWall);
}

int ShortestPath::calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall)
{
	/*
	calcShortestPath(start,goal,false);
	auto actions = convertActionList(shortestPath);
	for (auto x : actions) {
		if (x.action == ShortestPath::FORWARD) printf("F");
		if (x.action == ShortestPath::TURN_LEFT90) printf("L");
		if (x.action == ShortestPath::TURN_RIGHT90) printf("R");
		printf("%d ",x.n);
	}
	printf("\n");
	printf("cost %f\n", evalActionList(actions));
	 */


	if (calcKShortestDistancePath(start, goalList, k, onlyUseFoundWall) == 0) return false;
	std::vector<uint32_t> costs;

	float minCost = FLT_MAX;
	for (int i=k_shortestDistancePath.size()-1;i>=0;i--) {
		const int length = k_shortestDistancePath[i].size();
		const float cost = evalOperationList(convertOperationList(k_shortestDistancePath[i]));
		printf("dist %d\tcost %f\n", length, cost);
		if (cost < minCost) {
			minCost = cost;
			shortestTimePath_index = i;
		}
	}

	//TODO:Operationのリストをつくる
	auto opList = convertOperationList(k_shortestDistancePath[shortestTimePath_index]);
	shortestTimePath_operationList.clear();
	shortestTimePath_operationList.assign(opList.begin(),opList.end());
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
					if (!maze->wall[path[i].y][path[i].x][j+4]) {
						needToSearchWallIndex.push_back(path[i]);
					}
				}
			}
		}
	}
	needToSearchWallIndex.unique();
}
