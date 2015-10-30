#include <stdio.h>
#include <float.h>
#include <math.h>
#include <utility>

#include <stack>
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
			node[i][j].from = 0;
			node[i][j].minCost = 0;
		}
	}


	shortestDistancePath.clear();
	node[start.y][start.x].minCost = 1;

	//priority queueじゃなくてlistを使う
	//Node*の値はpushしたあとに勝手に変わる可能性があるため
	std::list<Node*> q;

	q.push_back(&node[start.y][start.x]);

	while (!q.empty()) {
		//最大のものをイテレータで取り出して、取り出したら消す
		auto maxCost_it = q.begin();
		for (auto it = q.begin(); it !=q.end(); it++) {
			if ((*it)->minCost > (*maxCost_it)->minCost) {
				maxCost_it = it;
			}
		}
		const Node* doneNode = *maxCost_it;
		q.erase(maxCost_it);

		const IndexVec cur = doneNode->index;


		printf("cur %d %d\n",cur.x,cur.y);
		uint8_t field_cost[MAZE_SIZE][MAZE_SIZE] = {0};
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				field_cost[i][j] = node[i][j].minCost;
			}
		}
		maze->printWall(field_cost);


		for (int i=0;i<4;i++) {
			//壁がある
			if (maze->getWall(cur)[i]) continue;
			//未探索の壁がある
			if (onlyUseFoundWall && !maze->getWall(cur)[i+4]) continue;

			if (!cur.canSum(IndexVec::vecDir[i])) continue;
			IndexVec toIndex(cur + IndexVec::vecDir[i]);

			if (maze->getWall(toIndex).nWall() == 3) {
				//枝の末端部分
				//コストは書き入れるべきだが、qにはいれない
				continue;
			}

			//道が分岐しない、1本道の部分は分岐点に到達するまでcostを書き込みつつたどる
			int16_t cost = doneNode->minCost + 1;
			int last_j = i;
			IndexVec prevIndex = cur;
			std::stack<IndexVec> indexList;
			indexList.push(cur);
			indexList.push(toIndex);


			//次の分岐点 or 枝の末端までtoIndexとcostをすすめる
			//TODO:なんかゴールのところでループする
			bool flag = false;
			while (maze->getWall(toIndex).nWall() == 2 ) {
				if (flag) break;
				if (maze->getWall(toIndex).nWall() == 3) {
					//枝の末端部分
					//コストは書き入れるべきだが、qにはいれない
					break;
				}

				auto isGoal = std::find(goalList.begin(), goalList.end(), toIndex);
				if (isGoal != goalList.end()) {
					break;
				}

				for (int j=0;j<4;j++) {
					//TODO:canSumではなく、一旦足してみてから範囲内に収まっているかをチェックしたほうがはやそう
					if (maze->getWall(toIndex)[j]) continue;
					if (onlyUseFoundWall && !maze->getWall(toIndex)[i+4]) continue;

					if (toIndex.canSum(IndexVec::vecDir[j])) {
						//来た道をもどらないようにする
						if (toIndex + IndexVec::vecDir[j] != prevIndex) {
							if (!maze->getWall(toIndex)[i+4]) flag = true;
							prevIndex = toIndex;
							toIndex += IndexVec::vecDir[j];
							indexList.push(toIndex);
							cost++;
							last_j = j;
							break;
						}
					}
				}
			}


			Node* toNode = &node[toIndex.y][toIndex.x];
			//cost < だと同じ距離の道が列挙されなくなったからcost <= にした
			if (toNode->minCost == 0  || cost <= toNode->minCost) {
				toNode->minCost = cost;
				toNode->from = Direction(0x01<<last_j);


				//ここでcurからtoIndexまでのノードにcostとfromを書き入れていく
				IndexVec prevIndex = indexList.top(); indexList.pop();
				while (!indexList.empty()) {
					const IndexVec curIndex = indexList.top(); indexList.pop();
					const IndexVec diff = prevIndex - curIndex;
					Direction fromDir;
					for (int j=0;j<4;j++) {
						if (IndexVec::vecDir[j] == diff) {
							fromDir = Direction(0x01<<j);
							break;
						}
					}
					node[prevIndex.y][prevIndex.x].from = fromDir;
					node[prevIndex.y][prevIndex.x].minCost = cost;
					cost--;

					prevIndex = curIndex;
				}

				//toNodeがqの中にまだないものだった場合のみqに入れる
				bool foundInList = false;
				for (auto &x : q) {
					if (x == toNode) {
						foundInList = true;
						break;
					}
				}
				if (!foundInList) {
					q.push_back(toNode);
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
				break;
			}
		}
	}

	std::reverse(shortestDistancePath.begin(), shortestDistancePath.end());

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

const std::list<Operation> ShortestPath::convertOperationList(const Path &path)
{
	std::vector<Operation> opList;
	std::list<Operation> result;

	int8_t robotDir = 0;
	for (size_t i=0;i<path.size()-1; i++) {
		const IndexVec dxdy = path[i+1] - path[i];
		int8_t dir = 0;
		for (int j=0;j<4;j++) {
			if (dxdy == IndexVec::vecDir[j]) {
				dir = j;
			}
		}

		const int8_t dirDiff = dir - robotDir;
		if (dirDiff == 0) {
			opList.push_back(Operation(Operation::FORWARD));
		}
		else if (dirDiff == 1 || dirDiff == -3) {
			opList.push_back(Operation(Operation::TURN_RIGHT90));
		}
		else if (dirDiff == -1 || dirDiff == 3) {
			opList.push_back(Operation(Operation::TURN_LEFT90));
		}
		//おかしい
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

	return std::move(result);

}

float ShortestPath::evalOperationList(const std::list<Operation> &actionList)
{
	float cost = 0.0;
	for (auto &operation : actionList) {
		if (operation.op == Operation::FORWARD) {
			//「直線は速度が台形になるように加速する」と仮定してコストを計算
			const float distance = operation.n * MAZE_1BLOCK_LENGTH;
			const float accelDistance = (MAX_VELOCITY*MAX_VELOCITY - MIN_VELOCITY*MIN_VELOCITY) / (2*ACCELERATION);

			if (distance > 2*accelDistance) {
				cost += (distance - 2*accelDistance)/MAX_VELOCITY + 2*( (MAX_VELOCITY-MIN_VELOCITY)/ACCELERATION);
			}
			else {
				const float rt = sqrt(MIN_VELOCITY*MIN_VELOCITY + 2*ACCELERATION*distance/2);
				cost += 2*( (-MIN_VELOCITY + rt)/ACCELERATION );
			}
		}
		else if (operation.op == Operation::TURN_LEFT90 || operation.op == Operation::TURN_RIGHT90) {
			cost += TURN90_TIME;
		}
	}

	return cost;
}


int ShortestPath::calcShortestTimePath(const IndexVec &start, const IndexVec &goal, int k, bool onlyUseFoundWall)
{
	std::list<IndexVec> goalList;
	goalList.push_back(goal);
	return calcShortestTimePath(start, goalList, k, onlyUseFoundWall);
}

int ShortestPath::calcShortestTimePath(const IndexVec &start, const std::list<IndexVec> &goalList, int k, bool onlyUseFoundWall)
{
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

	//Operationのリストをつくる
	auto opList = convertOperationList(k_shortestDistancePath[shortestTimePath_index]);
	shortestTimePath_operationList.clear();
	shortestTimePath_operationList.assign(opList.begin(),opList.end());

	//デバッグ用
	for (auto operation : shortestTimePath_operationList) {
		if (operation.op == Operation::FORWARD) printf("F");
		if (operation.op == Operation::TURN_LEFT90) printf("L");
		if (operation.op == Operation::TURN_RIGHT90) printf("R");
		printf("%d ",operation.n);
	}
	printf("\n");

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
