#include <stdio.h>
#include <algorithm>

#include "MazeSolver_conf.h"
#include "Agent.h"


void Agent::reset()
{
	state = Agent::IDLE;
	path.clear();
	distIndexList.clear();
}

Direction Agent::calcNextDirection(const IndexVec &cur, const IndexVec &_dist)
{
	maze->updateStepMap(_dist);
	const uint8_t curStep = maze->getStepMap(cur);
	if (curStep == 255) return Direction(0);

	Direction result(0);
	int nFoundWall = 10;
	const Direction cur_wall = maze->getWall(cur);
	for (int i=0;i<4;i++) {
		if (cur.canSum(IndexVec::vecDir[i])) {
			IndexVec neighbor(cur + IndexVec::vecDir[i]);
			if (!cur_wall[i] && maze->getStepMap(neighbor) < curStep ) {
				//北優先
				//return Direction(NORTH << i);
				//未探索の壁優先
				if (nFoundWall > maze->getWall(neighbor).nDoneWall()) {
					nFoundWall = maze->getWall(neighbor).nDoneWall();
					result = Direction(NORTH << i);
				}
			}
		}
	}
	if (result) return result;

	for (int i=0;i<4;i++) {
		if (cur.canSum(IndexVec::vecDir[i])) {
			IndexVec neighbor(cur + IndexVec::vecDir[i]);
			if (!cur_wall[i] && maze->getStepMap(neighbor) == curStep ) {
				//北優先
				//return Direction(NORTH << i);
				//未探索壁優先
				if (nFoundWall > maze->getWall(neighbor).nDoneWall()) {
					nFoundWall = maze->getWall(neighbor).nDoneWall();
					result = Direction(NORTH << i);
				}
			}
		}
	}

	return result;
}


void Agent::update(const IndexVec &cur, const Direction &cur_wall)
{
	maze->updateWall(cur, cur_wall);

	if (state == Agent::IDLE) {
		distIndexList.clear();
		distIndexList.assign(mazeGoalList.begin(), mazeGoalList.end());

		dist = distIndexList.front();
		state = Agent::SEARCHING_NOT_GOAL;
	}


	if (state == Agent::SEARCHING_NOT_GOAL) {
		for (auto it = distIndexList.begin();it!=distIndexList.end();) {
			if (*it == cur){
				it = distIndexList.erase(it);
				continue;
			}
			it++;
		}
		if (distIndexList.empty()) {
			state = Agent::SEARCHING_REACHED_GOAL;
		}
		else dist = distIndexList.front();
	}


	if (state == Agent::SEARCHING_REACHED_GOAL) {
		//distでは無いが、distIndexListの一部を通過したからそれをリストから削除する
		//TODO:distIndexListに含まれる壁情報が更新されることになるが、distIndexListは更新されない dist==curでなく、cur in distを条件にすべき
		for (auto it = distIndexList.begin();it!=distIndexList.end();) {
			if (*it == cur){
				it = distIndexList.erase(it);
				continue;
			}
			it++;
		}
		//TODO:到達不可能な壁がdistIndexListに入りっぱなしになっている可能性
		//TODO:到達不可能な壁が連続で出てくる可能性 詰みマスに囲まれた場合はどうなるのだろう
		if (dist == cur || calcNextDirection(cur, dist) == 0) {
			distIndexList.clear();
			path.calcKShortestDistancePath(IndexVec(0,0), mazeGoalList,SEARCH_DEPTH1, false);
			path.calcNeedToSearchWallIndex();
			distIndexList.assign(path.getNeedToSearchIndex().begin(), path.getNeedToSearchIndex().end());
			if (distIndexList.empty()) {
				distIndexList.clear();
				distIndexList.push_back(IndexVec(0,0));
				dist = distIndexList.front();
				state = Agent::BACK_TO_START;
			}
			else {
				maze->updateStepMap(dist);
				distIndexList.sort(
						[&](const IndexVec& lhs, const IndexVec& rhs)
						{
					const unsigned curStep = maze->getStepMap(cur);
					const unsigned lhsStep = maze->getStepMap(lhs);
					const unsigned rhsStep = maze->getStepMap(rhs);
					return (lhsStep - curStep) < (rhsStep - curStep);
						}
				);
				distIndexList.unique();
				dist = distIndexList.front();
			}
		}
	}


	if (state == Agent::BACK_TO_START) {
		if (dist == cur) {
			state = Agent::FINISHED;
			nextDir = 0;

			//最終的に走る最短経路を計算
			path.calcShortestTimePath(IndexVec(0,0), mazeGoalList, SEARCH_DEPTH2, true);
			return;
		}
		nextDir = calcNextDirection(cur, dist);
	}


	if (state == Agent::FINISHED) {

	}


	nextDir = calcNextDirection(cur, dist);
}
