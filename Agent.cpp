#include <stdio.h>
#include <algorithm>

#include "MazeSolver_conf.h"
#include "Agent.h"


void Agent::reset()
{
	state = Agent::IDLE;
	path.clear();
	distIndexList.clear();

	dist.x = 0;
	dist.y = 0;
	nextDir = 0;

	mazeGoalList.clear();
	mazeGoalList.push_back(MAZE_GOAL1);
	mazeGoalList.push_back(MAZE_GOAL2);
	mazeGoalList.push_back(MAZE_GOAL3);
	mazeGoalList.push_back(MAZE_GOAL4);
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
		//本当はdistLindexListのどこかに到達するたびにdistLindexListを更新したい
		//計算時間的に無理だからたまに更新する感じ
		//distIndexListのどれかに到達した or 目標地点が到達不能だと分かったら
		if (cur == dist || calcNextDirection(cur, dist) == 0) {
			distIndexList.clear();
			path.calcKShortestDistancePath(IndexVec(0,0), mazeGoalList,SEARCH_DEPTH1, false);
			path.calcNeedToSearchWallIndex();
			distIndexList.assign(path.getNeedToSearchIndex().begin(), path.getNeedToSearchIndex().end());
			if (distIndexList.empty()) {
				distIndexList.push_back(IndexVec(0,0));
				state = Agent::BACK_TO_START;
			}
		}
		maze->updateStepMap(cur);

		//distIndexListの中から現在座標に一番近い近いものをdistに入れる
		int minDistance = INT32_MAX;
		std::list<IndexVec>::iterator it_nearestDist;
		for (auto it=distIndexList.begin();it!=distIndexList.end();it++) {
			int stepDiff = maze->getStepMap(cur) - maze->getStepMap(*it);
			if (stepDiff<0) stepDiff = -stepDiff;
			if (stepDiff < minDistance) {
				minDistance = stepDiff;
				it_nearestDist = it;
			}
		}
		dist = *it_nearestDist;
	}


	if (state == Agent::BACK_TO_START) {
		if (dist == cur) {
			state = Agent::FINISHED;
			nextDir = 0;

			return;
		}
		nextDir = calcNextDirection(cur, dist);
	}


	if (state == Agent::FINISHED) {

	}


	nextDir = calcNextDirection(cur, dist);
}

void Agent::caclRunSequence()
{
	if (state != Agent::FINISHED) return ;
	path.calcShortestTimePath(IndexVec(0,0), mazeGoalList, SEARCH_DEPTH2, true);
}
