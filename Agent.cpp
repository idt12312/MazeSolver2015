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
		distIndexList.assign(MAZE_GOAL_LIST);

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
		//distIndexListのどれかに到達した or 目標地点が到達不能だと分かったら更新
		auto it = std::find(distIndexList.begin(), distIndexList.end(), cur);
		if (it != distIndexList.end() || calcNextDirection(cur, dist) == 0) {
			//暫定最短経路上の未探索壁のある座標を列挙
			//それらの座標をdistIndexListにいれる
			distIndexList.clear();
			path.calcKShortestDistancePath(IndexVec(0,0), MAZE_GOAL_LIST, SEARCH_DEPTH1, false);
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
	}


	if (state == Agent::FINISHED) {

	}


	nextDir = calcNextDirection(cur, dist);
}

void Agent::caclRunSequence()
{
	if (state != Agent::FINISHED) return ;
	path.calcShortestTimePath(IndexVec(0,0), MAZE_GOAL_LIST, SEARCH_DEPTH2, true, true);
}


void Agent::resumeAt(State resumeState, Maze &_maze)
{
	reset();
	maze = &_maze;

	if (resumeState == State::IDLE) {
		state = State::IDLE;
	}

	else if (resumeState == State::SEARCHING_NOT_GOAL) {
		distIndexList.assign(MAZE_GOAL_LIST);
		dist = distIndexList.front();
		state = State::SEARCHING_NOT_GOAL;
	}

	else if (resumeState == State::SEARCHING_REACHED_GOAL) {
		//暫定最短経路上の未探索壁のある座標を列挙
		//それらの座標をdistIndexListにいれる
		maze->updateStepMap(IndexVec(0,0));
		path.calcKShortestDistancePath(IndexVec(0,0), MAZE_GOAL_LIST, SEARCH_DEPTH1, false);
		path.calcNeedToSearchWallIndex();
		distIndexList.assign(path.getNeedToSearchIndex().begin(), path.getNeedToSearchIndex().end());

		//distIndexListの中から現在座標に一番近い近いものをdistに入れる
		int minDistance = INT32_MAX;
		std::list<IndexVec>::iterator it_nearestDist;
		for (auto it=distIndexList.begin();it!=distIndexList.end();it++) {
			int stepDiff = maze->getStepMap(IndexVec(0,0)) - maze->getStepMap(*it);
			if (stepDiff<0) stepDiff = -stepDiff;
			if (stepDiff < minDistance) {
				minDistance = stepDiff;
				it_nearestDist = it;
			}
		}
		dist = *it_nearestDist;

		state = State::SEARCHING_REACHED_GOAL;
	}

	else if (resumeState == State::BACK_TO_START) {
		state = State::FINISHED;
	}

	else if (resumeState == State::FINISHED) {
		state = State::FINISHED;
	}

}
