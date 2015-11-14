#include <cmath>
#include <cstdio>
#include "Operation.h"

float OperationList::eval() const
{
	float cost = 0.0;
	for (auto &operation : opList) {
		if (operation.op == Operation::FORWARD || operation.op == Operation::FORWARD_DIAG) {
			//「直線は速度が台形になるように加速する」と仮定してコストを計算
			float distance;
			if (operation.op == Operation::FORWARD_DIAG) distance = (float)operation.n * MAZE_1BLOCK_LENGTH / 2.0 * M_SQRT2;
			else distance = (float)operation.n * MAZE_1BLOCK_LENGTH;
			const float accelDistance = (MAX_VELOCITY*MAX_VELOCITY - MIN_VELOCITY*MIN_VELOCITY) / (2*ACCELERATION);

			if (distance > 2*accelDistance) {
				cost += (distance - 2*accelDistance)/MAX_VELOCITY + 2*( (MAX_VELOCITY-MIN_VELOCITY)/ACCELERATION);
			}
			else {
				const float rt = std::sqrt(MIN_VELOCITY*MIN_VELOCITY + 2*ACCELERATION*distance/2);
				cost += 2*( (-MIN_VELOCITY + rt)/ACCELERATION );
			}
		}
		else if (operation.op == Operation::TURN_LEFT90 || operation.op == Operation::TURN_RIGHT90) {
			cost += TURN90_TIME;
		}
		else if (operation.op == Operation::TURN_LEFT45 || operation.op == Operation::TURN_RIGHT45) {
			cost += TURN45_TIME;
		}
	}

	return cost;
}


void OperationList::loadFromPath(const Path& path, bool useDiagonalPath)
{
	opList.clear();
	std::vector<Operation> tmp_opList;

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
			tmp_opList.push_back(Operation(Operation::FORWARD));
		}
		else if (dirDiff == 1 || dirDiff == -3) {
			tmp_opList.push_back(Operation(Operation::TURN_RIGHT90));
		}
		else if (dirDiff == -1 || dirDiff == 3) {
			tmp_opList.push_back(Operation(Operation::TURN_LEFT90));
		}
		//おかしい
		else {
			while(1);
		}
		robotDir = dir;
	}
	opList = tmp_opList;
	tmp_opList.clear();

	//ななめ走行ありのありの場合の計算
	if (useDiagonalPath) {
		Operation currentDiagOp;
		for (size_t i=0;i<opList.size();i++) {
			if (opList[i].op == Operation::TURN_RIGHT90 || opList[i].op == Operation::TURN_LEFT90) {
				Operation::OperationType prevDiagOp = opList[i].op;

				//jは斜め区間の終わりを指す
				size_t j = i;
				//この後ろの部分でjを斜め区間のおわりまで進めていく
				//TODO:もっとエレガントにしたい
				while(1) {
					j++;
					if (prevDiagOp == Operation::TURN_RIGHT90) {
						if (opList[j].op != Operation::TURN_LEFT90) {
							break;
						}
						else {
							prevDiagOp = opList[j].op;
						}
					}
					else if (prevDiagOp == Operation::TURN_LEFT90) {
						if (opList[j].op != Operation::TURN_RIGHT90) {
							break;
						}
						else {
							prevDiagOp = opList[j].op;
						}
					}
				}

				if (j-i > 1) {
					//RLRL
					//斜め区間の入る方向と出る方向が同じ時
					if ((j-i)%2 == 0) {
						if (prevDiagOp == Operation::TURN_RIGHT90) tmp_opList.push_back(Operation(Operation::TURN_LEFT45));
						else tmp_opList.push_back(Operation(Operation::TURN_RIGHT45));
						if (j-i-2>0) {
							tmp_opList.push_back(Operation(Operation::FORWARD_DIAG,j-i-2));
						}
						if (prevDiagOp == Operation::TURN_RIGHT90) tmp_opList.push_back(Operation(Operation::TURN_RIGHT45));
						else tmp_opList.push_back(Operation(Operation::TURN_LEFT45));
					}
					//RLR
					//斜め区間の入る方向と出る方向が違う時
					else {
						if (prevDiagOp == Operation::TURN_RIGHT90) tmp_opList.push_back(Operation(Operation::TURN_RIGHT45));
						else tmp_opList.push_back(Operation(Operation::TURN_LEFT45));
						tmp_opList.push_back(Operation(Operation::FORWARD_DIAG,j-i-2));

						if (prevDiagOp == Operation::TURN_RIGHT90) tmp_opList.push_back(Operation(Operation::TURN_RIGHT45));
						else tmp_opList.push_back(Operation(Operation::TURN_LEFT45));
					}
					i = j - 1;
				}
				else {
					tmp_opList.push_back(opList[i]);
				}

			}
			else {
				tmp_opList.push_back(opList[i]);
			}
		}

		opList.clear();
		opList = tmp_opList;
	}

	tmp_opList = opList;
	opList.clear();

	//FORWARDの圧縮
	opList.push_back(tmp_opList[0]);
	for (size_t i=1;i<tmp_opList.size();i++) {
		if (tmp_opList[i].op == Operation::FORWARD && tmp_opList[i].op == opList.back().op) {
			opList.back().n++;
		}
		else {
			opList.push_back(tmp_opList[i]);
		}
	}

}


void OperationList::print()
{
	for (auto operation : opList) {
		if (operation.op == Operation::FORWARD) printf("F");
		if (operation.op == Operation::TURN_LEFT90) printf("L");
		if (operation.op == Operation::TURN_RIGHT90) printf("R");
		if (operation.op == Operation::TURN_RIGHT45) printf("r");
		if (operation.op == Operation::TURN_LEFT45) printf("l");
		if (operation.op == Operation::FORWARD_DIAG) printf("D");
		printf("%d ",operation.n);
	}
	printf("\n");
}
