#ifndef OPERATION_H_
#define OPERATION_H_

#include <vector>
#include "Maze.h"
//#include "ShortestPath.h"
#include "MazeSolver_conf.h"


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
		FORWARD_DIAG,
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


class OperationList {
private:
	std::vector<Operation> opList;

public:
	OperationList() { }
	OperationList(const Path &path, bool useDiagonalPath) { loadFromPath(path, useDiagonalPath); }

	inline std::vector<Operation>::const_iterator begin() const {return opList.begin(); }
	inline std::vector<Operation>::const_iterator end() const { return opList.end(); }
	inline void push_back(const Operation& op) { opList.push_back(op); }
	inline void pop_back() { opList.pop_back(); }
	const Operation &operator[](size_t i) const { return opList[i]; }
	float eval() const;
	void loadFromPath(const Path& path, bool useDiagonalPath);
};


#endif /* OPERATION_H_ */
