#ifndef OPERATION_H_
#define OPERATION_H_

#include <vector>
#include <cstddef>
#include "Maze.h"
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


/**************************************************************
 * Operation List
 *	スタートからゴールまでの一連のOperationの保持する
 *	コンストラクタのPathを入れると勝手に変換する
 **************************************************************/
class OperationList {
private:
	std::vector<Operation> opList;

public:
	OperationList() { }
	//Pathをいれると勝手に変換して保持する
	OperationList(const Path &path, bool useDiagonalPath) { loadFromPath(path, useDiagonalPath); }
	OperationList(const OperationList &obj) { opList = obj.opList; }

	const OperationList &operator=(const OperationList &rhs)
	{
		opList = rhs.opList;
		return (*this);
	}

	//std::vectorとおんなじようなインターフェース
	inline std::vector<Operation>::const_iterator begin() const {return opList.begin(); }
	inline std::vector<Operation>::const_iterator end() const { return opList.end(); }
	inline size_t size() const { return opList.size(); }
	inline void push_back(const Operation& op) { opList.push_back(op); }
	inline void pop_back() { opList.pop_back(); }
	const Operation &operator[](size_t i) const { return opList[i]; }

	//opListの全動作の合計コスト(時間)を計算して返す
	//マシンの走行をシミュレーションしてかかる時間を計算する
	float eval() const;

	//Path読み込む
	//Operationに変換してメンバのOpListに保存する
	//useDiagonalPath=trueにすると斜め走行ありで変換する
	void loadFromPath(const Path& path, bool useDiagonalPath);

	void print();
};


#endif /* OPERATION_H_ */
