#ifndef MAZE_H_
#define MAZE_H_

#include <cstdint>
#include <cstddef>
#include "MazeSolver_conf.h"


/**************************************************************
 * Direction
 *	壁の情報・ロボットがどちらに進むかの方向などを表現するのに使う
 **************************************************************/
union __attribute__ ((__packed__)) Direction {
public:
	uint8_t byte;
	struct __attribute__ ((__packed__)) {
		uint8_t North:1; 		//bit0 LSB
		uint8_t East:1; 		//bit1
		uint8_t South:1; 		//bit2
		uint8_t West:1; 		//bit3
		uint8_t DoneNorth:1; 	//bit4
		uint8_t DoneEast:1; 	//bit5
		uint8_t DoneSouth:1; 	//bit6
		uint8_t DoneWest:1; 	//bit7 MSB
	} bits;
public:
	Direction(uint8_t value=0) : byte(value) {}

	//演算関連は全てuint8_tにキャストしてから行う
	inline operator uint8_t() const { return byte; }
	inline uint8_t operator|(uint8_t value) const { return byte | value; }
	inline uint8_t operator&(uint8_t value) const { return byte & value; }
	inline uint8_t operator=(uint8_t value) { return byte = value; }
	inline uint8_t operator|=(uint8_t value) { return byte |= value; }
	inline uint8_t operator&=(uint8_t value) { return byte &= value; }
	inline uint8_t operator=(Direction &obj) { return byte = obj.byte; }

	//index番目のビットが立っているかどうかをuint8_tの0x00か0x01で返す
	inline uint8_t operator[](uint8_t index) const {return (byte & (0x01<<index)) ? 1:0; }

	//全ての壁が探索済みかどうか
	inline bool isDoneAll() const { return (byte | 0x0f) == 0xff; }

	//壁の数を数える
	int nWall() const {
		int cnt = 0;
		if (bits.North) cnt++;
		if (bits.East) cnt++;
		if (bits.South) cnt++;
		if (bits.West) cnt++;
		return cnt;
	}

	//探索済みの壁を数える
	int nDoneWall() const {
		int cnt = 0;
		if (bits.DoneNorth) cnt++;
		if (bits.DoneEast) cnt++;
		if (bits.DoneSouth) cnt++;
		if (bits.DoneWest) cnt++;
		return cnt;
	}
};

//便利な定数
extern const uint8_t NORTH;
extern const uint8_t EAST;
extern const uint8_t SOUTH;
extern const uint8_t WEST;
extern const uint8_t DONE_NORTH;
extern const uint8_t DONE_EAST;
extern const uint8_t DONE_SOUTH;
extern const uint8_t DONE_WEST;


/**************************************************************
 * IndexVec
 *	座標を表現するのに使う
 *	int8_tでxとyの成分を持ち、ベクトルとして+-の演算と代入ができる
 **************************************************************/
struct __attribute__ ((__packed__)) IndexVec {
	int8_t x;
	int8_t y;

	IndexVec(int8_t _x=0, int8_t _y=0) : x(_x), y(_y) {}
	IndexVec(const IndexVec &obj) : x(obj.x), y(obj.y) {}

	//ベクトルとしての演算
	inline IndexVec operator+(const IndexVec &obj) const { return IndexVec(x+obj.x, y+obj.y); }
	inline IndexVec operator-(const IndexVec &obj) const { return IndexVec(x-obj.x, y-obj.y); }
	inline void operator+=(const IndexVec &obj) { x+=obj.x; y+=obj.y; }
	inline void operator-=(const IndexVec &obj) { x-=obj.x; y-=obj.y; }
	inline const IndexVec& operator=(const IndexVec &obj) { x=obj.x; y=obj.y; return *this; }
	inline bool operator==(const IndexVec &obj) const { return x == obj.x && y == obj.y; }
	inline bool operator!=(const IndexVec &obj) const { return x != obj.x || y != obj.y; }

	//自分とobjを足しても迷路座標の範囲に収まるかどうか
	inline bool canSum(const IndexVec &obj) const
	{
		const int8_t res_x = x + obj.x;
		if (res_x<0 || MAZE_SIZE<=res_x) return false;
		const int8_t res_y = y + obj.y;
		if (res_y<0 || MAZE_SIZE<=res_y) return false;
		return true;
	}
	inline bool canSub(const IndexVec &obj) const
	{
		const int8_t res_x = x - obj.x;
		if (res_x<0 || MAZE_SIZE<=res_x) return false;
		const int8_t res_y = y - obj.y;
		if (res_y<0 || MAZE_SIZE<=res_y) return false;
		return true;
	}

	//L1ノルム
	inline uint8_t norm() const
	{
		const int8_t x_abs = x>0?x:-x;
		const int8_t y_abs = y>0?y:-y;
		return x_abs + y_abs;
	}

	inline bool isDiag() const
	{
		const int8_t x_abs = x>0?x:-x;
		const int8_t y_abs = y>0?y:-y;
		return x_abs == 1 && y_abs == 1;
	}

	inline bool isCorner(){ return x == MAZE_SIZE-1 || x == 0 || y == MAZE_SIZE-1 || y == 0; }

	//便利な定数
	//各方角を表すベクトル
	static const IndexVec vecNorth;
	static const IndexVec vecEast;
	static const IndexVec vecSouth;
	static const IndexVec vecWest;
	//[0]:北 [1]:東 [2]:南 [3]:西
	static const IndexVec vecDir[4];
};


/**************************************************************
 * Maze
 *	壁情報と歩数マップを保持する
 *	壁情報はMazeのupdateWallを使って更新をしていく
 **************************************************************/
class Maze {
private:
	Direction wall[MAZE_SIZE][MAZE_SIZE];
	uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];

	//無駄な計算をしないために、前回歩数マップを計算した時の情報を覚えとく
	//もし前回と同じ状況ならば計算結果は変わらないので実行しない
	bool dirty;
	bool lastOnlyUseFoundWall;
	IndexVec lastStepMapDist;

public:
	Maze() : dirty(true), lastOnlyUseFoundWall(true) { clear(); }
	Maze(const Maze &obj) : dirty(true), lastOnlyUseFoundWall(true)
	{
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				wall[i][j] = obj.wall[i][j];
				stepMap[i][j] = obj.stepMap[i][j];
			}
		}
	}

	const Maze& operator=(const Maze &obj)
	{
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				wall[i][j] = obj.wall[i][j];
				stepMap[i][j] = obj.stepMap[i][j];
			}
		}
		return *this;
	}

	//wallもstepMapも全て0になる
	void clear();

	//ファイルから迷路をロードする
	bool loadFromFile(const char *_filename);

	//配列からロードする
	//ロードするファイル、配列のデータの並びは迷路を実際に見た時と同じ並び方
	//Maze.wallは上下が逆転しているから注意
	//file[i][j] = ascii[i][j] = wall[MAZE_SIZE-1-i][j]
	void loadFromArray(const char asciiData[MAZE_SIZE+1][MAZE_SIZE+1]);

	//コンソール上にそれっぽく整形して迷路を表示する
	//引数に数字の配列を渡すと各区画にその数字が表示される
	void printWall(const uint8_t value[MAZE_SIZE][MAZE_SIZE] = nullptr) const;
	//引数にboolの配列を渡すと、trueの区画に*が表示される
	void printWall(const bool value[MAZE_SIZE][MAZE_SIZE]) const;
	//歩数マップを表示
	void printStepMap() const;


	//新しく壁情報が分かったときにこれを読んで壁情報を更新する
	//1つの座標だけではなく、4近傍の壁の情報も整合性が取れるように更新してくれる
	//cur:座標  newState:壁情報
	//forceSetDone = tureのとき(default)
	//	newStateは上位4bitが1111にセットされてwallに取り込まれる
	//	これはその座標の壁全てが探索済みとして更新したことになる
	//forceSetDone = falseのとき
	//	newStateはそのままwallに取り込まれる
	void updateWall(const IndexVec &cur, const Direction &newState, bool forceSetDone = true);

	//歩数マップの更新
	//適宜歩数マップが必要になるときにこれを呼んで歩数マップを更新してから参照する
	//distの座標の歩数マップを0として計算する
	//onlyUseFoundWall=trueにすると未探索の壁は通れないものとして歩数マップを計算する
	void updateStepMap(const IndexVec &dist, bool onlyUseFoundWall = false);

	//指定座標の壁情報を取得
	inline const Direction &getWall(const IndexVec &index) const { return wall[index.y][index.x]; }
	inline const Direction &getWall(int8_t x, int8_t y) const { return wall[y][x]; }

	//指定座標の歩数マップを取得
	inline const uint8_t &getStepMap(const IndexVec &index) const { return stepMap[index.y][index.x]; }
	inline const uint8_t &getStepMap(int8_t x, int8_t y) const { return stepMap[y][x]; }

};


#endif /* MAZE_H_ */
