#ifndef MAZE_H_
#define MAZE_H_

#include <stdint.h>
#include <stdlib.h>
#include <sys/types.h>

#include "MazeSolver_conf.h"


/*	壁の情報・ロボットがどちらに進むかの方向などを表現するのに使う
*
*/
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
	inline operator uint8_t() const { return byte; }
	inline uint8_t operator|(uint8_t value) const { return byte | value; }
	inline uint8_t operator&(uint8_t value) const { return byte & value; }
	inline uint8_t operator=(uint8_t value) { return byte = value; }
	inline uint8_t operator|=(uint8_t value) { return byte |= value; }
	inline uint8_t operator&=(uint8_t value) { return byte &= value; }
	inline uint8_t operator=(Direction &obj) { return byte = obj.byte; }
	inline uint8_t operator[](uint8_t index) const {return (byte & (0x01<<index)) ? 1:0; }
	inline bool isDoneAll() const { return (byte | 0x0f) == 0xff; }
	int nWall() const {
		int cnt = 0;
		if (bits.North) cnt++;
		if (bits.East) cnt++;
		if (bits.South) cnt++;
		if (bits.West) cnt++;
		return cnt;
	}
	int nDoneWall() const {
		int cnt = 0;
		if (bits.DoneNorth) cnt++;
		if (bits.DoneEast) cnt++;
		if (bits.DoneSouth) cnt++;
		if (bits.DoneWest) cnt++;
		return cnt;
	}
};

extern const uint8_t NORTH;
extern const uint8_t EAST;
extern const uint8_t SOUTH;
extern const uint8_t WEST;
extern const uint8_t DONE_NORTH;
extern const uint8_t DONE_EAST;
extern const uint8_t DONE_SOUTH;
extern const uint8_t DONE_WEST;

/*
 * 座標を表現するのに使う
 * int8_tでxとyの成分を持ち、ベクトルとして+-の演算と代入ができる
 */
struct __attribute__ ((__packed__)) IndexVec {
	int8_t x;
	int8_t y;

	IndexVec(int8_t _x=0, int8_t _y=0) : x(_x), y(_y) {}
	IndexVec(const IndexVec &obj) : x(obj.x), y(obj.y) {}
	inline IndexVec operator+(const IndexVec &obj) const { return IndexVec(x+obj.x, y+obj.y); }
	inline IndexVec operator-(const IndexVec &obj) const { return IndexVec(x-obj.x, y-obj.y); }
	inline void operator+=(const IndexVec &obj) { x+=obj.x; y+=obj.y; }
	inline void operator-=(const IndexVec &obj) { x-=obj.x; y-=obj.y; }
	inline const IndexVec& operator=(const IndexVec &obj) { x=obj.x; y=obj.y; return *this; }
	inline bool operator==(const IndexVec &obj) const { return x == obj.x && y == obj.y; }
	inline bool operator!=(const IndexVec &obj) const { return x != obj.x || y != obj.y; }
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
	inline uint norm() const
	{
		const int8_t x_abs = x>0?x:-x;
		const int8_t y_abs = y>0?y:-y;
		return x_abs + y_abs;
	}
	inline bool isCorner(){ return x == MAZE_SIZE-1 || x == 0 || y == MAZE_SIZE-1 || y == 0; }

	//各方角を表すベクトル
	static const IndexVec vecNorth;
	static const IndexVec vecEast;
	static const IndexVec vecSouth;
	static const IndexVec vecWest;
	//[0]:北 [1]:東 [2]:南 [3]:西
	static const IndexVec vecDir[4];
};

/*
 * 迷路の壁情報を扱う
 */
class Maze {
private:
	Direction wall[MAZE_SIZE][MAZE_SIZE];
	uint8_t stepMap[MAZE_SIZE][MAZE_SIZE];

public:
	Maze(){ clear(); }
	Maze(const Maze &obj)
	{
		for (int i=0;i<MAZE_SIZE;i++) {
			for (int j=0;j<MAZE_SIZE;j++) {
				wall[i][j] = obj.wall[i][j];
			}
		}
	}
	void clear();

	//ロードするファイル、配列のデータの並びは迷路を実際に見た時と同じ並び方
	//Maze.wallは上下が逆転しているから注意
	//file[i][j] = ascii[i][j] = wall[N-1-i][j]
	bool loadFromFile(const char *_filename);
	void loadFromArray(const char asciiData[MAZE_SIZE+1][MAZE_SIZE+1]);

	void printWall(const uint8_t value[MAZE_SIZE][MAZE_SIZE] = NULL) const;
	void printWall(const bool value[MAZE_SIZE][MAZE_SIZE]) const;
	void printStepMap() const;

	void updateWall(const IndexVec &cur, const Direction &newState, bool forceSetDone = true);
	void updateStepMap(const IndexVec &dist);

	const Direction &getWall(const IndexVec &index) const { return wall[index.y][index.x]; }
	const Direction &getWall(int8_t x, int8_t y) const { return wall[y][x]; }

	const uint8_t &getStepMap(const IndexVec &index) const { return stepMap[index.y][index.x]; }
	const uint8_t &getStepMap(int8_t x, int8_t y) const { return stepMap[y][x]; }

};


#endif /* MAZE_H_ */
