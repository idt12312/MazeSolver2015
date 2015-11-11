# MazeSolver 2015

マイクロマウスの迷路探索&走行経路ライブラリ

## アルゴリズム
### 探索(ゴールにつくまで)
* 足立法でゴールまで行く。
* ゴールの区画4つ全てを訪れる。

### 探索(ゴールについたら)
1. 未探索の壁は壁が無いものとしてスタートからゴールまでのk最短経路(マンハッタン距離)を計算する
2. k最短経路上の未探索の壁を目標座標リストに入れる
3. 目標座標リストを現在の位置に近い順(迷路の壁も考慮した歩数マップ)にソート
4. 目標座標リストの先頭の座標に足立法で向かっていく
5. 目標座標リストに含まれる座標に到達するたびに1~4を再び繰り返す
6. 目標座標リストが空(k最短経路上に未探索壁がない状態)になったらスタート座標に戻る

### 最終的な走行経路の算出
1. 未探索の壁は壁があるものとしてスタートからゴールまでのk最短経路(マンハッタン距離)を計算する
2. マシンの走行パラメータを考慮して、実際に経路を走った時の所要時間を計算する
3. 所要時間の一番短かったものを走行経路として決定する

## 使用例
データ形式とかは下の方を参照  
各種設定パラメータはMazeSolver_conf.hに入ってる

```
#!C
#include "Maze.h"
#include "Agent.h"
#include "mazeData.h"
#include <vector>

//探索した迷路の壁情報がはいる
Maze maze;
//クラッシュした時のためのバックアップ
Maze maze_backup;

//探索の指示を出す
Agent agent(maze);
//前回のAgentの状態を保存しとく
Agent::State prevState = IDLE;

int main()
{
	
    /**********************************
     * 迷路の探索
     *********************************/
	while(1) {
    	//1区画進んで壁の状態が分かるまで待機
        //ここで待っている時に割り込みでモーターを制御したりセンサの値を処理したりすることになる
    	while(!wallDataReady());
        
        
        //センサから取得した壁情報を入れる
		Direction wallData = getWallData();
		//ロボットの座標を取得
		IndexVec robotPos = getRobotPosion();

		//壁情報を更新 次に進むべき方向を計算
		agent.update(robotPos, wallData);
        
        //Agentの状態を確認
        //FINISHEDになったら計測走行にうつる
		if (agent.getState() == Agent::FINISHED) break;
        
        //ゴールにたどり着いた瞬間に一度だけmazeのバックアップをとる
        //Mazeクラスはoperator=が定義してあるからa = bでコピーできる
        if (prev_State == Agent::SEARCHING_NOT_GOAL && agent.getState() == SEARCHING_REACHED_GOAL) {
			maze_backup = maze;
        }
        prev_State = agent.getState();

		//一度はゴールにたどり着き、少なくともゴールできる状態で追加の探索をしているが、
        //もう時間が無いから探索をうちやめてスタート地点に戻る
        if (isTimeOut() && agent.getState() == Agent::SEARCHING_REACHED_GOAL){
        	agent.forceGotoStart();
        }

		//Agentの状態が探索中の場合は次に進むべき方向を取得する
		Direction nextDir = agent.getNextDirection();
        
		//nextDirの示す方向に進む
		//突然今と180度逆の方向を示してくる場合もあるので注意
        //止まらないと壁にぶつかる
        robotMove(nextDir);  //robotMove関数はDirection型を受け取ってロボットをそっちに動かす関数
	}
    
    //ロボットを停止させ、スタートする向きに戻す
    robotPositionInit();
    
    //最短経路の計算 割と時間がかかる(数秒)
    //引数は斜め走行をするかしないか
    //trueだと斜め走行をする
    agent.calcRunSequence(true);
    
     /**********************************
     * 計測走行
     *********************************/
    //コマンドリストみたいなやつを取り出す
    const OperationList &runSequence = agent.getRunSequence();
    
    //Operationを先頭から順番に実行していく
    for (size_t i=0;i<runSequence.size();i++) {
    	//Operationの実行が終わるまで待つ(nマス進んだ,右に曲がった)
        while(!operationFinished());
        
        //i番目のを実行
    	robotMove(runSequence[i]); //robotMode関数はOperation型を受け取ってそれを実行する関数
    }
    
    //おわり
    
    return 0;
}

//マシンがクラッシュして、迷路情報のバックアップから復帰する
//ここではFINISHEDに復帰をしている(これ以上の探索を諦めて、今わかっている情報から最速走行を行う)
//マシンが(0,0)にいる状態でSEARCHING_REACHED_GOALやSEARCHING_NOT_GOALに戻してやると、うまく探索を再開できるようになっている
//その場合はどの時のMazeに復旧するかがポイント
void recoveryFromCrash() {
	agent.resumeAt(FINISHED, maze_backup);
}
```

## Direction (Maze.h)
* 壁があるかないか・ロボットがどちらに進むかの方向(方角)などを表現する
* 8bit
* 探索済みどうかの情報も持てる
* 方角はある意味で絶対座標系のようなもの
* マウス界隈で壁情報を扱うのによく使われるデータ形式らしい([参考](http://ctrlwww.ee.noda.tus.ac.jp/wiki/kansai/index.php/%E5%BA%A7%E6%A8%99%E7%B3%BB%E3%83%BB%E8%BF%B7%E8%B7%AF%E8%A1%A8%E7%8F%BE%E3%81%AE%E5%AE%9A%E7%BE%A9}))

### データ形式
|bit7(MSB)|bit6|bit5|bit4|bit3|bit2|bit1|bit0(LSB)|
|---------|----|----|----|----|----|----|---------|
|探索済みbit||||壁or方向bit||||
|西|南|東|北|西|南|東|北|

#### 下位4bit : 壁・方向bit
壁の情報を表すときは、各方角に壁があるかないかを次のように表現する。

* 0:壁がない
* 1:壁がある

探索済みかどうかは上位4bitをみるとわかる。  

方向の情報を表すときは、伝えたい方向に対応するbitを立てる。  
例えば北を表現したいときは0x1(0001)

#### 上位4bit : 探索済みbit
* 0:未探索
* 1:探索済み

#### 定数
よく使いそうなものは定数として定義済み

```
#!C
const uint8_t NORTH = 0x01;
const uint8_t EAST = 0x02;
const uint8_t SOUTH = 0x04;
const uint8_t WEST = 0x08;
const uint8_t DONE_NORTH = 0x10;
const uint8_t DONE_EAST = 0x20;
const uint8_t DONE_SOUTH = 0x40;
const uint8_t DONE_WEST = 0x80;
```

### 使用例
```
#!C
//インスタンス化
Direction dir; 		//全てのbitは0で初期化される
Direction dir2(0xFA); //0xFAで初期化される

//バイトアクセス(read write両方可能)
dir = 0xAB; 		//0xABを代入(書き込み)
dir.byte = 0xAB;
dir = dir2;		//dir2をdirに代入

f(dir)			//uint8_tとして読み取り
f(dir.byte)

//ビットアクセス
dir.North = 1; 	//Northにあたるbitを1(or0)にセット
f(dir.North)

//ビットアクセス(読み取り専用)
f(dir[0])		//0bit目(North)を読み取る

//ビットの数を数える
dir.nWall(); //壁bitの1になっているbit数を返す
dir.nDoneWall(); //探索済みbitの1になっているbit数を返す
```
## IndexVec (Maze.h)
* 迷路上のx,y座標を扱う2次元ベクトル
* 中身はint8_tのxとyをメンバに持つ構造体
* ベクトルの足し算とかができる

図
### 定数
* 東西南北を表す大きさ1のベクトルがstaticメンバとして定義してある。
* IndexVecDir[i]はi=0,1,2,3 の順に北東南西の時計回りにベクトルを返す。
* 4近傍を調べるときに便利

```
#!C
const IndexVec IndexVec::vecNorth(0,1);
const IndexVec IndexVec::vecEast(1,0);
const IndexVec IndexVec::vecSouth(0,-1);
const IndexVec IndexVec::vecWest(-1,0);
const IndexVec IndexVec::vecDir[4] = {IndexVec::vecNorth, IndexVec::vecEast, IndexVec::vecSouth, IndexVec::vecWest};
```

### 使用例

```
#!C
//インスタンス化
IndexVec vec1; //x=0,y=0で初期化される
IndexVec vec2(1,2); //x=1,y=2で初期化される

//演算
vec3 = vec1 + vec2; //和
vec1 += vec2;//vec1 = vec1 + vec2

vec3 = vec1 - vec2; //差
vec1 -= vec2; //vec1 = vec1 - vec2

//比較
if (vec1 == vec2) {...} //x,y両方の値が等しい時にtrue
if (vec1 |= vec2) {...} //上の否定

//範囲内に収まるかのチェック
//vec1とvec2を足した結果が迷路の座標の範囲内(0~15)に収まっているかどうか
if ( (vec1+vec2).canSum() ) {...}
```

## Operation (ShortestPath.h)
* 経路上を走るためのコマンドのようなもの
* 「3マス分直進」「90度右に旋回」などの情報が入ってる
* ロボットに取り付けた、ある意味相対座標系で解釈をする

### データ
OperationType opとuint8_t nをメンバに持る構造体で、「opという動作をn回繰り返す」と解釈する

|OperationTypeの値|意味|
|---|---|
|FORWARD|直進|
|FORWARD_DIAG|斜めに直進(普通の直進とは進む距離が違う)|
|TURN_RIGHT90|右に90度旋回|
|TURN_RIGHT45|右に45度旋回|
|TURN_LEFT90|左に90度旋回|
|TURN_LEFT45|左に45度旋回|
|STOP|停止|
## ShortestPath (ShortestPath.h)
* 最短経路とかを算出する
* 触らない

## Agent (Agent.h)
* 探索アルゴリズムの最上位層
* 基本的にここに壁情報をいれて、ここから次動くべき方向を取得する
* 探索の指示を出し、終わったら最終的に走る経路を提示してくれる
* 時間がかかり過ぎている場合などに探索を打ち切ってスタートに戻ることもできる
* 探索走行中にマシンがクラッシュした時などのために、途中から再開することもできる

### 内部状態
getState()で取得できる

|Agent::Stateの値|意味|
|---|---|
|IDLE|まだ実行されていない|
|SEARCHING_NOT_GOAL|探索中　まだ一度もゴールに到達していない|
|SEARCHING_REACHED_GOAL|探索中　一度ゴールに到達したが、追加で探索をしている|
|BACK_TO_START|スタートに戻っている|
|FINISHED|スタート地点に到着をし、最短経路の計算が終わった|

状態がIDLEとFINISHED以外の時にはgetNextDirection()で次に進むべき方向が返ってくる

### クラッシュ時に途中から再開する
resumeAtメソッドを使う。
引数のresumeStateには再開したいAgentの状態を、
_mazeには再開したいMazeの状態を入れる。
例は一番上の使用例を参照

### 最終的に走る経路
* 状態がFINISHEDとときにcalcRunSequence()を実行すると最終的に走る経路が計算される
* calcRunSequenceの引数をtrueにすると斜め走行あり、falseにすると斜め走行なしで計算をする
* calcRunSequence()は数秒のオーダーで計算に時間がかかる
* 計算できたらgetRunSequence()で最終的に走る経路が取得できる
* 具体的にはconst OperationList &が返ってくる(読み取り専用)
* 先頭から順に実行をしていけばゴールにつく


### 使い方


## Maze (Maze.h)
* 迷路の壁情報と歩数マップを保持する。
* この情報さえ保存しとけば続きから探索したりできる(たぶん)
* ファイル・配列から迷路の壁情報をロードできる
* printfでそれっぽくコンソールに表示できる
* 新しく壁を見つけた時はupdateWall()で壁情報を更新する
* 迷路の壁情報はDirection wall[N][N]で持っている
* 歩数マップはuint8_t stepMap[N][N]で持っている
* 原点は左下、x正方向は右(西)、y正方向が上(北)

### 使い方
```
#!C
//インスタンス化
Maze maze; //壁情報(Direction)は全て0で初期化
Maze maze2(maze); //複製

//代入コピー
Maze maze1, maze2;
maze1 = maze2;

//ロード
maze.loadFromFile("maze_data.dat"); //ファイルから
maze.loadFromArray(mazeData); //配列から(mazeData.hと.cppにいくつかある)

//壁情報へのアクセス
maze.getWall(1,2); //(1,2)の壁情報にDirection型としてアクセスできる
f(maze.wall(1,2)[3]); //(1,2)のbit3(西)にアクセス
Direction walldata = maze.getWall(1,2); //(1,2)の壁情報で初期化

//歩数マップへのアクセス
maze.stepMap(1,2); //(1,2)の歩数をuint8_tで取得

//歩数マップへのアクセス
//壁情報の更新
//例として座標1,2の東西のみに壁があるという情報をセットする
//渡す壁情報wallDataの探索済みbitは無視され、全て探索済みとして読み込まれる
//つまり0x0A,0x3Aどちらを渡しても0xFAとして保存される。
IndexVec robotPos(1,2);
Direction wallData(0x0A);
maze.update(robotPos, wallData);
maze.update(robotPos, wallData, false); //こうすると探索済みbitを無視せずに保存する。

//歩数マップの再計算
//座標(7,7)を0歩とした歩数マップをつくる
IndexVec goal(7,7);
maze.updateStepMap(goal);
```

## マイコン上で計算にかかる時間
STM32F407 168MHz上で実行

### 探索時(ゴールに着くまで)
|最適化なし(-O0)|最適化あり(-O2)|
|---|---|
|3~4ms|1~2ms|

### 探索時(ゴールについたあと)
* 最短経路ランキングのどこまでを候補に入れるかによってめっちゃ変わる
* 基本的にはゴールに着くまでとおなじくらいの時間がかかる
* k=1で経路を探索するとO0で最悪でも15msくらいで計算がおわる
* O0でk=2にすると100ms以上かかることもある

### k最短経路の計算

|k|最適化なし(-O0)|最適化あり(-O2)|
|---|---|---|
|k=2|133ms|11ms|
|k=5|400ms|35ms|
|k=10|909ms|77ms|
|k=20|2082ms|178ms|
|k=50|6277ms|500ms|

色々な迷路で試した結果、k=10~20くらいあれば完璧なルートが算出できる


### 最終的な経路の計算
k最短経路にかかる時間+少し(数ms)

## マイコン上でのメモリ消費量
未検証  
とりあえずうごいた

## 開発環境
* Linux gcc 5.2.0
* コンパイルオプション -g -O0 -std=c++11