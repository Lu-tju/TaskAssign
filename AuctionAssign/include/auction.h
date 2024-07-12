/*
	基于拍卖的任务分配，完全/不完全拍卖
	无人机数量与目标数量可不相等
*/
#include <iostream>
#include <vector>
#include <limits>
#include <stdlib.h>
#include <math.h>
#include <chrono>
#include <algorithm>
using namespace std;

#define INF numeric_limits<int>::max()
#define VERBOSE false

class Auction
{
private:
	int N;
	int N_uav;
	int N_target;
	vector<double> ValueMatrix; // Array of UAV(row) and Value(col)
	double assign_value;

	void auctionRound(vector<int> *assignment, vector<double> *prices, vector<double> *C, double epsilon);
	vector<double> makeRandC(int size, int size2 = -1);
	void printMatrix(vector<auto> *mat, int size, int size2 = -1);
	vector<int> getIndicesWithVal(vector<int> *v, int val);
	void reset(vector<auto> *v, auto val);
	void printVec(vector<auto> *v);

public:
  	Auction();
	Auction(const vector<vector<double>>& C);
	~Auction() {}
	vector<int> runAuction();
	void setValueMatrix(const vector<vector<double>>& C);
	double getAssignValue() { return assign_value; }
};
