#include "auction.h"

Auction::Auction()
{
}

Auction::Auction(const vector<vector<double>> &C)
{
	setValueMatrix(C);
}

void Auction::setValueMatrix(const vector<vector<double>> &C)
{
	/*
		input: C is the vector of value (-cost) of every UAV,
			   each value vector is the value vector (-cost) to each target
		vector<vector<int>> = C(N_uav, vector<int>(N_target, -INF));
	*/

	// 1.维度检查，处理不完全分配
	N_uav = C.size();
	N_target = C.at(0).size();
	// std::cout << "uav num: " << N_uav << " target num " << N_target << std::endl;
	N = std::max(N_uav, N_target);

	if (N_uav != 1 && N_target != 1)
	{
		ValueMatrix = vector<double>(N * N, 0.0);
	}
	else
		ValueMatrix = vector<double>(N_uav * N_target, 0.0);

	// 2.对于不完全分配，需要velue不一致不然迭代无法停止,所以通过在均值附近添加很小的偏置
	// 可以把value_min - 1 给到已经分配过的飞机上\不完全分配虚拟任务
	if (N_target != N_uav && N_uav != 1 && N_target != 1)
	{
		double min_value = INF;
		for (const auto &row : C)
		{
			for (const auto &element : row)
			{
				if(min_value > element)
					min_value = element;
			}
		}
		min_value = min_value - 1;

		srand(time(NULL));
		for (auto &value : ValueMatrix)
		{
			value = static_cast<double>(std::rand()) / RAND_MAX * 0.001 + min_value;
		}
	}

	// 3.赋值ValueMatrix
	int N_temp = (N_uav == 1 || N_target == 1) ? N_target : N;
	for (int i = 0; i < N_uav; i++)
	{
		for (int j = 0; j < N_target; j++)
		{
			ValueMatrix[j + i * N_temp] = C[i][j];
		}
	}
}

vector<int> Auction::runAuction()
{
	/*
		1. setValueMatrix then 2. runAuction
		assignment[i] = j is the traget j assigned to UAV i.
		assignment[i] = INF is UAV i dose not assigned
	*/
	if (VERBOSE)
	{
		cout << "Cost matrix: " << endl;
		printMatrix(&ValueMatrix, N, N);
	}

	vector<int> assignment_return(N_uav, INF);
	if (N_target == 1)
	{
		double max_value = ValueMatrix[0];
		int max_index = 0;
		for (int i = 1; i < ValueMatrix.size(); i++) {
			if (ValueMatrix[i] > max_value) {
				max_value = ValueMatrix[i];
				max_index = i;
			}
		}
		assign_value = max_value;
		assignment_return[max_index] = 0;
		return assignment_return;
	}

	if (N_uav == 1)
	{
		double max_value = ValueMatrix[0];
		int max_index = 0;
		for (int i = 1; i < ValueMatrix.size(); i++) {
			if (ValueMatrix[i] > max_value) {
				max_value = ValueMatrix[i];
				max_index = i;
			}
		}
		assign_value = max_value;
		assignment_return[0] = max_index;
		return assignment_return;
	}
	
	/* Begin Time */
	auto t1 = std::chrono::high_resolution_clock::now();
	clock_t start = clock();

	vector<int> assignment(N, INF);
	vector<double> prices(N, 0);
	// NOTE! 如果分配很慢就请修改epsilon为一个很小的数，比如0.1，通过取次优结果来加快求解速度
	double epsilon = 1.0 / (N + 1);
	int iter = 1;

	while (find(assignment.begin(), assignment.end(), INF) != assignment.end())
	{
		iter++;
		auctionRound(&assignment, &prices, &ValueMatrix, epsilon);
	}

	// int N_assigned = 0;
	// while (N_assigned < N_uav && N_assigned < N_target)
	// {
	// 	iter++;
	// 	auctionRound(&assignment, &prices, &ValueMatrix, epsilon);
	// 	N_assigned = 0;
	// 	for (const int &element : assignment)
	// 	{
	// 		if (element != INF)
	// 		{
	// 			N_assigned++;
	// 		}
	// 	}
	// }

	for (size_t i = 0; i < N_uav; i++)
	{
		if (assignment[i] < N_target)
		{
			assignment_return[i] = assignment[i];
		}
	}

	clock_t end = clock();

	/* End Time */
	auto t2 = std::chrono::high_resolution_clock::now();
	double timing = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
	double time = (double)(end - start) / CLOCKS_PER_SEC * 1000.0;

	cout << "Num Iterations:\t" << iter << endl;
	cout << "Total time:\t" << timing / 1000.0 << endl;
	cout << "Total CPU time:\t" << time << endl;

	double Value = 0;
	if (VERBOSE)
		cout << endl
			 << endl
			 << "Solution: " << endl;
	for (int i = 0; i < assignment_return.size(); i++)
	{
		if (assignment_return[i] == INF)
			continue;
		Value = Value + ValueMatrix.at(N_target * i + assignment_return[i]);
		if (VERBOSE)
			cout << "Person " << i << " gets object " << assignment_return[i] << endl;
	}
	assign_value = Value;
	if (VERBOSE)
		cout << "Total Value: " << Value << endl;

	return assignment_return;
}

void Auction::auctionRound(vector<int> *assignment, vector<double> *prices, vector<double> *C, double epsilon)
{
	/*
		assignment:分配结果，assignment[i]=j代表第i个UAV分配目标j
		prices: 拍卖价格，prices[j]=Value代表当前目标j的竞标价格为Value
		C: IxJ，每个UVA分配每个目标的收益
	*/

	/* Prints the assignment and price vectors */
	// if (VERBOSE)
	// {
	// 	cout << endl
	// 		 << "Assignment: \t\t";
	// 	printVec(assignment);
	// 	cout << "prices: \t\t";
	// 	printVec(prices);
	// 	cout << endl;
	// }

	int N_ = prices->size();

	/*
		These are meant to be kept in correspondance such that bidded[i]
		and bids[i] correspond to person i bidding for bidded[i] with bid bids[i]
	*/
	vector<int> tmpBidded;
	vector<double> tmpBids;
	vector<int> unAssig; // 上一轮未分配者（本轮竞标者）

	/* Compute the bids of each unassigned individual and store them in temp */
	// 对于所有为分配的UAVi，其最佳分配目标存在tmpBidded[i]，竞标出价存在tmpBids[i]
	for (int i = 0; i < assignment->size(); i++)
	{
		/*
			1. Bidding Phase
		*/
		if (assignment->at(i) == INF)
		{
			unAssig.push_back(i);

			/*
				Need the best and second best value of each object to this person
				where value is calculated row_{j} - prices{j}
			*/
			double optValForI = -INF;
			double secOptValForI = -INF;
			int optObjForI, secOptObjForI;
			for (int j = 0; j < N_; j++)
			{
				double curVal = C->at(j + i * N_) - prices->at(j);
				if (curVal > optValForI)
				{
					secOptValForI = optValForI;
					secOptObjForI = optObjForI;
					optValForI = curVal;
					optObjForI = j;
				}
				else if (curVal > secOptValForI)
				{
					secOptValForI = curVal;
					secOptObjForI = j;
				}
			}

			/* Computes the highest reasonable bid for the best object for this person */
			double bidForI = optValForI - secOptValForI + epsilon;

			/* Stores the bidding info for future use */
			tmpBidded.push_back(optObjForI);
			tmpBids.push_back(bidForI);
		}
	}

	/*
		2. Assignment Phase:
		Each object which has received a bid determines the highest bidder and
		updates its price accordingly
		对于至少收到一个出价的物品，它找到最高出价者，并相应地更新该物品的价格。
		最高出价通过比较竞标者在 tmpBids 中的出价金额来确定。
	*/
	for (int j = 0; j < N_; j++)
	{
		vector<int> indices = getIndicesWithVal(&tmpBidded, j);
		if (indices.size() != 0)
		{
			/* Need the highest bid for object j */
			double highestBidForJ = -INF;
			int i_j; // the highest bidder for object j
			for (int i = 0; i < indices.size(); i++)
			{
				double curVal = tmpBids.at(indices.at(i));
				if (curVal > highestBidForJ)
				{
					highestBidForJ = curVal;
					i_j = indices.at(i);
				}
			}

			/* Find the other person who has object j and make them unassigned */
			// 对于物品的最高出价者，代码将该物品分配给相应的个体，并将任何先前的分配给该物品的个体标记为未分配。
			for (int i = 0; i < assignment->size(); i++)
			{
				if (assignment->at(i) == j)
				{
					// if (VERBOSE)
					// 	cout << "Person " << unAssig[i_j] << " was assigned object " << i << " but it will now be unassigned" << endl;
					assignment->at(i) = INF;
					break;
				}
			}
			// if (VERBOSE)
			// 	cout << "Assigning object " << j << " to person " << unAssig[i_j] << endl;

			/* Assign object j to i_j and update the price vector */
			assignment->at(unAssig[i_j]) = j;
			prices->at(j) = prices->at(j) + highestBidForJ;
		}
	}
}

/*<--------------------------------------   Utility Functions   -------------------------------------->*/

/*
		target1  target2  target3
	uav1	 1		  3			5
	uav2	 4		  1			2
	uav3	 5        3   		2

	value = - cost
*/
vector<double> Auction::makeRandC(int size, int size2)
{
	if (size2 == -1)
		size2 = size;
	srand(time(NULL));
	vector<double> mat(size * size2, 2);
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size2; j++)
		{
			mat[j + i * size2] = static_cast<double>(rand() % size2 + 1);
		}
	}
	return mat;
}

// N_uav, N_target
void Auction::printMatrix(vector<auto> *mat, int size, int size2)
{
	if (size2 == -1)
		size2 = size;
	for (int i = 0; i < size; i++)
	{
		for (int j = 0; j < size2; j++)
		{
			cout << mat->at(j + i * size2) << "\t";
		}
		cout << endl;
	}
}

void Auction::printVec(vector<auto> *v)
{
	for (int i = 0; i < v->size(); i++)
	{
		if (v->at(i) == INF)
		{
			cout << "INF"
				 << "\t";
		}
		else
		{
			cout << v->at(i) << "\t";
		}
	}
	cout << endl;
}

/* Returns a vector of indices from v which have the specified value val */
vector<int> Auction::getIndicesWithVal(vector<int> *v, int val)
{
	vector<int> out;
	for (int i = 0; i < v->size(); i++)
	{
		if (v->at(i) == val)
		{
			out.push_back(i);
		}
	}
	return out;
}

void Auction::reset(vector<auto> *v, auto val)
{
	for (int i = 0; i < v->size(); i++)
	{
		v->at(i) = val;
	}
}
