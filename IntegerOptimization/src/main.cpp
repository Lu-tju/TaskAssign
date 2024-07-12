#include "firepower_allocation.h"
using namespace firepower_allocation;

int main() {
	int target_num = 10;
	int uav_num = 30;
	int max_uav = 3;
	float r = 50;
	FirepowerAllocation allocation(uav_num, target_num, r, max_uav);
	allocation.random_init();
	allocation.run();

	return 1;
}