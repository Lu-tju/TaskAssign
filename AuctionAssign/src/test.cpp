#include <iostream>
#include <vector>
#include <algorithm>

int main() {
    std::vector<int> to_assigned_targets = {1, 2, 3, 4, 5};
    std::vector<int> assigned_uavs = {0, -1, 2, -1, 4};

    // 使用 remove_if 结合 lambda 表达式删除满足条件的元素
    auto removeCondition = [](int value) { return value == -1; };
    auto toAssignedTargetsIter = to_assigned_targets.begin();
    auto assignedUAVsIter = assigned_uavs.begin();

    while (assignedUAVsIter != assigned_uavs.end()) {
        if (removeCondition(*assignedUAVsIter)) {
            // 删除 assigned_uavs 中值为 -1 的元素
            assignedUAVsIter = assigned_uavs.erase(assignedUAVsIter);
            // 删除 to_assigned_targets 中对应位置上的元素
            toAssignedTargetsIter = to_assigned_targets.erase(toAssignedTargetsIter);
        } else {
            ++assignedUAVsIter;
            ++toAssignedTargetsIter;
        }
    }

    // 输出结果
    std::cout << "assigned_uavs: ";
    for (int value : assigned_uavs) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    std::cout << "to_assigned_targets: ";
    for (int value : to_assigned_targets) {
        std::cout << value << " ";
    }
    std::cout << std::endl;

    return 0;
}
