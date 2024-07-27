#include "KM_match.h"

int main() {
    // 输入顶点数量和权重矩阵
    int cn = 5;
    std::vector<std::vector<int>> cc = {
        {3, 4, 6, 4, 9, 4},
        {6, 4, 5, 3, 7, 2},
        {2, 5, 3, 4, 2, 2},
        {6, 3, 2, 2, 7, 3}
//        {8, 4, 5, 4, 7}
    };

    // 执行KM算法
    int a = KM_matching(4, 6, cc);
    return 0;
}