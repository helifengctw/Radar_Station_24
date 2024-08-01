#include <iostream>
#include <vector>
#include <algorithm>
#include <climits>
#include <cstring>
#include <chrono>


using namespace std;

double start_time = 0.0, c_time = 0.0;
const int MAXN = 20;
int m, n, dfs_count = 0; // 顶点数量
int lx[MAXN], ly[MAXN]; // 顶标
int weight[MAXN][MAXN]; // 权重矩阵
bool S[MAXN], T[MAXN]; // 辅助数组
int matchY[MAXN]; // Y集中的匹配点
int slack[MAXN]; // 松弛数组

bool DFS(int x) {
    cout << dfs_count++ << ", ";
    S[x] = true;
    for (int y = 0; y < n; ++y) {
        if (T[y]) continue;
        int tmp = lx[x] + ly[y] - weight[x][y];
        if (tmp == 0) {
            T[y] = true;
            if (matchY[y] == -1 || DFS(matchY[y])) {
                matchY[y] = x;
                return true;
            }
        } else {
            slack[y] = min(slack[y], tmp);
        }
    }
    return false;
}

void KM() {
    memset(matchY, -1, sizeof(matchY));
    memset(ly, 0, sizeof(ly));
    for (int i = 0; i < m; ++i) {
        lx[i] = *max_element(weight[i], weight[i] + n);
    }
    for (int x = 0; x < m; ++x) {
        fill(slack, slack + n, INT_MAX);
        while (true) {
            memset(S, false, sizeof(S));
            memset(T, false, sizeof(T));
            if (DFS(x)) break;
            int d = INT_MAX;
            for (int i = 0; i < n; ++i) {
                if (!T[i]) d = min(d, slack[i]);
            }
            for (int i = 0; i < m; ++i) {
                if (S[i]) lx[i] -= d;
            }
            for (int j = 0; j < n; ++j) {
                if (T[j]) ly[j] += d;
                else slack[j] -= d;
            }
        }
    }
}

int KM_matching(int now_size, int pred_size, std::vector<std::vector<int>> cost) {
    cout << "KM_matching_calculating..." << std::endl;
    m = now_size;
    n = pred_size;
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            weight[i][j] = cost[i][j];
            cout << weight[i][j] << ", ";
        }
        cout << endl;
    }
    // 执行KM算法
    cout << "dfs_count: ";
    KM();
    // 输出最大匹配的权值和
    int maxWeight = 0;
    for (int y = 0; y < n; ++y)
        if (matchY[y] != -1)
            maxWeight += weight[matchY[y]][y];

    cout << endl << "Maximum weight matching: " << maxWeight << endl;
    return 0;
}