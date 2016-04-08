#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <tuple>
#include <random>

using namespace std;

class CutTheRoots {
public:
    vector<int> makeCuts(int NP, vector<int> points, vector<int> roots) {
        // first NP points give coordinates of the bases of the plants, written as x, y
        // get x-coordinates of the bases, sort them, make cuts along y-axis to separate x-coordinates
        vector<pair<int,int>> xys(NP);
        for (int i = 0; i < NP; ++i) {
            xys[i].first  = points[2 * i];
            xys[i].second = points[2 * i + 1];
        }
        sort(xys.begin(), xys.end());
        
        double eps = 1e-1;
        random_device seed_gen;
        mt19937 engine(seed_gen());
        uniform_real_distribution<double> distRatio(0+eps, 1-eps);
        vector<int> ret(4 * (NP - 1));
        for (int i = 0; i < NP - 1; ++i) {
            double ratio = distRatio(engine);
            int a = ratio*xys[i].first + (1.0-ratio)*xys[i+1].first;
            int b = ratio*xys[i].second + (1.0-ratio)*xys[i+1].second;
            normal_distribution<double> distK(b, 1.0);
            int k = distK(engine);
            ret[4 * i] = a;
            ret[4 * i + 1] = b;
            if (k < 0 || k > 1024)
            {
                ret[4 * i + 2] = a;
                ret[4 * i + 3] = b + 1;
            }
            else
            {
                ret[4 * i + 2] = a + 1;
                ret[4 + i + 3] = k;
            }
        }
        return ret;
    }
};
// -------8<------- end of solution submitted to the website -------8<-------

template<class T> void getVector(vector<T>& v) {
    for (int i = 0; i < v.size(); ++i)
        cin >> v[i];
}

int main() {
    int NP;
    cin >> NP;

    int Npoints;
    cin >> Npoints;
    vector<int> points(Npoints);
    getVector(points);

    int Nroots;
    cin >> Nroots;
    vector<int> roots(Nroots);
    getVector(roots);

    CutTheRoots cr;
    vector<int> ret = cr.makeCuts(NP, points, roots);

    cout << ret.size() << endl;
    for (int i = 0; i < ret.size(); ++i) {
        cout << ret[i] << endl;
    }
    cout.flush();
}

