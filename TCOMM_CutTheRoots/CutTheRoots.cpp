#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>

using namespace std;

class CutTheRoots {
public:
    vector<int> makeCuts(int NP, vector<int> points, vector<int> roots) {
        // first NP points give coordinates of the bases of the plants, written as x, y
        // get x-coordinates of the bases, sort them, make cuts along y-axis to separate x-coordinates
        vector<int> xs(NP);
        for (int i = 0; i < NP; ++i) {
            xs[i] = points[2 * i];
        }
        sort(xs.begin(), xs.end());
        vector<int> ret(4 * (NP - 1));
        for (int i = 0; i < NP - 1; ++i) {
            int x = (xs[i]+xs[i+1])/2;
            ret[4 * i] = x;
            ret[4 * i + 1] = 1;
            ret[4 * i + 2] = x;
            ret[4 * i + 3] = 2;
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

