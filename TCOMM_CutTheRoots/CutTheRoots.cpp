#define _USE_MATH_DEFINES
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <map>
#include <sstream>
#include <vector>
#include <tuple>
#include <random>
#include <stack>
#include <cmath>
#include <set>
#include <iterator>
#include <chrono>
#include <functional>

using namespace std;

const int INFTY = 1e10;

class Point
{
public:
    double x, y;
    Point() {};
    Point(double x, double y) { this->x = x; this->y = y; }
};

class Line
{
public:
    Point p1, p2;
    Line() {};
    Line(Point p1, Point p2) { this->p1 = p1; this->p2 = p2; }
    Line(double x1, double y1, double x2, double y2) { this->p1 = Point(x1, y1); this->p2 = Point(x2, y2); }
};

class State
{
public:
    vector<double> points;
    vector<vector<int>> children;
    vector<Line> lines;
    vector<int> group;
    vector<double> pointScores;
};
class CutTheRoots
{
public:

    // ax + by = e
    // cx + dy = f
    Point solveEquation(double a, double b, double e, double c, double d, double f)
    {
        if (a*d == b*c) return{ -1,-1 };
        return Point((b*f - e*d) / (b*c - a*d),(e*c - a*f) / (b*c - a*d));
    }

    Point getIntersection(Point p1,Point p2,Point q1,Point q2)
    {
        return solveEquation(p2.y-p1.y,p1.x-p2.x,(p2.y-p1.y)*p1.x+(p1.x-p2.x)*p1.y,
                             q2.y-q1.y,q1.x-q2.x,(q2.y-q1.y)*q1.x+(q1.x-q2.x)*q1.y);
    }

    bool isOnTheLine(Line line, Point p)
    {
        return p.y >= (line.p2.y - line.p1.y) / (line.p2.x - line.p1.x)*(p.x - line.p1.x) + line.p1.y;
    }

    void cutAndMakeNewChild(State& state,Line line,int numOfParent,int numOfPoint)
    {
        if (numOfParent != -1)
        {
            auto parentP = Point(state.points[2 * numOfParent], state.points[2 * numOfParent + 1]);
            auto p = Point(state.points[2 * numOfPoint], state.points[2 * numOfPoint + 1]);
            auto intersection = getIntersection(line.p1, line.p2, parentP, p);
            if ((p.x - intersection.x)*(parentP.x - intersection.x) <= 0 && (p.y - intersection.y)*(parentP.y - intersection.y) <= 0)
            {
                state.children[numOfPoint].clear();
                state.points[2 * numOfPoint] = intersection.x;
                state.points[2 * numOfPoint + 1] = intersection.y;
                return;
            }
        }
        for (auto& numOfChild : state.children[numOfPoint])
        {
            cutAndMakeNewChild(state, line, numOfPoint, numOfChild);
        }
    }

    void calculatePointScores(State& state, int numOfPoint)
    {
        if (state.children[numOfPoint].empty())
        {
            state.pointScores[numOfPoint] = 0;
            return;
        }
        for (auto& numOfChild : state.children[numOfPoint]) calculatePointScores(state, numOfChild);
        double x = state.points[2 * numOfPoint];
        double y = state.points[2 * numOfPoint + 1];
        double score = 0;
        for (auto& numOfChild : state.children[numOfPoint])
        {
            score += state.pointScores[numOfChild];
            double childX = state.points[2 * numOfChild];
            double childY = state.points[2 * numOfChild + 1];
            score += sqrt((x - childX)*(x - childX) + (y - childY)*(y - childY));
        }
        state.pointScores[numOfPoint] = score;
    }
    vector<int> makeCuts(const int NP, vector<int> ps, vector<int> rs)
    {
        int NR = rs.size() / 2;
        State state;
        state.children = vector<vector<int>>(NP + NR);
        for (int i = 0; i < NR; i++) state.children[rs[2 * i]].push_back(rs[2 * i + 1]);
        auto initChildren = state.children;
        state.points = vector<double>(ps.size());
        for (int i = 0; i < ps.size(); i++) state.points[i] = ps[i];
        auto initPoints = state.points;
        state.pointScores = vector<double>(NP + NR, 0);
        state.lines = vector<Line>(NP - 1);
        for (int i = 0; i < NP; i++) calculatePointScores(state, i);
        double allRootsLength = 0;
        for (int i = 0; i < NP; i++) allRootsLength += state.pointScores[i];
        random_device seed_gen;
        mt19937 engine(seed_gen());
        uniform_int_distribution<> distCoordinate(0, 1024);
        uniform_int_distribution<> distNumOfLine(0, NP - 2);
        state.group = vector<int>(NP);
        int numOfGroup = 1;
        int beamWidth = 1;
        map<double, State, greater<double>> m;
        m[-INFTY] = state;
        for (int i = 0; i < NP - 1; i++)
        {
            map<double, State, greater<double>> newM;
            for (auto& prevStatePair : m)
            {
                bool ok = false;
                auto prevState = prevStatePair.second;
                for (int cnt = 0; cnt < beamWidth; cnt++)
                {
                    auto newState = prevState;
                    newState.lines[i] = Line(distCoordinate(engine), distCoordinate(engine), distCoordinate(engine), distCoordinate(engine));
                    newState.group = vector<int>(NP);
                    int newNumOfGroup = 0;
                    int pos = *max_element(prevState.group.begin(), prevState.group.end()) + 1;
                    map<int, int> allocated;
                    for (int j = 0; j < NP; j++)
                    {
                        if (isOnTheLine(newState.lines[i], Point(prevState.points[2 * j], prevState.points[2 * j + 1])))
                            newState.group[j] = prevState.group[j];
                        else
                        {
                            if (allocated.find(prevState.group[j]) != allocated.end())
                                newState.group[j] = allocated[prevState.group[j]];
                            else
                            {
                                newState.group[j] = pos;
                                allocated[prevState.group[j]] = pos;
                                pos++;
                            }
                        }
                    }
                    set<int> groupSet;
                    for (auto& g : newState.group)
                    {
                        groupSet.insert(g);
                    }
                    newNumOfGroup = groupSet.size();
                    for (int j = 0; j < NP; j++) cutAndMakeNewChild(newState, newState.lines[i], -1, j);
                    for (int j = 0; j < NP; j++) calculatePointScores(newState, j);
                    double rootsLength = 0;
                    for (int j = 0; j < NP; j++) rootsLength += newState.pointScores[j];
                    if (newNumOfGroup == NP || newNumOfGroup > i + 1)
                    {
                        //cerr << newNumOfGroup << endl;
                        newM[rootsLength] = newState;
                    }
                    else if (ok && i != NP - 1 && newNumOfGroup > sqrt(i))
                    {
                        newM[rootsLength] = newState;
                    }
                    if (cnt == beamWidth - 1 && newM.empty()) cnt = -1;
                }
            }
            m.clear();
            int k = 0;
            for (auto& newStatePair : newM)
            {
                if (k == beamWidth) break;
                //cerr << k << ":" << newStatePair.first << endl;
                m[newStatePair.first] = newStatePair.second;
                k++;
            }
        }
        vector<int> ret(4 * (NP - 1));
        auto head = *m.begin();
        if (m.empty()) cerr << "empty" << endl;
        state.lines = head.second.lines;
        double maxScore = -INFTY;
        uniform_int_distribution<> distD(-100, 100);
        for (int cnt = 0; cnt < 50000; cnt++)
        {
            state.children = initChildren;
            state.points = initPoints;
            state.pointScores = vector<double>(NP + NR, 0);
            auto newState = state;
            newState.group = vector<int>(NP);
            int numOfLine = distNumOfLine(engine);
            int x1 = newState.lines[numOfLine].p1.x + distD(engine);
            int y1 = newState.lines[numOfLine].p1.y + distD(engine);
            int x2 = newState.lines[numOfLine].p2.x + distD(engine);
            int y2 = newState.lines[numOfLine].p2.y + distD(engine);
            while (x1 < 0 || x1 > 1024) x1 = newState.lines[numOfLine].p1.x + distD(engine);
            while (y1 < 0 || y1 > 1024) y1 = newState.lines[numOfLine].p1.y + distD(engine);
            while (x2 < 0 || x2 > 1024) x2 = newState.lines[numOfLine].p2.x + distD(engine);
            while (y2 < 0 || y2 > 1024) y2 = newState.lines[numOfLine].p2.y + distD(engine);
            newState.lines[numOfLine] = Line(x1,y1,x2,y2);
            vector<int> prevGroup;
            for (int i = 0; i < NP - 1; i++)
            {
                prevGroup = newState.group;
                newState.group = vector<int>(NP);
                int pos = *max_element(prevGroup.begin(), prevGroup.end()) + 1;
                map<int, int> allocated;
                for (int j = 0; j < NP; j++)
                {
                    if (isOnTheLine(newState.lines[i], Point(newState.points[2 * j], newState.points[2 * j + 1])))
                        newState.group[j] = prevGroup[j];
                    else
                    {
                        if (allocated.find(prevGroup[j]) != allocated.end())
                            newState.group[j] = allocated[prevGroup[j]];
                        else
                        {
                            newState.group[j] = pos;
                            allocated[prevGroup[j]] = pos;
                            pos++;
                        }
                    }
                }
                for (int j = 0; j < NP; j++) cutAndMakeNewChild(newState, newState.lines[i], -1, j);
            }
            int newNumOfGroup = 0;
            set<int> groupSet;
            for (auto& g : newState.group)
            {
                groupSet.insert(g);
            }
            newNumOfGroup = groupSet.size();
            for (int i = 0; i < NP; i++) calculatePointScores(newState, i);
            double curRootsLength = 0;
            for (int i = 0; i < NP; i++) curRootsLength += newState.pointScores[i];
            double curScore = newNumOfGroup + curRootsLength/allRootsLength;
            if (curScore >= maxScore)
            {
                maxScore = curScore;
                state = newState;
                cerr << cnt << ":" << curRootsLength << endl;
            }
        }
        for (int i = 0; i < NP - 1; i++)
        {
            auto line = state.lines[i];
            ret[4 * i] = line.p1.x;
            ret[4 * i + 1] = line.p1.y;
            ret[4 * i + 2] = line.p2.x;
            ret[4 * i + 3] = line.p2.y;
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

    //cerr << NP << endl;
    //cerr << Npoints << endl;
    //for (int i = 0; i < points.size(); i++) cerr << points[i] << endl;
    //cerr << Nroots << endl;
    //for (int i = 0; i < roots.size(); i++) cerr << roots[i] << endl;
    //cerr << "input is end" << endl;

    CutTheRoots cr;
    vector<int> ret = cr.makeCuts(NP, points, roots);

    cout << ret.size() << endl;
    for (int i = 0; i < ret.size(); ++i) {
        cout << ret[i] << endl;
    }
    cout.flush();
}

