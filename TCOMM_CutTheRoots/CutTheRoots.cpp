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
#include <iomanip>
#include <queue>

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
    Point solveEquation(const double& a, const double& b, const double& e, const double& c, const double& d, const double& f)
    {
        if (a*d == b*c) return{ -1,-1 };
        return Point((b*f - e*d) / (b*c - a*d), (e*c - a*f) / (b*c - a*d));
    }

    Point getIntersection(const Point& p1, const Point& p2, const Point& q1, const Point& q2)
    {
        return solveEquation(p2.y - p1.y, p1.x - p2.x, (p2.y - p1.y)*p1.x + (p1.x - p2.x)*p1.y,
            q2.y - q1.y, q1.x - q2.x, (q2.y - q1.y)*q1.x + (q1.x - q2.x)*q1.y);
    }

    bool isOnTheLine(const Line& line, const Point& p)
    {
        return p.y >= (line.p2.y - line.p1.y) / (line.p2.x - line.p1.x)*(p.x - line.p1.x) + line.p1.y;
    }

    long long called = 0;
    void cutAndMakeNewChild(State& state, const Line& line, const int& numOfParent, const int& numOfPoint, int depth, const int& depthLimit)
    {
        called++;
        if (depthLimit >= depth)
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
                cutAndMakeNewChild(state, line, numOfPoint, numOfChild, depth + 1, depthLimit);
            }
        }
    }

    void calculatePointScores(State& state, const int& numOfPoint)
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
        int maxDepth = 0;
        queue<pair<int, int>> q;
        for (int i = 0; i < NP; i++) q.push(make_pair(i, 0));
        while (!q.empty())
        {
            int numOfPoint, depth;
            tie(numOfPoint, depth) = q.front(); q.pop();
            maxDepth = max(maxDepth, depth);
            for (auto& numOfChild : state.children[numOfPoint])
            {
                q.push(make_pair(numOfChild, depth + 1));
            }
        }
        cerr << maxDepth << endl;
        random_device seed_gen;
        mt19937 engine(seed_gen());
        uniform_int_distribution<> distCoordinate(0, 1024);
        uniform_int_distribution<> distNumOfLine(0, NP - 2);
        vector<vector<Line>> groupOfLines;
        auto start = chrono::system_clock::now();
        for (int cnt = 0; cnt < 10; cnt++)
        {
            int numOfGroup = 1;
            auto group = vector<int>(NP, 0);
            auto lines = vector<Line>(NP - 1);
            for (int i = 0; i < NP - 1; i++)
            {
                auto newLine = Line(distCoordinate(engine), distCoordinate(engine), distCoordinate(engine), distCoordinate(engine));
                vector<int> newGroup = group;
                while (numOfGroup != NP && numOfGroup <= i + 1)
                {
                    newLine = Line(distCoordinate(engine), distCoordinate(engine), distCoordinate(engine), distCoordinate(engine));
                    newGroup = vector<int>(NP);
                    int pos = *max_element(group.begin(), group.end()) + 1;
                    map<int, int> allocated;
                    for (int j = 0; j < NP; j++)
                    {
                        if (isOnTheLine(newLine, Point(ps[2 * j], ps[2 * j + 1])))
                            newGroup[j] = group[j];
                        else
                        {
                            if (allocated.find(group[j]) != allocated.end())
                                newGroup[j] = allocated[group[j]];
                            else
                            {
                                newGroup[j] = pos;
                                allocated[group[j]] = pos;
                                pos++;
                            }
                        }
                    }
                    set<int> groupSet;
                    for (auto& g : newGroup)
                    {
                        groupSet.insert(g);
                    }
                    numOfGroup = groupSet.size();
                }
                group = newGroup;
                lines[i] = newLine;
            }
            groupOfLines.push_back(lines);
        }
        vector<Line> retLines(NP - 1);
        double maxScore = -INFTY;
        map<double, vector<Line>, greater<double>> rank;
        uniform_real_distribution<> dist(0, 1);
        int maxGeneration = INFTY;
        int firstSize = groupOfLines.size();
        int sumGap = 0;
        for (int generation = 0; generation < maxGeneration; generation++)
        {
            rank.clear();
            for (int i = 0; i < groupOfLines.size(); i++)
            {
                state.lines = groupOfLines[i];
                state.children = initChildren;
                state.group = vector<int>(NP, 0);
                state.points = initPoints;
                state.pointScores = vector<double>(NP + NR);
                for (int j = 0; j < NP - 1; j++)
                {
                    auto prevGroup = state.group;
                    int pos = *max_element(prevGroup.begin(), prevGroup.end()) + 1;
                    map<int, int> allocated;
                    for (int k = 0; k < NP; k++)
                    {
                        if (isOnTheLine(state.lines[j], Point(state.points[2 * k], state.points[2 * k + 1])))
                            state.group[k] = prevGroup[k];
                        else
                        {
                            if (allocated.find(prevGroup[k]) != allocated.end())
                                state.group[k] = allocated[prevGroup[k]];
                            else
                            {
                                state.group[k] = pos;
                                allocated[prevGroup[k]] = pos;
                                pos++;
                            }
                        }
                    }
                }
                int depth = min((double)maxDepth, max(2., maxDepth * pow(M_E, 80 - NP)));
                for (int j = 0; j < (NP - 1); j++)
                {
                    for (int k = 0; k < NP; k++)
                    {
                        cutAndMakeNewChild(state, state.lines[j], -1, k, 0, depth);
                    }
                }
                int numOfGroup = 0;
                set<int> groupSet;
                for (auto& g : state.group)
                {
                    groupSet.insert(g);
                }
                numOfGroup = groupSet.size();
                for (int j = 0; j < NP; j++) calculatePointScores(state, j);
                double curRootsLength = 0;
                for (int j = 0; j < NP; j++) curRootsLength += state.pointScores[j];
                double curScore = numOfGroup + curRootsLength / allRootsLength;
                rank[curScore] = state.lines;
            }
            groupOfLines.clear();
            int i = 0;
            auto winnerPair = *rank.begin();
            cerr << generation << ":" << fixed << setprecision(6) << winnerPair.first << endl;
            for (auto& linesPair1 : rank)
            {
                groupOfLines.push_back(linesPair1.second);
                if (i == max(2, 60 / NP)) break;
                vector<Line> newLines;
                int j = 0;
                for (auto& linesPair2 : rank)
                {
                    if (linesPair2.first < NP) break;
                    if (i != j)
                    {
                        newLines = linesPair2.second;
                        double sum = linesPair1.first + linesPair2.first;
                        double mutationRate = min(1., abs(linesPair1.first - linesPair2.first));
                        vector<bool> selected(NP - 1, false);
                        for (int k = 0; k < (NP - 1)*0.87; k++)
                        {
                            int numOfLine = distNumOfLine(engine);
                            while (selected[numOfLine]) numOfLine = distNumOfLine(engine);
                            selected[numOfLine] = true;
                            if (dist(engine) <= mutationRate) newLines[numOfLine].p1.x = distCoordinate(engine);
                            else newLines[k].p1.x = linesPair1.second[numOfLine].p1.x;

                            if (dist(engine) <= mutationRate) newLines[numOfLine].p1.y = distCoordinate(engine);
                            else newLines[k].p1.y = linesPair1.second[numOfLine].p1.y;

                            if (dist(engine) <= mutationRate) newLines[numOfLine].p2.x = distCoordinate(engine);
                            else newLines[k].p2.x = linesPair1.second[numOfLine].p2.x;

                            if (dist(engine) <= mutationRate) newLines[numOfLine].p2.y = distCoordinate(engine);
                            else newLines[k].p2.y = linesPair1.second[numOfLine].p2.y;
                        }
                        groupOfLines.push_back(newLines);
                    }
                    ++j;
                }
                ++i;
            }
            int secondSize = groupOfLines.size();
            sumGap += abs(firstSize - secondSize);
            maxGeneration = max(5., NP*0.8 - (sumGap / (generation+1)));
            cerr << "MaxGeneration:" << maxGeneration << endl;
            firstSize = secondSize;
            cerr << groupOfLines.size() << endl;
        }
        auto winnerPair = *rank.begin();
        retLines = winnerPair.second;
        vector<int> ret(4 * (NP - 1));
        for (int i = 0; i < NP - 1; i++)
        {
            auto line = retLines[i];
            ret[4 * i] = line.p1.x;
            ret[4 * i + 1] = line.p1.y;
            ret[4 * i + 2] = line.p2.x;
            ret[4 * i + 3] = line.p2.y;
        }
        cerr << called << endl;
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

