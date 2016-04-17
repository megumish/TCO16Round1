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
    Line(Point p1, Point p2) { this->p1 = p1; this->p2 = p2; }
    Line(double x1, double y1, double x2, double y2) { this->p1 = Point(x1, y1); this->p2 = Point(x2, y2); }
};

class State
{
public:
    vector<int> points;
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
        int x = state.points[2 * numOfPoint];
        int y = state.points[2 * numOfPoint + 1];
        double score = 0;
        for (auto& numOfChild : state.children[numOfPoint])
        {
            score += state.pointScores[numOfChild];
            int childX = state.points[2 * numOfChild];
            int childY = state.points[2 * numOfChild + 1];
            score += sqrt((x - childX)*(x - childX) + (y - childY)*(y - childY));
        }
        state.pointScores[numOfPoint] = score;
    }

    vector<int> makeCuts(const int NP, vector<int> ps, vector<int> rs)
    {
        int NR = rs.size() / 2;
        vector<vector<int>> children(NP + NR);
        for (int i = 0; i < NR; i++) children[rs[2 * i]].push_back(rs[2 * i + 1]);
        vector<Line> lines(NP-1,Line(Point(-1,-1),Point(-1,-1)));
        vector<int> group(NP, 0);
        int numOfGroup = 1;
        random_device seed_gen;
        mt19937 engine(seed_gen());
        uniform_int_distribution<> distCoordinate(0, 1024);
        State state;
        state.points = ps;
        state.children = children;
        state.group = group;
        state.lines = lines;
        state.pointScores = vector<double>(NP + NR, 0.);
        for (int i = 0; i < NP; i++) calculatePointScores(state, i);
        double rootsLength = 0;
        for (int i = 0; i < NP; i++) rootsLength += state.pointScores[i];
        cerr << rootsLength << endl;
        state.points = ps;
        state.children = children;
        state.group = group;
        state.lines = lines;
        state.pointScores = vector<double>(NP + NR, 0.);
        group = vector<int>(NP, 0);
        numOfGroup = 1;
        for (int i = 0; i < NP - 1; i++)
        {
            auto newLine = Line(distCoordinate(engine), distCoordinate(engine), distCoordinate(engine), distCoordinate(engine));
            vector<int> newGroup = group;
            while (numOfGroup <= i + 1)
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
                group = newGroup;
            }
            for (int i = 0; i < NP; i++) cutAndMakeNewChild(state, newLine, -1, i);

            lines[i] = newLine;
        }
        cerr << "end" << endl;
        for (int i = 0; i < NP; i++) calculatePointScores(state, i);
        rootsLength = 0;
        for (int i = 0; i < NP; i++) rootsLength += state.pointScores[i];
        cerr << rootsLength << endl;
        vector<int> ret(4 * (NP - 1));
        for (int i = 0; i < NP - 1; i++)
        {
            auto line = lines[i];
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

