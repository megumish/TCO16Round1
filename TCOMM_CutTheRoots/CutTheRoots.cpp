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

using namespace std;

const int INFTY = 1e10;
class CutTheRoots
{
public:
    int NP;
    int NR;
    vector<int> points;
    vector<int> roots;
    vector<int> line;
    int numLine;
    vector<int> rightR;
    vector<int> leftR;
    vector<int> downR;
    vector<int> upR;
    vector<int> rightCnt;
    vector<int> leftCnt;
    vector<int> downCnt;
    vector<int> upCnt;
    vector<vector<int>> children;
    set<tuple<int, int>> linePoints;
    vector<int> group;
    int groupPos;
    int numOfGroup;
    vector<int> makeCuts(int NP, vector<int> points, vector<int> roots)
    {
        auto start = chrono::system_clock::now();
        this->NP = NP;
        this->NR = roots.size() / 2;
        this->points = points;
        this->roots = roots;
        this->line = vector<int>(4 * NP, -1);
        rightR = vector<int>(NP,0);
        leftR  = vector<int>(NP,0);
        downR  = vector<int>(NP,0);
        upR    = vector<int>(NP,0);
        children = vector<vector<int>>(NP + NR);
        random_device seedGen;
        mt19937 engine(seedGen());
        for (int i = 0; i < NR; i++)
        {
            children[roots[2 * i]].push_back(roots[2 * i + 1]);
        }
        double k = 1e-9;
        int cnt = 0;
        vector<int> ret(NP * 4);
        vector<int> decideRet;
        long long maxAllScore = -INFTY;
        double maxCnt = 270000 / (NP*NP);
        double maxK;
        cerr << maxCnt << endl;
        while (maxCnt > cnt)
        {
            cnt++;
            group = vector<int>(NP, 1);
            groupPos = 1;
            numOfGroup = 1;
            rightCnt = vector<int>(NP, 0);
            leftCnt = vector<int>(NP, 0);
            downCnt = vector<int>(NP, 0);
            upCnt = vector<int>(NP, 0);
            for (int i = 0; i < NP; i++)
            {
                int originX = points[2 * i];
                int originY = points[2 * i + 1];
                stack<int> s;
                s.push(i);
                while (!s.empty())
                {
                    int cur = s.top(); s.pop();
                    int curX = points[2 * cur];
                    int curY = points[2 * cur + 1];
                    if (curX >= originX)
                    {
                        rightR[i] = max(rightR[i], (int)sqrt((curX - originX)*(curX - originX) + (curY - originY)*(curY - originY)));
                        rightCnt[i]++;
                    }
                    else
                    {
                        leftR[i] = max(leftR[i], (int)sqrt((curX - originX)*(curX - originX) + (curY - originY)*(curY - originY)));
                        leftCnt[i]++;
                    }
                    if (curY >= originY)
                    {
                        downR[i] = max(downR[i], (int)sqrt((curX - originX)*(curX - originX) + (curY - originY)*(curY - originY)));
                        downCnt[i]++;
                    }
                    else
                    {
                        upR[i] = max(upR[i], (int)sqrt((curX - originX)*(curX - originX) + (curY - originY)*(curY - originY)));
                        upCnt[i]++;
                    }
                    for (int j = 0; j < children[cur].size(); j++)
                        s.push(children[cur][j]);
                }
                //cerr << i << endl;
                //cerr << "rightR:" << rightR[i] << endl;
                //cerr << "leftR:" << leftR[i] << endl;
                //cerr << "downR:" << downR[i] << endl;
                //cerr << "upR:" << upR[i] << endl;
            }
            uniform_real_distribution<double> dist(-M_PI / 4, 7 * M_PI / 4);
            for (int i = 0; i < NP; i++)
            {
                //cerr << i << endl;
                //cerr << "Group:" << endl;
                //for (int i = 0; i < NP; i++) cerr << group[i] << endl;
                vector<int> groupSorted = group;
                sort(groupSorted.begin(), groupSorted.end());
                vector<int> groupUnique;
                unique_copy(groupSorted.begin(), groupSorted.end(), back_inserter(groupUnique));
                numOfGroup = groupUnique.size();
                //cerr << "sepalated:" << numOfGroup << endl;
                int originX = points[2 * i];
                int originY = points[2 * i + 1];
                int maxContactX, maxContactY, maxAnotherX, maxAnotherY;
                vector<int> maxGroup;
                string maxContactLines;
                double maxScore = -INFTY;
                for (int deg = -45; deg <= 315; deg++)
                {
                    int contactX, contactY, anotherX, anotherY;
                    string contactLines;
                    int newRadius = 0;
                    double rad = (double)deg / 180.0 * M_PI;
                    if (rad <= M_PI / 4)
                    {
                        contactLines = "Right";
                        contactX = originX + sqrt(rightR[i]) * cos(rad);
                        contactY = originY + sqrt(rightR[i]) * sin(rad);
                        newRadius = rightR[i];
                    }
                    else if (rad <= 3 * M_PI / 4)
                    {
                        contactLines = "Down";
                        contactX = originX + sqrt(downR[i]) * cos(rad);
                        contactY = originY + sqrt(downR[i]) * sin(rad);
                        newRadius = downR[i];
                    }
                    else if (rad <= 5 * M_PI / 4)
                    {
                        contactLines = "Left";
                        contactX = originX + sqrt(leftR[i]) * cos(rad);
                        contactY = originY + sqrt(leftR[i]) * sin(rad);
                        newRadius = leftR[i];
                    }
                    else if (rad <= 7 * M_PI / 4)
                    {
                        contactLines = "Up";
                        contactX = originX + sqrt(upR[i]) * cos(rad);
                        contactY = originY + sqrt(upR[i]) * sin(rad);
                        newRadius = upR[i];
                    }
                    anotherX = contactX - (contactY - originY);
                    anotherY = contactY + (contactX - originX);
                    if (anotherX < 0 || anotherX >= 1024 || anotherY < 0 || anotherY >= 1024)
                    {
                        anotherX = contactX + (contactY - originY);
                        anotherY = contactY - (contactX - originX);
                    }
                    if (contactX < 0 || contactX >= 1024 || contactY < 0 || contactY >= 1024 ||
                        anotherX < 0 || anotherX >= 1024 || anotherY < 0 || anotherY >= 1024) continue;
                    if (linePoints.find(make_tuple(contactX, contactY)) != linePoints.end() ||
                        linePoints.find(make_tuple(anotherX, anotherY)) != linePoints.end()) continue;
                    if (contactX == anotherX && contactY == anotherY) continue;
                    double curScore = 0; /*(double)newRadius / (1000 * cnt / maxCnt);*/
                    //cerr << "RadiusScore:" << (double)newRadius << endl;
                    long long distScore = 0;
                    for (int j = 0; j < NP; j++)
                    {
                        //cerr << distScore << endl;
                        int x = points[2 * j];
                        int y = points[2 * j + 1];
                        int dist =
                            ((contactX - originX) * x + (contactY - originY) * y
                                - (contactY - originY) * contactY - (contactX - originX) * contactX)
                            / sqrt((contactX - originX)*(contactX - originX) + (contactY - originY)*(contactY - originY));
                        dist = abs(dist);
                        if (contactLines == "Right" || contactLines == "Left")
                        {
                            if (y < (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                            {
                                if ((contactX - originX)*(contactY - originY) >= 0)
                                {
                                    distScore += min(leftR[j], dist) * leftCnt[j];
                                    distScore += rightR[j]*rightCnt[i] + downR[j]*downCnt[j] + upR[j]*upCnt[j];
                                }
                                else
                                {
                                    distScore += min(rightR[j], dist) * rightCnt[j];
                                    distScore += leftR[j] * leftCnt[i] + downR[j] * downCnt[j] + upR[j] * upCnt[j];
                                }
                            }
                            else if (y > (double) - (contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                            {
                                if ((contactX - originX)*(contactY - originY) >= 0)
                                {
                                    distScore += min(rightR[j], dist) * rightCnt[j];
                                    distScore += leftR[j] * leftCnt[i] + downR[j] * downCnt[j] + upR[j] * upCnt[j];
                                }
                                else
                                {
                                    distScore += min(leftR[j], dist) * leftCnt[j];
                                    distScore += rightR[j] * rightCnt[i] + downR[j] * downCnt[j] + upR[j] * upCnt[j];
                                }
                            }
                        }
                        else
                        {
                            if (y < (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                            {
                                if ((contactX - originX)*(contactY - originY) >= 0)
                                {
                                    distScore += min(downR[j], dist) * downCnt[j];
                                    distScore += rightR[j] * rightCnt[i] + leftR[j] * leftCnt[j] + upR[j] * upCnt[j];
                                }
                                else
                                {
                                    distScore += min(upR[j], dist) * upCnt[j];
                                    distScore += rightR[j] * rightCnt[i] + leftR[j] * leftCnt[j] + downR[j] * downCnt[j];
                                }
                            }
                            else if (y > (double) - (contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                            {
                                if ((contactX - originX)*(contactY - originY) >= 0)
                                {
                                    distScore += min(upR[j], dist) * upCnt[j];
                                    distScore += rightR[j] * rightCnt[i] + leftR[j] * leftCnt[j] + downR[j] * downCnt[j];
                                }
                                else
                                {
                                    distScore += min(downR[j], dist) * downCnt[j];
                                    distScore += rightR[j] * rightCnt[i] + leftR[j] * leftCnt[j] + upR[j] * upCnt[j];
                                }
                            }
                        }
                    }
                    curScore += (double) distScore * distScore / (1e12 * k);
                    //cerr << "DistScore:" << (double)distScore  << endl;
                    vector<int> newGroup;
                    int newNumOfGroup = 0;
                    if (numOfGroup != NP)
                    {
                        newGroup = vector<int>(NP);
                        int pos = *max_element(group.begin(), group.end()) + 1;
                        map<int, int> allocated;
                        for (int j = 0; j < NP; j++)
                        {
                            int x = points[2 * j];
                            int y = points[2 * j + 1];
                            if (y > (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                                newGroup[j] = group[j];
                            else if (y < (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
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
                        vector<int> newGroupSorted = newGroup;
                        sort(newGroupSorted.begin(), newGroupSorted.end());
                        vector<int> newGroupUnique;
                        unique_copy(newGroupSorted.begin(), newGroupSorted.end(), back_inserter(newGroupUnique));
                        newNumOfGroup = newGroupUnique.size();
                    }
                    curScore += (newNumOfGroup - numOfGroup);
                    //cerr << "numOfGroupScore" << (double)newNumOfGroup / NP << endl;
                    if (maxScore < curScore)
                    {
                        maxScore = curScore;
                        maxContactX = contactX;
                        maxContactY = contactY;
                        maxAnotherX = anotherX;
                        maxAnotherY = anotherY;
                        maxContactLines = contactLines;
                        maxGroup = newGroup;
                    }
                }
                ret[4 * i] = maxContactX;
                ret[4 * i + 1] = maxContactY;
                ret[4 * i + 2] = maxAnotherX;
                ret[4 * i + 3] = maxAnotherY;
                linePoints.insert(make_tuple(maxContactX, maxContactY));
                linePoints.insert(make_tuple(maxAnotherX, maxAnotherY));
                if (numOfGroup != NP) group = maxGroup;
                //cerr << maxContactLines << endl;
                //cerr << "contactX:" << maxContactX << endl;
                //cerr << "contactY:" << maxContactY << endl;
                //cerr << "anotherX:" << maxContactX - (maxContactY - originY) << endl;
                //cerr << "anotherY:" << maxContactY + (maxContactX - originX) << endl;
                //cerr << maxScore << endl;
                for (int j = 0; j < NP; j++)
                {
                    int x = points[2 * j];
                    int y = points[2 * j + 1];
                    int dist =
                        ((maxContactX - originX) * x + (maxContactY - originY) * y
                            - (maxContactY - originY) * maxContactY - (maxContactX - originX) * maxContactX)
                        / sqrt((maxContactX - originX)*(maxContactX - originX) + (maxContactY - originY)*(maxContactY - originY));
                    dist = abs(dist);
                    if (maxContactLines == "Right" || maxContactLines == "Left")
                    {
                        if (y < (double)-(maxContactX - originX)*(x - maxContactX) / (maxContactY - originY) + maxContactY)
                        {
                            if ((maxContactX - originX)*(maxContactY - originY) >= 0)
                            {
                                leftR[j] = min(leftR[j], dist);
                            }
                            else
                            {
                                rightR[j] = min(rightR[j], dist);
                            }
                        }
                        else if (y > (double) - (maxContactX - originX)*(x - maxContactX) / (maxContactY - originY) + maxContactY)
                        {
                            if ((maxContactX - originX)*(maxContactY - originY) >= 0)
                            {
                                leftR[j] = min(leftR[j], dist);
                            }
                            else
                            {
                                rightR[j] = min(rightR[j], dist);
                            }
                        }
                    }
                    else
                    {
                        if (y < (double)-(maxContactX - originX)*(x - maxContactX) / (maxContactY - originY) + maxContactY)
                        {
                            if ((maxContactX - originX)*(maxContactY - originY) >= 0)
                            {
                                downR[j] = min(downR[j], dist);
                            }
                            else
                            {
                                upR[j] = min(upR[j], dist);
                            }
                        }
                        else if (y > (double) - (maxContactX - originX)*(x - maxContactX) / (maxContactY - originY) + maxContactY)
                        {
                            if ((maxContactX - originX)*(maxContactY - originY) >= 0)
                            {
                                downR[j] = min(downR[j], dist);
                            }
                            else
                            {
                                upR[j] = min(upR[j], dist);
                            }
                        }
                    }
                }
            }
            if (numOfGroup == NP)
            {
                long long curAllScore = 0;
                for (int i = 0; i < NP; i++)
                {
                    curAllScore += rightR[i] + leftR[i] + downR[i] + upR[i];
                }
                if (curAllScore > maxAllScore)
                {
                    maxAllScore = curAllScore;
                    decideRet = ret;
                    maxK = k;
                }
            }
            auto time = chrono::system_clock::now() - start;
            cerr << time.count()/1e7 << endl;
            k = 1 - (double)cnt /maxCnt;
        }
        auto end = chrono::duration_cast<chrono::seconds>(chrono::system_clock::now() - start);
        cerr << "time:" << end.count() << endl;
        cerr << maxK << endl;
        return decideRet;
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

