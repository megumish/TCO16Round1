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
    vector<vector<int>> children;
    set<tuple<int, int>> linePoints;
    vector<int> group;
    int groupPos;
    int numOfGroup;
    
    vector<int> makeCuts(int NP, vector<int> points, vector<int> roots)
    {
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
        group = vector<int>(NP,1);
        groupPos = 1;
        numOfGroup = 1;
        random_device seedGen;
        mt19937 engine(seedGen());
        for (int i = 0; i < NR; i++)
        {
            children[roots[2 * i]].push_back(roots[2 * i + 1]);
        }

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
                    rightR[i] = max(rightR[i], (curX - originX)*(curX - originX) + (curY - originY)*(curY - originY));
                else
                    leftR[i] = max(leftR[i], (curX - originX)*(curX - originX) + (curY - originY)*(curY - originY));
                if (curY >= originY)
                    downR[i] = max(downR[i], (curX - originX)*(curX - originX) + (curY - originY)*(curY - originY));
                else
                    upR[i] = max(upR[i], (curX - originX)*(curX - originX) + (curY - originY)*(curY - originY));
                for (int j = 0; j < children[cur].size(); j++)
                    s.push(children[cur][j]);
            }
            cerr << i << endl;
            cerr << "rightR:" << rightR[i] << endl;
            cerr << "leftR:" << leftR[i] << endl;
            cerr << "downR:" << downR[i] << endl;
            cerr << "upR:" << upR[i] << endl;
        }
        vector<int> ret(NP * 4);
        uniform_real_distribution<double> dist(- M_PI / 4,  7 * M_PI / 4);
        for (int i = 0; i < NP; i++)
        {
            cerr << i << endl;
            cerr << "Group:" << endl;
            for (int i = 0; i < NP; i++) cerr << group[i] << endl;
            vector<int> groupSorted = group;
            sort(groupSorted.begin(), groupSorted.end());
            vector<int> groupUnique;
            unique_copy(groupSorted.begin(), groupSorted.end(), back_inserter(groupUnique));
            numOfGroup = groupUnique.size();
            cerr << "sepalated:" << numOfGroup << endl;
            int originX = points[2 * i];
            int originY = points[2 * i + 1];
            int maxContactX, maxContactY, maxAnotherX, maxAnotherY;
            vector<int> maxGroup;
            string maxContactLines;
            int maxScore = -INFTY;
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
                double curScore =(double) newRadius/(1024*1024);
                //cerr << "RadiusScore:" << (double)newRadius / (1024 * 1024) << endl;
                int distScore = 0;
                for (int j = 0; j < NP; j++)
                {
                    int x = points[2 * j];
                    int y = points[2 * j + 1];
                    int dist =
                        (double)((contactX - originX) * points[2 * i] + (contactY - originY) * points[2 * i + 1]
                            - (contactY - originY) * contactY - (contactX - originX) * contactX)
                        *((contactX - originX)*points[2 * i] + (contactY - originY) * points[2 * i + 1]
                            - (contactY - originY) * contactY - (contactX - originX) * contactX)
                        / (contactX - originX)*(contactX - originX) + (contactY - originY)*(contactY - originY);
                    if (contactLines == "Right" || contactLines == "Left")
                    {
                        if (y < (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                        {
                            if ((contactX - originX)*(contactY - originY) >= 0)
                            {
                                distScore += min(0,leftR[i] - dist);
                            }
                            else
                            {
                                distScore += min(0,rightR[i] - dist);
                            }
                        }
                        else if (y > (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                        {
                            if ((contactX - originX)*(contactY - originY) >= 0)
                            {
                                distScore += min(0,rightR[i] - dist);
                            }
                            else
                            {
                                distScore += min(0,leftR[i] - dist);
                            }
                        }
                        else continue;
                    }
                    else
                    {
                        if (y < (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                        {
                            if ((contactX - originX)*(contactY - originY) >= 0)
                            {
                                distScore += min(0,downR[i] - dist);
                            }
                            else
                            {
                                distScore += min(0,upR[i] - dist);
                            }
                        }
                        else if (y > (double)-(contactX - originX)*(x - contactX) / (contactY - originY) + contactY)
                        {
                            if ((contactX - originX)*(contactY - originY) >= 0)
                            {
                                distScore += min(0,upR[i] - dist);
                            }
                            else
                            {
                                distScore += min(0,downR[i] - dist);
                            }
                        }
                        else continue;
                    }
                }
                curScore += (double)distScore / (1024 * 1024 * NP);
                //cerr << "DistScore:" << (double)distScore / (1024 * 1024 * NP) << endl;
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
                    sort(newGroupSorted.begin(),newGroupSorted.end());
                    vector<int> newGroupUnique;
                    unique_copy(newGroupSorted.begin(), newGroupSorted.end(), back_inserter(newGroupUnique));
                    newNumOfGroup = newGroupUnique.size();
                }
                curScore += (double) newNumOfGroup/NP * (max(0,(i+1)-numOfGroup)*100);
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
            if(numOfGroup != NP) group = maxGroup;
            cerr << maxContactLines << endl;
            cerr << "contactX:" << maxContactX << endl;
            cerr << "contactY:" << maxContactY << endl;
            cerr << "anotherX:" << maxContactX - (maxContactY - originY) << endl;
            cerr << "anotherY:" << maxContactY + (maxContactX - originX) << endl;
            cerr << maxScore << endl;
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

