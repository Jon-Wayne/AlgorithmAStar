#ifndef __ALGORITHM_ASTAR_H__
#define __ALGORITHM_ASTAR_H__

#include "cocos2d.h"
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;
using namespace cocos2d;

typedef struct Node Node;
struct Node
{
	int x;
	int y;
	int gid;
	int g;
	int h;
	int f;
	Node *parent;
};

class PathNode : public CCObject
{
public:
    static PathNode *create()
    {
        PathNode *obj = new PathNode();
        obj->autorelease();
        return obj;
    }
public:
    int y;
    int x;
    int gid;
public:
    int getX(){ return x; }
    int getY(){ return y; }
    int getGid(){ return gid; }
};

class AlgorithmAStar : public CCObject
{
public:
	AlgorithmAStar();
	~AlgorithmAStar();
    
    static AlgorithmAStar *getInstance();

    void setMapInfo(CCTMXTiledMap *map);
    
	CCArray *search(CCPoint &start, CCPoint &end);
private:
	Node *getMinFNodeFromOpenTableAndPushIntoCloesdTable(
										vector<Node *> &openTable, 
										vector<Node *> &closedTable);
	int getHWithManhattanDistance(int y, int x, CCPoint end);
	vector<Node *> *getChildNodesFromCurrNode(Node *currNode);
	bool isInTable(vector<Node *> &table, int y, int x);
	Node *getShouldFixedNode(vector<Node *> &openTable, int y, int x);
private:
	CCTMXTiledMap *m_pTiledMap;
    int *m_pMap;
	int m_nRow;
	int m_nCol;
};

#endif //__ALGORITHM_ASTAR_H__