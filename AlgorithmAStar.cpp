#include "AlgorithmAStar.h"

static AlgorithmAStar *_instance = NULL;

AlgorithmAStar::AlgorithmAStar()
{
    m_pMap = NULL;
    m_nRow = 0;
    m_nCol = 0;
}


AlgorithmAStar::~AlgorithmAStar()
{
}

AlgorithmAStar *AlgorithmAStar::getInstance()
{
    if (!_instance) {
        _instance = new AlgorithmAStar();
        _instance->retain();
    }
    
    return _instance;
}

void AlgorithmAStar::setMapInfo(CCTMXTiledMap *map)
{
    m_pTiledMap = map;
    m_nRow = map->getMapSize().width;
    m_nCol = map->getMapSize().height;
    m_pMap = new int[m_nRow * m_nCol];
    
    CCTMXLayer *mapLayer = m_pTiledMap->layerNamed("layer1");
    CCPoint coordinate;
    for (int y=0; y<m_nRow; ++y)
    {
        coordinate.y = y;
        for (int x=0; x<m_nCol; ++x)
        {
            coordinate.x = x;
            m_pMap[y*m_nCol + x] = mapLayer->tileGIDAt(coordinate);
            printf("%d ", m_pMap[y*m_nCol + x]);
        }
        printf("\n");
    }
}

CCArray *AlgorithmAStar::search(CCPoint &start, CCPoint &end)
{   
    if ((int)start.x == (int)end.x && (int)start.y == (int)end.y)
        return NULL;

    //CCLog(@"------------- map data array ---------------");
    /*for (int y=0; y<m_nRow; ++y)
    {
        for (int x=0; x<m_nCol; ++x)
        {
            printf("%d ", m_pMap[y*m_nCol + x]);
        }
        printf("\n");
    }*/
    
    vector<Node *> openTable;
    vector<Node *> closedTable;
    
    //设置起点
    Node *startNode = (Node *)malloc(sizeof(Node));
    startNode->y = (int)start.y;
    startNode->x = (int)start.x;
    startNode->gid = m_pMap[startNode->y*m_nCol + startNode->x];
    startNode->parent = NULL;

    //采用曼哈顿距离来估算
    startNode->g = 0;
    startNode->h = getHWithManhattanDistance(startNode->y, startNode->x, end);
    startNode->f = 0;
    openTable.push_back(startNode);
    //cout<<"放入("<<startNode->y<<", "<<startNode->x<<") - "<<startNode->f;
    
    Node *overNode = NULL;

    //两个终止条件：1.列表为空，此时为无法到达 2.终节点被添加到开放列表中，此时为找到了所求路径
    //循环结束条件1的判定
    while (!openTable.empty())
    {
        //从openTable中找到f值最小的节点，把当前节点从openTable中删除，加入到closedTable中，作为当前节点并返回其指针
        Node *currNode = getMinFNodeFromOpenTableAndPushIntoCloesdTable(openTable, closedTable);
        //cout<<"当前节点：("<<currNode->y<<", "<<currNode->x<<")-"<<currNode->g<<", "<<currNode->h<<", "<<currNode->f<<"\n";
        
        //循环结束条件2的判定
        if (currNode->y==(int)end.y && currNode->x==(int)end.x)
        {
            overNode = currNode;
            //cout<<"找到了终节点\n";
            break;
        }
        
        //找出该节点的所有子节点
        vector<Node *> *childNodes = getChildNodesFromCurrNode(currNode);

        //并遍历各节点继续查找
        for (int i=0; i<childNodes->size(); i++)
        {
            Node *node = (*childNodes)[i];
            
            //若该节点不可通行，则不采取任何操作
            if (node->gid == 0)
            {
                //cout<<" 当前子节点：("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"是无效节点\n";
                //该无效节点的指针由malloc创建，但是又不会被加入到列表里，故现在就释放
                free(node);
                node = NULL;
                continue;
            }
            
            //若该节点已在closedTable中，则不采取任何操作
            if (isInTable(closedTable, node->y, node->x))
            {
                //cout<<" 当前子节点：("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"已在closed\n";
                //该无效节点的指针由malloc创建，因为已经存在于close中，是又不会被加入到close表里的，故现在就释放
                free(node);
                node = NULL;
                continue;
            }
            
            //若该节点不在openTable中
            if (!isInTable(openTable, node->y, node->x))
            {
                node->parent = currNode;
                int g = (currNode->y == node->y || currNode->x == node->x) ? 10 : 14;
                node->g = currNode->g + g;
                node->h = getHWithManhattanDistance(node->y, node->x, end);
                node->f = node->g + node->h;
                openTable.push_back(node);
                //cout<<" 当前子节点：("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"被放入open\n";
            }
            //若该节点已经在openTable中，且满足修改条件，则修改表中的那个，现有的这个释放
            else
            {
                int g = (currNode->y == node->y || currNode->x == node->x) ? 10 : 14;
                if ( currNode->g + g < node->g) {
                    //取出表里的那个节点的指针
                    Node *shouldFixedNode = getShouldFixedNode(openTable, node->y, node->x);
                    //cout<<" 当前子节点：("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"已在open，需要修改";
                    shouldFixedNode->parent = currNode;
                    shouldFixedNode->g = currNode->g + g;
                    shouldFixedNode->f = shouldFixedNode->g + shouldFixedNode->h;
                    //cout<<"，修改后为：("<<node->g<<", "<<node->h<<")-"<<node->f<<"\n";
                }
                //cout<<" 当前子节点：("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"已在open，无需修改\n";
                free(node);
                node = NULL;
            }
        }
        //cout<<"\n";
        
        delete childNodes;
        childNodes = NULL;
    }
    
    if (overNode != NULL)
    {
        CCArray *pathNodes = CCArray::create();
        for (Node *currNode=overNode; currNode->parent!=NULL; currNode=currNode->parent)
        {
            PathNode *pathNode = PathNode::create();
            pathNode->y = currNode->y;
            pathNode->x = currNode->x;
            pathNode->gid = currNode->gid;
            pathNodes->addObject(pathNode);
        }

        return pathNodes;
    }
    else
    {
        cout<<"没有合适的路径\n";
        return NULL;
    }
}

/**
 * get the index of min f-value from the openTable
 */
Node *AlgorithmAStar::getMinFNodeFromOpenTableAndPushIntoCloesdTable(vector<Node *> &openTable, vector<Node *> &closedTable)
{
    int currMinF = INT_MAX;
    int y, x  = -1;
    Node *minFNode = NULL;
    
    for (vector<Node *>::iterator it=openTable.begin(); it!=openTable.end(); it++)
    {
        if ((*it)->f < currMinF)
        {
            currMinF = (*it)->f;
            y = (*it)->y;
            x = (*it)->x;
        }
    }
    
    for (vector<Node *>::iterator i=openTable.begin(); i!=openTable.end(); )
    {
        if ((*i)->y==y && (*i)->x==x)
        {
            minFNode = (*i);
            closedTable.push_back((*i));
            i = openTable.erase(i);
        }
        else
            i++;
    }

    //cout<<"得到最小f的是("<< minFNode->y <<", "<<minFNode->x<<")";
    
    return minFNode;
}

int AlgorithmAStar::getHWithManhattanDistance(int y, int x, CCPoint end)
{
    return (int)fabs(end.x-x)*10 + (int)fabs(end.y-y)*10;
}

/**
 * just set x, y position, g-value and the gid, uesd to judge 
 * whether this node is in closeTable or is a invalid node
 */
vector<Node *> *AlgorithmAStar::getChildNodesFromCurrNode(Node *currNode)
{
    vector<Node *> *childNodes = new vector<Node *>();
    int y = currNode->y;
    int x = currNode->x;
    
    //左
    if (x-1 >= 0)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x - 1;
        node->y = y;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
        /*
        //左上
        if (y-1 >= 0)
        {
            Node *node = (Node *)malloc(sizeof(Node));
            node->x = x - 1;
            node->y = y - 1;
            node->gid = m_pMap[node->y*m_nCol + node->x];
            childNodes->push_back(node);
        }
        //左下
        if (y+1 < m_nRow)
        {
            Node *node = (Node *)malloc(sizeof(Node));
            node->x = x - 1;
            node->y = y + 1;
            node->gid = m_pMap[node->y*m_nCol + node->x];
            childNodes->push_back(node);
        }
         */
    }
    //右
    if (x+1 < m_nRow)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x + 1;
        node->y = y;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
        /*
        //右上
        if (y-1 >= 0)
        {
            Node *node = (Node *)malloc(sizeof(Node));
            node->x = x + 1;
            node->y = y - 1;
            node->gid = m_pMap[node->y*m_nCol + node->x];
            childNodes->push_back(node);
        }
        //右下
        if (y+1 < m_nRow)
        {
            Node *node = (Node *)malloc(sizeof(Node));
            node->x = x + 1;
            node->y = y + 1;
            node->gid = m_pMap[node->y*m_nCol + node->x];
            childNodes->push_back(node);
        }
         */
    }
    //上
    if (y-1 >= 0)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x;
        node->y = y - 1;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
    }
    //下
    if (y+1 < m_nRow)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x;
        node->y = y + 1;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
    }
    
    return childNodes;
}

bool AlgorithmAStar::isInTable(vector<Node *> &table, int y, int x)
{
    for (vector<Node *>::iterator i=table.begin(); i!=table.end(); ++i) {
        if ( (*i)->y == y && (*i)->x == x ) {
            return true;
        }
    }
    return false;
}

Node *AlgorithmAStar::getShouldFixedNode(vector<Node *> &openTable, int y, int x)
{
    for (vector<Node *>::iterator it=openTable.begin(); it!=openTable.end(); ++it) {
        if ( (*it)->y == y && (*it)->x == x ) {
            return (*it);
        }
    }
    return NULL;
}