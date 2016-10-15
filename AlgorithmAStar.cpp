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
    
    //�������
    Node *startNode = (Node *)malloc(sizeof(Node));
    startNode->y = (int)start.y;
    startNode->x = (int)start.x;
    startNode->gid = m_pMap[startNode->y*m_nCol + startNode->x];
    startNode->parent = NULL;

    //���������پ���������
    startNode->g = 0;
    startNode->h = getHWithManhattanDistance(startNode->y, startNode->x, end);
    startNode->f = 0;
    openTable.push_back(startNode);
    //cout<<"����("<<startNode->y<<", "<<startNode->x<<") - "<<startNode->f;
    
    Node *overNode = NULL;

    //������ֹ������1.�б�Ϊ�գ���ʱΪ�޷����� 2.�սڵ㱻��ӵ������б��У���ʱΪ�ҵ�������·��
    //ѭ����������1���ж�
    while (!openTable.empty())
    {
        //��openTable���ҵ�fֵ��С�Ľڵ㣬�ѵ�ǰ�ڵ��openTable��ɾ�������뵽closedTable�У���Ϊ��ǰ�ڵ㲢������ָ��
        Node *currNode = getMinFNodeFromOpenTableAndPushIntoCloesdTable(openTable, closedTable);
        //cout<<"��ǰ�ڵ㣺("<<currNode->y<<", "<<currNode->x<<")-"<<currNode->g<<", "<<currNode->h<<", "<<currNode->f<<"\n";
        
        //ѭ����������2���ж�
        if (currNode->y==(int)end.y && currNode->x==(int)end.x)
        {
            overNode = currNode;
            //cout<<"�ҵ����սڵ�\n";
            break;
        }
        
        //�ҳ��ýڵ�������ӽڵ�
        vector<Node *> *childNodes = getChildNodesFromCurrNode(currNode);

        //���������ڵ��������
        for (int i=0; i<childNodes->size(); i++)
        {
            Node *node = (*childNodes)[i];
            
            //���ýڵ㲻��ͨ�У��򲻲�ȡ�κβ���
            if (node->gid == 0)
            {
                //cout<<" ��ǰ�ӽڵ㣺("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"����Ч�ڵ�\n";
                //����Ч�ڵ��ָ����malloc�����������ֲ��ᱻ���뵽�б�������ھ��ͷ�
                free(node);
                node = NULL;
                continue;
            }
            
            //���ýڵ�����closedTable�У��򲻲�ȡ�κβ���
            if (isInTable(closedTable, node->y, node->x))
            {
                //cout<<" ��ǰ�ӽڵ㣺("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"����closed\n";
                //����Ч�ڵ��ָ����malloc��������Ϊ�Ѿ�������close�У����ֲ��ᱻ���뵽close����ģ������ھ��ͷ�
                free(node);
                node = NULL;
                continue;
            }
            
            //���ýڵ㲻��openTable��
            if (!isInTable(openTable, node->y, node->x))
            {
                node->parent = currNode;
                int g = (currNode->y == node->y || currNode->x == node->x) ? 10 : 14;
                node->g = currNode->g + g;
                node->h = getHWithManhattanDistance(node->y, node->x, end);
                node->f = node->g + node->h;
                openTable.push_back(node);
                //cout<<" ��ǰ�ӽڵ㣺("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"������open\n";
            }
            //���ýڵ��Ѿ���openTable�У��������޸����������޸ı��е��Ǹ������е�����ͷ�
            else
            {
                int g = (currNode->y == node->y || currNode->x == node->x) ? 10 : 14;
                if ( currNode->g + g < node->g) {
                    //ȡ��������Ǹ��ڵ��ָ��
                    Node *shouldFixedNode = getShouldFixedNode(openTable, node->y, node->x);
                    //cout<<" ��ǰ�ӽڵ㣺("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"����open����Ҫ�޸�";
                    shouldFixedNode->parent = currNode;
                    shouldFixedNode->g = currNode->g + g;
                    shouldFixedNode->f = shouldFixedNode->g + shouldFixedNode->h;
                    //cout<<"���޸ĺ�Ϊ��("<<node->g<<", "<<node->h<<")-"<<node->f<<"\n";
                }
                //cout<<" ��ǰ�ӽڵ㣺("<<node->y<<", "<<node->x<<")-"<<node->g<<", "<<node->h<<","<<node->f<<"����open�������޸�\n";
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
        cout<<"û�к��ʵ�·��\n";
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

    //cout<<"�õ���Сf����("<< minFNode->y <<", "<<minFNode->x<<")";
    
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
    
    //��
    if (x-1 >= 0)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x - 1;
        node->y = y;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
        /*
        //����
        if (y-1 >= 0)
        {
            Node *node = (Node *)malloc(sizeof(Node));
            node->x = x - 1;
            node->y = y - 1;
            node->gid = m_pMap[node->y*m_nCol + node->x];
            childNodes->push_back(node);
        }
        //����
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
    //��
    if (x+1 < m_nRow)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x + 1;
        node->y = y;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
        /*
        //����
        if (y-1 >= 0)
        {
            Node *node = (Node *)malloc(sizeof(Node));
            node->x = x + 1;
            node->y = y - 1;
            node->gid = m_pMap[node->y*m_nCol + node->x];
            childNodes->push_back(node);
        }
        //����
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
    //��
    if (y-1 >= 0)
    {
        Node *node = (Node *)malloc(sizeof(Node));
        node->x = x;
        node->y = y - 1;
        node->gid = m_pMap[node->y*m_nCol + node->x];
        childNodes->push_back(node);
    }
    //��
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