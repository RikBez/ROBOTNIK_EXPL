//
// Created by lihao on 19-7-9.
//
#include <ros/ros.h>
#include "Astar.h"

#define pass (void) 0

namespace pathplanning{

void Astar::InitAstar(Mat& _Map, AstarConfig _config)
{
    Mat Mask;
    InitAstar(_Map, Mask, _config);
}

void Astar::InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config)
{
    Map = _Map;
    config = _config;

    MapProcess(Mask);
}

void Astar::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path, int semi_step)
{
    // Get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    Node* TailNode = FindPath(semi_step);
    GetPath(TailNode, path);
}

void Astar::Exploration(Point _startPoint, Point _targetPoint, Point _FAILp, vector<Point>& path, int d, vector<Point>& Tlist_def, int nof_layers, int semi_step, bool MULTIPATH_flag, int NofTarget, int q, vector<Point>& Tlist_to_pub, int last_q)
{
    // Get variables
    startPoint = _startPoint;
    targetPoint = _startPoint; 
    vector<Point> Tlist;
    vector<Point> Tlist_temp;
    Node* TailNode;
    int size;
    int actual_size;
    int j = 0;
    int sec;
    int l_size;
    int thresh = 15;
    // int last_q = 0;
    Point element;
    Point parag;
    Point Empty = Point(0, 0);

    if (MULTIPATH_flag == false)
    {
        //FIRST////////////////////////////////////////////////////////////////////
        d = 0;//    __/
        TailNode = FindPath_wt(d, _FAILp);
        if (TailNode -> point != _FAILp)
        {
            Tlist.push_back(TailNode -> point);
        }

        d = 3;//    __
        TailNode = FindPath_wt(d, _FAILp);
        if (TailNode -> point != _FAILp)
        {
            Tlist.push_back(TailNode -> point);
        }
        
        d = 5;//    \__
        TailNode = FindPath_wt(d, _FAILp);
        if (TailNode -> point != _FAILp)
        {
            Tlist.push_back(TailNode -> point);
        }

        size = Tlist.size();
        if (size == 0)//se il nodo non ha successori///////////
        {
            Tlist_temp.push_back(startPoint);
            targetPoint = Tlist_temp.at(0);
            //ROS_INFO("solo origine");
            //TailNode = FindPath(semi_step);
            //GetPath(TailNode, path);//////
        }
        else
        {
            for(int k = 0; k < nof_layers; k++)
            {
                l_size = Tlist.size();
                //LAYER////////////////////////////////////////////////////////////////////
                for (int i = 0; i < l_size; i++)
                {
                    if (i == 0) //estremo sinistro///////////////////////
                    {
                        startPoint = Tlist.at(0);
                        Tlist.erase(Tlist.begin()); //rimuovo l'elemento ormai salvato in startPoint
                        size = Tlist.size();

                        d = 8;  //  /
                        TailNode = FindPath_wt(d, _FAILp);
                        if (TailNode -> point != _FAILp)
                        {
                            Tlist.push_back(TailNode -> point);
                        }

                        d = 11; //  _
                        TailNode = FindPath_wt(d, _FAILp);
                        if (TailNode -> point != _FAILp)
                        {
                            Tlist.push_back(TailNode -> point);
                        }

                        if (size == 0) 
                        {
                            d = 13; /*  \   */
                            TailNode = FindPath_wt(d, _FAILp);
                            if (TailNode -> point != _FAILp)
                            {
                                Tlist.push_back(TailNode -> point);
                            }
                        }

                        //se il nodo parente attuale non ha figli validi,
                        //quindi se Tlist.size è diminuito di uno per l'erase fatto sopra
                        //e non gli è stato aggiunto nulla, 
                        if ((Tlist.size() == (size)) || (k == nof_layers - 1))
                        {
                            Tlist_temp.push_back(startPoint);
                        }
                    }
                    else if(i == (l_size - 1) && i != 0 ) //estremo destro/////////////////////////              
                    {
                        startPoint = Tlist.at(0);
                        Tlist.erase(Tlist.begin());
                        size = Tlist.size();

                        d = 11; //  _
                        TailNode = FindPath_wt(d, _FAILp);
                        if (TailNode -> point != _FAILp)
                        {
                            Tlist.push_back(TailNode -> point);
                        }   

                        d = 13; /*  \   */
                        TailNode = FindPath_wt(d, _FAILp);
                        if (TailNode -> point != _FAILp)
                        {
                            Tlist.push_back(TailNode -> point);
                        }

                        //se il nodo parente attuale non ha figli validi,
                        //quindi se Tlist.size è diminuito di uno per l'erase fatto sopra
                        //e non gli è stato aggiunto nulla, 
                        if ((Tlist.size() == (size)) || (k == nof_layers - 1))
                        {
                            Tlist_temp.push_back(startPoint);
                        }
                    }
                    else                //elementi in mezzo//////////////////////
                    {
                        startPoint = Tlist.at(0);
                        Tlist.erase(Tlist.begin()); 
                        size = Tlist.size();

                        d = 11; //  _
                        TailNode = FindPath_wt(d, _FAILp);
                        if (TailNode -> point != _FAILp)
                        {
                            Tlist.push_back(TailNode -> point);
                        }

                        //se il nodo parente attuale non ha figli validi,
                        //quindi se Tlist.size è diminuito di uno per l'erase fatto sopra
                        //e non gli è stato aggiunto nulla, 
                        if ((Tlist.size() == (size)) || (k == nof_layers - 1))
                        {
                            Tlist_temp.push_back(startPoint);
                        }
                    }
                }
            }
        }

        //Tlist_def = Tlist_temp;
        //Tlist_def = Tlist_temp;
        actual_size = Tlist_temp.size();
        //int conto = 0;
        //ROS_INFO("layer turn %d", actual_size);
        
        for (int clus = 0; clus < actual_size; clus ++)
        {
            element = Tlist_temp.at(0);
            Tlist_temp.erase(Tlist_temp.begin());
            Tlist_temp.push_back(element); //per non

            element = Tlist_temp.at(0);
            Tlist_temp.erase(Tlist_temp.begin());
            Tlist_to_pub.push_back(element);

            for (int sec = 0; sec < (actual_size - 1); sec ++)
            {
                parag = Tlist_temp.at(0);
                if ((element.y - thresh) < parag.y && parag.y < (element.y + thresh))
                {
                    if ((element.x - thresh) < parag.x && parag.x < (element.x + thresh))
                    {
                        Tlist_temp.erase(Tlist_temp.begin());
                    }
                    else
                    {
                        //sec = sec;
                        //conto = conto + 1;
                        Tlist_temp.erase(Tlist_temp.begin());
                        Tlist_temp.push_back(parag);
                    }
                }
                else
                {
                    //conto = conto + 1;
                    Tlist_temp.erase(Tlist_temp.begin());
                    Tlist_temp.push_back(parag);
                }
            }
            actual_size = Tlist_temp.size();
        }

        // targetPoint = Tlist_def.at(0);
        // Tlist_to_pub.push_back(targetPoint);
        // Tlist_def.erase(Tlist_def.begin());
        Tlist_def = Tlist_to_pub;
        targetPoint = Tlist_to_pub.at(0);

        actual_size = Tlist_to_pub.size();
        //int conto = 0;
        //ROS_INFO("clustered %d", actual_size);

        startPoint = _startPoint;/////////////////////////////////////////////////////////
        TailNode = FindPath(semi_step);

        //ROS_INFO("start %d %d", startPoint.x, startPoint.y);
        //ROS_INFO("target %d %d", targetPoint.x, targetPoint.y);

        GetPath(TailNode, path);
    }
    else
    {
        //qui trovo il path definitivo
        
        actual_size = Tlist_to_pub.size();
        //int conto = 0;
        //ROS_INFO("tlist_to_pub size  %d", actual_size);
        if (actual_size == 0)
        {
            Tlist_to_pub.push_back(Empty);
        }
        else
        {
            actual_size = actual_size;
        }

        //ROS_INFO("multipath %d", q);
        targetPoint = Tlist_to_pub.at(0);
        Tlist_to_pub.erase(Tlist_to_pub.begin());
        startPoint = _startPoint;/////////////////////////////////////////////////////////
        //Tlist_to_pub.push_back(targetPoint);
        //ROS_INFO("%d %d in exploring", targetPoint.x, targetPoint.y);
        TailNode = FindPath(semi_step);

        //ROS_INFO("st%d %d", startPoint.x, startPoint.y);
        //ROS_INFO("target %d %d", targetPoint.x, targetPoint.y);
        
        GetPath(TailNode, path);   
    }
}

void Astar::DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask, Scalar color,
        int thickness, Scalar maskcolor)
{
    if(path.empty())
    {
        cout << "Path is empty!" << endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    for(auto it:path)
    {
        rectangle(_Map, it, it, color, thickness);
    }
}

void Astar::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();

    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, cv::COLOR_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, cv::THRESH_OTSU);
    } 
    else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, cv::THRESH_BINARY);
    }

    // Inflate
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
}

Node* Astar::FindPath(int semi_step)
{
    //int semi_step = 2;
    int width = Map.cols;
    int height = Map.rows;
    //int d = 0;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        //qui nel caso il target sia acquisito, neigbour diventa completo ed esploro davvero
        //{
        char neighbor8[3][2] = {{-semi_step, semi_step}, {0, semi_step}, {semi_step, semi_step}};
        neighbor = Mat(3, 2, CV_8S, neighbor8).clone();
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                //Calculate G, H, F value
                int addG, G, H, F;////////////////////////////
                //int addG, G, F;
                if(dist1 == 2)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                if(config.Euclidean)
                {
                   int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                   H = round(10 * sqrt(dist2));
                }
                else
                {
                   H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                }
                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }
    return NULL; // Can not find a valid path
}

Node* Astar::FindPath_wt(int d, Point _FAILp)
{
    int step = 2;
    int width = Map.cols;
    int height = Map.rows;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    Node* startPointNode = new Node(startPoint);
    OpenList.push(pair<int, Point>(startPointNode->F, startPointNode->point));
    int index = point2index(startPointNode->point);
    OpenDict[index] = startPointNode;
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        OpenDict.erase(index);

        int curX = CurPoint.x;
        int curY = CurPoint.y;
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

            if (d == 0)///////////////////////////////////////////////////////////////
            {
                char neighbor8[1][2] = {{-step, 3*step}};//destra_alto
                neighbor = Mat(1, 2, CV_8S, neighbor8).clone();
            }

            // Determine whether arrive the target point////
            else if (d == 1)
            {
                //d = d + 1;
                targetPoint.x = curX;
                targetPoint.y = curY;
                return CurNode; //ritorno il nodo finale
            }

            else if (d == 3)///////////////////////////////////////////////////////////////
            {
                char neighbor8[1][2] = {{0, 3*step}};//destra
                neighbor = Mat(1, 2, CV_8S, neighbor8).clone();
            }

            else if (d == 4)
            {
                //d = d + 1;
                targetPoint.x = curX;
                targetPoint.y = curY;
                return CurNode; //ritorno il nodo finale
            }

            else if (d == 5)///////////////////////////////////////////////////////////////
            {
                char neighbor8[1][2] = {{step, 3*step}};//destra__basso
                neighbor = Mat(1, 2, CV_8S, neighbor8).clone();
                //ROS_INFO("check");
            }

            else if (d == 6)
            {
                //d = d + 1;
                targetPoint.x = curX;
                targetPoint.y = curY;
                return CurNode; //ritorno il nodo finale
            }

            else if (d == 8)///////////////////////////////////////////////////////////////
            {
                char neighbor8[1][2] = {{-step, 2*step}};//destra-alto
                neighbor = Mat(1, 2, CV_8S, neighbor8).clone();
            }

            else if (d == 9)
            {
                //d = d + 1;
                targetPoint.x = curX;
                targetPoint.y = curY;
                return CurNode; //ritorno il nodo finale
            }

            else if (d == 11)///////////////////////////////////////////////////////////////
            {
                char neighbor8[1][2] = {{0, 2*step}};//destra
                neighbor = Mat(1, 2, CV_8S, neighbor8).clone();
            }

            else if (d == 12)
            {
                //d = d + 1;
                targetPoint.x = curX;
                targetPoint.y = curY;
                return CurNode; //ritorno il nodo finale
            }

            else if (d == 13)///////////////////////////////////////////////////////////////
            {
                char neighbor8[1][2] = {{step, 2*step}};//destra-basso
                neighbor = Mat(1, 2, CV_8S, neighbor8).clone();
            }

            else if (d == 14)
            {
                d = d + 1;
                targetPoint.x = curX;
                targetPoint.y = curY;
                return CurNode; //ritorno il nodo finale
            }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            //ROS_INFO("check");
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                
                int dist1 = abs(neighbor.at<char>(k, 0)) + abs(neighbor.at<char>(k, 1));
                if(dist1 == 2 && _LabelMap.at<uchar>(y, curX) == obstacle && _LabelMap.at<uchar>(curY, x) == obstacle)
                    continue;

                // Calculate G, H, F value
                //int addG, G, H, F;////////////////////////////
                int H = 0;
                int addG, G, F;
                if(dist1 == 2)
                {
                    addG = 14;
                    d = d + 1;
                }
                else
                {
                    addG = 10;
                    d = d + 1;
                }
                G = CurNode->G + addG;

                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push(pair<int, Point>(node->F, node->point));
                    int index = point2index(node->point);
                    OpenDict[index] = node;
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    int index = point2index(Point(x, y));
                    Node* node = OpenDict[index];
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }
    Node* FAIL = new Node();
    FAIL -> point = _FAILp;
    return FAIL; 
    //return NULL;
}

void Astar::GetPath(Node* TailNode, vector<Point>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
    }

    // Release memory
    while(OpenList.size()) {
        Point CurPoint = OpenList.top().second;
        OpenList.pop();
        int index = point2index(CurPoint);
        Node* CurNode = OpenDict[index];
        delete CurNode;
    }
    OpenDict.clear();
}

}