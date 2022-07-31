//
// Created by lihao on 19-7-9.
//

#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <queue>
#include <unordered_map>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "OccMapTransform.h"

using namespace std;
using namespace cv;


namespace pathplanning{

enum NodeType{
    obstacle = 0,
    free,
    inOpenList,
    inCloseList
};

struct Node{/////////////////////////////////////////
    Point point;  // node coordinate
    int F, G, H;  // cost
    Node* parent; // parent node

    Node(Point _point = Point(0, 0)):point(_point), F(0), G(0), H(0), parent(NULL)
    {
    }
};

struct cmp
{
    bool operator() (pair<int, Point> a, pair<int, Point> b) // Comparison function for priority queue
    {
        return a.first > b.first; // min heap
    }
};

struct AstarConfig{
    bool Euclidean;         // true/false
    int OccupyThresh;       // 0~255
    int InflateRadius;      // integer

    AstarConfig(bool _Euclidean = true, int _OccupyThresh = -1, int _InflateRadius = -1):
        Euclidean(_Euclidean), OccupyThresh(_OccupyThresh), InflateRadius(_InflateRadius)
    {
    }
};

class Astar{

public:
    // Interface function
    void InitAstar(Mat& _Map, AstarConfig _config = AstarConfig());
    void InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config = AstarConfig());
    void PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path, int semi_step);
    void Exploration(Point _startPoint, Point _targetPoint, Point _FAILp, vector<Point>& path, int d, vector<Point>& Tlist_def, int nof_layers, int semi_step, bool MULTIPATH_flag, int NofTarget, int q, vector<Point>& Tlist_to_pub, int last_q);
    void DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask = noArray(), Scalar color = Scalar(0, 0, 255),
            int thickness = 1, Scalar maskcolor = Scalar(255, 255, 255));

    inline int point2index(Point point) {
        return point.y * Map.cols + point.x;
    }
    inline Point index2point(int index) {
        return Point(int(index / Map.cols), index % Map.cols);
    }

private:
    void MapProcess(Mat& Mask);
    Node* FindPath(int semi_step);
    Node* FindPath_wt(int d, Point _FAILp);
    void GetPath(Node* TailNode, vector<Point>& path);
    void Target_list(Node* TailNode, vector<Point>& path);


private:
    //Object
    Mat Map;
    Point startPoint, targetPoint;
    Mat neighbor;

    Mat LabelMap;
    AstarConfig config;

    priority_queue<pair<int, Point>, vector<pair<int, Point>>, cmp> OpenList; // open list
    unordered_map<int, Node*> OpenDict; // open dict
    vector<Node*> PathList;  // path list
};

}




#endif //ASTAR_H