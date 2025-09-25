#ifndef REGIONALQUADTREE_H
#define REGIONALQUADTREE_H

#include <list>
#include <vector>
#include <opencv2/features2d.hpp>

using namespace cv;
using namespace std;

namespace my_ORB_SLAM2 {

// 關鍵點區域四叉樹節點類
class KeyPointsRegionalQuadTreeNode {
    public:
        /*
        @brief 設定關鍵點區域四叉樹節點 */
        KeyPointsRegionalQuadTreeNode() {};
        ~KeyPointsRegionalQuadTreeNode() {};
        
        vector<KeyPoint*> vpKeyPoints; // 該節點內包含的關鍵點指標
        int xmin, ymin, xmax, ymax, xmid, ymid; // 該節點在影像中的框框座標
        bool locked = false; // 用於判斷該節點是否能夠進行任何更動
};

class KeyPointsRegionalQuadTree {
    public:
        /*
        @brief 設定關鍵點區域四叉樹 
        
        @param[in] width 影像寬度 
        @param[in] height 影像高度 
        @param[in] vKeyPoints 關鍵點 */
        KeyPointsRegionalQuadTree(int width, int height, vector<KeyPoint> &vKeyPoints);
        ~KeyPointsRegionalQuadTree() {};

        list<KeyPointsRegionalQuadTreeNode> lNodes; // 該四叉樹的節點串列
        vector<KeyPoint> &vKeyPoints; // 四叉樹的所有 Key Points

        /*
        @brief 對指定的節點進行分裂 
            
        @param[in] node 指定要分裂的節點的迭代器*/
        list<KeyPointsRegionalQuadTreeNode>::iterator divide(list<KeyPointsRegionalQuadTreeNode>::iterator &node);
};

}

#endif