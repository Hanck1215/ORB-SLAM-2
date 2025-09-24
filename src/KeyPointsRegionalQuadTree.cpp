#include <myORB-SLAM2/KeyPointsRegionalQuadTree.h>

namespace my_ORB_SLAM2 {
    /*
    @brief 設定關鍵點區域四叉樹 
    
    @param[in] width 影像寬度 
    @param[in] height 影像高度 
    @param[in] vKeyPoints 關鍵點 */
    KeyPointsRegionalQuadTree::KeyPointsRegionalQuadTree(int width, int height, const vector<KeyPoint> &vKeyPoints) {
        // 設定四叉樹的所有關鍵點
        int nKeyPoints = vKeyPoints.size();
        this->vKeyPoints.reserve(nKeyPoints);
        for(const KeyPoint &keyPoint : vKeyPoints) {
            this->vKeyPoints.push_back(keyPoint);
        }
        
        // 設定該四叉樹的第一個 Node
        lNodes.resize(1);
        KeyPointsRegionalQuadTreeNode &node = lNodes.front();

        // 設定該 Node 的 Box 座標
        node.xmin = 0;
        node.ymin = 0;
        node.xmax = width;
        node.ymax = height;

        // 設定該 Node 的容量
        node.vpKeyPoints.reserve(nKeyPoints);

        // 把所有關鍵點指標儲存到 Node
        for (KeyPoint &keyPoint : this->vKeyPoints) {
            node.vpKeyPoints.push_back(&keyPoint);
        }
    }
}