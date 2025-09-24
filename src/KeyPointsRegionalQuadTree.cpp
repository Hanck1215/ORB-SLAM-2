#include <myORB-SLAM2/KeyPointsRegionalQuadTree.h>

namespace my_ORB_SLAM2 {
    /*
    @brief 設定關鍵點區域四叉樹 
    
    @param[in] width 影像寬度 
    @param[in] height 影像高度 
    @param[in] vKeyPoints 關鍵點 */
    KeyPointsRegionalQuadTree::KeyPointsRegionalQuadTree(int width, int height, vector<KeyPoint> &vKeyPoints):
    vKeyPoints(vKeyPoints) {
        // 設定四叉樹的所有關鍵點
        int nKeyPoints = this->vKeyPoints.size();
        
        // 如果至少有一個關鍵點
        if(nKeyPoints >= 1) {
            // 設定該四叉樹的第一個 Node
            lNodes.resize(1);
            KeyPointsRegionalQuadTreeNode &node = lNodes.front();

            // 設定該 Node 的 Box 座標
            node.xmin = 0;
            node.ymin = 0;
            node.xmax = width;
            node.ymax = height;
            node.xmid = int(0.5 * (float)width);
            node.ymid = int(0.5 * (float)height);

            // 設定該 Node 的容量
            node.vpKeyPoints.reserve(nKeyPoints);

            // 把所有關鍵點指標儲存到 Node
            for (KeyPoint &keyPoint : this->vKeyPoints) {
                node.vpKeyPoints.push_back(&keyPoint);
            }

            // 檢查關鍵點數是否等於 1，等於 1 的話就封鎖
            node.locked = (nKeyPoints == 1);
        }
    }

    /*
    @brief 對指定的節點進行分裂 
        
    @param[in] node 指定要分裂的節點迭代器*/
    void KeyPointsRegionalQuadTree::divide(list<KeyPointsRegionalQuadTreeNode>::iterator &node) {
        if(node->locked) { return; }
        
        int nKeyPoints = node->vpKeyPoints.size();
        KeyPointsRegionalQuadTreeNode node00, node01, node10, node11;
        node00.xmin = node->xmin; node01.xmin = node->xmid;
        node00.ymin = node->ymin; node01.ymin = node->ymin;
        node00.xmax = node->xmid; node01.xmax = node->xmax;
        node00.ymax = node->ymid; node01.ymax = node->ymid;

        node10.xmin = node->xmin; node11.xmin = node->xmid;
        node10.ymin = node->ymid; node11.ymin = node->ymid;
        node10.xmax = node->xmid; node11.xmax = node->xmax;
        node10.ymax = node->ymax; node11.ymax = node->ymax;

        node00.vpKeyPoints.reserve(nKeyPoints);
        node01.vpKeyPoints.reserve(nKeyPoints);
        node10.vpKeyPoints.reserve(nKeyPoints);
        node11.vpKeyPoints.reserve(nKeyPoints);

        for(KeyPoint* &keyPoint : node->vpKeyPoints) {
            if(keyPoint->pt.x < node->xmid && keyPoint->pt.y < node->ymid) {
                node00.vpKeyPoints.push_back(keyPoint);
            }else if(keyPoint->pt.x >= node->xmid && keyPoint->pt.y < node->ymid) {
                node01.vpKeyPoints.push_back(keyPoint);
            }else if(keyPoint->pt.x < node->xmid && keyPoint->pt.y >= node->ymid) {
                node10.vpKeyPoints.push_back(keyPoint);
            }else {
                node11.vpKeyPoints.push_back(keyPoint);
            }
        }
        
        if(node00.vpKeyPoints.size() > 0)
            lNodes.push_front(node00);
        
        if(node01.vpKeyPoints.size() > 0)
            lNodes.push_front(node01);

        if(node10.vpKeyPoints.size() > 0)
            lNodes.push_front(node10);

        if(node11.vpKeyPoints.size() > 0)
            lNodes.push_front(node11);

        node = lNodes.erase(node);
    }
}