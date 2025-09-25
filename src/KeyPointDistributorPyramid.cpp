#include "myORB-SLAM2/KeyPointDistributorPyramid.h"

namespace my_ORB_SLAM2 {

    void KeyPointDistributorPyramid::distribute(
        const vector<int> &nFeaturesPerLevel,
        vector<vector<KeyPoint>> &resultKeyPointsPerLevel,
        vector<vector<KeyPoint>> &keyPointsPerLevel, 
        const vector<Mat> &imagesPerLevel
    ) {
        // 分配空間
        resultKeyPointsPerLevel.resize(imagesPerLevel.size());

        // 遍歷影像金字塔
        for(int level = 0; level < imagesPerLevel.size(); level++) {
            // 設定關鍵點區域四叉樹
            int width, height;
            width = imagesPerLevel[level].cols;
            height = imagesPerLevel[level].rows;
            vector<KeyPoint> &vKeyPoints = keyPointsPerLevel[level];
            KeyPointsRegionalQuadTree quadTree(width, height, vKeyPoints);

            while(true) {
                list<KeyPointsRegionalQuadTreeNode>::iterator lit = 
                quadTree.lNodes.begin();

                int nDistribution = 0;
                while(lit != quadTree.lNodes.end()) {
                    if(lit->vpKeyPoints.size() > 1) {
                        nDistribution++;
                    }
                    lit++;
                }
                if(nDistribution == 0) { break; }

                int nNextNodes = quadTree.lNodes.size() + nDistribution * 3;
                if(nNextNodes < nFeaturesPerLevel[level]) {
                    lit = quadTree.lNodes.begin();
                    while(lit != quadTree.lNodes.end()) {
                        lit = quadTree.divide(lit);
                    }
                }else {
                    quadTree.lNodes.sort([](const KeyPointsRegionalQuadTreeNode &n1, const KeyPointsRegionalQuadTreeNode &n2) { return n1.vpKeyPoints.size() > n2.vpKeyPoints.size();});
                    lit = quadTree.lNodes.begin();
                    while(lit != quadTree.lNodes.end()) {
                        lit = quadTree.divide(lit);
                    }
                    break;
                }
            }

            // 預先分配記憶體空間
            resultKeyPointsPerLevel[level].reserve(quadTree.lNodes.size());
            for(KeyPointsRegionalQuadTreeNode &node : quadTree.lNodes) {
                float maxResponse = -INFINITY;
                KeyPoint* pKeyPoint = nullptr;
                vector<KeyPoint*> &keyPoints = node.vpKeyPoints;
                for(vector<KeyPoint*>::iterator vit = keyPoints.begin(); vit != keyPoints.end(); ++vit) {
                    if((*vit)->response > maxResponse) {
                        maxResponse = (*vit)->response;
                        pKeyPoint = (*vit);
                    }
                }
                resultKeyPointsPerLevel[level].push_back(*pKeyPoint);
            }
        }
    };

}