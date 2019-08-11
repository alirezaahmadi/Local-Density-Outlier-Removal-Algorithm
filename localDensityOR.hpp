/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Local-Density-Outlier-Removal-Algorithm                   %
% by: Alireza Ahmadi                                        %
% University of Bonn- MSc Robotics & Geodetic Engineering   %
% Alireza.Ahmadi@uni-bonn.de                                %
% AlirezaAhmadi.xyz                                         %
/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
The MIT License

Copyright (c) 2010-2019 Google, Inc. 

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <stdlib.h>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <time.h>   

#include <algorithm>
#include <numeric>
#include <eigen3/Eigen/Dense>

#include <pcl_ros/point_cloud.h> //pcl
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace Eigen;

#define LD_Search_Type_NN 0         //  0: without FLANNKD-tree (window search in image), 1: with KD-tree 
#define minDensity 0.2              // minimum local density considered for a single point
#define knnLD 10                    // number of neighbours to consider for estimateing Density of each point
#define windowsHeightSize 100       // in pixels (size of search windows)

namespace MapRecon{

    double get_distance(PointXYZRGB& P_new, PointXYZRGB& P_old) {
        double result = pow((P_new.x - P_old.x),2) + pow((P_new.y - P_old.y),2) + pow((P_new.z - P_old.z), 2);
        return sqrt(abs(result));
    }

    void LocalDensityFilter(PointCloud<PointXYZRGB>::Ptr& src,PointCloud<PointXYZRGB>::Ptr& dsr){
        switch(LD_Search_Type_NN){
            case 0:{
                double tempDistance = 100.0;
                vector<double> Distances;
                vector<double> minDistances;
                minDistances.resize(knnLD);
                int cnt =0;
                for (int l = 0; l < knnLD; ++l) minDistances[l] = 100;

                pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;

                for (int i=0; i<src->points.size(); i++){
                    int tmp_loop_min = i - windowsHeightSize;
                    if(tmp_loop_min < 0) tmp_loop_min = 0; 
                    int tmp_loop_max = i + windowsHeightSize;
                    if(tmp_loop_max > src->points.size()) tmp_loop_max = src->points.size(); 
                    for (int x=tmp_loop_min; x<tmp_loop_max; x++){
                        if(x != i){
                            Distances.push_back(get_distance(src->points[i], src->points[x]));
                            cnt++;
                            if(knnLD <= cnt){
                                cnt = 0;
                                break;
                            }
                        }
                    }
                    std::sort(Distances.begin(), Distances.end());

                    double d_i = std::accumulate(Distances.begin(), Distances.end(), 0.0)/knnLD;
                    double target_LD = 0.0;
                    for (int l = 0; l < knnLD; ++l)
                    {
                        target_LD +=exp(-Distances[l]/d_i);
                    }
                    target_LD /= K; 
                    if(minDensity < target_LD){
                        inliers->indices.push_back(i);
                    }
                }
                extract.setInputCloud(src);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*dsr);
                break;
            }case 1:{
                pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                pcl::ExtractIndices<pcl::PointXYZRGB> extract;
                pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
                kdtree.setInputCloud (src);
                for(size_t i = 0; i < src->points.size(); i++){
                    pcl::PointXYZRGB searchPoint;
                    searchPoint.x = src->points[i].x;
                    searchPoint.y = src->points[i].y;
                    searchPoint.z = src->points[i].z;
                    std::vector<int> pointIdxNKNSearch(knnLD);
                    std::vector<float> pointNKNSquaredDistance(knnLD);
                    int K = kdtree.radiusSearch(searchPoint, this->MaxDist_LD, pointIdxNKNSearch, pointNKNSquaredDistance);
                    if ( K > 0 )
                    {
                        double d_i = std::accumulate(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end(), 0.0)/knnLD;
                        double target_LD = 0.0;
                        for (int l = 0; l < K; ++l)
                            target_LD +=exp(-pointNKNSquaredDistance[l]/d_i);
                        target_LD /= K; 
                        if(minDensity < target_LD){
                            inliers->indices.push_back(i);
                        }
                    }
                }
                extract.setInputCloud(src);
                extract.setIndices(inliers);
                extract.setNegative(false);
                extract.filter(*dsr);
                break;
            }
        }
    }
}


