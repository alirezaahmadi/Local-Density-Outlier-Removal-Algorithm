# Local-Density-Outlier-Removal-Algorithm

a small Host implementation of Local-Density  based Outlier Removal Point-Clouds or any Classifiable data-set with distinguishable elements 

## Problem statement

As scanned point clouds are usually noisy, sparse and temporarily incoherent, thus, outlier removal algorithm can be really beneficial to improve the accuracy of final results of registration and mapping, specially when the down-sampling is utilized to regrade the density of the point-cloud which causes to have more sparsity in the distribution of the points. Whereas, most of the matching algorithms are so fragile to dealing with outliers. 

So its better to get rid of them before feeding new scans into the main pipeline of the algorithm...

 ---

### Local density base

As the sample distribution of visual commodity sensors (like Kinect and Xtion) which provide depth information is scattered with different densities based on occlusion, viewing angle and the environment geometry, the aim of this section is to remove inconsistent points from input point-set based on the geometry features and density information. 

Almost, there are three different types of outliers which could be found in point clouds:
* the sparse outlier, 
* isolated outliers comprising from a limit number of points grouped together and far from true objects region
* non-isolated outliers which are closer to the model and it’s hard to remove them. 

If we assume to have, point set P = {p 1 , p 2 , p 3 , ..., p N }, we need to compute the covariance matrix of each point based on its neighborhood as a step in registration pipe-line, here, the value of a clean and suitable input would be more reveling than having noisy input because any outlier will cause a divergence to the true position of center of mass. 

Let define query Q = {q 1 , q 2 , q 3 , . . . , q k } which shows KNN(pi) as the k Nearest Neighbors set of point i-th p  in the main data-set . To compute the density information, we need to examine each point from point cloud over its k nearest neighbor. the local density can be estimated by calculating average distance of pi of its k nearest neighbor qj (j = 1, ..., k) as

<div align="center">
	<img src="/images/ld1.png"  width="200"/>
</div>

where i = 1, 2, ..., k and dist(p i , q j ) is the Euclidean distance between p i and q j . Hence the local density can be estimated using

<div align="center">
	<img src="/images/ld2.png"  width="300"/>
</div>

As this approach uses KNN radius search over whole points in the data-set, is really expensive and without utilizing the KD-tree its almost impossible to get proper runtime.

<div align="center">
	<img src="/images/ld3.png"  width="700"/>
</div>

Local density Analyses. The out put of outlier removal based on local density analyses is depicted in picture in the left beside the original data-set which is passed from down-sampling step.

--- 
 by: Alireza Ahmadi                                     
 University of Bonn- Robotics & Geodetic Engineering

 Alireza.Ahmadi@uni-bonn.de                             
 [www.AliezaAhmadi.xyz](https://www.AlirezaAhmadi.xyz)
