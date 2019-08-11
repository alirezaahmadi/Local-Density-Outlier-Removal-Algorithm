# Local-Density-Outlier-Removal-Algorithm
a small Host/Device implementation of Local-Density  based Outlier Removal Point-Clouds or any Classifiable data-set with distinguishable elements 


Local density base: As the sample distribution of
visual sensors which provide depth information is scattered
with different densities based on occlusion, viewing angle
and the environment geometry, the aim of this section is
to remove inconsistent points from input point set based
on the geometry and density information. Almost, there are
three different types of outliers which could be found in
point clouds, the sparse outlier, isolated outliers comprising
from a limit number of points grouped together and far
from true objects region and non-isolated outliers which are
closer to the model and it’s hard to remove them [15]. If
we suppose, point set P = {p 1 , p 2 , p 3 , ..., p N }, we need
to compute the covariance matrix of each point based on
its neighborhood. Let define query Q = {q 1 , q 2 , q 3 , . . . , q k }
which shows KN N (pi) as the k Nearest Neighbors of point
p i . To compute the density information, we need to examine
each point from point cloud over its k nearest neighbor.
the local density can be estimated by calculating average
distance of p i of its k nearest neighbor q j (j = 1, ..., k) as

<div align="center">
	<img src="/images/ld1.png"  width="300"/>
</div>


where i = 1, 2, ..., k and dist(p i , q j ) is the Euclidian distance
between p i and q j . Hence the local density can be estimated
using

<div align="center">
	<img src="/images/ld2.png"  width="300"/>
</div>

the implementation of this subject is integrated in the pre-
processes of the ICP algorithm and the result is depicted in 6.
As this approach uses KNN radius search over whole points
in the dataset, is really expensive and without utilizing the
KD-tree its almost impossible to get proper runtime.

<div align="center">
	<img src="/images/ld3.png"  width="300"/>
</div>

Local density Analyses. The out put of outlier removal based on
local density analyses is depicted in picture in the left beside the original
dataset which is passed from downsampling step.