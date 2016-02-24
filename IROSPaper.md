# Introduction
Mobile robotic system that can detect surrounding objects and do action based on this information is crucial in many application. Recently, \citeauthor{7139259} [@7139259] introduced a framework for outdoor person tracking and following by extending a trained legs detector from a previous work [@4209616]. The purpose of developing such a system is to improve autonomy and navigation safety of a smart wheel chair robot. ... (to be continued)

# System Description


# Person Following



## Leg Identification


### Clustering
Consecutive jumps to starts cluster: range with (i-1)>0.06m, with (i-2)>0.08m, with (i+2)>0.005m, with (i-1)>0.06m.
Consecutive jumps to ends cluster: range with (i+1)>0.06m, with (i+2)>0.08m.
Obstruction filtering: Ensure that there are no other objects that are both nearer and exactly next to the points where the cluster starts and ends.
Cluster position is limited to be within 5 meters.
Cluster size is limited to be less than the size of two legs next to each other (0.4m), larger than the size of a single leg (0.07cm) and consist more than 4 points. 

### Leg Segmentation

#### Dual leg filtering
Dual leg filtering is done to clusters that exceeded the maximum size for a single leg (0.18m). 

First, the local minimum points of the cluster with respect to the cluster edges are searched. To search the minimum points, the distance between each point in the cluster with a virtual line that starts and ends at the cluster edges is calculated. The local minimum point of the cluster is the point where it has shorter distance to the virtual line compared to several of its neighboring left and right points. 

A cluster is concluded to be a dual leg if there is a local minimum that crosses the virtual line and the size of the cluster is larger than a maximum size of one leg. Otherwise, the cluster will be discarded from the leg segmentation process.

As for the clusters that are concluded to be a dual leg clusters, they are dissected at the minimum point into two parts. The part with more sampling points is now considered as a new single leg cluster candidate and will be processed further. The other part with less sampling point is discarded from the leg segmentation process.

	 	 	
#### Non-arc shape fast filtering
To be an arc or convex shape, the middle point of the cluster must be within a reasonable distance (typically between 0.1-0.7) from the virtual line of the cluster edges. The single leg cluster that passes this filter will be processed further, otherwise it will be discarded. This filtering quickly discards any non convex shape cluster with low computation cost.

#### Modeling and Evaluation of clusters
To analyze the single leg clusters, three criterions are used as benchmark. First, the mean error of the inscribe angle variant (IAV) of the cluster. The second is the error of the standard deviation of the cluster. The third is related to the error in the size of the cluster.

The benchmark criterion is taken from the measurement of dummy humanoid with typical cylindrical legs. The data collected using the model is used as the reference and compared with the actual single leg clusters.
Then, based on the errors between the desired benchmark and the actual measurement, an equation to formulate the confidence is derived as follows.







## Leg and Person Tracking
In this paper, the pursuit controller is using a driving controller based on the selection of discrete spatial zones with the lowest cost. To determine the robot heading, the zones surrounding the robot are divided into multiple "buckets", and the costs of each bucket will be evaluated. The costs of the buckets are determined by several factors, such as how far is each bucket to the target, and how clear is the bucket from obstacles. 
In this paper, we propose a dynamic obstacle detection area, where the robot could avoid any obstacles on his path, while maintaining homing behavior to the target it chases. In order to limit the obstacle detection area, the depth of each bucket is limited by a constant. However, whenever the distance to the target is smaller compared to the constant, the observation range will shrink to be slightly less compared to the distance to the target. And whenever the target moves away from the robot, the observation range will expand up to the limit. In this way, the people being followed will not be regarded as an obstacle.
In our case, Hokuyo UTM-30-LX is used as the sensor. In our ISERA robot, the perception zone of the sensor is limited to 180 degree towards the front of the robot, which gives 720 sampling points with 0.25 degree separating each point. As a case study, we arbitrarily divide the observation into 72 buckets. With each bucket starts right next to each other, we have divided the laser perception zone into mini 2.5 degree buckets. 
Next, for each bucket, the nearest distance to obstacle inside the observation range is stored to be the depth of that bucket. 
The bearing of each bucket to the target is calculated as follows.
Then, the individual cost of each bucket is calculated as follows.
Naturally, when there is no obstacle inside the obstacle detection range, the buckets in the direction of the target would have the lowest cost.
To prevent the robot from going too near into an obstacle, each bucket cost is also influenced by the cost of its neighboring several buckets. This averaging of costs could be seen as a repulsive force effect to keep the robot away from the obstacles, while homing to the target pursuit. The average cost of each buckets are calculated to be the representative cost of selecting that particular direction as follows.
Finally, by comparing the average cost of each buckets, the bucket with the lowest average cost is chosen to be the pursuit heading. 
As an additional safety measure for operating in human environment where people can suddenly come towards the robot, a short range virtual bumper from the laser reading is used to halt the robot when any obstacles come within its boundary. Once the obstacle or pedestrian have moved away, the robot will continue the target pursuit, provided that the target has not disappeared from the robot’s point of view.
The pursuit velocity and turn are calculated based on several factors. For the pursuit velocity, the factors are how far the obstacle in the pursuit heading, the angle between the target person and the front direction of the robot, the angle between the pursuit heading from the current heading. For example, to chase with the highest speed, there should be no obstacle near the pursuit heading and the target person is in front of the robot. For other cases, the velocity will be reduced proportionally. 

The turn velocity is simply based on the angle between the pursuit heading and the current heading. 
Another factor that has a significant impact on the robot behavior is the range of the person. As a service robot, the robot is expected to behave in the manner that is socially acceptable. The closer the robot to the target person, the more gentle the motion should be. The control laws combining these considerations are as follows.



## Pursuit Controller


# Experimentation and Results


# Conclusion



