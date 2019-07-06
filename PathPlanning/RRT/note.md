## 简介
　　Rapidly-explorating Random Trees Path Planning(RRT)是一种基于采样的路径规划方法，这种方法源自对最优搜索方法的改进。当机器人的规划空间过大或动作维度过大时，搜索问题就会变得非常难解，而在相当多的情况下我们可以只关注“有解”问题，不必要关注“最优问题”，因此基于采样的方法成为一个高效的解决方案。需要指出的是，RRT方法只是最基础的采样方法的Baseline，如同Dijkstra方法在遍搜法中的地位相似，后续有非常多在其上的改进扩展。

## 方法
  RRT方法的核心就是两个问题：撒点、扩展树。其算法步骤如下所示：  
 - １ 初始化树T
 - ２ for i in <搜索次数限制>:
 - ３ qrand = Sample()
 - ４ q_nearest = Nearest(T, qrand)
 - ５ if ||q_nearest, goal|| \< \THRESHOLD:
 - ６ 　　return True
 - ７ q_new = Extend(q\_nearest, qrand, delta)
 - ８ if q_new != Null:
 - ９　　T.addNode(q_new)  
　　
　　在上述步骤中，关键的问题在于第三步“撒点”和第七步“扩展”。其中撒点并不是随机的全局撒，与A*相似，我们更希望朝着目标点的方向多撒一些，但同时也要考虑有些情况下并不是朝着目标点延伸就可以到达，还需要考虑避障的问题。因此经典的撒点方法与强化学习中的epsilon-greedy方法有异曲同工之处：
 - Sample():
 - p = random(0,1)
 - if Prob > p > 0:
 - 　　return q_goal
 - else:
 - 　　return RandomNode()

　　同时扩展过程中也有需要注意的地方，扩展即朝着指引点方向伸出delta长度，但这个delta是有说法的，如果delta过小，则搜索速度很慢，效率过低；而如果delta过大，则有可能导致伸展出的点越过障碍物，最终规划出的路径不够安全。

## 改进
　　前面提到RRT方法只是一个Baseline，可以改进的地方很多，而大多的改进工作都是在这几个方向入手的：
 - 引导点选择，与普通的RRT不同，connected RRT采用从起点和终点同时建树的方法将搜索时间大大缩短。
 - Nearest　Point的选择，传统的Nearest_point是找引导点欧式意义下的最近点，但是在高维度规划下考虑机器人的运动约束，有时候欧式意义下的最近并不是真实的最近。例如下图，角度上只差一点，但阿克曼底盘不能原地转向，事实上后方的位姿才是最近点。  
 [img](./Pictures/rrt_nearest_point.png)  
 - 路径平滑问题，RRT获得的路径通常不是最优路径，而且由于随机采样，路径非常坎坷，需要通过利用直线联接可以直接到达的点等方法，使得路径平滑。

## 实验
　　小delta时：
 [img](./Pictures/rrt_01.gif)  
　　合适的delta时：
 [img](./Pictures/rrt_02.gif)  
　　平滑路径后：
 [img](./Pictures/rrt_03.gif)  
