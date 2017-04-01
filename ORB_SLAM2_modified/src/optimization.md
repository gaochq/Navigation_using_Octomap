 　　ORB-SLAM2的后段共有5个优化器，在前端的tracking、mapping和闭环检测部分都有用到，下面对5个优化器做具体的解析：   
 \---**<font size=4>GlobalBundleAdjustemnt**    
 　　全局的BA优化。在5个优化器中最简单。    
误差项：
$$
e_{i,j}=x_{i,j}-\pi _{i}(T_{iw},X_{w,j})
$$
其中
\[\pi _{i}(T_{iw},X_{w,j})=\begin{bmatrix} f_{i,u}\frac{x_{i,j}}{zi,j}+c_{i,u} \\ f_{i,v}\frac{y_{i,j}}{zi,j}+c_{i,v} \end{bmatrix}\]
\[\begin{bmatrix} x_{i,j} & y_{i,j} & z_{i,j} \end{bmatrix}=R_{iw}X_{w,j}+t_{iw}\]
其实就是一个计算重投影误差的过程。    
代价函数:    
\[C=\sum_{i,j}=\rho _{h}(e_{i,j}^{T}\Omega _{i,j}^{-1}e_{i,j})\]
使用LM迭代求解即可。    
**(1) 函数参数列表：** 见函数注释
其他的顶点、边和优化函数度比较简单，可以直接看代码注释即可。    
  \---**<font size=4>LocalBundleAdjustment**    
(1) 进行局部的BA优化 。会优化当前帧和与其在Covisibility Graph中连接的关键帧，以及和这些关键帧链接的路标点。    
(2) 其他连接这些路标点的关键帧也会被包含进优化器中，但是其所有信息是固定不变的。    
(3) 观测中的异常值会在优化的过程中或者优化结束之后倍剔除。    
误差项和globalBA类似。  
**<font size=3>具体的优化函数及优化过程：**    
**(1) 函数参数列表：** 见函数注释   
**(2) 优化器Vertex：**    
添加了lcoal frames(当前帧和与其相连的Covisibility Graph)、fixed frames和3D路标点
具体的顶点类型，看代码注释。   
**(3) 优化器Edge：**    
添加优化器的边。按照与当前路标点相关联的关键帧添加。    
具体的边的类型，见代码注释。    
**(4) 优化过程：**   
先进行5次优化，在剔除异常值之后，再进行优化。最后更新优化变量。   
  \---**<font size=4>PoseOptimization**    
对当前帧的位姿进行优化(Pose Graph optimization)。这个就比较简单了。    
误差项和globalBA类似。    
**<font size=3>具体的优化函数及优化过程：**   
**(1) 函数参数列表：** 见函数注释     
**(2) 优化器Vertex：**    
g2o::VertexSE3Expmap()　　类型：public BaseVertex<6, SE3Quat>注意其更新方式    
**(3) 优化器Edge：**    
共添加了两种一元边：单目和双目。    
单目：g2o::EdgeSE3ProjectXYZOnlyPose()　　类型：BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>    
双目：g2o::EdgeStereoSE3ProjectXYZOnlyPose()　　类型：BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>    
具体的内容可以看函数注释   
**(4) 优化过程：**  
在这里共进行4次优化，每次优化迭代10次，每次优化为了区分outlier和inlier。    
在每次优化的过程中，outline不会被包括进去，但是在最后outline可能会变成inlier。  
  \---**<font size=4>OptimizeEssentialGraph**    
　　闭环优化，校正闭环，并且消除闭环误差。其目的就是为了最后能把环闭上，然后将闭环的误差沿着图分配，消除闭环误差。该优化器一共包含了四种边。它其中的Covisibility Graph(无向图模型)是一种特殊的Covisibility Graph形式，只不过边的的权重要求很高(要在100以上)，所以其图模型会更加稀疏。    
误差项：    
\[e_{i,j}=log_{Sim(3)}(S_{ij}S_{jw}S_{iw}^{-1})\]
其中$S_{ij}$ 表示由关键帧$K_{j}$ 到关键帧$K_{i}$ 的变换矩阵。那么这个误差项就可以理解为：    
\[T_{ij}=T_{iw}T_{jw}^{-1}=T_{iw}T_{wj}\]
\[T_{ij}T_{jw}T_{iw}^{-1}=I\]
如以上两式所示，由关键帧$K_{j}$ 到关键帧$K_{i}$ 的变换，如果所有的位姿没有误差的的话，我们对上式取log之后应该为0，添加的关键帧是没有经过图优化的，所以取log之后就会得到转换的误差。同理，如果我们在T中加入尺度s，就会变成变换群Sim3，同样也可以求其误差(参见ORB-SLAM参考文献[7]，[48])。所以我们由取代价函数为：    
\[C=\sum_{i,j}{(e_{i,j}^{T}\Lambda _{i.j}e_{i,j})}\]
其中的信息矩阵 $\Lambda$ 一般会将其去做单位阵，但是还可以根据参考文献[48]中所述，做更精确的设置。       
**<font size=3>具体的优化函数及优化过程：**   
**(1) 函数参数列表：** 见函数注释   
**(2) 优化器Vertex：**   
g2o::VertexSim3Expmap()　　类型：BaseVertex<7, Sim3>    
@误差项和估计值可以参考OptimizeSim3中的变换群定点。    
@顶点估计值添加过程： 只添加了关键帧定点，具体可看代码注释。    
**(3) 优化器Edge：**    
总共有4种边：    
1） 添加闭环边。 g2o::EdgeSim3()  　　　类型：BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap>　　　
这部分主要是由校正闭环而新形成的边，在添加的过程中对边的类型做了限制，具体的添加过程可以看代码注释。要注意一下边的误差计算。
```c++
void computeError()
{
    const VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
    const VertexSim3Expmap* v2 = static_cast<const VertexSim3Expmap*>(_vertices[1]);

    Sim3 C(_measurement);
    Sim3 error_=C*v1->estimate()*v2->estimate().inverse();
    _error = error_.log();
}
```
2）添加normal edge，即根据生成树添加，边的顶点是子帧和父帧。边的类型和误差计算和上面的类似。    
3）添加现有帧和与其相关的所有闭环关键帧的边。边的类型和误差计算和上面的类似。    
4）添加Covisibility graph，即边的权重在100以上    
**(4) 优化过程：**     
进行20次的迭代优化，然后更新关键帧位姿和3D路标点。   
  \---**<font size=4>OptimizeSim3**    
　　计算当前帧和闭环帧之间的相似变换群。当相机是单目时，计算相似变换群(Sim3)；当相机是双目或者RGBD时，计算特殊欧式群(SE3)。因为只是优化两帧之间的变换，以消除闭环误差，所以不对路标点做优化。   
　　误差项：    
\[e_{1}=x_{1,i}-\pi _{1}(S_{12},X_{2,j})\]    
\[e_{2}=x_{2,j}-\pi _{2}(S_{12}^{-1},X_{1,i})\]
其中$S_{12}$ 表示由关键帧$K_{2}$ 到关键帧$K_{2}$ 的变换矩阵，由上面两个误差函数可以直接计算出在反投影过程中重投影误差$e_{1}$ 和$e_{2}$ ，然后带入下面的代价函数中，求取最优的变换群。     
　　代价函数：   
\[C=\sum_{n}{(\rho_{h}(e_{1}^{T}\Omega _{1}^{-1}e_{1})+(e_{2}^{T}\Omega _{2}^{-1}e_{2}))}\]
其中 $\rho_{h}$ 表示鲁棒核函数， $\Omega_{i}^{-1}$ 表示信息矩阵(即协方差矩阵的逆)，此矩阵和尺度信息相关。    
**<font size=3>具体的优化函数及优化过程：**    
**(1) 函数参数列表：** 见函数注释    
**(2) 优化器Vertex：**    
1)变换群顶点:　g2o::VertexSim3Expmap()　　类型：BaseVertex<7, Sim3>    
@@参数：相机内参和尺度固定标志    
```c++
Vector2d  _principle_point1, _principle_point2;
Vector2d  _focal_length1, _focal_length2;
bool _fix_scale;
```    
@@更新方式：Sim3直接相乘
```c++
Eigen::Map<Vector7d> update(const_cast<double*>(update_));
if (_fix_scale)
  update[6] = 0;

Sim3 s(update);
setEstimate(s*estimate());
```        
@@顶点添加过程： 直接将函数形参的Sim3添加即可。   
2)路标点顶点：g2o::VertexSBAPointXYZ()　　　类型：BaseVertex<3, Vector3d>    
@@更新方式：3D点坐标相加    
```c++
Eigen::Map<const Vector3d> v(update);
_estimate += v;
```      
@@顶点添加过程：关键帧1中的路标点要经过关键帧1的位姿做坐标变换。关键帧2中的匹配点同理。     

**(3) 优化器Edge：** g2o::EdgeSim3ProjectXYZ() 　　类型：BaseBinaryEdge<2, Vector2d,  VertexSBAPointXYZ, VertexSim3Expmap>    
@@边添加过程：先添加由关键帧2到关键帧1的投影；再添加由关键帧2到关键帧1的投影。   
@@参数：只需要添加经过校准后的路标点在关键帧1(2)的图像投影点即可。    
@@边误差的计算：详见程序注释    
**(4) 优化过程：**   
1）先进行5次的迭代优化    
2）按照边的顺序，剔除outliners，留取inliners。    
3）根据inliners的个数，在进行特定次数的优化。     
4）在剔除依次outliners(这次剔除有什么作用么？)     
5）进行优化变量更新。     
<script type="text/javascript" src="http://cdn.mathjax.org/mathjax/latest/MathJax.js?config=default"></script>    
$$x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}$$     
\\(x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}\\)
