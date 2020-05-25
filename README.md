# hirop
hirop 2.0

在编译的过程中遇到 fatal error: Eigen/Geometry: ，解决方法如下：
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
