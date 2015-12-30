#include "torch-moveit.h"
#include "utils.h"

MOVIMP(void, Pose, getRotation)(THDoubleTensor* m, THDoubleTensor* quaternion_out)
{
  const Eigen::Matrix3d& t = Tensor2Mat<3,3>(m);
  Eigen::Quaternion<double> q(t); 
  copyMatrix(q.coeffs(), quaternion_out);
}

MOVIMP(void, Pose, setRotation)(THDoubleTensor* m, THDoubleTensor* quaternion_in)
{
  Eigen::Quaternion<double> q;
  q.coeffs() = Tensor2Vec4d(quaternion_in);
  copyMatrix(q.matrix(), m);
}
