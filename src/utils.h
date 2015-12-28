#ifndef _utils_h
#define _utils_h

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

template<int rows, int cols>
inline Eigen::Matrix<double, rows, cols> Tensor2Mat(THDoubleTensor *tensor)
{
  THArgCheck(tensor != NULL && tensor->nDimension == 2 && tensor->size[0] == rows && tensor->size[1] == cols, 1, "invalid tensor");
  tensor = THDoubleTensor_newContiguous(tensor);
  Eigen::Matrix<double, rows, cols> output(Eigen::Map<Eigen::Matrix<double, rows, cols, Eigen::RowMajor> >(THDoubleTensor_data(tensor)));
  THDoubleTensor_free(tensor);
  return output;
}

template<int rows, int cols, int options> void viewMatrix(Eigen::Matrix<double, rows, cols, options> &m, THDoubleTensor *output)
{
  // create new storage that views into the matrix
  THDoubleStorage* storage = NULL;
  if ((Eigen::Matrix<double, rows, cols, options>::Options & Eigen::RowMajor) == Eigen::RowMajor)
    storage = THDoubleStorage_newWithData(m.data(), (m.rows() * m.rowStride()));
  else
    storage = THDoubleStorage_newWithData(m.data(), (m.cols() * m.colStride()));
    
  storage->flag = TH_STORAGE_REFCOUNTED;
  THDoubleTensor_setStorage2d(output, storage, 0, rows, m.rowStride(), cols, m.colStride());
  THDoubleStorage_free(storage);   // tensor took ownership
}

template<int rows, int cols> void copyMatrix(const Eigen::Matrix<double, rows, cols> &m, THDoubleTensor *output)
{
  THDoubleTensor_resize2d(output, m.rows(), m.cols());
  THDoubleTensor* output_ = THDoubleTensor_newContiguous(output);
  // there are strange static-asserts in Eigen to disallow specifying RowMajor for vectors...
  Eigen::Map<Eigen::Matrix<double, rows, cols, (rows == 1 || cols == 1) ? Eigen::ColMajor : Eigen::RowMajor> >(THDoubleTensor_data(output)) = m;
  THDoubleTensor_freeCopyTo(output_, output);
}


#endif //_utils_h
