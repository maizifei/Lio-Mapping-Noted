/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 5/4/18.
//

/// adapted from VINS-mono

#ifndef LIO_MARGINALIZATIONFACTOR_H_
#define LIO_MARGINALIZATIONFACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Eigen>
#include <unordered_map>
#include <pthread.h>

#include "utils/CircularBuffer.h"
#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"

namespace lio {

const int NUM_THREADS = 4;

struct ResidualBlockInfo {
  ResidualBlockInfo(ceres::CostFunction *_cost_function,
                    ceres::LossFunction *_loss_function,
                    std::vector<double *> _parameter_blocks,
                    std::vector<int> _drop_set)
      : cost_function(_cost_function),
        loss_function(_loss_function),
        parameter_blocks(_parameter_blocks),
        drop_set(_drop_set) {}

  void Evaluate();

  ceres::CostFunction *cost_function;
  ceres::LossFunction *loss_function;
  std::vector<double *> parameter_blocks;  //优化变量数据
  std::vector<int> drop_set;  //待marg的优化变量id

  double **raw_jacobians;
  std::vector<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobians;
  Eigen::VectorXd residuals;  //残差,IMU:15×1,视觉:2×1

  int localSize(int size) {
    return size == 7 ? 6 : size;
  }
};

struct ThreadsStruct {
  std::vector<ResidualBlockInfo *> sub_factors;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  std::unordered_map<long, int> parameter_block_size; //global size
  std::unordered_map<long, int> parameter_block_idx; //local size
};

class MarginalizationInfo {
 public:
  ~MarginalizationInfo();
  int LocalSize(int size) const;
  int GlobalSize(int size) const;
  void AddResidualBlockInfo(ResidualBlockInfo *residual_block_info);  //加残差块相关信息(优化变量、待marg的变量)
  void PreMarginalize();  //计算每个残差对应的Jacobian,并更新parameter_block_data
  void Marginalize();  //pos为所有变量维度,m为需要marg掉的变量,n为需要保留的变量
  std::vector<double *> GetParameterBlocks(std::unordered_map<long, double *> &addr_shift);

  std::vector<ResidualBlockInfo *> factors;  //所有观测项
  //m为要marg掉的变量个数,也就是parameter_block_idx的总localSize,以double为单位,VBias为9,PQ为6,
  //n为要保留下的优化变量的变量个数,n=localSize(parameter_block_size) – m
  int m, n;
  std::unordered_map<long, int> parameter_block_size; //global size <优化变量内存地址,localSize>
  int sum_block_size;
  std::unordered_map<long, int> parameter_block_idx; //local size <待marg的优化变量内存地址,在parameter_block_size中的id,以double为单位>
  std::unordered_map<long, double *> parameter_block_data;  //<优化变量内存地址,数据>

  std::vector<int> keep_block_size; //global size
  std::vector<int> keep_block_idx;  //local size
  std::vector<double *> keep_block_data;

  Eigen::MatrixXd linearized_jacobians;
  Eigen::VectorXd linearized_residuals;
  const double eps = 1e-8;
};

class MarginalizationFactor : public ceres::CostFunction {
 public:
  MarginalizationFactor(MarginalizationInfo* _marginalization_info);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

  MarginalizationInfo* marginalization_info;
};

}

#endif //LIO_MARGINALIZATIONFACTOR_H_
