#ifndef __Controller_H
#define __Controller_H

#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mujoco.h>
#include <eigen3/Eigen/Eigen>
using namespace Eigen;
namespace Controller
{

  // 系统参数  公共变量
  const double g = -9.81; // 重力加速度
  const double M = 0.5;   // 小车质量
  const double m = 0.5;   // 小球质量
  const double l = 0.3;   // 连杆长度
  // const double M = 1.0;  // 小车质量
  // const double m = 0.5;  // 小球质量
  // const double l = 0.5;  // 连杆长度
  extern float x, dx, q, dq;
  extern Vector4f X;
  extern Matrix4f A;
  extern Matrix<float, 4, 1> B;
  extern float F;
  extern Matrix4f Q;
  extern float R;
  extern Matrix4f S;
  extern MatrixXf K; // 优化增益矩阵
  void Initialize_System();
  void UpdateX(mjModel *m, mjData *d);
  void APPLY_FORCE(mjModel *m, mjData *d);
  void APPLY_FORCE2(mjModel *m, mjData *d);
  extern Matrix4f Ad; // 离散系统矩阵
  extern Vector4f Bd;
  extern float Ts; // 控制周期
  extern MatrixXf Kd;
  extern MatrixXf Ka; // 离散LQR反馈增益矩阵
  extern Vector4f X_desire;
  extern MatrixXf Xa;
  extern float aa;
  extern float fd;
  namespace LQR
  {

    // LQR函数声明
    void LQR_Controller_Init();
    void LQR_D_Controller_Init();
    void LQR_D_Controller_Update();
    void LQR_D_NOZERO_();
    void LQR_D_NOZERO_Update();

  }; // namespace LQR

  namespace MPC
  {

  };

  namespace iLQR
  {

  };

  namespace NMPC
  {

  };

};

#endif