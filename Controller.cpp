#include "Controller.h"

// LQR   MPC
// iLQR  nMPC
namespace Controller
{

  // 连续lqr
  MatrixXf K(1, 4); // 优化增益矩阵
  float x, dx, q, dq;
  Vector4f X = {x, q, dx, dq};
  Matrix4f A;
  Matrix<float, 4, 1> B;
  float F = 0;
  Matrix4f Q; // 运行状态权重矩阵
  float R;    // 输入权重
  Matrix4f S; // 末端代价权重矩阵
  Matrix4f P; // 离散LQR求解迭代

  // 离散lqr
  Matrix4f Ad; // 离散系统矩阵
  Vector4f Bd;
  float Ts = 0.001;  // 控制周期
  MatrixXf Kd(1, 4); // 离散LQR反馈增益矩阵

  // 非零参考值
  MatrixXf Aa(8, 8);
  MatrixXf Xa(8, 1);
  VectorXf Ba(8);
  MatrixXf Ca(4, 8);
  MatrixXf X_desire(4, 1);
  MatrixXf Qa(8, 8);
  MatrixXf Pa(8, 8);
  MatrixXf Ka(1, 8); // 离散LQR反馈增益矩阵

  void Initialize_System()
  {
    A << 0, 0, 1, 0,
        0, 0, 0, 1,
        0, (m * g) / M, 0, 0,
        0, (g * (M + m)) / (l * M), 0, 0;
    B << 0, 0, 1 / M, 1 / (l * M);
    Q << 100, 0, 0, 0,
        0, 10, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    R = 1;
    S = Q;
    P = S;
  }

  void UpdateX(mjModel *m, mjData *d)
  {
    q = ((d->qpos[m->jnt_qposadr[1]]) * 57.29578); // 获取杆角度  转化为角度制
    // 获取关节速度
    dq = (d->qvel[m->jnt_qposadr[1]] * 57.29578);
    mjtNum vel[6];
    mj_objectVelocity(m, d, mjOBJ_BODY, 1, vel, 0);
    dx = vel[3]; // 物体在X轴方向的速度
    x = d->xpos[3];
    X << x, q, dx, dq;
  }
  void APPLY_FORCE(mjModel *m, mjData *d)
  {
    Vector4f X_desired;
    X_desired << 0, 0, 0, 0;
    F = ((K * (X - X_desired))[0]);
    d->ctrl[0] = F;
  }

  void APPLY_FORCE2(mjModel *m, mjData *d)
  {
    Vector4f X_desired;
    X_desired << 0, 0.3, 0, 0;
    F = ((K * (X - X_desired))[0]);
    d->ctrl[0] = F;
  }
  namespace LQR
  {

    void LQR_Controller_Init()
    {

      // 连续 lqr  解算出来的  里卡提方程
      // 连续方程
      K << 10., 3.223817, 5.25095341, 0.14434914;
    }

    // Discrete formula solution
    void LQR_D_Controller_Init()
    {
      Ad = Matrix4f::Identity() + A * Ts;
      Bd = B * Ts;
      K << 9.94253, 3.22607, 5.24073, 0.153588;
    }

    // 用于参数计算
    void LQR_D_Controller_Update()
    {
      Kd = (1 / (Bd.transpose() * P * Bd + R)) * (Bd.transpose() * P * Ad);
      P = ((Ad - Bd * Kd).transpose()) * P * (Ad - Bd * Kd) + Q + Kd.transpose() * R * Kd;
      // 经过迭代发现
      //    Kd 收敛于  9.94253  3.22607  5.24073 0.153588  //接近  连续时间  LQR
    }

    void LQR_D_NOZERO_()
    {
      Aa << Ad, (Matrix4f::Identity() - Ad),
          Matrix4f::Zero(), Matrix4f::Identity();

      Ba << Bd, 0, 0, 0, 0;
      Ca << Matrix4f::Identity(), -Matrix4f::Identity();
      Qa << Ca.transpose() * Q * Ca;
      Pa = Qa;

      X_Desire << 0, 10, 0, 0;
    }

    void LQR_D_NOZERO_Update()
    {
      Ka = (1 / (Ba.transpose() * Pa * Ba + R)) * (Ba.transpose() * Pa * Aa);
      Pa = ((Aa - Ba * Ka).transpose()) * Pa * (Aa - Ba * Ka) + Qa + Ka.transpose() * R * Ka;
      // 经过迭代发现
      //    Kd 收敛于   9.94246   3.22615    5.2407  0.153593   -9.9426  -3.22619   -5.2406 -0.153615
    }; // namespace LQR

    namespace MPC
    {

    };
  };