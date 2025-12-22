import numpy as np
from scipy.linalg import block_diag

# ==========================================
# 1. 基础参数与离散模型 (沿用之前的数值)
# ==========================================
g = 9.8; M = 1.0; m = 0.1; l = 0.5
Ts = 0.001
N = 5  # 预测步数

# 连续矩阵
A = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, m*g/M, 0, 0],
    [0, g*(M+m)/(l*M), 0, 0]
])
B = np.array([[0], [0], [1/M], [1/(l*M)]])

# 离散化 (Euler)
Ad = np.eye(4) + A * Ts
Bd = B * Ts

# 权重矩阵
Q = np.diag([100, 10, 1, 1])
R = np.array([[1]])

# ==========================================
# 2. 构建预测矩阵 M 和 C
#    X = M * x0 + C * U
# ==========================================
n = Ad.shape[0] # 状态维度 4
p = Bd.shape[1] # 输入维度 1

# 初始化 M (phi) 和 C (gamma)
# M 的大小: (N*n, n) -> (20, 4)
# C 的大小: (N*n, N*p) -> (20, 5)
M_big = np.zeros((N * n, n))
C_big = np.zeros((N * n, N * p))

# 填充 M 和 C
# 这里的逻辑是迭代：x(k+1) = A*x(k) + B*u(k)
temp_A = np.eye(n)
for i in range(N):
    # 计算 A^(i+1)
    temp_A = temp_A @ Ad
    M_big[i*n : (i+1)*n, :] = temp_A
    
    # 计算 C 的这一行
    for j in range(i + 1):
        # 对应 A^(i-j) * B
        pow_A = np.linalg.matrix_power(Ad, i - j)
        C_big[i*n : (i+1)*n, j*p : (j+1)*p] = pow_A @ Bd

# ==========================================
# 3. 构建大权重矩阵 Q_bar, R_bar
# ==========================================
# Q_bar: 对角线上重复 N 次 Q
Q_bar = block_diag(*([Q] * N))

# R_bar: 对角线上重复 N 次 R
R_bar = block_diag(*([R] * N))

# ==========================================
# 4. 计算 QP 核心矩阵 H 和 F
#    J = U' * H * U + 2 * x0' * F' * U
# ==========================================

# H = C' * Q_bar * C + R_bar
H = C_big.T @ Q_bar @ C_big + R_bar

# F_matrix = C' * Q_bar * M
# 注意：最终 QP 求解器的线性项向量 f = F_matrix @ x0 (或者带个系数2，取决于求解器定义)
F_matrix = C_big.T @ Q_bar @ M_big

print("=== H Matrix (Shape: {}) ===".format(H.shape))
# 为了显示整洁，保留4位小数
np.set_printoptions(precision=4, suppress=True, linewidth=200)
print(H)

print("\n=== F Matrix (Shape: {}) ===".format(F_matrix.shape))
print("注意：这是用于与 x0 相乘的矩阵，不是最终向量")
print(F_matrix)

# ==========================================
# 5. 验证一下 (假设 x0 不为0)
# ==========================================
x0 = np.array([0, 0.1, 0, 0]) # 假设初始角度偏了
f_vector = F_matrix @ x0      # 线性项向量 q 或 f
# print("\n假设初始状态 x0 =", x0)
# print("QP 线性向量 f = F * x0 =", f_vector)