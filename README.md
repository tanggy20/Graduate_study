
# 🚗 基于博弈论的无信号交叉口车辆交互建模与自动驾驶控制

本项目基于论文：

> **《Game-Theoretic Modeling of Vehicle Interactions at Unsignalized Intersections and Application to Autonomous Vehicle Control》**

实现了无信号交叉口下多车交互的博弈建模与自动驾驶控制策略。主要包含两种求解方法：**暴力搜索（Brute-force Search）** 和 **蒙特卡洛树搜索（MCTS）**，用于自动驾驶车辆（ego）在考虑其他车辆行为反应下的策略决策。

---

## 📌 项目简介

该项目模拟了一个典型的**无信号交叉口**交通场景，在该场景中，车辆基于博弈论框架进行交互建模。自车（ego）需要预测其他车辆行为，并据此规划自身最优策略，以实现安全、高效地通过交叉口。

### ✅ 特性

- 基于 level-k 推理的多智能体博弈建模  
- 两种策略规划方式：
  - **暴力搜索（Brute-force）**：遍历所有可行动作序列
  - **蒙特卡洛树搜索（MCTS）**：基于模拟和评估的树形搜索策略
- 支持不同车辆行为建模（合作/非合作）
- 支持可视化和性能评估

---

## 🧠 方法说明

### 1. 暴力搜索（Brute-force）

- 穷举所有参与车辆的动作组合
- 计算每个组合下的博弈收益和车辆反应
- 选择自车在当前情境下的最优行为策略

### 2. 蒙特卡洛树搜索（MCTS）

- 基于博弈状态构建搜索树
- 使用 UCB 策略（上置信界）进行动作选择
- 不断模拟、扩展、回溯评估车辆动作
- 适合高维复杂决策问题，效率优于暴力搜索

---

## 📚 参考文献

如需引用相关方法或论文，请参考原论文：

```bibtex
@article{schwarting2019game,
  title={Game-Theoretic Modeling of Vehicle Interactions at Unsignalized Intersections and Application to Autonomous Vehicle Control},
  author={Schwarting, Wilko and Pierson, Alyssa and Alonso-Mora, Javier and Karaman, Sertac and Rus, Daniela},
  journal={arXiv preprint arXiv:1901.04121},
  year={2019}
}
```
