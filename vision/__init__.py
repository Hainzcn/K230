"""K230 视觉循迹子系统的视觉模块包。

按 docs/vision_line_tracking_plan_v2.md §11.1 的目录结构演进。

- ``camera``        阶段 A：双通道 sensor + 显示绑定 + OSD 入口
- ``debug_overlay`` 阶段 B：检测几何与二值调试 OSD 可视化
                    阶段 C：``PathOverlayInfo`` + IPM 路径几何叠加
- ``photometric``   阶段 B：line_threshold bootstrap + 运行期漂移监测
- ``line_detector`` 阶段 B：L0 二值化 + L1 形态学 + L2 多扫描带质心
- ``quality``       阶段 B：Q_L2 评分；阶段 C：补 ``compute_q_full``
- ``ground_mapper`` 阶段 C：IPM 单应映射 (5 点 H 矩阵展开 + plan §4.1 占位 H)
- ``geometry``      阶段 C：RANSAC 圆 / 切线 / 一阶回归 + e_y/ψ_e 公式
- ``estimator``     阶段 C：EMA + 符号防抖；阶段 E 之后会接入 1-D Kalman

后续阶段将逐步加入：

- ``controller``：前馈 + 反馈 + 限幅 + 斜率限制（阶段 E）
"""

__all__ = [
    "camera", "debug_overlay",
    "photometric", "line_detector", "quality",
    "ground_mapper", "geometry", "estimator",
]
