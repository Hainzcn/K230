"""K230 视觉循迹子系统的视觉模块包。

按 docs/vision_line_tracking_plan_v2.md §11.1 的目录结构演进。
阶段 A 仅包含 :mod:`vision.camera`；后续阶段将逐步加入：

- ``photometric``：阈值与 Otsu 重标定
- ``line_detector``：二值化、形态学、扫描带质心
- ``ground_mapper``：IPM 查找表
- ``geometry``：RANSAC 圆 / 切线 / 一阶回归
- ``quality``：质量评分 Q
- ``estimator``：EMA / 一维 Kalman
- ``controller``：前馈 + 反馈 + 限幅 + 斜率限制
- ``debug_overlay``：LCD/IDE 调试可视化
"""

__all__ = ["camera"]
