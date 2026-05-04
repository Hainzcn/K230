"""K230 视觉循迹子系统的视觉模块包。

按 docs/vision_line_tracking_plan_v2.md §11.1 的目录结构演进。

- ``camera``        阶段 A：双通道 sensor + 显示绑定 + OSD（含阶段 B 检测可视化）
- ``photometric``   阶段 B：line_threshold bootstrap + 运行期漂移监测
- ``line_detector`` 阶段 B：L0 二值化 + L1 形态学 + L2 多扫描带质心
- ``quality``       阶段 B：Q_L2 评分（阶段 C/E 起补 geom / r_prior 子项）

后续阶段将逐步加入：

- ``ground_mapper``：IPM 查找表（阶段 C）
- ``geometry``：RANSAC 圆 / 切线 / 一阶回归（阶段 C）
- ``estimator``：EMA / 一维 Kalman（阶段 C/E）
- ``controller``：前馈 + 反馈 + 限幅 + 斜率限制（阶段 E）
- ``debug_overlay``：调试可视化的独立模块（如果 camera.render_overlay 体量过大再拆）
"""

__all__ = ["camera", "photometric", "line_detector", "quality"]
