# 更新记录

## v1.0.0 (2026-11)

- 首次整理发布：把毕业设计中的 14 个 `.m` 文件按 `src/`（核心函数）与 `scripts/`
  （仿真脚本）分层重组，补充 `startup.m` 路径配置。
- 新增 Python 参考实现 `tools/qsmc_sim.py`，与 MATLAB 版本交叉验证。
- 新增复算工具 `tools/export_results.py`、绘图工具 `tools/make_figures.py`、
  `tools/fig_flow.py`，全部插图与指标可一键再生。
- 整理 `data/raw`（2 个 .mat）与 `data/processed`（4 个 CSV + 指标），补齐数据字典。
- 记录并修复三处实现缺陷：`K(t)` 判据互斥、总推力映射符号、双环接口耦合。
- 新增专利体例技术文档 `docs/patent-draft.md` 与算法说明 `docs/algorithm.md`。
