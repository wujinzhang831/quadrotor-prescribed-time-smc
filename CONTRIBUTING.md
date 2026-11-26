# 贡献指南

欢迎提交 issue 与 pull request。

## 开发约定

1. MATLAB 代码遵循 `src/`（可复用函数）与 `scripts/`（可执行脚本）分层，
   新增脚本请在文件头写明输入、输出与运行方式。
2. Python 工具保持零第三方依赖（仅 `numpy`、`pillow`），便于在无 MATLAB 环境复现。
3. 修改算法后请运行 `python tools/export_results.py`，确认 `data/processed/metrics.csv`
   的变化并同步更新文档中的数字。
4. 插图由 `tools/make_figures.py` 生成，请勿手工替换，以免图文不一致。

## 提交前自检

```bash
python tools/export_results.py     # 指标复算
python tools/make_figures.py       # 插图再生
```

## 报告问题

请附上：MATLAB/Python 版本、操作系统、复现脚本、期望结果与实际结果。
