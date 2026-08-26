# MinerU MCP

这是一个面向当前研究 Vault 的最小 MCP stdio 服务，封装本机
`/home/dream/Applications/MinerU/.venv/bin/mineru`。

## 启动

```bash
python3 /home/dream/ros2_ws/tools/mineru_mcp/server.py
```

也可以通过 `MINERU_BIN` 指定 MinerU 可执行文件：

```bash
MINERU_BIN=/path/to/mineru \
  python3 /home/dream/ros2_ws/tools/mineru_mcp/server.py
```

## 工具

`parse_pdf(input_path, output_path, backend="pipeline", start_page=None, end_page=None)`

- `input_path`：本地 PDF。
- `output_path`：解析结果目录。
- `backend`：默认 `pipeline`，适合当前 CPU/GPU 混合环境。
- `start_page` / `end_page`：可选的 0-based 页码范围。

服务只负责文档解析，不自动修改 Vault；阅读笔记和论文引用仍由上层
工作流审核后写入。
