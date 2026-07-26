# mavctl

Headless ground control station CLI for MAVLink vehicles (ArduPilot-first),
designed to be driven by AI coding agents (Claude Code, Codex, etc.) as well as humans.

## 架构（不可违背的核心决策）

三层架构，依赖方向严格单向：CLI → Daemon → Adapter

1. **CLI 层**（`mavctl/cli/`）：Typer 实现。薄壳，只做参数解析、
   调用 daemon、格式化输出。不含任何业务逻辑。
2. **Daemon 层**（`mavctl/daemon/`）：常驻进程，维持 MAVLink 连接、
   缓存遥测状态、执行安全护栏检查。通过 Unix domain socket
   （JSON-RPC 风格）与 CLI 通信。
3. **Adapter 层**（`mavctl/adapter/`）：pymavlink 的唯一使用场所。
   pymavlink 的 import 不得出现在其他任何层。

## 技术栈

- Python >= 3.10，包管理用 uv，构建配置 pyproject.toml
- CLI: typer | 数据模型: pydantic v2 | MAVLink: pymavlink
- 测试: pytest + pytest-asyncio | Lint: ruff | 类型: mypy（strict）
- Daemon 内部用 asyncio；pymavlink 的阻塞调用包在 executor 里

## 铁律

- **所有命令支持 `--json`**：输出结构化 JSON 到 stdout；人类可读格式为默认
- **退出码语义**：0=成功 1=通用错误 2=参数错误 3=daemon 未运行
  4=飞控未连接 5=安全护栏拒绝 6=飞控 NACK/超时
- **安全护栏**：arm / takeoff / mode / rtl 等改变载具状态的命令
  必须要求 `--confirm` 标志，否则拒绝执行（退出码 5）并说明原因
- **危险命令支持 `--dry-run`**
- 错误信息输出到 stderr，JSON 模式下为 `{"error": {...}}`
- 每个公共函数写类型注解；不写无意义的注释

## 测试要求

- Adapter 层：用 mock MAVLink 连接做单元测试
- 集成测试：标记 `@pytest.mark.sitl`，需要本地 ArduPilot SITL
  （默认 udp:127.0.0.1:14550），CI 中可跳过
- 每次修改后运行：`ruff check . && mypy . && pytest -m "not sitl"`

## 常用命令

- 安装依赖：`uv sync`
- 启动 SITL（若已安装）：`sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550`
