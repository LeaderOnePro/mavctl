# mavctl

[English](README.md) | 简体中文

> 无图形界面（headless）、Agent 优先的 MAVLink 地面站 CLI，面向 ArduPilot 载具。

mavctl 以 **ArduPilot 为优先目标**，同时为终端前的人类和 AI 编码 agent
（Claude Code、Codex、OpenClaw 等）而设计。一个常驻 daemon 负责维持 MAVLink
链路并持续缓存载具状态；每条 CLI 命令都是向它发起的一次简短、结构化的请求。

**当前状态：** 在 ArduPilot SITL 上开发并完成验证。尚未在真实 MAVLink 载具的
广度上得到检验，**不**宣称已可用于真实飞行器上的生产飞行。

## 为什么是 mavctl

Mission Planner、QGroundControl 这类 GUI 地面站对亲自操控的人类非常出色——
但对 shell 脚本或 LLM agent 却很糟糕：可点击的 UI、没有稳定的退出码、没有
机器可读的输出。

mavctl 站在天平的另一端：

- daemon 独占 MAVLink 连接并持续缓存遥测，因此每条命令都快速且无状态；
- 所有命令默认输出人类可读格式，加 `--json` 则输出结构化 JSON；
- 失败携带明确的退出码（3 daemon 未运行、4 链路丢失、5 护栏拒绝、
  6 飞控 NACK/超时），而不是堆栈跟踪；
- 危险操作在触达载具之前必须先通过安全护栏；
- mavctl 不内嵌任何 LLM——它被设计为*被* Claude Code、Codex、OpenClaw
  这类 agent 调用，或被普通 bash 调用。

## 当前能力

已实现的命令——这就是完整列表：

```bash
mavctl daemon start|stop|status
mavctl status
mavctl telemetry
mavctl arm
mavctl disarm
mavctl mode <MODE>
mavctl takeoff --alt <米>
mavctl land
mavctl rtl
```

横切行为：

| 标志 / 行为 | 含义 |
| ---------------- | ------- |
| `--json` | stdout 输出结构化 JSON；错误以 `{"error": {...}}` 输出到 stderr |
| `--confirm` | 所有改变载具状态的命令必须携带；缺失则退出码 5 |
| `--dry-run` | 执行完全相同的护栏检查，绝不触碰载具 |
| `--wait --timeout <秒>` | 阻塞直到目标状态达成（默认 60 秒） |
| 幂等重复 | 重复执行已达成的变更返回成功（"already armed"） |
| 事务安全 | ACK/NACK 处理、命令串行化、失链中止 |

未实现——仅描述当前范围，不是路线图承诺：

```text
任务（mission）上传/下载/启动
参数（param）读写
地理围栏（geofence）
日志下载与分析
固件刷写
多载具协同
```

## ArduPilot SITL 快速上手

需要 Python >= 3.10 与 [uv](https://docs.astral.sh/uv/)。永远先启动 SITL；
不要把 agent 驱动的工作流指向真实载具。

终端 1 —— 启动 ArduPilot SITL：

```bash
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550
```

终端 2 —— 从源码安装并连接：

```bash
uv sync
uv run mavctl daemon start --connect udp:127.0.0.1:14550
uv run mavctl status --json
```

安全起飞至 10 米并返回起飞点：

```bash
uv run mavctl mode GUIDED --confirm --wait
uv run mavctl arm --confirm
# 轮询 status --json 直到 armed=true（arm 的 ACK 可能先于心跳状态到达）
uv run mavctl takeoff --alt 10 --confirm --wait --timeout 45
uv run mavctl rtl --confirm --wait --timeout 120
uv run mavctl daemon stop
```

安全须知——在把 mavctl 指向任何会飞的东西之前，先读这一节：

- 先在 SITL 中验证每一个工作流；把真机使用当作一次独立的评审流程。
- `arm` 之后、起飞之前，轮询 `status --json` 直到 `armed=true`：
  COMMAND_ACK 可能比上报状态提前约一个心跳到达。
- 用 `rtl` / `land` 结束飞行，而不是 `disarm`。普通 `disarm` 需要可证明的
  接地证据（否则 `ground_state_unknown`）。
- `disarm --force` 仅用于紧急停转——在空中使用可能导致坠机。
- mavctl 的任何一层都不存在 `arm --force`；pre-arm 检查不可绕过。

## 从 PyPI 安装

mavctl 0.2.0 已发布到正式 PyPI。安装方式：

```bash
uv tool install mavctl
# 或者不持久安装、跑一次就走：
uvx mavctl --help
# 或者：
pipx install mavctl
```

在把 mavctl 指向任何会飞的东西之前，请先过一遍上面的 SITL 优先快速上手。
发布历史与发布操作手册见 [docs/PUBLISHING.md](docs/PUBLISHING.md)。

## 从源码安装

从源码开发请使用 `uv sync` 与 `uv run mavctl …`：

```bash
git clone https://github.com/LeaderOnePro/mavctl.git
cd mavctl
uv sync
uv run mavctl --help
```

## Agent Skill

仓库自带一个可移植的 agent Skill，位于 `skills/mavctl-flight/`（入口文件加
workflows / safety / troubleshooting 参考文档）。它是本仓库的源资产，不属于
安装包的一部分。

要在某个 agent 运行时中使用它，请按照该运行时当前的 Skill 发现约定安装或
软链此目录。

以 Claude Code 为例（项目本地、作用于本仓库）：

```bash
mkdir -p .claude/skills
ln -s ../../skills/mavctl-flight .claude/skills/mavctl-flight
```

这只是一个具体示例，不是通用约定——各运行时并不相同。

## 安全模型

简版；完整细节见
[skills/mavctl-flight/references/safety.md](skills/mavctl-flight/references/safety.md)：

- daemon 独占载具链路；CLI 调用都是短事务；
- 改变状态的命令必须 `--confirm`；`--dry-run` 预览护栏决策；
- 退出码 4 = 没有实时的载具状态（链路丢失 / 心跳过期），包括 `--wait`
  中途失链（`link_lost_during_wait`）；
- 退出码 5 = 护栏拒绝，带结构化 `reason` + `hint`；
- 退出码 6 = 飞控 NACK / ACK 超时 / wait 超时；
- force-arm 在任何一层都不存在（CLI 选项、RPC 字段、adapter 动词）；
- 普通 `disarm` 需要正向地面证据，否则 `ground_state_unknown`（退出码 5）；
- 失链之后 `status` 反映的是过期缓存：`armed` 渲染为 unknown/n/a，
  绝不静默显示为 disarmed。

## 开发

```bash
uv sync
uv run ruff check .
uv run mypy .
uv run pytest -m "not sitl"
uv run pytest -m sitl   # 需要正在运行的 ArduPilot SITL
```

延伸阅读：

- [docs/SITL_ACCEPTANCE.md](docs/SITL_ACCEPTANCE.md)
- [docs/SITL_ACCEPTANCE_PHASE2.md](docs/SITL_ACCEPTANCE_PHASE2.md)
- [AGENTS.md](AGENTS.md) —— 架构规则与贡献约束
- [skills/mavctl-flight/SKILL.md](skills/mavctl-flight/SKILL.md) —— 面向 agent 的飞行操作指南

## 许可证

[MIT](LICENSE) —— Copyright (c) 2026 LeaderOnePro。
