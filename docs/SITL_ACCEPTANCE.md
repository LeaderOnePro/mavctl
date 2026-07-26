# SITL 手动验收指南（Phase 1）

本文档说明如何在本地 ArduPilot SITL 上手动验收 mavctl Phase 1 的最小可用链路：
`daemon start/stop/status`、`status`、`telemetry`，以及退出码与 `--json` 语义。

## 0. 前置条件

- 已安装 [uv](https://docs.astral.sh/uv/)
- 已安装 ArduPilot SITL（`sim_vehicle.py` 在 PATH 中）
- 在仓库根目录执行过一次依赖安装：

```bash
uv sync
```

## 1. 启动 SITL

新开一个终端，启动 ArduCopter SITL 并把 MAVLink 输出到本机 14550 端口：

```bash
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550
```

等待控制台出现 `Ready to FLY` / GPS 定位完成（几十秒）。保持该终端运行。

## 2. 启动 mavctl daemon

回到 mavctl 仓库终端：

```bash
uv run mavctl daemon start --connect udp:127.0.0.1:14550
# 期望：daemon started (pid <PID>), connecting to udp:127.0.0.1:14550
echo "exit=$?"          # 期望 0
```

确认 daemon 正在运行：

```bash
uv run mavctl daemon status
# 期望：daemon running (pid <PID>)
echo "exit=$?"          # 期望 0
```

> daemon 的运行时状态位于 `~/.mavctl/`：`daemon.sock`（Unix socket）、
> `daemon.pid`（pidfile）、`daemon.log`（日志）。

## 3. 查看飞控状态

```bash
uv run mavctl status
```

期望人类可读输出（数值随 SITL 而定）：

```
connection : CONNECTED (udp:127.0.0.1:14550)
heartbeat  : 0.05s ago  sys=1 comp=1
mode       : GUIDED
armed      : disarmed
battery    : 12.60 V  0.00 A  100 %
gps        : rtk_fixed  sats=10
```

验收要点：
- `connection` 为 `CONNECTED`
- `mode` 显示当前飞行模式（如 `STABILIZE` / `GUIDED`）
- `armed` 显示 `disarmed` 或 `ARMED`
- `battery` 有电压读数
- `gps` 显示 fix 类型与卫星数

JSON 形式：

```bash
uv run mavctl status --json
echo "exit=$?"          # 期望 0
```

## 4. 查看遥测快照

```bash
uv run mavctl telemetry --json
echo "exit=$?"          # 期望 0（已连接时）
```

期望包含 `position`（经纬度、海拔、相对高度）、`attitude`（roll/pitch/yaw，单位度）、
`velocity`（地速、航向、垂直速度）三组数据。

## 5. 停止 daemon

```bash
uv run mavctl daemon stop
# 期望：daemon stopped
echo "exit=$?"          # 期望 0
```

## 6. 退出码语义验收

在 daemon **未运行** 时验证退出码契约：

```bash
uv run mavctl daemon stop      # 已停止后再次执行
uv run mavctl status ; echo "exit=$?"          # 期望 exit=3（daemon 未运行）
uv run mavctl daemon status ; echo "exit=$?"   # 期望 exit=3
```

参数错误：

```bash
uv run mavctl daemon start ; echo "exit=$?"    # 缺少 --connect，期望 exit=2
```

飞控未连接（连一个没有飞控的端口，等待 3 秒心跳超时后）：

```bash
uv run mavctl daemon start --connect udp:127.0.0.1:14999
sleep 4
uv run mavctl telemetry ; echo "exit=$?"       # 期望 exit=4（飞控未连接）
uv run mavctl daemon stop
```

完整退出码表：

| 退出码 | 含义 |
| ------ | ---- |
| 0 | 成功 |
| 1 | 通用错误 |
| 2 | 参数错误 |
| 3 | daemon 未运行 |
| 4 | 飞控未连接 |
| 5 | 安全护栏拒绝（Phase 2） |
| 6 | 飞控 NACK/超时（Phase 2） |

## 7. 自动化集成测试（可选）

SITL 运行时，可直接跑集成测试：

```bash
uv run pytest -m sitl
```

该测试会启动 daemon、断言已连接并能取到遥测、再停止 daemon。
（默认 `pytest -m "not sitl"` 会跳过它。可用 `MAVCTL_SITL_CONNECT` 覆盖连接串。）

## 一键回归（不需要 SITL）

```bash
uv run ruff check . && uv run mypy . && uv run pytest -m "not sitl"
```
