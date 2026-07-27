# SITL 手动验收指南（Phase 2：飞行控制 + 安全护栏）

在本地 ArduPilot SITL 上手动验收 mavctl Phase 2：`arm`/`disarm`/`mode`/
`takeoff`/`land`/`rtl`，以及 `--confirm` 护栏、`--dry-run`、`--wait`、退出码语义。

> 阅读顺序：先做 [Phase 1 验收](SITL_ACCEPTANCE.md) 确认链路正常，再做本文。

## 0. 前置 & 启动

```bash
uv sync
# 终端 A：启动 SITL
sim_vehicle.py -v ArduCopter --out udp:127.0.0.1:14550
# 终端 B：启动 daemon
uv run mavctl daemon start --connect udp:127.0.0.1:14550
uv run mavctl status          # 确认 CONNECTED、gps 为 3d_fix 以上
```

## 1. 自描述 status（幂等 agent 的信息基础）

```bash
uv run mavctl status
```

预期（数值随 SITL 而定）——单次查询即可重建认知：

```
connection : CONNECTED (udp:127.0.0.1:14550)
heartbeat  : 0.20s ago  sys=1 comp=1
mode       : GUIDED
armed      : disarmed
status     : standby  landed=n/a  rel_alt=-0.00 m
battery    : 12.60 V  0.00 A  100 %
gps        : rtk_fixed  sats=10
home       : lat=-35.3632621  lon=149.1652374  alt_msl=584.09 m
```

> 说明：`landed_state` 在 ArduCopter 上常为 `n/a`（该机型默认不发
> `EXTENDED_SYS_STATE`）；护栏与 `--wait` 判定用 `relative_alt` / `armed`
> 作为稳健回退，不依赖它。`home` 在设定家点后出现。

## 2. 护栏：缺少 --confirm（退出码 5）

```bash
uv run mavctl arm ; echo "exit=$?"
```

预期：

```
error: arm is a state-changing command and requires explicit confirmation
hint: re-run with --confirm (and --dry-run first to preview): mavctl arm --confirm
exit=5
```

JSON 形式（错误结构含 reason/message/hint）：

```bash
uv run mavctl arm --json ; echo "exit=$?"
# {"error":{"code":5,"message":"...","detail":{"reason":"confirmation_required","hint":"...","checks":[...]}}}
# exit=5
```

## 3. --dry-run：跑完护栏但不执行

```bash
uv run mavctl arm --confirm --dry-run ; echo "exit=$?"
```

预期（退出码反映检查是否通过；此处通过 → 0，且未真正 arm）：

```
[dry-run] arm: WOULD EXECUTE
  [PASS] confirm: confirmed
  [PASS] gps_fix: fix_type=6 (rtk_fixed)
exit=0
```

反例——护栏会拒绝的 dry-run（未 arm 时 takeoff），退出码为 5：

```bash
uv run mavctl takeoff --alt 10 --confirm --dry-run ; echo "exit=$?"
# [dry-run] takeoff 会因 not_armed 被拒；exit=5，输出含 hint 指导正确顺序
```

## 4. 正常起飞流程（含 --wait）

```bash
uv run mavctl mode GUIDED --confirm --wait --timeout 10 ; echo "exit=$?"   # exit=0
uv run mavctl arm --confirm ; echo "exit=$?"                               # arm: ACCEPTED, exit=0
uv run mavctl status                                                       # 确认 armed=ARMED（心跳更新后）
uv run mavctl takeoff --alt 10 --confirm --wait --timeout 45 ; echo "exit=$?"
# takeoff: ACCEPTED (target state reached)   exit=0
uv run mavctl status                                                       # rel_alt ≈ 10 m
```

> 顺序很重要：takeoff 要求「已 GUIDED 且已 armed」。若顺序错，护栏会用
> hint 告诉你正确命令序列。arm 的 ACK 早于 armed 心跳约 1 秒，脚本里应
> `status` 轮询确认 armed 后再 takeoff。

## 5. 幂等：重复已达成的状态变更返回成功

```bash
uv run mavctl mode GUIDED --confirm ; echo "exit=$?"
# mode: already satisfied (already in GUIDED) — no action taken
# exit=0
uv run mavctl arm --confirm ; echo "exit=$?"
# arm: already satisfied (already armed) — no action taken   （已在空中/已 armed 时）
# exit=0
```

## 6. 返航 / 降落（含 --wait 直到落地上锁）

```bash
uv run mavctl rtl --confirm --wait --timeout 120 ; echo "exit=$?"
# rtl: ACCEPTED (target state reached)   exit=0（等待到 disarmed）
uv run mavctl status                                                       # armed=disarmed
```

或就地降落：

```bash
uv run mavctl land --confirm --wait --timeout 90 ; echo "exit=$?"
```

## 7. 护栏反例集（每条都应被拒绝并给出 hint）

| 场景 | 命令 | 预期退出码 | reason |
| ---- | ---- | ---------- | ------ |
| 未确认 | `mavctl arm` | 5 | confirmation_required |
| GPS 未就绪 | `mavctl arm --confirm`（GPS 无 3D fix 时） | 5 | gps_not_ready |
| 未 armed 起飞 | `mavctl takeoff --alt 10 --confirm`（disarmed） | 5 | not_armed |
| 非 GUIDED 起飞 | `mavctl mode STABILIZE --confirm` 后 `takeoff ... --confirm` | 5 | wrong_mode |
| 超高度上限 | `mavctl takeoff --alt 999 --confirm` | 5 | altitude_limit |
| 非法高度 | `mavctl takeoff --alt 0 --confirm` | 2 | invalid_altitude |
| 未知模式 | `mavctl mode WARP --confirm` | 2 | unknown_mode |
| 空中 disarm | 起飞后 `mavctl disarm --confirm`（非 landed） | 5 | in_flight |

验证几条：

```bash
uv run mavctl takeoff --alt 999 --confirm ; echo "exit=$?"    # exit=5 (altitude_limit)
uv run mavctl takeoff --alt 0 --confirm ; echo "exit=$?"      # exit=2 (invalid_altitude)
uv run mavctl mode WARP --confirm ; echo "exit=$?"            # exit=2 (unknown_mode)
# 空中时：
uv run mavctl disarm --confirm ; echo "exit=$?"               # exit=5 (in_flight)
uv run mavctl disarm --confirm --force ; echo "exit=$?"       # --force 覆盖（危险）
```

## 8. NACK / 超时（退出码 6）

飞控拒绝命令（NACK）或在重发 3 次后仍无 ACK（默认每次 5s 超时），退出码为 6。
`--wait` 超时（命令已 ACK 但目标状态未在 `--timeout` 内达成）同样返回 6：

```bash
uv run mavctl takeoff --alt 10 --confirm --wait --timeout 1 ; echo "exit=$?"
# 若 1s 内未到目标高度：exit=6，message 说明「accepted but did not complete within 1s」
```

## 9. 退出码总表

| 退出码 | 含义 |
| ------ | ---- |
| 0 | 成功（含幂等 no-op、dry-run 通过） |
| 1 | 通用错误 |
| 2 | 参数错误（非法高度、未知模式、缺少必填项） |
| 3 | daemon 未运行 |
| 4 | 飞控未连接 |
| 5 | 安全护栏拒绝 |
| 6 | 飞控 NACK / ACK 超时 / --wait 超时 |

## 10. 收尾

```bash
uv run mavctl daemon stop
```

## 自动化集成测试（可选）

SITL 运行时可跑完整流程集成测试（mode→arm→takeoff→rtl→落地上锁）：

```bash
uv run pytest -m sitl
```

## 一键回归（不需要 SITL）

```bash
uv run ruff check . && uv run mypy . && uv run pytest -m "not sitl"
```
