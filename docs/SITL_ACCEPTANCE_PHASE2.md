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
> 注意：正因 `landed_state` 常缺失，已 armed 的 disarm 只有在拿到明确的地面
> 证据时才会放行（见 §7 的 `ground_state_unknown` 说明）。

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
| force arm（协议层禁止） | 直发 RPC `{"method":"arm","params":{"confirm":true,"force":true}}`；CLI 无 `arm --force` | 2 | unsupported_force_arm |
| 非法 --timeout | `mode LOITER --confirm --wait --timeout 0 / -1 / nan / inf / abc` | 2 | invalid_timeout |
| 非有限起飞高度 | `takeoff --alt nan --confirm`（NaN/±Infinity 同理） | 2 | invalid_altitude |
| 空中 disarm | 起飞后 `mavctl disarm --confirm`（已明确离地） | 5 | in_flight |
| 地面状态未知 disarm | 已 armed 但无地面证据时 `disarm --confirm`（如刚起飞的过渡区间、遥测暂缺） | 5 | ground_state_unknown |
| 地面证据过期 disarm | 已 armed、缓存的地面证据（on_ground 报告或低 `relative_alt`）存在但对应 age 缺失或超过 `max_ground_evidence_age_s`（默认 3.0 s）时 `disarm --confirm` | 5 | ground_state_stale |
| 模式表未就绪 | daemon 刚连上、模式表未填充时 `mode LOITER --confirm` | 5 | mode_map_unavailable |

> **空中 / 地面状态不明的 disarm 说明**：非 force 的 `disarm --confirm` 只有在
> **能证明在地面，且地面证据足够新鲜**时才放行——显式
> `landed_state=on_ground`（要求 `landed_state_age_s` 已知且 ≤
> `max_ground_evidence_age_s`），或 `relative_alt` 已知且 ≤
> `max_on_ground_alt_m`（默认 0.5 m，保守值）且 `telemetry_age_s` ≤
> `max_ground_evidence_age_s`（默认 3.0 s）。判定顺序：
>
> 1. 确实离地（`landed_state=in_air` 或 `rel_alt` > `airborne_alt_threshold_m`
>    默认 1 m）→ `in_flight`，退出码 **5**。**离地证据不要求新鲜**——过期
>    离地证据同样拒绝，拒绝本身就是安全方向；
> 2. 有**新鲜**地面证据（两类证据任一新鲜即可）→ 放行；
> 3. 存在表面地面证据（on_ground 报告或低相对高度）但其 freshness age 缺失
>    或超过阈值 → **`ground_state_stale`，退出码 5**。缓存的旧数据不能等同
>    于"当前在地面"；
> 4. 其余（含 takeoff 后约 1 秒的过渡区间、ArduCopter 不发
>    `EXTENDED_SYS_STATE` 且遥测暂缺的情况）→ **`ground_state_unknown`，
>    退出码 5**。遥测缺失不再被当作"在地面"，飞控 NACK 不再作为设计上的防线。
>
> 只有 ordinary disarm 使用 freshness 门控；arm / takeoff / mode / land / rtl
> 的 guard 判定不使用 age。`--force`（仅 `disarm` 提供）可越过以上全部判定
> （checks 中标记 `forced`），语义为 **emergency motor stop; using in flight
> may cause a crash** —— 仅限紧急情况下的故意停桨。
>
> **arm 没有 --force**：`mavctl arm` 不提供该参数，adapter 层的 arm 固定发送
> param1=1.0 / param2=0.0，不存在绕过 pre-arm checks 的路径；即使恶意客户端
> 直发 RPC `{"confirm":true,"force":true}`，daemon 也以 exit 2
> （`unsupported_force_arm`）拒绝。

> **模式表未就绪说明**：连接建立但载具的模式映射表尚未填充时，目标模式无法
> 校验，此时 `mode` 以 `mode_map_unavailable` 拒绝（退出码 **5**），不会退化
> 为内部错误（退出码 1），也不会调用 adapter。按 hint 等 `mavctl status`
> 显示模式后再重试即可。语义约定：**退出码 4 只表示"没有存活的载具状态"
> （心跳缺失/过期/失链）**；链路存活但前置状态不可判定或未就绪的情况一律
> 归入退出码 5。

验证几条：

```bash
uv run mavctl takeoff --alt 999 --confirm ; echo "exit=$?"    # exit=5 (altitude_limit)
uv run mavctl takeoff --alt 0 --confirm ; echo "exit=$?"      # exit=2 (invalid_altitude)
uv run mavctl mode WARP --confirm ; echo "exit=$?"            # exit=2 (unknown_mode)
# 空中时：
uv run mavctl disarm --confirm ; echo "exit=$?"               # exit=5 (in_flight)
# 地面状态未知时（如刚起飞的过渡区间）：
uv run mavctl disarm --confirm ; echo "exit=$?"               # exit=5 (ground_state_unknown)
# --force 仅限紧急停桨（emergency motor stop; using in flight may cause a crash）：
uv run mavctl disarm --confirm --force ; echo "exit=$?"       # --force 覆盖（危险）
# arm 无 --force；直发 force=true 的 RPC 会被拒绝：
uv run mavctl arm --force --confirm ; echo "exit=$?"          # typer: no such option, exit=2
```

## 8. NACK / 超时（退出码 6）

飞控拒绝命令（NACK）或在重发 3 次后仍无 ACK（默认每次 5s 超时），退出码为 6。
`--wait` 超时（命令已 ACK 但目标状态未在 `--timeout` 内达成）同样返回 6：

```bash
uv run mavctl takeoff --alt 10 --confirm --wait --timeout 1 ; echo "exit=$?"
# 若 1s 内未到目标高度：exit=6，message 说明「accepted but did not complete within 1s」
```

### MAV_RESULT_IN_PROGRESS 语义

飞控对 `COMMAND_LONG` 可能先回 `MAV_RESULT_IN_PROGRESS`（5），表示**已接受、仍在执行**，
**不是** NACK。mavctl 将其与 `ACCEPTED`(0) 同等对待为 `outcome.accepted=true`：

- 默认模式：确认 ACK（含 IN_PROGRESS）后立即返回 exit 0，`result_name` 保留真实值
  （`IN_PROGRESS` 或 `ACCEPTED`）
- `--wait`：继续轮询目标状态；达成后 exit 0
- 真正的拒绝（`DENIED` / `FAILED` / `UNSUPPORTED` 等）仍为 exit 6

单测覆盖：`tests/test_adapter.py::test_command_in_progress_counts_as_accepted`、
`tests/test_server.py::test_in_progress_*`。

## 8b. --wait 中途失链（退出码 4，不是 6）

命令已被飞控 ACK 之后，若在 `--wait` 轮询期间心跳过期 / 链路断开，daemon 必须
**立即终止等待**并返回 **exit 4**（`VEHICLE_NOT_CONNECTED`），而不是把它当成
`--wait` 超时的 exit 6。错误 detail 含：

- `reason`: `link_lost_during_wait`
- `outcome`: 已收到的 COMMAND_ACK（证明命令侧已接受）
- message 说明：command was accepted but the link was lost during --wait

手动触发思路（SITL 侧可直接 kill 掉链路，或拔掉 `--out`）：

```bash
uv run mavctl takeoff --alt 10 --confirm --wait --timeout 60 &
# 在等待爬升时停止 SITL 或断开 udp 转发
# 预期：exit=4，JSON detail.reason == "link_lost_during_wait"（不是 exit=6）
```

单测：`tests/test_server.py::test_wait_link_loss_returns_exit_4_not_timeout`。

## 8c. 危险命令并发被安全串行化

状态变更命令在两层锁下串行执行，避免 TOCTOU 与 ACK 错归属：

1. **Adapter**：`_command_lock` 把「发送 COMMAND_LONG → 等 ACK → 重试」做成
   不可并发事务；`get_state` / `get_telemetry` **不**持该锁。
2. **Daemon**：`asyncio.Lock` 把「读最新 state → guard → dry-run/幂等 →
   adapter 调用 → `--wait`」包成一条事务。起飞/降落的 `--wait` 期间也不会
   插入另一条 arm/mode/rtl 等危险命令。
3. **status / telemetry** 始终可并发返回（不抢 command lock）。

行为预期：

- 同时发起 `arm` 与 `disarm`（二者共享 `MAV_CMD_COMPONENT_ARM_DISARM`）：
  按到达顺序串行完成，不会交叉消费对方的 `COMMAND_ACK`
- 两条同类命令（两个 `arm`）同样串行
- 一条命令卡在 `--wait` 时，另一条危险命令排队；`mavctl status` 仍立即返回

单测：

- `tests/test_adapter.py::test_concurrent_arm_and_disarm_are_serialized_no_cross_ack`
- `tests/test_adapter.py::test_concurrent_same_command_are_serialized`
- `tests/test_adapter.py::test_second_command_cannot_consume_first_ack`
- `tests/test_server.py::test_command_lock_serializes_and_status_stays_live`
- `tests/test_server.py::test_status_stays_live_during_wait_and_second_command_blocks`

## 8d. COMMAND_ACK 相关协议限制与迟到 ACK 策略

### MAVLink 限制（无法完美解决）

`COMMAND_ACK` 通常只携带 **command id** 与 **result**，**不**携带 confirmation
计数或 param1。因此：

- `arm` 与 `disarm` 共享 `MAV_CMD_COMPONENT_ARM_DISARM`，无法在 ACK 上区分二者；
- 若一次命令事务**整轮重试超时**后，迟到的 ACK 在「下一次同 command id 发送」
  之后到达，理论上可能被误匹配。

mavctl **不声称**能完美 correlation 每一次 ACK。

### 风险降低措施（策略 A：settle / quarantine window）

在 adapter 层实现可配置安静窗口 `command_ack_settle_s`（默认 1.0s）：

1. 某 command id 在完整超时后，进入 quarantine：该窗口内到达的 ACK **丢弃**，
   不写入 rendezvous map；
2. 下一次同 command id 的发送会先**等待 quarantine 结束**，再清 stale map 并发送；
3. 事务成功拿到 ACK 时清除该 id 的 quarantine。

这与 command lock、事务起始 stale-ACK clear、仅接受已锁定 autopilot 的 ACK
叠加使用，降低并发与迟到 ACK 风险，但**不是**完美协议层 correlation。

Agent 侧仍建议：危险命令后用 `mavctl status` 轮询确认 `armed` / `mode` /
`relative_alt` 等真实状态（幂等友好）。

单测：`tests/test_adapter.py::test_late_ack_after_timeout_dropped_during_quarantine`。

### 来源锁定

- 首个 `MAV_COMP_ID_AUTOPILOT1` HEARTBEAT 之前：**拒绝所有 COMMAND_ACK**（含默认 1/1）；
- 锁定后：仅该 system+component 的 HEARTBEAT / SYS_STATUS / GPS / 位置 / 姿态 /
  EXTENDED_SYS_STATE / HOME_POSITION / COMMAND_ACK 可更新快照或满足命令。

## 8e. freshness metadata（缓存陈旧度）

`status` 的 `*_age_s` 字段表示 **daemon 最后一次接受对应类别 MAVLink 消息
距当前的秒数**：基于 `time.monotonic()` 计算（不是 epoch wall-clock）；
`None` / `age=n/a` 表示从未收到该类消息。它是缓存陈旧度指标，不是物理测量。
来源映射：`battery_age_s`←SYS_STATUS、`gps_age_s`←GPS_RAW_INT、
`telemetry_age_s`←GLOBAL_POSITION_INT 或 ATTITUDE、
`home_position_age_s`←HOME_POSITION、`landed_state_age_s`←EXTENDED_SYS_STATE。

手动验收（SITL 运行中）：

```bash
uv run mavctl daemon start --connect udp:127.0.0.1:14550
uv run mavctl status
# 期望：battery / gps 行带 age=<秒>s；新增 telemetry : age=<秒>s 行；
#       home 行带 age（若 home 已知）；未收到的流显示 age=n/a
uv run mavctl status --json
# 期望 JSON 含 telemetry_age_s / gps_age_s / battery_age_s /
#   home_position_age_s / landed_state_age_s，非负数或 null
#   （ArduCopter SITL 常不发 EXTENDED_SYS_STATE，landed_state_age_s
#   为 null 是正常现象——这正是 guard 不依赖 landed_state 的原因）
```

断链后的陈旧度（停止 SITL 或断开链路，等 >3 秒无心跳）：

```bash
uv run mavctl status
# 期望：connection : DISCONNECTED，mode/armed/status 降为 n/a，
#       但各 age 字段继续增长（缓存陈旧度仍可见，不因失链归零/隐藏）
```

注意：

- `connected` 仍以 heartbeat 为唯一依据；freshness age 目前只参与
  ordinary disarm 的正向地面证据判定（`max_ground_evidence_age_s`，默认
  3.0 s；过期 → `ground_state_stale` / exit 5，见 §7），其余命令的
  guard 判定不使用 age；future guards may use stream freshness。

```bash
uv run mavctl daemon stop
```

## 9. 退出码总表

| 退出码 | 含义 |
| ------ | ---- |
| 0 | 成功（含幂等 no-op、dry-run 通过、IN_PROGRESS 已接受） |
| 1 | 通用错误 |
| 2 | 参数错误（非法/非有限高度、非法 --timeout、未知模式、缺少必填项、不支持的 force arm） |
| 3 | daemon 未运行 |
| 4 | 飞控未连接（含 --wait 中途失链）。**只表示没有存活的载具状态**：心跳缺失/过期/失链 |
| 5 | 安全护栏拒绝。链路存活但前置状态不可判定/未就绪/过期也归此码（`ground_state_unknown`、`ground_state_stale`、`mode_map_unavailable`），不占用 4 |
| 6 | 飞控 NACK / ACK 超时 / --wait 超时（链路仍在） |

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
