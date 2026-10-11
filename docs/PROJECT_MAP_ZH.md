# mavctl 项目地图（中文）

> 一页速览：mavctl 是什么、做到哪了、边界在哪、下一步是什么。
> 数据基准：main `0a5d5ca`（0.4.1.dev0 开发线）。本文件只记录已验证事实与
> 已定方向，不包含未验证的协议结论。

## 1. 生态位：Harness → mavctl → ArduPilot

```text
┌────────────────────────────┐
│  Future Drone Agent Harness │   候选方向（尚无仓库，见 roadmap §D）
│  / RL / 编排层（探索性）     │
└─────────────┬──────────────┘
              │  通过 CLI / --json 驱动（agent-first 契约）
              ▼
┌────────────────────────────┐
│          mavctl             │   headless 地面站：护栏、operation 观察、
│  CLI → daemon → adapter     │   mission JSON v1、结构化退出码
└─────────────┬──────────────┘
              │  MAVLink（pymavlink，独立 GCS 身份 sysid 254）
              ▼
┌────────────────────────────┐
│     ArduPilot 飞控          │   pre-arm checks 的最终安全权威
│  （ArduCopter 优先）        │   SITL 验证 / 真机未验证
└────────────────────────────┘
```

mavctl 的定位：**不是**自动驾驶仪的替代品，而是 agent 与飞控之间一层
可审计、可解释、护栏完备的指挥通道。ArduPilot 自己的 pre-arm 检查始终
是起飞安全的最后一道权威闸门；mavctl 的护栏是客户端侧的附加防线。

## 2. 内部分层（依赖方向严格单向）

```text
┌──────────────────────────────────────────────┐
│ CLI 层（src/mavctl/cli/，typer）              │  参数解析、格式化，
│                                              │  不含业务逻辑
├──────────────────────────────────────────────┤
│ Daemon 层（src/mavctl/daemon/，asyncio）      │  常驻进程：MAVLink 连接、
│                                              │  遥测缓存、guard 链、
│                                              │  operation registry、
│                                              │  Unix socket JSON-RPC
├──────────────────────────────────────────────┤
│ Adapter 层（src/mavctl/adapter/）             │  pymavlink 唯一使用处：
│                                              │  收发、ACK/NACK、
│                                              │  mission 事务、消息观察
└──────────────────────────────────────────────┘
                     │
                     ▼
              pymavlink / MAVLink wire
```

## 3. 已完成的阶段（命令 + 白话解释）

| 阶段 | 发布载体 | 命令 | 白话解释 |
| --- | --- | --- | --- |
| Phase 1 | tag `v0.1.0-phase1`（PR #1） | `daemon start/stop/status`、`status`、`telemetry` | 常驻 daemon 管住唯一的飞控连接；一条 `status` 让 agent 单次查询就能建立完整车辆认知；遥测是结构化快照 |
| Phase 2 | tags `v0.2.0-phase2` / `v0.2.0` | `arm`、`disarm`、`mode`、`takeoff --alt`、`land`、`rtl` | 飞行控制命令全部带安全护栏：`--confirm` 门控、`--dry-run` 预演、ACK/NACK 语义、新鲜地面证据才允许 disarm |
| Phase 2.1 | 0.2.1（PR #14） | `--version`；`status --json` 的 `*_age_s` 字段 | 每条遥测流都带"几秒前"标签，缓存过期看得见（`ground_state_stale` 拒绝用陈旧证据解锁） |
| Phase 3A | **0.3.0**（PRs #27/#28/#29，tag `v0.3.0`） | `mission upload/download/clear` | Mission JSON v1（takeoff/waypoint/land/rtl）无损上传下载；clear 经计数读回验证；对共享 MAVProxy 拓扑的 relay 残留做了收敛 |
| Phase 3B-0 | **0.4.0**（PR #35） | `operation get <id>`；`takeoff/land/rtl --wait` | 长命令的等待变成 daemon 侧的"可观察 operation"：命令被 ACK 后锁立即释放，等待期间车还能被指挥；后到的命令会**取代**进行中的等待（superseded ≠ 被取消） |
| Phase 3B-1 | **0.4.0**（PR #36；SITL conformance 见 SITL_ACCEPTANCE_PHASE2 §8i） | `mission start --confirm [--wait --timeout]` | 向已具备（已上传任务 + armed + 新鲜链路）的载具发送启动指令；ArduCopter 上会把车切到 AUTO 开始执行；`--wait` 只观察**启动里程碑**（AUTO + mission ACTIVE），不等整趟任务；已在跑时重复调用是幂等 no-op |

附：0.2.2 的 daemon 一致性修正（心跳单阈值、mode TOCTOU）随 0.3.0 一并发布。
0.4.0 通过 GitHub OIDC 正式发布 PyPI（tag `v0.4.0` + GitHub Release）。
0.4.1.dev0 开发线已合入：mission help 文案修正（PR #38）、0.4.0 发布记录
同步、vehicle health 设计文档（PR #39）。

## 4. 已实现命令树（0.4.1.dev0 @ main）

```text
mavctl
├── daemon start|stop|status        # 常驻进程管理（--connect/--source-system）
├── status [--json]                 # 车辆全貌（含 *_age_s 新鲜度）
├── telemetry [--json]              # 位置/姿态/速度快照
├── arm | disarm --confirm          # 电机解锁/上锁（disarm 需新鲜地面证据）
├── mode MODE --confirm [--wait]    # 切模式（--wait 为 legacy 等待，未迁移 operation）
├── takeoff --alt --confirm [--wait]
├── land --confirm [--wait]
├── rtl --confirm [--wait]
├── mission
│   ├── upload <file> --confirm [--dry-run]
│   ├── download [--output] [--json]
│   ├── clear --confirm [--dry-run]
│   └── start --confirm [--wait --timeout] [--dry-run]
├── operation get <id> [--json]     # 只读观察一次 --wait operation
└── --version
```

横切契约：所有命令支持 `--json`；改状态必须 `--confirm`；危险命令支持
`--dry-run`；错误输出到 stderr（JSON 模式为 `{"error": {...}}`）。
退出码：0 成功 / 1 通用错误 / 2 参数错误 / 3 daemon 未运行 / 4 飞控未连接 /
5 安全护栏拒绝 / 6 NACK/ACK 超时或 `--wait` 超时（超时 ≠ 车辆动作被取消）。

## 5. 明确未实现（当前边界，不是路线图承诺）

| 能力 | 状态 | 相关方向 |
| --- | --- | --- |
| mission pause / resume / stop / set-current | 未实现 | Phase 3B 剩余范围（design §C.2） |
| operation cancel | 未实现 | Issue #21（supersession 已随 0.4.0 发布，取消面仍开） |
| health / pre-arm runtime（EKF 观察等） | 未实现 | Issue #4：观察设计已合并（PR #39），observe-only 路线 |
| Windows 原生 daemon IPC | 未实现 | Issue #19：design-only |
| parameters / 日志下载 / geofence / 固件刷写 / 多载具 | 未实现 | Phase 4 候选（roadmap §C） |
| Agent Harness（独立仓库） | 未启动 | roadmap §D 候选，无仓库 |
| RL / 强化学习方向 | 未启动 | roadmap §F 探索性记录 |

## 6. 验证边界与诚实声明

- **latest production PyPI：`0.4.0`**；main 开发版本：**`0.4.1.dev0`**（未发布）。
- 验证范围：**仅 ArduPilot / ArduCopter SITL**（本机 loopback 拓扑：共享
  `udp:127.0.0.1:14550` 与隔离 `tcp:127.0.0.1:5770`）——覆盖链路/遥测、
  护栏飞行命令、mission 传输与 mission start 执行 conformance。
- **不声明任何真实飞机验证或支持**；ArduPilot checkout 为本地修订版
  （`4c98c9221a`，唯一审阅过的源码改动是 `AP_FWVersion.h` 的 macOS
  构建 workaround，不影响 mission/GCS 运行时）。
- 测试基线：非 SITL 门禁 ruff + mypy strict + 533 tests；SITL 套件
  13 tests（`pytest -m sitl`，CI 可跳过）。

## 7. 下一步候选（简单对照）

| 候选 | 当前状态 | 前置依赖 | 量级 |
| --- | --- | --- | --- |
| Health runtime（observe-only v1） | 设计已合并（Issue #4），零实现 | EKF 流在 SITL 的实际到达验证；flag→状态映射表取证 | 小-中 |
| mission pause/resume/stop | 未实现 | Issue #21 取消契约（与 supersession 的关系） | 中 |
| Windows 原生 daemon IPC | design-only（Issue #19） | 平台决策（命名管道/服务形态） | 中-大 |
| Harness POC（独立仓库） | 候选，无仓库 | mavctl CLI/--json 契约已足够稳定 | 中 |

（顺序不代表承诺；任何 health 相关工作都遵循
`docs/design/vehicle-health-prearm-v1.md` 的渐进路线：
observe → validate → advisory → 可能的 hard guard。）

## 8. 相关文档

- 设计：`docs/design/mission-protocol-v1.md`（0.3.0 已发布）、
  `docs/design/mission-execution-phase3b.md`、
  `docs/design/long-running-operation-interruption.md`、
  `docs/design/vehicle-health-prearm-v1.md`、
  `docs/design/product-architecture-roadmap.md`
- 验收：`docs/SITL_ACCEPTANCE.md`（Phase 1 手动指南）、
  `docs/SITL_ACCEPTANCE_PHASE2.md`（Phase 2 / 3A / 3B-1 conformance 记录）、
  `docs/SKILLS_CLI_ACCEPTANCE.md`
- 发布：`docs/PUBLISHING.md`（版本历史 + OIDC 流程）
- Agent Skill：`skills/mavctl-flight/`（安装：`npx skills add LeaderOnePro/mavctl -y -g`）
