# skills CLI 安装验收记录

本文档记录一次针对社区 skills CLI（`npx skills add`）的实测验收：
验证本仓库的 Agent Skill（`skills/mavctl-flight/`）能否通过该 CLI 安装，
以及安装产物的完整性。所有结论均来自隔离环境中的实际命令输出，
未对 CLI 行为做任何假设。

## 0. 验收环境与隔离方法

- 初轮验收：2026-08-29；补充验证轮：2026-08-31（README 候选全局安装命令
  与显式 agent target，见第 5 节）
- skills CLI：1.5.23（经 `npx skills --version` 确认，两轮一致）
- Node：v24.19.0 / npm 12.0.2（macOS）
- 隔离措施：
  - 每次安装都在独立的 `mktemp -d` 临时目录中进行；
  - `HOME`、`XDG_CONFIG_HOME`、npm cache 全部重定向到临时目录内，
    CLI 无法触及真实的 `~/.claude`、`~/.ssh`、全局 agent 配置；
  - mavctl 仓库本身未执行任何安装命令，仓库工作区全程 clean；
  - 未运行任何飞行控制命令，未发布 PyPI，未创建 tag。

## 1. 只读检查

安装探测前确认（全部通过）：

- `skills/mavctl-flight/SKILL.md`（79 行）及
  `references/workflows.md`（155 行）、`references/safety.md`（110 行）、
  `references/troubleshooting.md`（143 行）存在且内容完整；
- `SKILL.md` frontmatter 含 `name: mavctl-flight` 与完整 `description`；
- `tests/test_skill_docs.py` 14 个测试全部通过；
- `git status` clean。

## 2. 源解析探测（`add <source> -l`，只列出不安装）

| 源 | 实际结果 |
| --- | --- |
| 本地 skill 目录 `…/mavctl/skills/mavctl-flight` | `Local path validated` → `Found 1 skill: mavctl-flight` |
| 仓库根目录 `…/mavctl` | 同样发现 `mavctl-flight`（无根 SKILL.md 时递归搜索子目录） |
| GitHub 缩写 `LeaderOnePro/mavctl` | `Repository cloned` → `Found 1 skill: mavctl-flight` |

GitHub 源在 `HOME` 被重定向到空临时目录（无 ssh 密钥、无 gh 凭据）的
情况下克隆成功，说明走的是匿名 HTTPS——本仓库当前为 public
（`gh repo view` 确认 `isPrivate: false`）。

## 3. 安装实测与产物校验

实测一：本地路径，全量安装

```bash
npx skills add /path/to/mavctl/skills/mavctl-flight --all --copy
```

结果：CLI 在项目目录下创建了 56 个 agent 目录（`.claude`、`.zcode`、
`.codex`、`.cursor` 等）中的 skill 副本，并生成 `skills-lock.json`。
`cmp` 校验 `.claude` 与 `.zcode` 目录下的 SKILL.md / safety.md 与源文件
逐字节一致。`npx skills list` 将其注册为 project skill，并回链源路径。

实测二：GitHub 源，指定单 agent

```bash
npx skills add LeaderOnePro/mavctl -s mavctl-flight -a claude-code -y
```

结果：`Installed 1 skill ✓ mavctl-flight (copied) → ./.claude/skills/mavctl-flight`，
产物与源文件逐字节一致。

实测三：同一命令去掉 `--copy`（CLI 默认模式）

结果：输出同样标注 `(copied)`，`.claude/skills/mavctl-flight` 是真实目录
而非符号链接。即在 skills 1.5.23 的实测中，GitHub 源 + claude-code 目标
（project-local）的默认行为就是复制。

以上为初轮实测记录。README 最终呈现的安装命令由第 6 节的 README policy
决定；针对全局安装与显式 agent target 的第二轮补充实测见第 5 节。

## 4. 观察到的异常（CLI 侧行为，非本仓库问题）

- bare `./agent/` 目录目标下的 SKILL.md 被 CLI 重写为仅含 `description`
  的 frontmatter，丢失 `name` 字段；随后 `npx skills list` 对该副本告警
  `missing required frontmatter field(s): name`。其余 55 个目标目录的
  副本均与源文件逐字节一致。该怪癖属于 skills CLI 1.5.23 对特定 agent
  目标格式的转换行为，不影响 `.claude` 等主要目标的安装结果。

## 5. 补充验证（2026-08-31）：全局安装与显式 agent target

沿用同一隔离方法：每项实验使用独立 `mktemp -d` 临时目录，`HOME`、
`XDG_CONFIG_HOME`、`XDG_DATA_HOME`、`XDG_CACHE_HOME`、npm cache 全部重定向，
在独立 target project 目录中执行，从不从本仓库工作目录执行安装。

Agent id 枚举：CLI 没有列举 agent 的专用子命令（`skills agents` 返回
`Unknown command: agents`）。向 `-a` 传入无效 id 时，CLI 校验报错并在输出
中给出全部 79 个合法 id（该探针经树检查确认零副作用）。与本仓库相关的
确认：`zcode`、`claude-code`、`codex`、`pi` 均存在；`grok-build` 不存在
（实际 id 为 `grok`）；`agy` 不存在（相近 id 为 `antigravity` /
`antigravity-cli`）；`universal`（`~/.agents` store）与 `promptscript`
也是合法 id。

### 5.1 README 候选全局命令

```bash
npx skills add LeaderOnePro/mavctl -y -g
```

实测结果（不带 `-s`）：

- `Found 1 skill` → 自动选中 `mavctl-flight`（仓库内唯一 Skill，无需
  `-s` 即可靠发现）；
- CLI 检出 77 个 agent runtime，并按当前环境自动选择安装目标——本次隔离
  环境中自动选中 ZCode。该选择取决于宿主环境与配置，不由命令显式指定；
- 写入位置全部在用户 HOME：`~/.agents/skills/mavctl-flight`（universal
  store，真实复制，SKILL.md 与 3 个 references 逐字节一致）与
  `~/.zcode/skills/mavctl-flight`（symlink → universal store，经
  `readlink` 确认）；target project 与 XDG 目录零写入；
- 生成用户级 registry `~/.agents/.skill-lock.json`（记录 source 与
  content hash）；不生成项目级 `skills-lock.json`；
- `npx skills list -g` 显示 `~/.agents/skills/mavctl-flight`，
  Agents: ZCode；
- 每次都会输出一条 runtime-specific 失败行：
  `✗ PromptScript: PromptScript does not support global skill installation`
  （该 runtime 不支持 global 安装；属提示性失败，不影响其他目标）；
- 与带 `-s mavctl-flight` 的同形命令行为完全等价。

### 5.2 显式 agent target 矩阵（`-s mavctl-flight -a <id> -y -g`）

| id | 成功 | 实际安装位置 | 形态 | 备注 |
| --- | --- | --- | --- | --- |
| `zcode` | ✓ | `~/.zcode/skills/mavctl-flight` | 真实复制 | 显式指定时不创建 universal store；references 逐字节一致 |
| `claude-code` | ✓ | `~/.claude/skills/mavctl-flight` | 真实复制 | 与第 3 节实测二一致 |
| `codex` | ✓ | `~/.agents/skills/mavctl-flight` | 真实复制 | 不创建 `~/.codex`；`skills list -g` 显示 `Agents: not linked`（Codex 读取 universal 目录） |
| `pi` | ✓ | `~/.pi/agent/skills/mavctl-flight` | 真实复制 | references 逐字节一致 |

各 target 的落点形态不同（直拷 / universal store / symlink），且自动模式
（不带 `-a`）的目标选择随宿主环境变化——这是 README 不列举具体 runtime、
只声明"由 skills CLI 按环境与配置处理"的直接依据。

## 6. README policy（README 呈现策略）

README intentionally exposes only:

```bash
npx skills add LeaderOnePro/mavctl -y -g
```

The detailed target-specific, project-local, bulk-copy, and manual-linking
behaviour belongs to this acceptance record rather than the primary user
onboarding path. The README does not pin a skills CLI version, does not name
specific agent runtimes, and does not promise coverage of every runtime; the
skills CLI chooses supported runtimes according to its current environment
and configuration.

中文对应：README 只呈现上面这一条通用安装命令。指定 target、project-local、
批量复制（`--all --copy`）与手动 symlink 等细节行为只保留在本验收记录中，
不属于面向用户的主 onboarding 路径。README 不固定 skills CLI 版本、不点名
具体 agent runtime、也不承诺覆盖所有 runtime；skills CLI 会根据其当前环境
与配置选择它所支持的 runtime。

补充说明：

- `--all --copy`（第 3 节实测一：写入 56 个 agent 目录、生成项目级
  `skills-lock.json`；第 4 节：bare `./agent/` 目标的上游格式化异常）副作用
  大且存在已知上游怪癖，不作为 README 常规安装路径；
- project-local 安装（第 3 节实测二/三）与手动 symlink 仅作为验收记录中的
  技术参考保留，不是 README onboarding 的一部分。

## 7. 范围声明

- 本验收只覆盖 skills CLI 1.5.23 的实际行为（2026-08-29 初轮与
  2026-08-31 补充轮）；CLI 后续版本的行为变化不在本仓库承诺范围内。
- 未修改 mavctl 的任何实现代码、Skill 内容或包版本；本文档与 README
  的更新是本次验收的全部产出。
