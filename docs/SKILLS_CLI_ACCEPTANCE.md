# skills CLI 安装验收记录

本文档记录一次针对社区 skills CLI（`npx skills add`）的实测验收：
验证本仓库的 Agent Skill（`skills/mavctl-flight/`）能否通过该 CLI 安装，
以及安装产物的完整性。所有结论均来自隔离环境中的实际命令输出，
未对 CLI 行为做任何假设。

## 0. 验收环境与隔离方法

- 日期：2026-08-29
- skills CLI：1.5.23（经 `npx skills --version` 确认）
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
的默认行为就是复制；README 中的安装命令因此不区分 `--copy` 与否。

因此 README（中英双语）记录的安装命令只有以上实测过的形式；
未实测的调用形式一律不写。

## 4. 观察到的异常（CLI 侧行为，非本仓库问题）

- bare `./agent/` 目录目标下的 SKILL.md 被 CLI 重写为仅含 `description`
  的 frontmatter，丢失 `name` 字段；随后 `npx skills list` 对该副本告警
  `missing required frontmatter field(s): name`。其余 55 个目标目录的
  副本均与源文件逐字节一致。该怪癖属于 skills CLI 1.5.23 对特定 agent
  目标格式的转换行为，不影响 `.claude` 等主要目标的安装结果。

## 5. 范围声明

- 本验收只覆盖 skills CLI 1.5.23 的实际行为；CLI 后续版本的行为变化
  不在本仓库承诺范围内。
- 未修改 mavctl 的任何实现代码、Skill 内容或包版本；本文档与 README
  的更新是本次验收的全部产出。
