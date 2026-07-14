# Issue tracker

本仓库的 issue 和 PRD 当前作为 markdown 文件存放在 `.scratch/` 中。计划在仓库发布到 GitHub 后切换到 GitHub Issues。

## 当前：本地 Markdown

- 一个功能一个目录：`.scratch/<feature-slug>/`
- PRD 是 `.scratch/<feature-slug>/PRD.md`
- 实现 issue 是 `.scratch/<feature-slug>/issues/<NN>-<slug>.md`，从 `01` 开始编号
- 分诊状态记录在每个 issue 文件顶部附近的 `Status:` 行（角色字符串见 `triage-labels.md`）
- 评论和对话记录追加到文件底部 `## Comments` 标题下

### 当技能说"发布到 issue tracker"时

在 `.scratch/<feature-slug>/` 下创建新文件（必要时创建目录）。

### 当技能说"获取相关 ticket"时

读取引用路径处的文件。用户通常会直接传路径或 issue 编号。

## 计划：GitHub Issues

仓库发布到 GitHub 后，重新运行 `setup-ouyangjiahong-skills`，并把现有 `.scratch/<feature>/PRD.md` 转换为 GitHub issue（每个实现 issue 转为子任务或单独 issue，由届时的工作量决定）。