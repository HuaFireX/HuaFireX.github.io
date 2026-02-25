# GitHub Actions 入门

---

## 什么是 GitHub Actions

GitHub Actions 是 GitHub 提供的**自动化工作流**工具，可以在代码推送、合并等事件发生时自动执行任务，如：
- 自动运行测试
- 自动构建和部署网站
- 自动发布版本

---

## 核心概念

| 概念 | 说明 |
|------|------|
| **Workflow（工作流）** | 整个自动化流程，包含多个 Job |
| **Job（任务）** | 一组 Step，每个 Job 在独立的虚拟机中运行 |
| **Step（步骤）** | 具体的操作（如安装依赖、运行命令） |
| **Action（动作）** | 可复用的步骤单元 |

---

## 快速开始

### 1. 创建工作流文件

在仓库根目录下创建 `.github/workflows/文件名.yml`：

```
仓库/
├── .github/
│   └── workflows/
│       └── deploy.yml    ← 工作流文件
└── ...
```

### 2. 最小示例

```yaml
name: Hello World

on: push

jobs:
  hello:
    runs-on: ubuntu-latest
    steps:
      - name: Print Hello
        run: echo "Hello World!"
```

### 3. 推送到 GitHub

```bash
git add .
git commit -m "add workflow"
git push
```

然后在仓库的 **Actions** 标签页查看运行结果。

---

## 常用指令

### 触发条件（on）

```yaml
on:
  push:                    # 推送到任意分支时触发
  push:
    branches:
      - main              # 只在推送到 main 分支时触发
  pull_request:           # 发起 PR 时触发
  schedule:
    - cron: "0 0 * * *"  # 每天午夜执行
```

### 运行虚拟机（runs-on）

```yaml
jobs:
  build:
    runs-on: ubuntu-latest    # Linux
    # runs-on: windows-latest  # Windows
    # runs-on: macos-latest   # macOS
```

### 检出代码（actions/checkout）

```yaml
steps:
  - uses: actions/checkout@v4
```

### 安装 Node.js

```yaml
- uses: actions/setup-node@v4
  with:
    node-version: '20'
```

---

## 完整示例：Docsify 自动部署

```yaml
name: Deploy Docs

on:
  push:
    branches:
      - main

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: '20'

      - name: Install docsify-cli
        run: npm install -g docsify-cli

      - name: Build docs
        run: docsify build docs

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./docs/.nojekyll
```

---

## 常见问题

**在哪里查看运行结果？**
- 仓库页面 → Actions 标签页

**如何调试？**
- 在 Workflow 中添加 `run: echo "${{ toJson(github) }}"` 输出变量

**需要付费吗？**
- 免费额度：每月 2000 分钟（公共仓库无限制）

**为什么没触发？**
- 检查 `on` 的触发条件是否正确
- 确认 Workflow 文件在 `.github/workflows/` 目录下
