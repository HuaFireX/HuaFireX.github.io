# GitHub Pages 配置

---

## 什么是 GitHub Pages

GitHub Pages 是 GitHub 提供的静态站点托管服务，可以直接从 GitHub 仓库托管你的文档、项目网站或博客。

---

## 启用 GitHub Pages

### 1. 创建仓库

在 GitHub 上创建一个新仓库，仓库名建议：`username.github.io`

### 2. 推送代码

```bash
git init
git add .
git commit -m "initial commit"
git remote add origin https://github.com/username/username.github.io.git
git push -u origin main
```

### 3. 启用 Pages

1. 进入仓库设置 → Pages
2. Source 选择 **Deploy from a branch**
3. Branch 选择 `main`（或 `gh-pages`），目录选择 `/ (root)`
4. 点击 Save

### 4. 访问站点

访问 `https://username.github.io`

---

## Docsify 部署到 GitHub Pages

### 1. 初始化 Docsify 项目

```bash
npm i -g docsify-cli
docsify init ./docs
```

初始化后的目录结构：
```
docs/
├── index.html    # 入口文件（可参考根目录 index.html）
├── README.md     # 主页内容
├── _sidebar.md  # 侧边栏配置
└── ...
```

### 2. 本地预览

```bash
docsify serve docs
# 访问 http://localhost:3000
```

### 3. 推送到 GitHub

```bash
git init
git add .
git commit -m "docs: initial commit"
git remote add origin https://github.com/username/repo.git
git push -u origin main
```

### 4. 启用 Pages

1. 进入仓库 **Settings** → **Pages**
2. Source 选择 **Deploy from a branch**
3. Branch 选择 `main`，目录选择 `/ (root)` 或 `/docs`
4. 点击 Save，等待部署完成

---

## 部署方式

### 方式一：手动部署（适用于小型项目）

按照上面的步骤 1-4 操作即可。

### 方式二：使用 GitHub Actions（自动化部署）

创建 `.github/workflows/deploy.yml`：

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

> **注意**：docsify build 默认输出到 `docs/.nojekyll`

---

## index.html 配置参考

部署时可参考以下核心配置（完整配置见根目录 index.html）：

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>My Docs</title>
    <link rel="stylesheet" href="//cdn.jsdelivr.net/npm/docsify/lib/themes/vue.css">
</head>
<body>
    <div id="app">加载中...</div>
    <script>
        window.$docsify = {
            loadSidebar: true,
            subMaxLevel: 3,
            search: {
                paths: 'auto',
                placeholder: '搜索',
                noData: '找不到结果',
            },
            pagination: {
                previousText: '上一章节',
                nextText: '下一章节',
            }
        }
    </script>
    <script src="//cdn.jsdelivr.net/npm/docsify/lib/docsify.min.js"></script>
    <script src="//cdn.jsdelivr.net/npm/docsify/lib/plugins/search.min.js"></script>
</body>
</html>
```

常用插件：
- `docsify/lib/plugins/search.min.js` - 搜索功能
- `docsify/lib/plugins/emoji.min.js` - emoji 支持
- `docsify/lib/plugins/zoom-image.min.js` - 图片缩放
- `prismjs/components/prism-bash.js` - 代码高亮

---

## 自定义域名（可选）

1. 在仓库 Settings → Pages → Custom domain 输入你的域名
2. 在域名服务商处添加 CNAME 记录指向 `username.github.io`
3. 启用 HTTPS（GitHub 自动提供）

---

## 常见问题

**页面404？**
- 检查分支和目录是否正确
- 确认 index.html 存在于指定目录

**更新不生效？**
- 清除浏览器缓存
- 等待几分钟让 GitHub 部署完成

**自定义域名失效？**
- 检查 CNAME 文件是否被删除
- 重新在 Settings 中设置自定义域名
