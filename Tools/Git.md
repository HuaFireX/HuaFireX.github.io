# Git教程

## 结构图

![git](D:\main_ws\Learn_docs_ws\Tools\img\git.jpg)

## 常用命令

#### 克隆仓库
`git clone <https开头.git结尾的url>` - 将远程仓库的所有文件**克隆**到当前文件夹中

#### 分支管理
`git branch`

- `git branch <取一个分支名>`  - 新建一个本地分支 
- `git branch -d <指定分支名>` - 删除指定分支
- `git branch -D <指定分支名>` - 强制删除指定分支
- `git branch --merge` - 查看分支的合并情况
- `git branch -m <原名> <新名>` - 指定分支重命名

#### 分支合并

`git merge <被合并分支名>` -合并到当前分支

#### 暂存文件

`git add <.> / <文件名> / <文件夹名>` - **添加**文件**到暂存区**

#### 提交修改
`git commit` - **提交**请求命令

- `git commit -m "提交信息"` - 填写提交信息
- `git commit -a` - 将被修改的文件暂存并提交

#### 远程仓库
`git remote` - **远程仓库**命令

- `git remote add <仓库名> <URL>` - 创建远程仓库代号
- `git remote rm <仓库名>` - 删除远程仓库

#### 推送到远程
`git push <远程仓库名> <分支名>` - 将本地文件**更新到远程仓库**

- `git push --set-upstream origin dev` - 本地新分支推送到远程

#### 拉取更新
`git pull <远程仓库名> <分支名>` - 将远程文件**更新到本地**

- `git pull origin master --rebase` - 改变分支**基底** *(建议初建时使用)*

#### 临时保存
`git stash` - 保存当前分支未完成的修改供后续恢复

- `git stash save "信息"` - 将修改临时保管
- `git stash pop` - **取出** stash 中的文件

> **注意**：暂存区是栈结构，后放入的排在前面

#### 其他常用命令
- `git checkout` - 切换分支
- `git mv <旧> <新>` - **移动**或**重命名**文件
- `git log --all --pretty=oneline --abbrev-commit --graph` - 图形化**分支结构**
- `git status` - 查询当前状态
- `git branch -vv` - 查看分支链接关系

## 初始化仓库

### 从零开始创建仓库

**1. 初次使用前配置用户信息**

```bash
git config --list
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

**2. 在项目目录初始化仓库**

```bash
git init
git add .
git commit -m "初始提交"
```

**3. 关联远程仓库**（可选）

```bash
git remote add origin <仓库URL>
git push -u origin master
```

**4. 三步工作流：暂存、提交、推送**

```bash
git add .
git commit -m "更新说明"
git push
```

**5. 其他配置**

```bash
# 为本地分支创建跟踪关系
git branch --set-upstream-to=origin/<远程分支> <本地分支>

# 创建 .gitignore 防止二进制污染
echo "*.pkl" >> .gitignore
```

## 认证信息

**Gitee** - 用户名：`Y_H_U_A`  
**GitHub** - 用户名：`HuaFireX`

## 高级用法

### 大文件处理（LFS）

> **注意**：gitee 暂未开通免费 LFS，需企业账号

清理缓存并重置大文件提交：

```bash
git filter-branch --force --index-filter 'git rm -rf --cached --ignore-unmatch XXX.pkl' --prune-empty --tag-name-filter cat -- --all

# 增大缓存空间
git config --global http.postBuffer 7242880000

# 查询提交历史并回退
git reflog
git reset <commit-hash>

# 追踪大文件
git lfs track "*.zip"
git lfs track  # 查询已追踪文件
```

### 常见问题排查

**连接错误：**
```bash
fatal: unable to access 'https://github.com/...': Failed to connect
```
解决：取消代理配置
```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

**本地代理配置：**
```bash
git config --global -l  # 查询配置
git config --global http.proxy http://127.0.0.1:<端口>  # 设置代理
```