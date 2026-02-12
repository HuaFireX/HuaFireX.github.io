# Git - 一种版本控制系统

## 结构图

![Git 结构图](https://api2.mubu.com/v3/document_image/dd23f680-2f62-4027-bab5-9001b7716608-22676279.jpg)

## 常用 Git 命令

### `git clone <https开头.git结尾的url>`
将远程仓库的所有文件**克隆**到当前文件夹中

### `git branch`

| 命令 | 说明 |
|------|------|
| `<取一个分支名>` | 新建一个本地分支 |
| `-d <指定分支名>` | 删除指定分支 |
| `-D <指定分支名>` | 强制删除指定分支（一般在未完成所有合并的时候使用） |
| `--merge` | 查看当前分支的合并分支情况 |

### `git add <.> / <文件名> / <文件夹名>`
**添加**所有文件或指定文件**到暂存区**

### `git commit`
**提交**请求命令

- `-m "提交信息"` - message，填写此次提交的信息，并将被修改的文件提交到本地仓库，便于版本管理
- `-a` - add，将被修改的文件暂存并提交的命令

### `git remote`
**远程仓库**命令

- `add <取一个远程仓库名，一般取origin> <远程仓库URL>` - 创建远程仓库代号便于后面使用。比如上面的 origin 取代冗长的远程仓库 URL
- `rm <已存在远程仓库名>` - 删除指定的已存在远程仓库

### `git push <远程仓库名> <分支名>`
将缓存在本地仓库的所有文件**更新到远程仓库**的指定分支下

- `git push --set-upstream origin dev` - 假设本地创建了一个名为 dev 的分支，远程仓库还没有这个分支

### `git pull <远程仓库名> <分支名>`
将远程仓库的指定分支下的所有文件**更新到当前文件夹**中

- `git pull origin master --rebase` - 改变分支**基底**
  > **注意**：后期建议少用，初建仓库时可以使用

### `git stash`
一般在需要先 pull 远程仓库的新版本的情况的时候使用：先将本地修改过的版本保存在 stash 里，等 pull 好新版本再从 stash 里取出

- `save "保存信息"` - 将修改过的文件放入 stash 里面临时保管
- `pop` - **取出** stash 中的所有文件，此时根据情况解决和新版本的冲突问题

> **注意**：暂存区是一个栈，后放入的内容排在前面

### `git checkout`
切换到某一分支

### `git mv <test.txt> <newtest.txt>`
**移动**或**重命名**文件、文件夹

### `git log --all --pretty=oneline --abbrev-commit --graph`
图形化**分支结构显示**

### `git status`
查询当前 git 状态（add、commit 等情况）

### `git branch -vv`
查看本地与远程仓库分支链接关系

## 创建仓库

### 从零开始创建自己的仓库（含远程仓库）

#### 1. 初次在某设备上创建仓库时建议：先配置用户姓名和邮箱

```bash
git config --list
git config --global user.name "Huan Yan"
git config --global user.email "2567211508@qq.com"
```

#### 2. （可选）先在 gitee 或 github 创建好远程仓库

#### 3. 在本地项目文件夹下打开终端，创建仓库

```bash
git init
```

#### 4. 添加所有文件

```bash
git add .
```

#### 5. 提交带信息

```bash
git commit -m "初始提交"
```

#### 6. （可选）关联远程仓库

```bash
git remote add origin <仓库http或ssh>
```

#### 7. （可选）推送到远程仓库并建立跟踪

```bash
git push -u origin master  # -u 等价于 --set-upstream
```

#### 8. 其他用户本地拉取仓库

```bash
git pull origin master
```

#### 9. （可选）为本地分支创建跟踪信息

```bash
git branch --set-upstream-to=origin/<远程分支名> <本地分支名>
```

#### 10. 查看分支追踪关系

```bash
git branch -vv
```

#### 11. **暂存、提交、上推三步走**

```bash
git add .
git commit -m "更新信息"
git push
```

#### 12. （推荐）创建 `.gitignore` 文件，防止二进制文件污染远程仓库

## 认证信息

### Gitee 身份验证

- **用户名**：Y_H_U_A
- **密码**：常用的

### GitHub 身份验证

- **用户名**：HuaFireX
- **密码**：常用的

## 杂项

### 大文件上传方法 - LFS

> **注意**：gitee 暂未开通免费的 LFS，需要注册企业账号

#### 相关命令

##### 清理 Git 缓存

```bash
git filter-branch --force --index-filter 'git rm -rf --cached --ignore-unmatch XXX.pkl' --prune-empty --tag-name-filter cat -- --all
```
清理 git 缓存

##### 增大本地缓存空间大小

```bash
git config --global http.postBuffer 7242880000
```
减小错误概率

##### 查询 commit 历史

```bash
git reflog
```
查询 commit 历史，复制最前面的简写哈希码

##### 回退到上传文件前

```bash
git reset <指定回退的commit的哈希码>
```
回退到 add 和 commit 大文件前，不然后面会显示有关 "ref" 的失败信息

##### 追踪指定后缀的大文件

```bash
git lfs track "*.zip"
```
使用该命令让 git 的大文件管理跟踪指定后缀的大文件

###### 查询追踪的文件

```bash
git lfs track
```

> **注意**：使用该命令后需再次 add 和 commit 才能生效

### 错误信息及解决办法

#### 无法连接 GitHub

**错误信息：**
```
fatal: unable to access 'https://github.com/nobmaste/QH_Learning_Resources.git/': 
Failed to connect to github.com port 443 after 21130 ms: Couldn't connect to server
```

**解决办法：** 直接取消代理

```bash
git config --global --unset http.proxy
git config --global --unset https.proxy
```

#### 本地代理配置

有的时候需要设置本地代理

##### 查询全局配置情况

```bash
git config --global -l
```

##### 设置本地代理

```bash
git config --global http.proxy http://127.0.0.1:<查询设置里的代理服务器端口>
```