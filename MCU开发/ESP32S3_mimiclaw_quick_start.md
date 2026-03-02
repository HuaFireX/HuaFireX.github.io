# ESP32-S3 部署 MimicLaw 完整教程

本教程帮助你在 ESP32-S3 开发板上部署 MimicLaw 项目，接入 DeepSeek API 实现自然语言控制硬件。

---

## 1. MimicLaw 是什么？

MimicLaw 是一个运行在 ESP32 上的 **LLM 客户端框架**，可以：

- 连接 WiFi
- 调用大模型 API（OpenAI / DeepSeek 等）
- 解析返回的 JSON
- 本地执行逻辑（如控制 GPIO / RGB 灯）

**一句话概括**：
```
ESP32-S3 + WiFi + LLM = 可被自然语言驱动的嵌入式控制器
```

---

## 2. 准备工作

### 硬件

| 物品 | 说明 |
|------|------|
| ESP32-S3 核心板 | 推荐带 USB 直连版本（如 ESP32-S3-DevKitC） |
| USB 数据线 | 必须是**数据线**（纯充电线无法烧录） |

### 软件环境

| 项目 | 版本 |
|------|------|
| 操作系统 | Ubuntu（虚拟机建议分配 40GB+） |
| ESP-IDF | v5.5.2 |
| Python | 3.8+ |
| Git | - |

### DeepSeek API Key

1. 访问 https://platform.deepseek.com
2. 注册/登录
3. 进入「API Keys」页面创建 Key
4. 复制保存（只显示一次！）

> DeepSeek 注册后通常送免费额度，足够测试使用。

---

## 3. 安装 ESP-IDF

### 3.1 安装系统依赖

```bash
sudo apt update
sudo apt install -y git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

### 3.2 运行官方安装脚本

```bash
./scripts/setup_idf_ubuntu.sh
./scripts/build_ubuntu.sh
```

安装完成后，ESP-IDF 位于：
```
~/.espressif/esp-idf-v5.5.2
```

### 3.3 加载 ESP-IDF 环境

**每次打开新终端都需要执行：**

```bash
. $HOME/.espressif/esp-idf-v5.5.2/export.sh
```

> 注意：用 `$HOME` 不要用具体用户路径

验证是否成功：

```bash
idf.py --version
# 输出：ESP-IDF v5.5.2
```

### 3.4 设置自动加载（可选）

```bash
echo '. $HOME/.espressif/esp-idf-v5.5.2/export.sh' >> ~/.bashrc
source ~/.bashrc
```

---

## 4. 验证 DeepSeek API

在改代码之前，先确认 API 可用：

```bash
curl https://api.deepseek.com/chat/completions \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer 你的DeepSeek_API_Key" \
  -d '{
        "model": "deepseek-chat",
        "messages": [
          {"role": "user", "content": "Hello"}
        ],
        "stream": false
      }'
```

成功返回 JSON 说明正常。

---

## 5. 下载 MimicLaw 源码

```bash
cd ~
git clone https://github.com/memovai/mimiclaw.git
cd mimiclaw
```

> 如果 GitHub 访问超时，可以使用镜像：
> ```bash
> git clone https://ghproxy.com/https://github.com/memovai/mimiclaw.git
> ```

---

## 6. 配置 mimi_secrets.h

### 6.1 复制示例文件

```bash
cp mimi_secrets.h.example mimi_secrets.h
```

### 6.2 编辑配置

```bash
nano mimi_secrets.h
```

修改内容：

```c
#pragma once

#define MIMI_SECRET_WIFI_SSID       "你的WiFi名称"
#define MIMI_SECRET_WIFI_PASS       "你的WiFi密码"

/* DeepSeek API 配置 */
#define MIMI_SECRET_API_KEY         "sk-你的DeepSeek_API_Key"
#define MIMI_SECRET_MODEL           "deepseek-chat"
#define MIMI_SECRET_MODEL_PROVIDER  "openai"
```

按 `Ctrl + O` 保存，`Ctrl + X` 退出。

> 注意：
> - WiFi 必须是 **2.4GHz**（ESP32 不支持 5GHz）
> - Provider 填 `"openai"`，因为 DeepSeek 兼容 OpenAI 协议

> 安全提醒：不要把 `mimi_secrets.h` 提交到 Git

---

## 7. 修改源码支持 DeepSeek

### 7.1 为什么要改？

MimicLaw 默认连接 `api.openai.com`，DeepSeek 域名不同：

| 项目 | OpenAI | DeepSeek |
|------|--------|----------|
| Host | api.openai.com | api.deepseek.com |
| Path | /v1/chat/completions | /chat/completions |

### 7.2 修改 llm_proxy.c

```bash
nano main/llm/llm_proxy.c
```

修改以下函数：

```c
static bool provider_is_openai(void)
{
    return strcmp(s_provider, "openai") == 0 || strcmp(s_provider, "deepseek") == 0;
}

static const char *llm_api_url(void)
{
    if (strcmp(s_provider, "deepseek") == 0) return MIMI_DEEPSEEK_API_URL;
    if (strcmp(s_provider, "openai") == 0) return MIMI_OPENAI_API_URL;
    return MIMI_LLM_API_URL;
}

static const char *llm_api_host(void)
{
    if (strcmp(s_provider, "deepseek") == 0) return "api.deepseek.com";
    if (strcmp(s_provider, "openai") == 0) return "api.openai.com";
    return "api.anthropic.com";
}

static const char *llm_api_path(void)
{
    if (provider_is_openai()) return "/v1/chat/completions";
    return "/v1/messages";
}
```

保存退出。

---

## 8. 编译与烧录

### 8.1 加载 ESP-IDF 环境

```bash
. $HOME/.espressif/esp-idf-v5.5.2/export.sh
```

### 8.2 设置目标芯片

```bash
cd ~/mimiclaw
idf.py set-target esp32s3
```

### 8.3 清理缓存

```bash
idf.py fullclean
```

### 8.4 编译

```bash
idf.py build
```

首次编译需要 **5~15 分钟**。

编译成功会看到：
```
Project build complete. To flash, run:
 idf.py flash
```

### 8.5 烧录

确认设备：

```bash
ls /dev/ttyACM*
# 或
ls /dev/ttyUSB*
```

烧录并打开串口监视器：

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

---

## 9. 串口权限问题

如果报错 `Permission denied: '/dev/ttyACM0'`：

```bash
sudo usermod -aG dialout $USER
```

**必须重启**电脑后生效。

---

## 10. 成功运行的表现

串口监视器中应该看到：

```
✅ WiFi connected, IP: 192.168.x.x
✅ DNS resolved: api.deepseek.com
✅ TLS handshake OK
✅ HTTP POST → 200 OK
✅ Response parsed
✅ Assistant: Hello!
```

看到这些说明部署成功！

退出串口监视器：按 `Ctrl + ]`

---

## 11. 常见问题

| 问题 | 解决方案 |
|------|----------|
| `idf.py: command not found` | 执行 `export.sh` |
| `Permission denied: /dev/ttyACM0` | `sudo usermod -aG dialout $USER` 后重启 |
| WiFi 连不上 | 确认是 2.4GHz 网络 |
| API 返回 401 | 检查 API Key 是否正确 |
| 编译报找不到头文件 | 确认 `mimi_secrets.h` 已复制 |
| 烧录连接失败 | 按住 BOOT → 按 RST → 松开 RST → 开始烧录 |
| 终端无法输入 CLI（不出现 `mimi>`） | `idf.py monitor` 使用 USB Serial/JTAG，但 CLI 默认运行在 UART0。解决方法：修改 `sdkconfig`，将 `CONFIG_ESP_CONSOLE_UART_DEFAULT=y` 改为 `# CONFIG_ESP_CONSOLE_UART_DEFAULT is not set`，然后添加 `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`，最后执行 `idf.py fullclean` 后重新编译烧录 |

---

## 12. 下一步可以做什么？

### 示例：自然语言控制 RGB 灯

用户说："把灯调成暖色"
LLM 输出：`{"r": 255, "g": 180, "b": 100}`
ESP32 解析 JSON → 控制 PWM → 灯光变化

### 更多方向

- 语义物联网网关
- LLM 驱动 GPIO
- 边缘 AI 决策层

---

## 快速流程总结

```
1. 装环境 → ESP-IDF v5.5.2
2. 拿 Key → DeepSeek API
3. 改代码 → Host + Path
4. 编译烧录 → build + flash
```

> 原版项目地址：https://github.com/memovai/mimiclaw
> 
> 本教程 fork 版本：https://github.com/HuaFireX/mimiclaw
