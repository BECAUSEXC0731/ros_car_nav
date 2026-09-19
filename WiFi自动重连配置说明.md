# WiFi 自动重连服务配置说明

> 适用环境：树莓派 / Ubuntu 22.04 / NetworkManager
> 创建日期：2026-07-30

---

## 1. 问题描述

小车（树莓派）启动时需要自动开启 WiFi，并不断重试连接指定网络直到成功。

## 2. 最终方案

使用 **systemd 服务** 包装一个 **bash 脚本**，实现：

- 开机自动启动
- 每隔 10 秒重试连接目标 WiFi
- 已连接时每 60 秒检查一次，不反复断开重连

---

## 3. 文件结构

| 文件 | 作用 |
|---|---|
| `/usr/local/bin/auto-wifi-connect.sh` | 自动重连脚本 |
| `/etc/systemd/system/auto-wifi-connect.service` | systemd 服务单元 |

---

## 4. 操作步骤

### 4.1 创建自动重连脚本

```bash
sudo tee /usr/local/bin/auto-wifi-connect.sh << 'EOF'
#!/bin/bash

# 要连接的 WiFi 名称（改成你自己的）
SSID="doomsday"
# 重试间隔（秒）
INTERVAL=10

# 开启 WiFi 射频
nmcli radio wifi on

echo "[$(date)] WiFi 自动重连服务已启动，目标网络: $SSID"

while true; do
    # 用 iwgetid 检测当前连接的 SSID（比 nmcli 更轻量可靠）
    CURRENT_SSID=$(iwgetid -r 2>/dev/null)

    if [ "$CURRENT_SSID" = "$SSID" ]; then
        # 已连上目标网络，休眠 60 秒再检查
        sleep 60
        continue
    fi

    echo "[$(date)] 尝试连接 $SSID ... (当前: ${CURRENT_SSID:-未连接})"

    # 使用已保存的连接配置连接（比 nmcli dev wifi connect 更稳定）
    nmcli con up "$SSID" 2>/dev/null

    if [ $? -eq 0 ]; then
        echo "[$(date)] 连接 $SSID 成功"
    else
        echo "[$(date)] 连接失败，${INTERVAL} 秒后重试"
    fi

    sleep "$INTERVAL"
done
EOF
```

赋予执行权限：

```bash
sudo chmod +x /usr/local/bin/auto-wifi-connect.sh
```

### 4.2 创建 systemd 服务

```bash
sudo tee /etc/systemd/system/auto-wifi-connect.service << 'EOF'
[Unit]
Description=Auto WiFi Connect - 自动重连 WiFi 直到成功
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/local/bin/auto-wifi-connect.sh
Restart=always
RestartSec=5
User=root

[Install]
WantedBy=multi-user.target
EOF
```

### 4.3 启用并启动服务

```bash
sudo systemctl daemon-reload
sudo systemctl enable auto-wifi-connect.service
sudo systemctl start auto-wifi-connect.service
```

### 4.4 查看运行状态

```bash
sudo systemctl status auto-wifi-connect.service --no-pager -l
sudo journalctl -u auto-wifi-connect.service --no-pager -l
```

---

## 5. 注意事项

### 5.1 不要用 `nmcli dev wifi connect`

在 systemd 服务中以 root 身份运行时，`nmcli dev wifi connect` 会报错：

```
错误：Insufficient privileges
```

**正确做法**：使用 `nmcli con up "SSID"`（基于已保存的连接配置）。

### 5.2 不要用 `nmcli dev wifi` 扫描来检测 SSID

`nmcli dev wifi` 会触发 WiFi 扫描，速度慢且可能超时。

**正确做法**：使用 `iwgetid -r`，轻量且可靠。

### 5.3 已连接时不要频繁重试

脚本中已连接时使用 `sleep 60` 而非 `sleep 10`，避免日志刷屏和资源浪费。

---

## 6. 修改目标 WiFi

如果要改为连接其他 WiFi：

```bash
# 方法一：直接编辑
sudo nano /usr/local/bin/auto-wifi-connect.sh
# 修改 SSID="新的WiFi名称"

# 方法二：sed 一键替换
sudo sed -i 's/SSID=".*"/SSID="你的新WiFi名"/' /usr/local/bin/auto-wifi-connect.sh

# 重启服务
sudo systemctl restart auto-wifi-connect.service
```

---

## 7. 常用命令

```bash
# 启动服务
sudo systemctl start auto-wifi-connect.service

# 停止服务
sudo systemctl stop auto-wifi-connect.service

# 重启服务
sudo systemctl restart auto-wifi-connect.service

# 查看实时日志
sudo journalctl -u auto-wifi-connect.service -f

# 禁用开机自启
sudo systemctl disable auto-wifi-connect.service

# 彻底删除服务
sudo systemctl stop auto-wifi-connect.service
sudo systemctl disable auto-wifi-connect.service
sudo rm /etc/systemd/system/auto-wifi-connect.service
sudo rm /usr/local/bin/auto-wifi-connect.sh
sudo systemctl daemon-reload
```

---

## 8. 排错参考

| 现象 | 原因 | 解决 |
|---|---|---|
| 服务不停地每 10 秒重连 | 检测已连接的方法不对 | 改用 `iwgetid -r` |
| `nmcli dev wifi connect` 失败 | root 权限不足 | 改用 `nmcli con up` |
| 服务启动后不工作 | 无线网卡未就绪 | 检查 `After=network-online.target` |
| WiFi 频繁断开 | 省电模式导致 | 执行 `sudo iw dev wlan0 set power_save off` |
