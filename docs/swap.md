
# ubuntu扩大运行内存, 防止编译卡死

## Swap 扩大内存

[扩大内存](https://blog.csdn.net/qq_35102059/article/details/134083996?fromshare=blogdetail&sharetype=blogdetail&sharerId=134083996&sharerefer=PC&sharesource=2301_80216316&sharefrom=from_link)


## Swap 每次开机时自动启用

### 第一步：先确认当前 Swap 已生效

```sh
swapon --show
free -h
```

swapon --show 应该能看到 /swapfile 这一行，free -h 的 Swap 一栏应显示约 38Gi。

### 第二步：写入 /etc/fstab 实现永久启用
先检查是否已有旧的 swap 条目（避免重复或冲突）：

```sh
grep -n swap /etc/fstab
```
如果看到类似 /swapfile none swap sw 0 0 的旧行（且指向同一个 /swapfile），说明已经有了，跳过写入即可。如果指向的是别的文件（比如旧的 /swap.img），建议先注释掉那一行（行首加 #）。

如果没有，则追加新条目：

```sh
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### 第三步：验证 fstab 配置正确（重要）
不要直接重启，先用下面命令让 systemd 重新读取 fstab 并挂载所有条目，检查是否有报错：

```sh
sudo systemctl daemon-reload
sudo mount -a
```
如果没有任何输出，说明 fstab 语法正确。

如果报错（比如 parse error 或 unknown filesystem type 'swap' 之类），立即编辑 /etc/fstab 修正，否则可能导致下次开机进入 emergency mode。

### 第四步：模拟重启验证（可选但推荐）
```sh
sudo swapoff /swapfile
sudo swapon -a          # 让系统按 fstab 自动挂载所有 swap
swapon --show           # 确认 /swapfile 又回来了
```
如果能正常重新挂载，说明 fstab 配置没问题，下次开机也会自动生效。