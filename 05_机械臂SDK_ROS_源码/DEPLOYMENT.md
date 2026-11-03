# 部署目标

目标上位机：

- IP：`192.168.8.100`
- 主机名：`nvidia-desktop`
- 架构：ARM64 / aarch64
- Ubuntu：22.04.4 LTS
- ROS 2：Humble

当前从开发电脑检查结果：

- ICMP Ping：通过
- TCP 22：未开放

部署前需在目标机确认 SSH 服务：

```bash
sudo systemctl status ssh
sudo systemctl enable --now ssh
ip -br address
```

如果使用非标准 SSH 端口，需要记录用户名和端口。上位机地址与贞实控制器地址不是同一个参数：

```text
上位机：192.168.8.100
贞实控制器：当前旧配置为 192.168.11.11
```

目标机需要具有到控制器网段的独立网卡或正确路由。

