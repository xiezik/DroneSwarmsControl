# DroneSwarmsControl
A drone swarms control alg

## 启动方法

启动仿真平台

```bash
python3 start_scenario_local.py --scenario_name=waters_strike --host=192.168.3.143 --port=2000
```
port、host、scenario_name要按照自己实际情况设置

启动控制算法

```bash
python3 src/alg_demo.py
```

## TODO
~~环境状态信息回调函数~~

~~无人机集群控制信息的发送线程~~

~~基础查打决策逻辑~~

侦查目标信息管理

舰队建模预测

打击资源管理

