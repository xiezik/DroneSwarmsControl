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

无人机间距控制（组间和组内）

无人机分组

~~舰队建模预测~~

打击资源管理

~~对集群的控制进行包装~~

任务执行和信息可视化（已实现无人机位置、舰船位置）（未实现：无人机目标位置、舰船航向预测、舰队中心点、舰队状态）
