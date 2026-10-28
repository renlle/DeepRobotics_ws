0. 写代码增加中文注释
1. rviz config 保存在 ./tmp/deeprobotics_slam_${USER}.rviz 下
2. 算法层 ROS_DOMAIN_ID=42， 机器狗底盘层：export ROS_DOMAIN_ID=0  （ros2 topic list）
Domain 0 仿真/底盘传感器
  └── /clock
  └── /livox/lidar
  └── /livox/imu
  └── /left_camera/image_raw
        ↓
Domain 42 算法域
  └── FAST-LIVO
  └── 2D定位
  └── Nav2
  └── RViz
        ↓
  /NAV_CMD
        ↓
Domain 0 底盘/仿真底盘

- Domain 42 ：运行 FAST-LIVO、建图、定位、Nav2、RViz、Bag 回放等算法节点。
- Domain 0 ：运行 M20 机器狗底层控制节点。

- 跨域桥接 ： nav_cmd_domain_bridge 订阅 Domain 42 的 /NAV_CMD ，转发到 Domain 0，相关实现见 nav_cmd_domain_bridge.py 。
- 算法域配置位于 device_parameters.yaml 。
- 启动脚本在 start_app.sh


# 其他
1. 总控调度开发尽量不修改算法原有逻辑；可以通过新增文件或独立 ROS 2 package 完成时，优先新增实现。
2. debug项， 增加文档到 debug_md/ 目录下
3. 有问题向我提问，需要我来启动仿真复现问题，向我提问 （你无法启动仿真）
4. 处理代码中路径保存不存在时，保留原路径优先，os.path.exists()不存在时才回退到实际无 src 的目录结构。
5. 不要安装torch，  不要大幅动代码， 实际Orin下的 M20机器狗实机全流程都可跑通的，代码是没有大问题 --除了src我导致的路径问题
6. pip install -i                https://pypi.tuna.tsinghua.edu.cn/simple   xxx
7. 有代码更新，提醒我 ’git add‘(你也可以自动帮我执行一下，如果有权限), 有重大代码、版本变动提醒我 ’git commit‘

# Git 工作流（2026-10-23 起）

1. 以远程 Orin 为代码和调试的主工作区：SSH Host `10.0.40.127-Orin`，目录 `/home/ubuntu/Desktop/ROS2_szs/DeepRobotics_ws`。变更和 debug 优先在远程完成，覆盖此前以 Mac 为 Git 管理源的约定。
2. 完成变更及必要验证后，在 Orin 执行 `git add`、`git commit`，再 `git push` 到当前分支对应的远端；用户指定提交信息时原样使用。不要强制推送。
3. 远程 push 成功后，Mac 项目 `DeepRobotics_ws` 切到相同分支并执行 `git pull --ff-only`。Mac 用于拉取同步、查看代码和管理记录，避免两端同时修改相同文件。
4. 日常同步只核对两端分支、`git rev-parse HEAD` 和 `git status --short`。相同 commit hash 且工作区干净才表示已提交源码一致；不要每次全目录 diff 或逐文件计算校验值。仅在 hash 不同、存在未提交变更或出现同步错误时检查具体差异。
5. pull 前如有本地未提交修改，先保护并处理，不能覆盖用户改动；出现分叉时先检查原因，不执行强制 reset。使用 Git 同步版本，不通过双向 rsync 或复制 `.git` 覆盖历史。
6. 地图、录包、编译产物和临时数据仍按 `.gitignore` 排除，尤其是 `data/*`、`data_bak/*`、`data_szs/`、`data_szs*/`。不为源码同步复制这些大目录。

# 导航回归录包

1. 把 `log/` 下能复现故障、验证边界行为的高价值录包归档到 `log/导航算法回归bags/<原录包目录名>/`。保留各域原始文件和 metadata，不改写原始消息。
2. 每个录包目录必须有 `简单说明.md`，写明日期、场景、故障现象、关键时间段、对应 WWH/修复、回归关注点及数据缺失限制。索引维护在 `debug_md/Gpt6_Astra/导航回归录包索引.md`，索引随 Git 同步，原包不进入 Git、不复制到 Mac。
3. `log/导航算法回归bags/` 长期保留，任何按日期清理日志、磁盘整理或一键编译均不得删除该目录；删除其中资料须用户明确指定。
4. 每次导航算法优化，按本次改动涉及的场景跑一遍归档包的离线分析及生产逻辑回归，记录包名、分析窗口、测试命令、结果与未覆盖部分。缺少原始点云/控制状态的包不能宣称已完成 SLAM 或控制闭环回放。
5. 默认使用数据库只读解析和不创建 ROS 节点的测试。不得直接把包中的 NAV_CMD、控制模式等消息回放到实机 Domain 0/42；需要实机或仿真复测时由用户启动。
