. It is a duplicated workspace plus missing Pinocchio runtime libraries required by libfranka 0.20.4. 
This also explains why fake simulation works while the real hardware path fails.


ros2 pkg prefix rmw_cyclonedds_cpp
echo "$RMW_IMPLEMENTATION"

```
find /home/yujietang/franka_ros2_ws_jazzy/install/libfranka \
  \( -name 'libfranka.so*' -o -name 'FrankaConfig.cmake' \) \
  -print
```

## vertify the install of vital ros2 package
```bash
ros2 pkg prefix franka_fr3_moveit_config
ros2 pkg prefix franka_hardware
ros2 pkg prefix franka_bringup
```


# BUg 2 :

mismatch of ubuntu with official file : like in libfranka : coal cp /cr

the official libfranka have been updated before:

## solution:
```bash
build new environment 
source new environment
run our nmpc by the new environment


## Workflow:

下一步是在 Ubuntu 主机上建立一个干净、版本匹配的 Franka 底层工作空间。不要把 Franka 驱动复制到 `fr3_nmpc_ws`。

你的结构最终应该是：

```text
ROS 2 Jazzy
   ↓
franka_ros2_ws_jazzy（官方驱动）
   ↓
fr3_nmpc_ws（你的 NMPC）
```

## 第一步：退出当前临时 Shell

如果终端提示符是：

```text
bash-5.2$
```

运行：

```bash
exit
```

## 第二步：重新创建新工作空间

逐行执行，不要复制 Markdown 链接格式：

```bash
mkdir -p /home/yujietang/franka_ros2_ws_jazzy
cd /home/yujietang/franka_ros2_ws_jazzy
pwd
```

`pwd` 应显示：

```text
/home/yujietang/franka_ros2_ws_jazzy
```

检查目录是否为空：

```bash
ls -la
```

然后克隆官方 Jazzy 分支：

```bash
git clone --branch jazzy https://github.com/frankarobotics/franka_ros2.git src
```

成功时会看到：

```text
Cloning into 'src'...
```

## 第三步：确认新版本解决了 Coal 问题

```bash
git -C /home/yujietang/franka_ros2_ws_jazzy/src \
  describe --tags --always
```

应该是 `v3.4.0` 或更高版本。

检查碰撞库代码：

```bash
grep -nE 'hpp::fcl|coal::' \
  /home/yujietang/franka_ros2_ws_jazzy/src/franka_selfcollision/src/self_collision_checker.cpp
```

正确结果应该包含：

```cpp
coal::CollisionResult
```

不能再出现：

```cpp
hpp::fcl::CollisionResult
```

## 第四步：下载匹配的依赖源码

```bash
cd /home/yujietang/franka_ros2_ws_jazzy

vcs import src \
  < src/dependency.repos \
  --recursive \
  --skip-existing
```

完成后检查关键包：

```bash
source /opt/ros/jazzy/setup.bash

colcon list | grep -E \
'libfranka|franka_selfcollision|franka_hardware|franka_bringup|franka_fr3_moveit_config'
```

五个包都应该出现。

## 第五步：安装系统依赖

```bash
cd /home/yujietang/franka_ros2_ws_jazzy
source /opt/ros/jazzy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  -r -y \
  --skip-keys=zed_wrapper
```

如果最后只是说所有可解析依赖安装成功，就可以继续。

## 第六步：编译

```bash
cd /home/yujietang/franka_ros2_ws_jazzy
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
  --event-handlers console_direct+ \
  2>&1 | tee full_build.log
```

编译过程中不要运行其他 `source ~/franka_ros2_ws...` 命令。

成功摘要应类似：

```text
Summary: XX packages finished
```

不能出现：

```text
Failed
Aborted
packages not processed
```

如果失败，运行：

```bash
grep -nE \
'Failed <<<|Aborted <<<|CMake Error|error:|undefined reference|not found' \
/home/yujietang/franka_ros2_ws_jazzy/full_build.log | head -100
```

## 第七步：验证 Franka 包

只有完整构建成功后，打开一个新终端：

```bash
source /opt/ros/jazzy/setup.bash
source /home/yujietang/franka_ros2_ws_jazzy/install/setup.bash
```

验证：

```bash
ros2 pkg prefix franka_fr3_moveit_config
ros2 pkg prefix franka_hardware
ros2 pkg prefix franka_bringup
```

三条命令都应该返回：

```text
/home/yujietang/franka_ros2_ws_jazzy/install/...
```

验证 `libfranka`：

```bash
find /home/yujietang/franka_ros2_ws_jazzy/install/libfranka \
  \( -name 'libfranka.so*' -o -name 'FrankaConfig.cmake' \) \
  -print
```

## 第八步：只测试 fake hardware

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  load_gripper:=true \
  use_fake_hardware:=true
```

这一步成功后，才进入网络和真机测试。

暂时不要：

- 删除旧的 `/home/yujietang/franka_ros2_ws`；
- 复制旧配置到新工作空间；
- source 旧工作空间；
- 启动真实机器人；
- 把 Franka 驱动放进 `fr3_nmpc_ws`。

你现在先完成第二、第三步，确认新源码使用 `coal::CollisionResult`，然后再继续导入依赖和编译。









```
