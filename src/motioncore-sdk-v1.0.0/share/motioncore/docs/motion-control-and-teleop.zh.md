# MotionCore 运动控制与按键遥控说明

本文按当前代码实现编写，结论以源码为准。

## 先说结论

- `W/S/A/D/Q/E` 不是“持续加速键”，而是“目标速度状态键”。
- BodyMonitor 和 SDK demo 都是按下时更新一次目标速度，松开时发停止。
- 线速度和角速度在 BodyMonitor 里先按百分比映射，再变成固件速度单位。
- 固件的 `FUNC_MOTION` 直接走 `Motion_Ctrl(Vx, Vy, Vz, adjust)`，没有再做一层按键加速。
- 你看到“刚按下去很快，后面变慢”，通常是机械/闭环收敛的瞬态，不是协议层把速度降了。

## BodyMonitor 实际怎么控

代码在 `MotorControlSystem/src/ui/pages/motion/MotionControlPage.cpp`。

### 按键模型

- `keyPressEvent()` 和 `keyReleaseEvent()` 都忽略 `event->isAutoRepeat()`
- 也就是说，长按不会不断触发新的“加速命令”
- 真正起作用的是当前按键状态：
  - `m_keyForwardPressed`
  - `m_keyBackwardPressed`
  - `m_keyLeftPressed`
  - `m_keyRightPressed`
  - `m_keyRotateLeftPressed`
  - `m_keyRotateRightPressed`

`updateMotionState()` 会把这些状态合并成一个速度向量：

- `W` -> `vx > 0`
- `S` -> `vx < 0`
- `A` -> `vy > 0`
- `D` -> `vy < 0`
- `Q` -> `vz > 0`
- `E` -> `vz < 0`

### 速度映射

BodyMonitor 里的滑块是百分比，不是固件原始单位。

- 线速度：`0-100% -> 0-700 mm/s`
- 角速度：`0-100% -> 0-3200`，固件里按 `0.001 rad/s` 使用

默认值是 `50%`，所以默认大约是：

- 线速度 `350 mm/s`
- 角速度 `1.6 rad/s`

### 发送的帧

`sendMotionCommand()` 最终调用 `ProtocolParser::createMotionCommand()`，生成 `FUNC_MOTION (0x12)`：

```text
[param][vx_l][vx_h][vy_l][vy_h][vz_l][vz_h]
```

当前约定里：

- `param` 的 `0x80` 位表示 yaw adjust
- 纯旋转时会把 yaw adjust 关掉

### 松开键会怎样

当所有运动键都释放时，`updateMotionState()` 会调用 `stopMotion()`，最终发 `0, 0, 0`，固件收到后执行 `Motion_Stop(STOP_BRAKE)`。

## 固件侧实际怎么执行

代码在 `APP/Src/protocol.c` 和 `APP/Src/app_motion.c`。

- `FUNC_MOTION` 解析后，如果 `Vx/Vy/Vz` 全是 0，就停车
- 否则直接调用 `Motion_Ctrl(Vx, Vy, Vz, adjust)`
- `Motion_Ctrl()` 只做限幅和麦克纳姆轮分解
- `Motion_Handle()` 每 10 ms 跑一次，用当前 PID 结果更新 PWM

这里没有“按住键自动爬到更高速度”的逻辑。

## SDK 里怎么控制

### 直接发运动指令

Python:

```python
from motioncore import Client, MotionCommand, TransportSerial

client = Client()
transport = TransportSerial(port="/dev/ttyUSB0", baudrate=115200)
transport.open()
client.set_transport(transport)

client.send_motion(MotionCommand(vx=300, vy=0, vz=0, yaw_adjust=0x80))
client.send_motion(MotionCommand(vx=0, vy=0, vz=0, yaw_adjust=0x00))
```

`MotionCommand` 里放的是**固件原始速度值**，不是百分比。

### 现成键盘 demo

SDK 里已经有按键遥控 demo：

```bash
python3 demo-test/keyboard_teleop.py --port /dev/ttyUSB0
```

映射是：

- `W` 前
- `S` 后
- `A` 左
- `D` 右
- `Q` 左转
- `E` 右转
- 松开任意键停止
- `Esc` 退出

这个 demo 用 Tk 窗口，不用终端输入，因为终端拿不到稳定的按键释放事件。
它的默认速度值已对齐 BodyMonitor 的 50% 滑块：线速度 350，角速度 1600。
非纯旋转时会自动打开 yaw adjust，和上位机默认行为一致。
Tk 对长按键的释放事件处理和 Qt 不一样，demo 里加了一个很短的释放防抖，专门避免单键长按时被重复释放事件打断。

## 为什么你会感觉“先快后慢”

代码层面没有把同一次按键拆成“先高速、后低速”两段。

更常见的原因是：

1. 电机从静止到目标速度有加速过程
2. PID 收敛后进入稳定速度
3. 目标速度本身只有滑块设置的百分比
4. 电池电压、负载、地面摩擦会让稳态速度看起来更低

所以如果想要更“猛”的稳态速度，不是改长按逻辑，而是提高 BodyMonitor 的速度滑块，或者直接在 SDK 里提高 `vx/vy/vz` 的目标值。
如果你拿旧版 demo 对比上位机，最主要的差异就是默认速度更低、yaw adjust 没有自动打开。

## ROS 2 里怎么接

推荐把 `geometry_msgs/msg/Twist` 映射到 `MotionCommand`：

- `linear.x -> vx`
- `linear.y -> vy`
- `angular.z -> vz`

策略上建议：

- 按下键时发一次目标速度
- 松开键时发零速度
- 如果桥接层有看门狗，再按固定频率重发同一个目标值

## 最后给一个对照表

| 键 | 目标字段 | 含义 |
| --- | --- | --- |
| W | `vx > 0` | 前进 |
| S | `vx < 0` | 后退 |
| A | `vy > 0` | 左移 |
| D | `vy < 0` | 右移 |
| Q | `vz > 0` | 左转 |
| E | `vz < 0` | 右转 |
| 松开 | `0,0,0` | 停止 |

如果你要完全复现 BodyMonitor 的行为，就保持“按下更新一次、松开发停止”的模型，不要把键盘长按当成连续加速器。
