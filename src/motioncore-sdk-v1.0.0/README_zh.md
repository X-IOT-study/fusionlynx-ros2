# MotionCore SDK v1.0.0

这个目录是 MotionCore SDK 打包完成后的发布包。它面向下游项目消费，
不是源码开发目录。

MotionCore 提供共享 C 核心、轻量 C++ 包装层以及示例工程，方便你在应用程序
和工具中接入协议。

## 安装

### 在你的项目中使用打包后的 SDK

将 `motioncore-sdk-v1.0.0/` 解压到任意位置，然后在配置应用时把
`CMAKE_PREFIX_PATH` 指向 SDK 根目录：

```bash
cmake -S your-app -B build -DCMAKE_PREFIX_PATH=/path/to/motioncore-sdk-v1.0.0
cmake --build build
```

SDK 根目录的 `CMakeLists.txt` 会使用打包后的 `cmake/` 目录，因此能够
自动找到 `MotionCoreConfig.cmake`。

### 从发布包编译 SDK 示例

```bash
cmake -S . -B build -DMOTIONCORE_BUILD_EXAMPLES=ON
cmake --build build
```

这样可以将 SDK 自带的示例源码直接与打包后的 SDK 一起编译。
打包后的 SDK 根目录包含一个专用的 `CMakeLists.txt`，因此解压后即可直接
在根目录配置构建。

## 快速使用

1. 将 `motioncore-sdk-v1.0.0/` 解压到部署目录或工作目录。
2. 配置你的应用时，将 `CMAKE_PREFIX_PATH` 指向该 SDK 根目录。
3. 如需验证 SDK 包，可在 SDK 根目录启用示例编译。
4. 查看随包提供的 `share/motioncore/docs/` 文档。

## SDK 目录结构

- `CMakeLists.txt`：下游配置/构建入口
- `cmake/`：`MotionCoreConfig.cmake` 和导出目标文件
- `include/`：公开 C / C++ 头文件
- `lib/`：共享库及符号链接
- `examples/`：C、C++、Python 示例源码以及示例 CMake 文件
- `share/motioncore/docs/`：随包提供的 API 文档和运行说明
- `README.md` 和 `README_zh.md`：发布包说明
- `LICENSE`：项目许可证

## CMake 用法

```cmake
find_package(MotionCore CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE MotionCore::motioncore)
```

## 示例

SDK 包保留了示例源码和对应的简洁 CMake 文件，因此这些示例可以直接基于
打包后的 SDK 编译，而不必重新构建整个仓库。

## 兼容性说明

- 线协议是固定的，必须与固件实现保持兼容。
- C 核心是协议行为的唯一权威实现。
- 包装层应保持薄封装，不应重复实现帧解析逻辑。
