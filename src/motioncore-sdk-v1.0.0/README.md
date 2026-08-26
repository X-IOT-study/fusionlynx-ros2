# MotionCore SDK v1.0.0

This directory is the release bundle for the MotionCore SDK after packaging.
It is meant to be unpacked and consumed by downstream projects, not edited as
source.

MotionCore ships a shared C core, a thin C++ wrapper, and example projects so
you can integrate the protocol from applications or tooling.

## Installation

### Use the packaged SDK in your project

Unpack `motioncore-sdk-v1.0.0/` into a location of your choice, then point
`CMAKE_PREFIX_PATH` at the SDK root when configuring your application:

```bash
cmake -S your-app -B build -DCMAKE_PREFIX_PATH=/path/to/motioncore-sdk-v1.0.0
cmake --build build
```

The SDK root `CMakeLists.txt` uses the staged `cmake/` directory so the
packaged `MotionCoreConfig.cmake` is found automatically.

### Build the SDK examples from the release bundle

```bash
cmake -S . -B build -DMOTIONCORE_BUILD_EXAMPLES=ON
cmake --build build
```

This compiles the bundled example sources against the packaged SDK.
The staged SDK root now includes a bundle-specific `CMakeLists.txt`, so the
root can be configured directly after unpacking.

## Quick start

1. Unpack `motioncore-sdk-v1.0.0/` into your deployment or workspace directory.
2. Configure your application with `CMAKE_PREFIX_PATH=/path/to/motioncore-sdk-v1.0.0`.
3. Build the bundled examples from the SDK root if you want to verify the package.
4. Review the shipped docs under `share/motioncore/docs/`.

## SDK layout

- `CMakeLists.txt` — package-root entrypoint for downstream configure/build
- `cmake/` — `MotionCoreConfig.cmake` and exported targets
- `include/` — public C and C++ headers
- `lib/` — shared library and SONAME links
- `examples/` — C, C++, and Python example sources and example CMake files
- `share/motioncore/docs/` — shipped API references and runtime notes
- `README.md` and `README_zh.md` — bundle usage notes
- `LICENSE` — project license

## CMake usage

```cmake
find_package(MotionCore CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE MotionCore::motioncore)
```

## Examples

The bundle keeps the example source code and the small example CMake files so
those examples can compile against the packaged SDK without rebuilding the
whole repository.

## Documentation

- The bundle ships `share/motioncore/docs/` and excludes `docs/design/`.
- C API: `share/motioncore/docs/release/api-reference-c.zh.md` / `share/motioncore/docs/release/api-reference-c.md`
- C++ API: `share/motioncore/docs/release/api-reference-cpp.zh.md` / `share/motioncore/docs/release/api-reference-cpp.md`
- Python API: `share/motioncore/docs/release/api-reference-python.zh.md` / `share/motioncore/docs/release/api-reference-python.md`
- Teleop notes: `share/motioncore/docs/motion-control-and-teleop.zh.md`
- Timestamp notes: `share/motioncore/docs/timestamp-and-ros2-usage.md`

## Compatibility notes

- The wire protocol is fixed and must remain compatible with the firmware implementation.
- The C core is the source of truth for protocol behavior.
- Wrapper layers should remain thin and should not reimplement frame parsing.
