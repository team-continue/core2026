# core_test

Shared GTest package for `core2026`.

## Add a new test
1. Create a new `.cpp` in `test/`.
2. Register it in `CMakeLists.txt` with `core_add_gtest(<target> test/<file>.cpp)`.
3. Keep one test file per target for fast CI.

Example:

```
core_add_gtest(test_my_feature test/test_my_feature.cpp)
```

## Run tests locally

```bash
colcon test --packages-select core_test
colcon test-result --verbose
```

## CI usage

```bash
colcon test --event-handlers console_direct+ --packages-select core_test
colcon test-result --verbose
```
