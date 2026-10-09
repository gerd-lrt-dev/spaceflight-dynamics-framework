# SDF Test Workflow

This document summarizes the standard local workflow for building and executing the SDF test suite.

## 1. Configure the test build

From the repository root:

```bash
cmake -S . -B build-tests -DBUILD_TESTING=ON -DBUILD_FRONTEND=OFF
```

This creates a dedicated test build directory and disables the Qt frontend.

## 2. Build the tests

```bash
cmake --build build-tests
```

If only one test source file was changed, this command normally rebuilds only the affected targets.

## 3. Run the complete test suite

```bash
ctest --test-dir build-tests --output-on-failure
```

`--output-on-failure` prints the GoogleTest output when a test fails.

## 4. Run a specific verification test

Example:

```bash
ctest --test-dir build-tests -R VER_FRM_002 --output-on-failure
```

The `-R` argument filters tests by name using a regular expression.

Other examples:

```bash
ctest --test-dir build-tests -R VER_ROT_001 --output-on-failure
ctest --test-dir build-tests -R VER_TRA_001 --output-on-failure
```

## 5. Typical workflow while developing a test

After editing a test file:

```bash
cmake --build build-tests
ctest --test-dir build-tests -R VER_FRM_002 --output-on-failure
```

Once the individual test passes, run the complete regression suite:

```bash
ctest --test-dir build-tests --output-on-failure
```

The final result should show:

```text
100% tests passed, 0 tests failed
```

## 6. When CMake configuration changes

If a new test file was added to a `CMakeLists.txt`, re-run the configure step before building:

```bash
cmake -S . -B build-tests -DBUILD_TESTING=ON -DBUILD_FRONTEND=OFF
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

## 7. Clean rebuild

If the build directory becomes inconsistent or stale:

```bash
rm -rf build-tests
cmake -S . -B build-tests -DBUILD_TESTING=ON -DBUILD_FRONTEND=OFF
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

Use this only when necessary. A normal incremental build is usually sufficient.

## Standard short workflow

For everyday test development:

```bash
cmake --build build-tests
ctest --test-dir build-tests -R TEST_NAME --output-on-failure
ctest --test-dir build-tests --output-on-failure
```

If `build-tests` does not exist yet:

```bash
cmake -S . -B build-tests -DBUILD_TESTING=ON -DBUILD_FRONTEND=OFF
```
