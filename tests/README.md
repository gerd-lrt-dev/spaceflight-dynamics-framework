# SDF Testing

SDF uses GoogleTest for C++ test cases and CTest as the common test runner.

## Test categories

Automated tests are separated by purpose:

- `UT-*` — unit tests for isolated software components
- `IT-*` — integration tests for interactions between multiple components
- `VER-*` — mathematical or physical verification cases against a defined reference solution

The directory structure reflects this separation:

```text
tests/
├── unit/
├── integration/
└── verification/
```

A unit, integration, or verification test may also serve as a regression test once it remains permanently in the automated test suite.

## Configure a frontend-independent test build

From the repository root:

```bash
cmake -S . -B build-tests \
  -DBUILD_TESTING=ON \
  -DBUILD_FRONTEND=OFF
```

This configures the backend and automated tests without requiring the Qt frontend.

## Build the tests

```bash
cmake --build build-tests
```

## Run all registered tests

```bash
ctest --test-dir build-tests --output-on-failure
```

A successful run returns a zero exit status and reports all registered tests as passed. A failing GoogleTest assertion is propagated through CTest as a failed test and a non-zero result.

## Adding tests

### Unit tests

Add isolated component tests under `tests/unit/` and register their source files in the corresponding CMake target.

### Integration tests

Add tests that exercise the interaction of multiple SDF components under `tests/integration/`.

### Verification cases

Add analytically or physically defined reference cases under `tests/verification/`. Verification cases should document:

- initial conditions
- reference solution
- simulation timestep and duration
- numerical tolerances
- diagnostically useful failure output

## Naming convention

Use descriptive GoogleTest suite and test names in code. In issue tracking and verification documentation, use the following identifiers:

- `UT-...`
- `IT-...`
- `VER-...`

The first test implementation in T03 uses `EulerIntegrator` as a proof of the testing infrastructure. It is not intended to constitute comprehensive numerical or physical verification of the integrator.
