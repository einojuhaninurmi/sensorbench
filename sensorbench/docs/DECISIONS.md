# Initial engineering decisions

## Offline baseline

The simulator and local HTTP server use Python's standard library and SQLite. Windows report exports also use the IANA time zone data from `tzdata`. This keeps the demo reproducible offline and the server accessible on loopback.

## Raw input and effective configuration

The submitted JSON is hashed and stored independently of effective test defaults. Tests store the effective configuration as separate evidence. This preserves identical-request replay even after defaults change. A regression test changes a default and verifies that the existing run and original input are retained.

## Transactional traceability

A session, its rows and its checks must appear together. One SQLite transaction writes the complete run; conflicting session content is rejected. A test-only database trigger deliberately aborts a check insertion after earlier inserts, and the test verifies that all four evidence tables remain empty.

## Invalid values remain inspectable

Sequence values must fit SQLite's signed 64-bit integer range. Oversized sequence values are retained in raw evidence and produce failed sample validation, rather than crashing storage. Nonfinite JSON numbers and duplicate keys are rejected at the input boundary.

## Header rejection and Windows sockets

Integration tests exposed intermittent connection resets when a header-based rejection closed a Windows socket while the client was still uploading its body. The server now reads the bounded, timeout-limited body before returning a content-type or origin error. Oversized bodies are rejected before reading. This preserves clear error responses without accepting the rejected data.

## Physical meaning of a check

A stationary IMU can be functioning correctly. The application therefore evaluates motion response only for an explicitly guided movement exercise. Monitoring does not fail solely because angles are constant. Neither a motion span nor a bounds check establishes calibrated accuracy.
