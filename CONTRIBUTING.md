# Contributing

## Setup

1. Install [PlatformIO](https://platformio.org/install), either the VS Code
   extension or `pip install platformio`.
2. Install `clang-format` 18. For example, `pip install clang-format==18.1.8`.
3. Open the repository root in VS Code, so PlatformIO finds `platformio.ini`.

## Before every commit

```bash
pio run -e rov -e station -e thruster_test     # all firmwares build
pio test -e native                             # unit tests pass
clang-format -i $(git ls-files '*.cpp' '*.hpp') # code is formatted
```

CI runs the same three steps on every push and pull request.

## Code style

- **Formatting** is defined by [`.clang-format`](.clang-format): 4 spaces, 100
  columns, aligned consecutive declarations. Wrap hand-aligned tables in
  `// clang-format off` / `// clang-format on`.
- **Naming:**
  - `PascalCase` for types.
  - `camelCase` for functions and variables.
  - A `trailing_` underscore for private members.
  - `UPPER_CASE` for constants.
- **Pins and tunable values** go in the board's `config.hpp`, never as magic
  numbers in the code.
- **Shared code** goes in `lib/<Name>/src/` with a `library.json`. Keep
  hardware-free logic free of Arduino headers, so it can be unit tested on the
  PC.
- **Comments** explain *why*, plus units and sign conventions. Don't describe
  what the code obviously does.

## Changing the CAN protocol

See [docs/protocol.md](docs/protocol.md#changing-the-protocol). Always update
the round-trip tests, and flash both boards.

## Commits and pull requests

- **One topic per commit**, with an imperative subject line, for example
  "Add leak sensor flag to StatusTlm".
- **Link the roadmap item or issue** the change addresses.
- **For anything that affects thrust output, describe how it was tested**:
  bench, dry, or in water.
