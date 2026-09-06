# Team 23684 Tech Titans

FTC robot code for the 2026-2027 Biobuzz season.

## Commands

Format:
```sh
dprint fmt
```

Build:
```sh
./gradlew assembleDebug
```

Lint:
```sh
./gradlew lintDebug
```

## Git

- Always use the global git config author. Never add a co-author trailer
  (`Co-authored-by:` or similar) to commit messages.
- After every commit, run `git log -1` to verify:
  - The commit message formatted correctly with no escape sequences
    leaking into the message body.
  - The author matches the global git config (not an AI agent identity).