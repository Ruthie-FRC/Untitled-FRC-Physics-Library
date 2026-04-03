# RenSim Versioning Guide

## Format

RenSim uses **date-based versioning**: `YYYY.MM.DD.patch-no`

### Components

- **YYYY.MM.DD**: Release date (e.g., `2026.04.03` for April 3, 2026)
- **patch-no**: Incrementing counter for multiple releases on the same day (e.g., `.0`, `.1`, `.2`)
- **Suffix** (optional): 
  - `-prerelease` for unstable/beta releases
  - `-rc1`, `-rc2` etc. for release candidates
  - No suffix for stable/production releases

### Examples

| Version | Meaning |
|---------|---------|
| `2026.04.03.0` | First stable release on April 3, 2026 |
| `2026.04.03.0-prerelease` | First prerelease on April 3, 2026 |
| `2026.04.03.1-prerelease` | Second prerelease on April 3, 2026 |
| `2026.04.10.0` | First stable release on April 10, 2026 |
| `2026.04.10.0-rc1` | Release candidate 1 on April 10, 2026 |

## Where Versions Are Defined

Update these files when bumping version:

1. **CMakeLists.txt** - Line 3, `project(... VERSION ...)`
2. **vendordep/publish.gradle** - `def pubVersion = '...'`
3. **CHANGELOG.md** - Section header and date
4. **QUICKSTART.md** - Title (if mentioning version)
5. **README.md** - Status line (if mentioning version)
6. **RELEASE_SUMMARY.md** - Title and version field

## Release Workflow

### First Release of the Day
```bash
# version = 2026.04.DD.0
# Example for April 3: 2026.04.03.0-prerelease
```

### Second Release of the Day (patch)
```bash
# version = 2026.04.DD.1
# Example for April 3: 2026.04.03.1-prerelease
```

### Release Checklist

1. Update all 6 version files above
2. Run `cmake -B build && cmake --build build -j4`
3. Run `ctest --test-dir build --output-on-failure`
4. Verify tests pass (9/9)
5. Commit with message: `Release 2026.04.DD.X`
6. Tag: `git tag 2026.04.DD.X && git push --tags`
7. Create GitHub release with CHANGELOG section

## Why Date-Based Versioning?

- **Clarity**: Anyone can see when code was released
- **Ordering**: Dates naturally sort chronologically
- **No debate**: No semantic versioning arguments (major/minor/patch)
- **Continuous**: Can release updates anytime without artificial version bumps
