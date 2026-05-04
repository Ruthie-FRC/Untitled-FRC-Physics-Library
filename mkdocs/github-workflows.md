# CI/CD and GitHub Actions

JSim utilizes GitHub Actions to ensure codebase reliability, run simulation physics tests, and automatically build cross-platform artifacts and vendordeps.

## Automated Workflows

The CI/CD pipeline is defined in the `.github/workflows/` directory:

### 1. Continuous Integration (`ci.yml`)
Triggers on all pushes to the `main` branch and all Pull Requests.
- **Java/C++ Matrix:** Builds the native C++ physics driver alongside the Java JNI bindings for multiple architectures (Windows, Linux, macOS).
- **Unit Testing:** Executes Google Tests (`vendordep/tests/`) for collision, math integration, and drivetrain mechanisms using CTest and Gradle. 
- **Linting:** Enforces C++ formatting (wpiformat) and python styling.

### 2. Deployment & Releases (`deploy.yml`)
Triggers upon creating a new GitHub Release.
- **Vendordep Publishing:** Compiles and publishes the `.json` vendordep and zipped native binaries for FRC WPILib integration.
- **Documentation:** Builds the MkDocs and Doxygen documentation and pushes it to GitHub Pages.

## Running Tests Locally

You can replicate the CI environment locally using the provided shell scripts in the `scripts/` folder. Ensure you have Java 17 installed.

### Build Everything
To compile the C++ physics core, Java vendor dependencies, and documentation:
```bash
./scripts/build-all.sh
```

### Run Tests
To execute the integration tests, physics model unit tests, and JNI validation:
```bash
./scripts/run-tests.sh
```
This script resolves the correct Java 17 environment and delegates to `./gradlew test` and CTest.
