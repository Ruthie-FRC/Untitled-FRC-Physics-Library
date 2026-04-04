#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

resolve_java_home() {
  local candidate

  for candidate in /home/codespace/java/21* /usr/local/sdkman/candidates/java/21*; do
    if [[ -x "${candidate}/bin/java" ]] && "${candidate}/bin/java" -version 2>&1 | grep -q 'version "21'; then
      echo "${candidate}"
      return 0
    fi
  done

  if [[ -n "${JAVA_HOME:-}" && -x "${JAVA_HOME}/bin/java" ]] && "${JAVA_HOME}/bin/java" -version 2>&1 | grep -q 'version "21'; then
    echo "${JAVA_HOME}"
    return 0
  fi

  return 1
}

JAVA_HOME_OVERRIDE="$(resolve_java_home || true)"
if [[ -n "${JAVA_HOME_OVERRIDE}" ]]; then
  export JAVA_HOME="${JAVA_HOME_OVERRIDE}"
  export PATH="${JAVA_HOME}/bin:${PATH}"
else
  echo "Java 21 is required for vendordep Gradle build; install it or set JAVA_HOME to Java 21." >&2
  exit 1
fi

if [[ -f "${ROOT_DIR}/build/CTestTestfile.cmake" ]]; then
  (cd "${ROOT_DIR}/build" && ctest --output-on-failure)
fi

cd "${ROOT_DIR}/vendordep"
./gradlew build
