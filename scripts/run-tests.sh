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
	echo "Java 21 is required to run vendordep tests; install it or set JAVA_HOME to a Java 21 runtime." >&2
	exit 1
fi

python "${ROOT_DIR}/apps/sim-runtime/contract_check.py"

cd "${ROOT_DIR}/vendordep"
bash ./gradlew test "$@"
