#!/usr/bin/env python3
"""Isolated lifecycle tests for the host-side Mission Mode coordinator."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
import unittest


MISSION_SCRIPT = Path(__file__).resolve().parents[1] / "dogzilla-mission"


class MissionCommandTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        (self.root / "deploy").mkdir()
        (self.root / "logs").mkdir()
        (self.root / "maps").mkdir()
        (self.root / "deploy" / "compose.yaml").write_text(
            "services: {}\n", encoding="utf-8"
        )
        for suffix in ("pbstream", "yaml", "pgm"):
            (self.root / "maps" / f"room1.{suffix}").write_bytes(b"fixture\n")

        self.call_log = self.root / "calls.log"
        self.map_command = self._write_executable(
            "fake-map",
            r"""
            #!/usr/bin/env bash
            set -eu
            printf 'map:%s\n' "$*" >> "${TEST_CALL_LOG}"
            case "${1:-}" in
                navigate)
                    mkdir -p "${TEST_ROOT}/logs"
                    printf 'session-123\n' > "${TEST_ROOT}/logs/current-session"
                    : > "${TEST_ROOT}/navigation.running"
                    ;;
                stop)
                    if [[ "${TEST_MAP_STOP_FAIL:-0}" == 1 ]]; then
                        exit 8
                    fi
                    rm -f "${TEST_ROOT}/navigation.running"
                    ;;
                status)
                    printf 'navigation status\n'
                    ;;
            esac
            """,
        )
        self.web_command = self._write_executable(
            "fake-web",
            r"""
            #!/usr/bin/env bash
            set -eu
            printf 'web:%s:log=%s\n' "$*" "${DOGZILLA_ROS_LOG_DIR:-unset}" \
                >> "${TEST_CALL_LOG}"
            case "${1:-}" in
                start)
                    if [[ "${TEST_WEB_START_FAIL:-0}" == 1 ]]; then
                        exit 9
                    fi
                    : > "${TEST_ROOT}/web.running"
                    ;;
                stop)
                    rm -f "${TEST_ROOT}/web.running"
                    ;;
                status)
                    printf 'web status\n'
                    ;;
                show-token)
                    printf 'test-token\n'
                    ;;
            esac
            """,
        )
        self.docker_command = self._write_executable(
            "fake-docker",
            r"""
            #!/usr/bin/env bash
            set -eu
            printf 'docker:%s\n' "$*" >> "${TEST_CALL_LOG}"
            case "${1:-}" in
                inspect)
                    container="${@: -1}"
                    marker=''
                    case "${container}" in
                        dogzilla_mapping) marker="${TEST_ROOT}/mapping.running" ;;
                        dogzilla_drive) marker="${TEST_ROOT}/drive.running" ;;
                        dogzilla_navigation) marker="${TEST_ROOT}/navigation.running" ;;
                        dogzilla_web) marker="${TEST_ROOT}/web.running" ;;
                    esac
                    [[ -n "${marker}" && -f "${marker}" ]] || exit 1
                    if [[ "$*" == *'.State.Health'* ]]; then
                        printf 'healthy\n'
                    elif [[ "${container}" == dogzilla_mapping ]]; then
                        printf '%s\n' "${TEST_MAPPING_STATUS:-running}"
                    else
                        printf 'running\n'
                    fi
                    ;;
                exec)
                    [[ "${TEST_ROS_READY:-1}" == 1 ]]
                    ;;
                compose)
                    exit 0
                    ;;
                *)
                    exit 2
                    ;;
            esac
            """,
        )

        self.environment = os.environ.copy()
        self.environment.update(
            {
                "DOGZILLA_MISSION_REPO_ROOT": str(self.root),
                "DOGZILLA_MISSION_MAP_COMMAND": str(self.map_command),
                "DOGZILLA_MISSION_WEB_COMMAND": str(self.web_command),
                "DOGZILLA_MISSION_DOCKER_COMMAND": str(self.docker_command),
                "DOGZILLA_MISSION_STARTUP_TIMEOUT": "2",
                "TEST_CALL_LOG": str(self.call_log),
                "TEST_ROOT": str(self.root),
            }
        )

    def tearDown(self) -> None:
        self.temporary_directory.cleanup()

    def _write_executable(self, name: str, contents: str) -> Path:
        path = self.root / name
        path.write_text(textwrap.dedent(contents).lstrip(), encoding="utf-8")
        path.chmod(0o755)
        return path

    def _run(self, *arguments: str, **environment: str) -> subprocess.CompletedProcess[str]:
        run_environment = self.environment.copy()
        run_environment.update(environment)
        return subprocess.run(
            [str(MISSION_SCRIPT), *arguments],
            check=False,
            capture_output=True,
            env=run_environment,
            text=True,
            timeout=10,
        )

    def _calls(self) -> str:
        if not self.call_log.exists():
            return ""
        return self.call_log.read_text(encoding="utf-8")

    def test_start_launches_navigation_then_web_and_records_ready_state(self) -> None:
        result = self._run("start", "room1", "--headless")

        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self._calls()
        self.assertLess(
            calls.index("map:navigate room1 --headless"),
            calls.index("web:start room1"),
        )
        self.assertIn("web:start room1:log=/logs/sessions/session-123", calls)
        self.assertIn("docker:exec dogzilla_navigation", calls)
        state = (self.root / "logs" / "mission-current").read_text(encoding="utf-8")
        self.assertIn("state=ready\n", state)
        self.assertIn("map=room1\n", state)
        self.assertIn("session=session-123\n", state)
        self.assertIn("No movement has been queued automatically.", result.stdout)

    def test_map_name_is_a_start_shortcut_and_defaults_to_headless(self) -> None:
        result = self._run("room1")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("map:navigate room1 --headless", self._calls())

    def test_active_mapping_is_never_replaced(self) -> None:
        (self.root / "mapping.running").touch()

        result = self._run("start", "room1")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("dogzilla_mapping is already running", result.stderr)
        calls = self._calls()
        self.assertNotIn("map:navigate", calls)
        self.assertNotIn("web:start", calls)

    def test_restarting_mapping_is_also_treated_as_active(self) -> None:
        (self.root / "mapping.running").touch()

        result = self._run(
            "start",
            "room1",
            TEST_MAPPING_STATUS="restarting",
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("dogzilla_mapping is already running", result.stderr)
        self.assertNotIn("map:navigate", self._calls())

    def test_web_start_failure_rolls_back_navigation_and_removes_state(self) -> None:
        result = self._run("start", "room1", TEST_WEB_START_FAIL="1")

        self.assertNotEqual(result.returncode, 0)
        calls = self._calls()
        self.assertIn("web:start room1", calls)
        self.assertIn("web:stop", calls)
        self.assertIn("map:stop", calls)
        self.assertFalse((self.root / "navigation.running").exists())
        self.assertFalse((self.root / "logs" / "mission-current").exists())
        self.assertIn("startup failed", result.stderr)

    def test_failed_ros_readiness_rolls_back_both_services(self) -> None:
        result = self._run("start", "room1", TEST_ROS_READY="0")

        self.assertNotEqual(result.returncode, 0)
        calls = self._calls()
        self.assertIn("web:stop", calls)
        self.assertIn("map:stop", calls)
        self.assertFalse((self.root / "web.running").exists())
        self.assertFalse((self.root / "navigation.running").exists())

    def test_stop_without_managed_state_does_not_stop_manual_services(self) -> None:
        (self.root / "navigation.running").touch()
        (self.root / "web.running").touch()

        result = self._run("stop")

        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("no services were stopped", result.stdout)
        self.assertNotIn("map:stop", self._calls())
        self.assertNotIn("web:stop", self._calls())
        self.assertTrue((self.root / "navigation.running").exists())
        self.assertTrue((self.root / "web.running").exists())

    def test_managed_stop_uses_web_first_and_removes_state_after_success(self) -> None:
        state_file = self.root / "logs" / "mission-current"
        state_file.write_text(
            "state=ready\nmap=room1\nsession=session-123\n",
            encoding="utf-8",
        )
        (self.root / "navigation.running").touch()
        (self.root / "web.running").touch()

        result = self._run("stop")

        self.assertEqual(result.returncode, 0, result.stderr)
        calls = self._calls()
        self.assertLess(calls.index("web:stop"), calls.index("map:stop"))
        self.assertFalse(state_file.exists())
        self.assertFalse((self.root / "navigation.running").exists())
        self.assertFalse((self.root / "web.running").exists())

    def test_failed_managed_stop_keeps_state_for_a_retry(self) -> None:
        state_file = self.root / "logs" / "mission-current"
        state_file.write_text(
            "state=ready\nmap=room1\nsession=session-123\n",
            encoding="utf-8",
        )
        (self.root / "navigation.running").touch()
        (self.root / "web.running").touch()

        result = self._run("stop", TEST_MAP_STOP_FAIL="1")

        self.assertNotEqual(result.returncode, 0)
        self.assertTrue(state_file.exists())
        self.assertTrue((self.root / "navigation.running").exists())

    def test_missing_map_bundle_is_rejected_before_runtime_commands(self) -> None:
        result = self._run("start", "missing")

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Required mission map file is missing", result.stderr)
        self.assertEqual(self._calls(), "")


if __name__ == "__main__":
    unittest.main()
