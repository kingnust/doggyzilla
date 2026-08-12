"""Tests for durable machine-readable validation evidence."""

import json

import pytest

from dogzilla_slam.validation_report import make_validation_report
from dogzilla_slam.validation_report import ValidationReportError
from dogzilla_slam.validation_report import write_json_report


def test_report_is_valid_json_and_atomically_replaces_existing_file(tmp_path):
    output = tmp_path / 'report.json'
    output.write_text('old report\n', encoding='utf-8')
    report = make_validation_report(
        'visual-shadow-route',
        {'minimum_travel_metres': 1.0},
        {'path_length_metres': 1.4},
        [],
    )

    assert write_json_report(output, report) == output
    stored = json.loads(output.read_text(encoding='utf-8'))
    assert stored['schema_version'] == 1
    assert stored['kind'] == 'visual-shadow-route'
    assert stored['passed'] is True
    assert stored['failures'] == []
    assert output.stat().st_mode & 0o777 == 0o644
    assert not list(tmp_path.glob('.report.json.*.tmp'))


def test_failed_serialization_preserves_previous_report(tmp_path):
    output = tmp_path / 'report.json'
    output.write_text('previous\n', encoding='utf-8')
    with pytest.raises(ValidationReportError):
        write_json_report(output, {'invalid': float('nan')})
    assert output.read_text(encoding='utf-8') == 'previous\n'
    assert not list(tmp_path.glob('.report.json.*.tmp'))
