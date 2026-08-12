"""Atomic JSON reports for repeatable DOGZILLA acceptance checks."""

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile


class ValidationReportError(RuntimeError):
    """Raised when validation evidence cannot be stored safely."""


def make_validation_report(kind, requirements, measurements, failures):
    return {
        'schema_version': 1,
        'kind': kind,
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'passed': not failures,
        'requirements': requirements,
        'measurements': measurements,
        'failures': list(failures),
    }


def write_json_report(path, document):
    """Atomically replace a report without leaving partial JSON behind."""
    destination = Path(path)
    if not destination.parent.is_dir():
        raise ValidationReportError(
            f'report directory does not exist: {destination.parent}'
        )
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            dir=destination.parent,
            prefix=f'.{destination.name}.',
            suffix='.tmp',
            delete=False,
        ) as stream:
            temporary_path = Path(stream.name)
            json.dump(
                document,
                stream,
                allow_nan=False,
                indent=2,
                sort_keys=True,
            )
            stream.write('\n')
            stream.flush()
            os.fsync(stream.fileno())
        temporary_path.chmod(0o644)
        os.replace(temporary_path, destination)
    except (OSError, TypeError, ValueError) as exc:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise ValidationReportError(
            f'cannot write validation report {destination}: {exc}'
        ) from exc
    return destination
