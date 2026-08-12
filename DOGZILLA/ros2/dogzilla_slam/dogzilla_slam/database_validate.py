"""Read-only integrity checks for a persisted RTAB-Map database."""

import argparse
from pathlib import Path
import sqlite3

from dogzilla_slam.validation_report import make_validation_report
from dogzilla_slam.validation_report import write_json_report


REQUIRED_TABLES = {'Admin', 'Data', 'Link', 'Node', 'Statistics'}


def _count(connection, table):
    return connection.execute(
        f'SELECT COUNT(*) FROM "{table}"'
    ).fetchone()[0]


def validate_database(path, expected_version='0.23.7', minimum_nodes=1):
    source = Path(path)
    failures = []
    measurements = {
        'database_path': str(source),
        'file_size_bytes': source.stat().st_size if source.is_file() else 0,
    }
    if not source.is_file():
        failures.append(f'database file does not exist: {source}')
        return make_validation_report(
            'rtab-database',
            {
                'expected_version': expected_version,
                'minimum_nodes': minimum_nodes,
            },
            measurements,
            failures,
        )

    try:
        connection = sqlite3.connect(
            f'{source.resolve().as_uri()}?mode=ro',
            uri=True,
            timeout=2.0,
        )
        connection.execute('PRAGMA query_only = ON')
        quick_check = connection.execute(
            'PRAGMA quick_check'
        ).fetchone()[0]
        tables = {
            row[0]
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type='table'"
            )
        }
        measurements['sqlite_quick_check'] = quick_check
        measurements['tables'] = sorted(tables)
        if quick_check != 'ok':
            failures.append(f'SQLite quick_check failed: {quick_check}')
        missing_tables = sorted(REQUIRED_TABLES - tables)
        if missing_tables:
            failures.append(
                'required RTAB tables are missing: '
                + ', '.join(missing_tables)
            )

        if 'Admin' in tables:
            row = connection.execute(
                'SELECT version FROM Admin LIMIT 1'
            ).fetchone()
            version = row[0] if row else None
            measurements['rtab_database_version'] = version
            if version != expected_version:
                failures.append(
                    f'RTAB database version is {version!r}; '
                    f'expected {expected_version!r}'
                )

        counts = {
            table: _count(connection, table)
            for table in sorted(REQUIRED_TABLES & tables)
        }
        measurements['row_counts'] = counts
        if counts.get('Node', 0) < minimum_nodes:
            failures.append(
                f'database has only {counts.get("Node", 0)} nodes; '
                f'need {minimum_nodes}'
            )

        if {'Node', 'Data'} <= tables:
            missing_data = connection.execute(
                'SELECT COUNT(*) FROM Node '
                'LEFT JOIN Data ON Data.id = Node.id '
                'WHERE Data.id IS NULL'
            ).fetchone()[0]
            orphan_data = connection.execute(
                'SELECT COUNT(*) FROM Data '
                'LEFT JOIN Node ON Node.id = Data.id '
                'WHERE Node.id IS NULL'
            ).fetchone()[0]
            measurements['nodes_missing_data'] = missing_data
            measurements['orphan_data_rows'] = orphan_data
            if missing_data:
                failures.append(f'{missing_data} nodes have no Data row')
            if orphan_data:
                failures.append(f'{orphan_data} Data rows have no Node')

        if {'Node', 'Statistics'} <= tables:
            orphan_statistics = connection.execute(
                'SELECT COUNT(*) FROM Statistics '
                'LEFT JOIN Node ON Node.id = Statistics.id '
                'WHERE Node.id IS NULL'
            ).fetchone()[0]
            measurements['orphan_statistics_rows'] = orphan_statistics
            if orphan_statistics:
                failures.append(
                    f'{orphan_statistics} Statistics rows have no Node'
                )

        if {'Node', 'Link'} <= tables:
            dangling_links = connection.execute(
                'SELECT COUNT(*) FROM Link '
                'LEFT JOIN Node AS source_node '
                'ON source_node.id = Link.from_id '
                'LEFT JOIN Node AS target_node '
                'ON target_node.id = Link.to_id '
                'WHERE source_node.id IS NULL OR target_node.id IS NULL'
            ).fetchone()[0]
            measurements['dangling_links'] = dangling_links
            if dangling_links:
                failures.append(f'{dangling_links} links reference missing nodes')
        connection.close()
    except sqlite3.Error as exc:
        failures.append(f'cannot read RTAB database: {exc}')

    return make_validation_report(
        'rtab-database',
        {
            'expected_version': expected_version,
            'minimum_nodes': minimum_nodes,
        },
        measurements,
        failures,
    )


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--database', required=True)
    parser.add_argument('--expected-version', default='0.23.7')
    parser.add_argument('--minimum-nodes', type=int, default=1)
    parser.add_argument('--report-json')
    arguments = parser.parse_args(args)
    if arguments.minimum_nodes < 1:
        parser.error('--minimum-nodes must be at least 1')
    return arguments


def main(args=None):
    arguments = parse_arguments(args)
    report = validate_database(
        arguments.database,
        arguments.expected_version,
        arguments.minimum_nodes,
    )
    measurements = report['measurements']
    print(f'Database: {arguments.database}')
    print(f'SQLite quick_check: {measurements.get("sqlite_quick_check", "unavailable")}')
    print(f'RTAB version: {measurements.get("rtab_database_version", "unavailable")}')
    print(f'Nodes: {measurements.get("row_counts", {}).get("Node", 0)}')
    if arguments.report_json:
        write_json_report(arguments.report_json, report)
        print(f'Database report: {arguments.report_json}')
    if not report['passed']:
        print('RTAB database validation: FAILED')
        for failure in report['failures']:
            print(f'  - {failure}')
        raise SystemExit(1)
    print('RTAB database validation: PASSED')


if __name__ == '__main__':
    main()
