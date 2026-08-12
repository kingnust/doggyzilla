"""Read-only RTAB database integrity checks."""

import sqlite3

from dogzilla_slam.database_validate import validate_database


def _database(path, version='0.23.7'):
    connection = sqlite3.connect(path)
    connection.executescript('''
        CREATE TABLE Admin(version TEXT);
        CREATE TABLE Node(id INTEGER PRIMARY KEY);
        CREATE TABLE Data(id INTEGER PRIMARY KEY);
        CREATE TABLE Statistics(id INTEGER);
        CREATE TABLE Link(
            from_id INTEGER,
            to_id INTEGER,
            type INTEGER,
            information_matrix BLOB
        );
    ''')
    connection.execute('INSERT INTO Admin(version) VALUES (?)', (version,))
    connection.execute('INSERT INTO Node(id) VALUES (1)')
    connection.execute('INSERT INTO Data(id) VALUES (1)')
    connection.execute('INSERT INTO Statistics(id) VALUES (1)')
    connection.execute(
        'INSERT INTO Link(from_id, to_id, type, information_matrix) '
        'VALUES (1, 1, 0, ?)',
        (b'matrix',),
    )
    connection.commit()
    return connection


def test_consistent_database_passes_without_modification(tmp_path):
    path = tmp_path / 'rtabmap.db'
    connection = _database(path)
    connection.close()
    before = path.read_bytes()

    report = validate_database(path)

    assert report['passed'] is True
    assert report['measurements']['sqlite_quick_check'] == 'ok'
    assert report['measurements']['row_counts']['Node'] == 1
    assert report['measurements']['dangling_links'] == 0
    assert path.read_bytes() == before


def test_wrong_version_and_relational_damage_are_rejected(tmp_path):
    path = tmp_path / 'rtabmap.db'
    connection = _database(path, version='0.22.0')
    connection.execute('DELETE FROM Data WHERE id = 1')
    connection.execute(
        'INSERT INTO Link(from_id, to_id, type, information_matrix) '
        'VALUES (1, 99, 0, ?)',
        (b'matrix',),
    )
    connection.commit()
    connection.close()

    report = validate_database(path)

    assert report['passed'] is False
    assert any('expected' in failure for failure in report['failures'])
    assert '1 nodes have no Data row' in report['failures']
    assert '1 links reference missing nodes' in report['failures']


def test_missing_database_returns_a_reportable_failure(tmp_path):
    report = validate_database(tmp_path / 'missing.db')
    assert report['passed'] is False
    assert report['measurements']['file_size_bytes'] == 0
    assert 'does not exist' in report['failures'][0]
