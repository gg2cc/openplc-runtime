"""Uploaded source timestamps should reflect content changes for make."""

import io
import os
import zipfile

from webserver.plcapp_management import analyze_zip, replace_generated_files


def _upload(directory, files):
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w") as output:
        for name, content in files.items():
            output.writestr(name, content)
    archive.seek(0)
    safe, entries = analyze_zip(archive)
    assert safe
    archive.seek(0)
    replace_generated_files(archive, str(directory), entries)


def test_upload_preserves_unchanged_timestamps_and_removes_old_sources(tmp_path):
    generated = tmp_path / "generated"
    _upload(generated, {
        "generated.hpp": "same header",
        "pou_OLD.cpp": "old POU",
        "pou_CHANGED.cpp": "original body",
    })
    old_time = 1_600_000_000_000_000_000
    for path in generated.iterdir():
        os.utime(path, ns=(old_time, old_time))

    _upload(generated, {
        "generated.hpp": "same header",
        "pou_CHANGED.cpp": "updated body",
        "pou_NEW.cpp": "new POU",
    })

    assert (generated / "generated.hpp").stat().st_mtime_ns == old_time
    assert (generated / "pou_CHANGED.cpp").stat().st_mtime_ns > old_time
    assert (generated / "pou_NEW.cpp").exists()
    assert not (generated / "pou_OLD.cpp").exists()


def test_identical_upload_keeps_source_timestamps(tmp_path):
    generated = tmp_path / "generated"
    files = {"configuration.cpp": "same code", "generated.hpp": "same header"}
    _upload(generated, files)
    timestamps = {path.name: path.stat(
    ).st_mtime_ns for path in generated.iterdir()}

    _upload(generated, files)

    assert {path.name: path.stat(
    ).st_mtime_ns for path in generated.iterdir()} == timestamps
