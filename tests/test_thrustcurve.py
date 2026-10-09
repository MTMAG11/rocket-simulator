"""ThrustCurve.org client, local motor library and the motor browser dialog. No test touches the network."""

from __future__ import annotations

import base64
import hashlib
import io
import json
import os
import urllib.error

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from rocket_sim.errors import MotorError
from rocket_sim.motor import load_motor
from rocket_sim.motor import thrustcurve as tc
from tests.conftest import ROOT

RASP = """; test motor
G40W 29 124 4-7-10 0.0538 0.123 AeroTech
0.024 74.325
0.5 63.063
1.0 54.054
2.0 30.0
2.3 0
"""


def api_motor(i: int, abbrev="AeroTech", des="G40W", data_files=2, **kw):
    d = {
        "motorId": f"5f4294d2000231000000{i:04x}",
        "manufacturer": "AeroTech" if abbrev == "AeroTech" else abbrev,
        "manufacturerAbbrev": abbrev,
        "designation": des,
        "commonName": des[:3],
        "impulseClass": des[0],
        "diameter": 29,
        "length": 124,
        "totImpulseNs": 97.14,
        "burnTimeS": 2.38,
        "avgThrustN": 40.9,
        "maxThrustN": 84.5,
        "propWeightG": 53.8,
        "totalWeightG": 123,
        "dataFiles": data_files,
        "availability": "regular",
        "infoUrl": "http://example.invalid/info",
    }
    d.update(kw)
    return d


def b64(text: str) -> str:
    return base64.b64encode(text.encode()).decode()


class FakeAPI:
    def __init__(self, motors, files=None):
        self.motors = motors
        self.files = files or {}
        self.calls: list[tuple[str, dict]] = []

    def __call__(self, endpoint, payload, timeout=60.0):
        self.calls.append((endpoint, payload))
        if endpoint == "search.json":
            res = self.motors
            if "commonName" in payload:
                res = [m for m in res if m["commonName"].lower() == payload["commonName"].lower()]
            if "manufacturer" in payload:
                res = [m for m in res if m["manufacturer"] == payload["manufacturer"]]
            return {"matches": len(res), "results": res}
        ids = payload["motorIds"]
        return {"results": [r for i in ids for r in self.files.get(i, [])]}


def rec(motor_id, text=RASP, source="cert", sim="s1"):
    return {
        "motorId": motor_id,
        "simfileId": sim,
        "format": "RASP",
        "source": source,
        "license": "PD",
        "data": b64(text),
    }


# ----------------------------------------------------------------------------------------- client
def test_search_maps_fields_and_filters_motors_without_data():
    api = FakeAPI([api_motor(1), api_motor(2, des="G40", data_files=0)])
    res = tc.search(name="G40", post=api)
    assert [m.designation for m in res] == ["G40W"]  # the one without a data file is dropped
    m = res[0]
    assert (m.total_impulse_ns, m.burn_time_s, m.diameter_mm, m.impulse_class) == (97.14, 2.38, 29.0, "G")
    assert m.stem == "AeroTech_G40W"
    ep, payload = api.calls[0]
    assert ep == "search.json" and payload["commonName"] == "G40" and payload["availability"] == "all"
    assert "hasDataFiles" not in payload  # the API returns nothing when it is sent


def test_find_by_name_matches_designation_or_common_name():
    api = FakeAPI(
        [
            api_motor(1),
            api_motor(2, abbrev="Estes", des="G40", manufacturer="Estes Industries"),
            api_motor(3, des="G80"),
        ]
    )
    assert {m.designation for m in tc.search(name="G40", post=api)} == {"G40W", "G40"}
    # find_by_name goes through the module-level _post, so substitute it for this call
    tc_post, tc._post = tc._post, api
    try:
        assert [m.designation for m in tc.find_by_name("G40W")] == [
            "G40W"
        ]  # the common name G40 is queried, then filtered
        assert {m.designation for m in tc.find_by_name("g40")} == {"G40W", "G40"}
        assert tc.find_by_name("Z999") == []
    finally:
        tc._post = tc_post


def test_stem_is_a_safe_file_name():
    m = tc.MotorInfo.from_api(api_motor(1, abbrev="Cesaroni Tech", des="O8000/P"))
    assert m.stem == "Cesaroni-Tech_O8000-P"


def test_download_prefers_certified_data_and_decodes_base64():
    mid = api_motor(1)["motorId"]
    api = FakeAPI(
        [],
        {
            mid: [
                rec(mid, source="user", sim="u"),
                rec(mid, source="cert", sim="c"),
                rec(mid, source="mfr", sim="m"),
            ]
        },
    )
    out = tc.download_files([mid], post=api)
    assert [r["source"] for r in out[mid]] == ["cert", "mfr", "user"]
    assert out[mid][0]["text"] == RASP
    assert api.calls[0][1]["format"] == "RASP"


def test_download_is_batched(monkeypatch):
    monkeypatch.setattr(tc, "BATCH", 2)
    ids = [f"id{i}" for i in range(5)]
    api = FakeAPI([], {})
    tc.download_files(ids, post=api)
    assert [len(c[1]["motorIds"]) for c in api.calls] == [2, 2, 1]


# ---------------------------------------------------------------------------------------- library
def test_save_motor_writes_a_loadable_file_with_provenance(tmp_path):
    info = tc.MotorInfo.from_api(api_motor(1))
    path = tc.save_motor(
        info,
        [{"text": RASP, "source": "cert", "license": "PD", "simfileId": "s1", "source_url": "http://x/s1"}],
        tmp_path,
    )
    assert path == tmp_path / "AeroTech_G40W.eng"
    m = load_motor(path)
    assert m.designation == "G40W" and m.total_impulse == pytest.approx(m.cum_impulse[-1])
    meta = tc.read_sidecar(path)
    assert meta["motor_id"] == info.motor_id and meta["data_source"] == "cert" and meta["simfile_id"] == "s1"
    assert meta["sha256"] == hashlib.sha256(path.read_bytes()).hexdigest()
    assert (
        meta["total_impulse_ns"] == 97.14
    )  # catalogue value, kept separate from what the file integrates to
    assert meta["file_total_impulse_ns"] == pytest.approx(m.total_impulse)


def test_save_motor_skips_a_corrupt_file_and_uses_the_next(tmp_path):
    info = tc.MotorInfo.from_api(api_motor(1))
    path = tc.save_motor(
        info, [{"text": "garbage", "source": "cert"}, {"text": RASP, "source": "mfr"}], tmp_path
    )
    assert path is not None and tc.read_sidecar(path)["data_source"] == "mfr"
    assert tc.save_motor(info, [{"text": "garbage"}], tmp_path / "other") is None


def test_same_name_different_catalogue_entry_does_not_overwrite(tmp_path):
    a, b = tc.MotorInfo.from_api(api_motor(1)), tc.MotorInfo.from_api(api_motor(2))
    pa = tc.save_motor(a, [{"text": RASP}], tmp_path)
    pb = tc.save_motor(b, [{"text": RASP}], tmp_path)
    assert pa != pb and pa.exists() and pb.exists()
    assert tc.read_sidecar(pa)["motor_id"] == a.motor_id and tc.read_sidecar(pb)["motor_id"] == b.motor_id


def test_download_motors_reports_failures_and_skips_existing(tmp_path):
    good, bad, none = (
        tc.MotorInfo.from_api(api_motor(i, des=d)) for i, d in ((1, "G40W"), (2, "G80T"), (3, "G10"))
    )
    api = FakeAPI([], {good.motor_id: [rec(good.motor_id)], bad.motor_id: [rec(bad.motor_id, text="junk")]})
    seen = []
    saved, failed = tc.download_motors(
        [good, bad, none], tmp_path, progress=lambda i, n, s: seen.append((i, n)), post=api
    )
    assert [p.name for p in saved] == ["AeroTech_G40W.eng"]
    assert {m.designation: why for m, why in failed} == {
        "G80T": "the file could not be parsed",
        "G10": "no thrust-curve file available",
    }
    assert seen[-1] == (3, 3)
    calls = len(api.calls)
    saved2, _ = tc.download_motors([good], tmp_path, post=api)
    assert saved2 and len(api.calls) == calls  # already saved: no request


def test_network_errors_become_actionable_motor_errors(monkeypatch):
    def boom(*_a, **_k):
        raise urllib.error.URLError("no route to host")

    monkeypatch.setattr(tc.urllib.request, "urlopen", boom)
    with pytest.raises(MotorError, match=r"Could not reach ThrustCurve.org"):
        tc.search(name="G40")

    def bad_json(*_a, **_k):
        return io.BytesIO(b"<html>")

    monkeypatch.setattr(tc.urllib.request, "urlopen", bad_json)
    with pytest.raises(MotorError, match="unreadable"):
        tc.search(name="G40")


# -------------------------------------------------------------------------------------- bundled
def test_bundled_downloads_carry_provenance_and_load():
    for stem in ("AeroTech_G40W", "Estes_F15"):
        eng = ROOT / "data" / "motors" / f"{stem}.eng"
        if not eng.exists():
            pytest.skip("motor files not downloaded")
        meta = tc.read_sidecar(eng)
        assert meta["sha256"] == hashlib.sha256(eng.read_bytes()).hexdigest()
        assert load_motor(eng).designation == meta["file_designation"]


# --------------------------------------------------------------------------------------- catalog
def test_library_motors_are_listed_after_bundled_ones(tmp_path, monkeypatch):
    from rocket_sim.ui.catalog import list_motors

    monkeypatch.setenv("ROCKETSIM_WORKSPACE", str(tmp_path))
    info = tc.MotorInfo.from_api(api_motor(7, abbrev="ACME", des="H123"))
    tc.save_motor(info, [{"text": RASP.replace("G40W", "H123")}], tmp_path / "motors")
    shadow = tc.MotorInfo.from_api(api_motor(8, abbrev="AeroTech", des="G80T"))
    tc.save_motor(shadow, [{"text": RASP.replace("G40W", "G80T")}], tmp_path / "motors")
    entries = list_motors()
    labels = [e.label for e in entries]
    assert "ACME_H123" in labels and labels.index("AeroTech_G80T") < labels.index("ACME_H123")
    assert labels.count("AeroTech_G80T") == 1  # a downloaded copy never shadows the bundled motor
    h = next(e for e in entries if e.label == "ACME_H123")
    assert h.key.endswith("ACME_H123.eng") and "H123" in h.summary


# ------------------------------------------------------------------------------------------ CLI
def test_cli_fetch_list_and_download(tmp_path, monkeypatch, capsys):
    from rocket_sim.cli import main

    motors = [api_motor(1), api_motor(2, abbrev="Estes", des="F15", manufacturer="Estes Industries")]
    api = FakeAPI(motors, {motors[0]["motorId"]: [rec(motors[0]["motorId"])]})
    monkeypatch.setattr(tc, "_post", api)
    assert main(["fetch-motors", "G40W", "--list"]) == 0
    assert "AeroTech_G40W" in capsys.readouterr().out
    assert main(["fetch-motors", "G40W", "--dest", str(tmp_path)]) == 0
    assert (tmp_path / "AeroTech_G40W.eng").exists() and (tmp_path / "AeroTech_G40W.json").exists()
    assert main(["fetch-motors"]) == 1  # needs names or filters
    assert "give motor names" in capsys.readouterr().err
    assert main(["fetch-motors", "Z999"]) == 1


# ---------------------------------------------------------------------------------------- dialog
@pytest.fixture(scope="module")
def qapp():
    pytest.importorskip("PySide6")
    from PySide6 import QtWidgets

    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


def test_motor_browser_search_download_and_signal(qapp, tmp_path, monkeypatch):
    from rocket_sim.ui.motor_browser import MotorBrowser

    motors = [
        api_motor(1, abbrev="ACME", des="H123"),
        api_motor(2, abbrev="Estes", des="F15", manufacturer="Estes Industries", totImpulseNs=49.6),
    ]
    api = FakeAPI(
        motors,
        {m["motorId"]: [rec(m["motorId"], text=RASP.replace("G40W", m["designation"]))] for m in motors},
    )
    monkeypatch.setattr(tc, "_post", api)
    dlg = MotorBrowser(directory=tmp_path)
    got: list[str] = []
    dlg.motors_changed.connect(got.append)

    dlg.search()  # no criteria: refuses instead of fetching the whole catalogue
    assert not api.calls and "Enter a name" in dlg.status.text()

    dlg.name.setText("H12")
    dlg.search()
    dlg.wait()
    assert dlg.table.rowCount() == 1 and dlg.table.item(0, 0).text() == "H123"
    assert dlg.table.item(0, 7).text() == "Online"

    dlg.table.selectRow(0)
    assert dlg.dl_selected.isEnabled()
    dlg.download_selected()
    dlg.wait()
    assert (tmp_path / "ACME_H123.eng").exists()
    assert got == ["ACME_H123"]
    assert dlg.table.item(0, 7).text() == "Saved"
    assert "Saved 1 motors" in dlg.status.text()


def test_motor_browser_shows_network_errors(qapp, tmp_path, monkeypatch):
    from rocket_sim.ui.motor_browser import MotorBrowser

    def down(*_a, **_k):
        raise MotorError("Could not reach ThrustCurve.org (offline)")

    monkeypatch.setattr(tc, "_post", down)
    dlg = MotorBrowser(directory=tmp_path)
    dlg.name.setText("G40")
    dlg.search()
    dlg.wait()
    assert "Could not reach" in dlg.status.text()
    assert dlg.search_button.isEnabled()


def test_main_window_picks_up_downloaded_motors(qapp, tmp_path, monkeypatch):
    from rocket_sim.ui.main_window import MainWindow

    monkeypatch.setenv("ROCKETSIM_WORKSPACE", str(tmp_path))
    w = MainWindow()
    n = w.motor_box.count()
    info = tc.MotorInfo.from_api(api_motor(9, abbrev="ACME", des="H123"))
    tc.save_motor(info, [{"text": RASP.replace("G40W", "H123")}], tmp_path / "motors")
    w._fill_motors(select="ACME_H123")
    assert w.motor_box.count() == n + 1 and w.motor_box.currentText() == "ACME_H123"
    assert "H123" in w.motor_label.text()
    w.close()
    assert json.loads((tmp_path / "motors" / "ACME_H123.json").read_text())["designation"] == "H123"
