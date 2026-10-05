"""Export a FlightRecord: CSV, JSON, NumPy (.npz) and Parquet.

CSV      human-inspectable, one header row, ``#``-free; metadata goes in ``<name>.meta.json``.
JSON     everything (metadata, events, summary, schema, column arrays) in one file.
NPZ      ``columns`` (str array), ``data`` (float64 matrix), ``meta`` (JSON string).
Parquet  typed columns per the schema, ZSTD-compressed; metadata stored in the file footer
         under the key ``rocket_sim.meta`` so the file is self-describing.
Round-trip readers: ``read_record_parquet`` / ``read_record_npz``.
"""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from ..errors import RocketSimError
from ..simulation.record import FlightEvent, FlightRecord, RunMetadata
from ..version import SCHEMA_VERSION
from .schema import column, schema_dict

META_KEY = b"rocket_sim.meta"
FORMATS = ("csv", "json", "npz", "parquet")


def _jsonable(o: Any) -> Any:
    if isinstance(o, np.generic):
        return o.item()
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(f"not JSON serialisable: {type(o)}")


def record_meta_dict(rec: FlightRecord) -> dict[str, Any]:
    return {
        "metadata": rec.meta.to_dict(),
        "events": [asdict(e) for e in rec.events],
        "summary": rec.summary,
        "schema_version": SCHEMA_VERSION,
        "columns": rec.columns,
    }


def _arrow_table(rec: FlightRecord, extra_columns: dict[str, Any] | None = None) -> pa.Table:
    arrays, names = [], []
    for i, name in enumerate(rec.columns):
        dtype = column(name).dtype
        col = rec.data[:, i]
        arrays.append(pa.array(col.astype(dtype)))
        names.append(name)
    if extra_columns:
        for k, v in extra_columns.items():
            arrays.append(pa.array(np.full(rec.n_rows, v) if not isinstance(v, np.ndarray) else v))
            names.append(k)
    return pa.table(dict(zip(names, arrays, strict=True)))


def to_csv(rec: FlightRecord, path: Path) -> None:
    np.savetxt(path, rec.data, delimiter=",", header=",".join(rec.columns), comments="", fmt="%.9g")
    path.with_suffix(".meta.json").write_text(
        json.dumps(record_meta_dict(rec), indent=2, default=_jsonable), encoding="utf-8"
    )


def to_json(rec: FlightRecord, path: Path, include_data: bool = True) -> None:
    doc = record_meta_dict(rec)
    doc["schema"] = schema_dict()
    if include_data:
        doc["data"] = {c: rec.col(c).tolist() for c in rec.columns}
    path.write_text(json.dumps(doc, default=_jsonable), encoding="utf-8")


def to_npz(rec: FlightRecord, path: Path) -> None:
    np.savez_compressed(
        path,
        columns=np.array(rec.columns),
        data=rec.data,
        meta=np.array(json.dumps(record_meta_dict(rec), default=_jsonable)),
    )


def to_parquet(rec: FlightRecord, path: Path) -> None:
    table = _arrow_table(rec)
    meta = dict(table.schema.metadata or {})
    meta[META_KEY] = json.dumps(record_meta_dict(rec), default=_jsonable).encode()
    pq.write_table(table.replace_schema_metadata(meta), path, compression="zstd")


def export_record(
    rec: FlightRecord, directory: str | Path, stem: str | None = None, formats: tuple[str, ...] = ("csv",)
) -> list[Path]:
    """Write ``rec`` in each requested format; returns the written paths."""
    out = Path(directory)
    out.mkdir(parents=True, exist_ok=True)
    stem = stem or rec.meta.simulation_id
    written = []
    for fmt in formats:
        if fmt not in FORMATS:
            raise RocketSimError(f"unknown export format {fmt!r}; choose from {FORMATS}")
        p = out / f"{stem}.{'npz' if fmt == 'npz' else fmt}"
        {"csv": to_csv, "json": to_json, "npz": to_npz, "parquet": to_parquet}[fmt](rec, p)
        written.append(p)
    return written


def _record_from_meta(columns: list[str], data: np.ndarray, doc: dict[str, Any]) -> FlightRecord:
    md = doc["metadata"]
    meta = RunMetadata(**{k: md[k] for k in RunMetadata.__dataclass_fields__ if k in md})
    events = [FlightEvent(e["t"], e["name"], e["info"]) for e in doc["events"]]
    return FlightRecord(columns, data, events, meta, doc["summary"])


def read_record_parquet(path: str | Path) -> FlightRecord:
    t = pq.read_table(path)
    raw = (t.schema.metadata or {}).get(META_KEY)
    if raw is None:
        raise RocketSimError(f"{path}: not a rocket_sim record (missing metadata)")
    doc = json.loads(raw)
    cols = doc["columns"]
    data = np.column_stack([t.column(c).to_numpy().astype(np.float64) for c in cols])
    return _record_from_meta(cols, data, doc)


def read_record_npz(path: str | Path) -> FlightRecord:
    z = np.load(path, allow_pickle=False)
    doc = json.loads(str(z["meta"]))
    return _record_from_meta([str(c) for c in z["columns"]], z["data"], doc)


def read_record_csv(path: str | Path) -> FlightRecord:
    p = Path(path)
    with p.open(encoding="utf-8") as f:
        columns = f.readline().strip().split(",")
    data = np.loadtxt(p, delimiter=",", skiprows=1, ndmin=2)
    doc = json.loads(p.with_suffix(".meta.json").read_text(encoding="utf-8"))
    return _record_from_meta(columns, data, doc)
