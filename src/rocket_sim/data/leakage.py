"""Train/test leakage checks and dataset statistics.

Leakage that matters for ML on simulated flights:

1. *Trajectory leakage* - windows from one flight in more than one split. Prevented by construction (splits are
   made of whole runs with disjoint seed namespaces); ``check_run_disjoint`` verifies it on a finished dataset.
2. *Near-duplicate leakage* - different runs whose sampled parameters are almost identical (e.g. a hold-out
   distribution overlapping the training one, or a collapsed parameter range). The old check only caught bit-identical
   vectors. ``near_duplicate_report`` measures nearest-neighbour distances in standardised parameter space.
3. *Temporal leakage* - an input window that depends on data AFTER its end time. ``temporal_causality_check``
   perturbs everything after a cut time and verifies that every window ending before the cut is unchanged.
4. *Label leakage* is the user's choice of inputs (truth vs measured); it is recorded in the manifest, not policed.
"""

from __future__ import annotations

import math
from dataclasses import replace
from typing import Any

import numpy as np

from ..simulation.record import FlightRecord
from .spec import DatasetSection


def _param_matrix(rows: list[dict[str, Any]]) -> tuple[list[str], np.ndarray, list[str]]:
    """Numeric parameter matrix (accepted runs only) over parameters that actually vary."""
    keys = sorted({k for r in rows for k in r if k.startswith("param.")})
    use = [r for r in rows if r.get("accepted", True)]
    cols, names = [], []
    for k in keys:
        try:
            v = np.array([float(r[k]) for r in use])
        except (TypeError, ValueError, KeyError):
            continue
        if len(v) > 1 and np.isfinite(v).all() and v.std() > 0:
            cols.append(v)
            names.append(k)
    splits = [r["split"] for r in use]
    return names, (np.column_stack(cols) if cols else np.zeros((len(use), 0))), splits


def near_duplicate_report(
    rows: list[dict[str, Any]], reference: str = "train", eps: float = 1e-3, rel_eps: float = 0.05
) -> dict[str, Any]:
    """Nearest-neighbour distances (standardised parameters, divided by sqrt(d)) from every other split to ``reference``.

    A run is a near-duplicate if its distance to the nearest reference run is below ``max(eps, rel_eps x the median
    nearest-neighbour distance WITHIN the reference split)`` - i.e. it sits much closer to a training run than training runs
    sit to each other, which is the signature of leakage (a fixed absolute eps is meaningless in high dimension).
    ``n_near_duplicates`` counts them: a high count means the splits are not independent samples. The within-reference median NN distance is reported as the natural scale for comparison."""
    from scipy.spatial import cKDTree

    names, x, splits = _param_matrix(rows)
    all_keys = sorted({k for r in rows if r.get("accepted", True) for k in r if k.startswith("param.")})
    out: dict[str, Any] = {
        "parameters_not_compared": [k for k in all_keys if k not in names],
        "parameters_compared": names,
        "eps": eps,
        "rel_eps": rel_eps,
        "reference_split": reference,
        "splits": {},
    }
    if x.shape[1] == 0 or reference not in set(splits):
        out["note"] = "no varying numeric parameters or no reference split: nothing to compare"
        return out
    mu, sd = x.mean(axis=0), x.std(axis=0)
    z = (x - mu) / sd / math.sqrt(x.shape[1])
    sp = np.array(splits)
    ref = z[sp == reference]
    tree = cKDTree(ref)
    if len(ref) > 1:
        d_in, _ = tree.query(ref, k=2)
        out["reference_within_median_nn"] = float(np.median(d_in[:, 1]))
        thr = max(eps, rel_eps * out["reference_within_median_nn"])
    else:
        thr = eps
    out["threshold"] = thr
    for s in sorted(set(splits) - {reference}):
        d, _ = tree.query(z[sp == s], k=1)
        out["splits"][s] = {
            "n": int(len(d)),
            "min_nn": float(d.min()),
            "median_nn": float(np.median(d)),
            "n_near_duplicates": int((d < thr).sum()),
        }
    return out


def check_run_disjoint(rows: list[dict[str, Any]]) -> dict[str, int]:
    """Counts of simulation ids / seeds that appear in more than one split (must both be 0)."""
    ids: dict[str, set[str]] = {}
    seeds: dict[int, set[str]] = {}
    for r in rows:
        ids.setdefault(r["simulation_id"], set()).add(r["split"])
        seeds.setdefault(int(r["random_seed"]), set()).add(r["split"])
    return {
        "ids_in_multiple_splits": sum(1 for v in ids.values() if len(v) > 1),
        "seeds_in_multiple_splits": sum(1 for v in seeds.values() if len(v) > 1),
    }


def temporal_causality_check(rec: FlightRecord, section: DatasetSection, n_cuts: int = 24) -> dict[str, Any]:
    """Verify input features are causal. For several cut times, everything recorded after the cut is replaced by garbage
    and the resampled INPUT table is rebuilt: every row whose time is strictly before the cut must be IDENTICAL. (Windows
    are slices of this table, so causal rows imply causal windows.) A pipeline that interpolates toward the next
    native sample leaks only within one native step of a cut, which is why many cuts are tried."""
    from .dataset import build_tabular, build_windows

    t = rec.col("t")
    base = build_tabular(rec, section)
    max_diff, compared = 0.0, 0
    cuts = []
    for frac in np.linspace(0.08, 0.92, n_cuts):
        t_cut = float(t[0] + frac * (t[-1] - t[0]))
        corrupted = rec.data.copy()
        after = t > t_cut
        corrupted[after, 1:] = np.random.default_rng(0).normal(
            0.0, 1e6, size=corrupted[after, 1:].shape
        )  # col 0 = time
        alt = build_tabular(replace(rec, data=corrupted), section)
        m = base.t < t_cut - 1e-9
        compared += int(m.sum())
        d = float(np.max(np.abs(base.inputs[m] - alt.inputs[m]))) if m.any() else 0.0
        max_diff = max(max_diff, d)
        cuts.append(t_cut)
        if (
            section.kind == "windowed"
        ):  # the windowing code itself: windows ending before the cut must not change
            w1, w2 = build_windows(rec, section), build_windows(replace(rec, data=corrupted), section)
            mw = w1.t_end < t_cut - 1e-9
            if mw.any():
                max_diff = max(max_diff, float(np.max(np.abs(w1.x[mw] - w2.x[mw]))))
    return {
        "causal": bool(max_diff == 0.0),
        "rows_compared": compared,
        "max_abs_difference": max_diff,
        "cuts": len(cuts),
    }


def _stats(v: np.ndarray) -> dict[str, float]:
    return {
        "mean": float(v.mean()),
        "std": float(v.std(ddof=1)) if len(v) > 1 else 0.0,
        "min": float(v.min()),
        "p05": float(np.percentile(v, 5)),
        "median": float(np.median(v)),
        "p95": float(np.percentile(v, 95)),
        "max": float(v.max()),
    }


def dataset_statistics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Per-split counts, sampled-parameter statistics and result (``res.*``) statistics of ACCEPTED runs."""
    out: dict[str, Any] = {"splits": {}}
    for s in sorted({r["split"] for r in rows}):
        rs = [r for r in rows if r["split"] == s]
        acc = [r for r in rs if r["accepted"]]
        d: dict[str, Any] = {
            "runs": len(rs),
            "accepted": len(acc),
            "rejected": len(rs) - len(acc),
            "parameters": {},
            "results": {},
        }
        for kind, prefix in (("parameters", "param."), ("results", "res.")):
            keys = sorted({k for r in acc for k in r if k.startswith(prefix)})
            for k in keys:
                try:
                    v = np.array([float(r[k]) for r in acc if r.get(k) is not None])
                except (TypeError, ValueError):
                    continue
                if len(v) and np.isfinite(v).all():  # (n=1 gives std 0 by construction)
                    d[kind][k[len(prefix) :]] = _stats(v)
        out["splits"][s] = d
    return out


def parameter_distribution_checks(
    rows: list[dict[str, Any]], params: list[Any], base: dict[str, Any], min_n: int = 200
) -> list[dict[str, Any]]:
    """Kolmogorov-Smirnov test of sampled values against the DECLARED marginal (normal / uniform / lognormal without
    clipping). Catches sampler bugs; with n >= ``min_n`` a p-value below 1e-3 is flagged. Correlated parameters keep
    their declared marginals, so they are tested the same way."""
    from scipy import stats as st

    from .montecarlo import get_path

    res = []
    for p in params:
        if p.dist not in ("normal", "uniform", "lognormal") or (p.low is not None and p.dist != "uniform"):
            continue
        v = np.array([float(r[f"param.{p.path}"]) for r in rows if r["accepted"] and f"param.{p.path}" in r])
        if len(v) < min_n:
            continue
        b = get_path(base, p.path)
        if p.dist == "normal":
            mean = p.mean if p.mean is not None else float(b)
            std = p.std if p.std is not None else float(p.rel_std) * abs(mean)
            cdf = st.norm(mean, std).cdf
        elif p.dist == "uniform":
            lo, hi = (
                (p.low, p.high)
                if p.rel_range is None
                else (float(b) * (1 - p.rel_range), float(b) * (1 + p.rel_range))
            )
            cdf = st.uniform(lo, hi - lo).cdf
        else:
            med = p.median if p.median is not None else float(b)
            cdf = st.lognorm(s=p.sigma, scale=med).cdf
        pv = float(st.kstest(v, cdf).pvalue)
        res.append(
            {"parameter": p.path, "dist": p.dist, "n": int(len(v)), "ks_p_value": pv, "flag": bool(pv < 1e-3)}
        )
    return res


def statistics_markdown(
    manifest_stats: dict[str, Any],
    leakage: dict[str, Any],
    checks: list[dict[str, Any]],
    rejections: dict[str, int],
) -> str:
    L = ["# Dataset statistics", ""]
    for s, d in manifest_stats["splits"].items():
        L += [f"## split `{s}`: {d['accepted']}/{d['runs']} accepted", ""]
        if d["parameters"]:
            L += ["| parameter | mean | std | min | p05 | p95 | max |", "|---|---|---|---|---|---|---|"]
            for k, v in d["parameters"].items():
                L.append(
                    f"| {k} | {v['mean']:.4g} | {v['std']:.4g} | {v['min']:.4g} | {v['p05']:.4g} | {v['p95']:.4g} | {v['max']:.4g} |"
                )
            L.append("")
        if d["results"]:
            keep = [
                k
                for k in d["results"]
                if k
                in (
                    "apogee_m",
                    "max_velocity_ms",
                    "max_acceleration_ms2",
                    "impact_speed_ms",
                    "flight_time_s",
                    "range_m",
                    "landing_range_m",
                )
            ]
            if keep:
                L += ["| result | mean | std | min | p05 | p95 | max |", "|---|---|---|---|---|---|---|"]
                for k in keep:
                    v = d["results"][k]
                    L.append(
                        f"| {k} | {v['mean']:.4g} | {v['std']:.4g} | {v['min']:.4g} | {v['p05']:.4g} | {v['p95']:.4g} | {v['max']:.4g} |"
                    )
                L.append("")
    L += ["## Leakage checks", "", "```", str(leakage), "```", ""]
    if checks:
        L += [
            "## Declared-vs-sampled marginals (KS)",
            "",
            "| parameter | dist | n | p | flag |",
            "|---|---|---|---|---|",
        ]
        for c in checks:
            L.append(
                f"| {c['parameter']} | {c['dist']} | {c['n']} | {c['ks_p_value']:.3g} | {'**FLAG**' if c['flag'] else ''} |"
            )
        L.append("")
    if rejections:
        L += ["## Rejection reasons", ""] + [f"* {k}: {v}" for k, v in rejections.items()] + [""]
    return "\n".join(L)
