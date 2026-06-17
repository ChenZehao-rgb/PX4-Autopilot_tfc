#!/usr/bin/env python3
"""Generate static C++ BET aerodynamic and inverse-lookup tables."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np

RHO = 1.225
MU = 1.81e-5
R_ACTUAL = 0.2286
N_B = 2
R_0 = 1.5
N_PSI = 72
MAX_ITER = 80
LAMBDA_TOL = 1e-6
DAMPING = 0.5
CT_INIT_GUESS = 0.011
K_X_FIXED = 1.2
MU_T_SKEW_TOL = 1e-9
POST_STALL_BLEND_DEG = 5.0
POST_STALL_CD90 = 2.0
POST_STALL_CD_MIN = 0.02

RPM_GRID = np.arange(0.0, 6000.0 + 1e-6, 100.0)
FREESTREAM_GRID = np.linspace(0.0, 10.17, 11)
LIFT_ALPHA_GRID = np.arange(0.0, 90.0 + 1e-6, 15.0)


def fmt_float(value: float) -> str:
    text = f"{float(value):.9g}"

    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"

    return f"{text}f"


def write_array(name: str, values: list[float], per_line: int = 8) -> str:
    lines = [f"const float {name}[] = {{"]

    for i in range(0, len(values), per_line):
        chunk = ", ".join(fmt_float(v) for v in values[i:i + per_line])
        lines.append(f"\t{chunk},")

    lines.append("};")
    return "\n".join(lines)


def write_header(path: Path, text: str) -> None:
    path.write_text(text, encoding="utf-8")


def read_sections(path: Path) -> tuple[list[dict], list[str]]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    radius0 = float(rows[0]["radius"])
    r_over_r0 = float(rows[0]["r_over_R"])
    r_model = radius0 / r_over_r0
    scale = R_ACTUAL / r_model

    sections = []
    airfoil_order = []

    for row in rows:
        airfoil = row["dat_file"].replace(".dat", "")
        airfoil_order.append(airfoil)
        r_over_r = float(row["r_over_R"])

        sections.append({
            "r_over_R": r_over_r,
            "radius_m": r_over_r * R_ACTUAL,
            "chord_m": float(row["chord"]) * scale,
            "theta_rad": math.radians(90.0 - float(row["beta_deg"])),
            "airfoil": airfoil,
        })

    return sections, airfoil_order


def read_cl_cd_database(path: Path, airfoil_order: list[str]) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))

    alpha_grid = np.array(sorted({float(row["alpha"]) for row in rows}), dtype=float)
    re_grid = np.array(sorted({float(row["re"]) for row in rows}), dtype=float)
    airfoil_index = {name: i for i, name in enumerate(airfoil_order)}
    alpha_index = {value: i for i, value in enumerate(alpha_grid)}
    re_index = {value: i for i, value in enumerate(re_grid)}

    cl = np.zeros((len(airfoil_order), len(alpha_grid), len(re_grid)), dtype=float)
    cd = np.zeros_like(cl)
    seen = np.zeros_like(cl, dtype=bool)

    for row in rows:
        airfoil = row["airfoil"]

        if airfoil not in airfoil_index:
            continue

        ai = airfoil_index[airfoil]
        alpha_i = alpha_index[float(row["alpha"])]
        re_i = re_index[float(row["re"])]
        cl[ai, alpha_i, re_i] = float(row["cl"])
        cd[ai, alpha_i, re_i] = float(row["cd"])
        seen[ai, alpha_i, re_i] = True

    if not np.all(seen):
        missing = int(np.size(seen) - np.count_nonzero(seen))
        raise RuntimeError(f"missing {missing} Cl/Cd table entries")

    return alpha_grid, re_grid, cl, cd


def smoothstep(x: np.ndarray) -> np.ndarray:
    x_clip = np.clip(x, 0.0, 1.0)
    return x_clip * x_clip * (3.0 - 2.0 * x_clip)


def interpolate_2d(table: np.ndarray, alpha_grid: np.ndarray, re_grid: np.ndarray,
                   alpha_deg: np.ndarray, reynolds: np.ndarray) -> np.ndarray:
    alpha = np.clip(alpha_deg, alpha_grid[0], alpha_grid[-1])
    re = np.clip(reynolds, re_grid[0], re_grid[-1])

    ai = np.searchsorted(alpha_grid, alpha, side="right") - 1
    ri = np.searchsorted(re_grid, re, side="right") - 1
    ai = np.clip(ai, 0, len(alpha_grid) - 2)
    ri = np.clip(ri, 0, len(re_grid) - 2)

    aw = (alpha - alpha_grid[ai]) / (alpha_grid[ai + 1] - alpha_grid[ai])
    rw = (re - re_grid[ri]) / (re_grid[ri + 1] - re_grid[ri])

    v00 = table[ai, ri]
    v10 = table[ai + 1, ri]
    v01 = table[ai, ri + 1]
    v11 = table[ai + 1, ri + 1]

    v0 = v00 + aw * (v10 - v00)
    v1 = v01 + aw * (v11 - v01)
    return v0 + rw * (v1 - v0)


def interpolate_3d(table: np.ndarray, airfoil_idx: np.ndarray, alpha_grid: np.ndarray, re_grid: np.ndarray,
                   alpha_deg: np.ndarray, reynolds: np.ndarray) -> np.ndarray:
    alpha = np.clip(alpha_deg, alpha_grid[0], alpha_grid[-1])
    re = np.clip(reynolds, re_grid[0], re_grid[-1])

    ai = np.searchsorted(alpha_grid, alpha, side="right") - 1
    ri = np.searchsorted(re_grid, re, side="right") - 1
    ai = np.clip(ai, 0, len(alpha_grid) - 2)
    ri = np.clip(ri, 0, len(re_grid) - 2)

    aw = (alpha - alpha_grid[ai]) / (alpha_grid[ai + 1] - alpha_grid[ai])
    rw = (re - re_grid[ri]) / (re_grid[ri + 1] - re_grid[ri])

    v00 = table[airfoil_idx, ai, ri]
    v10 = table[airfoil_idx, ai + 1, ri]
    v01 = table[airfoil_idx, ai, ri + 1]
    v11 = table[airfoil_idx, ai + 1, ri + 1]

    v0 = v00 + aw * (v10 - v00)
    v1 = v01 + aw * (v11 - v01)
    return v0 + rw * (v1 - v0)


def lookup_cl_cd(table_cl: np.ndarray, table_cd: np.ndarray, alpha_grid: np.ndarray, re_grid: np.ndarray,
                 airfoil_i: int, alpha_deg: np.ndarray, reynolds: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    re_clamped = np.clip(reynolds, re_grid[0], re_grid[-1])
    cl_table = interpolate_2d(table_cl[airfoil_i], alpha_grid, re_grid, alpha_deg, re_clamped)
    cd_table = interpolate_2d(table_cd[airfoil_i], alpha_grid, re_grid, alpha_deg, re_clamped)
    cd0 = interpolate_2d(table_cd[airfoil_i], alpha_grid, re_grid, np.zeros_like(alpha_deg), re_clamped)

    alpha_rad = np.deg2rad(alpha_deg)
    sin_a = np.sin(alpha_rad)
    cos_a = np.cos(alpha_rad)
    cd0_eff = np.maximum(cd0, POST_STALL_CD_MIN)
    cl_stall = POST_STALL_CD90 * sin_a * cos_a
    cd_stall = cd0_eff * cos_a**2 + POST_STALL_CD90 * sin_a**2

    low_weight = smoothstep((alpha_grid[0] - alpha_deg) / POST_STALL_BLEND_DEG)
    high_weight = smoothstep((alpha_deg - alpha_grid[-1]) / POST_STALL_BLEND_DEG)
    post_stall_weight = np.maximum(low_weight, high_weight)

    cl = (1.0 - post_stall_weight) * cl_table + post_stall_weight * cl_stall
    cd = (1.0 - post_stall_weight) * cd_table + post_stall_weight * cd_stall
    return cl, cd


def lookup_cl_cd_all(table_cl: np.ndarray, table_cd: np.ndarray, alpha_grid: np.ndarray, re_grid: np.ndarray,
                     airfoil_idx: np.ndarray, alpha_deg: np.ndarray,
                     reynolds: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    re_clamped = np.clip(reynolds, re_grid[0], re_grid[-1])
    cl_table = interpolate_3d(table_cl, airfoil_idx, alpha_grid, re_grid, alpha_deg, re_clamped)
    cd_table = interpolate_3d(table_cd, airfoil_idx, alpha_grid, re_grid, alpha_deg, re_clamped)
    cd0 = interpolate_3d(table_cd, airfoil_idx, alpha_grid, re_grid, np.zeros_like(alpha_deg), re_clamped)

    alpha_rad = np.deg2rad(alpha_deg)
    sin_a = np.sin(alpha_rad)
    cos_a = np.cos(alpha_rad)
    cd0_eff = np.maximum(cd0, POST_STALL_CD_MIN)
    cl_stall = POST_STALL_CD90 * sin_a * cos_a
    cd_stall = cd0_eff * cos_a**2 + POST_STALL_CD90 * sin_a**2

    low_weight = smoothstep((alpha_grid[0] - alpha_deg) / POST_STALL_BLEND_DEG)
    high_weight = smoothstep((alpha_deg - alpha_grid[-1]) / POST_STALL_BLEND_DEG)
    post_stall_weight = np.maximum(low_weight, high_weight)

    cl_out = (1.0 - post_stall_weight) * cl_table + post_stall_weight * cl_stall
    cd_out = (1.0 - post_stall_weight) * cd_table + post_stall_weight * cd_stall
    return cl_out, cd_out


def prepare_section_arrays(sections: list[dict], airfoil_order: list[str]) -> dict[str, np.ndarray]:
    airfoil_index = {name: i for i, name in enumerate(airfoil_order)}
    return {
        "r_over_R": np.array([section["r_over_R"] for section in sections], dtype=float)[:, None],
        "radius_m": np.array([section["radius_m"] for section in sections], dtype=float)[:, None],
        "chord_m": np.array([section["chord_m"] for section in sections], dtype=float)[:, None],
        "theta_rad": np.array([section["theta_rad"] for section in sections], dtype=float)[:, None],
        "airfoil_i": np.array([airfoil_index[section["airfoil"]] for section in sections], dtype=int)[:, None],
        "radius_1d": np.array([section["radius_m"] for section in sections], dtype=float),
    }


def compute_lift_n(rpm: float, freestream_m_s: float, alpha_deg: float, section_data: dict[str, np.ndarray],
                   alpha_grid: np.ndarray, re_grid: np.ndarray, cl: np.ndarray, cd: np.ndarray,
                   airfoil_index=None) -> float:
    del airfoil_index

    if rpm <= 0.0:
        return 0.0

    omega = rpm * math.pi / 30.0
    alpha_p = math.radians(alpha_deg)
    omega_a = freestream_m_s / R_0
    mu_a = freestream_m_s * math.cos(alpha_p) / (omega * R_ACTUAL + 1e-12)
    mu_t = freestream_m_s * math.sin(alpha_p) / (omega * R_ACTUAL + 1e-12)
    lambda_i_0 = math.sqrt(max(CT_INIT_GUESS / 2.0, 1e-8))
    k_x = K_X_FIXED if abs(mu_t) > MU_T_SKEW_TOL else 0.0
    psi_vec = np.linspace(0.0, 2.0 * math.pi, N_PSI, endpoint=False)
    cos_psi = np.cos(psi_vec)[None, :]
    sin_psi = np.sin(psi_vec)[None, :]
    radius = section_data["radius_m"]
    chord = section_data["chord_m"]
    theta = section_data["theta_rad"]
    r_over_R = section_data["r_over_R"]
    airfoil_i = section_data["airfoil_i"]
    thrust_total = 0.0

    for _ in range(MAX_ITER):
        v_free = omega_a * (R_0 - radius * cos_psi)
        lambda_i = lambda_i_0 * (1.0 + k_x * r_over_R * sin_psi)
        lambda_i = np.maximum(lambda_i, 0.0)
        induced_velocity = lambda_i * omega * R_ACTUAL
        v_a = v_free * math.cos(alpha_p) + induced_velocity
        v_t = omega * radius - v_free * math.sin(alpha_p) * cos_psi
        v_r = np.sqrt(v_a * v_a + v_t * v_t + 1e-12)
        phi = np.arctan2(v_a, v_t)
        alpha_b_deg = np.rad2deg(theta - phi)
        reynolds = RHO * v_r * chord / MU
        cl_i, cd_i = lookup_cl_cd_all(cl, cd, alpha_grid, re_grid, airfoil_i, alpha_b_deg, reynolds)
        dthrust_dr = np.mean(0.5 * RHO * v_r**2 * (cl_i * np.cos(phi) - cd_i * np.sin(phi)) * chord, axis=1)
        thrust_blade = np.trapz(dthrust_dr, section_data["radius_1d"])
        thrust_total = N_B * thrust_blade
        ct_raw = thrust_total / (RHO * math.pi * R_ACTUAL**2 * (omega * R_ACTUAL)**2)
        ct = max(ct_raw, 0.0)
        lambda_0 = mu_a + lambda_i_0
        denom = math.hypot(mu_t, lambda_0)
        lambda_i_0_new = ct / (2.0 * denom) if denom > 1e-8 else math.sqrt(max(ct / 2.0, 0.0))
        err = abs(lambda_i_0_new - lambda_i_0)
        lambda_i_0 = DAMPING * lambda_i_0 + (1.0 - DAMPING) * lambda_i_0_new

        if err < LAMBDA_TOL:
            break

    return float(thrust_total)


def generate_lift_lookup(sections: list[dict], alpha_grid: np.ndarray, re_grid: np.ndarray,
                         cl: np.ndarray, cd: np.ndarray, airfoil_order: list[str]) -> np.ndarray:
    airfoil_index = {name: i for i, name in enumerate(airfoil_order)}
    section_data = prepare_section_arrays(sections, airfoil_order)
    table = np.zeros((len(LIFT_ALPHA_GRID), len(FREESTREAM_GRID), len(RPM_GRID)), dtype=float)
    total = table.size
    done = 0

    for alpha_i, alpha_deg in enumerate(LIFT_ALPHA_GRID):
        for freestream_i, freestream_m_s in enumerate(FREESTREAM_GRID):
            for rpm_i, rpm in enumerate(RPM_GRID):
                table[alpha_i, freestream_i, rpm_i] = compute_lift_n(
                    float(rpm),
                    float(freestream_m_s),
                    float(alpha_deg),
                    section_data,
                    alpha_grid,
                    re_grid,
                    cl,
                    cd,
                    airfoil_index,
                )
                done += 1

                if done % 250 == 0 or done == total:
                    print(f"  lift table {done}/{total}", flush=True)

    return table


def check_monotonic_lift_table(table: np.ndarray) -> int:
    warnings = 0

    for alpha_i, alpha_deg in enumerate(LIFT_ALPHA_GRID):
        for freestream_i, freestream_m_s in enumerate(FREESTREAM_GRID):
            diffs = np.diff(table[alpha_i, freestream_i])

            if np.any(diffs < -1e-4):
                warnings += 1
                print(
                    "WARNING: non-monotonic lift profile "
                    f"alpha={alpha_deg:.1f} deg freestream={freestream_m_s:.3f} m/s"
                )

    return warnings


def generate_aero_files(output_dir: Path, sections: list[dict], airfoil_order: list[str],
                        alpha_grid: np.ndarray, re_grid: np.ndarray, cl: np.ndarray, cd: np.ndarray) -> None:
    header = f"""\
/****************************************************************************
 *
 *   Generated by tools/generate_bet_tables.py. Do not edit by hand.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>

namespace thrust_feedback_control
{{
namespace bet
{{

struct BetSection {{
\tfloat r_over_R;
\tfloat radius_m;
\tfloat chord_m;
\tfloat theta_rad;
\tuint16_t airfoil_index;
}};

static constexpr std::size_t kNumBetSections = {len(sections)};
static constexpr std::size_t kNumBetAirfoils = {len(airfoil_order)};
static constexpr std::size_t kNumBetAlpha = {len(alpha_grid)};
static constexpr std::size_t kNumBetRe = {len(re_grid)};

extern const BetSection kBetSections[kNumBetSections];
extern const float kBetAlphaGridDeg[kNumBetAlpha];
extern const float kBetReGrid[kNumBetRe];
extern const float kBetClTable[kNumBetAirfoils * kNumBetAlpha * kNumBetRe];
extern const float kBetCdTable[kNumBetAirfoils * kNumBetAlpha * kNumBetRe];

}} // namespace bet
}} // namespace thrust_feedback_control
"""
    source_lines = [
        "/****************************************************************************",
        " *",
        " *   Generated by tools/generate_bet_tables.py. Do not edit by hand.",
        " *",
        " ****************************************************************************/",
        "",
        '#include "bet_aero_data.hpp"',
        "",
        "namespace thrust_feedback_control",
        "{",
        "namespace bet",
        "{",
        "",
        "const BetSection kBetSections[] = {",
    ]

    airfoil_index = {name: i for i, name in enumerate(airfoil_order)}

    for section in sections:
        source_lines.append(
            "\t{"
            f"{fmt_float(section['r_over_R'])}, "
            f"{fmt_float(section['radius_m'])}, "
            f"{fmt_float(section['chord_m'])}, "
            f"{fmt_float(section['theta_rad'])}, "
            f"{airfoil_index[section['airfoil']]}"
            "},"
        )

    source_lines.extend([
        "};",
        "",
        write_array("kBetAlphaGridDeg", alpha_grid.tolist()),
        "",
        write_array("kBetReGrid", re_grid.tolist()),
        "",
        write_array("kBetClTable", cl.reshape(-1).tolist()),
        "",
        write_array("kBetCdTable", cd.reshape(-1).tolist()),
        "",
        "} // namespace bet",
        "} // namespace thrust_feedback_control",
        "",
    ])

    write_header(output_dir / "bet_aero_data.hpp", header)
    write_header(output_dir / "bet_aero_data.cpp", "\n".join(source_lines))


def generate_lift_files(output_dir: Path, lift_table: np.ndarray) -> None:
    header = f"""\
/****************************************************************************
 *
 *   Generated by tools/generate_bet_tables.py. Do not edit by hand.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>

namespace thrust_feedback_control
{{
namespace bet
{{

static constexpr std::size_t kNumBetLiftRpm = {len(RPM_GRID)};
static constexpr std::size_t kNumBetLiftFreestream = {len(FREESTREAM_GRID)};
static constexpr std::size_t kNumBetLiftAlpha = {len(LIFT_ALPHA_GRID)};

extern const float kBetLiftRpmGrid[kNumBetLiftRpm];
extern const float kBetLiftFreestreamGrid[kNumBetLiftFreestream];
extern const float kBetLiftAlphaGridDeg[kNumBetLiftAlpha];
extern const float kBetLiftLookupN[kNumBetLiftAlpha * kNumBetLiftFreestream * kNumBetLiftRpm];

}} // namespace bet
}} // namespace thrust_feedback_control
"""
    source = "\n".join([
        "/****************************************************************************",
        " *",
        " *   Generated by tools/generate_bet_tables.py. Do not edit by hand.",
        " *",
        " ****************************************************************************/",
        "",
        '#include "bet_lift_lookup.hpp"',
        "",
        "namespace thrust_feedback_control",
        "{",
        "namespace bet",
        "{",
        "",
        write_array("kBetLiftRpmGrid", RPM_GRID.tolist()),
        "",
        write_array("kBetLiftFreestreamGrid", FREESTREAM_GRID.tolist()),
        "",
        write_array("kBetLiftAlphaGridDeg", LIFT_ALPHA_GRID.tolist()),
        "",
        write_array("kBetLiftLookupN", lift_table.reshape(-1).tolist()),
        "",
        "} // namespace bet",
        "} // namespace thrust_feedback_control",
        "",
    ])

    write_header(output_dir / "bet_lift_lookup.hpp", header)
    write_header(output_dir / "bet_lift_lookup.cpp", source)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--section-csv",
        type=Path,
        default=Path("/home/sia/bemt_model/bemt_sections/section_summary.csv"),
    )
    parser.add_argument(
        "--cl-cd-csv",
        type=Path,
        default=Path("/home/sia/bemt_model/lookup_table/cl_cd_database.csv"),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    args = parser.parse_args()

    print("Loading BET sections...", flush=True)
    sections, airfoil_order = read_sections(args.section_csv)
    print(f"  {len(sections)} sections", flush=True)

    print("Loading Cl/Cd database...", flush=True)
    alpha_grid, re_grid, cl, cd = read_cl_cd_database(args.cl_cd_csv, airfoil_order)
    print(f"  alpha={len(alpha_grid)} Re={len(re_grid)} airfoils={len(airfoil_order)}", flush=True)

    print("Generating C++ aero tables...", flush=True)
    generate_aero_files(args.output_dir, sections, airfoil_order, alpha_grid, re_grid, cl, cd)

    print("Generating lift lookup table...", flush=True)
    lift_table = generate_lift_lookup(sections, alpha_grid, re_grid, cl, cd, airfoil_order)
    warnings = check_monotonic_lift_table(lift_table)
    generate_lift_files(args.output_dir, lift_table)
    print(f"Generated BET tables in {args.output_dir}", flush=True)
    print(f"Monotonic warnings: {warnings}", flush=True)


if __name__ == "__main__":
    main()
