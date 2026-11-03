"""driveLogger session analysis: summary stats + corner extraction.
Usage: python3 analyze.py <session_dir>   -> prints summary, writes corners.csv
"""
import sys, glob, re
import numpy as np, pandas as pd

G = 9.81

def load(folder):
    fs = [f for f in glob.glob(f"{folder}/*.csv") if re.search(r"_(\d{3})_\d{4}-", f)]
    fs = sorted(fs, key=lambda f: int(re.search(r"_(\d{3})_\d{4}-", f).group(1)))
    d = pd.concat([pd.read_csv(f) for f in fs], ignore_index=True)
    d["impact"] = (d.accLat.abs() > 20) | (d.accLong.abs() > 20)
    d["yawS"] = d.yawRate.rolling(50, center=True, min_periods=1).mean()
    d["latG"] = d.vLong * d.yawRate / G
    d["lonG"] = d.accLong / G
    d["dist"] = (d.speed * d.dt).cumsum()
    return d

def summary(d):
    m = d[(d.speed > 3) & ~d.impact]
    b = m.brakeInput > 0.05
    c = m.latG.abs() > 0.3
    comb = np.sqrt(m.latG**2 + m.lonG**2)
    return {
        "rows": len(d), "hz": round(1 / d.dt.mean()), "total_s": round(d.dt.sum()), "moving_s": round(m.dt.sum()),
        "dist_km": round(m.speed.mul(m.dt).sum() / 1000, 2),
        "speed_kmh_p50/p90/max": tuple((m.speed * 3.6).quantile([.5, .9, 1]).round()),
        "latG_p90/p99": tuple(m.latG.abs().quantile([.9, .99]).round(2)),
        "brakeG_p50/p95/p99": tuple((-m.lonG[b]).quantile([.5, .95, .99]).round(2)),
        "accelG_p50/p99": tuple(m.lonG[m.throttleInput > .5].quantile([.5, .99]).round(2)),
        "combinedG_p99": round(comb.quantile(.99), 2), "mu": round(m.frictionCoef.median(), 2),
        "brake_pedal_p50/max": tuple(m.brakeInput[b].quantile([.5, 1]).round(2)),
        "trailbrake_share": round(m.dt[b & c].sum() / max(m.dt[b].sum(), 1e-9), 2),
        "corner_on_throttle_s/coast_s": (round(m.dt[(m.throttleInput > .3) & c].sum()), round(m.dt[(m.throttleInput < .05) & ~b & c].sum())),
        "impacts": int((d.impact.astype(int).diff() == 1).sum()),
        "gears": sorted(m.gear.unique().astype(int).tolist()),
        "heading_err_deg_p50/p90": tuple(np.degrees(m.roadHeadingErr.abs()).quantile([.5, .9]).round(1)),
        "latNorm_p05/p50/p95": tuple((m.roadLatOffset / m.roadHalfWidth.clip(lower=1)).quantile([.05, .5, .95]).round(2)),
        "elev_range_m": round(d.posZ.max() - d.posZ.min()),
    }

def corners(d, yaw_thr=0.08, min_angle=20, lookback_m=250):
    turning = (d.yawS.abs() > yaw_thr) & (d.speed > 3) & ~d.impact
    grp = (turning != turning.shift()).cumsum()
    rows = []
    for _, s in d[turning].groupby(grp[turning]):
        ang = np.degrees((s.yawS * s.dt).sum())
        if abs(ang) < min_angle or s.dt.sum() < 0.4:
            continue
        i0, i1 = s.index[0], s.index[-1]
        apex = s.speed.idxmin()
        pre = d.loc[(d.dist >= d.dist[i0] - lookback_m) & (d.index <= apex)]
        braking = pre[pre.brakeInput > 0.05]
        bp = braking.index[0] if len(braking) else None
        post = d.loc[apex:i1 + 2000]
        thr = post[post.throttleInput > 0.3]
        tp = thr.index[0] if len(thr) else None
        rows.append(dict(
            t=round(d.t[i0], 1), dir="L" if ang > 0 else "R", angle=round(abs(ang)),
            radius_m=round(float(np.median(s.speed / (s.yawS.abs() + 1e-6)))),
            v_entry_kmh=round(pre.speed.max() * 3.6), v_min_kmh=round(d.speed[apex] * 3.6),
            v_exit_kmh=round(d.speed.loc[i1:i1 + 400].max() * 3.6),
            brake_dist_before_apex_m=round(d.dist[apex] - d.dist[bp]) if bp is not None else None,
            peak_brake=round(braking.brakeInput.max(), 2) if len(braking) else 0,
            peak_brakeG=round(-braking.lonG.min(), 2) if len(braking) else 0,
            trail_brake_s=round(s.dt[s.brakeInput > 0.05].sum(), 2),
            throttle_on_after_apex_m=round(d.dist[tp] - d.dist[apex]) if tp is not None else None,
            peak_latG=round(s.latG.abs().max(), 2),
            lat_pos_at_apex=round(d.roadLatOffset[apex] / max(d.roadHalfWidth[apex], 1), 2),
            elev_change_m=round(d.posZ[i1] - d.posZ[i0], 1),
        ))
    return pd.DataFrame(rows)

if __name__ == "__main__":
    folder = sys.argv[1]
    d = load(folder)
    for k, v in summary(d).items():
        print(f"{k:32s} {v}")
    c = corners(d)
    c.to_csv(f"{folder}/corners.csv", index=False)
    print(f"\n{len(c)} corners -> {folder}/corners.csv")
    if len(c):
        print(pd.cut(c.angle, [20, 45, 70, 110, 150, 360]).value_counts().sort_index().to_string())
        print(pd.cut(c.radius_m, [0, 15, 30, 60, 120, 1e5]).value_counts().sort_index().to_string())
        print(c.describe().T[["mean", "50%", "min", "max"]].round(2).to_string())
