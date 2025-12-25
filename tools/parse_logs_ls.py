#!/usr/bin/env python3
import re
import csv
import glob
from pathlib import Path

logs = sorted(glob.glob("logs/big_runs/*.log"))
out = Path("benchmark_results/analysis_summary_ls.csv")
rows = []
for p in logs:
    txt = open(p, errors="ignore").read()
    fn = Path(p).name
    m_inst = re.search(r"Reading instance:\s*(\S+)", txt)
    instance = m_inst.group(1) if m_inst else fn.replace(.log,)
    m = re.match(r"U_(\d+)_([0-9.]+)_Num_(\d+)_pd(?:\.log)?", instance)
    if not m:
        m2 = re.match(r"(U_\d+_[0-9.]+_Num_\d+)_pd\.log", fn)
        if m2:
            instbase = m2.group(1) + "_pd.txt"
            m = re.match(r"U_(\d+)_([0-9.]+)_Num_(\d+)_pd\.txt", instbase)
    if m:
        customers = int(m.group(1))
        beta = float(m.group(2))
        runno = int(m.group(3))
    else:
        customers = 
        beta = 
        runno = 
    m_cust = re.search(r"Loaded \d+ nodes, (\d+) customers", txt)
    if m_cust:
        customers = int(m_cust.group(1))
    m_fin = re.search(r"Final best cost:\s*([0-9]+\.?[0-9]*)", txt)
    makespan = float(m_fin.group(1)) if m_fin else 
    m_before = re.search(r"\[ADAPTIVE LS\] Initial C_max:\s*([0-9]+\.?[0-9]*)", txt)
    m_after = re.search(r"\[ADAPTIVE LS\] Final C_max:\s*([0-9]+\.?[0-9]*)", txt)
    before = float(m_before.group(1)) if m_before else 
    after = float(m_after.group(1)) if m_after else 
    res_count = len(re.findall(r"Resupply Trip", txt))
    feasible = YES if re.search(r"Feasible:\s*YES", txt) else (NO if re.search(r"Feasible:\s*NO", txt) else )
    rows.append({
        Instance: instance,
        Beta: beta,
        Run: runno,
        Customers: customers,
        Makespan: makespan,
        Runtime_sec: ,
        Resupply_Events: res_count,
        Feasible: feasible,
        C_before_LS: before,
        C_after_LS: after,
    })

out.parent.mkdir(parents=True, exist_ok=True)
with out.open(w, newline=) as f:
    w = csv.DictWriter(f, fieldnames=[Instance,Beta,Run,Customers,Makespan,Runtime_sec,Resupply_Events,Feasible,C_before_LS,C_after_LS])
    w.writeheader()
    for r in rows:
        w.writerow(r)
print(f"Wrote {out} with {len(rows)} rows")
