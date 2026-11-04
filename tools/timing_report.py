#!/usr/bin/env python3
"""여러 회차의 시간 기록을 모아 표로 정리한다.

    python3 tools/timing_report.py ~/edu_pick_runs/20261104_*      # 회차 폴더들
    python3 tools/timing_report.py ~/edu_pick_runs/* -o report.md

각 회차 폴더 = pick_place 가 남긴 timing.csv · detector_frames.csv · summary.txt
출력(마크다운):
  1) 회차별 분류 합계 [s]
  2) 분류별 평균·표준편차 (회차 기준) 와 비율
  3) 단계(1~12)별 평균 시간 — 어디가 오래 걸리나
  4) 인식 노드 단계별 처리 시간 [ms]
"""
import argparse
import csv
import os
import statistics as st
import sys

ORDER = ["로봇동작", "그리퍼", "인식대기", "경로계획", "놓기확인", "장면갱신", "위치계산"]


def load(d):
    tl = list(csv.DictReader(open(os.path.join(d, "timing.csv"))))
    for r in tl:
        r["dur"] = float(r["dur"])
    det = []
    p = os.path.join(d, "detector_frames.csv")
    if os.path.exists(p):
        det = list(csv.DictReader(open(p)))
    return tl, det


def ms(v):
    return f"{v:.1f}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()
    runs = [d for d in a.runs if os.path.exists(os.path.join(d, "timing.csv"))]
    if not runs:
        sys.exit("timing.csv 가 있는 폴더가 없습니다")
    data = {os.path.basename(os.path.normpath(d)): load(d) for d in sorted(runs)}
    L = [f"# Pick and place 시간 분석 — 회차 {len(data)}개", ""]

    # 1) 회차별
    cats = [c for c in ORDER if any(r["category"] == c for tl, _ in data.values() for r in tl)]
    L += ["## 1. 회차별 분류 합계 [s] (사람 확인 대기 제외)", "",
          "| 회차 | 물체 수 | " + " | ".join(cats) + " | 합계 | 물체당 |",
          "|---|---|" + "---|" * (len(cats) + 2)]
    per_cat = {c: [] for c in cats}
    per_obj = []
    for name, (tl, _) in data.items():
        seq = [r for r in tl if r["category"] != "사람확인"]
        n_obj = len({r["cycle"] for r in seq if r["cycle"] not in ("0", 0)})
        sums = {c: sum(r["dur"] for r in seq if r["category"] == c) for c in cats}
        tot = sum(sums.values())
        for c in cats:
            per_cat[c].append(sums[c])
        per_obj.append(tot / max(n_obj, 1))
        L.append(f"| {name} | {n_obj} | " + " | ".join(f"{sums[c]:.1f}" for c in cats) +
                 f" | {tot:.1f} | {tot / max(n_obj, 1):.1f} |")

    # 2) 분류별 평균
    grand = sum(sum(v) for v in per_cat.values())
    L += ["", "## 2. 분류별 (회차 평균 ± 표준편차)", "", "| 분류 | 평균 [s] | ± | 비율 |", "|---|---|---|---|"]
    for c in cats:
        v = per_cat[c]
        L.append(f"| {c} | {st.mean(v):.1f} | {st.stdev(v) if len(v) > 1 else 0:.1f} | "
                 f"{100 * sum(v) / max(grand, 1e-9):.0f} % |")
    L.append(f"| **물체 1개당** | **{st.mean(per_obj):.1f}** | {st.stdev(per_obj) if len(per_obj) > 1 else 0:.1f} | |")

    # 3) 단계별
    steps = {}
    for tl, _ in data.values():
        for r in tl:
            if r["category"] == "사람확인" or r["cycle"] in ("0", 0):
                continue
            key = (r["step"], r["category"])
            steps.setdefault(key, []).append(r["dur"])
    L += ["", "## 3. 단계별 평균 [s] (물체 사이클 안, 같은 단계 여러 번이면 각각)", "",
          "| 단계 | 분류 | 평균 | 최대 | 횟수 |", "|---|---|---|---|---|"]

    def step_key(k):
        s = k[0].replace("+", ".5")
        try:
            return (float(s), k[1])
        except ValueError:
            return (99.0, k[1])
    for k in sorted(steps, key=step_key):
        v = steps[k]
        L.append(f"| {k[0]} | {k[1]} | {st.mean(v):.2f} | {max(v):.2f} | {len(v)} |")

    # 4) 인식 노드
    stages = []
    for _, det in data.values():
        for row in det[:1]:
            for k in row:
                if k not in ("stamp", "objects") and k not in stages:
                    stages.append(k)
    if stages:
        L += ["", "## 4. 인식 노드 단계별 처리 시간 [ms] (모든 프레임)", "",
              "| 단계 | 평균 | 중앙 | 95 % | 최대 |", "|---|---|---|---|---|"]
        for k in stages:
            v = sorted(float(row[k]) for _, det in data.values() for row in det if row.get(k))
            if not v:
                continue
            p95 = v[min(len(v) - 1, int(0.95 * len(v)))]
            L.append(f"| {k} | {ms(st.mean(v))} | {ms(st.median(v))} | {ms(p95)} | {ms(v[-1])} |")
        L += ["", "※ '물체판별' = 덩어리마다 모양·크기·방향·위치를 계산하는 단계 — 학습 모델이 없는 이 실습의 '추론'에 해당.",
              "※ '카메라→처리완료' = 점군 시각부터 결과 발행까지(카메라 전송·대기 포함)."]
    text = "\n".join(L) + "\n"
    if a.out:
        open(a.out, "w").write(text)
        print("저장:", a.out)
    print(text)


if __name__ == "__main__":
    main()
