#!/usr/bin/env python3
"""
Trajektorie wariantow EKF z CSV replay (--csv) na jednym wykresie XY.

  components/flora_nav/host/replay --csv traj.csv nagranie.log
  tools/rec/plot.py traj.csv [--tags nagranie.log] [-o traj.png]

Wymaga matplotlib (jedyne narzedzie z zaleznoscia - tylko do wykresow).
"""
import argparse
import csv


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv")
    p.add_argument("--tags", help="nagranie .log - paliki z linii M")
    p.add_argument("--only", help="warianty po przecinku, np. full,odom,head")
    p.add_argument("-o", "--out", help="zapis do pliku zamiast okna")
    a = p.parse_args()

    import matplotlib
    if a.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(a.csv) as f:
        rows = list(csv.DictReader(f))
    names = [k[:-2] for k in rows[0] if k.endswith("_x")]
    if a.only:
        names = [n for n in names if n in a.only.split(",")]

    fig, ax = plt.subplots(figsize=(8, 8))
    for n in names:
        xs = [float(r[f"{n}_x"]) / 1000 for r in rows]
        ys = [float(r[f"{n}_y"]) / 1000 for r in rows]
        ax.plot(xs, ys, label=n, linewidth=1.2)
    if a.tags:
        with open(a.tags) as f:
            for line in f:
                if line.startswith("M,"):
                    _, tid, x, y = line.strip().split(",")
                    ax.plot(float(x) / 1000, float(y) / 1000, "ks")
                    ax.annotate(tid, (float(x) / 1000, float(y) / 1000),
                                textcoords="offset points", xytext=(5, 5))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    if a.out:
        fig.savefig(a.out, dpi=150, bbox_inches="tight")
        print(a.out)
    else:
        plt.show()


if __name__ == "__main__":
    main()
