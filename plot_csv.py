#!/usr/bin/env python3
"""Plot captures saved by the DSO-510 custom firmware (CSV files from the scope's USB drive).

    python3 plot_csv.py CAP0001.CSV                 # one capture
    python3 plot_csv.py CAP0001.CSV CAP0002.CSV     # overlay several
    python3 plot_csv.py /media/$USER/DSO-510/CSV/*.CSV --save plot.png
    python3 plot_csv.py CAP0001.CSV --codes         # raw ADC codes instead of volts

Needs matplotlib (pip install matplotlib). The file format is described in README.md:
'#' header lines (sample rate, V/div, coupling, probe, trigger, ...), then rows of
time_s,volts,adc_code with t = 0 at the trigger point.
"""
import argparse
import csv
import os
import sys


def read_capture(path):
    """Return (header dict, times in s, volts, adc codes) of one CSV file."""
    header, t, v, c = {}, [], [], []
    with open(path, newline="") as f:
        for line in f:
            if not line.startswith("#"):
                break                                    # the column-name line
            text = line[1:].strip()
            if "=" in text:
                key, _, value = text.partition("=")
                header[key.strip()] = value.strip()
            elif text:
                header.setdefault("firmware", text)
        for row in csv.reader(f):
            if len(row) < 3:
                continue
            t.append(float(row[0]))
            v.append(float(row[1]))
            c.append(int(row[2]))
    if not t:
        raise ValueError("no samples found")
    return header, t, v, c


def time_unit(span_s):
    """Pick a readable unit for a time span: (scale factor, label)."""
    for scale, label in ((1e9, "ns"), (1e6, "us"), (1e3, "ms")):
        if span_s * scale < 2000:
            return scale, label
    return 1.0, "s"


def describe(header):
    """Short summary of the capture settings for the legend / title."""
    parts = []
    rate = header.get("sample_rate_hz")
    if rate:
        r = float(rate)
        parts.append("%g MS/s" % (r / 1e6) if r >= 1e6 else "%g kS/s" % (r / 1e3) if r >= 1e3 else "%g S/s" % r)
    vdiv = header.get("volts_per_div")
    if vdiv:
        parts.append("%g V/div" % float(vdiv))
    for key in ("coupling",):
        if key in header:
            parts.append(header[key])
    if header.get("probe") == "10":
        parts.append("x10 probe")
    return ", ".join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot DSO-510 CSV captures.")
    ap.add_argument("files", nargs="+", help="CSV files saved by the scope")
    ap.add_argument("--codes", action="store_true", help="plot raw ADC codes instead of volts")
    ap.add_argument("--save", metavar="IMAGE", help="write the plot to an image file (png, svg, pdf) instead of a window")
    ap.add_argument("--title", help="plot title")
    args = ap.parse_args(argv)

    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("matplotlib is needed: pip install matplotlib")

    captures = []
    for path in args.files:
        try:
            captures.append((path,) + read_capture(path))
        except (OSError, ValueError) as e:
            print("skipping %s: %s" % (path, e), file=sys.stderr)
    if not captures:
        sys.exit("nothing to plot")

    span = max(max(abs(x) for x in t) for _, _, t, _, _ in captures)
    scale, unit = time_unit(span)

    fig, ax = plt.subplots(figsize=(10, 5))
    for path, header, t, v, c in captures:
        y = c if args.codes else v
        label = os.path.basename(path)
        info = describe(header)
        if info:
            label += "  (" + info + ")"
        if header.get("triggered") == "0":
            label += "  untriggered"
        ax.plot([x * scale for x in t], y, linewidth=1, label=label)

    # trigger point and level of the first capture
    header = captures[0][1]
    ax.axvline(0, color="orange", linestyle="--", linewidth=0.8)
    trig = header.get("trigger", "").split(",")
    if not args.codes and len(trig) == 3:
        try:
            ax.axhline(float(trig[2]), color="orange", linestyle=":", linewidth=0.8)
        except ValueError:
            pass

    ax.set_xlabel("time (%s), 0 = trigger" % unit)
    ax.set_ylabel("ADC code" if args.codes else "volts (V)")
    ax.set_title(args.title or (os.path.basename(captures[0][0]) if len(captures) == 1 else "%d captures" % len(captures)))
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize="small")
    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=120)
        print("wrote", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
