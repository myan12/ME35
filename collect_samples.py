import time
from machine import Pin, I2C

LEFT_SDA, LEFT_SCL = 21, 22
RIGHT_SDA, RIGHT_SCL = 26, 25
KNN_K = 5
DATA_FILE = "training_data_new.csv"

CLASSES = ["WHITE", "BLACK", "RED", "GREEN", "BLUE", "EDGE", "TURN"]
COLORS = ("RED", "GREEN", "BLUE")
NOT_COLOR = ("WHITE", "BLACK", "TURN")    # surfaces that must never be read as a color
SAMPLES_PER_SENSOR = {"WHITE": 20, "BLACK": 20, "RED": 20, "GREEN": 20, "BLUE": 20,
                      "EDGE": 30, "TURN": 60}
INTERVAL_MS = 100        # time between samples (sensor updates every 40 ms)
HINTS = {
    "WHITE": "Plain white paper. Move around a bit.",
    "BLACK": "The black tape. Move along/across the middle of the tape.",
    "RED": "The RED patch. Move around inside the patch.",
    "GREEN": "The GREEN patch. Move around inside the patch.",
    "BLUE": "The BLUE patch. Move around inside the patch.",
    "EDGE": "SLIDE the sensor across the tape edge (white<->black) and across the borders of the\n"
            "colored patches (white<->patch, black<->patch), partly covering it.",
    "TURN": "Put the ROBOT on the track and push it slowly by hand around the turns and corners,\n"
            "the way it drives (sensors on, half on and off the tape). Stay away from colored patches.\n"
            "Do a few different corners; run this step more than once if your track has many.",
}


class VEML6040:
    ADDR = 0x10

    def __init__(self, i2c):
        self.i2c = i2c
        self.i2c.writeto_mem(self.ADDR, 0x00, b"\x00\x00")  # on, 40 ms integration

    def read(self):
        out = []
        for reg in (0x08, 0x09, 0x0A, 0x0B):
            d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
            out.append(d[0] | (d[1] << 8))
        return out  # [R, G, B, W]


sensors = {
    "left": VEML6040(I2C(0, sda=Pin(LEFT_SDA), scl=Pin(LEFT_SCL), freq=400000)),
    "right": VEML6040(I2C(1, sda=Pin(RIGHT_SDA), scl=Pin(RIGHT_SCL), freq=400000)),
}

white = {}
samples = []   # (label, r, g, b, w), all normalized to white paper


# ---------- features (main.py uses these exact two functions) ----------
min_bright = 0.05        # r+g+b below this = too dark to judge hue. Learned from BLACK vs. colors
                         # in update_model() and saved for main.py.


def chroma(r, g, b):
    """Hue only: r, g, b as fractions of their sum (brightness removed). None if too dark."""
    s = r + g + b
    if s < min_bright:
        return None
    return (r / s, g / s, b / s)


def saturation(c):
    """0 = gray (white, black, gray edges); larger = more colorful."""
    if c is None:
        return 0.0
    return max(c) - min(c)


# ---------- model: threshold + the two lookup sets ----------
sat_thresh = 0.12        # recomputed from your samples by update_model()
color_max_dist = 0.10    # recomputed from your samples by update_model()
color_set = []           # (sample index, label, cr, cg, cb)  -> step 2a (hue only)
bright_set = []          # (sample index, label, w)           -> step 2b (brightness only)


def percentile(vals, p):
    vals = sorted(vals)
    return vals[min(len(vals) - 1, int(len(vals) * p))]


def update_model():
    """Learn the dark cutoff, saturation threshold and max color distance, and split the samples."""
    global min_bright, sat_thresh, color_max_dist, color_set, bright_set

    # 1. dark cutoff: black tape is too dark to judge hue, colors are not
    blk = [t[1] + t[2] + t[3] for t in samples if t[0] == "BLACK"]
    colb = [t[1] + t[2] + t[3] for t in samples if t[0] in COLORS]
    if blk and colb:
        hi_blk = percentile(blk, 0.95)
        lo_col = percentile(colb, 0.05)
        if hi_blk < lo_col:
            min_bright = (hi_blk + lo_col) / 2
        else:
            min_bright = 0.9 * lo_col
            print("WARNING: a color is about as dark as the black tape "
                  "(black r+g+b up to %.3f, colors down to %.3f)." % (hi_blk, lo_col))
            print("  Hue will still be checked on black, so TURN samples matter even more.")

    # 2. saturation threshold: everything that is NOT a color vs. the colors
    gray = [saturation(chroma(t[1], t[2], t[3])) for t in samples if t[0] in NOT_COLOR]
    col = [saturation(chroma(t[1], t[2], t[3])) for t in samples if t[0] in COLORS]
    if gray and col:
        hi_gray = percentile(gray, 0.95)
        lo_col = percentile(col, 0.05)
        sat_thresh = (hi_gray + lo_col) / 2
        if lo_col <= hi_gray:
            print("WARNING: some color samples are as gray as white/black/turns "
                  "(non-color sat up to %.3f, colors down to %.3f)." % (hi_gray, lo_col))
            print("  Check sensor height/lighting, or recollect the faint color.")

    # 3. split: colorful EDGE/TURN samples stay in the hue comparison as "not a color" examples,
    #    so a corner that happens to look tinted is outvoted instead of read as GREEN/BLUE.
    color_set, bright_set = [], []
    for i, t in enumerate(samples):
        c = chroma(t[1], t[2], t[3])
        if t[0] in COLORS or (t[0] in ("EDGE", "TURN") and saturation(c) > sat_thresh):
            if c is not None:
                color_set.append((i, t[0], c[0], c[1], c[2]))
        else:                                   # WHITE, BLACK, gray EDGE/TURN
            bright_set.append((i, t[0], t[4]))

    # max color distance: how far apart samples of the SAME color usually are, times 3
    same = []
    for a in color_set:
        if a[1] not in COLORS:
            continue
        best = None
        for b in color_set:
            if b[0] == a[0] or b[1] != a[1]:
                continue
            d = (a[2] - b[2]) ** 2 + (a[3] - b[3]) ** 2 + (a[4] - b[4]) ** 2
            if best is None or d < best:
                best = d
        if best is not None:
            same.append(best ** 0.5)
    if same:
        color_max_dist = max(0.03, min(0.25, 3 * percentile(same, 0.95)))


def vote(best):
    votes = {}
    for d, lbl in best:
        votes[lbl] = votes.get(lbl, 0) + 1
    win = best[0][1]                  # ties go to the single nearest sample
    for lbl in votes:
        if votes[lbl] > votes[win]:
            win = lbl
    return win


def classify(v, exclude=-1):
    """v = normalized [r, g, b, w]. Returns (label, distance to nearest sample)."""
    c = chroma(v[0], v[1], v[2])
    if saturation(c) > sat_thresh and color_set:
        # step 2a: colored -> compare hue only
        ds = [((t[2] - c[0]) ** 2 + (t[3] - c[1]) ** 2 + (t[4] - c[2]) ** 2, t[1])
              for t in color_set if t[0] != exclude]
        if not ds:
            return None, 0.0
        ds.sort()
        dist = ds[0][0] ** 0.5
        if dist > color_max_dist:
            return "?", dist          # colorful, but unlike anything trained
        return vote(ds[:KNN_K]), dist
    # step 2b: not colored -> compare brightness only
    ds = [((t[2] - v[3]) ** 2, t[1]) for t in bright_set if t[0] != exclude]
    if not ds:
        return None, 0.0
    ds.sort()
    return vote(ds[:KNN_K]), ds[0][0] ** 0.5


# ---------- file ----------
def load():
    try:
        with open(DATA_FILE) as f:
            for line in f:
                p = line.strip().split(",")
                if len(p) == 5 and p[0] in CLASSES:
                    samples.append((p[0], float(p[1]), float(p[2]), float(p[3]), float(p[4])))
        print("Loaded %d existing samples from %s" % (len(samples), DATA_FILE))
    except OSError:
        print("No %s yet - starting fresh" % DATA_FILE)
    update_model()


def save():
    update_model()
    with open(DATA_FILE, "w") as f:
        f.write("#settings,%.4f,%.4f,%.4f\n" % (sat_thresh, color_max_dist, min_bright))
        for t in samples:
            f.write("%s,%.4f,%.4f,%.4f,%.4f\n" % t)


# ---------- sensing ----------
def calibrate_white():
    sums = {s: [0, 0, 0, 0] for s in sensors}
    n = 0
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < 2000:
        for s in sensors:
            v = sensors[s].read()
            for i in range(4):
                sums[s][i] += v[i]
        n += 1
        time.sleep_ms(45)
    for s in sensors:
        white[s] = [max(x / n, 1) for x in sums[s]]
    print("White reference:", white)


def norm(side):
    raw = sensors[side].read()
    return [raw[i] / white[side][i] for i in range(4)]


# ---------- menu actions ----------
def show_counts():
    c = {k: 0 for k in CLASSES}
    for t in samples:
        if t[0] in c:
            c[t[0]] += 1
    print("\n--- samples so far ---")
    for i, k in enumerate(CLASSES):
        print(" %d) %-5s %d" % (i + 1, k, c[k]))
    print(" color threshold (saturation) = %.3f   max color distance = %.3f   dark cutoff = %.3f"
          % (sat_thresh, color_max_dist, min_bright))


def collect(cls):
    print("\n=== %s ===\n%s" % (cls, HINTS[cls]))
    ans = input("Which sensor is over it? Enter = both, l = left, r = right: ").strip().lower()
    sides = ["left"] if ans == "l" else ["right"] if ans == "r" else ["left", "right"]
    n = SAMPLES_PER_SENSOR[cls]
    input("Press Enter, then move the sensor slowly for about %.1f s..." % (n * INTERVAL_MS / 1000))
    new = []
    for _ in range(n):
        for sd in sides:
            v = norm(sd)
            new.append((cls, v[0], v[1], v[2], v[3]))
        time.sleep_ms(INTERVAL_MS)
    samples.extend(new)
    save()
    print("Added %d %s samples (saved)." % (len(new), cls))


def evaluate():
    if len(samples) < KNN_K + 2:
        print("Not enough samples yet.")
        return
    print("Evaluating (leave-one-out)... this takes a few seconds.")
    right = {k: 0 for k in CLASSES}
    total = {k: 0 for k in CLASSES}
    sat_sum = {k: 0.0 for k in CLASSES}
    confused = {}
    for i, t in enumerate(samples):
        pred, _ = classify(t[1:], exclude=i)
        total[t[0]] += 1
        sat_sum[t[0]] += saturation(chroma(t[1], t[2], t[3]))
        if pred == t[0]:
            right[t[0]] += 1
        else:
            confused[(t[0], pred)] = confused.get((t[0], pred), 0) + 1
    print("\n--- accuracy per class (avg saturation) ---")
    for k in CLASSES:
        if total[k]:
            print(" %-5s %d/%d   (sat %.3f)" % (k, right[k], total[k], sat_sum[k] / total[k]))
    if confused:
        print("--- mix-ups (true -> predicted) ---")
        for (a, b), n in sorted(confused.items(), key=lambda x: -x[1]):
            print(" %s -> %s : %d" % (a, b, n))
    else:
        print("No mix-ups.")
    print("\nColor threshold (saturation) = %.3f, max color distance = %.3f (saved for main.py)"
          % (sat_thresh, color_max_dist))
    print("(Samples taken in one pass are very similar, so these scores look better than real life."
          "\n Collect several separate passes in different spots for a trustworthy model.)")


def live():
    if not samples:
        print("No samples yet.")
        return
    print("\nLive test - slide the sensors over things. Ctrl+C / Stop to leave.")
    try:
        while True:
            parts = []
            for sd in ("left", "right"):
                v = norm(sd)
                label, dist = classify(v)
                sat = saturation(chroma(v[0], v[1], v[2]))
                parts.append("%s: %-5s (dist %.3f, sat %.3f, w %.2f)"
                             % (sd[0].upper(), label, dist, sat, v[3]))
            print("   ".join(parts))
            time.sleep_ms(250)
    except KeyboardInterrupt:
        print("left live test")


# ---------------- main menu ----------------
load()
input("Put BOTH sensors over plain WHITE paper at working height, then press Enter...")
calibrate_white()

while True:
    show_counts()
    cmd = input("\n1-7 = collect that class | c = clear a class | e = evaluate | "
                "t = live test | q = quit: ").strip().lower()
    if cmd in ("1", "2", "3", "4", "5", "6", "7"):
        collect(CLASSES[int(cmd) - 1])
    elif cmd == "c":
        which = input("Clear which class number (1-7)? ").strip()
        if which in ("1", "2", "3", "4", "5", "6", "7"):
            name = CLASSES[int(which) - 1]
            samples[:] = [t for t in samples if t[0] != name]
            save()
            print("Cleared", name)
    elif cmd == "e":
        evaluate()
    elif cmd == "t":
        live()
    elif cmd == "q":
        save()
        print("Saved %d samples to %s. Now run/upload main.py." % (len(samples), DATA_FILE))
        break