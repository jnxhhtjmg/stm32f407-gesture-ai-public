# Dataset placeholder

No gesture data is included in this repository.  Put your own captures here:

```text
data/
├─ raw/          # captures from the first board/session
└─ raw_f407/     # optional: captures from a second board/session
```

Each file is one gesture, 2–5 s at 100 Hz:

```text
timestamp,ax,ay,az,gx,gy,gz,label
```

* `timestamp` — milliseconds, strictly increasing;
* `ax/ay/az` — accelerometer LSB, ±2 g (16384 LSB/g);
* `gx/gy/gz` — gyroscope LSB, ±250 °/s (131 LSB/°/s);
* `label` — `lift`, `move_left`, `move_right`, `neutral`, `tilt_left`,
  `tilt_right`.

Use [`../collect_f407.py`](../collect_f407.py) to record captures through the
firmware's KEY0 capture mode.  The F103/F407 split is only a convention; any
directory name works as a domain tag for `--oversample-domain`.

The dataset is not covered by the repository license; it is yours.
