# Training framework

This directory contains the PC pipeline only.  **No dataset, no trained model
and no trained normalization/quantization parameters are included.**

## Expected data

Put one CSV per capture under `data/raw/` (or `data/raw_f407/` for a different
board/domain):

```text
timestamp,ax,ay,az,gx,gy,gz,label
```

* 100 Hz, strictly increasing `timestamp` in milliseconds;
* accelerometer LSB ±2 g (16384 LSB/g), gyroscope LSB ±250 °/s (131 LSB/°/s);
* one label per file: `lift`, `move_left`, `move_right`, `neutral`,
  `tilt_left`, `tilt_right`;
* one complete out-and-back gesture per file, 2–5 s.

`collect_f407.py` records F407 captures through the firmware's CSV mode:
press KEY0 to start, perform the gesture, press KEY0 to stop.  It drops the
first 0.3 s (button transient), rejects captures without motion, and limits a
capture to 6 s.  A separate F103 collector is not included; any 100 Hz CSV in
the format above works.

## Pipeline

```text
CSV capture
  -> onset detection (3 °/s or 0.03 g, 5 consecutive samples)
  -> 200 samples from onset (2 s @ 100 Hz)
  -> 2:1 decimation to 100 samples @ 50 Hz
  -> per-fold z-score using the training captures only
  -> 1D-CNN -> INT8 TFLite
```

`sequence` mode uses one complete 2 s gesture per capture.  `onset` (3 x 1 s)
and `sliding` (1 s / 0.5 s) modes remain available for comparison.

## Commands

```bash
python -m pip install -r requirements.txt

# Build the dataset from one or more capture directories
python prepare_dataset.py \
  --raw-dir data/raw data/raw_f407 \
  --mode sequence --downsample 2 --window-size 100 \
  --output-dir data/processed_mixed

# Cross-validation, grouped by capture
python train_keras_litert.py \
  --data-dir data/processed_mixed \
  --output-dir runs/my_cv \
  --folds 5 --epochs 100 --augment basic \
  --oversample-domain raw_f407 --domain-repeat 3

# Final model: train on train+validation, evaluate on the held-out test split
python train_keras_litert.py \
  --data-dir data/processed_mixed \
  --output-dir runs/my_model \
  --folds 0 --epochs 140 --augment basic \
  --oversample-domain raw_f407 --domain-repeat 3 --export
```

`train_keras_litert.py`:

* splits by **capture**, never by window;
* computes normalization on the training data only;
* optionally oversamples a domain (`--oversample-domain raw_f407`);
* reports overall and per-domain window/capture metrics;
* writes per-class recall/precision/F1, confusion matrices, quantization
  parameters and normalization constants to `report.json`.

## Generated files (not committed)

A successful `--export` writes to `runs/my_model/`:

* `gesture_model.keras` — trained Keras model;
* `gesture_model_float32.tflite` — float reference;
* `gesture_model_int8.tflite` — deployed INT8 model;
* `report.json` — metrics, quantization and normalization.

Then generate the firmware C array:

```bash
python generate_model_c_array.py \
  runs/my_model/gesture_model_int8.tflite runs/my_model \
  --header-dir ../firmware/Core/Inc \
  --source-dir ../firmware/Core/Src
```

Copy `normalization_mean`, `normalization_std`, `INPUT_SCALE` and
`INPUT_ZERO_POINT` from `report.json` into
`../firmware/Core/Src/gesture_preprocess.c`.

## Reference

On the author's private dataset (188 F103 + 55 F407 captures), this pipeline
reached INT8 97.4% on a 39-capture held-out split (F407 subset 11/11).  The
data and trained artifacts are not included; your numbers will depend on your
captures, mounting and class definitions.
