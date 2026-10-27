# Model placeholder

No trained model is included in this repository.

`train_keras_litert.py --export` writes the following files to
`runs/<name>/`:

* `gesture_model.keras` — trained Keras model;
* `gesture_model_float32.tflite` — float reference;
* `gesture_model_int8.tflite` — deployed INT8 model;
* `report.json` — per-domain metrics, quantization and normalization.

To put the model into the firmware:

1. Convert the INT8 TFLite file into a C array:

   ```bash
   python generate_model_c_array.py \
     runs/<name>/gesture_model_int8.tflite runs/<name> \
     --header-dir ../firmware/Core/Inc \
     --source-dir ../firmware/Core/Src
   ```

2. Copy `normalization_mean`, `normalization_std`, `INPUT_SCALE` and
   `INPUT_ZERO_POINT` from `report.json` into
   `../firmware/Core/Src/gesture_preprocess.c`.
3. Rebuild the firmware.

The repository ships a 16-byte placeholder in
`firmware/Core/Src/g_gesture_model_data.c`; the firmware builds and boots with
it, but logs `gesture inference init failed` until a real model is generated.
