"""Train, cross-validate and export the INT8 gesture model.

Design goals
------------
* Grouped evaluation: every cross-validation fold holds out complete captures,
  never individual windows from a capture seen during training.
* No normalization leakage: mean/std are computed on the training fold only.
* Two accuracy views: window-level and capture-level.  The capture-level view
  averages the softmax over the offset windows belonging to one capture, which
  is exactly what the firmware does at inference time.
* Reproducible: one seed controls the split, the folds, TensorFlow and numpy.
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import tensorflow as tf

AUTOTUNE = tf.data.AUTOTUNE


def set_seed(seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)
    tf.keras.utils.set_random_seed(seed)


def load_dataset(
    data_dir: Path,
) -> tuple[dict, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    metadata = json.loads((data_dir / "metadata.json").read_text(encoding="utf-8"))
    data = np.load(data_dir / "dataset.npz")
    return (
        metadata,
        data["X"].astype(np.float32),
        data["y"].astype(np.int64),
        data["source"].astype(str),
        data["domain"].astype(str),
        data["split"].astype(str),
    )


def normalize(
    x: np.ndarray, train_index: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    flat = x[train_index].reshape(-1, x.shape[-1])
    mean = flat.mean(axis=0)
    std = flat.std(axis=0)
    std[std < 1e-6] = 1.0
    return (x - mean) / std, mean.astype(np.float32), std.astype(np.float32)


def stratified_group_folds(
    sources: np.ndarray, labels: np.ndarray, folds: int, seed: int
) -> list[tuple[np.ndarray, np.ndarray]]:
    """Assign complete captures to folds, balancing each class."""
    captures: dict[str, int] = {}
    for source, label in zip(sources, labels):
        captures[source] = int(label)

    grouped: dict[int, list[str]] = {}
    for source, label in captures.items():
        grouped.setdefault(label, []).append(source)

    rng = random.Random(seed)
    fold_of: dict[str, int] = {}
    for label in sorted(grouped):
        items = sorted(grouped[label])
        rng.shuffle(items)
        for index, source in enumerate(items):
            fold_of[source] = index % folds

    result: list[tuple[np.ndarray, np.ndarray]] = []
    for fold in range(folds):
        validation = np.asarray([fold_of[s] == fold for s in sources])
        result.append((np.flatnonzero(~validation), np.flatnonzero(validation)))
    return result


def holdout_indices(
    sources: np.ndarray, split: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    train = np.flatnonzero(split == "train")
    validation = np.flatnonzero(split == "validation")
    test = np.flatnonzero(split == "test")
    return train, validation, test


def build_model(
    class_count: int, width: int, dropout: float, l2: float
) -> tf.keras.Model:
    regularizer = tf.keras.regularizers.l2(l2) if l2 > 0 else None
    return tf.keras.Sequential(
        [
            tf.keras.layers.Input(shape=(100, 6)),
            tf.keras.layers.Conv1D(
                width, 5, padding="same", use_bias=False,
                kernel_regularizer=regularizer,
            ),
            tf.keras.layers.BatchNormalization(),
            tf.keras.layers.ReLU(),
            tf.keras.layers.MaxPooling1D(2),
            tf.keras.layers.Conv1D(
                width * 2, 5, padding="same", use_bias=False,
                kernel_regularizer=regularizer,
            ),
            tf.keras.layers.BatchNormalization(),
            tf.keras.layers.ReLU(),
            tf.keras.layers.MaxPooling1D(2),
            tf.keras.layers.Conv1D(
                width * 4, 3, padding="same", use_bias=False,
                kernel_regularizer=regularizer,
            ),
            tf.keras.layers.BatchNormalization(),
            tf.keras.layers.ReLU(),
            tf.keras.layers.GlobalAveragePooling1D(),
            tf.keras.layers.Dropout(dropout),
            tf.keras.layers.Dense(
                class_count, activation="softmax", kernel_regularizer=regularizer
            ),
        ]
    )


def augment(
    x: tf.Tensor, y: tf.Tensor, level: str
) -> tuple[tf.Tensor, tf.Tensor]:
    if level == "none":
        return x, y
    if level == "strong":
        shift, noise, scale_low, scale_high, bias = 8, 0.03, 0.95, 1.05, 0.05
    else:
        shift, noise, scale_low, scale_high, bias = 5, 0.02, 0.98, 1.02, 0.02

    offset = tf.random.uniform([], -shift, shift + 1, dtype=tf.int32)
    padded = tf.pad(x, [[shift, shift], [0, 0]], mode="SYMMETRIC")
    x = padded[shift + offset : shift + offset + 100]
    x = x * tf.random.uniform([6], scale_low, scale_high)
    x = x + tf.random.normal(tf.shape(x), stddev=noise)
    gyro_bias = tf.random.normal([3], stddev=bias)
    x = x + tf.concat([tf.zeros(3), gyro_bias], axis=0)
    return x, y


def make_dataset(
    x: np.ndarray,
    y: np.ndarray,
    batch_size: int,
    training: bool,
    augmentation: str,
    seed: int,
) -> tf.data.Dataset:
    dataset = tf.data.Dataset.from_tensor_slices((x, tf.one_hot(y, depth=6)))
    if training:
        dataset = dataset.shuffle(len(x), seed=seed, reshuffle_each_iteration=True)
        if augmentation != "none":
            dataset = dataset.map(
                lambda a, b: augment(a, b, augmentation),
                num_parallel_calls=AUTOTUNE,
            )
    return dataset.batch(batch_size).prefetch(AUTOTUNE)


def confusion_matrix(
    y: np.ndarray, prediction: np.ndarray, classes: int
) -> np.ndarray:
    matrix = np.zeros((classes, classes), dtype=np.int64)
    for actual, predicted in zip(y, prediction):
        matrix[int(actual), int(predicted)] += 1
    return matrix


def classification_metrics(
    y: np.ndarray, prediction: np.ndarray, classes: int
) -> dict:
    matrix = confusion_matrix(y, prediction, classes)
    support = matrix.sum(axis=1)
    predicted = matrix.sum(axis=0)
    recall = np.divide(
        np.diag(matrix), support, out=np.zeros(classes), where=support > 0
    )
    precision = np.divide(
        np.diag(matrix), predicted, out=np.zeros(classes), where=predicted > 0
    )
    f1 = np.divide(
        2 * precision * recall,
        precision + recall,
        out=np.zeros(classes),
        where=(precision + recall) > 0,
    )
    return {
        "accuracy": float(np.mean(y == prediction)),
        "per_class_recall": recall.tolist(),
        "per_class_precision": precision.tolist(),
        "per_class_f1": f1.tolist(),
        "macro_f1": float(f1.mean()),
        "confusion_matrix": matrix.tolist(),
        "support": support.tolist(),
    }


def capture_predictions(
    probabilities: np.ndarray, sources: np.ndarray, labels: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    unique = sorted(set(sources.tolist()))
    predictions = []
    actual = []
    for source in unique:
        mask = sources == source
        predictions.append(int(np.argmax(probabilities[mask].mean(axis=0))))
        values, counts = np.unique(labels[mask], return_counts=True)
        actual.append(int(values[np.argmax(counts)]))
    return np.asarray(actual), np.asarray(predictions)


def evaluate_model(
    model: tf.keras.Model,
    x: np.ndarray,
    y: np.ndarray,
    sources: np.ndarray,
    domains: np.ndarray,
    class_names: list[str],
    batch_size: int,
) -> dict:
    probabilities = model.predict(x, batch_size=batch_size, verbose=0)
    window_prediction = probabilities.argmax(axis=1)
    actual_captures, captured_prediction = capture_predictions(
        probabilities, sources, y
    )
    per_domain = {}
    for domain in sorted(set(domains.tolist())):
        mask = domains == domain
        domain_actual, domain_prediction = capture_predictions(
            probabilities[mask], sources[mask], y[mask]
        )
        per_domain[domain] = {
            "window": classification_metrics(
                y[mask], window_prediction[mask], len(class_names)
            ),
            "capture": classification_metrics(
                domain_actual, domain_prediction, len(class_names)
            ),
        }
    return {
        "window": classification_metrics(y, window_prediction, len(class_names)),
        "capture": classification_metrics(
            actual_captures, captured_prediction, len(class_names)
        ),
        "per_domain": per_domain,
        "window_probabilities": probabilities,
    }


def summarize_folds(results: list[dict]) -> dict:
    window_accuracy = [result["window"]["accuracy"] for result in results]
    capture_accuracy = [result["capture"]["accuracy"] for result in results]
    recall = np.asarray(
        [result["capture"]["per_class_recall"] for result in results]
    )
    return {
        "window_accuracy_mean": float(np.mean(window_accuracy)),
        "window_accuracy_std": float(np.std(window_accuracy)),
        "capture_accuracy_mean": float(np.mean(capture_accuracy)),
        "capture_accuracy_std": float(np.std(capture_accuracy)),
        "capture_recall_mean": recall.mean(axis=0).tolist(),
        "capture_recall_std": recall.std(axis=0).tolist(),
    }


def train_model(
    x_train: np.ndarray,
    y_train: np.ndarray,
    x_validation: np.ndarray,
    y_validation: np.ndarray,
    domains_train: np.ndarray,
    class_count: int,
    args: argparse.Namespace,
    seed: int,
) -> tf.keras.Model:
    if args.oversample_domain and args.domain_repeat > 1:
        extra = np.flatnonzero(domains_train == args.oversample_domain)
        if len(extra) > 0:
            x_train = np.concatenate(
                [x_train] + [x_train[extra]] * (args.domain_repeat - 1)
            )
            y_train = np.concatenate(
                [y_train] + [y_train[extra]] * (args.domain_repeat - 1)
            )
    set_seed(seed)
    model = build_model(class_count, args.width, args.dropout, args.l2)
    model.compile(
        optimizer=tf.keras.optimizers.Adam(1e-3),
        loss=tf.keras.losses.CategoricalCrossentropy(
            label_smoothing=args.label_smoothing
        ),
        metrics=["accuracy"],
    )
    train_dataset = make_dataset(
        x_train, y_train, args.batch_size, True, args.augment, seed
    )
    validation_dataset = make_dataset(
        x_validation, y_validation, args.batch_size, False, "none", seed
    )
    callbacks = [
        tf.keras.callbacks.EarlyStopping(
            monitor="val_loss", patience=args.patience,
            restore_best_weights=True, min_delta=1e-4,
        ),
        tf.keras.callbacks.ReduceLROnPlateau(
            monitor="val_loss", factor=0.5, patience=5,
            min_lr=1e-5, verbose=0,
        ),
    ]
    model.fit(
        train_dataset,
        validation_data=validation_dataset,
        epochs=args.epochs,
        callbacks=callbacks,
        verbose=0,
        shuffle=False,
    )
    return model


def quantize_model(
    model: tf.keras.Model, x_train: np.ndarray, path: Path
) -> tuple[Path, dict]:
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]

    def representative_dataset():
        for sample in x_train:
            yield [sample[np.newaxis, ...].astype(np.float32)]

    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    payload = converter.convert()
    path.write_bytes(payload)

    interpreter = tf.lite.Interpreter(model_content=payload)
    interpreter.allocate_tensors()
    input_detail = interpreter.get_input_details()[0]
    output_detail = interpreter.get_output_details()[0]
    return path, {
        "input_scale": float(input_detail["quantization"][0]),
        "input_zero_point": int(input_detail["quantization"][1]),
        "output_scale": float(output_detail["quantization"][0]),
        "output_zero_point": int(output_detail["quantization"][1]),
        "model_bytes": len(payload),
    }


def evaluate_int8(
    path: Path,
    x: np.ndarray,
    y: np.ndarray,
    sources: np.ndarray,
    domains: np.ndarray,
    class_names: list[str],
    quantization: dict,
) -> dict:
    interpreter = tf.lite.Interpreter(model_path=str(path))
    interpreter.allocate_tensors()
    input_detail = interpreter.get_input_details()[0]
    output_detail = interpreter.get_output_details()[0]
    scale = quantization["input_scale"]
    zero = quantization["input_zero_point"]
    output_scale = quantization["output_scale"]
    output_zero = quantization["output_zero_point"]

    probabilities = []
    for sample in x:
        quantized = np.clip(
            np.round(sample / scale + zero), -128, 127
        ).astype(np.int8)[None, ...]
        interpreter.set_tensor(input_detail["index"], quantized)
        interpreter.invoke()
        values = interpreter.get_tensor(output_detail["index"])[0].astype(np.float32)
        probabilities.append((values - output_zero) * output_scale)
    probabilities = np.asarray(probabilities)

    window_prediction = probabilities.argmax(axis=1)
    actual_captures, captured_prediction = capture_predictions(
        probabilities, sources, y
    )
    per_domain = {}
    for domain in sorted(set(domains.tolist())):
        mask = domains == domain
        domain_actual, domain_prediction = capture_predictions(
            probabilities[mask], sources[mask], y[mask]
        )
        per_domain[domain] = {
            "window": classification_metrics(
                y[mask], window_prediction[mask], len(class_names)
            ),
            "capture": classification_metrics(
                domain_actual, domain_prediction, len(class_names)
            ),
        }
    return {
        "window": classification_metrics(y, window_prediction, len(class_names)),
        "capture": classification_metrics(
            actual_captures, captured_prediction, len(class_names)
        ),
        "per_domain": per_domain,
        "window_probabilities": probabilities,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=Path("data/processed_onset"))
    parser.add_argument("--output-dir", type=Path, default=Path("runs/onset"))
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--patience", type=int, default=12)
    parser.add_argument("--batch-size", type=int, default=32)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--folds", type=int, default=5, help="0 = holdout split")
    parser.add_argument("--width", type=int, default=16)
    parser.add_argument("--dropout", type=float, default=0.3)
    parser.add_argument("--l2", type=float, default=1e-4)
    parser.add_argument("--label-smoothing", type=float, default=0.0)
    parser.add_argument(
        "--augment", choices=("none", "basic", "strong"), default="basic"
    )
    parser.add_argument("--oversample-domain", default="raw_f407")
    parser.add_argument("--domain-repeat", type=int, default=3)
    parser.add_argument(
        "--export", action="store_true", help="Train on train+validation and export"
    )
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    metadata, x, y, sources, domains, split = load_dataset(args.data_dir)
    class_names = metadata["label_names"]
    class_count = len(class_names)
    print(
        f"windows={len(x)} captures={len(set(sources.tolist()))} classes={class_count}"
    )

    report: dict = {
        "config": {
            **vars(args),
            "data_dir": str(args.data_dir),
            "output_dir": str(args.output_dir),
        },
        "dataset": {
            "window_mode": metadata["window_mode"],
            "offsets": metadata["offsets"],
            "window_size": metadata["window_size"],
            "capture_counts": metadata["capture_counts"],
            "class_counts": metadata["class_counts"],
        },
        "class_names": class_names,
    }

    if args.folds and args.folds > 1:
        folds = stratified_group_folds(sources, y, args.folds, args.seed)
        fold_results = []
        for index, (train_index, validation_index) in enumerate(folds):
            x_normalized, _, _ = normalize(x, train_index)
            model = train_model(
                x_normalized[train_index], y[train_index],
                x_normalized[validation_index], y[validation_index],
                domains[train_index], class_count, args, args.seed + index,
            )
            result = evaluate_model(
                model, x_normalized[validation_index], y[validation_index],
                sources[validation_index], domains[validation_index],
                class_names, args.batch_size,
            )
            result.pop("window_probabilities", None)
            result["fold"] = index
            result["captures"] = len(set(sources[validation_index].tolist()))
            fold_results.append(result)
            print(
                f"fold {index}: window={result['window']['accuracy']:.4f} "
                f"capture={result['capture']['accuracy']:.4f} "
                f"tilt_right={result['capture']['per_class_recall'][5]:.3f}"
            )
        report["cross_validation"] = {
            "folds": args.folds,
            "summary": summarize_folds(fold_results),
            "details": fold_results,
        }
        print(json.dumps(report["cross_validation"]["summary"], indent=2))

    if args.export:
        train_index, validation_index, test_index = holdout_indices(sources, split)
        train_all = np.concatenate([train_index, validation_index])
        x_normalized, mean, std = normalize(x, train_all)
        model = train_model(
            x_normalized[train_all], y[train_all],
            x_normalized[test_index], y[test_index],
            domains[train_all], class_count, args, args.seed + 100,
        )
        float_result = evaluate_model(
            model, x_normalized[test_index], y[test_index],
            sources[test_index], domains[test_index],
            class_names, args.batch_size,
        )

        model_path = args.output_dir / "gesture_model.keras"
        float_path = args.output_dir / "gesture_model_float32.tflite"
        int8_path = args.output_dir / "gesture_model_int8.tflite"
        model.save(model_path)

        converter = tf.lite.TFLiteConverter.from_keras_model(model)
        float_payload = converter.convert()
        float_path.write_bytes(float_payload)
        int8_path, quantization = quantize_model(
            model, x_normalized[train_all], int8_path
        )
        int8_result = evaluate_int8(
            int8_path, x_normalized[test_index], y[test_index],
            sources[test_index], domains[test_index], class_names, quantization,
        )

        report["holdout"] = {
            "train_captures": len(set(sources[train_all].tolist())),
            "test_captures": len(set(sources[test_index].tolist())),
            "float32": {
                "window": float_result["window"],
                "capture": float_result["capture"],
                "per_domain": float_result["per_domain"],
            },
            "int8": {
                "window": int8_result["window"],
                "capture": int8_result["capture"],
                "per_domain": int8_result["per_domain"],
            },
            "quantization": quantization,
            "model_bytes": {
                "float32": len(float_payload),
                "int8": int8_path.stat().st_size,
            },
        }
        report["normalization"] = {"mean": mean.tolist(), "std": std.tolist()}
        print(
            f"holdout float32 capture={float_result['capture']['accuracy']:.4f} "
            f"int8 capture={int8_result['capture']['accuracy']:.4f}"
        )
        print(f"int8 model={int8_path.resolve()}")

    (args.output_dir / "report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(f"report={(args.output_dir / 'report.json').resolve()}")


if __name__ == "__main__":
    main()
