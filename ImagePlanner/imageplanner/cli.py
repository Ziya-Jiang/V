import argparse
import json

from .core import evaluate, fit, load_dataset, plan, read_json, write_json


def main():
    parser = argparse.ArgumentParser(description="Offline wrist-camera visual keyframe baseline")
    commands = parser.add_subparsers(dest="command", required=True)
    validate = commands.add_parser("validate")
    validate.add_argument("dataset")
    train = commands.add_parser("fit")
    train.add_argument("dataset")
    train.add_argument("--out", required=True)
    test = commands.add_parser("evaluate")
    test.add_argument("model")
    test.add_argument("dataset")
    test.add_argument("--out", required=True)
    predict = commands.add_parser("plan")
    predict.add_argument("model")
    predict.add_argument("--bbox", type=float, nargs=4, required=True, metavar=("X", "Y", "W", "H"))
    predict.add_argument("--size", type=int, nargs=2, required=True, metavar=("WIDTH", "HEIGHT"))
    predict.add_argument("--camera-id", required=True)
    predict.add_argument("--target-id", required=True)
    predict.add_argument("--out", required=True)
    args = parser.parse_args()
    try:
        if args.command == "validate":
            data = load_dataset(args.dataset)
            print(json.dumps({"valid": True, "episodes": len(data["episodes"])}, ensure_ascii=False))
            return
        if args.command == "fit":
            result = fit(load_dataset(args.dataset))
        elif args.command == "evaluate":
            result = evaluate(read_json(args.model), load_dataset(args.dataset))
        else:
            result = plan(read_json(args.model), args.bbox, args.size, args.camera_id, args.target_id)
        write_json(args.out, result)
        print(f"Saved {args.command}: {args.out}")
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, f"Error: {exc}\n")


if __name__ == "__main__":
    main()
