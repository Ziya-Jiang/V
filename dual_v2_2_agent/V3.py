"""Default: fast book-centerline Y/tool-roll alignment, fixed Z, V1_3 X endpoint."""
import sys


def main(argv=None):
    args=list(sys.argv[1:] if argv is None else argv)
    if '--legacy-yz' in args:
        args.remove('--legacy-yz')
        from cart_book_grasp_v3 import main as run
    elif any(flag in args for flag in ('--capture-wrist','--teach','--set-reference')):
        # Preserve camera-only capture and the optional old point-teaching UI.
        from cart_book_grasp_v3 import main as run
    elif '--legacy-final-image' in args or any(flag in args for flag in ('--match-reference','--match-head-wrist','--preview-wrist-center')):
        if '--legacy-final-image' in args:args.remove('--legacy-final-image')
        from cart_book_grasp_v3_stage2 import main as run
    elif '--legacy-cotracker' in args or '--replay-cotracker' in args:
        if '--legacy-cotracker' in args:args.remove('--legacy-cotracker')
        from cart_book_grasp_v3_cotracker_main import main as run
    else:
        from cart_book_grasp_v3_column_main import main as run
    return run(args)


if __name__ == '__main__':
    try:main()
    except (ValueError,OSError) as error:
        print(f'V3: {error}',file=sys.stderr)
        raise SystemExit(2)
