# export_engine.py  — run once, outside ROS
from ultralytics import YOLO
import torch, argparse

def export(weights: str, imgsz: tuple, use_int8: bool):
    model = YOLO(weights)

    kwargs = dict(
        format    = 'engine',
        imgsz     = imgsz,          # (H, W) — must match runtime input exactly
        device    = 0,
        workspace = 3,              # GB of GPU RAM for TRT builder (use max available)
        simplify  = True,           # ONNX graph simplification before TRT build
        verbose   = False,
    )

    if use_int8:
        # INT8 needs calibration images — point to a folder of representative frames
        kwargs.update(int8=True, data='calib_frames/')
        print('Building INT8 engine — this takes several minutes')
    else:
        kwargs.update(half=True)    # FP16
        print('Building FP16 engine')

    model.export(**kwargs)
    out = weights.replace('.pt', '.engine')
    print(f'Engine saved → {out}')
    print(f'Expected size: {__import__("os").path.getsize(out)/1e6:.1f} MB')

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', default='yolo26n.pt')
    ap.add_argument('--imgsz',   default='480,640')
    ap.add_argument('--int8',    action='store_true')
    args = ap.parse_args()
    h, w = map(int, args.imgsz.split(','))
    export(args.weights, (h, w), args.int8)