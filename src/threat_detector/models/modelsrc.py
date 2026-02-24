from ultralytics import YOLO

model = YOLO("yolov8m.pt")

model.export(
    format="onnx",
    opset=12,
    imgsz=(480, 640),
    dynamic=False,      # IMPORTANT
    simplify=True
)
