from fastapi import FastAPI, File, UploadFile
import uvicorn
import numpy as np
import cv2
from ultralytics import YOLO

model = YOLO("best.pt")

app = FastAPI()

@app.post("/frame")
async def frame(file: UploadFile = File(...)):
    raw = await file.read()

    arr = np.frombuffer(raw, np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    if frame is None:
        print("Decode failed")
        return {"error": "decode_failed"}

    results = model(frame)[0]
    annotated = results.plot()

    # Show annotated frame in OpenCV window
    cv2.imshow("Webots Fire Detection", annotated)
    cv2.waitKey(1)

    # Print detections
    for box in results.boxes:
        cls = results.names[int(box.cls)]
        conf = float(box.conf)
        print(cls, conf)

    return {"status": "ok"}

if __name__ == "__main__":
    uvicorn.run("script:app", host="127.0.0.1", port=5001)
