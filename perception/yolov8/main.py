from ultralytics import YOLO as yolo
import cv2

file_path = './data/track_vid.mp4'

cap = cv2.VideoCapture(file_path)

model = yolo("yolov8m.pt")

# results = model.train(data='coco128.yaml', epochs=1, batch=-1)
# results = model.train(data='/home/ab/git/datasets/fsoco/data.yaml', epochs=1, batch=-1)

while cap.isOpened():

    success, frame = cap.read()

    if success:
        results = model(frame)

        annotated_frame = results[0].plot()

        cv2.imshow("Annotated", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    else:
        break

cap.release()
cv2.destroyAllWindows()