from ultralytics import YOLO as yolo
import cv2

file_path = './data/track_vid.mp4'
file_path2 = './data/amz_run1.mp4'
file_path3 = './data/av_run1.mp4'
file_path4 = './data/av_run2.mp4'

# cap = cv2.VideoCapture(file_path)
# cap = cv2.VideoCapture(file_path2)
cap = cv2.VideoCapture(file_path3)
# cap = cv2.VideoCapture(file_path4)

model = yolo("best.pt")

# results = model.train(data='coco128.yaml', epochs=1, batch=-1)
# results = model.train(data='/home/ab/git/2024_2025_av_sim_stack/perception/yolov8/data/datasets/fsoco/data.yaml', epochs=10, batch=-1)

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