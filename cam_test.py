import cv2

cam = cv2.VideoCapture(-1)

while True:
    ret, frame = cam.read()
    cv2.imshow("camera", frame)
    if cv2.waitKey(1) & 0xFF == ord("c"):
        break

cam.release()
cv2.destroyAllWindows()
