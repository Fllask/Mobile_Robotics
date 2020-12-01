import cv2


cap = cv2.VideoCapture(1)
while True:
    ret, img = cap.read()
    imgsmall = cv2.resize(img,(624,416))
    cv2.imshow("test",imgsmall)