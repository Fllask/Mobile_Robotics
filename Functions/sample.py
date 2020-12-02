import cv2

cap = cv2.VideoCapture(0)




ret, img = cap.read()    
cv2.imwrite('sample.jpg', img) 
