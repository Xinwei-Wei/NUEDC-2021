import cv2
import time
cap=cv2.VideoCapture(0)
i=0
while(1):
    ret, frame = cap.read()
    frame =cv2.flip(frame,-1)
    # time.sleep(0.2)
    # cv2.imwrite('./xxx/'+str(i)+'.jpg',frame)
    # i+=1
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break
    cv2.imshow("capture", frame)
    # time.sleep(2)
    print(i)
cap.release()
cv2.destroyAllWindows()