""" entry point for executing PaddleLite test """
import sys
import json
import numpy as np
import cv2
from serial import Serial
import predictor_wrapper
from camera import Camera
import time
import os
# ser = Serial('/dev/ttyPS1',115200)
STATE_DETECTED=False


# front_camera = Camera(0, [640, 480])
cap=cv2.VideoCapture(0)

count=[0]*9
aver_midpoint=[0]*9
check_list=[]
midpoint_list={}
# os.system('startx')
time.sleep(2)
print("xxx")
predictor = predictor_wrapper.PaddleLitePredictor()
#predictor.load({"model": "mask_new/data77976"})
predictor.load("model")
config = {
    "threshold": 0.35,
    "format": "RGB",
    "mean": [127.5, 127.5, 127.5],
    "scale": [0.007843, 0.007843, 0.007843],
    "shape": [1, 3, 480, 480]
}
label_list = {
    0:"0",
    1:"3",
    2:"4",
    3:"5",
    4:"6",
    5:"7",
    6:"8",

}
def uart_read():
    s=''
    ser = Serial('/dev/ttyPS1',115200,timeout=0.5)
    ser.flushInput()
    time.sleep(0.01)
    s = ser.read(1)
    print("read:",s)
    time.sleep(0.05)
    ser.close()
    return s

def uart_to_32():

    space = FindG(label)
    ser.write(b'\xff')
    time.sleep(0.05)

    uart(str(space))
    time.sleep(0.05)

    ser.write(b'\xee')
    time.sleep(0.05)

    time.sleep(0.05)
    ser.close()

def FindG(find):
    tup = (['red3',6],['red4',2],['redroll',4],['blue3',5],['blue4',1],['blueroll',3])
    dic = dict(tup)
    return dic[find]

def uart(gx):
    ser = Serial('/dev/ttyPS1',115200)
    ser.write(gx.encode()) 
    print("send:",gx)
    #ser.write("\r\n".encode()) 
    ser.close() 



def preprocess(frame, shape, format, mean, scale):
    """ preprocess image using formula y = (x - mean) x scale """
    img = cv2.resize(frame, (shape[2], shape[3]), interpolation=cv2.INTER_LINEAR)
    if format == "RGB":
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # BGR -> RGB
    img = np.array(img).astype(np.float32)

    img -= np.array(mean).reshape((1, 1, 3))
    img *= np.array(scale).reshape((1, 1, 3))
    return img

def valid_detection(out, threshold):

    valid_result = list()
    try:
        num = out.shape[0]
        for box_id in range(num):
            item = out[box_id]
            score = item[1]
            if score >= threshold:
                valid_result.append(item)
    except:
        pass
    return valid_result

def read_inputs(test, predictor):
    """ read inputs into predictor """
    frame = test["input"].copy()
    # print(frame.shape)
    origin_h, origin_w, _ = frame.shape
    img = preprocess(frame, test["shape"], test["format"], test["mean"], test["scale"])
    
    feed_h, feed_w, _ = img.shape
    data = np.zeros([1, feed_h, feed_w, 3], dtype=np.float32)

    shape = data.shape

    data[0, 0:shape[1], 0: shape[2], 0: shape[3]] = img

    data = data.reshape(test["shape"])
    # print(frame.shape)
    predictor.set_input(data, 0)

    # shape_feed = np.zeros((1, 2), dtype=np.int32)
    # shape_feed[0, 0] = origin_h
    # shape_feed[0, 1] = origin_w
    # predictor.set_input(shape_feed, 1)

def draw_boxes(test, valid_results):
    global label
    global count
    global aver_midpoint
    img = test["input"]
    h, w, _ = img.shape
    # print(img.shape)
    # import pdb;pdb.set_trace()
    for _, item in enumerate(valid_results):
        label = label_list[int(item[0])]
        
        #uart_to_32()
        score = item[1]
        print(label,str(score))



        left = int(item[2] * w)
        top = int(item[3] * h)
        right = int(item[4] * w)
        bottom = int(item[5] * h)
        start_point = (int(left), int(top))
        end_point = (int(right), int(bottom))
        ##
        mid_point = ((int(left)+int(right))/2,(int(top)+int(bottom))/2)
        ##
        color = (204, 0, 204)
        thickness = 2
        img = cv2.rectangle(img, start_point, end_point, color, thickness)
        cv2.putText(img, label + " " + str(score), start_point, cv2.FONT_HERSHEY_COMPLEX,1, (0, 0, 255))

        ##
        if score>0.5 and label=="1":
            count[1]=count[1]+1
            aver_midpoint[1]=aver_midpoint[1]+mid_point[0]
        if score>0.5 and label=="2":
            count[2]=count[2]+1
            aver_midpoint[2]=aver_midpoint[2]+mid_point[0]
        if score>0.5 and label=="3":
            count[3]=count[3]+1
            aver_midpoint[3]=aver_midpoint[3]+mid_point[0]
        if score>0.5 and label=="4":
            count[4]=count[4]+1
            aver_midpoint[4]=aver_midpoint[4]+mid_point[0]
        if score>0.5 and label=="5":
            count[5]=count[5]+1
            aver_midpoint[5]=aver_midpoint[5]+mid_point[0]
        if score>0.5 and label=="6":
            count[6]=count[6]+1
            aver_midpoint[6]=aver_midpoint[6]+mid_point[0]
        if score>0.5 and label=="7":
            count[7]=count[7]+1
            aver_midpoint[7]=aver_midpoint[7]+mid_point[0]
        if score>0.5 and label=="8":
            count[8]=count[8]+1
            aver_midpoint[8]=aver_midpoint[8]+mid_point[0]
        ##
        for i in range(1,9):
            if count[i]>=10:
                print("detected:",i)               
                print("midpoint:",aver_midpoint[i]/count[i])
        ##
    return img

def model_infer(config):
    """ validate a model specified in config_file """
    global predictor

    read_inputs(config, predictor)
    predictor.run()
    out = predictor.get_output(0)
    out = np.array(out)
    # print(out)
    out = valid_detection(out, config["threshold"])
    #print(out)
    # import pdb;pdb.set_trace()
    return draw_boxes(config, out)


if __name__ == "__main__":
    # front_camera.start()
    result=''
    # video_h = cap.get(4)
    # video_w = cap.get(3)
    t=20
    a=1
    while(True):
        #ser = Serial('/dev/ttyPS1',115200)
        #size = ser.inWaiting()               # 获得缓冲区字符
        result=''
        # size=uart_read()
        # if a==1:
        #     size = "1"
        #     a=0
        size = "1"
        if size :
            while(1):
                # frame = front_camera.read()
                ret, frame = cap.read()
                frame =cv2.flip(frame,-1)
                # cv2.imshow("result", frame)
                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break
                # frame =cv2.flip(frame,-1)          
                # frame = cv2.imread("mask_new/mask.jpg")
                config.update({"input": frame})
                res = model_infer(config)
                cv2.imshow("result", res)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                t=t-1

                
        #     for i in range(1,9):
        #         if count[i]>=10:
        #             # ser = Serial('/dev/ttyPS1',115200)
        #             # ser.write(str(i).encode())
        #             # ser.write(str(aver_midpoint[i]/count[i]).encode())
        #             # time.sleep(0.05)
        #             # ser.close()
        #             midpoint_list.update({str(i):aver_midpoint[i]/count[i]})
        #             # print(midpoint_list)
        #     new_midpoint = sorted(midpoint_list.items(),  key=lambda d: d[1], reverse=False)

        #     print(new_midpoint)
        #     for ls in new_midpoint:
        #         print(ls[0])
        #         result=result+ls[0]
        #     print(result)
        #     # print(new_midpoint.values)            
        #     # ser = Serial('/dev/ttyPS1',115200)
        #     # ser.write(result.encode())
        #     # ser.close()
        # else :
        #     t=15
        # # ser.close()

cap.release()
cv2.destroyAllWindows()
