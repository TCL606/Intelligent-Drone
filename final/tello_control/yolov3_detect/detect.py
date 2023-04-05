#!/usr/bin/env python
import time
from sys import platform
from models import *
import threading


def load_weight():
    # Initialize this once
    cfg = '/home/thudrone/final_catkin_ws/src/tello_control/cfg/yolov3.cfg'
    data = '/home/thudrone/final_catkin_ws/src/tello_control/data/ball.data'
    weights = '/home/thudrone/final_catkin_ws/src/tello_control/weights/best.pt'
    output = '/home/thudrone/final_catkin_ws/src/tello_control/data/output'
    img_size=416
    conf_thres=0.5
    nms_thres=0.5
    save_txt=False
    save_images=True
    save_path='/home/thudrone/final_catkin_ws/src/tello_control/data/output/result.jpg'
    
    device = torch_utils.select_device(force_cpu=ONNX_EXPORT)
    torch.backends.cudnn.benchmark = False  # set False for reproducible results
    if os.path.exists(output):
        shutil.rmtree(output)  # delete output folder
    os.makedirs(output)  # make new output folder

    # Initialize model
    if ONNX_EXPORT:
        s = (320, 192)  # (320, 192) or (416, 256) or (608, 352) onnx model image size (height, width)
        model = Darknet(cfg, s)
    else:
        model = Darknet(cfg, img_size)

    # Load weights
    if weights.endswith('.pt'):  # pytorch format
        model.load_state_dict(torch.load(weights, map_location=device)['model'])
    else:  # darknet format
        _ = load_darknet_weights(model, weights)

    # Eval mode
    model.to(device).eval()

    # Export mode
    if ONNX_EXPORT:
        img = torch.zeros((1, 3, s[0], s[1]))
        torch.onnx.export(model, img, '/home/thudrone/final_catkin_ws/src/tello_control/weights/export.onnx', verbose=True)
        return
    
    return model, device


def detect_ball(model, device, img):
    # Initialized  for every detection
    cfg = '/home/thudrone/final_catkin_ws/src/tello_control/cfg/yolov3.cfg'
    data = '/home/thudrone/final_catkin_ws/src/tello_control/data/ball.data'
    weights = '/home/thudrone/final_catkin_ws/src/tello_control/weights/best.pt'
    output = '/home/thudrone/final_catkin_ws/src/tello_control/data/output'
    img_size=416
    conf_thres=0.5
    nms_thres=0.5
    save_txt=False
    save_images=True
    save_path='/home/thudrone/final_catkin_ws/src/tello_control/data/output/result.jpg'

    # Set Dataloader
    img0 = img  # BGR

    # Padded resize
    tmpresultimg = letterbox(img0, new_shape=img_size)
    img = tmpresultimg[0]

    # Normalize RGB
    img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB
    img = np.ascontiguousarray(img, dtype=np.float32)  # uint8 to fp16/fp32
    img /= 255.0  # 0 - 255 to 0.0 - 1.0


    # Get classes and colors
    classes = load_classes(parse_data_cfg(data)['names'])
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(classes))]

    # Run inference
    t0 = time.time()
    
    # Get detections
    
    img = torch.from_numpy(img).unsqueeze(0).to(device)
    #print("img.shape")
    #print(img.shape )
    pred, _ = model(img)
    det = non_max_suppression(pred.float(), conf_thres, nms_thres)[0]
        
    if det is not None and len(det) > 0:
        # Rescale boxes from 416 to true image size
        det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            
        # Print results to screen
        #print("image_size")
        #print('%gx%g ' % img.shape[2:])  # print image size
        for c in det[:, -1].unique():
            n = (det[:, -1] == c).sum()
            print("result")
            print(str(n.item()) + '    ' + classes[int(c)] + '(s)' )
        
        # Draw bounding boxes and labels of detections
        for det_pack in det:
            xyxy = []
            result_obj=[]
            for index in range(4):
                xyxy.append(det_pack[index])
            conf = det_pack[4]
            cls_conf= det_pack[5]
            cls = det_pack[6]
            #print((xyxy,conf, cls_conf, cls ))
            if save_txt:  # Write to file
                with open(save_path + '.txt', 'a') as file:
                    file.write(('%g ' * 6 + '\n') % (xyxy, cls, conf))

            # Add bbox to the image
            label = '%s %.2f' % (classes[int(cls)], conf)
            plot_one_box(xyxy, img0, label=label, color=colors[int(cls)])
           # cv2.imshow('result',img0)
           #  cv2.waitKey(3)
            if save_images:  # Save image with detections
                cv2.imwrite(save_path, img0)
    
    #print('Done. (%.3fs)' % (time.time() - t0))


    
    if save_images:
        #print('Results saved to %s' % os.getcwd() + os.sep + output)
        if platform == 'darwin':  # macos
            os.system('open ' + output + ' ' + save_path)
    if (det is None):
        return [-1, -1]
    if (det.shape[0] <= 0):
        return [-1, -1]
    else:
        cv2.imwrite('ball_detect.jpg', img0)
        cv2.waitKey(3)
        return det


def letterbox(img, new_shape=416, color=(128, 128, 128), mode='auto'):
    # Resize a rectangular image to a 32 pixel multiple rectangle
    # https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]

    if isinstance(new_shape, int):
        ratio = float(new_shape) / max(shape)
    else:
        ratio = max(new_shape) / max(shape)  # ratio  = new / old
    ratiow, ratioh = ratio, ratio
    new_unpad = (int(round(shape[1] * ratio)), int(round(shape[0] * ratio)))

    # Compute padding https://github.com/ultralytics/yolov3/issues/232
    if mode is 'auto':  # minimum rectangle
        dw = np.mod(new_shape - new_unpad[0], 32) / 2  # width padding
        dh = np.mod(new_shape - new_unpad[1], 32) / 2  # height padding
    elif mode is 'square':  # square
        dw = (new_shape - new_unpad[0]) / 2  # width padding
        dh = (new_shape - new_unpad[1]) / 2  # height padding
    elif mode is 'rect':  # square
        dw = (new_shape[1] - new_unpad[0]) / 2  # width padding
        dh = (new_shape[0] - new_unpad[1]) / 2  # height padding
    elif mode is 'scaleFill':
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape, new_shape)
        ratiow, ratioh = new_shape / shape[1], new_shape / shape[0]

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_AREA)  # INTER_AREA is better, INTER_LINEAR is faster
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return (img, ratiow, ratioh, dw, dh)


if __name__ == '__main__':
    img = cv2.imread('data/samples/three_balls_2.jpg')
    model, device = load_weight()
    start = time.time()
    result_obj=detect_ball(model, device, img)
    end = time.time()
    print("time: "+str(end-start)+"s")
    print(result_obj)
