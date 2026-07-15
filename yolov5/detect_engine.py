import cv2
import numpy as np
import tensorrt as trt
import time
import sys
import pycuda.driver as cuda
import pycuda.autoinit

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(engine_path):
    with open(engine_path, 'rb') as f, trt.Runtime(TRT_LOGGER) as runtime:
        return runtime.deserialize_cuda_engine(f.read())

def preprocess(img, input_size=(640, 640)):
    h, w = img.shape[:2]
    r = min(input_size[0] / h, input_size[1] / w)
    new_w, new_h = int(w * r), int(h * r)
    dw, dh = (input_size[1] - new_w) // 2, (input_size[0] - new_h) // 2
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = dh, input_size[0] - new_h - dh
    left, right = dw, input_size[1] - new_w - dw
    img = cv2.copyMakeBorder(img, top, bottom, left, right,
                              cv2.BORDER_CONSTANT, value=(114, 114, 114))
    img = img[:, :, ::-1].transpose(2, 0, 1).astype(np.float32) / 255.0
    return np.ascontiguousarray(img[np.newaxis, ...])

def postprocess(outputs, conf_thres=0.25, iou_thres=0.45):
    # 使用最后一个 output binding（已拼接的 1x25200x85）
    pred = outputs[-1][0]
    boxes = []
    for det in pred:
        obj_score = det[4]
        if obj_score < conf_thres:
            continue
        scores = det[5:] * obj_score
        cls_id = np.argmax(scores)
        score = scores[cls_id]
        if score < conf_thres:
            continue
        x, y, w, h = det[0], det[1], det[2], det[3]
        x1, y1, x2, y2 = x-w/2, y-h/2, x+w/2, y+h/2
        boxes.append([x1, y1, x2, y2, score, cls_id])
    
    if not boxes:
        return np.array([])
    boxes = np.array(boxes)
    indices = cv2.dnn.NMSBoxes(
        boxes[:, :4].tolist(), boxes[:, 4].tolist(),
        conf_thres, iou_thres)
    if indices is None or len(indices) == 0:
        return np.array([])
    return boxes[indices.flatten()]

def scale_boxes(img_shape, boxes, input_size=(640, 640)):
    h, w = img_shape[:2]
    r = min(input_size[0] / h, input_size[1] / w)
    pad_x = (input_size[1] - w * r) / 2
    pad_y = (input_size[0] - h * r) / 2
    scaled = boxes.copy()
    scaled[:, [0, 2]] = (boxes[:, [0, 2]] - pad_x) / r
    scaled[:, [1, 3]] = (boxes[:, [1, 3]] - pad_y) / r
    scaled[:, [0, 2]] = np.clip(scaled[:, [0, 2]], 0, w)
    scaled[:, [1, 3]] = np.clip(scaled[:, [1, 3]], 0, h)
    return scaled

def main():
    engine_path = 'yolov5n_fp32.engine'
    source = sys.argv[1] if len(sys.argv) > 1 else '0'

    print('[INFO] Loading TensorRT engine...')
    engine = load_engine(engine_path)
    context = engine.create_execution_context()

    # 扫描所有 binding
    n_bindings = engine.num_bindings
    print(f'[INFO] {n_bindings} bindings:')
    d_buffers = []
    h_outputs = []
    stream = cuda.Stream()

    for i in range(n_bindings):
        name = engine.get_binding_name(i)
        shape = engine.get_binding_shape(i)
        vol = trt.volume(shape)
        size = vol * trt.float32.itemsize
        d_buf = cuda.mem_alloc(size)
        d_buffers.append(d_buf)
        
        if engine.binding_is_input(i):
            print(f'  INPUT [{i}] {name} {shape}')
        else:
            h_buf = np.empty(tuple(shape), dtype=np.float32)
            h_outputs.append((i, name, shape, h_buf))
            print(f'  OUTPUT [{i}] {name} {shape}')

    if source.isdigit():
        # 摄像头模式
        cap = cv2.VideoCapture(int(source))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fps_history = []

        print('[INFO] Camera ready. Press q to quit.')
        while True:
            t0 = time.time()
            ret, frame = cap.read()
            if not ret:
                break

            input_buf = preprocess(frame)

            # 拷贝输入到 GPU
            cuda.memcpy_htod_async(d_buffers[0], input_buf, stream)
            # 推理（传入所有 device pointer）
            context.execute_async_v2([int(p) for p in d_buffers], stream.handle)
            # 读取所有输出
            for idx, name, shape, h_buf in h_outputs:
                cuda.memcpy_dtoh_async(h_buf, d_buffers[idx], stream)
            stream.synchronize()

            # 取最后一个 output binding 做后处理
            all_outputs = [h_buf for _, _, _, h_buf in h_outputs]
            dets = postprocess(all_outputs)
            
            if len(dets) > 0:
                dets = scale_boxes(frame.shape, dets)
                for d in dets:
                    x1, y1, x2, y2 = d[:4].astype(int); conf = d[4]; cls = int(d[5])
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, f'{int(cls)} {conf:.2f}', (x1, y1-5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

            fps = 1.0 / (time.time() - t0)
            fps_history.append(fps)
            if len(fps_history) > 30:
                fps_history.pop(0)
            avg_fps = sum(fps_history) / len(fps_history)
            cv2.putText(frame, f'FPS: {avg_fps:.1f}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            cv2.imshow('Detection', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()
    else:
        # 图片模式
        img = cv2.imread(source)
        if img is None:
            print(f'[ERROR] Cannot read {source}')
            return
        input_buf = preprocess(img)

        cuda.memcpy_htod_async(d_buffers[0], input_buf, stream)
        context.execute_async_v2([int(p) for p in d_buffers], stream.handle)
        for idx, name, shape, h_buf in h_outputs:
            cuda.memcpy_dtoh_async(h_buf, d_buffers[idx], stream)
        stream.synchronize()

        all_outputs = [h_buf for _, _, _, h_buf in h_outputs]
        dets = postprocess(all_outputs)
        print(f'[INFO] Detected {len(dets)} objects')
        if len(dets) > 0:
            dets = scale_boxes(img.shape, dets)
            for d in dets:
                x1, y1, x2, y2 = d[:4].astype(int); conf = d[4]; cls = int(d[5])
                print(f'  class={int(cls)}, conf={conf:.3f}, box=({x1},{y1},{x2},{y2})')
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(img, f'{int(cls)} {conf:.2f}', (x1, y1-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
        else:
            print('  (no objects found)')
        cv2.imwrite('result.jpg', img)
        print('[INFO] Result saved to result.jpg')

if __name__ == '__main__':
    main()
