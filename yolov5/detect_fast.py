import cv2, numpy as np, tensorrt as trt, sys, time
import pycuda.driver as cuda, pycuda.autoinit

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(path):
    with open(path, 'rb') as f, trt.Runtime(TRT_LOGGER) as r:
        return r.deserialize_cuda_engine(f.read())

def letterbox(img, new=(640,640), color=(114,114,114)):
    h,w = img.shape[:2]
    r = min(new[0]/h, new[1]/w)
    nw,nh = int(w*r), int(h*r)
    dw,dh = (new[1]-nw)//2, (new[0]-nh)//2
    im = cv2.resize(img, (nw,nh))
    im = cv2.copyMakeBorder(im, dh, new[0]-nh-dh, dw, new[1]-nw-dw, cv2.BORDER_CONSTANT, value=color)
    return im, r, dw, dh

STRIDES = [8, 16, 32]

def decode_output(out, stride, conf_th=0.25):
    out = out[0]
    grid_h, grid_w = out.shape[1:3]
    boxes = []
    for i in range(3):
        anchor_out = out[i]
        for y in range(grid_h):
            for x in range(grid_w):
                det = anchor_out[y, x]
                obj = 1 / (1 + np.exp(-det[4]))
                if obj < conf_th:
                    continue
                cls_scores = det[5:]
                cls_id = np.argmax(cls_scores)
                cls_conf = 1/(1+np.exp(-cls_scores[cls_id])) * obj
                if cls_conf < conf_th:
                    continue
                cx = (x + 1/(1+np.exp(-det[0]))) * stride
                cy = (y + 1/(1+np.exp(-det[1]))) * stride
                bw = np.exp(det[2]) * stride
                bh = np.exp(det[3]) * stride
                boxes.append([cx-bw/2, cy-bh/2, cx+bw/2, cy+bh/2, cls_conf, cls_id])
    return np.array(boxes) if boxes else np.empty((0,6))

engine = load_engine('yolov5n_fp16_raw.engine')
ctx = engine.create_execution_context()
stream = cuda.Stream()

# 为所有 5 个 binding 分配 GPU 内存
n = engine.num_bindings  # = 5
d_bufs = []
h_outs = {}  # idx -> host buffer for output bindings
for i in range(n):
    vol = trt.volume(engine.get_binding_shape(i))
    d = cuda.mem_alloc(vol * 4)
    d_bufs.append(d)
    if not engine.binding_is_input(i):
        shape = engine.get_binding_shape(i)
        h_outs[i] = np.empty(tuple(shape), dtype=np.float32)

# 只读前 3 个输出层
OUT_IDXS = [1, 2, 3]  # 跳过的 binding 4

source = sys.argv[1] if len(sys.argv) > 1 else '0'

if source.isdigit():
    cap = cv2.VideoCapture(int(source))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fps_h = []
    print('[INFO] Camera running. Press q to quit.')
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret: break
        im, r, dw, dh = letterbox(frame)
        inp = im[:,:,::-1].transpose(2,0,1).astype(np.float32)[np.newaxis]/255.0
        inp = np.ascontiguousarray(inp)
        cuda.memcpy_htod_async(d_bufs[0], inp, stream)
        ctx.execute_async_v2([int(p) for p in d_bufs], stream.handle)
        for idx in OUT_IDXS:
            cuda.memcpy_dtoh_async(h_outs[idx], d_bufs[idx], stream)
        stream.synchronize()
        all_dets = []
        for si, idx in enumerate(OUT_IDXS):
            dets = decode_output(h_outs[idx], STRIDES[si])
            if len(dets) > 0: all_dets.append(dets)
        if all_dets:
            all_dets = np.vstack(all_dets)
            nms_idx = cv2.dnn.NMSBoxes(all_dets[:,:4].tolist(), all_dets[:,4].tolist(), 0.25, 0.45)
            if nms_idx is not None and len(nms_idx) > 0:
                for idx in nms_idx.flatten():
                    d = all_dets[idx]
                    x1 = int((d[0]-dw)/r)
                    y1 = int((d[1]-dh)/r)
                    x2 = int((d[2]-dw)/r)
                    y2 = int((d[3]-dh)/r)
                    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.putText(frame,f'{int(d[5])} {d[4]:.2f}',(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        fps = 1/(time.time()-t0)
        fps_h.append(fps)
        if len(fps_h) > 30: fps_h.pop(0)
        cv2.putText(frame,f'FPS:{sum(fps_h)/len(fps_h):.1f}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        cv2.imshow('FastDetect',frame)
        if cv2.waitKey(1)&0xFF==ord('q'): break
    cap.release()
    cv2.destroyAllWindows()
else:
    img = cv2.imread(source)
    im, r, dw, dh = letterbox(img)
    inp = im[:,:,::-1].transpose(2,0,1).astype(np.float32)[np.newaxis]/255.0
    inp = np.ascontiguousarray(inp)
    cuda.memcpy_htod_async(d_bufs[0], inp, stream)
    ctx.execute_async_v2([int(p) for p in d_bufs], stream.handle)
    for si, idx in enumerate(OUT_IDXS):
        cuda.memcpy_dtoh_async(h_outs[idx], d_bufs[idx], stream)
    stream.synchronize()
    all_dets = []
    for si, idx in enumerate(OUT_IDXS):
        dets = decode_output(h_outs[idx], STRIDES[si])
        if len(dets) > 0: all_dets.append(dets)
    if all_dets:
        all_dets = np.vstack(all_dets)
        nms_idx = cv2.dnn.NMSBoxes(all_dets[:,:4].tolist(), all_dets[:,4].tolist(), 0.25, 0.45)
        if nms_idx is not None:
            for idx in nms_idx.flatten():
                d = all_dets[idx]
                x1=int((d[0]-dw)/r)
                y1=int((d[1]-dh)/r)
                x2=int((d[2]-dw)/r)
                y2=int((d[3]-dh)/r)
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(img,f'{int(d[5])} {d[4]:.2f}',(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
    cv2.imwrite('result.jpg',img)
    print(f'Done')
