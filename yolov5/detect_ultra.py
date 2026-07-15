import cv2, numpy as np, tensorrt as trt, sys, time
import pycuda.driver as cuda, pycuda.autoinit

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def load_engine(path):
    with open(path, 'rb') as f, trt.Runtime(TRT_LOGGER) as r:
        return r.deserialize_cuda_engine(f.read())

def letterbox(img, new=(320,320), color=(114,114,114)):
    h,w = img.shape[:2]; r = min(new[0]/h, new[1]/w)
    nw,nh = int(w*r), int(h*r); dw,dh = (new[1]-nw)//2, (new[0]-nh)//2
    im = cv2.resize(img, (nw,nh), interpolation=cv2.INTER_LINEAR)
    im = cv2.copyMakeBorder(im, dh, new[0]-nh-dh, dw, new[1]-nw-dw, cv2.BORDER_CONSTANT, value=color)
    return im, r, dw, dh

engine = load_engine('yolov5n_fp32_320.engine')
ctx = engine.create_execution_context()
stream = cuda.Stream()

d_bufs = [cuda.mem_alloc(trt.volume(engine.get_binding_shape(i))*4) for i in range(engine.num_bindings)]
h_out = np.empty((1, 6300, 85), dtype=np.float32)

source = sys.argv[1] if len(sys.argv) > 1 else '0'
OUT_IDX = 4  # concat output

if source.isdigit():
    cap = cv2.VideoCapture(int(source))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fps_h = []
    print('[INFO] Camera running. Press q to quit.')
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret: break

        im, r, dw, dh = letterbox(frame, (320,320))
        inp = np.ascontiguousarray(im[:,:,::-1].transpose(2,0,1)[np.newaxis].astype(np.float32)/255.0)
        cuda.memcpy_htod_async(d_bufs[0], inp, stream)
        ctx.execute_async_v2([int(p) for p in d_bufs], stream.handle)
        cuda.memcpy_dtoh_async(h_out, d_bufs[OUT_IDX], stream)
        stream.synchronize()

        pred = h_out[0]
        obj = 1.0/(1.0+np.exp(-pred[:,4]))
        cls = 1.0/(1.0+np.exp(-pred[:,5:]))
        cls_max = cls.max(axis=1); cls_ids = cls.argmax(axis=1)
        scores = cls_max * obj

        keep = scores > 0.25
        if np.any(keep):
            bx = pred[keep]; sc = scores[keep]; ci = cls_ids[keep]
            cx,cy,w,h = bx[:,0],bx[:,1],bx[:,2],bx[:,3]
            boxes = np.column_stack([cx-w/2, cy-h/2, cx+w/2, cy+h/2])
            nms_idx = cv2.dnn.NMSBoxes(boxes.tolist(),sc.tolist(),0.25,0.45)
            if nms_idx is not None and len(nms_idx) > 0:
                for idx in nms_idx.flatten():
                    x1=int((boxes[idx,0]-dw)/r); y1=int((boxes[idx,1]-dh)/r)
                    x2=int((boxes[idx,2]-dw)/r); y2=int((boxes[idx,3]-dh)/r)
                    cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                    cv2.putText(frame,f'{int(ci[idx])} {sc[idx]:.2f}',(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)

        fps = 1/(time.time()-t0)
        fps_h.append(fps)
        if len(fps_h) > 30: fps_h.pop(0)
        cv2.putText(frame,f'FPS:{sum(fps_h)/len(fps_h):.1f}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        cv2.imshow('Detection',frame)
        if cv2.waitKey(1)&0xFF==ord('q'): break
    cap.release(); cv2.destroyAllWindows()
else:
    img = cv2.imread(source)
    im, r, dw, dh = letterbox(img, (320,320))
    inp = np.ascontiguousarray(im[:,:,::-1].transpose(2,0,1)[np.newaxis].astype(np.float32)/255.0)
    cuda.memcpy_htod_async(d_bufs[0], inp, stream)
    ctx.execute_async_v2([int(p) for p in d_bufs], stream.handle)
    cuda.memcpy_dtoh_async(h_out, d_bufs[OUT_IDX], stream)
    stream.synchronize()

    pred = h_out[0]
    obj = 1.0/(1.0+np.exp(-pred[:,4]))
    cls = 1.0/(1.0+np.exp(-pred[:,5:]))
    cls_max = cls.max(axis=1); cls_ids = cls.argmax(axis=1)
    scores = cls_max * obj

    keep = scores > 0.25
    if np.any(keep):
        bx = pred[keep]; sc = scores[keep]; ci = cls_ids[keep]
        cx,cy,w,h = bx[:,0],bx[:,1],bx[:,2],bx[:,3]
        boxes = np.column_stack([cx-w/2, cy-h/2, cx+w/2, cy+h/2])
        nms_idx = cv2.dnn.NMSBoxes(boxes.tolist(),sc.tolist(),0.25,0.45)
        if nms_idx is not None:
            for idx in nms_idx.flatten():
                x1=int((boxes[idx,0]-dw)/r); y1=int((boxes[idx,1]-dh)/r)
                x2=int((boxes[idx,2]-dw)/r); y2=int((boxes[idx,3]-dh)/r)
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(img,f'{int(ci[idx])} {sc[idx]:.2f}',(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
    cv2.imwrite('result.jpg',img)
    print('Done')
