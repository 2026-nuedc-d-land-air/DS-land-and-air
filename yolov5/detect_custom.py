import cv2, numpy as np, tensorrt as trt, sys, time
import pycuda.driver as cuda, pycuda.autoinit

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
engine = trt.Runtime(TRT_LOGGER).deserialize_cuda_engine(open('best_fp32.engine','rb').read())
ctx = engine.create_execution_context()
stream = cuda.Stream()

d_bufs = [cuda.mem_alloc(trt.volume(engine.get_binding_shape(i))*4) for i in range(engine.num_bindings)]
out_shape = engine.get_binding_shape(4)
h_out = np.empty(tuple(out_shape), dtype=np.float32)

# ====== 改成你的类名 ======
names = ['object', 'object2']

def letterbox(img, new=(256,256), color=(114,114,114)):
    h,w = img.shape[:2]; r = min(new[0]/h, new[1]/w)
    nw,nh = int(w*r), int(h*r); dw,dh = (new[1]-nw)//2, (new[0]-nh)//2
    im = cv2.copyMakeBorder(cv2.resize(img,(nw,nh)),dh,new[0]-nh-dh,dw,new[1]-nw-dw,cv2.BORDER_CONSTANT,value=color)
    return im, r, dw, dh

source = sys.argv[1] if len(sys.argv) > 1 else '0'
if source.isdigit():
    cap = cv2.VideoCapture(int(source))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    fps_h = []
    print(f'[INFO] Detecting: {names}')
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret: break
        im, r, dw, dh = letterbox(frame, (256,256))
        inp = np.ascontiguousarray(im[:,:,::-1].transpose(2,0,1)[np.newaxis].astype(np.float32)/255.0)
        cuda.memcpy_htod_async(d_bufs[0], inp, stream)
        ctx.execute_async_v2([int(p) for p in d_bufs], stream.handle)
        cuda.memcpy_dtoh_async(h_out, d_bufs[4], stream)
        stream.synchronize()

        raw = h_out[0]
        obj = 1.0/(1.0+np.exp(-raw[:,4]))
        mask = obj > 0.3
        if np.any(mask):
            bx = raw[mask]; ob = obj[mask]
            ci = bx[:,5:].argmax(axis=1)
            cls_max = np.maximum(bx[:,5:], 0)
            sc = ob * (1.0/(1.0+np.exp(-cls_max.max(axis=1))))
            mask2 = sc > 0.3
            if np.any(mask2):
                bx2 = bx[mask2]; sc2 = sc[mask2]; ci2 = ci[mask2]
                cx,cy,w,h = bx2[:,0],bx2[:,1],bx2[:,2],bx2[:,3]
                boxes = np.column_stack([cx-w/2, cy-h/2, cx+w/2, cy+h/2])
                nms_idx = cv2.dnn.NMSBoxes(boxes.tolist(),sc2.tolist(),0.3,0.45)
                if nms_idx is not None and len(nms_idx) > 0:
                    for idx in nms_idx.flatten():
                        x1=int((boxes[idx,0]-dw)/r); y1=int((boxes[idx,1]-dh)/r)
                        x2=int((boxes[idx,2]-dw)/r); y2=int((boxes[idx,3]-dh)/r)
                        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                        label = f'{names[ci2[idx]] if ci2[idx] < len(names) else ci2[idx]} {sc2[idx]:.2f}'
                        cv2.putText(frame,label,(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
        fps = 1/(time.time()-t0)
        fps_h.append(fps)
        if len(fps_h) > 30: fps_h.pop(0)
        cv2.putText(frame,f'FPS:{sum(fps_h)/len(fps_h):.1f}',(10,30),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,255),2)
        cv2.imshow('Detection',frame)
        if cv2.waitKey(1)&0xFF==ord('q'): break
    cap.release(); cv2.destroyAllWindows()
else:
    img = cv2.imread(source)
    im, r, dw, dh = letterbox(img, (256,256))
    inp = np.ascontiguousarray(im[:,:,::-1].transpose(2,0,1)[np.newaxis].astype(np.float32)/255.0)
    cuda.memcpy_htod_async(d_bufs[0], inp, stream)
    ctx.execute_async_v2([int(p) for p in d_bufs], stream.handle)
    cuda.memcpy_dtoh_async(h_out, d_bufs[4], stream)
    stream.synchronize()
    raw = h_out[0]
    obj = 1.0/(1.0+np.exp(-raw[:,4]))
    mask = obj > 0.3
    if np.any(mask):
        bx = raw[mask]; ob = obj[mask]
        ci = bx[:,5:].argmax(axis=1)
        sc = ob * (1.0/(1.0+np.exp(-np.maximum(bx[:,5:], 0).max(axis=1))))
        mask2 = sc > 0.3
        if np.any(mask2):
            bx2 = bx[mask2]; sc2 = sc[mask2]; ci2 = ci[mask2]
            cx,cy,w,h = bx2[:,0],bx2[:,1],bx2[:,2],bx2[:,3]
            boxes = np.column_stack([cx-w/2, cy-h/2, cx+w/2, cy+h/2])
            nms_idx = cv2.dnn.NMSBoxes(boxes.tolist(),sc2.tolist(),0.3,0.45)
            if nms_idx is not None:
                for idx in nms_idx.flatten():
                    x1=int((boxes[idx,0]-dw)/r); y1=int((boxes[idx,1]-dh)/r)
                    x2=int((boxes[idx,2]-dw)/r); y2=int((boxes[idx,3]-dh)/r)
                    cv2.rectangle(img,(x1,y1),(x2,y2),(0,255,0),2)
                    label = f'{names[ci2[idx]] if ci2[idx] < len(names) else ci2[idx]} {sc2[idx]:.2f}'
                    cv2.putText(img,label,(x1,y1-5),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
    cv2.imwrite('result.jpg',img)
    print('Done')
