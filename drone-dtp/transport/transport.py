import zmq
import json
import cv2
import numpy as np
import struct

class ZmqPublisher:
    def __init__(self, port=5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        
        # High Water Mark of 2 + CONFLATE ensures zero lag
        self.socket.setsockopt(zmq.SNDHWM, 2)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        
        self.socket.bind(f"tcp://*:{port}")
        print(f"[INFO] ZMQ Publisher bound to tcp://*:{port} (CONFLATE enabled)")

    def send(self, frame, metadata, quality=50, resize_width=640):
        # 1. Resize frame for network efficiency
        if resize_width and frame.shape[1] > resize_width:
            h, w = frame.shape[:2]
            scale = resize_width / float(w)
            new_h = int(h * scale)
            frame = cv2.resize(frame, (resize_width, new_h), interpolation=cv2.INTER_NEAREST)

        # 2. JPEG Compress
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, buffer = cv2.imencode('.jpg', frame, encode_param)
        
        # 3. Serialize metadata
        meta_json = json.dumps(metadata).encode('utf-8')
        image_bytes = buffer.tobytes()
        
        # 4. PACK INTO A SINGLE MESSAGE FOR CONFLATE
        # Format: [4-byte Unsigned Int (length of JSON)] + [JSON Bytes] + [Image Bytes]
        meta_len_header = struct.pack('<I', len(meta_json))
        single_message = meta_len_header + meta_json + image_bytes
        
        # Send as a single frame
        self.socket.send(single_message)

class ZmqSubscriber:
    def __init__(self, ip_address, port=5555):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        
        # Ensure receiver also drops old frames
        self.socket.setsockopt(zmq.RCVHWM, 2)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        
        self.socket.connect(f"tcp://{ip_address}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        print(f"[INFO] ZMQ Subscriber connected to tcp://{ip_address}:{port} (CONFLATE enabled)")

    def receive(self, timeout_ms=100):
        if self.socket.poll(timeout_ms):
            # Receive the single combined frame
            single_message = self.socket.recv()
            
            # 1. Extract the 4-byte header to find the JSON length
            meta_len = struct.unpack('<I', single_message[:4])[0]
            
            # 2. Split the bytes based on that length
            meta_json = single_message[4 : 4 + meta_len]
            image_bytes = single_message[4 + meta_len :]
            
            # 3. Decode
            metadata = json.loads(meta_json.decode('utf-8'))
            nparr = np.frombuffer(image_bytes, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            return frame, metadata
            
        return None, None