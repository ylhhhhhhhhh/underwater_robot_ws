#!/usr/bin/env python3
import cv2
import numpy as np
from rknnlite.api import RKNNLite
import time
import rclpy
from rclpy.node import Node
import socket
import os
import threading
from ament_index_python.packages import get_package_share_directory

# ------------------------- 配置 -------------------------
MODEL_PATH = os.path.join(
    get_package_share_directory("yolov5_ros2_rknn"),
    "resource", "underwater.rknn"
)
ANCHORS_TXT_PATH = os.path.join(
    get_package_share_directory("yolov5_ros2_rknn"),
    "resource", "underwater_anchors.txt"
)

CAM_DEVICE = 21
CAM_WIDTH = 640
CAM_HEIGHT = 480

# ===================== 【解决重叠：超强NMS】 =====================
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
# ================================================================

IMG_SIZE = (640, 640)

JPEG_QUALITY = 80
SERVER_IP = "192.168.2.100"
PORT_RAW = 6001
PORT_YOLO = 6002
SEND_RAW_IMAGE = True
SEND_YOLO_IMAGE = True

# ------------------------- 工具函数 -------------------------
def letter_box(im, new_shape, pad_color=(0, 0, 0)):
    shape = im.shape[:2]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)

    return im, r, (dw, dh)

def box_unletter(boxes, ratio, pad, origin_shape):
    bbox = np.copy(boxes)
    dw, dh = pad
    origin_h, origin_w = origin_shape

    bbox[:, 0] = np.clip((bbox[:, 0] - dw) / ratio, 0, origin_w)
    bbox[:, 1] = np.clip((bbox[:, 1] - dh) / ratio, 0, origin_h)
    bbox[:, 2] = np.clip((bbox[:, 2] - dw) / ratio, 0, origin_w)
    bbox[:, 3] = np.clip((bbox[:, 3] - dh) / ratio, 0, origin_h)

    return bbox

def filter_boxes(boxes, box_confidences, box_class_probs, obj_thresh):
    box_confidences = box_confidences.reshape(-1)
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    valid_mask = class_max_score * box_confidences >= obj_thresh
    scores = (class_max_score * box_confidences)[valid_mask]
    boxes = boxes[valid_mask]
    classes = classes[valid_mask]
    return boxes, classes, scores

def nms_boxes(boxes, scores, iou_thres=0.45):
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1 + 1e-6) * (y2 - y1 + 1e-6)
    order = scores.argsort()[::-1]  # 得分从高到低

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)

        # 计算 DIoU（官方同款，惩罚中心距离！）
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1 + 1e-6)
        h = np.maximum(0.0, yy2 - yy1 + 1e-6)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

        # 额外加中心距离惩罚（大小框必删）
        cx1 = (x1[i] + x2[i]) / 2
        cy1 = (y1[i] + y2[i]) / 2
        cx2 = (x1[order[1:]] + x2[order[1:]]) / 2
        cy2 = (y1[order[1:]] + y2[order[1:]]) / 2
        dist2 = (cx1 - cx2) ** 2 + (cy1 - cy2) ** 2

        cw = np.maximum(x2[i], x2[order[1:]]) - np.minimum(x1[i], x1[order[1:]])
        ch = np.maximum(y2[i], y2[order[1:]]) - np.minimum(y1[i], y1[order[1:]])
        diag2 = cw ** 2 + ch ** 2 + 1e-6

        diou = iou - dist2 / diag2

        # 官方逻辑：只要 DIoU 大于阈值，全部删掉！
        inds = np.where(diou <= iou_thres)[0]
        order = order[inds + 1]

    return keep




def box_process(position, anchors):
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    stride = np.array([IMG_SIZE[1] // grid_h, IMG_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)

    anchors_np = np.array(anchors)
    col = col.repeat(len(anchors_np), axis=0)
    row = row.repeat(len(anchors_np), axis=0)
    anchors_reshaped = anchors_np.reshape(*anchors_np.shape, 1, 1)

    if position.shape[1] < 2:
        return np.empty((0, 4, grid_h, grid_w))
    box_xy = position[:, :2, :, :] * 2 - 0.5

    if position.shape[1] < 4:
        box_wh = np.ones_like(box_xy) * anchors_reshaped[:2]
    else:
        box_wh = pow(position[:, 2:4, :, :] * 2, 2) * anchors_reshaped

    grid_xy = np.concatenate((col, row), axis=1)
    box_xy += grid_xy
    box_xy *= stride

    xyxy = np.zeros((box_xy.shape[0], 4, grid_h, grid_w), dtype=np.float32)
    xyxy[:, 0, :, :] = box_xy[:, 0, :, :] - box_wh[:, 0, :, :] / 2
    xyxy[:, 1, :, :] = box_xy[:, 1, :, :] - box_wh[:, 1, :, :] / 2
    xyxy[:, 2, :, :] = box_xy[:, 0, :, :] + box_wh[:, 0, :, :] / 2
    xyxy[:, 3, :, :] = box_xy[:, 1, :, :] + box_wh[:, 1, :, :] / 2

    return xyxy

def post_process(input_data, anchors):
    boxes, scores, classes_conf = [], [], []

    if not isinstance(input_data, list) or len(input_data) == 0:
        return None, None, None

    try:
        input_data = [_in.reshape([len(anchors[0]), -1] + list(_in.shape[-2:])) for _in in input_data]
    except Exception as e:
        return None, None, None

    valid_feature_num = min(len(input_data), len(anchors))
    for i in range(valid_feature_num):
        if input_data[i].shape[1] < 5:
            continue
        boxes.append(box_process(input_data[i][:, :4, :, :], anchors[i]))
        scores.append(input_data[i][:, 4:5, :, :])
        classes_conf.append(input_data[i][:, 5:, :, :])

    if len(boxes) == 0 or len(scores) == 0 or len(classes_conf) == 0:
        return None, None, None

    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    try:
        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]
    except:
        return None, None, None

    if any(len(arr) == 0 for arr in boxes):
        return None, None, None

    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    try:
        boxes, classes, scores = filter_boxes(boxes, scores, classes_conf, OBJ_THRESH)
    except:
        return None, None, None

    if len(boxes) == 0:
        return None, None, None

    # =============== 【修复重叠：全局NMS，跨层去重】 ===============
    keep = nms_boxes(boxes, scores, NMS_THRESH)
    boxes = boxes[keep]
    classes = classes[keep]
    scores = scores[keep]
    # ============================================================

    return boxes, classes, scores

def draw_detection_result(image, boxes, scores, classes, classes_list):
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = [int(_b) for _b in box]
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 255), 2)
        corner_length = 15
        cv2.line(image, (top, left), (top + corner_length, left), (0, 255, 0), 3)
        cv2.line(image, (top, left), (top, left + corner_length), (0, 255, 0), 3)
        cv2.line(image, (right, left), (right - corner_length, left), (0, 255, 0), 3)
        cv2.line(image, (right, left), (right, left + corner_length), (0, 255, 0), 3)
        cv2.line(image, (top, bottom), (top + corner_length, bottom), (0, 255, 0), 3)
        cv2.line(image, (top, bottom), (top, bottom - corner_length), (0, 255, 0), 3)
        cv2.line(image, (right, bottom), (right - corner_length, bottom), (0, 255, 0), 3)
        cv2.line(image, (right, bottom), (right, bottom - corner_length), (0, 255, 0), 3)
        label = f"{classes_list[cl]} {score:.2f}"
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        label_y = max(left, label_size[1] + 10)
        cv2.rectangle(image, (top, label_y - label_size[1] - 10),
                      (top + label_size[0], label_y), (255, 0, 255), -1)
        cv2.putText(image, label, (top, label_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

def read_anchors_from_txt(txt_path):
    try:
        with open(txt_path, 'r', encoding='utf-8') as f:
            anchor_lines = [line.strip() for line in f.readlines() if line.strip()]

        anchor_nums = [float(num) for num in anchor_lines]
        if len(anchor_nums) != 18:
            raise ValueError(f"txt文件应包含18个数值，当前读取到{len(anchor_nums)}个")

        anchors = np.array(anchor_nums).reshape(3, 3, 2).tolist()
        return anchors

    except FileNotFoundError:
        print(f"❌ 锚点txt文件不存在：{txt_path}")
        exit(1)
    except Exception as e:
        print(f"❌ 读取锚点txt文件失败：{e}")
        exit(1)

# ------------------------- TCP 图传 -------------------------
class TcpImageSender:
    def __init__(self, server_ip, port, name, quality=80):
        self.server_ip = server_ip
        self.port = port
        self.name = name
        self.quality = quality
        self.sock = None
        self.frame = None
        self.lock = threading.Lock()
        self.connected = False
        threading.Thread(target=self._connect_thread, daemon=True).start()
        threading.Thread(target=self._send_thread, daemon=True).start()

    def _connect_thread(self):
        while True:
            time.sleep(0.5)
            if self.connected:
                continue
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(2)
                self.sock.connect((self.server_ip, self.port))
                self.sock.settimeout(None)
                self.connected = True
            except Exception:
                self.connected = False
                time.sleep(1)

    def update_frame(self, frame):
        with self.lock:
            self.frame = frame.copy()

    def _send_thread(self):
        while True:
            time.sleep(0.01)
            if not self.connected or self.frame is None:
                continue
            try:
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
                ret, jpg = cv2.imencode('.jpg', self.frame, encode_param)
                if not ret:
                    continue
                data = jpg.tobytes()
                self.sock.send(len(data).to_bytes(4, 'big'))
                self.sock.sendall(data)
            except Exception:
                self.connected = False
                try:
                    self.sock.close()
                except:
                    pass

# ------------------------- YOLO NODE -------------------------
class YoloRknnRos2Node(Node):
    def __init__(self):
        super().__init__("yolo_rknn_elf2_node")

        self.sender_raw = TcpImageSender(SERVER_IP, PORT_RAW, "原始图", JPEG_QUALITY)
        self.sender_yolo = TcpImageSender(SERVER_IP, PORT_YOLO, "YOLO图", JPEG_QUALITY)

        self.CLASSES = ('holothurian', 'echinus', 'scallop', 'starfish')
        self.ANCHORS = read_anchors_from_txt(ANCHORS_TXT_PATH)

        self.rknn = RKNNLite(verbose=0)
        self.rknn.load_rknn(MODEL_PATH)
        self.rknn.init_runtime()

        self.cap = cv2.VideoCapture(CAM_DEVICE)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)

        self.detection_infinite_loop()

    def detection_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame_origin = frame.copy()
        origin_shape = frame_origin.shape[:2]

        if SEND_RAW_IMAGE:
            self.sender_raw.update_frame(frame_origin)

        img, ratio, pad = letter_box(frame_origin, new_shape=IMG_SIZE, pad_color=(0, 0, 0))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_input = np.expand_dims(img, axis=0)

        outputs = self.rknn.inference(inputs=[img_input], data_format=['nhwc'])

        if outputs is not None:
            boxes, classes, scores = post_process(outputs, self.ANCHORS)
            if boxes is not None:
                boxes = box_unletter(boxes, ratio, pad, origin_shape)
                draw_detection_result(frame, boxes, scores, classes, self.CLASSES)

        if SEND_YOLO_IMAGE:
            self.sender_yolo.update_frame(frame)

    def detection_infinite_loop(self):
        while rclpy.ok():
            self.detection_loop()
            rclpy.spin_once(self, timeout_sec=0.001)

    def destroy_node(self):
        self.cap.release()
        self.rknn.release()
        cv2.destroyAllWindows()
        super().destroy_node()

# ------------------------- 主函数 -------------------------
def main(args=None):
    rclpy.init(args=args)
    yolo_rknn_node = YoloRknnRos2Node()
    rclpy.spin(yolo_rknn_node)
    yolo_rknn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
