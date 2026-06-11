#!/usr/bin/env python3
# 仅依赖板端必备库+ROS2 Humble相关库
import cv2
import numpy as np
from rknnlite.api import RKNNLite
import time
# ROS2 Humble 必要导入
import rclpy
from rclpy.node import Node
import os
import socket
import threading

# ------------------------- 宏定义配置（替换原命令行参数，修改此处即可调整配置）-------------------------
# 文件路径配置（模型和锚点文件放置在resource目录下）
MODEL_PATH = os.path.join(os.path.dirname(__file__), "underwater.rknn")

ANCHORS_TXT_PATH = os.path.join(os.path.dirname(__file__), "underwater_anchors.txt")
# 摄像头配置
CAM_DEVICE = 21
CAM_WIDTH = 640
CAM_HEIGHT = 480
# 检测阈值配置
OBJ_THRESH = 0.5    # 目标置信度阈值
NMS_THRESH = 0.65   # NMS非极大值抑制阈值
IMG_SIZE = (640, 640)  # YOLO模型输入尺寸（需与转换RKNN模型时的尺寸一致）
# ROS2 话题配置
VIDEO_TOPIC_NAME = "video_yolo"

# ===================== 【以太网图传配置 - 仅新增部分】 =====================
JPEG_QUALITY = 80
SERVER_IP = "192.168.2.100"
PORT_RAW = 6001
PORT_YOLO = 6002

# 发送开关（True=发送，False=不发送）
SEND_RAW_IMAGE = True
SEND_YOLO_IMAGE = True
# ==========================================================================

# ------------------------- 核心辅助函数（完全保留原有实现，未做任何修改）-------------------------
def letter_box(im, new_shape, pad_color=(0, 0, 0)):
    """
    保持长宽比缩放+黑边填充（避免目标畸变，适配YOLO输入尺寸）
    """
    shape = im.shape[:2]  # 原始图像尺寸 [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # 计算缩放比例（取宽高缩放的最小值，保证图像完整放入目标尺寸）
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw /= 2  # 左右对称填充
    dh /= 2  # 上下对称填充

    # 缩放图像（双线性插值，板端性能友好）
    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    # 填充黑边（边界常量填充，适配模型输入尺寸）
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=pad_color)

    return im, r, (dw, dh)

def box_unletter(boxes, ratio, pad, origin_shape):
    """
    边界框还原（消除缩放和填充的影响，映射回原始图像尺寸）
    """
    bbox = np.copy(boxes)
    dw, dh = pad
    origin_h, origin_w = origin_shape

    # 还原坐标并限制在图像范围内，避免越界
    bbox[:, 0] = np.clip((bbox[:, 0] - dw) / ratio, 0, origin_w)
    bbox[:, 1] = np.clip((bbox[:, 1] - dh) / ratio, 0, origin_h)
    bbox[:, 2] = np.clip((bbox[:, 2] - dw) / ratio, 0, origin_w)
    bbox[:, 3] = np.clip((bbox[:, 3] - dh) / ratio, 0, origin_h)

    return bbox

def filter_boxes(boxes, box_confidences, box_class_probs, obj_thresh):
    """过滤低置信度目标框，保留有效候选框"""
    box_confidences = box_confidences.reshape(-1)
    class_max_score = np.max(box_class_probs, axis=-1)
    classes = np.argmax(box_class_probs, axis=-1)
    # 筛选出 置信度*类别概率 大于阈值的框
    valid_mask = class_max_score * box_confidences >= obj_thresh
    scores = (class_max_score * box_confidences)[valid_mask]
    boxes = boxes[valid_mask]
    classes = classes[valid_mask]
    return boxes, classes, scores

def nms_boxes(boxes, scores, nms_thresh):
    """非极大值抑制，移除冗余重叠框（板端纯numpy实现，高效稳定）"""
    x = boxes[:, 0]
    y = boxes[:, 1]
    w = boxes[:, 2] - boxes[:, 0]
    h = boxes[:, 3] - boxes[:, 1]
    areas = w * h
    order = scores.argsort()[::-1]  # 按得分从高到低排序
    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)
        # 计算当前框与其他框的交并比（IoU）
        xx1 = np.maximum(x[i], x[order[1:]])
        yy1 = np.maximum(y[i], y[order[1:]])
        xx2 = np.minimum(x[i] + w[i], x[order[1:]] + w[order[1:]])
        yy2 = np.minimum(y[i] + h[i], y[order[1:]] + h[order[1:]])
        inter_w = np.maximum(0.0, xx2 - xx1 + 0.00001)
        inter_h = np.maximum(0.0, yy2 - yy1 + 0.00001)
        inter_area = inter_w * inter_h
        iou = inter_area / (areas[i] + areas[order[1:]] - inter_area)
        # 保留IoU小于阈值的框（去除重叠度高的冗余框）
        valid_inds = np.where(iou <= nms_thresh)[0]
        order = order[valid_inds + 1]

    return np.array(keep)

def box_process(position, anchors):
    """解析YOLO输出特征图，转换为边界框坐标（适配YOLOv5锚点机制，修复维度越界）"""
    grid_h, grid_w = position.shape[2:4]
    col, row = np.meshgrid(np.arange(0, grid_w), np.arange(0, grid_h))
    col = col.reshape(1, 1, grid_h, grid_w)
    row = row.reshape(1, 1, grid_h, grid_w)
    stride = np.array([IMG_SIZE[1] // grid_h, IMG_SIZE[0] // grid_w]).reshape(1, 2, 1, 1)

    # 锚点适配（先转numpy数组，避免list无shape错误）
    anchors_np = np.array(anchors)
    col = col.repeat(len(anchors_np), axis=0)
    row = row.repeat(len(anchors_np), axis=0)
    anchors_reshaped = anchors_np.reshape(*anchors_np.shape, 1, 1)

    # 解析边界框中心坐标和宽高（确保切片有效，避免空数组）
    if position.shape[1] < 2:
        print("⚠️  position数组维度不足，无法解析xy坐标")
        return np.empty((0, 4, grid_h, grid_w))
    box_xy = position[:, :2, :, :] * 2 - 0.5

    if position.shape[1] < 4:
        # 若维度不足，给box_wh赋默认值，避免报错
        box_wh = np.ones_like(box_xy) * anchors_reshaped[:2]
    else:
        box_wh = pow(position[:, 2:4, :, :] * 2, 2) * anchors_reshaped

    # 修正中心坐标计算（拼接col/row，避免维度不匹配）
    grid_xy = np.concatenate((col, row), axis=1)
    box_xy += grid_xy
    box_xy *= stride

    # 关键修复：重新构建xyxy数组（形状为[N, 4, H, W]，避免维度越界）
    # 不再从box_xy复制，而是直接创建4通道数组存储左上角/右下角坐标
    xyxy = np.zeros((box_xy.shape[0], 4, grid_h, grid_w), dtype=np.float32)
    xyxy[:, 0, :, :] = box_xy[:, 0, :, :] - box_wh[:, 0, :, :] / 2  # 左上角x
    xyxy[:, 1, :, :] = box_xy[:, 1, :, :] - box_wh[:, 1, :, :] / 2  # 左上角y
    xyxy[:, 2, :, :] = box_xy[:, 0, :, :] + box_wh[:, 0, :, :] / 2  # 右下角x
    xyxy[:, 3, :, :] = box_xy[:, 1, :, :] + box_wh[:, 1, :, :] / 2  # 右下角y

    return xyxy

def post_process(input_data, anchors):
    """YOLO输出后处理总流程：解析特征图→过滤低置信度→NMS去重（增加维度校验）"""
    boxes, scores, classes_conf = [], [], []
    
    # 先校验输入数据是否有效
    if not isinstance(input_data, list) or len(input_data) == 0:
        print("⚠️  推理输出数据为空")
        return None, None, None
    
    # 重塑输入数据（适配静态形状RKNN模型，增加异常捕获）
    try:
        input_data = [_in.reshape([len(anchors[0]), -1] + list(_in.shape[-2:])) for _in in input_data]
    except Exception as e:
        print(f"⚠️  特征图重塑失败：{e}")
        return None, None, None
    
    # 解析多尺度特征图（仅处理与锚点组匹配的特征图）
    valid_feature_num = min(len(input_data), len(anchors))
    for i in range(valid_feature_num):
        # 校验特征图通道数是否足够（至少5通道：xywh+conf）
        if input_data[i].shape[1] < 5:
            print(f"⚠️  第{i}个特征图通道数不足，跳过处理")
            continue
        # 解析边界框、置信度、类别概率
        boxes.append(box_process(input_data[i][:, :4, :, :], anchors[i]))
        scores.append(input_data[i][:, 4:5, :, :])
        classes_conf.append(input_data[i][:, 5:, :, :])
    
    # 校验解析结果是否有效
    if len(boxes) == 0 or len(scores) == 0 or len(classes_conf) == 0:
        return None, None, None

    # 特征图展平（降维处理）
    def sp_flatten(_in):
        ch = _in.shape[1]
        _in = _in.transpose(0, 2, 3, 1)
        return _in.reshape(-1, ch)

    try:
        boxes = [sp_flatten(_v) for _v in boxes]
        classes_conf = [sp_flatten(_v) for _v in classes_conf]
        scores = [sp_flatten(_v) for _v in scores]
    except:
        print("⚠️  特征图展平失败")
        return None, None, None

    # 拼接多尺度结果（增加非空校验）
    if any(len(arr) == 0 for arr in boxes):
        return None, None, None
    boxes = np.concatenate(boxes)
    classes_conf = np.concatenate(classes_conf)
    scores = np.concatenate(scores)

    # 过滤和NMS
    try:
        boxes, classes, scores = filter_boxes(boxes, scores, classes_conf, OBJ_THRESH)
    except:
        return None, None, None
    if len(boxes) == 0:
        return None, None, None

    # 按类别分别执行NMS，提升检测精度
    nboxes, nclasses, nscores = [], [], []
    for c in set(classes):
        inds = np.where(classes == c)
        b = boxes[inds]
        c_cls = classes[inds]
        s = scores[inds]
        keep = nms_boxes(b, s, NMS_THRESH)
        if len(keep) != 0:
            nboxes.append(b[keep])
            nclasses.append(c_cls[keep])
            nscores.append(s[keep])

    if not nclasses or not nscores:
        return None, None, None

    return np.concatenate(nboxes), np.concatenate(nclasses), np.concatenate(nscores)

def draw_detection_result(image, boxes, scores, classes, classes_list):
    """绘制检测结果：边界框+角标+类别置信度（板端可视化友好）"""
    for box, score, cl in zip(boxes, scores, classes):
        top, left, right, bottom = [int(_b) for _b in box]
        # 绘制主边界框（紫色）
        cv2.rectangle(image, (top, left), (right, bottom), (255, 0, 255), 2)
        # 绘制框角标（绿色，提升辨识度）
        corner_length = 15
        cv2.line(image, (top, left), (top + corner_length, left), (0, 255, 0), 3)
        cv2.line(image, (top, left), (top, left + corner_length), (0, 255, 0), 3)
        cv2.line(image, (right, left), (right - corner_length, left), (0, 255, 0), 3)
        cv2.line(image, (right, left), (right, left + corner_length), (0, 255, 0), 3)
        cv2.line(image, (top, bottom), (top + corner_length, bottom), (0, 255, 0), 3)
        cv2.line(image, (top, bottom), (top, bottom - corner_length), (0, 255, 0), 3)
        cv2.line(image, (right, bottom), (right - corner_length, bottom), (0, 255, 0), 3)
        cv2.line(image, (right, bottom), (right, bottom - corner_length), (0, 255, 0), 3)
        # 绘制类别与置信度标签（带背景框，避免被图像遮挡）
        label = f"{classes_list[cl]} {score:.2f}"
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        label_y = max(left, label_size[1] + 10)
        cv2.rectangle(image, (top, label_y - label_size[1] - 10),
                      (top + label_size[0], label_y), (255, 0, 255), -1)
        cv2.putText(image, label, (top, label_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)

def read_anchors_from_txt(txt_path):
    """
    从txt文件读取锚点数值（每行一个数值），整理为YOLOv5需要的3组锚点格式
    """
    try:
        # 读取txt文件，按行提取数值，过滤空行
        with open(txt_path, 'r', encoding='utf-8') as f:
            anchor_lines = [line.strip() for line in f.readlines() if line.strip()]
        
        # 转换为float类型，校验是否为18个数值（9个锚框，每个2个数值）
        anchor_nums = [float(num) for num in anchor_lines]
        if len(anchor_nums) != 18:
            raise ValueError(f"txt文件应包含18个数值，当前读取到{len(anchor_nums)}个")
        
        # 重塑为YOLOv5需要的格式：[[[w1,h1],[w2,h2],[w3,h3]], [[w4,h4],...], [[w7,h7],...]]
        anchors = np.array(anchor_nums).reshape(3, 3, 2).tolist()
        return anchors
    
    except FileNotFoundError:
        print(f"❌ 锚点txt文件不存在：{txt_path}")
        exit(1)
    except Exception as e:
        print(f"❌ 读取锚点txt文件失败：{e}")
        exit(1)

# ===================== 【新增：TCP 图传客户端类】 =====================
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
                print(f"✅ {self.name} 连接 {self.server_ip}:{self.port} 成功")
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
# ========================================================================

class YoloRknnRos2Node(Node):
    def __init__(self):
        # 初始化ROS2节点
        super().__init__("yolo_rknn_elf2_node")
        
        
        # 2. 初始化图传发送器（双端口）
        self.sender_raw = TcpImageSender(SERVER_IP, PORT_RAW, "原始图", JPEG_QUALITY)
        self.sender_yolo = TcpImageSender(SERVER_IP, PORT_YOLO, "YOLO图", JPEG_QUALITY)

        # 3. 加载类别列表
        # self.CLASSES = (
        #     "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
        #     "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
        #     "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
        #     "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        #     "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
        #     "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
        #     "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        #     "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
        # )
        self.CLASSES = (
            'holothurian', 'echinus', 'scallop', 'starfish'
        )
        # 4. 加载锚点文件
        self.get_logger().info(f"📦 从 {ANCHORS_TXT_PATH} 加载锚点文件...")
        self.ANCHORS = read_anchors_from_txt(ANCHORS_TXT_PATH)
        
        # 5. 初始化RKNN模型
        self.get_logger().info("📦 加载RKNN模型（resource目录）...")
        self.rknn = RKNNLite(verbose=0)
        ret = self.rknn.load_rknn(MODEL_PATH)
        if ret != 0:
            self.get_logger().error(f"❌ 加载RKNN模型失败，错误码：{ret}")
            exit(ret)
        
        self.get_logger().info("🔧 初始化NPU运行时环境...")
        ret = self.rknn.init_runtime()
        if ret != 0:
            self.get_logger().error(f"❌ 初始化NPU运行时失败，错误码：{ret}")
            exit(ret)
        
        # 6. 初始化摄像头
        self.get_logger().info(f"📹 初始化摄像头（设备索引：{CAM_DEVICE}）...")
        self.cap = cv2.VideoCapture(CAM_DEVICE)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ 无法打开摄像头设备 {CAM_DEVICE}")
            exit(1)
        
        # 设置摄像头分辨率
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_HEIGHT)
        self.get_logger().info(f"✅ 摄像头初始化成功，采集分辨率：{CAM_WIDTH}x{CAM_HEIGHT}")
        self.get_logger().info(f"✅ YOLO模型输入分辨率：{IMG_SIZE[0]}x{IMG_SIZE[1]}")
        self.get_logger().info(f"✅ 以太网图传目标：192.168.2.100")
        self.get_logger().info(f"✅ 原始图：6001  状态：{SEND_RAW_IMAGE}")
        self.get_logger().info(f"✅ YOLO图：6002  状态：{SEND_YOLO_IMAGE}")
        
        # 启动循环
        self.detection_infinite_loop()
    
    def detection_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("⚠️  无法读取摄像头帧")
            return
        frame_origin = frame.copy()
        origin_shape = frame_origin.shape[:2]
        
        # ===================== 发送原始图像 =====================
        if SEND_RAW_IMAGE:
            self.sender_raw.update_frame(frame_origin)
        
        # 预处理 + 推理（完全不变）
        img, ratio, pad = letter_box(frame_origin, new_shape=IMG_SIZE, pad_color=(0, 0, 0))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_input = np.expand_dims(img, axis=0)
        
        start_time = time.time()
        outputs = self.rknn.inference(inputs=[img_input], data_format=['nhwc'])
        infer_time = time.time() - start_time
        
        # 后处理 + 绘图（完全不变）
        if outputs is not None:
            boxes, classes, scores = post_process(outputs, self.ANCHORS)
            if boxes is not None:
                boxes = box_unletter(boxes, ratio, pad, origin_shape)
                draw_detection_result(frame, boxes, scores, classes, self.CLASSES)
        
        # 绘制耗时
        cv2.putText(frame, f"Infer Time: {infer_time*1000:.1f} ms", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        # ===================== 发送YOLO处理后图像 =====================
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
        self.get_logger().info("✅ 模型与摄像头资源已释放，节点退出")


# ------------------------- 主函数 -------------------------
def main(args=None):
    rclpy.init(args=args)
    yolo_rknn_node = YoloRknnRos2Node()
    rclpy.spin(yolo_rknn_node)
    yolo_rknn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

