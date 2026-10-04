#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2026 RAICOM CAIR 魔力元宝服务组国赛主控程序。"""

import base64
import math
import json
import os
import re
import subprocess
import threading
import time
from collections import deque
from urllib import error as urlerror
from urllib import parse as urlparse
from urllib import request as urlrequest

import actionlib
import cv2
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from face_rec.srv import recognition_results, recognition_resultsRequest
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from rei_robot_base.msg import CarData
from robot_audio.srv import robot_tts
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger, TriggerResponse
from tf.transformations import euler_from_quaternion, quaternion_from_euler
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


DEFAULT_POINTS = {
    "corridor": [-1.2471, -2.2794, -1.4922],
    "restaurant": [-2.2801, -1.1501, 3.0699],
    "kitchen": [-1.2636, -1.1576, -1.5933],
    "livingroom": [-1.2529, -0.0868, 1.5591],
    "bedroom": [-2.2408, -0.0720, 1.4127],
    "start": [-0.0914, -0.1117, 0.0199],
    "charge": [-0.3600, 0.0040, -0.0260],

    # 双向安全中转点。
    # A：右侧红框；B：左侧红框。
    # 同一位置按行驶方向使用不同yaw，避免到中转点后无意义原地大转圈。
    "route_a_out": [-0.1751, -2.2659, -3.1147],
    "route_b_out": [-2.3981, -2.3256, 1.5085],
    "route_b_back": [-2.3981, -2.3256, 0.0268],
    "route_a_back": [-0.1751, -2.2659, 1.5320],
}

NATIONAL_CLASS_ORDER = (
    "shanghai_green", "corn", "shrimp", "cucumber", "pork", "eggplant",
    "potato", "egg", "tomato", "fish", "green_pepper", "tofu", "banana",
    "apple", "phone", "backpack",
)

FOOD_NAMES = {
    "shanghai_green": "上海青",
    "corn": "玉米",
    "shrimp": "虾",
    "cucumber": "黄瓜",
    "pork": "猪肉",
    "eggplant": "茄子",
    "potato": "土豆",
    "egg": "鸡蛋",
    "tomato": "番茄",
    "fish": "鱼肉",
    "green_pepper": "青椒",
    "tofu": "豆腐",
    "banana": "香蕉",
    "apple": "苹果",
}

TARGET_NAMES = {
    "phone": "手机",
    "backpack": "书包",
}

# 模型类别名会先去掉空格、下划线和短横线，再匹配此表。
CLASS_ALIASES = {
    # 上海青
    "shanghaigreen": "shanghai_green",
    "shanghaiqing": "shanghai_green",
    "bokchoy": "shanghai_green",
    "上海青": "shanghai_green",

    # 玉米
    "corn": "corn",
    "maize": "corn",
    "yumi": "corn",
    "玉米": "corn",

    # 虾
    "shrimp": "shrimp",
    "prawn": "shrimp",
    "xia": "shrimp",
    "虾": "shrimp",

    # 黄瓜
    "cucumber": "cucumber",
    "huanggua": "cucumber",
    "黄瓜": "cucumber",

    # 猪肉
    "pork": "pork",
    "pig": "pork",
    "猪肉": "pork",

    # 茄子
    "eggplant": "eggplant",
    "aubergine": "eggplant",
    "qiezi": "eggplant",
    "茄子": "eggplant",

    # 土豆
    "potato": "potato",
    "tudou": "potato",
    "土豆": "potato",

    # 鸡蛋
    "egg": "egg",
    "鸡蛋": "egg",

    # 番茄
    "tomato": "tomato",
    "fanqie": "tomato",
    "番茄": "tomato",

    # 鱼
    "fish": "fish",
    "fishmeat": "fish",
    "鱼肉": "fish",

    # 青椒
    "greenpepper": "green_pepper",
    "pepper": "green_pepper",
    "qingjiao": "green_pepper",
    "青椒": "green_pepper",

    # 豆腐
    "tofu": "tofu",
    "doufu": "tofu",
    "豆腐": "tofu",

    # 香蕉
    "banana": "banana",
    "香蕉": "banana",

    # 苹果
    "apple": "apple",
    "苹果": "apple",

    # 手机
    "phone": "phone",
    "cellphone": "phone",
    "mobilephone": "phone",
    "smartphone": "phone",
    "手机": "phone",

    # 书包
    "backpack": "backpack",
    "schoolbag": "backpack",
    "bag": "backpack",
    "书包": "backpack",
    "背包": "backpack",
}

# Open-Meteo实时接口返回WMO weather_code数字；此表只把数字翻译成中文，
# 不是预设天气。气温、湿度、风速、降水和实际weather_code均由接口实时返回。
WEATHER_CODE_NAMES = {
    0: "晴朗",
    1: "大致晴朗",
    2: "多云",
    3: "阴天",
    45: "有雾",
    48: "有雾凇",
    51: "小毛毛雨",
    53: "毛毛雨",
    55: "较强毛毛雨",
    56: "轻微冻雨",
    57: "较强冻雨",
    61: "小雨",
    63: "中雨",
    65: "大雨",
    66: "轻微冻雨",
    67: "较强冻雨",
    71: "小雪",
    73: "中雪",
    75: "大雪",
    77: "有雪粒",
    80: "小阵雨",
    81: "阵雨",
    82: "强阵雨",
    85: "小阵雪",
    86: "强阵雪",
    95: "雷雨",
    96: "雷雨并伴有冰雹",
    99: "强雷雨并伴有冰雹",
}

# 比赛临时DeepSeek密钥。后期更换时只修改这一行即可；若系统环境变量中
# 配置了DEEPSEEK_API_KEY，则环境变量优先，方便密钥轮换。
DEFAULT_DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")


class ServiceRobot:

    TASK_ORDER = ("tour", "assistant", "find")

    def __init__(self):
        rospy.init_node("service_robot")

        self.state = "WAIT_START"
        self.current_task_index = 0
        self.completed_tasks = set()
        self.current_location = "start"
        self.start_requested = False
        self.face_absent_count = 0
        self.latest_voice = ""
        self.corridor_wake_prompt = "你好，需要帮助吗？"
        self.latest_img = None
        self.latest_img_token = None
        self.image_lock = threading.Lock()
        # 只保留少量最新原始帧，供第四轮ROI和千问视觉兜底使用。
        self.image_history = deque(maxlen=8)
        self.last_detection_token = None
        self.is_speaking = False
        # 清除上一次异常退出可能遗留的语音回声抑制标志。
        rospy.set_param("/service_robot/is_speaking", False)
        self.voice_input_enabled = False
        rospy.set_param("/service_robot/voice_input_enabled", False)
        self.is_charging = False
        self.geofence_violation = False
        self.mission_start_time = None

        self.points = rospy.get_param("~points", DEFAULT_POINTS)

        configured_start_to_corridor = rospy.get_param(
            "~start_to_corridor_waypoints",
            ["route_a_out"],
        )
        self.start_to_corridor_waypoints = [
            str(name) for name in configured_start_to_corridor
            if str(name) in self.points and str(name) not in ("start", "corridor")
        ]

        configured_corridor_to_task = rospy.get_param(
            "~corridor_to_task_waypoints",
            ["route_b_out"],
        )
        self.corridor_to_task_waypoints = [
            str(name) for name in configured_corridor_to_task
            if str(name) in self.points and str(name) not in ("start", "corridor", "charge")
        ]

        configured_task_to_corridor = rospy.get_param(
            "~task_to_corridor_waypoints",
            ["restaurant", "route_b_back"],
        )
        self.task_to_corridor_waypoints = [
            str(name) for name in configured_task_to_corridor
            if str(name) in self.points and str(name) not in ("start", "corridor", "charge")
        ]

        configured_task_to_start = rospy.get_param(
            "~task_to_start_waypoints",
            ["restaurant", "route_b_back", "route_a_back"],
        )
        self.task_to_start_waypoints = [
            str(name) for name in configured_task_to_start
            if str(name) in self.points and str(name) not in ("start", "corridor")
        ]

        configured_corridor_to_charge = rospy.get_param(
            "~corridor_to_charge_waypoints",
            ["route_a_back"],
        )
        self.corridor_to_charge_waypoints = [
            str(name) for name in configured_corridor_to_charge
            if str(name) in self.points and str(name) not in ("start", "corridor", "charge")
        ]

        if len(self.start_to_corridor_waypoints) != len(configured_start_to_corridor):
            rospy.logwarn("出发区到走廊中转点配置含无效点，已自动忽略")
        if len(self.corridor_to_task_waypoints) != len(configured_corridor_to_task):
            rospy.logwarn("走廊到任务区域中转点配置含无效点，已自动忽略")
        if len(self.task_to_corridor_waypoints) != len(configured_task_to_corridor):
            rospy.logwarn("任务区域到走廊中转点配置含无效点，已自动忽略")
        if len(self.task_to_start_waypoints) != len(configured_task_to_start):
            rospy.logwarn("任务返程到出发区中转点配置含无效点，已自动忽略")
        if len(self.corridor_to_charge_waypoints) != len(configured_corridor_to_charge):
            rospy.logwarn("走廊到充电区中转点配置含无效点，已自动忽略")
        self.hold_seconds = float(rospy.get_param("~hold_seconds", 5.5))
        self.navigation_timeout = float(rospy.get_param("~navigation_timeout", 240.0))
        self.arrival_fallback_tolerance = float(
            rospy.get_param("~arrival_fallback_tolerance", 0.25)
        )
        self.arrival_fallback_yaw_tolerance = float(
            rospy.get_param("~arrival_fallback_yaw_tolerance", 0.20)
        )
        self.arrival_confirm_tolerance = float(
            rospy.get_param("~arrival_confirm_tolerance", 0.05)
        )
        self.route_arrival_confirm_tolerance = float(
            rospy.get_param("~route_arrival_confirm_tolerance", 0.15)
        )
        self.arrival_confirm_samples = max(
            2, int(rospy.get_param("~arrival_confirm_samples", 3))
        )
        self.mission_timeout = float(rospy.get_param("~mission_timeout", 1200.0))
        self.detect_seconds = float(rospy.get_param("~detect_seconds", 8.0))
        self.detect_min_hits = int(rospy.get_param("~detect_min_hits", 2))
        self.detect_confidence = float(rospy.get_param("~detect_confidence", 0.25))
        self.food_global_rounds = max(
            1, int(rospy.get_param("~food_global_rounds", 3))
        )
        self.food_round_seconds = max(
            1.0, float(rospy.get_param("~food_round_seconds", self.detect_seconds))
        )
        self.food_global_confidence = max(
            0.05, min(1.0, float(rospy.get_param(
                "~food_global_confidence", 0.30
            )))
        )
        self.food_global_min_hits = max(
            1, int(rospy.get_param("~food_global_min_hits", 3))
        )
        # YOLO输入边长按32对齐。960能显著改善全景画面中较小食材的召回率。
        requested_food_imgsz = max(
            320, int(rospy.get_param("~food_global_imgsz", 960))
        )
        self.food_global_imgsz = int(round(requested_food_imgsz / 32.0) * 32)
        self.food_recognition_timeout = max(
            self.food_round_seconds * (self.food_global_rounds + 1),
            float(rospy.get_param("~food_recognition_timeout", 75.0)),
        )
        self.food_retry_confidence = max(
            0.05, min(1.0, float(rospy.get_param(
                "~food_retry_confidence", self.detect_confidence
            )))
        )
        self.food_retry_min_hits = max(
            1, int(rospy.get_param("~food_retry_min_hits", self.detect_min_hits))
        )
        raw_food_roi = (
            float(rospy.get_param("~food_roi_x_min_ratio", -1.0)),
            float(rospy.get_param("~food_roi_y_min_ratio", -1.0)),
            float(rospy.get_param("~food_roi_x_max_ratio", -1.0)),
            float(rospy.get_param("~food_roi_y_max_ratio", -1.0)),
        )
        self.food_roi = (
            raw_food_roi
            if self.valid_ratio_roi(raw_food_roi)
            else None
        )
        self.auto_start = bool(rospy.get_param("~auto_start", False))

        # 代码内置比赛临时密钥，启动后可持续调用，无需每次手动输入。
        # 密钥内容永远不写入ROS日志；环境变量可在后期无改码覆盖它。
        self.deepseek_api_key = os.environ.get(
            "DEEPSEEK_API_KEY", DEFAULT_DEEPSEEK_API_KEY
        ).strip()
        self.deepseek_api_url = str(rospy.get_param(
            "~deepseek_api_url", "https://api.deepseek.com/chat/completions"
        )).strip()
        self.deepseek_model = str(rospy.get_param(
            "~deepseek_model", "deepseek-v4-flash"
        )).strip()
        self.deepseek_timeout = float(rospy.get_param("~deepseek_timeout", 15.0))
        self.deepseek_max_tokens = int(rospy.get_param("~deepseek_max_tokens", 220))

        # 千问只在“三轮全图 + 一轮ROI”仍不足三类时启用。Key和端点暂未
        # 配置时自动跳过，绝不影响现有YOLO流程。
        self.qwen_vision_enabled = bool(
            rospy.get_param("~qwen_vision_enabled", True)
        )
        self.qwen_api_key = os.environ.get(
            "DASHSCOPE_API_KEY",
            str(rospy.get_param("~qwen_api_key", "")),
        ).strip()
        self.qwen_api_url = str(rospy.get_param("~qwen_api_url", "")).strip()
        self.qwen_vision_model = str(rospy.get_param(
            "~qwen_vision_model", "qwen3-vl-plus"
        )).strip()
        self.qwen_timeout = max(
            1.0, float(rospy.get_param("~qwen_timeout", 8.0))
        )
        self.qwen_min_evidence = max(
            1, int(rospy.get_param("~qwen_min_evidence", 2))
        )
        self.qwen_breaker_until = 0.0

        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.recipe_cache_path = str(rospy.get_param(
            "~recipe_cache_path",
            os.path.join(package_dir, "config", "recipe_cache.json"),
        )).strip()
        self.recipe_network_timeout = max(
            1.0, float(rospy.get_param("~recipe_network_timeout", 6.0))
        )
        self.recipe_cache = self.load_recipe_cache()

        # 比赛地点固定为深圳；天气数据仍通过 Open-Meteo 实时联网获取。
        # 不再使用公网 IP 定位，避免手机热点/运营商出口导致城市识别错误。
        self.weather_location_name = "深圳"
        self.weather_latitude = 0.0
        self.weather_longitude = 0.0
        self.weather_use_ip_location = False
        self.weather_timeout = float(rospy.get_param("~weather_timeout", 6.0))
        self.cached_weather = None

        # auto：国赛模型可用时用真实识别，否则只对食品/物品使用虚拟结果。
        self.vision_mode = str(rospy.get_param("~vision_mode", "auto")).lower()
        if self.vision_mode not in ("auto", "real", "mock"):
            rospy.logwarn("未知vision_mode=%s，恢复为auto", self.vision_mode)
            self.vision_mode = "auto"
        self.mock_vision = self.vision_mode == "mock"
        self.mock_person_detection = bool(
            rospy.get_param("~mock_person_detection", False)
        )
        self.skip_auto_charge_for_test = bool(
            rospy.get_param("~skip_auto_charge_for_test", False)
        )
        self.mock_vision_delay = float(rospy.get_param("~mock_vision_delay", 0.5))
        # 寻物目标使用独立置信度，不影响食材的0.35检测门槛。
        # 当前采用0.50兼顾训练模型召回率；仍要求至少两张相机新帧连续命中。
        self.find_target_confidence = max(
            0.0, min(1.0, float(rospy.get_param("~find_target_confidence", 0.50)))
        )
        self.find_verify_confidence = max(
            0.0, min(1.0, float(rospy.get_param("~find_verify_confidence", 0.50)))
        )
        self.find_verify_min_hits = max(
            2, int(rospy.get_param("~find_verify_min_hits", 3))
        )
        self.find_verify_seconds = max(
            2.0, float(rospy.get_param("~find_verify_seconds", 4.0))
        )
        self.find_camera_settle_seconds = max(
            0.0, float(rospy.get_param("~find_camera_settle_seconds", 0.8))
        )
        self.mock_foods = [
            name for name in rospy.get_param(
                "~mock_foods", ["tomato", "egg", "tofu"]
            ) if name in FOOD_NAMES
        ]
        if len(self.mock_foods) < 3:
            rospy.logwarn("虚拟食材少于三类，恢复为番茄、鸡蛋、豆腐")
            self.mock_foods = ["tomato", "egg", "tofu"]
        raw_mock_locations = rospy.get_param(
            "~mock_find_locations",
            {"phone": "livingroom", "backpack": "bedroom"},
        )
        allowed_places = ("restaurant", "kitchen", "livingroom", "bedroom", "corridor")
        self.mock_find_locations = {
            target: place for target, place in raw_mock_locations.items()
            if target in TARGET_NAMES and place in allowed_places
        }
        configured_order = rospy.get_param(
            "~model_class_order", list(NATIONAL_CLASS_ORDER)
        )
        self.model_class_order = [
            name if name in NATIONAL_CLASS_ORDER else None
            for name in configured_order
        ]
        self.model_uses_index_order = False

        self.forbidden_zones = self.load_forbidden_zones(
            rospy.get_param("~forbidden_zones", [])
        )
        # 禁行区的底盘安全距离由 costmap 的 robot_radius 和虚拟墙负责。
        # 此处只作为“中心点已经进入禁行区”的最后停车保护，避免重复膨胀堵死通道。
        self.geofence_margin = float(rospy.get_param("~geofence_margin", 0.0))

        # 回充：始终使用负线速度倒车入桩。
        self.allow_charge_fallback = bool(
            rospy.get_param("~allow_charge_fallback", True)
        )
        self.charge_signal_confirm_seconds = float(
            rospy.get_param("~charge_signal_confirm_seconds", 2.0)
        )
        self.charge_timeout = float(rospy.get_param("~charge_timeout", 35.0))
        # 最终无电回充方案：
        # AR只负责把机器人送到最终直线段并对正；
        # 进入最终直线段后，不再依赖AR距离判断结束，而由 /odom 控制固定倒车距离。
        self.final_dock_trigger_x = float(
            rospy.get_param("~final_dock_trigger_x", -0.34)
        )
        self.final_dock_align_y = float(
            rospy.get_param("~final_dock_align_y", 0.05)
        )
        self.final_dock_distance = float(
            rospy.get_param("~final_dock_distance", 0.075)
        )
        self.final_dock_speed = abs(float(
            rospy.get_param("~final_dock_speed", 0.012)
        ))
        self.final_dock_timeout = float(
            rospy.get_param("~final_dock_timeout", 7.0)
        )
        self.final_dock_odom_topic = str(
            rospy.get_param("~final_dock_odom_topic", "/odom")
        ).strip()
        self.latest_odom_xy = None

        # 回充视觉调试：不改变AR模块使用的摄像头，只在开始视觉对接时
        # 自动打开同一个图像topic，方便现场确认是否选错物理摄像头。
        self.charge_camera_viewer_enabled = bool(
            rospy.get_param("~charge_camera_viewer_enabled", True)
        )
        self.charge_camera_topic = str(
            rospy.get_param("~charge_camera_topic", "/base_camera/image_raw")
        ).strip()
        self.charge_camera_viewer_process = None

        self.place_name = {
            "restaurant": "餐厅",
            "kitchen": "厨房",
            "livingroom": "客厅",
            "bedroom": "卧室",
            "corridor": "走廊",
            "start": "出发区",
        }
        self.intro_text = {
            "restaurant": "这里是餐厅，是家人用餐和交流的区域。",
            "kitchen": "这里是厨房，是准备和制作餐食的区域。",
            "livingroom": "这里是客厅，是休闲和会客的主要区域。",
            "bedroom": "这里是卧室，是日常休息的私人空间。",
        }

        rospy.loginfo("等待 move_base...")
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("move_base 已连接")

        self.clear_costmaps = None
        try:
            rospy.wait_for_service("/move_base_node/clear_costmaps", timeout=3.0)
            self.clear_costmaps = rospy.ServiceProxy(
                "/move_base_node/clear_costmaps", Empty
            )
        except rospy.ROSException:
            rospy.logwarn("未找到 clear_costmaps 服务，导航仍可继续")

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("等待语音服务...")
        rospy.wait_for_service("/voice_tts")
        # 复用同一条ROS服务连接，减少每句讯飞播报前的重复握手开销。
        self.tts = rospy.ServiceProxy("/voice_tts", robot_tts, persistent=True)

        self.face_client = None
        if self.mock_person_detection:
            rospy.logwarn("测试模式：人员靠近使用虚拟检测")
        else:
            rospy.loginfo("等待人脸识别服务...")
            rospy.wait_for_service("/face_recognition_results")
            self.face_client = rospy.ServiceProxy(
                "/face_recognition_results", recognition_results
            )

        self.yolo = None
        self.bridge = None
        if self.vision_mode == "mock":
            rospy.logwarn("测试模式：跳过YOLO模型，食品和物品使用虚拟识别")
        else:
            try:
                self.yolo = self.load_yolo_model(require_national=True)
            except RuntimeError as exc:
                if self.vision_mode == "real":
                    raise
                self.mock_vision = True
                rospy.logwarn("未找到合格国赛模型，自动启用虚拟食品/物品识别：%s", exc)

        if not self.mock_vision:
            self.bridge = CvBridge()
            camera_topic = rospy.get_param(
                "~camera_topic", "/berxel_camera/rgb/rgb_raw"
            )
            rospy.Subscriber(camera_topic, Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/voice_words", String, self.voice_callback, queue_size=10)
        rospy.Subscriber("/car_data", CarData, self.car_data_callback, queue_size=10)
        rospy.Subscriber(
            self.final_dock_odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=10,
        )

        rospy.Service(
            "/service_robot/start_competition", Trigger, self.start_service_callback
        )
        rospy.Timer(rospy.Duration(0.1), self.geofence_callback)
        rospy.loginfo("系统初始化完成，禁行区数量：%d", len(self.forbidden_zones))
        rospy.loginfo(
            "DeepSeek接口：%s，模型：%s",
            "已配置密钥" if self.deepseek_api_key else "未配置密钥，将使用本地兜底",
            self.deepseek_model,
        )
        rospy.loginfo(
            "食材识别：%d轮全图 + 1轮%s + 千问%s，任务时限%.1f秒",
            self.food_global_rounds,
            "ROI" if self.food_roi else "全图（ROI待标定）",
            "已配置" if self.qwen_ready() else "未配置、自动跳过",
            self.food_recognition_timeout,
        )

    def model_missing_classes(self, model):
        names = getattr(model, "names", {})
        if isinstance(names, dict):
            indexed_names = sorted(names.items())
        else:
            indexed_names = list(enumerate(names))
        # 只有模型恰好为国赛16类时，才允许按类别编号顺序兜底。
        # 防止把COCO等多类别通用模型误判为国赛模型。
        self.model_uses_index_order = (
            len(indexed_names) == len(self.model_class_order)
        )
        canonical = {
            self.canonical_model_class(
                int(class_id), raw_name, self.model_uses_index_order
            )
            for class_id, raw_name in indexed_names
        }
        canonical.discard(None)
        return set(NATIONAL_CLASS_ORDER) - canonical

    def load_yolo_model(self, require_national=False):
        if YOLO is None:
            raise RuntimeError("未安装ultralytics，无法加载YOLO模型")
        configured = rospy.get_param(
            "~yolo_model_path", "/home/bobac3/siyuu/best_national.pt"
        )
        fallback = rospy.get_param(
            "~fallback_yolo_model_path", "/home/bobac3/siyuu/best.pt"
        )
        candidates = []
        for path in (configured, fallback):
            if path and path not in candidates:
                candidates.append(path)
        last_error = None
        for path in candidates:
            if not path or not os.path.exists(path):
                rospy.logwarn("YOLO模型不存在：%s", path)
                continue
            try:
                rospy.loginfo("加载YOLO模型：%s", path)
                model = YOLO(path)
                missing = self.model_missing_classes(model)
                if require_national and missing:
                    last_error = RuntimeError("缺少%d个国赛类别" % len(missing))
                    rospy.logwarn(
                        "拒绝非完整国赛模型 %s，缺少：%s",
                        path, ", ".join(sorted(missing)),
                    )
                    continue
                rospy.loginfo("已启用国赛视觉模型：%s", path)
                return model
            except Exception as exc:
                last_error = exc
                rospy.logerr("YOLO模型加载失败 %s：%s", path, exc)
        raise RuntimeError("没有可用的YOLO模型：%s" % last_error)

    @staticmethod
    def normalize_class_name(name):
        return re.sub(r"[\s_\-]", "", str(name).strip().lower())

    def canonical_class_name(self, name):
        return CLASS_ALIASES.get(self.normalize_class_name(name))

    def canonical_model_class(self, class_id, raw_name, allow_index_fallback=None):
        canonical = self.canonical_class_name(raw_name)
        if canonical is not None:
            return canonical
        if allow_index_fallback is None:
            allow_index_fallback = self.model_uses_index_order
        if allow_index_fallback and 0 <= class_id < len(self.model_class_order):
            return self.model_class_order[class_id]
        return None

    def warn_if_model_is_not_national(self):
        missing = self.model_missing_classes(self.yolo)
        if missing:
            rospy.logwarn(
                "当前模型不是完整国赛16类模型，缺少%d类，任务二和任务三暂不可用",
                len(missing),
            )

    @staticmethod
    def load_forbidden_zones(raw_zones):
        zones = []
        for index, zone in enumerate(raw_zones or []):
            if not isinstance(zone, dict):
                rospy.logwarn("忽略格式错误的禁行区 #%d", index + 1)
                continue
            try:
                polygon = [
                    (float(point[0]), float(point[1]))
                    for point in zone.get("polygon", [])
                ]
            except (TypeError, ValueError, IndexError):
                rospy.logwarn("忽略坐标错误的禁行区 #%d", index + 1)
                continue
            if len(polygon) < 3:
                rospy.logwarn("禁行区 #%d 少于三个顶点，已忽略", index + 1)
                continue
            zones.append({
                "name": zone.get("name", "zone_%d" % (index + 1)),
                "polygon": polygon,
            })
        return zones

    def start_service_callback(self, _request):
        if self.state != "WAIT_START":
            return TriggerResponse(False, "比赛已经开始或当前状态不允许启动")
        self.start_requested = True
        self.set_voice_input_enabled(False)
        return TriggerResponse(True, "已收到开始比赛请求")

    def set_voice_input_enabled(self, enabled):
        self.voice_input_enabled = bool(enabled)
        rospy.set_param(
            "/service_robot/voice_input_enabled", self.voice_input_enabled
        )

    def car_data_callback(self, msg):
        self.is_charging = bool(msg.is_charge)

    def odom_callback(self, msg):
        """保存最新里程计位置，供最终7~8cm直线回充使用。"""
        self.latest_odom_xy = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        )

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            stamp = msg.header.stamp.to_nsec() if msg.header.stamp else 0
            token = (int(msg.header.seq), int(stamp))
            # 某些相机驱动不递增seq/stamp，使用单调时间保证新帧仍可区分。
            if token == self.latest_img_token:
                token = (token[0], time.monotonic_ns())
            with self.image_lock:
                self.latest_img = image
                self.latest_img_token = token
                self.image_history.append((token, image.copy()))
        except Exception as exc:
            rospy.logwarn("图像转换失败：%s", exc)

    def voice_callback(self, msg):
        # 任务执行、网络查询和机器人播报期间不接收新命令，避免环境声或
        # TTS回声覆盖下一次真正需要处理的语音。
        if (not self.voice_input_enabled) or self.is_speaking or self.state == "RUNNING_TASK":
            return
        text = re.sub(r"[。，“”，,\s]", "", msg.data.strip())
        if not text:
            return
        if self.state == "WAIT_START" and any(
            word in text for word in ("开始比赛", "启动比赛", "开始任务")
        ):
            rospy.loginfo("收到国赛语音开始指令：%s", text)
            self.start_requested = True
            self.set_voice_input_enabled(False)
            return
        ignore_words = (
            "系统启动完成", "需要帮助吗", "请跟我来", "开始返回充电区",
            "正在返回出发区", "已到达出发区", "已到达充电等待区",
            "这些就是参观区域啦", "我要回去继续工作了",
            "找到手机啦", "找到书包啦", "这里什么都没有",
            "我看到了手机", "我看到了书包", "但没有找到手机", "但没有找到书包",
        )
        if any(word in text for word in ignore_words):
            return
        rospy.loginfo("收到语音：%s", text)
        self.latest_voice = text
        self.set_voice_input_enabled(False)

    def speak(self, text):
        rospy.loginfo("播报: %s", text)
        self.is_speaking = True
        rospy.set_param("/service_robot/is_speaking", True)

        try:
            self.tts(text, True)
            # /voice_tts 会在音频播放结束后才返回，只保留很短的回声消退时间。
            # 原来的 1.2 秒固定等待会让每一句播报都显得明显迟钝。
            rospy.sleep(0.15)
        except Exception as exc:
            rospy.logwarn("TTS失败: %s", exc)
        finally:
            self.is_speaking = False
            rospy.set_param("/service_robot/is_speaking", False)

    @staticmethod
    def valid_ratio_roi(roi):
        try:
            x_min, y_min, x_max, y_max = [float(value) for value in roi]
        except (TypeError, ValueError):
            return False
        return (
            0.0 <= x_min < x_max <= 1.0
            and 0.0 <= y_min < y_max <= 1.0
        )

    def qwen_ready(self):
        return bool(
            self.qwen_vision_enabled
            and self.qwen_api_key
            and self.qwen_api_url
            and time.monotonic() >= self.qwen_breaker_until
        )

    @staticmethod
    def recipe_cache_key(ingredients):
        return "|".join(sorted(str(item).strip() for item in ingredients))

    def load_recipe_cache(self):
        if not self.recipe_cache_path or not os.path.isfile(self.recipe_cache_path):
            return {}
        try:
            with open(self.recipe_cache_path, "r", encoding="utf-8") as cache_file:
                content = json.load(cache_file)
            return content if isinstance(content, dict) else {}
        except Exception as exc:
            rospy.logwarn("读取菜谱缓存失败，将从空缓存继续：%s", exc)
            return {}

    def save_recipe_cache(self):
        if not self.recipe_cache_path:
            return
        temp_path = self.recipe_cache_path + ".tmp"
        try:
            directory = os.path.dirname(self.recipe_cache_path)
            if directory and not os.path.isdir(directory):
                os.makedirs(directory)
            with open(temp_path, "w", encoding="utf-8") as cache_file:
                json.dump(
                    self.recipe_cache, cache_file,
                    ensure_ascii=False, indent=2, sort_keys=True,
                )
            os.replace(temp_path, self.recipe_cache_path)
        except Exception as exc:
            rospy.logwarn("写入菜谱缓存失败：%s", exc)
            try:
                if os.path.exists(temp_path):
                    os.remove(temp_path)
            except OSError:
                pass

    @staticmethod
    def merge_detections(target, incoming):
        """跨轮累计证据；同一类别不因某轮漏检而丢失。"""
        for name, data in (incoming or {}).items():
            current = target.setdefault(name, {"hits": 0, "confidence": 0.0})
            current["hits"] += int(data.get("hits", 0))
            current["confidence"] = max(
                float(current.get("confidence", 0.0)),
                float(data.get("confidence", 0.0)),
            )
            if data.get("source"):
                current["source"] = data["source"]

    def recent_unique_images(self, limit=5):
        with self.image_lock:
            history = list(self.image_history)
        selected = []
        seen_tokens = set()
        for token, image in reversed(history):
            if token in seen_tokens:
                continue
            seen_tokens.add(token)
            selected.append(image.copy())
            if len(selected) >= int(limit):
                break
        selected.reverse()
        return selected


    @staticmethod
    def request_json(url, payload=None, headers=None, timeout=8.0):
        """使用Python标准库请求JSON，避免机器人额外安装requests依赖。"""
        request_headers = {
            "Accept": "application/json",
            "User-Agent": "RAICOM-ServiceRobot/2026",
        }
        request_headers.update(headers or {})
        body = None
        if payload is not None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            request_headers.setdefault("Content-Type", "application/json")
        request = urlrequest.Request(
            url, data=body, headers=request_headers,
            method="POST" if body is not None else "GET",
        )
        with urlrequest.urlopen(request, timeout=float(timeout)) as response:
            charset = response.headers.get_content_charset() or "utf-8"
            return json.loads(response.read().decode(charset))

    @staticmethod
    def clean_ai_text(text, max_length=220):
        """把大模型回答整理成适合TTS的一段纯文本。"""
        cleaned = str(text or "").replace("```", "")
        cleaned = re.sub(r"[*#>`_]+", "", cleaned)
        cleaned = re.sub(r"\s+", " ", cleaned).strip()
        return cleaned[:max_length].rstrip("，、；：")

    def deepseek_complete(self, system_prompt, user_prompt, timeout=None):
        if not self.deepseek_api_key:
            rospy.logwarn("DeepSeek密钥未配置，本次使用本地兜底内容")
            return None
        payload = {
            "model": self.deepseek_model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "stream": False,
            "thinking": {"type": "disabled"},
            "temperature": 0.2,
            "max_tokens": self.deepseek_max_tokens,
        }
        try:
            result = self.request_json(
                self.deepseek_api_url,
                payload=payload,
                headers={"Authorization": "Bearer " + self.deepseek_api_key},
                timeout=self.deepseek_timeout if timeout is None else float(timeout),
            )
            content = result["choices"][0]["message"]["content"]
            cleaned = self.clean_ai_text(content, self.deepseek_max_tokens * 2)
            if cleaned:
                return cleaned
            rospy.logwarn("DeepSeek返回了空内容，本次使用本地兜底")
        except (KeyError, IndexError, TypeError, ValueError,
                urlerror.URLError, urlerror.HTTPError) as exc:
            # 只记录异常类型和状态，不记录请求头，确保密钥不会进入ROS日志。
            rospy.logwarn("DeepSeek请求失败（%s）：%s", type(exc).__name__, exc)
        except Exception as exc:
            rospy.logwarn("DeepSeek请求异常（%s）：%s", type(exc).__name__, exc)
        return None

    def answer_distractor_with_deepseek(self, text):
        """回答干扰/闲聊命令，但绝不把回答转成导航或比赛任务。"""
        answer = self.deepseek_complete(
            (
                "你是RAICOM服务组比赛中的服务机器人。用户当前说的是干扰命令或"
                "日常问答。只回答用户当前问题，不要启动、声称完成或虚构任何导览、"
                "巡检、寻物、导航、回充任务。请用自然简短的中文口语回答，最多两句，"
                "不超过80个汉字，不使用Markdown。"
            ),
            "用户说：%s" % text,
        )
        if not answer:
            answer = "我暂时无法联网回答这个问题，请稍后再问我。"
        self.speak(answer)

    def resolve_weather_location(self):
        """经纬度优先，其次配置城市，最后按公网IP估算当前位置。"""
        if abs(self.weather_latitude) > 0.000001 or abs(self.weather_longitude) > 0.000001:
            return {
                "name": self.weather_location_name or "当前位置",
                "latitude": self.weather_latitude,
                "longitude": self.weather_longitude,
            }

        if self.weather_location_name:
            query = urlparse.urlencode({
                "name": self.weather_location_name,
                "count": 1,
                "language": "zh",
                "format": "json",
            })
            result = self.request_json(
                "https://geocoding-api.open-meteo.com/v1/search?" + query,
                timeout=self.weather_timeout,
            )
            matches = result.get("results") or []
            if matches:
                match = matches[0]
                city = str(match.get("name") or self.weather_location_name)
                region = str(match.get("admin1") or "")
                location_name = (
                    city if not region or region in city or city in region
                    else region + city
                )
                return {
                    "name": location_name,
                    "latitude": float(match["latitude"]),
                    "longitude": float(match["longitude"]),
                }
            raise ValueError("未查询到配置城市")

        if not self.weather_use_ip_location:
            raise ValueError("没有配置天气城市或经纬度")
        last_error = None
        for endpoint in ("https://ipapi.co/json/", "https://ipwho.is/"):
            try:
                result = self.request_json(endpoint, timeout=self.weather_timeout)
                if result.get("success") is False:
                    raise ValueError(str(result.get("message") or "IP定位失败"))
                latitude = result.get("latitude")
                longitude = result.get("longitude")
                if latitude is None or longitude is None:
                    raise ValueError("公网IP定位结果缺少经纬度")
                city = str(result.get("city") or "当前位置")
                region = str(result.get("region") or "")
                location_name = city if not region or region in city else region + city
                return {
                    "name": location_name,
                    "latitude": float(latitude),
                    "longitude": float(longitude),
                }
            except Exception as exc:
                last_error = exc
                rospy.logwarn("IP定位备用源失败：%s", type(exc).__name__)
        raise ValueError("两个公网IP定位源均不可用：%s" % last_error)

    def get_current_weather(self):
        try:
            location = self.resolve_weather_location()
            query = urlparse.urlencode({
                "latitude": location["latitude"],
                "longitude": location["longitude"],
                "current": (
                    "temperature_2m,apparent_temperature,relative_humidity_2m,"
                    "precipitation,weather_code,wind_speed_10m,uv_index,is_day"
                ),
                "timezone": "auto",
                "forecast_days": 1,
            })
            result = self.request_json(
                "https://api.open-meteo.com/v1/forecast?" + query,
                timeout=self.weather_timeout,
            )
            current = result.get("current") or {}
            if "temperature_2m" not in current or "weather_code" not in current:
                raise ValueError("天气接口未返回当前天气")
            weather = {
                "location": location["name"],
                "temperature": float(current["temperature_2m"]),
                "apparent_temperature": float(
                    current.get("apparent_temperature", current["temperature_2m"])
                ),
                "humidity": int(round(float(current.get("relative_humidity_2m", 0)))),
                "precipitation": float(current.get("precipitation", 0.0)),
                "weather_code": int(current["weather_code"]),
                "wind_speed": float(current.get("wind_speed_10m", 0.0)),
                "uv_index": float(current.get("uv_index", 0.0)),
                "is_day": bool(int(current.get("is_day", 1))),
            }
            weather["condition"] = WEATHER_CODE_NAMES.get(
                weather["weather_code"], "天气状况代码%d" % weather["weather_code"]
            )
            self.cached_weather = weather
            rospy.loginfo(
                "天气查询成功：%s %.1f摄氏度 %s",
                weather["location"], weather["temperature"], weather["condition"],
            )
            return weather
        except Exception as exc:
            rospy.logwarn("当前天气查询失败（%s）：%s", type(exc).__name__, exc)
            if self.cached_weather is not None:
                rospy.logwarn("使用本次程序启动后最近一次成功的天气数据")
                return self.cached_weather
            return None

    @staticmethod
    def local_clothing_advice(weather):
        temperature = weather["apparent_temperature"]
        if temperature <= 5:
            clothes = "建议穿厚羽绒服或棉服，并注意头颈和手部保暖"
        elif temperature <= 12:
            clothes = "建议穿毛衣配厚外套和长裤"
        elif temperature <= 20:
            clothes = "建议穿长袖上衣配薄外套和长裤"
        elif temperature <= 27:
            clothes = "建议穿薄长袖或短袖，早晚可带一件轻薄外套"
        else:
            clothes = "天气较热，建议穿透气短袖和轻薄下装，并注意补水"
        additions = []
        if weather["precipitation"] > 0 or weather["weather_code"] in (
            51, 53, 55, 56, 57, 61, 63, 65, 66, 67, 80, 81, 82, 95, 96, 99
        ):
            additions.append("记得带伞并穿防滑鞋")
        if weather["wind_speed"] >= 25:
            additions.append("风较大，请选择防风外套")
        if weather.get("is_day") and weather.get("uv_index", 0.0) >= 6:
            additions.append("紫外线较强，请做好防晒")
        return clothes + ("；" + "，".join(additions) if additions else "") + "。"

    def broadcast_weather_and_clothing(self):
        weather = self.get_current_weather()
        followup = "请问还有什么需要帮助的吗？"

        if weather is None:
            self.speak(
                "暂时无法获取深圳的实时天气，请检查网络。"
                + followup
            )
            return False

        # 天气播报控制得更短，并把追问合并在同一次TTS里。
        # 这样不会出现长天气播报结束后立刻再次调用 /voice_tts 导致语音模块卡在TTS状态。
        prompt = (
            "地点：{location}；天气：{condition}；气温：{temperature:.1f}℃；"
            "体感：{apparent_temperature:.1f}℃；湿度：{humidity}%；"
            "降水：{precipitation:.1f}毫米；风速：{wind_speed:.1f}公里/小时；"
            "紫外线指数：{uv_index:.1f}；当前是否白天：{is_day}。"
            "请用简短自然的中文播报当前天气和穿衣建议，重点说明温度、是否带伞、"
            "是否需要防晒或防风，总长度不超过100个汉字。"
        ).format(**weather)

        combined_report = self.deepseek_complete(
            "你是机器人天气与穿衣播报助手。输入中的地点和天气数据来自实时天气平台。"
            "不得修改或猜测数据，不使用Markdown，不添加开场寒暄。",
            prompt,
        )

        if combined_report:
            report = self.clean_ai_text(combined_report, 120)
        else:
            weather_text = (
                "%s当前%s，气温%.1f摄氏度，体感%.1f摄氏度，"
                "降水%.1f毫米，风速每小时%.1f公里。"
                % (
                    weather["location"], weather["condition"],
                    weather["temperature"], weather["apparent_temperature"],
                    weather["precipitation"], weather["wind_speed"],
                )
            )
            report = weather_text + self.local_clothing_advice(weather)

        # 关键：只调用一次TTS。
        self.speak(report + followup)
        return True


    def identify_foods_with_qwen(self):
        """最后一级视觉兜底；未配置、断网或结果不可靠时返回空字典。"""
        if not self.qwen_ready():
            rospy.loginfo("千问视觉未配置或熔断中，本轮自动跳过")
            return {}

        images = self.recent_unique_images(limit=5)
        if len(images) < 3:
            rospy.logwarn("千问视觉需要至少3张新图，当前只有%d张，跳过", len(images))
            return {}

        content = [{
            "type": "text",
            "text": (
                "你正在复核机器人冰箱货架中的食材。只允许从以下14类中选择："
                "上海青、玉米、虾、黄瓜、猪肉、茄子、土豆、鸡蛋、番茄、鱼肉、"
                "青椒、豆腐、香蕉、苹果。综合多张连续图片，只报告至少在两张图片"
                "中有明确视觉证据的类别，不猜测，不报告容器、家具和背景。严格返回"
                "JSON：{\"foods\":[{\"name\":\"番茄\",\"evidence_count\":3}]}。"
            ),
        }]
        for image in images:
            success, encoded = cv2.imencode(
                ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82]
            )
            if not success:
                continue
            data_url = "data:image/jpeg;base64," + base64.b64encode(
                encoded.tobytes()
            ).decode("ascii")
            content.append({
                "type": "image_url",
                "image_url": {"url": data_url},
            })

        if len(content) < 4:
            return {}
        endpoint = self.qwen_api_url.rstrip("/")
        if not endpoint.endswith("/chat/completions"):
            endpoint += "/chat/completions"
        payload = {
            "model": self.qwen_vision_model,
            "messages": [{"role": "user", "content": content}],
            "stream": False,
            "temperature": 0.0,
            "max_tokens": 180,
        }
        try:
            result = self.request_json(
                endpoint,
                payload=payload,
                headers={"Authorization": "Bearer " + self.qwen_api_key},
                timeout=self.qwen_timeout,
            )
            raw_text = result["choices"][0]["message"]["content"]
            if isinstance(raw_text, list):
                raw_text = "".join(
                    str(item.get("text", "")) if isinstance(item, dict) else str(item)
                    for item in raw_text
                )
            match = re.search(r"\{.*\}", str(raw_text), flags=re.S)
            parsed = json.loads(match.group(0) if match else str(raw_text))
            detected = {}
            for item in parsed.get("foods", []):
                if not isinstance(item, dict):
                    continue
                canonical = self.canonical_class_name(item.get("name", ""))
                evidence = int(item.get("evidence_count", 0))
                if canonical in FOOD_NAMES and evidence >= self.qwen_min_evidence:
                    detected[canonical] = {
                        "hits": evidence,
                        "confidence": 0.80,
                        "source": "qwen",
                    }
            rospy.loginfo("千问视觉复核结果：%s", detected)
            return detected
        except Exception as exc:
            # 网络故障后60秒内不重复阻塞任务，继续使用本地新帧YOLO。
            self.qwen_breaker_until = time.monotonic() + 60.0
            rospy.logwarn("千问视觉调用失败（%s），60秒内跳过并继续本地识别", type(exc).__name__)
            return {}

    def emergency_recipe(self, selected_cn):
        ingredients = "、".join(selected_cn)
        return (
            "推荐制作%s家常烩菜。做法是：第一步把%s清洗切好；"
            "第二步依次下锅翻炒并加少量清水；第三步加盐和酱油调味，熟透后出锅。"
            % (ingredients, ingredients)
        )

    def generate_recipe_with_deepseek(self, selected_cn):
        ingredients = "、".join(selected_cn)
        cache_key = self.recipe_cache_key(selected_cn)
        recipe_text = self.deepseek_complete(
            "你是中式家常菜助手。必须把用户给出的三种食材全部作为主食材，"
            "不得增加其他主食材；只允许额外使用油、盐、酱油、醋、糖、"
            "葱、姜、蒜和清水等基础调料。输出纯文本，不使用Markdown。",
            "三种已识别食材是%s。请推荐一道现实可做的菜并给出简洁做法。"
            "固定格式：推荐制作某某菜。做法是：第一步……第二步……第三步……。"
            "总长度不超过120个汉字。" % ingredients,
            timeout=self.recipe_network_timeout,
        )
        if not recipe_text:
            cached = self.recipe_cache.get(cache_key)
            if cached:
                rospy.logwarn("DeepSeek当前不可用，使用已缓存的DeepSeek菜谱")
                return self.clean_ai_text(cached, 180)
            rospy.logwarn("DeepSeek当前不可用且无缓存，使用离线应急做法")
            return self.emergency_recipe(selected_cn)
        recipe_text = self.clean_ai_text(recipe_text, 180)
        missing = [name for name in selected_cn if name not in recipe_text]
        extra = [
            name for name in FOOD_NAMES.values()
            if name not in selected_cn and name in recipe_text
        ]
        if missing or extra:
            rospy.logwarn(
                "DeepSeek食谱未通过三食材校验，缺少=%s，额外=%s，尝试缓存",
                missing, extra,
            )
            cached = self.recipe_cache.get(cache_key)
            return (
                self.clean_ai_text(cached, 180)
                if cached else self.emergency_recipe(selected_cn)
            )
        self.recipe_cache[cache_key] = recipe_text
        self.save_recipe_cache()
        return recipe_text

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    def hold_position(self, seconds=None):
        duration = self.hold_seconds if seconds is None else float(seconds)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.publish_stop()
            rate.sleep()

    def mission_time_expired(self):
        if self.mission_start_time is None:
            return False
        return (
            rospy.Time.now() - self.mission_start_time
            >= rospy.Duration(self.mission_timeout)
        )

    def detect_face_present(self):
        if self.mock_person_detection:
            # WAIT_CLEAR 返回无人，WAIT_PERSON 返回有人，自动走完“离开后再靠近”。
            present = self.state == "WAIT_PERSON"
            rospy.loginfo_throttle(2.0, "虚拟人员检测：%s", "有人" if present else "无人")
            return present
        try:
            response = self.face_client(recognition_resultsRequest(1, ""))
            return bool(response.success and response.result.num > 0)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "人脸检测失败：%s", exc)
            return False

    def detect_classes(
        self,
        duration=None,
        mock_classes=None,
        confidence=None,
        min_hits=None,
        roi=None,
        return_all=False,
        imgsz=None,
    ):
        duration = self.detect_seconds if duration is None else float(duration)
        confidence_threshold = (
            self.detect_confidence if confidence is None else float(confidence)
        )
        min_hits = self.detect_min_hits if min_hits is None else int(min_hits)
        min_hits = max(1, min_hits)
        if self.mock_vision:
            rospy.sleep(max(0.0, min(self.mock_vision_delay, duration)))
            detected = {
                name: {"hits": min_hits, "confidence": 0.99, "source": "mock"}
                for name in (mock_classes or [])
                if name in FOOD_NAMES or name in TARGET_NAMES
            }
            rospy.logwarn("虚拟视觉识别结果：%s", detected)
            return detected

        counts = {}
        max_confidence = {}
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(5)

        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            if self.mission_time_expired():
                break
            with self.image_lock:
                image = None if self.latest_img is None else self.latest_img.copy()
                image_token = self.latest_img_token
            if image is None:
                rate.sleep()
                continue
            # 同一相机帧只允许计数一次，避免相机冻结时把一张图重复累计成“稳定命中”。
            if image_token is None or image_token == self.last_detection_token:
                rate.sleep()
                continue
            self.last_detection_token = image_token

            inference_image = image
            if roi is not None and self.valid_ratio_roi(roi):
                height, width = image.shape[:2]
                x_min = max(0, min(width - 1, int(float(roi[0]) * width)))
                y_min = max(0, min(height - 1, int(float(roi[1]) * height)))
                x_max = max(x_min + 1, min(width, int(float(roi[2]) * width)))
                y_max = max(y_min + 1, min(height, int(float(roi[3]) * height)))
                inference_image = image[y_min:y_max, x_min:x_max]
            try:
                predict_kwargs = {
                    "conf": confidence_threshold,
                    "verbose": False,
                }
                if imgsz is not None:
                    predict_kwargs["imgsz"] = int(imgsz)
                results = self.yolo(inference_image, **predict_kwargs)
                seen_this_frame = set()
                for result in results:
                    for box in result.boxes:
                        class_id = int(box.cls[0])
                        box_confidence = float(box.conf[0])
                        raw_name = self.yolo.names[class_id]
                        canonical = self.canonical_model_class(class_id, raw_name)
                        if canonical is None:
                            continue
                        seen_this_frame.add(canonical)
                        max_confidence[canonical] = max(
                            box_confidence, max_confidence.get(canonical, 0.0)
                        )
                for name in seen_this_frame:
                    counts[name] = counts.get(name, 0) + 1
            except Exception as exc:
                rospy.logwarn("YOLO检测失败：%s", exc)
            rate.sleep()

        detected = {}
        for name, hits in counts.items():
            if return_all or hits >= min_hits:
                detected[name] = {
                    "hits": hits,
                    "confidence": max_confidence.get(name, 0.0),
                    "source": "yolo_roi" if roi is not None else "yolo_global",
                    "confirmed": hits >= min_hits,
                }
        rospy.loginfo(
            "视觉候选统计：mode=%s conf>=%.2f min_hits=%d counts=%s max_confidence=%s",
            "ROI" if roi is not None else "GLOBAL",
            confidence_threshold,
            min_hits,
            counts,
            max_confidence,
        )
        rospy.loginfo(
            "%s识别结果：%s",
            "候选" if return_all else "稳定",
            detected,
        )
        return detected

    def detect_find_objects(self, requested_target, mock_classes):
        """寻物使用两批独立新画面复核，首轮结果不能单独判定找到。"""
        rospy.sleep(self.find_camera_settle_seconds)
        first_pass = self.detect_classes(
            duration=max(4.0, self.detect_seconds),
            mock_classes=mock_classes,
            confidence=self.find_target_confidence,
            min_hits=2,
        )
        rospy.loginfo(
            "寻物首轮候选=%s；启动独立新画面二次确认：conf>=%.2f min_hits=%d",
            first_pass,
            self.find_verify_confidence,
            self.find_verify_min_hits,
        )
        second_pass = self.detect_classes(
            duration=self.find_verify_seconds,
            mock_classes=mock_classes,
            confidence=self.find_verify_confidence,
            min_hits=self.find_verify_min_hits,
        )
        # 第二批新画面是最终依据。首轮独有的瞬时误识别（例如厨房把
        # 食材误判为书包）不得进入播报结果；同时保留第二轮稳定识别到
        # 的食材，供寻物任务播报“看到了菜品，但没有目标”。
        verified = {}
        for class_name, data in second_pass.items():
            if class_name not in TARGET_NAMES and class_name not in FOOD_NAMES:
                continue
            verified[class_name] = dict(data)
            if class_name in first_pass:
                verified[class_name]["confidence"] = max(
                    float(data.get("confidence", 0.0)),
                    float(first_pass[class_name].get("confidence", 0.0)),
                )
                verified[class_name]["verified_in_both_passes"] = True
        rospy.loginfo(
            "寻物二次确认结果：目标=%s verified=%s",
            TARGET_NAMES[requested_target],
            verified,
        )
        return verified

    @staticmethod
    def point_in_polygon(x, y, polygon):
        inside = False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if (yi > y) != (yj > y):
                edge_x = (xj - xi) * (y - yi) / (yj - yi) + xi
                if x < edge_x:
                    inside = not inside
            j = i
        return inside

    @staticmethod
    def point_segment_distance(px, py, ax, ay, bx, by):
        vx = bx - ax
        vy = by - ay
        length_sq = vx * vx + vy * vy
        if length_sq == 0:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * vx + (py - ay) * vy) / length_sq
        t = max(0.0, min(1.0, t))
        nearest_x = ax + t * vx
        nearest_y = ay + t * vy
        return math.hypot(px - nearest_x, py - nearest_y)

    def near_forbidden_zone(self, x, y, polygon):
        if self.point_in_polygon(x, y, polygon):
            return True
        for index, point in enumerate(polygon):
            next_point = polygon[(index + 1) % len(polygon)]
            if self.point_segment_distance(
                x, y, point[0], point[1], next_point[0], next_point[1]
            ) <= self.geofence_margin:
                return True
        return False

    def geofence_callback(self, _event):
        if not self.forbidden_zones:
            return
        try:
            trans, _ = self.tf_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
            )
        except Exception:
            return
        x, y = trans[0], trans[1]
        for zone in self.forbidden_zones:
            if self.near_forbidden_zone(x, y, zone["polygon"]):
                self.geofence_violation = True
                self.client.cancel_all_goals()
                self.publish_stop()
                rospy.logerr_throttle(
                    1.0, "电子围栏停车：接近禁行区 %s", zone["name"]
                )
                return

    def clear_navigation_costmaps(self):
        """清除激光障碍层中的残留代价；静态地图和虚拟墙不会被删除。"""
        if self.clear_costmaps is None:
            return
        try:
            self.clear_costmaps()
            rospy.sleep(0.35)
        except Exception as exc:
            rospy.logwarn("清理导航代价地图失败：%s", exc)

    def cancel_active_navigation_goal(self):
        """发送新目标前收干净上一个活动目标，避免actionlib目标句柄串线。"""
        try:
            state = self.client.get_state()
            active_states = (
                GoalStatus.PENDING,
                GoalStatus.ACTIVE,
                GoalStatus.PREEMPTING,
                GoalStatus.RECALLING,
            )
            if state not in active_states:
                return
            rospy.logwarn("发送新目标前取消残留导航目标，旧状态：%s", state)
            self.client.cancel_all_goals()
            self.client.wait_for_result(rospy.Duration(0.5))
            self.publish_stop()
            rospy.sleep(0.10)
        except Exception as exc:
            rospy.logwarn("清理残留导航目标失败：%s", exc)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def rotate_relative(self, angle, timeout=14.0):
        """依据 odom 闭环原地转动；只改变朝向，不修改任何导航点。"""
        try:
            _, start_rotation = self.tf_listener.lookupTransform(
                "/odom", "/base_footprint", rospy.Time(0)
            )
            start_yaw = euler_from_quaternion(start_rotation)[2]
        except Exception as exc:
            rospy.logwarn("读取底盘朝向失败，无法执行原地转向：%s", exc)
            self.publish_stop()
            return False

        target_yaw = self.normalize_angle(start_yaw + float(angle))
        deadline = rospy.Time.now() + rospy.Duration(float(timeout))
        rate = rospy.Rate(20)
        stable_count = 0

        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                if self.mission_time_expired() or self.geofence_violation:
                    return False
                try:
                    _, rotation = self.tf_listener.lookupTransform(
                        "/odom", "/base_footprint", rospy.Time(0)
                    )
                except Exception as exc:
                    rospy.logwarn_throttle(1.0, "转向过程中读取朝向失败：%s", exc)
                    rate.sleep()
                    continue

                yaw = euler_from_quaternion(rotation)[2]
                error = self.normalize_angle(target_yaw - yaw)
                if abs(error) <= 0.06:
                    stable_count += 1
                    self.publish_stop()
                    if stable_count >= 3:
                        rospy.loginfo("走廊转向完成，误差 %.3f rad", error)
                        return True
                    rate.sleep()
                    continue

                stable_count = 0
                command = Twist()
                angular_speed = min(0.32, max(0.12, abs(error) * 0.70))
                command.angular.z = angular_speed if error > 0.0 else -angular_speed
                self.cmd_pub.publish(command)
                rate.sleep()
        finally:
            self.publish_stop()

        rospy.logwarn("走廊180度转向超时")
        return False

    def prepare_corridor_departure(self):
        """待机时摄像头面向场外；执行任务前转向赛道内部。"""
        if self.current_location != "corridor":
            return True
        rospy.loginfo("离开走廊前转向赛道内部")
        if not self.rotate_relative(math.pi):
            return False
        # 清掉人员靠近下达指令时可能留在局部代价地图中的动态障碍。
        self.clear_navigation_costmaps()
        rospy.sleep(0.35)
        return True

    def restore_corridor_wait_orientation(self):
        """走廊向内检查结束后转回原来的场外待机朝向。"""
        if self.current_location != "corridor":
            return True
        rospy.loginfo("走廊识别结束，转回原待机朝向")
        if not self.rotate_relative(math.pi):
            return False
        self.clear_navigation_costmaps()
        rospy.sleep(0.35)
        return True

    def is_physically_at(self, name, tolerance=None):
        """使用 map->base_footprint 实测坐标判断是否到达指定点。"""
        if name not in self.points:
            return False
        try:
            translation, _ = self.tf_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
            )
        except Exception as exc:
            rospy.logwarn("无法复核 %s 的实际位置：%s", name, exc)
            return False

        goal_x, goal_y = float(self.points[name][0]), float(self.points[name][1])
        distance = math.hypot(translation[0] - goal_x, translation[1] - goal_y)
        limit = (
            self.arrival_fallback_tolerance
            if tolerance is None else float(tolerance)
        )
        rospy.loginfo("%s 实际位置复核：距离=%.3fm", name, distance)
        return distance <= limit

    def confirm_goal_arrival(self, name, goal_x, goal_y, goal_yaw, tolerance):
        """连续多次复核终点位姿，防止move_base状态成功但底盘未真正入框。"""
        samples = []
        for _ in range(self.arrival_confirm_samples):
            try:
                translation, rotation = self.tf_listener.lookupTransform(
                    "/map", "/base_footprint", rospy.Time(0)
                )
                current_yaw = euler_from_quaternion(rotation)[2]
                samples.append((
                    math.hypot(translation[0] - goal_x, translation[1] - goal_y),
                    self.normalize_angle(goal_yaw - current_yaw),
                ))
            except Exception as exc:
                rospy.logwarn("%s 到点连续复核读取TF失败：%s", name, exc)
            rospy.sleep(0.10)

        if len(samples) < self.arrival_confirm_samples:
            return False
        max_distance = max(item[0] for item in samples)
        max_yaw_error = max(abs(item[1]) for item in samples)
        rospy.loginfo(
            "%s 连续到点复核：最大距离=%.3fm，最大朝向误差=%.3frad",
            name, max_distance, max_yaw_error,
        )
        if max_distance > float(tolerance):
            return False

        # 安全中转点只用于约束路径，不要求最终面向；计分点必须同时满足朝向。
        if name.startswith("route_"):
            return True
        if max_yaw_error <= self.arrival_fallback_yaw_tolerance:
            return True
        rospy.logwarn("%s 位置已达标，单独校正终点朝向", name)
        if not self.rotate_relative(samples[-1][1], timeout=10.0):
            return False
        return self.confirm_goal_position_only(name, goal_x, goal_y, tolerance)

    def confirm_goal_position_only(self, name, goal_x, goal_y, tolerance):
        try:
            translation, _ = self.tf_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
            )
        except Exception as exc:
            rospy.logwarn("%s 朝向校正后位置复核失败：%s", name, exc)
            return False
        distance = math.hypot(translation[0] - goal_x, translation[1] - goal_y)
        rospy.loginfo("%s 朝向校正后位置距离=%.3fm", name, distance)
        return distance <= float(tolerance)

    def resume_corridor_command_wait(self):
        """只有实测已回到走廊才恢复下一条指令等待。"""
        if not self.is_physically_at("corridor"):
            rospy.logerr("尚未实际回到走廊，禁止播报已返回")
            return False
        self.latest_voice = ""
        self.face_absent_count = 0
        self.state = "WAIT_COMMAND"
        self.set_voice_input_enabled(True)
        rospy.loginfo("已返回走廊，恢复下一条任务指令等待")
        return True

    def accept_arrived_goal_after_abort(self, name, goal_x, goal_y, goal_yaw):
        """move_base 终点优化失败时，用实测 map 位姿判断是否已到点。"""
        try:
            translation, rotation = self.tf_listener.lookupTransform(
                "/map", "/base_footprint", rospy.Time(0)
            )
        except Exception as exc:
            rospy.logwarn("无法读取当前 map 位姿，不启用到点兜底：%s", exc)
            return False

        current_x, current_y = translation[0], translation[1]
        current_yaw = euler_from_quaternion(rotation)[2]
        distance = math.hypot(current_x - goal_x, current_y - goal_y)
        yaw_error = self.normalize_angle(goal_yaw - current_yaw)

        rospy.loginfo(
            "%s 终点复核：距离=%.3fm，朝向误差=%.3frad",
            name, distance, yaw_error,
        )
        fallback_limit = (
            self.route_arrival_confirm_tolerance
            if name.startswith("route_")
            else self.arrival_fallback_tolerance
        )
        if distance > fallback_limit:
            return False

        if abs(yaw_error) > self.arrival_fallback_yaw_tolerance:
            rospy.logwarn("%s 已到兜底距离，单独校正终点朝向", name)
            if not self.rotate_relative(yaw_error, timeout=10.0):
                rospy.logwarn("%s 朝向校正未完成，不再误报到达", name)
                return False

        if not self.confirm_goal_position_only(name, goal_x, goal_y, fallback_limit):
            return False
        rospy.logwarn("%s 导航器优化失败，但连续物理复核已到达", name)
        self.current_location = name
        return True

    def goto(self, name):
        if self.mission_time_expired():
            rospy.logwarn("比赛任务时间已到，拒绝发送新的导航目标")
            return False

        if name not in self.points:
            rospy.logwarn("未找到导航点：%s", name)
            return False

        x, y, yaw = [float(value) for value in self.points[name]]

        # current_location 只能表示已确认到达的点。一旦开始前往其他点，
        # 立即清掉旧标记，避免导航失败后误以为仍在走廊。
        if self.current_location != name:
            self.current_location = None

        for zone in self.forbidden_zones:
            if self.near_forbidden_zone(x, y, zone["polygon"]):
                rospy.logerr("导航点 %s 位于禁行区安全范围内", name)
                return False

        q = quaternion_from_euler(0, 0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.geofence_violation = False
        self.cancel_active_navigation_goal()

        for attempt in range(2):
            # 第一次直接使用当前 costmap 正常规划。
            # 只有第一次真正失败后，第二次重试前才清理动态障碍残留。
            if attempt == 0:
                rospy.loginfo("前往 %s", name)
            else:
                rospy.logwarn("%s 首次导航失败，清理代价地图后重试", name)
                self.clear_navigation_costmaps()
                rospy.sleep(1.0)

            goal.target_pose.header.stamp = rospy.Time.now()
            self.client.send_goal(goal)
            finished = self.client.wait_for_result(
                rospy.Duration(self.navigation_timeout)
            )

            if self.geofence_violation:
                self.client.cancel_all_goals()
                self.publish_stop()
                return False

            if not finished:
                self.client.cancel_goal()
                self.publish_stop()
                rospy.logwarn("%s 导航超时", name)
                if attempt == 0:
                    rospy.sleep(0.5)
                    continue
                return False

            state = self.client.get_state()
            rospy.loginfo("%s 导航状态：%s", name, state)

            if state == GoalStatus.SUCCEEDED:
                tolerance = (
                    self.route_arrival_confirm_tolerance
                    if name.startswith("route_")
                    else self.arrival_confirm_tolerance
                )
                if self.confirm_goal_arrival(name, x, y, yaw, tolerance):
                    self.current_location = name
                    return True
                self.publish_stop()
                rospy.logwarn("%s move_base返回成功，但物理到点复核未通过", name)
                if attempt == 0:
                    rospy.sleep(0.5)
                    continue
                return False

            self.publish_stop()
            if self.accept_arrived_goal_after_abort(name, x, y, yaw):
                return True
            if attempt == 0:
                rospy.sleep(0.5)

        return False

    def goto_route_waypoints(self, waypoint_names, route_label):
        """按顺序经过安全中转点；中转点不停留、不播报。"""
        for waypoint in waypoint_names:
            if self.current_location == waypoint:
                rospy.loginfo("%s：已在中转点 %s，跳过重复导航", route_label, waypoint)
                continue
            rospy.loginfo("%s：前往安全中转点 %s", route_label, waypoint)
            if not self.goto(waypoint):
                rospy.logwarn("%s：中转点 %s 导航失败", route_label, waypoint)
                return False
        return True

    def is_at_start(self):
        return self.current_location == "start" or self.is_physically_at("start")

    def goto_and_hold(self, name):
        task_places = ("restaurant", "kitchen", "livingroom", "bedroom")

        # ① 出发区 -> A -> 走廊。这里不经过B。
        if name == "corridor" and self.is_at_start():
            if not self.goto_route_waypoints(
                self.start_to_corridor_waypoints,
                "出发区到走廊",
            ):
                return False

        # ② 从走廊真正出发执行任务时：
        # corridor -> B -> restaurant/kitchen/livingroom/bedroom。
        elif self.current_location == "corridor" and name in task_places:
            if not self.goto_route_waypoints(
                self.corridor_to_task_waypoints,
                "走廊到任务区域",
            ):
                return False

        # ④ 第三项任务结束回走廊：任务点 -> 餐厅 -> 原B -> 走廊。
        elif name == "corridor" and self.current_location in task_places:
            if not self.goto_route_waypoints(
                self.task_to_corridor_waypoints,
                "任务区域到走廊",
            ):
                return False

        if not self.goto(name):
            return False
        self.hold_position()
        return not self.geofence_violation

    def return_to_start_safely(self):
        """任务一、二结束后按当前位置 -> restaurant -> B -> A -> start返回。"""
        if self.is_at_start():
            self.current_location = "start"
            return True

        if not self.goto_route_waypoints(
            self.task_to_start_waypoints,
            "任务返程到出发区",
        ):
            return False

        if not self.goto("start"):
            return False

        # 出发区是每项任务的返程落点，按原规则稳定停留。
        self.hold_position()
        return not self.geofence_violation

    def arm_corridor_wait(self, prompt_text=None, require_clear=True):
        """关闭语音输入，等待人员靠近后播报提示并重新开放命令。"""
        self.latest_voice = ""
        self.face_absent_count = 0
        self.corridor_wake_prompt = (
            str(prompt_text).strip()
            if prompt_text
            else "你好，需要帮助吗？"
        )
        self.set_voice_input_enabled(False)
        self.state = "WAIT_CLEAR" if require_clear else "WAIT_PERSON"

    def prepare_next_task(self, already_at_corridor=False):
        if not already_at_corridor and not self.goto_and_hold("corridor"):
            self.speak("无法到达走廊待机区，请检查导航。")
            return False
        self.arm_corridor_wait()
        return True

    def tour_task(self):
        self.speak("好的，请跟我来。")
        # 机器人在走廊待机时摄像头朝向场外。导览开始后只原地转向
        # 赛道内部，随后直接前往四个计分区域；走廊不属于导览点。
        if not self.prepare_corridor_departure():
            self.speak("走廊转向失败，导览任务暂停。")
            return False
        for place in ("restaurant", "kitchen", "livingroom", "bedroom"):
            if not self.goto_and_hold(place):
                self.speak("%s导航失败，任务暂停。" % self.place_name[place])
                return False
            self.speak(self.intro_text[place])
        self.speak("这些就是参观区域啦，我要回去继续工作了。")
        return True

    def assistant_task(self):
        self.speak("好的，我去看看冰箱里有什么。")
        # 菜谱任务不在走廊执行视觉检查，直接交给move_base
        # 规划到厨房，避免先原地180度转圈。
        if not self.goto_and_hold("kitchen"):
            self.speak("厨房导航失败。")
            return False
        foods = {}
        global_evidence = {}
        recognition_deadline = time.monotonic() + self.food_recognition_timeout

        # 第1~3轮：全图高分辨率扫描。低于单轮稳定门槛的候选也保留，
        # 三轮之间真正累计新帧命中数，避免每轮1~2帧都被提前丢弃。
        for round_index in range(self.food_global_rounds):
            if len(foods) >= 3 or time.monotonic() >= recognition_deadline:
                break
            rospy.loginfo("食材全图识别第%d/%d轮", round_index + 1, self.food_global_rounds)
            detections = self.detect_classes(
                duration=min(
                    self.food_round_seconds,
                    max(1.0, recognition_deadline - time.monotonic()),
                ),
                mock_classes=self.mock_foods,
                confidence=self.food_global_confidence,
                min_hits=self.food_global_min_hits,
                return_all=True,
                imgsz=self.food_global_imgsz,
            )
            self.merge_detections(
                global_evidence,
                {name: data for name, data in detections.items() if name in FOOD_NAMES},
            )
            for name, data in global_evidence.items():
                if int(data.get("hits", 0)) >= self.food_global_min_hits:
                    foods[name] = dict(data)
            rospy.loginfo(
                "食材前三轮累计证据：round=%d evidence=%s confirmed=%s",
                round_index + 1,
                global_evidence,
                foods,
            )

        # 第4轮：只有ROI标定有效时才裁剪；暂未标定时安全退化为全图，绝不退出。
        if len(foods) < 3 and time.monotonic() < recognition_deadline:
            self.speak("食材还不足三类，我再框选确认一次。")
            if self.food_roi is None:
                rospy.logwarn("食材ROI尚未标定，第4轮暂用全图新帧识别")
            detections = self.detect_classes(
                duration=min(
                    self.food_round_seconds,
                    max(1.0, recognition_deadline - time.monotonic()),
                ),
                mock_classes=self.mock_foods,
                confidence=self.food_retry_confidence,
                min_hits=self.food_retry_min_hits,
                roi=self.food_roi,
            )
            self.merge_detections(
                foods,
                {name: data for name, data in detections.items() if name in FOOD_NAMES},
            )

        # 只有前三轮全图和第四轮ROI都不足三类，才调用一次千问视觉。
        if len(foods) < 3 and time.monotonic() < recognition_deadline:
            self.merge_detections(foods, self.identify_foods_with_qwen())

        # 千问未配置、断网或仍不足时，不反悔退出；继续用本地新帧识别到任务时限。
        extra_round = 0
        while (
            len(foods) < 3
            and time.monotonic() < recognition_deadline
            and not rospy.is_shutdown()
            and not self.mission_time_expired()
        ):
            extra_round += 1
            remaining = recognition_deadline - time.monotonic()
            rospy.loginfo("食材本地持续复核第%d轮，剩余%.1f秒", extra_round, remaining)
            detections = self.detect_classes(
                duration=min(self.food_round_seconds, max(1.0, remaining)),
                mock_classes=self.mock_foods,
                confidence=self.food_retry_confidence,
                min_hits=self.food_retry_min_hits,
                roi=self.food_roi,
            )
            self.merge_detections(
                foods,
                {name: data for name, data in detections.items() if name in FOOD_NAMES},
            )

        if len(foods) < 3:
            recognized = [FOOD_NAMES[name] for name in foods]
            if recognized:
                self.speak(
                    "识别时限内确认了%s，但仍不足三类，请调整货架或相机后重试。"
                    % "、".join(recognized)
                )
            else:
                self.speak("没有稳定识别到食材，请检查模型和相机角度。")
            return False
        # 按稳定帧数、最高置信度选出恰好三类，保证播报和大模型输入完全一致。
        selected = sorted(
            foods,
            key=lambda name: (
                foods[name].get("hits", 0),
                foods[name].get("confidence", 0.0),
            ),
            reverse=True,
        )[:3]
        selected_cn = [FOOD_NAMES[name] for name in selected]
        order_names = ("第一", "第二", "第三")
        for index, food_name in enumerate(selected_cn):
            self.speak("识别到%s种食材，%s。" % (order_names[index], food_name))

        # 联网优先；网络卡顿时在短超时后使用历史DeepSeek缓存或离线应急做法。
        recipe_text = self.generate_recipe_with_deepseek(selected_cn)
        self.speak(recipe_text)
        return True

    def find_object_task(self, target):
        target_cn = TARGET_NAMES[target]
        self.speak("好的，我去帮你找%s。" % target_cn)
        all_regions_reached = True
        target_found = False
        found_place = None

        # 走廊待机时摄像头面向场外，先转回赛道内部；随后严格按规则顺序
        # 餐厅->厨房->客厅->卧室->走廊检查。即使提前找到目标也继续巡检，
        # 保证五个区域的导航和事件播报完整。
        if self.current_location != "corridor" and not self.goto_and_hold("corridor"):
            self.speak("走廊导航失败，无法开始寻物。")
            return False
        if not self.prepare_corridor_departure():
            self.speak("走廊转向失败，请检查机器人周围是否有障碍物。")
            return False

        for place in ("restaurant", "kitchen", "livingroom", "bedroom", "corridor"):
            if not self.goto_and_hold(place):
                self.speak("%s导航失败，继续检查下一区域。" % self.place_name[place])
                all_regions_reached = False
                continue
            corridor_scan = place == "corridor"
            if corridor_scan:
                # 最后一个寻物点是走廊。到位时仍为面向场外的待机朝向，
                # 先原地转向赛道内部，再使用与其他货架完全相同的真实识别流程。
                if not self.prepare_corridor_departure():
                    self.speak("走廊转向失败，无法完成走廊货架识别。")
                    all_regions_reached = False
                    continue
            # 虚拟检测也同时模拟手机和书包，确保能测试“发现另一物品”的规则分支。
            mock_classes = [
                name for name, item_place in self.mock_find_locations.items()
                if item_place == place
            ]
            # 首轮未确认目标时自动增加一次低门槛、多新帧复核；同时保留
            # 对另一物品的识别结果，确保能正确播报“看到了书包/手机”。
            detections = self.detect_find_objects(target, mock_classes)
            others = [
                TARGET_NAMES[name]
                for name in TARGET_NAMES
                if name != target and name in detections
            ]
            foods = [
                FOOD_NAMES[name]
                for name in FOOD_NAMES
                if name in detections
            ]
            target_data = detections.get(target)
            if (
                target_data is not None
                and target_data.get("confidence", 0.0) < self.find_target_confidence
            ):
                rospy.logwarn(
                    "%s候选置信度%.3f低于寻物门槛%.3f，按误报忽略",
                    target_cn,
                    target_data.get("confidence", 0.0),
                    self.find_target_confidence,
                )
                target_data = None

            if target_data is not None:
                first_discovery = not target_found
                target_found = True
                if found_place is None:
                    found_place = place
                message = (
                    "找到%s啦，在这里！" % target_cn
                    if first_discovery
                    else "这里也看到了%s。" % target_cn
                )
                if others:
                    message += "我还看到了%s。" % "、".join(others)
                if corridor_scan and foods:
                    message += "我还发现了菜品：%s。" % "、".join(foods)
                self.speak(message)
                if corridor_scan and not self.restore_corridor_wait_orientation():
                    rospy.logwarn("走廊识别播报完成，但转回原待机朝向失败")
                continue

            requested_name = "背包" if target == "backpack" else target_cn
            corridor_orientation_restored = False
            if foods and others:
                self.speak(
                    "我在这里发现了菜品：%s，也看到了%s，但没有你要找的%s。"
                    % ("、".join(foods), "、".join(others), requested_name)
                )
            elif foods:
                self.speak(
                    "我在这里发现了菜品：%s，但没有你要找的%s。"
                    % ("、".join(foods), requested_name)
                )
            elif others:
                self.speak("我看到了%s，但没有找到%s。" % ("、".join(others), target_cn))
            else:
                # 走廊是最后一个检查点：什么也没有时先转回原朝向，
                # 再播报结果，不再说“继续前往下一个点”。
                if corridor_scan:
                    corridor_orientation_restored = self.restore_corridor_wait_orientation()
                    if not corridor_orientation_restored:
                        rospy.logwarn("走廊未识别到物品，但转回原待机朝向失败")
                    self.speak("这里什么也没有。")
                else:
                    self.speak("这里什么也没有，我继续前往下一个任务点找一找。")

            # 识别到其他物品时先面向走廊内部播报，再转回原待机朝向。
            if corridor_scan and not corridor_orientation_restored:
                corridor_orientation_restored = self.restore_corridor_wait_orientation()
                if not corridor_orientation_restored:
                    rospy.logwarn("走廊识别播报完成，但转回原待机朝向失败")

        if not target_found:
            self.speak("所有区域检查完成，但没有找到%s。" % target_cn)
        return all_regions_reached and target_found

    def classify_command(self, text):
        # 干扰命令必须先判断，防止“参观完了”误触发导览。
        if any(word in text for word in ("参观完", "不用参观", "结束参观")):
            return "distractor_tour", None
        if any(word in text for word in ("穿什么", "衣服", "怎么穿")):
            return "distractor_clothes", None
        # 手动导航口令不计入三项比赛任务顺序。
        if any(word in text for word in (
            "回去充电", "去充电", "返回充电", "回去充电桩", "去充电桩",
            "返回充电桩", "回到充电桩", "到充电桩", "回到充电区",
            "返回充电区", "到充电区", "开始回充", "回充", "充电",
        )):
            return "manual_charge", None
        if any(word in text for word in (
            "回到出发区", "返回出发区", "回去出发区", "去出发区",
            "到出发区", "回出发区", "前往出发区", "回到起点",
            "返回起点", "回去起点", "去起点", "到起点",
        )):
            return "manual_start", None
        negative_find_words = ("不用找", "不要找", "别找", "停止寻找")
        if "手机" in text and not any(word in text for word in negative_find_words):
            return "find", "phone"
        backpack_meaning = any(word in text for word in ("书包", "背包", "我的包")) or (
            "包" in text and any(word in text for word in (
                "找", "哪里", "哪儿", "在哪", "不见", "丢", "寻找",
            ))
        )
        if backpack_meaning and not any(word in text for word in negative_find_words):
            return "find", "backpack"
        cooking_phrase = any(word in text for word in (
            "做什么菜", "吃什么", "吃点什么", "推荐什么菜", "推荐一下",
            "推荐一道菜", "推荐菜", "推荐菜谱", "推荐食谱", "菜谱", "食谱",
            "看看冰箱", "冰箱里有什么", "冰箱有什么", "冰箱食材",
            "还有什么菜", "剩什么菜", "有什么菜可以做", "今天吃什么",
            "吃啥", "做啥菜",
        ))
        cooking_meaning = (
            cooking_phrase
            or ("菜" in text and any(word in text for word in ("推荐", "做", "吃", "什么")))
            or ("冰箱" in text and any(word in text for word in ("看", "有什么", "食材", "剩", "检查")))
        )
        if cooking_meaning:
            return "assistant", None
        if any(word in text for word in (
            "带我参观", "参观一下", "带我看看", "语音导览"
        )):
            return "tour", None
        return "unknown", None

    def process_command(self, text):
        intent, argument = self.classify_command(text)
        rospy.loginfo(
            "国赛语音指令解析：text=%s intent=%s argument=%s state=%s",
            text, intent, argument, self.state,
        )

        if intent == "distractor_clothes":
            self.state = "RUNNING_TASK"
            self.broadcast_weather_and_clothing()
            self.latest_voice = ""
            self.publish_stop()
            self.state = "WAIT_COMMAND"
            self.set_voice_input_enabled(True)
            rospy.loginfo("天气播报完成，已恢复语音等待")
            return

        if intent == "manual_start":
            self.state = "RUNNING_TASK"
            self.speak("好的，正在返回出发区。")
            if self.return_to_start_safely():
                self.state = "WAIT_COMMAND"
                self.speak("已到达出发区。")
            else:
                self.state = "WAIT_COMMAND"
                self.speak("返回出发区失败，请检查导航。")
            self.set_voice_input_enabled(True)
            return

        if intent == "manual_charge":
            # 手动回充允许两个合法起点：
            # 1) 走廊：corridor -> A -> charge
            # 2) 出发区：start -> charge
            # 手动回充不计入三项比赛任务完成数。
            at_corridor = self.is_physically_at("corridor")
            at_start = self.is_at_start()

            if not at_corridor and not at_start:
                self.speak("请在走廊或出发区下达充电指令。")
                self.state = "WAIT_COMMAND"
                self.set_voice_input_enabled(True)
                return

            self.state = "RUNNING_TASK"
            self.speak("好的，我回去充电了")

            if at_corridor:
                charge_ok = self.auto_charge(
                    require_start=False,
                    announce_departure=False,
                )
            else:
                # 已经在start，auto_charge(require_start=True)不会再绕行，
                # 会直接导航到charge再开始AR视觉对接。
                charge_ok = self.auto_charge(
                    require_start=True,
                    announce_departure=False,
                )

            self.state = "FINISHED" if charge_ok else "WAIT_COMMAND"
            self.set_voice_input_enabled(not charge_ok)
            return

        if intent in ("distractor_tour", "unknown"):
            self.state = "RUNNING_TASK"
            self.answer_distractor_with_deepseek(text)
            self.latest_voice = ""
            self.publish_stop()
            self.state = "WAIT_COMMAND"
            self.set_voice_input_enabled(True)
            return

        # 三项任务允许按裁判实际下达顺序穿插执行；完成过的任务不重复计数。
        if intent in self.completed_tasks:
            self.speak("这项任务已经完成，请下达其他任务指令。")
            self.set_voice_input_enabled(True)
            return

        self.state = "RUNNING_TASK"
        if intent == "tour":
            success = self.tour_task()
        elif intent == "assistant":
            success = self.assistant_task()
        else:
            success = self.find_object_task(argument)

        # 失败任务不计入 completed_tasks。
        # 仍然按普通任务收尾：先回出发区，再去走廊等待重新下达。
        if not success:
            if not self.return_to_start_safely():
                self.state = "ERROR"
                self.speak("无法沿安全路线返回出发区，请检查导航。")
                return

            if not self.goto_and_hold("corridor"):
                self.state = "ERROR"
                self.speak("无法返回走廊待机区，请检查导航。")
                return

            if not self.is_physically_at("corridor"):
                self.state = "ERROR"
                self.speak("机器人尚未实际回到走廊，请检查导航。")
                return

            self.speak("本项任务未完整完成，可以重新下达任务指令。")
            # 回到走廊后不直接接收指令；先等当前人员离开，再等人员重新靠近并主动问候。
            self.arm_corridor_wait()
            return

        # 成功完成后先记账，再判断它是不是第三个、也是最后一个任务。
        self.completed_tasks.add(intent)
        self.current_task_index = len(self.completed_tasks)
        is_final_task = len(self.completed_tasks) >= len(self.TASK_ORDER)

        if is_final_task:
            # 最后一项任务：绝不回出发区。
            # 任务当前位置 -> 餐厅 -> 原B -> corridor；确认到位后再播报并经A回充。
            if not self.goto_and_hold("corridor"):
                self.state = "ERROR"
                self.speak("最后一项任务完成，但无法返回走廊，请检查导航。")
                return

            if not self.is_physically_at("corridor"):
                self.state = "ERROR"
                self.speak("机器人尚未实际回到走廊，请检查导航。")
                return

            if self.skip_auto_charge_for_test:
                self.publish_stop()
                self.speak("所有任务都完成了，本次测试跳过自动回充。")
                rospy.logwarn("测试模式：已跳过自动回充，机器人保持在走廊")
                self.state = "FINISHED"
                return

            # 用户指定的最终播报必须在走廊真正到位以后出现。
            self.speak("所有任务都已经完成了，我要回去充电了，拜拜")

            # 不经过 start。走廊离开后按配置的安全中转点直接去 charge。
            charge_ok = self.auto_charge(
                require_start=False,
                announce_departure=False,
            )
            self.state = "FINISHED" if charge_ok else "ERROR"
            return

        # 任务一、任务二：逻辑保持不变。
        # 任务位置 -> 餐厅 -> 原B -> 原A -> start
        if not self.return_to_start_safely():
            self.state = "ERROR"
            self.speak("无法沿安全路线返回出发区，请检查导航。")
            return

        # start -> A(右) -> corridor
        if not self.goto_and_hold("corridor"):
            self.state = "ERROR"
            self.speak("无法返回走廊待机区，请检查导航。")
            return

        # 只有真正到走廊以后才播报任务完成。
        if not self.is_physically_at("corridor"):
            self.state = "ERROR"
            self.speak("机器人尚未实际回到走廊，请检查导航。")
            return

        rospy.loginfo("已回到走廊，等待人员靠近，下达命令")
        # 任务一、二完成后保持安静；只有再次检测到人脸才播报并开放语音命令。
        self.arm_corridor_wait(
            prompt_text="本项任务已经完成，你可以下达其他命令",
            require_clear=False,
        )

    def start_charge_camera_viewer(self):
        """视觉回充开始时打开底盘相机实时画面；失败不会影响回充主流程。"""
        if not self.charge_camera_viewer_enabled:
            rospy.loginfo("充电相机调试窗口已关闭")
            return

        if self.charge_camera_viewer_process is not None:
            if self.charge_camera_viewer_process.poll() is None:
                return
            self.charge_camera_viewer_process = None

        rospy.loginfo(
            "开始显示回充视觉相机：%s",
            self.charge_camera_topic,
        )

        try:
            # image_view 只是订阅并显示现有ROS图像，不会重启/抢占AR摄像头。
            self.charge_camera_viewer_process = subprocess.Popen(
                [
                    "rosrun",
                    "image_view",
                    "image_view",
                    "image:=%s" % self.charge_camera_topic,
                    "__name:=charge_camera_viewer",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception as exc:
            self.charge_camera_viewer_process = None
            rospy.logwarn(
                "无法自动打开回充相机窗口，但不会影响AR对接：%s。"
                "可手动执行：rosrun image_view image_view image:=%s",
                exc,
                self.charge_camera_topic,
            )

    def stop_charge_camera_viewer(self):
        """关闭由本程序启动的回充相机调试窗口。"""
        proc = self.charge_camera_viewer_process
        self.charge_camera_viewer_process = None
        if proc is None:
            return

        try:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.5)
                except Exception:
                    proc.kill()
        except Exception as exc:
            rospy.logwarn("关闭回充相机调试窗口失败：%s", exc)

    def auto_charge(self, require_start=True, announce_departure=True):
        """导航到充电等待点后执行最终回充。

        有电：
            真实充电信号为最高优先级，检测到后立即停止。

        无电：
            第1阶段 AR负责找桩、左右对正并靠近到 final_dock_trigger_x；
            第2阶段锁定直线方向，记录 /odom 起点，固定倒车 final_dock_distance；
            若 odom 异常，则 final_dock_timeout 到时强制停止，绝不无限后退。
        """
        if require_start:
            if not self.is_at_start():
                self.speak("先返回出发区，再前往充电桩。")
                if not self.return_to_start_safely():
                    self.speak("无法沿安全路线返回出发区，暂不执行回充。")
                    return False
        else:
            # 走廊直接回充：corridor -> A -> charge，不经过start。
            if not self.is_physically_at("corridor"):
                rospy.logerr("直接回充要求机器人先实际到达走廊")
                return False
            if self.corridor_to_charge_waypoints:
                if not self.goto_route_waypoints(
                    self.corridor_to_charge_waypoints,
                    "走廊直接回充",
                ):
                    self.speak("无法沿安全路线前往充电区。")
                    return False

        if announce_departure:
            self.speak("开始返回充电区。")

        if not self.goto("charge"):
            self.speak("无法到达充电区。")
            return False

        # 到达charge后打开当前AR底盘相机，便于现场确认摄像头是否正确。
        self.start_charge_camera_viewer()
        self.speak("已到达充电等待区，开始倒车视觉对接。")

        rate = rospy.Rate(10)
        cmd = Twist()
        approach_start_time = rospy.Time.now()
        charge_signal_since = None
        lost_count = 0

        # 最终Odom直线段状态
        final_dock_active = False
        final_dock_start_time = None
        final_dock_start_odom = None

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.mission_time_expired():
                self.publish_stop()
                self.speak("比赛时间已到，回充未完成。")
                self.stop_charge_camera_viewer()
                return False

            # ------------------------------------------------------
            # 第一优先级：有电时真实充电信号立即成功
            # ------------------------------------------------------
            if self.is_charging:
                if charge_signal_since is None:
                    charge_signal_since = now
                if now - charge_signal_since >= rospy.Duration(
                    self.charge_signal_confirm_seconds
                ):
                    self.publish_stop()
                    self.speak("已完成倒车对接，检测到充电信号。")
                    self.stop_charge_camera_viewer()
                    return True
            else:
                charge_signal_since = None

            # ------------------------------------------------------
            # 第2阶段：Odom固定距离直线倒车
            # 一旦进入这里，不再依赖AR是否还能看到。
            # ------------------------------------------------------
            if final_dock_active:
                cmd.linear.x = -self.final_dock_speed
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0

                travelled = None
                if final_dock_start_odom is not None and self.latest_odom_xy is not None:
                    dx = self.latest_odom_xy[0] - final_dock_start_odom[0]
                    dy = self.latest_odom_xy[1] - final_dock_start_odom[1]
                    travelled = math.hypot(dx, dy)

                    rospy.loginfo_throttle(
                        0.25,
                        "最终Odom回充：已倒 %.3fm / %.3fm，速度=-%.3fm/s",
                        travelled,
                        self.final_dock_distance,
                        self.final_dock_speed,
                    )

                    if travelled >= self.final_dock_distance:
                        self.publish_stop()
                        rospy.logwarn(
                            "无电回充完成：Odom最终倒车距离 %.3fm 已达到目标 %.3fm",
                            travelled,
                            self.final_dock_distance,
                        )
                        self.speak("已完成充电。比赛结束！")
                        self.stop_charge_camera_viewer()
                        return True

                elapsed = (now - final_dock_start_time).to_sec()

                # 时间只做绝对保险：即使odom异常也绝不无限倒车。
                if elapsed >= self.final_dock_timeout:
                    self.publish_stop()
                    if self.allow_charge_fallback:
                        rospy.logwarn(
                            "无电回充时间兜底：最终直线段已运行 %.2fs，"
                            "强制停止（Odom距离=%s）",
                            elapsed,
                            "%.3fm" % travelled if travelled is not None else "不可用",
                        )
                        self.speak("已完成倒车视觉回充对接。比赛结束！")
                        self.stop_charge_camera_viewer()
                        return True

                    self.speak("最终回充超时，已强制停止。")
                    self.stop_charge_camera_viewer()
                    return False

                self.cmd_pub.publish(cmd)
                rate.sleep()
                continue

            # ------------------------------------------------------
            # 第1阶段：AR负责找桩 + 对正 + 靠近最终直线段
            # ------------------------------------------------------
            try:
                trans, _ = self.tf_listener.lookupTransform(
                    "/base_link", "/ar_marker_0", rospy.Time(0)
                )
                x, y = float(trans[0]), float(trans[1])
                lost_count = 0

                rospy.loginfo_throttle(
                    0.5,
                    "AR位置 x=%.3f y=%.3f charging=%s",
                    x,
                    y,
                    self.is_charging,
                )

                # 横向还没对正：只旋转，不继续盲目倒车。
                if abs(y) > self.final_dock_align_y:
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.angular.z = -0.8 * y
                    self.cmd_pub.publish(cmd)
                    rate.sleep()
                    continue

                # 已经对正，并且到达最终直线段触发位置。
                if x >= self.final_dock_trigger_x:
                    self.publish_stop()

                    final_dock_start_time = rospy.Time.now()
                    final_dock_start_odom = self.latest_odom_xy
                    final_dock_active = True

                    rospy.logwarn(
                        "进入最终Odom直线回充：AR x=%.3f y=%.3f，"
                        "目标倒车 %.3fm，速度=-%.3fm/s，超时 %.1fs",
                        x,
                        y,
                        self.final_dock_distance,
                        self.final_dock_speed,
                        self.final_dock_timeout,
                    )

                    if final_dock_start_odom is None:
                        rospy.logwarn(
                            "当前尚未收到 %s；最终阶段先按时间保险执行，"
                            "收到Odom后将继续使用距离判定",
                            self.final_dock_odom_topic,
                        )

                    rate.sleep()
                    continue

                # AR阶段分段减速，只负责把机器人送到 x≈trigger_x。
                cmd.angular.z = -0.5 * y
                cmd.linear.y = 0.0

                if x < -0.45:
                    cmd.linear.x = -0.080
                elif x < -0.38:
                    cmd.linear.x = -0.050
                else:
                    cmd.linear.x = -0.025

                self.cmd_pub.publish(cmd)

            except Exception as exc:
                lost_count += 1

                # 最终Odom阶段尚未触发前，AR丢失绝不允许盲目倒车。
                self.publish_stop()

                if lost_count > 5:
                    # 只原地缓慢找AR，不向后移动。
                    cmd.linear.x = 0.0
                    cmd.linear.y = 0.0
                    cmd.angular.z = 0.12
                    self.cmd_pub.publish(cmd)

                rospy.logwarn_throttle(
                    1.0,
                    "最终直线段尚未触发，暂未检测到充电桩AR码：%s",
                    exc,
                )

            # AR阶段整体超时；进入Odom最终段之后由 final_dock_timeout 单独控制。
            if now - approach_start_time >= rospy.Duration(self.charge_timeout):
                self.publish_stop()
                self.speak("回充视觉对准超时，请检查充电桩AR码。")
                self.stop_charge_camera_viewer()
                return False

            rate.sleep()

        self.publish_stop()
        self.stop_charge_camera_viewer()
        return False

    def start_mission(self):
        self.set_voice_input_enabled(False)
        self.mission_start_time = rospy.Time.now()
        self.current_task_index = 0
        self.completed_tasks.clear()
        self.current_location = "start"
        self.latest_voice = ""
        return self.prepare_next_task()

    def run(self):
        self.speak("系统启动完成，等待比赛开始。")
        self.set_voice_input_enabled(True)
        if self.auto_start:
            self.start_requested = True

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if (
                self.state not in ("WAIT_START", "FINISHED", "ERROR")
                and self.mission_time_expired()
            ):
                self.publish_stop()
                self.speak("比赛任务时间已到。")
                self.state = "ERROR"
            elif self.state == "WAIT_START":
                if self.start_requested:
                    self.start_requested = False
                    if not self.start_mission():
                        self.state = "ERROR"
                elif self.latest_voice:
                    # 比赛尚未开始、机器人仍在出发区时，也允许直接下达
                    # “充电 / 去充电 / 回去充电”等手动回充口令。
                    # 其他任务口令仍然不会在 WAIT_START 状态执行。
                    text = self.latest_voice
                    self.latest_voice = ""
                    intent, _argument = self.classify_command(text)

                    if intent == "manual_charge":
                        rospy.loginfo("出发区收到手动充电指令：%s", text)
                        self.process_command(text)
                    else:
                        rospy.loginfo(
                            "比赛尚未开始，忽略非开始/非充电指令：%s",
                            text,
                        )
                        self.set_voice_input_enabled(True)
            elif self.state == "WAIT_CLEAR":
                if self.detect_face_present():
                    self.face_absent_count = 0
                else:
                    self.face_absent_count += 1
                    if self.face_absent_count >= 3:
                        self.state = "WAIT_PERSON"
                        rospy.loginfo("人员已离开检测范围，等待重新靠近")
            elif self.state == "WAIT_PERSON":
                if self.detect_face_present():
                    prompt = self.corridor_wake_prompt or "你好，需要帮助吗？"
                    self.speak(prompt)
                    self.corridor_wake_prompt = "你好，需要帮助吗？"
                    self.latest_voice = ""
                    self.state = "WAIT_COMMAND"
                    self.set_voice_input_enabled(True)
            elif self.state == "WAIT_COMMAND" and self.latest_voice:
                text = self.latest_voice
                self.latest_voice = ""
                self.process_command(text)
            elif self.state in ("FINISHED", "ERROR"):
                self.publish_stop()
            rate.sleep()


if __name__ == "__main__":
    try:
        ServiceRobot().run()
    except rospy.ROSInterruptException:
        pass
