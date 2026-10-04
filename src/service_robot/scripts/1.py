#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""2026 RAICOM 魔力元宝服务组主控程序。

适配 bobac3_ws（ROS Noetic），完成：
1. 人脸靠近激活
2. 语音导览
3. 智慧助手：冰箱食材识别、菜品和做法播报
4. 智能寻物：手机、书包，五区域依次巡检
5. 返回充电桩并调用视觉回充服务

首次使用必须修改下方 NAV_POINTS 中的地图点位。
"""

import math
import os
import socket
import threading
import time
import json
import urllib.error
import urllib.request
from collections import Counter

import actionlib
import cv2
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose2D
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from std_msgs.msg import String

from auto_charging.srv import SetCharge, SetChargeRequest
from robot_audio.srv import Collect, robot_iat, robot_tts


# 将机器人用 RViz 的 2D Nav Goal 实测坐标填入，格式为 (x, y, yaw弧度)
# 点位必须让底盘投影至少 50% 进入定位框，且不能压线
NAV_POINTS = {
    "corridor": [-1.2471, -2.2794, -1.4922],
    "restaurant": [-2.2801, -1.1501, 3.0699],
    "kitchen": [-1.2636, -1.1576, -1.5933],
    "livingroom": [-1.2529, -0.0868, 1.5591],
    "bedroom": [-2.2408, -0.0720, 1.4127],
    "start": [-0.0914, -0.1117, 0.0199],
    "charge": [-0.3600, 0.0040, -0.0260],
}

# best.pt 的真实类别顺序
CLASS_NAMES = [
    "apple", "fish", "doufu", "qingjiao", "egg", "qiezi", "xia", "pig",
    "shanghaiqing", "yumi", "fanqie", "tudou", "huanggua", "banana",
    "phone", "bag",
]

CN_NAMES = {
    "apple": "苹果", "fish": "鱼", "doufu": "豆腐", "qingjiao": "青椒",
    "egg": "鸡蛋", "qiezi": "茄子", "xia": "虾", "pig": "猪肉",
    "shanghaiqing": "上海青", "yumi": "玉米", "fanqie": "番茄",
    "tudou": "土豆", "huanggua": "黄瓜", "banana": "香蕉",
    "phone": "手机", "bag": "书包",
}

AREA_INTRO = {
    "restaurant": "这里是餐厅，是家人用餐和交流的区域。",
    "kitchen": "这里是厨房，是烹饪和准备食物的区域。",
    "livingroom": "这里是客厅，是休闲会客和家庭活动的区域。",
    "bedroom": "这里是卧室，是休息和睡眠的区域。",
}

YOLO_MODEL_PATH = "/home/bobac3/siyuu/best.pt"
DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")
DEEPSEEK_MODEL = "deepseek-v4-flash"
DEEPSEEK_URL = "https://api.deepseek.com/chat/completions"
DEEPSEEK_TIMEOUT = 8.0
WEATHER_API_URL = "https://wttr.in/Shenzhen?format=j1&lang=zh"
WEATHER_TIMEOUT = 5.0
WEATHER_FALLBACK_TEXT = "深圳今天多云，气温二十到二十八度，微风。"


class ServiceGroupRobot:
    def __init__(self):
        rospy.init_node("raicom_service_group", anonymous=False)
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.last_face_time = 0.0
        self.completed_tasks = set()

        self.model_path = YOLO_MODEL_PATH
        self.image_topic = rospy.get_param("~image_topic", "/head_camera/image_raw")
        self.confidence = float(rospy.get_param("~confidence", 0.55))
        self.scan_seconds = float(rospy.get_param("~scan_seconds", 4.0))
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 90.0))
        self.position_hold = float(rospy.get_param("~position_hold", 5.5))
        self.face_distance_ratio = float(rospy.get_param("~face_distance_ratio", 0.035))
        self.auto_charge_after_all = bool(rospy.get_param("~auto_charge_after_all", True))
        self.charge_track_id = int(rospy.get_param("~charge_track_id", 0))
        self.charge_track_dist = float(rospy.get_param("~charge_track_dist", 0.55))
        self.charge_docking_dist = float(rospy.get_param("~charge_docking_dist", -0.32))
        self.deepseek_api_key = DEEPSEEK_API_KEY
        self.deepseek_model = DEEPSEEK_MODEL
        self.deepseek_url = DEEPSEEK_URL
        self.deepseek_timeout = DEEPSEEK_TIMEOUT
        self.weather_api_url = WEATHER_API_URL
        self.weather_timeout = WEATHER_TIMEOUT
        self.weather_fallback_text = WEATHER_FALLBACK_TEXT

        self.points = dict(NAV_POINTS)
        ros_points = rospy.get_param("~nav_points", {})
        for key, value in ros_points.items():
            if isinstance(value, list) and len(value) == 3:
                self.points[key] = tuple(float(v) for v in value)

        self._validate_points()
        self._load_detector()

        cascade_path = os.path.join(cv2.data.haarcascades, "haarcascade_frontalface_default.xml")
        self.face_detector = cv2.CascadeClassifier(cascade_path)
        self.nav_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base 服务……")
        if not self.nav_client.wait_for_server(rospy.Duration(30.0)):
            raise RuntimeError("30 秒内未发现 move_base，请先启动 bobac3 导航")

        rospy.Subscriber(self.image_topic, Image, self._image_callback, queue_size=1, buff_size=2 ** 24)
        rospy.Subscriber("/service_group/manual_command", String, self._manual_callback, queue_size=5)

        self.tts = rospy.ServiceProxy("voice_tts", robot_tts)
        self.collect = rospy.ServiceProxy("voice_collect", Collect)
        self.iat = rospy.ServiceProxy("voice_iat", robot_iat)
        self.charge_service = rospy.ServiceProxy("auto_charging", SetCharge)

    def _validate_points(self):
        missing = [name for name, point in self.points.items() if point is None]
        if missing:
            raise RuntimeError("尚未配置地图点位：{}。请修改 NAV_POINTS 或设置 ~nav_points".format("、".join(missing)))

    def _load_detector(self):
        if not os.path.isfile(self.model_path):
            raise RuntimeError("找不到 YOLO 模型：{}".format(self.model_path))
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError("缺少 ultralytics，请执行：pip3 install ultralytics") from exc
        rospy.loginfo("加载模型：%s", self.model_path)
        self.detector = YOLO(self.model_path)
        model_names = self.detector.names
        actual = [model_names[i] for i in sorted(model_names)] if isinstance(model_names, dict) else list(model_names)
        if actual != CLASS_NAMES:
            rospy.logwarn("模型类别与预期不同，将以模型内置类别为准：%s", actual)

    def _image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.frame_lock:
                self.latest_frame = frame.copy()
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "图像转换失败：%s", exc)

    def _manual_callback(self, msg):
        rospy.loginfo("收到人工命令：%s", msg.data)
        self.pending_manual_command = msg.data.strip()

    def speak(self, text):
        rospy.loginfo("播报：%s", text)
        try:
            rospy.wait_for_service("voice_tts", timeout=5.0)
            self.tts(text=text, play=True)
        except Exception as exc:
            rospy.logerr("语音播报失败：%s", exc)

    def navigate(self, name, end_text=None):
        x, y, yaw = self.points[name]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        rospy.loginfo("导航至 %s：(%.3f, %.3f, %.3f)", name, x, y, yaw)
        self.nav_client.send_goal(goal)
        finished = self.nav_client.wait_for_result(rospy.Duration(self.nav_timeout))
        if not finished:
            self.nav_client.cancel_goal()
            self.speak("导航超时，我会重新尝试。")
            return False
        if self.nav_client.get_state() != 3:
            rospy.logerr("导航失败，状态码：%s", self.nav_client.get_state())
            return False
        if end_text:
            self.speak(end_text)
        rospy.sleep(self.position_hold)
        return True

    def wait_for_person(self):
        rospy.loginfo("在走廊等待人员靠近")
        consecutive = 0
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            with self.frame_lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()
            if frame is None:
                rate.sleep()
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = self.face_detector.detectMultiScale(gray, 1.12, 5, minSize=(65, 65))
            frame_area = float(frame.shape[0] * frame.shape[1])
            near = any((w * h) / frame_area >= self.face_distance_ratio for _, _, w, h in faces)
            consecutive = consecutive + 1 if near else 0
            if consecutive >= 3:
                self.last_face_time = time.time()
                self.speak("你好，需要帮助吗？")
                return True
            rate.sleep()
        return False

    def listen(self):
        manual = getattr(self, "pending_manual_command", "")
        if manual:
            self.pending_manual_command = ""
            return manual
        try:
            rospy.wait_for_service("voice_collect", timeout=8.0)
            rospy.wait_for_service("voice_iat", timeout=8.0)
            audio = self.collect(collect_flag=True).voice_filename
            text = self.iat(audiopath=audio).text.strip()
            rospy.loginfo("识别命令：%s", text)
            return text
        except Exception as exc:
            rospy.logerr("录音或识别失败：%s", exc)
            return ""

    def ask_deepseek(self, system_prompt, user_prompt, fallback):
        if not self.deepseek_api_key:
            rospy.logwarn("未配置 DeepSeek API Key，使用本地兜底回答")
            return fallback
        payload = {
            "model": self.deepseek_model,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
            "temperature": 0.4,
            "max_tokens": 220,
        }
        request = urllib.request.Request(
            self.deepseek_url,
            data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "Authorization": "Bearer {}".format(self.deepseek_api_key),
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.deepseek_timeout) as response:
                data = json.loads(response.read().decode("utf-8"))
            text = data["choices"][0]["message"]["content"].strip()
            return self._brief_tts_text(text) or fallback
        except (KeyError, IndexError, json.JSONDecodeError, urllib.error.URLError, TimeoutError, socket.timeout) as exc:
            rospy.logerr("DeepSeek 调用失败：%s", exc)
            return fallback

    def fetch_shenzhen_weather(self):
        request = urllib.request.Request(
            self.weather_api_url,
            headers={"User-Agent": "raicom-service-robot/1.0"},
            method="GET",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.weather_timeout) as response:
                data = json.loads(response.read().decode("utf-8"))
            current = data["current_condition"][0]
            forecast = data.get("weather", [{}])[0]
            desc = current.get("lang_zh", current.get("weatherDesc", [{"value": "天气不明"}]))[0]["value"]
            temp = current.get("temp_C", "")
            feels_like = current.get("FeelsLikeC", "")
            humidity = current.get("humidity", "")
            wind = current.get("windspeedKmph", "")
            min_temp = forecast.get("mintempC", "")
            max_temp = forecast.get("maxtempC", "")
            parts = ["深圳现在{}".format(desc)]
            if temp:
                parts.append("气温{}度".format(temp))
            if min_temp and max_temp:
                parts.append("今日{}到{}度".format(min_temp, max_temp))
            if feels_like:
                parts.append("体感{}度".format(feels_like))
            if humidity:
                parts.append("湿度{}%".format(humidity))
            if wind:
                parts.append("风速每小时{}公里".format(wind))
            return "，".join(parts) + "。"
        except (KeyError, IndexError, json.JSONDecodeError, urllib.error.URLError, TimeoutError, socket.timeout) as exc:
            rospy.logerr("深圳天气 API 调用失败：%s", exc)
            return self.weather_fallback_text

    @staticmethod
    def _brief_tts_text(text, limit=120):
        cleaned = str(text).replace("\n", "。")
        for token in ("**", "*", "#", "-", "1.", "2.", "3."):
            cleaned = cleaned.replace(token, "")
        cleaned = " ".join(cleaned.split())
        return cleaned[:limit]

    @staticmethod
    def parse_task(command):
        normalized = command.replace(" ", "")
        if any(word in normalized for word in ("参观", "导览", "介绍一下")):
            return "tour", None
        if any(word in normalized for word in ("做什么菜", "什么菜", "剩什么菜", "推荐菜", "食材", "冰箱")):
            return "assistant", None
        if "手机" in normalized:
            return "find", "phone"
        if "书包" in normalized or "背包" in normalized:
            return "find", "bag"
        return "interference", command

    def detect_objects(self):
        deadline = time.time() + self.scan_seconds
        votes = Counter()
        while time.time() < deadline and not rospy.is_shutdown():
            with self.frame_lock:
                frame = None if self.latest_frame is None else self.latest_frame.copy()
            if frame is None:
                rospy.sleep(0.1)
                continue
            results = self.detector.predict(frame, conf=self.confidence, verbose=False)
            for result in results:
                if result.boxes is None:
                    continue
                for class_id in result.boxes.cls.cpu().tolist():
                    name = result.names[int(class_id)]
                    votes[name] += 1
            rospy.sleep(0.12)
        detections = [name for name, count in votes.items() if count >= 2]
        detections.sort(key=lambda item: votes[item], reverse=True)
        rospy.loginfo("检测汇总：%s", [(name, votes[name]) for name in detections])
        return detections

    def task_tour(self):
        self.speak("好的，请跟我来。")
        for area in ("restaurant", "kitchen", "livingroom", "bedroom"):
            if not self.navigate(area, AREA_INTRO[area]):
                self.speak("前往下一个地点。")
        self.navigate("start", "参观结束，我们已经回到出发区。")
        self.completed_tasks.add("tour")

    def task_assistant(self):
        self.speak("好的，我去厨房查看冰箱里的食材。")
        if not self.navigate("kitchen", "已经到达厨房，我开始识别冰箱中的食材。"):
            return
        foods = [name for name in self.detect_objects() if name not in ("phone", "bag")]
        if not foods:
            self.speak("没有稳定识别到食材，请将食材放到摄像头视野内。")
        else:
            selected = foods[:5]
            self.speak("我识别到了{}。".format("、".join(CN_NAMES.get(x, x) for x in selected)))
            self.speak(self.recommend_recipe(foods))
        self.navigate("start", "智慧助手任务完成，已经回到出发区。")
        self.completed_tasks.add("assistant")

    def recommend_recipe(self, foods):
        fallback_dish, fallback_method = self.local_recipe(foods)
        fallback = "推荐制作{}。{}".format(fallback_dish, fallback_method)
        food_names = "、".join(CN_NAMES.get(x, x) for x in foods[:5])
        return self.ask_deepseek(
            "你是居家服务机器人。请根据冰箱识别到的食材推荐一道适合家庭制作的菜，并给出简短做法。回答必须适合语音播报，80字以内，不要分点。",
            "识别到的食材有：{}。请推荐一道菜并说明做法。".format(food_names),
            fallback,
        )

    @staticmethod
    def local_recipe(foods):
        recipes = [
            ({"fanqie", "egg"}, "番茄炒鸡蛋", "鸡蛋炒散盛出，番茄炒出汤汁，再放入鸡蛋，加盐翻炒均匀即可。"),
            ({"qingjiao", "pig"}, "青椒炒肉", "猪肉切片腌制后炒至变色，加入青椒大火翻炒，调味后即可出锅。"),
            ({"qiezi", "pig"}, "肉末茄子", "茄子切条炒软，加入炒香的猪肉末和调味汁，小火焖熟即可。"),
            ({"fish", "doufu"}, "鱼炖豆腐", "鱼煎至两面微黄，加水和豆腐炖煮，最后加盐和葱花调味。"),
            ({"xia", "huanggua"}, "黄瓜炒虾仁", "虾仁炒至变色，加入黄瓜片快速翻炒，少量盐调味即可。"),
            ({"tudou", "qingjiao"}, "青椒土豆丝", "土豆切丝浸水，和青椒丝一起大火快炒，加入盐和少量醋即可。"),
        ]
        food_set = set(foods)
        for required, dish, method in recipes:
            if required.issubset(food_set):
                return dish, method
        cn = [CN_NAMES.get(x, x) for x in foods[:3]]
        return "家常什锦小炒", "将{}清洗切好，按不易熟到易熟的顺序下锅翻炒，加盐调味，炒熟即可。".format("、".join(cn))

    def task_interference(self, command):
        fallback = "根据当前情况，我建议保持轻松舒适，注意安全和天气变化。"
        needs_weather = any(word in command for word in ("天气", "气温", "温度", "穿什么", "穿搭", "穿衣", "衣服"))
        weather_text = self.fetch_shenzhen_weather() if needs_weather else self.weather_fallback_text
        if needs_weather:
            fallback = "{}建议穿短袖或轻薄透气衣物，户外注意防晒，若下雨请带伞。".format(weather_text)
        answer = self.ask_deepseek(
            "你是居家服务机器人。当前用户下达的是比赛干扰命令，只需要原地语音回答，不能引导执行导航、识别、寻物或其他实际任务。若用户询问天气或穿衣，必须基于深圳天气给出天气和穿衣指南。回答必须适合语音播报，80字以内，不要分点。",
            "用户命令：{}。深圳天气：{}。请直接给出对应回答。".format(command, weather_text),
            fallback,
        )
        self.speak(answer)

    def task_find(self, target):
        target_cn = CN_NAMES[target]
        self.speak("好的，我现在去找{}。".format(target_cn))
        found_area = None
        for area in ("restaurant", "kitchen", "livingroom", "bedroom", "corridor"):
            if not self.navigate(area, "已经到达{}，开始识别。".format({
                "restaurant": "餐厅", "kitchen": "厨房", "livingroom": "客厅",
                "bedroom": "卧室", "corridor": "走廊"}[area])):
                continue
            objects = self.detect_objects()
            if target in objects:
                self.speak("找到{}啦，在这里！".format(target_cn))
                found_area = area
                break
            others = [CN_NAMES.get(x, x) for x in objects if x in ("phone", "bag")]
            if others:
                self.speak("我看到了{}，但没有发现{}。".format("、".join(others), target_cn))
            else:
                self.speak("这里什么都没有。")
        if found_area is None:
            self.speak("五个区域已经巡检完毕，没有找到{}。".format(target_cn))
        self.navigate("start", "寻物任务完成，已经回到出发区。")
        self.completed_tasks.add("find_" + target)

    def charge(self):
        self.speak("全部任务完成，现在返回充电桩。")
        try:
            rospy.wait_for_service("auto_charging", timeout=10.0)
            x, y, yaw = self.points["charge"]
            request = SetChargeRequest()
            request.nav = True
            request.track_id = self.charge_track_id
            request.track_dist = self.charge_track_dist
            request.docking_dist = self.charge_docking_dist
            request.pose = Pose2D(x=x, y=y, theta=yaw)
            response = self.charge_service(request)
            if response.success:
                self.speak("充电成功，比赛任务完成。")
                rospy.signal_shutdown("全部服务组任务完成并成功回充")
            else:
                self.speak("自动回充没有成功，请检查充电桩位置。")
                rospy.logerr("自动回充失败：%s", response.message)
        except Exception as exc:
            rospy.logerr("回充服务调用失败：%s", exc)
            self.speak("回充服务异常，请检查底盘摄像头和AR码。")

    def run(self):
        self.pending_manual_command = ""
        if not self.navigate("corridor"):
            raise RuntimeError("无法到达走廊待机点")
        while not rospy.is_shutdown():
            self.wait_for_person()
            command = self.listen()
            task, argument = self.parse_task(command)
            if task == "tour":
                self.task_tour()
            elif task == "assistant":
                self.task_assistant()
            elif task == "interference":
                self.task_interference(argument)
                continue
            elif task == "find":
                self.task_find(argument)
            else:
                self.speak("我没有听懂。你可以让我带你参观、推荐菜品、推荐穿搭、播报天气、寻找手机或寻找书包。")
                self.navigate("corridor")
                continue
            core_done = {"tour", "assistant"}.issubset(self.completed_tasks)
            find_done = bool({"find_phone", "find_bag"} & self.completed_tasks)
            if self.auto_charge_after_all and core_done and find_done:
                self.charge()
                break
            self.navigate("corridor")


if __name__ == "__main__":
    try:
        ServiceGroupRobot().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("服务组主控程序退出：%s", error)
        raise
