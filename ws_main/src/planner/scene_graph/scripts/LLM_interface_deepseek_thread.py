#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import queue
import sys
import threading
import urllib.error
import urllib.request

import rospy
from loguru import logger
from rospkg import RosPack
from scene_graph.msg import PromptMsg

# --- ROS 参数定义 ---
RESULT_TOPIC = '/scene_graph/llm_ans'
PROMPT_TOPIC = '/scene_graph/prompt'
NODE_NAME = 'LLM_DEEPSEEK_API_NODE'

# --- DeepSeek Anthropic 兼容接口参数 ---
# API_KEY 请在本机使用前手动填写；不要把真实密钥提交到仓库。
MODEL_TYPE = "deepseek-v4-flash"
BASE_URL = "https://api.deepseek.com/anthropic"
API_KEY = "sk-2db36e3a9bc24c958f45699b9febd3a9"
MAX_TOKENS = 4096
REQUEST_TIMEOUT = 120

SYSTEM_PROMPT_AREA_PREDICT = None
SYSTEM_PROMPT_AREA_CHOOSE = None
SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE = None
SYSTEM_PROMPT_DF_DEMO = None

# --- 全局变量定义 ---
result_publisher = None
prompt_queue = queue.Queue(maxsize=100)


def initialize_llm_client():
    """检查 DeepSeek Anthropic 接口参数是否已经配置。"""
    if not API_KEY:
        logger.warning("DeepSeek API key is empty. Please fill API_KEY before running this node.")
    logger.success("DeepSeek Anthropic API endpoint configured: {}", BASE_URL)
    return True


def _extract_text_from_anthropic_response(response_obj):
    """从 Anthropic messages 响应中提取文本结果。"""
    content_items = response_obj.get("content", [])
    text_parts = []

    for item in content_items:
        if isinstance(item, dict) and item.get("type") == "text":
            text_parts.append(item.get("text", ""))

    return "\n".join(text_parts).strip()


def _create_deepseek_message(system_prompt, user_prompt):
    """按照 DeepSeek Anthropic 兼容格式发送 messages 请求。"""
    endpoint = BASE_URL.rstrip("/") + "/v1/messages"
    payload = {
        "model": MODEL_TYPE,
        "max_tokens": MAX_TOKENS,
        "system": system_prompt,
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": user_prompt,
                    }
                ],
            }
        ],
    }

    request = urllib.request.Request(
        endpoint,
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Content-Type": "application/json",
            "x-api-key": API_KEY,
            "anthropic-version": "2023-06-01",
        },
        method="POST",
    )

    try:
        with urllib.request.urlopen(request, timeout=REQUEST_TIMEOUT) as response:
            response_body = response.read().decode("utf-8")
    except urllib.error.HTTPError as exc:
        error_body = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError("DeepSeek HTTP {}: {}".format(exc.code, error_body))
    except urllib.error.URLError as exc:
        raise RuntimeError("DeepSeek request failed: {}".format(exc.reason))

    response_obj = json.loads(response_body)
    answer_text = _extract_text_from_anthropic_response(response_obj)
    if not answer_text:
        raise RuntimeError("DeepSeek response does not contain text content: {}".format(response_body))

    return answer_text


def call_llm_api(prompt_in: PromptMsg) -> PromptMsg:
    """根据 PromptMsg 类型选择系统提示词，并调用 DeepSeek Anthropic 兼容接口。"""
    prompt_id = prompt_in.prompt_id

    try:
        mode_str = ""
        system_prompt = None

        if prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_ROOM_PREDICTION:
            mode_str = "Room Prediction"
            system_prompt = SYSTEM_PROMPT_AREA_PREDICT
        elif prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_EXPL_PREDICTION:
            mode_str = "Area Choose"
            system_prompt = SYSTEM_PROMPT_AREA_CHOOSE
        elif prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_TERMINATE_OBJ_ID:
            mode_str = "Terminate Object ID Choose"
            system_prompt = SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE
        elif prompt_in.prompt_type == PromptMsg.PROMPT_TYPE_DF_DEMO:
            mode_str = "DF Demo"
            system_prompt = SYSTEM_PROMPT_DF_DEMO
        else:
            raise ValueError("Invalid prompt_type: {}".format(prompt_in.prompt_type))

        logger.info("   [ID: {}] Calling DeepSeek in [{}] mode...", prompt_id, mode_str)
        answer_text = _create_deepseek_message(system_prompt, prompt_in.prompt)

        llm_ans = PromptMsg()
        llm_ans.header.stamp = rospy.Time.now()
        llm_ans.answer = answer_text
        llm_ans.prompt_id = prompt_id
        llm_ans.option = PromptMsg.SEND_ANSWER
        return llm_ans

    except Exception as exc:
        error_message = "[ID: {}] Error during DeepSeek API call: {}".format(prompt_id, exc)
        logger.error(error_message)
        err_ans = PromptMsg()
        err_ans.answer = error_message
        err_ans.prompt_id = prompt_id
        return err_ans


def prompt_callback(message: PromptMsg):
    """生产者：将 ROS prompt 消息放入队列。"""
    if not prompt_queue.full():
        logger.info(
            "[ID: {}] Received message. Added to queue. (Queue size: {})",
            message.prompt_id,
            prompt_queue.qsize() + 1,
        )
        prompt_queue.put(message)
    else:
        logger.warning("[ID: {}] Queue is full. Discarding message.", message.prompt_id)


def llm_processing_worker():
    """消费者：按队列顺序处理 DeepSeek 请求并发布结果。"""
    logger.info("Worker thread started. Ready for DeepSeek tasks.")
    while not rospy.is_shutdown():
        try:
            message = prompt_queue.get(timeout=1.0)
            prompt_id = message.prompt_id

            logger.warning("[ID: {}] Picked from queue. Starting processing...", prompt_id)

            time1 = rospy.Time.now()
            inference_result = call_llm_api(prompt_in=message)
            time2 = rospy.Time.now()

            duration = (time2 - time1).to_sec()
            logger.success("[ID: {}] DeepSeek inference finished in {:.3f}s", prompt_id, duration)

            log_answer_content = ""
            try:
                answer_obj = json.loads(inference_result.answer)
                log_answer_content = json.dumps(answer_obj, indent=2, ensure_ascii=False)
            except json.JSONDecodeError:
                # 如果 answer 不是 JSON 字符串，直接按普通文本记录。
                log_answer_content = inference_result.answer

            log_prompt_content = ""
            try:
                if message.prompt_type == PromptMsg.PROMPT_TYPE_EXPL_PREDICTION:
                    prompt_obj = json.loads(message.prompt)
                    log_prompt_content = json.dumps(prompt_obj, indent=2, ensure_ascii=False)
                else:
                    log_prompt_content = message.prompt
            except json.JSONDecodeError:
                # 如果 prompt 不是 JSON 字符串，直接按普通文本记录。
                log_prompt_content = message.prompt

            logger.info("[ID: {}] Prompt Input:\n{}", prompt_id, log_prompt_content)
            logger.info("[ID: {}] Received answer:\n{}", prompt_id, log_answer_content)

            if result_publisher is not None:
                result_publisher.publish(inference_result)
                logger.success("[ID: {}] Result published successfully.", prompt_id)

            prompt_queue.task_done()

        except queue.Empty:
            continue
        except Exception as exc:
            logger.error("[Worker Thread] An unexpected error occurred: {}", exc)


def main():
    """初始化 ROS 节点、加载提示词，并启动 DeepSeek 请求工作线程。"""
    logger.remove()
    logger.add(
        sys.stdout,
        colorize=True,
        format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> - <level>{message}</level>",
        level="INFO",
    )

    logger.info("Starting DeepSeek LLM API Node...")
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    if not initialize_llm_client():
        return

    worker_thread = threading.Thread(target=llm_processing_worker, daemon=True)
    worker_thread.start()

    global SYSTEM_PROMPT_AREA_PREDICT, SYSTEM_PROMPT_AREA_CHOOSE, SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE, SYSTEM_PROMPT_DF_DEMO
    try:
        rospack = RosPack()
        pkg_path = rospack.get_path('scene_graph')
        area_predict_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'room_prediction_syspt.txt')
        area_choosen_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'area_choose_syspt.txt')
        terminate_obj_id_choose_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'terminate_id_choose_syspt.txt')
        df_demo_prompt_def_path = os.path.join(pkg_path, 'prompts_definition', 'df_demo_syspt.txt')

        with open(area_predict_prompt_def_path, 'r') as prompt_file:
            SYSTEM_PROMPT_AREA_PREDICT = prompt_file.read()
        logger.info("System Prompt for [Area Prediction] loaded.")

        with open(area_choosen_prompt_def_path, 'r') as prompt_file:
            SYSTEM_PROMPT_AREA_CHOOSE = prompt_file.read()
        logger.info("System Prompt for [Area Choose] loaded.")

        with open(terminate_obj_id_choose_prompt_def_path, 'r') as prompt_file:
            SYSTEM_PROMPT_TERMINATE_OBJ_CHOOSE = prompt_file.read()
        logger.info("System Prompt for [Terminate Object ID Choose] loaded.")

        with open(df_demo_prompt_def_path, 'r') as prompt_file:
            SYSTEM_PROMPT_DF_DEMO = prompt_file.read()
        logger.info("System Prompt for [DF Demo] loaded.")

    except Exception as exc:
        logger.error("Failed to load system prompts: {}", exc)

    global result_publisher
    result_publisher = rospy.Publisher(RESULT_TOPIC, PromptMsg, queue_size=10)
    rospy.Subscriber(PROMPT_TOPIC, PromptMsg, prompt_callback)

    logger.success("Node is fully running. Waiting for prompts on topic: {}", PROMPT_TOPIC)
    rospy.spin()
    logger.info("Node is shutting down.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
