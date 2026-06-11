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
from vla_swarm_prompt_router import (
    create_answer,
    load_prompt_specs,
    resolve_text_request,
    structured_error,
)

# --- ROS 参数定义 ---
RESULT_TOPIC = '/scene_graph/llm_ans'
PROMPT_TOPIC = '/scene_graph/prompt'
NODE_NAME = 'LLM_DEEPSEEK_API_NODE'

# --- DeepSeek Anthropic 兼容接口参数 ---
# API key 通过环境变量或私有 ROS 参数提供，不在仓库内保存。
MODEL_TYPE = os.environ.get("SCENE_GRAPH_TEXT_MODEL", "deepseek-v4-flash")
BASE_URL = os.environ.get("SCENE_GRAPH_DEEPSEEK_BASE_URL", "https://api.deepseek.com/anthropic")
API_KEY = os.environ.get("SCENE_GRAPH_DEEPSEEK_API_KEY", "")
MAX_TOKENS = 4096
REQUEST_TIMEOUT = 120

# --- 全局变量定义 ---
result_publisher = None
prompt_queue = queue.Queue(maxsize=100)
prompt_routes = {}


def initialize_llm_client():
    """检查 DeepSeek Anthropic 接口参数是否已经配置。"""
    if not API_KEY:
        logger.error("SCENE_GRAPH_DEEPSEEK_API_KEY is empty.")
        return False
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
        route, route_error = resolve_text_request(prompt_in, prompt_routes)
        if route_error:
            return create_answer(PromptMsg, prompt_in, route_error, rospy.Time.now())

        logger.info("   [ID: {}] Calling DeepSeek in [{}] mode...", prompt_id, route["mode"])
        answer_text = _create_deepseek_message(route["system_prompt"], prompt_in.prompt)

        return create_answer(PromptMsg, prompt_in, answer_text, rospy.Time.now())

    except Exception as exc:
        logger.error("[ID: {}] Error during DeepSeek API call: {}", prompt_id, exc)
        error_message = structured_error("llm_request_failed", str(exc))
        return create_answer(PromptMsg, prompt_in, error_message, rospy.Time.now())


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
        if result_publisher is not None:
            result_publisher.publish(
                create_answer(
                    PromptMsg,
                    message,
                    structured_error("prompt_queue_full", "LLM request queue is full"),
                    rospy.Time.now(),
                )
            )


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

    global MODEL_TYPE, BASE_URL, API_KEY, MAX_TOKENS, REQUEST_TIMEOUT
    global PROMPT_TOPIC, RESULT_TOPIC, prompt_routes
    MODEL_TYPE = rospy.get_param("~text_model", MODEL_TYPE)
    BASE_URL = rospy.get_param("~base_url", BASE_URL)
    API_KEY = rospy.get_param("~api_key", API_KEY)
    MAX_TOKENS = int(rospy.get_param("~max_tokens", MAX_TOKENS))
    REQUEST_TIMEOUT = float(rospy.get_param("~request_timeout", REQUEST_TIMEOUT))
    PROMPT_TOPIC = rospy.get_param("~prompt_topic", PROMPT_TOPIC)
    RESULT_TOPIC = rospy.get_param("~answer_topic", RESULT_TOPIC)

    try:
        rospack = RosPack()
        pkg_path = rospack.get_path('scene_graph')
        prompt_routes = load_prompt_specs(PromptMsg, pkg_path)
        logger.info("Loaded {} SceneGraph prompt routes.", len(prompt_routes))

    except Exception as exc:
        logger.error("Failed to load system prompts: {}", exc)
        return

    if not initialize_llm_client():
        return

    global result_publisher
    result_publisher = rospy.Publisher(RESULT_TOPIC, PromptMsg, queue_size=10)
    rospy.Subscriber(PROMPT_TOPIC, PromptMsg, prompt_callback)

    worker_thread = threading.Thread(target=llm_processing_worker, daemon=True)
    worker_thread.start()

    logger.success("Node is fully running. Waiting for prompts on topic: {}", PROMPT_TOPIC)
    rospy.spin()
    logger.info("Node is shutting down.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
