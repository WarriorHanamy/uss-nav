#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from scene_graph.msg import PromptMsg
from rospkg import RosPack
import os
from openai import OpenAI
import queue
import threading
import sys
import json
from vla_swarm_prompt_router import (
    create_answer,
    load_prompt_specs,
    resolve_text_request,
    structured_error,
)

# 引入 Loguru 的 logger
from loguru import logger

# --- ROS 参数定义 ---
RESULT_TOPIC = '/scene_graph/llm_ans'
PROMPT_TOPIC = '/scene_graph/prompt'
NODE_NAME    = 'LLM_API_NODE'

# --- 大模型 API 参数 ---
MODEL_TYPE = os.environ.get("SCENE_GRAPH_TEXT_MODEL", "qwen3-max")
BASE_URL = os.environ.get(
    "SCENE_GRAPH_OPENAI_BASE_URL",
    "https://dashscope.aliyuncs.com/compatible-mode/v1",
)
API_KEY = os.environ.get("SCENE_GRAPH_OPENAI_API_KEY", "")

# --- 全局变量定义 ---
client = None
result_publisher = None
prompt_queue = queue.Queue(maxsize=100)
prompt_routes = {}

def initialize_llm_client():
    """初始化大模型 API 客户端"""
    global client
    try:
        if not API_KEY:
            logger.error("SCENE_GRAPH_OPENAI_API_KEY is empty.")
            return False
        client = OpenAI(base_url=BASE_URL, api_key=API_KEY)
        logger.success("✅ LLM client initialized successfully.")
        return True
    except Exception as e:
        logger.critical("🔥 Failed to initialize LLM client: {}", e)
        return False

def call_llm_api(prompt_in: PromptMsg) -> PromptMsg:
    """调用大模型 API"""
    prompt_id = prompt_in.prompt_id
    
    if client is None:
        error_msg = structured_error(
            "llm_client_not_initialized",
            "OpenAI-compatible client is not initialized",
        )
        return create_answer(PromptMsg, prompt_in, error_msg, rospy.Time.now())
    
    try:
        route, route_error = resolve_text_request(prompt_in, prompt_routes)
        if route_error:
            return create_answer(PromptMsg, prompt_in, route_error, rospy.Time.now())

        mode_str = route["mode"]
        messages = [
            {"role": "system", "content": route["system_prompt"]},
            {"role": "user", "content": prompt_in.prompt},
        ]

        logger.info(f"   [ID: {prompt_id}] 🤖 Calling LLM in [{mode_str}] mode...")
        completion = client.chat.completions.create(
            model=MODEL_TYPE,
            messages=messages,
            extra_body={"enable_thinking": False}
        )
        
        return create_answer(
            PromptMsg,
            prompt_in,
            completion.choices[0].message.content,
            rospy.Time.now(),
        )

    except Exception as e:
        logger.error("[ID: {}] Error during LLM API call: {}", prompt_id, e)
        error_message = structured_error("llm_request_failed", str(e))
        return create_answer(PromptMsg, prompt_in, error_message, rospy.Time.now())


def prompt_callback(message: PromptMsg):
    """生产者：将 ROS 消息放入队列，并附带追踪ID"""
    if not prompt_queue.full():
        logger.info(f"📥 [ID: {message.prompt_id}] Received message. Added to queue. (Queue size: {prompt_queue.qsize() + 1})")
        prompt_queue.put(message)
    else:
        logger.warning(f"⚠️ [ID: {message.prompt_id}] Queue is full! Discarding message.")
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
    """消费者：从队列中处理任务，所有日志都带追踪ID"""
    logger.info("👷 Worker thread started. Ready for tasks.")
    while not rospy.is_shutdown():
        try:
            message = prompt_queue.get(timeout=1.0)
            prompt_id = message.prompt_id

            logger.warning(f"⚙️ [ID: {prompt_id}] Picked from queue. Starting processing...")
            
            time1 = rospy.Time.now()
            inference_result = call_llm_api(prompt_in=message)
            time2 = rospy.Time.now()
            
            duration = (time2 - time1).to_sec()
            logger.success(f"⏱️ [ID: {prompt_id}] LLM inference finished in {duration:.3f}s")
            
            # --- 核心修改部分 ---
            # 尝试将 answer 字段作为 JSON 解析和格式化
            log_answer_content = ""
            try:
                answer_obj = json.loads(inference_result.answer)
                log_answer_content = json.dumps(answer_obj, indent=2, ensure_ascii=False)
            except json.JSONDecodeError:
                # 如果 answer 不是有效的 JSON，直接使用原始字符串
                log_answer_content = inference_result.answer

            log_prompt_content = ""
            # 尝试将 prompt imput作为 json格式解析
            try:
                if(message.prompt_type == PromptMsg.PROMPT_TYPE_EXPL_PREDICTION):
                    prompt_obj = json.loads(message.prompt)
                    log_prompt_content = json.dumps(prompt_obj, indent=2, ensure_ascii=False)
                else:
                    log_prompt_content = message.prompt
            except json.JSONDecodeError:
                log_prompt_content = message.prompt

            logger.info(f"💡 [ID: {prompt_id}] Prompt Input: \n{log_prompt_content}")
            logger.info(f"💡 [ID: {prompt_id}] Received answer:\n{log_answer_content}")
            
            if result_publisher is not None:
                result_publisher.publish(inference_result)
                logger.success(f"📤 [ID: {prompt_id}] Result published successfully!")
            
            prompt_queue.task_done()
            
        except queue.Empty:
            continue
        except Exception as e:
            logger.error(f"🔥 [Worker Thread] An unexpected error occurred: {e}")

def main():
    # 配置 Loguru logger
    logger.remove()
    logger.add(
        sys.stdout,
        colorize=True,
        format="<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | <cyan>{name}</cyan> - <level>{message}</level>",
        level="INFO"
    )
    
    logger.info("🚀 Starting LLM API Node...")
    rospy.init_node(NODE_NAME, anonymous=True, log_level=rospy.INFO)

    global MODEL_TYPE, BASE_URL, API_KEY, PROMPT_TOPIC, RESULT_TOPIC, prompt_routes
    MODEL_TYPE = rospy.get_param("~text_model", MODEL_TYPE)
    BASE_URL = rospy.get_param("~base_url", BASE_URL)
    API_KEY = rospy.get_param("~api_key", API_KEY)
    PROMPT_TOPIC = rospy.get_param("~prompt_topic", PROMPT_TOPIC)
    RESULT_TOPIC = rospy.get_param("~answer_topic", RESULT_TOPIC)

    try:
        rospack = RosPack()
        pkg_path = rospack.get_path('scene_graph')
        prompt_routes = load_prompt_specs(PromptMsg, pkg_path)
        logger.info("Loaded {} SceneGraph prompt routes.", len(prompt_routes))

    except Exception as e:
        logger.error("Failed to load system prompts: {}", e)
        return

    if not initialize_llm_client():
        return

    global result_publisher
    result_publisher = rospy.Publisher(RESULT_TOPIC, PromptMsg, queue_size=10)
    rospy.Subscriber(PROMPT_TOPIC, PromptMsg, prompt_callback)

    worker_thread = threading.Thread(target=llm_processing_worker, daemon=True)
    worker_thread.start()
    
    logger.success("✅ Node is fully running. Waiting for prompts on topic: {}", PROMPT_TOPIC)
    rospy.spin()
    logger.info("🛑 Node is shutting down.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
