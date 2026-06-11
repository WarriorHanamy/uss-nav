#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""SceneGraph Prompt 的共享路由与结构化错误工具。"""

import json
import os


PROMPT_SPECS = {
    "PROMPT_TYPE_ROOM_PREDICTION": ("Room Prediction", "prompts_definition/room_prediction_syspt.txt", None),
    "PROMPT_TYPE_EXPL_PREDICTION": ("Area Choose", "prompts_definition/area_choose_syspt.txt", None),
    "PROMPT_TYPE_TERMINATE_OBJ_ID": (
        "Terminate Object ID Choose",
        "prompts_definition/terminate_id_choose_syspt.txt",
        None,
    ),
    "PROMPT_TYPE_DF_DEMO": ("DF Demo", "prompts_definition/df_demo_syspt.txt", None),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION": (
        "VLA Swarm Local Plan",
        "prompts_definition_swarm/local_plan.txt",
        "small_map",
    ),
    "PROMPT_TYPE_TASK_CHAT_PREDICTION": (
        "VLA Swarm Task Chat",
        "prompts_definition_swarm/task_chat.txt",
        "observation_batch",
    ),
    "PROMPT_TYPE_PLACE_PREDICTION": (
        "VLA Swarm Place",
        "prompts_definition_swarm/place_choose.txt",
        None,
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A": (
        "VLA Swarm Local Plan A0",
        "prompts_definition_swarm/local_plana.txt",
        "observation_0",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B": (
        "VLA Swarm Local Plan B0",
        "prompts_definition_swarm/local_planbb.txt",
        "observation_0",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_AA": (
        "VLA Swarm Local Plan AA",
        "prompts_definition_swarm/local_planaa.txt",
        "observation_batch",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A1": (
        "VLA Swarm Local Plan A1",
        "prompts_definition_swarm/local_plana.txt",
        "observation_1",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A2": (
        "VLA Swarm Local Plan A2",
        "prompts_definition_swarm/local_plana.txt",
        "observation_2",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_A3": (
        "VLA Swarm Local Plan A3",
        "prompts_definition_swarm/local_plana.txt",
        "observation_3",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B1": (
        "VLA Swarm Local Plan B1",
        "prompts_definition_swarm/local_planbb.txt",
        "observation_1",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B2": (
        "VLA Swarm Local Plan B2",
        "prompts_definition_swarm/local_planbb.txt",
        "observation_2",
    ),
    "PROMPT_TYPE_LOCAL_PLAN_PREDICTION_B3": (
        "VLA Swarm Local Plan B3",
        "prompts_definition_swarm/local_planbb.txt",
        "observation_3",
    ),
    "PROMPT_TYPE_TASK_ASSIGN_PREDICTION": (
        "VLA Swarm Task Assign",
        "prompts_definition_swarm/task_assign_leader.txt",
        None,
    ),
    "PROMPT_TYPE_TASK_ASSIGN_FOLLOW_PREDICTION": (
        "VLA Swarm Task Assign Follow",
        "prompts_definition_swarm/task_assign_follow.txt",
        None,
    ),
    "PROMPT_TYPE_TASK_OVER_PREDICTION": (
        "VLA Swarm Task Over",
        "prompts_definition_swarm/task_over.txt",
        None,
    ),
}


def load_prompt_specs(prompt_msg_type, package_path):
    """按消息常量建立 prompt_type 到模板的映射，并在启动时完成模板读取。"""
    routes = {}
    for constant_name, (mode_name, relative_path, visual_input) in PROMPT_SPECS.items():
        if not hasattr(prompt_msg_type, constant_name):
            continue
        template_path = os.path.join(package_path, relative_path)
        with open(template_path, "r") as prompt_file:
            system_prompt = prompt_file.read()
        routes[getattr(prompt_msg_type, constant_name)] = {
            "mode": mode_name,
            "system_prompt": system_prompt,
            "visual_input": visual_input,
        }
    return routes


def structured_error(error, detail):
    """LLM 处理端发生错误时仍返回合法 JSON，供 C++ 状态机统一解析。"""
    return json.dumps(
        {
            "success": False,
            "error": str(error),
            "detail": str(detail),
        },
        ensure_ascii=False,
    )


def resolve_text_request(prompt_in, routes):
    """解析纯文本请求；依赖图像的类型由后续 Observation 阶段提供输入。"""
    route = routes.get(prompt_in.prompt_type)
    if route is None:
        return None, structured_error(
            "invalid_prompt_type",
            "Unsupported prompt_type: {}".format(prompt_in.prompt_type),
        )

    visual_input = route["visual_input"]
    if visual_input:
        return None, structured_error(
            "observation_not_ready",
            "{} requires {} from the VLA_Swarm observation pipeline".format(
                route["mode"],
                visual_input,
            ),
        )
    return route, None


def create_answer(prompt_msg_type, prompt_in, answer_text, stamp):
    """构造完整答案，始终回填请求的 ID、类型和 SEND_ANSWER。"""
    answer = prompt_msg_type()
    answer.header.stamp = stamp
    answer.prompt_id = prompt_in.prompt_id
    answer.prompt_type = prompt_in.prompt_type
    answer.option = prompt_msg_type.SEND_ANSWER
    answer.answer = answer_text
    return answer
