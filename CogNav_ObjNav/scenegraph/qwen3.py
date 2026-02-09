from openai import OpenAI
import json
import os
import requests
import base64
import re
import time
from vl_prompt.p_manager import extract_objects, \
     get_discover_prompt,get_relationship_prompt,\
    judge_Correspondence_prompt,judge_OneObject_prompt,discover_OneObject_prompt,get_room_prompt,query_node_prompt_txt,target_node_prompt,query_state_transition


class LLM():
    def __init__(self):
        self.history = []
        self.use_remote = os.environ.get("USE_REMOTE_INFERENCE", "0") == "1"

        # Qwen API 配置
        # 设置环境变量 QWEN_API_KEY 或 OPENAI_API_KEY
        api_key = os.environ.get("QWEN_API_KEY", os.environ.get("OPENAI_API_KEY", "EMPTY"))
        base_url = os.environ.get("INFERENCE_API_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1")

        # 模型名称：qwen-vl-max (视觉), qwen-plus (纯文本)
        self.model_name = os.environ.get("QWEN_MODEL", "qwen-vl-max")
        self.text_model_name = os.environ.get("QWEN_TEXT_MODEL", "qwen-plus")

        if not self.use_remote:
            self.client = OpenAI(
                api_key=api_key,
                base_url=base_url,
                timeout=60.0,  # 60秒超时
                max_retries=3  # 最多重试3次
            )
    

    def get_room(self, img_name):
        from vl_prompt.prompt_cog.roomJudge import \
        SYSTEM_PROMPT, USER
        with open(img_name, "rb") as image_file:
            img = base64.b64encode(image_file.read()).decode('utf-8')

        if self.use_remote:
            from detection.remote_inference import cogvlm_chat
            try:
                # 构造 prompt
                prompt = f"{SYSTEM_PROMPT}\n{USER}"
                # 注意：CogVLM2 API 可能需要调整 prompt 格式，这里简化处理
                # 如果 img 是路径，直接传路径；这里 img 是 base64 字符串，需要特殊处理
                # remote_inference.cogvlm_chat 期望 PIL Image 或路径
                # 由于这里已经读成了 base64，我们可以暂时不改 remote_inference，
                # 或者直接传 img_name 路径给 cogvlm_chat
                reply = cogvlm_chat(prompt, image=img_name) 
                return reply
            except Exception as e:
                print(f"Remote LLM Error: {e}")
                return ""

        completion = self.client.chat.completions.create(
            model = self.model_name, 
            messages= [{
                "role": "system", "content": SYSTEM_PROMPT
                }, {
                "role": "user", "content": [
                    {"type": "text", "text": USER}, 
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"}}
                ]
            }])
        reply = completion.choices[0].message.content
        return reply
    def query_target_obj(self,img_name,target_name):
        user=f"Is {target_name} in this image ?\n"
        from vl_prompt.prompt_gpt.target_object_makesure import \
            SYSTEM_PROMPT
        with open(img_name, "rb") as image_file:
            img = base64.b64encode(image_file.read()).decode('utf-8')
        if self.use_remote:
            from detection.remote_inference import cogvlm_chat
            try:
                prompt = f"{SYSTEM_PROMPT}\n{user}"
                reply = cogvlm_chat(prompt, image=img_name)
                return reply
            except Exception as e:
                print(f"Remote LLM Error: {e}")
                return ""

        completion = self.client.chat.completions.create(
            model = self.model_name, 
            messages= [{
                "role": "system", "content": SYSTEM_PROMPT
                }, {
                "role": "user", "content": [
                    {"type": "text", "text": user}, 
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"}}
                ]
            }])
        reply = completion.choices[0].message.content
        return reply

    def query_state_transition(self,user):
        from vl_prompt.prompt_gpt.state_transition import SYSTEM_PROMPT
        if self.use_remote:
            from detection.remote_inference import cogvlm_chat
            try:
                # 纯文本请求
                prompt = f"{SYSTEM_PROMPT}\n{user}"
                reply = cogvlm_chat(prompt)
            except Exception as e:
                print(f"Remote LLM Error: {e}")
                reply = "{}"
        else:
            completion = self.client.chat.completions.create(
                model=self.text_model_name,
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user', 'content': user}
                ],
            )
            reply = completion.choices[0].message.content

        # 更健壮的 JSON 解析
        try:
            # 检查是否为错误响应
            if reply.startswith("Error:"):
                print(f"Warning: LLM returned error: {reply[:200]}")
                parsed = {"Transition to state": "Broad Search", "Relative": None}
            else:
                # 尝试找到 JSON 对象
                start = reply.find("{")
                end = reply.rfind("}") + 1  # 使用 rfind 找最后一个 }
                if start != -1 and end > start:
                    json_str = reply[start:end]
                    parsed = json.loads(json_str)
                else:
                    print(f"Warning: No JSON found in LLM reply: {reply[:200]}")
                    parsed = {"Transition to state": "Broad Search", "Relative": None}
        except json.JSONDecodeError as e:
            print(f"Warning: Failed to parse LLM reply as JSON: {e}")
            print(f"Reply was: {reply[:200]}")
            parsed = {"Transition to state": "Broad Search", "Relative": None}

        return parsed.get('Transition to state', 'Broad Search'), parsed.get('Relative', None)

    def query_node_txt(self, user):
        from vl_prompt.prompt_gpt.queryNodetxt import SYSTEM_PROMPT
        if self.use_remote:
            from detection.remote_inference import cogvlm_chat
            try:
                prompt = f"{SYSTEM_PROMPT}\n{user}"
                reply = cogvlm_chat(prompt)
            except Exception as e:
                print(f"Remote LLM Error: {e}")
                reply = "{}"
        else:
            completion = self.client.chat.completions.create(
                model=self.text_model_name,
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user', 'content': user}
                ],
            )
            reply = completion.choices[0].message.content

        # 更健壮的 JSON 解析
        try:
            if reply.startswith("Error:"):
                print(f"Warning: LLM returned error: {reply[:200]}")
                parsed = {"result": 0}
            else:
                start = reply.find("{")
                end = reply.rfind("}") + 1
                if start != -1 and end > start:
                    json_str = reply[start:end]
                    parsed = json.loads(json_str)
                else:
                    print(f"Warning: No JSON found in LLM reply: {reply[:200]}")
                    parsed = {"result": 0}
        except json.JSONDecodeError as e:
            print(f"Warning: Failed to parse LLM reply as JSON: {e}")
            print(f"Reply was: {reply[:200]}")
            parsed = {"result": 0}

        if 'result' not in parsed:
            parsed['result'] = 0
        if isinstance(parsed['result'], int):
            result = parsed['result']
        else:
            nums = re.findall(r"\d+", str(parsed['result']))
            result = int(nums[0]) if nums else 0
        return parsed, result
    

    def get_relationship_prompt(self,obj1,obj2,image):
        from vl_prompt.prompt_gpt.relation import \
            SYSTEM_PROMPT, USER1,USER2
        question = f"""{USER1}" "{obj1}" "{USER2}\t{obj2}"""
        with open(image, "rb") as image_file:
            img = base64.b64encode(image_file.read()).decode('utf-8')
        if self.use_remote:
            from detection.remote_inference import cogvlm_chat
            try:
                prompt = f"{SYSTEM_PROMPT}\n{question}"
                reply = cogvlm_chat(prompt, image=image)
            except Exception as e:
                print(f"Remote LLM Error: {e}")
                reply = ""
        else:
            user_content =[{"type": "text", "text": question},{ "type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"} }]
            completion = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {'role': 'system', 'content': SYSTEM_PROMPT},
                    {'role': 'user', 'content': user_content}
                ],
            )
            reply = completion.choices[0].message.content
        # print("reply:",reply)
        return reply
   