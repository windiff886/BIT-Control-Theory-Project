import os
import requests
import base64
import json
import logging
from PIL import Image
import io
import time

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("RemoteInference")

# 获取 API 地址
# 默认回退到单个 INFERENCE_API_URL 以保持后向兼容
DEFAULT_API_URL = os.environ.get("INFERENCE_API_URL", "http://localhost:5000")
LLM_API_URL = os.environ.get("LLM_API_URL", DEFAULT_API_URL)
OPENSEED_API_URL = os.environ.get("OPENSEED_API_URL", DEFAULT_API_URL)

def check_api_health(service_type="llm"):
    """检查远程 API 服务是否健康"""
    url = LLM_API_URL if service_type == "llm" else OPENSEED_API_URL
    try:
        response = requests.get(f"{url}/health", timeout=5)
        if response.status_code == 200:
            logger.info(f"✅ {service_type.upper()} API is healthy: {url}")
            return True
        else:
            logger.warning(f"⚠️ {service_type.upper()} API returned {response.status_code}")
            return False
    except Exception as e:
        logger.error(f"❌ Failed to connect to {service_type.upper()} API at {url}: {e}")
        return False

def encode_image_to_base64(image):
    """将 PIL Image 编码为 base64 字符串"""
    if isinstance(image, str):  # 已经是文件路径
        with open(image, "rb") as f:
            return base64.b64encode(f.read()).decode('utf-8')
    elif isinstance(image, Image.Image):  # PIL Image 对象
        buffered = io.BytesIO()
        image.save(buffered, format="JPEG")
        return base64.b64encode(buffered.getvalue()).decode('utf-8')
    else:
        raise ValueError("Unsupported image type")

def openseed_predict(image, max_retries=3, retry_delay=2):
    """
    调用远程 OpenSeeD 预测接口

    Args:
        image: PIL Image 对象或者图片路径
        max_retries: 最大重试次数
        retry_delay: 重试间隔（秒）

    Returns:
        dict: 包含 mask 和 instances 的结果
    """
    endpoint = f"{OPENSEED_API_URL.rstrip('/')}/openseed/predict"

    for attempt in range(max_retries):
        try:
            img_b64 = encode_image_to_base64(image)

            # 构造请求
            payload = {"image_base64": img_b64}

            # 发送请求
            start_time = time.time()
            response = requests.post(endpoint, json=payload, timeout=30)

            if response.status_code == 200:
                logger.info(f"OpenSeeD inference took {time.time() - start_time:.2f}s")
                return response.json()
            else:
                logger.error(f"OpenSeeD API Error: {response.text}")
                if attempt < max_retries - 1:
                    logger.info(f"Retrying OpenSeeD request in {retry_delay}s... (attempt {attempt + 1}/{max_retries})")
                    time.sleep(retry_delay)
                    continue
                return None

        except Exception as e:
            logger.error(f"OpenSeeD Request Failed: {e}")
            if attempt < max_retries - 1:
                logger.info(f"Retrying OpenSeeD request in {retry_delay}s... (attempt {attempt + 1}/{max_retries})")
                time.sleep(retry_delay)
                continue
            return None

    return None

def cogvlm_chat(prompt, image=None, history=[]):
    """
    调用远程 LLM (CogVLM2) 聊天接口 (OpenAI 兼容格式)
    
    Args:
        prompt (str): 用户输入的文本提示
        image: PIL Image 对象 (可选)
        history (list): 对话历史 (可选)
        
    Returns:
        str: 模型的回复文本
    """
    endpoint = f"{LLM_API_URL.rstrip('/')}/v1/chat/completions"
    
    try:
        messages = []
        
        # 添加历史记录 (如果需要)
        # 注意: 这里简化处理，直接使用 history，实际可能需要格式转换
        # messages.extend(history)
        
        # 构造当前消息
        user_content = []
        user_content.append({"type": "text", "text": prompt})
        
        if image:
            img_b64 = encode_image_to_base64(image)
            user_content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpeg;base64,{img_b64}"
                }
            })
            
        messages.append({"role": "user", "content": user_content})
        
        payload = {
            "model": "cogvlm2",
            "messages": messages,
            "max_tokens": 512
        }
        
        # 发送请求
        start_time = time.time()
        response = requests.post(endpoint, json=payload, timeout=120)
        
        if response.status_code == 200:
            result = response.json()
            content = result['choices'][0]['message']['content']
            logger.info(f"LLM inference took {time.time() - start_time:.2f}s")
            return content
        else:
            logger.error(f"LLM API Error: {response.text}")
            return "Error: Remote API call failed."
            
    except Exception as e:
        logger.error(f"LLM Request Failed: {e}")
        return f"Error: {str(e)}"
