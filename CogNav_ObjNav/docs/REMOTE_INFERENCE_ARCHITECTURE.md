# OpenSeeD 远程推理架构详解

本文档详细说明本地 CogNav 仿真与远程 OpenSeeD 推理服务之间的通信机制。

## 架构概览

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              本地电脑 (Local)                                │
│  ┌─────────────────┐    ┌──────────────────┐    ┌────────────────────────┐  │
│  │   Habitat-Sim   │───▶│   CogNav Agent   │───▶│   remote_inference.py  │  │
│  │   (仿真环境)     │    │   (episode.py)   │    │   (HTTP 客户端)         │  │
│  └─────────────────┘    └──────────────────┘    └───────────┬────────────┘  │
│         ▲                        ▲                          │               │
│         │ RGB/Depth              │ 检测结果                  │ HTTP POST     │
│         │ 图像                   │ (mask, bbox, class)       │ (JSON)        │
└─────────┼────────────────────────┼──────────────────────────┼───────────────┘
          │                        │                          │
          │                        │                          ▼
          │                        │              ┌───────────────────────────┐
          │                        │              │   ngrok 隧道 (公网暴露)    │
          │                        │              └───────────┬───────────────┘
          │                        │                          │
          │                        │                          ▼
┌─────────┼────────────────────────┼──────────────────────────┼───────────────┐
│         │                        │              ┌───────────┴───────────┐   │
│         │                        └──────────────│   Flask API 服务器     │   │
│         │                                       │   (端口 5000)          │   │
│         │                                       └───────────┬───────────┘   │
│         │                                                   │               │
│         │                                       ┌───────────┴───────────┐   │
│         │                                       │   OpenSeeD 模型        │   │
│         │                                       │   (GPU 推理)           │   │
│         │                                       └───────────────────────┘   │
│                              Google Colab (Remote)                          │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 数据流详解

### 1. 本地仿真获取图像

```python
# episode.py 中
rgb_image = sim.get_sensor_observations()["rgb"]  # 从 Habitat 获取 RGB 图像
pil_image = Image.fromarray(rgb_image)
```

### 2. 调用 OpenSeeD 检测

当环境变量 `USE_REMOTE_INFERENCE=1` 时：

```python
# openseed.py → detection() 函数
from detection.remote_inference import openseed_predict
results = openseed_predict(pil_image)  # 调用远程 API
```

### 3. 远程推理请求格式

**请求方法**: `POST`
**端点**: `http://<ngrok_url>/openseed/predict`
**Content-Type**: `application/json`

```json
{
    "image_base64": "<Base64 编码的 JPEG 图像>"
}
```

### 4. 远程服务器处理

```python
# Colab 上的 Flask 服务器
@app.route('/openseed/predict', methods=['POST'])
def predict():
    # 1. 解码 Base64 图像
    img_data = base64.b64decode(request.json['image_base64'])
    image = Image.open(io.BytesIO(img_data))
    
    # 2. 预处理
    image_tensor = transform(image).cuda()
    
    # 3. OpenSeeD 推理
    with torch.no_grad():
        outputs = openseed_model.forward([{'image': image_tensor, ...}])
    
    # 4. 提取全景分割结果
    pano_seg = outputs[-1]['panoptic_seg'][0]  # 分割掩码
    pano_seg_info = outputs[-1]['panoptic_seg'][1]  # 实例信息
    
    # 5. 返回 JSON
    return jsonify({
        "mask": pano_seg.cpu().tolist(),
        "instances": [...]
    })
```

### 5. 响应格式

```json
{
    "mask": [[0, 0, 1, 1, ...], ...],   // H x W 的分割掩码
    "instances": [
        {
            "id": 1,
            "category_id": 5,
            "category_name": "chair",
            "bbox": [x1, y1, x2, y2],
            "score": 0.95
        },
        ...
    ]
}
```

### 6. 本地接收并使用结果

```python
# remote_inference.py
def openseed_predict(image):
    response = requests.post(endpoint, json={"image_base64": img_b64})
    return response.json()  # 返回给 openseed.py

# openseed.py
results = openseed_predict(image_ori)
# results 包含 mask, instances 等，供后续 Scene Graph 构建使用
```

## 环境变量配置

| 变量名                 | 说明              | 示例                         |
| ---------------------- | ----------------- | ---------------------------- |
| `USE_REMOTE_INFERENCE` | 是否启用远程推理  | `1`                          |
| `OPENSEED_API_URL`     | OpenSeeD 服务地址 | `http://xxxx.ngrok-free.app` |
| `LLM_API_URL`          | CogVLM2 服务地址  | `http://yyyy.ngrok-free.app` |

## 延迟分析

| 阶段              | 耗时 (估计)             |
| ----------------- | ----------------------- |
| 图像 Base64 编码  | ~10ms                   |
| 网络传输 (上传)   | ~100-500ms (取决于网络) |
| OpenSeeD GPU 推理 | ~200-500ms              |
| 网络传输 (下载)   | ~50-100ms               |
| **总延迟**        | **~400-1200ms/帧**      |

## 错误处理

```python
# remote_inference.py 中的错误处理
try:
    response = requests.post(endpoint, json=payload, timeout=30)
    if response.status_code == 200:
        return response.json()
    else:
        logger.error(f"OpenSeeD API Error: {response.text}")
        return None
except Exception as e:
    logger.error(f"OpenSeeD Request Failed: {e}")
    return None
```

## 调试建议

1. **测试连通性**：
   ```bash
   curl -X GET http://your-ngrok-url/health
   ```

2. **查看本地日志**：
   ```python
   import logging
   logging.getLogger("RemoteInference").setLevel(logging.DEBUG)
   ```

3. **Colab 端查看请求日志**：
   Flask 会自动打印每个请求的 URL 和状态码。
