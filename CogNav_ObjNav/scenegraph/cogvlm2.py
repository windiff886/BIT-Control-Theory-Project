import torch
from PIL import Image
import os
import json
from detection.remote_inference import cogvlm_chat

# 从JSON文件读取数据并转换为字典
# with open('css4_colors.json', 'r', encoding='utf-8') as f:
#     css4_colors_dict = json.load(f)
relationship_candidate=["next to","on","in","hanging on","none"]

def load_model(MODEL_PATH):
    # 检查是否使用远程推理
    use_remote = os.environ.get("USE_REMOTE_INFERENCE", "0") == "1"
    
    if use_remote:
        print("🚀 Using Remote Inference for CogVLM2. Skipping local model load.")
        return None, None
        
    from transformers import AutoModelForCausalLM, AutoTokenizer
    os.environ["TOKENIZERS_PARALLELISM"] = "false"
    
    DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    TORCH_TYPE = torch.bfloat16 if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 8 else torch.float16
    
    tokenizer = AutoTokenizer.from_pretrained(
        MODEL_PATH,
        trust_remote_code=True
    )
    model = AutoModelForCausalLM.from_pretrained(
        MODEL_PATH,
        torch_dtype=TORCH_TYPE,
        trust_remote_code=True,
        low_cpu_mem_usage=True,
    ).eval()
    return tokenizer,model

def find_element_in_string(string, elements):
    for element in elements:
        if element in string:
            return element
    return None

def COG_one(image_path,object_ids,tokenizer,model):
    from vl_prompt.prompt_cog.relationship_one import SYSTEM_PROMPT,USER1,USER2
    
    # 检查是否使用远程推理
    use_remote = os.environ.get("USE_REMOTE_INFERENCE", "0") == "1"
    
    if use_remote:
        # 远程推理模式
        system = SYSTEM_PROMPT 
        image = Image.open(image_path).convert('RGB')
        relationship={}
        
        for pair in object_ids:
            user_prompt = USER1 + str(pair[0]) + USER2 + str(pair[1])
            prompt = system + "\n" + user_prompt # CogVLM2 remote API handles context usually via history or single prompt construction
            # Simplified prompt for chat API
            
            # 使用远程 API
            response = cogvlm_chat(user_prompt, image=image, history=[]) # system prompt might need to be handled differently or prepended
            # Note: cogvlm_chat implementation sends prompt as user message. 
            # If SYSTEM_PROMPT is needed, we should probably prepend it to user_prompt or handle in remote_inference.py
            # For now, let's try prepending it.
            
            full_prompt = f"{system}\n{user_prompt}"
            response = cogvlm_chat(full_prompt, image=image)
            
            if response:
                result = find_element_in_string(response, relationship_candidate)
            else:
                result = None
                
            if result:
                 relationship[pair]=result
            else:
                 relationship[pair]="none"
        return relationship

    # 本地推理模式 (保持原有逻辑)
    DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    # ... origin code ...
    system = SYSTEM_PROMPT 
    image = Image.open(image_path).convert('RGB')
    relationship={}
    for pair in object_ids:
        user_prompt=USER1+str(pair[0]) + USER2 + str(pair[1])
        # user_prompt=USER1+str(object_ids[i+1])+ USER2 + str(object_ids[0])
        query = user_prompt + system
        input_by_model = model.build_conversation_input_ids(
                    tokenizer,
                    query=query,
                    images=[image],
                    template_version='chat'
                )
        inputs = {
                'input_ids': input_by_model['input_ids'].unsqueeze(0).to(DEVICE),
                'token_type_ids': input_by_model['token_type_ids'].unsqueeze(0).to(DEVICE),
                'attention_mask': input_by_model['attention_mask'].unsqueeze(0).to(DEVICE),
                'images': [[input_by_model['images'][0].to(DEVICE).to(TORCH_TYPE)]] if image is not None else None,
            }
        gen_kwargs = {
            "max_new_tokens": 2048,
            "pad_token_id": 128002,
        }
        with torch.no_grad():
            outputs = model.generate(**inputs, **gen_kwargs)
            outputs = outputs[:, inputs['input_ids'].shape[1]:]
            response = tokenizer.decode(outputs[0])
            response = response.split("<|end_of_text|>")[0]
            relation=find_element_in_string(response[response.find('is'):],relationship_candidate)
            if pair[0] in relationship.keys() :
                relationship[pair[0]][pair[1]]=relation
            else :
                rr={pair[1]:relation}
                relationship[pair[0]]=rr
    return relationship

def judgeRoom(image,tokenizer,model):
    from vl_prompt.prompt_cog.roomJudge import SYSTEM_PROMPT,USER
    # from prompt.relationship_multy import SYSTEM_PROMPT,USER1,USER2
    #image = Image.open(image_path).convert('RGB')
    query = SYSTEM_PROMPT
    input_by_model = model.build_conversation_input_ids(
                tokenizer,
                query=query,
                images=[image],
                template_version='chat'
            )
    inputs = {
            'input_ids': input_by_model['input_ids'].unsqueeze(0).to(DEVICE),
            'token_type_ids': input_by_model['token_type_ids'].unsqueeze(0).to(DEVICE),
            'attention_mask': input_by_model['attention_mask'].unsqueeze(0).to(DEVICE),
            'images': [[input_by_model['images'][0].to(DEVICE).to(TORCH_TYPE)]] if image is not None else None,
        }
    gen_kwargs = {
        "max_new_tokens": 2048,
        "pad_token_id": 128002,
    }
    with torch.no_grad():
        outputs = model.generate(**inputs, **gen_kwargs)
        outputs = outputs[:, inputs['input_ids'].shape[1]:]
        response = tokenizer.decode(outputs[0])
        response = response.split("<|end_of_text|>")[0]
    return response
