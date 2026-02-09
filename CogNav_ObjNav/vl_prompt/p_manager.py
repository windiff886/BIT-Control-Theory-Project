import re
from .utils import encode_image_gpt4v
from PIL import Image
import base64
import json
from io import BytesIO
def extract_integer_answer(s):
    match = re.search(r'Answer: \d+', s)
    if match:
        return int(match.group().split(' ')[-1])
    else:
        print('=====> No integer found in string')
        return -1
    
    
def extract_scores(s):
    match = re.search(r'Answer: \[(.*?)\]', s)
    if match:
        scores = [float(x) for x in match.group(1).split(',')]
        return scores.index(max(scores)), scores
    else:
        print('=====> No list found in string')
        return -1, []
    
def extract_objects(s):
    elements = re.findall(r'"([^"]*)"', s)
    return elements
    

def object_query_constructor(objects):
    """
    Construct a query string based on a list of objects

    Args:
        objects: torch.tensor of object indices contained in an area

    Returns:
        str query describing the area, eg "This area contains toilet and sink."
    """
    assert len(objects) > 0
    query_str = "This area contains "
    names = []
    for ob in objects:
        names.append(ob.replace("_", " "))
    if len(names) == 1:
        query_str += names[0]
    elif len(names) == 2:
        query_str += names[0] + " and " + names[1]
    else:
        for name in names[:-1]:
            query_str += name + ", "
        query_str += "and " + names[-1]
    query_str += "."
    return query_str

def get_frontier_prompt(prompt_type):
    if prompt_type == "deterministic":
        from vl_prompt.prompt_gpt.deterministic import \
            SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2
    elif prompt_type == "scoring":
        from vl_prompt.prompt_gpt.scoring import \
            SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2
    else:
        raise NotImplementedError("Froniter prompt type not implemented.")
    
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": USER1},
        {"role": "assistant", "content": ASSISTANT1},
        {"role": "user", "content": USER2},
        {"role": "assistant", "content": ASSISTANT2}
    ]
    
    return messages


def get_candidate_prompt(candidate_type):
    if candidate_type == "open":
        from vl_prompt.prompt_gpt.candidate_open import \
        SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2
    elif candidate_type == "close":
        from vl_prompt.prompt_gpt.candidate_close import \
        SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2
    else:
        raise NotImplementedError("Candidate prompt type not implemented.")
        
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": USER1},
        {"role": "assistant", "content": ASSISTANT1},
        {"role": "user", "content": USER2},
        {"role": "assistant", "content": ASSISTANT2}
    ]
    
    return messages

def get_grouping_prompt():
    from vl_prompt.prompt_gpt.group_obj import \
        SYSTEM_PROMPT, USER1, ASSISTANT1, USER2, ASSISTANT2
        
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT},
        {"role": "user", "content": USER1},
        {"role": "assistant", "content": ASSISTANT1},
        {"role": "user", "content": USER2},
        {"role": "assistant", "content": ASSISTANT2}
    ]
    
    return messages


def get_discover_prompt(img, objects):
    from vl_prompt.prompt_gpt.discover import \
        SYSTEM_PROMPT, USER
        
    img.save("current_for_gpt4.jpg")
    with open("current_for_gpt4.jpg", "rb") as image_file:
        img = base64.b64encode(image_file.read()).decode('utf-8')
    
    question = f"""Current object list: {objects}\n{USER}"""
    payload = {
        "model": "gpt-4-vision-preview", "messages": [{
            "role": "system", "content": SYSTEM_PROMPT
            }, {
            "role": "user", "content": [
                {"type": "text", "text": question}, 
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"}}
            ]
        }], "max_tokens": 300
    }
    return payload
def get_room_prompt(img_name):
    from vl_prompt.prompt_cog.roomJudge import \
        SYSTEM_PROMPT, USER
    with open(img_name, "rb") as image_file:
        img = base64.b64encode(image_file.read()).decode('utf-8')
    # question = f"""Current object list: {objects}\n{USER}"""
    payload = {
        "model": "gpt-4o", "messages": [{
            "role": "system", "content": SYSTEM_PROMPT
            }, {
            "role": "user", "content": [
                {"type": "text", "text": USER}, 
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"}}
            ]
        }], "max_tokens": 300
    }
    return payload
def target_node_prompt(img_name,user):
    from vl_prompt.prompt_gpt.target_object_makesure import \
        SYSTEM_PROMPT
    with open(img_name, "rb") as image_file:
        img = base64.b64encode(image_file.read()).decode('utf-8')
    # question = f"""Current object list: {objects}\n{USER}"""
    payload = {
        "model": "gpt-4o", "messages": [{
            "role": "system", "content": SYSTEM_PROMPT
            }, {
            "role": "user", "content": [
                {"type": "text", "text": user}, 
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{img}"}}
            ]
        }], "max_tokens": 300
    }
    return payload

def query_node_prompt_txt_o1(user):
    from vl_prompt.prompt_gpt.queryNodetxt import \
        SYSTEM_PROMPT
    payload = {
        "model": "o1-preview", "messages": [{
            "role": "system", "content": SYSTEM_PROMPT
            }, {
            "role": "user", "content": [
                {"type": "text", "text": user}
            ]
        }], "max_tokens": 800
    }
    return SYSTEM_PROMPT
def query_node_prompt_txt(user):
    from vl_prompt.prompt_gpt.queryNodetxt import \
        SYSTEM_PROMPT
    # question = f"""Current object list: {objects}\n{USER}"""
    payload = {
        "model": "gpt-4o", "messages": [{
            "role": "system", "content": SYSTEM_PROMPT
            }, {
            "role": "user", "content": [
                {"type": "text", "text": user}
            ]
        }], "max_tokens": 800
    }
    return payload

def get_relationship_prompt(key,value,image):
    from vl_prompt.prompt_gpt.relation import \
        SYSTEM_PROMPT, USER1,USER2
    message=[{"role": "system", "content": SYSTEM_PROMPT}]
    question = f"""{USER1}" "{key}" "{USER2}\t{value}"""
    user_content =[{"type": "text", "text": question},{ "type": "image_url", "image_url": { "url": encode_image_gpt4v(image)} }]
    message.append( {"role": "user", "content": user_content})
    payload = {
        "model": "gpt-4o", "messages": message, "max_tokens": 800
    }
    return payload

def get_relationship2_prompt(bbox,key,value,image_urls):
    from vl_prompt.prompt_gpt.relation import \
        SYSTEM_PROMPT, USER1,USER2
    message=[{"role": "system", "content": SYSTEM_PROMPT}]
    question = f"""{USER1}" "{key}" "{USER2}\t{value}"""
    user_content =[ {  "type": "text", "text": question}]
    for image in image_urls :
        user_content.append({ "type": "image_url", "image_url": { "url": encode_image_gpt4v(image)} } )
    message.append( {"role": "user", "content": user_content})
    payload = {
        "model": "gpt-4o", "messages": message, "max_tokens": 500
    }
    return payload
def get_one_relation_prompt(bbox,list,image_url):
    from vl_prompt.prompt_gpt.relation_one import \
        SYSTEM_PROMPT, USER
    message=[{"role": "system", "content": SYSTEM_PROMPT}]
    question = f"""{USER}" "{list}"""
    user_content =[ {  "type": "text", "text": question},{ "type": "image_url", "image_url": { "url": encode_image_gpt4v(image_url)} }]
    message.append( {"role": "user", "content": user_content})
    payload = {
        "model": "gpt-4-vision-preview", "messages": message, "max_tokens": 300
    }
    return payload

def judge_Correspondence_prompt(bbox,list,image_urls):
    from vl_prompt.prompt_gpt.judgeCorrespondence import \
        SYSTEM_PROMPT, USER
    message=[{"role": "system", "content": SYSTEM_PROMPT}]
    question = f"""{USER}" "{list}"""
    user_content =[ {  "type": "text", "text": question+f"\n{json.dumps(bbox,indent=2)}"  }]
    for image in image_urls :
        user_content.append({ "type": "image_url", "image_url": { "url": encode_image_gpt4v(image)} } )
    message.append( {"role": "user", "content": user_content})
    payload = {
        "model": "gpt-4-vision-preview", "messages": message, "max_tokens": 300
    }
    return payload

def judge_OneObject_prompt(objectlist,objects,image_url):
    from vl_prompt.prompt_gpt.judgeOneobject import \
        SYSTEM_PROMPT, USER
    message=[{"role": "system", "content": SYSTEM_PROMPT}]
    question = f"""{USER}" "{objectlist}" "{objects} """
    user_content =[ {  "type": "text", "text": question},{ "type": "image_url", "image_url": { "url": encode_image_gpt4v(image_url)} }]
    message.append( {"role": "user", "content": user_content})
    payload = {
        "model": "gpt-4-vision-preview", "messages": message, "max_tokens": 300
    }
    return payload
def discover_OneObject_prompt(objectlist,image_url):
    from vl_prompt.prompt_gpt.discoverObject import \
        SYSTEM_PROMPT, USER
    message=[{"role": "system", "content": SYSTEM_PROMPT}]
    question = f"""{USER}" "{objectlist}"""
    user_content =[ {  "type": "text", "text": question},{ "type": "image_url", "image_url": { "url": encode_image_gpt4v(image_url)} }]
    message.append( {"role": "user", "content": user_content})
    payload = {
        "model": "gpt-4-vision-preview", "messages": message, "max_tokens": 300
    }
    return payload
def query_state_transition(user):
    from vl_prompt.prompt_gpt.state_transition import \
        SYSTEM_PROMPT
    payload = {
        "model": "gpt-4o", "messages": [{
            "role": "system", "content": SYSTEM_PROMPT
            }, {
            "role": "user", "content": [
                {"type": "text", "text": user}
            ]
        }], "max_tokens": 800
    }
    return payload