相关库和依赖根据编译时的报错补充即可

llm_analyze.cpp:通过API将自然语言指令发给LLM，并发布解析结果
llm_config.yaml:API相关配置和prompt
运行：ros2 run api_invocation llm_analyze --ros-args --params-file src/api_invocation/config/llm_config.yaml

test_publisher.cpp:模拟自然语言发布，可在相应位置编辑问题测试
运行：ros2 run api_invocation test_publisher

plot_point.py:我用来打印曲线轨迹的点，看看对不对，可直接运行

感觉主要效果就是靠prompt，你们运行的时候如果结果不太符合预期可以帮我一起调一下prompt
我现在调用的是deepseek，llm_config.yaml里注释掉的是chatgpt5.2，但这个有时候会timeout，网速不太好