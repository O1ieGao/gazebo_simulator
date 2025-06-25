# hrstek_common/msg/Action.msg

# 核心动作定义
int32 command             # 动作命令码 (必填)
string action_id           # 动作唯一标识 (可选)
float32 timeout           # 超时时间(秒) (可选, 0表示使用默认)

# -----------------------------------------------------------------
# 参数联合体 (Parameters Union)
# 开发者应根据command的值，来填充和读取对应的参数消息。
# 在任何一个消息实例中，只有一个params_*字段应该被使用。
# -----------------------------------------------------------------
ActionWaitParams     params_wait
# ... 未来可扩展更多动作参数