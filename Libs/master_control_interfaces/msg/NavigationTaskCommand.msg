# 总控到航点管理器的有序命令；同一发布者保证clear/追加/返航不跨话题乱序。
string command_id
string command
geometry_msgs/PointStamped[] targets
