#!/usr/bin/env python3
import subprocess, time, sys

# 用 ros2 topic echo 持续读 + 管道解析
proc = subprocess.Popen(
    ['ros2', 'topic', 'echo', '/gazebo/model_states'],
    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
cnt = 0
for line in proc.stdout:
    s = line.strip()
    if s == 'linear:':
        cnt += 1
    if cnt == 4 and (s.startswith('x:') or s.startswith('y:') or s.startswith('z:')):
        print(s, flush=True)
    if s == '---':
        cnt = 0
