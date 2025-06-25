#!/bin/bash

ros2 service call /uagentsim/execute_navigation hrstek_msgs/srv/TaskSimplex2dSrv "
task_simplex:
  id: 1234567890
  seq: 0
  type: 1
  height: 0.0
  task_status: 0
  priority: 0
  destination:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    timestamp: 1234567890
    x: -3.85
    y: -10.9
    theta: -1.6243165952542118
    x_dot: 0.0
    y_dot: 0.0
    theta_dot: 0.0
    actions: []
  home:
    header:
      stamp:
        sec: 0
        nanosec: 0
      frame_id: ''
    timestamp: 0
    x: 0.0
    y: 0.0
    theta: 0.0
    x_dot: 0.0
    y_dot: 0.0
    theta_dot: 0.0
    actions: []
  name: 'Web路径任务-web-path-1750751188274'
  task_trajectory:
    id: 0
    current_trajectory_id: 0
    current_trajectory_seq: 0
    controller_type: 0
    destination:
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      timestamp: 0
      x: 0.0
      y: 0.0
      theta: 0.0
      x_dot: 0.0
      y_dot: 0.0
      theta_dot: 0.0
      actions: []
    task_type: 0
    points:
      - header:
          stamp:
            sec: 0
            nanosec: 0
          frame_id: ''
        timestamp: 1234567890
        x: -3.87
        y: -10.98
        theta: 1.5105245881105036
        x_dot: 0.0
        y_dot: 0.0
        theta_dot: 0.0
        actions: []
      - header:
          stamp:
            sec: 0
            nanosec: 0
          frame_id: ''
        timestamp: 1234567890
        x: -3.73
        y: -8.66
        theta: -1.6243165952542118
        x_dot: 0.0
        y_dot: 0.0
        theta_dot: 0.0
        actions:
          - command: 101
            action_id: ''
            timeout: 0.0
            params_wait:
              duration: 1.0
      - header:
          stamp:
            sec: 0
            nanosec: 0
          frame_id: ''
        timestamp: 1234567890
        x: -3.85
        y: -10.9
        theta: -1.6243165952542118
        x_dot: 0.0
        y_dot: 0.0
        theta_dot: 0.0
        actions: []
  servo_trajectory:
    id: 0
    current_trajectory_id: 0
    current_trajectory_seq: 0
    controller_type: 0
    destination:
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      timestamp: 0
      x: 0.0
      y: 0.0
      theta: 0.0
      x_dot: 0.0
      y_dot: 0.0
      theta_dot: 0.0
      actions: []
    task_type: 0
    points: []
  homing_trajectory:
    id: 0
    current_trajectory_id: 0
    current_trajectory_seq: 0
    controller_type: 0
    destination:
      header:
        stamp:
          sec: 0
          nanosec: 0
        frame_id: ''
      timestamp: 0
      x: 0.0
      y: 0.0
      theta: 0.0
      x_dot: 0.0
      y_dot: 0.0
      theta_dot: 0.0
      actions: []
    task_type: 0
    points: []
  trajectory_continue: false
  cached: false
  servo_done: false
  other: '由Fleet-Panel前端生成'
"
