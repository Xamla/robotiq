#!/usr/bin/env th

local ros = require 'ros'
local Robotiq2Finger85JointStatePublisher = torch.class('Robotiq2Finger85JointStatePublisher')
local default_values = {
    closed_angle = 46.41,
    max_gap = 0.085,
    ticks = 230,            -- number of ticks that the gripper publishes when it is assuming that it has reached its closed position which was determined during homing
    timeout_in_s = 1
}

local function initializePubSub(self, gripper_topic)
    local gripper_spec = ros.MsgSpec('robotiq_c_model_control/CModel_robot_input')
    self.joint_state_spec = ros.MsgSpec('sensor_msgs/JointState')
    self.joint_state_publisher = self.node_handle:advertise('/joint_states', self.joint_state_spec, 1)
    self.gripper_subscriber = self.node_handle:subscribe(gripper_topic, gripper_spec, 1)
    self.gripper_subscriber:registerCallback(function (msg, header) self:gripper_callback(msg, header) end)
end

function Robotiq2Finger85JointStatePublisher:__init(node_handle, joint_prefix, gripper_topic, correction_in_m)
    self.node_handle = node_handle
    self.joint_prefix = joint_prefix
    self.gripper_topic = gripper_topic
    self.correction_in_m = correction_in_m
    self.seq = 0

    local ok, err = pcall(function() initializePubSub(self, gripper_topic) end)
    if not ok then
        error('Gripper initialization failed: ' .. err)
    end
end

function Robotiq2Finger85JointStatePublisher:gripper_callback(msg, header)
    local distance_per_tick = (default_values.max_gap - self.correction_in_m) / default_values.ticks
    local position = default_values.max_gap - msg.gPO * distance_per_tick
    self.current_angle = default_values.closed_angle - position * (default_values.closed_angle / default_values.max_gap)
    self.last_update_time = ros.Time.now()

    print(msg.gPO, position, self.current_angle)
end

function Robotiq2Finger85JointStatePublisher:publish()
    if (self.last_update_time ~= nil and self.warn_once == nil) then
        if (self.last_update_time:toSec() - ros.Time.now():toSec() > default_values.timeout_in_s) then
            ros.WARN("Gripper %s timed out.", self.gripper_topic)
            self.warn_once = true
            return
        end
    end
    self.warn_once = nil

    if (self.current_angle ~= nil) then
        local m = ros.Message(self.joint_state_spec)
        m.header.seq = self.seq
        m.header.stamp = ros.Time.now()
        m.name = { self.joint_prefix .. 'robotiq_85_finger_right_1_joint' }
        m.position = torch.Tensor{math.rad(self.current_angle )}
        m.effort = torch.Tensor{0}
        m.velocity = torch.Tensor{0}

        self.joint_state_publisher:publish(m)
        self.seq = self.seq + 1
    end
end

function Robotiq2Finger85JointStatePublisher:shutdown()
    if self.joint_state_publisher ~= nil then
        self.joint_state_publisher:shutdown()
    end

    if self.gripper_subscriber ~= nil then
        self.gripper_subscriber:shutdown()
    end
end


local function main()
    local cmd = torch.CmdLine()
    cmd:text()
    cmd:text()
    cmd:text('Robotiq 2-Finger 85 joint state publisher')
    cmd:text()
    cmd:option('-topic', '', 'gripper input topic')
    cmd:option('-correction', 0, 'gripper opening correction')
    cmd:option('-prefix', '', 'joint prefix')
    cmd:option('-ros', '', 'additional ros params')
    local opt = cmd:parse(arg)

    if opt.topic == '' then
        print('No gripper topic was set. Terminating.')
        os.exit()
    end

    print('Start with following parameters:')
    print('topic', opt.topic)
    print('correction', opt.correction)
    print('joint prefix', opt.prefix)

    ros.init('robotiq2finger85', ros.init_options.AnonymousName)
    local node_handle = ros.NodeHandle()
    local joint_state_publisher = Robotiq2Finger85JointStatePublisher.new(node_handle, opt.prefix, opt.topic, opt.correction)

    local rate = ros.Rate(20)
    while ros.ok() do
        rate:sleep()
        ros.spinOnce()
        joint_state_publisher:publish()
        ros.spinOnce()
    end

    joint_state_publisher:shutdown()
    node_handle:shutdown()
    ros.shutdown()
end


main()