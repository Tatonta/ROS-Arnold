#!/bin/bash

sleep 430

echo "Killing nodes: pointcloud_assembler pointcloud_saver"

rosnode kill /point_cloud_assembler /pointcloud_saver
