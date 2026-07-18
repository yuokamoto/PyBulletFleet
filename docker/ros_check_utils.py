#!/usr/bin/env python3
"""Shared ROS 2 helper node utilities for Docker integration checks."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node


class RosCheckNode(Node):
    """Small Node base class for polling ROS graph state and services."""

    def spin_until(self, predicate, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def spin_for(self, duration: float) -> None:
        self.spin_until(lambda: False, duration)

    def present_topics(self) -> set[str]:
        return {name for name, _ in self.get_topic_names_and_types()}

    def present_services(self) -> set[str]:
        return {name for name, _ in self.get_service_names_and_types()}

    def call_service(self, client, request, timeout: float):
        service_name = getattr(client, "srv_name", getattr(client, "service_name", "<unknown>"))
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service unavailable: {service_name}")
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result()
        raise RuntimeError(f"service call timed out: {service_name}")
