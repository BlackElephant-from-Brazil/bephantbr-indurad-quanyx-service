#!/bin/bash
systemctl stop start-find-objects-ros.service
systemctl stop configuration-boot.service
systemctl start startup-boot.service