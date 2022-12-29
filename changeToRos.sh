#!/bin/bash
systemctl stop startup-boot.service
systemctl start configuration-boot.service
systemctl start start-find-objects-ros.service