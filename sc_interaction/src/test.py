#!/usr/bin/env python
#-*- coding: utf-8 -*-

import os
import sys
import rospy
import numpy as np
from sc_interaction.msg import ObjectInfo, ObjectInfos

test_info = ObjectInfo()
test_info.id = 777         
print(test_info)