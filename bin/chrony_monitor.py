#!/usr/bin/env python
############################################################################
#    Copyright (C) 2009, Willow Garage, Inc.                               #
#    Copyright (C) 2013 by Jerome Maye                                     #
#    jerome.maye@mavt.ethz.ch                                              #
#                                                                          #
#    All rights reserved.                                                  #
#                                                                          #
#    Redistribution and use in source and binary forms, with or without    #
#    modification, are permitted provided that the following conditions    #
#    are met:                                                              #
#                                                                          #
#    1. Redistributions of source code must retain the above copyright     #
#       notice, this list of conditions and the following disclaimer.      #
#                                                                          #
#    2. Redistributions in binary form must reproduce the above copyright  #
#       notice, this list of conditions and the following disclaimer in    #
#       the documentation and/or other materials provided with the         #
#       distribution.                                                      #
#                                                                          #
#    3. The name of the copyright holders may be used to endorse or        #
#       promote products derived from this software without specific       #
#       prior written permission.                                          #
#                                                                          #
#    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   #
#    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     #
#    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     #
#    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        #
#    COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  #
#    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  #
#    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      #
#    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      #
#    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    #
#    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN     #
#    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       #
#    POSSIBILITY OF SUCH DAMAGE.                                           #
############################################################################

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import re
import rospy
import socket
import sys
import time

from subprocess import Popen, PIPE


class ChronyMonitor(object):
    def __init__(self):
        hostname = socket.gethostname()
        diag_hostname = rospy.get_param('~diag_hostname', hostname)

        self.warn_offset = rospy.get_param('~offset_tolerance', 0.005)
        self.error_offset = rospy.get_param('~error_offset_tolerance', 0.1)

        stat = DiagnosticStatus()
        stat.level = 0
        stat.name = "chrony offset from "+ diag_hostname
        stat.message = "OK"
        stat.hardware_id = hostname
        stat.values = []
        self.stat = stat

        self.pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size = 20)

        update_period = rospy.get_param('~update_period', 1.0)
        self.timer = rospy.Timer(rospy.Duration(update_period), self.update)

    def update(self, event):
        try:
            proc = Popen(["chronyc", "tracking"], stdout=PIPE, stdin=PIPE, stderr=PIPE)
            res = proc.wait()
            (proc_output, error_output) = proc.communicate()
        except OSError, (errno, msg):
            if errno == 4:
                return
            else:
                raise

        if res == 0:
            rospy.logdebug(proc_output)
            reference_host = re.search("Reference ID    : (.*)", proc_output).group(1)
            measured_offset = float(re.search("System time     : (.*) seconds slow of NTP time", proc_output).group(1))
            self.stat.level = DiagnosticStatus.OK
            self.stat.message = "OK"
            self.stat.values = [KeyValue("Reference host", reference_host),
                                KeyValue("Offset (us)", str(measured_offset)),
                                KeyValue("Offset tolerance (us)", str(self.warn_offset)),
                                KeyValue("Offset tolerance (us) for Error", str(self.error_offset)) ]

            if (abs(measured_offset) > self.warn_offset):
                self.stat.level = DiagnosticStatus.WARN
                self.stat.message = "chrony Offset Too High"
            if (abs(measured_offset) > self.error_offset):
                self.stat.level = DiagnosticStatus.ERROR
                self.stat.message = "chrony Offset Too High"

        else:
            self.stat.level = DiagnosticStatus.ERROR
            self.stat.message = "Error Running chronyc tracking. Returned %d" % res
            self.stat.values = [KeyValue("Offset (us)", "N/A"),
                                KeyValue("Offset tolerance (us)", str(self.warn_offset)),
                                KeyValue("Offset tolerance (us) for Error", str(self.error_offset)),
                                KeyValue("Output", proc_output),
                                KeyValue("Errors", error_output)]

        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()
        msg.status = [self.stat]
        rospy.loginfo(msg)
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node('chrony_monitor')
    chrony_monitor = ChronyMonitor()
    rospy.spin()
