#!/usr/bin/env python
import rospy, rostopic 
from perception_unit.msg import Vel
import threading
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.animation as animation
from resource_unit.msg import Latency
from std_msgs.msg import String

# Class to computes the topic statistics
class ROSTopicHz():
    def __init__(self, window_size,filter_expr=None, use_wtime=False):
        
        self.lock = threading.Lock() 
        self.last_printed_tn = 0 
        self.msg_t0 = -1. 
        self.msg_tn = 0 
        self.times =[] 
        self.filter_expr = filter_expr 
        self.use_wtime = use_wtime 
            
        # can't have infinite window size due to memory restrictions 
        if window_size < 0: 
            window_size = 50000 
        self.window_size = window_size

    def callback_hz(self, m): 
        """ 
        ros sub callback 
        :param m: Message instance 
        """ 
        # #694: ignore messages that don't match filter 
        if self.filter_expr is not None and not self.filter_expr(m): 
           return 
        with self.lock:
            curr_rostime = rospy.get_rostime() if not self.use_wtime else rospy.Time.from_sec(time.time())

             # time reset 
            if curr_rostime.is_zero(): 
                if len(self.times) > 0: 
                    rospy.logwarn("time has reset, resetting counters") 
                    self.times = [] 
                return
            curr = curr_rostime.to_sec() if not self.use_wtime else rospy.Time.from_sec(time.time()).to_sec()

            if self.msg_t0 < 0 or self.msg_t0 > curr: 
                self.msg_t0 = curr 
                self.msg_tn = curr 
                self.times = [] 
            else: 
                self.times.append(curr - self.msg_tn) 
                self.msg_tn = curr
            #only keep statistics for the last 10000 messages so as not to run out of memory 
            if len(self.times) > self.window_size - 1: 
                self.times.pop(0)

    def print_hz(self):
        """ 
        print the average publishing rate to screen 
        """ 
        if not self.times: 
            return {"rate": 0, "std_dev": 0, "window": 0}
        elif self.msg_tn == self.last_printed_tn:
            # rospy.logerr("No new messages")
            return {"rate": 0, "std_dev": 0, "window": 0} 
        with self.lock:
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1./mean if mean > 0. else 0 
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.times) /n) 

            max_delta = max(self.times)
            min_delta = min(self.times)

            self.last_printed_tn = self.msg_tn
        result = {"rate": rate, "std_dev": std_dev, "window": (n+1)}
        return result
        print("average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(rate, min_delta, max_delta, std_dev, n+1))

# Class to monitor a topic and forwards the stats 
class TopicMonitor():
    def __init__(self, topic_name):
        rospy.logwarn("initializing topic_monitor")
        self.rate = ROSTopicHz(-1)
        self.topic_name = topic_name
        self.dict_rate = {"rate": 0, "std_dev": 0, "window": 0} 
        rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.rate.callback_hz)
        self.pub_topic_monitor = rospy.Publisher('/topic_monitor', Vel, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.pub_monitor)

    def pub_monitor(self, event):
        self.dict_rate = self.rate.print_hz()
        # rospy.logwarn(self.dict_rate)

    def getRate(self):
        return self.dict_rate


# /detection/lidar_detector/objects
# /detection/shape_estimation/objects
# Topics that monitors have to follow
topic_name1 = "detection/lidar_detector/objects"
topic_name2 = "detection/shape_estimation/objects"
topic_name3 = "obj_detection_node/detected_objects"
topic_name4 = "ekf/odometry"
topic_name5 = "rf2o/odometry"

# Init node and init monitoring classes for each topic
rospy.init_node('topic_monitor')
class_TM1 = TopicMonitor(topic_name1)
class_TM2 = TopicMonitor(topic_name2)
class_TM3 = TopicMonitor(topic_name3)
class_TM4 = TopicMonitor(topic_name4)
class_TM5 = TopicMonitor(topic_name5)


# Class to draw the GUI to show the plots 
class SubplotAnimation(animation.TimedAnimation):
    def __init__(self):
        fig = plt.figure()
        # axis
        self.ax1 = fig.add_subplot(5, 2, 1)
        self.ax2 = fig.add_subplot(5, 2, 3)
        self.ax3 = fig.add_subplot(5, 2, 5)
        self.ax4 = fig.add_subplot(5, 2, 7)
        self.ax5 = fig.add_subplot(5, 2, 9)

        self.ax6 = fig.add_subplot(5, 2, 2)
        self.ax7 = fig.add_subplot(5, 2, 4)
        self.ax8 = fig.add_subplot(5, 2, 6)
        self.ax9 = fig.add_subplot(5, 2, 8)
        self.ax10 = fig.add_subplot(5, 2, 10)

        self.t = np.linspace(0, 80, 400)
        self.x = np.cos(2 * np.pi * self.t / 10.)
        self.y = np.sin(2 * np.pi * self.t / 10.)
        self.z = 10 * self.t

        # Data lists
        self.xdata1 = []
        self.xdata2 = []
        self.xdata3 = []
        self.xdata4 = []
        self.xdata5 = []

        self.xdata6  = []
        self.xdata7  = []
        self.xdata8  = []
        self.xdata9  = []
        self.xdata10 = []
        
        self.ydata1 = []
        self.ydata2 = []
        self.ydata3 = []
        self.ydata4 = []
        self.ydata5 = []

        self.ydata6  = []
        self.ydata7  = []
        self.ydata8  = []
        self.ydata9  = []
        self.ydata10 = []

        # Subscribe the latency monitors
        rospy.Subscriber('/latency_monitor_' + topic_name1, Latency, self.callback_lat1)
        rospy.Subscriber('/latency_monitor_' + topic_name2, Latency, self.callback_lat2)
        rospy.Subscriber('/latency_monitor_' + topic_name3, Latency, self.callback_lat3)
        rospy.Subscriber('/latency_monitor_' + topic_name4, Latency, self.callback_lat4)
        rospy.Subscriber('/latency_monitor_' + topic_name5, Latency, self.callback_lat5)
        self.pub_perception_monitor = rospy.Publisher('/perception_timing', String, queue_size=10)
        self.pub_localization_monitor = rospy.Publisher('/localization_timing', String, queue_size=10)

        # Set the plots with title and line drawers
        self.ax1.set_ylabel('Hz')
        self.ax1.set_title(topic_name1)
        self.line1 = Line2D([], [], color='black')
        self.ax1.add_line(self.line1)
        self.ax1.set_xlim(0, 2)
        self.ax1.set_ylim(-1, 12)

        self.ax2.set_ylabel('Hz')
        self.ax2.set_title(topic_name2)
        self.line2 = Line2D([], [], color='black')
        self.ax2.add_line(self.line2)
        self.ax2.set_xlim(0, 2)
        self.ax2.set_ylim(-1, 12)

        self.ax3.set_ylabel('Hz')
        self.ax3.set_title(topic_name3)
        self.line3 = Line2D([], [], color='black')
        self.ax3.add_line(self.line3)
        self.ax3.set_xlim(0, 2)
        self.ax3.set_ylim(-1, 2)

        # self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Hz')
        self.ax4.set_title(topic_name4)
        self.line4 = Line2D([], [], color='black')
        self.ax4.add_line(self.line4)
        self.ax4.set_xlim(0, 2)
        self.ax4.set_ylim(-1, 2)

        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('Hz')
        self.ax5.set_title(topic_name5)
        self.line5 = Line2D([], [], color='black')
        self.ax5.add_line(self.line5)
        self.ax5.set_xlim(0, 2)
        self.ax5.set_ylim(-1, 2)

        # self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('Latency (s)')
        self.ax6.set_title('Latency ' + topic_name1)
        self.line6 = Line2D([], [], color='black')
        self.ax6.add_line(self.line6)
        self.ax6.set_xlim(0, 2)
        self.ax6.set_ylim(-1, 2)

        # self.ax7.set_xlabel('Time (s)')
        self.ax7.set_ylabel('Latency (s)')
        self.ax7.set_title('Latency ' + topic_name2)
        self.line7 = Line2D([], [], color='black')
        self.ax7.add_line(self.line7)
        self.ax7.set_xlim(0, 2)
        self.ax7.set_ylim(-1, 2)

        # self.ax8.set_xlabel('Time (s)')
        self.ax8.set_ylabel('Latency (ms)')
        self.ax8.set_title('Latency ' + topic_name3)
        self.line8 = Line2D([], [], color='black')
        self.ax8.add_line(self.line8)
        self.ax8.set_xlim(0, 2)
        self.ax8.set_ylim(-1, 2)

        # self.ax9.set_xlabel('Time (s)')
        self.ax9.set_ylabel('Latency (s)')
        self.ax9.set_title('Latency ' + topic_name4)
        self.line9 = Line2D([], [], color='black')
        self.ax9.add_line(self.line9)
        self.ax9.set_xlim(0, 2)
        self.ax9.set_ylim(-1, 2)

        self.ax10.set_xlabel('Time (s)')
        self.ax10.set_ylabel('Latency (s)')
        self.ax10.set_title('Latency ' + topic_name5)
        self.line10 = Line2D([], [], color='black')
        self.ax10.add_line(self.line10)
        self.ax10.set_xlim(0, 2)
        self.ax10.set_ylim(-1, 2)

        # Remove the x axis labels
        self.ax1.set_xticklabels([])
        self.ax2.set_xticklabels([])
        self.ax3.set_xticklabels([])
        self.ax4.set_xticklabels([])
        self.ax5.set_xticklabels([])
        self.ax6.set_xticklabels([])
        self.ax7.set_xticklabels([])
        self.ax8.set_xticklabels([])
        self.ax9.set_xticklabels([])
        self.ax10.set_xticklabels([])

        animation.TimedAnimation.__init__(self, fig, interval=50, blit=True)

    # Callback for latency
    def callback_lat1(self, msg):
        self.ydata6.append(msg.latency)
        self.xdata6.append(msg.header.seq)
        if len(self.ydata6) > 20:
            self.ydata6.pop(0)
            self.xdata6.pop(0)

    # Callback for latency
    def callback_lat2(self, msg):
        self.ydata7.append(msg.latency)
        self.xdata7.append(msg.header.seq)
        if len(self.ydata7) > 20:
            self.ydata7.pop(0)
            self.xdata7.pop(0)

    # Callback for latency
    def callback_lat3(self, msg):
        self.ydata8.append(msg.latency*1e3)
        self.xdata8.append(msg.header.seq)
        if len(self.ydata8) > 20:
            self.ydata8.pop(0)
            self.xdata8.pop(0)

    # Callback for latency
    def callback_lat4(self, msg):
        self.ydata9.append(msg.latency)
        self.xdata9.append(msg.header.seq)
        if len(self.ydata9) > 20:
            self.ydata9.pop(0)
            self.xdata9.pop(0)

    # Callback for latency
    def callback_lat5(self, msg):
        self.ydata10.append(msg.latency)
        self.xdata10.append(msg.header.seq)
        if len(self.ydata10) > 20:
            self.ydata10.pop(0)
            self.xdata10.pop(0)

    # Draw the frame
    def _draw_frame(self, framedata):
        i = framedata
        head = i - 1
        head_slice = (self.t > self.t[i] - 1.0) & (self.t < self.t[i])

        # Append x data
        self.xdata1.append(i)
        self.xdata2.append(i)
        self.xdata3.append(i)
        self.xdata4.append(i)
        self.xdata5.append(i)

        # Get the stats for each topic
        rate1 = math.fabs(class_TM1.getRate()['rate'])
        std_dev1 = class_TM1.getRate()['std_dev']

        rate2 = math.fabs(class_TM2.getRate()['rate'])
        std_dev2 = class_TM2.getRate()['std_dev']

        rate3 = math.fabs(class_TM3.getRate()['rate'])
        std_dev3 = class_TM3.getRate()['std_dev']

        rate4 = math.fabs(class_TM4.getRate()['rate'])
        std_dev4 = class_TM4.getRate()['std_dev']

        rate5 = math.fabs(class_TM5.getRate()['rate'])
        std_dev5 = class_TM5.getRate()['std_dev']

        # Check if the rate1 is in the right range
        # if not send the warning
        if math.fabs(rate1) < 9.8:
            self.line1.set_color(color='red')
            data_to_send = String()
            data_to_send.data = "Time violation in topic " + topic_name1 + ", frequency outside the designed range: " + str(rate1) + " Hz != 10 Hz"
            rospy.logerr(data_to_send.data)
            self.pub_perception_monitor.publish(data_to_send)
        else:
            self.line1.set_color(color='black')
            rospy.loginfo("Frequency from " + topic_name1 + ' with rate: ' + str(rate1) + ' Hz and std_dev: ' + str(std_dev1) + ' s')

        # Check if the rate2 is in the right range
        # if not send the warning
        if math.fabs(rate2) < 9.8:
            self.line2.set_color(color='red')
            data_to_send = String()
            data_to_send.data = "Time violation in topic " + topic_name2 + ", frequency outside the designed range: " + str(rate2) + " Hz != 10 Hz"
            rospy.logerr(data_to_send.data)
            self.pub_perception_monitor.publish(data_to_send)
        else:
            self.line2.set_color(color='black')
            rospy.loginfo("Frequency from " + topic_name2 + ' with rate: ' + str(rate2) + ' Hz and std_dev: ' + str(std_dev2) + ' s')

        # Check if the rate3 is in the right range
        # if not send the warning
        if math.fabs(rate3) < 0.9:
            self.line3.set_color(color='red')
            data_to_send = String()
            data_to_send.data = "Time violation in topic " + topic_name3 + ", frequency outside the designed range: " + str(rate3) + " Hz != 1 Hz"
            rospy.logerr(data_to_send.data)
            self.pub_perception_monitor.publish(data_to_send)
        else:
            self.line3.set_color(color='black')
            rospy.loginfo("Frequency from " + topic_name3 + ' with rate: ' + str(rate3) + ' Hz and std_dev: ' + str(std_dev3) + ' s')

        # Check if the rate4 is in the right range
        # if not send the warning
        if math.fabs(rate4) < 9.8:
            self.line4.set_color(color='red')
            data_to_send = String()
            data_to_send.data = "Time violation in topic " + topic_name4 + ", frequency outside the designed range: " + str(rate4) + " Hz != 10 Hz"
            rospy.logerr(data_to_send.data)
            self.pub_localization_monitor.publish(data_to_send)
        else:
            self.line4.set_color(color='black')
            rospy.loginfo("Frequency from " + topic_name4 + ' with rate: ' + str(rate4) + ' Hz and std_dev: ' + str(std_dev4) + ' s')

        # Check if the rate5 is in the right range
        # if not send the warning
        if math.fabs(rate5) < 9.8:
            self.line5.set_color(color='red')
            data_to_send = String()
            data_to_send.data = "Time violation in topic " + topic_name5 + ", frequency outside the designed range: " + str(rate5) + " Hz != 10 Hz"
            rospy.logerr(data_to_send.data)
            self.pub_localization_monitor.publish(data_to_send)
        else:
            self.line5.set_color(color='black')
            rospy.loginfo("Frequency from " + topic_name5 + ' with rate: ' + str(rate5) + ' Hz and std_dev: ' + str(std_dev5) + ' s')

        # Append data to Y axis
        self.ydata1.append(rate1)
        self.ydata2.append(rate2)
        self.ydata3.append(rate3)
        self.ydata4.append(rate4)
        self.ydata5.append(rate5)

        # Just show the last 20 samples
        if len(self.ydata1) > 20:
            self.ydata1.pop(0)
            self.xdata1.pop(0)

            self.ydata2.pop(0)
            self.xdata2.pop(0)

            self.ydata3.pop(0)
            self.xdata3.pop(0)

            self.ydata4.pop(0)
            self.xdata4.pop(0)

            self.ydata5.pop(0)
            self.xdata5.pop(0)

        # Update x limits
        self.ax1.set_xlim(self.xdata1[0], self.xdata1[-1])
        self.ax2.set_xlim(self.xdata2[0], self.xdata2[-1])
        self.ax3.set_xlim(self.xdata3[0], self.xdata3[-1])
        self.ax4.set_xlim(self.xdata4[0], self.xdata4[-1])
        self.ax5.set_xlim(self.xdata5[0], self.xdata5[-1])

        # Update y limits
        self.ax1.set_ylim(rate1-1, rate1+1)
        self.ax2.set_ylim(rate2-1, rate2+1)
        self.ax3.set_ylim(rate3-1, rate3+1)
        self.ax4.set_ylim(rate4-1, rate4+1)
        self.ax5.set_ylim(rate5-1, rate5+1)

        # Update x and y plots
        self.line1.set_data(self.xdata1, self.ydata1)
        self.line2.set_data(self.xdata2, self.ydata2)
        self.line3.set_data(self.xdata3, self.ydata3)
        self.line4.set_data(self.xdata4, self.ydata4)
        self.line5.set_data(self.xdata5, self.ydata5)

        # Update the latency plots
        try:
            self.ax6.set_xlim(self.ydata6[0], self.ydata6[-1])
            self.ax6.set_ylim(self.ydata6[-1]-.25, self.ydata6[-1]+.25)
            self.line6.set_data(self.ydata6, self.ydata6)
            
            self.ax7.set_xlim(self.ydata7[0], self.ydata7[-1])
            self.ax7.set_ylim(self.ydata7[-1]-.25, self.ydata7[-1]+.25)
            self.line7.set_data(self.ydata7, self.ydata7)
            
            self.ax8.set_xlim(self.ydata8[0], self.ydata8[-1])
            self.ax8.set_ylim(self.ydata8[-1]-1, self.ydata8[-1]+1)
            self.line8.set_data(self.ydata8, self.ydata8)
            
            self.ax9.set_xlim(self.ydata9[0], self.ydata9[-1])
            self.ax9.set_ylim(self.ydata9[-1]-.25, self.ydata9[-1]+.25)
            self.line9.set_data(self.ydata9, self.ydata9)
            
            self.ax10.set_xlim(self.ydata10[0], self.ydata10[-1])
            self.ax10.set_ylim(self.ydata10[-1]-.25, self.ydata10[-1]+.25)
            self.line10.set_data(self.ydata10, self.ydata10)
        except:
            pass

        self._drawn_artists = [self.line1,
                               self.line2,
                               self.line3,
                               self.line4,
                               self.line5,
                               self.line6,
                               self.line7,
                               self.line8,
                               self.line9,
                               self.line10]

    def new_frame_seq(self):
        return iter(range(self.t.size))

    def _init_draw(self):
        # Init drawing
        self.ax1.set_xlim(0, 20)
        self.ax2.set_xlim(0, 20)
        self.ax3.set_xlim(0, 20)
        lines = [self.line1,
                 self.line2,
                 self.line3,
                 self.line4,
                 self.line5,
                 self.line6,
                 self.line7,
                 self.line8,
                 self.line9,
                 self.line10]
        for l in lines:
            l.set_data([], [])

if __name__ == "__main__":
    try:
        ani = SubplotAnimation()
        while not rospy.is_shutdown():
            plt.show()
    except rospy.ROSInterruptException:
        pass
