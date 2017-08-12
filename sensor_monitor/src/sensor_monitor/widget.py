import os
import Queue
import threading
import time

import rospy
import rospkg
import rosbag
from sensor_msgs.msg import CompressedImage, PointCloud2, Imu

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QHBoxLayout, QLabel, QTreeWidgetItem, QImage
from QtCore import Qt, QSize

class SensorMonitor(Plugin):

    def __init__(self, context):
        super(SensorMonitor, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('SensorMonitor')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('sensor_monitor'), 'resource', 'sensor_monitor.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('SensorMonitorUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        self._widget.record_btn.clicked[bool].connect(self.on_record_clicked)
        img_file = os.path.join(rospkg.RosPack().get_path('sensor_monitor'), 'resource', 'red.png')
        self.abnormal_image = QImage(img_file).scaled(QSize(15, 15))
        img_file = os.path.join(rospkg.RosPack().get_path('sensor_monitor'), 'resource', 'green.png')
        self.normal_image = QImage(img_file).scaled(QSize(15, 15))

        self.sensors = {}
        self.ts_list = {}
        self.freq = {}
        self.has_msg = {}
        self.bag = None
        self.write_queue = Queue.Queue()
        # self.write_thread = threading.Thread(target=self.run_write)
        self.write_thread = None

        rospy.Timer(rospy.Duration(1), self.timer_callback)

        self.add_sensor("pointgrey_left", "/pointgrey/left/image_raw/compressed", CompressedImage, 25.0)
        self.add_sensor("pointgrey_right", "/pointgrey/right/image_raw/compressed", CompressedImage, 25.0)
        self.add_sensor("imu", "/mti/sensor/imu", Imu, 400.0)
        self.add_sensor("velodyne", "/velodyne_points", PointCloud2, 10.0)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog

    def add_sensor(self, name, topic, ttype, freq):

        sensor = QTreeWidgetItem()
        sensor.setData(0, Qt.DisplayRole, name)
        sensor.setData(1, Qt.DisplayRole, topic)
        sensor.setData(2, Qt.DisplayRole, "0/" + str(freq))
        sensor.setData(3, Qt.DecorationRole, self.abnormal_image)
        self.sensors[name] = sensor
        self._widget.sensor_list.addTopLevelItem(sensor)

        self.ts_list[name] = []
        self.freq[name] = freq
        self.has_msg[name] = False
        rospy.Subscriber(topic, ttype, self.callback, (name, topic), queue_size=10)

    def callback(self, msg, args):
        name = args[0]
        topic = args[1]
        self.ts_list[name].append(msg.header.stamp.to_sec())
        self.has_msg[name] = True
        if len(self.ts_list[name]) > self.freq[name] * 2:
            self.ts_list[name] = self.ts_list[name][1:]

        if not self.bag is None:
            self.write_queue.put((topic, msg, rospy.get_rostime()))

    def timer_callback(self, event):
        for name in self.sensors:
            if self.has_msg[name] and self.ts_list[name]>0:
                af = (len(self.ts_list[name])-1) / (self.ts_list[name][-1] - self.ts_list[name][0])
            else:
                af = 0.0

            self.has_msg[name] = False

            self.sensors[name].setData(2, Qt.DisplayRole, ("%.1f" % af) + "/" + str(self.freq[name]))

            if abs(af - self.freq[name]) / self.freq[name] > 0.05:
                self.sensors[name].setData(3, Qt.DecorationRole, self.abnormal_image)
            else:
                self.sensors[name].setData(3, Qt.DecorationRole, self.normal_image)

        self._widget.write_queue_label.setText("Write queue size: %d" % self.write_queue.qsize())

    def on_record_clicked(self):
        if self.bag is None:
            self._widget.record_btn.setText('Stop')

            prefix = 'thomas'
            filename = time.strftime('%Y-%m-%d-%H-%M-%S.bag', time.localtime(time.time()))
            filename = '%s_%s' % (prefix, filename)
            rospy.loginfo('Writing to %s' % filename)

            self.bag = rosbag.Bag(filename, 'w')
            self.write_thread = threading.Thread(target=self.run_write)
            self.write_thread.start()
        else:
            self._widget.record_btn.setText('Record')

            rospy.loginfo('Stop writing')
            self.write_queue.put(self)
            self.write_thread.join()

            self.bag.close()
            self.bag = None
            self.write_queue = Queue.Queue()

    def run_write(self):
        try:
            while True:
                # Wait for a message
                item = self.write_queue.get()

                if item == self:
                    break

                topic, m, t = item
                self.bag.write(topic, m, t)

        except Exception as ex:
            print('Error write to bag: %s' % str(ex))
