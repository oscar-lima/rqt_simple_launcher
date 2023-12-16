#!/usr/bin/env python3

import os
import sys
import signal
import time

import rospy
import rospkg
import yaml

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QFileDialog

from PyQt5.QtWidgets import QApplication

import subprocess

class OpenFileDialog(QWidget):
    '''
    this will open a dialog to select and open a yaml file
    '''
    def __init__(self, initial_path=None):
        super().__init__()
        left, top, width, height = 10, 10, 640, 480
        self.setGeometry(left, top, width, height)
        self.initial_path = initial_path

    def openFileNameDialog(self, save_file_name_dialog=False):
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        if self.initial_path is None:
            initial_path = os.environ['HOME']
        else:
            initial_path = self.initial_path
        if save_file_name_dialog:
            fileName, _ = QFileDialog.getSaveFileName(self, 'Select boxes yaml file',\
                          initial_path, 'Yaml Files (*.yaml)', options=options)
        else:
            fileName, _ = QFileDialog.getOpenFileName(self, 'Select boxes yaml file',\
                          initial_path, 'Yaml Files (*.yaml)', options=options)
        if fileName:
            return fileName

    def saveFileNameDialog(self):
        return self.openFileNameDialog(save_file_name_dialog=True)

class RqtSimpleLauncher(Plugin):

    def __init__(self, context):
        super(RqtSimpleLauncher, self).__init__(context)
        rospy.loginfo('Initializing rqt_simple_launcher')

        self.setObjectName('RqtSimpleLauncher')
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file (xml description of the gui window created with qtcreator)
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_simple_launcher'), 'config', 'rqt_simple_launcher.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('rqt_simple_launcher.ui')
        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # class variables
        self.launch_on = False
        self.launch_process = None

        # parameters
        # False: really launches the launch file, True: only prints the command that will be launched for debug purposes
        self.only_print_cmd = rospy.get_param('~only_print_cmd', False)
        brief = rospy.get_param('~brief', 'Bringup sim')
        self.launch = rospy.get_param('~launch', {})
        self.cmd_args = []
        self.bool_arg = rospy.get_param('~bool_arg', {})
        self.args_dictionary = rospy.get_param('~args', {})
        required_services = rospy.get_param('~required_services', [])
        self.robot_namespace = rospy.get_param('~robot_namespace', 'mobipick')
        placeholder_values = rospy.get_param('~placeholder_values', {'robot_namespace': self.robot_namespace})
        self.required_services = self.replace_placeholders(required_services, placeholder_values)

        # construct launch file path
        if self.launch == {}:
            rospy.logerr('Required parameter launch not set, exiting')
            sys.exit(0)
        else:
            pkg_path = rospkg.RosPack().get_path(self.launch['ros_pkg'])
            folder = self.launch['folder']
            launch_file = self.launch['launch_file']
            self.launch_file_path = f'{pkg_path}/{folder}/{launch_file}'

        # set label of what the button launch will do
        self._widget.lblBrief.setText(brief)

        # handle bool param, if required set its text, otherwise hide the widget
        if self.bool_arg == {}:
            self._widget.chkBoolParam.hide()
        else:
            self._widget.chkBoolParam.setText(self.bool_arg['text'])
            self._widget.chkBoolParam.setChecked(self.bool_arg['initial_state'])

        # hide all combo boxes with it's labels
        self.comboArgList = [self._widget.comboArg1, self._widget.comboArg2, self._widget.comboArg3, self._widget.comboArg4]
        for comboArg in self.comboArgList:
            comboArg.hide()
        self.lblArgList = [self._widget.lblArg1, self._widget.lblArg2, self._widget.lblArg3, self._widget.lblArg4]
        for lblArg in self.lblArgList:
            lblArg.hide()

        # show only the required combo boxes with it's labels
        for i, arg in enumerate(self.args_dictionary):
            # show combo box
            self.lblArgList[i].show()
            self.lblArgList[i].setText(arg['text'])
            # fill combo box with options
            self.comboArgList[i].show()
            self.comboArgList[i].addItems(arg['options'])

        # set default option for each combo box
        for i, arg in enumerate(self.args_dictionary):
            index = self.comboArgList[i].findText(arg['selected_option'])
            self.comboArgList[i].setCurrentIndex(index)

        # make button orange at startup
        self.configure_launch_button('orange', 'Down\nclick to\nLaunch')

        ## make a connection between the qt objects and this class methods
        self._widget.cmdLaunch.clicked.connect(self.cmd_launch_clicked)
        self._widget.cmdSaveConfig.clicked.connect(self.cmd_save_config_clicked)

        context.add_widget(self._widget)

        # to catch Ctrl + C signal from keyboard and close stream properly
        signal.signal(signal.SIGINT, self.signal_handler)

        rospy.loginfo(f'{brief} initialized')
        # end of constructor

    # ::::::::::::::  class methods

    def shutdown_plugin(self):
        rospy.loginfo('I detected you want to quit, calling destructor')
        self.__del__()

    def signal_handler(self, sig, frame):
        rospy.loginfo('You pressed Ctrl+C, calling destructor')
        self.__del__()

    def __del__(self):
        '''
        Destructor: perform cleanup
        '''
        self.kill_launch_process()
        rospy.loginfo('bye bye!')
        sys.exit(0)

    def replace_placeholders(self, item, placeholder_values):
        if isinstance(item, dict):
            for key, value in item.items():
                item[key] = self.replace_placeholders(value, placeholder_values)
        elif isinstance(item, list):
            return [self.replace_placeholders(elem, placeholder_values) for elem in item]
        elif isinstance(item, str):
            return item.format(**placeholder_values)
        return item

    def cmd_launch_clicked(self):
        self.launch_on = not self.launch_on
        if self.launch_on:
            self.launch_file_process()
        else:
            self.kill_launch_process()

    def configure_launch_button(self, color, text):
        self._widget.cmdLaunch.setText(f'State: {text}')
        self._widget.cmdLaunch.setStyleSheet(f'background-color: {color}; color: white;')
        QApplication.processEvents()

    def fill_args(self):
        self.cmd_args = []
        bool_arg_value = False
        if self._widget.chkBoolParam.isChecked():
            bool_arg_value = True
        self.cmd_args.append(f"{self.bool_arg['arg']}:={bool_arg_value}")
        for i, comboArg in enumerate(self.comboArgList):
            if comboArg.isVisible():
                self.cmd_args.append(f"{self.args_dictionary[i]['arg']}:={comboArg.currentText()}")

    def check_cmd(self, cmd):
        # prevent shell injection
        forbidden_chars = [' ', '|', '&', ';', '`', '$', '(', ')', '{', '}', '[', ']',
                           '*', '?', '~', '!', '<', '>', '\n', '\r', '\\', '"', "'"]
        if any(char in cmd for char in forbidden_chars):
            rospy.logerr(f'You can not use {forbidden_chars} in your command, will exit')
            sys.exit(0)

    def service_exists(self, service_name):
        try:
            output = subprocess.check_output(['rosservice', 'list'])
            return service_name in output.decode('utf-8').split('\n')
        except subprocess.CalledProcessError:
            return False

    def wait_for_services(self, required_services, timeout=60):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if all(self.service_exists(service) for service in required_services):
                rospy.loginfo('All services are available!')
                return True
            time.sleep(1)
        rospy.logwarn('Timeout: Not all services were available.')
        return False

    def pretty_print_cmd(self, cmd):
        return ' '.join(cmd)

    def launch_file_process(self):
        self.configure_launch_button('red', 'Launching\nplease wait...')
        self.check_cmd(self.launch_file_path)
        self.fill_args()
        for arg in self.cmd_args:
            self.check_cmd(arg)
        cmd = ['roslaunch', self.launch_file_path] + self.cmd_args
        rospy.loginfo(f'Launching:\n{self.pretty_print_cmd(cmd)}')
        if not self.only_print_cmd:
            # shell=False is important as it is safer
            self.launch_process = subprocess.Popen(cmd, shell=False, preexec_fn=os.setsid)
        self.wait_for_services(self.required_services)
        self.configure_launch_button('green', 'Running\nclick to\ntake down')

    def kill_launch_process(self):
        '''
        Function to send SIGINT to the launch file's process
        '''
        self.configure_launch_button('red', 'Killing...\nplease wait...')
        if self.launch_process:
            if not self.only_print_cmd:
                os.killpg(os.getpgid(self.launch_process.pid), signal.SIGINT)
                # wait for the process to complete
                self.launch_process.wait()
            self.launch_process = None
        self.configure_launch_button('orange', 'Down\nclick to\nLaunch')

    def cmd_save_config_clicked(self):
        rospy.loginfo('cmd_save_config_clicked')
        sfd = OpenFileDialog()
        yaml_path = sfd.saveFileNameDialog()
        if not '.yaml' in yaml_path:
            yaml_path = yaml_path + '.yaml'
        # create master dictionary
        master_dict = {}
        master_dict.update({'brief': self._widget.lblBrief.text()})
        master_dict.update({'launch': self.launch})
        bool_arg_checked = False
        if self._widget.chkBoolParam.isChecked():
            bool_arg_checked = True
        self.bool_arg.update({'initial_state': bool_arg_checked})
        master_dict.update({'bool_arg': self.bool_arg})
        master_dict.update({'args': self.args_dictionary})
        # get selected combo box options
        for i, arg in enumerate(self.args_dictionary):
            master_dict['args'][i].update({'selected_option': self.comboArgList[i].currentText()})
        rospy.loginfo(f'writing current configuration to yaml file: {yaml_path}')
        with open(yaml_path, 'w') as yaml_file:
            yaml.dump(master_dict, yaml_file, default_flow_style=False)
