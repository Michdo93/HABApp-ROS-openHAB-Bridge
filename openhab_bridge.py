import logging
import HABApp
from HABApp import Parameter
from HABApp.core.events import ValueChangeEvent, ValueUpdateEvent
from HABApp.core.events import ValueChangeEventFilter, ValueUpdateEventFilter
from HABApp.openhab.events import ItemStateEvent
from HABApp.openhab.events import ItemStateEventFilter
from HABApp.core.items import Item
from HABApp.openhab.items import OpenhabItem
from HABApp.openhab.items import ColorItem, ContactItem, DatetimeItem, DimmerItem, GroupItem, ImageItem, LocationItem, NumberItem, PlayerItem, RollershutterItem, StringItem, SwitchItem
import rospy
from openhab_msgs.msg import ColorState, ContactState, DateTimeState, DimmerState, GroupState, ImageState, LocationState, NumberState, PlayerState, RollershutterState, StringState, SwitchState
from openhab_msgs.msg import ColorCommand, ContactCommand, DateTimeCommand, DimmerCommand, ImageCommand, LocationCommand, NumberCommand, PlayerCommand, RollershutterCommand, StringCommand, SwitchCommand
from dateutil import parser
from datetime import datetime, timezone
import base64
import io
import cv2
#from imageio import imread
from imageio.v2 import imread
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np

log = logging.getLogger('openhab_bridge')
log_state = Parameter('openhab_bridge', 'log_state', default_value=True).value


class OpenHABBridge(HABApp.Rule):
    def __init__(self):
        super().__init__()

        self.bridge = CvBridge()

        for item in self.get_items(type=OpenhabItem):
            if type(item) is ColorItem:
                item.listen_event(self.ColorState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', ColorCommand, self.ColorCallback)
            elif type(item) is ContactItem:
                item.listen_event(self.ContactState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', ContactCommand, self.ContactCallback)
            elif type(item) is DatetimeItem:
                item.listen_event(self.DateTimeState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', DateTimeCommand, self.DateTimeCallback)
            elif type(item) is GroupItem:
                item.listen_event(self.GroupState, ItemStateEventFilter())
            elif type(item) is DimmerItem:
                item.listen_event(self.DimmerState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', DimmerCommand, self.DimmerCallback)
            elif type(item) is ImageItem:
                item.listen_event(self.ImageState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', ImageCommand, self.ImageCallback)
            elif type(item) is LocationItem:
                item.listen_event(self.LocationState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', LocationCommand, self.LocationCallback)
            elif type(item) is NumberItem:
                item.listen_event(self.NumberState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', NumberCommand, self.NumberCallback)
            elif type(item) is PlayerItem:
                item.listen_event(self.PlayerState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', PlayerCommand, self.PlayerCallback)
            elif type(item) is RollershutterItem:
                item.listen_event(self.RollershutterState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', RollershutterCommand, self.RollershutterCallback)
            elif type(item) is StringItem:
                item.listen_event(self.StringState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', StringCommand, self.StringCallback)
            elif type(item) is SwitchItem:
                item.listen_event(self.SwitchState, ItemStateEventFilter())
                rospy.Subscriber(
                    f'/openhab/items/{item.name}/command', SwitchCommand, self.SwitchCallback)

        rospy.spin()

    def ColorState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = ColorState()

        if value is None:
            msg.hue = 0
            msg.saturation = 0
            msg.brightness = 0
            msg.isnull = True
        else:
            log.info("is ColorItem")
            msg.hue = int(value[0])
            msg.saturation = int(value[1])
            msg.brightness = int(value[2])
            msg.isnull = False

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', ColorState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def ContactState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = ContactState()

        if value is None:
            msg.state = "NULL"
            msg.isnull = True
        else:
            log.info("is ContactItem")
            msg.isnull = False

            if value == "OPEN":
                msg.state = ContactState.OPEN
            elif value == "CLOSED":
                msg.state = ContactState.CLOSED

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', ContactState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def DateTimeState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = DateTimeState()

        if value is None:
            msg.state = 0
            msg.isnull = True
        else:
            log.info("is DatetimeItem")
            msg.isnull = False
            msg.state = rospy.Time.from_sec(parser.parse(
                str(value)).replace(tzinfo=timezone.utc).timestamp())

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', DateTimeState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def DimmerState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = DimmerState()

        if value is None:
            msg.state = 0
            msg.isnull = True
        else:
            log.info("is DimmerItem")
            msg.isnull = False

            if 0 <= value <= 100:
                msg.state = int(value)

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', DimmerState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def GroupState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = GroupState()

        if value is None:
            msg.isnull = True
        else:
            log.info("is GroupItem")
            msg.isnull = False

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', GroupState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def ImageState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = ImageState()

        if value is None:
            log.info("numpy null image")
            msg.isnull = True
            cv2_img = np.zeros((100,100,3), dtype=np.uint8)
        else:
            log.info("is ImageItem")
            msg.isnull = False

            if isinstance(value, bytes):
                img_bytes = value
            else:
                img_bytes = eval(value)

            log.info(img_bytes)

            # reconstruct image as an numpy array
            cv2_img = imread(io.BytesIO(img_bytes))
            height, width, channels = cv2_img.shape

            log.info("Got image with height %s, width %s and channels %s" % (height, width, channels))
            #rospy.loginfo("Got image with height %s, width %s and channels %s" % (height, width, channels))

            # finally convert RGB image to BGR for opencv
            cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_RGB2BGR)
        
        try:
            converted = self.bridge.cv2_to_imgmsg(cv2_img, 'bgr8')
        except CvBridgeError as e:
            log.info(e)

        msg.state = converted

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', ImageState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()
        

    def LocationState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = LocationState()

        if value is None:
            msg.latitude = 0
            msg.longitude = 0
            msg.altitude = 0
            msg.isnull = True
        else:
            log.info("is LocationItem")
            msg.isnull = False

            splitted = value.split(",")
            msg.latitude = float(splitted[0])
            msg.longitude = float(splitted[1])
            msg.altitude = float(splitted[2])

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', LocationState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def NumberState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = NumberState()

        if value is None:
            msg.state = float(0)
            msg.isnull = True
        else:
            log.info("is NumberItem")
            msg.isnull = False

            msg.state = value

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', NumberState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def PlayerState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = PlayerState()

        if value is None:
            msg.state = "NULL"
            msg.isnull = True
        else:
            log.info("is PlayerItem")
            msg.isnull = False

            if value == "PLAY":
                msg.state = PlayerState.PLAY
            elif value == "PAUSE":
                msg.state = PlayerState.PAUSE
            elif value == "NEXT":
                msg.state = PlayerState.NEXT
            elif value == "PREVIOUS":
                msg.state = PlayerState.PREVIOUS
            elif value == "REWIND":
                msg.state = PlayerState.REWIND
            elif value == "FASTFORWARD":
                msg.state = PlayerState.FASTFORWARD

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', PlayerState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def RollershutterState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = RollershutterState()

        if value is None:
            msg.state = "NULL"
            msg.isstate = False
            msg.ispercentage = False
            msg.percentage = 0
            msg.isnull = True
        else:
            log.info("is RollershutterItem")
            msg.isnull = False

            if isinstance(value, int):
                msg.isstate = False
                msg.state = "NULL"
                msg.ispercentage = True

                if 0 <= value <= 100:
                    msg.percentage = value
            elif isinstance(value, str):
                msg.isstate = True
                msg.ispercentage = False
                msg.percentage = 0

                if value == "UP":
                    msg.state = RollershutterState.UP
                elif value == "DOWN":
                    msg.state = RollershutterState.DOWN
                elif value == "STOP":
                    msg.state = RollershutterState.STOP
                elif value == "MOVE":
                    msg.state = RollershutterState.MOVE

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', RollershutterState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def StringState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = StringState()

        if value is None:
            value = "NULL"
            msg.isnull = True
        else:
            log.info("is StringItem")
            msg.isnull = False
            msg.state = str(value)

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', StringState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def SwitchState(self, event: ItemStateEvent):
        item = event.name
        value = event.value

        msg = SwitchState()

        if value is None:
            msg.state = "NULL"
            msg.isnull = True
        else:
            log.info("is SwitchItem")
            msg.isnull = False

            if value == "ON":
                msg.state = SwitchState.ON
            elif value == "OFF":
                msg.state = SwitchState.OFF

        pub = rospy.Publisher(
            f'/openhab/items/{item}/state', SwitchState, queue_size=1)

        for my_item in self.get_items(name=item):
            item_type = my_item

        if hasattr(item_type, 'last_update'):
            msg.header.stamp = rospy.Time.from_sec(parser.parse(
                str(item_type.last_update)).replace(tzinfo=timezone.utc).timestamp())
        else:
            msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/base_link"
        msg.item = str(item)

        log.info(
            f'Published ROS topic /openhab/items/{item}/state with {value}')

        rate = rospy.Rate(1)
        counter = 0

        while counter < 1:
            # wait for a connection to publisher
            # you can do whatever you like here or simply do nothing

            connections = pub.get_num_connections()
            if connections > 0:
                rospy.loginfo(
                    f'Published ROS topic /openhab/items/{item}/state with {value}')
                pub.publish(msg)
                counter = counter + 1
            else:
                rate.sleep()

    def ColorCallback(self, data):
        item = data.item

        if data.iscommand == True:
            if data.command == ColorCommand.ON or data.command == ColorCommand.OFF or data.command == ColorCommand.INCREASE or data.command == ColorCommand.DECREASE:
                value = data.command
            else:
                value = None
        elif data.ispercentage == True:
            if 0 <= data.percentage <= 100:
                value = data.percentage
        elif data.ishsb == True:
            value = (data.hue, data.saturation, data.brightness)

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def ContactCallback(self, data):
        item = data.item

        if data.command == ContactCommand.OPEN or data.command == ContactCommand.CLOSED:
            value = data.command

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        #self.oh.send_command(item, value)
        self.oh.post_update(item, value)

    def DateTimeCallback(self, data):
        item = data.item

        if data.isnull == True:
            value = None
        else:
            value = datetime.utcfromtimestamp(
                data.command.to_sec()).strftime("%Y-%m-%dT%H:%M:%SZ")

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.post_update(item, value)

    def DimmerCallback(self, data):
        item = data.item

        if data.iscommand == True:
            if data.command == DimmerCommand.ON or data.command == DimmerCommand.OFF or data.command == DimmerCommand.INCREASE or data.command == DimmerCommand.DECREASE:
                value = data.command
        elif data.ispercentage == True:
            if 0 <= data.percentage <= 100:
                value = data.percentage

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def ImageCallback(self, data):
        item = data.item

        if data.isnull == True:
            value = None
        else:
            cv_image = self.bridge.imgmsg_to_cv2(
                data.command, "bgr8")

            retval, buffer = cv2.imencode('.jpg', cv_image)
            value = buffer.tobytes()

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        #self.oh.post_update(item, value)
        image = ImageItem(item)
        image.oh_post_update(value)

    def LocationCallback(self, data):
        item = data.item

        if data.isnull == True:
            value = None
        else:
            value = str(data.latitude) + "," + \
                str(data.longitude) + "," + str(data.altitude)

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def NumberCallback(self, data):
        item = data.item

        if data.isnull == True:
            value = None
        else:
            value = float(data.command)

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def PlayerCallback(self, data):
        item = data.item

        if data.command == PlayerCommand.PLAY or data.command == PlayerCommand.PAUSE or data.command == PlayerCommand.NEXT or data.command == PlayerCommand.PREVIOUS or data.command == PlayerCommand.REWIND or data.command == PlayerCommand.FASTFORWARD:
            value = data.command

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def RollershutterCallback(self, data):
        item = data.item

        if data.iscommand == True:
            if data.command == RollershutterCommand.UP or data.command == RollershutterCommand.DOWN or data.command == RollershutterCommand.STOP or data.command == RollershutterCommand.MOVE:
                value = data.command
        elif data.ispercentage == True:
            if 0 <= data.percentage <= 100:
                value = data.percentage

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def StringCallback(self, data):
        item = data.item

        if isinstance(data.command, str):
            value = data.command

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)

    def SwitchCallback(self, data):
        item = data.item

        if data.command == SwitchCommand.ON or data.command == SwitchCommand.OFF:
            value = data.command
        else:
            value = None

        if data.isnull == True:
            value = None

        rospy.loginfo(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')
        log.info(
            f'{rospy.get_caller_id()} Subscribed ROS topic /openhab/items/{item}/command with {value}')

        self.oh.send_command(item, value)


class LogItemStateRule(HABApp.Rule):
    """This rule logs the item state in the mqtt event bus log file"""

    def __init__(self):
        super().__init__()

        for item in self.get_items(type=OpenhabItem):
            item.listen_event(self.on_item_change, ValueChangeEvent)

    def on_item_change(self, event):
        assert isinstance(event, ValueChangeEvent)
        log.info(f'{event.name} changed from {event.old_value} to {event.value}')


OpenHABBridge()

# Create logger rule only if configured
if log_state:
    LogItemStateRule()
