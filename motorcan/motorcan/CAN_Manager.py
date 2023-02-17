import can


# A class to manage the low level CAN communication protocols
class CAN_Manager(object):

    # Note, defining singletons in this way means that you cannot inherit
    # from this class, as apparently __init__ for the subclass will be called twice
    _instance = None
    notifier: can.Notifier
    bus: can.interface.Bus
    """
    Used to keep track of one instantation of the class to make a singleton object
    """

    def __new__(cls):
        """
        Makes a singleton object to manage a socketcan_native CAN bus.
        """
        if not cls._instance:
            cls._instance = super(CAN_Manager, cls).__new__(cls)
            print("Initializing CAN Manager")
            # create a python-can bus object
            cls._instance.bus = can.interface.Bus(channel="can0", bustype="socketcan")
            # create a python-can notifier object, which motors can later subscribe to
            cls._instance.notifier = can.Notifier(bus=cls._instance.bus, listeners=[])
            print("Connected on: " + str(cls._instance.bus))

        return cls._instance

    def __init__(self):
        """
        ALl initialization happens in __new__
        """
        pass

    def __del__(self):
        """
        # shut down the CAN bus when the object is deleted
        # This may not ever get called, so keep a reference and explicitly delete if this is important.
        """
        pass

    # subscribe a motor object to the CAN bus to be updated upon message reception
    def add_motor_listener(self, motor_listener) -> bool:
        """
        Subscribe a motor object to the CAN bus to be updated upon message reception

        Args:
            motor_Listener: The object to be subscribed to the notifier
        """
        self.notifier.add_listener(motor_listener)
        return True

    def send(self, msg: can.Message) -> bool:
        try:
            self.bus.send(msg)
            return True
        except can.CanError:
            return False
