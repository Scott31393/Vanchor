import queue
import serial
import re
from time import time
from time import sleep


class Controller:
    def __init__(self, main):
        self.logger = main.logging.getLogger(self.__class__.__name__)
        self.main = main
        self.emitter = main.event.emitter

        if self.main.debug != True:
            self.serial = serial.Serial(
                main.config.get("Serial/Controller/Device"),
                main.config.get("Serial/Controller/Baudrate"),
            )

        sleep(0.25)
        self.last_command = ""
        self.emitter.emit("status.set", ["Controller", {}])
        self.emitter.emit("controller.initialized")

        self.queue = queue.Queue()

        main.work_manager.start_worker(self.input_listener, **{"timer": 10})
        main.work_manager.start_worker(self.output_worker)
        main.work_manager.start_worker(
            self.command_stream_worker,
            **{"timer": main.config.get("Serial/Controller/OutputInterval")},
        )
        ###########################
        # EventHandlers
        ###########################
        self.emitter.on("controller.send", self.send)
        self.emitter.on("controller.status.reading", self.controller_status_reading)

    def send(self, command):
        self.logger.debug("Adding command {} to queue".format(command))

        if self.last_command != command:
            if isinstance(command, str):
                command = [time(), command]
            self.queue.put(command)
            self.last_command = command

    def output_worker(self, main):
        try:
            msg = self.queue.get()
            self.emitter.emit(
                "status.set", ["Controller/QueueSize", self.queue.qsize()]
            )

            timestamp = msg[0]
            command = msg[1]
            sent = time()
            delay = sent - timestamp

            self.logger.debug(f"Received command: {command}")

            if self.main.debug != True:
                if self.serial.isOpen():
                    self.serial.write(bytes((command + "\n"), "utf-8"))
                    if self.last_command != command:
                        self.logger.debug(
                            f"Sending command: {command} | Received:{timestamp} Sent:{sent} Delay:{delay}"
                        )
                else:
                    self.logger.error("Controller serial was not available")
            else:
                self.logger.debug(
                    f"Sending command: {command} | Received:{timestamp} Sent:{sent} Delay:{delay}"
                )

            self.queue.task_done()
            self.emitter.emit("status.set", ["Devices/Controller/Delay", delay])
        except Exception as e:
            self.logger.error(
                "Failed to send command to Controller serial device due to: {}".format(
                    e
                )
            )

    def controller_status_reading(self, reading):
        values = re.findall("[^ ]+:[^ $]+", reading)
        d = {}
        for entry in values:
            # Ensure entry is a string and matches the expected format
            if isinstance(entry, str) and re.match(r"[^ ]+:[^ $]+", entry):
                key, val = entry.split(":", 1)  # Split only on the first occurrence of ":"
                d[key] = val

        self.emitter.emit(
            "status.set.stepper.realposition", ["Stepper/RealPosition", d["SSP"]]
        )
        self.emitter.emit(
            "status.set.stepper.distancetogo", ["Stepper/DistanceToGo", d["SDTG"]]
        )
        self.emitter.emit("status.set.motor.realspeed", ["Motor/RealSpeed", d["MS"]])
        self.emitter.emit(
            "status.set.stepper.calibrationstart", ["Stepper/CalibrationStart", d["CB"]]
        )
        self.emitter.emit(
            "status.set.stepper.calibrationend", ["Stepper/CalibrationEnd", d["CE"]]
        )

    def input_listener(self, main):
        if self.main.debug != True:
            if self.serial.in_waiting > 0:
                try:
                    data = self.serial.read(self.serial.in_waiting)
                    data = data.decode('utf-8').strip()
                    # Split the data into lines and process each line
                    lines = data.splitlines()
                    for line in lines:
                        if line.startswith("STATUS "):
                            self.emitter.emit("controller.status.reading", line)
                        else:
                            # Optionally handle other data
                            pass

                except Exception as e:
                    self.logger.error("Error reading controller serial input: {}".format(e))

    def command_stream_worker(self, main, **kwargs):

        step = self.main.data.get("Stepper/Step")
        speed = self.main.config.get("Stepper/Speed")
        acceleration = self.main.config.get("Stepper/Acceleration")
        motor_speed = self.main.data.get("Motor/Speed")
        motor_rev = 0

        msg = f"UPD {step} {speed} {acceleration} {motor_speed} {motor_rev}"

        if self.queue.qsize() < 10:
            self.queue.put([time(), msg])
        else:
            self.logger.error("Queue is full")
