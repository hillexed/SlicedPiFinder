#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
This module is the camera
* Captures images
* Places preview images in queue
* Places solver images in queue
* Takes full res images on demand

"""

from PIL import Image
from PiFinder import config
from PiFinder.camera_interface import CameraInterface
from typing import Tuple
import logging
from PiFinder.multiproclogging import MultiprocLogging

logger = logging.getLogger("Camera.Pi")


class CameraPI(CameraInterface):
    """The camera class for PI cameras.  Implements the CameraInterface interface."""

    def __init__(self, exposure_time) -> None:
        from picamera2 import Picamera2

        self.camera = Picamera2()
        # Figure out camera type, hq or gs (global shutter)
        self.camera_type = "hq"
        self.camType = f"PI {self.camera_type}"
        self.exposure_time = exposure_time
        self.gain = 15
        self.initialize()

    def initialize(self) -> None:
        """Initializes the camera and set the needed control parameters"""
        self.camera.stop()
        # using this smaller scale auto-selects binning on the sensor...
        cam_config = self.camera.create_still_configuration()
        self.camera.configure(cam_config)
        self.camera.set_controls({"AeEnable": False})
        self.camera.set_controls({"AnalogueGain": self.gain})
        self.camera.set_controls({"ExposureTime": self.exposure_time})
        self.camera.start()

    def capture(self) -> Image.Image:
       
        cap_image = self.camera.capture_image()
        w, h = cap_image.size

        # all fovs here are horizontal fovs
        #camera_fov = 26.2
        #desired_fov = 10.2
        #crop_width = desired_fov / camera_fov * img.size[0]
        #crop_height = crop_width

        crop_width = 748 # desired_fov / camera_fov * img.size[0]
        crop_height = 748
        w_margin_size = int((w - crop_width) / 2) # assuming we have an image of size crop_size in the center, how much extra margin is there on either side of the image?
        h_margin_size = int((h - crop_height) / 2) 
        
        # crop to the middle of the image
        cap_image = cap_image.crop(
            (
                w_margin_size,
                h_margin_size,
                w - w_margin_size,
                h - h_margin_size,
            )
        )
        cap_image = cap_image.resize((512,512))
        return cap_image

    def capture_file(self, filename) -> None:
        return self.camera.capture_file(filename)

    def set_camera_config(
        self, exposure_time: float, gain: float
    ) -> Tuple[float, float]:
        self.camera.stop()
        self.camera.set_controls({"AnalogueGain": gain})
        self.camera.set_controls({"ExposureTime": exposure_time})
        self.camera.start()
        return exposure_time, gain

    def get_cam_type(self) -> str:
        return self.camType


def get_images(shared_state, camera_image, command_queue, console_queue, log_queue):
    """
    Instantiates the camera hardware
    then calls the universal image loop
    """
    MultiprocLogging.configurer(log_queue)

    cfg = config.Config()
    exposure_time = cfg.get_option("camera_exp")
    camera_hardware = CameraPI(exposure_time)
    camera_hardware.get_image_loop(
        shared_state, camera_image, command_queue, console_queue, cfg
    )
