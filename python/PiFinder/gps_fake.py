#!/usr/bin/python
# -*- coding:utf-8 -*-
"""
This module is for GPS related functions

"""
import time
from PiFinder import config
import logging
import datetime

cfg = config.Config()


def gps_monitor(gps_queue, console_queue):
    config_location = cfg.get_option("last_location")

    # todo: allow user to change this via the UI
    system_clock_correction_offset = datetime.timedelta(seconds=0)

    gps_locked = False
    while True:
        """
        Use the config file's location to spoof a GPS (assuming you've connected to wifi so the system clock is correct)
        """
        time.sleep(5)

        # fake send 
        msg = (
            "fix",
            {
                "lat": config_location["lat"],
                "lon": config_location["lon"],
                "altitude": config_location["altitude"]
            },
        )
        logging.debug("Fake gps fix: %s", msg)
        gps_queue.put(msg)

        # use system time, assuming we're connected to NTP
        clocktime = (datetime.datetime.utcnow() + system_clock_correction_offset)
        msg = ("time", clocktime)        
        gps_queue.put(msg)
