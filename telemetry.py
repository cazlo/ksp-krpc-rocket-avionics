import os
import random
import time
from datetime import datetime, UTC
import threading

from influxdb_client import InfluxDBClient, Point, WriteOptions

from mission import Telemetry


class KSPTelemetry:
    """
    A class to handle publishing metrics periodically
    """

    def __init__(self):
        influx_url = os.environ.get("INFLUXDB_URL", "http://localhost:8086")
        token = os.environ.get("INFLUXDB_TOKEN", "admintoken")
        org = os.environ.get("INFLUXDB_ORG", "example_org")
        bucket = os.environ.get("INFLUXDB_BUCKET", "telem_bucket")
        self.org = org
        self.bucket = bucket
        self.client = InfluxDBClient(url=influx_url, token=token, org=org)
        self.write_api = self.client.write_api(write_options=WriteOptions(batch_size=1))

        self.publish_rate_hz = int(os.environ.get('TELEM_PUBLISH_RATE', 0))

        # metrics which come from control system calculations, not krpc streams
        self.gauge_metrics = {}
        self.enum_metrics = {}

        self.gnc_debug = os.environ.get('GNC_DEBUG', None) is not None



    def start_metrics_server(self, port: int = 8012):
        pass # todo no longer needed but maybe helpful for debug/unit test
        # todo some kind of lock or better error handling here if this class is instantiated > once
        # start_http_server(port)


    def start_telem_publish(self, telemetry: Telemetry) -> bool:
        if self.publish_rate_hz == 0:
            print('not starting telemetry publish thread')
            return False

        updater_thread = threading.Thread(target=self._telem_publish_thread, args=(telemetry,), daemon=True)
        updater_thread.start()
        return True


    def _telem_publish_thread(self, telemetry: Telemetry):
        while True:
            point = (
               Point("stream_telemetry")
                .field("surface_altitude", telemetry.surface_altitude())
                .field("altitude", telemetry.altitude())
                .field("apoapsis", telemetry.apoapsis())
                .field("velocity", telemetry.velocity())
                .field("vertical_vel", telemetry.vertical_vel())
                .field("horizontal_vel", telemetry.horizontal_vel())
                .field("periapsis", telemetry.periapsis())
                .time(time.time_ns())
            )
            self.write_api.write(bucket=self.bucket, record=point)


            time.sleep(1/self.publish_rate_hz)


    def register_gauge_metric(self, name: str, description: str):
        pass # todo maybe helpful to avoid metric name typos?
        # self.gauge_metrics[name] = Gauge(name, description)


    def publish_gauge_metric(self, name: str, state: float, console_out: bool = False):
        if console_out:
            print(
                f'{str(datetime.now(UTC).isoformat())} [{name}] {state}')

        try:
            point = (
                Point("arbitrary_telemetry")
                .field(name, state)
                .time(time.time_ns())
            )
            self.write_api.write(bucket=self.bucket, record=point)
        except Exception as e:
            print(f"ERROR: could not publish gauge metric for reason: {str(e)}")


    def register_enum_metric(self, name: str, description: str, states: [str]):
        pass # todo this would change to new impl of enum
        # self.enum_metrics[name] = Enum(name, description, states=states)


    def publish_enum_metric(self, name:str, state: str, display_name: str = None, console_out: bool = True):

        # if name not in self.enum_metrics:
        #     print(f"WARNING: skipping publish of unknown enum '{name}'. remember to register it first with register_enum_metric!")
        #     return
        try:
            # self.enum_metrics[name].state(state)
            # todo mimic the prometheus enum behavior here or do something new
            point = (
                Point("arbitrary_enum_telemetry")
                .field(name, state)
                .time(time.time_ns())
            )
            self.write_api.write(bucket=self.bucket, record=point)
        except Exception as e:
            print(f"ERROR: could not publish enum metric for reason: {str(e)}")

