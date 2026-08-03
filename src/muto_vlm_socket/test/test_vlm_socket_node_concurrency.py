from pathlib import Path
import sys
import threading


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS_DIR))

from rclpy.action import CancelResponse  # noqa: E402
from vlm_socket_node import VlmSocketNode  # noqa: E402


class FakeConnection:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


def make_node_state():
    node = object.__new__(VlmSocketNode)
    node._state_lock = threading.Lock()
    node._request_cancel_event = threading.Event()
    node._active_connection = None
    return node


def test_connection_registered_after_cancellation_is_closed_immediately():
    node = make_node_state()
    connection = FakeConnection()
    node._request_cancel_event.set()

    node._set_active_connection(connection)

    assert connection.closed
    assert node._active_connection is None


def test_connection_registered_before_cancellation_is_closed_by_callback():
    node = make_node_state()
    connection = FakeConnection()
    node._set_active_connection(connection)

    response = node._cancel_callback(None)

    assert response == CancelResponse.ACCEPT
    assert connection.closed
    assert node._active_connection is None
