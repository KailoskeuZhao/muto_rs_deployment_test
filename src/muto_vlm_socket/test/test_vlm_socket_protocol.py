"""Tests for VLM request validation, encoding, and bounded transport."""

import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))

from vlm_socket_protocol import (  # noqa: E402
    build_request_body,
    encode_content,
    extract_response,
    parse_endpoint,
    post_chat_completion,
    ProtocolError,
    ProtocolLimits,
    TransportError,
)


@pytest.fixture
def limits():
    """Return small but usable protocol limits."""
    return ProtocolLimits(8, 1000, 1024, 2048, 8192, 4096)


def part(part_type, text='', jpeg_data=b''):
    """Construct a ROS-message-shaped content part."""
    return SimpleNamespace(
        type=part_type,
        text=text,
        jpeg_data=jpeg_data,
    )


def test_interleaved_content_preserves_order(limits):
    """Text and JPEG parts remain in their original order."""
    jpeg = b'\xff\xd8payload\xff\xd9'
    encoded = encode_content([
        part(1, text='before'),
        part(2, jpeg_data=jpeg),
        part(1, text='after'),
    ], limits)

    assert [item['type'] for item in encoded.parts] == [
        'text', 'image_url', 'text']
    data_url = encoded.parts[1]['image_url']['url']
    assert base64.b64decode(data_url.split(',', 1)[1]) == jpeg
    assert encoded.text_characters == 11
    assert encoded.jpeg_bytes == len(jpeg)


@pytest.mark.parametrize('content', [
    [],
    [part(0, text='unknown')],
    [part(1, text='')],
    [part(1, text='text', jpeg_data=b'bytes')],
    [part(2, text='also text', jpeg_data=b'\xff\xd8\xff\xd9')],
    [part(2, jpeg_data=b'not a jpeg')],
])
def test_invalid_content_is_rejected(content, limits):
    """Malformed union fields never reach the network."""
    with pytest.raises(ProtocolError):
        encode_content(content, limits)


def test_payload_contains_optional_fields(limits):
    """Configured system, token, and temperature fields are serialized."""
    encoded = encode_content([part(1, text='hello')], limits)
    body = build_request_body(
        encoded, 'model-a', 'be concise', 128, 0.2,
        limits.max_request_bytes)
    payload = json.loads(body)

    assert payload['model'] == 'model-a'
    assert payload['messages'][0] == {
        'role': 'system', 'content': 'be concise'}
    assert payload['messages'][1]['content'][0]['text'] == 'hello'
    assert payload['max_tokens'] == 128
    assert payload['temperature'] == 0.2
    assert payload['stream'] is False


def test_endpoint_appends_chat_completions():
    """Base and complete endpoint forms normalize identically."""
    base = parse_endpoint('https://example.test:8443/v1/')
    complete = parse_endpoint(
        'https://example.test:8443/v1/chat/completions')

    assert base == complete
    assert base.target == '/v1/chat/completions'
    assert base.port == 8443


@pytest.mark.parametrize('url', [
    '',
    'ftp://example.test/v1',
    'https://name:password@example.test/v1',
    'https://example.test/v1?debug=true',
])
def test_unsafe_endpoint_is_rejected(url):
    """Invalid schemes and credential-bearing URLs are rejected."""
    with pytest.raises(ProtocolError):
        parse_endpoint(url)


def test_response_text_and_usage_are_extracted():
    """String and structured text responses expose bounded token usage."""
    text, prompt_tokens, completion_tokens = extract_response({
        'choices': [{
            'message': {
                'content': [
                    {'type': 'text', 'text': 'hello '},
                    {'type': 'text', 'text': 'world'},
                ],
            },
        }],
        'usage': {
            'prompt_tokens': 12,
            'completion_tokens': 3,
        },
    })

    assert text == 'hello world'
    assert prompt_tokens == 12
    assert completion_tokens == 3


class _Handler(BaseHTTPRequestHandler):
    """Minimal successful OpenAI-compatible endpoint."""

    request_path = None
    authorization = None
    request_payload = None

    def do_POST(self):
        """Capture a request and return a chat-completions response."""
        length = int(self.headers['Content-Length'])
        type(self).request_path = self.path
        type(self).authorization = self.headers.get('Authorization')
        type(self).request_payload = json.loads(self.rfile.read(length))
        body = json.dumps({
            'choices': [{'message': {'content': 'observed'}}],
            'usage': {'prompt_tokens': 7, 'completion_tokens': 1},
        }).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, _format, *_args):
        """Keep tests quiet."""


def test_http_transport_sends_bounded_authenticated_request(limits):
    """The transport targets chat completions and returns decoded JSON."""
    server = ThreadingHTTPServer(('127.0.0.1', 0), _Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connections = []
    try:
        encoded = encode_content([part(1, text='look')], limits)
        body = build_request_body(
            encoded, 'model-a', '', 0, -1.0,
            limits.max_request_bytes)
        endpoint = parse_endpoint(
            f'http://127.0.0.1:{server.server_port}/v1')
        response = post_chat_completion(
            endpoint, body, 'test-token', 2.0,
            limits.max_response_bytes, connections.append)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)

    assert response['choices'][0]['message']['content'] == 'observed'
    assert _Handler.request_path == '/v1/chat/completions'
    assert _Handler.authorization == 'Bearer test-token'
    assert _Handler.request_payload['model'] == 'model-a'
    assert connections[0] is not None
    assert connections[-1] is None


def test_http_error_does_not_expose_response_body(limits):
    """Transport errors do not echo potentially sensitive server bodies."""
    class ErrorHandler(_Handler):
        """Endpoint returning a body that must not escape."""

        def do_POST(self):
            """Return a deliberately sensitive error body."""
            body = b'do-not-expose-this-body'
            self.send_response(401)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(('127.0.0.1', 0), ErrorHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        endpoint = parse_endpoint(
            f'http://127.0.0.1:{server.server_port}/v1')
        with pytest.raises(TransportError) as error:
            post_chat_completion(
                endpoint, b'{}', None, 2.0,
                limits.max_response_bytes, lambda _connection: None)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)

    assert 'do-not-expose-this-body' not in str(error.value)
