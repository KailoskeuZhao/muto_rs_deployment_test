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
    decode_response_body,
    decode_response_json_schema,
    encode_content,
    extract_response,
    parse_endpoint,
    post_vlm_request,
    ProtocolError,
    ProtocolLimits,
    TransportError,
    WIRE_API_RESPONSES,
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


def test_responses_payload_maps_ordered_multimodal_content(limits):
    """Responses requests use input content types and disable storage."""
    jpeg = b'\xff\xd8payload\xff\xd9'
    encoded = encode_content([
        part(1, text='before'),
        part(2, jpeg_data=jpeg),
        part(1, text='after'),
    ], limits)
    body = build_request_body(
        encoded, 'model-r', 'be concise', 128, 0.2,
        limits.max_request_bytes, WIRE_API_RESPONSES, False)
    payload = json.loads(body)

    content = payload['input'][0]['content']
    assert [item['type'] for item in content] == [
        'input_text', 'input_image', 'input_text']
    assert content[0]['text'] == 'before'
    assert content[1]['image_url'].startswith(
        'data:image/jpeg;base64,')
    assert content[2]['text'] == 'after'
    assert payload['instructions'] == 'be concise'
    assert payload['max_output_tokens'] == 128
    assert payload['temperature'] == 0.2
    assert payload['stream'] is False
    assert payload['store'] is False
    assert 'messages' not in payload


def test_responses_payload_carries_strict_json_schema(limits):
    """Responses structured output uses the documented text format shape."""
    encoded = encode_content([part(1, text='select')], limits)
    schema = {
        'type': 'object',
        'properties': {'matches': {'type': 'array'}},
        'required': ['matches'],
        'additionalProperties': False,
    }
    body = build_request_body(
        encoded, 'model-r', '', 0, -1.0,
        limits.max_request_bytes, WIRE_API_RESPONSES, False, schema)
    payload = json.loads(body)

    assert payload['text']['format'] == {
        'type': 'json_schema',
        'name': 'ros_vlm_response',
        'schema': schema,
        'strict': True,
    }


def test_chat_payload_carries_strict_json_schema(limits):
    """Chat Completions receives its corresponding response_format shape."""
    encoded = encode_content([part(1, text='select')], limits)
    schema = {
        'type': 'object',
        'properties': {},
        'additionalProperties': False,
    }
    body = build_request_body(
        encoded, 'model-c', '', 0, -1.0,
        limits.max_request_bytes,
        response_json_schema=schema,
    )
    payload = json.loads(body)

    assert payload['response_format'] == {
        'type': 'json_schema',
        'json_schema': {
            'name': 'ros_vlm_response',
            'schema': schema,
            'strict': True,
        },
    }


def test_optional_response_schema_is_bounded_and_validated(limits):
    """Malformed or unsuitable schemas fail before request dispatch."""
    schema = decode_response_json_schema(
        '{"type":"object","additionalProperties":false}', 100)

    assert schema == {
        'type': 'object',
        'additionalProperties': False,
    }
    assert decode_response_json_schema('  ', 100) is None
    with pytest.raises(ProtocolError):
        decode_response_json_schema('{bad json', 100)
    with pytest.raises(ProtocolError):
        decode_response_json_schema('{"type":"array"}', 100)
    with pytest.raises(ProtocolError):
        decode_response_json_schema('{"type":"object"}', 4)


def test_endpoint_appends_chat_completions():
    """Base and complete endpoint forms normalize identically."""
    base = parse_endpoint('https://example.test:8443/v1/')
    complete = parse_endpoint(
        'https://example.test:8443/v1/chat/completions')

    assert base == complete
    assert base.target == '/v1/chat/completions'
    assert base.port == 8443


def test_endpoint_appends_responses():
    """Responses mode targets the Responses API and rejects conflicts."""
    base = parse_endpoint(
        'https://example.test:8443/v1/', WIRE_API_RESPONSES)
    complete = parse_endpoint(
        'https://example.test:8443/v1/responses', WIRE_API_RESPONSES)

    assert base == complete
    assert base.target == '/v1/responses'
    with pytest.raises(ProtocolError):
        parse_endpoint(
            'https://example.test/v1/chat/completions',
            WIRE_API_RESPONSES,
        )


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


@pytest.mark.parametrize('choice', [
    {
        'finish_reason': 'length',
        'message': {'content': 'partial'},
    },
    {
        'finish_reason': 'content_filter',
        'message': {'content': ''},
    },
    {
        'finish_reason': 'stop',
        'message': {'content': '', 'refusal': 'cannot comply'},
    },
])
def test_chat_incomplete_filtered_and_refused_outputs_are_rejected(choice):
    """Non-success Chat terminal states never become action success."""
    with pytest.raises(ProtocolError):
        extract_response({'choices': [choice]})


def test_responses_output_and_usage_are_extracted():
    """Raw Responses output messages map to the existing ROS result."""
    text, input_tokens, output_tokens = extract_response({
        'status': 'completed',
        'output': [
            {'type': 'reasoning', 'content': []},
            {
                'type': 'message',
                'content': [
                    {'type': 'output_text', 'text': 'hello '},
                    {'type': 'output_text', 'text': 'world'},
                ],
            },
        ],
        'usage': {
            'input_tokens': 15,
            'output_tokens': 4,
        },
    }, WIRE_API_RESPONSES)

    assert text == 'hello world'
    assert input_tokens == 15
    assert output_tokens == 4


@pytest.mark.parametrize('response', [
    {
        'status': 'incomplete',
        'incomplete_details': {'reason': 'max_output_tokens'},
        'output_text': 'partial',
    },
    {
        'status': 'failed',
        'output': [],
    },
    {
        'status': 'completed',
        'output': [{
            'type': 'message',
            'content': [{'type': 'refusal', 'refusal': 'cannot comply'}],
        }],
    },
])
def test_responses_incomplete_failed_and_refused_outputs_are_rejected(
        response):
    """Non-success Responses terminal states never become action success."""
    with pytest.raises(ProtocolError):
        extract_response(response, WIRE_API_RESPONSES)


def test_responses_output_text_shortcut_is_supported():
    """Compatible proxies may expose the SDK-style output_text field."""
    text, input_tokens, output_tokens = extract_response({
        'output_text': 'direct',
        'usage': {},
    }, WIRE_API_RESPONSES)

    assert text == 'direct'
    assert input_tokens == 0
    assert output_tokens == 0


def test_responses_sse_returns_only_completed_response():
    """A proxy-forced SSE envelope yields its final response object."""
    completed = {
        'output': [{
            'type': 'message',
            'content': [{'type': 'output_text', 'text': 'done'}],
        }],
        'usage': {'input_tokens': 2, 'output_tokens': 1},
    }
    body = (
        'event: response.created\n'
        'data: {"type":"response.created","response":{}}\n\n'
        'event: response.output_text.delta\n'
        'data: {"type":"response.output_text.delta","delta":"done"}\n\n'
        'event: response.completed\n'
        'data: ' + json.dumps({
            'type': 'response.completed',
            'response': completed,
        }) + '\n\n'
    ).encode()

    assert decode_response_body(
        body, 'text/event-stream; charset=utf-8',
        WIRE_API_RESPONSES) == completed


def test_responses_sse_requires_completed_event():
    """Truncated or failed event streams cannot become successful results."""
    body = (
        'event: response.output_text.delta\n'
        'data: {"type":"response.output_text.delta","delta":"partial"}\n\n'
    ).encode()

    with pytest.raises(TransportError):
        decode_response_body(
            body, 'text/event-stream', WIRE_API_RESPONSES)


def test_responses_sse_recovers_text_from_done_event():
    """Sparse proxy snapshots inherit text from output_text.done."""
    body = (
        'event: response.output_text.done\n'
        'data: {"type":"response.output_text.done",'
        '"output_index":0,"content_index":0,"text":"recovered"}\n\n'
        'event: response.completed\n'
        'data: {"type":"response.completed","response":'
        '{"status":"completed","output":[],"usage":{}}}\n\n'
    ).encode()

    response = decode_response_body(
        body, 'text/event-stream', WIRE_API_RESPONSES)

    assert response['output_text'] == 'recovered'


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
        response = post_vlm_request(
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
            post_vlm_request(
                endpoint, b'{}', None, 2.0,
                limits.max_response_bytes, lambda _connection: None)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)

    assert 'do-not-expose-this-body' not in str(error.value)
