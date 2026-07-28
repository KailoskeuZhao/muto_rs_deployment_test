"""Protocol validation and OpenAI-compatible HTTP transport for VLM requests."""

import base64
from dataclasses import dataclass
import http.client
import json
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple
from urllib.parse import urlsplit


TYPE_TEXT = 1
TYPE_JPEG = 2
WIRE_API_CHAT_COMPLETIONS = 'chat_completions'
WIRE_API_RESPONSES = 'responses'
SUPPORTED_WIRE_APIS = {
    WIRE_API_CHAT_COMPLETIONS,
    WIRE_API_RESPONSES,
}


class ProtocolError(ValueError):
    """Raised when a request or response violates the VLM protocol contract."""


class TransportError(RuntimeError):
    """Raised for bounded, credential-safe HTTP transport failures."""


@dataclass(frozen=True)
class ProtocolLimits:
    """Limits applied before a request reaches the network."""

    max_content_parts: int
    max_text_characters: int
    max_jpeg_bytes: int
    max_total_jpeg_bytes: int
    max_request_bytes: int
    max_response_bytes: int


@dataclass(frozen=True)
class Endpoint:
    """Parsed endpoint for one selected OpenAI-compatible API call."""

    scheme: str
    host: str
    port: Optional[int]
    target: str


@dataclass(frozen=True)
class EncodedContent:
    """Validated OpenAI content and non-sensitive request statistics."""

    parts: List[Dict[str, Any]]
    text_characters: int
    jpeg_bytes: int


def validate_wire_api(wire_api: str) -> str:
    """Return a supported API name or reject it before network access."""
    if wire_api not in SUPPORTED_WIRE_APIS:
        raise ProtocolError(
            'wire_api must be chat_completions or responses')
    return wire_api


def parse_endpoint(
        base_url: str,
        wire_api: str = WIRE_API_CHAT_COMPLETIONS) -> Endpoint:
    """Turn a base URL into the selected OpenAI-compatible endpoint."""
    validate_wire_api(wire_api)
    if not isinstance(base_url, str) or not base_url.strip():
        raise ProtocolError('base_url must not be empty')
    parsed = urlsplit(base_url.strip())
    if parsed.scheme not in ('http', 'https'):
        raise ProtocolError('base_url scheme must be http or https')
    if parsed.hostname is None:
        raise ProtocolError('base_url must include a host')
    if parsed.username is not None or parsed.password is not None:
        raise ProtocolError('credentials must not be embedded in base_url')
    if parsed.query or parsed.fragment:
        raise ProtocolError('base_url must not include a query or fragment')
    try:
        port = parsed.port
    except ValueError as error:
        raise ProtocolError('base_url contains an invalid port') from error

    path = parsed.path.rstrip('/')
    expected_suffix = (
        '/responses'
        if wire_api == WIRE_API_RESPONSES
        else '/chat/completions'
    )
    known_suffixes = ('/responses', '/chat/completions')
    if path.endswith(known_suffixes) and not path.endswith(expected_suffix):
        raise ProtocolError('base_url endpoint conflicts with wire_api')
    if not path.endswith(expected_suffix):
        path += expected_suffix
    if not path.startswith('/'):
        path = '/' + path
    return Endpoint(parsed.scheme, parsed.hostname, port, path)


def validate_limits(limits: ProtocolLimits) -> None:
    """Reject nonsensical resource limits during node startup."""
    for field_name, value in limits.__dict__.items():
        if not isinstance(value, int) or value <= 0:
            raise ProtocolError(f'{field_name} must be a positive integer')
    if limits.max_total_jpeg_bytes < limits.max_jpeg_bytes:
        raise ProtocolError(
            'max_total_jpeg_bytes must be at least max_jpeg_bytes')


def encode_content(
        content: Sequence[Any], limits: ProtocolLimits) -> EncodedContent:
    """Validate ordered ROS content parts and encode JPEGs as data URLs."""
    validate_limits(limits)
    if not content:
        raise ProtocolError('at least one content part is required')
    if len(content) > limits.max_content_parts:
        raise ProtocolError(
            f'content has {len(content)} parts; limit is '
            f'{limits.max_content_parts}')

    encoded_parts: List[Dict[str, Any]] = []
    total_text_characters = 0
    total_jpeg_bytes = 0
    for index, part in enumerate(content):
        part_type = int(part.type)
        text = str(part.text)
        jpeg_data = bytes(part.jpeg_data)
        if part_type == TYPE_TEXT:
            if not text.strip():
                raise ProtocolError(f'content[{index}] text is empty')
            if jpeg_data:
                raise ProtocolError(
                    f'content[{index}] is text but also contains JPEG bytes')
            total_text_characters += len(text)
            if total_text_characters > limits.max_text_characters:
                raise ProtocolError(
                    'combined text exceeds max_text_characters')
            encoded_parts.append({'type': 'text', 'text': text})
            continue

        if part_type == TYPE_JPEG:
            if text:
                raise ProtocolError(
                    f'content[{index}] is JPEG but also contains text')
            if len(jpeg_data) > limits.max_jpeg_bytes:
                raise ProtocolError(
                    f'content[{index}] JPEG exceeds max_jpeg_bytes')
            if len(jpeg_data) < 4 or not jpeg_data.startswith(b'\xff\xd8') or \
                    not jpeg_data.endswith(b'\xff\xd9'):
                raise ProtocolError(
                    f'content[{index}] is not a complete JPEG byte stream')
            total_jpeg_bytes += len(jpeg_data)
            if total_jpeg_bytes > limits.max_total_jpeg_bytes:
                raise ProtocolError(
                    'combined JPEG data exceeds max_total_jpeg_bytes')
            encoded = base64.b64encode(jpeg_data).decode('ascii')
            encoded_parts.append({
                'type': 'image_url',
                'image_url': {
                    'url': f'data:image/jpeg;base64,{encoded}',
                },
            })
            continue

        raise ProtocolError(
            f'content[{index}] has unsupported type {part_type}')

    return EncodedContent(
        encoded_parts, total_text_characters, total_jpeg_bytes)


def decode_response_json_schema(
        schema_text: str, max_characters: int) -> Optional[Dict[str, Any]]:
    """Decode an optional bounded root-object JSON Schema from a ROS goal."""
    if not isinstance(schema_text, str):
        raise ProtocolError('response_json_schema must be a string')
    if max_characters <= 0:
        raise ProtocolError('schema character limit must be positive')
    schema_text = schema_text.strip()
    if not schema_text:
        return None
    if len(schema_text) > max_characters:
        raise ProtocolError('response_json_schema exceeds its size limit')
    try:
        schema = json.loads(schema_text)
    except json.JSONDecodeError as error:
        raise ProtocolError(
            'response_json_schema is not valid JSON') from error
    if not isinstance(schema, dict) or schema.get('type') != 'object':
        raise ProtocolError(
            'response_json_schema must describe a root object')
    return schema


def build_request_body(
        encoded: EncodedContent,
        model: str,
        system_prompt: str,
        max_tokens: int,
        temperature: float,
        max_request_bytes: int,
        wire_api: str = WIRE_API_CHAT_COMPLETIONS,
        store_response: bool = False,
        response_json_schema: Optional[Dict[str, Any]] = None) -> bytes:
    """Build a bounded request for the selected OpenAI-compatible API."""
    validate_wire_api(wire_api)
    selected_model = model.strip()
    if not selected_model:
        raise ProtocolError('model must not be empty')
    if len(selected_model) > 256:
        raise ProtocolError('model name is too long')
    if max_tokens < 0:
        raise ProtocolError('max_tokens must be nonnegative')
    if temperature < 0.0 and temperature != -1.0:
        raise ProtocolError('temperature must be -1 or nonnegative')
    if max_request_bytes <= 0:
        raise ProtocolError('max_request_bytes must be positive')

    if not isinstance(store_response, bool):
        raise ProtocolError('store_response must be boolean')
    if response_json_schema is not None and (
            not isinstance(response_json_schema, dict) or
            response_json_schema.get('type') != 'object'):
        raise ProtocolError(
            'response_json_schema must describe a root object')

    if wire_api == WIRE_API_RESPONSES:
        response_parts = []
        for part in encoded.parts:
            if part['type'] == 'text':
                response_parts.append({
                    'type': 'input_text',
                    'text': part['text'],
                })
            else:
                response_parts.append({
                    'type': 'input_image',
                    'image_url': part['image_url']['url'],
                })
        payload = {
            'model': selected_model,
            'input': [{
                'role': 'user',
                'content': response_parts,
            }],
            'stream': False,
            'store': store_response,
        }
        if system_prompt:
            payload['instructions'] = system_prompt
        if max_tokens > 0:
            payload['max_output_tokens'] = max_tokens
        if response_json_schema is not None:
            payload['text'] = {
                'format': {
                    'type': 'json_schema',
                    'name': 'ros_vlm_response',
                    'schema': response_json_schema,
                    'strict': True,
                },
            }
    else:
        messages: List[Dict[str, Any]] = []
        if system_prompt:
            messages.append({'role': 'system', 'content': system_prompt})
        messages.append({'role': 'user', 'content': encoded.parts})
        payload = {
            'model': selected_model,
            'messages': messages,
            'stream': False,
        }
        if max_tokens > 0:
            payload['max_tokens'] = max_tokens
        if response_json_schema is not None:
            payload['response_format'] = {
                'type': 'json_schema',
                'json_schema': {
                    'name': 'ros_vlm_response',
                    'schema': response_json_schema,
                    'strict': True,
                },
            }
    if temperature >= 0.0:
        payload['temperature'] = temperature

    body = json.dumps(
        payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    if len(body) > max_request_bytes:
        raise ProtocolError('encoded request exceeds max_request_bytes')
    return body


def extract_response(
        response: Any,
        wire_api: str = WIRE_API_CHAT_COMPLETIONS) -> Tuple[str, int, int]:
    """Extract text and usage from the selected API response."""
    validate_wire_api(wire_api)
    if not isinstance(response, dict):
        raise ProtocolError('VLM response root is not an object')

    if wire_api == WIRE_API_RESPONSES:
        return _extract_responses_response(response)

    choices = response.get('choices')
    if not isinstance(choices, list) or not choices:
        raise ProtocolError('VLM response has no choices')
    first_choice = choices[0]
    if not isinstance(first_choice, dict):
        raise ProtocolError('VLM response choice is not an object')
    finish_reason = first_choice.get('finish_reason')
    if finish_reason is not None and finish_reason != 'stop':
        if finish_reason == 'length':
            raise ProtocolError('VLM response is incomplete')
        if finish_reason == 'content_filter':
            raise ProtocolError('VLM response was filtered')
        raise ProtocolError('VLM response did not finish successfully')
    message = first_choice.get('message')
    if not isinstance(message, dict):
        raise ProtocolError('VLM response choice has no message')
    if message.get('refusal'):
        raise ProtocolError('VLM response was refused')
    content = message.get('content')

    if isinstance(content, str):
        response_text = content
    elif isinstance(content, list):
        text_parts = []
        for part in content:
            if isinstance(part, str):
                text_parts.append(part)
            elif isinstance(part, dict) and isinstance(part.get('text'), str):
                text_parts.append(part['text'])
        if not text_parts:
            raise ProtocolError('VLM response content has no text')
        response_text = ''.join(text_parts)
    else:
        raise ProtocolError('VLM response content is not text')

    usage = response.get('usage', {})
    if not isinstance(usage, dict):
        usage = {}
    prompt_tokens = _nonnegative_int(usage.get('prompt_tokens', 0))
    completion_tokens = _nonnegative_int(usage.get('completion_tokens', 0))
    return response_text, prompt_tokens, completion_tokens


def _extract_responses_response(response: Dict[str, Any]) \
        -> Tuple[str, int, int]:
    """Extract output text from a raw Responses API response object."""
    status = response.get('status')
    if status is not None and status != 'completed':
        if status == 'incomplete':
            raise ProtocolError('VLM Responses output is incomplete')
        if status == 'failed':
            raise ProtocolError('VLM Responses output failed')
        raise ProtocolError('VLM Responses output did not complete')

    output = response.get('output')
    if isinstance(output, list):
        for item in output:
            if not isinstance(item, dict):
                continue
            content = item.get('content')
            if not isinstance(content, list):
                continue
            if any(
                    isinstance(part, dict) and
                    part.get('type') == 'refusal'
                    for part in content):
                raise ProtocolError('VLM Responses output was refused')

    response_text = response.get('output_text')
    if not isinstance(response_text, str) or not response_text:
        text_parts = []
        if not isinstance(output, list):
            raise ProtocolError('VLM Responses output is not an array')
        for item in output:
            if not isinstance(item, dict) or item.get('type') != 'message':
                continue
            content = item.get('content')
            if not isinstance(content, list):
                continue
            for part in content:
                if not isinstance(part, dict):
                    continue
                if part.get('type') != 'output_text':
                    continue
                text = part.get('text')
                if isinstance(text, str):
                    text_parts.append(text)
        if not text_parts:
            raise ProtocolError('VLM Responses output has no text')
        response_text = ''.join(text_parts)

    usage = response.get('usage', {})
    if not isinstance(usage, dict):
        usage = {}
    input_tokens = _nonnegative_int(usage.get('input_tokens', 0))
    output_tokens = _nonnegative_int(usage.get('output_tokens', 0))
    return response_text, input_tokens, output_tokens


def _nonnegative_int(value: Any) -> int:
    """Convert optional token usage to a safe ROS uint32-compatible value."""
    try:
        result = int(value)
    except (TypeError, ValueError):
        return 0
    return min(max(result, 0), 0xffffffff)


def decode_response_body(
        response_body: bytes,
        content_type: str,
        wire_api: str) -> Any:
    """Decode either a JSON body or a bounded Responses SSE envelope."""
    validate_wire_api(wire_api)
    try:
        decoded = response_body.decode('utf-8')
    except UnicodeDecodeError as error:
        raise TransportError(
            'VLM endpoint returned invalid UTF-8') from error

    media_type = content_type.split(';', 1)[0].strip().lower()
    if media_type == 'text/event-stream':
        if wire_api != WIRE_API_RESPONSES:
            raise TransportError(
                'streaming Chat Completions responses are unsupported')
        return _extract_completed_sse_response(decoded)

    try:
        return json.loads(decoded)
    except json.JSONDecodeError as error:
        raise TransportError('VLM endpoint returned invalid JSON') from error


def _extract_completed_sse_response(event_stream: str) -> Dict[str, Any]:
    """Return the response nested in the final response.completed event."""
    normalized = event_stream.replace('\r\n', '\n').replace('\r', '\n')
    completed_response = None
    done_text = {}
    text_deltas = {}
    for block in normalized.split('\n\n'):
        event_name = ''
        data_lines = []
        for line in block.splitlines():
            if line.startswith('event:'):
                event_name = line[6:].strip()
            elif line.startswith('data:'):
                data_lines.append(line[5:].lstrip())
        if not data_lines:
            continue
        data = '\n'.join(data_lines)
        if data == '[DONE]':
            continue
        try:
            payload = json.loads(data)
        except json.JSONDecodeError as error:
            raise TransportError(
                'VLM endpoint returned malformed event data') from error
        if not isinstance(payload, dict):
            continue
        event_type = payload.get('type', event_name)
        if event_type in ('error', 'response.failed', 'response.incomplete'):
            raise TransportError('VLM Responses stream reported failure')
        if event_type == 'response.output_text.delta':
            key = (
                _nonnegative_int(payload.get('output_index', 0)),
                _nonnegative_int(payload.get('content_index', 0)),
            )
            delta = payload.get('delta')
            if isinstance(delta, str):
                text_deltas.setdefault(key, []).append(delta)
        if event_type == 'response.output_text.done':
            key = (
                _nonnegative_int(payload.get('output_index', 0)),
                _nonnegative_int(payload.get('content_index', 0)),
            )
            text = payload.get('text')
            if isinstance(text, str):
                done_text[key] = text
        if event_type == 'response.completed':
            response = payload.get('response')
            if not isinstance(response, dict):
                raise TransportError(
                    'VLM completed event has no response object')
            completed_response = response
    if completed_response is None:
        raise TransportError(
            'VLM Responses stream ended without a completed response')

    output = completed_response.get('output')
    has_output = isinstance(output, list) and bool(output)
    has_output_text = isinstance(
        completed_response.get('output_text'), str)
    if not has_output and not has_output_text:
        if done_text:
            completed_response['output_text'] = ''.join(
                done_text[key] for key in sorted(done_text))
        elif text_deltas:
            completed_response['output_text'] = ''.join(
                ''.join(text_deltas[key]) for key in sorted(text_deltas))
    return completed_response


def post_vlm_request(
        endpoint: Endpoint,
        body: bytes,
        api_key: Optional[str],
        timeout: float,
        max_response_bytes: int,
        connection_callback: Callable[[Optional[http.client.HTTPConnection]],
                                      None],
        wire_api: str = WIRE_API_CHAT_COMPLETIONS) -> Any:
    """POST one bounded request and return its decoded JSON response."""
    validate_wire_api(wire_api)
    if timeout <= 0.0:
        raise ProtocolError('request timeout must be positive')
    if max_response_bytes <= 0:
        raise ProtocolError('max_response_bytes must be positive')

    connection_type = (
        http.client.HTTPSConnection
        if endpoint.scheme == 'https'
        else http.client.HTTPConnection
    )
    connection = connection_type(
        endpoint.host, port=endpoint.port, timeout=timeout)
    connection_callback(connection)
    headers = {
        'Accept': 'application/json',
        'Content-Type': 'application/json',
        'User-Agent': 'muto_vlm_socket/0.1',
    }
    if api_key:
        headers['Authorization'] = f'Bearer {api_key}'

    try:
        connection.request('POST', endpoint.target, body=body, headers=headers)
        http_response = connection.getresponse()
        response_body = http_response.read(max_response_bytes + 1)
        if len(response_body) > max_response_bytes:
            raise TransportError('VLM response exceeds max_response_bytes')
        if not 200 <= http_response.status < 300:
            reason = str(http_response.reason).replace('\n', ' ')[:120]
            raise TransportError(
                f'VLM endpoint returned HTTP {http_response.status} {reason}')
        return decode_response_body(
            response_body,
            http_response.getheader('Content-Type', ''),
            wire_api,
        )
    except (OSError, http.client.HTTPException) as error:
        raise TransportError(
            f'VLM endpoint connection failed: {type(error).__name__}') from error
    finally:
        connection_callback(None)
        connection.close()
