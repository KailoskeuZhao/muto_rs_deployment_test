"""Protocol validation and OpenAI-compatible HTTP transport for VLM requests."""

import base64
from dataclasses import dataclass
import http.client
import json
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple
from urllib.parse import urlsplit


TYPE_TEXT = 1
TYPE_JPEG = 2


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
    """Parsed HTTP endpoint for an OpenAI-compatible chat-completions call."""

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


def parse_endpoint(base_url: str) -> Endpoint:
    """Turn a base URL into an explicit chat-completions HTTP endpoint."""
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
    if not path.endswith('/chat/completions'):
        path += '/chat/completions'
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


def build_request_body(
        encoded: EncodedContent,
        model: str,
        system_prompt: str,
        max_tokens: int,
        temperature: float,
        max_request_bytes: int) -> bytes:
    """Build a bounded OpenAI-compatible chat-completions JSON request."""
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

    messages: List[Dict[str, Any]] = []
    if system_prompt:
        messages.append({'role': 'system', 'content': system_prompt})
    messages.append({'role': 'user', 'content': encoded.parts})
    payload: Dict[str, Any] = {
        'model': selected_model,
        'messages': messages,
        'stream': False,
    }
    if max_tokens > 0:
        payload['max_tokens'] = max_tokens
    if temperature >= 0.0:
        payload['temperature'] = temperature

    body = json.dumps(
        payload, ensure_ascii=False, separators=(',', ':')).encode('utf-8')
    if len(body) > max_request_bytes:
        raise ProtocolError('encoded request exceeds max_request_bytes')
    return body


def extract_response(response: Any) -> Tuple[str, int, int]:
    """Extract text and token usage from a chat-completions response."""
    if not isinstance(response, dict):
        raise ProtocolError('VLM response root is not an object')
    choices = response.get('choices')
    if not isinstance(choices, list) or not choices:
        raise ProtocolError('VLM response has no choices')
    first_choice = choices[0]
    if not isinstance(first_choice, dict):
        raise ProtocolError('VLM response choice is not an object')
    message = first_choice.get('message')
    if not isinstance(message, dict):
        raise ProtocolError('VLM response choice has no message')
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


def _nonnegative_int(value: Any) -> int:
    """Convert optional token usage to a safe ROS uint32-compatible value."""
    try:
        result = int(value)
    except (TypeError, ValueError):
        return 0
    return min(max(result, 0), 0xffffffff)


def post_chat_completion(
        endpoint: Endpoint,
        body: bytes,
        api_key: Optional[str],
        timeout: float,
        max_response_bytes: int,
        connection_callback: Callable[[Optional[http.client.HTTPConnection]],
                                      None]) -> Any:
    """POST one bounded request and return its decoded JSON response."""
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
        try:
            return json.loads(response_body.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise TransportError('VLM endpoint returned invalid JSON') from error
    except (OSError, http.client.HTTPException) as error:
        raise TransportError(
            f'VLM endpoint connection failed: {type(error).__name__}') from error
    finally:
        connection_callback(None)
        connection.close()
