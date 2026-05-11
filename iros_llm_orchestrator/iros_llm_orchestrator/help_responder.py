"""CLI bridge for /llm/help_request and /llm/help_response.

When a BT leaf returns decision="ask_user" through the channel-1 LLM, the
leaf publishes a help request on /llm/help_request and stays RUNNING
until /llm/help_response answers with the same request_id. This node is
the minimum-viable operator side of that loop — it prints incoming
requests on stdout and blocks on stdin for the reply.

Topic payloads are JSON over std_msgs/String to avoid a new message type
for what is currently a developer-facing convenience. The RViz panel will
gain a richer wrapper around the same topics in a follow-up.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
from queue import Queue
from typing import Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String


class HelpResponder(Node):
    """Subscribes to /llm/help_request, publishes /llm/help_response."""

    def __init__(self, *, auto_answer: Optional[str] = None,
                 auto_abort: bool = False):
        super().__init__('llm_help_responder')

        self._auto_answer = auto_answer
        self._auto_abort  = auto_abort

        req_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        rsp_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub = self.create_publisher(String, '/llm/help_response', rsp_qos)
        self._sub = self.create_subscription(
            String, '/llm/help_request', self._on_request, req_qos)

        self._queue: Queue[dict] = Queue()
        self._seen: set[int] = set()

        self.get_logger().info(
            'HelpResponder ready — listening on /llm/help_request' +
            (f', auto_answer={auto_answer!r}' if auto_answer is not None else '') +
            (' [AUTO_ABORT]' if auto_abort else ''))

    def _on_request(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'malformed help_request: {exc}')
            return
        request_id = int(payload.get('request_id', 0) or 0)
        if request_id == 0 or request_id in self._seen:
            return
        self._seen.add(request_id)
        self._queue.put(payload)

    def answer(self, request_id: int, text: str, abort: bool = False) -> None:
        msg = String()
        msg.data = json.dumps({
            'request_id': int(request_id),
            'answer':     str(text),
            'abort':      bool(abort),
        }, ensure_ascii=False)
        self._pub.publish(msg)


def _format_request(payload: dict) -> str:
    rid = payload.get('request_id', '?')
    leaf = payload.get('leaf', '?')
    question = payload.get('question', '')
    context_json = payload.get('context_json', '')
    try:
        context = json.dumps(
            json.loads(context_json), ensure_ascii=False, indent=2)
    except (json.JSONDecodeError, TypeError):
        context = context_json or '(none)'
    return (
        f'\n=== LLM needs help (request #{rid}, leaf={leaf}) ===\n'
        f'Question: {question}\n'
        f'Context:\n{context}\n'
        f'---  reply with a hint, or "abort" to give up  ---'
    )


def _interactive_loop(node: HelpResponder) -> int:
    while rclpy.ok():
        try:
            payload = node._queue.get(timeout=0.25)
        except Exception:
            continue
        print(_format_request(payload), flush=True)
        if node._auto_answer is not None:
            print(f'[auto-answer] {node._auto_answer!r}', flush=True)
            node.answer(payload['request_id'], node._auto_answer,
                        abort=node._auto_abort)
            continue
        try:
            line = input('> ').strip()
        except EOFError:
            return 0
        if line.lower() in ('abort', 'cancel', 'stop', 'quit', 'q'):
            node.answer(payload['request_id'], line, abort=True)
            print('[sent abort]', flush=True)
            continue
        node.answer(payload['request_id'], line, abort=False)
        print(f'[sent answer]', flush=True)
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--auto-answer',
        help='Reply to every request with this string (non-interactive)')
    parser.add_argument(
        '--auto-abort', action='store_true',
        help='With --auto-answer, also flag the response as abort=true')
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    node = HelpResponder(
        auto_answer=args.auto_answer, auto_abort=args.auto_abort)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        rc = _interactive_loop(node)
    except KeyboardInterrupt:
        rc = 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
