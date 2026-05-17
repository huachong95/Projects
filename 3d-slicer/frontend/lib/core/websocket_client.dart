import 'dart:async';
import 'dart:convert';

import 'package:web_socket_channel/web_socket_channel.dart';

import 'api_client.dart';

class WebSocketClient {
  final Map<String, StreamController<Map<String, dynamic>>> _controllers = {};
  final Map<String, WebSocketChannel?> _channels = {};
  final Map<String, Timer?> _reconnectTimers = {};

  Stream<Map<String, dynamic>> stream(String channel) {
    _controllers.putIfAbsent(channel, () {
      final ctrl = StreamController<Map<String, dynamic>>.broadcast();
      _connect(channel);
      return ctrl;
    });
    return _controllers[channel]!.stream;
  }

  void _connect(String channel) {
    final wsUrl = apiClient.baseUrl
        .replaceFirst('http://', 'ws://')
        .replaceFirst('https://', 'wss://')
        .replaceAll(RegExp(r'/+$'), '');
    final uri = Uri.parse('$wsUrl/ws/$channel');

    try {
      final ws = WebSocketChannel.connect(uri);
      _channels[channel] = ws;

      ws.stream.listen(
        (data) {
          final msg = jsonDecode(data as String) as Map<String, dynamic>;
          _controllers[channel]?.add(msg);
        },
        onDone: () => _scheduleReconnect(channel),
        onError: (_) => _scheduleReconnect(channel),
      );
    } catch (_) {
      _scheduleReconnect(channel);
    }
  }

  void _scheduleReconnect(String channel, [int delaySeconds = 3]) {
    _reconnectTimers[channel]?.cancel();
    _reconnectTimers[channel] = Timer(
      Duration(seconds: delaySeconds),
      () => _connect(channel),
    );
  }

  void dispose(String channel) {
    _reconnectTimers[channel]?.cancel();
    _channels[channel]?.sink.close();
    _controllers[channel]?.close();
    _channels.remove(channel);
    _controllers.remove(channel);
    _reconnectTimers.remove(channel);
  }
}

final wsClient = WebSocketClient();
