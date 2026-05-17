import 'dart:async';
import 'dart:typed_data';

import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';
import '../../core/websocket_client.dart';

class PrintMonitorScreen extends StatefulWidget {
  const PrintMonitorScreen({super.key});

  @override
  State<PrintMonitorScreen> createState() => _PrintMonitorScreenState();
}

class _PrintMonitorScreenState extends State<PrintMonitorScreen> {
  Map<String, dynamic> _status = {};
  Map<String, dynamic>? _aiStatus;
  bool _aiAlertVisible = false;
  Uint8List? _cameraFrame;
  Timer? _cameraTimer;
  StreamSubscription? _statusSub;
  StreamSubscription? _aiSub;

  @override
  void initState() {
    super.initState();
    _statusSub = wsClient.stream('print_status').listen(_onStatus);
    _aiSub = wsClient.stream('ai_status').listen(_onAiStatus);
    _startCameraPolling();
  }

  void _onStatus(Map<String, dynamic> msg) {
    if (mounted && msg['type'] == 'status') {
      setState(() => _status = msg['data'] as Map<String, dynamic>? ?? {});
    }
  }

  void _onAiStatus(Map<String, dynamic> msg) {
    if (!mounted) return;
    if (msg['type'] == 'alert') {
      setState(() {
        _aiStatus = msg['data'] as Map<String, dynamic>?;
        _aiAlertVisible = true;
      });
    } else if (msg['type'] == 'detection') {
      setState(() => _aiStatus = msg['data'] as Map<String, dynamic>?);
    }
  }

  void _startCameraPolling() {
    _cameraTimer = Timer.periodic(const Duration(seconds: 1), (_) async {
      try {
        final resp = await apiClient.getBytes('/api/monitoring/camera/snapshot');
        if (resp.statusCode == 200 && resp.data != null && mounted) {
          setState(() => _cameraFrame = Uint8List.fromList(resp.data!));
        }
      } catch (_) {}
    });
  }

  @override
  void dispose() {
    _cameraTimer?.cancel();
    _statusSub?.cancel();
    _aiSub?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final progress = (_status['progress_percent'] as num?)?.toDouble() ?? 0.0;
    final tempHotend = (_status['temp_hotend'] as num?)?.toDouble() ?? 0.0;
    final tempBed = (_status['temp_bed'] as num?)?.toDouble() ?? 0.0;
    final state = _status['state']?.toString() ?? 'Unknown';
    final layer = _status['current_layer'] as int?;
    final totalLayers = _status['total_layers'] as int?;

    return Scaffold(
      appBar: AppBar(
        title: const Text('Print Monitor'),
        actions: [
          IconButton(
            icon: const Icon(Icons.videocam),
            tooltip: 'Timelapses',
            onPressed: () => context.push('/timelapse'),
          ),
        ],
      ),
      body: Column(
        children: [
          if (_aiAlertVisible)
            _AiAlertBanner(
              probability: (_aiStatus?['probability'] as num?)?.toDouble() ?? 0.0,
              onDismiss: () => setState(() => _aiAlertVisible = false),
            ),
          Expanded(
            flex: 3,
            child: _CameraFeed(frame: _cameraFrame),
          ),
          Expanded(
            flex: 2,
            child: Padding(
              padding: const EdgeInsets.all(16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  Row(
                    children: [
                      _StatusChip(state),
                      const SizedBox(width: 8),
                      if (layer != null && layer > 0 && totalLayers != null && totalLayers > 0)
                        Text('Layer $layer / $totalLayers'),
                    ],
                  ),
                  const SizedBox(height: 12),
                  LinearProgressIndicator(value: progress / 100),
                  const SizedBox(height: 4),
                  Text('${progress.toStringAsFixed(1)}%'),
                  const SizedBox(height: 16),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceAround,
                    children: [
                      _TempCard('Nozzle', tempHotend,
                          (_status['temp_hotend_target'] as num?)?.toDouble() ?? 0),
                      _TempCard('Bed', tempBed,
                          (_status['temp_bed_target'] as num?)?.toDouble() ?? 0),
                    ],
                  ),
                  const SizedBox(height: 16),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                    children: [
                      OutlinedButton.icon(
                        icon: const Icon(Icons.pause),
                        label: const Text('Pause'),
                        onPressed: () => _sendCommand('/api/printer/pause', 'Paused'),
                      ),
                      OutlinedButton.icon(
                        icon: const Icon(Icons.play_arrow),
                        label: const Text('Resume'),
                        onPressed: () => _sendCommand('/api/printer/resume', 'Resumed'),
                      ),
                      OutlinedButton.icon(
                        icon: const Icon(Icons.stop, color: Colors.red),
                        label: const Text('Cancel', style: TextStyle(color: Colors.red)),
                        onPressed: () => _confirmCancel(context),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }

  Future<void> _sendCommand(String path, String successMsg) async {
    try {
      await apiClient.post(path);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(successMsg)));
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Command failed: $e')),
        );
      }
    }
  }

  Future<void> _confirmCancel(BuildContext context) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text('Cancel print?'),
        content: const Text('This will stop the print job.'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text('No')),
          FilledButton(
            onPressed: () => Navigator.pop(context, true),
            child: const Text('Cancel print'),
          ),
        ],
      ),
    );
    if (confirmed == true) {
      await apiClient.post('/api/printer/cancel');
    }
  }
}

class _CameraFeed extends StatelessWidget {
  final Uint8List? frame;
  const _CameraFeed({this.frame});

  @override
  Widget build(BuildContext context) {
    if (frame == null) {
      return Container(
        color: Colors.black,
        child: const Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              Icon(Icons.videocam_off, size: 48, color: Colors.grey),
              SizedBox(height: 8),
              Text('No camera feed', style: TextStyle(color: Colors.grey)),
            ],
          ),
        ),
      );
    }
    return Image.memory(frame!, fit: BoxFit.contain, gaplessPlayback: true);
  }
}

class _AiAlertBanner extends StatelessWidget {
  final double probability;
  final VoidCallback onDismiss;

  const _AiAlertBanner({required this.probability, required this.onDismiss});

  @override
  Widget build(BuildContext context) => MaterialBanner(
        backgroundColor: Colors.deepOrange.shade900,
        content: Row(
          children: [
            const Icon(Icons.warning_amber_rounded, color: Colors.white),
            const SizedBox(width: 8),
            Text(
              'Possible print failure detected (${(probability * 100).toStringAsFixed(0)}% confidence)',
              style: const TextStyle(color: Colors.white),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: onDismiss,
            child: const Text('Dismiss', style: TextStyle(color: Colors.white70)),
          ),
          TextButton(
            onPressed: () async {
              await apiClient.post('/api/printer/pause');
              onDismiss();
            },
            child: const Text('Pause Print', style: TextStyle(color: Colors.white)),
          ),
        ],
      );
}

class _StatusChip extends StatelessWidget {
  final String state;
  const _StatusChip(this.state);

  @override
  Widget build(BuildContext context) {
    Color color;
    switch (state.toUpperCase()) {
      case 'PRINTING':
        color = Colors.greenAccent;
      case 'PAUSED':
        color = Colors.orangeAccent;
      case 'ERROR':
        color = Colors.redAccent;
      default:
        color = Colors.grey;
    }
    return Chip(
      avatar: Icon(Icons.circle, size: 10, color: color),
      label: Text(state),
      padding: EdgeInsets.zero,
    );
  }
}

class _TempCard extends StatelessWidget {
  final String label;
  final double current;
  final double target;
  const _TempCard(this.label, this.current, this.target);

  @override
  Widget build(BuildContext context) => Card(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 12),
          child: Column(
            children: [
              Text(label, style: Theme.of(context).textTheme.labelMedium),
              Text(
                '${current.toStringAsFixed(0)}°',
                style: Theme.of(context).textTheme.headlineSmall,
              ),
              Text('/ ${target.toStringAsFixed(0)}°',
                  style: Theme.of(context).textTheme.bodySmall),
            ],
          ),
        ),
      );
}
