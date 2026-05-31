import 'dart:async';
import 'dart:typed_data';

import 'package:dio/dio.dart';
import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';
import '../../core/websocket_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/pulsing_dot.dart';
import '../../shared/widgets/temp_gauge.dart';
import '../../theme/app_theme.dart';

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
        final resp = await apiClient.get<List<int>>('/api/monitoring/camera/snapshot');
        if (resp.statusCode == 200 && resp.data != null && mounted) {
          setState(() => _cameraFrame = Uint8List.fromList(resp.data!));
        }
      } catch (_) {}
    });
  }

  Future<void> _sendCommand(String path, String successMsg) async {
    try {
      await apiClient.post(path);
      _showSnack(successMsg);
    } catch (e) {
      _showSnack(_errorText(e));
    }
  }

  String _errorText(Object e) {
    if (e is DioException) {
      final detail = e.response?.data;
      if (detail is Map && detail['detail'] != null) {
        return detail['detail'].toString();
      }
      return 'Request failed (${e.response?.statusCode ?? 'no response'})';
    }
    return 'Request failed';
  }

  void _showSnack(String msg) {
    if (mounted) {
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
    }
  }

  @override
  void dispose() {
    _cameraTimer?.cancel();
    _statusSub?.cancel();
    _aiSub?.cancel();
    super.dispose();
  }

  String _eta(int? seconds) {
    if (seconds == null || seconds <= 0) return '—';
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return '${h}h ${m}m';
    return '${m}m';
  }

  @override
  Widget build(BuildContext context) {
    final progress = (_status['progress_percent'] as num?)?.toDouble() ?? 0.0;
    final tempHotend = (_status['temp_hotend'] as num?)?.toDouble() ?? 0.0;
    final tempBed = (_status['temp_bed'] as num?)?.toDouble() ?? 0.0;
    final state = _status['state']?.toString() ?? 'Unknown';
    final filename = _status['filename']?.toString();
    final eta = _status['eta_seconds'] as int?;
    final printing = state.toUpperCase() == 'PRINTING';

    return Scaffold(
      appBar: AppBar(
        title: const Text('Print Monitor'),
        actions: [
          IconButton(
            icon: const Icon(Icons.movie_outlined),
            tooltip: 'Timelapses',
            onPressed: () => context.push('/timelapse'),
          ),
          const SizedBox(width: 4),
        ],
      ),
      body: AnimatedBackground(
        child: SafeArea(
          top: false,
          child: Column(
            children: [
              if (_aiAlertVisible)
                _AiAlertBanner(
                  probability: (_aiStatus?['probability'] as num?)?.toDouble() ?? 0.0,
                  onDismiss: () => setState(() => _aiAlertVisible = false),
                  onPause: () => _sendCommand('/api/printer/pause', 'Print paused'),
                ),
              Expanded(
                child: SingleChildScrollView(
                  padding: const EdgeInsets.fromLTRB(20, 12, 20, 24),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      _CameraFeed(frame: _cameraFrame, live: printing),
                      const SizedBox(height: AppSpace.md),
                      _StatusBar(state: state, filename: filename, eta: _eta(eta)),
                      const SizedBox(height: AppSpace.md),
                      _ProgressCard(progress: progress),
                      const SizedBox(height: AppSpace.md),
                      _GaugeCard(hotend: tempHotend, bed: tempBed, status: _status),
                      const SizedBox(height: AppSpace.lg),
                      _Controls(
                        onPause: () => _sendCommand('/api/printer/pause', 'Print paused'),
                        onResume: () => _sendCommand('/api/printer/resume', 'Print resumed'),
                        onCancel: () => _confirmCancel(context),
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  Future<void> _confirmCancel(BuildContext context) async {
    final confirmed = await showDialog<bool>(
      context: context,
      builder: (_) => AlertDialog(
        title: const Text('Cancel print?'),
        content: const Text('This will stop the print job. It cannot be undone.'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text('Keep printing')),
          FilledButton(
            style: FilledButton.styleFrom(backgroundColor: AppColors.danger),
            onPressed: () => Navigator.pop(context, true),
            child: const Text('Cancel print'),
          ),
        ],
      ),
    );
    if (confirmed == true) {
      await _sendCommand('/api/printer/cancel', 'Print cancelled');
    }
  }
}

class _CameraFeed extends StatelessWidget {
  final Uint8List? frame;
  final bool live;
  const _CameraFeed({this.frame, required this.live});

  @override
  Widget build(BuildContext context) {
    return AspectRatio(
      aspectRatio: 16 / 10,
      child: Container(
        decoration: BoxDecoration(
          color: Colors.black,
          borderRadius: BorderRadius.circular(AppRadius.md),
          border: Border.all(color: AppColors.surfaceBorder),
        ),
        clipBehavior: Clip.antiAlias,
        child: Stack(
          fit: StackFit.expand,
          children: [
            AnimatedSwitcher(
              duration: AppMotion.med,
              child: frame == null
                  ? const _NoFeed()
                  : Image.memory(frame!,
                      key: ValueKey(frame!.length),
                      fit: BoxFit.contain,
                      gaplessPlayback: true),
            ),
            if (frame != null && live)
              Positioned(
                top: 12,
                left: 12,
                child: Container(
                  padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 5),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.55),
                    borderRadius: BorderRadius.circular(AppRadius.pill),
                  ),
                  child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: const [
                      PulsingDot(color: AppColors.danger, size: 7),
                      Text('LIVE',
                          style: TextStyle(
                              fontSize: 11,
                              fontWeight: FontWeight.w800,
                              letterSpacing: 1)),
                    ],
                  ),
                ),
              ),
          ],
        ),
      ),
    );
  }
}

class _NoFeed extends StatelessWidget {
  const _NoFeed();
  @override
  Widget build(BuildContext context) => const Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(Icons.videocam_off_outlined, size: 44, color: AppColors.onSurfaceDim),
            SizedBox(height: 8),
            Text('Waiting for camera feed…',
                style: TextStyle(color: AppColors.onSurfaceDim)),
          ],
        ),
      );
}

class _StatusBar extends StatelessWidget {
  final String state;
  final String? filename;
  final String eta;
  const _StatusBar({required this.state, this.filename, required this.eta});

  @override
  Widget build(BuildContext context) {
    return _Panel(
      child: Row(
        children: [
          _StateChip(state),
          const SizedBox(width: AppSpace.md),
          Expanded(
            child: Text(
              filename ?? 'No active job',
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: const TextStyle(fontWeight: FontWeight.w600),
            ),
          ),
          const Icon(Icons.schedule, size: 16, color: AppColors.onSurfaceDim),
          const SizedBox(width: 6),
          Text(eta, style: const TextStyle(fontWeight: FontWeight.w700)),
        ],
      ),
    );
  }
}

class _ProgressCard extends StatelessWidget {
  final double progress;
  const _ProgressCard({required this.progress});

  @override
  Widget build(BuildContext context) {
    return _Panel(
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              const Text('Progress',
                  style: TextStyle(color: AppColors.onSurfaceDim, fontWeight: FontWeight.w600)),
              TweenAnimationBuilder<double>(
                tween: Tween(begin: 0, end: progress),
                duration: AppMotion.slow,
                curve: AppMotion.curve,
                builder: (_, v, __) => Text('${v.toStringAsFixed(1)}%',
                    style: const TextStyle(
                        fontSize: 20, fontWeight: FontWeight.w800, letterSpacing: -0.5)),
              ),
            ],
          ),
          const SizedBox(height: 10),
          TweenAnimationBuilder<double>(
            tween: Tween(begin: 0, end: (progress / 100).clamp(0.0, 1.0)),
            duration: AppMotion.slow,
            curve: AppMotion.curve,
            builder: (_, v, __) => ClipRRect(
              borderRadius: BorderRadius.circular(AppRadius.pill),
              child: Stack(
                children: [
                  Container(height: 10, color: AppColors.surfaceHigh),
                  FractionallySizedBox(
                    widthFactor: v,
                    child: Container(
                      height: 10,
                      decoration: const BoxDecoration(
                        gradient: LinearGradient(
                          colors: [AppColors.primary, AppColors.primaryBright],
                        ),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

class _GaugeCard extends StatelessWidget {
  final double hotend;
  final double bed;
  final Map<String, dynamic> status;
  const _GaugeCard({required this.hotend, required this.bed, required this.status});

  @override
  Widget build(BuildContext context) {
    return _Panel(
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: [
          TempGauge(
            label: 'NOZZLE',
            icon: Icons.local_fire_department,
            current: hotend,
            target: (status['temp_hotend_target'] as num?)?.toDouble() ?? 0,
            color: AppColors.hot,
            max: 300,
          ),
          TempGauge(
            label: 'BED',
            icon: Icons.layers,
            current: bed,
            target: (status['temp_bed_target'] as num?)?.toDouble() ?? 0,
            color: AppColors.cool,
            max: 120,
          ),
        ],
      ),
    );
  }
}

class _Controls extends StatelessWidget {
  final VoidCallback onPause;
  final VoidCallback onResume;
  final VoidCallback onCancel;
  const _Controls({required this.onPause, required this.onResume, required this.onCancel});

  @override
  Widget build(BuildContext context) {
    return Row(
      children: [
        Expanded(
          child: OutlinedButton.icon(
            icon: const Icon(Icons.pause_rounded),
            label: const Text('Pause'),
            onPressed: onPause,
          ),
        ),
        const SizedBox(width: AppSpace.sm),
        Expanded(
          child: OutlinedButton.icon(
            icon: const Icon(Icons.play_arrow_rounded),
            label: const Text('Resume'),
            onPressed: onResume,
          ),
        ),
        const SizedBox(width: AppSpace.sm),
        Expanded(
          child: OutlinedButton.icon(
            style: OutlinedButton.styleFrom(
              foregroundColor: AppColors.danger,
              side: BorderSide(color: AppColors.danger.withOpacity(0.5)),
            ),
            icon: const Icon(Icons.stop_rounded),
            label: const Text('Cancel'),
            onPressed: onCancel,
          ),
        ),
      ],
    );
  }
}

class _Panel extends StatelessWidget {
  final Widget child;
  const _Panel({required this.child});
  @override
  Widget build(BuildContext context) => Container(
        padding: const EdgeInsets.all(AppSpace.md),
        decoration: BoxDecoration(
          color: AppColors.surface.withOpacity(0.85),
          borderRadius: BorderRadius.circular(AppRadius.md),
          border: Border.all(color: AppColors.surfaceBorder),
        ),
        child: child,
      );
}

class _AiAlertBanner extends StatelessWidget {
  final double probability;
  final VoidCallback onDismiss;
  final VoidCallback onPause;

  const _AiAlertBanner({
    required this.probability,
    required this.onDismiss,
    required this.onPause,
  });

  @override
  Widget build(BuildContext context) => Material(
        color: AppColors.danger.withOpacity(0.18),
        child: Padding(
          padding: const EdgeInsets.fromLTRB(16, 10, 8, 10),
          child: Row(
            children: [
              const Icon(Icons.warning_amber_rounded, color: AppColors.danger),
              const SizedBox(width: 8),
              Expanded(
                child: Text(
                  'Possible print failure (${(probability * 100).toStringAsFixed(0)}% confidence)',
                  style: const TextStyle(fontWeight: FontWeight.w600),
                ),
              ),
              TextButton(onPressed: onDismiss, child: const Text('Dismiss')),
              TextButton(
                onPressed: () {
                  onPause();
                  onDismiss();
                },
                child: const Text('Pause'),
              ),
            ],
          ),
        ),
      );
}

class _StateChip extends StatelessWidget {
  final String state;
  const _StateChip(this.state);

  @override
  Widget build(BuildContext context) {
    Color color;
    switch (state.toUpperCase()) {
      case 'PRINTING':
        color = AppColors.success;
      case 'PAUSED':
        color = AppColors.warning;
      case 'ERROR':
      case 'ATTENTION':
        color = AppColors.danger;
      case 'FINISHED':
        color = AppColors.cool;
      default:
        color = AppColors.onSurfaceDim;
    }
    final active = ['PRINTING', 'PAUSED'].contains(state.toUpperCase());
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: color.withOpacity(0.14),
        borderRadius: BorderRadius.circular(AppRadius.pill),
        border: Border.all(color: color.withOpacity(0.4)),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          PulsingDot(color: color, size: 7, active: active),
          const SizedBox(width: 2),
          Text(state,
              style: TextStyle(color: color, fontSize: 12.5, fontWeight: FontWeight.w700)),
        ],
      ),
    );
  }
}
