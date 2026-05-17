import 'dart:async';
import 'dart:math' as math;
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
  Timer? _historyTimer;
  StreamSubscription? _statusSub;
  StreamSubscription? _aiSub;
  List<Map<String, dynamic>> _tempHistory = [];

  @override
  void initState() {
    super.initState();
    _statusSub = wsClient.stream('print_status').listen(_onStatus);
    _aiSub = wsClient.stream('ai_status').listen(_onAiStatus);
    _startCameraPolling();
    _startHistoryPolling();
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

  void _startHistoryPolling() {
    _historyTimer = Timer.periodic(const Duration(seconds: 3), (_) async {
      try {
        final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/temp-history');
        if (resp.data != null && mounted) {
          final readings = (resp.data!['readings'] as List?)?.cast<Map<String, dynamic>>() ?? [];
          setState(() => _tempHistory = readings);
        }
      } catch (_) {}
    });
  }

  @override
  void dispose() {
    _cameraTimer?.cancel();
    _historyTimer?.cancel();
    _statusSub?.cancel();
    _aiSub?.cancel();
    super.dispose();
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
        content: const Text('This will stop the print job and cannot be undone.'),
        actions: [
          TextButton(onPressed: () => Navigator.pop(context, false), child: const Text('No')),
          FilledButton(
            style: FilledButton.styleFrom(backgroundColor: Colors.red),
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

  @override
  Widget build(BuildContext context) {
    final progress = (_status['progress_percent'] as num?)?.toDouble() ?? 0.0;
    final tempHotend = (_status['temp_hotend'] as num?)?.toDouble() ?? 0.0;
    final tempHotendTarget = (_status['temp_hotend_target'] as num?)?.toDouble() ?? 0.0;
    final tempBed = (_status['temp_bed'] as num?)?.toDouble() ?? 0.0;
    final tempBedTarget = (_status['temp_bed_target'] as num?)?.toDouble() ?? 0.0;
    final state = _status['state']?.toString() ?? 'Unknown';
    final layer = (_status['current_layer'] as num?)?.toInt();
    final totalLayers = (_status['total_layers'] as num?)?.toInt();
    final filename = _status['filename'] as String?;
    final etaSecs = (_status['eta_seconds'] as num?)?.toInt();

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
              onPause: () async {
                await _sendCommand('/api/printer/pause', 'Paused');
                setState(() => _aiAlertVisible = false);
              },
            ),
          Expanded(
            flex: 5,
            child: _CameraFeed(frame: _cameraFrame),
          ),
          Expanded(
            flex: 7,
            child: SingleChildScrollView(
              padding: const EdgeInsets.fromLTRB(16, 8, 16, 16),
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.stretch,
                children: [
                  // Status row
                  Row(
                    children: [
                      _StatusChip(state),
                      const SizedBox(width: 8),
                      if (layer != null && layer > 0 && totalLayers != null && totalLayers > 0)
                        Text('Layer $layer / $totalLayers',
                            style: Theme.of(context).textTheme.bodySmall),
                      const Spacer(),
                      if (etaSecs != null && etaSecs > 0)
                        Row(children: [
                          const Icon(Icons.access_time, size: 14, color: Colors.grey),
                          const SizedBox(width: 4),
                          Text(_fmtEta(etaSecs),
                              style: Theme.of(context).textTheme.bodySmall),
                        ]),
                    ],
                  ),
                  if (filename != null) ...[
                    const SizedBox(height: 2),
                    Text(filename,
                        style: Theme.of(context)
                            .textTheme
                            .bodySmall
                            ?.copyWith(color: Colors.grey),
                        overflow: TextOverflow.ellipsis),
                  ],
                  const SizedBox(height: 8),
                  // Progress bar
                  ClipRRect(
                    borderRadius: BorderRadius.circular(4),
                    child: LinearProgressIndicator(
                      value: progress / 100,
                      minHeight: 8,
                      backgroundColor: Theme.of(context).colorScheme.surfaceContainerHighest,
                    ),
                  ),
                  const SizedBox(height: 4),
                  Text('${progress.toStringAsFixed(1)}%',
                      style: Theme.of(context).textTheme.bodySmall),

                  const SizedBox(height: 12),

                  // Temperature readings row
                  Row(
                    children: [
                      _TempTile(
                        label: 'Nozzle',
                        current: tempHotend,
                        target: tempHotendTarget,
                        color: Colors.orange,
                      ),
                      const SizedBox(width: 12),
                      _TempTile(
                        label: 'Bed',
                        current: tempBed,
                        target: tempBedTarget,
                        color: Colors.blue,
                      ),
                    ],
                  ),

                  const SizedBox(height: 12),

                  // Temperature history chart
                  if (_tempHistory.isNotEmpty)
                    SizedBox(
                      height: 110,
                      child: _TempChart(readings: _tempHistory),
                    )
                  else
                    Container(
                      height: 110,
                      alignment: Alignment.center,
                      decoration: BoxDecoration(
                        borderRadius: BorderRadius.circular(8),
                        color: Theme.of(context).colorScheme.surfaceContainerHighest,
                      ),
                      child: const Text('Temperature chart — connects when printer is active',
                          style: TextStyle(color: Colors.grey, fontSize: 12),
                          textAlign: TextAlign.center),
                    ),

                  const SizedBox(height: 16),

                  // Control buttons
                  Row(
                    children: [
                      Expanded(
                        child: OutlinedButton.icon(
                          icon: const Icon(Icons.pause, size: 18),
                          label: const Text('Pause'),
                          onPressed: () => _sendCommand('/api/printer/pause', 'Paused'),
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: OutlinedButton.icon(
                          icon: const Icon(Icons.play_arrow, size: 18),
                          label: const Text('Resume'),
                          onPressed: () => _sendCommand('/api/printer/resume', 'Resumed'),
                        ),
                      ),
                      const SizedBox(width: 8),
                      Expanded(
                        child: OutlinedButton.icon(
                          icon: const Icon(Icons.stop, size: 18, color: Colors.red),
                          label: const Text('Cancel', style: TextStyle(color: Colors.red)),
                          onPressed: () => _confirmCancel(context),
                        ),
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

  static String _fmtEta(int seconds) {
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return 'ETA ${h}h ${m}m';
    return 'ETA ${m}m';
  }
}

// ─── Temperature chart (custom painter) ──────────────────────────────────────

class _TempChart extends StatelessWidget {
  final List<Map<String, dynamic>> readings;
  const _TempChart({required this.readings});

  @override
  Widget build(BuildContext context) {
    return CustomPaint(
      painter: _TempChartPainter(
        readings: readings,
        hotendColor: Colors.orange,
        bedColor: Colors.blue,
        gridColor: Theme.of(context).colorScheme.outlineVariant,
        labelColor: Theme.of(context).colorScheme.onSurface.withOpacity(0.6),
      ),
    );
  }
}

class _TempChartPainter extends CustomPainter {
  final List<Map<String, dynamic>> readings;
  final Color hotendColor;
  final Color bedColor;
  final Color gridColor;
  final Color labelColor;

  const _TempChartPainter({
    required this.readings,
    required this.hotendColor,
    required this.bedColor,
    required this.gridColor,
    required this.labelColor,
  });

  @override
  void paint(Canvas canvas, Size size) {
    if (readings.isEmpty) return;

    const leftPad = 40.0;
    const rightPad = 8.0;
    const topPad = 8.0;
    const bottomPad = 18.0;
    final chartW = size.width - leftPad - rightPad;
    final chartH = size.height - topPad - bottomPad;

    // Determine Y range
    double maxTemp = 30;
    for (final r in readings) {
      maxTemp = math.max(maxTemp, (r['hotend'] as num?)?.toDouble() ?? 0);
      maxTemp = math.max(maxTemp, (r['hotend_target'] as num?)?.toDouble() ?? 0);
      maxTemp = math.max(maxTemp, (r['bed'] as num?)?.toDouble() ?? 0);
      maxTemp = math.max(maxTemp, (r['bed_target'] as num?)?.toDouble() ?? 0);
    }
    maxTemp = ((maxTemp / 50).ceil() * 50).toDouble();
    maxTemp = math.max(maxTemp, 100);

    // Time range
    final tMin = (readings.first['ts'] as num).toDouble();
    final tMax = (readings.last['ts'] as num).toDouble();
    final tSpan = math.max(tMax - tMin, 1.0);

    Offset toPixel(double ts, double temp) => Offset(
          leftPad + (ts - tMin) / tSpan * chartW,
          topPad + chartH - (temp / maxTemp) * chartH,
        );

    // Background
    final bgPaint = Paint()..color = const Color(0x0AFFFFFF);
    canvas.drawRRect(
      RRect.fromRectAndRadius(
        Rect.fromLTWH(leftPad, topPad, chartW, chartH),
        const Radius.circular(4),
      ),
      bgPaint,
    );

    // Grid lines + Y labels
    final gridPaint = Paint()..color = gridColor..strokeWidth = 0.5;
    final labelStyle = TextStyle(color: labelColor, fontSize: 9);
    for (int i = 0; i <= 4; i++) {
      final temp = maxTemp / 4 * i;
      final y = topPad + chartH - (temp / maxTemp) * chartH;
      canvas.drawLine(Offset(leftPad, y), Offset(leftPad + chartW, y), gridPaint);
      _drawText(canvas, '${temp.toInt()}°', Offset(0, y - 5), labelStyle, 36);
    }

    // Draw dashed target lines
    _drawDashed(canvas, readings, 'hotend_target', tMin, tSpan, chartW, chartH, maxTemp,
        leftPad, topPad, hotendColor.withOpacity(0.4));
    _drawDashed(canvas, readings, 'bed_target', tMin, tSpan, chartW, chartH, maxTemp,
        leftPad, topPad, bedColor.withOpacity(0.4));

    // Draw solid temperature lines
    _drawLine(canvas, readings, 'hotend', toPixel, hotendColor, 2);
    _drawLine(canvas, readings, 'bed', toPixel, bedColor, 2);

    // Legend
    _drawLegendDot(canvas, Offset(leftPad + 4, topPad + 4), hotendColor, 'Nozzle', labelStyle);
    _drawLegendDot(canvas, Offset(leftPad + 56, topPad + 4), bedColor, 'Bed', labelStyle);
  }

  void _drawLine(Canvas canvas, List<Map<String, dynamic>> readings, String key,
      Offset Function(double, double) toPixel, Color color, double width) {
    final paint = Paint()
      ..color = color
      ..strokeWidth = width
      ..style = PaintingStyle.stroke
      ..strokeCap = StrokeCap.round
      ..strokeJoin = StrokeJoin.round;
    final path = Path();
    bool first = true;
    for (final r in readings) {
      final ts = (r['ts'] as num).toDouble();
      final temp = (r[key] as num?)?.toDouble() ?? 0;
      final pt = toPixel(ts, temp);
      if (first) { path.moveTo(pt.dx, pt.dy); first = false; }
      else { path.lineTo(pt.dx, pt.dy); }
    }
    canvas.drawPath(path, paint);
  }

  void _drawDashed(Canvas canvas, List<Map<String, dynamic>> readings, String key,
      double tMin, double tSpan, double chartW, double chartH, double maxTemp,
      double leftPad, double topPad, Color color) {
    if (readings.isEmpty) return;
    final lastVal = (readings.last[key] as num?)?.toDouble() ?? 0;
    if (lastVal <= 0) return;
    final y = topPad + chartH - (lastVal / maxTemp) * chartH;
    final paint = Paint()..color = color..strokeWidth = 1;
    const dashLen = 4.0;
    const gapLen = 3.0;
    double x = leftPad;
    while (x < leftPad + chartW) {
      canvas.drawLine(Offset(x, y), Offset(math.min(x + dashLen, leftPad + chartW), y), paint);
      x += dashLen + gapLen;
    }
  }

  void _drawLegendDot(Canvas canvas, Offset pos, Color color, String label, TextStyle style) {
    canvas.drawCircle(pos, 4, Paint()..color = color);
    _drawText(canvas, label, Offset(pos.dx + 6, pos.dy - 5), style, 40);
  }

  void _drawText(Canvas canvas, String text, Offset pos, TextStyle style, double maxWidth) {
    final tp = TextPainter(
      text: TextSpan(text: text, style: style),
      textDirection: TextDirection.ltr,
    )..layout(maxWidth: maxWidth);
    tp.paint(canvas, pos);
  }

  @override
  bool shouldRepaint(_TempChartPainter old) =>
      old.readings != readings || old.hotendColor != hotendColor;
}

// ─── Other widgets ────────────────────────────────────────────────────────────

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
              Icon(Icons.videocam_off, size: 40, color: Colors.grey),
              SizedBox(height: 8),
              Text('No camera feed', style: TextStyle(color: Colors.grey, fontSize: 12)),
            ],
          ),
        ),
      );
    }
    return Image.memory(frame!, fit: BoxFit.contain, gaplessPlayback: true);
  }
}

class _TempTile extends StatelessWidget {
  final String label;
  final double current;
  final double target;
  final Color color;

  const _TempTile({required this.label, required this.current, required this.target, required this.color});

  @override
  Widget build(BuildContext context) {
    final heating = target > 0 && current < target - 2;
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          border: Border.all(color: color.withOpacity(0.4)),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Row(
          children: [
            Icon(heating ? Icons.local_fire_department : Icons.thermostat,
                size: 16, color: color),
            const SizedBox(width: 6),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  Text(label,
                      style: const TextStyle(fontSize: 10, color: Colors.grey)),
                  Text(
                    '${current.toStringAsFixed(0)}° / ${target.toStringAsFixed(0)}°',
                    style: TextStyle(fontWeight: FontWeight.bold, color: color, fontSize: 14),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _AiAlertBanner extends StatelessWidget {
  final double probability;
  final VoidCallback onDismiss;
  final VoidCallback onPause;

  const _AiAlertBanner({required this.probability, required this.onDismiss, required this.onPause});

  @override
  Widget build(BuildContext context) => MaterialBanner(
        backgroundColor: Colors.deepOrange.shade900,
        content: Row(
          children: [
            const Icon(Icons.warning_amber_rounded, color: Colors.white),
            const SizedBox(width: 8),
            Expanded(
              child: Text(
                'Possible print failure (${(probability * 100).toStringAsFixed(0)}% confidence)',
                style: const TextStyle(color: Colors.white),
              ),
            ),
          ],
        ),
        actions: [
          TextButton(
            onPressed: onDismiss,
            child: const Text('Dismiss', style: TextStyle(color: Colors.white70)),
          ),
          TextButton(
            onPressed: onPause,
            child: const Text('Pause', style: TextStyle(color: Colors.white)),
          ),
        ],
      );
}

class _StatusChip extends StatelessWidget {
  final String state;
  const _StatusChip(this.state);

  @override
  Widget build(BuildContext context) {
    final color = switch (state.toUpperCase()) {
      'PRINTING' => Colors.greenAccent,
      'PAUSED' => Colors.orangeAccent,
      'ERROR' => Colors.redAccent,
      'FINISHED' => Colors.blueAccent,
      _ => Colors.grey,
    };
    return Chip(
      avatar: Icon(Icons.circle, size: 10, color: color),
      label: Text(state),
      padding: EdgeInsets.zero,
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
    );
  }
}
