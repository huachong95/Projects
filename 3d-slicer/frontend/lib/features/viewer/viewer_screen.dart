import 'dart:convert';
import 'dart:typed_data';

import 'package:file_picker/file_picker.dart';
import 'package:flutter/material.dart';
import 'package:flutter_inappwebview/flutter_inappwebview.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';

class ViewerScreen extends StatefulWidget {
  final String jobId;
  const ViewerScreen({super.key, required this.jobId});

  @override
  State<ViewerScreen> createState() => _ViewerScreenState();
}

class _ViewerScreenState extends State<ViewerScreen> {
  InAppWebViewController? _webViewController;
  bool _modelLoaded = false;
  String? _sliceJobId;
  Map<String, dynamic>? _sliceMeta;
  int _totalLayers = 0;
  int _currentLayer = 0;
  bool _layerMode = false;
  bool _downloadingGcode = false;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addPostFrameCallback((_) {
      final uri = GoRouterState.of(context).uri;
      _sliceJobId = uri.queryParameters['sliceJobId'];
      if (_sliceJobId != null) _loadSliceMetadata();
    });
  }

  Future<void> _loadSliceMetadata() async {
    try {
      final resp = await apiClient.get<Map<String, dynamic>>(
          '/api/slice/$_sliceJobId/metadata');
      if (resp.data == null) return;
      setState(() {
        _sliceMeta = resp.data;
        _totalLayers = (resp.data!['layer_count'] as num?)?.toInt() ?? 0;
      });
    } catch (_) {}
  }

  void _onWebViewCreated(InAppWebViewController controller) {
    _webViewController = controller;
    controller.addJavaScriptHandler(
      handlerName: 'onModelLoaded',
      callback: (_) => setState(() => _modelLoaded = true),
    );
    // Do NOT call _loadModel() here — the HTML page hasn't loaded yet.
    // onLoadStop fires once the page is ready and window.loadSTL exists.
  }

  Future<void> _loadModel() async {
    try {
      final resp = await apiClient.getBytes('/api/mesh/${widget.jobId}/download');
      if (!mounted || resp.data == null) return;
      final b64 = base64Encode(Uint8List.fromList(resp.data!));
      await _webViewController?.evaluateJavascript(source: "window.loadSTL('$b64')");
    } catch (_) {}
  }

  Future<void> _showLayer(int index) async {
    if (_sliceJobId == null) return;
    try {
      final resp = await apiClient.get<Map<String, dynamic>>(
          '/api/slice/$_sliceJobId/layer/$index');
      final json = jsonEncode(resp.data);
      await _webViewController?.evaluateJavascript(
          source: "window.showLayer($index, $json)");
      setState(() => _currentLayer = index);
    } catch (_) {}
  }

  Future<void> _downloadGcode() async {
    if (_sliceJobId == null) return;
    final savePath = await FilePicker.platform.saveFile(
      dialogTitle: 'Save G-code',
      fileName: 'print_$_sliceJobId.gcode',
      allowedExtensions: ['gcode'],
      type: FileType.custom,
    );
    if (savePath == null) return;

    setState(() => _downloadingGcode = true);
    try {
      await apiClient.downloadFile('/api/slice/$_sliceJobId/gcode', savePath);
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('G-code saved to $savePath')),
        );
      }
    } catch (e) {
      if (mounted) {
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(content: Text('Download failed: $e')),
        );
      }
    } finally {
      if (mounted) setState(() => _downloadingGcode = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('3D Viewer'),
        actions: [
          if (_sliceJobId != null) ...[
            TextButton.icon(
              icon: Icon(_layerMode ? Icons.view_in_ar : Icons.layers),
              label: Text(_layerMode ? '3D View' : 'Layers'),
              onPressed: () {
                setState(() => _layerMode = !_layerMode);
                if (!_layerMode) {
                  _webViewController?.evaluateJavascript(source: "window.show3D()");
                } else if (_totalLayers > 0) {
                  _showLayer(_currentLayer);
                }
              },
            ),
            IconButton(
              icon: _downloadingGcode
                  ? const SizedBox(
                      width: 20,
                      height: 20,
                      child: CircularProgressIndicator(strokeWidth: 2),
                    )
                  : const Icon(Icons.download),
              tooltip: 'Download G-code',
              onPressed: _downloadingGcode ? null : _downloadGcode,
            ),
          ],
          IconButton(
            icon: const Icon(Icons.tune),
            onPressed: () => context.push('/settings/${widget.jobId}'),
            tooltip: 'Slice settings',
          ),
        ],
      ),
      body: Column(
        children: [
          Expanded(
            child: Stack(
              children: [
                InAppWebView(
                  initialFile: 'assets/viewer/three_viewer.html',
                  onWebViewCreated: _onWebViewCreated,
                  onLoadStop: (_, __) => _loadModel(),
                  initialSettings: InAppWebViewSettings(
                    transparentBackground: true,
                    allowFileAccessFromFileURLs: true,
                    allowUniversalAccessFromFileURLs: true,
                  ),
                ),
                if (!_modelLoaded)
                  const Center(child: CircularProgressIndicator()),
              ],
            ),
          ),
          if (_sliceMeta != null)
            _MetaBar(meta: _sliceMeta!),
          if (_layerMode && _totalLayers > 1)
            _LayerSlider(
              currentLayer: _currentLayer,
              totalLayers: _totalLayers,
              onChanged: _showLayer,
            ),
        ],
      ),
    );
  }
}

class _MetaBar extends StatelessWidget {
  final Map<String, dynamic> meta;
  const _MetaBar({required this.meta});

  @override
  Widget build(BuildContext context) {
    final layers = meta['layer_count'] as int? ?? 0;
    final seconds = (meta['estimated_time_seconds'] as num?)?.toInt() ?? 0;
    final filamentMm = (meta['filament_used_mm'] as num?)?.toDouble() ?? 0.0;
    final filamentG = (meta['filament_used_g'] as num?)?.toDouble() ?? 0.0;
    final timeStr = seconds > 0 ? _fmtTime(seconds) : '—';
    final filamentStr = filamentG > 0
        ? '${filamentG.toStringAsFixed(1)} g'
        : filamentMm > 0
            ? '${(filamentMm / 1000).toStringAsFixed(2)} m'
            : '—';

    return Container(
      color: Theme.of(context).colorScheme.surfaceContainerHighest,
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 6),
      child: Row(
        mainAxisAlignment: MainAxisAlignment.spaceAround,
        children: [
          _Chip(Icons.layers, '$layers layers'),
          _Chip(Icons.access_time, timeStr),
          _Chip(Icons.straighten, filamentStr),
        ],
      ),
    );
  }

  static String _fmtTime(int seconds) {
    final h = seconds ~/ 3600;
    final m = (seconds % 3600) ~/ 60;
    if (h > 0) return '${h}h ${m}m';
    return '${m}m';
  }
}

class _Chip extends StatelessWidget {
  final IconData icon;
  final String label;
  const _Chip(this.icon, this.label);

  @override
  Widget build(BuildContext context) => Row(
        children: [
          Icon(icon, size: 14, color: Theme.of(context).colorScheme.onSurfaceVariant),
          const SizedBox(width: 4),
          Text(label, style: Theme.of(context).textTheme.bodySmall),
        ],
      );
}

class _LayerSlider extends StatelessWidget {
  final int currentLayer;
  final int totalLayers;
  final ValueChanged<int> onChanged;

  const _LayerSlider({
    required this.currentLayer,
    required this.totalLayers,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      color: Theme.of(context).colorScheme.surface,
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      child: Row(
        children: [
          IconButton(
            icon: const Icon(Icons.remove),
            onPressed: currentLayer > 0 ? () => onChanged(currentLayer - 1) : null,
          ),
          Expanded(
            child: Slider(
              value: currentLayer.toDouble(),
              min: 0,
              max: (totalLayers - 1).toDouble(),
              divisions: totalLayers - 1,
              label: 'Layer $currentLayer',
              onChanged: (v) => onChanged(v.round()),
            ),
          ),
          IconButton(
            icon: const Icon(Icons.add),
            onPressed:
                currentLayer < totalLayers - 1 ? () => onChanged(currentLayer + 1) : null,
          ),
          SizedBox(
            width: 80,
            child: Text(
              '$currentLayer / ${totalLayers - 1}',
              textAlign: TextAlign.center,
            ),
          ),
        ],
      ),
    );
  }
}
