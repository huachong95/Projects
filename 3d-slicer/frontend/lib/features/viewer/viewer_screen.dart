import 'dart:convert';
import 'dart:typed_data';

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
  int _totalLayers = 0;
  int _currentLayer = 0;
  bool _layerMode = false;

  @override
  void initState() {
    super.initState();
    // Extract sliceJobId from URI query params
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
      setState(() => _totalLayers = (resp.data?['layer_count'] as num?)?.toInt() ?? 0);
    } catch (_) {}
  }

  void _onWebViewCreated(InAppWebViewController controller) {
    _webViewController = controller;
    controller.addJavaScriptHandler(
      handlerName: 'onModelLoaded',
      callback: (args) {
        setState(() => _modelLoaded = true);
      },
    );
    _loadModel();
  }

  Future<void> _loadModel() async {
    try {
      final resp = await apiClient.get<List<int>>('/api/mesh/${widget.jobId}/download');
      if (resp.data == null) return;
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

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('3D Viewer'),
        actions: [
          if (_sliceJobId != null)
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
            icon: const Icon(Icons.tune),
            onPressed: () => context.go('/settings/${widget.jobId}'),
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
          if (_layerMode && _totalLayers > 0)
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
