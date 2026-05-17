import 'dart:async';

import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';
import '../../core/websocket_client.dart';
import '../../shared/models/slice_profile.dart';

class SliceSettingsScreen extends StatefulWidget {
  final String jobId;
  const SliceSettingsScreen({super.key, required this.jobId});

  @override
  State<SliceSettingsScreen> createState() => _SliceSettingsScreenState();
}

class _SliceSettingsScreenState extends State<SliceSettingsScreen> {
  SliceProfile _profile = const SliceProfile(name: 'pla_standard');
  String _selectedBaseProfile = 'pla_standard';
  List<String> _availableProfiles = [];
  bool _slicing = false;
  double _sliceProgress = 0.0;
  String? _sliceJobId;
  String? _errorMsg;
  StreamSubscription? _progressSub;
  Timer? _pollTimer;
  bool _navigating = false;

  @override
  void initState() {
    super.initState();
    _loadProfiles();
    _listenToProgress();
  }

  @override
  void dispose() {
    _progressSub?.cancel();
    _pollTimer?.cancel();
    super.dispose();
  }

  Future<void> _loadProfiles() async {
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/slice/profiles');
      final profiles = (resp.data?['profiles'] as List?)?.cast<String>() ?? [];
      setState(() => _availableProfiles = profiles);
    } catch (_) {}
  }

  void _listenToProgress() {
    _progressSub = wsClient.stream('slice_progress').listen((msg) {
      if (!mounted) return;
      if (msg['type'] == 'progress') {
        setState(() => _sliceProgress = (msg['data']['percent'] as num).toDouble());
      } else if (msg['type'] == 'complete') {
        _onSliceComplete(msg['data']['slice_job_id'] as String?);
      }
    });
  }

  void _startPollTimer() {
    _pollTimer?.cancel();
    _pollTimer = Timer.periodic(const Duration(seconds: 2), (_) async {
      if (!_slicing || _sliceJobId == null) {
        _pollTimer?.cancel();
        return;
      }
      try {
        final resp = await apiClient.get<Map<String, dynamic>>('/api/slice/$_sliceJobId/status');
        final state = resp.data?['state'] as String?;
        if (state == 'complete') {
          _onSliceComplete(_sliceJobId);
        } else if (state == 'failed') {
          _pollTimer?.cancel();
          if (mounted) {
            setState(() {
              _slicing = false;
              _errorMsg = resp.data?['error'] as String? ?? 'Slicing failed';
            });
          }
        }
      } catch (_) {}
    });
  }

  void _onSliceComplete(String? id) {
    if (_navigating) return;
    _navigating = true;
    _pollTimer?.cancel();
    if (mounted) setState(() { _slicing = false; _sliceProgress = 100.0; });
    if (id != null && mounted) {
      context.go('/viewer/${widget.jobId}?sliceJobId=$id');
    }
  }

  Future<void> _startSlice() async {
    setState(() {
      _slicing = true;
      _sliceProgress = 0.0;
      _errorMsg = null;
      _navigating = false;
    });
    try {
      final resp = await apiClient.post<Map<String, dynamic>>(
        '/api/slice/start',
        data: {
          'mesh_job_id': widget.jobId,
          'settings': {
            'profile_name': _selectedBaseProfile,
            'overrides': _profile.toOverrides(),
          },
          'timelapse_hooks': true,
        },
      );
      setState(() => _sliceJobId = resp.data?['slice_job_id']);
      _startPollTimer();
    } catch (e) {
      setState(() {
        _slicing = false;
        _errorMsg = e.toString();
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Slice Settings')),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            _Section(
              title: 'Material Profile',
              child: DropdownButtonFormField<String>(
                value: _availableProfiles.contains(_selectedBaseProfile)
                    ? _selectedBaseProfile
                    : null,
                items: _availableProfiles
                    .map((p) => DropdownMenuItem(value: p, child: Text(p)))
                    .toList(),
                onChanged: _slicing
                    ? null
                    : (v) => setState(() => _selectedBaseProfile = v ?? _selectedBaseProfile),
                decoration: const InputDecoration(labelText: 'Base profile'),
              ),
            ),
            _Section(
              title: 'Layer',
              child: Column(
                children: [
                  _Slider(
                    label: 'Layer height',
                    value: _profile.layerHeight,
                    min: 0.05,
                    max: 0.35,
                    divisions: 6,
                    display: '${_profile.layerHeight.toStringAsFixed(2)} mm',
                    onChanged: (v) => setState(() => _profile = _profile.copyWith(layerHeight: v)),
                  ),
                ],
              ),
            ),
            _Section(
              title: 'Infill',
              child: Column(
                children: [
                  _Slider(
                    label: 'Infill density',
                    value: _profile.infillDensity.toDouble(),
                    min: 0,
                    max: 100,
                    divisions: 20,
                    display: '${_profile.infillDensity}%',
                    onChanged: (v) =>
                        setState(() => _profile = _profile.copyWith(infillDensity: v.round())),
                  ),
                  const SizedBox(height: 8),
                  DropdownButtonFormField<String>(
                    value: _profile.infillPattern,
                    items: ['grid', 'gyroid', 'honeycomb', 'triangles', 'lines']
                        .map((p) => DropdownMenuItem(value: p, child: Text(p)))
                        .toList(),
                    onChanged: _slicing
                        ? null
                        : (v) => setState(() => _profile = _profile.copyWith(infillPattern: v)),
                    decoration: const InputDecoration(labelText: 'Pattern'),
                  ),
                ],
              ),
            ),
            _Section(
              title: 'Supports',
              child: SwitchListTile(
                title: const Text('Enable supports'),
                subtitle: const Text('Automatically generated'),
                value: _profile.supportsEnabled,
                onChanged: _slicing
                    ? null
                    : (v) => setState(() => _profile = _profile.copyWith(supportsEnabled: v)),
              ),
            ),
            _Section(
              title: 'Temperature',
              child: Column(
                children: [
                  _Slider(
                    label: 'Nozzle temp',
                    value: _profile.printTemp,
                    min: 170,
                    max: 280,
                    divisions: 22,
                    display: '${_profile.printTemp.round()}°C',
                    onChanged: (v) =>
                        setState(() => _profile = _profile.copyWith(printTemp: v)),
                  ),
                  _Slider(
                    label: 'Bed temp',
                    value: _profile.bedTemp,
                    min: 0,
                    max: 120,
                    divisions: 24,
                    display: '${_profile.bedTemp.round()}°C',
                    onChanged: (v) =>
                        setState(() => _profile = _profile.copyWith(bedTemp: v)),
                  ),
                ],
              ),
            ),
            if (_errorMsg != null)
              Padding(
                padding: const EdgeInsets.only(bottom: 12),
                child: Text(_errorMsg!,
                    style: TextStyle(color: Theme.of(context).colorScheme.error)),
              ),
            if (_slicing) ...[
              LinearProgressIndicator(value: _sliceProgress / 100),
              const SizedBox(height: 8),
              Text('Slicing… ${_sliceProgress.toStringAsFixed(0)}%'),
              const SizedBox(height: 12),
            ],
            FilledButton.icon(
              icon: const Icon(Icons.layers),
              label: Text(_slicing ? 'Slicing…' : 'Slice'),
              onPressed: _slicing ? null : _startSlice,
            ),
          ],
        ),
      ),
    );
  }
}

class _Section extends StatelessWidget {
  final String title;
  final Widget child;
  const _Section({required this.title, required this.child});

  @override
  Widget build(BuildContext context) => Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 12),
            child: Text(title, style: Theme.of(context).textTheme.titleSmall),
          ),
          child,
          const SizedBox(height: 8),
        ],
      );
}

class _Slider extends StatelessWidget {
  final String label;
  final double value;
  final double min;
  final double max;
  final int divisions;
  final String display;
  final ValueChanged<double> onChanged;

  const _Slider({
    required this.label,
    required this.value,
    required this.min,
    required this.max,
    required this.divisions,
    required this.display,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) => Row(
        children: [
          SizedBox(width: 120, child: Text(label)),
          Expanded(
            child: Slider(
              value: value.clamp(min, max),
              min: min,
              max: max,
              divisions: divisions,
              onChanged: onChanged,
            ),
          ),
          SizedBox(width: 60, child: Text(display, textAlign: TextAlign.right)),
        ],
      );
}
