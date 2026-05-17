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

class _SliceSettingsScreenState extends State<SliceSettingsScreen>
    with SingleTickerProviderStateMixin {
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
  late TabController _tabController;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 5, vsync: this);
    _loadProfiles();
    _listenToProgress();
  }

  @override
  void dispose() {
    _tabController.dispose();
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
      if (msg['type'] == 'progress' && _slicing) {
        setState(() => _sliceProgress =
            (msg['data']?['percent'] as num?)?.toDouble() ?? _sliceProgress);
      } else if (msg['type'] == 'complete' && _slicing) {
        _onSliceComplete(msg['data']?['slice_job_id'] as String?);
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

  Widget _buildSlider({
    required String label,
    required String unit,
    required double value,
    required double min,
    required double max,
    required int divisions,
    required ValueChanged<double> onChanged,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 4),
      child: Row(
        children: [
          SizedBox(width: 140, child: Text(label, style: Theme.of(context).textTheme.bodyMedium)),
          Expanded(
            child: Slider(
              value: value.clamp(min, max),
              min: min,
              max: max,
              divisions: divisions,
              onChanged: _slicing ? null : onChanged,
            ),
          ),
          SizedBox(
            width: 64,
            child: Text('${value.toStringAsFixed(value < 10 ? 2 : 0)}$unit',
                textAlign: TextAlign.right,
                style: Theme.of(context).textTheme.bodySmall),
          ),
        ],
      ),
    );
  }

  Widget _buildIntSlider({
    required String label,
    required String unit,
    required int value,
    required int min,
    required int max,
    required ValueChanged<int> onChanged,
  }) =>
      _buildSlider(
        label: label,
        unit: unit,
        value: value.toDouble(),
        min: min.toDouble(),
        max: max.toDouble(),
        divisions: max - min,
        onChanged: (v) => onChanged(v.round()),
      );

  Widget _buildDropdown<T>({
    required String label,
    required T value,
    required List<T> items,
    required ValueChanged<T?> onChanged,
    String Function(T)? display,
  }) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 6),
      child: Row(
        children: [
          SizedBox(width: 140, child: Text(label, style: Theme.of(context).textTheme.bodyMedium)),
          Expanded(
            child: DropdownButtonFormField<T>(
              value: value,
              isDense: true,
              items: items
                  .map((v) => DropdownMenuItem(value: v, child: Text(display?.call(v) ?? v.toString())))
                  .toList(),
              onChanged: _slicing ? null : onChanged,
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                contentPadding: EdgeInsets.symmetric(horizontal: 12, vertical: 8),
              ),
            ),
          ),
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Slice Settings'),
        bottom: TabBar(
          controller: _tabController,
          isScrollable: true,
          tabs: const [
            Tab(text: 'Basic'),
            Tab(text: 'Walls & Shell'),
            Tab(text: 'Speed'),
            Tab(text: 'Temperature'),
            Tab(text: 'Advanced'),
          ],
        ),
      ),
      body: Column(
        children: [
          Expanded(
            child: TabBarView(
              controller: _tabController,
              children: [
                _BasicTab(
                  profile: _profile,
                  selectedBaseProfile: _selectedBaseProfile,
                  availableProfiles: _availableProfiles,
                  slicing: _slicing,
                  onProfileChanged: (v) => setState(() => _selectedBaseProfile = v ?? _selectedBaseProfile),
                  buildSlider: _buildSlider,
                  buildIntSlider: _buildIntSlider,
                  buildDropdown: _buildDropdown,
                  onChanged: (p) => setState(() => _profile = p),
                ),
                _WallsTab(
                  profile: _profile,
                  slicing: _slicing,
                  buildIntSlider: _buildIntSlider,
                  buildSlider: _buildSlider,
                  onChanged: (p) => setState(() => _profile = p),
                ),
                _SpeedTab(
                  profile: _profile,
                  slicing: _slicing,
                  buildSlider: _buildSlider,
                  onChanged: (p) => setState(() => _profile = p),
                ),
                _TemperatureTab(
                  profile: _profile,
                  slicing: _slicing,
                  buildSlider: _buildSlider,
                  buildIntSlider: _buildIntSlider,
                  onChanged: (p) => setState(() => _profile = p),
                ),
                _AdvancedTab(
                  profile: _profile,
                  slicing: _slicing,
                  buildSlider: _buildSlider,
                  buildDropdown: _buildDropdown,
                  onChanged: (p) => setState(() => _profile = p),
                ),
              ],
            ),
          ),
          _BottomSliceBar(
            slicing: _slicing,
            progress: _sliceProgress,
            errorMsg: _errorMsg,
            onSlice: _startSlice,
          ),
        ],
      ),
    );
  }
}

// ─── Tab widgets ─────────────────────────────────────────────────────────────

class _BasicTab extends StatelessWidget {
  final SliceProfile profile;
  final String selectedBaseProfile;
  final List<String> availableProfiles;
  final bool slicing;
  final ValueChanged<String?> onProfileChanged;
  final Widget Function({required String label, required String unit, required double value, required double min, required double max, required int divisions, required ValueChanged<double> onChanged}) buildSlider;
  final Widget Function({required String label, required String unit, required int value, required int min, required int max, required ValueChanged<int> onChanged}) buildIntSlider;
  final Widget Function<T>({required String label, required T value, required List<T> items, required ValueChanged<T?> onChanged, String Function(T)? display}) buildDropdown;
  final ValueChanged<SliceProfile> onChanged;

  const _BasicTab({
    required this.profile,
    required this.selectedBaseProfile,
    required this.availableProfiles,
    required this.slicing,
    required this.onProfileChanged,
    required this.buildSlider,
    required this.buildIntSlider,
    required this.buildDropdown,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        _SectionHeader('Material Profile'),
        Padding(
          padding: const EdgeInsets.symmetric(vertical: 6),
          child: DropdownButtonFormField<String>(
            value: availableProfiles.contains(selectedBaseProfile) ? selectedBaseProfile : null,
            decoration: const InputDecoration(labelText: 'Base profile', border: OutlineInputBorder()),
            items: availableProfiles.map((p) => DropdownMenuItem(value: p, child: Text(p))).toList(),
            onChanged: slicing ? null : onProfileChanged,
          ),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Layer'),
        buildSlider(
          label: 'Layer height',
          unit: ' mm',
          value: profile.layerHeight,
          min: 0.05,
          max: 0.35,
          divisions: 6,
          onChanged: (v) => onChanged(profile.copyWith(layerHeight: v)),
        ),
        buildSlider(
          label: 'First layer',
          unit: ' mm',
          value: profile.firstLayerHeight,
          min: 0.1,
          max: 0.35,
          divisions: 5,
          onChanged: (v) => onChanged(profile.copyWith(firstLayerHeight: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Infill'),
        buildIntSlider(
          label: 'Density',
          unit: '%',
          value: profile.infillDensity,
          min: 0,
          max: 100,
          onChanged: (v) => onChanged(profile.copyWith(infillDensity: v)),
        ),
        buildDropdown<String>(
          label: 'Pattern',
          value: profile.infillPattern,
          items: const ['grid', 'gyroid', 'honeycomb', 'triangles', 'lines', 'cubic', 'lightning'],
          onChanged: (v) => onChanged(profile.copyWith(infillPattern: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Supports'),
        SwitchListTile(
          title: const Text('Enable supports'),
          subtitle: const Text('Auto-generated for overhangs > 50°'),
          contentPadding: EdgeInsets.zero,
          value: profile.supportsEnabled,
          onChanged: slicing ? null : (v) => onChanged(profile.copyWith(supportsEnabled: v)),
        ),
        if (profile.supportsEnabled)
          buildDropdown<String>(
            label: 'Support type',
            value: profile.supportType,
            items: const ['normal', 'tree', 'everywhere'],
            onChanged: (v) => onChanged(profile.copyWith(supportType: v)),
          ),
      ],
    );
  }
}

class _WallsTab extends StatelessWidget {
  final SliceProfile profile;
  final bool slicing;
  final Widget Function({required String label, required String unit, required int value, required int min, required int max, required ValueChanged<int> onChanged}) buildIntSlider;
  final Widget Function({required String label, required String unit, required double value, required double min, required double max, required int divisions, required ValueChanged<double> onChanged}) buildSlider;
  final ValueChanged<SliceProfile> onChanged;

  const _WallsTab({
    required this.profile,
    required this.slicing,
    required this.buildIntSlider,
    required this.buildSlider,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        _SectionHeader('Walls (Perimeters)'),
        buildIntSlider(
          label: 'Wall count',
          unit: ' walls',
          value: profile.wallCount,
          min: 1,
          max: 10,
          onChanged: (v) => onChanged(profile.copyWith(wallCount: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Top & Bottom Layers'),
        buildIntSlider(
          label: 'Top layers',
          unit: ' layers',
          value: profile.topLayers,
          min: 0,
          max: 15,
          onChanged: (v) => onChanged(profile.copyWith(topLayers: v)),
        ),
        buildIntSlider(
          label: 'Bottom layers',
          unit: ' layers',
          value: profile.bottomLayers,
          min: 0,
          max: 15,
          onChanged: (v) => onChanged(profile.copyWith(bottomLayers: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Brim / Skirt'),
        buildSlider(
          label: 'Brim width',
          unit: ' mm',
          value: profile.brimWidth,
          min: 0,
          max: 20,
          divisions: 20,
          onChanged: (v) => onChanged(profile.copyWith(brimWidth: v)),
        ),
        if (profile.brimWidth > 0)
          Padding(
            padding: const EdgeInsets.only(left: 8, top: 4),
            child: Text(
              '${(profile.brimWidth / profile.layerHeight).round()} brim lines',
              style: Theme.of(context).textTheme.bodySmall?.copyWith(color: Colors.grey),
            ),
          ),
      ],
    );
  }
}

class _SpeedTab extends StatelessWidget {
  final SliceProfile profile;
  final bool slicing;
  final Widget Function({required String label, required String unit, required double value, required double min, required double max, required int divisions, required ValueChanged<double> onChanged}) buildSlider;
  final ValueChanged<SliceProfile> onChanged;

  const _SpeedTab({
    required this.profile,
    required this.slicing,
    required this.buildSlider,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        _SectionHeader('Print Speed'),
        buildSlider(
          label: 'Print speed',
          unit: ' mm/s',
          value: profile.printSpeed,
          min: 10,
          max: 300,
          divisions: 29,
          onChanged: (v) => onChanged(profile.copyWith(printSpeed: v)),
        ),
        buildSlider(
          label: 'First layer',
          unit: ' mm/s',
          value: profile.firstLayerSpeed,
          min: 5,
          max: 60,
          divisions: 11,
          onChanged: (v) => onChanged(profile.copyWith(firstLayerSpeed: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Retraction'),
        buildSlider(
          label: 'Distance',
          unit: ' mm',
          value: profile.retractionDistance,
          min: 0,
          max: 6,
          divisions: 24,
          onChanged: (v) => onChanged(profile.copyWith(retractionDistance: v)),
        ),
        buildSlider(
          label: 'Speed',
          unit: ' mm/s',
          value: profile.retractionSpeed,
          min: 10,
          max: 100,
          divisions: 18,
          onChanged: (v) => onChanged(profile.copyWith(retractionSpeed: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Z-Hop'),
        buildSlider(
          label: 'Z-hop height',
          unit: ' mm',
          value: profile.zHop,
          min: 0,
          max: 2,
          divisions: 20,
          onChanged: (v) => onChanged(profile.copyWith(zHop: v)),
        ),
      ],
    );
  }
}

class _TemperatureTab extends StatelessWidget {
  final SliceProfile profile;
  final bool slicing;
  final Widget Function({required String label, required String unit, required double value, required double min, required double max, required int divisions, required ValueChanged<double> onChanged}) buildSlider;
  final Widget Function({required String label, required String unit, required int value, required int min, required int max, required ValueChanged<int> onChanged}) buildIntSlider;
  final ValueChanged<SliceProfile> onChanged;

  const _TemperatureTab({
    required this.profile,
    required this.slicing,
    required this.buildSlider,
    required this.buildIntSlider,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        _SectionHeader('Nozzle'),
        buildSlider(
          label: 'Print temperature',
          unit: '°C',
          value: profile.printTemp,
          min: 170,
          max: 300,
          divisions: 26,
          onChanged: (v) => onChanged(profile.copyWith(printTemp: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Bed'),
        buildSlider(
          label: 'Bed temperature',
          unit: '°C',
          value: profile.bedTemp,
          min: 0,
          max: 120,
          divisions: 24,
          onChanged: (v) => onChanged(profile.copyWith(bedTemp: v)),
        ),
        const SizedBox(height: 8),
        _SectionHeader('Cooling Fan'),
        buildIntSlider(
          label: 'Fan speed',
          unit: '%',
          value: profile.fanSpeed,
          min: 0,
          max: 100,
          onChanged: (v) => onChanged(profile.copyWith(fanSpeed: v)),
        ),
        buildIntSlider(
          label: 'First layer fan',
          unit: '%',
          value: profile.firstLayerFanSpeed,
          min: 0,
          max: 100,
          onChanged: (v) => onChanged(profile.copyWith(firstLayerFanSpeed: v)),
        ),
        Padding(
          padding: const EdgeInsets.only(left: 8, top: 4),
          child: Text(
            'Keep first-layer fan at 0% for better adhesion',
            style: Theme.of(context).textTheme.bodySmall?.copyWith(color: Colors.grey),
          ),
        ),
      ],
    );
  }
}

class _AdvancedTab extends StatelessWidget {
  final SliceProfile profile;
  final bool slicing;
  final Widget Function({required String label, required String unit, required double value, required double min, required double max, required int divisions, required ValueChanged<double> onChanged}) buildSlider;
  final Widget Function<T>({required String label, required T value, required List<T> items, required ValueChanged<T?> onChanged, String Function(T)? display}) buildDropdown;
  final ValueChanged<SliceProfile> onChanged;

  const _AdvancedTab({
    required this.profile,
    required this.slicing,
    required this.buildSlider,
    required this.buildDropdown,
    required this.onChanged,
  });

  @override
  Widget build(BuildContext context) {
    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        _SectionHeader('Seam'),
        buildDropdown<String>(
          label: 'Seam position',
          value: profile.seamPosition,
          items: const ['aligned', 'random', 'sharpest_corner', 'shortest'],
          display: (v) => switch (v) {
            'aligned' => 'Aligned (hidden)',
            'random' => 'Random',
            'sharpest_corner' => 'Sharpest corner',
            'shortest' => 'Shortest travel',
            _ => v,
          },
          onChanged: (v) => onChanged(profile.copyWith(seamPosition: v)),
        ),
        const SizedBox(height: 24),
        Card(
          color: Theme.of(context).colorScheme.surfaceContainerHighest,
          child: const Padding(
            padding: EdgeInsets.all(16),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Row(children: [
                  Icon(Icons.info_outline, size: 16),
                  SizedBox(width: 8),
                  Text('Profile note', style: TextStyle(fontWeight: FontWeight.bold)),
                ]),
                SizedBox(height: 8),
                Text(
                  'Settings here are merged on top of the selected base profile. '
                  'The base profile defines printer-specific optimisations for the Prusa Mk4.',
                  style: TextStyle(fontSize: 12),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }
}

// ─── Bottom bar ───────────────────────────────────────────────────────────────

class _BottomSliceBar extends StatelessWidget {
  final bool slicing;
  final double progress;
  final String? errorMsg;
  final VoidCallback onSlice;

  const _BottomSliceBar({
    required this.slicing,
    required this.progress,
    required this.errorMsg,
    required this.onSlice,
  });

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.fromLTRB(16, 8, 16, 16),
      decoration: BoxDecoration(
        color: Theme.of(context).colorScheme.surface,
        border: Border(top: BorderSide(color: Theme.of(context).colorScheme.outlineVariant)),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        mainAxisSize: MainAxisSize.min,
        children: [
          if (errorMsg != null)
            Padding(
              padding: const EdgeInsets.only(bottom: 8),
              child: Text(errorMsg!,
                  style: TextStyle(color: Theme.of(context).colorScheme.error, fontSize: 12)),
            ),
          if (slicing) ...[
            LinearProgressIndicator(value: progress / 100),
            const SizedBox(height: 4),
            Text('Slicing… ${progress.toStringAsFixed(0)}%',
                style: Theme.of(context).textTheme.bodySmall),
            const SizedBox(height: 8),
          ],
          FilledButton.icon(
            icon: const Icon(Icons.layers),
            label: Text(slicing ? 'Slicing…' : 'Slice Model'),
            onPressed: slicing ? null : onSlice,
          ),
        ],
      ),
    );
  }
}

// ─── Shared helpers ───────────────────────────────────────────────────────────

class _SectionHeader extends StatelessWidget {
  final String title;
  const _SectionHeader(this.title);

  @override
  Widget build(BuildContext context) => Padding(
        padding: const EdgeInsets.only(top: 8, bottom: 4),
        child: Text(title,
            style: Theme.of(context)
                .textTheme
                .labelLarge
                ?.copyWith(color: Theme.of(context).colorScheme.primary)),
      );
}
