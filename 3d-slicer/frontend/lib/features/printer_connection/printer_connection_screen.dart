import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

import '../../core/api_client.dart';

class PrinterConnectionScreen extends StatefulWidget {
  const PrinterConnectionScreen({super.key});

  @override
  State<PrinterConnectionScreen> createState() => _PrinterConnectionScreenState();
}

class _PrinterConnectionScreenState extends State<PrinterConnectionScreen>
    with SingleTickerProviderStateMixin {
  // Connection state
  final _hostController = TextEditingController();
  final _keyController = TextEditingController();
  bool _connecting = false;
  bool _connected = false;
  String? _errorMsg;
  List<Map<String, dynamic>> _discovered = [];
  bool _discovering = false;

  // Control panel state
  late TabController _tabController;
  Map<String, dynamic> _status = {};
  Timer? _statusTimer;
  double _jogStep = 1.0;
  int _extrudeAmount = 5;
  int _fanSpeed = 0;
  final _nozzleTempController = TextEditingController();
  final _bedTempController = TextEditingController();
  final _gcodeController = TextEditingController();
  final List<String> _terminalLog = [];
  final _terminalScrollController = ScrollController();
  bool _sendingGcode = false;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 2, vsync: this);
    _checkInitialStatus();
  }

  @override
  void dispose() {
    _tabController.dispose();
    _statusTimer?.cancel();
    _hostController.dispose();
    _keyController.dispose();
    _nozzleTempController.dispose();
    _bedTempController.dispose();
    _gcodeController.dispose();
    _terminalScrollController.dispose();
    super.dispose();
  }

  Future<void> _checkInitialStatus() async {
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/status');
      if (resp.statusCode == 200 && mounted) {
        setState(() { _connected = true; _status = resp.data ?? {}; });
        _startPolling();
      }
    } catch (_) {}
  }

  void _startPolling() {
    _statusTimer?.cancel();
    _statusTimer = Timer.periodic(const Duration(seconds: 2), (_) async {
      if (!_connected || !mounted) return;
      try {
        final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/status');
        if (resp.statusCode == 200 && mounted) {
          setState(() => _status = resp.data ?? {});
          // Pre-fill temp fields with current targets if empty
          final hotend = (_status['temp_hotend_target'] as num?)?.toDouble() ?? 0;
          final bed = (_status['temp_bed_target'] as num?)?.toDouble() ?? 0;
          if (_nozzleTempController.text.isEmpty && hotend > 0) {
            _nozzleTempController.text = hotend.toStringAsFixed(0);
          }
          if (_bedTempController.text.isEmpty && bed > 0) {
            _bedTempController.text = bed.toStringAsFixed(0);
          }
        }
      } catch (_) {}
    });
  }

  Future<void> _discover() async {
    setState(() { _discovering = true; _discovered = []; });
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/discover');
      final devices = (resp.data?['devices'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      setState(() => _discovered = devices);
      if (devices.isEmpty && mounted) _showSnack('No printers found on the network.');
    } catch (e) {
      _showSnack('Discovery failed: $e');
    } finally {
      if (mounted) setState(() => _discovering = false);
    }
  }

  Future<void> _connect() async {
    final host = _hostController.text.trim();
    final key = _keyController.text.trim();
    if (host.isEmpty || key.isEmpty) {
      setState(() => _errorMsg = 'Enter both IP address and API key.');
      return;
    }
    setState(() { _connecting = true; _errorMsg = null; });
    try {
      await apiClient.post('/api/printer/connect', data: {
        'type': 'prusalink', 'host': host, 'api_key': key,
      });
      setState(() => _connected = true);
      _showSnack('Connected to Prusa Mk4 at $host');
      _startPolling();
    } catch (_) {
      setState(() => _errorMsg = 'Connection failed. Check the IP and API key.');
    } finally {
      if (mounted) setState(() => _connecting = false);
    }
  }

  Future<void> _disconnect() async {
    try {
      await apiClient.delete('/api/printer/connect');
      _statusTimer?.cancel();
      setState(() { _connected = false; _status = {}; });
      _showSnack('Disconnected');
    } catch (e) {
      _showSnack('Disconnect failed: $e');
    }
  }

  // ── Printer commands ──────────────────────────────────────────────────────

  Future<void> _move(String axis, double dist) async {
    try {
      await apiClient.post('/api/printer/move', data: {
        'axis': axis,
        'distance': dist,
        'speed': axis == 'Z' ? 600 : 3000,
      });
    } catch (e) { _showSnack('Move failed: $e'); }
  }

  Future<void> _home(List<String>? axes) async {
    try {
      await apiClient.post('/api/printer/home', data: {'axes': axes});
      _showSnack('Homing ${axes == null ? 'all axes' : axes.join(', ')}…');
    } catch (e) { _showSnack('Home failed: $e'); }
  }

  Future<void> _setTemperature() async {
    final hotend = double.tryParse(_nozzleTempController.text);
    final bed = double.tryParse(_bedTempController.text);
    if (hotend == null && bed == null) {
      _showSnack('Enter a temperature value first');
      return;
    }
    try {
      await apiClient.post('/api/printer/temperature', data: {
        if (hotend != null) 'hotend': hotend,
        if (bed != null) 'bed': bed,
      });
      _showSnack('Temperature set');
    } catch (e) { _showSnack('Failed: $e'); }
  }

  Future<void> _cooldown() async {
    try {
      await apiClient.post('/api/printer/temperature', data: {'hotend': 0, 'bed': 0});
      _nozzleTempController.text = '0';
      _bedTempController.text = '0';
      _showSnack('Cooling down');
    } catch (e) { _showSnack('Failed: $e'); }
  }

  Future<void> _setFan(int pct) async {
    try {
      await apiClient.post('/api/printer/fan', data: {'speed_percent': pct});
      setState(() => _fanSpeed = pct);
    } catch (e) { _showSnack('Fan command failed: $e'); }
  }

  Future<void> _extrude(double mm) async {
    try {
      await apiClient.post('/api/printer/extrude',
          data: {'distance_mm': mm, 'speed_mm_per_min': 300});
    } catch (e) { _showSnack('Extrude failed: $e'); }
  }

  Future<void> _sendGcode() async {
    final cmd = _gcodeController.text.trim();
    if (cmd.isEmpty || _sendingGcode) return;
    setState(() { _terminalLog.add('> $cmd'); _gcodeController.clear(); _sendingGcode = true; });
    _scrollTerminal();
    try {
      await apiClient.post('/api/printer/gcode', data: {'command': cmd});
      setState(() { _terminalLog.add('  ok'); });
    } catch (e) {
      setState(() { _terminalLog.add('  error: $e'); });
    } finally {
      setState(() => _sendingGcode = false);
      _scrollTerminal();
    }
  }

  void _scrollTerminal() {
    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (_terminalScrollController.hasClients) {
        _terminalScrollController.animateTo(
          _terminalScrollController.position.maxScrollExtent,
          duration: const Duration(milliseconds: 200),
          curve: Curves.easeOut,
        );
      }
    });
  }

  void _showSnack(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  // ── Build ─────────────────────────────────────────────────────────────────

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Connect Printer'),
        bottom: _connected
            ? TabBar(
                controller: _tabController,
                tabs: const [Tab(text: 'Controls'), Tab(text: 'Terminal')],
              )
            : null,
      ),
      body: _connected ? _buildControlPanel() : _buildConnectionForm(),
    );
  }

  Widget _buildConnectionForm() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(24),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          _StatusBar(connected: false, status: {}, onDisconnect: _disconnect),
          const SizedBox(height: 24),
          Text('Auto-discover', style: Theme.of(context).textTheme.titleSmall),
          const SizedBox(height: 8),
          const Text('Scans your local network for Prusa printers.',
              style: TextStyle(color: Colors.grey)),
          const SizedBox(height: 12),
          OutlinedButton.icon(
            icon: _discovering
                ? const SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))
                : const Icon(Icons.search),
            label: Text(_discovering ? 'Searching…' : 'Discover on network'),
            onPressed: _discovering ? null : _discover,
          ),
          if (_discovered.isNotEmpty) ...[
            const SizedBox(height: 8),
            ..._discovered.map((d) => ListTile(
                  leading: const Icon(Icons.print),
                  title: Text(d['name'] as String? ?? 'Prusa Printer'),
                  subtitle: Text(d['host'] as String? ?? ''),
                  trailing: TextButton(
                    child: const Text('Use'),
                    onPressed: () => setState(() => _hostController.text = d['host'] as String? ?? ''),
                  ),
                )),
          ],
          const SizedBox(height: 16),
          const Divider(),
          const SizedBox(height: 16),
          Text('Manual entry', style: Theme.of(context).textTheme.titleSmall),
          const SizedBox(height: 4),
          const Text(
            'IP: Settings → Network → IP Address\nAPI key: Settings → Network → PrusaLink',
            style: TextStyle(color: Colors.grey),
          ),
          const SizedBox(height: 16),
          TextField(
            controller: _hostController,
            decoration: const InputDecoration(
              labelText: 'Printer IP address',
              hintText: '192.168.1.100',
              prefixIcon: Icon(Icons.router),
              border: OutlineInputBorder(),
            ),
            keyboardType: TextInputType.number,
          ),
          const SizedBox(height: 12),
          TextField(
            controller: _keyController,
            decoration: const InputDecoration(
              labelText: 'PrusaLink API key',
              hintText: '8-character key from the printer',
              prefixIcon: Icon(Icons.key),
              border: OutlineInputBorder(),
            ),
            obscureText: true,
          ),
          if (_errorMsg != null) ...[
            const SizedBox(height: 8),
            Text(_errorMsg!, style: TextStyle(color: Theme.of(context).colorScheme.error)),
          ],
          const SizedBox(height: 16),
          FilledButton.icon(
            icon: _connecting
                ? const SizedBox(width: 16, height: 16,
                    child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white))
                : const Icon(Icons.link),
            label: Text(_connecting ? 'Connecting…' : 'Connect'),
            onPressed: _connecting ? null : _connect,
          ),
        ],
      ),
    );
  }

  Widget _buildControlPanel() {
    return Column(
      children: [
        _StatusBar(connected: true, status: _status, onDisconnect: _disconnect),
        Expanded(
          child: TabBarView(
            controller: _tabController,
            children: [
              _buildControlsTab(),
              _buildTerminalTab(),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildControlsTab() {
    final hotend = (_status['temp_hotend'] as num?)?.toDouble() ?? 0;
    final hotendTarget = (_status['temp_hotend_target'] as num?)?.toDouble() ?? 0;
    final bed = (_status['temp_bed'] as num?)?.toDouble() ?? 0;
    final bedTarget = (_status['temp_bed_target'] as num?)?.toDouble() ?? 0;

    return ListView(
      padding: const EdgeInsets.all(16),
      children: [
        // ── Temperature ──────────────────────────────────────────────────
        _ControlSection(
          title: 'Temperature',
          child: Column(
            children: [
              Row(
                children: [
                  _TempDisplay('Nozzle', hotend, hotendTarget, Colors.orange),
                  const SizedBox(width: 16),
                  _TempDisplay('Bed', bed, bedTarget, Colors.blue),
                ],
              ),
              const SizedBox(height: 12),
              Row(
                children: [
                  Expanded(
                    child: TextField(
                      controller: _nozzleTempController,
                      decoration: const InputDecoration(
                        labelText: 'Nozzle °C',
                        border: OutlineInputBorder(),
                        isDense: true,
                        suffixText: '°C',
                      ),
                      keyboardType: TextInputType.number,
                      inputFormatters: [FilteringTextInputFormatter.digitsOnly],
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: TextField(
                      controller: _bedTempController,
                      decoration: const InputDecoration(
                        labelText: 'Bed °C',
                        border: OutlineInputBorder(),
                        isDense: true,
                        suffixText: '°C',
                      ),
                      keyboardType: TextInputType.number,
                      inputFormatters: [FilteringTextInputFormatter.digitsOnly],
                    ),
                  ),
                  const SizedBox(width: 8),
                  Column(
                    children: [
                      SizedBox(
                        height: 36,
                        child: FilledButton(
                          onPressed: _setTemperature,
                          child: const Text('Set'),
                        ),
                      ),
                      const SizedBox(height: 4),
                      SizedBox(
                        height: 36,
                        child: OutlinedButton(
                          onPressed: _cooldown,
                          child: const Text('Cool'),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
              const SizedBox(height: 8),
              // Quick-set buttons
              Wrap(
                spacing: 6,
                children: [
                  for (final t in [170, 190, 210, 215, 230, 250])
                    ActionChip(
                      label: Text('$t°'),
                      onPressed: () {
                        _nozzleTempController.text = '$t';
                        _setTemperature();
                      },
                    ),
                ],
              ),
            ],
          ),
        ),

        // ── Motion ───────────────────────────────────────────────────────
        _ControlSection(
          title: 'Motion Control',
          child: Column(
            children: [
              // Step size
              Row(
                children: [
                  const Text('Step:', style: TextStyle(fontSize: 13)),
                  const SizedBox(width: 8),
                  for (final step in [0.1, 1.0, 10.0, 100.0])
                    Padding(
                      padding: const EdgeInsets.only(right: 6),
                      child: ChoiceChip(
                        label: Text(step < 1 ? '0.1' : '${step.toInt()}'),
                        selected: _jogStep == step,
                        onSelected: (_) => setState(() => _jogStep = step),
                      ),
                    ),
                  const Text('mm'),
                ],
              ),
              const SizedBox(height: 12),
              // XY jog grid + Z column
              Row(
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  // XY grid
                  Expanded(
                    child: Column(
                      children: [
                        Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                          _JogButton(Icons.arrow_upward, 'Y+', () => _move('Y', _jogStep)),
                        ]),
                        Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                          _JogButton(Icons.arrow_back, 'X-', () => _move('X', -_jogStep)),
                          const SizedBox(width: 4),
                          _HomeButton('XY', () => _home(['X', 'Y'])),
                          const SizedBox(width: 4),
                          _JogButton(Icons.arrow_forward, 'X+', () => _move('X', _jogStep)),
                        ]),
                        Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                          _JogButton(Icons.arrow_downward, 'Y-', () => _move('Y', -_jogStep)),
                        ]),
                        const SizedBox(height: 4),
                        OutlinedButton.icon(
                          icon: const Icon(Icons.home, size: 16),
                          label: const Text('Home All'),
                          onPressed: () => _home(null),
                        ),
                      ],
                    ),
                  ),
                  const SizedBox(width: 16),
                  // Z column
                  Column(
                    children: [
                      const Text('Z', style: TextStyle(fontSize: 12, color: Colors.grey)),
                      _JogButton(Icons.keyboard_arrow_up, 'Z+', () => _move('Z', _jogStep)),
                      const SizedBox(height: 4),
                      _HomeButton('Z', () => _home(['Z'])),
                      const SizedBox(height: 4),
                      _JogButton(Icons.keyboard_arrow_down, 'Z-', () => _move('Z', -_jogStep)),
                    ],
                  ),
                ],
              ),
            ],
          ),
        ),

        // ── Extruder ─────────────────────────────────────────────────────
        _ControlSection(
          title: 'Extruder',
          child: Column(
            children: [
              Row(
                children: [
                  const Text('Amount:', style: TextStyle(fontSize: 13)),
                  const SizedBox(width: 8),
                  for (final mm in [1, 5, 10, 50])
                    Padding(
                      padding: const EdgeInsets.only(right: 6),
                      child: ChoiceChip(
                        label: Text('$mm mm'),
                        selected: _extrudeAmount == mm,
                        onSelected: (_) => setState(() => _extrudeAmount = mm),
                      ),
                    ),
                ],
              ),
              const SizedBox(height: 12),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  OutlinedButton.icon(
                    icon: const Icon(Icons.arrow_upward, size: 16),
                    label: const Text('Extrude'),
                    onPressed: () => _extrude(_extrudeAmount.toDouble()),
                  ),
                  OutlinedButton.icon(
                    icon: const Icon(Icons.arrow_downward, size: 16),
                    label: const Text('Retract'),
                    onPressed: () => _extrude(-_extrudeAmount.toDouble()),
                  ),
                ],
              ),
              const SizedBox(height: 8),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  TextButton.icon(
                    icon: const Icon(Icons.download, size: 16),
                    label: const Text('Load filament'),
                    onPressed: () => _extrude(100),
                  ),
                  TextButton.icon(
                    icon: const Icon(Icons.upload, size: 16),
                    label: const Text('Unload'),
                    onPressed: () => _extrude(-100),
                  ),
                ],
              ),
            ],
          ),
        ),

        // ── Fan ──────────────────────────────────────────────────────────
        _ControlSection(
          title: 'Fan',
          child: Row(
            children: [
              const Icon(Icons.air, size: 20),
              const SizedBox(width: 8),
              Expanded(
                child: Slider(
                  value: _fanSpeed.toDouble(),
                  min: 0,
                  max: 100,
                  divisions: 10,
                  label: '$_fanSpeed%',
                  onChanged: (v) => setState(() => _fanSpeed = v.round()),
                  onChangeEnd: (v) => _setFan(v.round()),
                ),
              ),
              SizedBox(
                width: 48,
                child: Text('$_fanSpeed%', textAlign: TextAlign.right),
              ),
              const SizedBox(width: 4),
              IconButton(
                icon: const Icon(Icons.power_off, size: 18),
                tooltip: 'Fan off',
                onPressed: () => _setFan(0),
              ),
            ],
          ),
        ),
      ],
    );
  }

  Widget _buildTerminalTab() {
    return Column(
      children: [
        Expanded(
          child: Container(
            color: const Color(0xFF0D0D0D),
            child: ListView.builder(
              controller: _terminalScrollController,
              padding: const EdgeInsets.all(12),
              itemCount: _terminalLog.length,
              itemBuilder: (_, i) {
                final line = _terminalLog[i];
                final isCommand = line.startsWith('>');
                return Text(
                  line,
                  style: TextStyle(
                    fontFamily: 'monospace',
                    fontSize: 12,
                    color: isCommand ? Colors.greenAccent : Colors.grey.shade400,
                  ),
                );
              },
            ),
          ),
        ),
        Container(
          padding: const EdgeInsets.all(8),
          color: Theme.of(context).colorScheme.surfaceContainerHighest,
          child: Row(
            children: [
              Expanded(
                child: TextField(
                  controller: _gcodeController,
                  decoration: const InputDecoration(
                    hintText: 'G-code command (e.g. M503, G28)',
                    border: OutlineInputBorder(),
                    isDense: true,
                    filled: true,
                  ),
                  style: const TextStyle(fontFamily: 'monospace', fontSize: 13),
                  onSubmitted: (_) => _sendGcode(),
                  textInputAction: TextInputAction.send,
                ),
              ),
              const SizedBox(width: 8),
              FilledButton(
                onPressed: _sendingGcode ? null : _sendGcode,
                child: const Text('Send'),
              ),
            ],
          ),
        ),
      ],
    );
  }
}

// ─── Sub-widgets ──────────────────────────────────────────────────────────────

class _StatusBar extends StatelessWidget {
  final bool connected;
  final Map<String, dynamic> status;
  final VoidCallback onDisconnect;

  const _StatusBar({required this.connected, required this.status, required this.onDisconnect});

  @override
  Widget build(BuildContext context) {
    final hotend = (status['temp_hotend'] as num?)?.toDouble() ?? 0;
    final bed = (status['temp_bed'] as num?)?.toDouble() ?? 0;
    return Container(
      color: connected ? Colors.green.shade900.withOpacity(0.5) : Colors.transparent,
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 10),
      child: Row(
        children: [
          Icon(connected ? Icons.print : Icons.print_disabled,
              size: 18, color: connected ? Colors.greenAccent : Colors.grey),
          const SizedBox(width: 8),
          Expanded(
            child: connected
                ? Wrap(
                    spacing: 16,
                    children: [
                      Text('Prusa Mk4 connected',
                          style: const TextStyle(color: Colors.greenAccent, fontSize: 13)),
                      Text('🌡 ${hotend.toStringAsFixed(0)}°C  🛏 ${bed.toStringAsFixed(0)}°C',
                          style: const TextStyle(fontSize: 12, color: Colors.white70)),
                    ],
                  )
                : const Text('No printer connected', style: TextStyle(color: Colors.grey)),
          ),
          if (connected)
            TextButton(onPressed: onDisconnect, child: const Text('Disconnect')),
        ],
      ),
    );
  }
}

class _TempDisplay extends StatelessWidget {
  final String label;
  final double current;
  final double target;
  final Color color;

  const _TempDisplay(this.label, this.current, this.target, this.color);

  @override
  Widget build(BuildContext context) {
    final isHeating = target > 0 && current < target - 2;
    return Expanded(
      child: Container(
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
        decoration: BoxDecoration(
          border: Border.all(color: color.withOpacity(0.4)),
          borderRadius: BorderRadius.circular(8),
        ),
        child: Row(
          children: [
            Icon(isHeating ? Icons.local_fire_department : Icons.thermostat,
                color: color, size: 18),
            const SizedBox(width: 6),
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(label, style: const TextStyle(fontSize: 11, color: Colors.grey)),
                Text(
                  '${current.toStringAsFixed(0)}° / ${target.toStringAsFixed(0)}°',
                  style: TextStyle(fontWeight: FontWeight.bold, color: color),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _ControlSection extends StatelessWidget {
  final String title;
  final Widget child;

  const _ControlSection({required this.title, required this.child});

  @override
  Widget build(BuildContext context) {
    return Card(
      margin: const EdgeInsets.only(bottom: 12),
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Text(title,
                style: Theme.of(context)
                    .textTheme
                    .labelLarge
                    ?.copyWith(color: Theme.of(context).colorScheme.primary)),
            const SizedBox(height: 12),
            child,
          ],
        ),
      ),
    );
  }
}

class _JogButton extends StatelessWidget {
  final IconData icon;
  final String tooltip;
  final VoidCallback onPressed;

  const _JogButton(this.icon, this.tooltip, this.onPressed);

  @override
  Widget build(BuildContext context) => Tooltip(
        message: tooltip,
        child: SizedBox(
          width: 48,
          height: 48,
          child: OutlinedButton(
            onPressed: onPressed,
            style: OutlinedButton.styleFrom(padding: EdgeInsets.zero),
            child: Icon(icon, size: 20),
          ),
        ),
      );
}

class _HomeButton extends StatelessWidget {
  final String axes;
  final VoidCallback onPressed;

  const _HomeButton(this.axes, this.onPressed);

  @override
  Widget build(BuildContext context) => Tooltip(
        message: 'Home $axes',
        child: SizedBox(
          width: 48,
          height: 48,
          child: FilledButton.tonal(
            onPressed: onPressed,
            style: FilledButton.styleFrom(padding: EdgeInsets.zero),
            child: const Icon(Icons.home, size: 18),
          ),
        ),
      );
}
