import 'package:dio/dio.dart';
import 'package:flutter/material.dart';

import '../../core/api_client.dart';
import '../../shared/widgets/animated_background.dart';
import '../../shared/widgets/entrance.dart';
import '../../shared/widgets/pulsing_dot.dart';
import '../../theme/app_theme.dart';

class PrinterConnectionScreen extends StatefulWidget {
  const PrinterConnectionScreen({super.key});

  @override
  State<PrinterConnectionScreen> createState() => _PrinterConnectionScreenState();
}

class _PrinterConnectionScreenState extends State<PrinterConnectionScreen> {
  final _hostController = TextEditingController();
  final _keyController = TextEditingController();
  final _cameraController = TextEditingController();
  bool _connecting = false;
  bool _connected = false;
  String? _errorMsg;
  List<Map<String, dynamic>> _discovered = [];
  bool _discovering = false;

  @override
  void initState() {
    super.initState();
    _loadCurrentStatus();
  }

  @override
  void dispose() {
    _hostController.dispose();
    _keyController.dispose();
    _cameraController.dispose();
    super.dispose();
  }

  Future<void> _loadCurrentStatus() async {
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/status');
      if (mounted) setState(() => _connected = resp.data?['connected'] == true);
    } catch (_) {
      if (mounted) setState(() => _connected = false);
    }
  }

  Future<void> _discover() async {
    setState(() {
      _discovering = true;
      _discovered = [];
    });
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/discover');
      final devices = (resp.data?['devices'] as List?)?.cast<Map<String, dynamic>>() ?? [];
      setState(() => _discovered = devices);
      if (devices.isEmpty) _showSnack('No printers found on the network.');
    } catch (e) {
      _showSnack('Discovery failed: $e');
    } finally {
      setState(() => _discovering = false);
    }
  }

  Future<void> _connect() async {
    final host = _hostController.text.trim();
    final key = _keyController.text.trim();
    if (host.isEmpty || key.isEmpty) {
      setState(() => _errorMsg = 'Enter both the IP address and API key.');
      return;
    }
    setState(() {
      _connecting = true;
      _errorMsg = null;
    });
    try {
      await apiClient.post('/api/printer/connect', data: {
        'type': 'prusalink',
        'host': host,
        'api_key': key,
      });
      setState(() => _connected = true);
      _showSnack('Connected to Prusa MK4 at $host');
    } catch (e) {
      setState(() => _errorMsg = _errorText(e));
    } finally {
      setState(() => _connecting = false);
    }
  }

  Future<void> _disconnect() async {
    try {
      await apiClient.delete('/api/printer/connect');
      setState(() => _connected = false);
      _showSnack('Disconnected');
    } catch (e) {
      _showSnack('Disconnect failed: $e');
    }
  }

  Future<void> _setCamera() async {
    final url = _cameraController.text.trim();
    if (url.isEmpty) return;
    try {
      await apiClient.post('/api/monitoring/camera/connect', data: {'url': url});
      _showSnack('External camera connected');
    } catch (e) {
      _showSnack(_errorText(e));
    }
  }

  String _errorText(Object e) {
    if (e is DioException) {
      final detail = e.response?.data;
      if (detail is Map && detail['detail'] != null) return detail['detail'].toString();
      return 'Connection failed. Check the IP address and API key.';
    }
    return 'Connection failed.';
  }

  void _showSnack(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Connect Printer')),
      body: AnimatedBackground(
        child: SafeArea(
          top: false,
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(24),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [
                FadeSlideIn(
                  child: _StatusCard(connected: _connected, onDisconnect: _disconnect),
                ),
                const SizedBox(height: AppSpace.xl),
                FadeSlideIn(
                  index: 1,
                  child: _Section(
                    title: 'Auto-discover',
                    subtitle: 'Scan the local network for PrusaLink printers.',
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        OutlinedButton.icon(
                          icon: _discovering
                              ? const SizedBox(
                                  width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))
                              : const Icon(Icons.radar),
                          label: Text(_discovering ? 'Searching…' : 'Discover on network'),
                          onPressed: _discovering ? null : _discover,
                        ),
                        ..._discovered.map((d) => Padding(
                              padding: const EdgeInsets.only(top: 8),
                              child: ListTile(
                                tileColor: AppColors.surfaceHigh,
                                shape: RoundedRectangleBorder(
                                    borderRadius: BorderRadius.circular(AppRadius.sm)),
                                leading: const Icon(Icons.print, color: AppColors.primary),
                                title: Text(d['name'] as String? ?? 'Prusa Printer'),
                                subtitle: Text(d['host'] as String? ?? ''),
                                trailing: TextButton(
                                  child: const Text('Use'),
                                  onPressed: () => setState(
                                      () => _hostController.text = d['host'] as String? ?? ''),
                                ),
                              ),
                            )),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: AppSpace.lg),
                FadeSlideIn(
                  index: 2,
                  child: _Section(
                    title: 'Manual entry',
                    subtitle:
                        'IP: Settings → Network → IP Address\nKey: Settings → Network → PrusaLink',
                    child: Column(
                      children: [
                        TextField(
                          controller: _hostController,
                          decoration: const InputDecoration(
                            labelText: 'Printer IP address',
                            hintText: '192.168.1.100',
                            prefixIcon: Icon(Icons.router_outlined),
                          ),
                        ),
                        const SizedBox(height: 12),
                        TextField(
                          controller: _keyController,
                          obscureText: true,
                          decoration: const InputDecoration(
                            labelText: 'PrusaLink API key',
                            hintText: 'From the printer screen',
                            prefixIcon: Icon(Icons.key_outlined),
                          ),
                        ),
                        if (_errorMsg != null) ...[
                          const SizedBox(height: 10),
                          Row(
                            children: [
                              const Icon(Icons.error_outline, size: 16, color: AppColors.danger),
                              const SizedBox(width: 6),
                              Expanded(
                                child: Text(_errorMsg!,
                                    style: const TextStyle(color: AppColors.danger)),
                              ),
                            ],
                          ),
                        ],
                        const SizedBox(height: AppSpace.md),
                        SizedBox(
                          width: double.infinity,
                          child: FilledButton.icon(
                            icon: _connecting
                                ? const SizedBox(
                                    width: 16,
                                    height: 16,
                                    child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white))
                                : const Icon(Icons.link),
                            label: Text(_connecting ? 'Connecting…' : 'Connect'),
                            onPressed: _connecting || _connected ? null : _connect,
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                const SizedBox(height: AppSpace.lg),
                FadeSlideIn(
                  index: 3,
                  child: _Section(
                    title: 'Camera',
                    subtitle:
                        'The Prusa Camera streams automatically once connected. '
                        'Only set a URL below if you use a separate webcam (MJPEG).',
                    child: Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: _cameraController,
                            decoration: const InputDecoration(
                              labelText: 'External MJPEG URL (optional)',
                              hintText: 'http://192.168.1.50:8080/stream',
                              prefixIcon: Icon(Icons.videocam_outlined),
                            ),
                          ),
                        ),
                        const SizedBox(width: 8),
                        FilledButton(onPressed: _setCamera, child: const Text('Set')),
                      ],
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}

class _Section extends StatelessWidget {
  final String title;
  final String subtitle;
  final Widget child;
  const _Section({required this.title, required this.subtitle, required this.child});

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.all(AppSpace.md),
      decoration: BoxDecoration(
        color: AppColors.surface.withOpacity(0.85),
        borderRadius: BorderRadius.circular(AppRadius.md),
        border: Border.all(color: AppColors.surfaceBorder),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.stretch,
        children: [
          Text(title, style: Theme.of(context).textTheme.titleMedium),
          const SizedBox(height: 4),
          Text(subtitle, style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 13, height: 1.4)),
          const SizedBox(height: AppSpace.md),
          child,
        ],
      ),
    );
  }
}

class _StatusCard extends StatelessWidget {
  final bool connected;
  final VoidCallback onDisconnect;

  const _StatusCard({required this.connected, required this.onDisconnect});

  @override
  Widget build(BuildContext context) {
    final color = connected ? AppColors.success : AppColors.onSurfaceDim;
    return AnimatedContainer(
      duration: AppMotion.med,
      padding: const EdgeInsets.all(AppSpace.lg),
      decoration: BoxDecoration(
        gradient: LinearGradient(
          colors: [color.withOpacity(0.16), AppColors.surface],
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
        ),
        borderRadius: BorderRadius.circular(AppRadius.md),
        border: Border.all(color: color.withOpacity(0.4)),
      ),
      child: Row(
        children: [
          PulsingDot(color: color, size: 12, active: connected),
          const SizedBox(width: AppSpace.sm),
          Expanded(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  connected ? 'Prusa MK4 connected' : 'No printer connected',
                  style: const TextStyle(fontWeight: FontWeight.w700, fontSize: 16),
                ),
                Text(
                  connected ? 'Ready to monitor and print' : 'Enter details below to connect',
                  style: const TextStyle(color: AppColors.onSurfaceDim, fontSize: 13),
                ),
              ],
            ),
          ),
          if (connected)
            TextButton(onPressed: onDisconnect, child: const Text('Disconnect')),
        ],
      ),
    );
  }
}
