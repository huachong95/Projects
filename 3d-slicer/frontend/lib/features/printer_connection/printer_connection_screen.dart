import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';

class PrinterConnectionScreen extends StatefulWidget {
  const PrinterConnectionScreen({super.key});

  @override
  State<PrinterConnectionScreen> createState() => _PrinterConnectionScreenState();
}

class _PrinterConnectionScreenState extends State<PrinterConnectionScreen> {
  final _hostController = TextEditingController();
  final _keyController = TextEditingController();
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
    super.dispose();
  }

  Future<void> _loadCurrentStatus() async {
    try {
      final resp = await apiClient.get<Map<String, dynamic>>('/api/printer/status');
      setState(() => _connected = resp.statusCode == 200);
    } catch (_) {
      setState(() => _connected = false);
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
      if (devices.isEmpty) {
        _showSnack('No printers found on the network.');
      }
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
      setState(() => _errorMsg = 'Enter both IP address and API key.');
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
      _showSnack('Connected to Prusa Mk4 at $host');
    } catch (e) {
      setState(() => _errorMsg = 'Connection failed. Check the IP and API key.');
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

  void _showSnack(String msg) {
    if (mounted) ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text(msg)));
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Connect Printer')),
      body: SingleChildScrollView(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            // Status card
            _StatusCard(connected: _connected, onDisconnect: _disconnect),
            const SizedBox(height: 24),

            // Auto-discover section
            Text('Auto-discover', style: Theme.of(context).textTheme.titleSmall),
            const SizedBox(height: 8),
            const Text(
              'Scans your local network for Prusa printers with PrusaLink enabled.',
              style: TextStyle(color: Colors.grey),
            ),
            const SizedBox(height: 12),
            OutlinedButton.icon(
              icon: _discovering
                  ? const SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2))
                  : const Icon(Icons.search),
              label: Text(_discovering ? 'Searching…' : 'Discover on network'),
              onPressed: _discovering ? null : _discover,
            ),
            if (_discovered.isNotEmpty) ...[
              const SizedBox(height: 12),
              ..._discovered.map((d) => ListTile(
                    leading: const Icon(Icons.print),
                    title: Text(d['name'] as String? ?? 'Prusa Printer'),
                    subtitle: Text(d['host'] as String? ?? ''),
                    trailing: TextButton(
                      child: const Text('Use'),
                      onPressed: () => setState(() {
                        _hostController.text = d['host'] as String? ?? '';
                      }),
                    ),
                  )),
            ],

            const SizedBox(height: 24),
            const Divider(),
            const SizedBox(height: 16),

            // Manual entry section
            Text('Manual entry', style: Theme.of(context).textTheme.titleSmall),
            const SizedBox(height: 4),
            const Text(
              'Find the IP on your Mk4: Settings → Network → IP Address.\nFind the API key: Settings → Network → PrusaLink.',
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
                hintText: '8-character key from the printer screen',
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
                  ? const SizedBox(width: 16, height: 16, child: CircularProgressIndicator(strokeWidth: 2, color: Colors.white))
                  : const Icon(Icons.link),
              label: Text(_connecting ? 'Connecting…' : 'Connect'),
              onPressed: _connecting || _connected ? null : _connect,
            ),

            const SizedBox(height: 32),
            const Divider(),
            const SizedBox(height: 16),
            Text('Camera', style: Theme.of(context).textTheme.titleSmall),
            const SizedBox(height: 4),
            const Text(
              'The Prusa Camera streams automatically via PrusaLink once connected.\nNo separate setup needed.',
              style: TextStyle(color: Colors.grey),
            ),
          ],
        ),
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
    return Card(
      color: connected
          ? Colors.green.shade900.withOpacity(0.4)
          : Theme.of(context).colorScheme.surfaceContainerHighest,
      child: Padding(
        padding: const EdgeInsets.all(16),
        child: Row(
          children: [
            Icon(
              connected ? Icons.print : Icons.print_disabled,
              color: connected ? Colors.greenAccent : Colors.grey,
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Text(
                connected ? 'Prusa Mk4 connected' : 'No printer connected',
                style: TextStyle(color: connected ? Colors.greenAccent : Colors.grey),
              ),
            ),
            if (connected)
              TextButton(onPressed: onDisconnect, child: const Text('Disconnect')),
          ],
        ),
      ),
    );
  }
}
