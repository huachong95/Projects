import 'package:flutter/material.dart';
import 'package:go_router/go_router.dart';

import '../../core/api_client.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  bool _backendOnline = false;

  @override
  void initState() {
    super.initState();
    _checkBackend();
  }

  Future<void> _checkBackend() async {
    try {
      final resp = await apiClient.get<Map>('/health');
      setState(() => _backendOnline = resp.statusCode == 200);
    } catch (_) {
      setState(() => _backendOnline = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final cs = Theme.of(context).colorScheme;
    return Scaffold(
      appBar: AppBar(
        title: const Text('3D Slicer'),
        actions: [
          Padding(
            padding: const EdgeInsets.only(right: 16),
            child: Icon(
              Icons.circle,
              size: 12,
              color: _backendOnline ? Colors.greenAccent : Colors.redAccent,
            ),
          ),
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(24),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            if (!_backendOnline)
              _BackendWarning(onRefresh: _checkBackend),
            const SizedBox(height: 24),
            Text('Quick actions', style: Theme.of(context).textTheme.titleMedium),
            const SizedBox(height: 16),
            Wrap(
              spacing: 16,
              runSpacing: 16,
              children: [
                _ActionCard(
                  icon: Icons.print_outlined,
                  label: 'Connect Printer',
                  description: 'PrusaLink / Mk4 setup',
                  onTap: () => context.go('/connect'),
                ),
                _ActionCard(
                  icon: Icons.file_upload_outlined,
                  label: 'Import Model',
                  description: 'Load STL, 3MF, or OBJ',
                  onTap: () => context.go('/import'),
                ),
                _ActionCard(
                  icon: Icons.monitor_heart_outlined,
                  label: 'Print Monitor',
                  description: 'Watch live print + camera',
                  onTap: () => context.go('/monitor'),
                ),
                _ActionCard(
                  icon: Icons.videocam_outlined,
                  label: 'Timelapses',
                  description: 'View completed timelapses',
                  onTap: () => context.go('/timelapse'),
                ),
              ],
            ),
          ],
        ),
      ),
    );
  }
}

class _BackendWarning extends StatelessWidget {
  final VoidCallback onRefresh;
  const _BackendWarning({required this.onRefresh});

  @override
  Widget build(BuildContext context) {
    return Card(
      color: Theme.of(context).colorScheme.errorContainer,
      child: Padding(
        padding: const EdgeInsets.all(12),
        child: Row(
          children: [
            const Icon(Icons.warning_amber_rounded),
            const SizedBox(width: 8),
            const Expanded(child: Text('Backend offline. Start the Python server.')),
            TextButton(onPressed: onRefresh, child: const Text('Retry')),
          ],
        ),
      ),
    );
  }
}

class _ActionCard extends StatelessWidget {
  final IconData icon;
  final String label;
  final String description;
  final VoidCallback onTap;

  const _ActionCard({
    required this.icon,
    required this.label,
    required this.description,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    return SizedBox(
      width: 180,
      child: Card(
        child: InkWell(
          onTap: onTap,
          borderRadius: BorderRadius.circular(12),
          child: Padding(
            padding: const EdgeInsets.all(20),
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Icon(icon, size: 32, color: Theme.of(context).colorScheme.primary),
                const SizedBox(height: 12),
                Text(label, style: Theme.of(context).textTheme.titleSmall),
                const SizedBox(height: 4),
                Text(
                  description,
                  style: Theme.of(context).textTheme.bodySmall?.copyWith(
                        color: Theme.of(context).colorScheme.onSurfaceVariant,
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
